// SPDX-License-Identifier: AGPL-3.0-or-later
// Portable two-stage VietMap iOS HUD sign classifier (CMD 0x08 & CMD 0x41):
//   Stage 1: Inner 10x8 luma speed-limit digit match (30..120 km/h)
//   Stage 2: Full 16x16 luma + 8x8 (R-B) chroma NCC match for prohibition,
//            area, road, and camera signs (VietMapSignCode 0x01..0x21).
//
// Zero dynamic allocation; shares static TJpgDec & Box16Accumulator with arrow_classifier.h.
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "arrow_classifier.h"
#include "speed_sign_classifier.h"
#include "traffic_sign_templates.h"

namespace vmhud {

enum VietMapSignCode : uint8_t {
  kSignNone              = 0x00,
  // Turn & entry prohibition signs
  kSignNoLeftTurn        = 0x01,
  kSignNoRightTurn       = 0x02,
  kSignNoLeftRightTurn   = 0x03,
  kSignNoUTurn           = 0x04,
  kSignNoLeftTurnUTurn   = 0x05,
  kSignNoRightTurnUTurn  = 0x06,
  kSignNoEntry           = 0x07,
  kSignNoEntryNoLeft     = 0x08,
  kSignNoEntryNoRight    = 0x09,
  // Overtaking, area, parking & road signs
  kSignNoPassingStart    = 0x10,
  kSignNoPassingEnd      = 0x11,
  kSignResidentialStart  = 0x12,
  kSignResidentialEnd    = 0x13,
  kSignNoParking         = 0x14,
  kSignTollStation       = 0x15,
  kSignRailway           = 0x16,
  kSignTunnel            = 0x17,
  // Camera & checkpoint alerts
  kSignSpeedCamera       = 0x20,
  kSignTrafficCamera     = 0x21,
};

static constexpr float kTrafficSignMinScore  = 0.86f;
static constexpr float kTrafficSignMinMargin = 0.02f;

struct TrafficSignMatch {
  uint8_t signCode = kSignNone;
  float   score    = 0.0f;
  float   margin   = 0.0f;
};

struct AlertImageClassification {
  uint8_t speedLimit = 0;          // Non-zero (30..120) if image is a speed-limit sign
  uint8_t signCode   = kSignNone;  // Non-zero if image is a prohibition/warning/camera sign
  float   score      = 0.0f;
  float   margin     = 0.0f;
};

inline TrafficSignMatch classifyTrafficSignLumaChroma(const uint8_t luma16[256],
                                                      const int8_t rb8[64]) {
  TrafficSignMatch out;

  uint32_t sum = 0;
  for (size_t i = 0; i < 256; ++i) sum += luma16[i];
  const float mean = float(sum) * (1.0f / 256.0f);

  float mabs = 0.0f;
  for (size_t i = 0; i < 256; ++i) {
    const float v = fabsf(float(luma16[i]) - mean);
    if (v > mabs) mabs = v;
  }
  for (size_t j = 0; j < 64; ++j) {
    const float v = fabsf(float(rb8[j]) * 1.5f);
    if (v > mabs) mabs = v;
  }
  if (mabs < 2.0f) return out;

  int8_t q[kTrafficSignFeatLen];
  uint32_t qNormSq = 0;
  const float scale = 127.0f / mabs;
  for (size_t i = 0; i < 256; ++i) {
    const float sv = (float(luma16[i]) - mean) * scale;
    int iv = int(sv >= 0.0f ? (sv + 0.5f) : (sv - 0.5f));
    if (iv < -127) iv = -127;
    if (iv > 127)  iv = 127;
    q[i] = static_cast<int8_t>(iv);
    qNormSq += uint32_t(iv * iv);
  }
  for (size_t j = 0; j < 64; ++j) {
    const float sv = (float(rb8[j]) * 1.5f) * scale;
    int iv = int(sv >= 0.0f ? (sv + 0.5f) : (sv - 0.5f));
    if (iv < -127) iv = -127;
    if (iv > 127)  iv = 127;
    q[256 + j] = static_cast<int8_t>(iv);
    qNormSq += uint32_t(iv * iv);
  }
  if (qNormSq == 0) return out;
  const float qInvNorm = 1.0f / sqrtf(float(qNormSq));

  float classBest[64];
  for (size_t i = 0; i < 64; ++i) classBest[i] = -2.0f;

  float bestScore = -2.0f;
  uint8_t bestCode = kSignNone;
  for (size_t t = 0; t < kTrafficSignTemplateCount; ++t) {
    const TrafficSignTemplate &tmpl = kTrafficSignTemplates[t];
    int32_t dot = 0;
    for (size_t i = 0; i < kTrafficSignFeatLen; ++i) {
      dot += int32_t(q[i]) * int32_t(tmpl.vec[i]);
    }
    const float sc = float(dot) * qInvNorm * tmpl.invNorm;
    if (tmpl.signCode < 64 && sc > classBest[tmpl.signCode]) {
      classBest[tmpl.signCode] = sc;
    }
    if (sc > bestScore) {
      bestScore = sc;
      bestCode = tmpl.signCode;
    }
  }

  float runnerUp = -2.0f;
  for (size_t c = 0; c < 64; ++c) {
    if (c == bestCode) continue;
    if (classBest[c] > runnerUp) runnerUp = classBest[c];
  }

  const float margin = (runnerUp > -1.5f) ? (bestScore - runnerUp) : bestScore;
  out.score = bestScore;
  out.margin = margin;
  if (bestCode != kSignNone && bestScore >= kTrafficSignMinScore &&
      margin >= kTrafficSignMinMargin) {
    out.signCode = bestCode;
  }
  return out;
}

inline AlertImageClassification classifyAlertGrids(const uint8_t luma16[256],
                                                   const int8_t rb8[64]) {
  AlertImageClassification res;
  const SpeedSignMatch sp = classifySpeedSignLuma16(luma16);
  if (sp.limit != 0) {
    res.speedLimit = sp.limit;
    res.score = sp.score;
    res.margin = sp.margin;
    return res;
  }
  const TrafficSignMatch ts = classifyTrafficSignLumaChroma(luma16, rb8);
  res.signCode = ts.signCode;
  res.score = ts.score;
  res.margin = ts.margin;
  return res;
}

inline AlertImageClassification classifyAlertRgb(const uint8_t *rgb,
                                                 uint16_t width,
                                                 uint16_t height) {
  AlertImageClassification res;
  if (!rgb || width == 0 || height == 0) return res;
  Box16Accumulator acc;
  acc.reset(width, height, true);
  acc.addRgbRect(0, 0, static_cast<uint16_t>(width - 1),
                 static_cast<uint16_t>(height - 1), rgb);
  uint8_t luma[256];
  int8_t rb8[64];
  acc.finalize(luma);
  acc.finalizeChroma8(rb8);
  return classifyAlertGrids(luma, rb8);
}

#ifdef ESP_PLATFORM
// Single-pass JPEG decode + two-stage speed/traffic sign classification.
// Must be called strictly from loop().
inline AlertImageClassification classifyAlertJpeg(const uint8_t *jpeg, size_t len) {
  AlertImageClassification res;
  uint8_t luma[256];
  int8_t rb8[64];
  if (!decodeJpegToLuma16Chroma8(jpeg, len, luma, rb8)) {
    return res;
  }
  return classifyAlertGrids(luma, rb8);
}
#endif

}  // namespace vmhud
