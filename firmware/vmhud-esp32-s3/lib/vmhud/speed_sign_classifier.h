// SPDX-License-Identifier: AGPL-3.0-or-later
// Portable camera/speed-limit frame parser and upcoming speed-sign classifier
// for VietMap Live BLE HUD (CMD 0x01, CMD 0x08, CMD 0x41).
//
// Zero dynamic allocation; shares the 16x16 luma accumulator and ROM TJpgDec
// pipeline with arrow_classifier.h.
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "arrow_classifier.h"
#include "hud_codec.h"
#include "speed_sign_templates.h"

namespace vmhud {

struct CameraParseResult {
  uint8_t  currentLimit = 0;   // Posted speed limit on current road (5..200 km/h, 0 = none)
  uint8_t  nextLimit    = 0;   // Upcoming DIFFERENT speed limit (5..200 km/h, 0 = none)
  uint16_t nextDist     = 0;   // Distance to upcoming limit in meters (0 = unknown/dummy)
  uint16_t camDist      = 0;   // Closest camera/alert distance for ESP32 display UI
  uint8_t  camType      = 0;   // Closest camera/alert type for ESP32 display UI
};

// Parse VietMap BLE CMD 0x01 payload (one or more 4-byte [type, dist_be16, lim] records).
//
// Verified against VietMap Live iOS DJBTSDKHandler (0x12f24):
//   - sendCameraInformation sends up to 2 records (8 bytes):
//       Slot 0: [type0, dist0_hi, dist0_lo, lim0]  (real countdown cameraDistance)
//       Slot 1: [type1, (lim1>0 ? 100 : 0), lim1]  (hardcoded 100m dummy distance)
//
// Disambiguation between currentLimit and nextLimit when 2 slots carry different limits:
//   1. Continuity: if prevCurrentLimit matches exactly one slot, that slot is currentLimit
//      and the other slot is nextLimit (works regardless of slot order).
//   2. Cold start / simultaneous jump: the slot with a real countdown distance
//      (dist > 0 && dist < 60000 && dist != 100) is upcoming; the dummy/zero-distance slot
//      is current.
//   3. Same-limit camera (e.g. 60 km/h speed camera on a 60 km/h road): nextLimit = 0.
inline CameraParseResult parseCameraFrame(const Frame &frame, uint8_t prevCurrentLimit) {
  CameraParseResult res;
  if (frame.command != 1 || frame.size < 4 || (frame.size % 4) != 0) {
    return res;
  }

  struct Slot {
    uint8_t  type;
    uint16_t dist;
    uint8_t  lim;
    size_t   index;
  };
  Slot valid[4];
  size_t nValid = 0;

  uint16_t bestCamDist = 0xFFFFu;
  for (size_t i = 0; i < frame.size; i += 4) {
    const uint8_t  type = frame.payload[i];
    const uint16_t dist = be16(frame.payload + i + 1);
    const uint8_t  lim  = frame.payload[i + 3];
    if (lim < 5 || lim > 200) {
      continue;
    }
    if (dist < 60000u && dist < bestCamDist) {
      bestCamDist = dist;
      res.camDist = dist;
      res.camType = type;
    }
    if (nValid < 4) {
      valid[nValid++] = {type, dist, lim, i / 4};
    }
  }

  if (nValid == 0) {
    return res;
  }
  if (bestCamDist == 0xFFFFu) {
    res.camDist = valid[0].dist;
    res.camType = valid[0].type;
  }
  if (nValid == 1) {
    res.currentLimit = valid[0].lim;
    return res;
  }

  // Check if all valid slots report the same speed limit (e.g. speed camera on same-limit road).
  size_t diffIdx = 0;
  for (size_t i = 1; i < nValid; ++i) {
    if (valid[i].lim != valid[0].lim) {
      diffIdx = i;
      break;
    }
  }
  if (diffIdx == 0) {
    res.currentLimit = valid[0].lim;
    return res;
  }

  const Slot &a = valid[0];
  const Slot &b = valid[diffIdx];

  // Helper: convert raw wire distance to usable upcoming distance (0 if dummy/invalid).
  auto cleanDist = [](const Slot &s) -> uint16_t {
    if (s.dist == 0 || s.dist >= 60000u) return 0;
    // Slot 1 hardcodes 100 in VietMap iOS DJBTSDKHandler when cameraSpeed > 0.
    if (s.index == 1 && s.dist == 100u) return 0;
    return s.dist;
  };

  // Rule 1: Continuity with previous steady-state currentLimit.
  const bool aMatchesPrev = (prevCurrentLimit >= 5 && a.lim == prevCurrentLimit);
  const bool bMatchesPrev = (prevCurrentLimit >= 5 && b.lim == prevCurrentLimit);
  if (aMatchesPrev && !bMatchesPrev) {
    res.currentLimit = a.lim;
    res.nextLimit    = b.lim;
    res.nextDist     = cleanDist(b);
    return res;
  }
  if (bMatchesPrev && !aMatchesPrev) {
    res.currentLimit = b.lim;
    res.nextLimit    = a.lim;
    res.nextDist     = cleanDist(a);
    return res;
  }

  // Rule 2: Cold start — prefer real countdown slot as upcoming, dummy/zero slot as current.
  const bool aHasRealDist = (a.dist > 0 && a.dist < 60000u && !(a.index == 1 && a.dist == 100u));
  const bool bHasRealDist = (b.dist > 0 && b.dist < 60000u && !(b.index == 1 && b.dist == 100u));
  if (aHasRealDist && !bHasRealDist) {
    res.currentLimit = b.lim;
    res.nextLimit    = a.lim;
    res.nextDist     = a.dist;
  } else if (bHasRealDist && !aHasRealDist) {
    res.currentLimit = a.lim;
    res.nextLimit    = b.lim;
    res.nextDist     = b.dist;
  } else {
    // Both have non-dummy distances: Slot 1 is current (iOS DJBTSDKHandler convention),
    // Slot 0 is upcoming alert.
    res.currentLimit = b.lim;
    res.nextLimit    = a.lim;
    res.nextDist     = cleanDist(a);
  }
  return res;
}

// Minimum NCC score and inter-class margin for classifying a CMD 0x08 60x60 JPEG
// as a VietMap speed limit sign (30..120 km/h).
// Empirically verified on all VietMap HUD assets:
//   - True speed signs: score ~1.00, worst inter-class margin 0.0597 (80 vs 90)
//   - Non-speed warning signs (noEntry, no_passing, residential, etc.): max score 0.4337
static constexpr float kSpeedSignMinScore  = 0.88f;
static constexpr float kSpeedSignMinMargin = 0.02f;

struct SpeedSignMatch {
  uint8_t limit  = 0;     // 30..120 km/h, or 0 if not a speed limit sign
  float   score  = 0.0f;
  float   margin = 0.0f;
};

// Classify a 16x16 grayscale luma grid by correlating its inner 10x8 digit region
// (80 cells, rows 4..11, cols 3..12) against the 10 VietMap speed-limit templates.
inline SpeedSignMatch classifySpeedSignLuma16(const uint8_t luma16[256]) {
  SpeedSignMatch out;

  uint32_t sum = 0;
  for (size_t r = kSpeedSignRowStart; r < kSpeedSignRowEnd; ++r) {
    for (size_t c = kSpeedSignColStart; c < kSpeedSignColEnd; ++c) {
      sum += luma16[r * 16 + c];
    }
  }
  const float mean = float(sum) * (1.0f / float(kSpeedSignFeatLen));

  float mabs = 0.0f;
  for (size_t r = kSpeedSignRowStart; r < kSpeedSignRowEnd; ++r) {
    for (size_t c = kSpeedSignColStart; c < kSpeedSignColEnd; ++c) {
      const float v = fabsf(float(luma16[r * 16 + c]) - mean);
      if (v > mabs) mabs = v;
    }
  }
  if (mabs < 2.0f) return out;  // Blank / uniform image

  int8_t q[kSpeedSignFeatLen];
  uint32_t qNormSq = 0;
  const float scale = 127.0f / mabs;
  size_t k = 0;
  for (size_t r = kSpeedSignRowStart; r < kSpeedSignRowEnd; ++r) {
    for (size_t c = kSpeedSignColStart; c < kSpeedSignColEnd; ++c) {
      int iv = int(lroundf((float(luma16[r * 16 + c]) - mean) * scale));
      if (iv < -127) iv = -127;
      if (iv > 127)  iv = 127;
      q[k++] = static_cast<int8_t>(iv);
      qNormSq += uint32_t(iv * iv);
    }
  }
  if (qNormSq == 0) return out;
  const float qInvNorm = 1.0f / sqrtf(float(qNormSq));

  float bestScore = -2.0f;
  float secondScore = -2.0f;
  uint8_t bestLimit = 0;

  for (size_t t = 0; t < kSpeedSignTemplateCount; ++t) {
    const SpeedSignTemplate &tmpl = kSpeedSignTemplates[t];
    int32_t dot = 0;
    for (size_t i = 0; i < kSpeedSignFeatLen; ++i) {
      dot += int32_t(q[i]) * int32_t(tmpl.vec[i]);
    }
    const float sc = float(dot) * qInvNorm * tmpl.invNorm;
    if (sc > bestScore) {
      secondScore = bestScore;
      bestScore = sc;
      bestLimit = tmpl.limit;
    } else if (sc > secondScore) {
      secondScore = sc;
    }
  }

  const float margin = (secondScore > -1.5f) ? (bestScore - secondScore) : bestScore;
  out.score = bestScore;
  out.margin = margin;
  if (bestLimit != 0 && bestScore >= kSpeedSignMinScore && margin >= kSpeedSignMinMargin) {
    out.limit = bestLimit;
  }
  return out;
}

// Host-testable helper: classify an uncompressed RGB888 image as a speed-limit sign.
inline SpeedSignMatch classifySpeedSignRgb(const uint8_t *rgb, uint16_t width, uint16_t height) {
  SpeedSignMatch out;
  if (!rgb || width == 0 || height == 0) return out;
  Box16Accumulator acc;
  acc.reset(width, height);
  acc.addRgbRect(0, 0, static_cast<uint16_t>(width - 1), static_cast<uint16_t>(height - 1), rgb);
  uint8_t luma[256];
  acc.finalize(luma);
  return classifySpeedSignLuma16(luma);
}

#ifdef ESP_PLATFORM
// Decode a baseline JPEG (from CMD 0x08 sendImage) via shared ESP32 ROM TJpgDec
// workspace and classify whether it is a VietMap speed-limit sign (30..120 km/h).
// Must be called single-threaded from loop().
inline SpeedSignMatch classifySpeedSignJpeg(const uint8_t *jpeg, size_t len) {
  SpeedSignMatch out;
  uint8_t luma[256];
  if (!decodeJpegToLuma16(jpeg, len, luma)) {
    return out;
  }
  return classifySpeedSignLuma16(luma);
}
#endif

}  // namespace vmhud
