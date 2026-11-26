// SPDX-License-Identifier: AGPL-3.0-or-later
// Portable 12-byte binary telemetry codec & distance-bucket delta policy for
// streaming VietMap iOS speed limits, prohibition signs, and approaching
// distances from vmhud-esp32-s3 (GATT service FFF8 / char FFF9) to CarPlay Helper.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "traffic_sign_classifier.h"

namespace vmhud {

static constexpr uint8_t kCphFrameMagic      = 0xA7;
static constexpr size_t  kCphMaxSignText     = 15;
static constexpr size_t  kCphMinFrameBytes   = 13;  // 0xA7, len, flags..signDist(9B), textLen(1B), xor(1B)
static constexpr size_t  kCphMaxFrameBytes   = kCphMinFrameBytes + kCphMaxSignText;  // 28 bytes max

static constexpr uint8_t kCphFlagVietMapConn = 0x01;
static constexpr uint8_t kCphFlagOverspeed   = 0x02;
static constexpr uint8_t kCphFlagNavActive   = 0x04;
static constexpr uint8_t kCphFlagCmuConn     = 0x08;

struct CphTelemetryState {
  bool     vietmapConnected   = false;
  bool     overspeed          = false;
  bool     navActive          = false;
  bool     cmuConnected       = false;
  uint8_t  postedSpeedLimit   = 0;  // 5..200 km/h, 0 = none
  uint8_t  currentSpeed       = 0;  // 0..255 km/h clamped
  uint8_t  upcomingSpeedLimit = 0;  // 5..200 km/h, 0 = none
  uint16_t upcomingSpeedDistM = 0;  // meters from CMD 0x01 / 0x41, 0 = unknown
  uint8_t  activeSignCode     = 0;  // VietMapSignCode, 0 = none
  uint16_t activeSignDistM    = 0;  // meters from CMD 0x01 / 0x41, 0 = unspecified
  char     activeSignText[kCphMaxSignText + 1] = {0};  // Raw CMD 0x06 text (e.g. "250 m", "1.2 km")
};

inline bool operator==(const CphTelemetryState &a, const CphTelemetryState &b) {
  return a.vietmapConnected   == b.vietmapConnected   &&
         a.overspeed          == b.overspeed          &&
         a.navActive          == b.navActive          &&
         a.cmuConnected       == b.cmuConnected       &&
         a.postedSpeedLimit   == b.postedSpeedLimit   &&
         a.currentSpeed       == b.currentSpeed       &&
         a.upcomingSpeedLimit == b.upcomingSpeedLimit &&
         a.upcomingSpeedDistM == b.upcomingSpeedDistM &&
         a.activeSignCode     == b.activeSignCode     &&
         a.activeSignDistM    == b.activeSignDistM    &&
         strcmp(a.activeSignText, b.activeSignText) == 0;
}

inline bool operator!=(const CphTelemetryState &a, const CphTelemetryState &b) {
  return !(a == b);
}

// Evaluate overspeed flag with +2 km/h set / 0 km/h clear hysteresis so minor
// GPS jitter around the limit does not flicker BLE & ActivityKit updates.
inline bool evaluateOverspeed(uint8_t postedLimit, uint16_t currentSpeedKmh,
                              bool prevOverspeed, uint8_t toleranceKmh = 2) {
  if (postedLimit < 5 || postedLimit > 200) return false;
  if (prevOverspeed) {
    return currentSpeedKmh > uint16_t(postedLimit);
  }
  return currentSpeedKmh > uint16_t(postedLimit) + uint16_t(toleranceKmh);
}

// Quantize raw approaching distance in meters into ActivityKit-safe display buckets:
//   0      -> 0 (no distance)
//   1..50  -> 50
//   51..100 -> 100
//   101..150 -> 150
//   151..200 -> 200
//   201..250 -> 250
//   251..300 -> 300
//   301..400 -> 400
//   401..500 -> 500
//   > 500  -> 501 (beyond 500m threshold: distance hidden on CarPlay UI)
inline uint16_t quantizeApproachBucket(uint16_t distM) {
  if (distM == 0)   return 0;
  if (distM <= 50)  return 50;
  if (distM <= 100) return 100;
  if (distM <= 150) return 150;
  if (distM <= 200) return 200;
  if (distM <= 250) return 250;
  if (distM <= 300) return 300;
  if (distM <= 400) return 400;
  if (distM <= 500) return 500;
  return 501;
}

// Map CMD 0x01 camera type byte into fallback VietMapSignCode when no explicit
// CMD 0x08 / CMD 0x41 prohibition sign image is active.
inline uint8_t cameraTypeToSignCode(uint8_t camType, uint16_t camDist) {
  if (camType == 0 || camDist == 0 || camDist >= 60000u) return kSignNone;
  // In VietMap DJBT CMD 0x01, type 1 represents speed-limit / speed-camera cell;
  // higher non-zero camera types represent red-light / surveillance / traffic cameras.
  return (camType == 1) ? kSignSpeedCamera : kSignTrafficCamera;
}

// Extract a genuine approaching camera alert from VietMap CMD 0x01 Slot 0 while
// ignoring steady-state posted-limit records (both 4-byte single-slot frames and
// 8-byte 2-slot frames where Slot 1 is empty or carries synthetic 100m distance)
// and avoiding masking an upcoming speed-limit transition (`cam.nextLimit != 0`).
inline bool extractCmd01CameraAlert(const Frame &frame,
                                    const CameraParseResult &cam,
                                    uint8_t &outSignCode,
                                    uint16_t &outDistM) {
  outSignCode = kSignNone;
  outDistM = 0;
  if (frame.command != 1 || frame.size < 4 || (frame.size % 4) != 0 || !frame.payload) {
    return false;
  }
  // If Slot 0 represents an upcoming speed-limit zone change, let upcomingSpeedLimit own the badge.
  if (cam.nextLimit >= 5 && cam.nextLimit <= 200 && cam.nextLimit != cam.currentLimit) {
    return false;
  }
  const uint8_t  type0 = frame.payload[0];
  const uint16_t dist0 = be16(frame.payload + 1);
  if (type0 == 0 || dist0 == 0 || dist0 >= 60000u) {
    return false;
  }
  // In VietMap DJBT CMD 0x01, type == 1 is used for the road's posted speed limit whenever
  // only a single active entry is sent (either a 4-byte frame or an 8-byte frame with an
  // empty second slot {0,0,0,0}) or when both slots carry the synthetic 100m placeholder.
  if (type0 == 1) {
    if (frame.size < 8) {
      return false;
    }
    const uint8_t  type1 = frame.payload[4];
    const uint16_t dist1 = be16(frame.payload + 5);
    const uint8_t  lim1  = frame.payload[7];
    if (type1 == 0 && lim1 == 0) {
      return false;
    }
    if (dist0 == 100u && dist1 == 100u) {
      return false;
    }
  }
  const uint8_t code = cameraTypeToSignCode(type0, dist0);
  if (code == kSignNone) {
    return false;
  }
  outSignCode = code;
  outDistM = dist0;
  return true;
}

inline size_t encodeCphFrame(const CphTelemetryState &st, uint8_t out[kCphMaxFrameBytes]) {
  uint8_t flags = 0;
  if (st.vietmapConnected) flags |= kCphFlagVietMapConn;
  if (st.overspeed)        flags |= kCphFlagOverspeed;
  if (st.navActive)        flags |= kCphFlagNavActive;
  if (st.cmuConnected)     flags |= kCphFlagCmuConn;

  size_t textLen = 0;
  if (st.vietmapConnected) {
    while (textLen < kCphMaxSignText && st.activeSignText[textLen] != '\0') {
      ++textLen;
    }
  }

  out[0]  = kCphFrameMagic;
  out[1]  = static_cast<uint8_t>(10u + textLen);  // bytes [2 .. 11+textLen] + XOR byte
  out[2]  = flags;
  out[3]  = (st.postedSpeedLimit >= 5 && st.postedSpeedLimit <= 200) ? st.postedSpeedLimit : 0;
  out[4]  = st.currentSpeed;
  out[5]  = (st.upcomingSpeedLimit >= 5 && st.upcomingSpeedLimit <= 200) ? st.upcomingSpeedLimit : 0;
  out[6]  = static_cast<uint8_t>((st.upcomingSpeedDistM >> 8) & 0xFFu);
  out[7]  = static_cast<uint8_t>(st.upcomingSpeedDistM & 0xFFu);
  out[8]  = st.activeSignCode;
  out[9]  = static_cast<uint8_t>((st.activeSignDistM >> 8) & 0xFFu);
  out[10] = static_cast<uint8_t>(st.activeSignDistM & 0xFFu);
  out[11] = static_cast<uint8_t>(textLen);
  if (textLen > 0) {
    memcpy(out + 12, st.activeSignText, textLen);
  }

  const size_t xorIdx = 12u + textLen;
  uint8_t x = 0;
  for (size_t i = 0; i < xorIdx; ++i) x ^= out[i];
  out[xorIdx] = x;
  return xorIdx + 1u;
}

inline bool decodeCphFrame(const uint8_t *buf, size_t len, CphTelemetryState &out) {
  if (!buf || len < kCphMinFrameBytes || len > kCphMaxFrameBytes) return false;
  if (buf[0] != kCphFrameMagic) return false;
  const size_t textLen = buf[11];
  if (textLen > kCphMaxSignText || len != kCphMinFrameBytes + textLen) return false;
  if (buf[1] != static_cast<uint8_t>(10u + textLen)) return false;

  uint8_t x = 0;
  for (size_t i = 0; i + 1 < len; ++i) x ^= buf[i];
  if (x != buf[len - 1]) return false;

  out = CphTelemetryState{};
  out.vietmapConnected   = (buf[2] & kCphFlagVietMapConn) != 0;
  out.overspeed          = (buf[2] & kCphFlagOverspeed) != 0;
  out.navActive          = (buf[2] & kCphFlagNavActive) != 0;
  out.cmuConnected       = (buf[2] & kCphFlagCmuConn) != 0;
  out.postedSpeedLimit   = buf[3];
  out.currentSpeed       = buf[4];
  out.upcomingSpeedLimit = buf[5];
  out.upcomingSpeedDistM = (uint16_t(buf[6]) << 8) | uint16_t(buf[7]);
  out.activeSignCode     = buf[8];
  out.activeSignDistM    = (uint16_t(buf[9]) << 8) | uint16_t(buf[10]);
  if (textLen > 0) {
    memcpy(out.activeSignText, buf + 12, textLen);
  }
  out.activeSignText[textLen] = '\0';
  return true;
}

// Parse CMD 0x06 meter strings (e.g. "295 m", "80m") into quantized buckets so
// 1Hz meter countdowns do not spam BLE notifications and exhaust iOS ActivityKit budget.
inline bool quantizeOverlayMeterText(const char *txt, uint16_t &outBucket) {
  if (!txt || txt[0] == '\0') return false;
  size_t i = 0;
  while (txt[i] == ' ') ++i;
  if (txt[i] < '0' || txt[i] > '9') return false;
  uint32_t val = 0;
  while (txt[i] >= '0' && txt[i] <= '9') {
    val = val * 10u + uint32_t(txt[i] - '0');
    if (val > 60000u) return false;
    ++i;
  }
  while (txt[i] == ' ') ++i;
  if ((txt[i] != 'm' && txt[i] != 'M') || txt[i + 1] != '\0') {
    return false;
  }
  outBucket = quantizeApproachBucket(static_cast<uint16_t>(val));
  return true;
}

// Returns true when two states differ in any CarPlay-visible semantic property
// (ignoring raw 1 km/h GPS speed drift, overspeed, and sub-bucket numeric/text meter ticks;
// overspeed is handled directly on the Mazda HUD instead of CarPlay Live Activity).
inline bool isCphSemanticChange(const CphTelemetryState &prev,
                                const CphTelemetryState &curr) {
  if (prev.vietmapConnected   != curr.vietmapConnected)   return true;
  if (prev.postedSpeedLimit   != curr.postedSpeedLimit)   return true;
  if (prev.upcomingSpeedLimit != curr.upcomingSpeedLimit) return true;
  if (prev.activeSignCode     != curr.activeSignCode)     return true;
  if (strcmp(prev.activeSignText, curr.activeSignText) != 0) {
    uint16_t prevTxtBucket = 0;
    uint16_t currTxtBucket = 0;
    if (quantizeOverlayMeterText(prev.activeSignText, prevTxtBucket) &&
        quantizeOverlayMeterText(curr.activeSignText, currTxtBucket)) {
      if (prevTxtBucket != currTxtBucket) return true;
    } else {
      return true;
    }
  }
  if (curr.activeSignText[0] == '\0' && curr.upcomingSpeedLimit != 0 &&
      quantizeApproachBucket(prev.upcomingSpeedDistM) !=
          quantizeApproachBucket(curr.upcomingSpeedDistM)) {
    return true;
  }
  if (curr.activeSignText[0] == '\0' && curr.activeSignCode != 0 &&
      quantizeApproachBucket(prev.activeSignDistM) !=
          quantizeApproachBucket(curr.activeSignDistM)) {
    return true;
  }
  return false;
}

}  // namespace vmhud
