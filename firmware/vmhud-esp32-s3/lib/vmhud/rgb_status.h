// SPDX-License-Identifier: AGPL-3.0-or-later
// Portable slow-blink RGB status LED state machine for Headless ESP32-S3 Super Mini.
// Cycles sequentially through whichever links are still waiting to connect:
//   - VietMap Live waiting   -> Red
//   - Mazda CMU waiting      -> Yellow
//   - CarPlay Helper waiting -> Blue
// Turns completely OFF once all 3 links are connected.
#pragma once

#include <stdint.h>

namespace vmhud {

static constexpr uint8_t  kDefaultRgbBrightness = 32;
static constexpr uint32_t kRgbBlinkOnMs         = 400;
static constexpr uint32_t kRgbBlinkOffMs        = 800;
static constexpr uint32_t kRgbBlinkPeriodMs     = kRgbBlinkOnMs + kRgbBlinkOffMs;

struct RgbColor {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  RgbColor(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0)
      : r(red), g(green), b(blue) {}

  bool isOff() const { return r == 0 && g == 0 && b == 0; }
};

inline bool operator==(const RgbColor &a, const RgbColor &b) {
  return a.r == b.r && a.g == b.g && a.b == b.b;
}

inline bool operator!=(const RgbColor &a, const RgbColor &b) {
  return !(a == b);
}

struct RgbConnState {
  bool vietmapConnected;
  bool cmuConnected;
  bool cphConnected;

  RgbConnState(bool vm = false, bool cmu = false, bool cph = false)
      : vietmapConnected(vm), cmuConnected(cmu), cphConnected(cph) {}

  bool allConnected() const {
    return vietmapConnected && cmuConnected && cphConnected;
  }
};

// Scales the green channel relative to red (~5/8) so WS2812 renders a warm
// amber/golden yellow rather than greenish-lime at equal PWM duty.
inline uint8_t rgbYellowGreen(uint8_t brightness) {
  if (brightness == 0) return 0;
  const uint8_t g = static_cast<uint8_t>((uint16_t(brightness) * 5u + 4u) / 8u);
  return (g == 0 && brightness >= 2u) ? 1u : g;
}

class RgbStatusBlinker {
 public:
  enum Slot : uint8_t {
    kSlotVietMap = 0,  // Red
    kSlotCmu     = 1,  // Yellow
    kSlotCph     = 2,  // Blue
    kSlotCount   = 3
  };

  RgbStatusBlinker() : slotStartMs_(0), currentSlot_(0), active_(false) {}

  void reset() {
    slotStartMs_ = 0;
    currentSlot_ = 0;
    active_ = false;
  }

  static bool isSlotPending(const RgbConnState &st, uint8_t slot) {
    switch (slot) {
      case kSlotVietMap: return !st.vietmapConnected;
      case kSlotCmu:     return !st.cmuConnected;
      case kSlotCph:     return !st.cphConnected;
      default:           return false;
    }
  }

  static RgbColor slotColor(uint8_t slot, uint8_t brightness = kDefaultRgbBrightness) {
    if (brightness == 0) return {0, 0, 0};
    switch (slot) {
      case kSlotVietMap: return {brightness, 0, 0};
      case kSlotCmu:     return {brightness, rgbYellowGreen(brightness), 0};
      case kSlotCph:     return {0, 0, brightness};
      default:           return {0, 0, 0};
    }
  }

  RgbColor update(const RgbConnState &st, uint32_t nowMs,
                  uint8_t brightness = kDefaultRgbBrightness) {
    if (brightness == 0 || st.allConnected()) {
      active_ = false;
      return {0, 0, 0};
    }

    if (!active_) {
      active_ = true;
      currentSlot_ = firstPendingSlot(st);
      slotStartMs_ = nowMs;
      return slotColor(currentSlot_, brightness);
    }

    uint32_t elapsed = nowMs - slotStartMs_;

    // If the currently blinking connection completed mid-pulse, turn off its color
    // immediately and preserve the standard dark pause before showing the next color.
    if (!isSlotPending(st, currentSlot_) && elapsed < kRgbBlinkOnMs) {
      slotStartMs_ = nowMs - kRgbBlinkOnMs;
      elapsed = kRgbBlinkOnMs;
    }

    if (elapsed >= kRgbBlinkPeriodMs) {
      currentSlot_ = nextPendingSlot(st, currentSlot_);
      if (elapsed >= 2u * kRgbBlinkPeriodMs) {
        slotStartMs_ = nowMs;
      } else {
        slotStartMs_ += kRgbBlinkPeriodMs;
      }
      elapsed = nowMs - slotStartMs_;
    }

    if (isSlotPending(st, currentSlot_) && elapsed < kRgbBlinkOnMs) {
      return slotColor(currentSlot_, brightness);
    }
    return {0, 0, 0};
  }

 private:
  static uint8_t firstPendingSlot(const RgbConnState &st) {
    for (uint8_t s = 0; s < kSlotCount; ++s) {
      if (isSlotPending(st, s)) return s;
    }
    return 0;
  }

  static uint8_t nextPendingSlot(const RgbConnState &st, uint8_t afterSlot) {
    for (uint8_t step = 1; step <= kSlotCount; ++step) {
      const uint8_t candidate = static_cast<uint8_t>((afterSlot + step) % kSlotCount);
      if (isSlotPending(st, candidate)) return candidate;
    }
    return 0;
  }

  uint32_t slotStartMs_;
  uint8_t  currentSlot_;
  bool     active_;
};

}  // namespace vmhud
