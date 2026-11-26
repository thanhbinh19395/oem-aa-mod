#pragma once

#include <Arduino.h>
#include "pin_config.h"
#include "rgb_status.h"

#if defined(VMHUD_HAS_RGB_LED) && VMHUD_HAS_RGB_LED

namespace vmhud_hw {

inline void writeRgbPixel(const vmhud::RgbColor &c) {
#if VMHUD_RGB_SWAP_RG
  neopixelWrite(VMHUD_RGB_PIN, c.g, c.r, c.b);
#else
  neopixelWrite(VMHUD_RGB_PIN, c.r, c.g, c.b);
#endif
}

inline vmhud::RgbStatusBlinker &blinker() {
  static vmhud::RgbStatusBlinker inst;
  return inst;
}

inline vmhud::RgbColor &lastColor() {
  static vmhud::RgbColor last{0, 0, 0};
  return last;
}

inline bool &initialized() {
  static bool init = false;
  return init;
}

}  // namespace vmhud_hw

inline void rgbLedInit() {
  vmhud_hw::blinker().reset();
  const vmhud::RgbColor off{0, 0, 0};
  vmhud_hw::writeRgbPixel(off);
  vmhud_hw::lastColor() = off;
  vmhud_hw::initialized() = true;
}

inline void rgbLedUpdate(const vmhud::RgbConnState &state, uint32_t nowMs) {
  const vmhud::RgbColor target =
      vmhud_hw::blinker().update(state, nowMs, static_cast<uint8_t>(VMHUD_RGB_BRIGHTNESS));
  if (!vmhud_hw::initialized() || target != vmhud_hw::lastColor()) {
    vmhud_hw::writeRgbPixel(target);
    vmhud_hw::lastColor() = target;
    vmhud_hw::initialized() = true;
  }
}

#else

inline void rgbLedInit() {}
inline void rgbLedUpdate(const vmhud::RgbConnState &, uint32_t) {}

#endif
