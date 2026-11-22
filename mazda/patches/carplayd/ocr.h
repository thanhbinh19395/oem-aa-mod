// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Template-match speed-limit OCR for VietMap CarPlay badges.
// Classifies a grayscale ROI against the embedded speed_limit_{10..120}
// templates and writes "/data_persist/splim" in the format the existing
// blmjcicarplay HUD reader expects: "<km/h> <unix_ts>\n".

#ifndef LIBPATCH_CARPLAYD_OCR_H
#define LIBPATCH_CARPLAYD_OCR_H

#include <stdint.h>

namespace splim_ocr {

enum PixelFormat {
    FMT_UNKNOWN = 0,
    FMT_I420,
    FMT_YV12,
    FMT_NV12,
    FMT_Y800,
};

struct FrameView {
    const uint8_t *data;
    uint32_t       size;
    int            width;
    int            height;
    PixelFormat    fmt;
};

// Run one classification pass. Rate-limits internally to config Hz.
// On a confident match, atomically updates /data_persist/splim.
void process_frame(const FrameView &frame);

} // namespace splim_ocr

#endif // LIBPATCH_CARPLAYD_OCR_H
