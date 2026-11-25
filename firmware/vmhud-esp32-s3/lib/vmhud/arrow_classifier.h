// SPDX-License-Identifier: AGPL-3.0-or-later
// On-device VietMap CMD 0x40 JPEG turn-arrow classifier -> Mazda HUD MazdaIcon.
//
// VietMap Live iOS sends direction=0xFE on all maneuvers over DJBT BLE (FFF0);
// the real turn arrow is a ~2 KB JPEG embedded in CMD 0x40 (when HUDInfo
// PROTOCOL does not contain "S", e.g. "2.2.2").
//
// This module decodes the JPEG in streaming MCU blocks (zero full-frame buffer,
// using ESP32 ROM TJpgDec on-chip), box-downsamples to a 16x16 luma grid, and
// matches via normalized cross-correlation (NCC) against the quantized template
// table in arrow_templates.h.
//
// Verified across all 212 non-blank VietMap direction assets (JPEG q=65):
//   - true arrows:     score >= 0.988
//   - non-arrow icons: score <= 0.731
// Acceptance threshold 0.86 cleanly separates arrows from weather/idle/alert icons.
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "arrow_templates.h"

#ifdef ESP_PLATFORM
#include "esp_rom_tjpgd.h"
#endif

namespace vmhud {

static constexpr uint8_t kMazdaIconNone = 0;
static constexpr uint8_t kMazdaIconStraight = 1;
static constexpr float kArrowMinScore = 0.86f;
static constexpr float kArrowMinMargin = 0.01f;

struct ArrowResult {
    uint8_t icon = kMazdaIconNone;
    float   score = 0.0f;
    float   margin = 0.0f;
    bool    confident = false;
};

struct Box16Accumulator {
    uint32_t sum[256];
    uint16_t cnt[256];
    uint32_t imgW;
    uint32_t imgH;

    void reset(uint32_t w, uint32_t h) {
        memset(sum, 0, sizeof(sum));
        memset(cnt, 0, sizeof(cnt));
        imgW = w;
        imgH = h;
    }

    inline void addRgbRect(uint32_t left, uint32_t top, uint32_t right, uint32_t bottom,
                           const uint8_t *rgb) {
        if (!imgW || !imgH || right < left || bottom < top) return;
        const uint32_t rw = right - left + 1u;
        const uint8_t *p = rgb;

        // Fast path for TJpgDec MCU blocks (rw <= 16): precompute x->dx once per block
        // instead of dividing on every single pixel (~60x fewer integer divisions).
        if (rw <= 16u) {
            uint8_t dxMap[16];
            for (uint32_t i = 0; i < rw; ++i) {
                uint32_t x = left + i;
                dxMap[i] = (x < imgW) ? static_cast<uint8_t>((x * 16u) / imgW) : 0xFFu;
            }
            for (uint32_t y = top; y <= bottom; ++y) {
                if (y >= imgH) {
                    p += rw * 3u;
                    continue;
                }
                const uint32_t rowBase = ((y * 16u) / imgH) * 16u;
                for (uint32_t i = 0; i < rw; ++i) {
                    const uint32_t r = p[0];
                    const uint32_t g = p[1];
                    const uint32_t b = p[2];
                    p += 3;
                    const uint8_t dx = dxMap[i];
                    if (dx == 0xFFu) continue;
                    const uint32_t luma = (77u * r + 150u * g + 29u * b) >> 8;
                    const uint32_t idx = rowBase + dx;
                    sum[idx] += luma;
                    cnt[idx] += 1u;
                }
            }
            return;
        }

        // General path (used by host tests passing a full image rect)
        for (uint32_t y = top; y <= bottom; ++y) {
            if (y >= imgH) {
                p += rw * 3u;
                continue;
            }
            const uint32_t rowBase = ((y * 16u) / imgH) * 16u;
            for (uint32_t x = left; x <= right; ++x) {
                const uint32_t r = p[0];
                const uint32_t g = p[1];
                const uint32_t b = p[2];
                p += 3;
                if (x >= imgW) continue;
                const uint32_t dx = (x * 16u) / imgW;
                const uint32_t luma = (77u * r + 150u * g + 29u * b) >> 8;
                const uint32_t idx = rowBase + dx;
                sum[idx] += luma;
                cnt[idx] += 1u;
            }
        }
    }

    void finalize(uint8_t outLuma[256]) const {
        for (size_t i = 0; i < 256; ++i) {
            outLuma[i] = cnt[i] ? static_cast<uint8_t>(sum[i] / cnt[i]) : 0u;
        }
    }
};

// Match a 16x16 grayscale grid against kArrowTemplates using NCC.
// Low-stack design (~600 B total): no 1 KB centered float buffer, and precomputed
// invNorm in kArrowTemplates eliminates all per-template sqrtf() and divisions.
inline bool classifyGrid16(const uint8_t luma[256], ArrowResult &out) {
    out = ArrowResult{};

    int32_t total = 0;
    for (size_t i = 0; i < 256; ++i) total += luma[i];
    const float mean = static_cast<float>(total) * (1.0f / 256.0f);

    float maxAbs = 0.0f;
    for (size_t i = 0; i < 256; ++i) {
        float v = static_cast<float>(luma[i]) - mean;
        float av = v < 0.0f ? -v : v;
        if (av > maxAbs) maxAbs = av;
    }
    if (maxAbs < 1.0f) {
        return false;  // blank / uniform image
    }

    int8_t q[256];
    uint32_t qNormSq = 0;
    const float scale = 127.0f / maxAbs;
    for (size_t i = 0; i < 256; ++i) {
        float sv = (static_cast<float>(luma[i]) - mean) * scale;
        int iv = static_cast<int>(sv >= 0.0f ? (sv + 0.5f) : (sv - 0.5f));
        if (iv > 127) iv = 127;
        if (iv < -127) iv = -127;
        q[i] = static_cast<int8_t>(iv);
        qNormSq += static_cast<uint32_t>(iv * iv);
    }
    if (qNormSq == 0) {
        return false;
    }
    const float invQNorm = 1.0f / sqrtf(static_cast<float>(qNormSq));

    float bestScore = -2.0f;
    uint8_t bestIcon = kMazdaIconNone;

    // Track best score per distinct MazdaIcon so identical/theme variants of the
    // same icon don't collapse the margin to 0.
    float iconBest[64];
    for (size_t i = 0; i < 64; ++i) iconBest[i] = -2.0f;

    for (size_t t = 0; t < kArrowTemplateCount; ++t) {
        const ArrowTemplate &tmpl = kArrowTemplates[t];
        int32_t dot = 0;
        const int8_t *tv = tmpl.vec;
        for (size_t i = 0; i < 256; ++i) {
            dot += static_cast<int32_t>(q[i]) * static_cast<int32_t>(tv[i]);
        }
        const float score = static_cast<float>(dot) * invQNorm * tmpl.invNorm;
        if (tmpl.icon < 64 && score > iconBest[tmpl.icon]) {
            iconBest[tmpl.icon] = score;
        }
        if (score > bestScore) {
            bestScore = score;
            bestIcon = tmpl.icon;
        }
    }

    float runnerUp = -2.0f;
    for (size_t ic = 0; ic < 64; ++ic) {
        if (ic == bestIcon) continue;
        if (iconBest[ic] > runnerUp) runnerUp = iconBest[ic];
    }

    out.icon = bestIcon;
    out.score = bestScore;
    out.margin = (runnerUp > -1.5f) ? (bestScore - runnerUp) : bestScore;
    out.confident = (bestIcon != kMazdaIconNone) &&
                    (out.score >= kArrowMinScore) &&
                    (out.margin >= kArrowMinMargin);
    return out.confident;
}

inline bool classifyRgbImage(const uint8_t *rgb, uint32_t w, uint32_t h, ArrowResult &out) {
    if (!rgb || !w || !h) {
        out = ArrowResult{};
        return false;
    }
    Box16Accumulator acc;
    acc.reset(w, h);
    acc.addRgbRect(0, 0, w - 1, h - 1, rgb);
    uint8_t luma[256];
    acc.finalize(luma);
    return classifyGrid16(luma, out);
}

#ifdef ESP_PLATFORM
namespace detail {

struct JpegDecodeCtx {
    const uint8_t *data;
    size_t         len;
    size_t         pos;
    Box16Accumulator acc;
};

inline uint32_t jpegInputCb(esp_rom_tjpgd_dec_t *dec, uint8_t *buf, uint32_t ndata) {
    JpegDecodeCtx *ctx = static_cast<JpegDecodeCtx *>(dec->device);
    if (!ctx || ctx->pos >= ctx->len) return 0;
    size_t rem = ctx->len - ctx->pos;
    size_t take = (static_cast<size_t>(ndata) < rem) ? static_cast<size_t>(ndata) : rem;
    if (buf) {
        memcpy(buf, ctx->data + ctx->pos, take);
    }
    ctx->pos += take;
    return static_cast<uint32_t>(take);
}

inline uint32_t jpegOutputCb(esp_rom_tjpgd_dec_t *dec, void *bitmap, esp_rom_tjpgd_rect_t *rect) {
    JpegDecodeCtx *ctx = static_cast<JpegDecodeCtx *>(dec->device);
    if (!ctx || !bitmap || !rect) return 0;
    ctx->acc.addRgbRect(rect->left, rect->top, rect->right, rect->bottom,
                        static_cast<const uint8_t *>(bitmap));
    return 1;
}

} // namespace detail

// Decode a baseline JPEG into a 16x16 grayscale luma grid using ESP32 ROM TJpgDec.
// Must be called single-threaded from loop() (never from NimBLE host task).
// Shared static work/ctx buffers keep stack usage minimal (~600 B) and avoid
// duplicating the ~4.5 KB TJpgDec workspace across arrow & speed-sign classifiers.
inline bool decodeJpegToLuma16(const uint8_t *jpeg, size_t len, uint8_t outLuma[256]) {
    if (!jpeg || len < 16 || jpeg[0] != 0xFF || jpeg[1] != 0xD8) {
        return false;
    }
    alignas(4) static uint8_t s_work[4096];
    static detail::JpegDecodeCtx s_ctx;
    esp_rom_tjpgd_dec_t dec;
    s_ctx.data = jpeg;
    s_ctx.len = len;
    s_ctx.pos = 0;

    if (esp_rom_tjpgd_prepare(&dec, detail::jpegInputCb, s_work, sizeof(s_work), &s_ctx) != JDR_OK) {
        return false;
    }
    if (dec.width == 0 || dec.height == 0 || dec.width > 512 || dec.height > 512) {
        return false;
    }
    s_ctx.acc.reset(dec.width, dec.height);
    if (esp_rom_tjpgd_decomp(&dec, detail::jpegOutputCb, 0) != JDR_OK) {
        return false;
    }
    s_ctx.acc.finalize(outLuma);
    return true;
}

// Decode a baseline JPEG (e.g. VietMap CMD 0x40 arrow) using ESP32 ROM TJpgDec
// and classify it into a MazdaIcon. Must be called single-threaded from loop()
// (never from NimBLE host task).
inline bool classifyJpeg(const uint8_t *jpeg, size_t len, ArrowResult &out) {
    out = ArrowResult{};
    uint8_t luma[256];
    if (!decodeJpegToLuma16(jpeg, len, luma)) {
        return false;
    }
    return classifyGrid16(luma, out);
}
#endif // ESP_PLATFORM

} // namespace vmhud
