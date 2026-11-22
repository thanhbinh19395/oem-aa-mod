// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Template-match OCR for VietMap CarPlay speed-limit badges on the CMU.
//
// Design constraints (CMU150 / i.MX6 Dual @ ~675 MHz, carplayd pinned to
// one core): no Tesseract, no OpenCV — just a 12-class NCC against the
// embedded 120x120 grayscale badge art, run at 1–2 Hz on a small ROI.

#include "ocr.h"
#include "templates_embed.h"

#define LOG_TAG "OCR"
#include "log.h"
#include "common/config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <atomic>

namespace splim_ocr {
namespace {

constexpr const char *kSplimPath = "/data_persist/splim";
constexpr const char *kDumpPath  = "/tmp/splim_roi.pgm";

// Working resolution for matching (templates downsampled once at init).
constexpr int kMatch = 60;

struct PrepTpl {
    uint16_t speed_kmh;
    float    mean;
    float    inv_norm;          // 1 / sqrt(sum((p-mean)^2))
    uint8_t  y[kMatch * kMatch];
};

PrepTpl g_tpl[kTplN];
bool    g_tpl_ready = false;

std::atomic<uint64_t> g_last_ns{0};
uint16_t              g_last_speed = 0;
int                   g_streak     = 0;

static uint64_t mono_ns()
{
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull +
           static_cast<uint64_t>(ts.tv_nsec);
}

static void downsample_nn(const uint8_t *src, int sw, int sh,
                          uint8_t *dst, int dw, int dh)
{
    for (int y = 0; y < dh; ++y) {
        int sy = y * sh / dh;
        for (int x = 0; x < dw; ++x) {
            int sx = x * sw / dw;
            dst[y * dw + x] = src[sy * sw + sx];
        }
    }
}

static void prepare_templates()
{
    if (g_tpl_ready) return;
    for (size_t i = 0; i < kTplN; ++i) {
        PrepTpl &t = g_tpl[i];
        t.speed_kmh = kTemplates[i].speed_kmh;
        downsample_nn(kTemplates[i].y, kTplW, kTplH, t.y, kMatch, kMatch);

        double sum = 0.0;
        const int n = kMatch * kMatch;
        for (int k = 0; k < n; ++k) sum += t.y[k];
        t.mean = static_cast<float>(sum / n);

        double ss = 0.0;
        for (int k = 0; k < n; ++k) {
            double d = static_cast<double>(t.y[k]) - t.mean;
            ss += d * d;
        }
        t.inv_norm = (ss > 1.0) ? static_cast<float>(1.0 / sqrt(ss)) : 0.0f;
    }
    g_tpl_ready = true;
    LOGD("templates ready: %zu classes @ %dx%d", kTplN, kMatch, kMatch);
}

// Sample luma from a planar/semi-planar frame at integer pixel (x,y).
static inline uint8_t sample_y(const FrameView &f, int x, int y)
{
    if (x < 0 || y < 0 || x >= f.width || y >= f.height) return 0;
    // All supported formats store Y as the first plane, tightly packed
    // (or with stride == width — CarPlay vpudec buffers on CMU are tight).
    return f.data[static_cast<size_t>(y) * static_cast<size_t>(f.width) +
                  static_cast<size_t>(x)];
}

static bool frame_size_plausible(const FrameView &f)
{
    if (f.width < 64 || f.height < 64 || f.width > 1920 || f.height > 1080)
        return false;
    if (!f.data || f.size == 0) return false;
    const uint32_t y_bytes =
        static_cast<uint32_t>(f.width) * static_cast<uint32_t>(f.height);
    switch (f.fmt) {
    case FMT_Y800:
        return f.size >= y_bytes;
    case FMT_I420:
    case FMT_YV12:
    case FMT_NV12:
        return f.size >= y_bytes + y_bytes / 2;
    default:
        // Unknown (possibly tiled): only proceed if a Y-plane-sized
        // prefix is present; matching may still fail on tiled layouts.
        return f.size >= y_bytes;
    }
}

static void resolve_roi(const FrameView &f, int &rx, int &ry, int &rw, int &rh)
{
    const auto &s = libpatch_config::settings();
    rw = s.carplay_ocr_roi_w > 0 ? s.carplay_ocr_roi_w : 160;
    rh = s.carplay_ocr_roi_h > 0 ? s.carplay_ocr_roi_h : 160;
    if (rw > f.width)  rw = f.width;
    if (rh > f.height) rh = f.height;

    // Auto: lower-left — VietMap draws CPSpeedAndLimit near the speed
    // cluster. Wired CarPlay on CMU150 is typically ~800x480.
    if (s.carplay_ocr_roi_x < 0)
        rx = 16;
    else
        rx = s.carplay_ocr_roi_x;
    if (s.carplay_ocr_roi_y < 0)
        ry = f.height - rh - 16;
    else
        ry = s.carplay_ocr_roi_y;

    if (rx < 0) rx = 0;
    if (ry < 0) ry = 0;
    if (rx + rw > f.width)  rx = f.width  - rw;
    if (ry + rh > f.height) ry = f.height - rh;
}

static void extract_roi_gray(const FrameView &f, int rx, int ry, int rw, int rh,
                             uint8_t *out_match /*kMatch*kMatch*/)
{
    // Nearest-neighbour scale ROI -> kMatch x kMatch.
    for (int y = 0; y < kMatch; ++y) {
        int sy = ry + y * rh / kMatch;
        for (int x = 0; x < kMatch; ++x) {
            int sx = rx + x * rw / kMatch;
            out_match[y * kMatch + x] = sample_y(f, sx, sy);
        }
    }
}

static float ncc(const uint8_t *patch, const PrepTpl &t)
{
    if (t.inv_norm <= 0.0f) return -1.0f;
    const int n = kMatch * kMatch;
    double sum = 0.0;
    for (int k = 0; k < n; ++k) sum += patch[k];
    const double mean = sum / n;

    double dot = 0.0;
    double ss  = 0.0;
    for (int k = 0; k < n; ++k) {
        double d = static_cast<double>(patch[k]) - mean;
        ss  += d * d;
        dot += d * (static_cast<double>(t.y[k]) - t.mean);
    }
    if (ss < 1.0) return -1.0f;
    return static_cast<float>(dot * t.inv_norm / sqrt(ss));
}

static void dump_pgm(const uint8_t *patch)
{
    FILE *f = fopen(kDumpPath, "wb");
    if (!f) return;
    fprintf(f, "P5\n%d %d\n255\n", kMatch, kMatch);
    fwrite(patch, 1, static_cast<size_t>(kMatch * kMatch), f);
    fclose(f);
}

static void write_splim(uint16_t speed_kmh)
{
    // Atomic replace: write temp then rename so the HUD reader never
    // observes a truncated line.
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "%s.tmp", kSplimPath);
    FILE *f = fopen(tmp, "w");
    if (!f) {
        LOGE("splim: fopen(%s) failed", tmp);
        return;
    }
    const long ts = static_cast<long>(time(nullptr));
    fprintf(f, "%u %ld\n", static_cast<unsigned>(speed_kmh), ts);
    fclose(f);
    if (rename(tmp, kSplimPath) != 0) {
        LOGE("splim: rename to %s failed", kSplimPath);
        unlink(tmp);
        return;
    }
    LOGD("splim: wrote %u km/h", static_cast<unsigned>(speed_kmh));
}

} // namespace

void process_frame(const FrameView &frame)
{
    if (!libpatch_config::carplay_ocr()) return;
    if (!frame_size_plausible(frame)) {
        LOGD("ocr: skip implausible frame %dx%d size=%u fmt=%d",
             frame.width, frame.height, frame.size, static_cast<int>(frame.fmt));
        return;
    }

    prepare_templates();

    const int hz = libpatch_config::carplay_ocr_hz();
    const uint64_t min_period_ns =
        1000000000ull / static_cast<uint64_t>(hz > 0 ? hz : 2);
    const uint64_t now = mono_ns();
    const uint64_t last = g_last_ns.load(std::memory_order_relaxed);
    if (now - last < min_period_ns) return;
    g_last_ns.store(now, std::memory_order_relaxed);

    int rx, ry, rw, rh;
    resolve_roi(frame, rx, ry, rw, rh);

    uint8_t patch[kMatch * kMatch];
    extract_roi_gray(frame, rx, ry, rw, rh, patch);

    if (libpatch_config::carplay_ocr_dump()) {
        dump_pgm(patch);
    }

    const float min_ncc = libpatch_config::carplay_ocr_min_ncc();
    float best_score = -1.0f;
    uint16_t best_speed = 0;
    for (size_t i = 0; i < kTplN; ++i) {
        float s = ncc(patch, g_tpl[i]);
        if (s > best_score) {
            best_score = s;
            best_speed = g_tpl[i].speed_kmh;
        }
    }

    LOGV("ocr: roi=(%d,%d %dx%d) best=%u score=%.3f (min=%.3f)",
         rx, ry, rw, rh, static_cast<unsigned>(best_speed), best_score, min_ncc);

    if (best_score < min_ncc || best_speed == 0) {
        g_streak = 0;
        return;
    }

    if (best_speed == g_last_speed) {
        ++g_streak;
    } else {
        g_last_speed = best_speed;
        g_streak = 1;
    }

    // Require two consecutive agreeing reads before publishing — kills
    // single-frame false positives from map chrome that vaguely matches.
    if (g_streak >= 2) {
        write_splim(best_speed);
    }
}

} // namespace splim_ocr
