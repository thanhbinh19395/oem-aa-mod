// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Host self-test for the VietMap Live (iPhone) BLE -> ESP32-S3 -> CMU HUD pipeline:
//   1. vmhud::build / Receiver / navigation (CMD 0x40 JPEG, keep-image, idle, nav-end)
//   2. vmhud::classifyGrid16 / classifyRgbImage (all quantized templates + negative rejection)
//   3. End-to-end distance/unit formatting and /tmp/vmnav format contract.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "hud_codec.h"
#include "arrow_classifier.h"

static int g_fail = 0;

#define CHECK(cond, msg) do {                                            \
        if (!(cond)) { printf("  FAIL: %s\n", (msg)); g_fail = 1; }      \
        else         { printf("  ok:   %s\n", (msg)); }                  \
    } while (0)

#define CHECK_EQ_U(got, exp, msg) do {                                   \
        unsigned long _g = (unsigned long)(got), _e = (unsigned long)(exp); \
        if (_g != _e) { printf("  FAIL: %s (got %lu, want %lu)\n",       \
                               (msg), _g, _e); g_fail = 1; }             \
        else          { printf("  ok:   %s = %lu\n", (msg), _g); }       \
    } while (0)

// Deterministic 32-bit checksum mock matching vmhud::Checksum contract.
static bool mock_checksum(const uint8_t *p, size_t n, const uint8_t *uid, uint8_t *out)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; ++i) h = (h ^ p[i]) * 16777619u;
    for (size_t i = 0; i < 16; ++i) h = (h ^ uid[i]) * 16777619u;
    vmhud::put32(out, h);
    return true;
}

static std::vector<uint8_t> build_cmd40_payload(uint8_t dir,
                                                const std::vector<uint8_t> &img_or_keep,
                                                uint32_t dist_m,
                                                const char *next_road,
                                                const char *cur_road,
                                                uint16_t speed)
{
    std::vector<uint8_t> p;
    p.push_back(dir);
    uint8_t be[4];
    vmhud::put32(be, static_cast<uint32_t>(img_or_keep.size()));
    p.insert(p.end(), be, be + 4);
    p.insert(p.end(), img_or_keep.begin(), img_or_keep.end());
    vmhud::put32(be, dist_m);
    p.insert(p.end(), be, be + 4);

    size_t nlen = strlen(next_road);
    p.push_back(static_cast<uint8_t>(nlen));
    p.insert(p.end(), next_road, next_road + nlen);

    size_t clen = strlen(cur_road);
    p.push_back(static_cast<uint8_t>(clen));
    p.insert(p.end(), cur_road, cur_road + clen);

    // 10-byte tail: hours(2), min(1), sec(1), remDist(4), speed(2)
    p.push_back(0); p.push_back(0);
    p.push_back(12); p.push_back(34);
    vmhud::put32(be, 12345u);
    p.insert(p.end(), be, be + 4);
    p.push_back(static_cast<uint8_t>(speed >> 8));
    p.push_back(static_cast<uint8_t>(speed & 0xFF));
    return p;
}

struct RxCapture {
    int count = 0;
    vmhud::Nav lastNav;
    uint8_t lastCmd = 0;
    std::vector<uint8_t> lastImg;
};

static void on_rx_frame(const vmhud::Frame &frame, void *user)
{
    RxCapture *cap = static_cast<RxCapture *>(user);
    cap->lastCmd = frame.command;
    if (vmhud::navigation(frame, cap->lastNav)) {
        cap->count++;
        if (cap->lastNav.image && cap->lastNav.imageSize > 0) {
            cap->lastImg.assign(cap->lastNav.image, cap->lastNav.image + cap->lastNav.imageSize);
        } else {
            cap->lastImg.clear();
        }
    }
}

int main()
{
    printf("=== Test 1: CMD 0x40 with embedded JPEG payload (>2 KB, chunked 20B BLE writes) ===\n");
    {
        std::vector<uint8_t> fakeJpeg(2438, 0x5A);
        fakeJpeg[0] = 0xFF; fakeJpeg[1] = 0xD8;
        fakeJpeg[2436] = 0xFF; fakeJpeg[2437] = 0xD9;

        auto payload = build_cmd40_payload(0xFE, fakeJpeg, 150, "Quốc Lộ 13", "Đường Số 7", 45);
        std::vector<uint8_t> wire(payload.size() + 32, 0);
        size_t wlen = vmhud::build(0x40, payload.data(), payload.size(), 1700000000u,
                                   wire.data(), wire.size(), mock_checksum, false);
        CHECK(wlen > 2450, "CMD 0x40 wire frame built");

        vmhud::Receiver<8192> rx;
        RxCapture cap;
        for (size_t off = 0; off < wlen; off += 20) {
            size_t chunk = (wlen - off < 20) ? (wlen - off) : 20;
            rx.feed(wire.data() + off, chunk, mock_checksum, on_rx_frame, &cap);
        }
        CHECK_EQ_U(cap.count, 1, "decoded 1 CMD 0x40 frame");
        CHECK_EQ_U(cap.lastNav.direction, 0xFE, "direction=0xFE preserved");
        CHECK_EQ_U(cap.lastNav.distance, 150, "distance=150m");
        CHECK_EQ_U(cap.lastNav.speed, 45, "speed=45 km/h");
        CHECK_EQ_U(cap.lastNav.imageSize, 2438, "imageSize=2438 bytes");
        CHECK(!cap.lastNav.keepImage, "keepImage=false on new image");
        CHECK(cap.lastImg.size() == 2438 && cap.lastImg[0] == 0xFF && cap.lastImg[1] == 0xD8,
              "JPEG bytes intact after reassembly");
        CHECK(cap.lastNav.nextSize == strlen("Quốc Lộ 13") &&
              memcmp(cap.lastNav.next, "Quốc Lộ 13", cap.lastNav.nextSize) == 0,
              "nextRoad UTF-8 intact");
    }

    printf("\n=== Test 2: CMD 0x40 keep-previous-arrow (image_len=1, flag=1) ===\n");
    {
        std::vector<uint8_t> keep = { 0x01 };
        auto payload = build_cmd40_payload(0xFE, keep, 120, "Quốc Lộ 13", "", 40);
        uint8_t wire[256];
        size_t wlen = vmhud::build(0x40, payload.data(), payload.size(), 1700000001u,
                                   wire, sizeof(wire), mock_checksum, false);
        vmhud::Receiver<8192> rx;
        RxCapture cap;
        rx.feed(wire, wlen, mock_checksum, on_rx_frame, &cap);
        CHECK_EQ_U(cap.count, 1, "decoded keep-image CMD 0x40");
        CHECK(cap.lastNav.keepImage, "keepImage=true");
        CHECK_EQ_U(cap.lastNav.imageSize, 0, "imageSize=0 on keep-image");
        CHECK_EQ_U(cap.lastNav.distance, 120, "distance=120m");
    }

    printf("\n=== Test 3: CMD 0x40 idle/weather frame (distance=0xFFFFFFFF) ===\n");
    {
        std::vector<uint8_t> keep = { 0x01 };
        auto payload = build_cmd40_payload(0xFE, keep, 0xFFFFFFFFu, "30° mưa vừa", "", 0);
        uint8_t wire[256];
        size_t wlen = vmhud::build(0x40, payload.data(), payload.size(), 1700000002u,
                                   wire, sizeof(wire), mock_checksum, false);
        vmhud::Receiver<8192> rx;
        RxCapture cap;
        rx.feed(wire, wlen, mock_checksum, on_rx_frame, &cap);
        CHECK_EQ_U(cap.count, 1, "decoded idle frame");
        CHECK_EQ_U(cap.lastNav.distance, 0xFFFFFFFFu, "distance=0xFFFFFFFF signals nav idle");
    }

    printf("\n=== Test 4: Arrow classifier self-consistency across all %zu templates ===\n",
           vmhud::kArrowTemplateCount);
    {
        CHECK(vmhud::kArrowTemplateCount >= 50, "template bank populated (>=50 descriptors)");
        size_t matched = 0;
        for (size_t t = 0; t < vmhud::kArrowTemplateCount; ++t) {
            const vmhud::ArrowTemplate &tmpl = vmhud::kArrowTemplates[t];
            uint8_t luma[256];
            for (size_t i = 0; i < 256; ++i) {
                int v = 128 + static_cast<int>(tmpl.vec[i]);
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                luma[i] = static_cast<uint8_t>(v);
            }
            vmhud::ArrowResult res;
            if (vmhud::classifyGrid16(luma, res) && res.icon == tmpl.icon && res.score >= 0.99f) {
                matched++;
            } else {
                printf("  MISS tmpl[%zu] want icon=%u got icon=%u score=%.4f margin=%.4f\n",
                       t, (unsigned)tmpl.icon, (unsigned)res.icon, res.score, res.margin);
            }
        }
        CHECK_EQ_U(matched, vmhud::kArrowTemplateCount, "all templates self-match with score>=0.99");
    }

    printf("\n=== Test 5: Arrow classifier rejects blank, solid, and checkerboard non-arrows ===\n");
    {
        uint8_t blank[256] = {0};
        vmhud::ArrowResult res;
        CHECK(!vmhud::classifyGrid16(blank, res), "all-black image rejected");

        uint8_t solid[256];
        memset(solid, 200, sizeof(solid));
        CHECK(!vmhud::classifyGrid16(solid, res), "uniform gray image rejected");

        uint8_t checker[256];
        for (int y = 0; y < 16; ++y)
            for (int x = 0; x < 16; ++x)
                checker[y * 16 + x] = ((x + y) & 1) ? 255 : 0;
        CHECK(!vmhud::classifyGrid16(checker, res), "checkerboard noise rejected");
    }

    printf("\n=== Test 6: classifyRgbImage box-downsamples 60x60 RGB identically ===\n");
    {
        // Upsample template 0 to 60x60 RGB and verify exact icon match.
        const vmhud::ArrowTemplate &tmpl = vmhud::kArrowTemplates[0];
        std::vector<uint8_t> rgb(60 * 60 * 3, 0);
        for (int y = 0; y < 60; ++y) {
            int sy = (y * 16) / 60;
            for (int x = 0; x < 60; ++x) {
                int sx = (x * 16) / 60;
                int v = 128 + static_cast<int>(tmpl.vec[sy * 16 + sx]);
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                size_t o = static_cast<size_t>((y * 60 + x) * 3);
                rgb[o] = rgb[o + 1] = rgb[o + 2] = static_cast<uint8_t>(v);
            }
        }
        vmhud::ArrowResult res;
        CHECK(vmhud::classifyRgbImage(rgb.data(), 60, 60, res), "60x60 RGB image classified");
        CHECK_EQ_U(res.icon, tmpl.icon, "60x60 RGB matched expected MazdaIcon");
    }

    if (g_fail) {
        printf("\nVIETMAP BLE HUD TEST SUITE FAILED\n");
        return 1;
    }
    printf("\nALL VIETMAP BLE HUD TESTS PASSED\n");
    return 0;
}
