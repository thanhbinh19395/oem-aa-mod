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
#include "speed_sign_classifier.h"
#include "traffic_sign_classifier.h"
#include "cph_telemetry.h"
#include "blmjcicarplay/hud/next_splim_policy.h"
#include "common/translit.h"

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

    printf("\n=== Test 7: CMD 0x01 2-slot camera/speed parser (current vs upcoming) ===\n");
    {
        // Case A: Single-slot steady driving at 60 km/h
        uint8_t p1[4] = { 0x01, 0x00, 0x64, 60 };
        vmhud::Frame f1; f1.command = 1; f1.payload = p1; f1.size = sizeof(p1);
        auto r1 = vmhud::parseCameraFrame(f1, 0);
        CHECK_EQ_U(r1.currentLimit, 60, "single-slot current=60");
        CHECK_EQ_U(r1.nextLimit, 0, "single-slot next=0");

        // Case B: Speed camera on same 60 km/h road -> no next-speed alert
        uint8_t p2[8] = { 0x02, 0x01, 0x2C, 60,  0x01, 0x00, 0x64, 60 };
        vmhud::Frame f2; f2.command = 1; f2.payload = p2; f2.size = sizeof(p2);
        auto r2 = vmhud::parseCameraFrame(f2, 60);
        CHECK_EQ_U(r2.currentLimit, 60, "same-limit camera current=60");
        CHECK_EQ_U(r2.nextLimit, 0, "same-limit camera next=0");

        // Case C: VietMap iOS 2-slot upcoming speed limit 50 at 250m while currently on 60 (slot 1 dist=100)
        uint8_t p3[8] = { 0x01, 0x00, 0xFA, 50,  0x01, 0x00, 0x64, 60 };
        vmhud::Frame f3; f3.command = 1; f3.payload = p3; f3.size = sizeof(p3);
        auto r3 = vmhud::parseCameraFrame(f3, 60);
        CHECK_EQ_U(r3.currentLimit, 60, "2-slot approaching 50: current stays 60");
        CHECK_EQ_U(r3.nextLimit, 50, "2-slot approaching 50: next=50");
        CHECK_EQ_U(r3.nextDist, 250, "2-slot approaching 50: nextDist=250m");

        // Case D: Same layout at 80m (< 100m dummy!) -> continuity still keeps current=60, next=50
        uint8_t p4[8] = { 0x01, 0x00, 0x50, 50,  0x01, 0x00, 0x64, 60 };
        vmhud::Frame f4; f4.command = 1; f4.payload = p4; f4.size = sizeof(p4);
        auto r4 = vmhud::parseCameraFrame(f4, 60);
        CHECK_EQ_U(r4.currentLimit, 60, "2-slot at 80m: current stays 60 (no premature flip)");
        CHECK_EQ_U(r4.nextLimit, 50, "2-slot at 80m: next=50");
        CHECK_EQ_U(r4.nextDist, 80, "2-slot at 80m: nextDist=80m");

        // Case E: Cold start (prev=0) with slot 0 real countdown 300m -> slot 0 is next, slot 1 (dummy 100) is current
        uint8_t p5[8] = { 0x01, 0x01, 0x2C, 80,  0x01, 0x00, 0x64, 60 };
        vmhud::Frame f5; f5.command = 1; f5.payload = p5; f5.size = sizeof(p5);
        auto r5 = vmhud::parseCameraFrame(f5, 0);
        CHECK_EQ_U(r5.currentLimit, 60, "cold-start current=60 (slot 1 dummy dist=100)");
        CHECK_EQ_U(r5.nextLimit, 80, "cold-start next=80 (slot 0 countdown=300m)");
        CHECK_EQ_U(r5.nextDist, 300, "cold-start nextDist=300m");

        // Case F: Reversed slot order (slot 0 current=60, slot 1 upcoming=50 with dummy 100)
        uint8_t p6[8] = { 0x01, 0x00, 0x00, 60,  0x01, 0x00, 0x64, 50 };
        vmhud::Frame f6; f6.command = 1; f6.payload = p6; f6.size = sizeof(p6);
        auto r6 = vmhud::parseCameraFrame(f6, 60);
        CHECK_EQ_U(r6.currentLimit, 60, "reversed-slot continuity current=60");
        CHECK_EQ_U(r6.nextLimit, 50, "reversed-slot continuity next=50");
        CHECK_EQ_U(r6.nextDist, 0, "reversed-slot dummy dist=100 normalized to 0");
    }

    printf("\n=== Test 8: Speed-sign classifier self-consistency across all %zu signs (30..120) ===\n",
           vmhud::kSpeedSignTemplateCount);
    {
        CHECK_EQ_U(vmhud::kSpeedSignTemplateCount, 10, "10 VietMap speed sign templates (30..120)");
        size_t matched = 0;
        for (size_t t = 0; t < vmhud::kSpeedSignTemplateCount; ++t) {
            const vmhud::SpeedSignTemplate &tmpl = vmhud::kSpeedSignTemplates[t];
            uint8_t luma[256];
            memset(luma, 128, sizeof(luma));
            size_t k = 0;
            for (size_t r = vmhud::kSpeedSignRowStart; r < vmhud::kSpeedSignRowEnd; ++r) {
                for (size_t c = vmhud::kSpeedSignColStart; c < vmhud::kSpeedSignColEnd; ++c) {
                    int v = 128 + static_cast<int>(tmpl.vec[k++]);
                    if (v < 0) v = 0;
                    if (v > 255) v = 255;
                    luma[r * 16 + c] = static_cast<uint8_t>(v);
                }
            }
            auto m = vmhud::classifySpeedSignLuma16(luma);
            if (m.limit == tmpl.limit && m.score >= 0.99f) {
                matched++;
            } else {
                printf("  MISS sign[%zu] want=%u got=%u score=%.4f margin=%.4f\n",
                       t, (unsigned)tmpl.limit, (unsigned)m.limit, m.score, m.margin);
            }
        }
        CHECK_EQ_U(matched, vmhud::kSpeedSignTemplateCount, "all 10 speed signs self-match >= 0.99");

        uint8_t blank[256] = {0};
        CHECK_EQ_U(vmhud::classifySpeedSignLuma16(blank).limit, 0, "blank image rejected as speed sign");
        uint8_t checker[256];
        for (int y = 0; y < 16; ++y)
            for (int x = 0; x < 16; ++x)
                checker[y * 16 + x] = ((x + y) & 1) ? 255 : 0;
        CHECK_EQ_U(vmhud::classifySpeedSignLuma16(checker).limit, 0, "checkerboard rejected as speed sign");
    }

    printf("\n=== Test 9: 10-second Next-Speed-Limit HUD Alert + Deferred Nav Queue ===\n");
    {
        hud_next_splim::AlertController ctrl;
        hud_next_splim::IncomingAlert a1;
        a1.id = 1u; a1.limit = 50u; a1.dist_m = 250u;

        // t=1000ms: New alert id=1 arrives -> holds HUD until t=11000ms
        auto s0 = ctrl.step(1000, &a1, /*cp_guiding=*/true);
        CHECK(s0.active && s0.changed && !s0.just_ended, "alert id=1 activates at t=1000ms");
        CHECK_EQ_U(ctrl.limit(), 50, "alert limit=50");
        CHECK_EQ_U(ctrl.dist_m(), 250, "alert dist=250m");

        // t=3000ms: New navigation turn arrives -> queued, MUST NOT interrupt alert
        ctrl.on_nav_update();
        CHECK(ctrl.has_queued_nav(), "nav update queued during 10s alert");
        a1.dist_m = 180u;
        auto s1 = ctrl.step(3000, &a1, /*cp_guiding=*/true);
        CHECK(s1.active && s1.changed && !s1.just_ended, "same id updates distance without breaking hold");
        CHECK_EQ_U(ctrl.dist_m(), 180, "alert distance updated to 180m");

        // t=9500ms: Repeated same alert_id=1 MUST NOT extend timer past t=11000ms!
        auto s2 = ctrl.step(9500, &a1, /*cp_guiding=*/true);
        CHECK(s2.active && !s2.changed && !s2.just_ended, "still holding at t=9500ms (no redundant repaint)");

        // t=11000ms (exact 10.0s): Alert expires -> just_ended=true so queued nav flushes!
        auto s3 = ctrl.step(11000, &a1, /*cp_guiding=*/true);
        CHECK(!s3.active && s3.just_ended && !s3.run_deferred_clear,
              "alert ends at exact t=11000ms even while same id=1 still present");

        // t=12000ms: Same id=1 still in file -> MUST NOT re-trigger
        auto s4 = ctrl.step(12000, &a1, /*cp_guiding=*/true);
        CHECK(!s4.active && !s4.changed && !s4.just_ended, "same id=1 does not re-trigger after expiry");

        // t=15000ms: New alert id=2 (40 km/h); in-app nav_clear arrives at t=17000ms -> deferred until t=25000ms!
        hud_next_splim::IncomingAlert a2;
        a2.id = 2u; a2.limit = 40u; a2.dist_m = 150u;
        auto s5 = ctrl.step(15000, &a2, /*cp_guiding=*/true);
        CHECK(s5.active && s5.changed, "new alert id=2 activates at t=15000ms");
        CHECK(!ctrl.on_nav_clear(), "in-app nav_clear deferred during active 10s alert");
        auto s6 = ctrl.step(20000, nullptr, /*cp_guiding=*/false);
        CHECK(s6.active && !s6.just_ended, "alert stays active at t=20000ms even after /tmp/vmnextsplim gone");
        auto s7 = ctrl.step(25000, nullptr, /*cp_guiding=*/false);
        CHECK(!s7.active && s7.just_ended && s7.run_deferred_clear,
              "deferred nav_clear executes cleanly at t=25000ms");

        // Full session disconnect aborts active alert immediately
        hud_next_splim::IncomingAlert a3;
        a3.id = 3u; a3.limit = 80u; a3.dist_m = 0u;
        auto s8 = ctrl.step(30000, &a3, /*cp_guiding=*/false);
        CHECK(s8.active, "alert id=3 active");
        ctrl.on_full_clear();
        CHECK(!ctrl.active(), "full_clear aborts active alert immediately");
    }

    printf("\n=== Test 10: VietMap label transliteration for Mazda HUD font ===\n");
    {
        char label[64];
        strncpy(label, hud_next_splim::kAlertLabel, sizeof(label) - 1);
        label[sizeof(label) - 1] = '\0';
        hud_translit::fold(label, '?');
        CHECK(strcmp(label, "T\xC3\xB4" "c \xC4\x91\xC3\xB4 ti\xC3\xAAp theo") == 0,
              "\"Tốc độ tiếp theo\" folds cleanly to 1-/2-byte HUD glyphs (\"Tôc đô tiêp theo\")");
        // Ensure zero '?' fallback characters and zero 3-byte UTF-8 lead bytes (>= 0xE0)
        bool all_hud_safe = true;
        for (size_t i = 0; label[i]; ++i) {
            unsigned char ch = static_cast<unsigned char>(label[i]);
            if (ch == '?' || ch >= 0xE0u) { all_hud_safe = false; break; }
        }
        CHECK(all_hud_safe, "folded label contains zero unrenderable >=U+0800 code points");
    }

    printf("\n=== Test 11: Traffic/prohibition sign classifier across all %zu templates ===\n",
           vmhud::kTrafficSignTemplateCount);
    {
        CHECK(vmhud::kTrafficSignTemplateCount >= 20, "traffic sign template bank populated (>=20)");
        size_t matched = 0;
        for (size_t t = 0; t < vmhud::kTrafficSignTemplateCount; ++t) {
            const vmhud::TrafficSignTemplate &tmpl = vmhud::kTrafficSignTemplates[t];
            uint8_t luma[256];
            int8_t rb8[64];
            for (size_t i = 0; i < 256; ++i) {
                int v = 128 + static_cast<int>(tmpl.vec[i]);
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                luma[i] = static_cast<uint8_t>(v);
            }
            for (size_t j = 0; j < 64; ++j) {
                int v = static_cast<int>(lroundf(float(tmpl.vec[256 + j]) / 1.5f));
                if (v < -127) v = -127;
                if (v > 127) v = 127;
                rb8[j] = static_cast<int8_t>(v);
            }
            auto m = vmhud::classifyAlertGrids(luma, rb8);
            if (m.speedLimit == 0 && m.signCode == tmpl.signCode && m.score >= 0.98f && m.margin >= 0.04f) {
                matched++;
            } else {
                printf("  MISS traffic[%zu] want=0x%02X got=0x%02X (spd=%u) score=%.4f margin=%.4f\n",
                       t, (unsigned)tmpl.signCode, (unsigned)m.signCode, (unsigned)m.speedLimit,
                       m.score, m.margin);
            }
        }
        CHECK_EQ_U(matched, vmhud::kTrafficSignTemplateCount,
                   "all prohibition/area/camera sign templates self-match without cross-class confusion");

        uint8_t blankLuma[256] = {0};
        int8_t blankRb[64] = {0};
        auto blankRes = vmhud::classifyAlertGrids(blankLuma, blankRb);
        CHECK_EQ_U(blankRes.signCode, vmhud::kSignNone, "blank image rejected by traffic sign classifier");
    }

    printf("\n=== Test 12: CMD 0x06 approaching distance text parser & CMD 0x07/0x47 clear ===\n");
    {
        auto makeCmd06 = [](const char *txt, std::vector<uint8_t> &buf) -> vmhud::Frame {
            buf.assign(9, 0);
            buf[0] = 0x00; buf[1] = 0x20; // posX
            buf[2] = 0x00; buf[3] = 0x40; // posY
            buf[4] = 0x02;                // font
            buf[6] = 0xFF; buf[7] = 0xFF; buf[8] = 0xFF; // RGB
            size_t n = strlen(txt);
            buf.insert(buf.end(), txt, txt + n);
            buf.push_back(0x00);
            vmhud::Frame f;
            f.command = 0x06;
            f.payload = buf.data();
            f.size = buf.size();
            return f;
        };

        std::vector<uint8_t> b;
        char txt[vmhud::kMaxOverlayTextBytes + 1];
        CHECK(vmhud::extractOverlayText(makeCmd06("250 m", b), txt, sizeof(txt)) &&
                  strcmp(txt, "250 m") == 0,
              "CMD 0x06 '250 m' forwarded verbatim");
        CHECK(vmhud::extractOverlayText(makeCmd06("150m", b), txt, sizeof(txt)) &&
                  strcmp(txt, "150m") == 0,
              "CMD 0x06 '150m' forwarded verbatim");
        CHECK(vmhud::extractOverlayText(makeCmd06("  1.2 km \n", b), txt, sizeof(txt)) &&
                  strcmp(txt, "1.2 km") == 0,
              "CMD 0x06 '  1.2 km \\n' trimmed verbatim -> '1.2 km'");
        CHECK(vmhud::extractOverlayText(makeCmd06("0,85km", b), txt, sizeof(txt)) &&
                  strcmp(txt, "0,85km") == 0,
              "CMD 0x06 '0,85km' forwarded verbatim");
        CHECK(!vmhud::extractOverlayText(makeCmd06("   ", b), txt, sizeof(txt)),
              "CMD 0x06 blank string ignored");

        vmhud::Frame clr7; clr7.command = 0x07;
        vmhud::Frame clr47; clr47.command = 0x47;
        CHECK(vmhud::isOverlayTextClear(clr7) && vmhud::isOverlayTextClear(clr47),
              "CMD 0x07 and 0x47 recognized as overlay text clear");
    }

    printf("\n=== Test 13: CPH FFF9 binary frame codec (with raw CMD 0x06 text), XOR, overspeed & buckets ===\n");
    {
        vmhud::CphTelemetryState st;
        st.vietmapConnected = true;
        st.postedSpeedLimit = 60;
        st.currentSpeed = 61;
        st.overspeed = vmhud::evaluateOverspeed(60, 61, false);
        CHECK(!st.overspeed, "61 km/h on 60 limit does not trigger overspeed (+2 hysteresis)");
        st.currentSpeed = 63;
        st.overspeed = vmhud::evaluateOverspeed(60, 63, st.overspeed);
        CHECK(st.overspeed, "63 km/h on 60 limit triggers overspeed");
        st.currentSpeed = 61;
        st.overspeed = vmhud::evaluateOverspeed(60, 61, st.overspeed);
        CHECK(st.overspeed, "61 km/h stays overspeed until <= 60");
        st.currentSpeed = 60;
        st.overspeed = vmhud::evaluateOverspeed(60, 60, st.overspeed);
        CHECK(!st.overspeed, "60 km/h clears overspeed");

        st.upcomingSpeedLimit = 50;
        st.upcomingSpeedDistM = 295;
        st.activeSignCode = vmhud::kSignNoLeftTurn;
        st.activeSignDistM = 0;
        strcpy(st.activeSignText, "1.2 km");

        uint8_t wire[vmhud::kCphMaxFrameBytes];
        const size_t wireLen = vmhud::encodeCphFrame(st, wire);
        CHECK_EQ_U(wire[0], 0xA7, "magic byte 0xA7");
        CHECK_EQ_U(wireLen, 13 + 6, "frame len 19 bytes (13 + '1.2 km')");

        vmhud::CphTelemetryState dec;
        CHECK(vmhud::decodeCphFrame(wire, wireLen, dec), "valid CPH frame with raw text decoded");
        CHECK(dec.vietmapConnected && !dec.overspeed, "flags round-tripped");
        CHECK_EQ_U(dec.postedSpeedLimit, 60, "postedSpeedLimit=60");
        CHECK_EQ_U(dec.upcomingSpeedLimit, 50, "upcomingSpeedLimit=50");
        CHECK_EQ_U(dec.upcomingSpeedDistM, 295, "upcomingSpeedDistM=295");
        CHECK_EQ_U(dec.activeSignCode, vmhud::kSignNoLeftTurn, "activeSignCode=noLeftTurn");
        CHECK(strcmp(dec.activeSignText, "1.2 km") == 0, "activeSignText='1.2 km' round-tripped");

        // Identical raw text + 1 km/h GPS drift suppressed
        vmhud::CphTelemetryState nextTick = st;
        nextTick.currentSpeed = 59;
        CHECK(!vmhud::isCphSemanticChange(st, nextTick),
              "identical raw text & 1 km/h GPS drift suppressed");

        // Changing raw CMD 0x06 text triggers notification
        strcpy(nextTick.activeSignText, "250 m");
        CHECK(vmhud::isCphSemanticChange(st, nextTick),
              "changed raw CMD 0x06 text triggers semantic change");

        // Corrupt XOR checksum byte -> rejected
        wire[wireLen - 1] ^= 0xFF;
        CHECK(!vmhud::decodeCphFrame(wire, wireLen, dec), "corrupted XOR byte rejected");

        // Verify CMD 0x01 Slot 0 camera alert extraction vs Slot 1 dummy 100m posted limit
        auto checkCmd01Alert = [](const uint8_t *payload, size_t n, uint8_t prevLim,
                                  uint8_t &outSign, uint16_t &outDist) -> bool {
            vmhud::Frame f;
            f.command = 1;
            f.payload = payload;
            f.size = n;
            const vmhud::CameraParseResult cam = vmhud::parseCameraFrame(f, prevLim);
            return vmhud::extractCmd01CameraAlert(f, cam, outSign, outDist);
        };
        uint8_t cSign = 0;
        uint16_t cDist = 0;
        // 1) Ordinary 60 km/h road, no camera in Slot 0, Slot 1 has dummy 100m
        const uint8_t roadOnly[8] = {0, 0x00, 0x00, 0, 1, 0x00, 0x64, 60};
        CHECK(!checkCmd01Alert(roadOnly, 8, 60, cSign, cDist),
              "CMD 0x01 ordinary road (Slot 1 dummy 100m) does not false-trigger camera sign");
        // 2) Same-limit speed camera at 350m on 60 km/h road
        const uint8_t spdCam[8] = {1, 0x01, 0x5E, 60, 1, 0x00, 0x64, 60};
        CHECK(checkCmd01Alert(spdCam, 8, 60, cSign, cDist) &&
                  cSign == vmhud::kSignSpeedCamera && cDist == 350,
              "CMD 0x01 same-limit speed camera -> kSignSpeedCamera 350m");
        // 3) Red-light/traffic camera (type=2, lim=0) at 220m on 60 km/h road
        const uint8_t trfCam[8] = {2, 0x00, 0xDC, 0, 1, 0x00, 0x64, 60};
        CHECK(checkCmd01Alert(trfCam, 8, 60, cSign, cDist) &&
                  cSign == vmhud::kSignTrafficCamera && cDist == 220,
              "CMD 0x01 red-light camera -> kSignTrafficCamera 220m");
        // 4) Upcoming 50 km/h zone at 300m on 60 km/h road -> owned by upcomingSpeedLimit, not camera sign
        const uint8_t nextZone[8] = {1, 0x01, 0x2C, 50, 1, 0x00, 0x64, 60};
        CHECK(!checkCmd01Alert(nextZone, 8, 60, cSign, cDist),
              "CMD 0x01 upcoming 50 km/h zone yields to upcomingSpeedLimit");
    }

    if (g_fail) {
        printf("\nVIETMAP BLE HUD TEST SUITE FAILED\n");
        return 1;
    }
    printf("\nALL VIETMAP BLE HUD TESTS PASSED\n");
    return 0;
}
