// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Adapted from headunit (https://github.com/Trevelopment/headunit),
// licensed under GNU AGPL v3.
// See NOTICE.md at the repo root for the full attribution.
//
// HUD output module — converts the AAP nav events surfaced by nav.cpp
// into Mazda "Active Driving Display" HUD frames.
//
// === Transport: OEM libjcidbus (NOT dbus-c++) ================
//
// This module talks to com.jci.vbs.navi through the OEM's own
// libjcidbus + libjcivbsnaviclient C libraries (see oem/), NOT the
// vendored dbus-c++ proxies. Reasons (all observed on real cars during
// this project's own testing):
//   * libjcidbus disables libdbus exit-on-disconnect, so a private
//     service-bus teardown (e.g. an AndroidAuto<->CarPlay source
//     switch) is NON-FATAL — it no longer raises a self-directed
//     SIGSEGV that kills {L_jciCARPLAY} (= the "đi 1 lúc bị reboot" bug).
//   * libjcidbus runs its OWN I/O worker thread, so there is no
//     dispatcher WE own polling a dead socket after a drop (= the
//     "mất HUD sau 1 lúc" bug) and no use-after-free at teardown.
//   * All D-Bus bring-up happens on the sender thread, so creating an
//     AA session never blocks on a synchronous HUD probe.
// This replaces the previous dbus-c++ path and its hand-rolled
// reconnect/leak/outer-catch workarounds (the OEM lib handles it).
//
// === Producer / consumer ====================================
//
// On every 0x500 / 0x501 / 0x502 event nav.cpp pushes via the hud_on_*
// functions (SDK protobuf-decode thread — MUST NOT block): write the
// seqlock-protected snapshot, bump g_seq twice, notify the sender cv.
// The sender thread re-reads the snapshot, diffs vs the previous, and
// makes the warranted OEM calls (SetHUDDisplayMsgReq for the
// maneuver/distance block, SetHUD_Display_Msg2 for the road strip).
// Coalescing (seqlock + "latest wins") means a burst of updates folds
// into one send.

#include "../patch.h"
#include "hud_send.h"
#include "../oem/libjcivbsnaviclient.h"   // OEM transport (libjcidbus + VBS_NAVI_*)
#include "common/config.h"                // runtime libpatch.conf (carplay_* keys)
#include "common/thread_util.h"           // preload_thread_create (256 KiB bounded stack)
#include "common/translit.h"              // hud_translit::fold() — shared Latin street-name fold

#include <cstring>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>

// [2026-06-13 #3] Defined in nav.cpp — flags the nav-side selection/emit state for reset on the
// next maneuver (msgrcv thread) so the HUD repaints when a route is re-started after a nav-off.
extern "C" void nav_request_reset(void);

namespace {

// ===== [VN-PATCH] Vietnamese UTF-8 normalization =============================
//
// Mazda HUD cluster ECU font (baked into HUD firmware) renders 1-byte ASCII
// and 2-byte UTF-8 (Latin-1 Supp + Latin Extended-A/B: ô ư ơ â ê ă đ á à) but
// silently DROPS 3-byte UTF-8. Vietnamese precomposed chars with 2 stacked
// diacritics (ề ớ ấ …) live in Latin Extended Additional U+1E00.. and are
// 3-byte, so without patch "về hướng Gia Mô" shows as "v  hư ng Gia Mô".
//
// Fix: fold the U+1E00..U+1EFF block to the nearest renderable base, keeping
// the circumflex/horn/breve (ề->ê, ớ->ơ, ấ->â) and dropping only the tone.
// This is the SAME fold the Android Auto patch uses — see common/translit.h
// (hud_translit::fold) for the full table + rationale. CarPlay passes '?' as
// the unrenderable placeholder so a non-VN 3-/4-byte char keeps a visible slot
// instead of rendering ECU font garbage; AA passes 0 (leaves it blank).
//
// Toggle: libpatch.conf carplay_vn_normalize (default true = fold to the HUD
//         font). Set false to pass UTF-8 unchanged (a HUD with a full
//         Vietnamese font, or a non-VN locale).

// Well-known name we acquire on the service bus (libjcidbus registers it
// via dbus_bus_request_name). A novel name avoids colliding with the real
// com.jci.vbs.navi service (which we only send method calls to). MUST also
// differ from the AA shim's name "com.jci.aapa.hud": on a unit where BOTH the
// AA (jciAAPA) and CarPlay (jciCARPLAY) shims run, the 2nd process to request
// the SAME well-known name fails dbus_bus_request_name -> JCIDBUS_conn_connect
// returns 0 -> sender thread exits -> no HUD. So CarPlay uses its own name.
constexpr const char *kBusName = "com.jci.carplay.hud";

// === Mazda HUD turn-icon enum =================================
//
// Mirrors `NaviTurns` in reference hud.h. These are the integer
// codes the HUD ECU interprets — the OEM glyph atlas is baked
// into the ECU firmware and is out of our scope. Same numbering
// as the reference, no remapping.
enum MazdaIcon : uint8_t {
    HUD_STRAIGHT          = 1,
    HUD_LEFT              = 2,
    HUD_RIGHT             = 3,
    HUD_SLIGHT_LEFT       = 4,
    HUD_SLIGHT_RIGHT      = 5,
    HUD_OFF_RAMP_RIGHT    = 7,
    HUD_DESTINATION       = 8,
    HUD_SHARP_RIGHT       = 9,
    HUD_U_TURN_RIGHT      = 10,
    HUD_SHARP_LEFT        = 11,
    HUD_FLAG              = 12,
    HUD_U_TURN_LEFT       = 13,
    HUD_FORK_RIGHT        = 14,
    HUD_FORK_LEFT         = 15,
    HUD_MERGE_LEFT        = 16,
    HUD_MERGE_RIGHT       = 17,
    HUD_OFF_RAMP_LEFT     = 30,
    HUD_DESTINATION_LEFT  = 33,
    HUD_DESTINATION_RIGHT = 34,
    HUD_FLAG_LEFT         = 35,
    HUD_FLAG_RIGHT        = 36,
};

// === Android turn_event → Mazda icon ==========================
//
// Lookup: kTurnIcons[android_turn_event][side_index]
//   side_index: 0=LEFT, 1=RIGHT, 2=UNSPECIFIED/STRAIGHT
//
// Indexed by hu.proto NAVTurnMessage.TURN_EVENT (0..19, sparse at
// 15 and 18). A `0` entry means "no glyph" — the HUD draws blank.
// ROUNDABOUT_ENTER_AND_EXIT (13) is handled separately by
// roundabout_icon() because the icon depends on exit angle.
//
// This table is copied verbatim from reference hud.cpp's turns[][]
// (just renamed). The reference table has been validated on real
// cars; do not "improve" it without a road test.
constexpr uint8_t kTurnIcons[20][3] = {
    /*  0 TURN_UNKNOWN                  */ {0, 0, 0},
    /*  1 TURN_DEPART                   */ {HUD_FLAG_LEFT, HUD_FLAG_RIGHT, HUD_FLAG},
    /*  2 TURN_NAME_CHANGE              */ {HUD_STRAIGHT, HUD_STRAIGHT, HUD_STRAIGHT},
    /*  3 TURN_SLIGHT_TURN              */ {HUD_SLIGHT_LEFT, HUD_SLIGHT_RIGHT, HUD_STRAIGHT},
    /*  4 TURN_TURN                     */ {HUD_LEFT, HUD_RIGHT, HUD_STRAIGHT},
    /*  5 TURN_SHARP_TURN               */ {HUD_SHARP_LEFT, HUD_SHARP_RIGHT, HUD_STRAIGHT},
    /*  6 TURN_U_TURN                   */ {HUD_U_TURN_LEFT, HUD_U_TURN_RIGHT, HUD_STRAIGHT},
    /*  7 TURN_ON_RAMP                  */ {HUD_LEFT, HUD_RIGHT, HUD_STRAIGHT},
    /*  8 TURN_OFF_RAMP                 */ {HUD_OFF_RAMP_LEFT, HUD_OFF_RAMP_RIGHT, HUD_STRAIGHT},
    /*  9 TURN_FORK                     */ {HUD_FORK_LEFT, HUD_FORK_RIGHT, HUD_STRAIGHT},
    /* 10 TURN_MERGE                    */ {HUD_MERGE_LEFT, HUD_MERGE_RIGHT, HUD_STRAIGHT},
    /* 11 TURN_ROUNDABOUT_ENTER         */ {0, 0, 0},
    /* 12 TURN_ROUNDABOUT_EXIT          */ {0, 0, 0},
    /* 13 TURN_ROUNDABOUT_ENTER_AND_EXIT*/ {0, 0, 0},  // handled by roundabout_icon()
    /* 14 TURN_STRAIGHT                 */ {HUD_STRAIGHT, HUD_STRAIGHT, HUD_STRAIGHT},
    /* 15 unassigned in proto           */ {0, 0, 0},
    /* 16 TURN_FERRY_BOAT               */ {0, 0, 0},
    /* 17 TURN_FERRY_TRAIN              */ {0, 0, 0},
    /* 18 unassigned in proto           */ {0, 0, 0},
    /* 19 TURN_DESTINATION              */ {HUD_DESTINATION_LEFT, HUD_DESTINATION_RIGHT, HUD_DESTINATION},
};

// Roundabout exit icon — 12 directional roundabout glyphs per
// side, indexed by exit angle (rounded to nearest 30°). Offsets
// 37 (right-hand traffic) and 49 (left-hand traffic) come from
// the reference implementation; the HUD ECU has roundabout
// glyphs at IDs 37..48 and 49..60.
uint8_t roundabout_icon(int32_t degrees, int32_t side_index_lr)
{
    // [BUILD-20] Reflect + 180° offset BEFORE bucketing into 12 × 30° sectors.
    // The Mazda HUD roundabout bank's sector 0 (icon 37/49) points DOWN (6 o'clock =
    // the entry leg) and its sector index runs COUNTER-clockwise, whereas iAP2
    // junctionElementExitAngle is 0=straight-ahead, +=right/CW. So feed d = 180 - bearing.
    // Verified LIVE on the HUD via /tmp/hudtest (RHT bank 37..48): straight(0)->icon43=UP,
    // right(+90)->icon40=RIGHT, left(270)->icon46=LEFT, behind(180)->icon37=DOWN.
    // (d+15)/30 rounds to nearest 30°; %12 wraps [345,360) back to sector 0.
    int32_t d = (180 - degrees) % 360;
    if (d < 0) d += 360;
    uint8_t nearest = static_cast<uint8_t>(((d + 15) / 30) % 12);
    uint8_t offset  = (side_index_lr == 0) ? 49 : 37;
    return static_cast<uint8_t>(nearest + offset);
}

// === Distance-unit enum translation ===========================
//
// hu.proto DISPLAY_DISTANCE_UNIT:
//   1=METERS, 2=KILOMETERS10, 3=KILOMETERS, 4=MILES10,
//   5=MILES,  6=FEET
//
// Mazda HUD (HudDistanceUnit in reference hud.h):
//   1=METERS, 2=MILES, 3=KILOMETERS, 4=YARDS, 5=FEET
uint8_t map_distance_unit(uint32_t android_unit)
{
    switch (android_unit) {
    case 1: return 1;  // METERS       -> METERS
    case 2: return 3;  // KILOMETERS10 -> KILOMETERS
    case 3: return 3;  // KILOMETERS   -> KILOMETERS
    case 4: return 2;  // MILES10      -> MILES
    case 5: return 2;  // MILES        -> MILES
    case 6: return 5;  // FEET         -> FEET
    default: return 0; // Unknown — HUD will render nothing.
    }
}

// === Shared snapshot (seqlock-protected) ======================
struct NaviSnapshot {
    // [VN-PATCH] Buffer 256 to fit long Vietnamese street names (each VN
    // char with diacritic = 2-3 bytes in UTF-8).
    char     road_name[256];     // NUL-terminated UTF-8
    uint32_t turn_side;          // proto TURN_SIDE: 1=L, 2=R, 3=U
    uint32_t turn_event;         // proto TURN_EVENT: 0..19 sparse
    int32_t  turn_angle;         // degrees, signed
    int32_t  turn_number;        // maneuver / exit number
    int32_t  distance_dec;       // value * 10 (one decimal place)
    uint8_t  distance_unit;      // ALREADY mapped to Mazda enum
    int32_t  time_until;         // seconds — ETA to next maneuver
    uint32_t direct_icon;        // [VIETMAP-BLE] 0 = resolve via kTurnIcons/roundabout; 1..60 = direct MazdaIcon
};

NaviSnapshot            g_snapshot   = {};
std::atomic<uint32_t>   g_seq{0};
std::condition_variable g_cv;
std::mutex              g_cv_mu;
std::atomic<bool>       g_active{false};        // sender pipeline live (setup done)
std::atomic<bool>       g_stop{false};          // stop requested
std::atomic<uint32_t>   g_splim_tick{0};        // [VN-PATCH] ticker -> wakes sender to re-poll splim + re-assert
std::atomic<bool>       g_have_guidance{false}; // [KEEPALIVE] route active -> re-assert HUD frame each tick
std::atomic<long>       g_last_nav{0};          // [NAV-END] unix sec of last nav msg; sender clears HUD if stale
std::atomic<bool>       g_clear_req{false};     // [NAV-END] in-app nav-off (TBT off, CarPlay still connected) -> blank maneuver, KEEP sign
std::atomic<bool>       g_fullclear_req{false}; // [NAV-END] session gone / phone unplugged (cp_deactive_cb) -> FULL wipe incl sign

// === OEM connection state =====================================
//
// The connection handle is created and owned by the sender thread for
// its whole lifetime. libjcidbus runs its OWN I/O worker thread, so —
// unlike the old dbus-c++ path — we run no event loop ourselves and
// have nothing polling a dead socket after a source switch.
void *g_conn = nullptr;

// HUD-present gate, fail-OPEN. Default false (= "not known absent" -> send).
// A one-shot VBS_NAVI_GetHUDStatus reply flips this true only on a no-HUD
// car. Fail-open means a lost/late reply never silences us for the drive
// (replaces the old synchronous probe + background re-probe). The cost is a
// few harmless frames on a no-HUD car until the reply lands.
std::atomic<bool> g_hud_absent{false};

pthread_t g_sender_thread       = 0;
bool      g_sender_thread_up    = false;
pthread_t g_splim_ticker_thread = 0;
bool      g_splim_ticker_up     = false;

// libjcidbus connection error callback (stored at conn+0x10 and invoked
// as cb(conn, closure) on connection-level errors). Must be non-NULL —
// the library dereferences and calls it. Trivial + non-throwing; with
// exit-on-disconnect disabled, a bus drop is no longer fatal.
extern "C" void hud_dbus_error_cb(void *conn, void *closure)
{
    (void)conn; (void)closure;
    LOGW("hud sender: libjcidbus signalled a service-bus connection error "
         "(non-fatal; exit-on-disconnect is off)");
}

// VBS_NAVI_GetHUDStatus async reply. ABI from the decompiled trampoline:
// cb(conn, status_byte, user). A zero status byte means no HUD installed.
extern "C" void hud_status_cb(void *conn, unsigned char status, void *user)
{
    (void)conn; (void)user;
    const bool absent = (status == 0);
    g_hud_absent.store(absent, std::memory_order_release);
    LOGD("hud sender: GetHUDStatus reply = %u (%s)",
         static_cast<unsigned>(status), absent ? "no HUD" : "HUD present");
}

// Zero-allocation stack read of a tiny runtime flag/status file (avoids fopen's
// heap FILE + 4 KiB buffer alloc/free on every 2 Hz tick, and enforces O_CLOEXEC).
static ssize_t read_tiny_file(const char *path, char *buf, size_t cap)
{
    int fd = ::open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    ssize_t n = ::read(fd, buf, cap - 1);
    ::close(fd);
    if (n <= 0) return -1;
    buf[n] = '\0';
    return n;
}

// [VN-PATCH] Speed limit written by splim_seriald ("<limit> <unixtime>").
// Prefers RAM tmpfs (/tmp/splim, zero eMMC flash wear) and falls back to the
// legacy /data_persist/splim path for compatibility. Returns km/h, or 0 if
// missing / stale / clock-skewed / implausible.
static uint16_t read_splim()
{
    if (!libpatch_config::carplay_speed_limit()) return 0;

    char buf[64];
    if (read_tiny_file("/tmp/splim", buf, sizeof(buf)) <= 0 &&
        read_tiny_file("/data_persist/splim", buf, sizeof(buf)) <= 0) {
        return 0;
    }
    int limit = 0; long ts = 0;
    if (std::sscanf(buf, "%d %ld", &limit, &ts) < 2) return 0;
    long now = static_cast<long>(std::time(nullptr));
    // Reject both stale (>8s old) and future-skewed (>2s ahead, e.g. wall clock
    // stepped backward) timestamps so an old limit can never stick forever.
    if (now - ts > 8 || ts - now > 2 || limit < 5 || limit > 200) return 0;
    return static_cast<uint16_t>(limit);
}

// [OVERSPEED] Read the over-speed flag written by carspeed_d (1 = over -> blink).
static bool read_over()
{
    if (!libpatch_config::carplay_speed_limit()) return false;

    char buf[16];
    if (read_tiny_file("/tmp/splim_over", buf, sizeof(buf)) <= 0 &&
        read_tiny_file("/data_persist/splim_over", buf, sizeof(buf)) <= 0) {
        return false;
    }
    int v = 0;
    return std::sscanf(buf, "%d", &v) == 1 && v == 1;
}

// [VIETMAP-BLE] Read VietMap Live iPhone turn-by-turn navigation written by
// splim_seriald ("<icon> <dist_m> <unix_ts>[ <road_utf8>]") in RAM tmpfs.
// Returns true iff a fresh, valid navigation frame is present.
static bool read_vmnav(NaviSnapshot &out)
{
    if (!libpatch_config::carplay_vietmap_nav()) return false;

    char buf[384];
    if (read_tiny_file("/tmp/vmnav", buf, sizeof(buf)) <= 0) return false;

    // Strip trailing CR/LF so road_name stays single-line.
    size_t len = std::strlen(buf);
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
        buf[--len] = '\0';
    }
    if (buf[0] < '0' || buf[0] > '9') return false;

    unsigned int icon = 0;
    unsigned long dist_m = 0;
    long ts = 0;
    int consumed = 0;
    if (std::sscanf(buf, "%u %lu %ld%n", &icon, &dist_m, &ts, &consumed) < 3) {
        return false;
    }
    long now = static_cast<long>(std::time(nullptr));
    // Reject stale (>6s old) or future-skewed (>2s ahead) snapshots.
    if (now - ts > 6 || ts - now > 2) return false;
    if (icon < 1u || icon > 60u || dist_m > 999000ul) return false;

    std::memset(&out, 0, sizeof(out));
    out.direct_icon = icon;
    if (dist_m < 1000ul) {
        out.distance_dec  = static_cast<int32_t>(dist_m * 10ul);
        out.distance_unit = 1;  // METERS
    } else {
        out.distance_dec  = static_cast<int32_t>(dist_m / 100ul);
        out.distance_unit = 3;  // KILOMETERS
    }

    const char *road = buf + consumed;
    while (*road == ' ' || *road == '\t') ++road;
    if (*road) {
        const size_t max = sizeof(out.road_name) - 1;
        size_t n = strnlen(road, max + 1);
        if (n > max) {
            n = max;
            while (n > 0 && (static_cast<unsigned char>(road[n]) & 0xC0u) == 0x80u) {
                --n;
            }
        }
        std::memcpy(out.road_name, road, n);
        out.road_name[n] = '\0';
    }
    return true;
}

// Wake the sender thread without a lost-wakeup race against g_cv.wait().
inline void wake_sender()
{
    {
        std::lock_guard<std::mutex> lk(g_cv_mu);
    }
    g_cv.notify_one();
}

// Helper: snap vs prev → which OEM HUD calls to make. sender_main owns the
// prev/sync_bit state and threads them through. Returns false on a send
// error so the caller can log; the periodic re-assert retries next tick.
bool send_one(const NaviSnapshot &cur,
              const NaviSnapshot &prev,
              uint8_t            &sync_bit,
              uint16_t            splim,
              bool                splim_changed,
              bool                force)
{
    // HUD reported absent by GetHUDStatus — drain the snapshot but emit
    // nothing (producers keep updating it regardless).
    if (g_hud_absent.load(std::memory_order_acquire)) return true;

    // Any maneuver-field change (road, turn type/side/number, or direct icon) must still trigger
    // a SEND so the Msg1 maneuver block (icon/distance) repaints. This is the SEND gate, NOT the re-page gate.
    bool maneuver_changed = (std::strncmp(cur.road_name, prev.road_name,
                                          sizeof(cur.road_name)) != 0) ||
                            cur.direct_icon != prev.direct_icon ||
                            cur.turn_event  != prev.turn_event  ||
                            cur.turn_side   != prev.turn_side   ||
                            cur.turn_number != prev.turn_number;
    // [2026-06-13 REAL-DRIVE REVERT of the BUILD-9 road-only sync gate] On a real drive the HUD
    // LAGGED: when the maneuver changed but the road name stayed the same/blank (consecutive turns,
    // road-less turns), the road-only sync_bit gate did NOT bump, so the cluster did NOT refresh and
    // held the OLD maneuver ("map đổi mà HUD không đổi" + arrow shows blank/stale). The sync_bit now
    // bumps on ANY maneuver change (maneuver_changed) so the cluster re-renders every time the turn
    // changes. Cost: the minor street re-page "nháy" on a maneuver change returns — but a
    // correct+prompt maneuver beats a smooth-but-stale one. (Pure distance ticks still don't bump.)
    bool distance_changed = maneuver_changed ||
                            cur.distance_dec  != prev.distance_dec  ||
                            cur.distance_unit != prev.distance_unit ||
                            cur.turn_angle    != prev.turn_angle;

    // [KEEPALIVE] `force` = periodic re-assert (re-send the same frame so the
    // cluster ECU doesn't age out the maneuver). Skip only if nothing changed
    // AND this is not a forced re-assert.
    if (!maneuver_changed && !distance_changed && !splim_changed && !force)
        return true;  // nothing to send = OK (not a failure)

    // Plain-C OEM wire structs (zero heap alloc on every 2 Hz keepalive push).
    //   VbsNaviHudDisplay (uqyqyy): maneuver icon + distance + speed-limit.
    //   VbsNaviHudMsg2    (sy):     street-name strip.
    VbsNaviHudDisplay disp = {};
    VbsNaviHudMsg2    msg2 = {};
    char              road_buf[sizeof(cur.road_name)];
    road_buf[0] = '\0';

    // [MIRROR NATIVE — verified by decompile RE of thUpdateGuidanceChangeToHUD]
    // Native re-sends Msg1 (SetHUDDisplayMsgReq) AND Msg2 (SetHUD_Display_Msg2) as an
    // INSEPARABLE PAIR, UNCONDITIONALLY, on every guidance push — Msg2 is NEVER gated on
    // road-name change and is NEVER sent without Msg1. So we COUPLE Msg2 to Msg1: build+send
    // the street strip on the SAME condition as the maneuver frame (distance_changed ||
    // splim_changed || force), NOT only on a road-name change. This stops the street strip aging
    // out on the cluster between turns ("tên đường lúc hiện lúc không") and keeps the whole
    // TBT alive while driving straight (a distance-only push used to send Msg1 ALONE).
    if (distance_changed || splim_changed || force) {
        // sync_bit (cluster refresh hint) bumps on ANY maneuver change (road OR turn type/side/
        // number) — NOT on distance-only refreshes / keep-alives (those keep the same sync_bit).
        // This forces the cluster to re-render whenever the turn changes (fixes the real-drive
        // "HUD didn't update / blank arrow" lag).
        if (maneuver_changed) {
            // Per reference: bump 1..7 cyclically. The HUD treats a sync_bit
            // change as a hint to refresh the street-name page.
            sync_bit = static_cast<uint8_t>((sync_bit % 7) + 1);
        }

        std::strncpy(road_buf, cur.road_name, sizeof(road_buf) - 1);
        road_buf[sizeof(road_buf) - 1] = '\0';
        if (libpatch_config::carplay_vn_normalize()) {
            // [VN-PATCH] Fold Vietnamese 3-byte UTF-8 to the 2-byte/ASCII base
            // the HUD font can render. E.g. "về hướng" -> "vê hương". Fold in
            // place (the fold only ever shrinks). '?' = keep unfoldable 3-/4-byte
            // chars as a visible slot.
            hud_translit::fold(road_buf, '?');
        }
        // [VN-PATCH] An EMPTY road-name makes the Mazda HUD cluster ECU render
        // GARBAGE (uninitialised font memory) in the street slot. Emit a single
        // space -> HUD draws a clean blank slot, not garbage.
        if (road_buf[0] == '\0') {
            road_buf[0] = ' ';
            road_buf[1] = '\0';
        }
        msg2.guidancePointName = road_buf;
        msg2.syncBit           = sync_bit;
    }

    if (distance_changed || splim_changed || force) {
        uint32_t icon = 0;
        if (cur.direct_icon != 0) {
            icon = cur.direct_icon;
        } else if (cur.turn_event == 13 /*TURN_ROUNDABOUT_ENTER_AND_EXIT*/) {
            int32_t side_lr = (cur.turn_side == 1) ? 0 : 1;
            icon = roundabout_icon(cur.turn_angle, side_lr);
        } else if (cur.turn_event < 20) {
            int32_t side_idx = static_cast<int32_t>(cur.turn_side) - 1;
            if (side_idx < 0 || side_idx > 2) side_idx = 2;
            icon = kTurnIcons[cur.turn_event][side_idx];
        }

#ifdef DEBUG
        // Debug aid: in debug builds, when our mapping produced no glyph for a REAL
        // maneuver (turn_event != 0), hijack the street strip to show the raw codes so
        // an observer can record + extend the table.
        // DO NOT hijack when turn_event == 0 (idle / no maneuver / blankSnap), which
        // overwrites real road names or pollutes the display with "EV=0 S=0 A=0".
        if (icon == 0 && cur.turn_event != 0) {
            snprintf(road_buf, sizeof(road_buf), "EV=%u S=%u A=%d",
                     static_cast<unsigned>(cur.turn_event),
                     static_cast<unsigned>(cur.turn_side),
                     static_cast<int>(cur.turn_angle));
            // The dbg_label tracks the raw turn fields, so re-page when it changes even
            // though the maneuver fields did not already trigger a bump above.
            if (!maneuver_changed) {
                sync_bit = static_cast<uint8_t>((sync_bit % 7) + 1);
            }
            msg2.guidancePointName = road_buf;
            msg2.syncBit           = sync_bit;
            icon = MazdaIcon::HUD_STRAIGHT;
            // Msg2 is sent below on (distance_changed || splim_changed || force), already true here.
        }
#endif

        // [4.3] never emit unit=0 with a live maneuver; keep unit=0 when blank
        // (icon==0, e.g. speed-limit-only mode). [4.5] clamp distance to uint16
        // so a far maneuver (>65535) can't wrap to a tiny value.
        int32_t draw = cur.distance_dec;
        if (draw < 0)      draw = 0;
        if (draw > 0xFFFF) draw = 0xFFFF;
        disp.nextManeuverInfo  = icon;
        disp.distanceValue     = icon ? static_cast<uint16_t>(draw) : static_cast<uint16_t>(0);
        disp.distanceUnit      = icon ? (cur.distance_unit ? cur.distance_unit
                                                           : static_cast<uint8_t>(1))
                                      : static_cast<uint8_t>(0);
        disp.displaySpeedLimit = splim;                                // [VN-PATCH] km/h
        disp.displaySpeedUnit  = static_cast<uint8_t>(splim ? 1 : 0);  // [VN-PATCH] 1=km/h, 0=none
        disp.text_ID3          = sync_bit;
    }

    if (distance_changed || splim_changed || force) {
        LOGV("hud_send: SetHUDDisplayMsgReq icon=%u dist=%u unit=%u splim=%u sync=%u",
             static_cast<unsigned>(disp.nextManeuverInfo),
             static_cast<unsigned>(disp.distanceValue),
             static_cast<unsigned>(disp.distanceUnit),
             static_cast<unsigned>(disp.displaySpeedLimit),
             static_cast<unsigned>(disp.text_ID3));
        int rc = VBS_NAVI_SetHUDDisplayMsgReq(g_conn, &disp, nullptr, nullptr, nullptr);
        if (rc != 0) { LOGE("hud_send: SetHUDDisplayMsgReq failed rc=%d", rc); return false; }
    }
    if (distance_changed || splim_changed || force) {   // [MIRROR NATIVE] Msg2 ALWAYS paired with Msg1, every push
        LOGV("hud_send: SetHUD_Display_Msg2 road=\"%s\" sync=%u",
             road_buf, static_cast<unsigned>(msg2.syncBit));
        int rc = VBS_NAVI_TMC_SetHUD_Display_Msg2(g_conn, &msg2, nullptr, nullptr, nullptr);
        if (rc != 0) { LOGE("hud_send: SetHUD_Display_Msg2 failed rc=%d", rc); return false; }
    }
    return true;
}

// [2026-06-13] kNavStaleSec + stale-route auto-clear REMOVED — they wiped the HUD when the nav
// stream merely PAUSED (stopped at a light) during ACTIVE nav. send_clear() below remains, used
// only by the one-shot startup-clear (wipe a stale cluster frame at bring-up before any route).

// Push a blank maneuver frame + blank street strip to WIPE the HUD. Used by the stale-route
// auto-clear and by the fresh-bring-up clear (the cluster ECU keeps the last frame across a
// cold boot / source switch and we never get a teardown to undo it). Returns false on a send
// error so the caller can retry next tick. Bumps sync_bit so the cluster re-pages the strip.
bool send_clear(uint8_t &sync_bit)
{
    if (g_conn == nullptr) return false;
    if (g_hud_absent.load(std::memory_order_acquire)) return true;   // no HUD -> nothing to wipe

    VbsNaviHudDisplay disp = {};                 // all-zero -> blank maneuver/distance/splim
    int rc = VBS_NAVI_SetHUDDisplayMsgReq(g_conn, &disp, nullptr, nullptr, nullptr);
    if (rc != 0) { LOGE("hud_send: clear Msg1 failed rc=%d", rc); return false; }

    sync_bit = static_cast<uint8_t>((sync_bit % 7) + 1);   // bump -> cluster re-pages the strip
    VbsNaviHudMsg2 msg2 = {};
    const char *blank = " ";                     // empty -> ECU draws garbage; a space = clean blank
    msg2.guidancePointName = blank;
    msg2.syncBit           = sync_bit;
    rc = VBS_NAVI_TMC_SetHUD_Display_Msg2(g_conn, &msg2, nullptr, nullptr, nullptr);
    if (rc != 0) { LOGE("hud_send: clear Msg2 failed rc=%d", rc); return false; }

    LOGD("hud_send: HUD cleared (blank maneuver + street)");
    return true;
}

// Bring up the OEM service-bus connection. Runs ON the sender thread so
// session start stays cheap. Creates+connects the libjcidbus connection
// (which disables exit-on-disconnect), starts its worker, fires a one-shot
// fail-open HUD-presence probe. Returns true once the connection is up.
bool sender_setup()
{
    if (g_stop.load(std::memory_order_relaxed)) return false;

    g_conn = JCIDBUS_conn_create(reinterpret_cast<void *>(&hud_dbus_error_cb), 0);
    if (g_conn == nullptr) {
        LOGC("hud sender: JCIDBUS_conn_create failed (OEM symbols unavailable?) "
             "— no HUD this session");
        return false;
    }

    LOGD("hud sender: connecting to service bus as %s", kBusName);
    if (JCIDBUS_conn_connect(g_conn, kBusName, kJciServiceBus, nullptr) == 0) {
        LOGE("hud sender: JCIDBUS_conn_connect failed — no HUD this session");
        JCIDBUS_conn_free(g_conn);
        g_conn = nullptr;
        return false;
    }

    JCIDBUS_worker_start(g_conn);
    LOGD("hud sender: service bus connected, worker started");

    if (g_stop.load(std::memory_order_relaxed)) return false;

    VBS_NAVI_GetHUDStatus(g_conn, reinterpret_cast<void *>(&hud_status_cb), nullptr);
    LOGD("hud sender: HUD-status probe sent (sending enabled, fail-open)");
    return true;
}

// Clear the HUD and release the OEM connection. Runs on the sender thread as
// it winds down (it owns the connection). Safe when setup never completed
// (NULL g_conn skips everything). CLEAN teardown — the thing the old dbus-c++
// dispatcher never did (it was left polling a dead socket). No leak needed.
void sender_teardown()
{
    if (g_conn == nullptr) return;

    if (!g_hud_absent.load(std::memory_order_acquire)) {
        VbsNaviHudDisplay clear = {};   // all fields zero -> blank the HUD
        int rc = VBS_NAVI_SetHUDDisplayMsgReq(g_conn, &clear, nullptr, nullptr, nullptr);
        if (rc != 0) LOGE("hud sender: clear-frame send failed rc=%d", rc);
        else         LOGD("hud sender: sent HUD clear frame");
        usleep(50 * 1000);   // let the worker flush the clear before we stop it
    }

    JCIDBUS_worker_stop(g_conn);
    JCIDBUS_conn_disconnect(g_conn);
    JCIDBUS_conn_free(g_conn);
    g_conn = nullptr;
    LOGD("hud sender: worker stopped, connection freed");
}

// [VN-PATCH] ~2Hz ticker: nudges the sender to re-read /data_persist/splim and
// re-assert the HUD frame even when the AA snapshot is static. Uses usleep()
// (libc, relative) + a plain cv notify — deliberately NOT
// condition_variable::wait_for, whose std::chrono dependency needs
// GLIBCXX_3.4.19, a libstdc++ symbol the CMU's older libstdc++ lacks.
void *splim_ticker_main(void *)
{
    while (!g_stop.load(std::memory_order_relaxed)) {
        usleep(500 * 1000);   // ~2Hz native cadence
        g_splim_tick.fetch_add(1, std::memory_order_relaxed);
        wake_sender();
    }
    return nullptr;
}

void *sender_main(void *)
{
    // All D-Bus bring-up on this thread -> session start is never blocked.
    if (!sender_setup()) {
        sender_teardown();
        LOGD("hud sender: setup did not complete; thread exiting");
        return nullptr;
    }
    g_active.store(true, std::memory_order_release);
    LOGD("hud sender: HUD plumbing ready (OEM transport)");

    // Start the ~2Hz splim/keep-alive ticker now the pipeline is live.
    if (preload_thread_create(&g_splim_ticker_thread, splim_ticker_main, nullptr) == 0) {
        g_splim_ticker_up = true;
    } else {
        LOGC("hud sender: failed to spawn splim ticker (HUD still works on CarPlay changes)");
    }

    // [VIETMAP-BLE] When carplay_speed_limit OR carplay_vietmap_nav is enabled,
    // auto-launch splim_seriald (ESP32 CDC "SPLIM" → /tmp/splim, "NAV" → /tmp/vmnav).
    // Clear LD_PRELOAD/LD_LIBRARY_PATH so the child daemon never maps libpatch-blmjcicarplay.so.
    if (libpatch_config::carplay_speed_limit() || libpatch_config::carplay_vietmap_nav()) {
        (void)system("LD_PRELOAD= LD_LIBRARY_PATH= pgrep splim_seriald >/dev/null 2>&1 || "
                     "LD_PRELOAD= LD_LIBRARY_PATH= setsid sh /data_persist/splim_seriald_start.sh >/dev/null 2>&1 &");
    }

    NaviSnapshot prev = {};
    uint8_t      sync_bit = 0;
    uint32_t     last_processed = 0;
    uint16_t     prev_splim = 0;
    bool         prev_vmnav = false;
    uint32_t     last_tick = 0;

    // [NAV-END] Fresh bring-up: if no route is active (neither CarPlay iAP2 nor
    // VietMap BLE), wipe any maneuver the cluster ECU retained from a previous
    // session / cold boot.
    NaviSnapshot init_vmnav = {};
    if (!g_have_guidance.load(std::memory_order_relaxed) && !read_vmnav(init_vmnav)) {
        if (send_clear(sync_bit))
            LOGD("hud sender: startup HUD clear (no active route)");
    }

    while (!g_stop.load(std::memory_order_relaxed)) {
      try {  // [STABILITY] no exception may escape this thread (std::terminate -> SIGABRT -> reboot)
        // Seqlock read of g_snapshot. Consistent iff seq is even before AND
        // after the memcpy AND they match. Retry on tear.
        NaviSnapshot snap;
        uint32_t s1, s2;
        for (;;) {
            s1 = g_seq.load(std::memory_order_acquire);
            if (s1 & 1u) { sched_yield(); continue; }
            std::memcpy(&snap, &g_snapshot, sizeof(snap));
            std::atomic_thread_fence(std::memory_order_acquire);
            s2 = g_seq.load(std::memory_order_relaxed);
            if (s1 == s2) break;
        }

        uint16_t splim = read_splim();
        bool splim_changed = (splim != prev_splim);
        // [OVERSPEED] When carspeed_d flags over-speed, BLINK the limit glyph by
        // painting 0 on alternate 2Hz-ticker phases (~1Hz blink, steady regardless
        // of AA wake rate). Modulate ONLY the painted value: the TRUE splim still
        // drives splim_changed + prev_splim below, so the blink never triggers a
        // spurious Msg2/re-page, and turn/distance/street are untouched.
        uint16_t painted = splim;
        if (splim != 0 && read_over() &&
            (g_splim_tick.load(std::memory_order_relaxed) & 1u))
            painted = 0;

        static const NaviSnapshot kBlankSnap = {};

        // Process explicit clear requests FIRST so a clear isn't preceded by a
        // redundant maneuver send on the same wake.
        bool full_req = g_fullclear_req.exchange(false, std::memory_order_acquire);
        bool nav_req  = g_clear_req.exchange(false, std::memory_order_acquire);
        if (full_req) {
            LOGD("hud: CarPlay session gone -> full HUD wipe (incl sign)");
            if (send_clear(sync_bit)) {
                g_have_guidance.store(false, std::memory_order_relaxed);
                prev           = {};
                prev_splim     = 0;
                prev_vmnav     = false;
                last_processed = g_seq.load(std::memory_order_acquire);
                nav_request_reset();
            }
        } else if (nav_req) {
            // CarPlay iAP2 TBT turned off (TurnByTurnEntity != CARPLAY). If VietMap
            // BLE nav is active, hand the HUD straight over to VietMap without wiping.
            g_have_guidance.store(false, std::memory_order_relaxed);
            nav_request_reset();
            last_processed = g_seq.load(std::memory_order_acquire);
            NaviSnapshot vmnav_snap;
            if (read_vmnav(vmnav_snap)) {
                LOGD("hud: CarPlay iAP2 TBT off -> keeping VietMap BLE nav");
                if (send_one(vmnav_snap, prev, sync_bit, painted, splim_changed, /*force=*/true)) {
                    prev       = vmnav_snap;
                    prev_splim = splim;
                    prev_vmnav = true;
                }
            } else {
                LOGD("hud: CarPlay TBT off (still connected) -> maneuver off, keep sign");
                if (libpatch_config::carplay_speed_limit() && splim > 0) {
                    if (send_one(kBlankSnap, prev, sync_bit, painted, /*splim_changed=*/true, /*force=*/true)) {
                        prev       = kBlankSnap;
                        prev_splim = splim;
                        prev_vmnav = false;
                    }
                } else if (send_clear(sync_bit)) {
                    prev       = {};
                    prev_splim = 0;
                    prev_vmnav = false;
                }
            }
        } else {
            const bool cp_guiding = g_have_guidance.load(std::memory_order_acquire);
            // Only poll /tmp/vmnav when CarPlay iAP2 is NOT actively guiding (zero overhead
            // during Apple Maps / Google Maps turns).
            NaviSnapshot vmnav_snap;
            const bool using_vmnav = (!cp_guiding && read_vmnav(vmnav_snap));
            const NaviSnapshot &curSnap =
                cp_guiding  ? snap :
                using_vmnav ? vmnav_snap : kBlankSnap;

            const bool vmnav_ended = (prev_vmnav && !using_vmnav && !cp_guiding);
            const bool vmnav_changed =
                (!cp_guiding) &&
                (using_vmnav != prev_vmnav ||
                 (using_vmnav &&
                  (vmnav_snap.direct_icon   != prev.direct_icon   ||
                   vmnav_snap.distance_dec  != prev.distance_dec  ||
                   vmnav_snap.distance_unit != prev.distance_unit ||
                   std::strncmp(vmnav_snap.road_name, prev.road_name, sizeof(prev.road_name)) != 0)));

            if (vmnav_ended) {
                LOGD("hud: VietMap BLE nav ended -> maneuver off (splim=%u)",
                     static_cast<unsigned>(splim));
                if (libpatch_config::carplay_speed_limit() && splim > 0) {
                    if (send_one(kBlankSnap, prev, sync_bit, painted, /*splim_changed=*/true, /*force=*/true)) {
                        prev           = kBlankSnap;
                        prev_splim     = splim;
                        prev_vmnav     = false;
                        last_processed = s2;
                    }
                } else if (send_clear(sync_bit)) {
                    prev           = {};
                    prev_splim     = 0;
                    prev_vmnav     = false;
                    last_processed = s2;
                }
            } else if (s2 != last_processed || splim_changed || vmnav_changed) {
                if (send_one(curSnap, prev, sync_bit, painted, splim_changed, /*force=*/false)) {
                    prev           = curSnap;
                    prev_splim     = splim;
                    prev_vmnav     = using_vmnav;
                    last_processed = s2;
                }
            } else if (cp_guiding || using_vmnav || prev_splim != 0) {
                // [KEEPALIVE] Ticker woke us with no change -> re-assert the
                // last frame so the cluster ECU doesn't age out the maneuver OR
                // the speed-limit (native re-asserts ~2Hz).
                send_one(prev, prev, sync_bit, painted, false, /*force=*/true);
            }
        }

        // Predicated wait — wake on stop, snapshot change, clear request, or
        // splim ticker tick. Plain wait() (no timeout/clock): wait_for would
        // pull std::chrono::steady_clock::now@GLIBCXX_3.4.19 the CMU lacks.
        last_tick = g_splim_tick.load(std::memory_order_relaxed);
        std::unique_lock<std::mutex> lk(g_cv_mu);
        g_cv.wait(lk, [&]() {
            return g_stop.load(std::memory_order_relaxed) ||
                   g_clear_req.load(std::memory_order_acquire) ||
                   g_fullclear_req.load(std::memory_order_acquire) ||
                   g_seq.load(std::memory_order_acquire) != last_processed ||
                   g_splim_tick.load(std::memory_order_relaxed) != last_tick;
        });
      } catch (...) {
        // [STABILITY] Swallow everything: a thrown exception escaping this thread
        // aborts jciCARPLAY (-> reboot). Log, brief back-off, keep looping.
        LOGE("hud: sender loop exception swallowed — continuing (no thread escape)");
        usleep(200 * 1000);
      }
    }

    // Wind down: stop producers from mattering, join the ticker, clear + free.
    g_active.store(false, std::memory_order_release);
    if (g_splim_ticker_up) {
        pthread_join(g_splim_ticker_thread, nullptr);
        g_splim_ticker_up     = false;
        g_splim_ticker_thread = 0;
    }
    sender_teardown();
    LOGD("hud sender: thread exiting cleanly");
    return nullptr;
}

// Seqlock write helpers used by hud_on_* below.
inline void seqlock_begin() { g_seq.fetch_add(1, std::memory_order_acq_rel); }
inline void seqlock_end()   { g_seq.fetch_add(1, std::memory_order_acq_rel);
                              wake_sender(); }

} // namespace

// === Lifecycle (called from lifecycle.cpp PLT shims) ==========

void hud_send_start(void)
{
    if (g_sender_thread_up) {
        LOGD("hud_send_start: already running");
        return;
    }
    // Keep session start cheap: spawn the sender thread and return
    // immediately. ALL D-Bus work (connect, HUD-presence probe, bus attach)
    // happens on that thread (sender_setup). If the HUD is absent the thread
    // just keeps draining snapshots with sends gated off.
    g_stop.store(false, std::memory_order_release);
    g_active.store(false, std::memory_order_release);

    if (preload_thread_create(&g_sender_thread, sender_main, nullptr) != 0) {
        LOGC("hud_send_start: failed to spawn sender thread");
        return;
    }
    g_sender_thread_up = true;
    LOGD("hud_send_start: sender thread spawned (OEM D-Bus setup deferred to thread)");
}

void hud_send_stop(void)
{
    if (!g_sender_thread_up) return;

    // Signal stop + wake the sender from its cv wait under g_cv_mu so we can
    // never lose the wakeup and deadlock pthread_join.
    {
        std::lock_guard<std::mutex> lk(g_cv_mu);
        g_stop.store(true, std::memory_order_release);
        g_have_guidance.store(false, std::memory_order_relaxed);
    }
    g_cv.notify_all();

    pthread_join(g_sender_thread, nullptr);
    g_sender_thread_up = false;
    g_sender_thread    = 0;
    g_active.store(false, std::memory_order_release);
    LOGD("hud_send_stop: sender thread stopped, OEM D-Bus released");
}

// [NAV-END] Request an immediate HUD wipe. Called from the OEM CarPlay TBT-status callback
// (devmgr_shim cp_*_cb, running on the HMI D-Bus worker thread) when CarPlay nav goes inactive
// (TurnByTurnEntity != CARPLAY, or session deactivated). Cross-thread safe: it ONLY sets an
// atomic flag and wakes the sender; the sender does the actual send_clear on its own thread, so
// there is never a second writer to the seqlock-protected snapshot.
void hud_request_clear(void)
{
    g_clear_req.store(true, std::memory_order_release);
    wake_sender();
}

void hud_request_fullclear(void)
{
    g_fullclear_req.store(true, std::memory_order_release);
    wake_sender();
}

// === Producer side (runs on the SDK callback thread) ==========

void hud_on_status(uint32_t status)
{
    // [RESUME-FIX] Always keep g_snapshot current (no g_active gate) so a
    // late-starting sender replays the live maneuver on HUD bring-up after a
    // mid-nav power-cycle.
    if (status != 1u) {
        g_have_guidance.store(false, std::memory_order_release);  // [KEEPALIVE] route ended
        seqlock_begin();
        std::memset(&g_snapshot, 0, sizeof(g_snapshot));
        seqlock_end();
    } else {
        g_have_guidance.store(true, std::memory_order_release);   // [KEEPALIVE] route active
        wake_sender();   // wake sender to re-evaluate promptly on route start
    }
}

void hud_on_next_turn(const char *road_name,
                      uint32_t    turn_side,
                      uint32_t    turn_event,
                      int32_t     turn_angle,
                      int32_t     turn_number)
{
    seqlock_begin();
    if (road_name) {
        // [VN-PATCH] Smart UTF-8 truncation: don't cut mid multi-byte sequence
        // (VN diacritic chars are 2-3 bytes; a mid-sequence cut -> invalid UTF-8).
        const size_t max = sizeof(g_snapshot.road_name) - 1;
        size_t n = strnlen(road_name, max + 1);
        if (n > max) {
            n = max;
            while (n > 0 &&
                   (static_cast<unsigned char>(road_name[n]) & 0xC0) == 0x80) {
                n--;
            }
        }
        std::memcpy(g_snapshot.road_name, road_name, n);
        g_snapshot.road_name[n] = '\0';
    } else {
        g_snapshot.road_name[0] = '\0';
    }
    g_snapshot.turn_side   = turn_side;
    g_snapshot.turn_event  = turn_event;
    g_snapshot.turn_angle  = turn_angle;
    g_snapshot.turn_number = turn_number;
    // Publish g_have_guidance BEFORE seqlock_end() wakes the sender so the
    // sender never sees the new snapshot while g_have_guidance is still false.
    g_have_guidance.store(true, std::memory_order_release);
    g_last_nav.store((long)std::time(nullptr), std::memory_order_relaxed);
    seqlock_end();
}

void hud_on_distance(int32_t  /*distance_meters*/,
                     int32_t  time_until_seconds,
                     int32_t  display_distance,
                     uint32_t display_distance_unit)
{
    seqlock_begin();
    g_snapshot.distance_dec  = display_distance / 100;   // raw*1000 (proto) -> raw*10 (HUD)
    g_snapshot.distance_unit = map_distance_unit(display_distance_unit);
    g_snapshot.time_until    = time_until_seconds;
    g_last_nav.store((long)std::time(nullptr), std::memory_order_relaxed);  // [NAV-END]
    seqlock_end();
}
