// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Adapted from headunit (https://github.com/Trevelopment/headunit),
// licensed under GNU AGPL v3.
// See NOTICE.md at the repo root for the full attribution.
//
// HUD output module — D-Bus client that converts the AAP nav events
// surfaced by hud.cpp into Mazda "Active Driving Display" HUD frames.
//
// === What this file does =====================================
//
// On every 0x500 / 0x501 / 0x502 event the SDK delivers to hud.cpp,
// hud.cpp calls one of the `hud_on_*` push functions in hud_send.h.
// Those run on the SDK protobuf-decode thread and MUST NOT block,
// so all they do here is:
//
//   1. Grab a seqlock-protected snapshot struct (g_snapshot).
//   2. Write the new fields into it.
//   3. Bump g_seq (acq_rel) twice, framing the write between odd
//      and even values so a reader can detect torn snapshots.
//   4. Notify the sender thread's condition variable.
//
// The dedicated sender thread (`sender_main` below) then:
//
//   1. Wakes from the cv.
//   2. Re-reads the snapshot via the standard seqlock pattern.
//   3. Diffs against the previous snapshot to decide which OEM
//      D-Bus calls are warranted (typically `SetHUDDisplayMsgReq`
//      for the maneuver/distance block, `SetHUD_Display_Msg2` for
//      a road-name change).
//   4. Calls the proxies and goes back to sleep.
//
// Threading is intentionally seqlock-with-cv rather than a SPSC
// ring because the HUD only cares about the LATEST state — if the
// phone sends 20 distance updates while the sender thread is busy
// with one D-Bus round-trip, we coalesce them into a single send
// of the latest values.

#define LOG_TAG "HUD"
#include "../log.h"
#include "hud_send.h"
#include "../oem/libjcivbsnaviclient.h"

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <cstdio>
#include <string>
#include <atomic>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

namespace {

// Well-known name we acquire on the service bus. libjcidbus's
// conn_by_env_name registers it via dbus_bus_request_name; a novel
// name avoids colliding with the real com.jci.vbs.navi *service*
// (which we only send method calls to, never host).
//
// The OEM D-Bus ABI (libjcidbus + libjcivbsnaviclient typedefs, the
// VbsNaviHud* wire structs, the resolved g_oem table) and its
// dlopen/dlsym symbol resolution live in
// oem/libjcivbsnaviclient.{h,cpp}.
constexpr const char *kBusName = "com.jci.aapa.hud";

// === Mazda HUD turn-icon enum =================================
//
// Mirrors `NaviTurns` in reference hud.h. These are the integer
// codes the HUD ECU interprets — the OEM glyph atlas is baked
// into the ECU firmware and is out of our scope. Same numbering
// as the reference, no remapping.
enum MazdaIcon : uint8_t {
    HUD_STRAIGHT            = 1,
    HUD_LEFT                = 2,
    HUD_RIGHT               = 3,
    HUD_SLIGHT_LEFT         = 4,
    HUD_SLIGHT_RIGHT        = 5,
    HUD_UNDER_BRIDGE        = 6,
    HUD_OFF_RAMP_RIGHT      = 7,
    HUD_DESTINATION         = 8,
    HUD_SHARP_RIGHT         = 9,
    HUD_U_TURN_RIGHT        = 10,
    HUD_SHARP_LEFT          = 11,
    HUD_FLAG                = 12,
    HUD_U_TURN_LEFT         = 13,
    HUD_FORK_RIGHT          = 14,
    HUD_FORK_LEFT           = 15,
    HUD_MERGE_LEFT          = 16,
    HUD_MERGE_RIGHT         = 17,
    HUD_EMPTY               = 18,
    // 19 is empty
    HUD_CROSS_RIGHT         = 20,
    HUD_CROSS_LEFT          = 21,
    HUD_MEDIAN_U_TURN_LEFT  = 22,
    HUD_MEDIAN_U_TURN_RIGHT = 23,
    HUD_CAR                 = 24,
    HUD_NO_CAR              = 25,
    // 26..29 are empty
    HUD_OFF_RAMP_LEFT       = 30,
    HUD_T_LEFT              = 31,
    HUD_T_RIGHT             = 32,
    HUD_DESTINATION_LEFT    = 33,
    HUD_DESTINATION_RIGHT   = 34,
    HUD_FLAG_LEFT           = 35,
    HUD_FLAG_RIGHT          = 36,
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
    uint8_t nearest = static_cast<uint8_t>((degrees + 15) / 30);
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
//
// The Mazda HUD doesn't distinguish the "rounded > 10" sub-units
// — it always shows one decimal and lets the magnitude speak for
// itself. So KILOMETERS10 and KILOMETERS both map to Mazda
// KILOMETERS, etc.
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
//
// Single producer (the AAP cb), single consumer (the sender
// thread). g_seq is the seqlock: even = consistent / readable,
// odd = write in progress. We never block writers, never block
// readers; readers retry on torn snapshots.
struct NaviSnapshot {
    char     road_name[64];      // NUL-terminated UTF-8
    uint32_t turn_side;          // proto TURN_SIDE: 1=L, 2=R, 3=U
    uint32_t turn_event;         // proto TURN_EVENT: 0..19 sparse
    int32_t  turn_angle;         // degrees, signed
    int32_t  turn_number;        // maneuver / exit number
    int32_t  distance_dec;       // value * 10 (one decimal place)
    uint8_t  distance_unit;      // ALREADY mapped to Mazda enum
    int32_t  time_until;         // seconds — ETA to next maneuver
};

NaviSnapshot           g_snapshot   = {};
std::atomic<uint32_t>  g_seq{0};
std::condition_variable g_cv;
std::mutex             g_cv_mu;
std::atomic<bool>      g_active{false};   // sender pipeline live: producers may write
std::atomic<bool>      g_stop{false};     // stop requested: sender thread should wind down

// Shared sender-thread handle (both backends run the same loop).
pthread_t g_sender_thread       = 0;
bool      g_sender_thread_up    = false;

// === Turn-icon resolution (backend-agnostic) ==================
//
// Pure function of the snapshot — both transports send the same
// glyph id, so the mapping lives here rather than inside send_one.
uint32_t compute_turn_icon(const NaviSnapshot &cur)
{
    if (cur.turn_event == 13 /*TURN_ROUNDABOUT_ENTER_AND_EXIT*/) {
        // side_index: 0=left-hand traffic, 1=right-hand. Convert
        // proto TURN_SIDE (1=L, 2=R, 3=U) to that binary —
        // UNSPECIFIED falls back to right-hand, matching the
        // reference's `side - 1`.
        int32_t side_lr = (cur.turn_side == 1) ? 0 : 1;
        return roundabout_icon(cur.turn_angle, side_lr);
    }
    if (cur.turn_event < 20) {
        int32_t side_idx = static_cast<int32_t>(cur.turn_side) - 1;
        if (side_idx < 0 || side_idx > 2) side_idx = 2;
        return kTurnIcons[cur.turn_event][side_idx];
    }
    return 0;
}

// === OEM connection state =====================================
//
// The connection handle is created and owned by the sender thread
// for its whole lifetime. libjcidbus spawns its OWN worker thread
// (JCIDBUS_worker_start) to do socket I/O, so — unlike the old
// dbus-c++ path — we run no event loop ourselves and have nothing
// polling a dead socket after a source switch.
void *g_conn = nullptr;

// HUD-present gate, fail-OPEN. Default false (= "not known absent",
// i.e. send). A one-shot VBS_NAVI_GetHUDStatus reply flips this to
// true on vehicles with no HUD. Fail-open means a lost/late reply
// never silences us for the whole drive (the failure mode of the
// old synchronous probe); the cost is a few harmless frames on a
// no-HUD car until the reply lands (the OEM module just returns an
// error byte and forwards nothing). sm recycles the PID, which
// re-probes, so a per-process latch is fine.
std::atomic<bool> g_hud_absent{false};

// libjcidbus connection error callback (stored at conn+0x10 and
// invoked by the library as cb(conn, closure) on connection-level
// errors). Must be non-NULL — the library dereferences and calls
// it. Keep it trivial and non-throwing; with exit-on-disconnect
// disabled, a bus drop is no longer fatal.
extern "C" void hud_dbus_error_cb(void *conn, void *closure)
{
    (void)conn; (void)closure;
    LOGW("hud sender: libjcidbus signalled a service-bus connection "
         "error (non-fatal; exit-on-disconnect is off)");
}

// VBS_NAVI_GetHUDStatus async reply. ABI from the decompiled
// internal trampoline: cb(conn, status_byte, user). A zero status
// byte means the HUD is not installed / not powered.
extern "C" void hud_status_cb(void *conn, unsigned char status, void *user)
{
    (void)conn; (void)user;
    const bool absent = (status == 0);
    g_hud_absent.store(absent, std::memory_order_release);
    LOGD("hud sender: GetHUDStatus reply = %u (%s)",
         static_cast<unsigned>(status), absent ? "no HUD" : "HUD present");
}

// Helper: snap vs prev → which OEM HUD calls to make. Pulled out
// for readability; sender_main owns the prev/sync_bit state and
// threads them through here. Sends are async (the OEM setters
// marshal and queue, then return); on a non-zero return we just
// log and let the next change retry.
void send_one(const NaviSnapshot &cur,
              const NaviSnapshot &prev,
              uint8_t            &sync_bit)
{
    // HUD reported absent by GetHUDStatus — drain the snapshot but
    // emit nothing (cheap; producers keep updating it regardless).
    if (g_hud_absent.load(std::memory_order_acquire)) {
        return;
    }

    // The OEM's NAVI_SendHUDGuidaceDataToVBS bumps syncBitCount (and
    // re-latches the Msg2 street page) on a new *maneuver instance* —
    // i.e. when the maneuver icon changes OR the street name changes —
    // not on distance/speed ticks. Mirror that: a new road name or a
    // new resolved turn-icon both start a new instance.
    bool event_changed = (std::strncmp(cur.road_name, prev.road_name,
                                       sizeof(cur.road_name)) != 0) ||
                         (compute_turn_icon(cur) != compute_turn_icon(prev));
    bool distance_changed = event_changed ||
                            cur.distance_dec  != prev.distance_dec  ||
                            cur.distance_unit != prev.distance_unit ||
                            cur.time_until    != prev.time_until    ||
                            cur.turn_angle    != prev.turn_angle    ||
                            cur.turn_side     != prev.turn_side     ||
                            cur.turn_event    != prev.turn_event;

    if (!event_changed && !distance_changed) {
        LOGV("hud_send: snapshot unchanged vs previous — no HUD frame sent");
        return;
    }

    // Plain-C OEM wire structs (no dbus-c++ marshalling types now).
    //   VbsNaviHudDisplay (uqyqyy): maneuver icon + distance block.
    //   VbsNaviHudMsg2    (sy):     street-name strip.
    // `road` owns the bytes msg2.guidancePointName points at; it must
    // outlive the set_hud_msg2() call below (it does — the library
    // copies the string while marshalling).
    VbsNaviHudDisplay disp = {};
    VbsNaviHudMsg2    msg2 = {};
    std::string       road;

    if (event_changed) {
        // Per reference: bump 1..7 cyclically. The HUD treats a
        // sync_bit change as a hint to refresh the street-name page
        // even if the underlying string is identical.
        sync_bit = static_cast<uint8_t>((sync_bit % 7) + 1);
        road = cur.road_name;
        msg2.guidancePointName = road.c_str();
        msg2.syncBit           = sync_bit;
    }

    if (distance_changed) {
        uint32_t icon = compute_turn_icon(cur);

#ifdef DEBUG
        // Debug aid: when our mapping produced no glyph for this
        // (turn_event, turn_side) pair, hijack the street-name strip
        // to show the raw codes so an observer on the HUD can record
        // them and extend the table later. Stripped in release.
        char dbg_label[33];
        if (icon == 0) {
            snprintf(dbg_label, sizeof(dbg_label),
                     "EV=%u S=%u A=%d",
                          static_cast<unsigned>(cur.turn_event),
                          static_cast<unsigned>(cur.turn_side),
                          static_cast<int>(cur.turn_angle));
            if (!event_changed) {
                sync_bit = static_cast<uint8_t>((sync_bit % 7) + 1);
            }
            road = dbg_label;
            msg2.guidancePointName = road.c_str();
            msg2.syncBit           = sync_bit;
            icon = MazdaIcon::HUD_EMPTY;
            event_changed = true;   // force Msg2 send below
        }
#endif

        disp.nextManeuverInfo  = icon;
        disp.distanceValue     = static_cast<uint16_t>(cur.distance_dec);
        disp.distanceUnit      = cur.distance_unit;
        disp.displaySpeedLimit = 0;     // speed limit — unused here
        disp.displaySpeedUnit  = 0;     // speed units — unused here
        disp.text_ID3          = sync_bit;
    }

    if (distance_changed) {
        LOGV("hud_send: SetHUDDisplayMsgReq icon=%u dist=%u unit=%u sync=%u",
             static_cast<unsigned>(disp.nextManeuverInfo),
             static_cast<unsigned>(disp.distanceValue),
             static_cast<unsigned>(disp.distanceUnit),
             static_cast<unsigned>(disp.text_ID3));
        int rc = VBS_NAVI_SetHUDDisplayMsgReq(g_conn, &disp, nullptr, nullptr, nullptr);
        if (rc != 0) {
            LOGE("hud_send: SetHUDDisplayMsgReq failed rc=%d", rc);
        }
    }
    if (event_changed) {
        LOGV("hud_send: SetHUD_Display_Msg2 road=\"%s\" sync=%u",
             road.c_str(), static_cast<unsigned>(msg2.syncBit));
        int rc = VBS_NAVI_TMC_SetHUD_Display_Msg2(g_conn, &msg2, nullptr, nullptr, nullptr);
        if (rc != 0) {
            LOGE("hud_send: SetHUD_Display_Msg2 failed rc=%d", rc);
        }
    }
}

// Bring up the OEM service-bus connection the sender needs. Runs ON
// the sender thread so session start stays cheap. Creates + connects
// the libjcidbus connection (which disables exit-on-disconnect for
// us), starts its worker, and fires a one-shot fail-open HUD-presence
// probe. Returns true once the connection is up; false if the OEM
// stack is unavailable, the connect failed, or a stop was requested
// mid-setup. (The OEM entry points self-resolve on first call and
// return a benign failure value if unavailable — see
// oem/libjcivbsnaviclient.h — so there is no separate resolve step.)
bool sender_setup()
{
    if (g_stop.load(std::memory_order_relaxed)) {
        LOGD("hud sender: stop requested before connect — setup aborting");
        return false;
    }

    // Create the connection object. error_cb must be non-NULL. A NULL
    // return also covers "OEM D-Bus symbols unavailable".
    g_conn = JCIDBUS_conn_create(reinterpret_cast<void *>(&hud_dbus_error_cb), 0);
    if (g_conn == nullptr) {
        LOGC("hud sender: JCIDBUS_conn_create failed (OEM symbols "
             "unavailable?) — no HUD this session");
        return false;
    }

    // Connect to the SERVICE bus ($JCI_SERVICE_BUS) and acquire our
    // well-known name. conn_connect returns non-zero on success;
    // internally it sets exit-on-disconnect FALSE — the whole point.
    LOGD("hud sender: connecting to service bus as %s", kBusName);
    if (JCIDBUS_conn_connect(g_conn, kBusName, kJciServiceBus, nullptr) == 0) {
        LOGE("hud sender: JCIDBUS_conn_connect failed (name not acquired "
             "or $JCI_SERVICE_BUS unset) — no HUD this session");
        JCIDBUS_conn_free(g_conn);
        g_conn = nullptr;
        return false;
    }

    // Start libjcidbus's own I/O worker thread.
    JCIDBUS_worker_start(g_conn);
    LOGD("hud sender: service bus connected, worker started");

    if (g_stop.load(std::memory_order_relaxed)) {
        LOGD("hud sender: stop requested after connect — setup aborting");
        return false;
    }

    // Fail-open HUD-presence probe: one async GetHUDStatus. The reply
    // (hud_status_cb) flips g_hud_absent only if there is no HUD.
    VBS_NAVI_GetHUDStatus(g_conn, reinterpret_cast<void *>(&hud_status_cb),
                          nullptr);
    LOGD("hud sender: HUD-status probe sent (sending enabled, fail-open)");
    return true;
}

// Clear the HUD and release the OEM connection. Runs on the sender
// thread as it winds down (it owns the connection), so the stop path
// on the lifecycle thread only has to signal + join. Safe to call
// when setup never completed: a NULL g_conn skips everything.
void sender_teardown()
{
    if (g_conn == nullptr) {
        return;
    }

    // Send one zeroed-out HUD frame so the display clears instead of
    // holding whatever maneuver was last shown when the phone
    // disconnected. Skipped when the HUD is known absent. We don't
    // touch the street strip (Msg2) — it fades on its own when the
    // main frame blanks, and an empty-string Msg2 is non-trivial
    // (the producer hard-caps at 1 page).
    if (!g_hud_absent.load(std::memory_order_acquire)) {
        VbsNaviHudDisplay clear = {};   // all fields zero
        int rc = VBS_NAVI_SetHUDDisplayMsgReq(g_conn, &clear, nullptr, nullptr, nullptr);
        if (rc != 0) {
            LOGE("hud sender: clear-frame send failed rc=%d", rc);
        } else {
            LOGD("hud sender: sent HUD clear frame");
        }
        // The clear frame is queued on the libjcidbus worker; give it
        // a brief moment to flush before we stop the worker below.
        usleep(50 * 1000);
    }

    // Clean teardown — the thing the old dbus-c++ dispatcher never
    // did (it was left polling a dead socket, which is what crashed
    // the process). Stop the worker, drop the bus name, free it.
    JCIDBUS_worker_stop(g_conn);
    JCIDBUS_conn_disconnect(g_conn);
    JCIDBUS_conn_free(g_conn);
    g_conn = nullptr;
    LOGD("hud sender: worker stopped, connection freed");
}

void *sender_main(void *)
{
    if (!sender_setup()) {
        // HUD absent, setup failed, or stop during setup. Leave
        // g_active false so producers keep ignoring events; clean up
        // anything sender_setup may have left behind on a stop race.
        sender_teardown();
        LOGD("hud sender: setup did not complete; thread exiting");
        return nullptr;
    }

    // Pipeline is live — let the producers start filling snapshots.
    g_active.store(true, std::memory_order_release);
    LOGD("hud sender: HUD plumbing ready");

    NaviSnapshot prev = {};
    uint8_t      sync_bit = 0;
    uint32_t     last_processed = 0;

    while (!g_stop.load(std::memory_order_relaxed)) {
        // Seqlock read of g_snapshot. Standard pattern: snapshot
        // is consistent iff seq is even before AND after the
        // memcpy AND they match. Retry forever on tear.
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

        if (s2 != last_processed) {
            send_one(snap, prev, sync_bit);
            prev = snap;
            last_processed = s2;
        }

        // Predicated wait — wake when seq advances past what we
        // just processed, or when shutdown is requested. Closes
        // the lost-wakeup race window between the snapshot read
        // and the wait.
        std::unique_lock<std::mutex> lk(g_cv_mu);
        g_cv.wait(lk, [&]() {
            return g_stop.load(std::memory_order_relaxed) ||
                   g_seq.load(std::memory_order_acquire) != last_processed;
        });
    }

    // Stop the producers from writing, then clear the HUD and
    // release the clients we own.
    LOGD("hud sender: stop requested — winding down (last processed seq=%u)",
         static_cast<unsigned>(last_processed));
    g_active.store(false, std::memory_order_release);
    sender_teardown();
    LOGD("hud sender: thread exiting cleanly");
    return nullptr;
}

// Seqlock write helpers used by hud_on_* below.
inline void seqlock_begin() { g_seq.fetch_add(1, std::memory_order_acq_rel); }
inline void seqlock_end()   { g_seq.fetch_add(1, std::memory_order_acq_rel);
                              g_cv.notify_one(); }

// Diagnostic: nav events arriving while the sender pipeline is not
// live (g_active==false) are silently dropped by the hud_on_*
// producers below. The SDK callback fires at high rate, so rate-limit
// the log: emit on the first drop and every 256th thereafter, carrying
// the running total. A session that keeps accumulating drops but never
// logs "HUD plumbing ready" never brought the pipeline up — the whole
// drive shows no HUD output. (A healthy session drops only a handful
// during the brief start-up window before g_active flips true, then
// the count stops climbing.)
std::atomic<uint32_t> g_dropped_inactive{0};

void note_inactive_drop(const char *which)
{
    uint32_t n = g_dropped_inactive.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n == 1u || (n & 0xffu) == 0u) {
        LOGW("hud producer: %s dropped — sender pipeline not active "
             "(g_active=false); %u nav event(s) dropped so far",
             which, static_cast<unsigned>(n));
    }
}

} // namespace

// === Lifecycle (called from lifecycle.cpp PLT shims) ==========

void hud_send_start(void)
{
    if (g_sender_thread_up) {
        LOGD("hud_send_start: already running");
        return;
    }

    // Keep session start cheap: spawn the sender thread and return
    // immediately. ALL D-Bus work — dispatcher init, the synchronous
    // HUD-presence probe, and the service-bus attach — happens on
    // that thread (sender_setup). If the HUD is absent the thread
    // just exits and nav events are ignored for this session.
    g_stop.store(false, std::memory_order_release);
    g_active.store(false, std::memory_order_release);

    if (pthread_create(&g_sender_thread, nullptr,
                       sender_main, nullptr) != 0) {
        LOGC("hud_send_start: failed to spawn sender thread");
        return;
    }
    g_sender_thread_up = true;
    LOGD("hud_send_start: sender thread spawned (D-Bus setup deferred to thread)");
}

void hud_send_stop(void)
{
    if (!g_sender_thread_up) {
        return;  // not running
    }

    // Signal stop and wake the sender from its cv wait. If it's still
    // mid-setup (e.g. blocked in the synchronous HUD-presence probe),
    // it picks the flag up at the next checkpoint. The sender thread
    // sends the HUD clear frame and releases the D-Bus clients itself
    // on its way out (sender_teardown), so here we only signal + join.
    g_stop.store(true, std::memory_order_release);
    g_cv.notify_all();

    pthread_join(g_sender_thread, nullptr);
    g_sender_thread_up = false;
    g_sender_thread    = 0;
    g_active.store(false, std::memory_order_release);
    LOGD("hud_send_stop: sender thread stopped, D-Bus clients released");
}

// === Producer side (runs on the SDK callback thread) ==========

void hud_on_status(uint32_t status)
{
    // No sender running (HUD not installed, or start failed) —
    // hud.cpp's dump_status() already logged the event's arrival;
    // record the drop (rate-limited) so a stuck session is visible.
    if (!g_active.load(std::memory_order_relaxed)) {
        note_inactive_drop("status (0x500)");
        return;
    }

    // Per hu.proto NAVMessagesStatus: START=1, STOP=2.
    // Treat STOP (or any non-START) as "clear the HUD": zero the
    // snapshot so the sender's diff produces a blank frame.
    if (status != 1u) {
        seqlock_begin();
        std::memset(&g_snapshot, 0, sizeof(g_snapshot));
        seqlock_end();
    } else {
        // Just wake the sender so it re-evaluates promptly on
        // route start.
        g_cv.notify_one();
    }
}

void hud_on_next_turn(const char *road_name,
                      uint32_t    turn_side,
                      uint32_t    turn_event,
                      int32_t     turn_angle,
                      int32_t     turn_number)
{
    if (!g_active.load(std::memory_order_relaxed)) {
        note_inactive_drop("next_turn (0x501)");
        return;
    }

    seqlock_begin();
    if (road_name) {
        std::strncpy(g_snapshot.road_name, road_name,
                     sizeof(g_snapshot.road_name) - 1);
        g_snapshot.road_name[sizeof(g_snapshot.road_name) - 1] = '\0';
    } else {
        g_snapshot.road_name[0] = '\0';
    }
    g_snapshot.turn_side   = turn_side;
    g_snapshot.turn_event  = turn_event;
    g_snapshot.turn_angle  = turn_angle;
    g_snapshot.turn_number = turn_number;
    seqlock_end();
}

void hud_on_distance(int32_t  /*distance_meters*/,
                     int32_t  time_until_seconds,
                     int32_t  display_distance,
                     uint32_t display_distance_unit)
{
    if (!g_active.load(std::memory_order_relaxed)) {
        note_inactive_drop("distance (0x502)");
        return;
    }

    seqlock_begin();
    // raw_unit * 1000 (proto) → raw_unit * 10 (HUD wire).
    g_snapshot.distance_dec  = display_distance / 100;
    g_snapshot.distance_unit = map_distance_unit(display_distance_unit);
    g_snapshot.time_until    = time_until_seconds;
    seqlock_end();
}
