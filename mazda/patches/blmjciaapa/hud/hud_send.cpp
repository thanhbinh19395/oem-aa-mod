// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Adapted from headunit (https://github.com/Trevelopment/headunit),
// licensed under GNU AGPL v3.
// See NOTICE.md at the repo root for the full attribution.
//
// HUD output module — D-Bus client that converts the AAP nav events
// surfaced by nav.cpp into Mazda "Active Driving Display" HUD frames.
//
// === What this file does =====================================
//
// On every 0x500 / 0x501 / 0x502 event the SDK delivers to nav.cpp,
// nav.cpp calls one of the `hud_on_*` push functions in hud_send.h.
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

#include "../patch.h"
#include "hud_send.h"

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <atomic>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#include <dbus/dbus.h>
#include <dbus-c++/dbus.h>

#include "generated_cmu.h"

namespace {

// === D-Bus addresses ==========================================
//
// Mazda's HMI and Service buses are NOT the standard system /
// session buses; they're private daemons listening on UNIX
// sockets at well-known paths. These addresses come from the
// reference implementation and have been stable across every
// observed FW (74.00.324A confirmed).
constexpr const char *kServiceBusAddr = "unix:path=/tmp/dbus_service_socket";
constexpr const char *kHmiBusAddr     = "unix:path=/tmp/dbus_hmi_socket";

// === Proxy class stubs ========================================
//
// `generated_cmu.h` generates each interface as a fully-abstract
// `_proxy` base class with one pure-virtual per signal. We only
// care about outgoing method calls; the inherited signal handlers
// get empty bodies so the classes are instantiable. The exact
// list of pure-virtuals was confirmed against the vendored
// generated_cmu.h before writing these.

class HUDSettingsClient : public com::jci::navi2IHU::HUDSettings_proxy,
                          public DBus::ObjectProxy {
public:
    HUDSettingsClient(DBus::Connection &c, const char *path, const char *name)
        : DBus::ObjectProxy(c, path, name) {}

    void HUDInstalledChanged(const bool &)                          override {}
    void SetHUDSettingFailed(const int32_t &, const int32_t &)      override {}
    void HUDControlAllowed(const bool &)                            override {}
    void HUDSettingChanged(const int32_t &, const int32_t &)        override {}
};

class NaviClient : public com::jci::vbs::navi_proxy,
                   public DBus::ObjectProxy {
public:
    NaviClient(DBus::Connection &c, const char *path, const char *name)
        : DBus::ObjectProxy(c, path, name) {}

    void FuelTypeResp(const uint8_t &)                              override {}
    void HUDResp(const uint8_t &)                                   override {}
    void TSRResp(const uint8_t &)                                   override {}
    void GccConfigMgmtResp(
        const ::DBus::Struct<std::vector<uint8_t>> &)               override {}
    void TSRFeatureMode(const uint8_t &)                            override {}
};

class TMCClient : public com::jci::vbs::navi::tmc_proxy,
                  public DBus::ObjectProxy {
public:
    TMCClient(DBus::Connection &c, const char *path, const char *name)
        : DBus::ObjectProxy(c, path, name) {}

    void ServiceListResponse(
        const ::DBus::Struct<uint8_t, std::vector<uint8_t>,
                             std::vector<uint8_t>, std::vector<uint8_t>,
                             std::vector<uint8_t>, std::vector<uint8_t>> &)
                                                                    override {}
    void ResponseToTMCSelection(const uint8_t &, const uint8_t &,
                                const uint8_t &, const uint8_t &,
                                const uint8_t &, const uint8_t &,
                                const uint8_t &)                    override {}
};

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
    /*  4 TURN_TURN                     */ {HUD_LEFT, HUD_RIGHT, 0},
    /*  5 TURN_SHARP_TURN               */ {HUD_SHARP_LEFT, HUD_SHARP_RIGHT, 0},
    /*  6 TURN_U_TURN                   */ {HUD_U_TURN_LEFT, HUD_U_TURN_RIGHT, 0},
    /*  7 TURN_ON_RAMP                  */ {HUD_LEFT, HUD_RIGHT, HUD_STRAIGHT},
    /*  8 TURN_OFF_RAMP                 */ {HUD_OFF_RAMP_LEFT, HUD_OFF_RAMP_RIGHT, HUD_STRAIGHT},
    /*  9 TURN_FORK                     */ {HUD_FORK_LEFT, HUD_FORK_RIGHT, 0},
    /* 10 TURN_MERGE                    */ {HUD_MERGE_LEFT, HUD_MERGE_RIGHT, 0},
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
std::atomic<bool>      g_active{false};

// === D-Bus state ==============================================
//
// All raw pointers because dbus-c++ requires `default_dispatcher`
// to be set before any Connection is constructed, and Connections
// have to outlive their proxies. Simpler to manage as bare
// new/delete from the start/stop pair than to fight ordered
// destruction of statics.
DBus::BusDispatcher  *g_dispatcher    = nullptr;
DBus::Connection     *g_service_bus   = nullptr;
NaviClient           *g_navi_client   = nullptr;
TMCClient            *g_tmc_client    = nullptr;

pthread_t g_dispatch_thread     = 0;
bool      g_dispatch_thread_up  = false;
pthread_t g_sender_thread       = 0;
bool      g_sender_thread_up    = false;

void *dispatcher_main(void *)
{
    // Blocks until BusDispatcher::leave(). We don't call leave()
    // anywhere — the dispatcher lives for the rest of the
    // process lifetime, which is OK because sm restarts the
    // {L_jciAAPA} PID on crash.
    g_dispatcher->enter();
    return nullptr;
}

// One-shot HUD-presence probe, cached for the rest of the process
// lifetime. Mazda HUDs are a factory option, wired into the dash —
// they don't hot-plug, so caching the answer forever is correct.
// (A misdetect would only be recoverable across an sm-managed PID
// restart, which already invalidates the cache.)
//
// Spins up a transient HMI-bus connection and HUDSettingsClient
// just long enough for the synchronous GetHUDIsInstalled() round
// trip. Both destruct on return, so cars with a HUD keep ZERO
// HMI-side state and cars without never open the service bus at
// all (hud_send_start gates the entire phase-2 setup on this).
//
// Caller is hud_send_start only, which is itself serialised by
// lifecycle.cpp's g_session_mu — no concurrent invocation, so no
// extra synchronisation needed on the static cache.
bool hud_installed_safe()
{
    // -1 = not yet probed, 0 = absent, 1 = installed.
    static int8_t cached = -1;
    if (cached >= 0) return cached == 1;

    try {
        // `false` second arg = don't auto-Hello. We call
        // register_bus() explicitly. Matches the reference.
        DBus::Connection  hmi_bus(kHmiBusAddr, false);
        hmi_bus.register_bus();
        HUDSettingsClient hud_client(hmi_bus,
                                     "/com/jci/navi2IHU",
                                     "com.jci.navi2IHU");
        cached = hud_client.GetHUDIsInstalled() ? 1 : 0;
        LOGD("hud_installed_safe: cached result = %s",
             cached == 1 ? "true" : "false");
    } catch (const DBus::Error &e) {
        // Probe couldn't run — most likely the HMI daemon isn't
        // up yet. Leave cached == -1 so the next aap_create_session
        // retries the probe; return false to this caller so we
        // don't try to send anything we can't validate.
        LOGE("hud_installed_safe: HMI bus / GetHUDIsInstalled failed: "
             "%s: %s — will retry on next session",
             e.name(), e.message());
    }

    return cached == 1;
}

// Helper: snap < prev → which OEM D-Bus calls to make. Pulled
// out for readability; sender_main owns the prev/sync_bit state
// and threads them through here. Returns void; on D-Bus error
// just logs and continues (next call will retry).
void send_one(const NaviSnapshot &cur,
              const NaviSnapshot &prev,
              uint8_t            &sync_bit)
{
    bool event_changed = (std::strncmp(cur.road_name, prev.road_name,
                                       sizeof(cur.road_name)) != 0);
    bool distance_changed = event_changed ||
                            cur.distance_dec  != prev.distance_dec  ||
                            cur.distance_unit != prev.distance_unit ||
                            cur.time_until    != prev.time_until    ||
                            cur.turn_angle    != prev.turn_angle    ||
                            cur.turn_side     != prev.turn_side     ||
                            cur.turn_event    != prev.turn_event;

    if (!event_changed && !distance_changed) return;

    // 12-byte SetHUDDisplayMsgReq struct: (uqyqyy)
    //   _1 u  nextManeuverInfo  (we use icon code from kTurnIcons)
    //   _2 q  distanceValue
    //   _3 y  distanceUnit
    //   _4 q  displaySpeedLimit (not used — left for HUD merger)
    //   _5 y  displaySpeedUnit  (not used)
    //   _6 y  text_ID3          (carries the sync bit per reference)
    ::DBus::Struct<uint32_t, uint16_t, uint8_t,
                   uint16_t, uint8_t,  uint8_t> hudDisplayMsg;
    ::DBus::Struct<std::string, uint8_t>         guidancePointData;

    if (event_changed) {
        // Per reference: bump 1..7 cyclically. The HUD treats
        // sync_bit changes as a hint to refresh the street-name
        // page even if the underlying string is identical.
        sync_bit = static_cast<uint8_t>((sync_bit % 7) + 1);
        guidancePointData._1 = cur.road_name;
        guidancePointData._2 = sync_bit;
    }

    if (distance_changed) {
        uint32_t icon = 0;
        if (cur.turn_event == 13 /*TURN_ROUNDABOUT_ENTER_AND_EXIT*/) {
            // side_index: 0=left-hand traffic, 1=right-hand.
            // Convert proto TURN_SIDE (1=L, 2=R, 3=U) to that
            // binary — UNSPECIFIED falls back to right-hand,
            // matching the reference's `side - 1` (which would
            // yield 2 for UNSPECIFIED, evaluated as truthy/right).
            int32_t side_lr = (cur.turn_side == 1) ? 0 : 1;
            icon = roundabout_icon(cur.turn_angle, side_lr);
        } else if (cur.turn_event < 20) {
            int32_t side_idx = static_cast<int32_t>(cur.turn_side) - 1;
            if (side_idx < 0 || side_idx > 2) side_idx = 2;
            icon = kTurnIcons[cur.turn_event][side_idx];
        }

#ifdef DEBUG
        // Debug aid: when our mapping produced no glyph for this
        // (turn_event, turn_side) pair — i.e. the kTurnIcons
        // entry is 0 or turn_event is out of range — hijack the
        // street-name strip to show the raw codes so an observer
        // on the HUD can record them and extend the mapping
        // table later. Stripped in release builds.
        char dbg_label[33];
        if (icon == 0) {
            snprintf(dbg_label, sizeof(dbg_label),
                     "EV=%u S=%u A=%d",
                     static_cast<unsigned>(cur.turn_event),
                     static_cast<unsigned>(cur.turn_side),
                     static_cast<int>(cur.turn_angle));
            // Bump sync_bit if event_changed didn't already do it
            // — the HUD treats a sync_bit change as a refresh hint.
            // (hudDisplayMsg._6 picks up the new value via the
            // unconditional assignment below.)
            if (!event_changed) {
                sync_bit = static_cast<uint8_t>((sync_bit % 7) + 1);
            }
            guidancePointData._1 = dbg_label;
            guidancePointData._2 = sync_bit;
            event_changed = true;   // force Msg2 send below
        }
#endif

        hudDisplayMsg._1 = icon;
        hudDisplayMsg._2 = static_cast<uint16_t>(cur.distance_dec);
        hudDisplayMsg._3 = cur.distance_unit;
        hudDisplayMsg._4 = 0;          // speed limit  — unused here
        hudDisplayMsg._5 = 0;          // speed units  — unused here
        hudDisplayMsg._6 = sync_bit;
    }

    try {
        if (distance_changed) {
            LOGV("hud_send: SetHUDDisplayMsgReq icon=%u dist=%u unit=%u sync=%u",
                 static_cast<unsigned>(hudDisplayMsg._1),
                 static_cast<unsigned>(hudDisplayMsg._2),
                 static_cast<unsigned>(hudDisplayMsg._3),
                 static_cast<unsigned>(hudDisplayMsg._6));
            g_navi_client->SetHUDDisplayMsgReq(hudDisplayMsg);
        }
        if (event_changed) {
            LOGV("hud_send: SetHUD_Display_Msg2 road=\"%s\" sync=%u",
                 cur.road_name, static_cast<unsigned>(sync_bit));
            g_tmc_client->SetHUD_Display_Msg2(guidancePointData);
        }
    } catch (const DBus::Error &e) {
        LOGE("hud_send: D-Bus send failed: %s: %s",
             e.name(), e.message());
    }
}

void *sender_main(void *)
{
    // Reference sleeps 1 s before first poll to give the HMI bus
    // time to bring `com.jci.navi2IHU` online after our connect.
    sleep(1);

    NaviSnapshot prev = {};
    uint8_t      sync_bit = 0;
    uint32_t     last_processed = 0;

    while (g_active.load(std::memory_order_relaxed)) {
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
            return !g_active.load(std::memory_order_relaxed) ||
                   g_seq.load(std::memory_order_acquire) != last_processed;
        });
    }
    return nullptr;
}

// Seqlock write helpers used by hud_on_* below.
inline void seqlock_begin() { g_seq.fetch_add(1, std::memory_order_acq_rel); }
inline void seqlock_end()   { g_seq.fetch_add(1, std::memory_order_acq_rel);
                              g_cv.notify_one(); }

} // namespace

// === Lifecycle (called from lifecycle.cpp PLT shims) ==========

void hud_send_start(void)
{
    if (g_active.load()) {
        LOGD("hud_send_start: already running");
        return;
    }

    // dbus-c++ requires a default dispatcher to be installed
    // BEFORE any Connection is constructed (it registers itself
    // for I/O readiness with the dispatcher in its ctor). Set
    // up once per process — the dispatcher stays alive forever.
    if (!g_dispatcher) {
        dbus_threads_init_default();
        g_dispatcher = new DBus::BusDispatcher();
        DBus::default_dispatcher = g_dispatcher;
        if (pthread_create(&g_dispatch_thread, nullptr,
                           dispatcher_main, nullptr) != 0) {
            LOGC("hud_send_start: failed to spawn dispatcher thread");
            return;
        }
        g_dispatch_thread_up = true;
        LOGD("hud_send_start: dispatcher thread up");
    }

    // Phase 1 — HUD-presence gate. Cached across sessions, so
    // the HMI bus only gets touched on the very first connect of
    // the {L_jciAAPA} PID's lifetime. nav.cpp keeps logging
    // incoming 0x500/0x501/0x502 events via its dump_* path
    // regardless of the outcome; the hud_on_* push functions
    // below early-return on !g_active so no snapshot work or
    // wakeups happen when no sender is running.
    if (!hud_installed_safe()) {
        LOGD("hud_send_start: HUD not installed on this vehicle — "
             "skipping service-bus open and sender thread");
        return;
    }

    // Phase 2 — HUD is present, bring up the service-side proxies
    // that the sender thread actually uses.
    try {
        g_service_bus = new DBus::Connection(kServiceBusAddr, false);
        g_service_bus->register_bus();
        g_navi_client = new NaviClient(*g_service_bus,
                                       "/com/jci/vbs/navi",
                                       "com.jci.vbs.navi");
        g_tmc_client  = new TMCClient(*g_service_bus,
                                      "/com/jci/vbs/navi",
                                      "com.jci.vbs.navi");
    } catch (const DBus::Error &e) {
        LOGE("hud_send_start: service bus attach failed: %s: %s",
             e.name(), e.message());
        delete g_tmc_client;  g_tmc_client  = nullptr;
        delete g_navi_client; g_navi_client = nullptr;
        delete g_service_bus; g_service_bus = nullptr;
        return;
    }

    g_active.store(true, std::memory_order_release);

    if (pthread_create(&g_sender_thread, nullptr,
                       sender_main, nullptr) != 0) {
        LOGC("hud_send_start: failed to spawn sender thread");
        g_active.store(false);
        return;
    }
    g_sender_thread_up = true;
    LOGD("hud_send_start: sender thread up; HUD plumbing ready");
}

void hud_send_stop(void)
{
    if (!g_active.exchange(false, std::memory_order_acq_rel)) {
        return;  // not running
    }

    g_cv.notify_all();
    if (g_sender_thread_up) {
        pthread_join(g_sender_thread, nullptr);
        g_sender_thread_up = false;
        g_sender_thread    = 0;
    }

    // Sender thread has exited; we're the only D-Bus user now.
    // Send one zeroed-out HUD frame so the display clears instead
    // of holding whatever maneuver was last shown when the phone
    // disconnected. Without this the HUD stays lit with stale
    // turn info until something else (NNG, BLM, or ignition cycle)
    // overwrites the VIP slot.
    //
    // Issued synchronously from the calling thread (the SDK's
    // aap_destroy_session caller). Safe — the sender thread is
    // joined above and no one else is touching g_navi_client.
    // We deliberately don't touch SetHUD_Display_Msg2 here: the
    // street-name strip naturally fades when the main frame goes
    // blank, and emitting an empty-string Msg2 is non-trivial
    // (the producer in libjcimod_navigation hard-caps at 1 page).
    if (g_navi_client) {
        try {
            ::DBus::Struct<uint32_t, uint16_t, uint8_t,
                           uint16_t, uint8_t,  uint8_t> clearMsg;
            clearMsg._1 = 0;  // icon
            clearMsg._2 = 0;  // distance
            clearMsg._3 = 0;  // distance unit
            clearMsg._4 = 0;  // speed limit (not used by us)
            clearMsg._5 = 0;  // speed unit  (not used by us)
            clearMsg._6 = 0;  // text_ID3 / sync bit
            g_navi_client->SetHUDDisplayMsgReq(clearMsg);
            LOGD("hud_send_stop: sent HUD clear frame");
        } catch (const DBus::Error &e) {
            LOGE("hud_send_stop: clear send failed: %s: %s",
                 e.name(), e.message());
        }
    }

    // Leave the dispatcher running. dbus-c++ doesn't expose a
    // clean teardown path that doesn't risk crashing inside the
    // dispatcher's own loop, and sm will recycle the PID anyway.
    delete g_navi_client; g_navi_client = nullptr;
    delete g_tmc_client;  g_tmc_client  = nullptr;
    delete g_service_bus; g_service_bus = nullptr;
    LOGD("hud_send_stop: sender thread stopped, D-Bus clients released");
}

// === Producer side (runs on the SDK callback thread) ==========

void hud_on_status(uint32_t status)
{
    // No sender running (HUD not installed, or start failed) —
    // nav.cpp's dump_status() already logged the event; nothing
    // else to do here.
    if (!g_active.load(std::memory_order_relaxed)) return;

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
    if (!g_active.load(std::memory_order_relaxed)) return;

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
    if (!g_active.load(std::memory_order_relaxed)) return;

    seqlock_begin();
    // raw_unit * 1000 (proto) → raw_unit * 10 (HUD wire).
    g_snapshot.distance_dec  = display_distance / 100;
    g_snapshot.distance_unit = map_distance_unit(display_distance_unit);
    g_snapshot.time_until    = time_until_seconds;
    seqlock_end();
}
