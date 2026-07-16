// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Mute -> Android Auto media pause/resume bridge.
//
// The stock head unit only mutes the AMPLIFIER on a user mute — the phone
// keeps streaming, so you miss podcast/audiobook/music content while muted.
// The OEM AA projection key path (HMIEventHandler::InputKey ->
// RaceAap::SendKeyInput) can send KEYCODE_MEDIA_PLAY but has NO pause
// keycode, so stock Android Auto is never paused on mute. This module fills
// that gap:
//
//   user mute   -> KEYCODE_MEDIA_PAUSE (0x7f)  (only if AA media has focus)
//   user unmute -> KEYCODE_MEDIA_PLAY  (0x7e)  (only if WE paused on mute)
//
// Trigger: the OEM audio manager broadcasts the user-visible mute state on
// the JCI service bus as com.jci.vbs.am / EntertainmentMuteStatus, a single
// byte (1 = muted, 0 = unmuted). That is the OEM's own high-level mute — the
// same signal libjciuiasystem.so uses to raise the on-screen mute wink.
// Transient anti-pop / source-switch mutes travel on the separate
// MuteStatus(muteType) signals and do NOT set EntertainmentMuteStatus, so
// keying off it gives exactly the deliberate mute the user perceives.
//
// Action: RaceAap::SendKeyInput with the same AAP_KeyEvent the OEM builds
// (eType = 0xe00, an Android keycode, a press then a release). We only PAUSE
// when AudioManager::IsAAMediaInPlaying reports AA media is actively playing
// (mirroring the OEM's own InputKey media handling) — so muting media the user
// already paused never arms a resume. A "we paused it" latch, updated only when
// a key is actually delivered, means we only ever resume what we paused; a
// source change while muted (AA loses focus) can't spuriously resume AA.
//
// Gated by libpatch.conf `mute_pauses_phone` (default off). Lifecycle-
// bracketed by aap_create_session / aap_destroy_session, so exactly one
// watcher thread is alive per AA session.

#define LOG_TAG "MUTE"
#include "../log.h"
#include "mute.h"
#include "../oem/blmjciaapa.h"   // AAP_KeyEvent, keycodes, OEM thunks
#include "../oem/libdbus.h"      // raw libdbus-1 receive path
#include "common/thread_util.h"  // preload_thread_create (bounded stack)

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <pthread.h>

namespace {

// Service-bus address. The OEM exports it in $JCI_SERVICE_BUS; the shipped
// value is unix:path=/tmp/dbus_service_socket, our fallback if it is unset
// in this PID (same fallback the HUD sender uses).
constexpr const char *kServiceBusFallback = "unix:path=/tmp/dbus_service_socket";

// com.jci.vbs.am EntertainmentMuteStatus — signal member + a subscribe
// match rule. Verified against the server module's introspection XML
// (jci/vbs/modules/libjcimod_am.so): one arg of D-Bus type 'y' (byte),
// 1 = EntertainmentMuteON, 0 = EntertainmentMuteOFF.
constexpr const char *kAmIface     = "com.jci.vbs.am";
constexpr const char *kMuteMember  = "EntertainmentMuteStatus";
constexpr const char *kMatchRule =
    "type='signal',interface='com.jci.vbs.am',member='EntertainmentMuteStatus'";

// How long dbus_connection_read_write blocks per iteration before we
// re-check the stop flag. Bounds the stop/join latency; the session teardown
// path is not latency-critical, so a few hundred ms is fine.
constexpr int kDispatchTimeoutMs = 250;

// Backoff before re-opening the bus connection after a disconnect. The
// service bus is stable during an AA session (blmjciaapa itself depends on
// it), so this is purely defensive; the wait is interruptible by stop.
constexpr int kReconnectDelayMs = 1000;

// SDK "session not connected yet" — expected briefly after create until the
// session reaches CONNECTED; do not log it at error level.
constexpr int kAapSessionNotConnected = 0x108;

pthread_t g_thread;
bool      g_thread_up = false;

std::atomic<bool>       g_stop{false};
std::mutex              g_stop_mu;
std::condition_variable g_stop_cv;

void *g_conn = nullptr;

// True once WE issued a media PAUSE in response to a mute, so we only resume
// (PLAY) what we paused. Touched only on the watcher thread.
bool g_we_paused = false;

// Interruptible sleep: wait up to `ms`, returning true if a stop was
// requested (caller should bail), false on timeout.
bool wait_stop(int ms)
{
    std::unique_lock<std::mutex> lk(g_stop_mu);
    return g_stop_cv.wait_for(lk, std::chrono::milliseconds(ms),
                              [] { return g_stop.load(std::memory_order_relaxed); });
}

// Return true if AA media is the active/focused audio source right now.
// IsAAMediaInFocus reads the AudioManager focus field (set by
// AudioFocusChangeCb), which stays true through a pause — so it correctly
// gates the resume (on unmute) even after we paused.
bool aa_media_in_focus()
{
    void *aap = Singleton_AapProc_GetInstance();
    if (!aap) return false;
    void *am = AapProc_GetAudioManager(aap);
    if (!am) return false;
    return AudioManager_IsAAMediaInFocus(am) != 0;
}

// Return true if AA media is actively PLAYING right now (not paused/stopped).
// Used to decide whether a mute should pause: if the user already paused the
// media, IsAAMediaInFocus is still true but IsAAMediaInPlaying is false, so we
// must not arm the resume latch (else unmute would force playback back on).
bool aa_media_in_playing()
{
    void *aap = Singleton_AapProc_GetInstance();
    if (!aap) return false;
    void *am = AapProc_GetAudioManager(aap);
    if (!am) return false;
    return AudioManager_IsAAMediaInPlaying(am) != 0;
}

// Send one media key press (down then up) to the phone via the OEM
// RaceAap::SendKeyInput, mirroring HMIEventHandler::InputKey's press/release
// pair and its eType = 0xe00. Returns true only if BOTH the press and release
// were accepted (rc 0). A rejected key — most commonly rc 0x108 while the
// session hasn't reached CONNECTED — means the phone did NOT act on it, so the
// caller must not treat the media as (un)paused.
bool send_media_key(uint32_t keycode)
{
    void *aap = Singleton_AapProc_GetInstance();
    if (!aap) { LOGV("send_media_key: AapProc instance not available"); return false; }
    void *race = AapProc_GetRaceAap(aap);
    if (!race) { LOGV("send_media_key: RaceAap not available"); return false; }

    AAP_KeyEvent evt;
    std::memset(&evt, 0, sizeof(evt));
    evt.eType    = kAAPKeyEventType;   // 0xe00, exactly as the OEM does
    evt.eKeyCode = keycode;

    evt.bDown = 1;                      // press
    int rc_down = RaceAap_SendKeyInput(race, &evt);
    if (rc_down != 0 && rc_down != kAapSessionNotConnected)
        LOGE("SendKeyInput(down key=0x%x) -> %d", keycode, rc_down);

    evt.bDown = 0;                      // release
    int rc_up = RaceAap_SendKeyInput(race, &evt);
    if (rc_up != 0 && rc_up != kAapSessionNotConnected)
        LOGE("SendKeyInput(up key=0x%x) -> %d", keycode, rc_up);

    bool delivered = (rc_down == 0 && rc_up == 0);
    LOGD("media key 0x%x (down+up) delivered=%d", keycode, delivered ? 1 : 0);
    return delivered;
}

// Act on a fresh EntertainmentMuteStatus value.
void on_mute_changed(bool muted)
{
    if (muted) {
        if (g_we_paused) {
            LOGV("mute: already paused by us — ignoring");
            return;
        }
        // Only pause when AA media is actually PLAYING. If the user (or the
        // phone) already paused it, IsAAMediaInFocus would still be true, but
        // arming the latch here would make the later unmute force playback of
        // media the user deliberately paused. IsAAMediaInPlaying implies AA
        // media is also the focused source, so it subsumes the focus check.
        if (!aa_media_in_playing()) {
            LOGD("mute: AA media not actively playing — not pausing");
            return;
        }
        // Arm the latch ONLY if the PAUSE was actually delivered; a rejected
        // key did not pause the phone, so we must not later resume it.
        if (send_media_key(kAAPKeyMediaPause))
            g_we_paused = true;
    } else {
        if (!g_we_paused) {
            LOGV("unmute: we did not pause — ignoring");
            return;
        }
        if (!aa_media_in_focus()) {
            // Source switched away while muted; abandon the pending resume so
            // we never resume AA in the background.
            LOGD("unmute: AA media not in focus — dropping pending resume");
            g_we_paused = false;
            return;
        }
        // Clear the latch ONLY once the PLAY is delivered, so a rejected resume
        // is retried on the next unmute rather than silently lost.
        if (send_media_key(kAAPKeyMediaPlay))
            g_we_paused = false;
    }
}

// Interpret one popped message; act on it iff it is the mute signal.
void handle_message(void *msg)
{
    if (dbus_message_is_signal(msg, kAmIface, kMuteMember) == 0)
        return;

    DBusMessageIter it;
    if (dbus_message_iter_init(msg, &it) == 0) {
        LOGW("EntertainmentMuteStatus signal carried no args");
        return;
    }
    int argtype = dbus_message_iter_get_arg_type(&it);
    if (argtype != DBUS_TYPE_BYTE) {
        LOGW("EntertainmentMuteStatus arg is not a byte (type=%d)", argtype);
        return;
    }

    // get_basic writes the byte into *value; use an over-sized zeroed union
    // so a mis-sized write can never scribble past a lone uint8_t.
    union { uint8_t y; uint64_t pad; } bv;
    bv.pad = 0;
    dbus_message_iter_get_basic(&it, &bv);
    uint8_t val = bv.y;

    LOGD("EntertainmentMuteStatus = %u (%s)", val, val ? "muted" : "unmuted");
    on_mute_changed(val != 0);
}

// Open a private service-bus connection and subscribe to the mute signal.
// Returns true on success; on any failure the connection is released.
bool setup_connection()
{
    const char *addr = getenv("JCI_SERVICE_BUS");
    if (addr == nullptr || addr[0] == '\0') {
        addr = kServiceBusFallback;
        LOGW("$JCI_SERVICE_BUS unset — falling back to %s", addr);
    }

    g_conn = dbus_connection_open_private(addr, nullptr);
    if (g_conn == nullptr) {
        LOGC("dbus_connection_open_private(%s) failed — mute bridge inactive", addr);
        return false;
    }

    // exit_on_disconnect FALSE before register, so a bus teardown (e.g. on a
    // source switch) can't raise a fatal signal in this PID.
    dbus_connection_set_exit_on_disconnect(g_conn, 0);

    if (dbus_bus_register(g_conn, nullptr) == 0) {
        LOGE("dbus_bus_register failed — mute bridge inactive");
        dbus_connection_close(g_conn);
        dbus_connection_unref(g_conn);
        g_conn = nullptr;
        return false;
    }

    dbus_bus_add_match(g_conn, kMatchRule, nullptr);
    dbus_connection_flush(g_conn);
    LOGD("subscribed to %s.%s on %s", kAmIface, kMuteMember, addr);
    return true;
}

void teardown_connection()
{
    if (g_conn == nullptr)
        return;
    // Private connections must be closed before the final unref.
    dbus_connection_close(g_conn);
    dbus_connection_unref(g_conn);
    g_conn = nullptr;
}

void *watcher_main(void *)
{
    // Initial mute state is not queried: we act on signal edges, so a "muted at
    // connect" session simply stays as stock until the next mute cycle, then
    // self-syncs. Likewise, each (RE)connection below starts from an unknown
    // mute state — after a bus disconnect we may have missed an unmute, so we
    // clear the "we paused it" latch on every connect. That prefers "mute
    // always silences" over "always auto-resume": worst case we miss one resume
    // right after a (rare) reconnect, instead of a mute becoming a silent no-op.
    while (!g_stop.load(std::memory_order_relaxed)) {
        g_we_paused = false;

        if (!setup_connection()) {
            teardown_connection();
            if (wait_stop(kReconnectDelayMs)) break;
            continue;
        }

        while (!g_stop.load(std::memory_order_relaxed)) {
            // Block up to kDispatchTimeoutMs for I/O. FALSE => disconnected.
            if (dbus_connection_read_write(g_conn, kDispatchTimeoutMs) == 0) {
                LOGW("service bus disconnected — will reconnect");
                break;
            }
            void *msg;
            while ((msg = dbus_connection_pop_message(g_conn)) != nullptr) {
                handle_message(msg);
                dbus_message_unref(msg);
            }
        }

        teardown_connection();
        if (g_stop.load(std::memory_order_relaxed)) break;
        if (wait_stop(kReconnectDelayMs)) break;
    }

    LOGD("mute watcher thread exiting");
    return nullptr;
}

} // namespace

void mute_post_aap_create_session(void)
{
    if (g_thread_up)
        return;

    g_stop.store(false, std::memory_order_release);
    g_we_paused = false;

    if (preload_thread_create(&g_thread, watcher_main, nullptr) != 0) {
        LOGC("failed to spawn mute watcher thread");
        return;
    }
    g_thread_up = true;
    LOGD("mute watcher thread started");
}

void mute_pre_aap_destroy_session(void)
{
    if (!g_thread_up)
        return;

    g_stop.store(true, std::memory_order_release);
    {
        std::lock_guard<std::mutex> lk(g_stop_mu);
    }
    g_stop_cv.notify_all();

    pthread_join(g_thread, nullptr);
    g_thread_up = false;
    g_we_paused = false;
    LOGD("mute watcher thread stopped");
}
