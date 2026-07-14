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
// (eType = 0xe00, an Android keycode, a press then a release). Guarded by
// AudioManager::IsAAMediaInFocus so we only touch playback when AA media is
// the active audio source. A "we paused it" latch means we only ever resume
// what we paused, so a source change while muted (AA loses focus) can't
// spuriously resume AA in the background.
//
// Gated by libpatch.conf `mute_pauses_phone` (default off). Lifecycle-
// bracketed by aap_create_session / aap_destroy_session, so exactly one
// watcher thread is alive per AA session.

#define LOG_TAG "MUTE"
#include "../log.h"
#include "mute.h"
#include "../oem/blmjciaapa.h"   // AAP_KeyEvent, keycodes, OEM thunks
#include "../oem/libdbus.h"      // raw libdbus-1 receive path

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
// gates both the pause (on mute) and the resume (on unmute).
bool aa_media_in_focus()
{
    void *aap = Singleton_AapProc_GetInstance();
    if (!aap) return false;
    void *am = AapProc_GetAudioManager(aap);
    if (!am) return false;
    return AudioManager_IsAAMediaInFocus(am) != 0;
}

// Send one media key press (down then up) to the phone via the OEM
// RaceAap::SendKeyInput, mirroring HMIEventHandler::InputKey's press/release
// pair and its eType = 0xe00.
void send_media_key(uint32_t keycode)
{
    void *aap = Singleton_AapProc_GetInstance();
    if (!aap) { LOGV("send_media_key: AapProc instance not available"); return; }
    void *race = AapProc_GetRaceAap(aap);
    if (!race) { LOGV("send_media_key: RaceAap not available"); return; }

    AAP_KeyEvent evt;
    std::memset(&evt, 0, sizeof(evt));
    evt.eType    = kAAPKeyEventType;   // 0xe00, exactly as the OEM does
    evt.eKeyCode = keycode;

    evt.bDown = 1;                      // press
    int rc = RaceAap_SendKeyInput(race, &evt);
    if (rc != 0 && rc != kAapSessionNotConnected)
        LOGE("SendKeyInput(down key=0x%x) -> %d", keycode, rc);

    evt.bDown = 0;                      // release
    rc = RaceAap_SendKeyInput(race, &evt);
    if (rc != 0 && rc != kAapSessionNotConnected)
        LOGE("SendKeyInput(up key=0x%x) -> %d", keycode, rc);

    LOGD("sent media key 0x%x (down+up)", keycode);
}

// Act on a fresh EntertainmentMuteStatus value.
void on_mute_changed(bool muted)
{
    if (muted) {
        if (g_we_paused) {
            LOGV("mute: already paused by us — ignoring");
            return;
        }
        if (!aa_media_in_focus()) {
            LOGD("mute: AA media not in focus — not pausing");
            return;
        }
        send_media_key(kAAPKeyMediaPause);
        g_we_paused = true;
    } else {
        if (!g_we_paused) {
            LOGV("unmute: we did not pause — ignoring");
            return;
        }
        // Clear the latch regardless: even if AA is no longer the focused
        // source (source switched while muted), the mute cycle is over.
        g_we_paused = false;
        if (!aa_media_in_focus()) {
            LOGD("unmute: AA media not in focus — not resuming");
            return;
        }
        send_media_key(kAAPKeyMediaPlay);
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
    // We track our own pause latch across reconnects; only session start
    // (mute_post_aap_create_session) resets it. Initial mute state is not
    // queried: we act on edges, so a "muted at connect" session simply stays
    // as stock until the next mute cycle, then self-syncs.
    while (!g_stop.load(std::memory_order_relaxed)) {
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

    if (pthread_create(&g_thread, nullptr, watcher_main, nullptr) != 0) {
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
