// evdev type-B multi-touch reader for /dev/input/filtered-touchscreen0.
//
// Started by the aap_create_session shim (lifecycle.cpp) once a
// session is established; stopped on aap_destroy_session. Runs in a
// single dedicated pthread that:
//
//   1. blocks on poll() (so we can exit cleanly on touch_stop()
//      without races on close()-from-another-thread);
//   2. drains a non-blocking read() into struct input_event;
//   3. tracks current slot + tracking_id + position per slot;
//   4. on each SYN_REPORT, builds an MtState snapshot and hands it
//      to touch_on_frame() (touch_send.cpp).
//
// We talk to evdev directly because TUI_OpenReaderTouchScreen (from
// libjcituireader.so) only delivers one finger per callback — its
// TUI_TouchInput_s has no array — so it can't carry multi-touch.

#include "../patch.h"

#include <atomic>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

namespace {

constexpr char kTouchDevice[] = "/dev/input/filtered-touchscreen0";

int               g_fd = -1;
pthread_t         g_thread;
std::atomic<bool> g_stop{false};
bool              g_thread_alive = false;

// Initialise a brand-new MtState with all slots empty.
void init_state(MtState *s)
{
    for (int i = 0; i < kMaxFingers; ++i) {
        s->fingers[i].tracking_id = -1;
        s->fingers[i].x = 0;
        s->fingers[i].y = 0;
    }
    s->n_active = 0;
}

void recount_active(MtState *s)
{
    int n = 0;
    for (int i = 0; i < kMaxFingers; ++i) {
        if (s->fingers[i].tracking_id >= 0) ++n;
    }
    s->n_active = n;
}

void *reader_main(void *)
{
    MtState cur;
    init_state(&cur);

    int active_slot = 0;

    while (!g_stop.load(std::memory_order_relaxed)) {
        struct pollfd pfd = { g_fd, POLLIN, 0 };

        // 250 ms timeout — short enough that touch_stop() doesn't
        // block visibly, long enough not to burn CPU.
        int pr = poll(&pfd, 1, 250);
        if (pr < 0) {
            if (errno == EINTR) continue;
            LOGE("touch reader: poll() failed: %s", strerror(errno));
            break;
        }
        if (pr == 0) continue;                 // timeout, recheck stop flag
        if (!(pfd.revents & POLLIN)) continue;

        struct input_event ev;
        ssize_t n = read(g_fd, &ev, sizeof(ev));
        if (n != static_cast<ssize_t>(sizeof(ev))) {
            if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            LOGE("touch reader: short read (%zd, errno=%d)", n, errno);
            break;
        }

        LOGV("evdev: type=%u code=%u value=%d (slot=%d)",
             ev.type, ev.code, ev.value, active_slot);

        switch (ev.type) {
        case EV_ABS:
            switch (ev.code) {
            case ABS_MT_SLOT:
                if (ev.value >= 0 && ev.value < kMaxFingers) {
                    active_slot = ev.value;
                } else {
                    active_slot = 0;
                }
                break;
            case ABS_MT_TRACKING_ID:
                // ev.value == -1  -> finger released (slot becomes empty)
                // ev.value >= 0   -> new finger landed in this slot
                cur.fingers[active_slot].tracking_id = ev.value;
                break;
            case ABS_MT_POSITION_X:
                cur.fingers[active_slot].x = static_cast<uint32_t>(ev.value);
                break;
            case ABS_MT_POSITION_Y:
                cur.fingers[active_slot].y = static_cast<uint32_t>(ev.value);
                break;
            default:
                break;
            }
            break;

        case EV_SYN:
            if (ev.code == SYN_REPORT) {
                recount_active(&cur);
                LOGV("SYN_REPORT: n_active=%d slot0=(id=%d x=%u y=%u) "
                     "slot1=(id=%d x=%u y=%u)",
                     cur.n_active,
                     cur.fingers[0].tracking_id, cur.fingers[0].x, cur.fingers[0].y,
                     cur.fingers[1].tracking_id, cur.fingers[1].x, cur.fingers[1].y);
                touch_on_frame(cur);
            } else if (ev.code == SYN_MT_REPORT) {
                // Type-A protocol marker — we don't expect it on a
                // type-B device, but accept it as a no-op for safety.
            } else if (ev.code == SYN_DROPPED) {
                // Kernel queue overflowed; the device state we have
                // may be stale. Reset and let the next genuine touch
                // re-establish state. This will emit a synthetic UP
                // path naturally because tracking_ids drop to -1.
                LOGW("touch reader: SYN_DROPPED — resetting slot state");
                init_state(&cur);
                touch_on_frame(cur);
            }
            break;

        default:
            break;
        }
    }

    LOGD("touch reader thread exiting");
    return nullptr;
}

} // namespace

void touch_start(void)
{
    if (g_thread_alive) return;

    g_fd = open(kTouchDevice, O_RDONLY | O_CLOEXEC | O_NONBLOCK);
    if (g_fd < 0) {
        LOGE("touch reader: open(%s) failed: %s",
                  kTouchDevice, strerror(errno));
        return;
    }

    g_stop.store(false, std::memory_order_release);

    int rc = pthread_create(&g_thread, nullptr, reader_main, nullptr);
    if (rc != 0) {
        LOGE("touch reader: pthread_create failed: %d", rc);
        close(g_fd);
        g_fd = -1;
        return;
    }
    g_thread_alive = true;
    LOGD("touch reader: opened %s (fd=%d), thread started",
         kTouchDevice, g_fd);
}

void touch_stop(void)
{
    if (!g_thread_alive) return;

    g_stop.store(true, std::memory_order_release);

    // The reader is polling with a 250 ms timeout; it will pick the
    // stop flag up promptly. Wait for it to exit before closing the
    // fd (so we don't race a read on a closed descriptor).
    pthread_join(g_thread, nullptr);
    g_thread_alive = false;

    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
}
