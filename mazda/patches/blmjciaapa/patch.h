// Shared globals, logging helper, and small types used across the
// patch's translation units.

#ifndef LIBPATCH_BLMJCIAAPA_PATCH_H
#define LIBPATCH_BLMJCIAAPA_PATCH_H

#include <stdint.h>
#include <cstdio>

// Master enable. Set by main.cpp's constructor after the self-gate
// confirms blmjciaapa.so is loaded (i.e. we are in the {L_jciAAPA}
// PID). Read by every other TU before doing anything that touches
// OEM memory.
extern bool g_enabled;

// Coarse log helpers. The launcher PID's stderr is captured by sm
// into the device log. Each message is tagged with a level letter so
// they're easy to grep:
//
//   [libpatch-blmjciaapa][V] verbose — per-frame / per-event touch chatter
//   [libpatch-blmjciaapa][D] debug   — lifecycle, one-shot resolution results
//   [libpatch-blmjciaapa][W] warn    — recoverable oddities (e.g. SYN_DROPPED)
//   [libpatch-blmjciaapa][E] error   — recoverable failures (open(), poll())
//   [libpatch-blmjciaapa][C] critical— shim cannot function (dlsym(RTLD_NEXT) miss)
//
// Levels are compile-time-gated by LOG_LEVEL. Anything below
// the threshold expands to an `if (false)` guard so the format string
// is still type-checked but the call is never executed (and is
// trivially dead-code-eliminated even at -O0). The Makefile passes
// -DDEBUG for the debug config and -DNDEBUG for release; we key off
// those: debug => VERBOSE, release => ERROR.
//
// We deliberately don't depend on a fancy logger — the OEM
// COMMON_Log_Write is reachable via the same anchor-and-offset trick,
// but pulling it in would add 2 more offset-bound symbols for no
// real benefit during bring-up.

#define LOG_LEVEL_VERBOSE  0
#define LOG_LEVEL_DEBUG    1
#define LOG_LEVEL_WARN     2
#define LOG_LEVEL_ERROR    3
#define LOG_LEVEL_CRITICAL 4
#define LOG_LEVEL_SILENT   5

#ifndef LOG_LEVEL
#  ifdef NDEBUG
#    define LOG_LEVEL LOG_LEVEL_ERROR
#  else
#    define LOG_LEVEL LOG_LEVEL_VERBOSE
#  endif
#endif

#define LOG_EMIT(tag, fmt, ...)                                            \
    do {                                                                   \
        std::fprintf(stderr, "[libpatch-blmjciaapa][" tag "] " fmt "\n",  \
                     ##__VA_ARGS__);                                       \
        std::fflush(stderr);                                               \
    } while (0)

// Keep the format string type-checked even when the level is
// compiled out. `if (false)` is folded by the compiler at all
// optimisation levels, so neither the fprintf nor the argument
// expressions are evaluated at runtime.
#define LOG_DROP(fmt, ...)                                                 \
    do {                                                                   \
        if (false) {                                                       \
            std::fprintf(stderr, "[libpatch-blmjciaapa] " fmt "\n",       \
                         ##__VA_ARGS__);                                   \
        }                                                                  \
    } while (0)

#if LOG_LEVEL <= LOG_LEVEL_VERBOSE
#  define LOGV(fmt, ...) LOG_EMIT("V", fmt, ##__VA_ARGS__)
#else
#  define LOGV(fmt, ...) LOG_DROP(fmt, ##__VA_ARGS__)
#endif

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#  define LOGD(fmt, ...) LOG_EMIT("D", fmt, ##__VA_ARGS__)
#else
#  define LOGD(fmt, ...) LOG_DROP(fmt, ##__VA_ARGS__)
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARN
#  define LOGW(fmt, ...) LOG_EMIT("W", fmt, ##__VA_ARGS__)
#else
#  define LOGW(fmt, ...) LOG_DROP(fmt, ##__VA_ARGS__)
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
#  define LOGE(fmt, ...) LOG_EMIT("E", fmt, ##__VA_ARGS__)
#else
#  define LOGE(fmt, ...) LOG_DROP(fmt, ##__VA_ARGS__)
#endif

#if LOG_LEVEL <= LOG_LEVEL_CRITICAL
#  define LOGC(fmt, ...) LOG_EMIT("C", fmt, ##__VA_ARGS__)
#else
#  define LOGC(fmt, ...) LOG_DROP(fmt, ##__VA_ARGS__)
#endif

// Maximum multi-touch fingers, fixed by AAP_TouchEvent's array sizes
// and aap_service's `< 10` validation at aap_service.c:15772.
constexpr int kMaxFingers = 10;

// Wire-format AAP_TouchEvent (0x88 / 136 bytes). Decoded by tracing
// AAPInputSource::onTouchEvent (aap_service.c:35057) and
// InputSource::populateTouchEvent (aap_service.c:205285).
struct AAP_TouchEvent {
    uint32_t device_type;            // +0x00, 1 = touchscreen
    uint32_t finger_count;           // +0x04, 1..10
    uint32_t x[kMaxFingers];         // +0x08, display pixels
    uint32_t y[kMaxFingers];         // +0x30, display pixels
    uint32_t pointer_id[kMaxFingers];// +0x58, evdev tracking IDs
    uint32_t action;                 // +0x80, 1..5
    uint32_t action_index;           // +0x84, compacted slot index
};
static_assert(sizeof(AAP_TouchEvent) == 0x88, "AAP_TouchEvent must be 0x88 bytes");

// Wire-format `action` values, mapped to Android MotionEvent
// semantics at aap_service.c:35099.
enum AAPTouchAction : uint32_t {
    kAAPTouchActionDown        = 1,  // first finger landed (n: 0 -> 1)
    kAAPTouchActionUp          = 2,  // last finger lifted (n: 1 -> 0)
    kAAPTouchActionMove        = 3,  // any finger moved
    kAAPTouchActionPointerDown = 4,  // additional finger landed (n: k -> k+1, k>=1)
    kAAPTouchActionPointerUp   = 5,  // one of several lifted (n: k -> k-1, k>=2)
};

// One physical contact's current state inside an evdev frame.
struct Finger {
    int      tracking_id;   // ABS_MT_TRACKING_ID; -1 = slot empty
    uint32_t x;             // ABS_MT_POSITION_X (device units)
    uint32_t y;             // ABS_MT_POSITION_Y (device units)
};

// Snapshot of all slots at one evdev SYN_REPORT.
struct MtState {
    Finger fingers[kMaxFingers];
    int    n_active;
};

// Defined in touch_reader.cpp.
void touch_start(void);
void touch_stop(void);

// Defined in touch_send.cpp; called by the reader thread on each
// SYN_REPORT with the freshly-built current state.
void touch_on_frame(const MtState &cur);

// Defined in nav.cpp. Called from the aap_create_session PLT shim
// (lifecycle.cpp) BEFORE chaining to the real SDK entry point, to
// substitute the (NULL on OEM) navigation callback slot in cb_list
// with one that dumps received 0x500 / 0x501 / 0x502 events to the
// LOGD sink. Safe to call with cb_list==NULL (becomes a no-op).
void nav_install_cb(void *cb_list);

// Defined in hud_send.cpp. HUD output lifecycle, called from the
// same PLT shims as touch_start/stop. hud_send_start opens the OEM
// HMI + Service D-Bus connections, spawns a dispatcher thread and
// a sender thread, and starts forwarding the per-event updates
// nav.cpp pushes via hud_send.h. hud_send_stop tears the threads
// down and releases the D-Bus connections. Both are idempotent.
void hud_send_start(void);
void hud_send_stop(void);

// Defined in main.cpp. Idempotent. Called from the first
// aap_create_session shim invocation to perform the deferred self-gate
// (probe blmjciaapa.so for the GetServiceInterfaces anchor and set
// g_enabled / g_oem_anchor_GetServiceInterfaces accordingly).
void oem_self_gate(void);

#endif // LIBPATCH_BLMJCIAAPA_PATCH_H
