// Action state machine + send path for the touch shim.
//
// Receives evdev MtState snapshots from touch_reader.cpp on every
// SYN_REPORT. Diffs against the previous snapshot to figure out
// which Android-MotionEvent-style events the phone should see, then
// builds an AAP_TouchEvent and hands it to RaceAap::SendTouchInput
// (resolved through the anchor-and-offset machinery in offsets.h).
//
// The OEM VideoManager::IsAAVideoInFocus gate is enforced before any
// send — this is the only behaviour we cherry-pick from
// HMIEventHandler::InputTouchScreen. SBN handling and contact-id
// dedup are deliberately skipped.

#include "../offsets.h"
#include "../patch.h"

#include <pthread.h>
#include <string.h>

namespace {

// ---------------------------------------------------------------
// Cached OEM function pointers — lazily resolved on first frame.
//
// We don't resolve in the constructor because the AapProc singleton
// may not be fully constructed yet at that point. The OEM has
// definitely finished constructing it by the time RaceAap::Init
// successfully returns from aap_create_session, which is when our
// touch_start() fires; by the time the first touch frame arrives,
// the singleton is safe to query.
// ---------------------------------------------------------------

typedef void *(*pfn_get_singleton)(void);
typedef void *(*pfn_get_member)(void *self);
typedef int   (*pfn_is_focus)(void *vm);
typedef int   (*pfn_send_touch)(void *race, AAP_TouchEvent *evt);

pfn_get_singleton p_GetInstance      = nullptr;
pfn_get_member    p_GetVideoManager  = nullptr;
pfn_get_member    p_GetRaceAap       = nullptr;
pfn_is_focus      p_IsAAVideoInFocus = nullptr;
pfn_send_touch    p_SendTouchInput   = nullptr;

bool g_symbols_resolved = false;

void resolve_touch_symbols_once(void)
{
    if (g_symbols_resolved) return;
    using namespace blm_offsets_FW_74_00_324A;

    p_GetInstance      = resolve_blm<void *(void)>     (Singleton_AapProc_GetInstance);
    p_GetVideoManager  = resolve_blm<void *(void *)>   (AapProc_GetVideoManager);
    p_GetRaceAap       = resolve_blm<void *(void *)>   (AapProc_GetRaceAap);
    p_IsAAVideoInFocus = resolve_blm<int  (void *)>    (VideoManager_IsAAVideoInFocus);
    p_SendTouchInput   = resolve_blm<int  (void *, AAP_TouchEvent *)>(RaceAap_SendTouchInput);

    g_symbols_resolved = true;
    LOGD("touch send: OEM symbols resolved "
              "(GetInstance=%p SendTouchInput=%p)",
              (void *)p_GetInstance, (void *)p_SendTouchInput);
}

// ---------------------------------------------------------------
// Coordinate scaling.
//
// AAP_TouchEvent expects display pixels (0..799 X, 0..479 Y).
// Whether /dev/input/filtered-touchscreen0 already reports pixels or
// reports raw 0..~4099 device units is an open empirical question.
// We adapt at runtime by inspecting the first non-trivial frame:
// if any value exceeds the display's pixel range, switch into
// "raw / 4099 * size" scaling for the rest of the process lifetime.
// ---------------------------------------------------------------

constexpr uint32_t kDisplayWidth  = 800;
constexpr uint32_t kDisplayHeight = 480;
constexpr uint32_t kRawMax        = 4099;

bool g_scaling_decided = false;
bool g_scale_from_raw  = false;  // true => values are 0..~4099, scale down

void decide_scaling(const MtState &s)
{
    if (g_scaling_decided) return;
    if (s.n_active == 0) return;  // wait for a real touch

    uint32_t max_x = 0, max_y = 0;
    for (int i = 0; i < kMaxFingers; ++i) {
        if (s.fingers[i].tracking_id < 0) continue;
        if (s.fingers[i].x > max_x) max_x = s.fingers[i].x;
        if (s.fingers[i].y > max_y) max_y = s.fingers[i].y;
    }

    g_scale_from_raw = (max_x >= kDisplayWidth) || (max_y >= kDisplayHeight);
    g_scaling_decided = true;
    LOGD("touch send: scaling mode = %s (first-frame max %u,%u)",
              g_scale_from_raw ? "raw->pixels" : "passthrough",
              max_x, max_y);
}

inline uint32_t scale_x(uint32_t v)
{
    if (!g_scale_from_raw) return v;
    uint64_t out = static_cast<uint64_t>(v) * kDisplayWidth / kRawMax;
    return static_cast<uint32_t>(out);
}

inline uint32_t scale_y(uint32_t v)
{
    if (!g_scale_from_raw) return v;
    uint64_t out = static_cast<uint64_t>(v) * kDisplayHeight / kRawMax;
    return static_cast<uint32_t>(out);
}

// ---------------------------------------------------------------
// AAP_TouchEvent build + send.
// ---------------------------------------------------------------

// Build an AAP_TouchEvent from a list of (slot_idx, finger) pairs.
// The output's pointer arrays are filled in input order; the caller
// decides slot ordering. action_index is the array position of the
// "interesting" finger (the one POINTER_UP/POINTER_DOWN refers to);
// pass 0 for DOWN / UP / MOVE.
struct SnapEntry {
    int    slot;            // for debug only
    Finger fg;
};

void send_event(const SnapEntry *entries, int n,
                AAPTouchAction action, int action_index)
{
    if (!g_enabled) return;
    if (!p_SendTouchInput) return;
    if (n <= 0 || n > kMaxFingers) return;

    // Respect the OEM video-focus gate — don't push touches into a
    // phone that isn't currently the foreground video sink.
    void *aap = p_GetInstance ? p_GetInstance() : nullptr;
    if (!aap) {
        LOGV("send_event: AapProc instance not available, skipping");
        return;
    }

    if (!p_IsAAVideoInFocus(p_GetVideoManager(aap))) {
        // Common when AAP is connected but the user is on the native
        // HMI / CarPlay; suppress quietly.
        LOGV("send_event: video not in focus, suppressing action=%u n=%d",
             static_cast<uint32_t>(action), n);
        return;
    }

    AAP_TouchEvent evt;
    memset(&evt, 0, sizeof(evt));
    evt.device_type  = 1;                              // touchscreen
    evt.finger_count = static_cast<uint32_t>(n);
    evt.action       = static_cast<uint32_t>(action);
    evt.action_index = static_cast<uint32_t>(action_index);

    for (int i = 0; i < n; ++i) {
        evt.x[i]          = scale_x(entries[i].fg.x);
        evt.y[i]          = scale_y(entries[i].fg.y);
        evt.pointer_id[i] = static_cast<uint32_t>(entries[i].fg.tracking_id);
    }

    // Verbose dump of the outgoing event — one line per finger so
    // the format string stays bounded.
    LOGV("send_event: action=%u n=%d action_index=%d",
         static_cast<uint32_t>(action), n, action_index);
    for (int i = 0; i < n; ++i) {
        LOGV("send_event:   [%d] slot=%d id=%u xy=(%u,%u)",
             i, entries[i].slot, evt.pointer_id[i], evt.x[i], evt.y[i]);
    }

    void *race = p_GetRaceAap(aap);
    if (!race) {
        LOGV("send_event: RaceAap not available, skipping");
        return;
    }
    int rc = p_SendTouchInput(race, &evt);
    LOGV("send_event: SendTouchInput -> %d", rc);
    if (rc != 0) {
        // 0x108 = AAP_SESSION_NOT_CONNECTED — expected briefly after
        //         create until the SDK reaches AAP_SESSION_STATE_CONNECTED
        //         (state value 0x302). Stay quiet for that one at
        //         error level; verbose log above still records it.
        if (rc != 0x108) {
            LOGE("SendTouchInput action=%u n=%d -> %d",
                      static_cast<uint32_t>(action), n, rc);
        }
    }
}

// ---------------------------------------------------------------
// Per-frame diff + dispatch.
// ---------------------------------------------------------------

// Snapshot persistence across calls. touch_on_frame() is called by
// the reader thread only — no need for atomics on the snapshot itself.
MtState g_prev;
bool    g_prev_init = false;

// Build a "survivors" snapshot: for every slot that exists in both
// prev and cur with the same tracking id, take the cur finger
// (current position). Output is sorted by slot index. Returns count.
int build_survivors(const MtState &prev, const MtState &cur,
                    SnapEntry *out)
{
    int n = 0;
    for (int slot = 0; slot < kMaxFingers; ++slot) {
        int pid = prev.fingers[slot].tracking_id;
        int cid = cur.fingers[slot].tracking_id;
        if (pid >= 0 && cid == pid) {
            out[n].slot = slot;
            out[n].fg   = cur.fingers[slot];
            ++n;
        }
    }
    return n;
}

// Insert a finger into a slot-sorted snapshot. Returns the array
// index where the new entry was placed.
int insert_sorted(SnapEntry *arr, int n, int slot, const Finger &fg)
{
    int pos = n;
    for (int i = 0; i < n; ++i) {
        if (arr[i].slot > slot) { pos = i; break; }
    }
    for (int i = n; i > pos; --i) arr[i] = arr[i - 1];
    arr[pos].slot = slot;
    arr[pos].fg   = fg;
    return pos;
}

} // namespace

void touch_on_frame(const MtState &cur)
{
    resolve_touch_symbols_once();
    decide_scaling(cur);

    if (!g_prev_init) {
        // One-shot init for g_prev. The "slot empty" sentinel is
        // tracking_id = -1, but `MtState g_prev;` at file scope is
        // zero-initialised (tracking_id = 0 in every slot), which
        // collides with valid kernel-assigned IDs. Without this
        // fix-up the very first diff would see "old phantom finger
        // id=0 vs new finger id=N" in every used slot and emit
        // spurious UP/POINTER_UP events for fingers that never
        // existed — corrupting single-touch's first DOWN and (worse)
        // breaking the first multi-touch sequence on each fresh
        // slot. AA would silently drop the phantom UPs (no prior
        // DOWN to match) but the wire stream is malformed and
        // gesture detectors can react unpredictably.
        for (int s = 0; s < kMaxFingers; ++s) {
            g_prev.fingers[s].tracking_id = -1;
            g_prev.fingers[s].x = 0;
            g_prev.fingers[s].y = 0;
        }
        g_prev.n_active = 0;
        g_prev_init = true;
        // Fall through into the normal diff path.
    }

    // Categorise per-slot transitions.
    int falling_slots[kMaxFingers]; int n_falling = 0;
    int rising_slots [kMaxFingers]; int n_rising  = 0;
    bool any_moved = false;

    for (int s = 0; s < kMaxFingers; ++s) {
        int pid = g_prev.fingers[s].tracking_id;
        int cid = cur     .fingers[s].tracking_id;

        if (pid < 0 && cid < 0) {
            continue;                       // empty -> empty
        } else if (pid < 0 && cid >= 0) {
            rising_slots[n_rising++] = s;   // new finger
        } else if (pid >= 0 && cid < 0) {
            falling_slots[n_falling++] = s; // finger released
        } else if (pid != cid) {
            // Different finger in same slot — treat as release-then-rise.
            falling_slots[n_falling++] = s;
            rising_slots [n_rising++]  = s;
        } else {
            // Same finger; possibly moved.
            if (g_prev.fingers[s].x != cur.fingers[s].x ||
                g_prev.fingers[s].y != cur.fingers[s].y) {
                any_moved = true;
            }
        }
    }

    // Survivors (slots in both prev and cur with same id) — snapshot
    // base used for all generated events.
    SnapEntry survivors[kMaxFingers];
    int n_survivors = build_survivors(g_prev, cur, survivors);

    LOGV("on_frame: falling=%d rising=%d moved=%d survivors=%d",
         n_falling, n_rising, any_moved ? 1 : 0, n_survivors);

    // ----- Step 1: POINTER_UP / UP for each falling slot ------------
    for (int i = 0; i < n_falling; ++i) {
        int slot = falling_slots[i];

        // Snapshot: survivors + this falling finger (at its PREV pos
        // with its PREV tracking_id), inserted in slot-sorted order.
        SnapEntry snap[kMaxFingers];
        memcpy(snap, survivors, sizeof(SnapEntry) * n_survivors);
        int n = n_survivors;
        int idx = insert_sorted(snap, n, slot, g_prev.fingers[slot]);
        ++n;

        AAPTouchAction act = (n == 1) ? kAAPTouchActionUp
                                      : kAAPTouchActionPointerUp;
        int action_index = (n == 1) ? 0 : idx;
        send_event(snap, n, act, action_index);
    }

    // ----- Step 2: POINTER_DOWN / DOWN for each rising slot ---------
    //
    // Incremental snapshot: start from survivors, add risen fingers
    // one at a time, emitting an event each time.
    SnapEntry inc[kMaxFingers];
    memcpy(inc, survivors, sizeof(SnapEntry) * n_survivors);
    int inc_n = n_survivors;

    for (int i = 0; i < n_rising; ++i) {
        int slot = rising_slots[i];
        int idx  = insert_sorted(inc, inc_n, slot, cur.fingers[slot]);
        ++inc_n;

        AAPTouchAction act = (inc_n == 1) ? kAAPTouchActionDown
                                          : kAAPTouchActionPointerDown;
        int action_index = (inc_n == 1) ? 0 : idx;
        send_event(inc, inc_n, act, action_index);
    }

    // ----- Step 3: MOVE if nothing else changed but positions did ---
    if (n_falling == 0 && n_rising == 0 && any_moved) {
        // All current fingers, in slot order.
        SnapEntry move_snap[kMaxFingers];
        int n = 0;
        for (int s = 0; s < kMaxFingers; ++s) {
            if (cur.fingers[s].tracking_id < 0) continue;
            move_snap[n].slot = s;
            move_snap[n].fg   = cur.fingers[s];
            ++n;
        }
        send_event(move_snap, n, kAAPTouchActionMove, 0);
    }

    g_prev = cur;
}
