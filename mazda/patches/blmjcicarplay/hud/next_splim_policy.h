// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Next-speed-limit 10-second HUD alert policy + deferred navigation queue
// for Mazda Connect CarPlay HUD (blmjcicarplay).
//
// Design:
//   - Header-only, zero dynamic allocation, C++11 / GCC 4.9.1 compatible.
//   - When VietMap announces an upcoming different speed limit ("Tốc độ giới hạn
//     tiếp theo X"), this state machine holds the HUD for exactly `kHoldMs` (10 s)
//     showing speed = X and road_name = "Tốc độ tiếp theo".
//   - Keyed strictly on monotonic event `alert_id`: repeated serial frames with
//     the same `alert_id` update distance only and NEVER extend the 10 s window.
//   - Any CarPlay iAP2 or VietMap BLE navigation updates arriving during the 10 s
//     window are recorded as queued (latest-wins) without interrupting the alert.
//     In-app nav-off (`on_nav_clear`) is also deferred until the 10 s ends.
//   - Only full session teardown (`on_full_clear`) aborts an active alert immediately.
//   - The exact instant `now_ms >= until_ms`, `tick()` returns `just_ended = true`
//     so the caller flushes the queued navigation state (or deferred clear) cleanly.

#ifndef LIBPATCH_BLMJCICARPLAY_NEXT_SPLIM_POLICY_H
#define LIBPATCH_BLMJCICARPLAY_NEXT_SPLIM_POLICY_H

#include <stdint.h>

namespace hud_next_splim {

static const int64_t  kHoldMs         = 10000LL;
static const char     kAlertLabel[]   = "T\xE1\xBB\x91" "c \xC4\x91\xE1\xBB\x99 ti\xE1\xBA\xBFp theo"; // "Tốc độ tiếp theo" UTF-8
static const char     kOverspeedLabel[] = "QU\xC3\x81 T\xE1\xBB\x90" "C"; // "QUÁ TỐC" UTF-8

struct IncomingAlert {
    uint32_t id     = 0;  // Monotonic event ID (>= 1)
    uint16_t limit  = 0;  // Upcoming speed limit in km/h (5..200)
    uint32_t dist_m = 0;  // Distance to sign in meters (0 = unspecified)
};

struct StepResult {
    bool active       = false; // True while 10 s alert owns the HUD
    bool changed      = false; // Alert started, changed limit/distance -> push HUD frame
    bool just_ended   = false; // 10 s expired on this step -> flush queued nav/clear
    bool run_deferred_clear = false; // Deferred in-app nav-off should run now
};

class AlertController {
public:
    void reset() {
        last_seen_id_     = 0;
        active_           = false;
        limit_            = 0;
        dist_m_           = 0;
        until_ms_         = 0;
        queued_nav_       = false;
        queued_nav_clear_ = false;
    }

    // CarPlay session ended / phone unplugged -> abort alert immediately.
    void on_full_clear() {
        reset();
    }

    // In-app navigation stopped while CarPlay stays connected.
    // Returns true if the caller should execute the clear immediately, or false
    // if it has been queued until the 10 s speed-limit alert finishes.
    bool on_nav_clear() {
        if (!active_) {
            queued_nav_clear_ = false;
            return true;
        }
        queued_nav_clear_ = true;
        queued_nav_       = false;
        return false;
    }

    // A new navigation update arrived (CarPlay iAP2 or VietMap BLE) while alert
    // may be active. Queues it (latest-wins) and cancels any earlier deferred clear.
    void on_nav_update() {
        if (active_) {
            queued_nav_       = true;
            queued_nav_clear_ = false;
        }
    }

    // Advance the alert state machine at monotonic timestamp `now_ms`.
    // Pass `incoming = nullptr` if no fresh `/tmp/vmnextsplim` record is present.
    StepResult step(int64_t now_ms, const IncomingAlert *incoming, bool cp_guiding) {
        StepResult r;

        // 1. Check for a newly arrived alert event or distance refresh.
        if (incoming != nullptr &&
            incoming->id != 0 &&
            incoming->limit >= 5 && incoming->limit <= 200) {
            if (incoming->id != last_seen_id_) {
                last_seen_id_ = incoming->id;
                active_       = true;
                limit_        = incoming->limit;
                dist_m_       = incoming->dist_m;
                until_ms_     = now_ms + kHoldMs;
                r.changed     = true;
            } else if (active_ && incoming->dist_m != dist_m_) {
                dist_m_   = incoming->dist_m;
                r.changed = true;
            }
        }

        // 2. Check 10-second monotonic expiry (immune to wall-clock jumps).
        if (active_ && now_ms >= until_ms_) {
            active_      = false;
            r.just_ended = true;
            // Only run a deferred nav-off clear if guidance did not resume in the meantime.
            r.run_deferred_clear = (queued_nav_clear_ && !cp_guiding);
            queued_nav_clear_    = false;
            queued_nav_          = false;
        }

        r.active = active_;
        return r;
    }

    bool     active() const { return active_; }
    uint16_t limit()  const { return limit_; }
    uint32_t dist_m() const { return dist_m_; }
    bool     has_queued_nav() const { return queued_nav_; }

private:
    uint32_t last_seen_id_     = 0;
    bool     active_           = false;
    uint16_t limit_            = 0;
    uint32_t dist_m_           = 0;
    int64_t  until_ms_         = 0;
    bool     queued_nav_       = false;
    bool     queued_nav_clear_ = false;
};

}  // namespace hud_next_splim

#endif  // LIBPATCH_BLMJCICARPLAY_NEXT_SPLIM_POLICY_H
