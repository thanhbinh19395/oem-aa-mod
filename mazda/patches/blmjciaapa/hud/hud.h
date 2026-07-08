// HUD hooks used by lifecycle.cpp around aap_create_session.
// The per-event producer interface and sender lifecycle live in
// the selected transport header (vbs_tx.h / svcnavi_tx.h).

#ifndef LIBPATCH_BLMJCIAAPA_HUD_HUD_H
#define LIBPATCH_BLMJCIAAPA_HUD_HUD_H

#include <stdint.h>

// Defined in hud.cpp. Called from the aap_create_session PLT shim
// BEFORE chaining to the real SDK entry point, to substitute the
// (NULL on OEM) navigation callback slot in cb_list with one that
// dumps received 0x500 / 0x501 / 0x502 events to the LOGD sink.
// Safe to call with cb_list==NULL (becomes a no-op).
void hud_pre_aap_create_session(void *cb_list);

// Defined in hud.cpp. Called after a successful real
// aap_create_session to start the HUD sender side.
void hud_post_aap_create_session(void);

// Defined in hud.cpp. Called before the real aap_destroy_session to
// stop the HUD sender side.
void hud_pre_aap_destroy_session(void);

// === Android Auto GAL 1.6 path ================================
//
// The aap_service shim (a SEPARATE process) swallows the 1.6 navigation frames
// and relays them RAW over an abstract AF_UNIX datagram socket. The receiver
// thread (nav16_rx.cpp) hands each raw frame to hud_feed_nav16_raw(), which is
// the single place that decodes the AA protocol, maps it to the Mazda HUD
// domain, and renders it through the SAME transport (glyph emit, street fold,
// lanes) the 1.5 path uses.

// Defined in hud.cpp. Decode + render one raw GAL 1.6 nav frame (full frame,
// leading 2-byte big-endian msgId). Runs on the nav16 receiver thread.
void hud_feed_nav16_raw(const uint8_t *frame, int len);

// Defined in hud.cpp. Reset hud_feed_nav16_raw's accumulator + change-gate.
// Called from hud_nav16_rx_start() while the receiver thread is NOT running
// (it is the only other toucher of that state), so a session that ended
// abnormally — cable pull, no INACTIVE/STOP received — can't leave a stale
// gate that suppresses the next session's first frame.
void hud_feed_nav16_reset(void);

// Defined in nav16_rx.cpp. Start/stop the receiver thread + socket. Started
// from hud_post_aap_create_session when use_protocol_v1_6 is set, stopped from
// hud_pre_aap_destroy_session. Both idempotent.
void hud_nav16_rx_start(void);
void hud_nav16_rx_stop(void);

// Defined in nav16_rx.cpp. TRUE once a validated 1.6 frame has arrived on the
// rx socket this session (reset by hud_nav16_rx_start). our_nav_cb keys the
// legacy-1.5-callback drop on THIS — evidence the 1.6 chain really took — not
// on the config flag, so a 1.6 setup that didn't take (aap_service shim not
// preloaded / byte-verify aborted / phone declined the advertisement) falls
// back to the stock 1.5 path instead of leaving the HUD dark.
bool hud_nav16_rx_seen(void);

#endif // LIBPATCH_BLMJCIAAPA_HUD_HUD_H
