// HUD hooks used by lifecycle.cpp around aap_create_session.
// The per-event producer interface and sender lifecycle live in
// hud_send.h.

#ifndef LIBPATCH_BLMJCIAAPA_HUD_HUD_H
#define LIBPATCH_BLMJCIAAPA_HUD_HUD_H

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

#endif // LIBPATCH_BLMJCIAAPA_HUD_HUD_H
