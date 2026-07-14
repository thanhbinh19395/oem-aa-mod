// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Session-lifecycle entry points for the mute -> phone media bridge.

#ifndef LIBPATCH_BLMJCIAAPA_MUTE_MUTE_H
#define LIBPATCH_BLMJCIAAPA_MUTE_MUTE_H

// Defined in mute.cpp. Spawns / joins the com.jci.vbs.am mute-watcher
// thread that pauses the phone's Android Auto media on the user mute
// button and resumes it on unmute. Bracketed by aap_create_session /
// aap_destroy_session exactly like the touch reader and HUD sender.
void mute_post_aap_create_session(void);
void mute_pre_aap_destroy_session(void);

#endif // LIBPATCH_BLMJCIAAPA_MUTE_MUTE_H
