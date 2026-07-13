// SPDX-License-Identifier: AGPL-3.0-or-later
//
// CarPlay -> HUD bridge for Mazda Connect (CMU150).
// Copyright (C) 2026 KID MIXER-MODER.
//
// Part of an AGPL-3.0 work - see NOTICE.md for full attribution.
//
// Shared globals, logging, and cross-TU decls for the CarPlay->HUD bridge.
//
// This patch is the CarPlay analogue of patches/blmjciaapa: instead of tapping
// Android Auto's aap_create_session callback table, it taps the iAP2 device-
// manager client API (libdevmgr_interface.so) that the jciCARPLAY service
// (blmjcicarplay.so) already links and uses. We:
//   1. PLT-interpose eDevMgrRegisterProcess  -> capture the process handle +
//      the callback table, and overwrite the (OEM-NULL) nav-route-guidance
//      callback slot with our own.
//   2. PLT-interpose eDevMgrOpenDevice       -> capture the connected iAP2
//      device handle, then call eDevMgriPodStartNaviRouteGuidenceUpdates so
//      iOS actually starts streaming maneuver/guidance updates.
//   3. our nav callback decodes the maneuver/guidance struct and pushes it to
//      the SAME HUD sender (hud/hud_send.cpp) reused verbatim from blmjciaapa.

#ifndef LIBPATCH_BLMJCICARPLAY_PATCH_H
#define LIBPATCH_BLMJCICARPLAY_PATCH_H

#include <stdint.h>
#include <cstdio>

// Logging (LOGV..LOGC + LOG_LEVEL_*). Shared helper from patches/common,
// routed to this patch's /tmp file sink — see log.h. TUs that want a
// subsystem tag define LOG_TAG before including patch.h (or log.h directly).
#include "log.h"

// Master enable. Set by main.cpp once the self-gate confirms we are in the
// {L_jciCARPLAY} launcher PID.
extern bool g_enabled;

// ---- nav message decoder (nav.cpp) -----------------------------------------
// Called from the msgrcv tap for every devmgr message. Decodes iAP2 nav
// maneuver messages (mtype 0x8059) and drives the HUD sender. Passive: only
// reads the already-received buffer.
extern "C" void nav_on_devmgr_msg(const void *msgp, long n);

// ---- HUD output lifecycle (hud/hud_send.cpp, reused verbatim) ---------------
void hud_send_start(void);
void hud_send_stop(void);
// [NAV-END] request an immediate HUD wipe (called from the OEM CarPlay TBT-status cb thread).
void hud_request_clear(void);
// [NAV-END] FULL wipe incl the speed-limit sign — for CarPlay session gone / phone unplugged
// (cp_deactive_cb), like the AA mod's session-teardown clear. hud_request_clear() keeps the sign.
void hud_request_fullclear(void);

// ---- self-gate (main.cpp) --------------------------------------------------
// True when /proc/self/cmdline shows we are the jciCARPLAY launcher.
bool in_carplay_launcher(void);

#endif // LIBPATCH_BLMJCICARPLAY_PATCH_H
