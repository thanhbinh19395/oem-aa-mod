// SPDX-License-Identifier: AGPL-3.0-or-later
//
// CarPlay -> HUD bridge for Mazda Connect (CMU150).
// Copyright (C) 2026 KID MIXER-MODER.
//
// Part of an AGPL-3.0 work - see NOTICE.md for full attribution.
//
// blmjcicarplay logging shim — sets the library name + output sink and pulls
// in the shared logging helper (patches/common/log.h), exactly like the other
// patches' local log.h. Per-TU includes stay `#include "../log.h"` /
// `#include "log.h"`; set the subsystem tag per-TU via `#define LOG_TAG`
// before the include.

#ifndef LIBPATCH_BLMJCICARPLAY_LOG_H
#define LIBPATCH_BLMJCICARPLAY_LOG_H

#include <cstdio>

// Bring-up log sink. jciCARPLAY's stderr is swallowed by syslog-ng (routed to
// console / dropped, not a file we can read over SSH), so main.cpp opens this
// file in /tmp on load and every LOGx goes there. NULL (before it is opened,
// or in a non-launcher PID) falls back to stderr.
extern std::FILE *g_logf;

#define LIBPATCH_NAME     "blmjcicarplay"
#define LIBPATCH_LOG_SINK (g_logf ? g_logf : stderr)
#include "common/log.h"

#endif // LIBPATCH_BLMJCICARPLAY_LOG_H
