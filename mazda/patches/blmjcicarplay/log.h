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

#include <atomic>
#include <cstdio>
#include <unistd.h>

// Bring-up log sink. jciCARPLAY's stderr is swallowed by syslog-ng (routed to
// console / dropped, not a file we can read over SSH), so main.cpp opens this
// file in /tmp on load and every LOGx goes there. NULL (before it is opened,
// or in a non-launcher PID) falls back to stderr.
// Cap /tmp/carplay_bridge.log at 256 KiB so tmpfs RAM can never grow unboundedly.
// Sample ftell() only once every 64 writes under flockfile() to avoid per-line
// lseek syscalls and cross-thread races with fprintf().
extern std::FILE *g_logf;

inline std::FILE *carplay_bounded_log_sink(void)
{
    std::FILE *f = g_logf;
    if (!f) return stderr;
    static std::atomic<uint32_t> s_writes{0};
    if (((s_writes.fetch_add(1u, std::memory_order_relaxed) + 1u) & 63u) == 0u) {
        flockfile(f);
        const long pos = std::ftell(f);
        if (pos > (256L * 1024L)) {
            const int fd = fileno(f);
            if (fd >= 0) (void)!ftruncate(fd, 0);
            std::rewind(f);
        }
        funlockfile(f);
    }
    return f;
}

#define LIBPATCH_NAME     "blmjcicarplay"
#define LIBPATCH_LOG_SINK (carplay_bounded_log_sink())
#include "common/log.h"

#endif // LIBPATCH_BLMJCICARPLAY_LOG_H
