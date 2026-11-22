// SPDX-License-Identifier: AGPL-3.0-or-later
//
// carplayd OCR patch logging shim.
//
// carplayd's stderr is usable over SSH, but we also mirror to a bring-up
// file so a field session can `tail -f` without fighting syslog-ng.

#ifndef LIBPATCH_CARPLAYD_LOG_H
#define LIBPATCH_CARPLAYD_LOG_H

#include <cstdio>

extern std::FILE *g_logf;

#define LIBPATCH_NAME     "carplayd"
#define LIBPATCH_LOG_SINK (g_logf ? g_logf : stderr)
#include "common/log.h"

#endif // LIBPATCH_CARPLAYD_LOG_H
