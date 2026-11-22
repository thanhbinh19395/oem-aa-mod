// SPDX-License-Identifier: AGPL-3.0-or-later
//
// CarPlay video OCR patch for Mazda Connect (CMU150).
// Copyright (C) 2026 KID MIXER-MODER.
//
// Part of an AGPL-3.0 work - see NOTICE.md for full attribution.
//
// LD_PRELOAD into the `carplayd` process (NOT jciCARPLAY). carplayd owns
// the GStreamer pipeline that decodes the CarPlay H.264 plane onto the
// V4L overlay; that is the only place on the CMU where the VietMap
// speed-limit badge pixels are visible. This library:
//   * gates on /proc/self/cmdline containing "carplayd"
//   * reads libpatch-carplay.conf (shared with blmjcicarplay)
//   * interposes gst_element_factory_make / gst_element_link (gst_hook.cpp)
//   * template-matches the badge and writes /data_persist/splim for the
//     existing HUD reader in libpatch-blmjcicarplay.so

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "log.h"
#include "common/config.h"

#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/ucontext.h>
#include <unistd.h>

#include <cstdio>

std::FILE *g_logf = nullptr;

namespace {

pid_t gettid_compat(void)
{
#ifdef SYS_gettid
    return (pid_t)syscall(SYS_gettid);
#else
    return -1;
#endif
}

void fatal_signal_handler(int sig, siginfo_t *info, void *ucontext)
{
    static const char hdr[] =
        "\n[libpatch-carplayd][C] *** fatal signal caught ***\n";
    (void)!write(STDERR_FILENO, hdr, sizeof(hdr) - 1);

    char buf[256];
    int m = snprintf(buf, sizeof(buf),
                     "[libpatch-carplayd][C] signal=%d pid=%d tid=%d "
                     "si_code=%d si_addr=%p\n",
                     sig, (int)getpid(), (int)gettid_compat(),
                     info ? info->si_code : -1,
                     info ? info->si_addr : nullptr);
    if (m > 0) (void)!write(STDERR_FILENO, buf, (size_t)m);

    if (ucontext) {
        ucontext_t *uc = static_cast<ucontext_t *>(ucontext);
        const mcontext_t &mc = uc->uc_mcontext;
        m = snprintf(buf, sizeof(buf),
                     "[libpatch-carplayd][C] regs: pc=%08lx lr=%08lx "
                     "sp=%08lx\n",
                     (unsigned long)mc.arm_pc, (unsigned long)mc.arm_lr,
                     (unsigned long)mc.arm_sp);
        if (m > 0) (void)!write(STDERR_FILENO, buf, (size_t)m);
    }

    signal(sig, SIG_DFL);
    raise(sig);
}

void install_fatal_handler(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = fatal_signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO | SA_NODEFER;
    sigaction(SIGSEGV, &sa, nullptr);
    sigaction(SIGBUS,  &sa, nullptr);
    sigaction(SIGABRT, &sa, nullptr);
    sigaction(SIGFPE,  &sa, nullptr);
    sigaction(SIGILL,  &sa, nullptr);
}

int read_cmdline(char *out, size_t outsz)
{
    if (!out || outsz == 0) return 0;
    out[0] = '\0';
    int fd = open("/proc/self/cmdline", O_RDONLY | O_CLOEXEC);
    if (fd < 0) return 0;
    ssize_t n = read(fd, out, outsz - 1);
    close(fd);
    if (n <= 0) { out[0] = '\0'; return 0; }
    for (ssize_t i = 0; i < n; ++i) if (out[i] == '\0') out[i] = ' ';
    out[n] = '\0';
    return (int)n;
}

bool in_carplayd(void)
{
    char cmdline[512];
    read_cmdline(cmdline, sizeof(cmdline));
    // sm launches `/usr/bin/carplayd`. Also match bare `carplayd` in case
    // a wrapper rewrites argv0.
    return strstr(cmdline, "carplayd") != nullptr;
}

} // namespace

__attribute__((constructor))
static void on_load(void)
{
    if (!in_carplayd()) {
        // Inherited by sm's once-a-second killall probes, etc. Stay silent.
        return;
    }

    g_logf = std::fopen("/tmp/carplayd_ocr.log", "a");
    if (g_logf) std::setvbuf(g_logf, nullptr, _IOLBF, 0);
    install_fatal_handler();

    try {
        libpatch_config::load(reinterpret_cast<const void *>(&on_load),
                              libpatch_config::kCarplayConfigFile);
    } catch (...) {
        LOGE("config: load threw — keeping defaults");
    }

    LOGD("=== loaded into carplayd pid=%d ppid=%d ocr=%s force_linear=%s "
         "hz=%d ===",
         (int)getpid(), (int)getppid(),
         libpatch_config::carplay_ocr() ? "on" : "off",
         libpatch_config::carplay_ocr_force_linear() ? "on" : "off",
         libpatch_config::carplay_ocr_hz());
}
