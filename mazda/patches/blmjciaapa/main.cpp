// Patch entry point. Runs once at LD_PRELOAD time (after the dynamic
// linker has mapped this library, before main() / before
// sm_svclauncher dlopen's blmjciaapa.so).
//
// IMPORTANT ordering: at constructor time blmjciaapa.so has NOT yet
// been dlopen'd, so we cannot probe for OEM symbols here. sm_svclauncher
// uses dlopen(name, RTLD_NOW) — *without* RTLD_GLOBAL — so even after
// it loads blmjciaapa.so the symbols stay in a local scope that
// dlsym(RTLD_DEFAULT, ...) cannot see.
//
// We therefore defer the self-gate to the first invocation of our
// aap_create_session PLT shim (lifecycle.cpp), which fires from
// RaceAap::Init *after* blmjciaapa.so is fully loaded. The gate uses
// dlopen("/jci/aapa/blmjciaapa.so", RTLD_NOW | RTLD_NOLOAD) to confirm
// the BLM is mapped without re-loading it. The handle reference is held
// forever (one extra refcount on a library that's already permanently
// mapped). GetServiceInterfaces resolution happens later in
// oem/blmjciaapa.cpp.
//
// The constructor just logs that we were loaded. If we ever land in
// a process that never calls aap_create_session, the library stays
// inert.

#include "log.h"

#include <execinfo.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/ucontext.h>
#include <unistd.h>

namespace {

// gettid() isn't in glibc 2.20 (the device's ancient libc). Fall
// back to syscall(SYS_gettid) — async-signal-safe via the raw
// syscall path.
pid_t gettid_compat(void)
{
#ifdef SYS_gettid
    return (pid_t)syscall(SYS_gettid);
#else
    return -1;
#endif
}

// -------------------------------------------------------------------
// Fatal-signal handler.
//
// Catches SIGSEGV / SIGBUS / SIGABRT / SIGFPE / SIGILL.
//
// On ARM, backtrace() called from a synchronous signal handler can't
// unwind past the kernel-provided __default_sa_restorer_v2
// trampoline (no .ARM.exidx for it), so we'd only see the handler
// itself in the stack. The actual faulting PC, LR, SP and the
// dereferenced address are recoverable from siginfo_t /
// ucontext_t — so we use SA_SIGINFO and print those raw. They're
// sufficient to identify the failing library + offset via
//   addr2line / readelf / `info symbol <addr>` in cross-gdb,
// or by comparing to the known library load bases logged earlier.
//
// All calls used here (write, snprintf into a stack buffer, signal,
// raise, getpid, syscall) are async-signal-safe per POSIX. We
// deliberately avoid LOGx (fprintf+fflush) and dladdr (not safe).
//
// After dumping diagnostics we restore the default disposition and
// re-raise so the kernel still writes a core dump.
// -------------------------------------------------------------------

void fatal_signal_handler(int sig, siginfo_t *info, void *ucontext)
{
    static const char hdr[] =
        "\n[libpatch-blmjciaapa][C] *** fatal signal caught ***\n";
    (void)!write(STDERR_FILENO, hdr, sizeof(hdr) - 1);

    char buf[256];
    int  m;

    m = snprintf(buf, sizeof(buf),
                 "[libpatch-blmjciaapa][C] signal=%d pid=%d tid=%d "
                 "si_code=%d si_addr=%p\n",
                 sig, (int)getpid(), (int)gettid_compat(),
                 info ? info->si_code : -1,
                 info ? info->si_addr : nullptr);
    if (m > 0) (void)!write(STDERR_FILENO, buf, (size_t)m);

    // Pull the saved register state out of the ucontext. On
    // Linux/ARM (32-bit EABI) it lives in uc_mcontext.arm_* fields.
    // glibc's <sys/ucontext.h> defines ucontext_t with mcontext_t
    // matching the kernel sigcontext layout.
    if (ucontext) {
        ucontext_t *uc = static_cast<ucontext_t *>(ucontext);
        const mcontext_t &mc = uc->uc_mcontext;
        m = snprintf(buf, sizeof(buf),
                     "[libpatch-blmjciaapa][C] regs: pc=%08lx lr=%08lx "
                     "sp=%08lx fp=%08lx ip=%08lx cpsr=%08lx fault=%08lx\n",
                     (unsigned long)mc.arm_pc,
                     (unsigned long)mc.arm_lr,
                     (unsigned long)mc.arm_sp,
                     (unsigned long)mc.arm_fp,
                     (unsigned long)mc.arm_ip,
                     (unsigned long)mc.arm_cpsr,
                     (unsigned long)mc.fault_address);
        if (m > 0) (void)!write(STDERR_FILENO, buf, (size_t)m);

        m = snprintf(buf, sizeof(buf),
                     "[libpatch-blmjciaapa][C] regs: r0=%08lx r1=%08lx "
                     "r2=%08lx r3=%08lx r4=%08lx r5=%08lx r6=%08lx r7=%08lx\n",
                     (unsigned long)mc.arm_r0,
                     (unsigned long)mc.arm_r1,
                     (unsigned long)mc.arm_r2,
                     (unsigned long)mc.arm_r3,
                     (unsigned long)mc.arm_r4,
                     (unsigned long)mc.arm_r5,
                     (unsigned long)mc.arm_r6,
                     (unsigned long)mc.arm_r7);
        if (m > 0) (void)!write(STDERR_FILENO, buf, (size_t)m);
    }

    // backtrace() will likely only show this handler + the kernel
    // sigreturn trampoline (no unwind info past the signal frame on
    // ARM), but include it anyway in case any frames are recoverable.
    static const char btm[] =
        "[libpatch-blmjciaapa][C] backtrace (may be truncated at sigreturn):\n";
    (void)!write(STDERR_FILENO, btm, sizeof(btm) - 1);
    void *frames[64];
    int   n = backtrace(frames, 64);
    backtrace_symbols_fd(frames, n, STDERR_FILENO);

    // Dump /proc/self/maps so the pc/lr/fault addresses can be
    // resolved to a (library, file_offset) pair without needing to
    // also collect maps from the running process separately. Bounded
    // copy through a stack buffer to stay async-signal-safe.
    static const char maps_hdr[] =
        "[libpatch-blmjciaapa][C] /proc/self/maps:\n";
    (void)!write(STDERR_FILENO, maps_hdr, sizeof(maps_hdr) - 1);
    int mfd = open("/proc/self/maps", O_RDONLY | O_CLOEXEC);
    if (mfd >= 0) {
        char mbuf[4096];
        ssize_t rn;
        while ((rn = read(mfd, mbuf, sizeof(mbuf))) > 0) {
            ssize_t off = 0;
            while (off < rn) {
                ssize_t wn = write(STDERR_FILENO, mbuf + off, (size_t)(rn - off));
                if (wn <= 0) break;
                off += wn;
            }
        }
        close(mfd);
    }

    // Restore default disposition and re-raise so the kernel still
    // writes a core dump that can be cross-gdb'd later.
    signal(sig, SIG_DFL);
    raise(sig);
}

void install_fatal_handler(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = fatal_signal_handler;
    sigemptyset(&sa.sa_mask);
    // SA_SIGINFO: deliver siginfo_t + ucontext_t to the 3-arg handler.
    // SA_NODEFER: allow the re-raise inside the handler to deliver
    // immediately rather than be held pending.
    sa.sa_flags = SA_SIGINFO | SA_NODEFER;
    sigaction(SIGSEGV, &sa, nullptr);
    sigaction(SIGBUS,  &sa, nullptr);
    sigaction(SIGABRT, &sa, nullptr);
    sigaction(SIGFPE,  &sa, nullptr);
    sigaction(SIGILL,  &sa, nullptr);
}

// Read /proc/self/cmdline, replacing NUL separators with spaces so
// it's printable. Returns the number of bytes written (excluding the
// trailing NUL). Async-signal-safe: only uses open/read/close.
static int read_cmdline(char *out, size_t outsz)
{
    if (!out || outsz == 0) return 0;
    out[0] = '\0';
    int fd = open("/proc/self/cmdline", O_RDONLY | O_CLOEXEC);
    if (fd < 0) return 0;
    ssize_t n = read(fd, out, outsz - 1);
    close(fd);
    if (n <= 0) { out[0] = '\0'; return 0; }
    for (ssize_t i = 0; i < n; ++i) {
        if (out[i] == '\0') out[i] = ' ';
    }
    out[n] = '\0';
    // Trim trailing space from the final NUL→space conversion.
    if (n > 0 && out[n - 1] == ' ') out[n - 1] = '\0';
    return (int)n;
}

// Return true if `cmdline` (space-joined /proc/self/cmdline) looks
// like a process that could plausibly dlopen blmjciaapa.so — i.e.
// argv[0]'s basename is "sm_svclauncher". The CMU runs a watchdog
// shell loop (`sh -c killall -s 0 jci-linux_imx6_volans-release`)
// roughly once per second; with LD_PRELOAD set in the launching
// shell those short-lived children all inherit the preload and
// would otherwise log + install signal handlers for nothing.
static bool cmdline_is_target_process(const char *cmdline)
{
    if (!cmdline || !*cmdline) return false;
    // argv[0] = chars up to the first space (we already replaced
    // NULs with spaces in read_cmdline).
    const char *end = strchr(cmdline, ' ');
    size_t      len = end ? (size_t)(end - cmdline) : strlen(cmdline);
    // Compare basename of argv[0] to "sm_svclauncher".
    const char *base = cmdline;
    for (size_t i = 0; i < len; ++i) {
        if (cmdline[i] == '/') base = cmdline + i + 1;
    }
    size_t blen = len - (size_t)(base - cmdline);
    static const char kTarget[] = "sm_svclauncher";
    const size_t      kTlen     = sizeof(kTarget) - 1;
    return blen == kTlen && memcmp(base, kTarget, kTlen) == 0;
}

__attribute__((constructor))
void on_load()
{
    // Gate the constructor body on argv[0] so the CMU's
    // once-a-second `sh -c killall ...` watchdog probe (which also
    // inherits LD_PRELOAD from the launching shell) doesn't spam
    // logs or install signal handlers in throwaway children.
    char cmdline[256];
    read_cmdline(cmdline, sizeof(cmdline));
    if (!cmdline_is_target_process(cmdline)) {
        // Silent no-op. Returning before installing the fatal
        // handler is deliberate: the child process is about to
        // execve() into killall (or similar) anyway, which will
        // wipe handlers regardless — installing them here just
        // costs syscalls.
        return;
    }

    LOGD("loading (target FW 74.00.324A NA, blmjciaapa.so touch shim) "
         "pid=%d ppid=%d cmdline=[%s]",
         (int)getpid(), (int)getppid(), cmdline);
    install_fatal_handler();
    LOGD("fatal-signal handler installed (SEGV/BUS/ABRT/FPE/ILL) pid=%d",
         (int)getpid());
    LOGD("self-gate deferred to first aap_create_session call pid=%d",
         (int)getpid());
}

} // namespace

