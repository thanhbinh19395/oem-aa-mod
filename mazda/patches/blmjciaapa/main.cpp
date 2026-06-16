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
// inert. The process gate + fatal-signal handler are shared across
// patches in common/preload_guard.h.

#include "log.h"
#include "common/preload_guard.h"

#include <unistd.h>

namespace {

__attribute__((constructor))
void on_load()
{
    // Gate the constructor body on argv[0] so the CMU's
    // once-a-second `sh -c killall ...` watchdog probe (which also
    // inherits LD_PRELOAD from the launching shell) doesn't spam
    // logs or install signal handlers in throwaway children. Silent
    // no-op otherwise: the child is about to execve() into killall
    // anyway, which wipes handlers regardless.
    char cmdline[256];
    preload_read_cmdline(cmdline, sizeof(cmdline));
    if (!preload_is_launcher_process(cmdline)) {
        return;
    }

    LOGD("loading (target FW 74.00.324A NA, blmjciaapa.so touch shim) "
         "pid=%d ppid=%d cmdline=[%s]",
         (int)getpid(), (int)getppid(), cmdline);
    preload_install_fatal_handler();
    LOGD("fatal-signal handler installed (SEGV/BUS/ABRT/FPE/ILL) pid=%d",
         (int)getpid());
    LOGD("self-gate deferred to first aap_create_session call pid=%d",
         (int)getpid());
}

} // namespace
