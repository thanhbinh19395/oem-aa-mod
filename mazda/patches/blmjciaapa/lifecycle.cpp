// PLT shadows for aap_create_session / aap_destroy_session.
//
// Both symbols are dynamically exported by /usr/lib/libaap_interface.so
// (verified by `nm -D`). The OEM caller is blmjciaapa.so's
// RaceAap::Init / RaceAap::UnInit. Because our library is
// LD_PRELOAD'ed into the {L_jciAAPA} launcher PID before
// blmjciaapa.so is dlopen'd, the dynamic linker binds OEM PLT lookups
// of these names to OUR exported symbols.
//
// We use them as a pure lifecycle bracket for every per-session
// shim thread (currently: the touch reader and the HUD sender):
// session up   -> install nav cb, start the touch reader + HUD sender
// session down -> stop the HUD sender + touch reader, chain through
//
// These are the only cleanly preload-shadowable lifecycle anchors.
// Resolving HMIEventHandler::Init/UnInit via dlsym(RTLD_NEXT) does NOT work
// — those symbols are LOCAL in blmjciaapa.so's static symtab, and
// intra-library OEM callers don't go through the PLT anyway.

#include "patch.h"

#include <dlfcn.h>
#include <pthread.h>
#include <string.h>

namespace {

typedef int (*pfn_aap_create_session)(const char *cfg, void *unknown_r1,
                                      void *cb_list, void **out_handle);
typedef int (*pfn_aap_destroy_session)(void *handle);

pfn_aap_create_session  g_real_create  = nullptr;
pfn_aap_destroy_session g_real_destroy = nullptr;

pthread_mutex_t g_session_mu = PTHREAD_MUTEX_INITIALIZER;
void           *g_session    = nullptr;   // last successfully-created handle
bool            g_session_up = false;     // per-session shim threads (touch + HUD) alive

// Resolve a single symbol against the real libaap_interface.so.
//
// dlsym(RTLD_NEXT, ...) is the textbook way to do this, BUT it only
// searches the global scope past our own image. sm_svclauncher
// dlopens blmjciaapa.so without RTLD_GLOBAL, and libaap_interface.so
// is pulled in as a DT_NEEDED dependency of blmjciaapa.so — meaning
// it lands in blmjciaapa.so's *local* scope only. RTLD_NEXT therefore
// returns NULL here, even though the library is mapped into the
// process. (Same trap as documented for HMIEventHandler::Init in the
// file-header comment above.)
//
// Workaround: get a handle to libaap_interface.so via dlopen with
// RTLD_NOLOAD — that consults the loader's global list of mapped
// objects regardless of scope, so it finds the already-loaded
// instance and bumps its refcount by one. dlsym on that handle then
// returns the real address.
//
// We still try RTLD_NEXT first so that in any environment where the
// library *does* happen to be globally visible (e.g. a future host
// process that LD_PRELOADs both libs, or a test harness) we don't
// take an unnecessary refcount.
void *g_libaap_handle = nullptr;

// Forward declarations of our own exported PLT shadows so we can
// take their addresses to detect self-loops in resolve_oem_symbol.
// Defined further down in this file with PRELOAD_EXPORT.
extern "C" int aap_create_session(const char *, void *, void *, void **);
extern "C" int aap_destroy_session(void *);

// IMPORTANT: the OEM's libaap_interface.so `aap_create_session`
// takes FOUR register arguments, not three.
//
// The Ghidra decompilation of libaap_interface.so shows only three
// parameters because Ghidra dropped the always-NULL r1 argument.
// The OEM caller in blmjciaapa.so's RaceAap::Init at
// blmjciaapa.so:0x8c7a0..0x8c7b4 (verified with `objdump -d`)
// actually sets up:
//
//   r0 = cfg            (config_file_name string)
//   r1 = 0              (always NULL in shipped builds)
//   r2 = &local_cb_list (88-byte stack struct of callbacks)
//   r3 = this+4         (where the session handle should be written)
//
// The callee:
//   * uses r2 as cb_list  (reads function pointers at offsets 4/8/c/10)
//   * uses r3 as out_handle (writes the malloc'd session struct ptr there)
//
// If we declare the function pointer with 3 args:
//   - the compiler emits `blx r3` with r3=&aap_create_session, so the
//     callee writes the session handle to its own .text (read-only) page
//     and we SIGSEGV.
// If we declare with 3 args + dummy `int*` for r3:
//   - the crash goes away, but the OEM writes the handle to our local
//     dummy, NOT to blmjciaapa.so's `this+4` slot, so subsequent SDK
//     calls see handle=NULL and the SDK errors with
//     "ERR::AAP_SDK_IFACE::3262::Invalid handle 0, actual handle X".
//
// The correct shape is 4 args; we just forward `unknown_r1` verbatim
// from caller to callee so the OEM's NULL stays NULL and the
// real `out_handle` (= this+4) gets the session pointer it expects.

// Returns true if `addr` is one of our exported PLT shadows. If
// dlsym ever returns this, calling it would recurse straight back
// into us (because LD_PRELOAD'd libs sit at the front of every
// scope, including the one a regular dlopen-handle dlsym searches),
// stack-overflow, SIGSEGV.
bool addr_is_own_shim(void *addr)
{
    return addr == reinterpret_cast<void *>(&aap_create_session) ||
           addr == reinterpret_cast<void *>(&aap_destroy_session);
}

void *resolve_oem_symbol(const char *name)
{
    // 1) RTLD_NEXT path. Only walks the *global* scope past our
    // image. libaap_interface.so isn't globally scoped in this
    // process, so this is expected to return NULL — but if it does
    // return a real address, that's the cleanest answer (no extra
    // refcount on libaap_interface.so).
    void *p = dlsym(RTLD_NEXT, name);
    LOGD("resolve_oem_symbol(%s): dlsym(RTLD_NEXT) -> %p", name, p);
    if (p && !addr_is_own_shim(p)) {
        return p;
    }
    if (p) {
        LOGW("resolve_oem_symbol(%s): dlsym(RTLD_NEXT) returned OUR "
             "shim %p — LD_PRELOAD interposition trap, falling through",
             name, p);
    }

    // 2) NOLOAD-dlopen path. Get a handle to the already-mapped
    // libaap_interface.so regardless of its scope.
    if (!g_libaap_handle) {
        // Try the unversioned soname first (lets the dynamic linker's
        // search path / DT_RUNPATH find the right copy), then fall
        // back to the absolute path documented at the top of this file.
        g_libaap_handle = dlopen("libaap_interface.so",
                                 RTLD_NOW | RTLD_NOLOAD);
        if (!g_libaap_handle) {
            g_libaap_handle = dlopen("/usr/lib/libaap_interface.so",
                                     RTLD_NOW | RTLD_NOLOAD);
        }
        if (!g_libaap_handle) {
            LOGE("resolve_oem_symbol: dlopen(libaap_interface.so, "
                 "RTLD_NOLOAD) failed: %s", dlerror());
            return nullptr;
        }
        LOGD("resolve_oem_symbol: libaap_interface.so handle=%p "
             "(via dlopen NOLOAD fallback)", g_libaap_handle);
    }

    p = dlsym(g_libaap_handle, name);
    LOGD("resolve_oem_symbol(%s): dlsym(handle=%p) -> %p "
         "(our shims: create=%p destroy=%p)",
         name, g_libaap_handle, p,
         reinterpret_cast<void *>(&aap_create_session),
         reinterpret_cast<void *>(&aap_destroy_session));
    if (!p) {
        LOGE("resolve_oem_symbol: dlsym(%s) on libaap_interface.so "
             "handle failed: %s", name, dlerror());
        return nullptr;
    }

    // 3) Self-loop guard. If glibc's per-handle dlsym still walks
    // through preloaded objects (it does on Linux: LD_PRELOAD libs
    // are inserted at the front of every link-map's search scope,
    // including ones used by handle-based dlsym), we get OUR address
    // back here. Calling it would recurse forever and stack-overflow
    // into a SIGSEGV — which is exactly what was happening before
    // this guard was added.
    if (addr_is_own_shim(p)) {
        LOGC("resolve_oem_symbol(%s): dlsym(handle) returned OUR shim "
             "%p — LD_PRELOAD interposition trap; refusing to recurse. "
             "The real libaap_interface.so impl is unreachable through "
             "the loader's per-handle search path; need ELF-symtab "
             "fallback to bypass it.", name, p);
        return nullptr;
    }
    return p;
}

void resolve_real_once(void)
{
    if (!g_real_create) {
        g_real_create = reinterpret_cast<pfn_aap_create_session>(
            resolve_oem_symbol("aap_create_session"));
    }
    if (!g_real_destroy) {
        g_real_destroy = reinterpret_cast<pfn_aap_destroy_session>(
            resolve_oem_symbol("aap_destroy_session"));
    }
}

} // namespace

// NOTE: -fvisibility=hidden is in the build flags, so each shim symbol
// must be explicitly exported with default visibility for the loader
// to bind OEM PLT lookups to it.
#define PRELOAD_EXPORT __attribute__((visibility("default")))

extern "C" PRELOAD_EXPORT
int aap_create_session(const char *cfg, void *unknown_r1,
                       void *cb_list, void **out_handle)
{
    LOGD("aap_create_session: ENTER cfg=%p r1=%p cb_list=%p out_handle=%p",
         (const void *)cfg, unknown_r1, cb_list, (void *)out_handle);

    resolve_real_once();
    LOGD("aap_create_session: resolve_real_once done "
         "(real_create=%p real_destroy=%p)",
         (void *)g_real_create, (void *)g_real_destroy);

    // First invocation: try to set g_enabled by probing for the OEM
    // anchor. We have to do this here (not in the constructor) because
    // blmjciaapa.so isn't dlopen'd at constructor time.
    oem_self_gate();
    LOGD("aap_create_session: self-gate done (g_enabled=%d)",
         g_enabled ? 1 : 0);

    if (!g_real_create) {
        // Nothing we can do — without the original, the OEM startup
        // breaks. Return the SDK's "internal error" code.
        LOGC("aap_create_session: real implementation not resolved "
             "— passing through impossible");
        return 0x103;
    }

    // Nav-side hijack: substitute cb_list[10] (E_AAP_EVENT_NAV_DATA_CB)
    // with our dump callback before the SDK copies the table into the
    // session handle. The OEM leaves slot 10 NULL, so without this
    // every 0x500 / 0x501 / 0x502 event the phone sends gets dropped
    // by libaap_interface.so's dispatcher (case 0xc, branches on
    // handle+0x48 == NULL). Gated on g_enabled so a misdeployed
    // library still transparently passes through.
    //
    // Modifying the caller's stack buffer in place is safe: inside
    // aap_create_session the SDK does memcpy(handle+0x20, cb_list, 76)
    // before returning.
    if (g_enabled) {
        nav_install_cb(cb_list);
    }

    LOGD("aap_create_session: calling real impl at %p",
         (void *)g_real_create);
    int rc = g_real_create(cfg, unknown_r1, cb_list, out_handle);
    LOGD("aap_create_session: real impl returned rc=%d *out_handle=%p",
         rc, (out_handle && *out_handle) ? *out_handle : nullptr);

    if (rc != 0 || !g_enabled || !out_handle || !*out_handle) {
        LOGD("aap_create_session: NOT starting touch reader "
             "(rc=%d g_enabled=%d out_handle=%p *out_handle=%p)",
             rc, g_enabled ? 1 : 0, (void *)out_handle,
             (out_handle && *out_handle) ? *out_handle : nullptr);
        return rc;
    }

    // OEM retries with sleep(1) on rc==0x104; our reader is keyed off
    // the FIRST successful create, so we only start once per process.
    pthread_mutex_lock(&g_session_mu);
    g_session = *out_handle;
    if (!g_session_up) {
        LOGD("aap_create_session: spawning touch reader");
        touch_start();
        g_session_up = true;
        LOGD("touch reader started");

        // HUD sender: opens the OEM HMI + Service D-Bus connections
        // and brings up dispatcher + sender threads. Same
        // "once per process" gate as touch_start — the sender
        // thread itself handles HUD-not-installed and routing
        // start/stop transitions internally.
        LOGD("aap_create_session: spawning HUD sender");
        hud_send_start();
    }
    pthread_mutex_unlock(&g_session_mu);

    LOGD("aap_create_session: LEAVE rc=%d", rc);
    return rc;
}

extern "C" PRELOAD_EXPORT
int aap_destroy_session(void *handle)
{
    LOGD("aap_destroy_session: ENTER handle=%p", handle);
    resolve_real_once();

    if (g_enabled) {
        pthread_mutex_lock(&g_session_mu);
        if (g_session_up) {
            LOGD("aap_destroy_session: stopping touch reader");
            touch_stop();
            g_session_up = false;
            LOGD("touch reader stopped");

            // Tear the HUD sender down on the same edge that
            // stops the touch reader. hud_send_stop is
            // idempotent so the gate on g_session_up keeps it
            // matched 1:1 with the start above.
            LOGD("aap_destroy_session: stopping HUD sender");
            hud_send_stop();
        }
        g_session = nullptr;
        pthread_mutex_unlock(&g_session_mu);
    }

    if (!g_real_destroy) {
        LOGC("aap_destroy_session: real implementation not resolved");
        return 0x103;
    }

    LOGD("aap_destroy_session: calling real impl at %p",
         (void *)g_real_destroy);
    int rc = g_real_destroy(handle);
    LOGD("aap_destroy_session: LEAVE rc=%d", rc);
    return rc;
}
