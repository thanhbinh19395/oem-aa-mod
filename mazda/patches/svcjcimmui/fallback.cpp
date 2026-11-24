// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Redirect CMU audio-source fallback away from FM/AM/SiriusXM/DAB.
//
// Stock libjciuiasystem.so (FW 74.00.324A) has multiple hardcoded paths
// that fall back to broadcast radio (almost always FM):
//
// 1. Startup source activation (ACC ON / Boot):
//    UIA_SYSTEM_ActivateActiveOrLastAudio calls getFirstAvailableSource()
//    (0x0003a358). If CarPlay / Bluetooth is not connected yet, stock logic
//    skips unready sources and falls back directly to findByName("fmradio"),
//    blasting harsh FM static at the start of every drive.
//    We hook getFirstAvailableSource(): if the candidate is broadcast and
//    was NOT the user's active source before shutdown, return nullptr so
//    ActivateActiveOrLastAudio cleanly enters stock SelectEntertainmentNoAudio
//    (Audio Off). If the user was genuinely listening to radio, it is kept.
//
// 2. Disconnect fallback (activatePrevious):
//    activatePrevious() (0x0003b37c) runs when CarPlay/AA/USB disconnects.
//    If the candidate is broadcast radio, we inject SelectEntertainmentNoAudio.
//
// 3. Reconnect-timer expiry (ActivateLastBroadcast):
//    UIA_SYSTEM_ActivateLastBroadcast() (0x0003dc88) runs when the reconnect
//    timer expires. We hook it to inject SelectEntertainmentNoAudio instead.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#define LOG_TAG "FALLBACK"
#include "log.h"
#include "fallback.h"
#include "common/config.h"
#include "common/thread_util.h"

#include <cstdint>
#include <cstring>
#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <sys/mman.h>
#include <unistd.h>

namespace {

// --- FW 74.00.324A libjciuiasystem.so layout ----------------------------
constexpr const char *kUiaSystemSo = "/jci/mmui/uia/libjciuiasystem.so";
constexpr const char *kAnchorSym   = "UiApp_InjectEvent";
constexpr uintptr_t   kOffAnchor   = 0x0005c220; // UiApp_InjectEvent (exported)

constexpr uintptr_t kOffActivatePrevious        = 0x0003b37c;
constexpr uintptr_t kOffGetFirstAvailableSource = 0x0003a358;
constexpr uintptr_t kOffActivateLastBroadcast   = 0x0003dc88;
constexpr uintptr_t kOffGetCurrent              = 0x00039e2c;
constexpr uintptr_t kOffGetPrevious             = 0x00039e3c;
constexpr uintptr_t kOffGetLastBroadcast        = 0x00039e50;
constexpr uintptr_t kOffIsAvailable             = 0x00038290;
constexpr uintptr_t kOffIsBroadcastSource       = 0x00039d1c;

// Expected first two instructions of activatePrevious (push + sub sp).
constexpr uint32_t kPrologue0_ap  = 0xe92d4070; // push {r4,r5,r6,lr}
constexpr uint32_t kPrologue1_ap  = 0xe24dd018; // sub  sp, sp, #24

// Expected first two instructions of getFirstAvailableSource (push + sub sp).
constexpr uint32_t kPrologue0_fas = 0xe92d4010; // push {r4,lr}
constexpr uint32_t kPrologue1_fas = 0xe24dd010; // sub  sp, sp, #16

// Expected first two instructions of ActivateLastBroadcast (push + sub sp).
constexpr uint32_t kPrologue0_alb = 0xe92d4010; // push {r4,lr}
constexpr uint32_t kPrologue1_alb = 0xe24dd018; // sub  sp, sp, #24

// Absolute jump: ldr pc, [pc, #-4] ; .word target
constexpr uint32_t kLdrPcM4   = 0xe51ff004;

using ActivatePreviousFn      = int (*)(int);
using ActivateLastBroadcastFn = int (*)();
using GetAppFn                = void *(*)();
using IsAvailableFn           = int (*)(void *);
using IsBroadcastFn           = int (*)(void *);
using InjectToNameFn          = void (*)(const char *, const char *, void *, int);

uintptr_t               g_base                      = 0;
ActivatePreviousFn      g_real_activate             = nullptr;
GetAppFn                g_real_first_available      = nullptr;
ActivateLastBroadcastFn g_real_activate_last_bcast  = nullptr;

GetAppFn                g_get_current               = nullptr;
GetAppFn                g_get_previous              = nullptr;
GetAppFn                g_get_last_bcast            = nullptr;
IsAvailableFn           g_is_available              = nullptr;
IsBroadcastFn           g_is_broadcast              = nullptr;
InjectToNameFn          g_inject_to_name            = nullptr;

bool                    g_installed                 = false;

bool set_prot(uintptr_t addr, size_t len, int prot)
{
    const uintptr_t ps   = static_cast<uintptr_t>(sysconf(_SC_PAGESIZE));
    const uintptr_t page = addr & ~(ps - 1);
    const uintptr_t end  = addr + len;
    return mprotect(reinterpret_cast<void *>(page), end - page, prot) == 0;
}

void *at(uintptr_t off)
{
    return reinterpret_cast<void *>(g_base + off);
}

int select_audio_off()
{
    if (!g_inject_to_name) {
        LOGE("fallback: UiApp_InjectEventToUiaName unresolved");
        return 0;
    }
    LOGD("fallback: injecting SelectEntertainmentNoAudio (Audio Off)");
    g_inject_to_name("system", "SelectEntertainmentNoAudio", nullptr, 0);
    return 1;
}

// 1. Hook activatePrevious (disconnect fallback).
int hook_activatePrevious(int flag)
{
    if (!libpatch_config::fallback_audio_off() || !g_real_activate) {
        return g_real_activate ? g_real_activate(flag) : 0;
    }

    void *cand = nullptr;
    if (g_get_previous && g_is_available) {
        void *prev = g_get_previous();
        if (prev && g_is_available(prev) == 1) {
            cand = prev;
        }
    }
    if (!cand && g_get_last_bcast && g_is_available) {
        void *lb = g_get_last_bcast();
        if (lb && g_is_available(lb) == 1) {
            cand = lb;
        }
    }

    if (cand && g_is_broadcast && g_is_broadcast(cand) == 1) {
        LOGD("fallback: activatePrevious broadcast candidate (%p) — Audio Off instead of radio", cand);
        return select_audio_off();
    }

    return g_real_activate(flag);
}

// 2. Hook getFirstAvailableSource (bootup / ACC ON source activation fallback).
void *hook_getFirstAvailableSource()
{
    if (!libpatch_config::fallback_audio_off() || !g_real_first_available) {
        return g_real_first_available ? g_real_first_available() : nullptr;
    }

    void *cand = g_real_first_available();
    if (!cand) {
        return nullptr;
    }

    // Only intercept broadcast radio (FM, AM, SiriusXM, DAB).
    if (!g_is_broadcast || g_is_broadcast(cand) != 1) {
        return cand;
    }

    // If candidate was the actively playing source before shutdown, keep it (do not break user choice).
    if (g_get_current && g_is_available) {
        void *cur = g_get_current();
        if (cur == cand && g_is_available(cur) == 1) {
            LOGD("fallback: getFirstAvailableSource user-selected active radio (%p) — keeping", cand);
            return cand;
        }
    }

    LOGD("fallback: getFirstAvailableSource broadcast fallback (%p) — returning nullptr (Audio Off)", cand);
    return nullptr;
}

// 3. Hook ActivateLastBroadcast (reconnect-timer expiry fallback).
int hook_ActivateLastBroadcast()
{
    if (!libpatch_config::fallback_audio_off() || !g_real_activate_last_bcast) {
        return g_real_activate_last_bcast ? g_real_activate_last_bcast() : 0;
    }

    LOGD("fallback: ActivateLastBroadcast triggered — Audio Off instead of radio");
    return select_audio_off();
}

bool install_trampoline(uintptr_t target_off, uint32_t p0, uint32_t p1,
                        void *hook_fn, void **out_orig_fn)
{
    uint32_t *target = reinterpret_cast<uint32_t *>(at(target_off));
    if (target[0] != p0 || target[1] != p1) {
        LOGE("fallback: ABORT hook @0x%08lx prologue %08x %08x (expected %08x %08x); wrong FW?",
             static_cast<unsigned long>(target_off), target[0], target[1], p0, p1);
        return false;
    }

    void *rx = mmap(nullptr, 16, PROT_READ | PROT_WRITE | PROT_EXEC,
                    MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (rx == MAP_FAILED) {
        LOGE("fallback: mmap trampoline failed: %s", strerror(errno));
        return false;
    }
    uint32_t *tramp = reinterpret_cast<uint32_t *>(rx);
    tramp[0] = target[0];
    tramp[1] = target[1];
    tramp[2] = kLdrPcM4;
    tramp[3] = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(target) + 8);
    __builtin___clear_cache(reinterpret_cast<char *>(tramp),
                            reinterpret_cast<char *>(tramp + 4));
    *out_orig_fn = reinterpret_cast<void *>(tramp);

    if (!set_prot(reinterpret_cast<uintptr_t>(target), 8, PROT_READ | PROT_WRITE)) {
        LOGE("fallback: mprotect RW failed @%p", target);
        return false;
    }
    // Write target address first, barrier, then jump opcode (thread safety).
    target[1] = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(hook_fn));
    __sync_synchronize();
    target[0] = kLdrPcM4;
    __builtin___clear_cache(reinterpret_cast<char *>(target),
                            reinterpret_cast<char *>(target + 2));
    if (!set_prot(reinterpret_cast<uintptr_t>(target), 8, PROT_READ | PROT_EXEC)) {
        LOGW("fallback: mprotect RX restore failed (hook is live @%p)", target);
    }

    LOGD("fallback: hooked @%p -> %p (trampoline %p)",
         target, hook_fn, tramp);
    return true;
}

bool install_hook(void *h)
{
    void *anchor = dlsym(h, kAnchorSym);
    if (!anchor) {
        LOGE("fallback: dlsym(%s) failed", kAnchorSym);
        return false;
    }

    g_base = reinterpret_cast<uintptr_t>(anchor) - kOffAnchor;
    LOGD("fallback: libjciuiasystem base=0x%08lx",
         static_cast<unsigned long>(g_base));

    g_get_current    = reinterpret_cast<GetAppFn>(at(kOffGetCurrent));
    g_get_previous   = reinterpret_cast<GetAppFn>(at(kOffGetPrevious));
    g_get_last_bcast = reinterpret_cast<GetAppFn>(at(kOffGetLastBroadcast));
    g_is_available   = reinterpret_cast<IsAvailableFn>(at(kOffIsAvailable));
    g_is_broadcast   = reinterpret_cast<IsBroadcastFn>(at(kOffIsBroadcastSource));
    g_inject_to_name = reinterpret_cast<InjectToNameFn>(
        dlsym(h, "UiApp_InjectEventToUiaName"));
    if (!g_inject_to_name) {
        LOGE("fallback: UiApp_InjectEventToUiaName missing");
        return false;
    }

    // 1. Hook activatePrevious (disconnect fallback).
    if (!install_trampoline(kOffActivatePrevious, kPrologue0_ap, kPrologue1_ap,
                            reinterpret_cast<void *>(&hook_activatePrevious),
                            reinterpret_cast<void **>(&g_real_activate))) {
        return false;
    }

    // 2. Hook getFirstAvailableSource (bootup / ACC ON fallback).
    if (!install_trampoline(kOffGetFirstAvailableSource, kPrologue0_fas, kPrologue1_fas,
                            reinterpret_cast<void *>(&hook_getFirstAvailableSource),
                            reinterpret_cast<void **>(&g_real_first_available))) {
        return false;
    }

    // 3. Hook ActivateLastBroadcast (reconnect-timer expiry fallback).
    if (!install_trampoline(kOffActivateLastBroadcast, kPrologue0_alb, kPrologue1_alb,
                            reinterpret_cast<void *>(&hook_ActivateLastBroadcast),
                            reinterpret_cast<void **>(&g_real_activate_last_bcast))) {
        return false;
    }

    g_installed = true;
    LOGD("fallback: all 3 Audio Off hooks successfully installed (FW 74.00.324A)");
    return true;
}

void *waiter_main(void *)
{
    // mmuiLoadUias() runs after constructor; wait until mapped, then install once.
    for (int i = 0; i < 100; ++i) {
        void *h = dlopen(kUiaSystemSo, RTLD_NOW | RTLD_NOLOAD);
        if (h) {
            if (!install_hook(h)) {
                LOGE("fallback: install_hook failed (unsupported FW?) — aborting");
            }
            return nullptr;
        }
        usleep(100 * 1000);
    }
    LOGW("fallback: libjciuiasystem.so never mapped — hook not installed");
    return nullptr;
}

} // namespace

namespace svcjcimmui_fallback {

void start()
{
    if (!libpatch_config::fallback_audio_off()) {
        LOGD("fallback: disabled by config");
        return;
    }
    pthread_t tid;
    if (preload_thread_create(&tid, waiter_main, nullptr) != 0) {
        LOGE("fallback: failed to start waiter thread");
        return;
    }
    pthread_detach(tid);
}

} // namespace svcjcimmui_fallback
