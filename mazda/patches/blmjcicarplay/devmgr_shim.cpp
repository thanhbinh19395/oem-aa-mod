// SPDX-License-Identifier: AGPL-3.0-or-later
//
// CarPlay -> HUD bridge for Mazda Connect (CMU150).
// Copyright (C) 2026 KID MIXER-MODER.
//
// Part of an AGPL-3.0 work - see NOTICE.md for full attribution.
//
// PLT shadows for the devmgr client API that jciCARPLAY (blmjcicarplay.so)
// imports from /usr/lib/libdevmgr_interface.so, PLUS a libc `msgrcv` interposer
// that passively taps the device-manager receive path.
//
// Why msgrcv: RE of eDevMgrRegisterProcess + the dispatch loop (FUN_000402c8 in
// libdevmgr_interface.so) shows the process handle stores cbTable@+0x6c, ctx@+0x70,
// msgq@+0x14, and the receive thread (vDevMgrProcessRcvThread, running INSIDE the
// jciCARPLAY process) pulls every event with race_msg_queue_receivemessage ->
// libc `msgrcv` (imported via PLT, confirmed). ipoddev's viAP2NavRouteManeuverUpdate
// pushes a 0x348-byte message (guidance = 0x640). So interposing msgrcv lets us
// SEE every message ipoddev delivers — including maneuvers — without touching the
// OEM vtable/dispatch (zero crash risk, fully passive: we always chain the real
// call and never alter the data). This is BOTH a definitive probe (does iOS ever
// send maneuvers?) AND the hook point a future build-2b can decode -> hud_send.
//
// ARM EABI: devmgr fns take args in r0..r3; we declare 4 void* and forward all.

#include "patch.h"
#include "oem/libjcidbus.h"
#include "common/config.h"      // runtime libpatch-carplay.conf (carplay_* keys)
#include "common/thread_util.h" // preload_thread_create (256 KiB bounded stack)

#include <atomic>
#include <dlfcn.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>

namespace {

typedef int (*pfn4)(void *, void *, void *, void *);

// RouteGuidanceDisplayComponent identifier advertised by ipoddev_full_auth.
// Disasm proof on FW 74.00.324A ipoddev_full_auth:
//   - /etc/devmgr_config_master.xml sets `<name>NaviCompId</name><value>0x5555</value>`.
//   - eDevMgriPodInitConfigParams (@0x53fc4) passes "0x5555" to eDevMgrSetIntParam (@0x36fa0),
//     which calls `strtol(val, NULL, 10)` (BASE 10 @0x37004, NOT base 0!).
//   - `strtol("0x5555", NULL, 10)` stops at 'x' and returns 0 (>= 0), overwriting the 0x5555
//     default at cfg+3712 with 0. Thus iAP2 IdentificationInformation advertises
//     RouteGuidanceDisplayComponent Identifier = 0 (NOT 0x5555) to the iPhone.
//   - Sending StartNavigationRouteGuidanceUpdate (0x5200) with compId=0x5555 returns rc=1
//     from ipoddev IPC, but iOS silently drops it because component 0x5555 was never declared!
constexpr uintptr_t kNaviCompId         = 0u;
constexpr uintptr_t kNaviCompIdFallback = 0x5555u;

pfn4  g_real_register  = nullptr;
pfn4  g_real_callstate = nullptr;
pfn4  g_real_commupd   = nullptr;
pfn4  g_real_sendloc   = nullptr;
pfn4  g_real_startnavi = nullptr;

void            *g_devmgr_handle    = nullptr;
pthread_once_t   g_resolve_once     = PTHREAD_ONCE_INIT;
pthread_mutex_t  g_mu               = PTHREAD_MUTEX_INITIALIZER;
bool             g_services_started = false;
bool             g_retry_thread_up  = false;
bool             g_session_active   = false;
void            *g_procH            = nullptr;
void            *g_devH             = nullptr;
void            *g_started_devH     = nullptr;
std::atomic<bool>     g_armed{false};         // gates the passive msgrcv tap
std::atomic<bool>     g_nav_rx_seen{false};   // true once 0x8058/0x8059 received in this session
std::atomic<bool>     g_cp_tbt_active{false}; // true while OEM TurnByTurnEntity == 1 (CARPLAY)
std::atomic<uint32_t> g_tbt_epoch{0};         // bumped on each TurnByTurnEntity -> 1 transition

extern "C" int eDevMgrRegisterProcess(void *, void *, void *, void *);
extern "C" int eDevMgriPodStartCallStateUpdates(void *, void *, void *, void *);
extern "C" int eDevMgriPodStartCommunicationUpdates(void *, void *, void *, void *);
extern "C" int eDevMgriPodSendLocationInformation(void *, void *, void *, void *);

bool addr_is_own(void *a)
{
    return a == (void *)&eDevMgrRegisterProcess ||
           a == (void *)&eDevMgriPodStartCallStateUpdates ||
           a == (void *)&eDevMgriPodStartCommunicationUpdates ||
           a == (void *)&eDevMgriPodSendLocationInformation;
}

void *resolve_oem(const char *name)
{
    void *p = dlsym(RTLD_NEXT, name);
    if (p && !addr_is_own(p)) return p;
    if (!g_devmgr_handle) {
        g_devmgr_handle = dlopen("libdevmgr_interface.so", RTLD_NOW | RTLD_NOLOAD);
        if (!g_devmgr_handle)
            g_devmgr_handle = dlopen("/usr/lib/libdevmgr_interface.so", RTLD_NOW | RTLD_NOLOAD);
        if (!g_devmgr_handle) { LOGE("resolve_oem: dlopen NOLOAD failed: %s", dlerror()); return nullptr; }
    }
    p = dlsym(g_devmgr_handle, name);
    if (!p) { LOGE("resolve_oem(%s): dlsym failed: %s", name, dlerror()); return nullptr; }
    if (addr_is_own(p)) { LOGC("resolve_oem(%s): interposition trap", name); return nullptr; }
    return p;
}

void resolve_all_impl(void)
{
    g_real_register  = (pfn4)resolve_oem("eDevMgrRegisterProcess");
    g_real_callstate = (pfn4)resolve_oem("eDevMgriPodStartCallStateUpdates");
    g_real_commupd   = (pfn4)resolve_oem("eDevMgriPodStartCommunicationUpdates");
    g_real_sendloc   = (pfn4)resolve_oem("eDevMgriPodSendLocationInformation");
    g_real_startnavi = (pfn4)resolve_oem("eDevMgriPodStartNaviRouteGuidenceUpdates");
}

void resolve_all_once(void)
{
    pthread_once(&g_resolve_once, resolve_all_impl);
}

// Retry worker:
//  1) Retries StartNaviRouteGuidenceUpdates(compId=0) during CarPlay bring-up if the initial
//     call returned rc != 1.
//  2) Also acts as a bounded active-TBT recovery (at most 4 attempts per TBT activation): if
//     CarPlay owns Turn-by-Turn (TurnByTurnEntity == 1, e.g. Google Maps / Apple Maps / VietMap)
//     yet zero 0x8058/0x8059 frames have arrived (e.g. CallState fired slightly before iAP2
//     Identification settled, or decimal NaviCompId=21845 was configured), re-sends compId=0
//     (and fallback 0x5555 on odd retries). Stays 100% idle during non-navigating drives and
//     as soon as 0x8058/0x8059 traffic is seen.
void *navi_retry_thread(void *)
{
    void    *last_devH     = nullptr;
    uint32_t last_epoch    = 0;
    int      init_attempts = 0;
    int      tbt_attempts  = 0;
    for (;;) {
        sleep(init_attempts >= 10 ? 15 : 2);

        const bool     rx_seen    = g_nav_rx_seen.load(std::memory_order_relaxed);
        const bool     tbt_active = g_cp_tbt_active.load(std::memory_order_relaxed);
        const uint32_t cur_epoch  = g_tbt_epoch.load(std::memory_order_relaxed);
        if (cur_epoch != last_epoch) {
            last_epoch   = cur_epoch;
            tbt_attempts = 0;
        }

        void *procH      = nullptr;
        void *devH       = nullptr;
        pfn4  fn         = nullptr;
        bool  need_init  = false;
        pthread_mutex_lock(&g_mu);
        if (g_session_active && g_procH && g_devH && !rx_seen) {
            need_init = (g_started_devH != g_devH);
            if (need_init || (tbt_active && tbt_attempts < 4)) {
                procH = g_procH;
                devH  = g_devH;
                fn    = g_real_startnavi;
            }
        }
        pthread_mutex_unlock(&g_mu);

        if (!fn || !procH || !devH) {
            init_attempts = 0;
            continue;
        }
        if (devH != last_devH) {
            last_devH     = devH;
            init_attempts = 0;
            tbt_attempts  = 0;
        }

        uintptr_t compId = kNaviCompId;
        int attempt_no   = 0;
        if (need_init) {
            init_attempts++;
            attempt_no = init_attempts;
        } else {
            // Alternate 0, 0x5555, 0, 0x5555 across the 4 bounded TBT recovery tries.
            compId = (tbt_attempts & 1) ? kNaviCompIdFallback : kNaviCompId;
            tbt_attempts++;
            attempt_no = tbt_attempts;
        }

        int rc = fn(procH, devH, reinterpret_cast<void *>(compId), nullptr);
        LOGD("retry#%d (%s) StartNaviRouteGuidence(procH=%p devH=%p compId=0x%lx) rc=%d",
             attempt_no, need_init ? "init" : "tbt-active",
             procH, devH, static_cast<unsigned long>(compId), rc);
        if (rc == 1 && compId == kNaviCompId) {
            pthread_mutex_lock(&g_mu);
            if (g_session_active && g_devH == devH) {
                g_started_devH = devH;
            }
            pthread_mutex_unlock(&g_mu);
            init_attempts = 0;
        }
    }
    return nullptr;
}

// ===== [NAV-END via OEM CarPlay TBT status] =================================
// The OEM blmjcicarplay.so (this very process) broadcasts the explicit CarPlay
// nav-status on the HMI bus's com.jci.carplay interface — the same event AA gets
// decoded from the AAP SDK, which the CarPlay bridge previously lacked (it only
// cleared via a 6s staleness timer). We SUBSCRIBE to it using the OEM's own
// generated client stubs (libjcicarplay_client.so), so the OEM does the match +
// decode for us. Signals of interest:
//   TurnByTurnEntitySignal(entityType u): 0=NONE 1=CARPLAY 2=NATIVE — who owns TBT.
//   SessionDeactiveSignal: CarPlay session ended (e.g. phone unplugged).
// cb ABI (RE-confirmed byte-exact: client dispatcher signal_carplay_* @0xb2d0 +
// consumer svcjcinavi CP_RoutingStatusChanged_cb @0x2e0c4): the worker calls
//   cb(r0=conn, r1=<field0 ptr>, r2=<field1/aux ptr>, r3=userdata)
// with each decoded field delivered BY POINTER (deref r1 for the u32), userdata in
// r3, and r1/r2 freed right after the callback returns (copy out, don't retain).
typedef int (*pfn_cp_enable)(void *conn, int enable, void *cb, void *userdata);
pfn_cp_enable g_cp_tbt_enable      = nullptr;
pfn_cp_enable g_cp_deactive_enable = nullptr;
void *g_cp_hmi_conn   = nullptr;
void *g_cp_client_lib = nullptr;
bool  g_cp_subscribed = false;

void cp_hmi_err(void *conn, void *closure)
{ (void)conn; (void)closure; LOGW("carplay_status: HMI bus error cb (non-fatal)"); }

// TurnByTurnEntitySignal handler. r1 = uint32_t* entityType (deref). DEBUG BUILD:
// LOGD only — wire hud_on_status(0) clear AFTER a real-drive test confirms the values.
void cp_tbt_entity_cb(void *conn, void *p_entity, void *p_aux, void *userdata)
{
    (void)conn; (void)p_aux; (void)userdata;
    try {
        unsigned e = p_entity ? *reinterpret_cast<const uint32_t *>(p_entity) : 0xffffffffu;
        LOGD("carplay TBT entity=%u (%s)",
             e, e == 1 ? "CARPLAY" : e == 2 ? "NATIVE" : e == 0 ? "NONE" : "?");
        // entityType 1=CARPLAY (CarPlay owns turn-by-turn = nav active). Anything else — 0=NONE
        // (nav switched off) or 2=NATIVE (the OEM nav took the HUD) — means CarPlay is no longer
        // doing TBT, so wipe our HUD. Verified live: turning nav off fires entity 1->0. The 6s
        // staleness timer stays as a backstop for any unit where this subscribe didn't come up.
        if (e == 1) {
            g_cp_tbt_active.store(true, std::memory_order_relaxed);
            g_tbt_epoch.fetch_add(1, std::memory_order_relaxed);
        } else {
            g_cp_tbt_active.store(false, std::memory_order_relaxed);
            hud_request_clear();
        }
    } catch (...) { LOGE("cp_tbt_entity_cb exception swallowed (OEM dbus thread)"); }
}

// SessionDeactiveSignal handler. Multi-field payload (clientId,type,reason,...) — we don't
// read the fields; arrival = session gone.
void cp_deactive_cb(void *a0, void *a1, void *a2, void *a3)
{
    (void)a0; (void)a1; (void)a2; (void)a3;
    try {
        LOGD("carplay SessionDeactive (session ended)");
        pthread_mutex_lock(&g_mu);
        g_session_active = false;
        g_started_devH   = nullptr;
        pthread_mutex_unlock(&g_mu);
        g_cp_tbt_active.store(false, std::memory_order_relaxed);
        g_nav_rx_seen.store(false, std::memory_order_relaxed);
        hud_request_fullclear();   // CarPlay session gone (e.g. phone unplugged) -> FULL wipe incl sign (like AA teardown)
    } catch (...) { LOGE("cp_deactive_cb exception swallowed (OEM dbus thread)"); }
}

void *cp_resolve(const char *name)
{
    void *p = g_cp_client_lib ? dlsym(g_cp_client_lib, name) : nullptr;
    if (!p) p = dlsym(RTLD_DEFAULT, name);
    return p;
}

// Subscribe to the OEM CarPlay TBT-status signals on the HMI bus. Idempotent; best-effort
// (any failure just leaves the staleness timer as the fallback — never fatal).
void carplay_status_start(void)
{
    if (g_cp_subscribed) return;
    // The CARPLAY_*Signal_enable stubs live in the CONSUMER lib libjcicarplay_client.so, which
    // jciCARPLAY (the producer) does not normally load — dlopen it GLOBAL so its JCIDBUS imports
    // resolve against the already-loaded provider.
    if (!g_cp_client_lib) {
        g_cp_client_lib = dlopen("libjcicarplay_client.so", RTLD_NOW | RTLD_GLOBAL);
        if (!g_cp_client_lib)
            g_cp_client_lib = dlopen("/jci/lib/libjcicarplay_client.so", RTLD_NOW | RTLD_GLOBAL);
    }
    if (!g_cp_client_lib) { LOGW("carplay_status: dlopen libjcicarplay_client.so failed: %s", dlerror()); return; }
    g_cp_tbt_enable      = (pfn_cp_enable)cp_resolve("CARPLAY_TurnByTurnEntitySignal_enable");
    g_cp_deactive_enable = (pfn_cp_enable)cp_resolve("CARPLAY_SessionDeactiveSignal_enable");
    if (!g_cp_tbt_enable) { LOGW("carplay_status: TurnByTurnEntitySignal_enable unresolved — staleness only"); return; }

    g_cp_hmi_conn = JCIDBUS_conn_create(reinterpret_cast<void *>(&cp_hmi_err), 0);
    if (!g_cp_hmi_conn) { LOGW("carplay_status: HMI conn_create failed"); return; }
    if (JCIDBUS_conn_connect(g_cp_hmi_conn, "com.jci.carplay.hudstatus", kJciHmiBus, nullptr) == 0) {
        LOGW("carplay_status: HMI conn_connect failed — staleness only");
        JCIDBUS_conn_free(g_cp_hmi_conn); g_cp_hmi_conn = nullptr; return;
    }
    JCIDBUS_worker_start(g_cp_hmi_conn);
    int re = g_cp_tbt_enable(g_cp_hmi_conn, 1, reinterpret_cast<void *>(&cp_tbt_entity_cb), nullptr);
    int rd = g_cp_deactive_enable
               ? g_cp_deactive_enable(g_cp_hmi_conn, 1, reinterpret_cast<void *>(&cp_deactive_cb), nullptr)
               : -1;
    g_cp_subscribed = true;
    LOGD("carplay_status: subscribed HMI com.jci.carplay TurnByTurnEntity(rc=%d) + SessionDeactive(rc=%d)", re, rd);
}

void arm_navi(const char *from, void *procH, void *devH)
{
    // Only run inside the real jciCARPLAY launcher (main.cpp's constructor gate set g_enabled).
    if (!g_enabled || !procH || !devH) return;

    bool need_start = false;
    pfn4 startnavi  = nullptr;

    pthread_mutex_lock(&g_mu);
    if (!g_services_started) {
        g_services_started = true;
        // Bring up the HUD sender (D-Bus to com.jci.vbs.navi) and subscribe to
        // OEM CarPlay TBT-status signals on the HMI bus. Idempotent.
        hud_send_start();
        carplay_status_start();
    }
    if (!g_session_active || g_devH != devH) {
        need_start = true;
        g_started_devH = nullptr;
        g_nav_rx_seen.store(false, std::memory_order_relaxed);
    }
    g_procH          = procH;
    g_devH           = devH;
    g_session_active = true;
    startnavi        = g_real_startnavi;
    if (need_start && startnavi && !g_retry_thread_up) {
        pthread_t th;
        if (preload_thread_create(&th, navi_retry_thread, nullptr) == 0) {
            pthread_detach(th);
            g_retry_thread_up = true;
        }
    }
    pthread_mutex_unlock(&g_mu);

    g_armed.store(true, std::memory_order_release);

    if (need_start) {
        if (startnavi) {
            LOGD("arm_navi from %s: StartNaviRouteGuidence(procH=%p devH=%p compId=0x%lx)",
                 from, procH, devH, static_cast<unsigned long>(kNaviCompId));
            int rc = startnavi(procH, devH, reinterpret_cast<void *>(kNaviCompId), nullptr);
            LOGD("StartNaviRouteGuidence rc=%d", rc);
            if (rc == 1) {
                pthread_mutex_lock(&g_mu);
                if (g_session_active && g_devH == devH) {
                    g_started_devH = devH;
                }
                pthread_mutex_unlock(&g_mu);
            }
        } else {
            LOGE("arm_navi: StartNavi unresolved");
        }
    }
}

// ---- msgrcv tap (passive) --------------------------------------------------
typedef ssize_t (*pfn_msgrcv)(int, void *, size_t, long, int);
std::atomic<pfn_msgrcv> g_real_msgrcv{nullptr};

// [DIAGNOSTIC] In debug builds (DEBUG / !NDEBUG), message types and raw nav
// payloads are automatically logged to the system log (stderr / g_logf) with
// namespaces [CP-MSG] and [CP-NAV] for easy filtering, avoiding flash wear.
void msg_tap(int msqid, const void *msgp, ssize_t n)
{
    (void)msqid;

#ifndef NDEBUG
    // In DEBUG mode: log every received message's (type, size) + a payload peek for small messages
    if (msgp && n >= 4) {
        uint32_t mt_raw = 0;
        memcpy(&mt_raw, msgp, sizeof(mt_raw));
        uint32_t mt = mt_raw & 0xffffu;
        char peek[128] = "";
        if (mt != 0x8058 && mt != 0x8059) {          // peek the unknown / small messages
            const unsigned char *b = (const unsigned char *)msgp;
            ssize_t dn = (n < 20) ? n : 20;
            char *p = peek;
            for (ssize_t i = 0; i < dn; i++) {
                p += snprintf(p, sizeof(peek) - (size_t)(p - peek), " %02x", b[i]);
            }
        }
        LOGD("[CP-MSG] mt=0x%04x n=0x%lx%s", (unsigned)mt, (long)n, peek);
    }
#endif

    // Only big messages: maneuver=0x348, guidance=0x65c. Skips chatty small msgs.
    if (n < 0x300 || !msgp) return;

    uint32_t mt_raw = 0;
    memcpy(&mt_raw, msgp, sizeof(mt_raw));
    uint32_t mt = mt_raw & 0xffffu;
    if (mt == 0x8059 || mt == 0x8058) {
        g_nav_rx_seen.store(true, std::memory_order_relaxed);
    }

    // Decode maneuver -> HUD (passive read of the received buffer).
    nav_on_devmgr_msg(msgp, (long)n);

#ifndef NDEBUG
    // In DEBUG mode: dump raw nav payload to system log
    if (mt == 0x8059 || mt == 0x8058) {
        LOGD("[CP-NAV] msgrcv n=0x%lx mtype=0x%x", (long)n, (unsigned)mt);
        const unsigned char *b = (const unsigned char *)msgp;
        char hex[100];
        for (ssize_t i = 0; i < n; i += 16) {
            ssize_t chunk = (n - i < 16) ? (n - i) : 16;
            char *p = hex;
            for (ssize_t j = 0; j < chunk; j++) {
                p += snprintf(p, sizeof(hex) - (size_t)(p - hex), " %02x", b[i + j]);
            }
            LOGD("[CP-NAV] +%04lx:%s", (long)i, hex);
        }
    }
#endif
}

} // namespace

#define PRELOAD_EXPORT __attribute__((visibility("default")))

extern "C" PRELOAD_EXPORT
ssize_t msgrcv(int msqid, void *msgp, size_t msgsz, long msgtyp, int msgflg)
{
    pfn_msgrcv fn = g_real_msgrcv.load(std::memory_order_acquire);
    if (!fn) {
        fn = reinterpret_cast<pfn_msgrcv>(dlsym(RTLD_NEXT, "msgrcv"));
        if (fn) g_real_msgrcv.store(fn, std::memory_order_release);
    }
    ssize_t n = fn ? fn(msqid, msgp, msgsz, msgtyp, msgflg) : -1;
    // Only tap once nav is armed by a live CarPlay session. In release builds,
    // fast-path skip small IPC messages without entering msg_tap.
#ifdef NDEBUG
    if (g_enabled && n >= 0x300 && g_armed.load(std::memory_order_relaxed))
        msg_tap(msqid, msgp, n);
#else
    if (g_enabled && n > 0 && g_armed.load(std::memory_order_relaxed))
        msg_tap(msqid, msgp, n);
#endif
    return n;
}

extern "C" PRELOAD_EXPORT
int eDevMgrRegisterProcess(void *procStruct, void *role, void *cbTable, void *ctx)
{
    resolve_all_once();
    LOGD("eDevMgrRegisterProcess: procStruct=%p role=%ld cbTable=%p ctx=%p",
         procStruct, (long)role, cbTable, ctx);
    if (!g_real_register) { LOGC("RegisterProcess: real unresolved"); return 0x103; }
    int rc = g_real_register(procStruct, role, cbTable, ctx);
    LOGD("eDevMgrRegisterProcess: rc=%d", rc);
    return rc;
}

extern "C" PRELOAD_EXPORT
int eDevMgriPodStartCallStateUpdates(void *a0, void *a1, void *a2, void *a3)
{
    resolve_all_once();
    LOGD("StartCallStateUpdates: a0=%p a1=%p a2=%p a3=%p", a0, a1, a2, a3);
    int rc = g_real_callstate ? g_real_callstate(a0, a1, a2, a3) : 0x103;
    arm_navi("CallState", a0, a1);
    return rc;
}

extern "C" PRELOAD_EXPORT
int eDevMgriPodStartCommunicationUpdates(void *a0, void *a1, void *a2, void *a3)
{
    resolve_all_once();
    int rc = g_real_commupd ? g_real_commupd(a0, a1, a2, a3) : 0x103;
    arm_navi("CommUpdates", a0, a1);
    return rc;
}

extern "C" PRELOAD_EXPORT
int eDevMgriPodSendLocationInformation(void *a0, void *a1, void *a2, void *a3)
{
    resolve_all_once();
    int rc = g_real_sendloc ? g_real_sendloc(a0, a1, a2, a3) : 0x103;
    arm_navi("SendLocation", a0, a1);
    return rc;
}
