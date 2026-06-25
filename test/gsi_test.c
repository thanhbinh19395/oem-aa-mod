// SPDX-License-Identifier: AGPL-3.0-or-later
//
// gsi_test — toggle the HUD "Gear Shift Indicator" (GearShiftIndicator)
// setting on a Mazda CMU, to test whether the head-up display actually
// renders the GSI overlay.
//
// Background: GearShiftIndicator is the one HUD-content element that is
// still fully wired through the OEM settings stack (registry + VBS push
// + the HUD_Gsi_OnOff HEC field on the CAN link to the HUD ECU) but has
// no UI menu item on these firmware builds. This tool drives the same
// path the (removed) Vehicle Settings menu used to, via the public
// settings client library — so no firmware patching is needed to test.
//
// It does NOT link the OEM library at build time. It dlopen()s
// /jci/lib/libjcisettings_client.so on the device and resolves the three
// C-linkage exports it needs (verified present on FW 74.00.324A):
//   BLM_SETTINGS_Client_Connect(name, on_connect, on_disconnect)
//   SETTINGS_Client_Get_GearShiftIndicator(cb)
//   SETTINGS_Client_Set_GearShiftIndicator(value, cb)
//   BLM_SETTINGS_Client_Disconnect()
// The callbacks are async (they fire on the library's own IPC worker
// thread), so we wait on a condvar with a timeout after each call.
//
// Usage (run on the device, as root):
//   gsi_test            # connect + read the current value, print it
//   gsi_test 2          # read, set GearShiftIndicator=2, read back
//   gsi_test 1          # read, set GearShiftIndicator=1, read back
//
// GearShiftIndicator is an S16 setting with range 1..2 in the OEM
// schema. Which of 1/2 is "on" vs "off" is exactly what this tool is
// for — try both and watch the HUD. The value PERSISTS in the registry,
// so set it back when you're done.
//
// Caveats:
//   * On a monochrome combiner HUD (Hud_Type == CHUD_mono / 0) the OEM
//     push is an explicit no-op, so nothing will change regardless.
//   * Your HUD ECU must support the GSI overlay for anything to show.
//   * If the connect callback never arrives (timeout), the settings
//     service bus wasn't reachable from this process — see the note
//     printed at the bottom.

#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static const char *kClientLib = "/jci/lib/libjcisettings_client.so";

// OEM client-lib signatures (C linkage; see header comment).
typedef int  (*connect_fn)(const char *name,
                           void (*on_connect)(int status),
                           void (*on_disconnect)(int status));
typedef int  (*disconnect_fn)(void);
typedef int  (*set_short_fn)(short value, void (*cb)(short value, int status));
typedef int  (*get_fn)(void (*cb)(short value, int status));

// The OEM convention: callback status 100 (0x64) == success/OK.
#define JCI_OK 100

// ---- async plumbing ------------------------------------------------
static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv  = PTHREAD_COND_INITIALIZER;

static int   g_conn_done = 0, g_conn_status = 0;
static int   g_get_done  = 0, g_get_status  = 0;
static short g_get_value  = 0;
static int   g_set_done  = 0, g_set_status  = 0;
static short g_set_value  = 0;

static void on_connect(int status)
{
    printf("[cb] connect    status=%d\n", status);
    pthread_mutex_lock(&g_mtx);
    g_conn_status = status; g_conn_done = 1;
    pthread_cond_broadcast(&g_cv);
    pthread_mutex_unlock(&g_mtx);
}

static void on_disconnect(int status)
{
    printf("[cb] disconnect status=%d\n", status);
}

static void on_get(short value, int status)
{
    printf("[cb] get        value=%d status=%d\n", (int)value, status);
    pthread_mutex_lock(&g_mtx);
    g_get_value = value; g_get_status = status; g_get_done = 1;
    pthread_cond_broadcast(&g_cv);
    pthread_mutex_unlock(&g_mtx);
}

static void on_set(short value, int status)
{
    printf("[cb] set        value=%d status=%d\n", (int)value, status);
    pthread_mutex_lock(&g_mtx);
    g_set_value = value; g_set_status = status; g_set_done = 1;
    pthread_cond_broadcast(&g_cv);
    pthread_mutex_unlock(&g_mtx);
}

// Wait until *done becomes non-zero, or timeout_ms elapses.
// Returns 0 if the callback fired, -1 on timeout.
static int wait_done(volatile int *done, int timeout_ms)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec  += timeout_ms / 1000;
    ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }

    int rc = 0;
    pthread_mutex_lock(&g_mtx);
    while (!*done && rc == 0) {
        rc = pthread_cond_timedwait(&g_cv, &g_mtx, &ts);
    }
    int fired = *done;
    pthread_mutex_unlock(&g_mtx);
    return fired ? 0 : -1;
}

int main(int argc, char **argv)
{
    int   do_set   = 0;
    short set_val  = 0;

    if (argc >= 2 && strcmp(argv[1], "get") != 0) {
        do_set  = 1;
        set_val = (short)atoi(argv[1]);
        if (set_val < 1 || set_val > 2) {
            fprintf(stderr,
                    "warning: GearShiftIndicator range is 1..2; you passed %d\n",
                    (int)set_val);
        }
    }

    // The JCI D-Bus layer (libjcidbus) locates each bus by reading its
    // address from an environment variable: JCI_HMI_BUS for the HMI bus
    // (where com.jci.settings is hosted) and JCI_SERVICE_BUS for the
    // service bus. sm_svclauncher sets these for every JCI service; a
    // plain shell does NOT, so BLM_SETTINGS_Client_Connect() fails
    // synchronously and returns 104. Provide the standard socket paths if
    // they're absent. Override either by exporting it before running, e.g.
    //   JCI_HMI_BUS=unix:path=/tmp/dbus_hmi_socket ./gsi_test 2
    if (!getenv("JCI_HMI_BUS")) {
        setenv("JCI_HMI_BUS", "unix:path=/tmp/dbus_hmi_socket", 0);
    }
    if (!getenv("JCI_SERVICE_BUS")) {
        setenv("JCI_SERVICE_BUS", "unix:path=/tmp/dbus_service_socket", 0);
    }
    printf("JCI_HMI_BUS=%s\n", getenv("JCI_HMI_BUS"));
    printf("JCI_SERVICE_BUS=%s\n", getenv("JCI_SERVICE_BUS"));

    // libjcisettings_client.so imports JCIDBUS_* symbols (e.g.
    // JCIDBUS_elem_get_ui32) from libjcidbus.so, but does NOT list it as a
    // NEEDED dependency — it assumes the hosting JCI process already has
    // libjcidbus.so loaded in the global symbol scope. Standalone we must
    // pull it in first, with RTLD_GLOBAL, so its symbols satisfy the
    // client lib's imports. Add more libs here if further undefined
    // symbols surface.
    static const char *const kDeps[] = { "/jci/lib/libjcidbus.so" };
    for (size_t i = 0; i < sizeof(kDeps) / sizeof(kDeps[0]); ++i) {
        if (!dlopen(kDeps[i], RTLD_NOW | RTLD_GLOBAL)) {
            fprintf(stderr, "warning: dlopen(%s) failed: %s\n",
                    kDeps[i], dlerror());
        }
    }

    void *h = dlopen(kClientLib, RTLD_NOW | RTLD_GLOBAL);
    if (!h) {
        fprintf(stderr, "dlopen(%s) failed: %s\n", kClientLib, dlerror());
        return 1;
    }

    connect_fn    Connect    = (connect_fn)   dlsym(h, "BLM_SETTINGS_Client_Connect");
    disconnect_fn Disconnect = (disconnect_fn)dlsym(h, "BLM_SETTINGS_Client_Disconnect");
    set_short_fn  SetGsi      = (set_short_fn) dlsym(h, "SETTINGS_Client_Set_GearShiftIndicator");
    get_fn        GetGsi      = (get_fn)       dlsym(h, "SETTINGS_Client_Get_GearShiftIndicator");

    if (!Connect || !SetGsi || !GetGsi) {
        fprintf(stderr,
                "dlsym failed: Connect=%p Set=%p Get=%p Disconnect=%p\n",
                (void *)Connect, (void *)SetGsi, (void *)GetGsi,
                (void *)Disconnect);
        return 1;
    }

    // The client name is requested on the bus via dbus_bus_request_name
    // (JCIDBUS acquire_name → HMI bus), so it MUST be a valid D-Bus
    // well-known name: at least two dot-separated elements, each
    // [A-Za-z_][A-Za-z0-9_]*. A bare "gsi_test" (no dot) is rejected by
    // dbus_bus_request_name, which makes BLM_SETTINGS_Client_Connect()
    // return 104 before any async callback. A dotted name fixes it.
    static const char *const kClientName = "com.jci.gsitest";
    printf("connecting to com.jci.settings as \"%s\" ...\n", kClientName);
    int rc = Connect(kClientName, on_connect, on_disconnect);
    printf("BLM_SETTINGS_Client_Connect() returned %d\n", rc);

    if (wait_done(&g_conn_done, 5000) != 0) {
        fprintf(stderr,
                "TIMEOUT waiting for connect callback — the settings service\n"
                "bus was not reachable from this process. Make sure the CMU is\n"
                "fully booted and you are running as root in a normal shell.\n");
        return 2;
    }
    if (g_conn_status != JCI_OK) {
        fprintf(stderr, "connect did not report OK (status=%d); continuing anyway\n",
                g_conn_status);
    }

    // Read current value.
    g_get_done = 0;
    printf("reading current GearShiftIndicator ...\n");
    GetGsi(on_get);
    if (wait_done(&g_get_done, 5000) == 0) {
        printf("current GearShiftIndicator = %d\n", (int)g_get_value);
    } else {
        printf("get timed out\n");
    }

    if (do_set) {
        g_set_done = 0;
        printf("setting GearShiftIndicator = %d ...\n", (int)set_val);
        SetGsi(set_val, on_set);
        if (wait_done(&g_set_done, 5000) == 0) {
            printf("set returned value=%d status=%d (%s)\n",
                   (int)g_set_value, g_set_status,
                   g_set_status == JCI_OK ? "OK" : "NOT OK");
        } else {
            printf("set timed out\n");
        }

        // Read back to confirm it stuck.
        g_get_done = 0;
        printf("reading back GearShiftIndicator ...\n");
        GetGsi(on_get);
        if (wait_done(&g_get_done, 5000) == 0) {
            printf("GearShiftIndicator is now = %d\n", (int)g_get_value);
        } else {
            printf("readback timed out\n");
        }
    }

    if (Disconnect) {
        Disconnect();
    }
    printf("done.\n");
    return 0;
}
