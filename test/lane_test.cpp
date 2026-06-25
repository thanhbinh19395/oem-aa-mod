// SPDX-License-Identifier: AGPL-3.0-or-later
//
// lane_test — sweep HUD lane-recommendation glyphs on the OEM
// com.jci.vbs.navi service, with a primer maneuver frame + a street
// strip that labels the current glyph value so you can read it off the
// head-up display.
//
// === Why dbus-c++ and not libjcivbsnaviclient ================
//
// An earlier version of this tool called the OEM C client
// (libjcivbsnaviclient.so:VBS_NAVI_SetHUDDisplayMsgReq, etc.) over a
// libjcidbus connection. It connected and queued frames but NOTHING
// rendered. The proven-working path (confirmed on-device) talks to the
// com.jci.vbs.navi service DIRECTLY through the dbus-c++ generated
// proxies — synchronous method calls — exactly like the OEM headunit
// tooling. So this version mirrors that.
//
// The device ships libdbus-c++-1 and libdbus-1 (but NOT the glib
// integration variant), so we use dbus-c++'s core DBus::BusDispatcher.
// Blocking method calls (send_blocking under the hood) don't need a
// running dispatcher loop, so we never enter() it.
//
// Each iteration calls three com.jci.vbs.navi methods, sharing one
// "sync" byte (cycled 1..7) so the HUD treats them as one generation:
//   1. tmc.SetHUD_Display_Msg2((sy))      — street strip = glyph value.
//   2. navi.SetHUDDisplayMsgReq((uqyqyy)) — primer maneuver frame.
//   3. navi.SetRecommLaneReq(((ay)))      — the lane-glyph array.
//
// NOTE: this tool includes reference/dbus/generated_cmu.h (the dbus-c++
// proxy classes for the OEM interfaces) — a local-only reference asset,
// not part of the shipping mazda/ build. It is a bench diagnostic.
//
// Usage (run on the device, as root):
//   lane_test           # sweep lane byte[1] = 0..62, 1s each, then exit
//   lane_test <start>   # start the byte[1] sweep at <start> (0..62)
//
// Lane array sent each iteration (8 bytes; server consumes 8):
//   [0]=0  [1]=sweep(0..62)  [2]=0  [3..7]=0xFF (empty slots)
// i.e. three lane slots shown — the outer two fixed at glyph 0, the
// middle one swept across the atlas so you can read each glyph.
//
// Env overrides for the primer maneuver frame:
//   HUD_MANEUVER  next-maneuver icon   (default 3 = right; 0 = blank)
//   HUD_DUNIT     distance unit        (default 1 = meters)
//   HUD_SPEED     speed-limit value    (default 0 = no speed shown)
//
// The distance field is NOT fixed: it carries the current lane-glyph
// value too (so the number shows both on the distance readout and in the
// street strip).

#include <dbus-c++/dbus.h>
#include "generated_cmu.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <unistd.h>

#define logd(fmt, ...) fprintf(stdout, "[lane_test] " fmt "\n", ##__VA_ARGS__)
#define loge(fmt, ...) fprintf(stderr, "[lane_test] " fmt "\n", ##__VA_ARGS__)

// com.jci.vbs.navi is hosted on the SERVICE bus.
#define SERVICE_BUS_ADDRESS "unix:path=/tmp/dbus_service_socket"

// Mazda HUD distance-unit enum (HudDistanceUnit in the reference HUD
// headers): 1=METERS, 2=MILES, 3=KILOMETERS, 4=YARDS, 5=FEET.
enum HudDistanceUnit : uint8_t {
    METERS = 1, MILES = 2, KILOMETERS = 3, YARDS = 4, FEET = 5
};

// Minimal proxies: derive from the generated _proxy + ObjectProxy and
// stub every pure-virtual signal handler (we send method calls only).
class NaviClient : public com::jci::vbs::navi_proxy,
                   public DBus::ObjectProxy {
public:
    NaviClient(DBus::Connection &c, const char *path, const char *name)
        : DBus::ObjectProxy(c, path, name) {}

    virtual void FuelTypeResp(const uint8_t &) override {}
    virtual void HUDResp(const uint8_t &) override {}
    virtual void TSRResp(const uint8_t &) override {}
    virtual void GccConfigMgmtResp(
        const ::DBus::Struct< std::vector< uint8_t > > &) override {}
    virtual void TSRFeatureMode(const uint8_t &) override {}
};

class TMCClient : public com::jci::vbs::navi::tmc_proxy,
                  public DBus::ObjectProxy {
public:
    TMCClient(DBus::Connection &c, const char *path, const char *name)
        : DBus::ObjectProxy(c, path, name) {}

    virtual void ServiceListResponse(
        const ::DBus::Struct< uint8_t, std::vector< uint8_t >,
                              std::vector< uint8_t >, std::vector< uint8_t >,
                              std::vector< uint8_t >,
                              std::vector< uint8_t > > &) override {}
    virtual void ResponseToTMCSelection(
        const uint8_t &, const uint8_t &, const uint8_t &, const uint8_t &,
        const uint8_t &, const uint8_t &, const uint8_t &) override {}
};

static DBus::BusDispatcher g_dispatcher;

static long env_long(const char *name, long dflt)
{
    const char *s = getenv(name);
    return (s && *s) ? strtol(s, NULL, 0) : dflt;
}

int main(int argc, char **argv)
{
    int start = 0;
    if (argc >= 2) start = atoi(argv[1]);
    if (start < 0)  start = 0;
    if (start > 62) start = 62;

    uint32_t maneuver = (uint32_t)env_long("HUD_MANEUVER", 3);
    uint8_t  dunit    = (uint8_t) env_long("HUD_DUNIT", METERS);
    uint16_t speed    = (uint16_t)env_long("HUD_SPEED", 0);

    DBus::default_dispatcher = &g_dispatcher;

    DBus::Connection *service_bus = NULL;
    NaviClient *navi = NULL;
    TMCClient  *tmc  = NULL;
    try {
        // Private connection; register_bus() does the Hello handshake to
        // get a unique name (we never acquire a well-known name). The
        // proxies hold their own refs to the connection, so it stays
        // alive for their lifetime.
        service_bus = new DBus::Connection(SERVICE_BUS_ADDRESS, false);
        service_bus->register_bus();
        navi = new NaviClient(*service_bus, "/com/jci/vbs/navi",
                              "com.jci.vbs.navi");
        tmc  = new TMCClient(*service_bus, "/com/jci/vbs/navi",
                             "com.jci.vbs.navi");
    } catch (DBus::Error &e) {
        loge("connect to SERVICE bus failed: %s: %s", e.name(), e.message());
        return 1;
    }

    logd("connected to com.jci.vbs.navi on the service bus");
    logd("primer: maneuver=0x%x dunit=%u speed=%u (distance = glyph value)",
         maneuver, dunit, speed);
    logd("sweeping lane byte[1] %d..62 (then exit); bytes[0]=0, bytes[2]=0, "
         "bytes[3..7]=0xFF, 1s apart. Ctrl-C to stop.", start);

    uint8_t sync = 0;  // cycles 1..7 so each frame is a new generation

    for (int v = start; v <= 62; ++v) {
        sync = (uint8_t)((sync % 7) + 1);

        char street[64];
        snprintf(street, sizeof(street), "lane %d  0x%02X", v, v);

        // 1. Street strip — show the current glyph value as text.
        ::DBus::Struct< std::string, uint8_t > gpd;
        gpd._1 = street;
        gpd._2 = sync;

        // 2. Primer maneuver frame (uqyqyy). The distance field carries
        // the glyph value so it shows on the HUD distance readout too.
        ::DBus::Struct< uint32_t, uint16_t, uint8_t,
                        uint16_t, uint8_t, uint8_t > md;
        md._1 = maneuver;        // nextManeuverInfo (icon)
        md._2 = (uint16_t)v * 10;     // distanceValue = current glyph value
        md._3 = dunit;           // distanceUnit
        md._4 = speed;           // displaySpeedLimit
        md._5 = 0;               // displaySpeedUnit
        md._6 = sync;            // text_ID3 (sync)

        // 3. Lane array — 8 bytes. Only the first three slots are shown:
        // [0]=0, [1]=swept glyph value, [2]=0; [3..7]=0xFF (empty).
        std::vector< uint8_t > bytes(8, 0xFF);
        bytes[0] = 0;
        bytes[1] = 1;
        bytes[2] = (uint8_t)v;
        ::DBus::Struct< std::vector< uint8_t > > lane;
        lane._1 = bytes;

        try {
            tmc->SetHUD_Display_Msg2(gpd);
            navi->SetHUDDisplayMsgReq(md);
            uint8_t r = navi->SetRecommLaneReq(lane);
            logd("v=%3d (0x%02x) sync=%u street=\"%s\" "
                 "lanes=[%u,%u,%u,%u,%u,%u,%u,%u] laneResp=%u",
                 v, v, sync, street,
                 bytes[0], bytes[1], bytes[2], bytes[3],
                 bytes[4], bytes[5], bytes[6], bytes[7], r);
        } catch (DBus::Error &e) {
            loge("send failed: %s: %s", e.name(), e.message());
            break;
        }

        usleep(1000 * 1000);  // 1 s between iterations
    }

    delete navi;
    delete tmc;
    delete service_bus;
    return 0;
}
