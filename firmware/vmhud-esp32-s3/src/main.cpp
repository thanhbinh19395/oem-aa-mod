// VietMap Live → BLE HUD emulator → USB CDC "SPLIM <km/h>" for Mazda CMU.
// Supports Headless (Super Mini) and Display (LilyGO T-Display S3 or I2C OLED).
// Codec: lib/vmhud/hud_codec.h (AGPL-3.0, shared with mzd-vietnav-hud).

#include <NimBLEDevice.h>
#include <Preferences.h>
#include <atomic>
#include <mutex>
#include "mbedtls/md5.h"
#include "hud_codec.h"
#include "display_ui.h"

static String g_name = "VIETMAP-HUD";
static String g_model = "H50";
static String g_hw = "1.0.0";
static String g_fw = "1.0.2";
static String g_protocol = "2.2.2";
static String g_sid = "VMS3SM000001";
static String g_adv = "BOTH";

static Preferences g_store;
static std::mutex g_cfgMutex;
static std::mutex g_serialMutex;
static std::atomic<bool> g_connected{false};
static std::atomic<bool> g_hudInfoOk{false};
static std::atomic<uint16_t> g_conn{BLE_HS_CONN_HANDLE_NONE};
static NimBLECharacteristic *g_h1 = nullptr;
static NimBLECharacteristic *g_h50 = nullptr;
static std::atomic<bool> g_subH1{false};
static std::atomic<bool> g_subH50{false};
static vmhud::Receiver<4096> g_rxH1, g_rxH50;

static std::atomic<uint8_t> g_postedLimit{0};
static std::atomic<uint16_t> g_currentSpeed{0};
static std::atomic<uint16_t> g_camDist{0};
static std::atomic<uint8_t> g_camType{0};
static std::mutex g_roadMutex;
static String g_currentRoad = "";

static std::atomic<uint32_t> g_camPktCount{0};
static std::atomic<uint32_t> g_navPktCount{0};
static std::atomic<uint32_t> g_hbPktCount{0};
static std::atomic<uint32_t> g_splimTxCount{0};
static std::atomic<uint32_t> g_lastSplimMs{0};

static std::atomic<bool> g_cmuConnected{false};
static std::atomic<uint32_t> g_lastCmuRxMs{0};
static char g_cmuRxBuf[64];
static size_t g_cmuRxLen = 0;

static std::atomic<uint8_t> g_lastEmittedLimit{0};
static std::atomic<bool> g_splimForceEmit{false};
static uint32_t g_lastKeepaliveMs = 0;
static std::atomic<bool> g_displayDirty{true};
static uint32_t g_lastDisplayMs = 0;

// CMU read_splim() rejects entries older than 8 s — re-emit every 2 s while
// a valid limit is held so the HUD sign does not go stale between CAM frames.
static const uint32_t kKeepaliveMs = 2000;

static void serialWriteLine(const char *line, size_t len) {
  std::lock_guard<std::mutex> guard(g_serialMutex);
  Serial.write(reinterpret_cast<const uint8_t *>(line), len);
}

#ifdef VMHUD_SERIAL_DEBUG
static void debugLogf(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0) {
    size_t len = (size_t)n < sizeof(buf) ? (size_t)n : sizeof(buf) - 1;
    serialWriteLine(buf, len);
  }
}
#else
#define debugLogf(...) ((void)0)
#endif

static bool sdkChecksum(const uint8_t *p, size_t n, const uint8_t *uid, uint8_t *out) {
  uint8_t digest[16];
  mbedtls_md5_context ctx;
  mbedtls_md5_init(&ctx);
  mbedtls_md5_starts_ret(&ctx);
  mbedtls_md5_update_ret(&ctx, p, n);
  mbedtls_md5_update_ret(&ctx, uid, 16);
  mbedtls_md5_finish_ret(&ctx, digest);
  mbedtls_md5_free(&ctx);
  vmhud::put32(out, vmhud::be32(digest) + vmhud::be32(digest + 4) + vmhud::be32(digest + 8) +
                         vmhud::be32(digest + 12));
  return true;
}

static String textUtf8(const uint8_t *p, size_t n) {
  String value;
  value.reserve(n);
  for (size_t i = 0; i < n && p[i]; i++) {
    value += (char)p[i];
  }
  return value;
}

// Called strictly from loop() so SPLIM lines are never interleaved with other tasks
// and the NimBLE host task never stalls on USB CDC TX timeouts.
static void emitSplim(uint8_t limit) {
  if (limit < 5 || limit > 200) {
    return;
  }
  char buf[24];
  int n = snprintf(buf, sizeof(buf), "SPLIM %u\n", (unsigned)limit);
  if (n <= 0 || (size_t)n >= sizeof(buf)) {
    return;
  }
  const uint32_t now = millis();
  g_lastEmittedLimit.store(limit, std::memory_order_relaxed);
  g_lastKeepaliveMs = now;
  g_lastSplimMs.store(now, std::memory_order_relaxed);
  g_splimTxCount.fetch_add(1, std::memory_order_relaxed);
  g_displayDirty.store(true, std::memory_order_relaxed);
  serialWriteLine(buf, (size_t)n);
}

static uint8_t postedLimitFromCamera(const vmhud::Frame &frame, uint16_t &outDist, uint8_t &outType) {
  outDist = 0;
  outType = 0;
  if (frame.command != 1 || frame.size < 4 || (frame.size % 4) != 0) {
    return 0;
  }
  uint8_t pick = 0;
  uint16_t pickDist = 0xFFFF;
  uint8_t pickType = 0;

  for (size_t i = 0; i < frame.size; i += 4) {
    const uint8_t type = frame.payload[i];
    const uint16_t dist = vmhud::be16(frame.payload + i + 1);
    const uint8_t lim = frame.payload[i + 3];
    if (lim < 5 || lim > 200) {
      continue;
    }
    if (dist >= 60000) {
      continue;
    }
    if (dist < pickDist) {
      pickDist = dist;
      pick = lim;
      pickType = type;
    }
  }
  if (pick) {
    outDist = pickDist;
    outType = pickType;
    return pick;
  }
  for (size_t i = 0; i < frame.size; i += 4) {
    const uint8_t lim = frame.payload[i + 3];
    if (lim >= 5 && lim <= 200) {
      outDist = vmhud::be16(frame.payload + i + 1);
      outType = frame.payload[i];
      return lim;
    }
  }
  return 0;
}

static void notifyInfo(const vmhud::Frame &request, NimBLECharacteristic *route) {
  const bool subscribed = route == g_h1 ? g_subH1.load(std::memory_order_relaxed)
                                        : g_subH50.load(std::memory_order_relaxed);
  const uint16_t connHandle = g_conn.load(std::memory_order_relaxed);
  if (!g_connected.load(std::memory_order_relaxed) || !subscribed ||
      connHandle == BLE_HS_CONN_HANDLE_NONE) {
    debugLogf("[HUD] HUDInfo before notify subscribe\n");
    return;
  }
  String info;
  {
    std::lock_guard<std::mutex> guard(g_cfgMutex);
    info = "MODEL:" + g_model + ",HW:" + g_hw + ",FW:" + g_fw + ",PROTOCOL:" + g_protocol +
           ",OBDV:0_1.0,SID:" + g_sid;
  }
  uint8_t frame[256];
  const size_t count =
      vmhud::build(0x0e, (const uint8_t *)info.c_str(), info.length(), request.timestamp, frame,
                   sizeof(frame), sdkChecksum, request.legacy);
  if (!count) {
    debugLogf("[HUD] HUDInfo build failed\n");
    return;
  }
  for (size_t offset = 0; offset < count; offset += 20) {
    const size_t chunk = count - offset < 20 ? count - offset : 20;
    if (!route->notify(frame + offset, chunk, connHandle)) {
      debugLogf("[HUD] notify failed\n");
      return;
    }
  }
  g_hudInfoOk.store(true, std::memory_order_relaxed);
  g_displayDirty.store(true, std::memory_order_relaxed);
  debugLogf("[HUD] HUDInfo %u bytes\n", (unsigned)count);
}

static void onFrame(const vmhud::Frame &frame, void *user) {
  debugLogf("[RX] CMD 0x%02X plen=%u\n", frame.command, (unsigned)frame.size);

  if (frame.command == 0x0e) {
    notifyInfo(frame, (NimBLECharacteristic *)user);
    return;
  }

  if (frame.command == 0 || frame.command == 0x40) {
    vmhud::Nav nav;
    if (!vmhud::navigation(frame, nav)) {
      return;
    }
    g_navPktCount.fetch_add(1, std::memory_order_relaxed);
    g_currentSpeed.store(nav.speed, std::memory_order_relaxed);
    const String current = textUtf8(nav.current, nav.currentSize);
    {
      std::lock_guard<std::mutex> guard(g_roadMutex);
      g_currentRoad = current;
    }
    g_displayDirty.store(true, std::memory_order_relaxed);
    debugLogf("[NAV] speed=%u cur=\"%s\"\n", (unsigned)nav.speed, current.c_str());
    return;
  }

  // Camera and speed limit (CMD 0x01, multiple of 4 bytes)
  if (frame.command == 1 && frame.size >= 4 && (frame.size % 4) == 0) {
    g_camPktCount.fetch_add(1, std::memory_order_relaxed);
    uint16_t dist = 0;
    uint8_t type = 0;
    const uint8_t lim = postedLimitFromCamera(frame, dist, type);
    g_camDist.store(dist, std::memory_order_relaxed);
    g_camType.store(type, std::memory_order_relaxed);
    g_postedLimit.store(lim, std::memory_order_release);
    if (lim >= 5 && lim <= 200) {
      // Request prompt USB CDC transmission from loop() (<= 20 ms latency)
      g_splimForceEmit.store(true, std::memory_order_release);
    }
    g_displayDirty.store(true, std::memory_order_relaxed);
    debugLogf("[CAM] type=%u dist=%u limit=%u\n", (unsigned)type, (unsigned)dist, (unsigned)lim);
    return;
  }

  if (frame.command == 2 && (frame.size == 1 || frame.size == 2)) {
    g_hbPktCount.fetch_add(1, std::memory_order_relaxed);
    const uint16_t spd = frame.size == 1 ? frame.payload[0] : vmhud::be16(frame.payload);
    g_currentSpeed.store(spd, std::memory_order_relaxed);
    g_displayDirty.store(true, std::memory_order_relaxed);
    debugLogf("[HB] speed=%u\n", (unsigned)spd);
  }
}

static void startConnectable() { NimBLEDevice::startAdvertising(0); }

struct ServerCB : NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &conn) override {
    if (g_connected.load(std::memory_order_relaxed)) {
      server->disconnect(conn.getConnHandle());
      return;
    }
    g_conn.store(conn.getConnHandle(), std::memory_order_release);
    g_connected.store(true, std::memory_order_release);
    g_hudInfoOk.store(false, std::memory_order_relaxed);
    g_subH1.store(false, std::memory_order_relaxed);
    g_subH50.store(false, std::memory_order_relaxed);
    g_rxH1.reset();
    g_rxH50.reset();
    g_displayDirty.store(true, std::memory_order_relaxed);
    debugLogf("[BLE] connected\n");
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &conn, int reason) override {
    if (conn.getConnHandle() != g_conn.load(std::memory_order_acquire)) {
      return;
    }
    g_connected.store(false, std::memory_order_release);
    g_hudInfoOk.store(false, std::memory_order_relaxed);
    g_conn.store(BLE_HS_CONN_HANDLE_NONE, std::memory_order_release);
    g_subH1.store(false, std::memory_order_relaxed);
    g_subH50.store(false, std::memory_order_relaxed);
    g_rxH1.reset();
    g_rxH50.reset();
    g_postedLimit.store(0, std::memory_order_release);
    g_camDist.store(0, std::memory_order_relaxed);
    g_camType.store(0, std::memory_order_relaxed);
    g_lastEmittedLimit.store(0, std::memory_order_relaxed);
    g_splimForceEmit.store(false, std::memory_order_relaxed);
    g_displayDirty.store(true, std::memory_order_relaxed);
    debugLogf("[BLE] disconnected reason=%d\n", reason);
    startConnectable();
  }
};

struct SubscribeCB : NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *c, NimBLEConnInfo &conn, uint16_t sub) override {
    if (conn.getConnHandle() != g_conn.load(std::memory_order_acquire)) {
      return;
    }
    if (c == g_h1) {
      g_subH1.store((sub & 1) != 0, std::memory_order_relaxed);
    } else {
      g_subH50.store((sub & 1) != 0, std::memory_order_relaxed);
    }
    g_displayDirty.store(true, std::memory_order_relaxed);
    debugLogf("[BLE] notify sub=0x%04X\n", sub);
  }
};

struct WriteCB : NimBLECharacteristicCallbacks {
  NimBLECharacteristic *route;
  const char *label;
  vmhud::Receiver<4096> *rx;
  WriteCB(NimBLECharacteristic *r, const char *l, vmhud::Receiver<4096> *b) : route(r), label(l), rx(b) {}
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &conn) override {
    if (conn.getConnHandle() != g_conn.load(std::memory_order_acquire)) {
      return;
    }
    const std::string data = c->getValue();
    debugLogf("[W:%s] %u bytes\n", label, (unsigned)data.size());
    rx->feed((const uint8_t *)data.data(), data.size(), sdkChecksum, onFrame, route);
  }
};

template <typename Advertisement>
static void fillServiceUuids(Advertisement &data) {
  if (g_adv == "BOTH" || g_adv == "FFF0") {
    data.addServiceUUID("FFF0");
  }
  if (g_adv == "BOTH" || g_adv == "FFFF") {
    data.addServiceUUID("FFFF");
  }
}

static void applyAdvertising() {
  const std::string name(g_name.c_str());
  NimBLEDevice::setDeviceName(name);

  NimBLEAdvertisementData primary;
  primary.setFlags(0x06);
  fillServiceUuids(primary);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(name);

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->setAdvertisementData(primary);
  adv->setScanResponseData(scanResp);
  adv->enableScanResponse(true);
  startConnectable();
  Serial.printf("[ADV] name=%s (scan response) adv=%s\n", g_name.c_str(), g_adv.c_str());
}

void setup() {
  Serial.begin(115200);
  // Native USB CDC blocks each write up to 100 ms when the host is attached but not
  // reading (CMU daemon not running yet). Our prints run on the BLE host task, so cap
  // the stall; a dropped line is harmless because SPLIM is re-sent every 2 s.
  Serial.setTxTimeoutMs(10);
  delay(400);
  Serial.println("\n[BOOT] vmhud-esp32-s3 — VietMap BLE HUD → USB SPLIM");

  displayInit();

  if (g_store.begin("vmhud", false)) {
    g_name = g_store.getString("name", g_name);
    g_model = g_store.getString("model", g_model);
    g_hw = g_store.getString("hw", g_hw);
    g_fw = g_store.getString("fw", g_fw);
    g_protocol = g_store.getString("protocol", g_protocol);
    g_sid = g_store.getString("sid", g_sid);
    g_adv = g_store.getString("adv", g_adv);
  }

  NimBLEDevice::init(std::string(g_name.c_str()));
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  NimBLEService *h1 = server->createService("FFF0");
  g_h1 = h1->createCharacteristic("FFF1", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_h1->setCallbacks(new SubscribeCB());
  const uint32_t writable = NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR;
  h1->createCharacteristic("FFF2", writable)->setCallbacks(new WriteCB(g_h1, "FFF2", &g_rxH1));
  h1->createCharacteristic("FFF3", writable)->setCallbacks(new WriteCB(g_h1, "FFF3", &g_rxH1));
  h1->createCharacteristic("FFF4", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  h1->start();

  NimBLEService *h50 = server->createService("FFFF");
  g_h50 = h50->createCharacteristic("1234", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_h50->setCallbacks(new SubscribeCB());
  h50->createCharacteristic("9ABC", writable)->setCallbacks(new WriteCB(g_h50, "9ABC", &g_rxH50));
  h50->start();

  applyAdvertising();
}

static void processCmuSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      g_cmuRxBuf[g_cmuRxLen] = '\0';
      if (g_cmuRxLen > 0) {
        // Any handshake/ACK message from CMU splim service: CMU:OK, CMU:ACK, CMU:HB, CMU:PONG
        if (strncmp(g_cmuRxBuf, "CMU:", 4) == 0) {
          g_lastCmuRxMs = millis();
          if (!g_cmuConnected.load(std::memory_order_relaxed)) {
            g_cmuConnected.store(true, std::memory_order_relaxed);
            g_displayDirty.store(true, std::memory_order_relaxed);
            debugLogf("[CMU] splim service connected: %s\n", g_cmuRxBuf);
          }
        }
      }
      g_cmuRxLen = 0;
    } else if (g_cmuRxLen + 1 < sizeof(g_cmuRxBuf)) {
      g_cmuRxBuf[g_cmuRxLen++] = c;
    } else {
      g_cmuRxLen = 0;
    }
  }
}

void loop() {
  const uint32_t now = millis();
  const uint8_t lim = g_postedLimit.load();

  // Read incoming serial packets from CMU splim service
  processCmuSerial();

  // Check timeout: if no heartbeat/ACK from CMU service for > 4.5 seconds
  if (g_cmuConnected.load(std::memory_order_relaxed)) {
    if (now - g_lastCmuRxMs.load(std::memory_order_relaxed) > 4500) {
      g_cmuConnected.store(false, std::memory_order_relaxed);
      g_displayDirty.store(true, std::memory_order_relaxed);
      debugLogf("[CMU] splim service disconnected (timeout)\n");
    }
  }

  // SPLIM USB serial keepalive / probe emission to CMU (single-task owned).
  if (lim >= 5 && lim <= 200) {
    const bool forceEmit = g_splimForceEmit.exchange(false, std::memory_order_acquire);
    if (forceEmit || lim != g_lastEmittedLimit.load(std::memory_order_relaxed) ||
        (now - g_lastKeepaliveMs) >= kKeepaliveMs) {
      emitSplim(lim);
    }
  } else {
    g_splimForceEmit.store(false, std::memory_order_relaxed);
    // When no limit is held from VietMap, poll CMU service every 2 s
    if ((now - g_lastKeepaliveMs) >= kKeepaliveMs) {
      g_lastKeepaliveMs = now;
      serialWriteLine("PING\n", 5);
    }
  }

  // Display update (refresh every 500ms or when state changes)
#if VMHUD_HAS_DISPLAY
  if (g_displayDirty.exchange(false, std::memory_order_relaxed) || (now - g_lastDisplayMs >= 500)) {
    g_lastDisplayMs = now;

    DisplayState st;
    st.bleConnected = g_connected.load(std::memory_order_relaxed);
    st.hudInfoOk = g_hudInfoOk.load(std::memory_order_relaxed);
    st.notifySubscribed =
        g_subH1.load(std::memory_order_relaxed) || g_subH50.load(std::memory_order_relaxed);
    st.cmuConnected = g_cmuConnected.load(std::memory_order_relaxed);
    st.postedLimit = lim;
    st.camDist = g_camDist.load(std::memory_order_relaxed);
    st.camType = g_camType.load(std::memory_order_relaxed);
    st.currentSpeed = g_currentSpeed.load(std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> guard(g_roadMutex);
      st.currentRoad = g_currentRoad;
    }
    st.lastEmittedLimit = g_lastEmittedLimit.load(std::memory_order_relaxed);
    st.splimTxCount = g_splimTxCount.load(std::memory_order_relaxed);
    st.lastSplimMs = g_lastSplimMs.load(std::memory_order_relaxed);
    st.camPktCount = g_camPktCount.load(std::memory_order_relaxed);
    st.navPktCount = g_navPktCount.load(std::memory_order_relaxed);
    st.hbPktCount = g_hbPktCount.load(std::memory_order_relaxed);
    st.uptimeSec = now / 1000;

    displayUpdate(st);
  }
#endif

  delay(20);
}
