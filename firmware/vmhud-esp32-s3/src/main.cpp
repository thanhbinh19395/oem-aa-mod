// VietMap Live (iPhone) → BLE HUD emulator → USB CDC "SPLIM <km/h>" + "NAV <icon> <dist_m> <road>" for Mazda CMU.
// Supports Headless (Super Mini) and Display (LilyGO T-Display S3 or I2C OLED).
// Codec: lib/vmhud/hud_codec.h + lib/vmhud/arrow_classifier.h (AGPL-3.0).

#include <NimBLEDevice.h>
#include <Preferences.h>
#include <atomic>
#include <mutex>
#include "mbedtls/md5.h"
#include "hud_codec.h"
#include "arrow_classifier.h"
#include "speed_sign_classifier.h"
#include "traffic_sign_classifier.h"
#include "cph_telemetry.h"
#include "display_ui.h"

static constexpr uint8_t  kAdvInstHud      = 0;
static constexpr uint8_t  kAdvInstWake     = 1;
static constexpr uint32_t kVmUnsubGraceMs  = 1500;
static constexpr uint32_t kCphKeepaliveMs  = 5000;
static constexpr uint32_t kSignHoldStaleMs = 15000;
static constexpr uint32_t kSignDistStaleMs = 8000;

#ifdef VMHUD_BOARD_C3
static constexpr size_t kRxBufCap = 6144;
#else
static constexpr size_t kRxBufCap = 8192;
#endif
static constexpr size_t kMaxJpegBytes = 6144;
static constexpr size_t kMaxAlertImgBytes = 4096;
static constexpr size_t kMaxRoadBytes = 192;

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
static NimBLECharacteristic *g_cph = nullptr;
static std::atomic<bool> g_subH1{false};
static std::atomic<bool> g_subH50{false};
static std::atomic<bool> g_subCph{false};
static std::atomic<uint16_t> g_cphConn{BLE_HS_CONN_HANDLE_NONE};
static std::atomic<bool> g_cphForceNotify{false};
static std::atomic<bool> g_vmUnsubWatchdogActive{false};
static std::atomic<uint32_t> g_vmUnsubWatchdogMs{0};
static bool g_wakeAdvRunning = false;
static vmhud::Receiver<kRxBufCap> g_rxH1;
static vmhud::Receiver<512> g_rxH50;  // H50 GATT stub (option 2 future work); saves ~7.5 KB SRAM

static std::atomic<uint8_t> g_postedLimit{0};
static std::atomic<uint16_t> g_currentSpeed{0};
static std::atomic<uint16_t> g_camDist{0};
static std::atomic<uint8_t> g_camType{0};
static std::atomic<uint8_t>  g_cmd01SignCode{0};
static std::atomic<uint16_t> g_cmd01SignDist{0};
static std::atomic<uint32_t> g_cmd01SignRxMs{0};

// Upcoming ("next") speed limit detection from CMD 0x01 (2-slot), CMD 0x41, and CMD 0x08 (JPEG sign).
static std::atomic<uint8_t>  g_camNextLimit{0};
static std::atomic<uint16_t> g_camNextDist{0};
static std::atomic<uint32_t> g_camNextRxMs{0};
static std::atomic<uint8_t>  g_customNextLimit{0};
static std::atomic<uint16_t> g_customNextDist{0};
static std::atomic<uint32_t> g_customNextRxMs{0};
static std::atomic<uint16_t> g_customSignDist{0};
static std::atomic<uint32_t> g_customSignRxMs{0};
static std::atomic<uint32_t> g_textSignRxMs{0};

// CMD 0x08 / CMD 0x41 alert JPEG & CMD 0x06 raw text handoff (BLE task -> loop() task).
struct PendingAlertImg {
  uint8_t jpeg[kMaxAlertImgBytes];
  size_t  len = 0;
};
static std::mutex g_alertImgMutex;
static PendingAlertImg g_pendingAlertImg;
static char g_overlaySignText[vmhud::kMaxOverlayTextBytes + 1] = {0};
static std::atomic<bool> g_alertImgDirty{false};
static std::atomic<bool> g_alertImgCancel{false};
static std::atomic<bool> g_nextResetReq{false};

// loop()-owned upcoming speed-limit edge-trigger latch & prohibition sign state.
static uint8_t  g_imgNextLimit = 0;
static uint32_t g_imgNextRxMs = 0;
static uint8_t  g_imgSignCode = vmhud::kSignNone;
static uint32_t g_imgSignRxMs = 0;
static vmhud::CphTelemetryState g_lastCphBuiltState;
static vmhud::CphTelemetryState g_lastCphNotified;
static uint8_t  g_lastCphWire[vmhud::kCphMaxFrameBytes] = {0};
static size_t   g_lastCphWireLen = 0;
static bool     g_cphWireInit = false;
static uint32_t g_lastCphNotifyMs = 0;
static bool     g_cphOverspeed = false;
static uint32_t g_lastAlertImgHash = 0;
static size_t   g_lastAlertImgLen = 0;
static uint32_t g_nextAlertId = 0;
static uint8_t  g_latchedNextLimit = 0;
static uint32_t g_nextAbsentSinceMs = 0;
static uint32_t g_lastNextEmitMs = 0;
static uint16_t g_lastNextEmitDist = 0xFFFFu;
static constexpr uint32_t kNextSourceStaleMs = 5000;
static constexpr uint32_t kNextClearDebounceMs = 3000;
static constexpr uint32_t kNextReemitMs = 1000;
#if VMHUD_HAS_DISPLAY
static std::mutex g_roadMutex;
static String g_currentRoad = "";
#endif

// VietMap BLE turn-by-turn navigation handoff (BLE task -> loop() task).
// JPEG decode runs exclusively on loop() (large stack) so NimBLE task stays tiny & fast.
struct PendingNav {
  bool     active = false;
  uint32_t distance = 0;
  char     road[kMaxRoadBytes + 1] = {0};
  uint8_t  jpeg[kMaxJpegBytes];
  size_t   jpegLen = 0;
  bool     hasNewJpeg = false;
};
static std::mutex g_navMutex;
static PendingNav g_pendingNav;
static std::atomic<bool> g_navDirty{false};
static std::atomic<bool> g_navOffReq{false};
static std::atomic<uint32_t> g_lastNavRxMs{0};

// loop()-owned active nav state & dedup
static bool     g_navGuiding = false;
static uint8_t  g_navIcon = 0;
static uint32_t g_navDist = 0;
static char     g_navRoad[kMaxRoadBytes + 1] = {0};
static uint8_t  g_lastEmitNavIcon = 0;
static uint32_t g_lastEmitNavDist = 0xFFFFFFFFu;
static char     g_lastEmitNavRoad[kMaxRoadBytes + 1] = {0};
static uint32_t g_lastNavEmitMs = 0;
static uint32_t g_lastJpegHash = 0;
static size_t   g_lastJpegLen = 0;

static uint32_t fnv1a32(const uint8_t *p, size_t n) {
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < n; ++i) h = (h ^ p[i]) * 16777619u;
  return h;
}

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

// CMU read_splim() rejects entries older than 8 s and read_vmnav() rejects > 6 s.
static const uint32_t kKeepaliveMs = 2000;
static const uint32_t kNavKeepaliveMs = 1500;
static const uint32_t kNavTimeoutMs = 6000;

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

#if VMHUD_HAS_DISPLAY
static String textUtf8(const uint8_t *p, size_t n) {
  String value;
  value.reserve(n);
  for (size_t i = 0; i < n && p[i]; i++) {
    value += (char)p[i];
  }
  return value;
}
#endif

// Sanitize UTF-8 road name into single-line NUL-terminated buffer without
// splitting multi-byte UTF-8 sequences. Strips CR/LF/control chars and trims ends.
static void sanitizeRoadUtf8(const uint8_t *p, size_t n, char *out, size_t cap) {
  if (!out || cap == 0) return;
  size_t o = 0;
  if (p && n > 0) {
    for (size_t i = 0; i < n && p[i] && o + 1 < cap; ++i) {
      uint8_t c = p[i];
      if (c < 0x20 || c == 0x7F) {
        if (o > 0 && out[o - 1] != ' ') out[o++] = ' ';
      } else {
        out[o++] = (char)c;
      }
    }
    // Back off if truncated mid-UTF-8 sequence.
    if (o > 0 && o + 1 == cap) {
      size_t k = o;
      while (k > 0 && (static_cast<uint8_t>(out[k - 1]) & 0xC0u) == 0x80u) --k;
      if (k > 0) {
        uint8_t lead = static_cast<uint8_t>(out[k - 1]);
        size_t need = (lead < 0x80u) ? 1u : ((lead & 0xE0u) == 0xC0u) ? 2u :
                      ((lead & 0xF0u) == 0xE0u) ? 3u : 4u;
        if (o - (k - 1) < need) o = k - 1;
      }
    }
  }
  while (o > 0 && out[o - 1] == ' ') --o;
  size_t leadSpaces = 0;
  while (leadSpaces < o && out[leadSpaces] == ' ') ++leadSpaces;
  if (leadSpaces > 0) {
    o -= leadSpaces;
    memmove(out, out + leadSpaces, o);
  }
  out[o] = '\0';
}

// Called strictly from loop() so NAV lines never interleave with SPLIM lines.
static void emitNav(uint8_t icon, uint32_t distM, const char *road) {
  if (icon == 0 || icon > 60) return;
  if (distM > 999000u) distM = 999000u;
  char buf[kMaxRoadBytes + 48];
  int n;
  if (road && road[0]) {
    n = snprintf(buf, sizeof(buf), "NAV %u %lu %s\n",
                 (unsigned)icon, (unsigned long)distM, road);
  } else {
    n = snprintf(buf, sizeof(buf), "NAV %u %lu\n",
                 (unsigned)icon, (unsigned long)distM);
  }
  if (n <= 0 || (size_t)n >= sizeof(buf)) return;
  g_lastEmitNavIcon = icon;
  g_lastEmitNavDist = distM;
  strncpy(g_lastEmitNavRoad, road ? road : "", kMaxRoadBytes);
  g_lastEmitNavRoad[kMaxRoadBytes] = '\0';
  g_lastNavEmitMs = millis();
  serialWriteLine(buf, (size_t)n);
}

static void emitNavOff() {
  static const char kNavOff[] = "NAV OFF\n";
  g_lastEmitNavIcon = 0;
  g_lastEmitNavDist = 0xFFFFFFFFu;
  g_lastEmitNavRoad[0] = '\0';
  g_lastNavEmitMs = millis();
  serialWriteLine(kNavOff, sizeof(kNavOff) - 1);
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

// Called strictly from loop() so NEXTSPLIM lines never interleave with SPLIM/NAV lines.
static void emitNextSplim(uint32_t alertId, uint8_t limit, uint16_t distM) {
  if (alertId == 0 || limit < 5 || limit > 200) {
    return;
  }
  char buf[40];
  int n = snprintf(buf, sizeof(buf), "NEXTSPLIM %lu %u %u\n",
                   (unsigned long)alertId, (unsigned)limit, (unsigned)distM);
  if (n <= 0 || (size_t)n >= sizeof(buf)) {
    return;
  }
  g_lastNextEmitMs = millis();
  g_lastNextEmitDist = distM;
  serialWriteLine(buf, (size_t)n);
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

    const bool navIdle = (nav.distance == 0xFFFFFFFFu) || (nav.direction == 0xFFu);
    if (navIdle) {
      {
        std::lock_guard<std::mutex> guard(g_navMutex);
        g_pendingNav.active = false;
        g_pendingNav.hasNewJpeg = false;
        g_pendingNav.jpegLen = 0;
        g_pendingNav.road[0] = '\0';
      }
      g_navDirty.store(true, std::memory_order_release);
#if VMHUD_HAS_DISPLAY
      const String current = textUtf8(nav.current, nav.currentSize);
      {
        std::lock_guard<std::mutex> guard(g_roadMutex);
        g_currentRoad = current;
      }
#endif
      g_displayDirty.store(true, std::memory_order_relaxed);
      debugLogf("[NAV] idle/end (dir=0x%02X dist=0x%lX)\n",
                (unsigned)nav.direction, (unsigned long)nav.distance);
      return;
    }

    char roadBuf[kMaxRoadBytes + 1];
    if (nav.next && nav.nextSize > 0) {
      sanitizeRoadUtf8(nav.next, nav.nextSize, roadBuf, sizeof(roadBuf));
    } else {
      roadBuf[0] = '\0';
    }
    if (!roadBuf[0] && nav.current && nav.currentSize > 0) {
      sanitizeRoadUtf8(nav.current, nav.currentSize, roadBuf, sizeof(roadBuf));
    }
    {
      std::lock_guard<std::mutex> guard(g_navMutex);
      g_pendingNav.active = true;
      g_pendingNav.distance = nav.distance;
      strncpy(g_pendingNav.road, roadBuf, kMaxRoadBytes);
      g_pendingNav.road[kMaxRoadBytes] = '\0';
      if (nav.image && nav.imageSize >= 16) {
        if (nav.imageSize <= kMaxJpegBytes) {
          memcpy(g_pendingNav.jpeg, nav.image, nav.imageSize);
          g_pendingNav.jpegLen = nav.imageSize;
          g_pendingNav.hasNewJpeg = true;
        } else {
          debugLogf("[ARROW] jpeg oversize %u > %u\n",
                    (unsigned)nav.imageSize, (unsigned)kMaxJpegBytes);
        }
      }
    }
    g_lastNavRxMs.store(millis(), std::memory_order_relaxed);
    g_navDirty.store(true, std::memory_order_release);

#if VMHUD_HAS_DISPLAY
    {
      std::lock_guard<std::mutex> guard(g_roadMutex);
      g_currentRoad = roadBuf[0] ? String(roadBuf) : textUtf8(nav.current, nav.currentSize);
    }
#endif
    g_displayDirty.store(true, std::memory_order_relaxed);
    debugLogf("[NAV] cmd=0x%02X dist=%lu img=%u keep=%u road=\"%s\"\n",
              (unsigned)frame.command, (unsigned long)nav.distance,
              (unsigned)nav.imageSize, (unsigned)nav.keepImage, roadBuf);
    return;
  }

  // Camera and speed limit (CMD 0x01, multiple of 4 bytes: current + optional upcoming slot)
  if (frame.command == 1 && frame.size >= 4 && (frame.size % 4) == 0) {
    g_camPktCount.fetch_add(1, std::memory_order_relaxed);
    const uint8_t prevLim = g_postedLimit.load(std::memory_order_relaxed);
    const vmhud::CameraParseResult cam = vmhud::parseCameraFrame(frame, prevLim);
    g_camDist.store(cam.camDist, std::memory_order_relaxed);
    g_camType.store(cam.camType, std::memory_order_relaxed);
    g_postedLimit.store(cam.currentLimit, std::memory_order_release);
    if (cam.currentLimit >= 5 && cam.currentLimit <= 200) {
      // Request prompt USB CDC transmission from loop() (<= 20 ms latency)
      g_splimForceEmit.store(true, std::memory_order_release);
    }
    if (cam.nextLimit >= 5 && cam.nextLimit <= 200) {
      g_camNextDist.store(cam.nextDist, std::memory_order_relaxed);
      g_camNextRxMs.store(millis(), std::memory_order_relaxed);
      g_camNextLimit.store(cam.nextLimit, std::memory_order_release);
    } else {
      g_camNextLimit.store(0, std::memory_order_release);
      g_camNextDist.store(0, std::memory_order_relaxed);
    }
    uint8_t c01Sign = vmhud::kSignNone;
    uint16_t c01Dist = 0;
    if (vmhud::extractCmd01CameraAlert(frame, cam, c01Sign, c01Dist)) {
      g_cmd01SignDist.store(c01Dist, std::memory_order_relaxed);
      g_cmd01SignRxMs.store(millis(), std::memory_order_relaxed);
      g_cmd01SignCode.store(c01Sign, std::memory_order_release);
    } else {
      g_cmd01SignCode.store(vmhud::kSignNone, std::memory_order_release);
      g_cmd01SignDist.store(0, std::memory_order_relaxed);
    }
    g_displayDirty.store(true, std::memory_order_relaxed);
    debugLogf("[CAM] type=%u dist=%u cur=%u next=%u(d=%u)\n",
              (unsigned)cam.camType, (unsigned)cam.camDist,
              (unsigned)cam.currentLimit, (unsigned)cam.nextLimit, (unsigned)cam.nextDist);
    return;
  }

  // Custom camera/sign alert with embedded image (CMD 0x41: [type, dist_be16, limit, imgLen_be32, ...])
  if (frame.command == 0x41 && frame.size >= 8) {
    const uint16_t dist = vmhud::be16(frame.payload + 1);
    const uint8_t lim = frame.payload[3];
    const uint32_t imgLen = vmhud::be32(frame.payload + 4);
    if (lim >= 5 && lim <= 200) {
      g_customSignDist.store(0, std::memory_order_relaxed);
      g_customNextDist.store(dist < 60000u ? dist : 0u, std::memory_order_relaxed);
      g_customNextRxMs.store(millis(), std::memory_order_relaxed);
      g_customNextLimit.store(lim, std::memory_order_release);
      debugLogf("[CAM41] dist=%u lim=%u\n", (unsigned)dist, (unsigned)lim);
    } else if (imgLen >= 16 && frame.size >= 8u + size_t(imgLen) && imgLen <= kMaxAlertImgBytes) {
      const uint8_t *jpg = frame.payload + 8;
      if (jpg[0] == 0xFF && jpg[1] == 0xD8) {
        g_customSignDist.store(dist < 60000u ? dist : 0u, std::memory_order_relaxed);
        g_customSignRxMs.store(millis(), std::memory_order_release);
        {
          std::lock_guard<std::mutex> guard(g_alertImgMutex);
          memcpy(g_pendingAlertImg.jpeg, jpg, imgLen);
          g_pendingAlertImg.len = imgLen;
        }
        g_alertImgCancel.store(false, std::memory_order_relaxed);
        g_alertImgDirty.store(true, std::memory_order_release);
        debugLogf("[CAM41] sign jpeg=%lu dist=%u\n", (unsigned long)imgLen, (unsigned)dist);
      }
    } else if (dist == 0 && lim == 0 && imgLen == 0) {
      g_customSignDist.store(0, std::memory_order_relaxed);
    }
    return;
  }

  // Overlay text (CMD 0x06: [posX_be16, posY_be16, font, 0x00, r, g, b, utf8..., 0x00]):
  // forward raw UTF-8 string sent by VietMap directly without converting to meters.
  if (frame.command == 0x06) {
    char txtBuf[vmhud::kMaxOverlayTextBytes + 1];
    if (vmhud::extractOverlayText(frame, txtBuf, sizeof(txtBuf))) {
      {
        std::lock_guard<std::mutex> guard(g_alertImgMutex);
        memcpy(g_overlaySignText, txtBuf, sizeof(g_overlaySignText));
      }
      g_textSignRxMs.store(millis(), std::memory_order_release);
      debugLogf("[TXT06] raw=\"%s\"\n", txtBuf);
    }
    return;
  }

  // Cancel/clear overlay text (CMD 0x07 / CMD 0x47)
  if (vmhud::isOverlayTextClear(frame)) {
    {
      std::lock_guard<std::mutex> guard(g_alertImgMutex);
      g_overlaySignText[0] = '\0';
    }
    return;
  }

  // Warning/sign overlay image (CMD 0x08: [posX_be16, posY_be16, JPEG...]).
  // Offload JPEG decode to loop() so NimBLE task stack stays tiny.
  if (frame.command == 0x08 && frame.size >= 20) {
    const uint8_t *jpg = frame.payload + 4;
    const size_t jpgLen = frame.size - 4;
    if (jpg[0] == 0xFF && jpg[1] == 0xD8) {
      if (jpgLen <= kMaxAlertImgBytes) {
        {
          std::lock_guard<std::mutex> guard(g_alertImgMutex);
          memcpy(g_pendingAlertImg.jpeg, jpg, jpgLen);
          g_pendingAlertImg.len = jpgLen;
        }
        g_alertImgCancel.store(false, std::memory_order_relaxed);
        g_alertImgDirty.store(true, std::memory_order_release);
      } else {
        debugLogf("[SIGN] jpeg oversize %u > %u\n",
                  (unsigned)jpgLen, (unsigned)kMaxAlertImgBytes);
      }
    }
    return;
  }

  // Cancel overlay image (CMD 0x09): also clear any overlay text from the cancelled sign.
  if (frame.command == 0x09) {
    {
      std::lock_guard<std::mutex> guard(g_alertImgMutex);
      g_overlaySignText[0] = '\0';
    }
    g_customSignDist.store(0, std::memory_order_relaxed);
    g_alertImgDirty.store(false, std::memory_order_release);
    g_alertImgCancel.store(true, std::memory_order_release);
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

static void startConnectable() {
  NimBLEDevice::getAdvertising()->start(kAdvInstHud);
}

static void refreshWakeBeaconRandomAddr() {
  ble_addr_t rnd{};
  rnd.type = BLE_ADDR_RANDOM;
  esp_fill_random(rnd.val, sizeof(rnd.val));
  rnd.val[5] |= 0xC0u;                     // BLE Core Spec static random MSB bits (11)
  rnd.val[0] = (rnd.val[0] & 0xFEu) | 0x02u;  // Guarantee lower 46 bits are neither all-0 nor all-1
  ble_gap_ext_adv_set_addr(kAdvInstWake, &rnd);
}

static void resetVietMapSessionState() {
  g_hudInfoOk.store(false, std::memory_order_release);
  g_rxH1.reset();
  g_rxH50.reset();
  g_postedLimit.store(0, std::memory_order_release);
  g_currentSpeed.store(0, std::memory_order_relaxed);
  g_camDist.store(0, std::memory_order_relaxed);
  g_camType.store(0, std::memory_order_relaxed);
  g_cmd01SignCode.store(vmhud::kSignNone, std::memory_order_release);
  g_cmd01SignDist.store(0, std::memory_order_relaxed);
  g_camNextLimit.store(0, std::memory_order_release);
  g_camNextDist.store(0, std::memory_order_relaxed);
  g_customNextLimit.store(0, std::memory_order_release);
  g_customNextDist.store(0, std::memory_order_relaxed);
  g_customSignDist.store(0, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> guard(g_alertImgMutex);
    g_overlaySignText[0] = '\0';
  }
  g_alertImgDirty.store(false, std::memory_order_release);
  g_alertImgCancel.store(true, std::memory_order_release);
  g_nextResetReq.store(true, std::memory_order_release);
  g_lastEmittedLimit.store(0, std::memory_order_relaxed);
  g_splimForceEmit.store(false, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> guard(g_navMutex);
    g_pendingNav.active = false;
    g_pendingNav.hasNewJpeg = false;
    g_pendingNav.jpegLen = 0;
    g_pendingNav.road[0] = '\0';
  }
  g_navDirty.store(false, std::memory_order_release);
  g_navOffReq.store(true, std::memory_order_release);
  g_cphForceNotify.store(true, std::memory_order_release);
  g_displayDirty.store(true, std::memory_order_relaxed);
}

struct ServerCB : NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &conn) override {
    if (!g_connected.load(std::memory_order_acquire)) {
      g_conn.store(conn.getConnHandle(), std::memory_order_release);
      g_connected.store(true, std::memory_order_release);
      g_hudInfoOk.store(false, std::memory_order_relaxed);
      g_subH1.store(false, std::memory_order_relaxed);
      g_subH50.store(false, std::memory_order_relaxed);
      g_vmUnsubWatchdogActive.store(false, std::memory_order_relaxed);
      g_vmUnsubWatchdogMs.store(0, std::memory_order_relaxed);
      g_rxH1.reset();
      g_rxH50.reset();
      g_displayDirty.store(true, std::memory_order_relaxed);
    }
    debugLogf("[BLE] connected handle=%u\n", (unsigned)conn.getConnHandle());
  }
  void onDisconnect(NimBLEServer *, NimBLEConnInfo &conn, int reason) override {
    const uint16_t h = conn.getConnHandle();
    if (h == g_cphConn.load(std::memory_order_acquire)) {
      g_subCph.store(false, std::memory_order_release);
      g_cphConn.store(BLE_HS_CONN_HANDLE_NONE, std::memory_order_release);
    }
    if (h == g_conn.load(std::memory_order_acquire)) {
      g_connected.store(false, std::memory_order_release);
      g_conn.store(BLE_HS_CONN_HANDLE_NONE, std::memory_order_release);
      g_subH1.store(false, std::memory_order_relaxed);
      g_subH50.store(false, std::memory_order_relaxed);
      g_vmUnsubWatchdogActive.store(false, std::memory_order_release);
      g_vmUnsubWatchdogMs.store(0, std::memory_order_relaxed);
      resetVietMapSessionState();
      debugLogf("[BLE] disconnected reason=%d\n", reason);
      startConnectable();
    } else {
      debugLogf("[BLE] secondary disconnected handle=%u reason=%d\n", (unsigned)h, reason);
    }
  }
};

struct SubscribeCB : NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *c, NimBLEConnInfo &conn, uint16_t sub) override {
    const uint16_t h = conn.getConnHandle();
    const bool enabled = (sub & 1) != 0;
    if (c == g_cph) {
      g_subCph.store(enabled, std::memory_order_release);
      g_cphConn.store(enabled ? h : BLE_HS_CONN_HANDLE_NONE, std::memory_order_release);
      if (enabled) {
        g_cphForceNotify.store(true, std::memory_order_release);
        // Bench-test support: if a standalone CPH device connected before VietMap Live,
        // keep Instance 0 advertising so VietMap Live on a 2nd phone can still connect.
        if (!g_subH1.load(std::memory_order_relaxed) &&
            !g_subH50.load(std::memory_order_relaxed)) {
          startConnectable();
        }
      }
      debugLogf("[CPH] FFF9 sub=%u handle=%u\n", (unsigned)enabled, (unsigned)h);
      return;
    }

    if (!g_connected.load(std::memory_order_acquire) ||
        (!g_subH1.load(std::memory_order_relaxed) && !g_subH50.load(std::memory_order_relaxed))) {
      g_conn.store(h, std::memory_order_release);
      g_connected.store(true, std::memory_order_release);
    }
    if (h != g_conn.load(std::memory_order_acquire)) {
      return;
    }

    const bool wasVmSub =
        g_subH1.load(std::memory_order_relaxed) || g_subH50.load(std::memory_order_relaxed);
    if (c == g_h1) {
      g_subH1.store(enabled, std::memory_order_relaxed);
    } else if (c == g_h50) {
      g_subH50.store(enabled, std::memory_order_relaxed);
    }
    const bool nowVmSub =
        g_subH1.load(std::memory_order_relaxed) || g_subH50.load(std::memory_order_relaxed);
    if (nowVmSub) {
      g_vmUnsubWatchdogActive.store(false, std::memory_order_release);
    } else if (wasVmSub && !nowVmSub) {
      resetVietMapSessionState();
      g_vmUnsubWatchdogMs.store(millis(), std::memory_order_relaxed);
      g_vmUnsubWatchdogActive.store(true, std::memory_order_release);
      debugLogf("[BLE] VietMap unsubscribed; armed 1.5s ACL release watchdog\n");
    }
    g_displayDirty.store(true, std::memory_order_relaxed);
    debugLogf("[BLE] notify sub=0x%04X\n", sub);
  }
};

template <size_t RxCap>
struct WriteCB : NimBLECharacteristicCallbacks {
  NimBLECharacteristic *route;
  const char *label;
  vmhud::Receiver<RxCap> *rx;
  WriteCB(NimBLECharacteristic *r, const char *l, vmhud::Receiver<RxCap> *b) : route(r), label(l), rx(b) {}
  void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &conn) override {
    const uint16_t h = conn.getConnHandle();
    if (!g_connected.load(std::memory_order_acquire)) {
      g_conn.store(h, std::memory_order_release);
      g_connected.store(true, std::memory_order_release);
    }
    if (h != g_conn.load(std::memory_order_acquire)) {
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

  NimBLEExtAdvertising *adv = NimBLEDevice::getAdvertising();

  // Instance 0: Connectable legacy VIETMAP-HUD advertisement (FFF0 / FFFF ONLY — never FFF8).
  NimBLEExtAdvertisement primary0;
  primary0.setLegacyAdvertising(true);
  primary0.setConnectable(true);
  primary0.setScannable(true);
  primary0.setFlags(0x06);
  fillServiceUuids(primary0);

  NimBLEExtAdvertisement scanResp0;
  scanResp0.setName(name);

  adv->setInstanceData(kAdvInstHud, primary0);
  adv->setScanResponseData(kAdvInstHud, scanResp0);

  // Instance 1: Non-connectable FFF8 wakeup beacon (ADV_NONCONN_IND on static random MAC).
  // Started strictly while VietMap Live is active and FFF9 is unsubscribed.
  NimBLEExtAdvertisement wakeAdv;
  wakeAdv.setLegacyAdvertising(true);
  wakeAdv.setConnectable(false);
  wakeAdv.setScannable(false);
  wakeAdv.setMinInterval(320);  // 200 ms
  wakeAdv.setMaxInterval(400);  // 250 ms
  wakeAdv.setFlags(0x06);
  wakeAdv.addServiceUUID("FFF8");
  ble_addr_t rnd = *NimBLEDevice::getAddress().getBase();
  rnd.type = BLE_ADDR_RANDOM;
  rnd.val[5] |= 0xC0u;  // BLE Core Spec static random address MSB bits (11)
  rnd.val[0] ^= 0x5Au;  // Distinct LSB from Instance 0 public address
  wakeAdv.setAddress(NimBLEAddress(rnd));
  adv->setInstanceData(kAdvInstWake, wakeAdv);
  g_wakeAdvRunning = false;

  startConnectable();
  Serial.printf("[ADV] name=%s (scan response) adv=%s + FFF8 wake beacon configured\n",
                g_name.c_str(), g_adv.c_str());
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
    // VietMap iOS DJBTManager strips CMD 0x40 JPEG turn arrows whenever PROTOCOL
    // contains "S" (e.g. legacy "2.2.2S"). Auto-migrate stale NVS to "2.2.2".
    if (g_protocol.indexOf('S') >= 0 || g_protocol.indexOf('s') >= 0) {
      g_protocol = "2.2.2";
      g_store.putString("protocol", g_protocol);
    }
    g_store.end();
  }

  NimBLEDevice::init(std::string(g_name.c_str()));
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  NimBLEService *h1 = server->createService("FFF0");
  g_h1 = h1->createCharacteristic("FFF1", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_h1->setCallbacks(new SubscribeCB());
  const uint32_t writable = NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR;
  h1->createCharacteristic("FFF2", writable)->setCallbacks(new WriteCB<kRxBufCap>(g_h1, "FFF2", &g_rxH1));
  h1->createCharacteristic("FFF3", writable)->setCallbacks(new WriteCB<kRxBufCap>(g_h1, "FFF3", &g_rxH1));
  h1->createCharacteristic("FFF4", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  h1->start();

  NimBLEService *h50 = server->createService("FFFF");
  g_h50 = h50->createCharacteristic("1234", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_h50->setCallbacks(new SubscribeCB());
  h50->createCharacteristic("9ABC", writable)->setCallbacks(new WriteCB<512>(g_h50, "9ABC", &g_rxH50));
  h50->start();

  // CarPlay Helper shared-ACL telemetry service (FFF8 / FFF9: compact 0xA7 frame).
  NimBLEService *cphSvc = server->createService("FFF8");
  g_cph = cphSvc->createCharacteristic("FFF9", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_cph->setCallbacks(new SubscribeCB());
  {
    vmhud::CphTelemetryState initSt{};
    g_lastCphWireLen = vmhud::encodeCphFrame(initSt, g_lastCphWire);
    g_cph->setValue(g_lastCphWire, g_lastCphWireLen);
    g_cphWireInit = true;
  }
  cphSvc->start();

  applyAdvertising();
  // Seed alert ID from ESP32 hardware TRNG so a board reboot never collides with
  // the CMU's last_seen_id_ on an ongoing CarPlay session.
  g_nextAlertId = (esp_random() & 0x0FFFFFFFu) | 1u;
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

  // Process VietMap BLE turn-by-turn navigation updates on loop() (safe stack for JPEG decode).
  if (g_navOffReq.exchange(false, std::memory_order_acquire)) {
    g_navGuiding = false;
    g_navIcon = 0;
    g_navDist = 0;
    g_navRoad[0] = '\0';
    g_lastJpegHash = 0;
    g_lastJpegLen = 0;
    emitNavOff();
  }

  // Shared single-task scratch buffer for loop() JPEG decoding (reused by both
  // CMD 0x40 nav arrows and CMD 0x08 speed signs — saves 4 KB static SRAM).
  static uint8_t jpegScratch[kMaxJpegBytes];

  if (g_navDirty.exchange(false, std::memory_order_acquire)) {
    bool     active = false;
    uint32_t dist = 0;
    char     roadCopy[kMaxRoadBytes + 1] = {0};
    size_t   jpegLen = 0;
    bool     hasJpeg = false;
    {
      std::lock_guard<std::mutex> guard(g_navMutex);
      active = g_pendingNav.active;
      dist = g_pendingNav.distance;
      strncpy(roadCopy, g_pendingNav.road, kMaxRoadBytes);
      roadCopy[kMaxRoadBytes] = '\0';
      if (g_pendingNav.hasNewJpeg && g_pendingNav.jpegLen > 0) {
        jpegLen = g_pendingNav.jpegLen;
        memcpy(jpegScratch, g_pendingNav.jpeg, jpegLen);
        hasJpeg = true;
        g_pendingNav.hasNewJpeg = false;
      }
    }

    if (!active) {
      g_lastJpegHash = 0;
      g_lastJpegLen = 0;
      if (g_navGuiding) {
        g_navGuiding = false;
        g_navIcon = 0;
        g_navDist = 0;
        g_navRoad[0] = '\0';
        emitNavOff();
      }
    } else {
      if (hasJpeg) {
        const uint32_t jhash = fnv1a32(jpegScratch, jpegLen);
        if (jhash != g_lastJpegHash || jpegLen != g_lastJpegLen || g_navIcon == 0) {
          g_lastJpegHash = jhash;
          g_lastJpegLen = jpegLen;
          vmhud::ArrowResult res;
          if (vmhud::classifyJpeg(jpegScratch, jpegLen, res)) {
            g_navIcon = res.icon;
            debugLogf("[ARROW] icon=%u score=%.3f margin=%.3f\n",
                      (unsigned)res.icon, res.score, res.margin);
          } else {
            debugLogf("[ARROW] weak/non-arrow score=%.3f margin=%.3f (keep=%u)\n",
                      res.score, res.margin, (unsigned)g_navIcon);
            if (g_navIcon == 0) g_navIcon = vmhud::kMazdaIconStraight;
          }
        }
      } else if (g_navIcon == 0) {
        g_navIcon = vmhud::kMazdaIconStraight;
      }
      g_navGuiding = true;
      g_navDist = dist;
      strncpy(g_navRoad, roadCopy, kMaxRoadBytes);
      g_navRoad[kMaxRoadBytes] = '\0';

      const bool changed = (g_navIcon != g_lastEmitNavIcon) ||
                           (g_navDist != g_lastEmitNavDist) ||
                           (strcmp(g_navRoad, g_lastEmitNavRoad) != 0);
      if (changed) {
        emitNav(g_navIcon, g_navDist, g_navRoad);
      }
    }
  }

  // Nav keepalive / staleness guard (while guiding)
  if (g_navGuiding) {
    if (now - g_lastNavRxMs.load(std::memory_order_relaxed) > kNavTimeoutMs) {
      g_navGuiding = false;
      g_navIcon = 0;
      g_navDist = 0;
      g_navRoad[0] = '\0';
      g_lastJpegHash = 0;
      g_lastJpegLen = 0;
      emitNavOff();
      debugLogf("[NAV] timeout -> NAV OFF\n");
    } else if (now - g_lastNavEmitMs >= kNavKeepaliveMs) {
      emitNav(g_navIcon, g_navDist, g_navRoad);
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
    // When neither limit nor nav is active, poll CMU service every 2 s
    if (!g_navGuiding && (now - g_lastKeepaliveMs) >= kKeepaliveMs) {
      g_lastKeepaliveMs = now;
      serialWriteLine("PING\n", 5);
    }
  }

  // Process upcoming ("next") speed limit & prohibition sign alerts from CMD 0x01 / CMD 0x41 / CMD 0x08.
  if (g_nextResetReq.exchange(false, std::memory_order_acquire)) {
    g_imgNextLimit = 0;
    g_imgSignCode = vmhud::kSignNone;
    g_lastAlertImgHash = 0;
    g_lastAlertImgLen = 0;
    g_latchedNextLimit = 0;
    g_nextAbsentSinceMs = 0;
  }
  if (g_alertImgCancel.exchange(false, std::memory_order_acquire)) {
    g_imgNextLimit = 0;
    g_imgSignCode = vmhud::kSignNone;
    g_lastAlertImgHash = 0;
    g_lastAlertImgLen = 0;
  }
  if (g_alertImgDirty.exchange(false, std::memory_order_acquire)) {
    size_t alertLen = 0;
    {
      std::lock_guard<std::mutex> guard(g_alertImgMutex);
      alertLen = g_pendingAlertImg.len;
      if (alertLen > 0 && alertLen <= kMaxAlertImgBytes) {
        memcpy(jpegScratch, g_pendingAlertImg.jpeg, alertLen);
      } else {
        alertLen = 0;
      }
    }
    if (alertLen >= 16) {
      const uint32_t h = fnv1a32(jpegScratch, alertLen);
      if (h != g_lastAlertImgHash || alertLen != g_lastAlertImgLen) {
        g_lastAlertImgHash = h;
        g_lastAlertImgLen = alertLen;
        const vmhud::AlertImageClassification cls = vmhud::classifyAlertJpeg(jpegScratch, alertLen);
        g_imgNextLimit = cls.speedLimit;
        g_imgSignCode = cls.signCode;
        if (cls.speedLimit != 0) {
          g_imgNextRxMs = now;
          debugLogf("[SIGN] speed=%u score=%.3f margin=%.3f\n",
                    (unsigned)cls.speedLimit, cls.score, cls.margin);
        } else if (cls.signCode != vmhud::kSignNone) {
          g_imgSignRxMs = now;
          debugLogf("[SIGN] code=0x%02X score=%.3f margin=%.3f\n",
                    (unsigned)cls.signCode, cls.score, cls.margin);
        } else {
          // Non-sign image (e.g. weather icon): discard any accompanying overlay text.
          std::lock_guard<std::mutex> guard(g_alertImgMutex);
          g_overlaySignText[0] = '\0';
          debugLogf("[SIGN] unrecognized score=%.3f margin=%.3f\n", cls.score, cls.margin);
        }
      } else {
        if (g_imgNextLimit != 0) g_imgNextRxMs = now;
        if (g_imgSignCode != vmhud::kSignNone) g_imgSignRxMs = now;
      }
    }
  }

  // Arbitrate upcoming speed-limit candidate across CMD 0x01 > CMD 0x41 > CMD 0x08.
  uint8_t  candNextLim = 0;
  uint16_t candNextDist = 0;
  const uint8_t camNext = g_camNextLimit.load(std::memory_order_acquire);
  if (camNext >= 5 && camNext <= 200 &&
      (now - g_camNextRxMs.load(std::memory_order_relaxed)) <= kNextSourceStaleMs) {
    candNextLim = camNext;
    candNextDist = g_camNextDist.load(std::memory_order_relaxed);
  } else {
    const uint8_t c41Next = g_customNextLimit.load(std::memory_order_acquire);
    if (c41Next >= 5 && c41Next <= 200 &&
        (now - g_customNextRxMs.load(std::memory_order_relaxed)) <= kNextSourceStaleMs) {
      candNextLim = c41Next;
      candNextDist = g_customNextDist.load(std::memory_order_relaxed);
    } else if (g_imgNextLimit >= 5 && g_imgNextLimit <= 200 &&
               (now - g_imgNextRxMs) <= kNextSourceStaleMs) {
      candNextLim = g_imgNextLimit;
      candNextDist = 0;
    }
  }

  // Edge-triggered alert ID latch: fires a NEW alert_id once per distinct upcoming speed limit.
  if (candNextLim >= 5 && candNextLim <= 200 && candNextLim != lim) {
    g_nextAbsentSinceMs = 0;
    if (candNextLim != g_latchedNextLimit) {
      if (++g_nextAlertId == 0) g_nextAlertId = 1;
      g_latchedNextLimit = candNextLim;
      emitNextSplim(g_nextAlertId, candNextLim, candNextDist);
      debugLogf("[NEXT] trigger id=%lu lim=%u dist=%u (cur=%u)\n",
                (unsigned long)g_nextAlertId, (unsigned)candNextLim,
                (unsigned)candNextDist, (unsigned)lim);
    } else {
      const uint16_t distDiff = (candNextDist > g_lastNextEmitDist)
                                    ? (candNextDist - g_lastNextEmitDist)
                                    : (g_lastNextEmitDist - candNextDist);
      if ((now - g_lastNextEmitMs) >= kNextReemitMs ||
          (candNextDist != g_lastNextEmitDist &&
           (candNextDist == 0 || g_lastNextEmitDist == 0 || distDiff >= 10u))) {
        emitNextSplim(g_nextAlertId, candNextLim, candNextDist);
      }
    }
  } else if (g_latchedNextLimit != 0) {
    if (lim == g_latchedNextLimit) {
      // Vehicle entered the new speed-limit zone -> release latch immediately.
      g_latchedNextLimit = 0;
      g_nextAbsentSinceMs = 0;
    } else if (g_nextAbsentSinceMs == 0) {
      g_nextAbsentSinceMs = now;
    } else if ((now - g_nextAbsentSinceMs) >= kNextClearDebounceMs) {
      g_latchedNextLimit = 0;
      g_nextAbsentSinceMs = 0;
    }
  }

  // Manage Instance 1 non-connectable FFF8 wakeup beacon.
  const bool vmActive = g_connected.load(std::memory_order_relaxed) &&
                        g_hudInfoOk.load(std::memory_order_acquire) &&
                        (g_subH1.load(std::memory_order_relaxed) ||
                         g_subH50.load(std::memory_order_relaxed));
  const bool shouldWakeAdv = vmActive && !g_subCph.load(std::memory_order_acquire);
  if (shouldWakeAdv && !g_wakeAdvRunning) {
    // Rotate random beacon MAC on every session start so iOS CoreBluetooth background
    // scanning (AllowDuplicates=false) never suppresses a new drive's wakeup packet.
    refreshWakeBeaconRandomAddr();
    if (NimBLEDevice::getAdvertising()->start(kAdvInstWake)) {
      g_wakeAdvRunning = true;
      debugLogf("[WAKE] FFF8 beacon started\n");
    }
  } else if (!shouldWakeAdv && g_wakeAdvRunning) {
    NimBLEDevice::getAdvertising()->stop(kAdvInstWake);
    g_wakeAdvRunning = false;
    debugLogf("[WAKE] FFF8 beacon stopped\n");
  }

  // Build and publish CarPlay Helper telemetry frame on FFF9 (READ | NOTIFY)
  // BEFORE running the lingering-ACL disconnect watchdog so `vietmapConnected = false`
  // is guaranteed to notify first.
  if (g_cph != nullptr) {
    vmhud::CphTelemetryState cphSt{};
    cphSt.vietmapConnected = vmActive;
    cphSt.cmuConnected = g_cmuConnected.load(std::memory_order_relaxed);

    if (vmActive) {
      const uint16_t rawSpd = g_currentSpeed.load(std::memory_order_relaxed);
      g_cphOverspeed = vmhud::evaluateOverspeed(lim, rawSpd, g_cphOverspeed);
      cphSt.overspeed = g_cphOverspeed;
      cphSt.navActive = g_navGuiding;
      cphSt.postedSpeedLimit = (lim >= 5 && lim <= 200) ? lim : 0;
      cphSt.currentSpeed = rawSpd > 255u ? 255u : static_cast<uint8_t>(rawSpd);

      char rawText[vmhud::kMaxOverlayTextBytes + 1] = {0};
      {
        std::lock_guard<std::mutex> guard(g_alertImgMutex);
        memcpy(rawText, g_overlaySignText, sizeof(rawText));
      }
      const uint32_t txtRxMs = g_textSignRxMs.load(std::memory_order_acquire);
      const bool txtFresh = (rawText[0] != '\0') && ((now - txtRxMs) <= kSignDistStaleMs);
      const uint16_t c41Dist = g_customSignDist.load(std::memory_order_acquire);
      const bool c41Fresh = c41Dist > 0 && (now - g_customSignRxMs.load(std::memory_order_relaxed)) <= kSignDistStaleMs;
      const uint8_t c01Sign = g_cmd01SignCode.load(std::memory_order_acquire);
      const uint16_t c01Dist = g_cmd01SignDist.load(std::memory_order_relaxed);
      const bool c01Fresh = c01Sign != vmhud::kSignNone &&
                            (now - g_cmd01SignRxMs.load(std::memory_order_relaxed)) <= kSignDistStaleMs;

      // Upcoming speed limit + distance fusion.
      uint8_t nextLim = (candNextLim >= 5 && candNextLim <= 200 && candNextLim != lim) ? candNextLim : 0;
      uint16_t nextDist = (nextLim != 0) ? candNextDist : 0;
      if (nextLim == 0 && g_imgNextLimit >= 5 && g_imgNextLimit <= 200 && g_imgNextLimit != lim) {
        const uint32_t lastImgSpdMs = txtFresh ? txtRxMs : g_imgNextRxMs;
        if ((now - lastImgSpdMs) <= kSignHoldStaleMs) {
          nextLim = g_imgNextLimit;
        }
      }
      cphSt.upcomingSpeedLimit = nextLim;
      cphSt.upcomingSpeedDistM = nextDist;

      // Prohibition / area / camera sign + distance fusion.
      if (g_imgSignCode != vmhud::kSignNone) {
        uint32_t lastSignMs = g_imgSignRxMs;
        if (txtFresh) lastSignMs = txtRxMs;
        if (c41Fresh) lastSignMs = g_customSignRxMs.load(std::memory_order_relaxed);
        if ((now - lastSignMs) > kSignHoldStaleMs) {
          g_imgSignCode = vmhud::kSignNone;
        }
      }
      if (g_imgSignCode != vmhud::kSignNone) {
        cphSt.activeSignCode = g_imgSignCode;
        if (c41Fresh) {
          cphSt.activeSignDistM = c41Dist;
        } else if (c01Fresh && (g_imgSignCode == vmhud::kSignSpeedCamera ||
                                g_imgSignCode == vmhud::kSignTrafficCamera)) {
          cphSt.activeSignDistM = c01Dist;
        }
      } else if (c01Fresh) {
        cphSt.activeSignCode = c01Sign;
        cphSt.activeSignDistM = c01Dist;
      }

      // Forward raw CMD 0x06 string verbatim whenever an image sign or upcoming speed limit is active.
      if (txtFresh && (cphSt.activeSignCode != vmhud::kSignNone || cphSt.upcomingSpeedLimit != 0)) {
        memcpy(cphSt.activeSignText, rawText, sizeof(cphSt.activeSignText));
      }
    } else {
      g_cphOverspeed = false;
    }

    // Fast path: skip XOR encode & NimBLE ATT mutex lock on the ~98% of 50Hz ticks with no state change.
    if (!g_cphWireInit || cphSt != g_lastCphBuiltState) {
      g_lastCphWireLen = vmhud::encodeCphFrame(cphSt, g_lastCphWire);
      g_cph->setValue(g_lastCphWire, g_lastCphWireLen);
      g_lastCphBuiltState = cphSt;
      g_cphWireInit = true;
    }

    const bool forceCph = g_cphForceNotify.exchange(false, std::memory_order_acquire);
    if (g_subCph.load(std::memory_order_acquire)) {
      const uint16_t cphH = g_cphConn.load(std::memory_order_acquire);
      if (cphH != BLE_HS_CONN_HANDLE_NONE &&
          (forceCph || vmhud::isCphSemanticChange(g_lastCphNotified, cphSt) ||
           (now - g_lastCphNotifyMs) >= kCphKeepaliveMs)) {
        g_cph->notify(g_lastCphWire, g_lastCphWireLen, cphH);
        g_lastCphNotified = cphSt;
        g_lastCphNotifyMs = now;
      }
    }
  }

  // Enforce 1.5s lingering-ACL disconnect watchdog when VietMap Live unsubscribes FFF1.
  if (g_vmUnsubWatchdogActive.load(std::memory_order_acquire)) {
    const bool vmSub = g_subH1.load(std::memory_order_relaxed) ||
                       g_subH50.load(std::memory_order_relaxed);
    if (vmSub) {
      g_vmUnsubWatchdogActive.store(false, std::memory_order_release);
    } else {
      const uint32_t unsubMs = g_vmUnsubWatchdogMs.load(std::memory_order_relaxed);
      const uint32_t elapsed = millis() - unsubMs;
      if (elapsed >= kVmUnsubGraceMs && elapsed < 0x80000000u) {
        g_vmUnsubWatchdogActive.store(false, std::memory_order_release);
        const uint16_t h = g_conn.load(std::memory_order_acquire);
        if (h != BLE_HS_CONN_HANDLE_NONE) {
          debugLogf("[BLE] Force-releasing lingering ACL handle=%u\n", (unsigned)h);
          NimBLEDevice::getServer()->disconnect(h);
        }
      }
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
