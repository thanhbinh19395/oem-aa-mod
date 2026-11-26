# Design: ESP32 CarPlay BLE Telemetry Service (`vmhud-esp32-s3`)

## Context

See `proposal.md` for motivation. Three hardware and OS-level facts verified directly from NimBLE's controller source (`ble_ll_conn.c`) and iOS `vietmap_hud_sdk.framework` (`Runner.app` IPA) govern how a single ESP32 simultaneously receives VietMap telemetry and transmits it to `CarPlay Helper` on the **same iPhone**:

1. **BLE Link Layer Single-ACL Constraint (`ble_ll_conn.c:4088`)**: Per Bluetooth Core Spec Vol 6 Part B §4.5 and NimBLE's `ble_ll_conn_periph_start()`, a peripheral controller rejects a second connectable `CONNECT_IND` from the same Initiator address (`inita`) if an active ACL connection with that `inita` already exists—even across multiple Extended Advertising MACs! Therefore, two apps on the **same iPhone** MUST share one full-duplex BLE ACL connection via iOS `bluetoothd`'s built-in GATT connection multiplexer rather than opening two separate ACL links.
2. **iOS `vietmap_hud_sdk` Discovery & GATT Isolation**:
   - `-[DJBTManager didConnectPeripheral:]` calls `[peripheral discoverServices: nil]` (caching **all** ESP32 GATT services in iOS `bluetoothd`) and retains **only** `FFF0` (`FFF2` write, `FFF1` notify), completely ignoring additional services like `FFFF` or `FFF8`.
   - However, `-[DJBTManager startSearchingBluetooth]` only uses `scanForPeripheralsWithServices:nil` (never `retrieveConnectedPeripherals`). Thus `CarPlay Helper` MUST NOT open the ACL link before `VietMap Live`; instead, `CarPlay Helper` attaches via `retrieveConnectedPeripherals(withServices: [FFF0, FFF8])` *after* `VietMap Live` connects, and releases when `VietMap Live` unsubscribes `FFF1`.
3. **VietMap iOS Prohibition Sign & Distance Wire Path**:
   - Prohibition, area, and hazard signs (`lib/assets/hud/*.png`) arrive as ~60×60 JPEGs via **`CMD 0x08`** (`[posX:u16be, posY:u16be, JPEG...]`, cancelled by **`CMD 0x09`**) or **`CMD 0x41`** (`[type:u8, dist:u16be, limit:u8, imgLen:u32be, JPEG...]`).
   - Approaching distance text for overlay signs arrives via **`CMD 0x06`** (`sendTextWithPositionX`: `[posX:u16be, posY:u16be, font:u8, 0x00, r:u8, g:u8, b:u8, utf8_text..., 0x00]`, cancelled by **`CMD 0x07`** / **`CMD 0x47`**), alongside `u16be` meter distances in **`CMD 0x01`** and **`CMD 0x41`**.

## Goals / Non-Goals

**Goals:**
- Share a single full-duplex BLE connection cleanly between `VietMap Live` (`FFF0` RX) and `CarPlay Helper` (`FFF8`/`FFF9` TX) on the same iPhone via iOS `bluetoothd`, while also supporting a second bench-test central on a different MAC (`CONFIG_BT_NIMBLE_MAX_CONNECTIONS=3`).
- Wake `CarPlay Helper` reliably in the iOS background as soon as `VietMap Live` connects, even when no C33 TPMS receiver is present, using a non-connectable `FFF8` beacon (`ADV_NONCONN_IND`).
- Guarantee `VIETMAP-HUD` immediately returns to connectable advertising whenever `VietMap Live` disconnects.
- Classify speed-limit (`30..120`) and prohibition/warning signs on-chip in `< 3 ms` on `loop()` with zero heap allocation, emitting a 12-byte unfragmented `FFF9` frame.

**Non-Goals:**
- Modifying Mazda CMU `splim_seriald` or `blmjcicarplay`, or touching Android Auto / Android VietMap paths.

## Decisions

1. **Shared Full-Duplex GATT Server (`FFF0` + `FFF8`) + Non-Connectable `FFF8` Wakeup Beacon**
   - **Decision**:
     - Keep **Instance 0 (`instId = 0`)** as connectable + scannable legacy advertising (`VIETMAP-HUD`, UUID `FFF0`, **never** advertising `FFF8` while unconnected so `CarPlay Helper` never connects first!).
     - Expose GATT service **`FFF8`** with characteristic **`FFF9`** (`READ | NOTIFY`) on the same `NimBLEServer`. When `CarPlay Helper` calls `retrieveConnectedPeripherals(withServices: [FFF0, FFF8])` and `central.connect(...)` on the same iPhone, iOS `bluetoothd` multiplexes both apps onto the single `connHandle` and writes CCCD `0x0001` on `FFF9` (`g_subCph = true`).
     - Use **Instance 1 (`instId = 1`, `NimBLEExtAdvertisement` with `setLegacyAdvertising(true)` and `setConnectable(false)`)** on a static random address to broadcast a **non-connectable** `ADV_NONCONN_IND` carrying service UUID `FFF8` **only while `g_hudInfoOk == true && !g_subCph`**. Because Instance 1 is non-connectable, it never triggers `ble_ll_conn.c`'s duplicate-ACL check, yet immediately wakes a backgrounded `CarPlay Helper` waiting on `scanForPeripherals(withServices: [FFF8])` so it can call `retrieveConnectedPeripherals` and attach to the shared connection! Once `g_subCph == true`, Instance 1 stops broadcasting.
     - **Clean Release on VietMap Exit**: When `g_subH1` transitions `true -> false` (`VietMap Live` closed or disconnected), immediately notify `FFF9` with `flags.vietmapConnected = 0` (prompting `CarPlay Helper` to call `cancelPeripheralConnection`), and arm a `1500 ms` timer on `loop()` that calls `server->disconnect(connHandle)` if the shared ACL handle has not yet closed, restarting Instance 0 (`VIETMAP-HUD`) advertising immediately.
   - **Rationale**: Respects Bluetooth LE's 1-ACL-per-peer Link Layer rule (`ble_ll_conn.c:4088`), leverages iOS `bluetoothd` GATT multiplexing natively, and eliminates every race condition regardless of which app starts first.
   - **Alternatives considered**:
     - *Two connectable advertising instances to the same iPhone*: Rejected—proven in `ble_ll_conn.c:4088` to drop the second `CONNECT_IND` from the same iPhone `inita`.
     - *CarPlay Helper connecting directly before VietMap Live*: Rejected—hides `VIETMAP-HUD` from `DJBTManager`'s `scanForPeripherals`.

2. **12-Byte Binary Telemetry Frame (`FFF8` / `FFF9`) & Distance Bucketing**
   - **Decision**: Define `lib/vmhud/cph_telemetry.h` packing a fixed 12-byte frame:
     ```text
     Offset  Field                 Type    Description
     0       magic                 u8      0xA7
     1       payloadLen            u8      0x0A (10 payload bytes; total = 12 bytes)
     2       flags                 u8      bit0: vietmapConnected, bit1: overspeed,
                                           bit2: navActive, bit3: cmuConnected
     3       postedSpeedLimit      u8      km/h (5..200, 0 = none)
     4       currentSpeed          u8      GPS km/h clamped 0..255
     5       upcomingSpeedLimit    u8      km/h (5..200, 0 = none)
     6..7    upcomingSpeedDistM    u16be   meters (0 = unknown)
     8       activeSignCode        u8      VietMapSignCode enum (0 = none)
     9..10   activeSignDistM       u16be   meters (0 = unspecified)
     11      checksum              u8      XOR of bytes 0..10
     ```
     Notify `FFF9` on every subscribed connection (`g_cphConnHandle`) immediately when `flags` (`vietmapConnected` / `overspeed` with `+2 / 0 km/h` hysteresis), `postedSpeedLimit`, `upcomingSpeedLimit`, `activeSignCode`, or quantized distance bucket (`>500m`, `500m`, `400m`, `300m`, `250m`, `200m`, `150m`, `100m`, `50m`, `0`) changes, plus a `5000 ms` keepalive.
   - **Rationale**: Fits inside the 20-byte minimum BLE ATT payload with zero fragmentation and prevents ActivityKit throttling.

3. **Two-Stage 16×16 Luma + Chroma Classifier & `CMD 0x06` Distance Fusion**
   - **Decision**: Reuse the `loop()` `TJpgDec` pass: Stage 1 checks inner `10×8` luma for speed limits `30..120`; Stage 2 matches full `16×16` luma + `8×8` R-B chroma against quantized templates for `VietMapSignCode` (`0x01..0x21`: turn/U-turn/entry bans, no-passing, residential area, parking, toll/railway/tunnel, cameras) across both `CMD 0x08` and `CMD 0x41`. Parse `CMD 0x06` UTF-8 text (`payload + 9`) for meter/km numbers (`"250 m"` → `250`, cleared on `0x07`/`0x47`/`0x09` or `5 s` timeout) and fuse with `CMD 0x41` / `CMD 0x01` distances.
   - **Rationale**: Zero heap usage, `< 4 KB` flash table, covers all iOS `vietmap_hud_sdk` sign and distance channels.

## Risks / Trade-offs

- **[Risk] Lingering shared ACL link if `CarPlay Helper` is suspended before processing `vietmapConnected = false`** → **Mitigation**: The `1500 ms` server-side `server->disconnect(connHandle)` watchdog after `g_subH1` unsubscribes guarantees `VIETMAP-HUD` always resumes connectable advertising promptly.
- **[Risk] Left-vs-right turn prohibition symmetry at 16×16 resolution** → **Mitigation**: Assert pairwise NCC margin across all iOS IPA `lib/assets/hud/*.png` assets at JPEG `q=65` in `test/vietmap_ble_hud_test.cpp`.

## Migration Plan

1. Generate sign templates from the iOS IPA HUD assets and run host unit tests (`make -C test vietmap_ble_hud_test`).
2. Build all PlatformIO environments (`pio run`) and flash the ESP32 via `./scripts/flash.sh`.
3. Zero Mazda CMU changes required; rollback is reflashing the previous ESP32 firmware commit.
