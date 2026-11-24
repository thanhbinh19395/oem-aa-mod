# Lane Guidance on Mazda HUD with VietMap Live (iPhone) — Research Findings & Future Plan

> **Status:** Deferred (not implemented yet). Turn-by-turn navigation + posted speed limit for VietMap Live on iPhone is implemented via the proven BLE `FFF0` (`PROTOCOL:2.2.2`) path without lane guidance.
>
> **Chosen future direction:** **Option 2** — probe and reverse-engineer VietMap Live's native **H50 BLE (`FFFF` / `9ABC` / `1234`)** structured protocol on ESP32-S3 on the bench first, before modifying the CMU.

---

## 1. Executive Summary

- **Mazda Connect HUD (CMU150, FW `74.00.324A`) fully supports Lane Guidance** via the OEM D-Bus method `VBS_NAVI_SetRecommLaneReq` in `libjcivbsnaviclient.so` (up to 8 lane slots left-to-right, recommended vs. greyed neighbor arrows).
- **VietMap Live on iPhone does NOT emit Lane Guidance over CarPlay iAP2**, and Mazda's stock `ipoddev_full_auth` on FW `74.00.324A` does not decode iAP2 LaneGuidance anyway.
- **Over BLE HUD**, static analysis (Hopper disassembly of `vietmap_hud_sdk.framework` + Dart AOT snapshot inspection of `App.framework/App`) revealed **two candidate paths**:
  1. **DJBT (`FFF0` / `FFF2`) — JPEG lane strip (`CMD 0x03` / `CMD 0x04`):** App composites all lanes into a single horizontal JPEG strip (no structured lane enum is sent; `CMD 0x05` is dead code). Hard to reverse into 8 discrete Mazda lane codes reliably.
  2. **H50 (`FFFF` / `9ABC` / `1234`, `isH50Type = true`) — Structured `H50Protocol`:** iOS `vietmap_hud_sdk` contains a full native H50 byte-pipe (`connectDeviceNativeApi` + `sendBytesData`) paired with Dart enums **`H50LaneInfoType`**, **`H50ManeuverType`**, and **`H50ProtocolCmd`**. If triggered from an ESP32-S3, this is the cleanest path to structured lane guidance (and potentially numeric turn maneuvers without JPEG decoding).

---

## 2. Mazda HUD Side (Verified in This Repo)

Lane guidance on Mazda Active Driving Display is already implemented and proven in the Android Auto shim (`blmjciaapa`):

- **D-Bus call:** `VBS_NAVI_SetRecommLaneReq(conn, &lane_data, &lane_count, nullptr, nullptr)` — wire signature `(ay)`, taking an 8-byte array (left → right).
  - Reference implementation: [`mazda/patches/blmjciaapa/hud/vbs_tx.cpp`](../mazda/patches/blmjciaapa/hud/vbs_tx.cpp) and [`mazda/patches/blmjciaapa/oem/libjcivbsnaviclient.h`](../mazda/patches/blmjciaapa/oem/libjcivbsnaviclient.h).
- **Code space & glyph mapping:** [`mazda/patches/blmjciaapa/hud/hud_lane.h`](../mazda/patches/blmjciaapa/hud/hud_lane.h)
  - Canonical OEM lane codes `0..70`:
    - `0` = empty/hidden (remapped to `0xFF` on the `VBS_NAVI_SetRecommLaneReq` wire).
    - `1..22` = recommended lane (single arrow, combo arrow, or HOV/bus/taxi).
    - `23..48` = recommended multi-direction lane with one highlighted direction.
    - `49..70` = non-recommended (greyed neighbor) lane (`base + 48`).
  - `oem_lane_glyph(code, OEM_LANE_TABLE_A)` converts each code `0..70` to the cluster glyph ID (`0..35` on Table A / basic cluster; `0..62` on Table B / extended cluster).
- **Synchronization requirement:** `VBS_NAVI_SetRecommLaneReq` carries no `syncBit` of its own. The HUD pairs it with the active `syncBit` generation (`1..7`), so whenever lanes change, `SetHUDDisplayMsgReq` + `SetHUD_Display_Msg2` (with bumped `syncBit`) and `SetRecommLaneReq` must be emitted together.
- **CarPlay shim delta needed later:** Add the `VBS_NAVI_SetRecommLaneReq` thunk to [`mazda/patches/blmjcicarplay/oem/libjcivbsnaviclient.h`](../mazda/patches/blmjcicarplay/oem/libjcivbsnaviclient.h) / `.cpp` (same `libjcivbsnaviclient.so` binary) and extend the serial protocol + `/tmp/vmnav` format with 8 lane bytes.

---

## 3. VietMap Live iOS Findings (Static Binary Evidence)

Evidence from VietMap Live iOS `3.4.3 (179094172)` (`vietmap_hud_sdk.framework`, SHA-256 `fb139532…`; `vietmap_live_navigation_plugin.framework`, SHA-256 `70b0c0e5…`; `App.framework/App`, SHA-256 `2c0e9c89…`) and cross-checked with Android `3.4.5` (`smali_classes2/Da/*.smali`, `libapp.so`):

### 3.1 CarPlay iAP2 — Not Viable
- `VIETMAPCarPlayManager2` does **not** implement `mapTemplateShouldProvideNavigationMetadata:` (iOS 17.4+ opt-in gate; defaults to `false`), so iOS emits zero iAP2 guidance/maneuver packets.
- `vietmap_live_navigation_plugin` renders lanes using its own UI overlay (`LaneInfoView` + PNGs under `lib/assets_v2/lane-info-carplay/`), never `CPLaneGuidance`.
- Mazda's `ipoddev_full_auth` (FW `74.00.324A`) contains no iAP2 LaneGuidance support.

### 3.2 BLE DJBT Path (`FFF0` / `FFF2`) — JPEG Strip Only (No Structured Lane Enum)
- **Structured `CMD 0x05` is unused:**
  - `-[DJBTManager sendRoadInfoWithBackInfo:andSelectedInfo:]` (`0x65f4`) packs `[count, back[], selected[]]` into `CMD 0x05`.
  - However, `0x65f4` has **zero callers** in `vietmap_hud_sdk` (only its own ObjC method-list entry at `0x2e170`; no `_objc_msgSend$sendRoadInfoWithBackInfo:andSelectedInfo:` exists).
  - `"sendRoadInfoWithBackInfo"` has **0 occurrences** in both iOS `App.framework` and Android `libapp.so`.
- **JPEG strip `CMD 0x03` / `CMD 0x04` exists in the SDK:**
  - `DJBTSDKHandler.handle` (`0x1b244`) exposes:
    - `sendRoadImageWithPositionX` → `-[DJBTManager sendRoadImageWithPositionX:andPositionY:andImageData:]` (`0x610c`) → sends **`CMD 0x03`** (`[posX:u16be, posY:u16be, JPEG...]`). Skipped when `hudInfo.protocol` contains `"S"`, enabled when `PROTOCOL:2.2.2`.
    - `sendCancleRoadImageWithPositionX` → `-[DJBTManager cancelShowRoadImage]` (`0x6594`) → sends **`CMD 0x04`** (`5 × 0x00`).
  - Dart (`App.framework`) contains `sendRoadImageWithPositionX`, `sendCancleRoadImageWithPositionX`, `sendHUDLaneImage`, and `cancelHUDLaneImage`.
  - On Android (`Da/m$a.smali`), `genLaneImageWithListBase64` composites `34 × 34 px` per-lane PNG bitmaps side-by-side into a single `(N × 34) × 34` image before JPEG encoding.
  - **Why not preferred:** Reverse-parsing a variable-width composited JPEG strip into 8 Mazda lane codes + recommended/non-recommended highlight bits is fragile compared to structured lane bytes, and real-world emission of `CMD 0x03` on iOS has not yet been captured on a multi-lane junction.

### 3.3 BLE H50 Native Path (`FFFF` / `9ABC` / `1234`) — Structured Protocol (Option 2 Target)
Disassembly of `SwiftVietmapHudSdkPlugin` in `vietmap_hud_sdk.framework` proves iOS ships a dedicated native H50 BLE transport alongside `DJBTManager`:

- **Entry point & mode switch:**
  - `SwiftVietmapHudSdkPlugin.handle` (`0x26814`) handles Flutter method `"connectDeviceNativeApi"` (`{ "address": String, "type": Int }`).
  - `0x24a38` (`connectDeviceNativeApi(address:)`) tail-calls `0x24a48` with `w2 = 1`, setting **`self.isH50Type = true`** (whereas `"connectDevice"` at `0x24a40` passes `w2 = 0` → `isH50Type = false` → `DJBTManager` `FFF0`).
- **GATT takeover when `isH50Type == true`:**
  - In `didConnect` (`0x25ebc`), the plugin sets `peripheral.delegate = self` (taking ownership from `DJBTManager`), discovers all services (`0x28d9c`), and matches (`0x28ec4`):
    - Service UUID: `0000ffff-0000-1000-8000-00805f9b34fb`
    - Write UUID: `00009abc-0000-1000-8000-00805f9b34fb`
    - Notify UUID: `00001234-0000-1000-8000-00805f9b34fb`
  - Emits `"dataTransferReady"` to Dart once characteristics are discovered/subscribed.
- **Raw byte pipe:**
  - `sendBytesData` (`0x24b74`) writes Dart `Uint8List` frames directly to `writeCharacteristic` (`type: .withResponse`) when `isH50Type == true`, and relays notifications back via `"receiveBytesData"`.
- **Structured Dart H50 protocol (confirmed in `App.framework` string pool):**
  - `H50Protocol`, `H50ProtocolCmd.`
  - **`H50LaneInfoType.`**, `sendLaneInfo`
  - **`H50ManeuverType.`**, `H50DirectionInfo(distance: `
  - `H50UpcomingAlertType.`, `H50StickyAlertType.`, `H50BluetoothMode.`, `HUDH50BluetoothType.`, `HUDH50DisplaySectionType.`
- **Open unknowns (because Dart AOT is obfuscated):**
  1. **Exact Dart trigger for `connectDeviceNativeApi` on iOS:**
     - `DJBTManager.didDiscoverPeripheral` (`0xbeb8`) filters only on `[name.uppercaseString containsString:@"HUD"]` and does **not** retain `advertisementData` (so manufacturer data cannot be the selector on iOS).
     - Therefore, Dart must choose `connectDeviceNativeApi` vs `connectDevice` based on the **peripheral name** (or a UI selection).
     - Candidate BLE name literals found in `App.framework`: **`HUD_V3(BLE)`** and **`HUD(BLE)`** (previous trials only tested `VIETMAP-HUD`, `HUD-H50`, and `HUD_V3` without `(BLE)`).
  2. **H50 wire framing & handshake:** Exact framing bytes, init/handshake exchange on `9ABC`/`1234`, and numeric tables for `H50ProtocolCmd`, `H50LaneInfoType`, and `H50ManeuverType`.

---

## 4. Planned Execution Roadmap (Option 2 — Bench-First, Zero CMU Churn)

Do **all** discovery on the bench with **iPhone + ESP32-S3 + USB serial monitor** before touching the car/CMU:

### Phase 1: Trigger `connectDeviceNativeApi` (`FFFF` / `9ABC` / `1234`) on iPhone
1. On ESP32-S3 (test branch or serial NVS override), advertise GATT service `FFFF` (`9ABC` write / `1234` notify) and log raw hex of every write to `9ABC` and subscription to `1234`.
2. Test candidate BLE names containing `"HUD"` (so `DJBTManager`'s native scan filter passes them to Dart):
   - `HUD_V3(BLE)`
   - `HUD(BLE)`
   - `VIETMAP-H50-HUD` / `HUD-H50(BLE)`
3. Watch serial log when tapping the device in VietMap Live iOS (*Settings → Device Settings → HUD → Connect*):
   - **Success signal:** iPhone subscribes to **`1234`** (instead of `FFF1`) and writes handshake bytes to **`9ABC`** (instead of `FFF2`).
   - If none of those names triggers `1234`/`9ABC`, inspect the Dart AOT callsite of `connectDeviceNativeApi` in `App.framework` (using an ARM64 Dart object-pool tracer or Frida hook on `SwiftVietmapHudSdkPlugin.handle` on a test device) to read the exact name predicate.

### Phase 2: Capture & Decode `H50Protocol` Frames
1. Answer the H50 handshake on `1234` (if required) and start a simulated or real route through multi-lane junctions in VietMap Live iOS.
2. Log every raw frame arriving on `9ABC` alongside screenshots of the VietMap banner (turn arrow + lane bar + distance + speed limit).
3. Map out:
   - Frame header, length, command byte (`H50ProtocolCmd`), checksum/CRC.
   - `H50LaneInfoType` byte values → `OemLaneCode` (`0..70` in `hud_lane.h`).
   - `H50ManeuverType` byte values → `MazdaIcon` (`1..60` in `hud_send.cpp`) — bonus: if clean, can replace JPEG arrow matching!
4. **Fallback check during the same drive:** On `FFF0`, also log raw hex/dimensions of any incoming `CMD 0x03` (`sendRoadImageWithPositionX`) and `CMD 0x04` (`cancelShowRoadImage`) frames to confirm whether VietMap iOS emits lane images on `FFF0` at multi-lane junctions.

### Phase 3: Wire Lane Guidance into ESP32-S3 → `splim_seriald` → `blmjcicarplay` (Single CMU Install)
Once Phase 2 yields verified lane codes:
1. **Serial protocol extension (ESP32-S3 → `splim_seriald`):**
   - `LANE <c0> <c1> <c2> <c3> <c4> <c5> <c6> <c7>\n` (8 `OemLaneCode` values `0..70`; `LANE OFF\n` to clear).
2. **`splim_seriald.c`:**
   - Atomically write `/tmp/vmlane` (`"<c0> <c1> <c2> <c3> <c4> <c5> <c6> <c7> <unix_ts>\n"`) and unlink on `LANE OFF` / disconnect.
3. **`blmjcicarplay`:**
   - Add `VBS_NAVI_SetRecommLaneReq` thunk to `mazda/patches/blmjcicarplay/oem/libjcivbsnaviclient.{h,cpp}` (identical ABI to `blmjciaapa`).
   - Reuse [`mazda/patches/blmjciaapa/hud/hud_lane.h`](../mazda/patches/blmjciaapa/hud/hud_lane.h) (`oem_lane_glyph(code, OEM_LANE_TABLE_A)`, `0 -> 0xFF`).
   - In `hud_send.cpp`, poll `/tmp/vmlane` alongside `/tmp/vmnav`, bump `sync_bit` on lane changes, and emit `SetHUDDisplayMsgReq` + `SetHUD_Display_Msg2` + `SetRecommLaneReq` in the same sync generation.
   - Add host unit tests covering lane parsing, clearing, and sync-bit pairing.
