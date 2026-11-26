# Proposal

## Why

While `vmhud-esp32-s3` already emulates a VietMap BLE HUD (`VIETMAP-HUD` on `FFF0`) and forwards posted speed limits and turn-by-turn guidance over USB CDC to the Mazda CMU, drivers using Apple CarPlay also want posted speed limits, upcoming speed-limit changes, and approaching traffic/prohibition signs (such as no-left-turn, no-right-turn, no-U-turn, no-entry, no-passing, residential-area, and camera alerts with approaching distance) synced directly to the `CarPlay Helper` iOS app's CarPlay Live Activity alongside tire pressure telemetry—without modifying Mazda CMU binaries or touching Android Auto. On a single iPhone running both `VietMap Live` and `CarPlay Helper`, the BLE Link Layer permits only one ACL connection per `(iPhone Initiator, ESP32 Peripheral)` pair while iOS `bluetoothd` multiplexes multiple apps over that single connection across separate GATT services—requiring `vmhud-esp32-s3` to host an isolated `FFF8`/`FFF9` telemetry GATT service on the shared connection plus a non-connectable `FFF8` wakeup beacon so `CarPlay Helper` attaches only after `VietMap Live` connects and releases cleanly when `VietMap Live` disconnects.

## What Changes

- Expose a dedicated GATT service `FFF8` with `READ | NOTIFY` telemetry characteristic `FFF9` on `vmhud-esp32-s3` alongside `FFF0` (which iOS `vietmap_hud_sdk` ignores during service discovery), allowing iOS `bluetoothd` to share the single physical BLE connection between `VietMap Live` (`FFF2`/`FFF1` RX/handshake) and `CarPlay Helper` (`FFF9` TX notify) on the same iPhone.
- Emit a non-connectable BLE 5.0 wakeup advertisement (`ADV_NONCONN_IND` carrying service UUID `FFF8` via `NimBLEExtAdvertising` Instance 1) whenever `VietMap Live` is connected on `FFF0` (`HUDInfo` active) and `FFF9` has no subscriber yet, waking `CarPlay Helper` in the background to piggyback via `retrieveConnectedPeripherals` without attempting a second Link-Layer ACL connection from the same iPhone.
- Release the shared BLE link cleanly when `VietMap Live` unsubscribes from `FFF1` (sending a final `vietmapConnected = false` notification on `FFF9` followed by disconnecting any lingering shared link after a short grace delay) so `VIETMAP-HUD` immediately resumes connectable advertising for future `VietMap Live` scans.
- Stream a compact 12-byte binary frame (`0xA7` magic + XOR checksum, fitting within standard 20-byte ATT MTU) on `FFF9` whenever speed limits, overspeed state, prohibition/warning signs, or discrete approaching distance buckets change, plus a 5-second keepalive.
- Extend on-chip VietMap iOS BLE sign & distance extraction in `lib/vmhud/`:
  - Expand the quantized 16×16 luma/chroma NCC sign classifier to recognize VietMap HUD prohibition, warning, area, and camera JPEGs sent via `CMD 0x08` (`sendImageWithPositionX`) and `CMD 0x41` (`sendCustomeCameraInformation`), cleared on `CMD 0x09` (`cancelshowImage`) or staleness timeout.
  - Parse approaching sign distance text frames on `CMD 0x06` (`sendTextWithPositionX`, e.g., `"250 m"`, cleared on `CMD 0x07` / `CMD 0x47`) and fuse with `CMD 0x01` / `CMD 0x41` meter distances.
- Keep Mazda CMU (`splim_seriald`, `blmjcicarplay`) and all Android Auto components completely untouched.

## Capabilities

### New Capabilities
- `esp32-carplay-ble-telemetry`: Shared-connection `FFF8`/`FFF9` GATT binary telemetry service, non-connectable `FFF8` background wakeup beacon, clean VietMap disconnect release, and on-chip classification of VietMap Live (iOS) speed limits, prohibition/warning signs, and approaching distances.

### Modified Capabilities
<!-- No synced specs currently exist under openspec/specs/ in oem-aa-mod. -->

## Impact

- **Firmware (`firmware/vmhud-esp32-s3/`)**:
  - `platformio.ini`: Enable `CONFIG_BT_NIMBLE_EXT_ADV=1`, `CONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES=2`, and `CONFIG_BT_NIMBLE_MAX_CONNECTIONS=3` across S3/C3 environments.
  - `lib/vmhud/`: Extend template generator & header-only classifier for prohibition/warning HUD sign assets from the iOS IPA (`lib/assets/hud/*.png`), add `CMD 0x06`/`0x07`/`0x47` distance text parser, and add 12-byte `cph_telemetry` frame encoder + distance-bucket delta policy.
  - `src/main.cpp` & `src/display_ui.cpp`: Host `FFF8`/`FFF9` alongside `FFF0`, track `g_subH1` and `g_subCph` independently on the shared `connHandle` (plus optional second central for bench testing), manage the non-connectable `FFF8` wakeup beacon, and release lingering connections when `VietMap Live` unsubscribes `FFF1`.
- **Host Tests (`test/vietmap_ble_hud_test.cpp`)**: Add unit tests for prohibition sign template classification, `CMD 0x06` distance text parsing, and 12-byte `FFF9` frame encoding/bucketing.
- **Out of Scope**: Android Auto shims (`blmjciaapa`, `aap_service`), VietMap Android APK, and Mazda CMU C/C++ binaries (`splim_seriald`, `blmjcicarplay`).
