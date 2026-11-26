# Tasks

## 1. Prohibition Sign Classifier & Distance Text Parser (`lib/vmhud/`)

- [x] 1.1 Add template generation script (`firmware/vmhud-esp32-s3/scripts/gen_traffic_sign_templates.py` or extend `gen_speed_sign_templates.py`) reading VietMap iOS HUD assets (`lib/assets/hud/*.png`) to emit quantized descriptors for prohibition, area, road, and camera signs (`VietMapSignCode` `0x01..0x21`); verify the generated header compiles cleanly and covers all target sign codes.
- [x] 1.2 Extend `lib/vmhud/` (`Box16Accumulator` / `speed_sign_classifier.h` or `traffic_sign_classifier.h`) to classify non-speed JPEGs from `CMD 0x08` and `CMD 0x41` into `VietMapSignCode` using static stack/BSS buffers only, and add host unit tests in `test/vietmap_ble_hud_test.cpp` verifying zero pairwise confusion across all VietMap HUD sign assets (`make -C test vietmap_ble_hud_test && test/build/vietmap_ble_hud_test`).
- [x] 1.3 Add `CMD 0x06` (`sendTextWithPositionX`) UTF-8 meter/km distance parser and `CMD 0x07` / `CMD 0x47` clear helpers in `lib/vmhud/hud_codec.h`, and verify via unit tests in `test/vietmap_ble_hud_test.cpp` that strings such as `"250 m"`, `"150m"`, and `"1.2 km"` decode accurately while non-distance strings are ignored.

## 2. Compact 12-Byte `FFF9` Binary Protocol & Delta Policy (`lib/vmhud/`)

- [x] 2.1 Implement `lib/vmhud/cph_telemetry.h` encoding the 12-byte `0xA7 0x0A` telemetry frame (status flags, overspeed hysteresis, posted/upcoming speed limits, `activeSignCode`, fused `activeSignDistM`, and trailing XOR checksum) plus discrete approaching distance bucket transitions (`500m..50m`), and verify packet encoding, XOR checksum, and bucket suppression in `test/vietmap_ble_hud_test.cpp`.

## 3. Shared GATT Service `FFF8`/`FFF9` & Non-Connectable Wakeup Beacon (`firmware/vmhud-esp32-s3/`)

- [x] 3.1 Update `firmware/vmhud-esp32-s3/platformio.ini` with `-DCONFIG_BT_NIMBLE_EXT_ADV=1`, `-DCONFIG_BT_NIMBLE_MAX_EXT_ADV_INSTANCES=2`, and `-DCONFIG_BT_NIMBLE_MAX_CONNECTIONS=3`, and verify `pio run -e esp32-s3-supermini` compiles NimBLE extended advertising support cleanly.
- [x] 3.2 Update `firmware/vmhud-esp32-s3/src/main.cpp` to expose GATT service `FFF8` / characteristic `FFF9` (`READ | NOTIFY`) alongside `FFF0`, track `g_subH1` and `g_subCph` independently on the shared iPhone `connHandle` (plus optional secondary bench central), start/stop Instance 1's non-connectable `FFF8` wakeup beacon when `g_hudInfoOk && !g_subCph`, and enforce the `1.5 s` lingering-ACL release watchdog when `g_subH1` unsubscribes; verify all four PlatformIO environments build without errors.
- [x] 3.3 Wire `CMD 0x06`/`0x07`/`0x47`, `CMD 0x08`/`0x09`, `CMD 0x41` embedded JPEGs, and `cph_telemetry` `FFF9` notify/keepalive emission into `loop()` in `src/main.cpp` (plus `firmware/vmhud-esp32-s3/README.md` documentation), verifying `SPLIM` / `NEXTSPLIM` / `NAV` USB CDC output remains non-blocking.

## 4. End-to-End Firmware Verification

- [x] 4.1 Run the complete host test suite (`make -C test vietmap_ble_hud_test && test/build/vietmap_ble_hud_test`) and full PlatformIO multi-board build (`pio run` in `firmware/vmhud-esp32-s3`), verifying zero warnings/errors and confirming SRAM/Flash usage fits within ESP32-S3 and ESP32-C3 limits.

## Workflow follow-up

- Archive `add-esp32-carplay-ble-telemetry` after bench/in-car verification with `CarPlay Helper` (`add-vietmap-speed-signs-live-activity`) succeeds.
