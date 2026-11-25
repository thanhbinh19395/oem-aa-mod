# Proposal

## Why

`oem-aa-mod` has evolved into a multi-component system spanning five Mazda CMU `LD_PRELOAD` shims, an ESP32 BLE HUD bridge firmware, a CMU USB serial daemon, and a 3-tier standalone USB installer, but none of its existing capabilities are currently documented in OpenSpec. Establishing baseline specifications in OpenSpec gives future changes a durable source of truth for behavioral contracts, runtime configuration keys, and cross-process boundaries.

## What Changes

- Document all existing features of `oem-aa-mod` as eight durable OpenSpec capability specifications covering the CMU shims, ESP32/CMU VietMap bridge, runtime configuration infrastructure, and USB installer/recovery system.
- No runtime code, firmware, or installer behavior is modified by this change; it strictly captures current shipped behavior as baseline specs.

## Capabilities

### New Capabilities
- `runtime-config-and-preload`: Shared `LD_PRELOAD` process self-gating, fatal-signal crash diagnostics, sibling `.conf` runtime configuration loading, and HUD Latin Extended Additional street-name transliteration.
- `android-auto-shim`: `libpatch-blmjciaapa.so` features inside `jciAAPA` — evdev touch passthrough, AA 1.5/1.6 HUD guidance forwarding over `svcnavi`/`vbs`, AAWireless GAL 1.6 Bluetooth-pairing bypass, mute-to-media play/pause bridge, CMU auto `MEDIA_PLAY` blocking, and debug D-Bus navi monitoring.
- `aap-service-shim`: `libpatch-aap_service.so` features inside `aap_service` — Android Auto GAL 1.6 protocol upgrade with raw navigation frame relay over AF_UNIX and optional low-latency ALSA playback start threshold.
- `svcjcinavi-hud-merge`: `libpatch-svcjcinavi.so` features inside `svcjcinavi` — flicker-free merging of Android Auto HUD maneuvers/streets with OEM navigation speed limits when the nav SD card is present, plus optional `force_street_name`.
- `carplay-hud-bridge`: `libpatch-blmjcicarplay.so` features inside `jciCARPLAY` — Apple iAP2 turn-by-turn decoding (`maneuverList[0]`, roundabout exit angle `0x346`), Vietnamese street normalization, posted speed-limit HUD display, 10-second upcoming speed-limit alert with deferred navigation queue, and VietMap BLE turn-by-turn fallback.
- `audio-off-fallback`: `libpatch-svcjcimmui.so` features inside `jciMMUI` — inline hooks in `libjciuiasystem.so` that select stock Audio Off (`SelectEntertainmentNoAudio`) instead of broadcast radio (FM/AM/SiriusXM/DAB) on boot, disconnect, and reconnect-timer expiry.
- `vietmap-ble-serial-bridge`: End-to-end VietMap Live (iPhone) BLE-to-CMU bridge comprising the `vmhud-esp32-s3` BLE HUD emulator (codec, JPEG arrow and speed-sign classifiers, optional TFT/OLED telemetry UI, MAC-pinned flashing) and the CMU `splim_seriald` USB CDC daemon writing atomic tmpfs IPC files.
- `usb-installer-and-recovery`: Standalone 3-tier USB installer (CVE-2024-8359 `.up` injection, MP3 ID3 XSS overlay, legacy `dataRetrieval_config.txt`), interactive `jci-dialog` install/uninstall and temporary SSH provisioning, zero-touch emergency Auto-Revert USB, and host/on-device verification tooling.

### Modified Capabilities
<!-- None — this change establishes the initial baseline specs for the repository. -->

## Impact

- **OpenSpec artifacts**: Adds `proposal.md` and eight baseline capability specs under `openspec/changes/document-existing-features/specs/`.
- **Codebase**: None (documentation-only baseline; zero source or binary changes).
