# Spec Delta

## Purpose

Defines the `jciMMUI` Audio-Off fallback shim (`libpatch-svcjcimmui.so`), preventing the CMU from automatically switching to noisy broadcast radio (FM/AM/SiriusXM/DAB) on ignition boot, phone/USB disconnect, or reconnect-timer expiry.

## ADDED Requirements

### Requirement: Suppress automatic broadcast radio fallback on boot, disconnect, and timer expiry
When `fallback_audio_off = true` (default `false`) in `libpatch.conf`, the shim SHALL intercept `jciMMUI` audio source fallback paths so that startup activation (`ACC ON`), CarPlay/Android Auto/USB disconnect, and reconnect-timer expiry select stock Audio Off (`SelectEntertainmentNoAudio`) instead of falling back to broadcast radio.

#### Scenario: Vehicle boots while CarPlay or Bluetooth is not yet ready
- **WHEN** `fallback_audio_off = true`, the CMU boots (`ACC ON`), and the last active audio source was not broadcast radio
- **THEN** the shim prevents fallback to FM/AM/SiriusXM/DAB and enters stock Audio Off

#### Scenario: Phone or USB media disconnects or reconnect timer expires
- **WHEN** `fallback_audio_off = true` and disconnecting CarPlay, Android Auto, or USB (or reconnect-timer expiry) would activate a broadcast radio source
- **THEN** the shim injects `SelectEntertainmentNoAudio` so the head unit stays silent

#### Scenario: User was intentionally listening to broadcast radio
- **WHEN** `fallback_audio_off = true` and broadcast radio was the user's active source prior to shutdown
- **THEN** the shim preserves broadcast radio playback on startup

#### Scenario: Feature disabled in configuration
- **WHEN** `fallback_audio_off = false`
- **THEN** stock CMU audio source fallback behavior is left untouched
