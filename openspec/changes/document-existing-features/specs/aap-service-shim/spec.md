# Spec Delta

## Purpose

Defines the Android Auto projection daemon shim (`libpatch-aap_service.so`) injected into `aap_service`, providing GAL protocol 1.6 upgrade with navigation frame relay and optional low-latency ALSA audio playback startup.

## ADDED Requirements

### Requirement: Android Auto GAL 1.6 negotiation and navigation relay
When `use_protocol_v1_6 = true` (default `false`) in `libpatch.conf`, the shim SHALL advertise Android Auto GAL protocol version 1.6 instead of the stock 1.5 during session negotiation and SHALL relay incoming GAL 1.6 navigation frames (maneuver, lanes, distance, ETA) over an abstract `AF_UNIX` datagram socket to `libpatch-blmjciaapa.so`.

#### Scenario: GAL 1.6 enabled on Android Auto connection
- **WHEN** `use_protocol_v1_6 = true` and a phone negotiates an Android Auto session
- **THEN** `aap_service` advertises GAL 1.6 and forwards received 1.6 navigation payloads to the HUD receiver in `jciAAPA`

#### Scenario: GAL 1.6 disabled in configuration
- **WHEN** `use_protocol_v1_6 = false`
- **THEN** the navigation upgrade module remains inert and `aap_service` negotiates stock GAL 1.5

### Requirement: Low-latency Android Auto ALSA playback start threshold
When `aa_audio_low_latency = true` (default `false`) in `libpatch.conf`, the shim SHALL lower the per-client ALSA playback `start_threshold` from the full buffer size to one negotiated period exclusively for Android Auto playback PCM handles (`androidautoMainAudio`, `androidautoMainAudioVR`, `androidautoAltAudio`), without altering shared dmix buffer geometry or non-AA PCM handles.

#### Scenario: Android Auto opens a tracked playback PCM
- **WHEN** `aa_audio_low_latency = true` and `aap_service` configures software parameters on `androidautoMainAudio`, `androidautoMainAudioVR`, or `androidautoAltAudio`
- **THEN** the shim sets the ALSA start threshold to one negotiated period (approximately 32 ms at 48 kHz / 1536-frame period instead of 128 ms)

#### Scenario: Non-Android Auto PCM or feature disabled
- **WHEN** `aa_audio_low_latency = false` or an unrecognized PCM handle is opened
- **THEN** ALSA software parameters are passed through unchanged
