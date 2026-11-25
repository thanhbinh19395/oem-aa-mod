# Spec Delta

## Purpose

Defines the Android Auto head-unit shim (`libpatch-blmjciaapa.so`) injected into `jciAAPA`, providing touchscreen passthrough, HUD navigation guidance, wireless GAL 1.6 session activation, media mute/play-pause control, and debug navigation bus monitoring.

## ADDED Requirements

### Requirement: Android Auto touchscreen passthrough
When `touch = true` (default) in `libpatch.conf`, the shim SHALL read touchscreen input events from the CMU evdev device during an active Android Auto session and forward them to the Android Auto projection session.

#### Scenario: User touches screen during active Android Auto session
- **WHEN** an Android Auto session is active and `touch = true`
- **THEN** touch press, move, and release events are forwarded to the phone's projected interface

#### Scenario: Touch passthrough disabled in configuration
- **WHEN** `touch = false` in `libpatch.conf`
- **THEN** the shim does not start the evdev touch reader thread

### Requirement: Android Auto HUD navigation forwarding
When `hud = true` (default) in `libpatch.conf`, the shim SHALL forward Android Auto turn-by-turn navigation guidance (maneuver icon, distance, street name, and lane guidance when available) to the Mazda HUD via the configured `hud_transport` (`svcnavi` default, or `vbs` for cardless direct writes), applying Latin folding when `hud_fold_latin = true` (default).

#### Scenario: Guidance received over AA 1.5 or relayed GAL 1.6
- **WHEN** navigation updates arrive from Android Auto 1.5 callbacks or the `aap_service` GAL 1.6 AF_UNIX relay socket
- **THEN** the shim translates maneuver, distance, lane guidance, and folded street name into Mazda HUD frames and transmits them using the configured `hud_transport`

#### Scenario: Session ends abnormally without stop event
- **WHEN** an Android Auto session tears down or restarts after an abrupt disconnect
- **THEN** the shim resets its GAL 1.6 navigation accumulator and change-gate so stale state does not suppress the next session's first frame

### Requirement: AAWireless GAL 1.6 Bluetooth-pairing bypass
When GAL 1.6 navigation is enabled and the connected USB device is an `AAWireless` dongle, the shim SHALL activate the Android Auto session automatically once it reaches the pending-pairing state, compensating for phones that skip in-band Bluetooth pairing over wireless GAL 1.6.

#### Scenario: Wireless GAL 1.6 session stalls awaiting in-band BT pairing
- **WHEN** `use_protocol_v1_6 = true`, an `AAWireless` dongle is connected, and projection setup enters the pending-pairing state
- **THEN** the shim triggers session activation so video focus and projection start normally without disconnecting

#### Scenario: Wired USB connection or GAL 1.5 configuration
- **WHEN** the phone is connected directly over USB or `use_protocol_v1_6 = false`
- **THEN** the bypass does not activate and stock Bluetooth pairing behavior is preserved

### Requirement: Mute-to-media pause and resume bridge
When `mute_pauses_phone = true` (default) in `libpatch.conf`, the shim SHALL monitor the OEM audio manager mute state and send Android Auto a media `PAUSE` command on user mute and a media `PLAY` command on unmute, acting only while Android Auto is the active audio source and only resuming playback that it paused.

#### Scenario: User mutes and unmutes while Android Auto media is active
- **WHEN** Android Auto is the active entertainment source and the user toggles mute on and then off
- **THEN** the shim sends a media `PAUSE` on mute and a media `PLAY` on unmute

#### Scenario: User mutes while another audio source is active
- **WHEN** FM radio or another non-Android Auto source is active and the user toggles mute
- **THEN** the shim does not send media pause or play commands to Android Auto

### Requirement: Block head-unit-generated Android Auto MEDIA_PLAY
When `block_headunit_media_play = true` (default `false`) in `libpatch.conf`, the shim SHALL suppress CMU-generated `MEDIA_PLAY` commands (including startup restore, source activation, and focus regain) while still allowing its own unmute-resume command and user-initiated playback from the phone or projected UI.

#### Scenario: CMU attempts automatic playback resume on connect or focus regain
- **WHEN** `block_headunit_media_play = true` and the head unit emits an automatic `MEDIA_PLAY` command
- **THEN** the shim blocks the command while keeping Android Auto connected and selected

### Requirement: Debug-only D-Bus navigation bus monitor
In debug builds (`-DDEBUG`), the shim SHALL spawn a `dbus-monitor` eavesdropper on the JCI service bus during an active Android Auto session to log messages addressed to `com.jci.vbs.navi` and `com.jci.vbs.navi.tmc`; in release builds this monitor SHALL be compiled out as a no-op.

#### Scenario: Session starts in a debug build
- **WHEN** an Android Auto session starts with a `-DDEBUG` build of `libpatch-blmjciaapa.so`
- **THEN** `dbus-monitor` captures navigation bus traffic into the shim log and terminates cleanly when the session ends
