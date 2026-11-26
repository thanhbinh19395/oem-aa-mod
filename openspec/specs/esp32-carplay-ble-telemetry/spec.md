# esp32-carplay-ble-telemetry Specification

## Purpose

Exposes a dedicated, non-conflicting Bluetooth Low Energy telemetry channel on the ESP32 bridge to stream decoded VietMap iOS speed limits, prohibition signs, and approaching distances to CarPlay Helper on the same iPhone.

## Requirements

### Requirement: Shared-connection GATT coexistence and background wakeup beacon
The firmware SHALL expose GATT service `FFF8` alongside `FFF0` on the same BLE peripheral so iOS `bluetoothd` multiplexes `VietMap Live` (`FFF0`) and `CarPlay Helper` (`FFF8`) over one ACL connection, emit a non-connectable `FFF8` wakeup advertisement only while `VietMap Live` is active and `FFF9` is unsubscribed, and release the ACL link if `VietMap Live` unsubscribes `FFF1` so `VIETMAP-HUD` resumes connectable advertising.

#### Scenario: Both VietMap Live and CarPlay Helper share one iPhone BLE connection
- **WHEN** `VietMap Live` connects to `VIETMAP-HUD` (`FFF0`) on an iPhone and completes the `HUDInfo` handshake
- **THEN** the firmware activates the non-connectable `FFF8` wakeup beacon until `CarPlay Helper` subscribes to `FFF9` on that same connection, delivering independent `FFF1` and `FFF9` notifications without creating a second ACL link

#### Scenario: VietMap Live disconnects while CarPlay Helper is subscribed
- **WHEN** `VietMap Live` disconnects or unsubscribes `FFF1` while `CarPlay Helper` still holds a subscription on `FFF9`
- **THEN** the firmware notifies `FFF9` with `vietmapConnected = false` and cleared signs, disconnects the lingering ACL handle if still open after `1.5` seconds, and restarts connectable `VIETMAP-HUD` advertising

### Requirement: Compact binary telemetry read and notify stream
The firmware SHALL expose GATT service `FFF8` with characteristic `FFF9` (`READ | NOTIFY`) encoding a compact binary frame (`0xA7` magic, `10 + textLen` payload length, status flags, posted speed limit, GPS speed, upcoming speed limit and distance, active traffic/prohibition sign code and distance, raw UTF-8 `CMD 0x06` overlay text `0..15` bytes, and trailing XOR checksum).

#### Scenario: Initial GATT read on CarPlay Helper connection
- **WHEN** CarPlay Helper attaches to `FFF8` and reads characteristic `FFF9`
- **THEN** the firmware returns a valid `0xA7` frame with an accurate trailing XOR byte reflecting current VietMap telemetry state

#### Scenario: Immediate notify on speed limit or sign change with periodic keepalive
- **WHEN** a subscribed client is listening on `FFF9` and posted speed limit, overspeed state, upcoming speed limit, active sign code, raw overlay text, or numeric distance bucket changes (or `5` seconds elapse without change)
- **THEN** the firmware sends a single unfragmented BLE notification on `FFF9`

### Requirement: On-device classification of VietMap speed and prohibition signs
The firmware SHALL classify VietMap iOS HUD sign images received on `CMD 0x08` and `CMD 0x41` into posted/upcoming speed limits (`30..120` km/h) and discrete prohibition/warning sign codes (no-left-turn, no-right-turn, no-left/right-turn, no-U-turn, turn+U-turn bans, no-entry variants, no-passing start/end, residential start/end, no-parking, toll, railway, tunnel, and cameras), clearing on `CMD 0x09` or timeout.

#### Scenario: Prohibition turn sign received from VietMap Live
- **WHEN** VietMap Live sends a `CMD 0x08` JPEG matching a no-left-turn, no-right-turn, or no-U-turn HUD asset
- **THEN** the firmware classifies the corresponding `activeSignCode` and emits an updated telemetry notification without altering the posted speed limit

#### Scenario: Sign cancelled by VietMap Live
- **WHEN** VietMap Live sends `CMD 0x09` (`cancelshowImage`) or the sign source exceeds its staleness window
- **THEN** the firmware clears the image-derived sign state and notifies `activeSignCode = 0` unless a camera alert from `CMD 0x01` is still active

### Requirement: Approaching sign distance extraction and bucketed emission
The firmware SHALL forward raw UTF-8 overlay text from VietMap iOS `CMD 0x06` verbatim (e.g., `"250 m"`, `"1.2 km"`, cleared on `CMD 0x07`, `CMD 0x47`, or `CMD 0x09`) alongside numeric distances from `CMD 0x01` and `CMD 0x41`, and trigger notifications on raw text changes or discrete numeric distance-bucket transitions rather than every single GPS tick.

#### Scenario: Overlay text provides countdown distance for prohibition sign
- **WHEN** VietMap Live sends a no-left-turn JPEG on `CMD 0x08` accompanied by `"1.2 km"` or `"200 m"` on `CMD 0x06`
- **THEN** the firmware pairs the raw `activeSignText` verbatim with the no-left-turn sign code in the `FFF9` telemetry frame without converting to integer meters

#### Scenario: Distance countdown triggers notifications at discrete buckets
- **WHEN** an approaching sign counts down from `295m` to `285m` within the same distance bucket (`250..300m`) and no other field changes
- **THEN** the firmware suppresses intermediate BLE notifications until the distance crosses into the next bucket (such as `<= 250m`) or the keepalive interval elapses

### Requirement: Zero regression on CMU USB serial telemetry
The firmware SHALL preserve existing `SPLIM`, `NEXTSPLIM`, `NAV`, `NAV OFF`, and `PING` USB CDC serial transmissions to the Mazda CMU without blocking or interleaving lines while servicing `FFF9` BLE subscribers.

#### Scenario: Simultaneous CMU serial output and CarPlay Helper BLE notify
- **WHEN** both Mazda CMU (`splim_seriald` over USB CDC) and CarPlay Helper (`FFF9` over BLE) are connected during active VietMap navigation
- **THEN** the firmware continues emitting valid USB CDC `SPLIM` / `NEXTSPLIM` / `NAV` lines within existing keepalive windows while notifying `FFF9` subscribers
