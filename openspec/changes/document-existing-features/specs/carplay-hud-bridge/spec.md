# Spec Delta

## Purpose

Defines the CarPlay-to-HUD bridge shim (`libpatch-blmjcicarplay.so`) injected into `jciCARPLAY`, decoding Apple iAP2 turn-by-turn navigation, normalizing Vietnamese street names, displaying VietMap posted and upcoming speed limits, and falling back to VietMap BLE turn-by-turn guidance.

## ADDED Requirements

### Requirement: Apple CarPlay iAP2 turn-by-turn HUD decoding
The shim SHALL intercept CarPlay iAP2 navigation guidance (`0x8058`) and maneuver list (`0x8059`) messages in `jciCARPLAY`, select the authoritative imminent maneuver via `maneuverList[0]`, decode roundabout chosen-exit bearings from `junctionElementExitAngle`, and forward maneuver icons, distances, and road names to `com.jci.vbs.navi`.

#### Scenario: Active CarPlay navigation emits maneuver burst and guidance update
- **WHEN** CarPlay streams a maneuver burst (`0x8059`) and guidance message (`0x8058`) with a valid `maneuverList[0]` index
- **THEN** the shim selects the matching buffered maneuver (or holds the previous selection during a transient rebuild) and updates the Mazda HUD with the turn icon, distance, and road name

#### Scenario: Roundabout maneuver with exit bearing
- **WHEN** a maneuver indicates a roundabout junction with a signed exit bearing (`junctionElementExitAngle`)
- **THEN** the shim maps the bearing and drive side to the corresponding Mazda HUD roundabout glyph

### Requirement: CarPlay Vietnamese street-name normalization
When `carplay_vn_normalize = true` (default) in `libpatch-carplay.conf`, the shim SHALL fold 3-byte UTF-8 Vietnamese street-name characters into HUD-renderable forms before sending them to the HUD, replacing unfoldable multi-byte sequences with `?`.

#### Scenario: CarPlay road name contains stacked Vietnamese diacritics
- **WHEN** `carplay_vn_normalize = true` and a CarPlay road name contains characters such as `ề`, `ớ`, or `ấ`
- **THEN** the shim folds them to renderable forms (`ê`, `ơ`, `â`) before transmitting the street strip

#### Scenario: Vietnamese normalization disabled
- **WHEN** `carplay_vn_normalize = false`
- **THEN** road names are sent to the HUD unmodified

### Requirement: Posted speed-limit HUD display and daemon auto-launch
When `carplay_speed_limit = true` (default) in `libpatch-carplay.conf`, the shim SHALL ensure `splim_seriald` is running, read the current posted speed limit from `/tmp/splim`, display it on the HUD speed-limit circle, and drop values older than 8 seconds.

#### Scenario: Fresh posted speed limit available
- **WHEN** `carplay_speed_limit = true` and `/tmp/splim` contains a valid limit (5–200 km/h) updated within the last 8 seconds
- **THEN** the shim includes that speed limit in HUD display frames

#### Scenario: Speed limit becomes stale or feature is disabled
- **WHEN** `/tmp/splim` is older than 8 seconds or `carplay_speed_limit = false`
- **THEN** the shim omits the speed-limit sign from the HUD

### Requirement: Upcoming speed-limit 10-second alert and deferred navigation queue
When both `carplay_speed_limit = true` and `carplay_next_speed_limit = true` (default) in `libpatch-carplay.conf`, the shim SHALL display a newly triggered upcoming speed limit from `/tmp/vmnextsplim` with the label `"Tốc độ tiếp theo"` for exactly 10 seconds, queueing any concurrent turn-by-turn updates (latest-wins) or in-app navigation clears until the 10-second alert completes.

#### Scenario: New upcoming speed-limit alert arrives during active navigation
- **WHEN** `/tmp/vmnextsplim` delivers a new monotonic `alert_id` with a valid upcoming speed limit
- **THEN** the HUD displays the upcoming limit and `"Tốc độ tiếp theo"` for 10 seconds, updating distance on repeated frames of the same `alert_id` without extending the 10-second timer, and flushes any queued navigation update or deferred clear immediately upon expiry

#### Scenario: Full CarPlay session teardown during active alert
- **WHEN** the CarPlay session ends or the phone is unplugged while a 10-second alert is active
- **THEN** the shim aborts the alert immediately and clears the HUD

### Requirement: VietMap Live BLE turn-by-turn fallback
When `carplay_vietmap_nav = true` (default) in `libpatch-carplay.conf`, the shim SHALL auto-start `splim_seriald` and display VietMap Live turn-by-turn guidance from `/tmp/vmnav` (dropping records older than 6 seconds) whenever CarPlay iAP2 navigation is not actively guiding.

#### Scenario: VietMap BLE guidance active while CarPlay iAP2 is idle
- **WHEN** `carplay_vietmap_nav = true`, CarPlay iAP2 is not guiding, and `/tmp/vmnav` has a fresh maneuver within 6 seconds
- **THEN** the shim renders the VietMap maneuver icon, distance, and normalized road name on the Mazda HUD

#### Scenario: CarPlay iAP2 navigation becomes active
- **WHEN** Apple Maps or Google Maps starts active CarPlay iAP2 guidance while `/tmp/vmnav` is present
- **THEN** CarPlay iAP2 guidance takes priority over VietMap BLE fallback
