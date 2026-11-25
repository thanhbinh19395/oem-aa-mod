# Spec Delta

## Purpose

Defines the VietMap Live (iPhone) BLE-to-CMU bridge consisting of the `vmhud-esp32-s3` ESP32 BLE HUD emulator firmware and the CMU `splim_seriald` USB CDC daemon that delivers speed limits, upcoming speed-limit warnings, and turn-by-turn navigation to `libpatch-blmjcicarplay.so`.

## ADDED Requirements

### Requirement: ESP32 VietMap BLE HUD emulation and packet decoding
The `vmhud-esp32-s3` firmware SHALL advertise and emulate a VietMap accessory HUD over BLE (default name `VIETMAP-HUD`, model `H50`, service `FFF0`), decode incoming VietMap HUD protocol frames, and extract posted speed limits, upcoming speed-limit warnings (from camera/custom packets and alert JPEG sign classification), GPS speed, road names, and turn-by-turn maneuvers (including JPEG arrow classification).

#### Scenario: VietMap Live connects over BLE and sends HUD telemetry
- **WHEN** VietMap Live on iOS connects to the ESP32 BLE peripheral and streams speed-limit, camera, and navigation frames
- **THEN** the firmware decodes the frames, classifies arrow and speed-sign JPEGs on the main loop task, and updates its live HUD state

### Requirement: ESP32-to-CMU USB CDC wire protocol and keepalive
The ESP32 firmware SHALL transmit line-oriented commands over USB CDC at 115200 baud — `SPLIM <km/h>`, `NEXTSPLIM <id> <km/h> <dist_m>`, `NAV <icon> <dist_m> [road]`, and `NAV OFF` — and re-emit held valid values periodically (every 1.5–2 seconds) so the CMU does not expire active data.

#### Scenario: Valid posted speed limit held while driving
- **WHEN** the ESP32 holds a valid posted speed limit from VietMap
- **THEN** it emits `SPLIM <km/h>` immediately on change and re-sends it every 2 seconds as a keepalive

#### Scenario: Upcoming different speed limit detected
- **WHEN** VietMap signals an upcoming different speed limit
- **THEN** the ESP32 edge-triggers a new monotonic alert ID and emits `NEXTSPLIM <id> <km/h> <dist_m>`

### Requirement: Optional on-board display telemetry and MAC-pinned flashing
The ESP32 firmware SHALL support headless builds (`esp32-s3-supermini`, `esp32-c3-supermini`) and display builds (`lilygo-t-display-s3` TFT, `esp32-s3-oled` I2C SSD1306) showing real-time CMU `splim_seriald` handshake status, BLE state, posted speed limit, camera distance, GPS speed, road name, and packet diagnostics, and SHALL provide MAC-pinned port resolution scripts for safe flashing.

#### Scenario: Display build renders CMU link status
- **WHEN** a display-enabled board is connected to the CMU running `splim_seriald`
- **THEN** the screen displays live CMU connected/disconnected status alongside BLE state and decoded VietMap telemetry

#### Scenario: Developer flashes ESP32 board
- **WHEN** the flash script runs with a configured board MAC address
- **THEN** it resolves the exact USB serial port matching that MAC instead of blindly flashing the first serial device

### Requirement: CMU USB CDC serial daemon with zero-wear tmpfs IPC
The `splim_seriald` daemon SHALL enforce single-instance execution via `/tmp/splim_seriald.lock`, auto-discover `/dev/ttyACM*` or `/dev/ttyUSB*` serial ports at 115200 baud, validate incoming `SPLIM`, `NEXTSPLIM`, `NAV`, and `NAV OFF` lines, atomically write `/tmp/splim`, `/tmp/vmnextsplim`, and `/tmp/vmnav` on RAM tmpfs (avoiding eMMC flash wear), and send non-blocking heartbeat responses to the ESP32.

#### Scenario: Daemon receives valid serial lines from ESP32
- **WHEN** `splim_seriald` reads valid `SPLIM`, `NEXTSPLIM`, or `NAV` lines over USB CDC
- **THEN** it writes temporary files in `/tmp` and atomically renames them to `/tmp/splim`, `/tmp/vmnextsplim`, or `/tmp/vmnav` with fresh timestamps, removing `/tmp/vmnav` on `NAV OFF`

#### Scenario: Second instance of splim_seriald is launched
- **WHEN** another process attempts to start `splim_seriald` while an instance holds `/tmp/splim_seriald.lock`
- **THEN** the second instance exits without opening or contending for the serial device
