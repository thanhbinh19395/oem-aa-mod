# Spec Delta

## Purpose

Defines the standalone 3-tier USB installer, interactive CMU install/uninstall and temporary SSH provisioning, zero-touch emergency Auto-Revert USB, and host/on-device verification tooling for `oem-aa-mod`.

## ADDED Requirements

### Requirement: 3-tier USB autorun and trigger architecture
The USB installer payload SHALL package three trigger mechanisms on a FAT32 MBR drive labeled `MZD` — CVE-2024-8359 `.up` filename command injection with `jci-autoupdate` (FW v70–v74), MP3 ID3-tag XSS overlay controls (`SSH`, `HUD+`, `HUD-`, `HUD?`) for manual fallback, and `dataRetrieval_config.txt` for legacy FW v55–v59 — guarded by an atomic lock to prevent duplicate execution.

#### Scenario: Modern firmware CMU detects installer USB
- **WHEN** the installer USB is inserted into a CMU running FW v70–v74
- **THEN** the `.up` payload triggers the trampoline script once under an atomic lock and launches the interactive installer

#### Scenario: Automatic trigger is disabled or unavailable
- **WHEN** the user plays an included MP3 file from Entertainment -> USB
- **THEN** the on-screen overlay renders action buttons allowing manual install, uninstall, status check, or SSH activation

### Requirement: Interactive CMU mod management and temporary SSH
The on-device installer (`run.sh`) SHALL present `jci-dialog` menus allowing the user to install or uninstall Android Auto shims, CarPlay shims, or both, deploy matching configs and `splim_seriald`, write execution logs to `logs/` on the USB drive, and optionally enable temporary session-only SSH (port 22 and 36000) over either an existing Wi-Fi connection or an automatically created `CMU-<MAC>` access point (`192.168.53.1`).

#### Scenario: User installs Android Auto and CarPlay shims via USB
- **WHEN** the user selects Install Mod -> Both -> Install in `jci-dialog`
- **THEN** the installer backs up and updates `sm.conf`, copies the shims, `splim_seriald`, and config files to `/data_persist/oem-aa-mod`, logs to the USB drive, and reboots the CMU

#### Scenario: User enables temporary SSH
- **WHEN** the user selects Enable SSH in `jci-dialog`
- **THEN** the installer opens firewall rules, starts SSH for the current session only (reverting on power-off), and displays the reachable Wi-Fi or Car AP IP address

### Requirement: Zero-touch emergency Auto-Revert USB
The repository SHALL provide scripts (`make_auto_revert_usb.sh` and `make_auto_revert_usb.bat`) to build a zero-touch recovery USB that automatically restores the stock `sm.conf`, removes all installed shims and files under `/data_persist/oem-aa-mod`, and reboots the CMU without requiring screen interaction.

#### Scenario: Emergency Auto-Revert USB is inserted into a malfunctioning CMU
- **WHEN** the Auto-Revert USB is inserted into USB port 1 with ignition on
- **THEN** the CMU automatically restores stock `sm.conf`, removes `/data_persist/oem-aa-mod`, displays completion status, and reboots to stock firmware

### Requirement: Automated USB creation and cross-platform verification tooling
The repository SHALL provide automated USB creation scripts (`make_usb.sh`, `make_usb.bat`) and `make installer` / `make installer-debug` staging targets, alongside host-side unit tests (`nav16_test`, `nav_carplay_test`, `vietmap_ble_hud_test`) and on-device CMU diagnostic tools (`gsi_test`, `lane_test`, `collect-log.sh`).

#### Scenario: Developer runs host unit tests
- **WHEN** the host test suite is built and executed under `test/`
- **THEN** `nav16_test`, `nav_carplay_test`, and `vietmap_ble_hud_test` validate AA GAL 1.6 decoding, CarPlay iAP2 decoding, and VietMap BLE codec/arrow classification with zero-exit pass status
