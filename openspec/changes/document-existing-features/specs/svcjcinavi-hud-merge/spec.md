# Spec Delta

## Purpose

Defines the OEM navigation service HUD merge shim (`libpatch-svcjcinavi.so`) injected into `svcjcinavi`, reconciling Android Auto guidance with OEM navigation speed-limit frames so the HUD displays both without flicker when the navigation SD card is inserted.

## ADDED Requirements

### Requirement: Flicker-free merge of Android Auto maneuver and OEM speed limit
While Android Auto is actively sending HUD guidance via the `svcnavi` transport (identified by the reserved sentinel speed value), the shim SHALL merge the cached Android Auto maneuver and street name onto OEM navigation HUD frames and splice the cached OEM speed limit onto Android Auto HUD frames, preserving each frame generation's synchronization bit.

#### Scenario: OEM navigation emits speed-limit frame during active Android Auto guidance
- **WHEN** Android Auto guidance is active and the OEM nav engine emits a blank-maneuver speed-limit frame
- **THEN** the shim overwrites the blank maneuver and blank street strip with the active Android Auto maneuver and street while keeping the OEM speed limit and generation sync bit

#### Scenario: Android Auto emits maneuver frame with sentinel speed
- **WHEN** an Android Auto frame arrives carrying the sentinel speed value and a non-empty maneuver
- **THEN** the shim caches the maneuver and street, splices in the last known OEM speed limit, and forwards the merged frame to the HUD

#### Scenario: Android Auto guidance stops
- **WHEN** Android Auto sends an empty blanking maneuver frame
- **THEN** the shim clears Android Auto ownership and allows subsequent OEM navigation frames to pass through unmodified

### Requirement: Optional forced HUD street name display
When `force_street_name = true` (default `false`) in `libpatch.conf`, the shim SHALL populate the HUD street-name strip from the OEM service's un-blanked street buffer even on firmware/market configurations that normally suppress the street line.

#### Scenario: Street strip suppressed on EU or restricted HUD market
- **WHEN** `force_street_name = true` and an Android Auto guidance frame is processed on a market where the OEM blanks the outbound street strip
- **THEN** the shim rewrites the street strip with the captured Android Auto street name while leaving maneuver and distance unchanged
