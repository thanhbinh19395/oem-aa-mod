# Spec Delta

## Purpose

Defines the shared `LD_PRELOAD` runtime platform used across all Mazda CMU shims, including process self-gating, crash diagnostics, sibling runtime configuration loading, and HUD street-name transliteration.

## ADDED Requirements

### Requirement: Process-scoped LD_PRELOAD self-gating
Each CMU shim SHALL verify at load time or first hook invocation that it is running inside its intended OEM host process and SHALL remain completely inert in any unrelated process that inherits `LD_PRELOAD`.

#### Scenario: Watchdog or unrelated child inherits LD_PRELOAD
- **WHEN** an unrelated process (such as a `sh -c killall` watchdog probe) inherits `LD_PRELOAD` for a shim
- **THEN** the shim skips initialization, avoids log spam, and passes any interposed calls straight through to the real symbols

#### Scenario: Target host process loads the shim
- **WHEN** the intended OEM host process (`sm_svclauncher` for the target BLM/SVC or `aap_service`) loads the shim
- **THEN** the shim activates its configured hooks and features

### Requirement: Fatal-signal crash diagnostics
Each active CMU shim SHALL install a fatal-signal handler for `SIGSEGV`, `SIGBUS`, `SIGABRT`, `SIGFPE`, and `SIGILL` that logs register state, backtrace, and process memory maps before re-raising the signal with default disposition.

#### Scenario: Fatal signal occurs in a patched process
- **WHEN** a patched host process encounters a fatal signal
- **THEN** the handler writes async-signal-safe crash diagnostics (signal info, ARM registers, backtrace, `/proc/self/maps`) to stderr and re-raises the signal so normal OS crash handling proceeds

### Requirement: Sibling runtime configuration loading
Each CMU shim SHALL load its runtime `<key> = <value>` configuration file (`libpatch.conf` for the Android Auto family and `svcjcimmui`; `libpatch-carplay.conf` for `blmjcicarplay`) from the same directory as the loaded shim shared object, falling back to compiled-in defaults when the file or any key is absent.

#### Scenario: Config file present next to shim library
- **WHEN** a shim loads and its sibling `.conf` file exists in the same directory
- **THEN** the shim parses case-insensitive keys, ignores `#` comments and blank lines, accepts lenient booleans (`true`/`false`, `1`/`0`, `yes`/`no`, `on`/`off`), and applies recognized settings while ignoring keys belonging to other shims

#### Scenario: Config file missing or unreadable
- **WHEN** the sibling `.conf` file is absent or cannot be opened
- **THEN** the shim continues normally using its compiled-in default settings without failing initialization

### Requirement: HUD Latin Extended Additional transliteration
The shared transliteration utility SHALL fold precomposed Latin letters in the Unicode block `U+1E00..U+1EFF` down to HUD-renderable characters below `U+0800` without lengthening the UTF-8 string, preserving already-renderable base accents (`ă`, `â`, `ê`, `ô`, `ơ`, `ư` and uppercase forms) and passing ASCII and already-renderable 2-byte UTF-8 letters through unchanged.

#### Scenario: Vietnamese street name contains 3-byte UTF-8 letters
- **WHEN** a street name containing precomposed Latin Extended Additional characters (such as `ễ`, `ớ`, `ậ`) is folded
- **THEN** each letter maps to its renderable `< U+0800` base form (such as `ê`, `ơ`, `â`) while preserving ASCII and 2-byte letters like `đ`

#### Scenario: Caller specifies unrenderable fallback policy
- **WHEN** a non-Latin character at or above `U+0800` is encountered
- **THEN** the transliterator either leaves the bytes untouched (when no replacement byte is requested, as in Android Auto) or replaces the sequence with the caller-specified fallback byte (such as `?` in CarPlay)
