# Installation

This covers installing a prebuilt release and the on-device setup. To
build from source instead, see [build.md](build.md). For
a project overview, see the [main README](../README.md).

## Install from a release

If you just want to run the mod, grab a prebuilt binary from this
repository's Releases page. Each release has two archives:
`oem-aa-mod-<version>.zip` and `oem-aa-mod-<version>-debug.zip`.
Use `oem-aa-mod-<version>.zip`; only grab the `-debug` one if you're
actually debugging.

The archive has the same layout — the shims under their canonical names
plus the patched system-attribute XMLs and the sample config — so it's
everything the on-device install needs:

```
oem-aa-mod-<version>/
  libpatch-blmjciaapa.so              # -> /data_persist/oem-aa-mod/
  libpatch-svcjcinavi.so              # -> /data_persist/oem-aa-mod/ (optional; HUD merge)
  resources/
    aap_system_attributes.xml         # -> /etc/
    aap_system_attributes_UCP.xml     # -> /etc/
    libpatch.conf                     # -> /data_persist/oem-aa-mod/ (optional; runtime config)
```

These shims are per-service `LD_PRELOAD` libraries, each scoped to the
single `sm_svclauncher` PID hosting its target service. Installing is
two on-device steps: drop the binaries somewhere `sm` can read them at
boot, then tell `sm.conf` to preload each one for its service
(`jciAAPA` for the Android Auto shim, and optionally `jcinavi` for the
HUD-merge shim).

> All commands below assume a root shell on the CMU (telnet/SSH after
> the usual jailbreak; the standard JCI dev shell works).

### 1. Place the binaries in `/data_persist/`

`/data_persist/` survives firmware updates and factory resets, which
keeps the mod from getting wiped by routine OEM activity. Use a
dedicated subdirectory so the mod's files are easy to find and remove:

```sh
mkdir -p /data_persist/oem-aa-mod
cp /mnt/sdb1/libpatch-blmjciaapa.so /data_persist/oem-aa-mod/
# Optional: the HUD-merge shim (only useful with the nav SD card; see
# hud_transport=svcnavi under Configuration below).
cp /mnt/sdb1/libpatch-svcjcinavi.so /data_persist/oem-aa-mod/
# Optional: runtime config (see Configuration below). Omit it to use
# the built-in defaults.
cp /mnt/sdb1/libpatch.conf /data_persist/oem-aa-mod/
chmod 0644 /data_persist/oem-aa-mod/libpatch-*.so
sync
```

`sync` is important — `/data_persist/` is backed by flash with deferred
writes, and without an explicit flush a power cycle before the next
write-back can leave the file truncated or missing.

The `libpatch-svcjcinavi.so` and `libpatch.conf` lines are optional:
copy `libpatch-svcjcinavi.so` only if you want the HUD-merge shim (it
self-disables in any process that isn't the nav service), and
`libpatch.conf` only if you want to override defaults. The two libraries
load into different services — `libpatch-blmjciaapa.so` is preloaded for
`jciAAPA` and `libpatch-svcjcinavi.so` for `jcinavi`, each via its own
`<service>` block in `sm.conf` (step 2 below).

SD cards mount under `/mnt/sd<x><n>` (e.g. `/mnt/sda1`, `/mnt/sdb1`, …)
depending on which USB/SD slot the kernel enumerates first on your
unit. Check `mount | grep /mnt/sd` if unsure.

### 2. Inject `LD_PRELOAD` via `sm.conf`

`sm` builds each spawned process's environment from `<environ_var>`
elements inside the matching `<service>` block, then `posix_spawn`s the
binary with that envp. That envp is given to the per-service
`sm_svclauncher` *before* it `dlopen`s the OEM `.so`, so `LD_PRELOAD` is
honoured at the right moment and the shim ends up mapped into exactly
one PID.

Edit `/jci/sm/sm.conf` and add an `<environ_var>` to each service you
want to patch.

**`jciAAPA`** (required — Android Auto shim). Find the
`<service … name="jciAAPA" …>` block and add the env var inside it:

```xml
<service type="jci_service" name="jciAAPA"
         path="/jci/aapa/blmjciaapa.so" …>
    <!-- existing <dependency>/<param>/… entries left in place -->
    <environ_var env_name="LD_PRELOAD"
                 env_value="/data_persist/oem-aa-mod/libpatch-blmjciaapa.so"/>
</service>
```

**`jcinavi`** (optional — HUD-merge shim; only needed with
`hud_transport = svcnavi` and the navigation SD card). Find the
`<service … name="jcinavi" …>` block and add the same env var inside it:

```xml
<service type="jci_service" name="jcinavi" path="/jci/navi/svcjcinavi.so" …>
    <!-- existing <dependency>/<param>/… entries left in place -->
    <environ_var env_name="LD_PRELOAD"
                 env_value="/data_persist/oem-aa-mod/libpatch-svcjcinavi.so"/>
</service>
```

(Leave the existing `<dependency>`/`<connection>`/attributes exactly as
they are — only the `<environ_var>` line is added.) Skip this block
entirely if you aren't using the svcnavi HUD transport; the shim
self-disables in any process that isn't the nav service, but there's no
reason to preload it where it does nothing.

### 3. Update the AAP system attribute XMLs in `/etc`

`blmjciaapa.so` reads its one-time configuration from
`/etc/aap_system_attributes.xml` and `/etc/aap_system_attributes_UCP.xml`
at startup. The patched copies live in [resources/](../resources/) — back
up the originals and overwrite:

```sh
cp /etc/aap_system_attributes.xml          /etc/aap_system_attributes.xml.orig
cp /etc/aap_system_attributes_UCP.xml      /etc/aap_system_attributes_UCP.xml.orig
cp /mnt/sdb1/aap_system_attributes.xml     /etc/aap_system_attributes.xml
cp /mnt/sdb1/aap_system_attributes_UCP.xml /etc/aap_system_attributes_UCP.xml
sync
```

(Adjust the source path to wherever you staged the files from
[resources/](../resources/) — SD card mount, `scp`, etc.)

### 4. Restart the services (or reboot)

Restart just the services you changed:

```sh
smctl -r -n jciAAPA
smctl -r -n jcinavi    # only if you patched jcinavi in step 2
```

…or reboot the unit.

## Configuration (`libpatch.conf`)

Runtime behaviour is controlled by an optional `libpatch.conf` placed in
the same folder as the libraries (`/data_persist/oem-aa-mod/` — step 1
copies it there). It is the common config for the whole mod: every
library reads the same file and acts on the keys it understands,
ignoring the rest. A sample is shipped at
[resources/libpatch.conf](../resources/libpatch.conf).

The file is entirely optional — if it is missing, or a key is omitted,
the built-in default is used. Syntax is one `key = value` per line; `#`
begins a comment; whitespace around keys/values is ignored; keys are
case-insensitive.

| Key | Values | Default | Effect |
| --- | --- | --- | --- |
| `touch` | `true` / `false` | `true` | Enable the Android Auto touch-input passthrough. |
| `hud` | `true` / `false` | `true` | Enable HUD guidance forwarding (turn arrow + distance to the head-up display). |
| `hud_transport` | `svcnavi` / `vbs` | `svcnavi` | Which path HUD guidance takes (only relevant when `hud = true`). |

Booleans are lenient — `true`/`1`/`yes`/`on` and `false`/`0`/`no`/`off`
are all accepted.

`hud_transport` picks the route for HUD frames:

- **`svcnavi`** (default) — emit guidance to the OEM `svcjcinavi`
  navigation service, which becomes the single writer of the HUD frame.
  Required for the speed-limit display and clean coexistence with OEM
  nav, but needs the navigation SD card.
  The `libpatch-svcjcinavi.so` HUD-merge shim only matters with this
  transport. (`svcjcinavi` is accepted as an alias.)
- **`vbs`** — write the HUD frame directly to `com.jci.vbs.navi`. Works
  with no navigation SD card, but it is one of two writers, so it can
  conflict with OEM nav guidance.

After editing `libpatch.conf`, restart the affected service(s) —
`jciAAPA`, and `jcinavi` if you patched it — or reboot for the change to
take effect. The config is read once at library load.

## Uninstall / disable

Remove (or comment out) the `<environ_var>` line(s) you added in
`/jci/sm/sm.conf` (the `jciAAPA` one, and the `jcinavi` one if present),
restore the original `/etc/aap_system_attributes*.xml` files from the
`.orig` backups, then restart the affected services (`smctl -r -n
jciAAPA` / `smctl -r -n jcinavi`) or reboot. The
`/data_persist/oem-aa-mod/` directory can be deleted afterwards.
