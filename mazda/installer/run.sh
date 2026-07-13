#!/bin/sh
# oem-aa-mod installer/uninstaller for Mazda CMU
# created by Bijan
# https://github.com/VitaliyKurokhtin/oem-aa-mod
#
# A single install patches BOTH:
#   * Android Auto  -> jciAAPA (libpatch-blmjciaapa.so) + jcinavi (libpatch-svcjcinavi.so)
#   * CarPlay HUD   -> jciCARPLAY (libpatch-blmjcicarplay.so)
# Each service is patched INDEPENDENTLY and WCP-safely (see patch_service): on
# Wireless-CarPlay units the OEM service manager MERGES a service's stanza from
# sm.conf + sm_WCP.conf, so injecting LD_PRELOAD into both yields a duplicate
# environ_var and crash-loops that service. We therefore patch exactly ONE conf
# per service, sanity-check the edit, and set reset_board="no" so a shim fault
# restarts only that service instead of rebooting the whole unit.

hwclock --hctosys

echo 1 > /sys/class/gpio/Watchdog\ Disable/value
mount -o rw,remount /

MYDIR=$(dirname "$(readlink -f "$0")")
mount -o rw,remount ${MYDIR}

mkdir -p "${MYDIR}/logs"

MOD_DIR="/data_persist/oem-aa-mod"

# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

# back up a file to <file>.orig exactly once (pristine pre-install copy)
bak_orig() {
  f="$1"
  if [ ! -e "${f}.orig" ]; then
    cp -a "$f" "${f}.orig"
    echo "  backup -> ${f}.orig"
  fi
}

# patch_service <service-name> <preload-so-abs-path> [extra_environ_line]
# Injects LD_PRELOAD (+ an optional extra environ_var) into the stanza named
# <service-name>, in exactly ONE sm config, idempotently and safely.
patch_service() {
  svc="$1"; so="$2"; extra="$3"
  token=$(basename "$so")
  preload_line="            <environ_var env_name=\"LD_PRELOAD\" env_value=\"${so}\"/>"

  # which confs actually define this service?
  present=""
  for c in /jci/sm/sm.conf /jci/sm/sm_WCP.conf; do
    [ -f "$c" ] && grep -q "name=\"${svc}\"" "$c" && present="${present} ${c}"
  done
  if [ -z "$present" ]; then
    echo "  ${svc}: no stanza in any sm config, skipping"
    return 0
  fi

  # pick exactly one target: prefer sm_WCP.conf when present (it is the active
  # boot config on WCP units and its stanza is what actually launches).
  target=""
  for c in $present; do
    case "$c" in */sm_WCP.conf) target="$c" ;; esac
  done
  [ -z "$target" ] && target=$(echo $present | awk '{print $1}')
  echo "  ${svc}: patching ${target}"

  # strip any stale inject THIS service left in the non-target conf(s), so the
  # merge can never produce a duplicate environ_var.
  for c in $present; do
    [ "$c" = "$target" ] && continue
    if grep -q "$token" "$c"; then
      bak_orig "$c"
      grep -v "$token" "$c" > /tmp/oaam.clean && cp /tmp/oaam.clean "$c"
      rm -f /tmp/oaam.clean
      echo "  ${svc}: removed stale ${token} from ${c}"
    fi
  done

  bak_orig "$target"

  if grep -q "$token" "$target"; then
    echo "  ${svc}: already patched in ${target}, skipping insert"
  else
    awk -v anchor="name=\"${svc}\"" -v pl="$preload_line" -v ex="$extra" '
      index($0, anchor) { print; print pl; if (ex != "") print ex; next } 1
    ' "$target" > /tmp/oaam.new
    old=$(wc -l < "$target"); new=$(wc -l < /tmp/oaam.new)
    exp=$((old + 1)); [ -n "$extra" ] && exp=$((old + 2))
    if [ "$new" -eq "$exp" ] && grep -q "$token" /tmp/oaam.new; then
      cp /tmp/oaam.new "$target"
      echo "  ${svc}: inserted LD_PRELOAD (${old}->${new} lines)"
    else
      echo "  ${svc}: SANITY FAILED (${old}->${new}, expected ${exp}), left untouched"
    fi
    rm -f /tmp/oaam.new
  fi

  # a shim fault restarts just this service, not the whole unit
  sed -i "/name=\"${svc}\"/ s/reset_board=\"yes\"/reset_board=\"no\"/" "$target"
}

# unpatch_service <service-name> <preload-so-basename>
# Removes our injected LD_PRELOAD/LD_LIBRARY_PATH lines for this service's .so
# from every sm config (used only when a .orig restore isn't available).
unpatch_service() {
  token="$2"
  for c in /jci/sm/sm.conf /jci/sm/sm_WCP.conf; do
    [ -f "$c" ] || continue
    if grep -q "$token" "$c"; then
      grep -v "$token" "$c" > /tmp/oaam.clean && cp /tmp/oaam.clean "$c"
      rm -f /tmp/oaam.clean
      echo "  removed ${token} lines from ${c}"
    fi
  done
}

# restore <file> from <file>.orig if the backup exists
restore_orig() {
  f="$1"
  if [ -e "${f}.orig" ]; then
    cp -a "${f}.orig" "$f"
    rm -f "${f}.orig"
    echo "  restored ${f} from backup"
    return 0
  fi
  return 1
}

# --- Prompt user: Install or Uninstall ---
/jci/tools/jci-dialog --question \
  --title="oem-aa-mod" \
  --text="What would you like to do?" \
  --ok-label="Install" \
  --cancel-label="Uninstall"
CHOICE=$?

if [ $CHOICE -eq 0 ]
then
  # ============================================================
  # INSTALL
  # ============================================================
  exec > "${MYDIR}/logs/install.log" 2>&1
  echo "=== oem-aa-mod install start ==="

  /jci/tools/jci-dialog --info --title="oem-aa-mod" --text="Installing oem-aa-mod...\nDo not remove USB drive." --no-cancel &

  # --- Install libraries -------------------------------------------------
  mkdir -p "$MOD_DIR"
  for so in libpatch-blmjciaapa.so libpatch-svcjcinavi.so libpatch-blmjcicarplay.so
  do
    cp "${MYDIR}/${so}" "${MOD_DIR}/" && echo "${so} has been copied"
  done
  for conf in libpatch.conf libpatch-carplay.conf
  do
    cp "${MYDIR}/resources/${conf}" "${MOD_DIR}/" && echo "${conf} has been copied"
  done
  chmod 0644 "${MOD_DIR}"/libpatch-*.so

  # --- Install the shared speed-limit UDP daemon (splim_udpd) ------------
  # Only used when carplay_speed_limit=true (or the AA side needs it); the
  # binary/launcher are harmless when idle since nothing invokes them. The
  # launcher must live at the exact path the HUD bridge calls:
  # /data_persist/splim_udpd_start.sh
  if [ -f "${MYDIR}/splim_udpd" ]; then
    cp "${MYDIR}/splim_udpd" "${MOD_DIR}/" && echo "splim_udpd has been copied"
    chmod 0755 "${MOD_DIR}/splim_udpd"
    cp "${MYDIR}/splim_udpd_start.sh" /data_persist/ && echo "splim_udpd_start.sh has been copied"
    chmod 0755 /data_persist/splim_udpd_start.sh
  else
    echo "splim_udpd not in payload, skipping"
  fi

  # --- Patch each service independently (WCP-safe) -----------------------
  echo "--- patching Android Auto ---"
  patch_service jciAAPA "${MOD_DIR}/libpatch-blmjciaapa.so"
  patch_service jcinavi "${MOD_DIR}/libpatch-svcjcinavi.so"

  echo "--- patching CarPlay ---"
  patch_service jciCARPLAY "${MOD_DIR}/libpatch-blmjcicarplay.so" \
    '            <environ_var env_name="LD_LIBRARY_PATH" env_value="/jci/lib:/usr/lib"/>'

  # --- Android Auto: install patched XML system attributes ---------------
  for xml in aap_system_attributes.xml aap_system_attributes_UCP.xml
  do
    if [ -e "/etc/${xml}" ]
    then
      bak_orig "/etc/${xml}"
      cp "${MYDIR}/resources/${xml}" "/etc/${xml}"
      echo "Installed ${xml}"
    else
      echo "WARNING: /etc/${xml} absent, skipping"
    fi
  done

  # --- CarPlay: advertise nav support in the devmgr master template ------
  MASTER="/etc/devmgr_config_master.xml"
  if [ -f "$MASTER" ]
  then
    bak_orig "$MASTER"
    sed -i 's#<name>NaviSupported</name><value>FALSE</value>#<name>NaviSupported</name><value>TRUE</value>#' "$MASTER"
    echo "set NaviSupported=TRUE in ${MASTER}"
  else
    echo "WARNING: ${MASTER} absent, NaviSupported not set"
  fi

  sync
  echo "=== oem-aa-mod install complete ==="

  killall -q jci-dialog
  /jci/tools/jci-dialog --info --title="oem-aa-mod" --text="Installation complete!\nYou can remove the USB drive.\nRebooting in 5 seconds..." --no-cancel &

else
  # ============================================================
  # UNINSTALL
  # ============================================================
  exec > "${MYDIR}/logs/uninstall.log" 2>&1
  echo "=== oem-aa-mod uninstall start ==="

  /jci/tools/jci-dialog --info --title="oem-aa-mod" --text="Uninstalling oem-aa-mod...\nDo not remove USB drive." --no-cancel &

  # --- Restore sm configs (fall back to line-stripping) ------------------
  for conf in /jci/sm/sm.conf /jci/sm/sm_WCP.conf
  do
    [ -f "$conf" ] || continue
    if ! restore_orig "$conf"
    then
      echo "  ${conf}.orig not found; stripping injected lines"
      unpatch_service jciAAPA    libpatch-blmjciaapa.so
      unpatch_service jcinavi    libpatch-svcjcinavi.so
      unpatch_service jciCARPLAY libpatch-blmjcicarplay.so
      # the CarPlay LD_LIBRARY_PATH line has no .so token; strip it explicitly
      grep -v 'LD_LIBRARY_PATH.*jci/lib:/usr/lib' "$conf" > /tmp/oaam.clean && cp /tmp/oaam.clean "$conf"
      rm -f /tmp/oaam.clean
    fi
  done

  # --- Restore Android Auto XML configs ----------------------------------
  for xml in /etc/aap_system_attributes.xml /etc/aap_system_attributes_UCP.xml
  do
    restore_orig "$xml" || echo "  ${xml}.orig not found, skipping"
  done

  # --- Restore CarPlay devmgr master (or revert NaviSupported) -----------
  MASTER="/etc/devmgr_config_master.xml"
  if [ -f "$MASTER" ] && ! restore_orig "$MASTER"
  then
    sed -i 's#<name>NaviSupported</name><value>TRUE</value>#<name>NaviSupported</name><value>FALSE</value>#' "$MASTER"
    echo "  reverted NaviSupported=FALSE in ${MASTER}"
  fi

  # --- Remove installed libraries and config -----------------------------
  for so in libpatch-blmjciaapa.so libpatch-svcjcinavi.so libpatch-blmjcicarplay.so
  do
    rm -f "${MOD_DIR}/${so}" && echo "Removed ${so}"
  done
  for conf in libpatch.conf libpatch-carplay.conf
  do
    rm -f "${MOD_DIR}/${conf}" && echo "Removed ${conf}"
  done

  # --- Remove the speed-limit UDP daemon ---------------------------------
  killall -q splim_udpd 2>/dev/null
  rm -f "${MOD_DIR}/splim_udpd" && echo "Removed splim_udpd"
  rm -f /data_persist/splim_udpd_start.sh && echo "Removed splim_udpd_start.sh"

  rmdir "$MOD_DIR" 2>/dev/null && echo "Removed ${MOD_DIR} directory"

  sync
  echo "=== oem-aa-mod uninstall complete ==="

  killall -q jci-dialog
  /jci/tools/jci-dialog --info --title="oem-aa-mod" --text="Uninstall complete!\nYou can remove the USB drive.\nRebooting in 5 seconds..." --no-cancel &

fi

sleep 5
killall -q jci-dialog
reboot
exit 0
