#!/bin/sh
# ============================================================================
#  auto-revert.sh - Non-interactive Emergency Rollback Script for Mazda CMU
#  Automatically restores stock system and removes all oem-aa-mod patches.
#  Runs with root privileges via CVE-2024-8359, dataRetrieval, or MP3-XSS.
# ============================================================================

hwclock --hctosys 2>/dev/null
echo 1 > "/sys/class/gpio/Watchdog Disable/value" 2>/dev/null
mount -o rw,remount / 2>/dev/null

MYDIR=$(dirname "$(readlink -f "$0" 2>/dev/null || echo "$0")")
mount -o rw,remount "${MYDIR}" 2>/dev/null

mkdir -p "${MYDIR}/logs"
exec > "${MYDIR}/logs/auto_revert.log" 2>&1

echo "=== oem-aa-mod Auto-Revert Emergency Recovery Start ==="
echo "Date: $(date 2>/dev/null || echo 'unknown')"

# Show non-blocking emergency notification on screen (if display service is running)
killall -q jci-dialog 2>/dev/null
/jci/tools/jci-dialog --info \
  --title="oem-aa-mod - Emergency Recovery" \
  --text="EMERGENCY RECOVERY IN PROGRESS...\nRestoring stock system to factory state.\nDo NOT remove USB drive!" \
  --no-cancel 2>/dev/null &

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------
restore_orig() {
  f="$1"
  if [ -e "${f}.orig" ]; then
    cp -a "${f}.orig" "$f"
    rm -f "${f}.orig"
    echo "  restored ${f} from .orig backup"
    return 0
  fi
  return 1
}

safe_replace() {
  src="$1"; dst="$2"
  if [ ! -s "$src" ]; then
    echo "  refusing to replace ${dst}: new content is empty"
    return 1
  fi
  if cp -a "$dst" "${dst}.oaam.tmp" && cat "$src" > "${dst}.oaam.tmp" &&
     sync && mv "${dst}.oaam.tmp" "$dst"; then
    return 0
  fi
  rm -f "${dst}.oaam.tmp"
  return 1
}

strip_carplay_ldpath() {
  c="$1"
  awk '
    index($0, "name=\"jciCARPLAY\"") && /<service/ { inside = 1 }
    inside && /LD_LIBRARY_PATH/ && /jci\/lib:\/usr\/lib/ && !done { done = 1; next }
    { print }
    inside && /<\/service>/ { inside = 0 }
  ' "$c" > /tmp/oaam.clean && safe_replace /tmp/oaam.clean "$c"
  rm -f /tmp/oaam.clean
}

unpatch_conf() {
  conf="$1"
  [ -f "$conf" ] || return 0

  if restore_orig "$conf"; then
    echo "  restored ${conf} from pristine backup"
  else
    echo "  ${conf}.orig not found; stripping patch lines"
    for token in libpatch-blmjciaapa.so libpatch-svcjcinavi.so libpatch-blmjcicarplay.so libpatch-svcjcimmui.so; do
      if grep -q "$token" "$conf"; then
        grep -v "$token" "$conf" > /tmp/oaam.clean && safe_replace /tmp/oaam.clean "$conf"
        rm -f /tmp/oaam.clean
        echo "  removed ${token} lines from ${conf}"
      fi
    done
    strip_carplay_ldpath "$conf"
  fi

  # Restore OEM watchdog reset behavior
  for svc in jciAAPA jcinavi jciCARPLAY jciMMUI; do
    sed -i "/name=\"${svc}\"/ s/reset_board=\"no\"/reset_board=\"yes\"/" "$conf" 2>/dev/null
  done
}

# ---------------------------------------------------------------------------
# 1. Restore Service Manager configs (/jci/sm/sm*.conf)
# ---------------------------------------------------------------------------
echo "--- Restoring Service Manager configs ---"
unpatch_conf /jci/sm/sm.conf
unpatch_conf /jci/sm/sm_WCP.conf

# ---------------------------------------------------------------------------
# 2. Restore Android Auto XML configs
# ---------------------------------------------------------------------------
echo "--- Restoring Android Auto XML configs ---"
for xml in /etc/aap_system_attributes.xml /etc/aap_system_attributes_UCP.xml; do
  restore_orig "$xml" || echo "  ${xml}.orig not found, skipping"
done

# ---------------------------------------------------------------------------
# 3. Restore CarPlay devmgr master config
# ---------------------------------------------------------------------------
echo "--- Restoring CarPlay devmgr master config ---"
MASTER="/etc/devmgr_config_master.xml"
if [ -f "$MASTER" ]; then
  if ! restore_orig "$MASTER"; then
    sed -i 's#<name>NaviSupported</name><value>TRUE</value>#<name>NaviSupported</name><value>FALSE</value>#' "$MASTER" 2>/dev/null
    echo "  reverted NaviSupported=FALSE in ${MASTER}"
  fi
fi

# ---------------------------------------------------------------------------
# 4. Clean up any residual stage_wifi.sh / SSH modifications
# ---------------------------------------------------------------------------
echo "--- Cleaning up network / SSH overrides ---"
if [ -f /jci/scripts/stage_wifi.sh.bak ]; then
  cp -a /jci/scripts/stage_wifi.sh.bak /jci/scripts/stage_wifi.sh && rm -f /jci/scripts/stage_wifi.sh.bak
  echo "  restored stage_wifi.sh from backup"
elif grep -q "sshd" /jci/scripts/stage_wifi.sh 2>/dev/null; then
  sed -i '/sshd/d' /jci/scripts/stage_wifi.sh 2>/dev/null
  echo "  removed sshd line from stage_wifi.sh"
fi

# ---------------------------------------------------------------------------
# 5. Stop running daemons and remove mod files
# ---------------------------------------------------------------------------
echo "--- Removing mod files and daemons ---"
killall -9 splim_seriald splim_udpd 2>/dev/null
rm -rf /data_persist/oem-aa-mod
rm -f /data_persist/splim*
rm -f /tmp/hud-up-lock

echo "=== Auto-Revert Complete: Stock system restored successfully ==="
sync
sleep 1
sync

# ---------------------------------------------------------------------------
# 6. Notify user and reboot
# ---------------------------------------------------------------------------
killall -q jci-dialog 2>/dev/null
/jci/tools/jci-dialog --info \
  --title="oem-aa-mod - Recovery Complete" \
  --text="AUTO-REVERT COMPLETE!\nStock system has been fully restored.\nYou can remove the USB drive.\nRebooting in 5 seconds..." \
  --no-cancel 2>/dev/null &

sleep 5
killall -q jci-dialog 2>/dev/null
echo "Rebooting system now..."
sync
reboot -f 2>/dev/null || reboot
