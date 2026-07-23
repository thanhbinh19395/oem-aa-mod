#!/usr/bin/env sh
set -euo pipefail

# collect_diag_logs.sh
# Copies diagnostic logs from /data_persist to /mnt/sda1 and cleans originals.
# Usage:
#   sudo chmod +x resources/collect_diag_logs.sh
#   sudo resources/collect_diag_logs.sh

SRC_DIR=/data_persist
DST_BASE=/mnt/sda1/diag-logs
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
DST_DIR="${DST_BASE}/${TIMESTAMP}"

mkdir -p "$DST_DIR"

echo "Destination: $DST_DIR"

copy_file() {
  src="$1"
  if [ -e "$src" ]; then
    if [ -d "$src" ]; then
      cp -a "$src" "$DST_DIR/" || { echo "Failed to copy directory $src"; return 1; }
    else
      cp -a "$src" "$DST_DIR/" || { echo "Failed to copy file $src"; return 1; }
    fi
    return 0
  else
    return 2
  fi
}

# Copy known log files/paths.
for name in msgtype_diag.log nav_diag.log splim_udpd.log; do
  src_path="$SRC_DIR/$name"
  copy_file "$src_path"
  rc=$?
  if [ "$rc" -eq 0 ]; then
    echo "Copied $src_path"
  elif [ "$rc" -eq 2 ]; then
    echo "Not found: $src_path"
  fi
done

# Ensure data is written to disk
sync

# Clean up originals safely:
# - truncate files to zero length
# - remove contents of nav_diag directory if present

if [ -f "$SRC_DIR/msgtype_diag.log" ]; then
  : > "$SRC_DIR/msgtype_diag.log" || echo "Warning: failed to truncate msgtype_diag.log"
  echo "Truncated $SRC_DIR/msgtype_diag.log"
fi

if [ -f "$SRC_DIR/nav_diag.log" ]; then
  : > "$SRC_DIR/nav_diag.log" || echo "Warning: failed to truncate nav_diag.log"
  echo "Truncated $SRC_DIR/nav_diag.log"
fi

if [ -f "$SRC_DIR/splim_udpd.log" ]; then
  : > "$SRC_DIR/splim_udpd.log" || echo "Warning: failed to truncate splim_udpd.log"
  echo "Truncated $SRC_DIR/splim_udpd.log"
fi

if [ -d "$SRC_DIR/nav_diag" ]; then
  # remove files inside but keep the directory itself
  rm -rf "$SRC_DIR/nav_diag"/* || echo "Warning: failed to clean $SRC_DIR/nav_diag"
  echo "Cleaned $SRC_DIR/nav_diag/"
fi

echo "Done: logs copied to $DST_DIR and originals cleaned."

exit 0
