# CarPlay VietMap speed-limit OCR

VietMap Live draws the posted speed limit as a **UI overlay** on the CarPlay
map. That value never appears in the iAP2 guidance messages (`0x8058` /
`0x8059`) that `libpatch-blmjcicarplay.so` already decodes for turn / distance
/ road name. The only on-CMU place the badge pixels exist is the CarPlay
**video plane** inside `carplayd`:

```
appsrc → capsfilter → vpudec → mfw_v4lsink   (V4L overlay, not Wayland/fb0)
```

`libpatch-carplayd.so` is an `LD_PRELOAD` shim for that process. It:

1. Interposes `gst_element_factory_make` and attaches a GStreamer 0.10 buffer
   probe to the `mfw_v4lsink` sink pad.
2. Optionally interposes `gst_element_link` to insert
   `mfw_ipucsc ! capsfilter(I420)` when `carplay_ocr_force_linear=true`
   (needed if `vpudec` feeds tiled `TNVP` buffers the CPU cannot match).
3. Template-matches a small ROI against the 12 embedded VietMap badge
   grayscale templates (`10`…`120` km/h) using normalized cross-correlation.
4. Writes `/data_persist/splim` as `<km/h> <unix_ts>` — the same file the
   existing HUD bridge already polls when `carplay_speed_limit=true`.

Wayland `/usr/bin/screenshot` and `/dev/fb0` do **not** contain this plane;
do not try to OCR from those.

## Enable on device

In `/data_persist/oem-aa-mod/libpatch-carplay.conf`:

```
carplay_speed_limit = true
carplay_udpd_launch = false   # OCR is the writer; don't also run splim_udpd
carplay_ocr = true
```

Reboot (or restart `carplayd` + `jciCARPLAY`). Logs:

* OCR: `/tmp/carplayd_ocr.log`
* HUD: `/tmp/carplay_bridge.log`

## Calibrate the ROI

1. Set `carplay_ocr_dump = true`, reboot, start VietMap nav with a visible
   speed badge.
2. Pull `/tmp/splim_roi.pgm` (60×60 grayscale of the current ROI).
3. Adjust `carplay_ocr_roi_{x,y,w,h}` until the badge fills most of the PGM,
   then turn dump back off.

`-1` for `x`/`y` means “auto lower-left” (16 px margin), which matches the
usual VietMap `CPSpeedAndLimit` placement on an ~800×480 CarPlay plane.

## If nothing ever matches

* Confirm `libpatch-carplayd.so` is preloaded for the `carplayd` service in
  `sm.conf` / `sm_WCP.conf` (the USB installer does this).
* Set `carplay_ocr_force_linear = true` and retry — tiled VPU frames look
  like noise in the dump PGM.
* Lower `carplay_ocr_min_ncc` slightly (e.g. `0.45`) only after the ROI is
  visually correct; too low invites false positives from map chrome.
