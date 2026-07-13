#!/bin/sh
# splim_udpd_start.sh — launcher for the shared speed-limit UDP listener.
#
# Installed to /data_persist/splim_udpd_start.sh — the exact path the HUD
# bridge invokes (see hud_send.cpp):
#   pgrep splim_udpd >/dev/null 2>&1 || setsid sh /data_persist/splim_udpd_start.sh &
#
# `exec` replaces this shell with the daemon so the running process is named
# "splim_udpd" (what the caller's pgrep guard matches) and inherits the new
# session the caller created with setsid.

exec /data_persist/oem-aa-mod/splim_udpd
