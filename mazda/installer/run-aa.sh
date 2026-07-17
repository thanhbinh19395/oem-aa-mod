#!/bin/sh
# Android Auto-only entry point for the oem-aa-mod installer.

MYDIR=$(dirname "$(readlink -f "$0")")
exec /bin/sh "${MYDIR}/run.sh" aa
