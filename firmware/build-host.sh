#!/usr/bin/env bash
set -eu
root=$(cd "$(dirname "$0")" && pwd)
side=${1:?specify left or right}
case "$side" in left|right);; *) exit 2;; esac
: "${ZEPHYR_BASE:?select the patched, pinned Zephyr checkout}"
: "${ZEPHYR_SDK_INSTALL_DIR:?select Zephyr SDK 1.0.1}"
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr SOURCE_DATE_EPOCH=0
modules=$(dirname "$ZEPHYR_BASE")/modules
out=${2:-"$root/../build/firmware-$side"}
cmake -S "$root/source" -B "$out" -G Ninja -DBOARD=frdm_mcxn947/mcxn947/cpu0 \
    -DZEPHYR_MODULES="$modules/hal/cmsis_6;$modules/hal/nxp;$modules/debug/segger" \
    -DDTC_OVERLAY_FILE="$root/local-mcxn947.overlay" \
    -DEXTRA_CONF_FILE="$root/local-workspace.conf;$root/dual-common.conf;$root/boards/$side.conf;$root/host-control.conf"
cmake --build "$out"
