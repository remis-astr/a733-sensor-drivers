#!/bin/sh
# Select the camera sensor of the A7S: enable its overlay, disable the
# others, regenerate extlinux.conf. Takes effect after a power cycle (no
# soft reboot on this board). Run as root, after drivers/install.sh:
#   sudo sh scripts/board/select-sensor.sh imx585
# Back to the IMX327: same command with imx327. Backup overlays (*.bak) are
# never touched.
set -e
s=${1:?usage: select-sensor.sh <imx327|imx462|imx477|imx585|imx678|imx662>}
want=/boot/dtbo/cubie-a7s-$s.dtbo
if [ ! -e "$want" ] && [ ! -e "$want.disabled" ]; then
	echo "no overlay $want(.disabled): run drivers/install.sh first" >&2
	exit 1
fi
for f in /boot/dtbo/cubie-a7s-imx*.dtbo; do
	[ -e "$f" ] && [ "$f" != "$want" ] && mv "$f" "$f.disabled" && echo "disabled $(basename "$f")"
done
[ -e "$want" ] || mv "$want.disabled" "$want"
echo "enabled $(basename "$want")"
modname=${s}_mipi
modinfo "$modname" >/dev/null 2>&1 || echo "WARNING: module $modname not installed (drivers/install.sh)" >&2
u-boot-update
grep -m1 fdtoverlays /boot/extlinux/extlinux.conf
sync
echo "Now: sudo systemctl poweroff, then cycle the power (camera cable connected)."
