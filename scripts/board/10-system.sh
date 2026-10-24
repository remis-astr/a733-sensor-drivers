#!/usr/bin/env bash
# Stage 10, on the board as root: light system + kernel 6.6.98-5.
#  - OpenClaw (shipped by Radxa's beta image, no service, inert) is left in
#    place; remove it with `npm uninstall -g openclaw` if you want;
#  - full-upgrade: brings the 6.6.98-5 kernel + headers, whose bsp/include has
#    the sunxi-sid.h the GPU module needs (fails on the image's 6.6.98-1);
#  - installs the tools used by stages 20/30;
#  - removes systemd-boot (see below), then rebuilds what its removal wipes.
# Ends by asking for a power cycle onto the new kernel.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "== $(date -Is) stage 10 on $(hostname), kernel $(uname -r)"

echo "== apt update / full-upgrade"
apt-get update
apt-get -y full-upgrade
# The image pins its local-apt-repository at priority 1999, which holds the
# linux-*-radxa-a733 meta-packages at 6.6.98-1: install the -5 kernel by name
# (-1 stays installed as the fallback).
apt-get install -y linux-image-6.6.98-5-aw2511 linux-headers-6.6.98-5-aw2511

echo "== tools"
apt-get install -y --no-install-recommends \
	device-tree-compiler cpp i2c-tools v4l-utils dkms build-essential \
	vulkan-tools clinfo iperf3 ethtool iw \
	htop lm-sensors rsync

# The full-upgrade bumps systemd-boot, whose postinst copies BOOTAA64.EFI to
# the (until then empty) ESP; U-Boot's EFI boot manager then runs it and the
# boot hangs with no console output, instead of falling back to extlinux.
# Removing the package cleans the ESP, but its prerm runs `kernel-install
# remove` for every kernel: 50-depmod deletes modules.dep & co (no autofs4 ->
# emergency mode) and 40-dkms uninstalls the DKMS modules. Rebuild both.
echo "== removing systemd-boot"
if dpkg -s systemd-boot >/dev/null 2>&1; then
	apt-get remove -y systemd-boot
fi
for k in /lib/modules/*/; do
	k=$(basename "$k")
	[[ -e /boot/vmlinuz-$k ]] || continue
	depmod -a "$k" -F "/boot/System.map-$k"
	dkms autoinstall -k "$k"
done
find /boot/efi -maxdepth 2

echo "== installed kernels"
ls /boot/vmlinuz-*
grep -E '^\s*(menu label|linux)' /boot/extlinux/extlinux.conf | head -4

if [[ ! -e /boot/vmlinuz-6.6.98-5-aw2511 ]]; then
	echo "WARNING: 6.6.98-5-aw2511 not installed; the prebuilt GPU/IMX327 modules target it." >&2
fi

echo
# A soft reboot leaves the SD card unresponsive to boot0 ("SD/MMC 0 init
# error", "Failed preparing the second boot"): cut the power instead.
echo "Stage 10 done. Power the board off and on again (a soft reboot hangs in boot0)."
