#!/usr/bin/env bash
# Stage 20, on the board as root, after rebooting onto 6.6.98-5-aw2511.
#  - GPU: img-bxm-dkms (pvrsrvkm) + the IMG userspace (Vulkan, OpenCL, firmware)
#    taken out of Radxa's xserver-xorg-img-bxm .deb without its Xorg/Mesa parts
#    and without its /etc/environment; Vulkan/OpenCL ICD files written here.
#    Kernel module and userspace are both DDK 24.2.6603887.
#  - NPU: npu-runtime (VIPLite userspace; the kernel driver is built in).
#  - rawcap: RAW capture tool (tools/rawcap), built here into /usr/local/bin.
# The camera drivers are installed afterwards by drivers/install.sh.
# Run from the repository: sudo scripts/board/20-hardware.sh
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

KVER="$(uname -r)"
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
WANT=6.6.98-5-aw2511

echo "== $(date -Is) stage 20 on $(hostname), kernel $KVER"
if [[ "$KVER" != "$WANT" ]]; then
	echo "ERROR: running $KVER, expected $WANT (run stage 10 and reboot)." >&2
	exit 1
fi

# DKMS builds for every kernel that has headers; the image's 6.6.98-1 headers
# lack sunxi-sid.h, so img-bxm-dkms would fail there and break dpkg. The -1
# kernel image itself stays installed as a fallback.
if dpkg -s linux-headers-6.6.98-1-aw2511 >/dev/null 2>&1; then
	echo "== removing linux-headers-6.6.98-1-aw2511 (GPU module cannot build there)"
	apt-get remove -y linux-headers-6.6.98-1-aw2511
fi

echo "== GPU kernel module (img-bxm-dkms)"
apt-get install -y img-bxm-dkms
dkms status img-bxm-dkms

echo "== GPU userspace (from xserver-xorg-img-bxm, libraries and firmware only)"
work="$(mktemp -d)"
( cd "$work" && apt-get download xserver-xorg-img-bxm-1.21.1-2.deb )
dpkg-deb -x "$work"/*.deb "$work/x"
cp -a "$work"/x/lib/firmware/rgx.* /lib/firmware/
for lib in VK_IMG srv_um PVROCL glslcompiler usc ufwriter PVRScopeServices sutu_display pvr_dri_support; do
	cp -a "$work"/x/usr/lib/lib"$lib".so* /usr/lib/
done
# libVK_IMG dlopens its WSI layer at vkCreateInstance (returns -3 without it);
# the package ships it under /usr/local/lib.
cp -a "$work"/x/usr/local/lib/libpvr_mesa_wsi.so /usr/lib/
rm -rf "$work"

# The Vulkan ICD manifest comes with img-bxm-dkms (/usr/share/vulkan/icd.d/
# img_icd.json); a second one would list the GPU twice.
rm -f /etc/vulkan/icd.d/powervr_icd.json
mkdir -p /etc/OpenCL/vendors
echo /usr/lib/libPVROCL.so.1 > /etc/OpenCL/vendors/pvr.icd
ldconfig

modprobe pvrsrvkm
lsmod | grep -E '^pvrsrvkm' || echo "WARNING: pvrsrvkm not loaded" >&2
ls -l /dev/dri/ 2>/dev/null || true

echo "== NPU runtime"
apt-get install -y npu-runtime
ls -l /dev/vipcore 2>/dev/null || echo "WARNING: /dev/vipcore missing" >&2

echo "== rawcap (RAW capture tool)"
# The image's linux-libc-dev 6.6.98-1 (local repo, pinned 1999) is an empty
# meta-package: no <linux/*.h>. Take the BSP's own uapi headers, built with -5.
apt-get install -y linux-libc-dev=6.6.98-5
cc -O2 -Wall -o /usr/local/bin/rawcap "$ROOT/tools/rawcap/rawcap.c"
ls -l /usr/local/bin/rawcap

echo
echo "Stage 20 done. Next: sudo sh drivers/install.sh, then select a sensor (scripts/board/select-sensor.sh)."
