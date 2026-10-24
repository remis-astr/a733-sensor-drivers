#!/bin/sh
# Install (or update) every sensor driver of the A7S as DKMS packages, and
# their overlays disabled. Run as root on the board from a copy of the repo's
# drivers/ and overlays/ folders (side by side):
#   sudo sh drivers/install.sh
# Packages: imx327-mipi (imx327_mipi, imx462_mipi), starvis2-mipi
# (imx585_mipi, imx678_mipi, imx662_mipi), imx477-mipi (imx477_mipi). A
# driver is only loaded when the active overlay names it (sensor0_mname):
# installing them changes nothing until a sensor is selected with
# scripts/board/select-sensor.sh. The active overlay is left as is.
# Never reload a sensor module by hand while vin_v4l2 is loaded (the board
# hangs): after an update, sync and power-cycle.
set -e
HERE=$(cd "$(dirname "$0")/.." && pwd)
KVER=${1:-$(uname -r)}

for pkg in imx327 starvis2 imx477; do
	conf="$HERE/drivers/$pkg/dkms.conf"
	name=$(sed -n 's/^PACKAGE_NAME="\(.*\)"/\1/p' "$conf")
	ver=$(sed -n 's/^PACKAGE_VERSION="\(.*\)"/\1/p' "$conf")
	echo "== $name $ver"
	if dkms status "$name/$ver" | grep -q .; then
		dkms remove "$name/$ver" --all
	fi
	dst="/usr/src/$name-$ver"
	rm -rf "${dst:?}"
	mkdir -p "$dst"
	# -L: bsp/ is a symlink to ../imx327/bsp in the starvis2 and imx477 folders
	cp -rL "$HERE/drivers/$pkg/." "$dst/"
	rm -f "$dst"/*.ko
	dkms add "$name/$ver"
	dkms install "$name/$ver" -k "$KVER"
done

echo "== overlays (installed disabled; the active one is not touched)"
inc="/usr/src/linux-headers-$KVER/include"
for src in "$HERE"/overlays/cubie-a7s-imx*.dtso; do
	base=$(basename "$src" .dtso)
	if [ -e "/boot/dtbo/$base.dtbo" ]; then
		echo "$base.dtbo active, left as is"
		continue
	fi
	cpp -nostdinc -undef -x assembler-with-cpp -I "$inc" "$src" \
		| dtc -q -@ -I dts -O dtb -o "/boot/dtbo/$base.dtbo.disabled" -
	echo "$base.dtbo.disabled"
done
sync
echo "Select a sensor: sh scripts/board/select-sensor.sh <imx327|imx462|imx477|imx585|imx678|imx662>"
