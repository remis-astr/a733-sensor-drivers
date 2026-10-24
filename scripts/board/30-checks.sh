#!/usr/bin/env bash
# Stage 30, on the board as root: report the state of every phase-1 component.
# Read-only except for an optional iperf3 run against a PC: PC_IP=<pc> (iperf3 -s on the PC).
set -uo pipefail

section() { echo; echo "===== $*"; }

section "System"
uname -a
uptime
free -h
df -h /
systemctl --failed --no-legend || true
systemctl list-units --type=service --state=running --no-legend | wc -l | xargs echo "running services:"
sensors 2>/dev/null | grep -E 'temp|°C' | head -5

section "Ethernet"
for i in $(ls /sys/class/net | grep -E '^(end|eth)'); do
	ip -br addr show "$i"
	ethtool "$i" 2>/dev/null | grep -E 'Speed|Duplex|Link detected'
	ethtool -S "$i" 2>/dev/null | grep -iE 'err|drop' | grep -v ': 0$' | head
done
if [[ -n "${PC_IP:-}" ]] && command -v iperf3 >/dev/null; then
	iperf3 -c "$PC_IP" -t 10 | tail -4
fi

section "Wi-Fi"
lsmod | grep -E 'aic8800' || echo "aic8800 module not loaded"
iw dev 2>/dev/null || true
nmcli -t -f DEVICE,TYPE,STATE device 2>/dev/null

section "GPU"
lsmod | grep -E '^pvrsrvkm' || echo "pvrsrvkm NOT loaded"
dmesg | grep -iE 'pvr|rgx|powervr' | tail -8
vulkaninfo --summary 2>&1 | grep -E 'deviceName|apiVersion|driverVersion|ERROR' | head -8
clinfo -l 2>&1 | head -5

section "NPU"
ls -l /dev/vipcore 2>&1
dmesg | grep -iE 'vip|npu' | tail -5
dpkg -L npu-runtime 2>/dev/null | grep -E 'bin/|\.so' | head -10
bash "$(dirname "$0")/npu-test.sh"

section "Camera (VIN)"
lsmod | grep -E '^vin_|_mipi' || true
ls /dev/video* /dev/media* 2>/dev/null
ls /boot/dtbo/ | grep -i 'cubie-a7s-imx' 
grep -E 'fdtoverlays' /boot/extlinux/extlinux.conf || echo "no overlay enabled"
dmesg | grep -iE 'vin|imx|sensor|twi3' | tail -12
i2cdetect -l 2>/dev/null | head

section "Errors since boot"
dmesg --level=err,crit,alert,emerg | tail -20
