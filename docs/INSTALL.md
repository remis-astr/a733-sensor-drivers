# Installation

Tested with Radxa's image `radxa_cubie_a733_trixie_cli_beta_v2` (Debian 13,
no desktop) and the BSP kernel **6.6.98-5-aw2511**. Each step names the trap we
fell into, so you do not have to.

## 0. Before powering the board

- **Cooling first.** Without a heatsink the A733 hits its critical trip
  within minutes of `apt full-upgrade` and the protective shutdown can fail
  (it repeated ~860 times). With a heatsink only, the board idles near 70 °C
  and stays throttled (passive trip at 60 °C). Add a 5 V fan (a `pwm-fan` is
  declared in the device tree).
- **Debug UART** (optional, 115200 8N1, console `ttyAS0`): a 3.3 V USB-serial
  adapter. Its TX line can back-power the SoC when the board is off, and the
  next boot then fails to find the SD card. Unplug the adapter before cutting
  power, or put ~1 kΩ in series on TX.
- **Camera cable**: connect or disconnect only with the board powered off.

## 1. Flash the image

```sh
wget https://dl.radxa.com/cubie/a7s/images/radxa_cubie_a733_trixie_cli_beta_v2.gpt.zip
sha256sum radxa_cubie_a733_trixie_cli_beta_v2.gpt.zip
# 3bc8689afb6a734b99299d2f9438969e4e9c71ceba3c6883d1e9b5306eaeaa51
unzip radxa_cubie_a733_trixie_cli_beta_v2.gpt.zip
sudo dd if=radxa_cubie_a733_trixie_cli_beta_v2.gpt of=/dev/sdX bs=4M conv=fsync status=progress
```

First boot: users `radxa/radxa` and `rock/rock`; SSH is on when no screen is
connected. Get the IP from your router (Ethernet, DHCP), log in, change the
password, install your SSH key.

## 2. System: kernel 6.6.98-5 — `scripts/board/10-system.sh`

```sh
sudo apt update && sudo apt install -y git
git clone https://github.com/remis-astr/a733-sensor-drivers
cd a733-sensor-drivers
sudo scripts/board/10-system.sh
sudo systemctl poweroff      # then cycle the power
```

What it does, and why:

- `apt full-upgrade`, then installs `linux-image-6.6.98-5-aw2511` and
  `linux-headers-6.6.98-5-aw2511` **by name**: the image pins its local
  repository at priority 1999, which keeps the `linux-*-radxa-a733`
  meta-packages at 6.6.98-1. The image's kernel stays installed as a fallback
  (pick it in the extlinux menu on the UART console).
  6.6.98-5 is needed: its headers have `sunxi-sid.h` (the GPU module does not
  build without it) and it brings working cpufreq.
- **Removes `systemd-boot`**. The upgrade updates it and its postinst copies
  `BOOTAA64.EFI` to the (until then empty) EFI partition; U-Boot's EFI boot
  manager then runs it and the board hangs with no console output, instead of
  falling back to `extlinux.conf`. Removing the package cleans the ESP, but its
  prerm runs `kernel-install remove` for every kernel, which deletes
  `modules.dep` (next boot: emergency mode, `autofs4` not found) and
  uninstalls the DKMS modules: the script rebuilds both (`depmod`, `dkms autoinstall`).
- Installs the tools used later (dtc, cpp, i2c-tools, v4l-utils, dkms,
  build tools, vulkan-tools, clinfo, iperf3, ethtool, iw, lm-sensors…).

If the board does not come back: on the UART console, stop autoboot and run
`sysboot mmc 0:3 any ${scriptaddr} /boot/extlinux/extlinux.conf`.

## 3. GPU, NPU, rawcap — `scripts/board/20-hardware.sh`

```sh
sudo scripts/board/20-hardware.sh
```

- **GPU** (PowerVR BXM-4-64): `img-bxm-dkms` from Radxa's repository, plus the
  IMG userspace (Vulkan, OpenCL, firmware) taken out of
  `xserver-xorg-img-bxm` **without** its Xorg/Mesa parts and its
  `/etc/environment`. It also copies `libpvr_mesa_wsi.so` to `/usr/lib`
  (without it `vkCreateInstance` fails with `ERROR_INCOMPATIBLE_DRIVER`) and
  writes the OpenCL ICD. It removes `linux-headers-6.6.98-1-aw2511`, on which
  the GPU module cannot build (dpkg would stay broken).
- **NPU**: `npu-runtime` (VIPLite 2.0 userspace; the kernel driver is built in).
- **rawcap** built into `/usr/local/bin` (installs the BSP's
  `linux-libc-dev=6.6.98-5`: the image's version is an empty package).

## 4. Camera drivers — `drivers/install.sh`

```sh
sudo sh drivers/install.sh
```

Installs the three DKMS packages (rebuilt automatically for every new
kernel): `imx327-mipi` (`imx327_mipi`, `imx462_mipi`), `imx477-mipi`,
`starvis2-mipi` (`imx585_mipi`, `imx678_mipi`, `imx662_mipi`), and compiles
every overlay into `/boot/dtbo/cubie-a7s-<sensor>.dtbo.disabled`. Nothing
changes until a sensor is selected: a driver is only loaded when the active
overlay names it.

The VIN headers are not part of `linux-headers-*-aw2511`; the drivers carry
the 35 they need (`drivers/imx327/bsp/`, from the Radxa BSP v1.5.0). If Radxa
changes the VIN structures in a later BSP, refresh that folder.

## 5. Select the sensor

```sh
sudo sh scripts/board/select-sensor.sh imx585    # imx327 imx462 imx477 imx585 imx678 imx662
sudo systemctl poweroff
```

Then connect the camera (board off) and cycle the power. The script enables
that overlay, disables the others and regenerates `extlinux.conf`
(`u-boot-update`).

Optional, permanent module options (`/etc/modprobe.d/`):

```sh
# IMX327/IMX462: 891 Mbit/s per lane, twice the frame rates (validated)
echo "options imx327_mipi mipi_891=1" | sudo tee /etc/modprobe.d/imx327-mipi.conf
# IMX585: 4 data lanes (needs a 22-pin cable that carries D2/D3, validated)
echo "options imx585_mipi lanes=4" | sudo tee /etc/modprobe.d/imx585-mipi.conf
```

## 6. Check

```sh
dmesg | grep -i -E 'imx|probe'                 # e.g. "IMX477 probe: chip ID 0x0477"
ls /dev/video0 /dev/v4l-subdev*
sudo cat /sys/kernel/debug/mpp/mipi            # every lane must read "HS_S (ok)" while streaming
rawcap -d /dev/video0 -W 1920 -H 1080 -k 1 -n 10 -t 0.01 -g 0 -o /tmp/test
sudo scripts/board/30-checks.sh                # report: system, network, GPU, NPU, camera
```

`rawcap` sets exposure and gain on the sensor sub-device (`/dev/v4l-subdevN`):
controls set on `/dev/video0` go to the ISP handler, which ignores them in RAW.
Pass `-b BGGR` for the IMX477 (the others are RGGB). The first frame after
stream-on has a different duration: skip it (`-k 1`).

## Updating

`git pull`, then `sudo sh drivers/install.sh` and **power-cycle**. Do not
reload a sensor module by hand while `vin_v4l2` is loaded: the next capture
blocks and the board stops answering.
