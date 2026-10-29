# a733-sensor-drivers

Sony RAW camera sensors on the **Radxa Cubie A7S** (Allwinner A733), with the
vendor BSP kernel and its VIN camera framework: DKMS drivers, device-tree
overlays, board setup scripts, test tools and the measurements behind them.

The goal was astrophotography: RAW Bayer frames straight from the sensor
(the A733 ISP leaves them untouched), manual exposure from microseconds up to
115 s, analog gain, high frame rates for planetary imaging. Everything here
was developed and measured on one Cubie A7S (4 GB) between 2026-10-10 and
2026-10-23. *Résumé en français en bas de page.*

## Status

| Sensor | Module tested | Status | Link | Modes (measured, 0 dropped frame) |
| --- | --- | --- | --- | --- |
| **IMX327** | Innomaker CAM-MIPI327RAW | ✅ validated | 2 × 891 Mbit/s | 1920×1080 60 fps · 1920×540 112.5 · 1920×480 125 · free height window 1920×h (64 lines: 544 fps) · exposure 1 line … 115 s · automatic LCG/HCG (×2.535) |
| IMX462 | — | ⚠️ built, not tested | same driver as IMX327 | same modes as IMX327, gain up to 72 dB |
| **IMX477** | Raspberry Pi HQ Camera | ✅ validated | 2 × 900 Mbit/s | 4056×3040 · 2028×1520 binned 38.4 · 1916×1080 65.6 · 1920×1080 binned 77.9 · 1276×720 140.8 · 848×480 262.2 · 640×480 binned 446.4 · free ROI (256×128: 804 fps) |
| **IMX585** | Soho Enterprise IMX585 | ✅ validated | **4** × 891 Mbit/s | 3840×2160 30.0 · 1916×1080 115.4 · 1280×720 191.0 · 640×480 271.4 · binned 1920×1080 68.7 · free ROI (256×128: 710 fps) · automatic LCG/HCG · ClearHDR (12-bit compressed) |
| **IMX678** | Soho Enterprise SE678MBD-C | ✅ validated | **4** × 891 Mbit/s | 3840×2160 30.0 · 1916×1080 115.4 · 1280×720 166.7 · 640×480 236.9 · binned 1920×1080 30.0 · automatic LCG/HCG (×2.48) · Bayer RGGB in all four flips · row banding at high gain under investigation (module supply) |
| IMX662 | — | ⚠️ built, not tested | 2 or 4 lanes | same code as IMX585 |

All drivers: RAW12 (RAW10 for the IMX477 binned modes), exposure and gain on
the sensor sub-device, exposure in a common unit (lines of 29.63 µs, Q4), long
exposures by stretching the frame then the line, live-movable crop window
(controls `Crop X/Y Offset`, `ROI Width/Height`). Details: [docs/SENSORS.md](docs/SENSORS.md).

Also checked on the same board: Ethernet 941/933 Mbit/s, Wi-Fi 6 access point
(600 Mbit/s HE-MCS 11), PowerVR BXM-4-64 GPU (Vulkan 1.3, OpenCL), NPU
(VIPLite 2.0). See [docs/TEST-RESULTS.md](docs/TEST-RESULTS.md).

## What you need

- Radxa Cubie A7S, **a heatsink and a 5 V fan** (the board throttles at
  60 °C and reaches critical temperature without cooling), a good 5 V supply,
  a microSD card (UHS-I).
- A camera cable for the board's **31-pin 0.3 mm** connector:
  Radxa 31→15-pin cable for Raspberry Pi 15-pin modules (IMX327, IMX477),
  Radxa 31→22-pin 0.5 mm cable for 22-pin modules (IMX585, 4 lanes).
- Optional: a 3.3 V USB-serial adapter for the debug UART (unplug it before
  cutting power, see [docs/HARDWARE.md](docs/HARDWARE.md)).

Wiring, pinout, cables, the I2C fix and the connector latch problem:
**[docs/HARDWARE.md](docs/HARDWARE.md)**.

## Quick start

On the board (Radxa Debian 13 CLI image, see [docs/INSTALL.md](docs/INSTALL.md)
for every step and its pitfalls):

```sh
sudo apt install -y git
git clone https://github.com/remis-astr/a733-sensor-drivers
cd a733-sensor-drivers
sudo scripts/board/10-system.sh        # kernel 6.6.98-5, tools; then power off and cycle power
sudo scripts/board/20-hardware.sh      # GPU, NPU runtime, rawcap
sudo sh drivers/install.sh             # the 3 DKMS packages, overlays installed disabled
sudo sh scripts/board/select-sensor.sh imx327
sudo systemctl poweroff                # connect the camera, then cycle the power
rawcap -d /dev/video0 -W 1920 -H 1080 -t 0.01 -g 0 -n 5 -o /tmp/test   # 5 FITS frames
```

**Never use `reboot` on this board** (the SD card is not found again): always
power off and cycle the power. Never `rmmod`/`modprobe` a sensor module while
`vin_v4l2` is loaded (the board hangs).

## Repository layout

| Folder | Content |
| --- | --- |
| `drivers/imx327/` | `imx327_mipi` and `imx462_mipi` (DKMS package `imx327-mipi`), with the VIN headers the BSP kernel headers lack (`bsp/`) |
| `drivers/imx477/` | `imx477_mipi` (DKMS package `imx477-mipi`) |
| `drivers/starvis2/` | `imx585_mipi`, `imx678_mipi`, `imx662_mipi` (one source, DKMS package `starvis2-mipi`) |
| `drivers/install.sh` | installs/updates the three DKMS packages and compiles the overlays (disabled) |
| `overlays/` | one overlay per sensor (`cubie-a7s-<sensor>.dtso`) |
| `scripts/board/` | board setup (`10-system`, `20-hardware`), report (`30-checks`), NPU smoke test, sensor selection |
| `tools/rawcap/` | RAW capture to FITS / raw 16-bit, per-frame statistics |
| `tools/i2cbang/` | bit-banged I2C on the camera pins, to debug the bus without the TWI controller |
| `tools/vkcompute/` | minimal headless Vulkan compute check |
| `tools/serlog/` | read-only logger for the debug UART |
| `docs/` | [INSTALL](docs/INSTALL.md) · [HARDWARE](docs/HARDWARE.md) · [SENSORS](docs/SENSORS.md) · [TEST-RESULTS](docs/TEST-RESULTS.md) · [PITFALLS](docs/PITFALLS.md) |

## Sources and credits

The drivers are written for the Allwinner VIN framework of the Radxa BSP and
reuse, with attribution in each file:

- the Radxa Allwinner BSP ([radxa/allwinner-bsp](https://github.com/radxa/allwinner-bsp),
  branch `cubie-aiot-v1.5.0`, GPL-2.0): VIN framework, `imx219.c` as the
  6.6 template, `imx307_mipi.c` timings; the VIN headers in `drivers/imx327/bsp/`;
- the mainline / Raspberry Pi `imx290.c` (IMX327/462 registers, 891 Mbit/s CSI table)
  and its planetary crop modes from the Minicam / Multicam projects;
- the Raspberry Pi `imx477.c` ([raspberrypi/linux](https://github.com/raspberrypi/linux) rpi-6.12.y, GPL-2.0) and its planetary crop modes;
- Will Whang's STARVIS 2 drivers ([will127534](https://github.com/will127534), GPL-2.0):
  init tables (extracted unchanged by `extract_tables.py`), stream-on sequence, HCG and ClearHDR registers;
- lessons from [itssme/radxa-cubie-a7s-IMX477](https://github.com/itssme/radxa-cubie-a7s-IMX477)
  (first IMX477 port on this board, through the ISP);
- Innomaker's [CAM-MIPI327RAW manual](https://github.com/INNO-MAKER/CAM-MIPI327RAW-and-CAM-MIPI462RAW).

Board documentation: Radxa [Cubie A7S hardware page](https://docs.radxa.com/en/cubie/a7s)
and schematic v1.10 (download from Radxa).

## License

GPL-2.0 (see `LICENSE`), like the kernel code the drivers derive from.
Measurements and documentation: free to reuse with attribution.

---

## Résumé en français

Drivers pour capteurs Sony en RAW sur la **Radxa Cubie A7S** (Allwinner A733),
avec le noyau BSP de Radxa (Debian 13, 6.6.98-aw2511) et son framework caméra
VIN : paquets DKMS, overlays, scripts d'installation, outils de test et
résultats de mesure. **Validés** : IMX327 (Innomaker, 1080p à 60 i/s, poses
jusqu'à 115 s), IMX477 (caméra HQ Raspberry Pi, 12 Mpx, recadrages jusqu'à
800 i/s), IMX585 (Soho Enterprise, 4 voies : 4K à 30 i/s, HCG automatique,
ClearHDR), IMX678 (Soho Enterprise, 4 voies : 4K à 30 i/s, HCG automatique dès ×8).
**Écrits mais non testés** : IMX462, IMX662.

Points essentiels :
- **I2C de la caméra** : l'overlay d'origine de Radxa met PE3/PE4 (TWI3) à
  l'intensité la plus faible et le bus échoue avec les modules Raspberry Pi ;
  nos overlays utilisent `drive-strength = <40>` ([docs/HARDWARE.md](docs/HARDWARE.md)).
- **Nappes** : 31→15 broches de Radxa pour les modules Pi 15 broches, 31→22
  broches (0,5 mm) pour les modules 22 broches à 4 voies.
- **Connecteur caméra** : sur notre carte il n'avait pas de clapet ; la nappe
  tient mal et une voie MIPI peut se perdre. Toujours manipuler carte éteinte
  et vérifier les voies au démarrage.
- **Jamais de `reboot`** : arrêt puis coupure de l'alimentation.
- Dissipateur et ventilateur obligatoires.

Installation pas à pas : [docs/INSTALL.md](docs/INSTALL.md) (en anglais).
