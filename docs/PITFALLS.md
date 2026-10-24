# Pitfalls (and their fixes)

Collected while bringing up cameras on the Cubie A7S. Most cost hours.

| Symptom | Cause | Fix |
| --- | --- | --- |
| Board silent after `reboot` | boot0 no longer reads the SD card after a warm reset | always `poweroff`, then cut the power |
| Board does not boot although it was off | USB-serial adapter back-powering the SoC through TX | unplug the adapter before cutting power (or 1 kΩ in series) |
| Hang with no console output after `apt full-upgrade` | updated `systemd-boot` put `BOOTAA64.EFI` on the ESP; U-Boot's EFI manager runs it | remove `systemd-boot`, then `depmod` + `dkms autoinstall` (done by `10-system.sh`) |
| Emergency mode, `autofs4` not found | removing `systemd-boot` ran `kernel-install remove`, deleting `modules.dep` and the DKMS modules | `depmod -a`, `dkms autoinstall` for each kernel |
| `full-upgrade` keeps kernel 6.6.98-1 | local repository pinned at 1999 | install `linux-image/headers-6.6.98-5-aw2511` by name |
| GPU module fails to build / dpkg broken | 6.6.98-1 headers lack `sunxi-sid.h` | use 6.6.98-5, remove the -1 headers |
| `vkCreateInstance` → incompatible driver | `libpvr_mesa_wsi.so` only in `/usr/local/lib` | copy it to `/usr/lib` |
| GPU listed twice in Vulkan | a second ICD manifest | keep only `img_icd.json` |
| Camera I2C: `bus error` (state 0x0), sensor never detected | PE3/PE4 at drive level 0 in Radxa's overlay, too weak for the cable's level translator | `drive-strength = <40>` in `twi3_pins_default` |
| Only the first stream after boot delivers frames (`clk_lane: 0x0 (error)`) | the sensor's standby does not return the clock lane to LP-11; the A733 receiver does not resync | software reset of the sensor before each stream (in all drivers) |
| Exposure/gain controls answer OK but nothing changes | controls `VOLATILE` only: the V4L2 core never calls `s_ctrl` | `VOLATILE \| EXECUTE_ON_WRITE` |
| Exposure/gain set on `/dev/video0` ignored | RAW sensor → ISP in the pipeline; the ISP handler swallows them | set them on the sensor's `/dev/v4l-subdevN` |
| Scrambled image after a change of width | `G_FMT` returns the previous `bytesperline` until `REQBUFS` | read the format again after `REQBUFS` |
| Wrong mode selected | the VIN chooses by size only | distinct sizes per mode (1916 native vs 1920 binned) |
| One lane `0x1 (error)` while the others are in HS, no frame | cable not fully engaged (either end); connector without latch | re-seat, board off |
| IMX585 1916-wide frames missing their last 9 lines | `PIX_HWIDTH` must be a multiple of 16 | window rounded up to 16 on the sensor |
| `tdm0 rx0 buffer full`, no frame in RAW16 | the TDM accepts 8/10/12 bits only | 12-bit output (ClearHDR via gradation compression) |
| Corrupted frames at 1188 Mbit/s, none at 1440 | receiver / signal integrity limit of this board | stay at 891 Mbit/s per lane |
| Board hangs after `rmmod`/`modprobe` of a sensor driver | reloading under a loaded `vin_v4l2` | power-cycle after updating a driver |
| HQ Camera not detected (`no answer on I2C`) | it is powered through PE8 and needs ~173 ms | the IMX477 driver waits 300 ms |
| Frames dropped by a capture tool at 30 fps with high gain | per-frame statistics and FITS writing on a slow A55 core | lighter processing, `taskset` on an A76, or no file output |
| `pkill -f pattern` kills its own `sh -c` | the pattern is in its own command line | use a pid file or `pgrep` with a stricter pattern |
| Reading port E data register shows 0 on PE3/PE4 | pins in TWI function read 0; unbinding TWI3 turns VCC-PE off | do not draw conclusions from those reads |
