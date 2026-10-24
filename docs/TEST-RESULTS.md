# Test results

One Radxa Cubie A7S (4 GB), Debian 13, kernel 6.6.98-5-aw2511, heatsink,
October 2026. Numbers are measurements unless marked "computed".

## System

| Item | Result |
| --- | --- |
| Idle (after setup) | 470 MB RAM used of 3.9 GB, 15 services, CPU 100 % idle |
| cpufreq (6.6.98-5) | `ondemand`; A55 416–1794 MHz, A76 416–1976 MHz (absent on the image's 6.6.98-1) |
| Temperature, heatsink only | 41–49 °C idle at first, ~70 °C settled (throttled), 75.7 °C during a DKMS build |
| microSD (SanDisk, UHS SDR104) | read 88 MB/s; sustained write 55–60 MB/s (`O_DIRECT`), 68–73 MB/s buffered |
| USB | one 10 Gbit/s xHCI controller (an SSD would hold several hundred MB/s) |

## Network

| Link | board → PC | PC → board |
| --- | --- | --- |
| Ethernet 1 Gbit/s (`end0`, iperf3) | 941 Mbit/s | 933 Mbit/s |
| Wi-Fi client, 5 GHz ch. 100 (−34 dBm) | 245 Mbit/s | 297 Mbit/s |

Access point (AIC8800D80): NetworkManager's AP mode (wpa_supplicant 2.10)
gives 5 GHz VHT only (phone: 351/390 Mbit/s, VHT-MCS 8); **Wi-Fi 6 needs
hostapd** with `ieee80211ax=1`: phone at HE-MCS 11, 600 Mbit/s link, 80 MHz.
Do not set `country_code`/`ieee80211d` in hostapd (the phy is self-managed:
"Could not set channel for kernel driver").

## GPU (PowerVR BXM-4-64 MC1, DDK 24.2.6603887)

- Vulkan 1.3.277 and OpenCL OK after `scripts/board/20-hardware.sh`.
- `tools/vkcompute` (SAXPY compute shader): bit-exact against the CPU on
  1 Mi and 16 Mi floats.
- Halide 19 (Debian packages), a 1080p RAW → 960×540 colour preview: 1.6 ms
  on the 8 CPU cores, ~14 ms on the GPU (Vulkan, bit-exact) + 2.4 ms copy back.
  For light per-pixel work the CPU wins; keep the GPU for heavier kernels.

## NPU (VIPLite 2.0.3.4, `npu-runtime` 2.0.3)

- `scripts/board/npu-test.sh` (prebuilt models of
  [ZIFENG278/ai-sdk](https://github.com/ZIFENG278/ai-sdk), `v3`): lenet
  ~110 µs, test operator ~2.76 ms per inference.
- Own models converted with Allwinner's ACUITY 6.30.22 (not
  redistributable, not included): a DnCNN-type denoiser runs a 512×512 tile in
  15.5 ms in uint8 (53 ms in int16); a 0.5 M-parameter U-Net, 4×544×960 in
  int16, in 191 ms, identical to PyTorch (correlation 1.0000).
- Pitfalls: an int16 model takes its input in fixed point (value × 2^fl, read
  `fixed_point_pos` from the input buffer); an element-wise op that re-reads
  the network input gave zeros on the board while ACUITY's simulator was right
  (export only the network body, add the input on the CPU); uint8 output can
  bias a small correction (output step too coarse).
- `/dev/vipcore` is mode 666: no root needed. `vpm_run` is root-only (744).

## IMX327 (Innomaker), 1080p unless stated

**Link and frame rates**: see [SENSORS.md](SENSORS.md); 891 Mbit/s validated
with 2 × 2000 frames without a sequence gap, test patterns 1–4 bit-exact,
same signal as at 445.5 Mbit/s.

**Exposure and gain** (signal above black, fixed scene):

| Setting | Signal |
| --- | --- |
| gain 0: 0.5 / 1 / 2 / 4 ms | 3.2 / 6.1 / 11.8 / 23.1 |
| 1 ms: gain 0 / 20 (6 dB) / 40 (12 dB) | 6.1 / 11.2 / 22.4 |

Linear with exposure, ×2 every 6 dB.

**Long exposures** (interval between frames):

| Requested | Measured | HMAX / VMAX |
| --- | --- | --- |
| 0.1 s | 100.0 ms | 4400 / 3377 |
| 1 s | 1000.1 ms | 4400 / 33752 |
| 10 s | 10 000.1 ms | 5665 / 262138 |
| 60 s | 60 000.5 ms | 33990 / 262138 |
| 115 s | 115 001.1 ms | 65147 / 262140 |

The VIN accepts frame and line stretching while streaming
(`lost_cnt 0, error_cnt 0`); `DQBUF` has no timeout.

**Dark frames** (cap on, 54 °C, green G1, 12-bit levels):

| Series | Mean black | Temporal noise | Fixed pattern | Row noise |
| --- | --- | --- | --- | --- |
| 10 ms, gain 0 | 240.41 | 0.91 | 0.13 | 0.096 |
| 10 ms, gain 120 (36 dB) | 240.16 | 34.2 | 8.7 | 2.85 |
| 10 s, gain 0 | 252.09 | 2.48 | 4.77 | 0.117 |

Read noise below 1 level at gain 0; row noise 8–10 % of the pixel noise and
temporal (averages out); 10 s: dark current ~12 levels, column pattern ~1.3,
240 hot pixels (dark subtraction).

**ISP path** (`libawispapi`, `AWISPdemo`, NV12): works mechanically (~27 fps
1080p) but there is no IMX327 ISP profile (default ov13850 one: strong magenta
cast) and `isp_ae=1` overrides manual exposure. For colour previews, debayer
the RAW in software.

## IMX477 (Raspberry Pi HQ Camera)

All 13 modes stream at the computed rates (exposure 2 ms, 150 frames per mode,
0 dropped); ten streams of different modes in a row; 12 MP RAW works; colours
and orientation correct with BGGR. Crop moved live: offsets measured by phase
correlation equal those written; free ROI rates within 0.1 % of the model.
Exposure linear (÷4 → ÷4.0, ×2 → ×2.0).

## IMX585 (Soho Enterprise)

- 2 and 4 lanes at 891 Mbit/s, every mode complete on the test pattern and
  on a scene, rates = model to 0.1 fps; final check after a power cycle:
  4K 30.0, 1916×1080 115.4, 1280×720 191.0, 640×480 271.4, binned 68.7,
  256×128 709.6 fps.
- HCG factor 5.85 (×18.9 LCG vs ×19 HCG: 1.033 vs 1.021 normalised signal).
- Binning = digital 2×2 average (temporal noise 3.04 binned vs 6.05 in 4K,
  3.02 for a software 2×2 average).
- ClearHDR curve measured with a √2 exposure ramp (see SENSORS.md).

## Frame access from the CPU

V4L2 **MMAP buffers of the VIN are uncached**: copying a 1080p frame (4.15 MB)
takes 5.9 ms (700 MB/s), against 0.6 ms RAM to RAM. **USERPTR works**
(vb2-dma-contig + IOMMU): buffers allocated by the application are cached,
copy 0.7 ms. The buffer must be at least the size returned by `queue_setup`
(4 177 920 bytes in 1080p), larger than `sizeimage` (4 147 200), otherwise
`QBUF` fails with EINVAL. With USERPTR: 1080p 60 fps, 540p 112.5, 480p 125
with a copy of every frame and 0 loss.
