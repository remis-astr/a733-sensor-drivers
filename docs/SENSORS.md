# Sensors and drivers

## Common design

- One module per sensor, named `<sensor>_mipi` (= `sensor0_mname` of the
  overlay). Sources of one family are compiled several times by tiny files
  that define a macro and include the family source.
- Output on `/dev/video0` as V4L2 multiplanar Bayer (`RG12`, `BG12`, `RG10`…),
  12-bit data in the low bits of 16-bit words. **The ISP does not modify RAW
  data**: sensor test patterns arrive bit-exact (checked on the IMX327, IMX477,
  IMX585, at every link rate).
- **Controls on the sensor sub-device** (`/dev/v4l-subdevN`, name starts with
  the sensor): with a RAW sensor the VIN forces the ISP into the pipeline,
  and controls set on `/dev/video0` go to the ISP handler, which ignores them.
  - `V4L2_CID_EXPOSURE`: **lines of 29.63 µs in Q4** (16 = one line of HMAX
    4400 at 148.5 MHz), the same unit for every driver, converted by each
    driver into lines of its sensor. Long exposures: the frame (VMAX) grows,
    then the line (HMAX).
  - `V4L2_CID_ANALOGUE_GAIN`: raw gain register of the sensor.
  - `V4L2_CID_GAIN`: linear gain in Q4.
  - All marked `VOLATILE | EXECUTE_ON_WRITE` (with `VOLATILE` alone the V4L2
    core never calls `s_ctrl`: the control answers OK and writes nothing).
- **Movable window** (same control ids in every driver):
  `Crop X Offset` 0x0098290a, `Crop Y Offset` 0x0098290b (offset from the
  centred window, positive = right/down, applied live while streaming),
  `ROI Width` 0x0098290c, `ROI Height` 0x0098290d (free-size mode, refused
  with `-EBUSY` while streaming; it becomes the last mode of the table).
- `isp_ae` parameter (all drivers, off by default): accept exposure/gain
  requests from the ISP 3A. Leave it off for manual RAW work (the ISP
  overwrites exposure and gain).
- A software reset before each stream: on the A733 the MIPI receiver does not
  resynchronise after the sensor's standby (no LP-11 on the clock lane), only
  the first stream after boot would work otherwise.
- **The VIN picks a mode by size only** (the closest size ≥ the request; the
  frame rate only breaks ties when it equals `fps_fixed` exactly). Two modes of
  the same size cannot be told apart, hence the 1916-wide native crops next to
  the 1920-wide binned modes. A free ROI of exactly a binned mode's size
  selects the binned mode: use a slightly different size.
- `fps_fixed` is the real frame rate of each mode: the VIN's TDM sizes its
  buffers from it.
- **`bytesperline`**: after a change of width, `G_FMT` still returns the
  previous stride until `REQBUFS`; read the format again after `REQBUFS`.

## IMX327 / IMX462 — `drivers/imx327`

Modules `imx327_mipi`, `imx462_mipi` (package `imx327-mipi`). 2 lanes.

| Mode | 445.5 Mbit/s (default) | 891 Mbit/s (`mipi_891=1`) |
| --- | --- | --- |
| 1920×1080 | 30 fps | 60.0 fps |
| 1920×540 (centred band) | 56.2 | 112.5 |
| 1920×480 (centred band) | 62.5 | 125.0 |
| 1920×h (`ROI Height` 64…1080, step 4) | 148.5 MHz / (4400 × (h + 60)) | 148.5 MHz / (2200 × (h + 60)): 64 → 544 fps, 300 → 187.5 |

- Exposure from 1 line to **115.7 s** (VMAX ≤ 0x3ffff, HMAX ≤ 0xffff;
  measured: 0.1 s → 100.0 ms … 115 s → 115 001 ms between frames).
- Gain register 0–230 on the IMX327 (0.3 dB steps, 27 dB analog then
  digital, 69 dB), 0–240 on the IMX462 (72 dB).
- Bayer RGGB. Black level 240 (12-bit), `black_level` parameter.
- Window: only the lines are placed on the sensor (`Crop Y Offset`, step 2,
  `ROI Height`): WINMODE window + WINPV, moved live under REGHOLD. **Drop the
  2 frames after a move** (N+1 old position, N+2 over-exposed transition).
  The line duration does not depend on the width: crop X in software.
- **LCG/HCG** (FDG_SEL, 0x3009 bit 4): `V4L2_CID_GAIN` is the effective gain
  in LCG units; in automatic mode (`hcg=2`, default) the driver switches to
  HCG from ×3 (`hcg_gain`), gain register = effective / HCG factor, FDG_SEL
  and gain written under REGHOLD (same frame). IMX327 (Innomaker module):
  **HCG/LCG = 2.535** at every register (±0.2 %, measured on pixels that never
  saturate — a partly saturated scene gave a wrong 2.08), gain law exactly
  0.3 dB/step, no step at the switch, **about 26 % less read noise in HCG** at
  the same effective gain (×2.5 to ×32), full well about 2.5× lower.
  IMX462: factor not measured, stays in LCG (`hcg_factor=0`). Read-only
  control "HCG Active" 0x00982906, as on the IMX585.
- **Orientation**: `flip` parameter (bit 0 horizontal = HREV, bit 1
  vertical = VREV of 0x3007; 3 = 180°), taken at the next stream on (can be
  changed in `/sys/module/imx327_mipi/parameters/flip` without reloading).
  Measured on the IMX327: **Bayer stays RGGB in all four orientations** (the
  sensor shifts its readout by one pixel), and `Crop Y Offset` changes sign
  under a vertical flip so that positive still moves the band down in the
  image.
- Parameters: `mipi_891` (change only while not streaming), `inck`
  (74250000 default, or 37125000), `hcg` (0 LCG, 1 HCG, 2 automatic),
  `hcg_gain`, `hcg_factor` (×1000, set at load), `flip`, `black_level`,
  `test_pattern` (1–7), `isp_ae`.

## IMX477 — `drivers/imx477`

Module `imx477_mipi` (package `imx477-mipi`). 2 lanes at 900 Mbit/s
(PLL 450 MHz from 24 MHz).

| Mode | Measured fps |
| --- | --- |
| 4056×3040 (12 MP) | ~10.3 (computed) |
| 4056×2160 | — |
| 2028×1520 binned | 38.4 |
| 2028×1080 | — |
| 1332×990 binned | 119.1 |
| 1920×1080 binned (RAW10) / 1916×1080 native | 77.9 / 65.6 |
| 1280×720 binned / 1276×720 native | 166.4 / 140.8 |
| 852×480 binned / 848×480 native | 347.5 / 262.2 |
| 640×480 binned / 636×480 native | 446.4 / 262.2 |
| free ROI 256…4048 (step 16) × 128…3040 (step 4) | 256×128: 804 · 1024×768: 165 · 2000×1500: 46 |

- RAW12 for native modes, RAW10 when a binned format is requested.
- `flip=3` (default): readout rotated 180°, Bayer **BGGR**, same orientation
  as libcamera on the Pi. Unlike the STARVIS sensors, the Bayer order follows
  the flip (0..3: RGGB, GRBG, GBRG, BGGR, as the Pi driver) while the format
  stays SBGGR: derive the order from `flip` in the application (not yet
  measured on the A7S).
- Crop X/Y step 4, live, 0 dropped frame; the new position shows 2 frames
  later. In 12-bit the line length follows the MIPI rate
  (`width × 12 × 840/1800 + 500`, at least 5 932): narrowing below ~1200 px
  also speeds up the readout.
- 4056×3040 works in RAW through `/dev/video0` (the large-image problem met
  by other ports concerns the ISP NV12 path).
- Parameters: `flip`, `dpc_enable` (on-sensor defect correction, default on),
  `isp_ae`.

## IMX585 / IMX678 / IMX662 (STARVIS 2) — `drivers/starvis2`

Modules `imx585_mipi`, `imx678_mipi`, `imx662_mipi` (package `starvis2-mipi`).
Shared register map; init tables extracted unchanged from Will Whang's
drivers. **Only the IMX585 has been tested.**

IMX585 (Soho Enterprise module, 24 MHz), 891 Mbit/s per lane, 0 dropped frame:

| Mode | 2 lanes | 4 lanes (`lanes=4`) |
| --- | --- | --- |
| 3840×2160 native | 15.2 | **30.0** |
| 1920×1080 binned | 59.7 | 68.7 |
| 1916×1080 crop | 57.8 | 115.4 |
| 1280×720 crop | 125.0 | 191.0 |
| 640×480 crop | 237.1 | 271.4 |
| free ROI | — | 256×128: 709.6 · 1024×768: 147.5 (2 lanes) |

- **Window width rounded up to a multiple of 16** on the sensor
  (`PIX_HWIDTH` must be a multiple of 16: with 1916 the sensor sent only 1071
  lines); `PIX_VST` aligned on 4.
- **Readout floor**: HMAX 480 on the IMX585 (measured, ~7 % margin; below
  420 the readout breaks: the frame rate collapses and the signal is wrong);
  550 kept for the IMX678/662 until measured (`hmax_min`).
- **Binning** on the IMX585 is a digital 2×2 average on the chip (same
  signal as 4K, half the noise, exactly like a software 2×2 average: no
  read-noise gain). Fast binned mode: HMAX = full frame / 4.
- **LCG/HCG**: `V4L2_CID_GAIN` is the effective gain in LCG units; in
  automatic mode (`hcg=2`, default on the IMX585) the driver switches to HCG
  from ×19 (analog = effective / 5.85, register ≥ 34). HCG/LCG factor 5.85
  measured, no step at the switch, linear to ×374 within ±3 %. Read-only
  control "HCG Active" 0x00982906.
- **ClearHDR** (IMX585): `V4L2_CID_WIDE_DYNAMIC_RANGE`, refused while
  streaming. 16-bit linear output is impossible on the A733 (the capture path
  goes through the TDM, which takes 8/10/12 bits only), so the driver uses the
  sensor's **gradation compression into 12 bits**: output = 200 + C(f), f = the
  fused 16-bit value ≈ 24.8 × normal 12-bit ADU, C = slope 1 up to 500, 1/4 up
  to 11500, 1/64 above. Inverse: c = output − 200; f = c (c ≤ 500),
  500 + 4 (c − 500) (c ≤ 3250), 11500 + 64 (c − 3250). Half the frame rate,
  exposure ≥ 3 lines. `hdr_gain`, `hdr_blend` parameters.
- Black level: BLKLEVEL 50 (= 200 in 12-bit data).
- Link: **891 Mbit/s per lane is the practical limit** on this board: 1188
  gave corrupted frames whatever the D-PHY settle time, 1440 no frame
  (`link_mbps` accepts them, for other boards).
- **Orientation**: `flip` parameter (bit 0 = WINMODEH 0x3020, bit 1 =
  WINMODEV 0x3021; 3 = 180°), taken at the next stream on; Crop X/Y offsets
  change sign on a flipped axis. Will Whang's drivers report the same Bayer
  code whatever the flip; **not measured yet** on these sensors.
- Parameters: `lanes` (2/4), `link_mbps`, `inck` (24 MHz default), `hcg`,
  `hcg_gain`, `hdr_gain`, `hdr_blend`, `flip`, `black_level`, `vmargin`,
  `hmax_min`, `isp_ae`. Change `lanes`/`link_mbps` only while not streaming.

IMX678 / IMX662 checklist when you first connect one: `dmesg` (BLKLEVEL read
back = 50), all lanes `HS_S ok`, measured frame rates equal to the computed
ones (otherwise the module oscillator differs: `inck`), Bayer order on a real
image (also after each `flip`), then 4 lanes, HCG factor, readout floor.
