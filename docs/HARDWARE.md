# Hardware

## Board

Radxa Cubie A7S, Allwinner A733 (2 × Cortex-A76 + 6 × Cortex-A55), 4 GB,
PowerVR BXM-4-64 GPU, VIP9000-family NPU, Gigabit Ethernet, Wi-Fi 6 / BT
(AIC8800D80 on internal USB), microSD boot. Schematic v1.10 from Radxa.

- **Cooling is mandatory.** Thermal trips: passive 60 °C, critical 110 °C
  (`power_allocator`). Heatsink only: ~70 °C idle, permanently throttled
  (A76 limited to 1404 MHz). Heatsink + fan for long captures.
- **Power**: 5 V. Always `systemctl poweroff`, then cut the power: after a
  soft `reboot`, the first-stage loader (boot0, closed source) no longer gets
  an answer from the SD card (`SD/MMC 0 init error`) and the board stays
  silent. We tried power-cycling the SD slot (`dc1sw2` of the AXP8191 PMIC)
  and the PF bank voltage from a shutdown hook: no change.
- **Debug UART**: 3.3 V, 115200 8N1. A connected adapter back-powers the SoC
  through TX when the board is off and leaves it in the same failed state:
  unplug it before cutting power.

## Camera connector (J5)

31-pin, 0.3 mm pitch, bottom contact. Radxa's documentation describes a flip
latch ("make sure the connector latch is fully closed"). **Our board arrived
with no latch at all**: the cable is held by friction only, and a slightly
moved cable loses one MIPI lane (see below). Handle it board off, push the
cable fully in, and check the lanes at every boot.

Signals (schematic v1.10, same pinout as the ROCK 5B camera connector):

| J5 pin | Signal | A733 |
| --- | --- | --- |
| 2/3, 5/6, 11/12, 14/15 | MIPI D3, D2, D1, D0 (port `MCSIB`, 4 lanes) | mipi1 |
| 17/18 | MIPI clock (CLKA) | mipi1 |
| 8/9 | CLKB (not connected on the A7S) | — |
| 20 | MCLK | PE5 |
| 23 | CM-PDN0, power enable of Raspberry Pi modules (100 kΩ to VCC-PE) | PE8 |
| 24 / 25 | SCL / SDA, 2.2 kΩ pull-ups to VCC-PE (**1.8 V**) | TWI3 = PE3 / PE4 |
| 27 | RESET (100 kΩ) | PE9 |
| 28/29 | 3.3 V (always on) | — |
| 30/31 | 5 V | — |

The A733 `mipi1` port handles the 4 lanes by itself: 4-lane sensors work
(validated with the IMX585).

## Cables

| Cable | Use | Result |
| --- | --- | --- |
| Radxa 31→15-pin FPC "designed for ROCK 5" (marked KS2101124O, v1.1) | Raspberry Pi 15-pin modules (IMX327 Innomaker, IMX477 HQ Camera) | ✅ works, **with the I2C drive-strength fix below** |
| Radxa 31→22-pin FPC, 0.5 mm on the 22-pin side | 22-pin modules, **carries D2/D3** | ✅ IMX585 on 4 lanes |

The 31→15 cable (schematic "FPC cable 008" v1.1) carries an I2C level
translator (UM3204H, a TXS0104-type transmission-gate translator, its 1.8 V
side fed by a 2 kΩ / 2.4 kΩ divider on 3.3 V) and leaves pin 4 (MCLK) open.
Its 15-pin side: 1 3V3, 2 SDA, 3 SCL, 4 CAM0CLK (resistor not fitted),
5 PDN, 6 GND, 7/8 CLK, 9 GND, 10/11 D1, 12 GND, 13/14 D0, 15 GND. It is the
mirror of the Raspberry Pi pinout (cable pin n = module pin 16 − n): a
Pi-pinout module plugs in the normal way, contacts towards the PCB. Radxa
announced a v1.2 of this cable that fixes I2C instability (forum thread
"A7Z + Waveshare IMX219", 2026-10).

**Lane diagnosis**: while streaming, `/sys/kernel/debug/mpp/mipi` must show
every lane in `0x5 (HS_S, ok)`. A lane that follows LP-11 but reads
`0x1 (error)` when the others enter HS, or a `clk_lane: 0x1 (error)` with
`mipi_bps 0`, is a contact problem: re-seat the cable at **both** ends (an
IMX585 lane 1 failure turned out to be the module side).

## Camera I2C: the drive-strength fix

With Radxa's camera overlay (made for the Radxa Camera 8M 219), PE3/PE4 are
configured with `drive-strength = <10>`, i.e. **level 0** of 4. With the
31→15 cable and a Raspberry Pi module, the TWI3 controller then fails:
`bus error` (state 0x0), `i2cdetect -y -r 3` sees nothing, the sensor is
never detected. The level translator on the cable only accepts a low level
below ~0.27 V, and a level-0 output sinking the pull-ups of both sides (and
those of the module, through the translator) stays above it.

Measured on the Pi HQ Camera, 100 kHz, changing the level live:

| PE3/PE4 drive level | ID reads of the IMX477 |
| --- | --- |
| 0 (`drive-strength = <10>`) | fail (timeouts, bus errors) |
| 1 | detected, one bus error |
| 2 | 300 / 300 |
| 3 (`drive-strength = <40>`) | 300 / 300 |

**Every overlay here uses `drive-strength = <40>`**, at 400 kHz in `drv`
mode (Allwinner's recommended settings): 500/500 reads, no TWI error.
Nothing else needed changing (not the cable, not the module).

`tools/i2cbang` (bit-banged I2C on PE3/PE4 through `/dev/mem`) and the PE
bank registers (base 0x02000280: CFG +0x00, DAT +0x10, **DRV +0x20/+0x24,
4 bits per pin**, PULL +0x30) were the tools of this diagnosis. Reading the
data register of a pin in TWI function returns 0, and unbinding TWI3 turns
off `bldo1` (VCC-PE): neither proves anything about the bus.

## Camera modules tested

| Module | Sensor | Oscillator | Notes |
| --- | --- | --- | --- |
| Innomaker CAM-MIPI327RAW | IMX327 | 74.25 MHz | Pi pinout, pins 11/12 not connected (always powered), own regulators and I2C level shifter (2.2 kΩ to 3.3 V); RGGB; no ID register (presence read only) |
| Raspberry Pi HQ Camera | IMX477 | 24 MHz | powered through pin 11 (PE8): ID readable 173 ms after power-on, the driver waits 300 ms; BGGR with the 180° readout |
| Soho Enterprise IMX585 (22-pin) | IMX585 | 24 MHz | 4 lanes on the 31→22 cable; module ≠ StarlightEye |

All sensors at I2C address 0x1a (0x34 in the VIN's 8-bit notation:
`sensor0_twi_addr`). The overlays differ only by `sensor0_mname`, which must
equal the driver's module name (`<sensor>_mipi`).
