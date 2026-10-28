/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Sony Starvis 2 RAW sensor driver (IMX585, IMX678, IMX662) for the Allwinner
 * VIN framework (Radxa Cubie A7S, A733 / sun60iw2, BSP kernel 6.6.98-aw2511).
 *
 * One source, three modules: imx585_mipi.c / imx678_mipi.c / imx662_mipi.c
 * define STARVIS2_IMX585 / _IMX678 / _IMX662 and include this file. The three
 * sensors share their register map (STANDBY 0x3000, XMSTA 0x3002, INCK_SEL
 * 0x3014, DATARATE_SEL 0x3015, VMAX 0x3028, HMAX 0x302c, SHR0 0x3050, gain
 * 0x306c / 0x3070 on the IMX678, BLKLEVEL 0x30dc, window 0x303c-0x3046);
 * their init tables differ (starvis2_tables.h).
 *
 * Written 2026-10-17 WITHOUT hardware, from:
 *  - the VIN framework, exposure/gain handling and long-pose strategy of
 *    drivers/imx327/imx327_mipi.c (validated on the A7S);
 *  - Will Whang's Raspberry Pi drivers (github.com/will127534, GPL-2.0):
 *    init tables (extracted unchanged), stream-on sequence, link rates,
 *    INCK table, HMAX per link rate (4-lane table x2 on 2 lanes), window
 *    cropping registers (imx585.c), 500 ms power-on delay.
 *
 * Output: MEDIA_BUS_FMT_SRGGB12_1X12 (Will's unflipped RGGB), 2 data lanes (4 with
 * the "lanes" parameter).
 *
 * Exposure: same unit as every A7S driver, lines of 29.63 us in Q4 (16 = one
 * line of HMAX 4400 at 148.5 MHz, the IMX327's 445.5 Mbit/s line). Converted
 * here to lines of the current HMAX (HMAX / 74.25 MHz; at 891 Mbit/s the
 * full-width HMAX is 2200, i.e. exactly that unit). Poses longer than the
 * frame stretch VMAX (20 bits), then HMAX, as in imx327_mipi.c.
 *
 * Modes:
 *  - IMX585 / IMX678: 1920x1080 2x2 binned (full field; IMX585: digital
 *    average, HMAX a quarter of the native one), 3840x2160 native,
 *    and centered native-resolution "planetary" crops 1916x1080 (not 1920:
 *    the VIN picks a window by size alone), 1280x720, 640x480 through the sensor's window cropping mode (WINMODE crop,
 *    PIX_HST/HWIDTH/VST/VWIDTH), which reads fewer lines and pixels;
 *  - IMX662: 1920x1080 native and the 1280x720, 640x480 crops.
 *  - a free window, last ("ROI Width/Height", as imx477_mipi.c and the
 *    RpiCamera3 imx585.c): native crop of any size (16 x 4 steps) set while
 *    not streaming; the VIN reads sensor_win_sizes at S_FMT, so request the
 *    new size afterwards.
 *
 * ClearHDR (IMX585, 2026-10-23, from RpiCamera3's imx585.c): control
 * V4L2_CID_WIDE_DYNAMIC_RANGE (set while not streaming) streams the sensor's
 * on-chip HG/LG fusion in RAW12 with gradation compression: the 16-bit fused
 * value is compressed piecewise linearly (slope 1 up to 500, 1/4 up to 11500,
 * 1/64 above: 0..65535 -> 0..4094), to be linearised by the application.
 * The A733 cannot take the sensor's linear RAW16: its TDM (offline path)
 * accepts 8/10/12-bit buses only. VMAX doubled, SHR >= 10, gain register
 * <= 80 (24 dB), always LCG. Fusion: hdr_gain (EXP_GAIN), hdr_blend (EXP_BK),
 * thresholds 512 / 1024 (RpiCamera3's defaults).
 *
 * Movable crop (2026-10-23): "Crop X/Y Offset" move the crop window from its
 * centred position, in image pixels (positive = right/down), in steps of 4
 * (Bayer phase kept, PIX_VST multiple of 4), clamped to the active area.
 * Applied at stream start and live while streaming (PIX_HST/PIX_VST under
 * REGHOLD, as RpiCamera3's driver). No effect on full-field modes.
 *
 * Orientation (2026-10-28): parameter "flip" (bit 0 horizontal = WINMODEH
 * 0x3020, bit 1 vertical = WINMODEV 0x3021, 3 = 180 degrees, as the A7S
 * IMX327/IMX477 drivers), written at stream on; the crop offsets change sign
 * on a flipped axis so that positive stays right/down in the image. Will's
 * drivers report the same Bayer code whatever the flip (sensor shifts its
 * readout, as measured on the IMX327): to check on each sensor.
 *
 * TO VALIDATE ON HARDWARE (nothing here has run yet):
 *  - crop timing: VMAX = height + the full frame's vertical blanking, HMAX
 *    scaled to the cropped width (link budget), floored at hmax_min (the
 *    4-lane minimum, i.e. the readout limit) — parameters vmargin/hmax_min;
 *  - window origin (CROP_HST0/VST0 below: Will's IMX585 values, assumed for
 *    the IMX678/IMX662) and the Bayer phase of the crops;
 *  - REGHOLD at 0x3001 (Sony convention, not used by Will's drivers);
 *  - whether stream off (STANDBY) puts the MIPI clock lane back to LP-11
 *    (the IMX327 needed a software reset for the A733 receiver; here the
 *    module power is cycled by PE8 at each open of /dev/video0);
 *  - 1188 / 1440 Mbit/s on the A733 receiver (only 891 is validated).
 */

#include "utility/vin_log.h"
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>

#include "camera.h"
#include "sensor_helper.h"

MODULE_AUTHOR("Remi Sanderre");
MODULE_DESCRIPTION("A low-level driver for Sony Starvis 2 sensors (RAW, Allwinner VIN)");
MODULE_LICENSE("GPL");

/* ------------------------------------------------------------------------
 * Per-sensor constants
 */
#if defined(STARVIS2_IMX585)
#define SENSOR_NAME		"imx585_mipi"
#define SENSOR_LABEL		"IMX585"
#define V4L2_IDENT_SENSOR	0x0585
#define SV2_REG_GAIN		0x306c
#define SV2_ACTIVE_W		3840
#define SV2_ACTIVE_H		2160
#define SV2_VMAX_FULL		2250
#define SV2_HAS_BINNING		1
/* min HMAX, 4-lane, 12-bit, all pixels, at 891 / 1188 / 1440 Mbit/s */
#define SV2_HMAX_4L		{1100, 792, 660}
/* HCG / LCG sensitivity x1000, measured 2026-10-20 (RpiCamera3,
 * tools/measure_imx585_hcg.py); 0 = unknown, no automatic switch */
#define SV2_HCG_FACTOR_MILLI	5850
#define SV2_HAS_CLEARHDR	1
/* Lowest HMAX of the readout, measured 2026-10-23 (4 lanes): 450 good, 420
 * and below wrong signal / frame-rate collapse; 480 keeps ~7 % margin */
#define SV2_HMAX_MIN		480
/* Binned: HMAX a quarter of the native one (measured, see sv2_mode_hmax) */
#define SV2_FAST_BINNING	1
#elif defined(STARVIS2_IMX678)
#define SENSOR_NAME		"imx678_mipi"
#define SENSOR_LABEL		"IMX678"
#define V4L2_IDENT_SENSOR	0x0678
#define SV2_REG_GAIN		0x3070
#define SV2_ACTIVE_W		3840
#define SV2_ACTIVE_H		2160
#define SV2_VMAX_FULL		2250
#define SV2_HAS_BINNING		1
#define SV2_HMAX_4L		{1100, 792, 660}
#define SV2_HCG_FACTOR_MILLI	0
#elif defined(STARVIS2_IMX662)
#define SENSOR_NAME		"imx662_mipi"
#define SENSOR_LABEL		"IMX662"
#define V4L2_IDENT_SENSOR	0x0662
#define SV2_REG_GAIN		0x306c
#define SV2_ACTIVE_W		1920
#define SV2_ACTIVE_H		1080
#define SV2_VMAX_FULL		1250
#define SV2_HAS_BINNING		0
#define SV2_HMAX_4L		{990, 660, 660}
#define SV2_HCG_FACTOR_MILLI	0
#else
#error "define STARVIS2_IMX585, STARVIS2_IMX678 or STARVIS2_IMX662"
#endif
#ifndef SV2_HAS_CLEARHDR
#define SV2_HAS_CLEARHDR	0
#endif
#ifndef SV2_HMAX_MIN
#define SV2_HMAX_MIN		550	/* the 4-lane readout limit of Will's tables */
#endif
#ifndef SV2_FAST_BINNING
#define SV2_FAST_BINNING	0	/* binned at the native HMAX until measured */
#endif

/* Window origin of the active area (Will's imx585.c: PIX_HST 8, PIX_VST 12). */
#define CROP_HST0		8
#define CROP_VST0		12

/* Movable crop and free window: same control IDs as imx477_mipi.c */
#define V4L2_CID_SV2_BASE	(V4L2_CID_USER_BASE + 0x2000)
#define V4L2_CID_SV2_HCG	(V4L2_CID_SV2_BASE + 6)	/* RpiCamera3 id, read only here */
#define V4L2_CID_SV2_CROP_X	(V4L2_CID_SV2_BASE + 10)
#define V4L2_CID_SV2_CROP_Y	(V4L2_CID_SV2_BASE + 11)
#define V4L2_CID_SV2_ROI_WIDTH	(V4L2_CID_SV2_BASE + 12)
#define V4L2_CID_SV2_ROI_HEIGHT	(V4L2_CID_SV2_BASE + 13)
#define SV2_ROI_MIN_W		256
#define SV2_ROI_MIN_H		128
#define SV2_ROI_STEP_W		16	/* PIX_HWIDTH */
#define SV2_ROI_STEP_H		4	/* PIX_VWIDTH */
#define SV2_ROI_DEF_W		1024
#define SV2_ROI_DEF_H		768

/* ------------------------------------------------------------------------
 * Registers
 */
#define SV2_STANDBY		0x3000
#define SV2_REGHOLD		0x3001
#define SV2_XMSTA		0x3002
#define SV2_INCK_SEL		0x3014
#define SV2_DATARATE_SEL	0x3015
#define SV2_WINMODE		0x3018
#define SV2_WINMODE_CROP	0x04	/* bit 2: window cropping */
#define SV2_BIN_MODE		0x3019	/* 0 = colour */
#define SV2_ADDMODE		0x301b	/* 1 = 2x2 binning */
#define SV2_FDG_SEL0		0x3030	/* 1 = HCG */
#define SV2_WINMODEH		0x3020	/* bit 0: horizontal flip */
#define SV2_WINMODEV		0x3021	/* bit 0: vertical flip */
#define SV2_PIX_HST		0x303c
#define SV2_PIX_HWIDTH		0x303e
#define SV2_LANEMODE		0x3040
#define SV2_PIX_VST		0x3044
#define SV2_PIX_VWIDTH		0x3046
#define SV2_VMAX		0x3028	/* 20 bits, little endian */
#define SV2_VMAX_MAX		0xfffff
#define SV2_HMAX		0x302c	/* 16 bits */
#define SV2_HMAX_MAX		0xffff
#define SV2_SHR0		0x3050	/* 20 bits */
#define SV2_SHR_MIN		8
#define SV2_SHR_MIN_HDR		10
#define SV2_XXS_OUTSEL		0x30a4
#define SV2_XXS_DRV		0x30a6
#define SV2_EXTMODE		0x30ce
#define SV2_DIG_CLP_VSTART	0x30d5	/* IMX585: 4 native, 2 binned */
#define SV2_BLKLEVEL		0x30dc	/* 16 bits */
#define SV2_DIGITAL_CLAMP	0x3458

#define SV2_CLK			74250000	/* HMAX counts this clock */
#define SV2_HMAX_NOMINAL	2200		/* one 29.63 us exposure unit */
#define SV2_EXP_MIN		2
/* Longest pose in nominal lines: (VMAX_MAX - SHR_MIN) x HMAX_MAX / 2200. */
#define SV2_EXP_LINES_MAX	31237000
/* 0..240 at 0.3 dB/step (72 dB, analog then digital). */
#define SV2_GAIN_REG_MAX	240
#define SV2_GAIN_REG_MIN_HCG	34
#define SV2_GAIN_REG_MAX_HDR	80	/* 24 dB in ClearHDR */
#define SV2_GAIN_Q4_MAX		(3981 << 4)	/* 10^(72/20) */
/* Effective gain (LCG equivalent): x HCG factor at the top */
#define SV2_GAIN_Q4_MAX_EFF	(SV2_HCG_FACTOR_MILLI ?				\
	(unsigned int)((u64)SV2_GAIN_Q4_MAX * SV2_HCG_FACTOR_MILLI / 1000) :	\
	SV2_GAIN_Q4_MAX)

/* ------------------------------------------------------------------------
 * Module parameters
 */
static uint inck = 24000000;
module_param(inck, uint, 0444);
MODULE_PARM_DESC(inck, "Module oscillator in Hz: 24000000 (StarlightEye and Soho Enterprise IMX585 modules, default), 27000000, 37125000, 72000000, 74250000");

static uint link_mbps = 891;	/* set through link_ops, after the mode table */
static uint lanes = 2;		/* idem: 2 or 4 MIPI data lanes */

/*
 * Conversion gain (FDG_SEL0). V4L2_CID_GAIN is the effective gain, LCG
 * equivalent: in auto mode the driver switches to HCG from hcg_gain on, with
 * analog gain = effective / HCG factor (register >= 34, i.e. x3.24: x19
 * effective on the IMX585, continuous with LCG). Gain and FDG_SEL0 are
 * written under REGHOLD, so they take effect on the same frame.
 */
static uint hcg = SV2_HCG_FACTOR_MILLI ? 2 : 0;
module_param(hcg, uint, 0664);
MODULE_PARM_DESC(hcg, "Conversion gain: 0 = LCG, 1 = HCG (gain register >= 34), 2 = automatic from hcg_gain (default on the IMX585, the only one measured)");

static uint hcg_gain = 19;
module_param(hcg_gain, uint, 0664);
MODULE_PARM_DESC(hcg_gain, "Automatic mode: effective gain (x) from which HCG is used (default 19, the lowest HCG analog gain x HCG factor)");

static uint hdr_gain = 2;
module_param(hdr_gain, uint, 0664);
MODULE_PARM_DESC(hdr_gain, "ClearHDR: HG/LG gain difference EXP_GAIN (0 = +0, 1 = +6, 2 = +12 dB default, 3 = +18, 4 = +24, 5 = +29.1 dB)");

static uint hdr_blend = 0;
module_param(hdr_blend, uint, 0664);
MODULE_PARM_DESC(hdr_blend, "ClearHDR: HG/LG blending EXP_BK (0 = 1/2-1/2 default, 1 = HG 3/4, 3 = HG 7/8, 4 = HG 15/16, 6 = HG 1/16, 7 = HG 1/8, 8 = HG 1/4)");

static uint black_level = 50;
module_param(black_level, uint, 0664);
MODULE_PARM_DESC(black_level, "BLKLEVEL register (default 50, Will Whang's drivers; 200 in 12-bit data)");

static bool isp_ae;
module_param(isp_ae, bool, 0664);
MODULE_PARM_DESC(isp_ae, "Accept exposure/gain requests from the ISP 3A; off = manual RAW control only");

static uint flip;
module_param(flip, uint, 0664);
MODULE_PARM_DESC(flip, "Readout orientation: bit 0 horizontal, bit 1 vertical (3 = 180 degrees); taken at the next stream on");

static uint vmargin = SV2_VMAX_FULL - SV2_ACTIVE_H;	/* set through timing_ops */
static uint hmax_min = SV2_HMAX_MIN;			/* idem */

/* ------------------------------------------------------------------------
 * Register tables
 */
#include "starvis2_tables.h"

static struct regval_list sensor_fmt_raw[] = {
};

static int sv2_write_table(struct v4l2_subdev *sd, struct regval_list *regs,
			   int size, const char *name)
{
	int i, ret;

	for (i = 0; i < size; i++) {
		ret = sensor_write(sd, regs[i].addr, regs[i].data);
		if (ret < 0) {
			sensor_err("%s failed at %d/%d: reg=0x%04x val=0x%02x ret=%d\n",
				   name, i, size, regs[i].addr, regs[i].data, ret);
			return ret;
		}
		usleep_range(100, 200);
	}
	return 0;
}

static int sv2_write16(struct v4l2_subdev *sd, addr_type reg, unsigned int val)
{
	int ret = sensor_write(sd, reg, val & 0xff);

	return ret ? ret : sensor_write(sd, reg + 1, (val >> 8) & 0xff);
}

#if SV2_HAS_CLEARHDR
/* RpiCamera3 imx585.c: common_clearHDR_mode / common_normal_mode */
static struct regval_list sv2_clearhdr_regs[] = {
	{0x301a, 0x10},	/* WDMODE: Clear HDR */
	{0x3024, 0x02},	/* COMBI_EN */
	{0x3069, 0x02},
	{0x3074, 0x63},
	{0x3930, 0xe6}, {0x3931, 0x00},	/* DUR */
	{0x3a4c, 0x61}, {0x3a4d, 0x02},
	{0x3a50, 0x70}, {0x3a51, 0x02},
	{0x3e10, 0x17},	/* ADTHEN */
	{0x493c, 0x41}, {0x4940, 0x41},
	{0x36ef, 0x01},	/* CCMP_EN: gradation compression to 12 bits */
	{0x36e8, 0xf4}, {0x36e9, 0x01}, {0x36ea, 0x00},	/* CCMP1_EXP 500 */
	{0x36e4, 0xec}, {0x36e5, 0x2c}, {0x36e6, 0x00},	/* CCMP2_EXP 11500 */
	{0x36ee, 0x02},	/* ACMP1_EXP: slope 1/4 */
	{0x36ec, 0x06},	/* ACMP2_EXP: slope 1/64 */
	{0x36d0, 0x00}, {0x36d1, 0x02},	/* EXP_TH_H 512 */
	{0x36d4, 0x00}, {0x36d5, 0x04},	/* EXP_TH_L 1024 */
};

static struct regval_list sv2_normal_regs[] = {
	{0x301a, 0x00},	/* WDMODE: Normal */
	{0x3024, 0x00},
	{0x3069, 0x00},
	{0x3074, 0x64},
	{0x3930, 0x0c}, {0x3931, 0x01},
	{0x3a4c, 0x39}, {0x3a4d, 0x01},
	{0x3a50, 0x48}, {0x3a51, 0x01},
	{0x3e10, 0x10},
	{0x493c, 0x23}, {0x4940, 0x23},
	{0x3081, 0x00},	/* EXP_GAIN */
};
#endif

static int sv2_write20(struct v4l2_subdev *sd, addr_type reg, unsigned int val)
{
	int ret = sensor_write(sd, reg, val & 0xff);

	if (!ret)
		ret = sensor_write(sd, reg + 1, (val >> 8) & 0xff);
	return ret ? ret : sensor_write(sd, reg + 2, (val >> 16) & 0x0f);
}

/* ------------------------------------------------------------------------
 * Modes. hts/vts/fps_fixed/mipi_bps are recomputed by sv2_update_timing()
 * from the link rate and the crop parameters.
 */
struct sv2_mode {
	unsigned int width, height;
	bool binned;
	bool roi;		/* free window: size from the ROI controls */
};

static struct sv2_mode sv2_modes[] = {
#if SV2_HAS_BINNING
	{1920, 1080, true},			/* full field, 2x2 binned */
	{SV2_ACTIVE_W, SV2_ACTIVE_H, false},	/* native */
#endif
	/*
	 * Centered crop. The VIN picks a window by size alone, so it must not
	 * equal the binned mode's: 1916 wide, as the Pi's IMX477 crop modes
	 * (1920 on the IMX662, which has no binned mode).
	 */
	{SV2_HAS_BINNING ? 1916 : 1920, 1080, false},
	{1280, 720, false},
	{640, 480, false},
	{SV2_ROI_DEF_W, SV2_ROI_DEF_H, false, true},	/* free window, last */
};
#define SV2_ROI_MODE	(&sv2_modes[ARRAY_SIZE(sv2_modes) - 1])

#define SV2_WIN(i) {						\
	.width = 0, .height = 0, .hoffset = 0, .voffset = 0,	\
	.pclk = SV2_CLK, .bin_factor = 1,			\
	.intg_min = SV2_EXP_MIN << 4,				\
	.intg_max = SV2_EXP_LINES_MAX << 4,			\
	.gain_min = 1 << 4, .gain_max = SV2_GAIN_Q4_MAX_EFF,	\
	.regs = NULL, .regs_size = 0, .set_size = NULL,		\
	.top_clk = 300000000, .isp_clk = 297000000,		\
	.mipi_bps = 891 * 1000 * 1000,				\
}

static struct sensor_win_size sensor_win_sizes[ARRAY_SIZE(sv2_modes)] = {
	SV2_WIN(0), SV2_WIN(1), SV2_WIN(2), SV2_WIN(3),
#if SV2_HAS_BINNING
	SV2_WIN(4), SV2_WIN(5),
#endif
};
#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static struct sv2_mode *sv2_mode_of(struct sensor_win_size *w)
{
	return &sv2_modes[w - sensor_win_sizes];
}

static int sv2_rate_index(void)
{
	return link_mbps >= 1440 ? 2 : link_mbps >= 1188 ? 1 : 0;
}

/*
 * Width of the sensor's crop window. PIX_HWIDTH must be a multiple of 16:
 * with 1916 the IMX585 sends only 1071 of 1080 lines (measured 23/10);
 * with 1920 the VIN drops the extra columns of each line.
 */
static unsigned int sv2_win_width(const struct sv2_mode *m)
{
	return min_t(unsigned int, ALIGN(m->width, 16), SV2_ACTIVE_W);
}

/* Minimum HMAX of a mode: the full-width value (4-lane table, x2 on 2
 * lanes), scaled to the transmitted width, never below the readout limit. */
static unsigned int sv2_mode_hmax(const struct sv2_mode *m)
{
	static const unsigned int hmax_4l[] = SV2_HMAX_4L;
	unsigned int full = hmax_4l[sv2_rate_index()] * (lanes == 4 ? 1 : 2);
	unsigned int h;

	/*
	 * Binned: the sensor still reads the whole array, but averages 2x2 in
	 * the digital domain (measured 2026-10-23: same signal as native, noise
	 * /2) and sends half-width lines, each over two HMAX periods: the link
	 * needs a quarter of the full-width HMAX. IMX585, 891 Mbit/s: HMAX 550
	 * = 55 fps, complete frames down to 500 (61 fps), 450 loses lines.
	 * Other sensors: the native HMAX until their binning is measured.
	 */
	if (m->binned)
		return SV2_FAST_BINNING ? max(DIV_ROUND_UP(full, 4), hmax_min) : full;
	if (m->width >= SV2_ACTIVE_W)
		return full;
	h = DIV_ROUND_UP(full * sv2_win_width(m), SV2_ACTIVE_W);
	return max(h, hmax_min);
}

static unsigned int sv2_mode_vmax(const struct sv2_mode *m)
{
	if (m->binned || m->height >= SV2_ACTIVE_H)
		return SV2_VMAX_FULL;
	return m->height + vmargin;
}

static bool sv2_streaming;
static unsigned int sv2_base_vts;	/* VMAX for the requested frame rate */
static unsigned int sv2_vts;		/* VMAX programmed */
static unsigned int sv2_hmax;		/* HMAX programmed */
static unsigned int sv2_gain_reg;
static bool sv2_hcg;			/* FDG_SEL0 programmed */
static bool sv2_hdr;			/* ClearHDR stream */
static bool sv2_hdr_req;		/* ClearHDR control */
static unsigned int sv2_shr_min = SV2_SHR_MIN;
static int sv2_crop_x, sv2_crop_y;	/* image pixels from the centred window */

static void sv2_update_timing(void)
{
	static const unsigned int mbps[] = {891, 1188, 1440};
	int i;

	link_mbps = mbps[sv2_rate_index()];
	lanes = lanes == 4 ? 4 : 2;
	for (i = 0; i < N_WIN_SIZES; i++) {
		struct sensor_win_size *w = &sensor_win_sizes[i];
		const struct sv2_mode *m = &sv2_modes[i];

		w->width = m->width;
		w->height = m->height;
		w->hts = sv2_mode_hmax(m);
		w->vts = sv2_mode_vmax(m);
		w->mipi_bps = link_mbps * 1000 * 1000;
		w->fps_fixed = SV2_CLK / (w->hts * w->vts);
	}
}

static int sv2_set_uint(const char *val, const struct kernel_param *kp)
{
	int ret;

	if (sv2_streaming)
		return -EBUSY;
	ret = param_set_uint(val, kp);
	if (!ret)
		sv2_update_timing();
	return ret;
}

static const struct kernel_param_ops sv2_timing_ops = {
	.set = sv2_set_uint,
	.get = param_get_uint,
};
module_param_cb(link_mbps, &sv2_timing_ops, &link_mbps, 0664);
MODULE_PARM_DESC(link_mbps, "MIPI rate per lane: 891 (default, validated on the A733 with the IMX327), 1188, 1440; change only while not streaming");
module_param_cb(lanes, &sv2_timing_ops, &lanes, 0664);
MODULE_PARM_DESC(lanes, "MIPI data lanes: 2 (default) or 4 (the A7S connector and the A733 mipi1 port have 4; the 22-pin cable must carry D2/D3); change only while not streaming");
module_param_cb(vmargin, &sv2_timing_ops, &vmargin, 0664);
MODULE_PARM_DESC(vmargin, "Crop modes: VMAX = height + vmargin (default: the full frame's blanking)");
module_param_cb(hmax_min, &sv2_timing_ops, &hmax_min, 0664);
MODULE_PARM_DESC(hmax_min, "Crop and binned modes: lowest HMAX, the readout limit (IMX585: 480, measured; others: 550)");

/* ------------------------------------------------------------------------
 * Exposure (see the header): exp_val in Q4 nominal 29.63 us lines.
 */
static int sv2_write_timing(struct v4l2_subdev *sd, unsigned int hmax,
			    unsigned int vmax, unsigned int shr)
{
	int ret = 0;

	ret |= sensor_write(sd, SV2_REGHOLD, 0x01);
	ret |= sv2_write16(sd, SV2_HMAX, hmax);
	ret |= sv2_write20(sd, SV2_VMAX, vmax);
	ret |= sv2_write20(sd, SV2_SHR0, shr);
	ret |= sensor_write(sd, SV2_REGHOLD, 0x00);
	return ret ? -EIO : 0;
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	*value = to_state(sd)->exp;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	struct sensor_info *info = to_state(sd);
	struct sensor_win_size *wsize = info->current_wins;
	unsigned int base_h = wsize ? wsize->hts : sensor_win_sizes[0].hts;
	unsigned int lines, eff, hmax, vmax;
	int ret;

	if (exp_val > SV2_EXP_LINES_MAX << 4)
		exp_val = SV2_EXP_LINES_MAX << 4;
	lines = DIV_ROUND_CLOSEST_ULL((u64)exp_val * SV2_HMAX_NOMINAL, 16 * base_h);
	if (lines < SV2_EXP_MIN)
		lines = SV2_EXP_MIN;

	if (lines + sv2_shr_min <= SV2_VMAX_MAX) {
		hmax = base_h;
		eff = lines;
		vmax = max(sv2_base_vts, eff + sv2_shr_min);
	} else {
		hmax = DIV_ROUND_UP_ULL((u64)lines * base_h, SV2_VMAX_MAX - sv2_shr_min);
		if (hmax > SV2_HMAX_MAX)
			hmax = SV2_HMAX_MAX;
		eff = DIV_ROUND_CLOSEST_ULL((u64)lines * base_h, hmax);
		if (eff > SV2_VMAX_MAX - sv2_shr_min)
			eff = SV2_VMAX_MAX - sv2_shr_min;
		vmax = eff + sv2_shr_min;
	}

	ret = sv2_write_timing(sd, hmax, vmax, vmax - eff);
	if (ret)
		return ret;
	sv2_hmax = hmax;
	sv2_vts = vmax;
	info->exp = exp_val;
	sensor_dbg("exp q4 %u -> lines %u, hmax %u, vmax %u\n", exp_val, lines, hmax, vmax);
	return 0;
}

/* ------------------------------------------------------------------------
 * Gain: register in 0.3 dB steps, as on the IMX327 (same conversion).
 */
static const u16 sv2_db_steps_q10[20] = {
	1024, 1060, 1097, 1136, 1176, 1217, 1260, 1304, 1350, 1397,
	1446, 1497, 1550, 1604, 1661, 1719, 1780, 1842, 1907, 1974,
};
#define SV2_6DB_Q10		2043

static unsigned int sv2_gain_q4_to_reg(unsigned int gain_q4)
{
	u64 x;
	unsigned int reg = 0;
	int k;

	if (gain_q4 <= 16)
		return 0;
	x = (u64)gain_q4 << 6;
	while (x >= SV2_6DB_Q10 && reg + 20 <= SV2_GAIN_REG_MAX) {
		x = div_u64(x * 1024, SV2_6DB_Q10);
		reg += 20;
	}
	for (k = 19; k > 0; k--)
		if (x >= sv2_db_steps_q10[k])
			break;
	return min_t(unsigned int, reg + k, SV2_GAIN_REG_MAX);
}

static unsigned int sv2_gain_reg_to_q4(unsigned int reg)
{
	u64 x = sv2_db_steps_q10[reg % 20];
	unsigned int n;

	for (n = reg / 20; n; n--)
		x = div_u64(x * SV2_6DB_Q10, 1024);
	return (unsigned int)(x >> 6);
}

/* Gain register and conversion gain, same frame */
static int sv2_write_gain(struct v4l2_subdev *sd, unsigned int reg, bool high)
{
	int ret;

	if (sv2_hdr) {
		high = false;
		reg = min_t(unsigned int, reg, SV2_GAIN_REG_MAX_HDR);
	}
	if (high && reg < SV2_GAIN_REG_MIN_HCG)
		reg = SV2_GAIN_REG_MIN_HCG;
	if (reg > SV2_GAIN_REG_MAX)
		reg = SV2_GAIN_REG_MAX;
	ret = sensor_write(sd, SV2_REGHOLD, 0x01);
	ret |= sensor_write(sd, SV2_FDG_SEL0, high ? 0x01 : 0x00);
	ret |= sv2_write16(sd, SV2_REG_GAIN, reg);
	ret |= sensor_write(sd, SV2_REGHOLD, 0x00);
	if (ret)
		return -EIO;
	sv2_gain_reg = reg;
	sv2_hcg = high;
	return 0;
}

static int sv2_write_gain_reg(struct v4l2_subdev *sd, unsigned int reg)
{
	return sv2_write_gain(sd, reg, sv2_hcg);
}

/* Effective gain (Q4) of the programmed register and conversion gain */
static unsigned int sv2_gain_eff_q4(void)
{
	unsigned int g = sv2_gain_reg_to_q4(sv2_gain_reg);

	if (sv2_hcg && SV2_HCG_FACTOR_MILLI)
		g = (unsigned int)div_u64((u64)g * SV2_HCG_FACTOR_MILLI + 500, 1000);
	return g;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	*value = to_state(sd)->gain;
	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);

	unsigned int g = max(gain_val, 16);
	bool high = !sv2_hdr && (hcg == 1 ||
		    (hcg == 2 && SV2_HCG_FACTOR_MILLI && g >= hcg_gain << 4));
	int ret;

	if (high && SV2_HCG_FACTOR_MILLI)
		g = (unsigned int)div_u64((u64)g * 1000 + SV2_HCG_FACTOR_MILLI / 2,
					  SV2_HCG_FACTOR_MILLI);
	ret = sv2_write_gain(sd, sv2_gain_q4_to_reg(g), high);
	if (!ret)
		info->gain = sv2_gain_eff_q4();
	return ret;
}

static int sensor_s_analogue_gain(struct v4l2_subdev *sd, int reg)
{
	struct sensor_info *info = to_state(sd);
	int ret = sv2_write_gain_reg(sd, reg);

	if (!ret)
		info->gain = sv2_gain_eff_q4();
	return ret;
}

static int sensor_s_exp_gain(struct v4l2_subdev *sd, struct sensor_exp_gain *eg)
{
	int ret = sensor_s_exp(sd, eg->exp_val);

	return ret ? ret : sensor_s_gain(sd, eg->gain_val);
}

static int sensor_s_fps(struct v4l2_subdev *sd, struct sensor_fps *fps)
{
	struct sensor_info *info = to_state(sd);
	struct sensor_win_size *wsize = info->current_wins;
	unsigned int vts;

	if (!wsize || !fps->fps)
		return -EINVAL;
	vts = wsize->pclk / fps->fps / wsize->hts;
	vts = clamp_t(unsigned int, vts, wsize->vts, SV2_VMAX_MAX);
	sv2_base_vts = vts;
	return sensor_s_exp(sd, info->exp);
}

/* ------------------------------------------------------------------------
 * Power, reset, detection. Pi camera modules: own oscillator and
 * regulators switched by the CAM_GPIO line (connector pin 11 = PE8 here,
 * driven as PWDN, active high). Will's drivers wait 500 ms after it.
 */
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	switch (on) {
	case STBY_ON:
		cci_lock(sd);
		sensor_write(sd, SV2_STANDBY, 0x01);
		cci_unlock(sd);
		break;
	case STBY_OFF:
		cci_lock(sd);
		sensor_write(sd, SV2_STANDBY, 0x00);
		cci_unlock(sd);
		break;
	case PWR_ON:
		cci_lock(sd);
		vin_gpio_set_status(sd, PWDN, 1);
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_set_status(sd, POWER_EN, 1);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_HIGH);
		vin_set_pmu_channel(sd, IOVDD, ON);
		usleep_range(10000, 12000);
		vin_set_pmu_channel(sd, DVDD, ON);
		vin_set_pmu_channel(sd, AVDD, ON);
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);
		vin_set_mclk_freq(sd, inck);
		vin_set_mclk(sd, ON);
		msleep(500);
		cci_unlock(sd);
		break;
	case PWR_OFF:
		cci_lock(sd);
		vin_gpio_set_status(sd, PWDN, 1);
		vin_gpio_set_status(sd, RESET, 1);
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		vin_gpio_write(sd, PWDN, CSI_GPIO_LOW);
		vin_set_mclk(sd, OFF);
		vin_set_pmu_channel(sd, AVDD, OFF);
		vin_set_pmu_channel(sd, DVDD, OFF);
		vin_set_pmu_channel(sd, IOVDD, OFF);
		vin_gpio_write(sd, POWER_EN, CSI_GPIO_LOW);
		vin_gpio_set_status(sd, RESET, 0);
		vin_gpio_set_status(sd, PWDN, 0);
		vin_gpio_set_status(sd, POWER_EN, 0);
		cci_unlock(sd);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	vin_gpio_write(sd, RESET, val ? CSI_GPIO_LOW : CSI_GPIO_HIGH);
	usleep_range(1000, 1200);
	return 0;
}

/* No chip ID register (Will's drivers read BLKLEVEL as a presence test). */
static int sensor_detect(struct v4l2_subdev *sd)
{
	data_type lo = 0, hi = 0;

	if (sensor_read(sd, SV2_BLKLEVEL, &lo) || sensor_read(sd, SV2_BLKLEVEL + 1, &hi)) {
		sensor_err("no answer on I2C\n");
		return -ENODEV;
	}
	sensor_print(SENSOR_LABEL " probe: BLKLEVEL = %u\n", (hi << 8) | lo);
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	struct sensor_info *info = to_state(sd);
	int ret = sensor_detect(sd);

	if (ret)
		return ret;
	info->focus_status = 0;
	info->low_speed = 0;
	info->width = sensor_win_sizes[0].width;
	info->height = sensor_win_sizes[0].height;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 16;
	info->exp = 0;
	info->tpf.numerator = 1;
	info->tpf.denominator = sensor_win_sizes[0].fps_fixed;
	info->preview_first_flag = 1;
	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sensor_info *info = to_state(sd);

	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
		if (!info->current_wins)
			return -1;
		memcpy(arg, info->current_wins, sizeof(*info->current_wins));
		return 0;
	case SET_FPS:
		return 0;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		return isp_ae ? sensor_s_exp_gain(sd, (struct sensor_exp_gain *)arg) : 0;
	case VIDIOC_VIN_SENSOR_SET_FPS:
		return sensor_s_fps(sd, (struct sensor_fps *)arg);
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, (struct sensor_config *)arg);
		return 0;
	}
	return -EINVAL;
}

static struct sensor_format_struct sensor_formats[] = {
	{
		.desc = "Raw RGB Bayer",
		.mbus_code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.regs = sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp = 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/* ------------------------------------------------------------------------
 * Stream on/off
 */
/* Window in native pixels: the whole active area when binned. */
static unsigned int sv2_mode_win_w(const struct sv2_mode *m)
{
	return m->binned ? SV2_ACTIVE_W : sv2_win_width(m);
}

static unsigned int sv2_mode_win_h(const struct sv2_mode *m)
{
	return m->binned ? SV2_ACTIVE_H : m->height;
}

/* Start of the window on one axis: centred, plus the crop offset in
 * multiples of 4, clamped to the active area. */
static unsigned int sv2_crop_start(unsigned int origin, unsigned int size,
				   unsigned int active, int off)
{
	int centre = ((active - size) / 2) & ~3U;
	int d = clamp_t(int, off, -centre, (int)(active - size) - centre);

	return origin + centre + (d >= 0 ? d & ~3 : -(-d & ~3));
}

static unsigned int sv2_hst(const struct sv2_mode *m)
{
	return sv2_crop_start(CROP_HST0, sv2_mode_win_w(m), SV2_ACTIVE_W,
			      flip & 1 ? -sv2_crop_x : sv2_crop_x);
}

static unsigned int sv2_vst(const struct sv2_mode *m)
{
	return sv2_crop_start(CROP_VST0, sv2_mode_win_h(m), SV2_ACTIVE_H,
			      flip & 2 ? -sv2_crop_y : sv2_crop_y);
}

/* Move the window while streaming */
static int sv2_write_window_pos(struct v4l2_subdev *sd, const struct sv2_mode *m)
{
	int ret;

	ret = sensor_write(sd, SV2_REGHOLD, 0x01);
	ret |= sv2_write16(sd, SV2_PIX_HST, sv2_hst(m));
	ret |= sv2_write16(sd, SV2_PIX_VST, sv2_vst(m));
	ret |= sensor_write(sd, SV2_REGHOLD, 0x00);
	sensor_dbg("window at %u,%u\n", sv2_hst(m), sv2_vst(m));
	return ret ? -EIO : 0;
}

static int sv2_write_window(struct v4l2_subdev *sd, const struct sv2_mode *m)
{
	data_type winmode = 0;
	int ret;

	ret = sensor_read(sd, SV2_WINMODE, &winmode);
	ret |= sensor_write(sd, SV2_WINMODE, winmode | SV2_WINMODE_CROP);
	ret |= sv2_write16(sd, SV2_PIX_HST, sv2_hst(m));
	ret |= sv2_write16(sd, SV2_PIX_HWIDTH, sv2_mode_win_w(m));
	ret |= sv2_write16(sd, SV2_PIX_VST, sv2_vst(m));
	ret |= sv2_write16(sd, SV2_PIX_VWIDTH, sv2_mode_win_h(m));
	ret |= sensor_write(sd, SV2_ADDMODE, m->binned ? 0x01 : 0x00);
#ifdef STARVIS2_IMX585
	/* imx585.c mode tables: ADBIT/MDBIT 12-bit, clamp start per binning */
	ret |= sensor_write(sd, 0x3022, 0x02);
	ret |= sensor_write(sd, 0x3023, 0x01);
	ret |= sensor_write(sd, SV2_DIG_CLP_VSTART, m->binned ? 0x02 : 0x04);
	ret |= sensor_write(sd, SV2_BIN_MODE, 0x00);
#endif
	return ret ? -EIO : 0;
}

static int sensor_reg_init(struct sensor_info *info)
{
	static const struct { u32 hz; u8 sel; } inck_tab[] = {
		{74250000, 0x00}, {37125000, 0x01}, {72000000, 0x02},
		{27000000, 0x03}, {24000000, 0x04},
	};
	static const u8 datarate[] = {0x05, 0x04, 0x03};	/* 891, 1188, 1440 */
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_win_size *wsize = info->current_wins;
	struct sv2_mode *m = sv2_mode_of(wsize);
	u8 inck_sel = 0x04;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(inck_tab); i++)
		if (inck_tab[i].hz == inck)
			inck_sel = inck_tab[i].sel;

	sensor_write(sd, SV2_STANDBY, 0x01);
	ret = sv2_write_table(sd, starvis2_common_regs,
			      ARRAY_SIZE(starvis2_common_regs), "common");
	if (ret < 0)
		return ret;
	ret = sensor_write(sd, SV2_INCK_SEL, inck_sel);
	ret |= sv2_write16(sd, SV2_BLKLEVEL, black_level);
	ret |= sensor_write(sd, SV2_DATARATE_SEL, datarate[sv2_rate_index()]);
	ret |= sensor_write(sd, SV2_LANEMODE, lanes == 4 ? 0x03 : 0x01);
	/* internal sync leader: XVS/XHS outputs (Will's SYNC_INT_LEADER) */
	ret |= sensor_write(sd, SV2_EXTMODE, 0x00);
	ret |= sensor_write(sd, SV2_XXS_DRV, 0x00);
	ret |= sensor_write(sd, SV2_XXS_OUTSEL, 0x0a);
	ret |= sv2_write_window(sd, m);
	ret |= sensor_write(sd, SV2_WINMODEH, flip & 1);
	ret |= sensor_write(sd, SV2_WINMODEV, (flip >> 1) & 1);
	ret |= sensor_write(sd, SV2_DIGITAL_CLAMP, 0x00);
	if (ret)
		return -EIO;
#if SV2_HAS_CLEARHDR
	sv2_hdr = sv2_hdr_req;
	if (sv2_hdr) {
		ret = sv2_write_table(sd, sv2_clearhdr_regs,
				      ARRAY_SIZE(sv2_clearhdr_regs), "clearhdr");
		ret |= sensor_write(sd, 0x36e2, min(hdr_blend, 8U));	/* EXP_BK */
		ret |= sensor_write(sd, 0x3081, min(hdr_gain, 5U));	/* EXP_GAIN */
		sv2_hcg = false;
	} else {
		ret = sv2_write_table(sd, sv2_normal_regs,
				      ARRAY_SIZE(sv2_normal_regs), "normal");
	}
	if (ret)
		return -EIO;
#endif
	sv2_shr_min = sv2_hdr ? SV2_SHR_MIN_HDR : SV2_SHR_MIN;

	info->width = wsize->width;
	info->height = wsize->height;
	sv2_base_vts = wsize->vts * (sv2_hdr ? 2 : 1);
	sensor_s_exp(sd, info->exp ? info->exp :
		     ((wsize->vts - SV2_SHR_MIN) * wsize->hts / SV2_HMAX_NOMINAL) << 4);
	sv2_write_gain(sd, sv2_gain_reg, sv2_hcg);

	sensor_write(sd, SV2_XMSTA, 0x00);
	sensor_write(sd, SV2_STANDBY, 0x00);
	msleep(25);	/* Will's STREAM_DELAY */

	sensor_print("s_fmt %ux%u%s%s%s, hmax %u, vmax %u, %u lanes x %u Mbit/s, inck %u, window %u,%u, flip %u\n",
		     wsize->width, wsize->height, m->binned ? " binned" : "",
		     m->roi ? " roi" : "", sv2_hdr ? " clear-hdr" : "", sv2_hmax, sv2_vts, lanes, link_mbps, inck,
		     sv2_hst(m), sv2_vst(m), flip & 3);
	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);

	if (!enable) {
		sensor_write(sd, SV2_STANDBY, 0x01);
		sensor_write(sd, SV2_XMSTA, 0x01);
		sv2_streaming = false;
		return 0;
	}
	sv2_streaming = true;
	return sensor_reg_init(info);
}

static int sensor_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = (lanes == 4 ? V4L2_MBUS_CSI2_4_LANE : V4L2_MBUS_CSI2_2_LANE) |
					    V4L2_MBUS_CSI2_CHANNEL_0;
	return 0;
}

/* ------------------------------------------------------------------------
 * Controls (VOLATILE | EXECUTE_ON_WRITE: see imx327_mipi.c)
 */
static int sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info = container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		ctrl->val = sv2_gain_reg;
		return 0;
	case V4L2_CID_SV2_HCG:
		ctrl->val = sv2_hcg;
		return 0;
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info = container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return sensor_s_analogue_gain(sd, ctrl->val);
	case V4L2_CID_SV2_CROP_X:
	case V4L2_CID_SV2_CROP_Y:
		if (ctrl->id == V4L2_CID_SV2_CROP_X)
			sv2_crop_x = ctrl->val;
		else
			sv2_crop_y = ctrl->val;
		if (sv2_streaming && info->current_wins)
			return sv2_write_window_pos(sd, sv2_mode_of(info->current_wins));
		return 0;
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		if (sv2_streaming)
			return -EBUSY;
		sv2_hdr_req = ctrl->val;
		return 0;
	case V4L2_CID_SV2_ROI_WIDTH:
	case V4L2_CID_SV2_ROI_HEIGHT:
		if (sv2_streaming)
			return -EBUSY;
		if (ctrl->id == V4L2_CID_SV2_ROI_WIDTH)
			SV2_ROI_MODE->width = ctrl->val;
		else
			SV2_ROI_MODE->height = ctrl->val;
		sv2_update_timing();
		return 0;
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
};

static const struct v4l2_ctrl_config sv2_ctrl_cfgs[] = {
	{
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_SV2_HCG,
		.name = "HCG Active", .type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0, .max = 1, .step = 1, .def = 0,
		.flags = V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE,
	}, {
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_SV2_CROP_X,
		.name = "Crop X Offset", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = -SV2_ACTIVE_W, .max = SV2_ACTIVE_W, .step = 1, .def = 0,
		.flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	}, {
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_SV2_CROP_Y,
		.name = "Crop Y Offset", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = -SV2_ACTIVE_H, .max = SV2_ACTIVE_H, .step = 1, .def = 0,
		.flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	}, {
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_SV2_ROI_WIDTH,
		.name = "ROI Width", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = SV2_ROI_MIN_W, .max = SV2_ACTIVE_W,
		.step = SV2_ROI_STEP_W, .def = SV2_ROI_DEF_W,
	}, {
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_SV2_ROI_HEIGHT,
		.name = "ROI Height", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = SV2_ROI_MIN_H, .max = SV2_ACTIVE_H,
		.step = SV2_ROI_STEP_H, .def = SV2_ROI_DEF_H,
	},
};

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
#if IS_ENABLED(CONFIG_COMPAT)
	.compat_ioctl32 = sensor_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream = sensor_s_stream,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.enum_mbus_code = sensor_enum_mbus_code,
	.enum_frame_size = sensor_enum_frame_size,
	.enum_frame_interval = sensor_enum_frame_interval,
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
	.get_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};

static struct cci_driver cci_drv = {
	.name = SENSOR_NAME,
	.addr_width = CCI_BITS_16,
	.data_width = CCI_BITS_8,
};

static int sensor_init_controls(struct v4l2_subdev *sd, const struct v4l2_ctrl_ops *ops)
{
	struct sensor_info *info = to_state(sd);
	struct v4l2_ctrl_handler *handler = &info->handler;
	struct v4l2_ctrl *ctrl;
	int i, ret = 0;

	v4l2_ctrl_handler_init(handler, 4 + ARRAY_SIZE(sv2_ctrl_cfgs));
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, SV2_EXP_MIN << 4,
				 SV2_EXP_LINES_MAX << 4, 1, 1000 << 4);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 << 4, SV2_GAIN_Q4_MAX_EFF, 1, 1 << 4);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_ANALOGUE_GAIN, 0, SV2_GAIN_REG_MAX, 1, 0);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	for (i = 0; i < ARRAY_SIZE(sv2_ctrl_cfgs); i++)
		v4l2_ctrl_new_custom(handler, &sv2_ctrl_cfgs[i], NULL);
	if (SV2_HAS_CLEARHDR)
		v4l2_ctrl_new_std(handler, ops, V4L2_CID_WIDE_DYNAMIC_RANGE, 0, 1, 1, 0);
	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
	}
	sd->ctrl_handler = handler;
	return ret;
}

/* ------------------------------------------------------------------------
 * I2C driver
 */
static int sensor_probe(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	sv2_update_timing();
	sd = &info->sd;
	cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv);
	sensor_init_controls(sd, &sensor_ctrl_ops);
	mutex_init(&info->lock);

	info->fmt = &sensor_formats[0];
	info->fmt_pt = &sensor_formats[0];
	info->win_pt = &sensor_win_sizes[0];
	info->fmt_num = N_FMTS;
	info->win_size_num = N_WIN_SIZES;
	info->sensor_field = V4L2_FIELD_NONE;
	info->af_first_flag = 1;
	info->exp = 0;
	info->gain = 16;

	sv2_base_vts = sensor_win_sizes[0].vts;
	sv2_vts = sv2_base_vts;
	sv2_hmax = sensor_win_sizes[0].hts;
	sv2_gain_reg = 0;
	sv2_hcg = false;
	return 0;
}

static void sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = cci_dev_remove_helper(client, &cci_drv);

	kfree(to_state(sd));
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static __init int init_sensor(void)
{
	sv2_update_timing();
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

VIN_INIT_DRIVERS(init_sensor);
module_exit(exit_sensor);
