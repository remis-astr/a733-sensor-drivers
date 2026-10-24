/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Sony IMX327 RAW sensor driver for the Allwinner VIN framework
 * (Radxa Cubie A7S, A733 / sun60iw2, BSP kernel 6.6.98-aw2511).
 *
 * Written 2026-10-10 without hardware, validated on the board 2026-10-16
 * with the Innomaker IMX327 module: 1080p 30 fps, 540p 56 fps, 480p 62 fps,
 * exposures 0.1 s to 115 s. Findings in the A7S project's NOTES.md.
 *
 * Sources this file is assembled from:
 *  - Framework / 6.6 API (probe/remove signatures, get_mbus_config pad op,
 *    control handler): Radxa's imx219.c, allwinner-bsp cubie-aiot-v1.5.0.
 *  - Register sequence, IMX327 init block, 12-bit settings, INCK tables,
 *    exposure/VMAX relation, black level, test pattern: mainline imx290.c as
 *    patched for Multicam/Minicam (usr/src/imx290-crop), including the two
 *    "planetary" 1920x540 / 1920x480 VMAX-height crop modes.
 *  - 2-lane 445.5 Mbit/s CSI timing, HMAX 4400 and FRSEL=30fps:
 *    Allwinner's imx307_mipi.c (same register family), whose 2-lane mode
 *    targets this VIN (default); mainline's 891 Mbit/s 60 fps 2-lane timing
 *    (HMAX 2200, as Minicam on the Pi) behind the mipi_891 parameter.
 *
 * Output: MEDIA_BUS_FMT_SRGGB12_1X12 (same Bayer label as Minicam's IMX327
 * profile), 2 data lanes, virtual channel 0.
 *
 * Exposure follows the VIN convention (lines, Q4: 16 = one line of 29.63 us).
 * Exposures longer than the current frame automatically stretch VMAX, then
 * HMAX beyond ~7.7 s, so a long pose needs no separate frame-rate change;
 * ceiling ~115.7 s (VMAX 0x3ffff x HMAX 0xffff), as on the Pi.
 *
 * Gain: two controls write the same register (0x3014, 0.3 dB/step):
 *  - V4L2_CID_ANALOGUE_GAIN: raw register value 0..230, for RAW/astro use
 *    (0..90 = analog up to 27 dB, above that the sensor adds digital gain);
 *  - V4L2_CID_GAIN and VIDIOC_VIN_SENSOR_EXP_GAIN: VIN/ISP convention,
 *    linear Q4 (16 = 1x), converted to the nearest 0.3 dB step.
 * The last one written wins.
 *
 * TODO before/at bring-up:
 *  - INCK: Innomaker documents 74.25 MHz (default here), 37.125 MHz optional;
 *  - the IMX290 family has no ID register: 0x319a/0x319b is only a probe read;
 *  - check if the VIN needs time_hs / combo_mode for 445.5 Mbit/s;
 *  - check hoffset/voffset: this driver outputs exactly WxH (mainline style,
 *    X_OUT_SIZE/Y_OUT_SIZE), Allwinner's imx307 outputs 1948x1097 and crops;
 *  - flips (0x3007 bits 0/1) not exposed yet: check Bayer order after flip;
 *  - check that the VIN tolerates HMAX changes while streaming (long poses).
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
MODULE_DESCRIPTION("A low-level driver for IMX327 sensors (RAW, Allwinner VIN)");
MODULE_LICENSE("GPL");

/*
 * The same source builds imx462_mipi.ko (imx462_mipi.c defines IMX462): the
 * IMX462 shares the IMX290 register map, timing and modes, and differs only
 * by its global init block and a 72 dB gain ceiling (Pi driver: models
 * IMX327LQR / IMX462LQR of imx290.c).
 */
#ifdef IMX462
#define SENSOR_NAME		"imx462_mipi"
#define V4L2_IDENT_SENSOR	0x0462
#define SENSOR_LABEL		"IMX462"
#else
#define SENSOR_NAME		"imx327_mipi"
#define V4L2_IDENT_SENSOR	0x0327
#define SENSOR_LABEL		"IMX327"
#endif

/* ------------------------------------------------------------------------
 * Registers (names from mainline imx290.c)
 */
#define IMX327_STANDBY		0x3000
#define IMX327_REGHOLD		0x3001
#define IMX327_XMSTA		0x3002
#define IMX327_SW_RESET		0x3003
#define IMX327_CTRL_07		0x3007	/* WINMODE[6:4], HREV[1], VREV[0] */
#define IMX327_WINMODE_1080P	(0 << 4)
#define IMX327_WINMODE_CROP	(4 << 4)
#define IMX327_WINPV		0x303c	/* 16 bits LE: first row of the window */
#define IMX327_WINWV		0x303e	/* 16 bits LE: rows of the window */
#define IMX327_Y_OUT_SIZE	0x3418	/* 16 bits LE */
#define IMX327_FR_FDG_SEL	0x3009	/* FRSEL[1:0], FDG_SEL (HCG)[4] */
#define IMX327_FRSEL_30FPS	0x02
#define IMX327_FRSEL_60FPS	0x01
#define IMX327_FDG_HCG		BIT(4)
#define IMX327_BLKLEVEL_L	0x300a
#define IMX327_BLKLEVEL_H	0x300b
#define IMX327_GAIN		0x3014
#define IMX327_VMAX_L		0x3018
#define IMX327_VMAX_M		0x3019
#define IMX327_VMAX_H		0x301a
#define IMX327_VMAX_MAX		0x3ffff
#define IMX327_HMAX_L		0x301c
#define IMX327_HMAX_H		0x301d
#define IMX327_SHS1_L		0x3020
#define IMX327_SHS1_M		0x3021
#define IMX327_SHS1_H		0x3022
#define IMX327_PGCTRL		0x308c
#define IMX327_PGCTRL_REGEN	BIT(0)
#define IMX327_PGCTRL_THRU	BIT(1)
#define IMX327_PGCTRL_MODE(n)	((n) << 4)
#define IMX327_CHIP_ID_L	0x319a
#define IMX327_CHIP_ID_H	0x319b

#define IMX327_EXPOSURE_OFFSET	2	/* exposure <= VMAX - 2, SHS1 >= 1 */
/*
 * IMX327LQR (Innomaker manual V1.3 / Sony): analog 0..27 dB (reg 0..90),
 * then digital up to 69 dB total (reg 230), 0.3 dB/step. 240 = 72 dB is the
 * IMX290's ceiling, not this sensor's.
 */
#ifdef IMX462
#define IMX327_GAIN_REG_MAX	240
#define IMX327_GAIN_Q4_MAX	(3981 << 4)	/* 10^(72/20) = 3981x */
#else
#define IMX327_GAIN_REG_MAX	230
#define IMX327_GAIN_Q4_MAX	(2818 << 4)	/* 10^(69/20) = 2818x */
#endif
#define IMX327_HMAX_1080P_2LANE	4400	/* 30 fps at 445.5 Mbit/s/lane */
#define IMX327_HMAX_1080P_891	2200	/* 60 fps at 891 Mbit/s/lane */
#define IMX327_HMAX_MAX		0xffff
/* Longest pose in nominal (HMAX 4400) lines: 262141 * 65535 / 4400. */
#define IMX327_EXP_LINES_MAX	3904411
#define IMX327_PIXEL_RATE	148500000

/*
 * Movable band (2026-10-21): in window cropping mode (WINMODE 4) WINPV moves
 * the rows read, counted like the rows of the 1080p image (measured: WINPV
 * 270 + Y_OUT_SIZE 540 = rows 270-809 of the 1080p frame), and it is taken
 * while streaming. WINWV = output height + 13 (value that worked). Same ids
 * as the IMX477 driver of the A7S and Rpicamera3's IMX585: "Crop Y Offset"
 * (rows from the centred position, positive = down, steps of 2 for the
 * Bayer phase) and "ROI Height" (free-height band, last mode, set while not
 * streaming; VMAX = height + 60 as Minicam's vmax_min, hence the rate).
 * Horizontal crops stay in software: a line lasts HMAX whatever its width.
 */
#define V4L2_CID_IMX327_BASE		(V4L2_CID_USER_BASE + 0x2000)
#define V4L2_CID_IMX327_CROP_Y		(V4L2_CID_IMX327_BASE + 11)
#define V4L2_CID_IMX327_ROI_HEIGHT	(V4L2_CID_IMX327_BASE + 13)
#define IMX327_ROWS		1080	/* rows WINPV can reach (1080p image) */
#define IMX327_WINWV_MARGIN	13
#define IMX327_VBLANK_BAND	60
#define IMX327_ROI_MIN_H	64
#define IMX327_ROI_STEP_H	4
#define IMX327_ROI_DEF_H	300

/* ------------------------------------------------------------------------
 * Module parameters
 */
static uint inck = 74250000;
module_param(inck, uint, 0444);
MODULE_PARM_DESC(inck, "Module oscillator / MCLK in Hz: 37125000 or 74250000 (default, as on Minicam)");

static bool hcg_mode;
module_param(hcg_mode, bool, 0664);
MODULE_PARM_DESC(hcg_mode, "Enable HCG (high conversion gain) mode, as in Minicam's imx290 driver");

static uint black_level = 240;
module_param(black_level, uint, 0664);
MODULE_PARM_DESC(black_level, "Black level in 12-bit units (default 240 = 3840 in 16-bit, as on Minicam)");

static bool isp_ae = false;
module_param(isp_ae, bool, 0664);
MODULE_PARM_DESC(isp_ae, "Accept exposure/gain requests from the ISP 3A (VIDIOC_VIN_SENSOR_EXP_GAIN); off = manual RAW control only");

static bool mipi_891;	/* set through mipi_891_ops, after the mode table */

static uint test_pattern;
module_param(test_pattern, uint, 0664);
MODULE_PARM_DESC(test_pattern, "Sensor test pattern applied at stream on: 0 = off, 1..7 = PGCTRL mode");

/* ------------------------------------------------------------------------
 * Register tables
 */

/* Common to every mode: analog window, sync outputs (imx290_global_init_settings). */
static struct regval_list sensor_default_regs[] = {
	{IMX327_STANDBY, 0x01},
	{IMX327_XMSTA, 0x01},
	{0x303a, 12},			/* WINWV_OB */
	{0x3040, 0x00}, {0x3041, 0x00},	/* WINPH = 0 */
	{0x303c, 0x00}, {0x303d, 0x00},	/* WINPV = 0 */
	{0x3042, 0x9c}, {0x3043, 0x07},	/* WINWH = 1948 */
	{0x303e, 0x49}, {0x303f, 0x04},	/* WINWV = 1097 */
	{0x304b, 0x0a},			/* XVS/XHS outputs = VSYNC/HSYNC */
	{0x3012, 0x64},
	{0x3013, 0x00},

#ifdef IMX462
	/* IMX462 specific (imx290_global_init_settings_462) */
	{0x300f, 0x00}, {0x3010, 0x21}, {0x3011, 0x02}, {0x3016, 0x09},
	{0x3070, 0x02}, {0x3071, 0x11}, {0x309b, 0x10}, {0x309c, 0x22},
	{0x30a2, 0x02}, {0x30a6, 0x20}, {0x30a8, 0x20}, {0x30aa, 0x20},
	{0x30ac, 0x20}, {0x30b0, 0x43}, {0x3119, 0x9e}, {0x311c, 0x1e},
	{0x311e, 0x08}, {0x3128, 0x05}, {0x313d, 0x83}, {0x3150, 0x03},
	{0x317e, 0x00}, {0x32b8, 0x50}, {0x32b9, 0x10}, {0x32ba, 0x00},
	{0x32bb, 0x04}, {0x32c8, 0x50}, {0x32c9, 0x10}, {0x32ca, 0x00},
	{0x32cb, 0x04}, {0x332c, 0xd3}, {0x332d, 0x10}, {0x332e, 0x0d},
	{0x3358, 0x06}, {0x3359, 0xe1}, {0x335a, 0x11}, {0x3360, 0x1e},
	{0x3361, 0x61}, {0x3362, 0x10}, {0x33b0, 0x50}, {0x33b2, 0x1a},
	{0x33b3, 0x04},
#else
	/* IMX327 specific (imx290_global_init_settings_327) */
	{0x3011, 0x02},
	{0x309e, 0x4a},
	{0x309f, 0x4a},
	{0x313b, 0x61},
#endif

	/* 12-bit A/D and output (imx290_12bit_settings) */
	{0x3005, 0x01},			/* ADBIT */
	{0x3046, 0x01},			/* ODBIT, CSI-2 */
	{0x3129, 0x00},			/* ADBIT1 */
	{0x317c, 0x00},			/* ADBIT2 */
	{0x31ec, 0x0e},			/* ADBIT3 */
	{0x3441, 0x0c}, {0x3442, 0x0c},	/* CSI_DT_FMT = RAW12 */

	/* 2 data lanes */
	{0x3407, 0x01},			/* PHY_LANE_NUM = lanes - 1 */
	{0x3443, 0x01},			/* CSI_LANE_MODE = lanes - 1 */
};

/*
 * CSI-2 timing for 222.75 MHz DDR = 445.5 Mbit/s per lane
 * (imx290_csi_222_75mhz, identical to Allwinner's imx307 2-lane table).
 */
static struct regval_list sensor_csi_445_regs[] = {
	{0x3405, 0x10},			/* REPETITION */
	{0x3446, 87}, {0x3447, 0x00},	/* TCLKPOST */
	{0x3448, 55}, {0x3449, 0x00},	/* THSZERO */
	{0x344a, 31}, {0x344b, 0x00},	/* THSPREPARE */
	{0x344c, 31}, {0x344d, 0x00},	/* TCLKTRAIL */
	{0x344e, 31}, {0x344f, 0x00},	/* THSTRAIL */
	{0x3450, 119}, {0x3451, 0x00},	/* TCLKZERO */
	{0x3452, 31}, {0x3453, 0x00},	/* TCLKPREPARE */
	{0x3454, 23}, {0x3455, 0x00},	/* TLPX */
};

/* 445.5 MHz DDR = 891 Mbit/s per lane (imx290_csi_445_5mhz, as on the Pi). */
static struct regval_list sensor_csi_891_regs[] = {
	{0x3405, 0x00},			/* REPETITION */
	{0x3446, 119}, {0x3447, 0x00},	/* TCLKPOST */
	{0x3448, 103}, {0x3449, 0x00},	/* THSZERO */
	{0x344a, 71}, {0x344b, 0x00},	/* THSPREPARE */
	{0x344c, 55}, {0x344d, 0x00},	/* TCLKTRAIL */
	{0x344e, 63}, {0x344f, 0x00},	/* THSTRAIL */
	{0x3450, 255}, {0x3451, 0x00},	/* TCLKZERO */
	{0x3452, 63}, {0x3453, 0x00},	/* TCLKPREPARE */
	{0x3454, 55}, {0x3455, 0x00},	/* TLPX */
};

/* INCK-dependent registers, 1080p readout family (imx290_1080p_clock_config + xclk_regs). */
static struct regval_list sensor_inck_37m125_regs[] = {
	{0x305c, 0x18},			/* INCKSEL1 */
	{0x305d, 0x03},			/* INCKSEL2 */
	{0x305e, 0x20},			/* INCKSEL3 */
	{0x305f, 0x01},			/* INCKSEL4 */
	{0x315e, 0x1a},			/* INCKSEL5 */
	{0x3164, 0x1a},			/* INCKSEL6 */
	{0x3444, 0x20}, {0x3445, 0x25},	/* EXTCK_FREQ = 37.125 * 256 */
	{0x3480, 0x49},			/* INCKSEL7 */
};

static struct regval_list sensor_inck_74m25_regs[] = {
	{0x305c, 0x0c},
	{0x305d, 0x03},
	{0x305e, 0x10},
	{0x305f, 0x01},
	{0x315e, 0x1b},
	{0x3164, 0x1b},
	{0x3444, 0x40}, {0x3445, 0x4a},	/* EXTCK_FREQ = 74.25 * 256 */
	{0x3480, 0x92},
};

/*
 * Per-mode output size. The 540/480 modes (and the free-height band) reduce
 * Y_OUT_SIZE and VMAX like Minicam's "planetary" crop modes, and are read in
 * window cropping mode so the band can be placed (imx327_write_window; the
 * Minicam comment calling WINMODE_CROP a no-op was wrong for WINPV) — width
 * is cropped in software by the application.
 */
static struct regval_list sensor_1080p_regs[] = {
	{0x303a, 12},			/* WINWV_OB */
	{0x3414, 10},			/* OPB_SIZE_V */
	{0x3472, 0x80}, {0x3473, 0x07},	/* X_OUT_SIZE = 1920 */
	{0x3418, 0x38}, {0x3419, 0x04},	/* Y_OUT_SIZE = 1080 */
};

static struct regval_list sensor_h540_regs[] = {
	{0x303a, 12},
	{0x3414, 10},
	{0x3472, 0x80}, {0x3473, 0x07},	/* X_OUT_SIZE = 1920 */
	{0x3418, 0x1c}, {0x3419, 0x02},	/* Y_OUT_SIZE = 540 */
};

static struct regval_list sensor_h480_regs[] = {
	{0x303a, 12},
	{0x3414, 10},
	{0x3472, 0x80}, {0x3473, 0x07},	/* X_OUT_SIZE = 1920 */
	{0x3418, 0xe0}, {0x3419, 0x01},	/* Y_OUT_SIZE = 480 */
};

/* Video format changes are all in the tables above. */
static struct regval_list sensor_fmt_raw[] = {
};

/*
 * Register table writer, after itssme's IMX477 bring-up on the same board:
 * one write at a time with a short pause (the A733 TWI controller overruns
 * on long tables) and the failing register reported.
 */
static int imx327_write_table(struct v4l2_subdev *sd, struct regval_list *regs,
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

/* ------------------------------------------------------------------------
 * State. One sensor per board, like the other VIN sensor drivers.
 */
static unsigned int imx327_base_vts;	/* VMAX for the requested frame rate */
static unsigned int imx327_vts;		/* VMAX currently programmed */
static unsigned int imx327_gain_reg;

static unsigned int imx327_hmax = IMX327_HMAX_1080P_2LANE; /* HMAX currently programmed */

static int imx327_write_timing(struct v4l2_subdev *sd, unsigned int hmax,
			       unsigned int vmax, unsigned int shs1)
{
	int ret = 0;

	ret |= sensor_write(sd, IMX327_REGHOLD, 0x01);
	ret |= sensor_write(sd, IMX327_HMAX_L, hmax & 0xff);
	ret |= sensor_write(sd, IMX327_HMAX_H, (hmax >> 8) & 0xff);
	ret |= sensor_write(sd, IMX327_VMAX_L, vmax & 0xff);
	ret |= sensor_write(sd, IMX327_VMAX_M, (vmax >> 8) & 0xff);
	ret |= sensor_write(sd, IMX327_VMAX_H, (vmax >> 16) & 0x03);
	ret |= sensor_write(sd, IMX327_SHS1_L, shs1 & 0xff);
	ret |= sensor_write(sd, IMX327_SHS1_M, (shs1 >> 8) & 0xff);
	ret |= sensor_write(sd, IMX327_SHS1_H, (shs1 >> 16) & 0x03);
	ret |= sensor_write(sd, IMX327_REGHOLD, 0x00);

	return ret ? -EIO : 0;
}

/* ------------------------------------------------------------------------
 * Exposure: exp_val in lines, Q4, where a "line" is always one line of the
 * 445.5 Mbit/s timing (HMAX 4400 = 29.63 us), whatever HMAX is in use: at
 * 891 Mbit/s (HMAX 2200) the pose is converted to twice as many sensor lines,
 * so rawcap and Multicam need not know the link rate.
 *
 * Up to VMAX_MAX lines the frame is simply stretched (VMAX). Beyond that
 * (~7.7 s) the line itself is lengthened: HMAX grows (up to 0xffff) and the
 * pose is re-expressed in those longer lines, quantised to one long line
 * (<= 441 us). Ceiling: 262141 lines x 441 us = ~115.7 s, the same limit
 * libcamera reaches with this sensor on the Pi.
 */
static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->exp;
	sensor_dbg("sensor_get_exposure = %d\n", info->exp);
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	struct sensor_info *info = to_state(sd);
	struct sensor_win_size *wsize = info->current_wins;
	unsigned int base_h = wsize ? wsize->hts : IMX327_HMAX_1080P_2LANE;
	unsigned int lines, eff, hmax, vmax, shs1;
	u64 max_lines;
	int ret;

	if (exp_val > IMX327_EXP_LINES_MAX << 4)
		exp_val = IMX327_EXP_LINES_MAX << 4;
	max_lines = div_u64((u64)(IMX327_VMAX_MAX - IMX327_EXPOSURE_OFFSET) *
			    IMX327_HMAX_MAX, base_h);
	/* Nominal Q4 lines -> lines of base_h. */
	lines = DIV_ROUND_CLOSEST_ULL((u64)exp_val * IMX327_HMAX_1080P_2LANE,
				      16 * base_h);
	if (lines < 1)
		lines = 1;
	if (lines > max_lines)
		lines = max_lines;

	if (lines + IMX327_EXPOSURE_OFFSET <= IMX327_VMAX_MAX) {
		/* Normal and long-frame range: nominal line, frame stretched. */
		hmax = base_h;
		eff = lines;
		vmax = max(imx327_base_vts, eff + IMX327_EXPOSURE_OFFSET);
	} else {
		/* Very long pose: lengthen the line so VMAX fits in 18 bits. */
		hmax = DIV_ROUND_UP_ULL((u64)lines * base_h,
					IMX327_VMAX_MAX - IMX327_EXPOSURE_OFFSET);
		if (hmax > IMX327_HMAX_MAX)
			hmax = IMX327_HMAX_MAX;
		eff = DIV_ROUND_CLOSEST_ULL((u64)lines * base_h, hmax);
		if (eff > IMX327_VMAX_MAX - IMX327_EXPOSURE_OFFSET)
			eff = IMX327_VMAX_MAX - IMX327_EXPOSURE_OFFSET;
		vmax = eff + IMX327_EXPOSURE_OFFSET;
	}

	shs1 = vmax - eff - 1;

	ret = imx327_write_timing(sd, hmax, vmax, shs1);
	if (ret)
		return ret;

	imx327_hmax = hmax;
	imx327_vts = vmax;
	info->exp = exp_val;
	sensor_dbg("sensor_set_exp q4 = %u, lines = %u -> hmax = %u, vmax = %u, shs1 = %u\n",
		   exp_val, lines, hmax, vmax, shs1);
	return 0;
}

/* ------------------------------------------------------------------------
 * Gain
 */

/* round(1024 * 10^(0.015 k)) for k = 0..19: one 6 dB span in 0.3 dB steps. */
static const u16 imx327_db_steps_q10[20] = {
	1024, 1060, 1097, 1136, 1176, 1217, 1260, 1304, 1350, 1397,
	1446, 1497, 1550, 1604, 1661, 1719, 1780, 1842, 1907, 1974,
};
#define IMX327_6DB_Q10		2043	/* round(1024 * 10^0.3) */

/* Linear gain Q4 (16 = 1x) -> register value (0.3 dB steps), rounded down. */
static unsigned int imx327_gain_q4_to_reg(unsigned int gain_q4)
{
	u64 x;
	unsigned int reg = 0;
	int k;

	if (gain_q4 <= 16)
		return 0;

	x = (u64)gain_q4 << 6;		/* Q4 -> Q10 */
	while (x >= IMX327_6DB_Q10 && reg + 20 <= IMX327_GAIN_REG_MAX) {
		x = div_u64(x * 1024, IMX327_6DB_Q10);
		reg += 20;
	}
	for (k = 19; k > 0; k--)
		if (x >= imx327_db_steps_q10[k])
			break;
	reg += k;

	return min_t(unsigned int, reg, IMX327_GAIN_REG_MAX);
}

/* Register value -> linear gain Q4, for reporting. */
static unsigned int imx327_gain_reg_to_q4(unsigned int reg)
{
	u64 x = imx327_db_steps_q10[reg % 20];
	unsigned int n;

	for (n = reg / 20; n; n--)
		x = div_u64(x * IMX327_6DB_Q10, 1024);

	return (unsigned int)(x >> 6);
}

static int imx327_write_gain_reg(struct v4l2_subdev *sd, unsigned int reg)
{
	int ret;

	if (reg > IMX327_GAIN_REG_MAX)
		reg = IMX327_GAIN_REG_MAX;
	ret = sensor_write(sd, IMX327_GAIN, reg);
	if (ret)
		return ret;
	imx327_gain_reg = reg;
	return 0;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);

	*value = info->gain;
	sensor_dbg("sensor_get_gain = %d\n", info->gain);
	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);
	int ret;

	if (gain_val < 16)
		gain_val = 16;
	ret = imx327_write_gain_reg(sd, imx327_gain_q4_to_reg(gain_val));
	if (ret)
		return ret;
	info->gain = imx327_gain_reg_to_q4(imx327_gain_reg);
	sensor_dbg("sensor_set_gain q4 = %d -> reg %u\n", gain_val, imx327_gain_reg);
	return 0;
}

static int sensor_s_analogue_gain(struct v4l2_subdev *sd, int reg)
{
	struct sensor_info *info = to_state(sd);
	int ret;

	ret = imx327_write_gain_reg(sd, reg);
	if (ret)
		return ret;
	info->gain = imx327_gain_reg_to_q4(imx327_gain_reg);
	return 0;
}

static int sensor_s_exp_gain(struct v4l2_subdev *sd,
			     struct sensor_exp_gain *exp_gain)
{
	int ret;

	ret = sensor_s_exp(sd, exp_gain->exp_val);
	if (ret)
		return ret;
	return sensor_s_gain(sd, exp_gain->gain_val);
}

/* Frame rate: sets the base VMAX; a longer exposure still stretches it. */
static int sensor_s_fps(struct v4l2_subdev *sd, struct sensor_fps *fps)
{
	struct sensor_info *info = to_state(sd);
	struct sensor_win_size *wsize = info->current_wins;
	unsigned int vts;

	if (!wsize || !fps->fps)
		return -EINVAL;

	vts = wsize->pclk / fps->fps / wsize->hts;
	if (vts < wsize->vts)
		vts = wsize->vts;
	if (vts > IMX327_VMAX_MAX)
		vts = IMX327_VMAX_MAX;
	imx327_base_vts = vts;

	/* Re-apply the current pose against the new frame length. */
	return sensor_s_exp(sd, info->exp);
}

/* ------------------------------------------------------------------------
 * Power, reset, detection
 *
 * Pi-style IMX327 modules (Innomaker, Minicam's) carry their own oscillator
 * and regulators switched by one enable line; MCLK from the SoC is still
 * enabled at the configured INCK in case the board routes it.
 */
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	switch (on) {
	case STBY_ON:
		sensor_dbg("STBY_ON!\n");
		cci_lock(sd);
		sensor_write(sd, IMX327_STANDBY, 0x01);
		usleep_range(1000, 1200);
		cci_unlock(sd);
		break;
	case STBY_OFF:
		sensor_dbg("STBY_OFF!\n");
		cci_lock(sd);
		usleep_range(1000, 1200);
		sensor_write(sd, IMX327_STANDBY, 0x00);
		cci_unlock(sd);
		break;
	case PWR_ON:
		sensor_dbg("PWR_ON!\n");
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
		usleep_range(1000, 1200);
		vin_set_mclk_freq(sd, inck);
		vin_set_mclk(sd, ON);
		/* XCLR high to first I2C access: >= 20 us; settle generously. */
		usleep_range(20000, 22000);
		cci_unlock(sd);
		break;
	case PWR_OFF:
		sensor_dbg("PWR_OFF!\n");
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
	switch (val) {
	case 0:
		vin_gpio_write(sd, RESET, CSI_GPIO_HIGH);
		usleep_range(1000, 1200);
		break;
	case 1:
		vin_gpio_write(sd, RESET, CSI_GPIO_LOW);
		usleep_range(1000, 1200);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * The IMX290 family has no documented ID register in the public tables;
 * 0x319a/0x319b is used here as a probe read. Step 2 of the roadmap records
 * the value read on the real module before this check is tightened.
 */
static int sensor_detect(struct v4l2_subdev *sd)
{
	data_type id_l = 0, id_h = 0;
	int ret;

	ret = sensor_read(sd, IMX327_CHIP_ID_L, &id_l);
	ret |= sensor_read(sd, IMX327_CHIP_ID_H, &id_h);
	if (ret) {
		sensor_err("no answer on I2C\n");
		return -ENODEV;
	}
	sensor_print(SENSOR_LABEL " probe: reg 0x319a/0x319b = 0x%02x%02x\n", id_h, id_l);
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	struct sensor_info *info = to_state(sd);
	int ret;

	sensor_dbg("sensor_init\n");

	ret = sensor_detect(sd);
	if (ret) {
		sensor_err("chip found is not an target chip.\n");
		return ret;
	}

	info->focus_status = 0;
	info->low_speed = 0;
	info->width = HD1080_WIDTH;
	info->height = HD1080_HEIGHT;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 16;
	info->exp = 0;

	info->tpf.numerator = 1;
	info->tpf.denominator = 30;	/* 30fps */

	info->preview_first_flag = 1;

	return 0;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct sensor_info *info = to_state(sd);

	switch (cmd) {
	case GET_CURRENT_WIN_CFG:
		if (info->current_wins != NULL) {
			memcpy(arg, info->current_wins,
			       sizeof(*info->current_wins));
			ret = 0;
		} else {
			sensor_err("empty wins!\n");
			ret = -1;
		}
		break;
	case SET_FPS:
		break;
	case VIDIOC_VIN_SENSOR_EXP_GAIN:
		/*
		 * In RAW use the pose and gain are set by the application; ISP
		 * 3A requests would override them (and, on itssme's IMX477,
		 * two ISP instances updating the sensor caused TWI failures).
		 */
		if (isp_ae)
			ret = sensor_s_exp_gain(sd, (struct sensor_exp_gain *)arg);
		break;
	case VIDIOC_VIN_SENSOR_SET_FPS:
		ret = sensor_s_fps(sd, (struct sensor_fps *)arg);
		break;
	case VIDIOC_VIN_SENSOR_CFG_REQ:
		sensor_cfg_req(sd, (struct sensor_config *)arg);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

/* ------------------------------------------------------------------------
 * Formats and window sizes
 */
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

/*
 * pclk / (hts * vts) = fps. HMAX 4400 at 445.5 Mbit/s per lane on 2 lanes:
 * one 1920-pixel RAW12 line needs 25.9 us on the link, a line lasts 29.6 us.
 * vts for 540/480 = Minicam's vmax_min (height + 60).
 */
static struct sensor_win_size sensor_win_sizes[] = {
	/* 1920x1080 @ 30 fps */
	{
		.width = 1920,
		.height = 1080,
		.hoffset = 0,
		.voffset = 0,
		.hts = IMX327_HMAX_1080P_2LANE,
		.vts = 1125,
		.pclk = IMX327_PIXEL_RATE,
		.mipi_bps = 446 * 1000 * 1000,
		.fps_fixed = 30,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = IMX327_EXP_LINES_MAX << 4,
		.gain_min = 1 << 4,
		.gain_max = IMX327_GAIN_Q4_MAX,
		.regs = sensor_1080p_regs,
		.regs_size = ARRAY_SIZE(sensor_1080p_regs),
		.set_size = NULL,
		.top_clk = 300000000,
		.isp_clk = 297000000,
	},
	/* 1920x540 @ 56.25 fps (Minicam "540p", app crops width) */
	{
		.width = 1920,
		.height = 540,
		.hoffset = 0,
		.voffset = 0,
		.hts = IMX327_HMAX_1080P_2LANE,
		.vts = 600,
		.pclk = IMX327_PIXEL_RATE,
		.mipi_bps = 446 * 1000 * 1000,
		.fps_fixed = 56,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = IMX327_EXP_LINES_MAX << 4,
		.gain_min = 1 << 4,
		.gain_max = IMX327_GAIN_Q4_MAX,
		.regs = sensor_h540_regs,
		.regs_size = ARRAY_SIZE(sensor_h540_regs),
		.set_size = NULL,
		.top_clk = 300000000,
		.isp_clk = 297000000,
	},
	/* 1920x480 @ 62.5 fps (Minicam "480p", app crops width) */
	{
		.width = 1920,
		.height = 480,
		.hoffset = 0,
		.voffset = 0,
		.hts = IMX327_HMAX_1080P_2LANE,
		.vts = 540,
		.pclk = IMX327_PIXEL_RATE,
		.mipi_bps = 446 * 1000 * 1000,
		.fps_fixed = 62,
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = IMX327_EXP_LINES_MAX << 4,
		.gain_min = 1 << 4,
		.gain_max = IMX327_GAIN_Q4_MAX,
		.regs = sensor_h480_regs,
		.regs_size = ARRAY_SIZE(sensor_h480_regs),
		.set_size = NULL,
		.top_clk = 300000000,
		.isp_clk = 297000000,
	},
	/* 1920 x free height ("ROI Height"), last: Y_OUT_SIZE written at stream on */
	{
		.width = 1920,
		.height = IMX327_ROI_DEF_H,
		.hoffset = 0,
		.voffset = 0,
		.hts = IMX327_HMAX_1080P_2LANE,
		.vts = IMX327_ROI_DEF_H + IMX327_VBLANK_BAND,
		.pclk = IMX327_PIXEL_RATE,
		.mipi_bps = 446 * 1000 * 1000,
		.fps_fixed = IMX327_PIXEL_RATE / (IMX327_HMAX_1080P_2LANE * (IMX327_ROI_DEF_H + IMX327_VBLANK_BAND)),
		.bin_factor = 1,
		.intg_min = 1 << 4,
		.intg_max = IMX327_EXP_LINES_MAX << 4,
		.gain_min = 1 << 4,
		.gain_max = IMX327_GAIN_Q4_MAX,
		.regs = sensor_h540_regs,
		.regs_size = ARRAY_SIZE(sensor_h540_regs),
		.set_size = NULL,
		.top_clk = 300000000,
		.isp_clk = 297000000,
	},
};
#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))
#define IMX327_ROI_WIN (&sensor_win_sizes[N_WIN_SIZES - 1])

/*
 * mipi_891: 891 Mbit/s per lane and HMAX 2200 (the Pi's timing), doubling
 * every frame rate (1080p 60, 540p 112, 480p 125 fps). Off by default.
 * Writable while no stream runs: the mode table (hts, mipi_bps, fps_fixed)
 * is patched at once, so the VIN (TDM buffers sized from fps_fixed) and
 * S_PARM see the new timing at the next stream on; no module reload needed.
 */
static bool imx327_streaming;
static int imx327_crop_y;	/* rows from the centred band position */

/*
 * Bands (height < 1080): window cropping mode, rows placed by WINPV. While
 * streaming only WINPV may be rewritten: rewriting WINMODE/WINWV there too
 * left the electronic shutter on the previous window from the second move on
 * (rows outside it never reset, overexposed bands; measured 2026-10-21).
 */
static int imx327_write_window(struct v4l2_subdev *sd, const struct sensor_win_size *w,
			       bool live)
{
	int pv = ((int)IMX327_ROWS - (int)w->height) / 2 + imx327_crop_y;
	int ret;

	if (w->height >= IMX327_ROWS)
		return 0;
	pv = clamp_t(int, pv, 0, IMX327_ROWS - w->height) & ~1;
	ret = sensor_write(sd, IMX327_REGHOLD, 0x01);
	if (!live) {
		ret |= sensor_write(sd, IMX327_CTRL_07, IMX327_WINMODE_CROP);
		ret |= sensor_write(sd, IMX327_WINWV, (w->height + IMX327_WINWV_MARGIN) & 0xff);
		ret |= sensor_write(sd, IMX327_WINWV + 1, (w->height + IMX327_WINWV_MARGIN) >> 8);
	}
	ret |= sensor_write(sd, IMX327_WINPV, pv & 0xff);
	ret |= sensor_write(sd, IMX327_WINPV + 1, pv >> 8);
	ret |= sensor_write(sd, IMX327_REGHOLD, 0x00);
	return ret ? -EIO : 0;
}

static void imx327_apply_link_rate(void)
{
	unsigned int hts = mipi_891 ? IMX327_HMAX_1080P_891 : IMX327_HMAX_1080P_2LANE;
	int i;

	for (i = 0; i < N_WIN_SIZES; i++) {
		struct sensor_win_size *w = &sensor_win_sizes[i];

		w->hts = hts;
		w->mipi_bps = (mipi_891 ? 891 : 446) * 1000 * 1000;
		w->fps_fixed = w->pclk / (hts * w->vts);
	}
	imx327_hmax = hts;
}

/* Free-height band (already on the control step), while not streaming */
static void imx327_set_roi_height(unsigned int h)
{
	struct sensor_win_size *w = IMX327_ROI_WIN;

	w->height = h;
	w->vts = h + IMX327_VBLANK_BAND;
	w->fps_fixed = w->pclk / (w->hts * w->vts);
}

static int mipi_891_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	if (imx327_streaming)
		return -EBUSY;
	ret = param_set_bool(val, kp);
	if (!ret)
		imx327_apply_link_rate();
	return ret;
}

static const struct kernel_param_ops mipi_891_ops = {
	.set = mipi_891_set,
	.get = param_get_bool,
};
module_param_cb(mipi_891, &mipi_891_ops, &mipi_891, 0664);
MODULE_PARM_DESC(mipi_891, "891 Mbit/s per lane, HMAX 2200: twice the frame rate (default off = 445.5 Mbit/s); change only while not streaming");

/* ------------------------------------------------------------------------
 * Stream on/off
 */
static int sensor_reg_init(struct sensor_info *info)
{
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_format_struct *sensor_fmt = info->fmt;
	struct sensor_win_size *wsize = info->current_wins;
	unsigned int hmax = wsize->hts;
	unsigned int blk = test_pattern ? 0 : black_level;
	int ret;

	/*
	 * Standby alone (stream off) leaves the MIPI clock lane running: the
	 * A733 receiver then never sees the LP-11 -> HS start again and the
	 * second stream gets no frame (clk_lane error). A software reset puts
	 * the lanes back to LP-11 and the registers to their defaults.
	 */
	sensor_write(sd, IMX327_SW_RESET, 0x01);
	usleep_range(2000, 2500);

	ret = imx327_write_table(sd, sensor_default_regs,
				 ARRAY_SIZE(sensor_default_regs), "default");
	if (ret < 0)
		return ret;

	if (mipi_891)
		ret = imx327_write_table(sd, sensor_csi_891_regs,
					 ARRAY_SIZE(sensor_csi_891_regs), "csi 891");
	else
		ret = imx327_write_table(sd, sensor_csi_445_regs,
					 ARRAY_SIZE(sensor_csi_445_regs), "csi 445.5");
	if (ret < 0)
		return ret;

	if (inck == 37125000)
		ret = imx327_write_table(sd, sensor_inck_37m125_regs,
					 ARRAY_SIZE(sensor_inck_37m125_regs), "inck 37.125");
	else
		ret = imx327_write_table(sd, sensor_inck_74m25_regs,
					 ARRAY_SIZE(sensor_inck_74m25_regs), "inck 74.25");
	if (ret < 0)
		return ret;

	ret = imx327_write_table(sd, sensor_fmt->regs, sensor_fmt->regs_size, "format");
	if (ret < 0)
		return ret;

	if (wsize->regs) {
		ret = imx327_write_table(sd, wsize->regs, wsize->regs_size, "mode");
		if (ret < 0)
			return ret;
	}
	if (wsize->set_size)
		wsize->set_size(sd);

	sensor_write(sd, IMX327_CTRL_07, IMX327_WINMODE_1080P);
	sensor_write(sd, IMX327_Y_OUT_SIZE, wsize->height & 0xff);
	sensor_write(sd, IMX327_Y_OUT_SIZE + 1, wsize->height >> 8);
	imx327_write_window(sd, wsize, false);
	sensor_write(sd, IMX327_FR_FDG_SEL,
		     (mipi_891 ? IMX327_FRSEL_60FPS : IMX327_FRSEL_30FPS) |
		     (hcg_mode ? IMX327_FDG_HCG : 0));
	sensor_write(sd, IMX327_HMAX_L, hmax & 0xff);
	sensor_write(sd, IMX327_HMAX_H, (hmax >> 8) & 0xff);
	sensor_write(sd, IMX327_BLKLEVEL_L, blk & 0xff);
	sensor_write(sd, IMX327_BLKLEVEL_H, (blk >> 8) & 0x01);

	info->width = wsize->width;
	info->height = wsize->height;
	imx327_base_vts = wsize->vts;

	/* Re-apply the pose and gain the user set before stream on. */
	sensor_s_exp(sd, info->exp ? info->exp :
		     ((wsize->vts - IMX327_EXPOSURE_OFFSET) * hmax /
		      IMX327_HMAX_1080P_2LANE) << 4);
	imx327_write_gain_reg(sd, imx327_gain_reg);

	if (test_pattern)
		sensor_write(sd, IMX327_PGCTRL, IMX327_PGCTRL_REGEN |
			     IMX327_PGCTRL_THRU |
			     IMX327_PGCTRL_MODE(test_pattern & 0x0f));
	else
		sensor_write(sd, IMX327_PGCTRL, 0x00);

	/* Leave standby, wait for the internal regulators, start the master. */
	sensor_write(sd, IMX327_STANDBY, 0x00);
	msleep(30);
	sensor_write(sd, IMX327_XMSTA, 0x00);

	sensor_print("s_fmt set width = %d, height = %d, hmax = %u, vts = %u, inck = %u, mipi %s Mbit/s, crop y %d\n",
		     wsize->width, wsize->height, imx327_hmax, imx327_vts, inck,
		     mipi_891 ? "891" : "445.5", imx327_crop_y);

	return 0;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct sensor_info *info = to_state(sd);

	sensor_print("%s on = %d, %d*%d %x\n", __func__, enable,
		     info->current_wins->width,
		     info->current_wins->height, info->fmt->mbus_code);

	if (!enable) {
		sensor_write(sd, IMX327_XMSTA, 0x01);
		sensor_write(sd, IMX327_STANDBY, 0x01);
		imx327_streaming = false;
		return 0;
	}
	imx327_streaming = true;
	return sensor_reg_init(info);
}

static int sensor_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	cfg->flags = 0 | V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0;
#else
	/* Same encoding as Radxa's imx219.c on this BSP. */
	cfg->bus.mipi_csi2.num_data_lanes = 0 | V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0;
#endif

	return 0;
}

/* ------------------------------------------------------------------------
 * Controls
 */
static int sensor_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
			container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		ctrl->val = imx327_gain_reg;
		return 0;
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor_info *info =
			container_of(ctrl->handler, struct sensor_info, handler);
	struct v4l2_subdev *sd = &info->sd;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->val);
	case V4L2_CID_ANALOGUE_GAIN:
		return sensor_s_analogue_gain(sd, ctrl->val);
	case V4L2_CID_IMX327_CROP_Y:
		imx327_crop_y = ctrl->val;
		if (imx327_streaming && info->current_wins)
			return imx327_write_window(sd, info->current_wins, true);
		return 0;
	case V4L2_CID_IMX327_ROI_HEIGHT:
		if (imx327_streaming)
			return -EBUSY;
		imx327_set_roi_height(ctrl->val);
		return 0;
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
};

static const struct v4l2_ctrl_config imx327_ctrl_cfgs[] = {
	{
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_IMX327_CROP_Y,
		.name = "Crop Y Offset", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = -IMX327_ROWS, .max = IMX327_ROWS, .step = 1, .def = 0,
		.flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	}, {
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_IMX327_ROI_HEIGHT,
		.name = "ROI Height", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = IMX327_ROI_MIN_H, .max = IMX327_ROWS,
		.step = IMX327_ROI_STEP_H, .def = IMX327_ROI_DEF_H,
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
	int ret = 0, i;

	v4l2_ctrl_handler_init(handler, 3 + ARRAY_SIZE(imx327_ctrl_cfgs));

	/*
	 * VOLATILE alone makes the control core skip s_ctrl (a volatile value
	 * never counts as "changed"): S_CTRL then returns 0 without touching
	 * the sensor. EXECUTE_ON_WRITE forces the write through.
	 */

	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, 1 << 4,
				 IMX327_EXP_LINES_MAX << 4,
				 1, (1125 - IMX327_EXPOSURE_OFFSET) << 4);
	if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 << 4,
				 IMX327_GAIN_Q4_MAX, 1, 1 << 4);
	if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_ANALOGUE_GAIN, 0,
				 IMX327_GAIN_REG_MAX, 1, 0);
	if (ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	for (i = 0; i < ARRAY_SIZE(imx327_ctrl_cfgs); i++)
		v4l2_ctrl_new_custom(handler, &imx327_ctrl_cfgs[i], NULL);

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
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 6, 0))
static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
#else
static int sensor_probe(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
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

	imx327_base_vts = sensor_win_sizes[0].vts;
	imx327_vts = imx327_base_vts;
	imx327_gain_reg = 0;

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
static int sensor_remove(struct i2c_client *client)
#else
static void sensor_remove(struct i2c_client *client)
#endif
{
	struct v4l2_subdev *sd;

	sd = cci_dev_remove_helper(client, &cci_drv);
	kfree(to_state(sd));
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
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
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

VIN_INIT_DRIVERS(init_sensor);
module_exit(exit_sensor);
