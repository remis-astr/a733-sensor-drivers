/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Sony IMX477 RAW sensor driver for the Allwinner VIN framework (Radxa
 * Cubie A7S, A733 / sun60iw2, BSP kernel 6.6.98-aw2511), 2 CSI-2 lanes, for
 * the Raspberry Pi HQ camera and compatible modules.
 *
 * Written 2026-10-17 WITHOUT hardware, from:
 *  - the VIN framework and exposure/gain handling of
 *    drivers/imx327/imx327_mipi.c (validated on the A7S);
 *  - the Raspberry Pi imx477.c with Multicam's planetary crop modes
 *    (minicam_program usr/src/imx477-planetary): register tables extracted
 *    unchanged (imx477_tables.h), 450 MHz link (900 Mbit/s per lane) PLL
 *    and D-PHY settings for a 24 MHz oscillator, minimum line length
 *    formula, long exposure shift, gain registers, stream-on sequence;
 *  - itssme's IMX477 bring-up on the Cubie A7A (src/itssme-imx477, 4 lanes):
 *    chip ID check, per-register table writes with pacing.
 *
 * Modes (all of the Pi driver's, same sizes so the VIN, which picks a window
 * by size alone, can tell binned from native crops): 4056x3040, 4056x2160,
 * 2028x1520 and 2028x1080 binned, 1332x990 binned, and the planetary crops
 * 1920x1080 / 1280x720 / 852x480 / 640x480 binned and 1916x1080 / 1276x720 /
 * 848x480 / 636x480 native. RAW12 or RAW10 by the requested format (Multicam:
 * RAW10 for binned modes, RAW12 otherwise). Readout rotated 180 degrees by
 * default (flip=3), as libcamera does for the HQ camera on the Pi, so the
 * Bayer order is BGGR and images are oriented as with Multicam on the Pi.
 * Other flips change the order as on the Pi (flip 0..3: RGGB, GRBG, GBRG,
 * BGGR) but the format stays SBGGR: the application derives the order from
 * the flip parameter (Multicam: SensorProfile.flip_bayer).
 *
 * Exposure: same unit as every A7S driver, lines of 29.63 us in Q4 (16 = one
 * line of HMAX 4400 at 148.5 MHz), converted here to lines of LINE_LENGTH at
 * 840 MHz. Long poses: frame length, then the sensor's x2^n shift (n <= 7),
 * then a longer line, as on the Pi.
 *
 * Movable crop (2026-10-21): "Crop X/Y Offset" move the analog window
 * (X/Y_ADD_STA/END) from its table position, in image pixels (flip taken into
 * account, positive = right/down in the image), in steps of 4 so the Bayer
 * phase never changes, clamped to the 4056x3040 array. Applied at stream start
 * and live while streaming (measured: frame N+2, no torn frame, no loss).
 * Free window ("ROI Width/Height"): the last mode, native 12/10-bit, its size
 * set by the controls while not streaming (the VIN reads sensor_win_sizes at
 * S_FMT, so request the new size afterwards), built on the 1916x1080 table.
 *
 * TO VALIDATE ON HARDWARE (nothing here has run yet):
 *  - power-up: the HQ camera's regulators are enabled by connector pin 11
 *    (PE8 here): measured 2026-10-17, the sensor answers 173 ms after PE8
 *    goes high (100 ms failed at boot); 300 ms is used;
 *  - embedded data lines disabled (0xbcf1 = 0, the Pi enables 2): the VIN
 *    expects image lines only;
 *  - software reset 0x0103 before each stream, for the A733 receiver's
 *    LP-11 start (as the IMX327 needed);
 *  - 4056-wide modes: itssme needed the A733's paired TDM/ISP large-image
 *    path above ~4K width; probably unusable through /dev/video0 as is.
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
MODULE_DESCRIPTION("A low-level driver for IMX477 sensors (RAW, Allwinner VIN)");
MODULE_LICENSE("GPL");

#define SENSOR_NAME		"imx477_mipi"
#define V4L2_IDENT_SENSOR	0x0477

/* ------------------------------------------------------------------------
 * Registers (CCS-style, names from the Pi driver)
 */
#define IMX477_CHIP_ID_H	0x0016
#define IMX477_CHIP_ID_L	0x0017
#define IMX477_MODE_SELECT	0x0100
#define IMX477_ORIENTATION	0x0101
#define IMX477_SW_RESET		0x0103
#define IMX477_GROUP_HOLD	0x0104
#define IMX477_CSI_DT_FMT_H	0x0112
#define IMX477_CSI_DT_FMT_L	0x0113
#define IMX477_EXPOSURE		0x0202	/* 16 bits, lines */
#define IMX477_ANA_GAIN		0x0204	/* 16 bits, 0..978: gain = 1024 / (1024 - code) */
#define IMX477_DGTL_GAIN	0x020e	/* 16 bits, Q8 */
#define IMX477_IOP_PXCK_DIV	0x0309
#define IMX477_IOP_MPY		0x030e	/* 16 bits */
#define IMX477_FRAME_LENGTH	0x0340	/* 16 bits */
#define IMX477_LINE_LENGTH	0x0342	/* 16 bits */
#define IMX477_REQ_LINK_RATE	0x0820	/* 32 bits, Mbit/s (all lanes) << 16 */
#define IMX477_DPC_1		0x0b05
#define IMX477_DPC_2		0x0b06
#define IMX477_XVS_IO_CTRL	0x3040
#define IMX477_MS_SEL		0x3041
#define IMX477_LONG_EXP_SHIFT	0x3100
#define IMX477_MC_MODE		0x3f0b
#define IMX477_ADBIT_MODE	0x3f0d
#define IMX477_EXTOUT_EN	0x4b81
#define IMX477_EBD_LINES	0xbcf1	/* embedded data lines (Pi: 2) */

#define IMX477_PIXEL_RATE	840000000
#define IMX477_LINK_FREQ	450000000	/* 900 Mbit/s per lane */
#define IMX477_IOP_MPY_450	150		/* 450 MHz x 2 x 2 x 2 / 24 MHz */
#define IMX477_FRAME_LENGTH_MAX	0xffdc
#define IMX477_LINE_LENGTH_MAX	0xfff0
#define IMX477_LONG_EXP_SHIFT_MAX 7
#define IMX477_EXPOSURE_OFFSET	22
#define IMX477_EXPOSURE_MIN	4
#define IMX477_VBLANK_MIN	48
#define IMX477_ANA_GAIN_MAX	978		/* 22.26x */
#define IMX477_X_ADD_STA	0x0344	/* 16 bits each: STA, then END at +4 */
#define IMX477_Y_ADD_STA	0x0346
#define IMX477_X_ADD_END	0x0348
#define IMX477_Y_ADD_END	0x034a
#define IMX477_X_OUT_SIZE	0x034c
#define IMX477_Y_OUT_SIZE	0x034e
#define IMX477_DIG_CROP_W	0x040c
#define IMX477_DIG_CROP_H	0x040e
#define IMX477_ARRAY_W		4056
#define IMX477_ARRAY_H		3040

/* Custom controls, same ids as Rpicamera3's IMX585 driver */
#define V4L2_CID_IMX477_BASE		(V4L2_CID_USER_BASE + 0x2000)
#define V4L2_CID_IMX477_CROP_X		(V4L2_CID_IMX477_BASE + 10)
#define V4L2_CID_IMX477_CROP_Y		(V4L2_CID_IMX477_BASE + 11)
#define V4L2_CID_IMX477_ROI_WIDTH	(V4L2_CID_IMX477_BASE + 12)
#define V4L2_CID_IMX477_ROI_HEIGHT	(V4L2_CID_IMX477_BASE + 13)
#define IMX477_ROI_MIN_W	256
#define IMX477_ROI_MIN_H	128
#define IMX477_ROI_MAX_W	4048
#define IMX477_ROI_STEP_W	16
#define IMX477_ROI_STEP_H	4
#define IMX477_ROI_DEF_W	1024
#define IMX477_ROI_DEF_H	768

/* nominal exposure line (4400 / 148.5 MHz) in pixel clocks at 840 MHz, x9 */
#define IMX477_NOMINAL_X9	224000		/* 4400 x 840 / 148.5 x 9 */
/* Longest pose in nominal lines: 0xffdc x 128 lines of 0xfff0 / 840 MHz */
#define IMX477_EXP_LINES_MAX	22000000
#define IMX477_GAIN_Q4_MAX	(64 << 4)	/* 22.26x analog, then digital */

/* ------------------------------------------------------------------------
 * Module parameters
 */
static uint flip = 3;
module_param(flip, uint, 0664);
MODULE_PARM_DESC(flip, "Readout orientation: bit 0 horizontal, bit 1 vertical (default 3 = 180 degrees, BGGR, as on the Pi)");

static bool dpc_enable = true;
module_param(dpc_enable, bool, 0664);
MODULE_PARM_DESC(dpc_enable, "On-sensor defect pixel correction (Pi driver default: on)");

static bool isp_ae;
module_param(isp_ae, bool, 0664);
MODULE_PARM_DESC(isp_ae, "Accept exposure/gain requests from the ISP 3A; off = manual RAW control only");

/* ------------------------------------------------------------------------
 * Register tables
 */
#include "imx477_tables.h"

static struct regval_list sensor_fmt_raw[] = {
};

static int imx477_write_table(struct v4l2_subdev *sd, struct regval_list *regs,
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

static int imx477_write16(struct v4l2_subdev *sd, addr_type reg, unsigned int val)
{
	int ret = sensor_write(sd, reg, (val >> 8) & 0xff);	/* big endian */

	return ret ? ret : sensor_write(sd, reg + 1, val & 0xff);
}

/* ------------------------------------------------------------------------
 * Modes
 */
struct imx477_mode {
	unsigned int width, height, frm_length;
	struct regval_list *regs;
	unsigned int nregs;
	bool binned;		/* 2x2 binning (0x0900 = 1 in the table) */
	bool roi;		/* free window: size from the ROI controls */
	unsigned int x0, y0, aw, ah;	/* analog window of the table */
};

#define IMX477_MODE(w, h, frm, tab) {w, h, frm, tab, ARRAY_SIZE(tab)},
static struct imx477_mode imx477_modes[] = {
	IMX477_PI_MODES
	/* free window, last: native crop registers of the 1916x1080 mode */
	{IMX477_ROI_DEF_W, IMX477_ROI_DEF_H, 0, imx477_mode_1080p_regs,
	 ARRAY_SIZE(imx477_mode_1080p_regs), false, true},
};
#undef IMX477_MODE
#define N_MODES ARRAY_SIZE(imx477_modes)

static struct sensor_win_size sensor_win_sizes[N_MODES];
#define N_WIN_SIZES N_MODES

static struct imx477_mode *imx477_mode_of(struct sensor_win_size *w)
{
	return &imx477_modes[w - sensor_win_sizes];
}

/* Pi driver: link time of one line + 500 clocks for HS<>LP, and at least
 * 1332 + 4600 in 12-bit. */
static unsigned int imx477_line_length_min(const struct imx477_mode *m, int bpp)
{
	u64 ll = (u64)m->width * bpp * IMX477_PIXEL_RATE;

	do_div(ll, (u64)IMX477_LINK_FREQ * 2 * 2);
	ll += 500;
	if (bpp == 12)
		ll = max_t(u64, ll, 1332 + 4600);
	return (unsigned int)ll;
}

/* 16-bit big-endian value of a table register pair, 0 if absent */
static unsigned int imx477_table16(const struct imx477_mode *m, unsigned int reg)
{
	unsigned int v = 0;
	int j;

	for (j = 0; j < m->nregs; j++) {
		if (m->regs[j].addr == reg)
			v |= m->regs[j].data << 8;
		else if (m->regs[j].addr == reg + 1)
			v |= m->regs[j].data;
	}
	return v;
}

static void imx477_build_window(int i)
{
	struct imx477_mode *m = &imx477_modes[i];
	struct sensor_win_size *w = &sensor_win_sizes[i];
	int bpp = m->binned ? 10 : 12;

	w->width = m->width;
	w->height = m->height;
	w->hts = imx477_line_length_min(m, bpp);
	w->vts = max(m->frm_length, m->height + IMX477_VBLANK_MIN);
	w->pclk = IMX477_PIXEL_RATE;
	w->mipi_bps = 2 * IMX477_LINK_FREQ;
	w->fps_fixed = IMX477_PIXEL_RATE / (w->hts * w->vts);
	w->bin_factor = 1;
	w->intg_min = 1 << 4;
	w->intg_max = IMX477_EXP_LINES_MAX << 4;
	w->gain_min = 1 << 4;
	w->gain_max = IMX477_GAIN_Q4_MAX;
	w->top_clk = 300000000;
	w->isp_clk = 297000000;
}

/* Free window size (already on the control steps), centred, start on 4 */
static void imx477_set_roi(unsigned int width, unsigned int height)
{
	struct imx477_mode *m = &imx477_modes[N_MODES - 1];

	m->width = m->aw = width;
	m->height = m->ah = height;
	m->x0 = ((IMX477_ARRAY_W - width) / 2) & ~3U;
	m->y0 = ((IMX477_ARRAY_H - height) / 2) & ~3U;
	imx477_build_window(N_MODES - 1);
}

static void imx477_build_windows(void)
{
	int i, j;

	for (i = 0; i < N_MODES; i++) {
		struct imx477_mode *m = &imx477_modes[i];

		for (j = 0; j < m->nregs; j++)
			if (m->regs[j].addr == 0x0900 && m->regs[j].data == 0x01)
				m->binned = true;
		if (m->roi) {
			imx477_set_roi(m->width, m->height);
			continue;
		}
		m->x0 = imx477_table16(m, IMX477_X_ADD_STA);
		m->y0 = imx477_table16(m, IMX477_Y_ADD_STA);
		m->aw = imx477_table16(m, IMX477_X_ADD_END) + 1 - m->x0;
		m->ah = imx477_table16(m, IMX477_Y_ADD_END) + 1 - m->y0;
		imx477_build_window(i);
	}
}

/* ------------------------------------------------------------------------
 * State. One sensor per board.
 */
static unsigned int imx477_line_len;	/* LINE_LENGTH of the current stream */
static unsigned int imx477_base_vts;	/* frame length for the requested fps */
static unsigned int imx477_ana_code, imx477_dgain = 0x100;
static int imx477_crop_x, imx477_crop_y;	/* image pixels from the table position */
static bool imx477_streaming;

/* Start of the analog window: table start + offset, clamped to the array,
 * offset in multiples of 4 (Bayer phase kept, binned or not). */
static unsigned int imx477_crop_start(unsigned int start, unsigned int size,
				      unsigned int array, int off)
{
	int d = clamp_t(int, off, -(int)start, (int)(array - size) - (int)start);

	return start + (d >= 0 ? d & ~3 : -(-d & ~3));
}

static int imx477_write_window(struct v4l2_subdev *sd, const struct imx477_mode *m)
{
	int ox = flip & 1 ? -imx477_crop_x : imx477_crop_x;
	int oy = flip & 2 ? -imx477_crop_y : imx477_crop_y;
	unsigned int x = imx477_crop_start(m->x0, m->aw, IMX477_ARRAY_W, ox);
	unsigned int y = imx477_crop_start(m->y0, m->ah, IMX477_ARRAY_H, oy);
	int ret;

	ret = sensor_write(sd, IMX477_GROUP_HOLD, 0x01);
	ret |= imx477_write16(sd, IMX477_X_ADD_STA, x);
	ret |= imx477_write16(sd, IMX477_Y_ADD_STA, y);
	ret |= imx477_write16(sd, IMX477_X_ADD_END, x + m->aw - 1);
	ret |= imx477_write16(sd, IMX477_Y_ADD_END, y + m->ah - 1);
	ret |= sensor_write(sd, IMX477_GROUP_HOLD, 0x00);
	sensor_dbg("window %ux%u at %u,%u\n", m->aw, m->ah, x, y);
	return ret ? -EIO : 0;
}

/* ------------------------------------------------------------------------
 * Exposure: exp_val in Q4 nominal 29.63 us lines (see the header).
 */
static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	*value = to_state(sd)->exp;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, unsigned int exp_val)
{
	struct sensor_info *info = to_state(sd);
	unsigned int ll = imx477_line_len ? imx477_line_len : sensor_win_sizes[0].hts;
	u64 lines, frame;
	unsigned int shift = 0;
	int ret = 0;

	if (exp_val > IMX477_EXP_LINES_MAX << 4)
		exp_val = IMX477_EXP_LINES_MAX << 4;
	lines = div_u64((u64)exp_val * IMX477_NOMINAL_X9 + 72 * ll, 144 * ll);
	/* Too long even with the x128 shift: lengthen the line. */
	if (lines + IMX477_EXPOSURE_OFFSET > (u64)IMX477_FRAME_LENGTH_MAX << IMX477_LONG_EXP_SHIFT_MAX) {
		ll = min_t(u64, IMX477_LINE_LENGTH_MAX,
			   DIV_ROUND_UP_ULL(lines * ll, ((u64)IMX477_FRAME_LENGTH_MAX
				<< IMX477_LONG_EXP_SHIFT_MAX) - IMX477_EXPOSURE_OFFSET));
		lines = div_u64((u64)exp_val * IMX477_NOMINAL_X9 + 72 * ll, 144 * ll);
	}
	if (lines < IMX477_EXPOSURE_MIN)
		lines = IMX477_EXPOSURE_MIN;
	frame = max_t(u64, imx477_base_vts, lines + IMX477_EXPOSURE_OFFSET);
	while ((frame >> shift) > IMX477_FRAME_LENGTH_MAX && shift < IMX477_LONG_EXP_SHIFT_MAX)
		shift++;
	if ((frame >> shift) > IMX477_FRAME_LENGTH_MAX)
		frame = (u64)IMX477_FRAME_LENGTH_MAX << shift;

	ret |= sensor_write(sd, IMX477_GROUP_HOLD, 0x01);
	ret |= imx477_write16(sd, IMX477_LINE_LENGTH, ll);
	ret |= imx477_write16(sd, IMX477_FRAME_LENGTH, (unsigned int)(frame >> shift));
	ret |= sensor_write(sd, IMX477_LONG_EXP_SHIFT, shift);
	ret |= imx477_write16(sd, IMX477_EXPOSURE, (unsigned int)min_t(u64, lines >> shift, 0xffff));
	ret |= sensor_write(sd, IMX477_GROUP_HOLD, 0x00);
	if (ret)
		return -EIO;
	info->exp = exp_val;
	sensor_dbg("exp q4 %u -> %llu lines, ll %u, frame %llu, shift %u\n",
		   exp_val, lines, ll, frame, shift);
	return 0;
}

/* ------------------------------------------------------------------------
 * Gain: analog code up to 22.26x, the rest in digital gain (Q8).
 */
static int imx477_write_gain(struct v4l2_subdev *sd, unsigned int code, unsigned int dgain)
{
	int ret;

	code = min_t(unsigned int, code, IMX477_ANA_GAIN_MAX);
	dgain = clamp_t(unsigned int, dgain, 0x100, 0xffff);
	ret = sensor_write(sd, IMX477_GROUP_HOLD, 0x01);
	ret |= imx477_write16(sd, IMX477_ANA_GAIN, code);
	ret |= imx477_write16(sd, IMX477_DGTL_GAIN, dgain);
	ret |= sensor_write(sd, IMX477_GROUP_HOLD, 0x00);
	if (ret)
		return -EIO;
	imx477_ana_code = code;
	imx477_dgain = dgain;
	return 0;
}

/* linear gain Q4 of the current registers */
static unsigned int imx477_gain_q4(void)
{
	return (unsigned int)div_u64((u64)16 * 1024 * imx477_dgain,
				     (u64)(1024 - imx477_ana_code) * 256);
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	*value = to_state(sd)->gain;
	return 0;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int gain_val)
{
	struct sensor_info *info = to_state(sd);
	unsigned int g = clamp_t(unsigned int, gain_val, 16, IMX477_GAIN_Q4_MAX);
	unsigned int code = 1024 - DIV_ROUND_CLOSEST(1024 * 16, g);
	unsigned int dgain = 0x100;
	int ret;

	if (code > IMX477_ANA_GAIN_MAX) {
		code = IMX477_ANA_GAIN_MAX;
		/* remaining gain = g / 16 / (1024 / 46), in Q8 */
		dgain = DIV_ROUND_CLOSEST(g * (1024 - IMX477_ANA_GAIN_MAX) * 256, 16 * 1024);
	}
	ret = imx477_write_gain(sd, code, dgain);
	if (!ret)
		info->gain = imx477_gain_q4();
	return ret;
}

static int sensor_s_analogue_gain(struct v4l2_subdev *sd, int code)
{
	struct sensor_info *info = to_state(sd);
	int ret = imx477_write_gain(sd, code, 0x100);

	if (!ret)
		info->gain = imx477_gain_q4();
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
	unsigned int ll = imx477_line_len ? imx477_line_len : (wsize ? wsize->hts : 0);

	if (!wsize || !fps->fps || !ll)
		return -EINVAL;
	imx477_base_vts = clamp_t(unsigned int, IMX477_PIXEL_RATE / fps->fps / ll,
				  wsize->vts, IMX477_FRAME_LENGTH_MAX);
	return sensor_s_exp(sd, info->exp);
}

/* ------------------------------------------------------------------------
 * Power, reset, detection
 */
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	switch (on) {
	case STBY_ON:
		cci_lock(sd);
		sensor_write(sd, IMX477_MODE_SELECT, 0x00);
		cci_unlock(sd);
		break;
	case STBY_OFF:
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
		vin_gpio_write(sd, PWDN, CSI_GPIO_HIGH);	/* pin 11: HQ camera regulators */
		vin_set_mclk_freq(sd, 24000000);
		vin_set_mclk(sd, ON);
		msleep(300);	/* HQ camera: ID readable 173 ms after PE8 high */
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

static int sensor_detect(struct v4l2_subdev *sd)
{
	data_type hi = 0, lo = 0;
	unsigned int id;

	if (sensor_read(sd, IMX477_CHIP_ID_H, &hi) || sensor_read(sd, IMX477_CHIP_ID_L, &lo)) {
		sensor_err("no answer on I2C\n");
		return -ENODEV;
	}
	id = (hi << 8) | lo;
	sensor_print("IMX477 probe: chip ID 0x%04x\n", id);
	return id == V4L2_IDENT_SENSOR ? 0 : -ENODEV;
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

/* Bayer order after the default 180 degree readout (flip=3). */
static struct sensor_format_struct sensor_formats[] = {
	{
		.desc = "Raw RGB Bayer 12",
		.mbus_code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.regs = sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp = 1
	}, {
		.desc = "Raw RGB Bayer 10",
		.mbus_code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.regs = sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp = 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

/* ------------------------------------------------------------------------
 * Stream on/off
 */
static int sensor_reg_init(struct sensor_info *info)
{
	struct v4l2_subdev *sd = &info->sd;
	struct sensor_win_size *wsize = info->current_wins;
	struct imx477_mode *m = imx477_mode_of(wsize);
	int bpp = info->fmt->mbus_code == MEDIA_BUS_FMT_SBGGR10_1X10 ? 10 : 12;
	u32 rate = (IMX477_LINK_FREQ / 1000000) * 2 * 2;
	int ret;

	/* software reset: lanes back to LP-11 for the A733 receiver */
	sensor_write(sd, IMX477_SW_RESET, 0x01);
	usleep_range(10000, 12000);

	ret = imx477_write_table(sd, imx477_mode_common_regs,
				 ARRAY_SIZE(imx477_mode_common_regs), "common");
	if (ret < 0)
		return ret;
	ret = sensor_write(sd, IMX477_EBD_LINES, 0x00);
	ret |= imx477_write16(sd, IMX477_IOP_MPY, IMX477_IOP_MPY_450);
	ret |= sensor_write(sd, IMX477_REQ_LINK_RATE, (rate >> 8) & 0xff);
	ret |= sensor_write(sd, IMX477_REQ_LINK_RATE + 1, rate & 0xff);
	ret |= sensor_write(sd, IMX477_REQ_LINK_RATE + 2, 0);
	ret |= sensor_write(sd, IMX477_REQ_LINK_RATE + 3, 0);
	if (ret)
		return -EIO;

	ret = imx477_write_table(sd, m->regs, m->nregs, "mode");
	if (ret < 0)
		return ret;
	if (m->roi) {
		ret = imx477_write16(sd, IMX477_DIG_CROP_W, m->width);
		ret |= imx477_write16(sd, IMX477_DIG_CROP_H, m->height);
		ret |= imx477_write16(sd, IMX477_X_OUT_SIZE, m->width);
		ret |= imx477_write16(sd, IMX477_Y_OUT_SIZE, m->height);
		if (ret)
			return -EIO;
	}
	ret = imx477_write_window(sd, m);
	if (ret)
		return ret;
	ret = sensor_write(sd, IMX477_CSI_DT_FMT_H, bpp);
	ret |= sensor_write(sd, IMX477_CSI_DT_FMT_L, bpp);
	ret |= sensor_write(sd, IMX477_IOP_PXCK_DIV, bpp);
	ret |= sensor_write(sd, IMX477_ADBIT_MODE, bpp == 12 ? 1 : 0);
	ret |= sensor_write(sd, IMX477_DPC_1, dpc_enable ? 1 : 0);
	ret |= sensor_write(sd, IMX477_DPC_2, dpc_enable ? 1 : 0);
	ret |= sensor_write(sd, IMX477_ORIENTATION, flip & 0x03);
	/* standalone: no XVS in or out (Pi trigger_mode 0) */
	ret |= sensor_write(sd, IMX477_EXTOUT_EN, 0);
	ret |= sensor_write(sd, IMX477_MS_SEL, 1);
	ret |= sensor_write(sd, IMX477_XVS_IO_CTRL, 0);
	ret |= sensor_write(sd, IMX477_MC_MODE, 0);
	if (ret)
		return -EIO;

	info->width = wsize->width;
	info->height = wsize->height;
	imx477_line_len = imx477_line_length_min(m, bpp);
	imx477_base_vts = wsize->vts;
	sensor_s_exp(sd, info->exp ? info->exp : 1000 << 4);
	imx477_write_gain(sd, imx477_ana_code, imx477_dgain);

	ret = sensor_write(sd, IMX477_MODE_SELECT, 0x01);
	imx477_streaming = !ret;
	sensor_print("s_fmt %ux%u%s%s RAW%d, line %u, frame %u, crop %d,%d\n",
		     wsize->width, wsize->height, m->binned ? " binned" : "",
		     m->roi ? " roi" : "", bpp, imx477_line_len, imx477_base_vts,
		     imx477_crop_x, imx477_crop_y);
	return ret;
}

static int sensor_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (!enable) {
		imx477_streaming = false;
		return sensor_write(sd, IMX477_MODE_SELECT, 0x00);
	}
	return sensor_reg_init(to_state(sd));
}

static int sensor_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = 0 | V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0;
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
		ctrl->val = imx477_ana_code;
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
	case V4L2_CID_IMX477_CROP_X:
	case V4L2_CID_IMX477_CROP_Y:
		if (ctrl->id == V4L2_CID_IMX477_CROP_X)
			imx477_crop_x = ctrl->val;
		else
			imx477_crop_y = ctrl->val;
		if (imx477_streaming && info->current_wins)
			return imx477_write_window(sd, imx477_mode_of(info->current_wins));
		return 0;
	case V4L2_CID_IMX477_ROI_WIDTH:
	case V4L2_CID_IMX477_ROI_HEIGHT: {
		struct imx477_mode *roi = &imx477_modes[N_MODES - 1];

		if (imx477_streaming)
			return -EBUSY;
		if (ctrl->id == V4L2_CID_IMX477_ROI_WIDTH)
			imx477_set_roi(ctrl->val, roi->height);
		else
			imx477_set_roi(roi->width, ctrl->val);
		return 0;
	}
	}
	return -EINVAL;
}

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.g_volatile_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
};

static const struct v4l2_ctrl_config imx477_ctrl_cfgs[] = {
	{
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_IMX477_CROP_X,
		.name = "Crop X Offset", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = -IMX477_ARRAY_W, .max = IMX477_ARRAY_W, .step = 1, .def = 0,
		.flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	}, {
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_IMX477_CROP_Y,
		.name = "Crop Y Offset", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = -IMX477_ARRAY_H, .max = IMX477_ARRAY_H, .step = 1, .def = 0,
		.flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
	}, {
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_IMX477_ROI_WIDTH,
		.name = "ROI Width", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = IMX477_ROI_MIN_W, .max = IMX477_ROI_MAX_W,
		.step = IMX477_ROI_STEP_W, .def = IMX477_ROI_DEF_W,
	}, {
		.ops = &sensor_ctrl_ops, .id = V4L2_CID_IMX477_ROI_HEIGHT,
		.name = "ROI Height", .type = V4L2_CTRL_TYPE_INTEGER,
		.min = IMX477_ROI_MIN_H, .max = IMX477_ARRAY_H,
		.step = IMX477_ROI_STEP_H, .def = IMX477_ROI_DEF_H,
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

	v4l2_ctrl_handler_init(handler, 3 + ARRAY_SIZE(imx477_ctrl_cfgs));
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_EXPOSURE, 1 << 4,
				 IMX477_EXP_LINES_MAX << 4, 1, 1000 << 4);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_GAIN, 1 << 4, IMX477_GAIN_Q4_MAX, 1, 1 << 4);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	ctrl = v4l2_ctrl_new_std(handler, ops, V4L2_CID_ANALOGUE_GAIN, 0, IMX477_ANA_GAIN_MAX, 1, 0);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	for (i = 0; i < ARRAY_SIZE(imx477_ctrl_cfgs); i++)
		v4l2_ctrl_new_custom(handler, &imx477_ctrl_cfgs[i], NULL);
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

	imx477_base_vts = sensor_win_sizes[0].vts;
	imx477_ana_code = 0;
	imx477_dgain = 0x100;
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
	imx477_build_windows();
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	cci_dev_exit_helper(&sensor_driver);
}

VIN_INIT_DRIVERS(init_sensor);
module_exit(exit_sensor);
