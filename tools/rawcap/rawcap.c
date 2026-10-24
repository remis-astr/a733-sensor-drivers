// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * rawcap — minimal RAW Bayer capture for the Allwinner VIN (Cubie A7S).
 *
 * Follows the ioctl sequence of Allwinner's own test program
 * (drivers/vin/vin_test/mplane_image/csi_test_mplane.c):
 *   S_INPUT -> S_PARM (capturemode VIDEO) -> S_FMT (MPLANE, Bayer) -> G_FMT
 *   -> REQBUFS/QUERYBUF/mmap -> QBUF -> STREAMON -> DQBUF/QBUF loop.
 *
 * Writes each kept frame as FITS (16-bit, BZERO 32768, BAYERPAT) and/or raw
 * 16-bit little-endian, and prints per-frame statistics used by roadmap
 * step 6: sequence gaps, frame interval, mean/min/max, count at max value.
 *
 *   rawcap -d /dev/video0 -W 1920 -H 1080 -n 20 -k 5 -e 1000 -g 30 -o dark
 *
 * Exposure is -e in lines of 29.63 us (the unit of every A7S driver, whatever
 * the sensor) or -t in seconds (up to ~115.7 s
 * with the imx327_mipi driver), gain (-g) is the IMX327 register value
 * (0.3 dB steps); both are optional and set through V4L2 controls on the
 * sensor subdev (the first imx*_mipi one, or given with -s): the video node
 * hands them to the ISP, which ignores them for a RAW sensor.
 *
 * Build on the board:  cc -O2 -Wall -o rawcap rawcap.c
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <linux/videodev2.h>

#ifndef V4L2_MODE_VIDEO
#define V4L2_MODE_VIDEO 0x0002	/* include/uapi/media/sunxi_camera_v2.h */
#endif

#define NBUF 4
/* IMX327 nominal line (HMAX 4400 at 148.5 MHz), the unit of V4L2_CID_EXPOSURE. */
#define LINE_S (4400.0 / 148.5e6)

struct buffer {
	void *start;
	size_t length;
};

static int xioctl(int fd, unsigned long req, void *arg)
{
	int r;

	do
		r = ioctl(fd, req, arg);
	while (r == -1 && errno == EINTR);
	return r;
}

/*
 * Sensor subdev first: with a RAW sensor and the ISP in the pipeline, the VIN
 * routes EXPOSURE/GAIN set on the video node to the ISP control handler,
 * which ignores them (vin_s_ctrl -> __sunxi_isp_ctrl), so they never reach
 * the sensor. The video node is only a fallback.
 */
static int set_ctrl(int fd, int sfd, uint32_t id, int32_t val, const char *name)
{
	struct v4l2_control c = { .id = id, .value = val };

	if (sfd >= 0 && xioctl(sfd, VIDIOC_S_CTRL, &c) == 0)
		return 0;
	if (sfd < 0 && xioctl(fd, VIDIOC_S_CTRL, &c) == 0) {
		fprintf(stderr, "warning: %s set on the video node, may not reach the sensor\n", name);
		return 0;
	}
	fprintf(stderr, "warning: cannot set %s = %d: %s\n", name, val, strerror(errno));
	return -1;
}

/* /dev/v4l-subdevN whose name starts with prefix (e.g. "imx327"), or NULL. */
static const char *find_subdev(const char *prefix)
{
	static char path[64];
	char name[64];

	for (int i = 0; i < 64; i++) {
		FILE *f;

		snprintf(path, sizeof(path), "/sys/class/video4linux/v4l-subdev%d/name", i);
		f = fopen(path, "r");
		if (!f)
			continue;
		if (fgets(name, sizeof(name), f) && !strncmp(name, prefix, strlen(prefix))) {
			fclose(f);
			snprintf(path, sizeof(path), "/dev/v4l-subdev%d", i);
			return path;
		}
		fclose(f);
	}
	return NULL;
}

static const char *bayer_name(uint32_t f)
{
	switch (f) {
	case V4L2_PIX_FMT_SRGGB12: case V4L2_PIX_FMT_SRGGB10: case V4L2_PIX_FMT_SRGGB8: return "RGGB";
	case V4L2_PIX_FMT_SBGGR12: case V4L2_PIX_FMT_SBGGR10: case V4L2_PIX_FMT_SBGGR8: return "BGGR";
	case V4L2_PIX_FMT_SGRBG12: case V4L2_PIX_FMT_SGRBG10: case V4L2_PIX_FMT_SGRBG8: return "GRBG";
	case V4L2_PIX_FMT_SGBRG12: case V4L2_PIX_FMT_SGBRG10: case V4L2_PIX_FMT_SGBRG8: return "GBRG";
	}
	return "NONE";
}

static uint32_t bayer_fourcc(const char *pattern)
{
	if (!strcmp(pattern, "BGGR")) return V4L2_PIX_FMT_SBGGR12;
	if (!strcmp(pattern, "GRBG")) return V4L2_PIX_FMT_SGRBG12;
	if (!strcmp(pattern, "GBRG")) return V4L2_PIX_FMT_SGBRG12;
	return V4L2_PIX_FMT_SRGGB12;
}

/* One FITS header card, 80 characters, space padded. */
static void card(char *hdr, int *n, const char *fmt, ...)
	__attribute__((format(printf, 3, 4)));
static void card(char *hdr, int *n, const char *fmt, ...)
{
	char line[81];
	va_list ap;
	int len;

	va_start(ap, fmt);
	len = vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	if (len > 80)
		len = 80;
	memset(hdr + *n * 80, ' ', 80);
	memcpy(hdr + *n * 80, line, len);
	(*n)++;
}

static int write_fits(const char *path, const uint16_t *px, unsigned w, unsigned h,
		      const char *bayer, int exp_lines, int gain_reg,
		      const struct timespec *ts, uint32_t seq)
{
	char hdr[2880 * 2];
	char date[32];
	struct tm tm;
	int n = 0, blocks;
	size_t data_bytes = (size_t)w * h * 2, pad;
	uint16_t *be;
	FILE *f;

	memset(hdr, ' ', sizeof(hdr));
	gmtime_r(&ts->tv_sec, &tm);
	strftime(date, sizeof(date), "%Y-%m-%dT%H:%M:%S", &tm);

	card(hdr, &n, "SIMPLE  = %20s", "T");
	card(hdr, &n, "BITPIX  = %20d", 16);
	card(hdr, &n, "NAXIS   = %20d", 2);
	card(hdr, &n, "NAXIS1  = %20u", w);
	card(hdr, &n, "NAXIS2  = %20u", h);
	card(hdr, &n, "BZERO   = %20d", 32768);
	card(hdr, &n, "BSCALE  = %20d", 1);
	card(hdr, &n, "INSTRUME= '%-8s'", "IMX327");
	card(hdr, &n, "BAYERPAT= '%-8s'", bayer);
	card(hdr, &n, "XBAYROFF= %20d", 0);
	card(hdr, &n, "YBAYROFF= %20d", 0);
	if (exp_lines > 0) {
		card(hdr, &n, "EXPLINES= %20d / exposure in sensor lines", exp_lines);
		card(hdr, &n, "EXPTIME = %20.6f / [s]", exp_lines * LINE_S);
	}
	if (gain_reg >= 0)
		card(hdr, &n, "GAIN    = %20d / sensor register, 0.3 dB/step", gain_reg);
	card(hdr, &n, "DATE-OBS= '%s.%03ld'", date, ts->tv_nsec / 1000000);
	card(hdr, &n, "FRAMESEQ= %20u", seq);
	card(hdr, &n, "SWCREATE= '%s'", "rawcap (Cubie A7S)");
	card(hdr, &n, "END");
	blocks = (n * 80 + 2879) / 2880;

	be = malloc(data_bytes);
	if (!be)
		return -1;
	for (size_t i = 0; i < (size_t)w * h; i++) {
		uint16_t v = (uint16_t)(px[i] - 32768);	/* unsigned -> signed + BZERO */

		be[i] = (uint16_t)((v >> 8) | (v << 8));
	}

	f = fopen(path, "wb");
	if (!f) {
		free(be);
		return -1;
	}
	fwrite(hdr, 1, blocks * 2880, f);
	fwrite(be, 1, data_bytes, f);
	pad = (2880 - data_bytes % 2880) % 2880;
	for (size_t i = 0; i < pad; i++)
		fputc(0, f);
	fclose(f);
	free(be);
	return 0;
}

static void usage(const char *p)
{
	fprintf(stderr,
		"usage: %s [-d dev] [-s subdev] [-i input] [-W width] [-H height] [-b RGGB|BGGR|GRBG|GBRG]\n"
		"          [-r fps] [-n frames] [-k skip] [-e lines | -t seconds] [-g gainreg] [-o prefix]\n"
		"          [-F fits|raw|both|none]\n",
		p);
}

int main(int argc, char **argv)
{
	const char *dev = "/dev/video0", *subdev = NULL, *prefix = "rawcap", *out = "fits", *pattern = "RGGB";
	unsigned w = 1920, h = 1080, frames = 10, skip = 3, fps = 30, input = 0;
	int exp_lines = -1, gain_reg = -1, opt, fd, sfd = -1;
	struct buffer bufs[NBUF];
	struct v4l2_plane planes[VIDEO_MAX_PLANES];
	struct v4l2_format fmt;
	struct v4l2_streamparm parm;
	struct v4l2_requestbuffers req;
	struct v4l2_input inp;
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	unsigned stride, got = 0, kept = 0;
	uint32_t last_seq = 0;
	uint16_t all_or = 0, all_max = 0;
	double last_t = 0;
	uint16_t *frame;

	while ((opt = getopt(argc, argv, "d:s:i:W:H:b:r:n:k:e:t:g:o:F:h")) != -1) {
		switch (opt) {
		case 'd': dev = optarg; break;
		case 's': subdev = optarg; break;
		case 'i': input = atoi(optarg); break;
		case 'W': w = atoi(optarg); break;
		case 'H': h = atoi(optarg); break;
		case 'b': pattern = optarg; break;
		case 'r': fps = atoi(optarg); break;
		case 'n': frames = atoi(optarg); break;
		case 'k': skip = atoi(optarg); break;
		case 'e': exp_lines = atoi(optarg); break;
		case 't': exp_lines = (int)(atof(optarg) / LINE_S + 0.5); break;
		case 'g': gain_reg = atoi(optarg); break;
		case 'o': prefix = optarg; break;
		case 'F': out = optarg; break;
		default: usage(argv[0]); return 2;
		}
	}

	fd = open(dev, O_RDWR);
	if (fd < 0) {
		perror(dev);
		return 1;
	}
	if (!subdev)
		subdev = find_subdev("imx");	/* imx327_mipi, imx585_mipi... */
	if (subdev) {
		printf("sensor subdev %s\n", subdev);
		sfd = open(subdev, O_RDWR);
		if (sfd < 0)
			perror(subdev);
	}

	memset(&inp, 0, sizeof(inp));
	inp.index = input;
	if (xioctl(fd, VIDIOC_S_INPUT, &inp)) {
		perror("VIDIOC_S_INPUT");
		return 1;
	}

	memset(&parm, 0, sizeof(parm));
	parm.type = type;
	parm.parm.capture.timeperframe.numerator = 1;
	parm.parm.capture.timeperframe.denominator = fps;
	parm.parm.capture.capturemode = V4L2_MODE_VIDEO;
	if (xioctl(fd, VIDIOC_S_PARM, &parm))
		perror("VIDIOC_S_PARM (continuing)");

	memset(&fmt, 0, sizeof(fmt));
	fmt.type = type;
	fmt.fmt.pix_mp.width = w;
	fmt.fmt.pix_mp.height = h;
	fmt.fmt.pix_mp.pixelformat = bayer_fourcc(pattern);
	fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
	if (xioctl(fd, VIDIOC_S_FMT, &fmt)) {
		perror("VIDIOC_S_FMT");
		return 1;
	}
	if (xioctl(fd, VIDIOC_G_FMT, &fmt)) {
		perror("VIDIOC_G_FMT");
		return 1;
	}
	w = fmt.fmt.pix_mp.width;
	h = fmt.fmt.pix_mp.height;
	stride = fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
	if (stride < w * 2)
		stride = w * 2;
	printf("format %ux%u %.4s (%s) planes=%u bytesperline=%u sizeimage=%u\n",
	       w, h, (char *)&fmt.fmt.pix_mp.pixelformat, bayer_name(fmt.fmt.pix_mp.pixelformat),
	       fmt.fmt.pix_mp.num_planes, fmt.fmt.pix_mp.plane_fmt[0].bytesperline,
	       fmt.fmt.pix_mp.plane_fmt[0].sizeimage);
	if (fmt.fmt.pix_mp.num_planes != 1) {
		fprintf(stderr, "expected 1 plane for Bayer output\n");
		return 1;
	}

	/* Only values the driver accepted end up in the FITS header. */
	if (exp_lines > 0 && set_ctrl(fd, sfd, V4L2_CID_EXPOSURE, exp_lines << 4, "exposure"))
		exp_lines = -1;
	if (gain_reg >= 0 && set_ctrl(fd, sfd, V4L2_CID_ANALOGUE_GAIN, gain_reg, "analogue gain"))
		gain_reg = -1;

	memset(&req, 0, sizeof(req));
	req.count = NBUF;
	req.type = type;
	req.memory = V4L2_MEMORY_MMAP;
	if (xioctl(fd, VIDIOC_REQBUFS, &req) || req.count < 2) {
		perror("VIDIOC_REQBUFS");
		return 1;
	}
	for (unsigned i = 0; i < req.count; i++) {
		struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP, .index = i,
					 .length = 1, .m.planes = planes };

		memset(planes, 0, sizeof(planes));
		if (xioctl(fd, VIDIOC_QUERYBUF, &b)) {
			perror("VIDIOC_QUERYBUF");
			return 1;
		}
		bufs[i].length = planes[0].length;
		bufs[i].start = mmap(NULL, planes[0].length, PROT_READ | PROT_WRITE,
				     MAP_SHARED, fd, planes[0].m.mem_offset);
		if (bufs[i].start == MAP_FAILED) {
			perror("mmap");
			return 1;
		}
		if (xioctl(fd, VIDIOC_QBUF, &b)) {
			perror("VIDIOC_QBUF");
			return 1;
		}
	}

	if (xioctl(fd, VIDIOC_STREAMON, &type)) {
		perror("VIDIOC_STREAMON");
		return 1;
	}

	frame = malloc((size_t)w * h * 2);
	if (!frame)
		return 1;

	printf("  n    seq  dt_ms     mean   min   max  n_at_max\n");
	while (kept < frames) {
		struct v4l2_buffer b = { .type = type, .memory = V4L2_MEMORY_MMAP,
					 .length = 1, .m.planes = planes };
		struct timespec now;
		uint64_t sum = 0;
		uint16_t vmin = 0xffff, vmax = 0;
		unsigned nmax = 0;
		double t;

		memset(planes, 0, sizeof(planes));
		if (xioctl(fd, VIDIOC_DQBUF, &b)) {
			perror("VIDIOC_DQBUF");
			break;
		}
		clock_gettime(CLOCK_REALTIME, &now);
		t = b.timestamp.tv_sec + b.timestamp.tv_usec / 1e6;

		for (unsigned y = 0; y < h; y++) {
			const uint16_t *row = (const uint16_t *)((uint8_t *)bufs[b.index].start + (size_t)y * stride);

			memcpy(frame + (size_t)y * w, row, (size_t)w * 2);
		}
		for (size_t i = 0; i < (size_t)w * h; i++) {
			uint16_t v = frame[i];

			sum += v;
			if (v < vmin)
				vmin = v;
			if (v > vmax) {
				vmax = v;
				nmax = 0;
			}
			if (v == vmax)
				nmax++;
		}
		if (xioctl(fd, VIDIOC_QBUF, &b)) {
			perror("VIDIOC_QBUF");
			break;
		}
		for (size_t i = 0; i < (size_t)w * h; i++)
			all_or |= frame[i];
		if (vmax > all_max)
			all_max = vmax;

		printf("%3u %6u %6.1f %8.1f %5u %5u %8u%s%s\n", got, b.sequence,
		       got ? (t - last_t) * 1000 : 0.0, (double)sum / ((double)w * h), vmin, vmax, nmax,
		       (got && b.sequence > last_seq + 1) ? "  SEQ GAP" : "",
		       got < skip ? "  (skipped)" : "");
		last_t = t;
		last_seq = b.sequence;

		if (got++ < skip)
			continue;

		if (!strcmp(out, "fits") || !strcmp(out, "both")) {
			char path[512];

			snprintf(path, sizeof(path), "%s_%04u.fits", prefix, kept);
			if (write_fits(path, frame, w, h, bayer_name(fmt.fmt.pix_mp.pixelformat),
				       exp_lines, gain_reg, &now, b.sequence))
				perror(path);
		}
		if (!strcmp(out, "raw") || !strcmp(out, "both")) {
			char path[512];
			FILE *f;

			snprintf(path, sizeof(path), "%s_%04u.raw", prefix, kept);
			f = fopen(path, "wb");
			if (f) {
				fwrite(frame, 2, (size_t)w * h, f);
				fclose(f);
			} else {
				perror(path);
			}
		}
		kept++;
	}

	/* Where the 12 bits sit in each 16-bit word (unknown on this VIN). */
	if (all_max > 4095 && !(all_or & 0x000f))
		printf("12-bit data: MSB-aligned (low 4 bits always 0, max %u)\n", all_max);
	else if (all_max <= 4095)
		printf("12-bit data: LSB-aligned (max %u)\n", all_max);
	else
		printf("12-bit data: unclear (max %u, low bits 0x%x)\n", all_max, all_or & 0xf);

	xioctl(fd, VIDIOC_STREAMOFF, &type);
	for (unsigned i = 0; i < req.count; i++)
		munmap(bufs[i].start, bufs[i].length);
	free(frame);
	close(fd);
	if (sfd >= 0)
		close(sfd);
	return kept == frames ? 0 : 1;
}
