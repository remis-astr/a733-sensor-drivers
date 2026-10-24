/*
 * i2cbang: slow bit-banged I2C on PE3 (SCL) / PE4 (SDA) of the Cubie A7S,
 * through /dev/mem, to test the camera I2C bus without the TWI3 controller.
 *
 * The TWI3 driver stays bound (it keeps bldo1 = VCC-PE on); only the pin
 * function of PE3/PE4 is switched to GPIO, then restored on exit.
 * Open-drain emulation: "1" = input (released, external pull-up),
 * "0" = output low.
 *
 * Usage: i2cbang [half_period_us]   (default 500 us, ~1 kHz)
 *        i2cbang hold scl|sda|pe8 <seconds>   hold one line low (for a multimeter)
 *        i2cbang watch <seconds>          print the A-side levels every 0.5 s
 *        i2cbang read <addr> <reg16> <n>  read n bytes from a 16-bit register
 */
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#define PIO_BASE 0x02000000
#define BANK_E   0x280		/* bank n at n * 0x80, E = 5? checked below */
#define CFG0     0x00
#define DAT      0x10
#define SCL      3
#define SDA      4

static volatile uint32_t *bank;
static unsigned hp = 500;
static uint32_t cfg0_saved;

static void set_cfg(int pin, uint32_t f)
{
	int r = CFG0 / 4 + pin / 8, sh = (pin % 8) * 4;
	uint32_t v = bank[r];
	v &= ~(0xfu << sh);
	v |= f << sh;
	bank[r] = v;
}

static void delay_us(unsigned us)	/* busy wait: usleep() rounds up to ~0.6 ms here */
{
	struct timespec t0, t;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	do
		clock_gettime(CLOCK_MONOTONIC, &t);
	while ((t.tv_sec - t0.tv_sec) * 1000000000L + (t.tv_nsec - t0.tv_nsec) < us * 1000L);
}

static void line(int pin, int high)
{
	if (high) {
		set_cfg(pin, 0);			/* input: released */
	} else {
		bank[DAT / 4] &= ~(1u << pin);
		set_cfg(pin, 1);			/* output low */
	}
	delay_us(hp);
}

static int get(int pin) { return (bank[DAT / 4] >> pin) & 1; }

static void start(void) { line(SDA, 1); line(SCL, 1); line(SDA, 0); line(SCL, 0); }
static void stop(void)  { line(SDA, 0); line(SCL, 1); line(SDA, 1); }

static int wbyte(uint8_t b)		/* returns 1 on ACK */
{
	for (int i = 7; i >= 0; i--) {
		line(SDA, (b >> i) & 1);
		line(SCL, 1);
		line(SCL, 0);
	}
	line(SDA, 1);
	line(SCL, 1);
	int ack = !get(SDA);
	line(SCL, 0);
	return ack;
}

static uint8_t rbyte(int ack)
{
	uint8_t b = 0;
	line(SDA, 1);
	for (int i = 0; i < 8; i++) {
		line(SCL, 1);
		b = (b << 1) | get(SDA);
		line(SCL, 0);
	}
	line(SDA, !ack);
	line(SCL, 1);
	line(SCL, 0);
	line(SDA, 1);
	return b;
}

static void sample(const char *what)
{
	int n = 2000, scl0 = 0, sda0 = 0;
	for (int i = 0; i < n; i++) {
		uint32_t d = bank[DAT / 4];
		scl0 += !((d >> SCL) & 1);
		sda0 += !((d >> SDA) & 1);
	}
	printf("%s: SCL low %d/%d, SDA low %d/%d\n", what, scl0, n, sda0, n);
}

int main(int argc, char **argv)
{
	int hold = argc > 3 && !strcmp(argv[1], "hold");
	int watch = argc > 2 && !strcmp(argv[1], "watch");
	int rd = argc > 4 && !strcmp(argv[1], "read");
	if (argc > 1 && !hold && !watch && !rd)
		hp = atoi(argv[1]);
	if (getenv("HP"))			/* half period for the other modes */
		hp = atoi(getenv("HP"));
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) { perror("/dev/mem"); return 1; }
	volatile uint32_t *pio = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
				      MAP_SHARED, fd, PIO_BASE);
	if (pio == MAP_FAILED) { perror("mmap"); return 1; }
	bank = pio + BANK_E / 4;
	cfg0_saved = bank[CFG0 / 4];
	printf("bank E cfg0=0x%08x cfg1=0x%08x dat=0x%08x\n",
	       cfg0_saved, bank[1], bank[DAT / 4]);
	unsigned f3 = (cfg0_saved >> 12) & 0xf, f4 = (cfg0_saved >> 16) & 0xf;
	if ((argc > 2 && !hold && !watch && !rd) || f3 != f4 || f3 < 2 || f3 == 0xf) {
		printf("PE3/PE4 functions %u/%u: not the expected twi3 mux, stop\n", f3, f4);
		return 1;
	}

	set_cfg(SCL, 0);
	set_cfg(SDA, 0);
	usleep(1000);
	sample("released, idle");
	if (rd) {
		int a = strtol(argv[2], NULL, 0), r = strtol(argv[3], NULL, 0);
		int n = atoi(argv[4]);
		struct timespec t0, t1;
		clock_gettime(CLOCK_MONOTONIC, &t0);
		start();
		int ok = wbyte(a << 1) && wbyte(r >> 8) && wbyte(r & 0xff);
		if (ok) {
			start();
			ok = wbyte((a << 1) | 1);
		}
		if (ok) {
			printf("0x%02x reg 0x%04x:", a, r);
			for (int i = 0; i < n; i++)
				printf(" %02x", rbyte(i < n - 1));
			printf("\n");
		} else {
			printf("0x%02x: no ACK\n", a);
		}
		stop();
		clock_gettime(CLOCK_MONOTONIC, &t1);
		long us = (t1.tv_sec - t0.tv_sec) * 1000000 + (t1.tv_nsec - t0.tv_nsec) / 1000;
		/* 5 bytes x 9 clocks + start/restart/stop ~ 48 clocks */
		printf("transaction %ld us, ~%ld kHz\n", us, 48000L / (us ? us : 1));
		goto out;
	}
	if (watch) {
		for (int i = 0; i < 2 * atoi(argv[2]); i++) {
			char t[16];
			snprintf(t, sizeof(t), "t=%4.1fs", i / 2.0);
			sample(t);
			fflush(stdout);
			usleep(500000);
		}
		goto out;
	}
	if (hold) {
		int pin = !strcmp(argv[2], "scl") ? SCL :
			  !strcmp(argv[2], "pe8") ? 8 : SDA;
		uint32_t cfg1_saved = bank[1];
		line(pin, 0);
		printf("%s held low for %s s\n", argv[2], argv[3]);
		fflush(stdout);
		for (int i = 0; i < atoi(argv[3]); i++) {
			sleep(1);
			printf("t=%ds cfg0=0x%08x cfg1=0x%08x dat=0x%08x\n", i + 1,
			       bank[CFG0 / 4], bank[1], bank[DAT / 4]);
			fflush(stdout);
		}
		line(pin, 1);
		bank[1] = cfg1_saved;
		sample("released");
		goto out;
	}

	/* bus recovery: 9 clocks then STOP */
	for (int i = 0; i < 9; i++) { line(SCL, 0); line(SCL, 1); }
	stop();
	sample("after recovery");

	printf("scan:");
	int found = 0;
	for (int a = 0x08; a < 0x78; a++) {
		start();
		int ack = wbyte(a << 1);
		stop();
		if (ack) { printf(" 0x%02x", a); found++; }
	}
	printf("%s\n", found ? "" : " none");

	/* IMX327 at 0x1a: read registers 0x3000 (STANDBY, reset 0x01) .. 0x3003 */
	start();
	int ok = wbyte(0x1a << 1) && wbyte(0x30) && wbyte(0x00);
	if (ok) {
		start();
		ok = wbyte((0x1a << 1) | 1);
		if (ok) {
			printf("imx327 0x3000..0x3003:");
			for (int i = 0; i < 4; i++)
				printf(" %02x", rbyte(i < 3));
			printf("\n");
		}
	}
	stop();
	if (!ok)
		printf("imx327 register read: no ACK\n");

	sample("end");
out:
	bank[CFG0 / 4] = (bank[CFG0 / 4] & ~0xff000u) | (cfg0_saved & 0xff000u);
	printf("restored cfg0=0x%08x\n", bank[CFG0 / 4]);
	return 0;
}
