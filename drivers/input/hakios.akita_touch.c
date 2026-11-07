/*
 * The touch screen of the Pixel 8a (akita): a Goodix GT9895 ("nottingham", Goodix's Berlin family) on the SPI controller
 * at 0x111D0000 (USI2 of zuma's HSI0 block), as a source of touches for /dev/hakios-touch (sys/hakios.input.h). Only with
 * boot-arg touchscreen=1.
 *
 * Who does what:
 *   U-Boot (devices/akita/u-boot, $hakios_touch, run at every start): the chip's two supplies through Google's power
 *       firmware (S2MPG14 LDO4M, AVDD, set to 3.0 V; LDO25M, DVDD, 1.8 V: `acpm`), the reset pin low and then high around
 *       it (gpp21-2). It has the ACPM client; the kernel has none.
 *   this driver: the SPI controller's clock (CMU_HSI0: USI2's mux to the 24.576 MHz crystal, divider 1, its gates held
 *       on: 6.1 MHz on the wire, the controller dividing by four), the USI block in SPI mode, the controller itself
 *       (Samsung's s3c64xx: polled, 8-bit, mode 0, chip select by hand), and the Goodix protocol: where the chip keeps its
 *       touch report and how big a finger's record is (14 bytes on this chip, 8 on others), read from its description.
 *
 * The chip keeps its firmware in its own flash (Android's driver finds it current and loads nothing), so nothing is
 * loaded here. No interrupt is used: the interrupt line (gpn0-0, low while a report waits) is looked at 250 times a
 * second, and the report read when it is low, or every 80 ms anyway.
 *
 * Found on the phone (2026-11-01): from U-Boot (uefi cmd/hakios_tspi.c) the chip's version block read back
 * "BERLIN", "9895", vid 1f 01 00 11, checksum good, once the USI2 clock was forced on: the bootloader leaves its gates
 * to the hardware and its mux on a parent that is not running.
 *
 * Register layouts: Linux drivers/spi/spi-s3c64xx.c (Google's, soc/gs), drivers/soc/google/cal-if/zuma/cmucal-sfr.c
 * (the clocks), Google's goodix_brl_spi.c and goodix_brl_hw.c (the protocol), the phone's device tree.
 */
#include <stdint.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);

/* ---- the addresses ------------------------------------------------------------------------------------------------ */

#define SPI_PHYS                0x111d0000ULL   /* spi@111D0000, USI2 of HSI0 */
#define CMU_PHYS                0x11001000ULL   /* CMU_HSI0 (0x11000000) from its muxes on */
#define CMU_MUX_USI2            0x01c           /* CLK_CON_MUX_MUX_CLK_HSI0_USI2: bit 0, 1 = the crystal (OSCCLK) */
#define CMU_DIV_USI2            0x81c           /* CLK_CON_DIV_DIV_CLK_HSI0_USI2: the ratio less one, bits 3:0 */
#define CMU_GATE_RSTNSYNC_USI2  0x10bc
#define CMU_GATE_IPCLK_USI2     0x10fc
#define CMU_GATE_PCLK_USI2      0x1100
#define CMU_GATE_USI2           0x1120
#define CMU_GATE_ON             0x00300000      /* MANUAL (bit 20) and CG_VAL (bit 21): held on, not left to the hardware */
#define SYSREG_PHYS             0x11021000ULL   /* sysreg_hsi0 */
#define SYSREG_USI2_MODE        0x01c           /* + 0x1000 in the tree's terms: samsung,usi-offset = <0x101c> */
#define USI_MODE_SPI            0x2
#define PIN_INT_PHYS            0x15060000ULL   /* gpn0 of pinctrl@15060000: DAT at +4, bit 0 */
#define PIN_RESET_PHYS          0x10c40000ULL   /* gpp21 at +0x40 of pinctrl@10C40000: DAT at +0x44, bit 2 */

/* s3c64xx SPI */
#define CH_CFG                  0x00
#define MODE_CFG                0x08
#define CS_REG                  0x0c
#define INT_EN                  0x10
#define STATUS                  0x14
#define TX_DATA                 0x18
#define RX_DATA                 0x1c
#define PACKET_CNT              0x20
#define PENDING_CLR             0x24
#define SWAP_CFG                0x28
#define USI_CON                 0xc4
#define USI_OPTION              0xc8
#define CH_TXCH_ON              (1u << 0)
#define CH_RXCH_ON              (1u << 1)
#define CH_SW_RST               (1u << 5)
#define CS_INACTIVE             (1u << 0)
#define PACKET_CNT_EN           (1u << 16)
#define USI_CON_RESET           (1u << 0)
#define USI_OPTION_CLKREQ_ON    (1u << 1)
#define USI_OPTION_CLKSTOP_ON   (1u << 2)
#define RX_LVL(s)               (((s) >> 15) & 0x7f)
#define FIFO_INFLIGHT           32              /* bytes out and not yet back, at most (the FIFOs hold 64) */

static vm_offset_t spi, cmu, sysreg, pin_int, pin_reset;

static inline uint32_t
rd(vm_offset_t a)
{
	return *(volatile uint32_t *)a;
}

static inline void
wr(vm_offset_t a, uint32_t v)
{
	*(volatile uint32_t *)a = v;
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

static uint64_t
micros_now(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000;
}

/* ---- the clock, the USI block, the controller --------------------------------------------------------------------- */

static void
spi_clock_on(void)
{
	wr(cmu + CMU_GATE_PCLK_USI2, CMU_GATE_ON);
	wr(cmu + CMU_GATE_RSTNSYNC_USI2, CMU_GATE_ON);
	wr(cmu + CMU_GATE_USI2, CMU_GATE_ON);
	wr(cmu + CMU_MUX_USI2, 1);
	wr(cmu + CMU_DIV_USI2, 0);
	wr(cmu + CMU_GATE_IPCLK_USI2, CMU_GATE_ON);
}

static bool
spi_init(void)
{
	uint32_t v;

	wr(sysreg + SYSREG_USI2_MODE, USI_MODE_SPI);
	wr(spi + USI_CON, rd(spi + USI_CON) & ~USI_CON_RESET);
	delay_for_interval(10, NSEC_PER_USEC);
	v = rd(spi + USI_OPTION);
	wr(spi + USI_OPTION, (v & ~USI_OPTION_CLKSTOP_ON) | USI_OPTION_CLKREQ_ON);

	wr(spi + CH_CFG, CH_SW_RST);
	delay_for_interval(10, NSEC_PER_USEC);
	wr(spi + CH_CFG, 0);                    /* master, mode 0, channels off */
	wr(spi + MODE_CFG, 0);                  /* 8-bit, polled */
	wr(spi + CS_REG, CS_INACTIVE);          /* chip select by hand */
	wr(spi + INT_EN, 0);
	wr(spi + SWAP_CFG, 0);
	wr(spi + PENDING_CLR, 0x1f);
	wr(spi + PENDING_CLR, 0);

	/* the controller keeps what is written only with its bus clock running */
	wr(spi + MODE_CFG, 0x55u << 5);
	v = rd(spi + MODE_CFG);
	wr(spi + MODE_CFG, 0);
	return v == 0x55u << 5;
}

/* one transfer with chip select held; at most FIFO_INFLIGHT bytes on their way at a time */
static bool
spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t length)
{
	uint32_t sent = 0, got = 0;
	uint64_t start = micros_now();
	bool ok = true;

	wr(spi + CH_CFG, CH_SW_RST);
	wr(spi + CH_CFG, 0);
	wr(spi + PACKET_CNT, (length & 0xffff) | PACKET_CNT_EN);
	wr(spi + CH_CFG, CH_TXCH_ON | CH_RXCH_ON);
	wr(spi + CS_REG, 0);
	while (got < length) {
		while (sent < length && sent - got < FIFO_INFLIGHT) {
			wr(spi + TX_DATA, tx ? tx[sent] : 0);
			sent++;
		}
		for (uint32_t n = RX_LVL(rd(spi + STATUS)); n > 0 && got < length; n--) {
			uint8_t b = (uint8_t)rd(spi + RX_DATA);

			if (rx) {
				rx[got] = b;
			}
			got++;
		}
		if (micros_now() - start > 20000) {
			ok = false;
			break;
		}
	}
	wr(spi + CS_REG, CS_INACTIVE);
	wr(spi + CH_CFG, 0);
	wr(spi + PACKET_CNT, 0);
	return ok;
}

/* ---- the Goodix Berlin protocol (Google's goodix_brl_spi.c, goodix_brl_hw.c) -------------------------------------- */

#define GOODIX_FW_VERSION       0x10014         /* 28 bytes */
#define GOODIX_IC_INFO          0x10070         /* a 16-bit length, then the rest */
#define GOODIX_MAX_READ         1024
#define GOODIX_MAX_TOUCHES      10
#define GOODIX_HEADER           8
#define GOODIX_STATUS_TOUCH     0x80
#define GOODIX_STATUS_REQUEST   0x40
#define GOODIX_REQUEST_RESET    3

static uint8_t spi_tx[8 + GOODIX_MAX_READ], spi_rx[8 + GOODIX_MAX_READ];
static uint32_t touch_address, point_length = 8;
static uint32_t max_x = 1080, max_y = 2400;
static uint32_t flip;           /* boot-arg touch_flip: 1 = mirror left-right, 2 = mirror top-bottom */

static bool
goodix_read(uint32_t address, uint8_t *data, uint32_t length)
{
	if (length > GOODIX_MAX_READ) {
		return false;
	}
	spi_tx[0] = 0xf1;
	spi_tx[1] = (uint8_t)(address >> 24);
	spi_tx[2] = (uint8_t)(address >> 16);
	spi_tx[3] = (uint8_t)(address >> 8);
	spi_tx[4] = (uint8_t)address;
	spi_tx[5] = spi_tx[6] = spi_tx[7] = 0xff;
	memset(spi_tx + 8, 0, length);
	if (!spi_transfer(spi_tx, spi_rx, 8 + length)) {
		return false;
	}
	memcpy(data, spi_rx + 8, length);
	return true;
}

static bool
goodix_write(uint32_t address, const uint8_t *data, uint32_t length)
{
	if (length > GOODIX_MAX_READ) {
		return false;
	}
	spi_tx[0] = 0xf0;
	spi_tx[1] = (uint8_t)(address >> 24);
	spi_tx[2] = (uint8_t)(address >> 16);
	spi_tx[3] = (uint8_t)(address >> 8);
	spi_tx[4] = (uint8_t)address;
	memcpy(spi_tx + 5, data, length);
	return spi_transfer(spi_tx, NULL, 5 + length);
}

/* the chip's blocks end in the 16-bit sum of their bytes; all zeros or all ones is no answer at all */
static bool
goodix_checksum_ok(const uint8_t *data, uint32_t length)
{
	uint32_t sum = 0;
	bool blank = true;

	if (length < 3) {
		return false;
	}
	for (uint32_t i = 0; i < length - 2; i++) {
		sum += data[i];
		blank &= data[i] == 0 || data[i] == 0xff;
	}
	return !blank && (uint16_t)sum == (uint16_t)(data[length - 2] | data[length - 1] << 8);
}

static uint16_t
le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | p[1] << 8);
}

static uint32_t
le32(const uint8_t *p)
{
	return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

static void
touch_reset(void)
{
	wr(pin_reset + 0x44, rd(pin_reset + 0x44) & ~(1u << 2));
	sleep_ms(2);
	wr(pin_reset + 0x44, rd(pin_reset + 0x44) | (1u << 2));
	sleep_ms(100);
}

static bool
goodix_probe(void)
{
	static uint8_t info[GOODIX_MAX_READ];
	uint8_t version[28];
	uint32_t length, offset;

	if (!goodix_read(GOODIX_FW_VERSION, version, sizeof(version))) {
		printf("akita touch: the SPI transfer did not complete\n");
		return false;
	}
	if (!goodix_checksum_ok(version, sizeof(version))) {
		printf("akita touch: no valid version block (%02x %02x %02x %02x %02x %02x ... %02x %02x): is the chip powered "
		    "(U-Boot's $hakios_touch)?\n", version[0], version[1], version[2], version[3], version[4], version[5],
		    version[26], version[27]);
		return false;
	}
	printf("akita touch: chip \"%.6s\", firmware \"%.8s\" %02x.%02x.%02x.%02x\n", (const char *)version,
	    (const char *)version + 10, version[18], version[19], version[20], version[21]);

	if (!goodix_read(GOODIX_IC_INFO, info, 2)) {
		return false;
	}
	length = le16(info);
	if (length < 40 || length >= GOODIX_MAX_READ || !goodix_read(GOODIX_IC_INFO, info, length) ||
	    !goodix_checksum_ok(info, length)) {
		printf("akita touch: bad chip description (length %u)\n", length);
		return false;
	}
	/* the length, the version (16 bytes), the features (10), four counts, then five lists of 16-bit numbers */
	offset = 2 + 16 + 10 + 4;
	for (int i = 0; i < 5; i++) {
		if (offset >= length) {
			return false;
		}
		offset += 1 + 2u * info[offset];
	}
	/* then the "misc" part (122 bytes: touch_data_addr at 44, point_struct_len at 50), then the "other" (the size at 16) */
	if (offset + 122 + 20 + 2 > length) {
		printf("akita touch: chip description too short (%u, lists end at %u)\n", length, offset);
		return false;
	}
	touch_address = le32(info + offset + 44);
	point_length = le16(info + offset + 50);
	if (point_length < 8 || point_length > 32) {
		printf("akita touch: a finger's record is %u bytes? taking 8\n", point_length);
		point_length = 8;
	}
	if (le16(info + offset + 122 + 16) && le16(info + offset + 122 + 18)) {
		max_x = le16(info + offset + 122 + 16);
		max_y = le16(info + offset + 122 + 18);
	}
	printf("akita touch: report at %#x, %u bytes a finger, coordinates up to %u x %u\n", touch_address, point_length,
	    max_x, max_y);
	return touch_address != 0;
}

static struct {
	uint32_t polls, reports, touches, bad, last_x, last_y;
} seen;

static struct {
	bool down, seen;
	uint32_t x, y;
} contact[GOODIX_MAX_TOUCHES];

static void
goodix_report(const uint8_t *touches, uint32_t count)
{
	static unsigned shown;

	for (int i = 0; i < GOODIX_MAX_TOUCHES; i++) {
		contact[i].seen = false;
	}
	for (uint32_t i = 0; i < count; i++) {
		const uint8_t *t = touches + point_length * i;
		uint32_t id = t[0] >> 4;
		uint32_t x = le16(t + 2), y = le16(t + 4);

		if (id >= GOODIX_MAX_TOUCHES) {
			continue;
		}
		if (shown < 40) {
			shown++;
			printf("akita touch: finger %u at %u, %u (of %u x %u)\n", id, x, y, max_x, max_y);
		}
		seen.touches++;
		seen.last_x = x;
		seen.last_y = y;
		x = x >= max_x ? 65535 : x * 65535 / max_x;
		y = y >= max_y ? 65535 : y * 65535 / max_y;
		if (flip & 1) {
			x = 65535 - x;
		}
		if (flip & 2) {
			y = 65535 - y;
		}
		contact[id].seen = true;
		if (!contact[id].down) {
			hakios_touch_post(HAKIOS_TOUCH_DOWN, id, x, y);
		} else if (contact[id].x != x || contact[id].y != y) {
			hakios_touch_post(HAKIOS_TOUCH_MOVE, id, x, y);
		}
		contact[id].down = true;
		contact[id].x = x;
		contact[id].y = y;
	}
	for (uint32_t id = 0; id < GOODIX_MAX_TOUCHES; id++) {
		if (contact[id].down && !contact[id].seen) {
			contact[id].down = false;
			hakios_touch_post(HAKIOS_TOUCH_UP, id, contact[id].x, contact[id].y);
		}
	}
}

/* read the report if there is one, and tell the chip it has been read */
static void
goodix_poll(void)
{
	static uint8_t report[GOODIX_HEADER + 32 * GOODIX_MAX_TOUCHES + 2];
	static unsigned bad;
	const uint8_t zero = 0;
	uint32_t first = GOODIX_HEADER + 2 * point_length + 2;  /* the header and two fingers: most of the time, all */
	uint32_t count;

	seen.polls++;
	if (!goodix_read(touch_address, report, first) || report[0] == 0) {
		return;
	}
	seen.reports++;
	count = report[2] & 0xf;
	if ((report[0] & GOODIX_STATUS_TOUCH) && count > 2 && count <= GOODIX_MAX_TOUCHES &&
	    !goodix_read(touch_address + first, report + first, point_length * count + 2 - (first - GOODIX_HEADER))) {
		return;
	}
	goodix_write(touch_address, &zero, 1);
	if (!goodix_checksum_ok(report, GOODIX_HEADER)) {
		seen.bad++;
		if (bad++ < 8) {
			printf("akita touch: bad report header: %02x %02x %02x %02x %02x %02x %02x %02x\n", report[0], report[1],
			    report[2], report[3], report[4], report[5], report[6], report[7]);
		}
		return;
	}
	if (report[0] & GOODIX_STATUS_REQUEST) {
		if ((report[2] & 0xf) == GOODIX_REQUEST_RESET || report[2] == GOODIX_REQUEST_RESET) {
			printf("akita touch: the chip asks to be reset\n");
			touch_reset();
		}
		return;
	}
	if (!(report[0] & GOODIX_STATUS_TOUCH)) {
		return;
	}
	if (count > GOODIX_MAX_TOUCHES ||
	    (count && !goodix_checksum_ok(report + GOODIX_HEADER, count * point_length + 2))) {
		seen.bad++;
		if (bad++ < 8) {
			printf("akita touch: bad report (%u touches)\n", count);
		}
		return;
	}
	goodix_report(report + GOODIX_HEADER, count);
}

/* ---- the thread -------------------------------------------------------------------------------------------------- */

static bool
interrupt_line_low(void)
{
	return (rd(pin_int + 4) & 1u) == 0;
}

static void
akita_touch_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint32_t poll_us = 4000, idle = 0, idle_looks;     /* 250 a second: about two looks in each 120 Hz refresh */
	bool found = false;

	PE_parse_boot_argn("touch_poll_us", &poll_us, sizeof(poll_us));
	if (poll_us < 250 || poll_us > 100000) {
		poll_us = 4000;
	}
	idle_looks = 80000 / poll_us;

	spi_clock_on();
	if (!spi_init()) {
		printf("akita touch: the SPI controller does not keep what is written: its clock is off\n");
		thread_terminate(current_thread());
		return;
	}
	for (int attempt = 0; attempt < 3 && !found; attempt++) {
		if (attempt == 2) {
			printf("akita touch: resetting the chip\n");
			touch_reset();
		}
		found = goodix_probe();
		if (!found) {
			sleep_ms(200);
		}
	}
	if (!found) {
		printf("akita touch: no touch screen\n");
		thread_terminate(current_thread());
		return;
	}
	printf("akita touch: ready; the interrupt line is looked at every %u us\n", poll_us);
	for (unsigned tick = 1;; tick++) {
		if (interrupt_line_low()) {
			idle = 0;
			goodix_poll();
		} else if (++idle >= idle_looks) {
			idle = 0;
			goodix_poll();
		}
		if (tick % (60 * (1000000 / poll_us)) == 0) {
			printf("akita touch: so far %u reports in %u looks (%u bad), %u finger positions, the last at %u, %u; "
			    "interrupt line %s\n", seen.reports, seen.polls, seen.bad, seen.touches, seen.last_x, seen.last_y,
			    interrupt_line_low() ? "low" : "high");
		}
		delay_for_interval(poll_us, NSEC_PER_USEC);
	}
}

static void
akita_touch_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(akita_touch_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("akita touch: no thread\n");
	}
}

__startup_func
static void
akita_touch_init(void)
{
	uint32_t enabled = 0;
	uint64_t deadline;
	static thread_call_t start;

	if (!PE_parse_boot_argn("touchscreen", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	PE_parse_boot_argn("touch_flip", &flip, sizeof(flip));
	spi = ml_io_map(SPI_PHYS, 0x1000);
	cmu = ml_io_map(CMU_PHYS, 0x2000);
	sysreg = ml_io_map(SYSREG_PHYS, 0x1000);
	pin_int = ml_io_map(PIN_INT_PHYS, 0x1000);
	pin_reset = ml_io_map(PIN_RESET_PHYS, 0x1000);
	if (!spi || !cmu || !sysreg || !pin_int || !pin_reset) {
		printf("akita touch: could not map its registers; off\n");
		return;
	}
	/* a thread of its own once the kernel is up: started from a thread call, a second from now */
	start = thread_call_allocate(akita_touch_start, NULL);
	clock_interval_to_deadline(1, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_touch_init);
