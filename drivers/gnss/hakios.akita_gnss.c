/*
 * The Pixel 8a's GNSS receiver: Samsung SLSI's "Kepler" (gps.cfg's K041), a chip of its own on zuma's spi@111E0000 (USI3
 * of HSI0), powered and code-loaded by the modem (hakios.akita_modem.c: gnss_boot_run, channel 240) and then spoken to
 * over SPI. It computes the fix itself: what it sends once a second holds the position, the time and the satellites, so
 * the phone needs no positioning engine -- only the link, the transport and the messages that start a session.
 *
 * Only with boot-arg gnss=N (and modem=9: the modem starts the chip). Steps:
 *   1  the pins (AP2GNSS gpp4-3, an output; GNSS2AP gpa5-3, an input) and the SPI controller (USI3's clock on the
 *      24.576 MHz crystal divided by 4 -- gnss_spidiv=N-1; undivided, as the touch screen's, the chip's frames came back
 *      with bits wrong and no CRC held (11-20) -- the USI block in SPI mode, Samsung's s3c64xx polled, 8-bit, mode 0)
 *   2  waits for the modem to have started the chip, then CHPP (Android's open Context Hub peripheral protocol,
 *      system/chre/chpp): the transport's reset, discovery (one service, "BETP"), time sync, the service opened
 *   3  gnssd's own start messages (fw/gnss-start.bin: { u32 ms to wait, u16 length, the service's payload }, taken out
 *      of a trace of gnssd, 11-20) sent as it sent them; then everything the chip says read, acknowledged, counted, and
 *      its fixes kept in kern.hakios.gnss_* -- THE POSITION IS NEVER WRITTEN TO THE LOG
 *
 * The link (Google's drivers/soc/google/gnssif_spi): the chip raises GNSS2AP when it has something; the phone pulses
 * AP2GNSS for 100 us and reads 64-byte pieces while GNSS2AP stays high. To send, the phone raises AP2GNSS, waits for
 * GNSS2AP (the chip ready), clocks the whole message (padded to 4 bytes) full duplex -- what comes back is the chip's --
 * and lowers AP2GNSS. Android's driver uses 32-bit words with the bytes swapped back: on the wire the same as 8-bit.
 *
 * CHPP (as seen and as the open source has it): a frame is 68 43, { u8 flags (bit 0: more of this datagram follows),
 * u8 packet code (0x10 reset, 0x20 reset's ack; low nibble an error), u8 ack (the next sequence number expected),
 * u8 sequence, u16 length, u16 0 }, the payload, a CRC-32 (IEEE) of header and payload. An application payload starts
 * { u8 handle, u8 type (0 request, 1 response, 2 client notification, 3 service notification), u8 transaction,
 * u8 error, u16 command }. The receiver's service is handle 0x10: the phone's messages are notifications 2 / command 2,
 * the chip's 3 / 3, each carrying Samsung's messages { u8 class, u8 id, u16 2, ..., u16 header size (20 the phone's,
 * 16 the chip's), u16 length } + data, several in one datagram.
 *
 * The chip's fix, class 0x43 id 1 (160 bytes, once a second): u32 flags (bit 0: no fix), +8 u16 GPS week, +10 u32 time
 * of week (ms), +16 u32 satellites used, +20 u8 kind, +22 the UTC date and time (u16 year, u8 month, day, hour,
 * minute, u16 milliseconds of the minute), +48 i32 latitude, +52 i32 longitude (1e-7 degrees), +56 u32 an accuracy
 * (its unit to be confirmed). The rest is not yet read.
 */
#include <stdint.h>
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/sched_prim.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t thread_terminate(thread_t thread);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
extern int akita_gnss_state;            /* hakios.akita_modem.c: 2 once the modem has started the chip, -1 failed */

/* ---- the addresses ------------------------------------------------------------------------------------------------ */

#define SPI_PHYS                0x111e0000ULL   /* spi@111E0000, USI3 of HSI0 */
#define CMU_PHYS                0x11001000ULL   /* CMU_HSI0 (0x11000000) from its muxes on */
#define CMU_MUX_USI3            0x020           /* CLK_CON_MUX_MUX_CLK_HSI0_USI3: 1 = the crystal */
#define CMU_DIV_USI3            0x820
#define CMU_GATE_RSTNSYNC_USI3  0x10d0
#define CMU_GATE_IPCLK_USI3     0x1104
#define CMU_GATE_PCLK_USI3      0x1108
#define CMU_GATE_USI3           0x1124
#define CMU_GATE_ON             0x00300000      /* MANUAL and CG_VAL: held on */
#define SYSREG_PHYS             0x11021000ULL   /* sysreg_hsi0 */
#define SYSREG_USI3_MODE        0x020           /* samsung,usi-offset = <0x1020> */
#define USI_MODE_SPI            0x2
#define PIN_PERIC0_PHYS         0x10840000ULL   /* pinctrl@10840000: gpp4 at +0x80 */
#define PIN_FAR_ALIVE_PHYS      0x154e0000ULL   /* pinctrl@154E0000: gpa5 at +0 (its pin 1 is Wi-Fi's host wake) */
#define GPP4                    0x80
#define GPA5                    0x00
#define PIN_AP2GNSS             3               /* gpp4-3 */
#define PIN_GNSS2AP             3               /* gpa5-3 */

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
#define FIFO_INFLIGHT           32

static vm_offset_t spi, cmu, sysreg, pin_peric0, pin_far;
static uint32_t steps, spi_div = 3, spi_fb;      /* gnss_spidiv: USI3's divider less one; gnss_fb: the feedback clock's delay */

static inline uint32_t rd(vm_offset_t a) { return *(volatile uint32_t *)a; }
static inline void wr(vm_offset_t a, uint32_t v) { *(volatile uint32_t *)a = v; __asm__ volatile ("dsb sy" ::: "memory"); }

static void
sleep_ms(uint32_t ms)
{
	assert_wait_timeout((event_t)&steps, THREAD_UNINT, ms, NSEC_PER_MSEC);
	thread_block(THREAD_CONTINUE_NULL);
}

static uint64_t
now_ms(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000000;
}

static uint64_t
now_us(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000;
}

/* ---- what iOS reads ------------------------------------------------------------------------------------------------ */

static int gnss_state;                  /* 0 off, 1 the link set up, 2 CHPP open, 3 the session started */
static int gnss_fixes, gnss_valid, gnss_sv, gnss_lat, gnss_lon, gnss_acc, gnss_flags, gnss_kind;
static int gnss_utc_date, gnss_utc_time;        /* yyyymmdd, hhmmssmmm (with the milliseconds) */
static int gnss_fix_seq;                /* counts every fix read (valid or not): a reader sees a new one by it */

SYSCTL_DECL(_kern_hakios);
SYSCTL_INT(_kern_hakios, OID_AUTO, gnss_state, CTLFLAG_RD | CTLFLAG_LOCKED, &gnss_state, 0,
    "the GNSS receiver: 0 off, 1 the link set up, 2 CHPP open, 3 the session started");
SYSCTL_INT(_kern_hakios, OID_AUTO, gnss_fix_seq, CTLFLAG_RD | CTLFLAG_LOCKED, &gnss_fix_seq, 0, "fix reports read");
SYSCTL_INT(_kern_hakios, OID_AUTO, gnss_valid, CTLFLAG_RD | CTLFLAG_LOCKED, &gnss_valid, 0, "1: the last report is a fix");
SYSCTL_INT(_kern_hakios, OID_AUTO, gnss_sv, CTLFLAG_RD | CTLFLAG_LOCKED, &gnss_sv, 0, "satellites used");
SYSCTL_INT(_kern_hakios, OID_AUTO, gnss_lat, CTLFLAG_RD | CTLFLAG_LOCKED, &gnss_lat, 0, "latitude, 1e-7 degrees");
SYSCTL_INT(_kern_hakios, OID_AUTO, gnss_lon, CTLFLAG_RD | CTLFLAG_LOCKED, &gnss_lon, 0, "longitude, 1e-7 degrees");
SYSCTL_INT(_kern_hakios, OID_AUTO, gnss_acc, CTLFLAG_RD | CTLFLAG_LOCKED, &gnss_acc, 0, "the report's accuracy field (unit: TBD)");
SYSCTL_INT(_kern_hakios, OID_AUTO, gnss_flags, CTLFLAG_RD | CTLFLAG_LOCKED, &gnss_flags, 0, "the report's flags (bit 0: no fix)");
SYSCTL_INT(_kern_hakios, OID_AUTO, gnss_utc_date, CTLFLAG_RD | CTLFLAG_LOCKED, &gnss_utc_date, 0, "the report's UTC date, yyyymmdd");
SYSCTL_INT(_kern_hakios, OID_AUTO, gnss_utc_time, CTLFLAG_RD | CTLFLAG_LOCKED, &gnss_utc_time, 0, "the report's UTC time, hhmmssmmm");

/* ---- the pins, the clock, the controller --------------------------------------------------------------------------- */

static void
pin_mode(vm_offset_t bank, unsigned pin, uint32_t con, uint32_t pud)
{
	wr(bank + 0x0, (rd(bank + 0x0) & ~(0xfu << (4 * pin))) | con << (4 * pin));
	wr(bank + 0x8, (rd(bank + 0x8) & ~(0xfu << (4 * pin))) | pud << (4 * pin));
}

static void
ap2gnss(int v)
{
	vm_offset_t dat = pin_peric0 + GPP4 + 0x4;

	wr(dat, v ? rd(dat) | 1u << PIN_AP2GNSS : rd(dat) & ~(1u << PIN_AP2GNSS));
}

static int
gnss2ap(void)
{
	return (rd(pin_far + GPA5 + 0x4) >> PIN_GNSS2AP) & 1;
}

static bool
spi_setup(void)
{
	uint32_t v;

	wr(cmu + CMU_GATE_PCLK_USI3, CMU_GATE_ON);
	wr(cmu + CMU_GATE_RSTNSYNC_USI3, CMU_GATE_ON);
	wr(cmu + CMU_GATE_USI3, CMU_GATE_ON);
	wr(cmu + CMU_MUX_USI3, 1);
	wr(cmu + CMU_DIV_USI3, spi_div & 0xf);
	wr(cmu + CMU_GATE_IPCLK_USI3, CMU_GATE_ON);
	wr(sysreg + SYSREG_USI3_MODE, USI_MODE_SPI);
	wr(spi + USI_CON, rd(spi + USI_CON) & ~USI_CON_RESET);
	delay_for_interval(10, NSEC_PER_USEC);
	v = rd(spi + USI_OPTION);
	wr(spi + USI_OPTION, (v & ~USI_OPTION_CLKSTOP_ON) | USI_OPTION_CLKREQ_ON);
	wr(spi + CH_CFG, CH_SW_RST);
	delay_for_interval(10, NSEC_PER_USEC);
	wr(spi + CH_CFG, 0);
	wr(spi + MODE_CFG, 0);
	wr(spi + CS_REG, CS_INACTIVE);
	wr(spi + INT_EN, 0);
	wr(spi + SWAP_CFG, 0);
	wr(spi + 0x2c, spi_fb & 3);             /* FB_CLK: when the input is sampled (the tree's feedback delay: 0) */
	wr(spi + PENDING_CLR, 0x1f);
	wr(spi + PENDING_CLR, 0);
	wr(spi + MODE_CFG, 0x55u << 5);
	v = rd(spi + MODE_CFG);
	wr(spi + MODE_CFG, 0);
	return v == 0x55u << 5;
}

/* one transfer, chip select held; tx NULL: zeros out */
static bool
spi_transfer(const uint8_t *tx, uint8_t *rx, uint32_t length)
{
	uint32_t sent = 0, got = 0;
	uint64_t start = now_us();
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
		if (now_us() - start > 50000) {
			ok = false;
			break;
		}
	}
	wr(spi + CS_REG, CS_INACTIVE);
	wr(spi + CH_CFG, 0);
	wr(spi + PACKET_CNT, 0);
	return ok;
}

/* ---- CHPP ------------------------------------------------------------------------------------------------------------ */

static uint32_t crc_table[256];

static uint32_t
crc32_ieee(const uint8_t *b, uint32_t n)
{
	uint32_t c = 0xffffffffu;

	if (crc_table[1] == 0) {
		for (uint32_t i = 0; i < 256; i++) {
			uint32_t x = i;

			for (int k = 0; k < 8; k++) {
				x = (x & 1) ? 0xedb88320u ^ (x >> 1) : x >> 1;
			}
			crc_table[i] = x;
		}
	}
	for (uint32_t i = 0; i < n; i++) {
		c = crc_table[(c ^ b[i]) & 0xff] ^ (c >> 8);
	}
	return ~c;
}

#define STREAM_SIZE     16384
#define DGRAM_SIZE      16384
static uint8_t *stream, *dgram, *txbuf, *rxbuf;
static uint32_t stream_len, dgram_len;
static uint8_t tx_seq, rx_next, chip_ack;
static bool ack_owed;
static uint32_t frames_in, frames_bad_seq, frames_out, resets_in, class_count[256];
static uint32_t app_responses;          /* responses to our requests (discovery, time sync, open) */

static void link_send(const uint8_t *frame, uint32_t len);

static void
chpp_frame_out(uint8_t flags, uint8_t code, uint8_t seq, const uint8_t *payload, uint32_t n)
{
	static uint8_t f[1400 + 16];
	uint32_t c;

	if (n > 1400) {
		return;
	}
	f[0] = 0x68;
	f[1] = 0x43;
	f[2] = flags;
	f[3] = code;
	f[4] = rx_next;
	f[5] = seq;
	f[6] = (uint8_t)n;
	f[7] = (uint8_t)(n >> 8);
	f[8] = f[9] = 0;
	if (n != 0) {
		memcpy(f + 10, payload, n);
	}
	c = crc32_ieee(f + 2, 8 + n);
	memcpy(f + 10 + n, &c, 4);
	frames_out++;
	ack_owed = false;
	link_send(f, 14 + n);
}

static void
fix_read(const uint8_t *b, uint32_t n)
{
	uint32_t flags, sv, acc;
	int32_t lat, lon;
	int was_valid = gnss_valid;

	if (n < 64) {
		return;
	}
	memcpy(&flags, b, 4);
	memcpy(&sv, b + 16, 4);
	memcpy(&lat, b + 48, 4);
	memcpy(&lon, b + 52, 4);
	memcpy(&acc, b + 56, 4);
	gnss_flags = (int)flags;
	gnss_kind = b[20];
	gnss_sv = (int)sv;
	gnss_acc = (int)acc;
	gnss_utc_date = (b[22] | b[23] << 8) * 10000 + b[24] * 100 + b[25];
	gnss_utc_time = b[26] * 10000000 + b[27] * 100000 + (b[28] | b[29] << 8);
	gnss_valid = (flags & 1) == 0 && sv >= 3 && (lat != 0 || lon != 0);
	if (gnss_valid) {
		gnss_lat = lat;
		gnss_lon = lon;
		gnss_fixes++;
	}
	gnss_fix_seq++;
	/* (the log gets the fix's quality only: never its position) */
	if (gnss_fix_seq <= 3 || gnss_valid != was_valid || gnss_fix_seq % 60 == 0) {
		printf("gnss: report %d: %s, flags 0x%x, kind %u, %u satellites, accuracy field %u, UTC %08d %09d\n", gnss_fix_seq,
		    gnss_valid ? "A FIX" : "no fix", flags, b[20], sv, acc, gnss_utc_date, gnss_utc_time);
	}
}

static void
betp_in(const uint8_t *q, uint32_t n)
{
	uint32_t i = 0;

	while (i + 16 <= n && q[i + 2] == 2 && q[i + 3] == 0) {
		uint32_t hs = q[i + 12] | q[i + 13] << 8, len = q[i + 14] | q[i + 15] << 8;

		if (hs != 16 || i + 16 + len > n) {
			break;
		}
		if (class_count[q[i]]++ == 0) {
			printf("gnss: the chip's first message of class 0x%02x (id 0x%02x, %u bytes)\n", q[i], q[i + 1], len);
		}
		if (q[i] == 0x43 && q[i + 1] == 0x01) {
			fix_read(q + i + 16, len);
		}
		i += 16 + len;
	}
}

static void
chpp_frame_in(const uint8_t *h, const uint8_t *payload, uint32_t n)
{
	uint8_t flags = h[0], code = h[1], ack = h[2], seq = h[3];

	frames_in++;
	chip_ack = ack;
	if ((code & 0xf0) == 0x10) {
		resets_in++;
		if (resets_in <= 3) {
			printf("gnss: the chip asks for a transport reset (code 0x%02x)\n", code);
		}
		return;
	}
	if (n == 0) {
		return;
	}
	if (seq != rx_next) {
		frames_bad_seq++;
		ack_owed = true;        /* (a resend of one we have: our ack again) */
		return;
	}
	rx_next = seq + 1;
	ack_owed = true;
	if ((code & 0xf0) == 0x20) {
		return;                 /* the reset's ack (its payload: the chip's configuration) */
	}
	if (dgram_len + n <= DGRAM_SIZE) {
		memcpy(dgram + dgram_len, payload, n);
		dgram_len += n;
	}
	if (flags & 1) {
		return;
	}
	if (dgram_len >= 6 && dgram[0] == 0x10 && dgram[1] == 3) {
		betp_in(dgram + 6, dgram_len - 6);
	} else if (dgram_len >= 6) {
		app_responses++;
		printf("gnss: CHPP handle 0x%02x type %u command %u: %u bytes%s\n", dgram[0], dgram[1], dgram[4] | dgram[5] << 8,
		    dgram_len, dgram_len >= 26 && dgram[0] == 0x0f ? " (discovery: the service's name follows its id)" : "");
	}
	dgram_len = 0;
}

static void
stream_in(const uint8_t *b, uint32_t n)
{
	uint32_t i = 0;

	if (stream_len + n > STREAM_SIZE) {
		stream_len = 0;
	}
	memcpy(stream + stream_len, b, n);
	stream_len += n;
	while (i + 14 <= stream_len) {
		uint32_t len, c;

		if (stream[i] != 0x68 || stream[i + 1] != 0x43) {
			i++;
			continue;
		}
		len = stream[i + 6] | stream[i + 7] << 8;
		if (len > 1400) {
			i++;
			continue;
		}
		if (i + 14 + len > stream_len) {
			break;
		}
		memcpy(&c, stream + i + 10 + len, 4);
		if (c != crc32_ieee(stream + i + 2, 8 + len)) {
			i++;
			continue;
		}
		chpp_frame_in(stream + i + 2, stream + i + 10, len);
		i += 14 + len;
	}
	memmove(stream, stream + i, stream_len - i);
	stream_len -= i;
}

/* ---- the link ------------------------------------------------------------------------------------------------------ */

static uint32_t link_timeouts;

/* a message to the chip: AP2GNSS up, the chip's ready (GNSS2AP), the whole message full duplex, AP2GNSS down */
static void
link_send(const uint8_t *frame, uint32_t len)
{
	uint32_t padded = (len + 3) & ~3u;
	uint64_t until;

	if (padded > 4096) {
		return;
	}
	memcpy(txbuf, frame, len);
	bzero(txbuf + len, padded - len);
	ap2gnss(1);
	until = now_us() + 500000;
	while (!gnss2ap() && now_us() < until) {
		delay_for_interval(20, NSEC_PER_USEC);
	}
	if (!gnss2ap()) {
		if (link_timeouts++ < 5) {
			printf("gnss: the chip did not say it was ready (GNSS2AP low 500 ms after AP2GNSS)\n");
		}
		ap2gnss(0);
		return;
	}
	{
		uint64_t t0 = now_us();
		bool ok = spi_transfer(txbuf, rxbuf, padded);
		static uint32_t said;

		ap2gnss(0);
		/* (the first exchanges said in full: what the chip clocks back while we send) */
		if (said < 4) {
			said++;
			printf("gnss: link: sent %u bytes in %llu us (%s); back: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x "
			    "%02x %02x %02x %02x; GNSS2AP now %d\n", padded, now_us() - t0, ok ? "done" : "TIMED OUT", rxbuf[0], rxbuf[1],
			    rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5], rxbuf[6], rxbuf[7], rxbuf[8], rxbuf[9], rxbuf[10], rxbuf[11], rxbuf[12],
			    rxbuf[13], rxbuf[14], rxbuf[15], gnss2ap());
		}
	}
	stream_in(rxbuf, padded);
}

/* what the chip has: the pulse, then 64-byte pieces while GNSS2AP stays high (2 KB at most at a time) */
static void
link_receive(void)
{
	uint32_t got = 0;

	if (!gnss2ap()) {
		return;
	}
	ap2gnss(1);
	delay_for_interval(100, NSEC_PER_USEC);
	ap2gnss(0);
	do {
		(void)spi_transfer(NULL, rxbuf + got, 64);
		got += 64;
	} while (gnss2ap() && got + 64 <= 2048);
	{
		static uint32_t said;

		if (said < 4) {
			said++;
			printf("gnss: link: the chip asked; %u bytes read: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x "
			    "%02x %02x\n", got, rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5], rxbuf[6], rxbuf[7], rxbuf[8],
			    rxbuf[9], rxbuf[10], rxbuf[11], rxbuf[12], rxbuf[13], rxbuf[14], rxbuf[15]);
		}
	}
	stream_in(rxbuf, got);
}

static void
link_poll(uint32_t ms)
{
	uint64_t until = now_ms() + ms;

	do {
		link_receive();
		if (ack_owed) {
			chpp_frame_out(0, 0, 0, NULL, 0);
		}
		if (now_ms() >= until) {
			break;
		}
		sleep_ms(2);
	} while (now_ms() < until);
}

/* a datagram (one frame: all ours fit the chip's 1280) sent until the chip acknowledges it, four tries */
static bool
chpp_send(const uint8_t *payload, uint32_t n, const char *what)
{
	uint8_t want = (uint8_t)(tx_seq + 1);

	for (int tries = 0; tries < 4; tries++) {
		chpp_frame_out(0, 0, tx_seq, payload, n);
		for (int k = 0; k < 40 && chip_ack != want; k++) {
			link_poll(25);
		}
		if (chip_ack == want) {
			tx_seq++;
			return true;
		}
	}
	printf("gnss: %s: not acknowledged (the chip's ack %u, ours %u)\n", what, chip_ack, tx_seq);
	tx_seq++;
	return false;
}

/* ---- the run ------------------------------------------------------------------------------------------------------- */

static void
gnss_run(void)
{
	static const uint8_t config[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01, 0x00, 0xe8, 0x03 }; /* 1.0.0, MTU 1280, window 1, 1 s */
	static const uint8_t discover[] = { 0x0f, 0, 0, 0, 0x01, 0 }, timesync[] = { 0x02, 0, 0, 0, 0x01, 0 },
	    open_[] = { 0x10, 0, 0, 0, 0, 0 };
	const uint8_t *script;
	uint64_t script_size = 0, t;
	static uint8_t payload[1400];

	stream = kalloc_data(STREAM_SIZE, Z_WAITOK | Z_ZERO);
	dgram = kalloc_data(DGRAM_SIZE, Z_WAITOK | Z_ZERO);
	txbuf = kalloc_data(4096, Z_WAITOK | Z_ZERO);
	rxbuf = kalloc_data(4096, Z_WAITOK | Z_ZERO);
	if (!stream || !dgram || !txbuf || !rxbuf) {
		return;
	}
	pin_mode(pin_peric0 + GPP4, PIN_AP2GNSS, 1, 0);         /* an output, low (Android's gnss_spi_int) */
	ap2gnss(0);
	pin_mode(pin_far + GPA5, PIN_GNSS2AP, 0, 1);            /* an input, pulled down (Android: its interrupt) */
	if (!spi_setup()) {
		printf("gnss: 1: the SPI controller keeps nothing written (its clock?): off\n");
		return;
	}
	gnss_state = 1;
	printf("gnss: 1: the link set up (GNSS2AP %d; the crystal divided by %u, feedback delay %u)\n", gnss2ap(), spi_div + 1, spi_fb);
	if (steps < 2) {
		return;
	}
	t = now_ms();
	while (akita_gnss_state != 2) {
		if (akita_gnss_state < 0 || now_ms() - t > 300000) {
			printf("gnss: 2: the modem did not start the receiver (state %d): off\n", akita_gnss_state);
			return;
		}
		sleep_ms(100);
	}
	{
		uint32_t highs = 0;

		for (int k = 0; k < 100; k++) {
			highs += (uint32_t)gnss2ap();
			sleep_ms(3);
		}
		printf("gnss: 2: the modem has started the chip (GNSS2AP high in %u of 100 looks over 0.3 s; the pins: gpp4 CON 0x%08x DAT 0x%02x, "
		    "gpa5 CON 0x%08x DAT 0x%02x PUD 0x%08x)\n", highs, rd(pin_peric0 + GPP4), rd(pin_peric0 + GPP4 + 4) & 0xff, rd(pin_far + GPA5),
		    rd(pin_far + GPA5 + 4) & 0xff, rd(pin_far + GPA5 + 8));
	}
	link_poll(200);
	stream_len = 0;
	for (int k = 0; k < 10 && chip_ack != 1; k++) {
		rx_next = 0;
		chpp_frame_out(0, 0x10, 0, config, sizeof(config));
		link_poll(500);
	}
	if (chip_ack != 1) {
		printf("gnss: 2: no answer to the transport's reset (%u frames in, %u resets asked): off\n", frames_in, resets_in);
		return;
	}
	tx_seq = 1;
	(void)chpp_send(discover, sizeof(discover), "discovery");
	link_poll(200);
	(void)chpp_send(timesync, sizeof(timesync), "time sync");
	link_poll(200);
	if (!chpp_send(open_, sizeof(open_), "the service's opening")) {
		return;
	}
	link_poll(300);
	gnss_state = 2;
	printf("gnss: 2: CHPP open (%u responses)\n", app_responses);
	if (steps < 3) {
		return;
	}
	script = hakios_image_file("fw", "gnss-start.bin", &script_size, NULL);
	if (script == NULL) {
		printf("gnss: 3: no fw/gnss-start.bin in the image\n");
	} else {
		uint32_t sent = 0;

		for (uint64_t o = 0; o + 6 <= script_size;) {
			uint32_t wait;
			uint16_t n;

			memcpy(&wait, script + o, 4);
			memcpy(&n, script + o + 4, 2);
			if (o + 6 + n > script_size || 6u + n > sizeof(payload)) {
				break;
			}
			link_poll(wait > 3000 ? 3000 : wait);
			memcpy(payload, "\x10\x02\x00\x00\x02\x00", 6);
			memcpy(payload + 6, script + o + 6, n);
			(void)chpp_send(payload, 6u + n, "a start message");
			sent++;
			o += 6u + n;
		}
		printf("gnss: 3: gnssd's %u start messages sent\n", sent);
	}
	gnss_state = 3;
	for (uint64_t said = now_ms();;) {
		link_poll(1000);
		if (now_ms() - said >= 60000) {
			said = now_ms();
			printf("gnss: a minute: %u frames in (%u out of order), %u out, %u resets asked, %u timeouts; %d reports, %d fixes\n",
			    frames_in, frames_bad_seq, frames_out, resets_in, link_timeouts, gnss_fix_seq, gnss_fixes);
		}
	}
}

static void
gnss_thread(__unused void *a, __unused wait_result_t w)
{
	gnss_run();
	thread_terminate(current_thread());
}

static void
gnss_start(__unused thread_call_param_t a, __unused thread_call_param_t b)
{
	thread_t thread;

	if (kernel_thread_start(gnss_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

__startup_func
static void
akita_gnss_init(void)
{
	static thread_call_t call;
	uint64_t deadline;

	if (!PE_parse_boot_argn("gnss", &steps, sizeof(steps)) || steps == 0) {
		return;
	}
	PE_parse_boot_argn("gnss_spidiv", &spi_div, sizeof(spi_div));
	PE_parse_boot_argn("gnss_fb", &spi_fb, sizeof(spi_fb));
	spi = ml_io_map(SPI_PHYS, 0x1000);
	cmu = ml_io_map(CMU_PHYS, 0x2000);
	sysreg = ml_io_map(SYSREG_PHYS, 0x1000);
	pin_peric0 = ml_io_map(PIN_PERIC0_PHYS, 0x1000);
	pin_far = ml_io_map(PIN_FAR_ALIVE_PHYS, 0x1000);
	if (!spi || !cmu || !sysreg || !pin_peric0 || !pin_far) {
		printf("gnss: could not map its registers: off\n");
		return;
	}
	printf("gnss: steps 1 to %u (the chip is started by the modem: modem=9)\n", steps);
	call = thread_call_allocate(gnss_start, NULL);
	clock_interval_to_deadline(5, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_gnss_init);
