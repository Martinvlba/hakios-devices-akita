/*
 * The Pixel 8a's Bluetooth: the BCM4383 (the Wi-Fi chip's other half) speaks HCI in H4 on a UART of its own --
 * Android's device tree: odm/btbcm "goog,nitrous" and serial@155D0000 (uart18: a USI of the APM block in UART mode,
 * four pins, RTS/CTS). So far a look, with boot-arg bt=1:
 *
 *   1. what the bootloader left: the UART's clocks (CMU_APM), the USI's mode, the pins, the UART's registers;
 *   2. the clocks on, the USI a UART, the pins the UART's;
 *   3. the UART's source clock MEASURED (Android's clk_summary cannot be read: reading it panics Android's kernel):
 *      the UART in loop-back at a known divisor sends a FIFO's worth and the time it takes says the rate;
 *   4. 115200 8N1 with automatic RTS/CTS, the chip's BT_REG_ON low then high, device-wake held;
 *   5. HCI_Reset, Broadcom's Read_Chip_Id (0xfc79) and Read_Local_Version_Information, each answer said.
 *
 * What Android does after that (its HAL's firmware_config.json): the baud rate up (0xfc18, 4 000 000), the fast
 * download switched on (0xfe72), Download_Minidriver (0xfc2e, 50 ms), the .hcd's records sent as they are (each is
 * an HCI command), Launch_RAM (0xfc4e, 250 ms), then the low-power mode (0xfc27) and the address (0xfc01). Not here yet.
 *
 * Registers: Linux's drivers/tty/serial/samsung_tty.c; the clocks: Google's cal-if zuma cmucal-sfr.c / cmucal-node.c
 * (DIV_CLK_APM_USI1_UART under GATE_CLKCMU_APM_FUNC; the gates of APM_USI1_UART_INT); the pins: pinctrl-gs.c's zuma
 * tables (gpa3 in GPIO_ALIVE 0x154D0000 at 0x60; gpn6/8/9 in GPIO_CUSTOM_ALIVE 0x15060000 at 0xc0/0x100/0x120).
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_time.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define CMU_APM_PHYS            0x15400000ULL
#define CMU_MUX_APM_FUNC        0x1000
#define CMU_MUX_APM_FUNCSRC     0x1004
#define CMU_DIV_USI1_UART       0x1818          /* ratio in bits 0-6, busy bit 16 */
#define CMU_GATE_IPCLK          0x20f4
#define CMU_GATE_PCLK           0x20f8
#define CMU_GATE_RSTNSYNC       0x2148
#define CMU_QCH                 0x306c
#define CMU_GATE_ON             0x00300000
#define SYSREG_APM_PHYS         0x15420000ULL
#define SYSREG_USI_MODE         0x4e4           /* syscon@154204E0, samsung,usi-offset 4 */
#define USI_MODE_UART           0x1
#define GPIO_ALIVE_PHYS         0x154d0000ULL
#define GPA3                    0x060           /* pins 0-3: the UART's (function 2) */
#define GPIO_CUSTOM_PHYS        0x15060000ULL
#define GPN6                    0x0c0           /* pin 0: host wake (the chip's, an input) */
#define GPN8                    0x100           /* pin 0: BT_REG_ON */
#define GPN9                    0x120           /* pin 0: device wake */
#define PIN_CON                 0x0
#define PIN_DAT                 0x4
#define PIN_PUD                 0x8
#define UART_PHYS               0x155d0000ULL
#define ULCON                   0x00
#define UCON                    0x04
#define UFCON                   0x08
#define UMCON                   0x0c
#define UTRSTAT                 0x10
#define UERSTAT                 0x14
#define UFSTAT                  0x18
#define UMSTAT                  0x1c
#define UTXH                    0x20
#define URXH                    0x24
#define UBRDIV                  0x28
#define UFRACVAL                0x2c
#define UINTP                   0x30
#define UINTM                   0x38
#define USI_CON                 0xc4
#define USI_OPTION              0xc8
#define UTRSTAT_TX_EMPTY        (1u << 2)       /* the shifter too */
#define UFSTAT_RX_COUNT(v)      ((v) & 0x1ffu)
#define UFSTAT_TX_FULL          (1u << 24)
#define UCON_LOOPBACK           (1u << 5)

static vm_offset_t cmu, sysreg, gpio_alive, gpio_custom, uart;
static uint32_t src_hz;

static uint32_t rd(vm_offset_t a) { return *(volatile uint32_t *)a; }
static void wr(vm_offset_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

static void
pin_set(vm_offset_t bank, unsigned pin, uint32_t function, int pud, int level)
{
	if (level >= 0) {
		wr(bank + PIN_DAT, (rd(bank + PIN_DAT) & ~(1u << pin)) | ((uint32_t)level << pin));
	}
	if (pud >= 0) {
		wr(bank + PIN_PUD, (rd(bank + PIN_PUD) & ~(0xfu << (4 * pin))) | ((uint32_t)pud << (4 * pin)));
	}
	wr(bank + PIN_CON, (rd(bank + PIN_CON) & ~(0xfu << (4 * pin))) | (function << (4 * pin)));
}

static void
say_state(const char *when)
{
	printf("akita bt: %s: CMU_APM mux %#x/%#x div %#x gates %#x/%#x/%#x qch %#x; USI mode %#x; gpa3 con %#x dat %#x pud %#x; "
	    "gpn6 %#x/%#x gpn8 %#x/%#x gpn9 %#x/%#x\n", when, rd(cmu + CMU_MUX_APM_FUNC), rd(cmu + CMU_MUX_APM_FUNCSRC),
	    rd(cmu + CMU_DIV_USI1_UART), rd(cmu + CMU_GATE_IPCLK), rd(cmu + CMU_GATE_PCLK), rd(cmu + CMU_GATE_RSTNSYNC),
	    rd(cmu + CMU_QCH), rd(sysreg + SYSREG_USI_MODE), rd(gpio_alive + GPA3 + PIN_CON), rd(gpio_alive + GPA3 + PIN_DAT),
	    rd(gpio_alive + GPA3 + PIN_PUD), rd(gpio_custom + GPN6 + PIN_CON), rd(gpio_custom + GPN6 + PIN_DAT),
	    rd(gpio_custom + GPN8 + PIN_CON), rd(gpio_custom + GPN8 + PIN_DAT), rd(gpio_custom + GPN9 + PIN_CON),
	    rd(gpio_custom + GPN9 + PIN_DAT));
}

static void
say_uart(const char *when)
{
	printf("akita bt: %s: ULCON %#x UCON %#x UFCON %#x UMCON %#x UTRSTAT %#x UERSTAT %#x UFSTAT %#x UMSTAT %#x UBRDIV %#x "
	    "UFRACVAL %#x UINTM %#x USI_CON %#x USI_OPTION %#x\n", when, rd(uart + ULCON), rd(uart + UCON), rd(uart + UFCON),
	    rd(uart + UMCON), rd(uart + UTRSTAT), rd(uart + UERSTAT), rd(uart + UFSTAT), rd(uart + UMSTAT), rd(uart + UBRDIV),
	    rd(uart + UFRACVAL), rd(uart + UINTM), rd(uart + USI_CON), rd(uart + USI_OPTION));
}

static uint64_t
now_ns(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns;
}

static void
uart_setup(uint32_t baud, bool loopback, bool flow)
{
	/* samsung_tty.c: quot = clk / baud; UBRDIV = quot / 16 - 1, UFRACVAL = quot % 16 */
	uint32_t quot = (src_hz + baud / 2) / baud;

	wr(uart + UINTM, 0xf);                          /* no interrupts: polled */
	wr(uart + UFCON, 0x7);                          /* FIFOs on, both reset */
	delay_for_interval(1, NSEC_PER_MSEC);
	wr(uart + UFCON, 0x1);
	wr(uart + ULCON, 0x3);                          /* 8N1 */
	wr(uart + UBRDIV, quot / 16 - 1);
	wr(uart + UFRACVAL, quot % 16);
	wr(uart + UCON, 0x5 | (loopback ? UCON_LOOPBACK : 0));  /* receive and transmit: polled / interrupt mode */
	wr(uart + UMCON, flow ? 0x10 : 0x1);            /* automatic RTS/CTS, or RTS held asserted */
	wr(uart + UINTP, 0xf);
}

/* the source clock: a FIFO's worth of bytes in loop-back at divisor 1+DIV, timed */
static uint32_t
uart_measure(uint32_t ubrdiv)
{
	enum { BYTES = 200 };
	uint64_t t0, t1;

	wr(uart + UINTM, 0xf);
	wr(uart + UFCON, 0x7);
	delay_for_interval(1, NSEC_PER_MSEC);
	wr(uart + UFCON, 0x1);
	wr(uart + ULCON, 0x3);
	wr(uart + UBRDIV, ubrdiv);
	wr(uart + UFRACVAL, 0);
	wr(uart + UMCON, 0x1);
	wr(uart + UCON, 0x5 | UCON_LOOPBACK);
	delay_for_interval(1, NSEC_PER_MSEC);
	t0 = now_ns();
	for (unsigned i = 0; i < BYTES; i++) {
		wr(uart + UTXH, 0x55);
	}
	while (!(rd(uart + UTRSTAT) & UTRSTAT_TX_EMPTY)) {
		if (now_ns() - t0 > 2 * NSEC_PER_SEC) {
			printf("akita bt: the UART does not send (UTRSTAT %#x UFSTAT %#x after 2 s): no clock?\n", rd(uart + UTRSTAT),
			    rd(uart + UFSTAT));
			return 0;
		}
	}
	t1 = now_ns();
	/* BYTES * 10 bits in (t1 - t0); the clock is 16 * (1 + UBRDIV) times the bit rate */
	uint64_t baud = (uint64_t)BYTES * 10 * NSEC_PER_SEC / (t1 - t0);

	printf("akita bt: loop-back at UBRDIV %u: %u bytes in %llu us -> %llu baud -> the source clock %llu Hz; %u bytes came back\n",
	    ubrdiv, BYTES, (t1 - t0) / 1000, baud, baud * 16 * (ubrdiv + 1), UFSTAT_RX_COUNT(rd(uart + UFSTAT)));
	return (uint32_t)(baud * 16 * (ubrdiv + 1));
}

static void
uart_send(const uint8_t *bytes, unsigned length)
{
	uint64_t t0 = now_ns();

	for (unsigned i = 0; i < length; i++) {
		while (rd(uart + UFSTAT) & UFSTAT_TX_FULL) {
			if (now_ns() - t0 > NSEC_PER_SEC) {
				printf("akita bt: send: the FIFO stays full (UFSTAT %#x UMSTAT %#x): CTS not given?\n", rd(uart + UFSTAT),
				    rd(uart + UMSTAT));
				return;
			}
		}
		wr(uart + UTXH, bytes[i]);
	}
}

/* what comes within MS milliseconds, at most MAX bytes; stops 20 ms after the last byte once something came */
static unsigned
uart_receive(uint8_t *out, unsigned max, unsigned ms)
{
	uint64_t t0 = now_ns(), last = 0;
	unsigned n = 0;

	while (n < max && now_ns() - t0 < (uint64_t)ms * NSEC_PER_MSEC) {
		if (UFSTAT_RX_COUNT(rd(uart + UFSTAT)) != 0) {
			out[n++] = (uint8_t)rd(uart + URXH);
			last = now_ns();
		} else if (n != 0 && now_ns() - last > 20 * NSEC_PER_MSEC) {
			break;
		} else {
			delay_for_interval(1, NSEC_PER_MSEC);
		}
	}
	return n;
}

static bool
hci_command(const char *name, const uint8_t *command, unsigned length, unsigned ms)
{
	uint8_t answer[64];
	char text[3 * sizeof(answer) + 1];
	unsigned n, at = 0;

	uart_send(command, length);
	n = uart_receive(answer, sizeof(answer), ms);
	text[0] = 0;
	for (unsigned i = 0; i < n; i++) {
		at += (unsigned)snprintf(text + at, sizeof(text) - at, "%02x ", answer[i]);
	}
	printf("akita bt: %s: %u bytes back: %s(UERSTAT %#x UFSTAT %#x UMSTAT %#x)\n", name, n, text, rd(uart + UERSTAT),
	    rd(uart + UFSTAT), rd(uart + UMSTAT));
	/* an event (04), Command Complete (0e), for this opcode, status 0 */
	return n >= 7 && answer[0] == 0x04 && answer[1] == 0x0e && answer[4] == command[1] && answer[5] == command[2] &&
	       answer[6] == 0;
}

static void
akita_bt_thread(__unused void *arg, __unused wait_result_t wr_)
{
	static const uint8_t reset[] = { 0x01, 0x03, 0x0c, 0x00 };
	static const uint8_t chip_id[] = { 0x01, 0x79, 0xfc, 0x00 };
	static const uint8_t version[] = { 0x01, 0x01, 0x10, 0x00 };
	uint32_t wake = 1, v;

	PE_parse_boot_argn("bt_wake", &wake, sizeof(wake));
	say_state("as found");
	/* the clocks: the bus clock and the reset synchroniser first, then the UART's own */
	wr(cmu + CMU_GATE_PCLK, CMU_GATE_ON);
	wr(cmu + CMU_GATE_RSTNSYNC, CMU_GATE_ON);
	wr(cmu + CMU_GATE_IPCLK, CMU_GATE_ON);
	delay_for_interval(1, NSEC_PER_MSEC);
	say_uart("as found");
	wr(sysreg + SYSREG_USI_MODE, USI_MODE_UART);
	wr(uart + USI_CON, rd(uart + USI_CON) & ~1u);           /* out of the USI's reset */
	delay_for_interval(1, NSEC_PER_MSEC);
	v = rd(uart + USI_OPTION);
	wr(uart + USI_OPTION, (v & ~(1u << 2)) | (1u << 1));    /* (as the I2C buses: clock request on, clock stop off) */
	for (unsigned pin = 0; pin < 4; pin++) {
		pin_set(gpio_alive + GPA3, pin, 2, 0, -1);
	}
	pin_set(gpio_custom + GPN6, 0, 0, 1, -1);               /* host wake: an input, pulled down as Android has it */
	say_state("set up");

	src_hz = uart_measure(15);
	if (src_hz == 0) {
		return;
	}
	(void)uart_measure(3);
	if (PE_parse_boot_argn("bt_hz", &v, sizeof(v)) && v != 0) {
		src_hz = v;
	}

	/* the chip: off, then on (the HAL's reg_on_delay_ms is 100), awake */
	uart_setup(115200, false, true);
	/* the four lines' levels at each step (gpa3's DAT: which of them the chip moves says whether it lives) */
	printf("akita bt: lines: UART set up %#x", rd(gpio_alive + GPA3 + PIN_DAT));
	pin_set(gpio_custom + GPN9, 0, 1, 0, (int)wake);
	pin_set(gpio_custom + GPN8, 0, 1, 0, 0);
	delay_for_interval(50, NSEC_PER_MSEC);
	printf(", REG_ON low %#x", rd(gpio_alive + GPA3 + PIN_DAT));
	pin_set(gpio_custom + GPN8, 0, 1, 0, 1);
	delay_for_interval(2, NSEC_PER_MSEC);
	printf(", high +2 ms %#x", rd(gpio_alive + GPA3 + PIN_DAT));
	delay_for_interval(48, NSEC_PER_MSEC);
	printf(", +50 ms %#x", rd(gpio_alive + GPA3 + PIN_DAT));
	delay_for_interval(100, NSEC_PER_MSEC);
	printf(", +150 ms %#x (gpn8 dat %#x)\n", rd(gpio_alive + GPA3 + PIN_DAT), rd(gpio_custom + GPN8 + PIN_DAT));
	say_state("the chip on");
	/* the chip says it listens by giving CTS (UMSTAT bit 0); if it does not, the other level of device-wake */
	for (unsigned try = 0; try < 2; try++) {
		uint64_t t0 = now_ns();
		bool cts = false;

		while (now_ns() - t0 < 1500 * NSEC_PER_MSEC) {
			if (rd(uart + UMSTAT) & 1u) {
				cts = true;
				break;
			}
			delay_for_interval(5, NSEC_PER_MSEC);
		}
		printf("akita bt: device-wake %u: CTS %s after %llu ms (UMSTAT %#x, gpa3 dat %#x, host-wake %u)\n", wake,
		    cts ? "given" : "NOT given", (now_ns() - t0) / NSEC_PER_MSEC, rd(uart + UMSTAT), rd(gpio_alive + GPA3 + PIN_DAT),
		    rd(gpio_custom + GPN6 + PIN_DAT) & 1u);
		if (cts) {
			break;
		}
		wake = !wake;
		pin_set(gpio_custom + GPN9, 0, 1, 0, (int)wake);
	}
	say_uart("115200 8N1");
	{
		uint8_t junk[32];
		unsigned n = uart_receive(junk, sizeof(junk), 50);

		if (n != 0) {
			printf("akita bt: %u bytes came by themselves after power-on (first %02x)\n", n, junk[0]);
		}
	}
	if (!(rd(uart + UMSTAT) & 1u)) {
		/* no CTS: said anyway, with the flow control off (RTS held asserted) -- does anything come back? */
		wr(uart + UMCON, 0x1);
		printf("akita bt: no CTS: HCI_Reset sent without flow control\n");
	}
	if (!hci_command("HCI_Reset", reset, sizeof(reset), 1000)) {
		printf("akita bt: no answer to HCI_Reset at 115200 with the clock taken as %u Hz (bt_hz=, bt_wake=%u)\n", src_hz, wake);
		return;
	}
	(void)hci_command("Read_Chip_Id (0xfc79)", chip_id, sizeof(chip_id), 500);
	(void)hci_command("Read_Local_Version_Information", version, sizeof(version), 500);
	printf("akita bt: the controller answers\n");
}

static void
akita_bt_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(akita_bt_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

__startup_func
static void
akita_bt_init(void)
{
	static thread_call_t start;
	uint32_t mode = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("bt", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	cmu = ml_io_map(CMU_APM_PHYS, 0x4000);
	sysreg = ml_io_map(SYSREG_APM_PHYS, 0x1000);
	gpio_alive = ml_io_map(GPIO_ALIVE_PHYS, 0x1000);
	gpio_custom = ml_io_map(GPIO_CUSTOM_PHYS, 0x1000);
	uart = ml_io_map(UART_PHYS, 0x1000);
	if (!cmu || !sysreg || !gpio_alive || !gpio_custom || !uart) {
		printf("akita bt: could not map its registers\n");
		return;
	}
	start = thread_call_allocate(akita_bt_start, NULL);
	/* well after the Wi-Fi half of the chip is up (pcie_delay=10 and its firmware) */
	clock_interval_to_deadline(30, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_bt_init);
