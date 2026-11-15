/*
 * The Pixel 8a's Bluetooth: the BCM4383 (the Wi-Fi chip's other half) speaks HCI in H4 on a UART of its own --
 * Android's device tree: odm/btbcm "goog,nitrous" and serial@155D0000 (uart18: the APM block's USI1 as a UART, RTS/CTS;
 * no pins to set: its pads are its own). So far, with boot-arg bt=1: the chip switched on and asked HCI_Reset,
 * Broadcom's Read_Chip_Id (0xfc79) and Read_Local_Version_Information.
 *
 * How it was found (11-15), because none of it is in a public source for this phone: Android's kernel traced on the
 * phone itself (events/rwmmio: every register it writes while Bluetooth is switched on) --
 *   - BT_REG_ON (gpn8) goes through a LATCH: each change is followed by a pulse on gpp24-3 ("bt-ble-dbo-le");
 *   - the UART's source clock is 200 MHz (UBRDIV 0x6b, UFRACVAL 8 for 115200);
 *   - USI_OPTION is written 2, UCON 0xf3c5, UFCON 0x111, UMCON 0x30.
 * The AoC has no part in it (Android's Bluetooth starts with the AoC never booted); gpa3, the tree's "uart18-bus", is
 * not this UART's (gpa3-0/1 are a bit-banged SPMI bus: never to be touched).
 *
 * What Android's HAL does next (its firmware_config.json and its I/O traced): the baud rate up (0xfc18, 4 000 000), the
 * fast download on (0xfe72), Download_Minidriver (0xfc2e, 50 ms), the .hcd's records sent as they are (each is an HCI
 * command), Launch_RAM (0xfc4e, 250 ms), HCI_Reset, the low-power mode (0xfc27) and the address (0xfc01). Not here yet.
 *
 * Registers: Linux's drivers/tty/serial/exynos_tty.c; the clocks: Google's cal-if zuma cmucal-sfr.c (the gates of
 * APM_USI1_UART_INT); the pins: pinctrl-gs.c's zuma tables (gpn6/8/9 in GPIO_CUSTOM_ALIVE 0x15060000 at 0xc0 / 0x100 /
 * 0x120; gpp24 in GPIO_PERIC1 0x10C40000 at 0x60).
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
#define GPA3                    0x060           /* read only: NOT the UART (pins 0/1 are a bit-banged SPMI bus) */
#define GPIO_PERIC1_PHYS        0x10c40000ULL
#define GPP24                   0x060           /* pin 3: the latch enable of BT_REG_ON ("bt-ble-dbo-le") */
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

static vm_offset_t cmu, sysreg, gpio_alive, gpio_custom, gpio_peric1, uart;
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

/* Linux's exynos_serial_pm / _resetport / _startup / _set_termios for this port, as its register writes were traced on
 * this phone (rwmmio events, Android's kernel): the USI into its reset and out, USI_OPTION 2, UMCON 0x20, UFCON 0x111,
 * UCON 0x3c5 | 0xf000, the divisor for a 200 MHz clock (115200: UBRDIV 0x6b, UFRACVAL 8), UMCON 0x30 (RTS/CTS) */
static void
uart_open(uint32_t baud)
{
	uint32_t quot;

	wr(cmu + CMU_GATE_PCLK, CMU_GATE_ON);
	wr(cmu + CMU_GATE_RSTNSYNC, CMU_GATE_ON);
	wr(cmu + CMU_GATE_IPCLK, CMU_GATE_ON);
	delay_for_interval(1, NSEC_PER_MSEC);
	wr(sysreg + SYSREG_USI_MODE, USI_MODE_UART);
	wr(uart + USI_CON, 1);
	delay(10);
	wr(uart + USI_CON, 0);
	wr(uart + USI_OPTION, 2);
	wr(uart + UMCON, 0x20);
	wr(uart + UINTM, 0xf);                          /* polled here: every interrupt masked */
	wr(uart + UFCON, 0x117);                        /* FIFOs on, both reset */
	delay_for_interval(1, NSEC_PER_MSEC);
	wr(uart + UFCON, 0x111);
	wr(uart + UCON, 0xf3c5);
	wr(uart + ULCON, 0x3);
	quot = (src_hz + baud / 2) / baud;
	wr(uart + UBRDIV, quot / 16 - 1);
	wr(uart + UFRACVAL, quot % 16);
	wr(uart + UMCON, 0x30);
	wr(uart + UINTP, 0xf);
}

/*
 * BT_REG_ON reaches the chip through a latch: after each change of gpn8 Android's kernel pulses gpp24-3 (the tree's
 * "bt-ble-dbo-le": a latch enable -- the chip keeps its power state while the phone is off, for Find My) high and low.
 * Traced: DAT of GPIO_PERIC1 +0x60 written 0x1, 0x9, 0x1 right after each write of gpn8. Without the pulse the chip
 * never sees the pin: that was the silence of every run before.
 */
static void
power_pin(int on)
{
	pin_set(gpio_custom + GPN8, 0, 1, 0, on);
	delay(20);
	pin_set(gpio_peric1 + GPP24, 3, 1, 0, 0);
	delay(10);
	pin_set(gpio_peric1 + GPP24, 3, 1, 0, 1);
	delay(10);
	pin_set(gpio_peric1 + GPP24, 3, 1, 0, 0);
}

static void
akita_bt_thread(__unused void *arg, __unused wait_result_t wr_)
{
	static const uint8_t reset[] = { 0x01, 0x03, 0x0c, 0x00 };
	static const uint8_t chip_id[] = { 0x01, 0x79, 0xfc, 0x00 };
	static const uint8_t version[] = { 0x01, 0x01, 0x10, 0x00 };
	uint32_t v;
	uint8_t junk[32];
	unsigned n;

	src_hz = 200000000;                             /* (Linux's divisor for 115200 says so; the loop-back measured 196.5) */
	if (PE_parse_boot_argn("bt_hz", &v, sizeof(v)) && v != 0) {
		src_hz = v;
	}
	say_state("as found");
	say_uart("as found");
	/* nitrous_rfkill_set_power, as traced: off, 43 ms, on, device-wake high; the UART opened 26 ms later, the first
	 * command ~150 ms after that */
	pin_set(gpio_custom + GPN6, 0, 0, 1, -1);               /* host wake: an input, pulled down as Android has it */
	pin_set(gpio_custom + GPN9, 0, 1, 0, 0);
	power_pin(0);
	delay_for_interval(43, NSEC_PER_MSEC);
	power_pin(1);
	pin_set(gpio_custom + GPN9, 0, 1, 0, 1);
	delay_for_interval(26, NSEC_PER_MSEC);
	uart_open(115200);
	delay_for_interval(150, NSEC_PER_MSEC);
	say_state("the chip on");
	say_uart("before HCI_Reset");
	printf("akita bt: gpp24 con %#x dat %#x; CTS %s; host-wake %u\n", rd(gpio_peric1 + GPP24 + PIN_CON),
	    rd(gpio_peric1 + GPP24 + PIN_DAT), (rd(uart + UMSTAT) & 1u) ? "given" : "NOT given", rd(gpio_custom + GPN6 + PIN_DAT) & 1u);
	n = uart_receive(junk, sizeof(junk), 20);
	if (n != 0) {
		printf("akita bt: %u bytes came by themselves after power-on (first %02x)\n", n, junk[0]);
	}
	if (!hci_command("HCI_Reset", reset, sizeof(reset), 1000)) {
		printf("akita bt: no answer to HCI_Reset at 115200 with the clock taken as %u Hz (bt_hz=)\n", src_hz);
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
	gpio_peric1 = ml_io_map(GPIO_PERIC1_PHYS, 0x1000);
	uart = ml_io_map(UART_PHYS, 0x1000);
	if (!cmu || !sysreg || !gpio_alive || !gpio_custom || !gpio_peric1 || !uart) {
		printf("akita bt: could not map its registers\n");
		return;
	}
	start = thread_call_allocate(akita_bt_start, NULL);
	clock_interval_to_deadline(30, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_bt_init);
