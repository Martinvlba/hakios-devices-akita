/*
 * The Pixel 8a's battery, for Darwin: the fuel gauge and the charger of its MAX77759, read over I2C every five seconds
 * (fuelgauge=1), and handed to drivers/power/hakios.battery.c (hakios_battery_report), which keeps them for the
 * kern.hakios.battery_* sysctls and passes them on to HakiosBattery, the IOPMPowerSource the status bar's battery,
 * the lock screen's "Charging" and Settings > Battery read.
 *
 * Read only: nothing here tells the charger anything. What it does is what Google's bootloader and the chip's own
 * defaults left (seen in iOS: CHG_CNFG_00 mode 5, charger and buck on).
 *
 * The bus: zuma's hsi2c@10cb0000 (Linux: i2c-11 under 10cb0000.hsi2c), an Exynos 7 HSI2C in a USI block, also the eUSB
 * repeater's. U-Boot's driver has set it up before the kernel (cmd/hakios_eusb.c, `eusb repeater`: auto mode, FIFOs on,
 * the bootloader's timings kept); here only transfers, polled, as U-Boot's drivers/i2c/exynos_hs_i2c.c does them (the
 * Exynos 7 variant: completion and errors in INT_STATUS). Nothing else in the kernel uses the bus.
 *
 * The chips (Android: /sys/class/power_supply/maxfg is 11-0036, main-charger 11-0069), registers little-endian words
 * for the gauge, bytes for the charger. The scales were read against Android's own values on this phone (2026-11-02):
 *   gauge 0x36: 0x00 Status (bit 3: no battery), 0x05 RepCap and 0x10 FullCapRep and 0x18 DesignCap in 2 mAh,
 *               0x06 RepSOC in 1/256 %, 0x08 Temp in 1/256 C, 0x09 VCell in 78.125 uV, 0x0a Current in 312.5 uA
 *               (signed, more than 0 while charging), 0x17 Cycles in 1/100 of a cycle
 *   TCPC 0x25 (the USB-C port controller, TCPCI): 0x1d CC_STATUS (bit 4 the phone the sink; bits 1:0 / 3:2 CC1 / CC2:
 *               what the source advertises, 1 default, 2 1.5 A, 3 3.0 A), 0x1e POWER_STATUS: the charger's type and current
 *   charger 0x69: 0xb4 CHG_INT_OK (bit 6 CHGIN_OK: a source on the USB port), 0xb6 CHG_DETAILS_01 (bits 3:0 CHG_DTLS:
 *               0 precharge, 1 constant current, 2 constant voltage, 3 top-off, 4 done, others not charging)
 */
#include <kern/clock.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define HSI2C_PHYS              0x10cb0000UL

/* the HSI2C's registers (U-Boot's struct exynos5_hsi2c) */
#define CTL                     0x00
#define FIFO_CTL                0x04
#define INT_STAT                0x24
#define FIFO_STAT               0x30
#define TX_DATA                 0x34
#define RX_DATA                 0x38
#define CONF                    0x40
#define AUTO_CONF               0x44
#define TIMEOUT                 0x48
#define TRANS_STATUS            0x50
#define ADDR                    0x70

#define CTL_FUNC_MODE_I2C       (1u << 0)
#define CTL_MASTER              (1u << 3)
#define CTL_RXCHON              (1u << 6)
#define CTL_TXCHON              (1u << 7)
#define FIFO_RX_EN              (1u << 0)
#define FIFO_TX_EN              (1u << 1)
#define CONF_AUTO_MODE          (1u << 31)
#define AUTO_READ               (1u << 16)
#define AUTO_STOP               (1u << 17)
#define AUTO_RUN                (1u << 31)
#define TIMEOUT_EN              (1u << 31)
#define INT_DONE                (1u << 7)
#define INT_ABORT               (1u << 8)
#define INT_NO_ACK              (1u << 9)
#define INT_NO_DEV              (1u << 10)
#define INT_TIMEOUT             (1u << 11)
#define INT_ALL                 (INT_DONE | INT_ABORT | INT_NO_ACK | INT_NO_DEV | INT_TIMEOUT)
#define FIFO_TX_FULL            (1u << 7)
#define FIFO_RX_EMPTY           (1u << 24)
#define MASTER_BUSY             (1u << 17)
#define WAIT_US                 10000

#define GAUGE                   0x36
#define CHARGER                 0x69
#define TCPC                    0x25            /* the MAX77759's USB-C port controller (TCPCI) */
#define TCPC_CC_STATUS          0x1d
#define TCPC_POWER_STATUS       0x1e

#if defined(HAKIOS_AKITA_USB)
extern bool akita_usb_host_configured(void);    /* drivers/usb/hakios.akita_usb.c */
#else
static bool akita_usb_host_configured(void) { return false; }
#endif

static vm_offset_t i2c;
static uint32_t errors;

static inline uint32_t
rd(uint32_t off)
{
	return *(volatile uint32_t *)(i2c + off);
}

static inline void
wr(uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(i2c + off) = v;
}

/* until (register & mask) == want; false after WAIT_US */
static bool
wait_for(uint32_t off, uint32_t mask, uint32_t want)
{
	for (unsigned i = 0; i < WAIT_US; i++) {
		if ((rd(off) & mask) == want) {
			return true;
		}
		delay(1);
	}
	return false;
}

/* the end of a transfer: done, or what went wrong (0 for done) */
static uint32_t
transfer_end(void)
{
	for (unsigned i = 0; i < WAIT_US; i++) {
		uint32_t s = rd(INT_STAT);

		if (s & INT_ALL) {
			wr(INT_STAT, s);
			return s & (INT_ABORT | INT_NO_ACK | INT_NO_DEV | INT_TIMEOUT) ? s : 0;
		}
		delay(1);
	}
	return ~0u;
}

/* one transfer to or from the chip at addr: len bytes, a stop after it or not (a repeated start follows) */
static bool
transfer(uint8_t addr, bool read, uint8_t *buf, unsigned len, bool stop)
{
	uint32_t end;
	bool ok = true;

	wr(TIMEOUT, rd(TIMEOUT) & ~TIMEOUT_EN);
	wr(ADDR, (uint32_t)(addr & 0x3ff) << 10);
	wr(CTL, (read ? CTL_RXCHON : CTL_TXCHON) | CTL_FUNC_MODE_I2C | CTL_MASTER);
	wr(INT_STAT, INT_ALL);
	wr(AUTO_CONF, len | AUTO_RUN | (stop ? AUTO_STOP : 0) | (read ? AUTO_READ : 0));
	for (unsigned i = 0; i < len && ok; i++) {
		if (read) {
			ok = wait_for(FIFO_STAT, FIFO_RX_EMPTY, 0);
			if (ok) {
				buf[i] = (uint8_t)rd(RX_DATA);
			}
		} else {
			ok = wait_for(FIFO_STAT, FIFO_TX_FULL, 0);
			if (ok) {
				wr(TX_DATA, buf[i]);
			}
		}
	}
	end = transfer_end();
	if (stop) {
		wait_for(TRANS_STATUS, MASTER_BUSY, 0);
	}
	wr(CTL, CTL_FUNC_MODE_I2C);
	if (!ok || end) {
		if (errors++ < 8) {
			printf("akita battery: I2C %s of %u bytes at %#x failed (%s, INT_STAT %#x)\n", read ? "read" : "write",
			    len, addr, ok ? "the end" : "the FIFO", end);
		}
		return false;
	}
	return true;
}

/* len bytes from register reg of the chip at addr: the register's number written, then a repeated start and the read */
static bool
read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, unsigned len)
{
	return transfer(addr, false, &reg, 1, false) && transfer(addr, true, buf, len, true);
}

static bool
gauge_word(uint8_t reg, int32_t *value, bool is_signed)
{
	uint8_t b[2];

	if (!read_regs(GAUGE, reg, b, 2)) {
		return false;
	}
	*value = is_signed ? (int32_t)(int16_t)(b[0] | b[1] << 8) : (int32_t)(b[0] | b[1] << 8);
	return true;
}

static bool
battery_read(struct hakios_battery_state *s)
{
	int32_t status, rep_cap, soc, temp, vcell, current, full, cycles, design;
	uint8_t int_ok, details;

	if (!gauge_word(0x00, &status, false) || !gauge_word(0x05, &rep_cap, false) || !gauge_word(0x06, &soc, false) ||
	    !gauge_word(0x08, &temp, true) || !gauge_word(0x09, &vcell, false) || !gauge_word(0x0a, &current, true) ||
	    !gauge_word(0x10, &full, false) || !gauge_word(0x17, &cycles, false) || !gauge_word(0x18, &design, false) ||
	    !read_regs(CHARGER, 0xb4, &int_ok, 1) || !read_regs(CHARGER, 0xb6, &details, 1)) {
		return false;
	}
	bzero(s, sizeof(*s));
	s->size = sizeof(*s);
	s->percent_hundredths = (int32_t)(((int64_t)soc * 100 + 128) / 256);
	s->voltage_uv = (int32_t)(((int64_t)vcell * 78125) / 1000);
	s->current_ua = (int32_t)(((int64_t)current * 3125) / 10);
	s->temperature_hundredths = (int32_t)(((int64_t)temp * 100) / 256);
	s->present = !(status & 0x08);
	s->external = (int_ok & 0x40) != 0;
	if (!s->external) {
		s->status = 2;                                  /* POWER_SUPPLY_STATUS_DISCHARGING */
	} else {
		switch (details & 0x0f) {
		case 0: case 1: case 2: case 3:
			s->status = 1;                          /* charging */
			break;
		case 4:
			s->status = 4;                          /* full */
			break;
		default:
			s->status = 3;                          /* on a charger, not charging */
			break;
		}
	}
	s->charge_uah = rep_cap * 2000;
	s->full_uah = full * 2000;
	s->design_uah = design * 2000;
	s->cycles = cycles / 100;
	/*
	 * What is on the cable (the TCPC, TCPCI's standard registers): CC_STATUS says, when the phone is the sink (bit 4),
	 * what the other end advertises on the CC line that is connected -- 1 USB's default current, 2 1.5 A, 3 3.0 A
	 * (Android's current_max agreed: 3.0 A from a computer's USB-C port, CC_STATUS 0x13). Advertising 1.5 or 3 A makes
	 * it a USB-C source; else a computer that configured the phone's USB is a computer's port (Android's "SDP"); else
	 * not known. 5 V either way: nothing here asks for Power Delivery.
	 */
	{
		uint8_t cc = 0, power = 0;

		s->usb_type = 0;                                /* Darwin's "USB, unknown" */
		if (s->external && read_regs(TCPC, TCPC_CC_STATUS, &cc, 1) && read_regs(TCPC, TCPC_POWER_STATUS, &power, 1) &&
		    (cc & 0x10)) {
			unsigned level = (cc & 0x03) ? (cc & 0x03) : ((cc >> 2) & 0x03);

			s->usb_voltage_uv = 5000000;
			s->usb_current_ua = level == 3 ? 3000000 : level == 2 ? 1500000 : 500000;
			if (level >= 2) {
				s->usb_type = 5;                /* USB-C: kIOPSFamilyCodeUSBCTypeC */
			} else if (akita_usb_host_configured()) {
				s->usb_type = 1;                /* SDP, a computer's port: kIOPSFamilyCodeUSBHost */
			}
		}
	}
	return true;
}

static void
akita_battery_thread(__unused void *parameter, __unused wait_result_t result)
{
	struct hakios_battery_state s;
	uint32_t reads = 0, failed = 0;

	for (unsigned tick = 0;; tick++) {
		if (battery_read(&s)) {
			reads++;
			hakios_battery_report(&s);
			if (tick % 60 == 0) {           /* every five minutes, and the first */
				printf("akita battery: %d.%02d%%, %d mV, %d mA, %d.%d C, %s (CHG_DTLS %d), %d of %d mAh, "
				    "%d cycles (%u reads, %u failed)\n", s.percent_hundredths / 100, s.percent_hundredths % 100,
				    s.voltage_uv / 1000, s.current_ua / 1000, s.temperature_hundredths / 100,
				    (s.temperature_hundredths % 100) / 10, s.external ? (s.status == 1 ? "charging" :
				    s.status == 4 ? "full" : "on a charger, not charging") : "not on a charger", s.status,
				    s.charge_uah / 1000, s.full_uah / 1000, s.cycles, reads, failed);
			}
		} else {
			failed++;
		}
		delay_for_interval(5000, NSEC_PER_MSEC);
	}
}

static void
akita_battery_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(akita_battery_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("akita battery: no thread\n");
	}
}

__startup_func
static void
akita_battery_init(void)
{
	uint32_t enabled = 0;
	uint64_t deadline;
	static thread_call_t start;

	if (!PE_parse_boot_argn("fuelgauge", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	i2c = ml_io_map(HSI2C_PHYS, 0x1000);
	if (!i2c) {
		printf("akita battery: could not map the I2C controller; off\n");
		return;
	}
	/* as U-Boot left it, normally; else the two things a transfer needs (the timings are the bootloader's either way) */
	if (!(rd(CONF) & CONF_AUTO_MODE) || (rd(FIFO_CTL) & (FIFO_RX_EN | FIFO_TX_EN)) != (FIFO_RX_EN | FIFO_TX_EN)) {
		printf("akita battery: the I2C controller not as U-Boot leaves it (CONF %#x FIFO_CTL %#x): auto mode, FIFOs on\n",
		    rd(CONF), rd(FIFO_CTL));
		wr(CONF, rd(CONF) | CONF_AUTO_MODE);
		wr(FIFO_CTL, rd(FIFO_CTL) | FIFO_RX_EN | FIFO_TX_EN);
	}
	/* a thread of its own once the kernel is up (IOKit for the power source): started from a thread call, 5 s from now */
	start = thread_call_allocate(akita_battery_start, NULL);
	clock_interval_to_deadline(5, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_battery_init);
