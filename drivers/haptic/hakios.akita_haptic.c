/*
 * The Pixel 8a's vibration motor: a Cirrus Logic CS40L26A (a boosted haptic amplifier with a HALO DSP) at 0x43 on zuma's
 * hsi2c@10C80000 -- USI0 of PERIC1 -- driving a linear resonant actuator (Android's device tree, devices/akita/boot-info/
 * fdt.dtb: cs40l26a@43, reset-gpios gph2-3 active low, the interrupt on gpa6-3; f0-default 0x2a8000, redc-default
 * 0x68000, boost 10 V and 2.5 A peak).
 *
 * The chip boots its DSP from ROM; Google's driver (kernel/google-modules/amplifiers, cs40l26/cs40l26.c, akita 5.15
 * android15) then loads Cirrus's RAM firmware (cs40l26.wmfw and the tuning .bin files of /vendor/firmware) and plays
 * its wavetable's effects through the DSP's mailbox (CS40L26_DSP_VIRTUAL1_MBOX_1, 0x13020: 0x01000000 + n a RAM
 * effect, 0x01800000 + n a ROM one, 0x05000000 stop). Registers are 32-bit addresses and 32-bit values, both
 * big-endian on the bus.
 *
 * The bus is brought up here as the flashlight's is (hakios.akita_torch.c: USI clocks held on, the USI in I2C mode,
 * its pins the I2C's, the controller set up as U-Boot's exynos_hs_i2c.c does), with PERIC1's addresses: CMU_PERIC1
 * 0x10c00000 (USI0: Google's cal-if zuma cmucal-sfr.c), sysreg_peric1 0x10c21000 + 0 (samsung,usi-offset), pins
 * gpp19-0/1 function 3 (hsi2c0-bus; Google's pinctrl-gs.c zuma_pin_peric1).
 *
 *   haptic=1   a look: the chip out of reset, its ids and the DSP's state read -- nothing written to the chip
 *   haptic=2   ... and one of the ROM's effects played (haptic_effect=N, 0-38; 0 by default) through the mailbox, as
 *              Google's driver triggers one (hibernation prevented first, allowed after), with the ROM's own setup --
 *              no RAM firmware, no tuning: whether the motor moves at all
 * (Playing Darwin's haptics is for later: the RAM firmware's loader and its configuration first.)
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define CMU_PERIC1_PHYS         0x10c00000ULL
#define CMU_MUX_USI0            0x0620          /* PLL_CON0_MUX_CLKCMU_PERIC1_USI0_USI_USER: bit 4, 1 = CMU_TOP's clock */
#define CMU_DIV_USI0            0x1808
#define CMU_GATE_IPCLK_USI0     0x2038
#define CMU_GATE_PCLK_USI0      0x203c
#define CMU_GATE_RSTNSYNC_USI0  0x208c
#define CMU_QCH_USI0            0x3038
#define CMU_GATE_ON             0x00300000
#define SYSREG_PERIC1_PHYS      0x10c21000ULL
#define SYSREG_USI0_MODE        0x00
#define USI_MODE_I2C            0x4
#define GPIO_PERIC1_PHYS        0x10c40000ULL
#define GPP19                   0x000
#define GPIO_HSI1_PHYS          0x12040000ULL
#define GPH2                    0x040           /* the reset: pin 3, active low */
#define HSI2C0_PHYS             0x10c80000ULL
#define HSI2C_BATTERY_PHYS      0x10cb0000ULL   /* (PERIC1's bus U-Boot set up, the battery's: timing to copy) */
#define CRYSTAL_HZ              24576000u
#define BUS_HZ                  400000u         /* (the tree says 1 MHz, 0xf4240: 400 kHz to start with) */

#define CTL                     0x00
#define FIFO_CTL                0x04
#define TRAILING_CTL            0x08
#define INT_EN                  0x20
#define INT_STAT                0x24
#define ERROR_STAT              0x2c
#define FIFO_STAT               0x30
#define TX_DATA                 0x34
#define RX_DATA                 0x38
#define CONF                    0x40
#define AUTO_CONF               0x44
#define TIMEOUT                 0x48
#define TRANS_STATUS            0x50
#define TIMING_FS1              0x60
#define TIMING_FS2              0x64
#define TIMING_FS3              0x68
#define TIMING_SLA              0x6c
#define ADDR                    0x70
#define USI_CON                 0xc4
#define USI_OPTION              0xc8
#define USI_CON_RESET           (1u << 0)
#define USI_OPTION_CLKREQ_ON    (1u << 1)
#define USI_OPTION_CLKSTOP_ON   (1u << 2)
#define CTL_FUNC_MODE_I2C       (1u << 0)
#define CTL_MASTER              (1u << 3)
#define CTL_RXCHON              (1u << 6)
#define CTL_TXCHON              (1u << 7)
#define CTL_SW_RST              (1u << 31)
#define FIFO_RX_EN              (1u << 0)
#define FIFO_TX_EN              (1u << 1)
#define CONF_AUTO_MODE          (1u << 31)
#define AUTO_READ               (1u << 16)
#define AUTO_STOP               (1u << 17)
#define AUTO_RUN                (1u << 31)
#define TIMEOUT_EN              (1u << 31)
#define INT_TRANS_EN            0x0f80
#define INT_DONE                (1u << 7)
#define INT_ABORT               (1u << 8)
#define INT_NO_ACK              (1u << 9)
#define INT_NO_DEV              (1u << 10)
#define INT_TIMEOUT             (1u << 11)
#define INT_ALL                 (INT_DONE | INT_ABORT | INT_NO_ACK | INT_NO_DEV | INT_TIMEOUT)
#define FIFO_TX_FULL            (1u << 7)
#define FIFO_RX_EMPTY           (1u << 24)
#define MASTER_BUSY             (1u << 17)
#define TRAILING_COUNT          0xf
#define WAIT_US                 10000

#define CS40L26                 0x43
#define REG_DEVID               0x0000
#define REG_REVID               0x0004
#define REG_GLOBAL_ENABLES      0x2014
#define REG_PWRMGT_STS          0x290c
#define REG_IRQ1_STATUS         0x10004
#define REG_IRQ1_EINT_1         0x10010
#define REG_PM_CUR_STATE        0x02800370      /* (the A1 ROM's) */
#define REG_HALO_STATE          0x02800fa8
#define REG_REQ_ACTIVE          0x02800c08
#define DEVID_CS40L26A          0x40a260
#define REG_MBOX_1              0x13020         /* CS40L26_DSP_VIRTUAL1_MBOX_1: commands, read back as 0 once taken */
#define MBOX_PREVENT_HIBER      0x02000003
#define MBOX_ALLOW_HIBER        0x02000004
#define MBOX_ROM_EFFECT(n)      (0x01800000u + (n))

static vm_offset_t cmu, sysreg, gpio, gpio_hsi1, i2c, i2c_battery;
static uint32_t mode;
static uint32_t errors;
static bool reset_touched;                      /* gph2-3 driven by us (HSI1's pins: not read otherwise) */

static inline uint32_t rd(vm_offset_t a) { return *(volatile uint32_t *)a; }
static inline void wr(vm_offset_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

static bool
wait_for(uint32_t off, uint32_t mask, uint32_t want)
{
	for (unsigned i = 0; i < WAIT_US; i++) {
		if ((rd(i2c + off) & mask) == want) {
			return true;
		}
		delay(1);
	}
	return false;
}

/* the end of a transfer: INT_STAT's report (done, or what went wrong), within 50 ms (TRANS_STATUS's low bits are the
 * Exynos 7 master's state, not a result -- 6 is ADDR1, 3 STOP: misread as errors once, 11-11) */
static uint32_t
transfer_end(void)
{
	uint64_t t0 = mach_absolute_time(), ns = 0;

	while (ns < 50 * NSEC_PER_MSEC) {
		uint32_t s = rd(i2c + INT_STAT);

		if (s & INT_ALL) {
			wr(i2c + INT_STAT, s);
			return s & (INT_ABORT | INT_NO_ACK | INT_NO_DEV | INT_TIMEOUT) ? s : 0;
		}
		delay(2);
		absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	}
	return ~0u;
}

static bool
transfer(uint8_t addr, bool read, uint8_t *buf, unsigned len, bool stop)
{
	uint32_t end;
	bool ok = true;

	wr(i2c + TIMEOUT, rd(i2c + TIMEOUT) & ~TIMEOUT_EN);
	wr(i2c + ADDR, (uint32_t)(addr & 0x3ff) << 10);
	wr(i2c + CTL, (read ? CTL_RXCHON : CTL_TXCHON) | CTL_FUNC_MODE_I2C | CTL_MASTER);
	wr(i2c + INT_STAT, INT_ALL);
	wr(i2c + AUTO_CONF, len | AUTO_RUN | (stop ? AUTO_STOP : 0) | (read ? AUTO_READ : 0));
	for (unsigned i = 0; i < len && ok; i++) {
		if (read) {
			ok = wait_for(FIFO_STAT, FIFO_RX_EMPTY, 0);
			if (ok) {
				buf[i] = (uint8_t)rd(i2c + RX_DATA);
			}
		} else {
			ok = wait_for(FIFO_STAT, FIFO_TX_FULL, 0);
			if (ok) {
				wr(i2c + TX_DATA, buf[i]);
			}
		}
	}
	end = transfer_end();
	if (stop) {
		wait_for(TRANS_STATUS, MASTER_BUSY, 0);
	}
	wr(i2c + CTL, CTL_FUNC_MODE_I2C);
	if (!ok || end) {
		if (errors++ < 8) {
			printf("akita haptic: I2C %s of %u bytes at %#x failed (%s, INT_STAT %#x): now INT_STAT %#x FIFO_STAT %#x "
			    "TRANS_STATUS %#x ERROR_STAT %#x\n", read ? "read" : "write", len, addr, ok ? "the end" : "the FIFO", end,
			    rd(i2c + INT_STAT), rd(i2c + FIFO_STAT), rd(i2c + TRANS_STATUS), rd(i2c + ERROR_STAT));
		}
		return false;
	}
	return true;
}

/* a 32-bit register: its address written big-endian, then a repeated start and four bytes read, big-endian; the chip
 * may sleep (hibernate) and miss the first try, which wakes it */
static bool
reg_read(uint32_t reg, uint32_t *value)
{
	uint8_t a[4] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg }, v[4] = { 0 };

	for (int tries = 0; tries < 3; tries++) {
		if (transfer(CS40L26, false, a, 4, false) && transfer(CS40L26, true, v, 4, true)) {
			*value = (uint32_t)v[0] << 24 | (uint32_t)v[1] << 16 | (uint32_t)v[2] << 8 | v[3];
			return true;
		}
		delay(1000);
	}
	return false;
}

static void
say_bus(const char *name, vm_offset_t b)
{
	printf("akita haptic: %s: CTL %#x FIFO_CTL %#x TRAILING %#x INT_EN %#x CONF %#x TIMEOUT %#x timing fs %#x/%#x/%#x sla %#x "
	    "TRANS_STATUS %#x USI_CON %#x USI_OPTION %#x\n", name, rd(b + CTL), rd(b + FIFO_CTL), rd(b + TRAILING_CTL),
	    rd(b + INT_EN), rd(b + CONF), rd(b + TIMEOUT), rd(b + TIMING_FS1), rd(b + TIMING_FS2), rd(b + TIMING_FS3),
	    rd(b + TIMING_SLA), rd(b + TRANS_STATUS), rd(b + USI_CON), rd(b + USI_OPTION));
}

/*
 * The bus as the bootloader left it, if it did (the USI in I2C mode, the pins the I2C's: Google's bootloader uses this
 * one -- 11-11): its clock and timing kept, only what a transfer needs made sure of, as for the battery's bus. (Not so:
 * the crystal for its clock, as the flashlight's -- which stalled this controller, 11-11: its FIFO filled, nothing sent.)
 */
static bool
bus_as_found(void)
{
	say_bus("HSI2C0 as found", i2c);
	if (rd(sysreg + SYSREG_USI0_MODE) != USI_MODE_I2C || (rd(gpio + GPP19) & 0xff) != 0x33) {
		return false;
	}
	wr(i2c + CONF, rd(i2c + CONF) | CONF_AUTO_MODE);
	wr(i2c + FIFO_CTL, rd(i2c + FIFO_CTL) | FIFO_RX_EN | FIFO_TX_EN);
	return true;
}

/* the battery's bus's timing, on the same CMU's clock, copied (its controller set up by U-Boot) */
static void
bus_timing_from_battery(void)
{
	wr(i2c + CTL, rd(i2c + CTL) | CTL_SW_RST);
	wr(i2c + CTL, rd(i2c + CTL) & ~CTL_SW_RST);
	wr(i2c + TRAILING_CTL, rd(i2c_battery + TRAILING_CTL));
	wr(i2c + CONF, rd(i2c_battery + CONF));
	wr(i2c + INT_EN, rd(i2c_battery + INT_EN));
	wr(i2c + FIFO_CTL, FIFO_RX_EN | FIFO_TX_EN);
	wr(i2c + TIMING_FS1, rd(i2c_battery + TIMING_FS1));
	wr(i2c + TIMING_FS2, rd(i2c_battery + TIMING_FS2));
	wr(i2c + TIMING_FS3, rd(i2c_battery + TIMING_FS3));
	wr(i2c + TIMING_SLA, rd(i2c_battery + TIMING_SLA));
	say_bus("HSI2C0 with the battery bus's timing", i2c);
}

static bool
reg_write(uint32_t reg, uint32_t value)
{
	uint8_t b[8] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg,
		         (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value };

	for (int tries = 0; tries < 3; tries++) {
		if (transfer(CS40L26, false, b, 8, true)) {
			return true;
		}
		delay(1000);
	}
	return false;
}

/* a mailbox command, and until the DSP has taken it (the mailbox reads 0 again: Google's cs40l26_ack_write), 100 ms */
static bool
mailbox(uint32_t command)
{
	uint32_t v = ~0u;

	if (!reg_write(REG_MBOX_1, command)) {
		return false;
	}
	for (int i = 0; i < 100; i++) {
		if (reg_read(REG_MBOX_1, &v) && v == 0) {
			return true;
		}
		delay(1000);
	}
	printf("akita haptic: mailbox command %#x not taken (%#x)\n", command, v);
	return false;
}

static __unused bool
bus_up_crystal(void)
{
	uint32_t v, t_ftl, cycles, div = 0, cycle = 0, scl;

	printf("akita haptic: CMU_PERIC1 USI0: mux %#x div %#x gates %#x/%#x/%#x qch %#x; sysreg mode %#x; gpp19 con %#x dat %#x\n",
	    rd(cmu + CMU_MUX_USI0), rd(cmu + CMU_DIV_USI0), rd(cmu + CMU_GATE_IPCLK_USI0), rd(cmu + CMU_GATE_PCLK_USI0),
	    rd(cmu + CMU_GATE_RSTNSYNC_USI0), rd(cmu + CMU_QCH_USI0), rd(sysreg + SYSREG_USI0_MODE), rd(gpio + GPP19),
	    rd(gpio + GPP19 + 4));
	wr(cmu + CMU_GATE_PCLK_USI0, CMU_GATE_ON);
	wr(cmu + CMU_GATE_RSTNSYNC_USI0, CMU_GATE_ON);
	wr(cmu + CMU_MUX_USI0, rd(cmu + CMU_MUX_USI0) & ~(1u << 4));
	wr(cmu + CMU_DIV_USI0, rd(cmu + CMU_DIV_USI0) & ~0xfu);
	wr(cmu + CMU_GATE_IPCLK_USI0, CMU_GATE_ON);
	delay(10);
	wr(sysreg + SYSREG_USI0_MODE, USI_MODE_I2C);
	wr(i2c + USI_CON, rd(i2c + USI_CON) & ~USI_CON_RESET);
	delay(10);
	v = rd(i2c + USI_OPTION);
	wr(i2c + USI_OPTION, (v & ~USI_OPTION_CLKSTOP_ON) | USI_OPTION_CLKREQ_ON);
	wr(gpio + GPP19 + 8, rd(gpio + GPP19 + 8) & ~0xffu);
	wr(gpio + GPP19, (rd(gpio + GPP19) & ~0xffu) | 0x33);
	wr(i2c + CTL, rd(i2c + CTL) | CTL_SW_RST);
	wr(i2c + CTL, rd(i2c + CTL) & ~CTL_SW_RST);
	t_ftl = (rd(i2c + CONF) >> 16) & 7;
	cycles = CRYSTAL_HZ / BUS_HZ - 8 - 2 * t_ftl;
	for (div = 0; div < 256; div++) {
		uint32_t c = cycles / (div + 1);
		if (c < 512 && c > 4) {
			cycle = c - 2;
			break;
		}
	}
	scl = cycle / 2;
	wr(i2c + TRAILING_CTL, TRAILING_COUNT);
	wr(i2c + TIMEOUT, rd(i2c + TIMEOUT) & ~TIMEOUT_EN);
	wr(i2c + CONF, rd(i2c + CONF) | CONF_AUTO_MODE);
	wr(i2c + INT_EN, INT_TRANS_EN);
	wr(i2c + FIFO_CTL, FIFO_RX_EN | FIFO_TX_EN);
	wr(i2c + TIMING_FS1, scl << 24 | scl << 16 | scl << 8);
	wr(i2c + TIMING_FS2, (scl / 2) << 24 | scl << 8 | scl);
	wr(i2c + TIMING_FS3, div << 16 | cycle);
	wr(i2c + TIMING_SLA, scl / 2);
	printf("akita haptic: the bus up (div %u, cycle %u): CONF %#x FIFO_CTL %#x USI_CON %#x USI_OPTION %#x\n", div, cycle,
	    rd(i2c + CONF), rd(i2c + FIFO_CTL), rd(i2c + USI_CON), rd(i2c + USI_OPTION));
	return true;
}

static void
akita_haptic_thread(__unused void *arg, __unused wait_result_t wr_)
{
	uint32_t devid = 0, revid = 0, v[6] = { 0 };

	say_bus("the battery's bus (U-Boot's)", i2c_battery);
	printf("akita haptic: CMU_PERIC1 USI0: mux %#x div %#x gates %#x/%#x/%#x qch %#x; sysreg mode %#x; gpp19 con %#x dat %#x\n",
	    rd(cmu + CMU_MUX_USI0), rd(cmu + CMU_DIV_USI0), rd(cmu + CMU_GATE_IPCLK_USI0), rd(cmu + CMU_GATE_PCLK_USI0),
	    rd(cmu + CMU_GATE_RSTNSYNC_USI0), rd(cmu + CMU_QCH_USI0), rd(sysreg + SYSREG_USI0_MODE), rd(gpio + GPP19),
	    rd(gpio + GPP19 + 4));
	if (!bus_as_found()) {
		printf("akita haptic: the bus not set up by the bootloader: nothing more\n");
		return;
	}
	if (!reg_read(REG_DEVID, &devid)) {
		bus_timing_from_battery();
	}
	/* as the bootloader left it first; if it does not answer (held in reset), out of reset as Google's probe does it:
	 * low 1.5 ms, then high, then 6 ms before the control port is ready (gph2 is HSI1's: said before it is touched) */
	if (!reg_read(REG_DEVID, &devid)) {
		printf("akita haptic: no answer as found; the reset (gph2-3 of pinctrl@12040000) pulsed\n");
		delay(100000);
		printf("akita haptic: gph2 con %#x dat %#x\n", rd(gpio_hsi1 + GPH2), rd(gpio_hsi1 + GPH2 + 4));
		wr(gpio_hsi1 + GPH2 + 4, rd(gpio_hsi1 + GPH2 + 4) & ~(1u << 3));
		wr(gpio_hsi1 + GPH2, (rd(gpio_hsi1 + GPH2) & ~(0xfu << 12)) | (1u << 12));
		reset_touched = true;
		delay(1600);
		wr(gpio_hsi1 + GPH2 + 4, rd(gpio_hsi1 + GPH2 + 4) | (1u << 3));
		delay(7000);
		if (!reg_read(REG_DEVID, &devid)) {
			printf("akita haptic: no answer from the CS40L26 at %#x\n", CS40L26);
			return;
		}
	}
	(void)reg_read(REG_REVID, &revid);
	(void)reg_read(REG_GLOBAL_ENABLES, &v[0]);
	(void)reg_read(REG_PWRMGT_STS, &v[1]);
	(void)reg_read(REG_IRQ1_STATUS, &v[2]);
	(void)reg_read(REG_PM_CUR_STATE, &v[3]);
	(void)reg_read(REG_HALO_STATE, &v[4]);
	(void)reg_read(REG_REQ_ACTIVE, &v[5]);
	printf("akita haptic: CS40L26 devid %#x%s revid %#x; global enables %#x, power status %#x, IRQ1 status %#x; the DSP: "
	    "pm state %#x, halo state %#x, req active %#x\n", devid, (devid & 0xffffff) == DEVID_CS40L26A ? " (CS40L26A)" : "",
	    revid, v[0], v[1], v[2], v[3], v[4], v[5]);
	if (mode >= 2 && (devid & 0xffffff) == DEVID_CS40L26A) {
		uint32_t effect = 0, state = 0, irq = 0;

		PE_parse_boot_argn("haptic_effect", &effect, sizeof(effect));
		effect = effect > 38 ? 0 : effect;
		if (mailbox(MBOX_PREVENT_HIBER)) {
			(void)reg_read(REG_PM_CUR_STATE, &state);
			printf("akita haptic: awake (pm state %#x); the ROM's effect %u\n", state, effect);
			bool played = mailbox(MBOX_ROM_EFFECT(effect));
			delay(600000);
			(void)reg_read(REG_IRQ1_STATUS, &irq);
			(void)reg_read(REG_PWRMGT_STS, &state);
			printf("akita haptic: the ROM's effect %u %s; IRQ1 status %#x, power status %#x\n", effect,
			    played ? "taken" : "NOT taken", irq, state);
			(void)mailbox(MBOX_ALLOW_HIBER);
		}
	}
}

static void
akita_haptic_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(akita_haptic_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

/* (drivers/core/hakios.stubs.c's names) */
void hakios_haptic_init(void);
void hakios_haptic_quiesce(void);
void hakios_haptic_pretend(uint32_t peak);

void hakios_haptic_init(void) {}
void hakios_haptic_pretend(__unused uint32_t peak) {}

void
hakios_haptic_quiesce(void)
{
	/* the chip back into reset, if it was let out */
	if (reset_touched) {
		wr(gpio_hsi1 + GPH2 + 4, rd(gpio_hsi1 + GPH2 + 4) & ~(1u << 3));
	}
}

__startup_func
static void
akita_haptic_init(void)
{
	static thread_call_t start;
	uint64_t deadline;

	if (!PE_parse_boot_argn("haptic", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	cmu = ml_io_map(CMU_PERIC1_PHYS, 0x4000);
	sysreg = ml_io_map(SYSREG_PERIC1_PHYS, 0x1000);
	gpio = ml_io_map(GPIO_PERIC1_PHYS, 0x1000);
	gpio_hsi1 = ml_io_map(GPIO_HSI1_PHYS, 0x1000);
	i2c = ml_io_map(HSI2C0_PHYS, 0x1000);
	i2c_battery = ml_io_map(HSI2C_BATTERY_PHYS, 0x1000);
	if (!cmu || !sysreg || !gpio || !gpio_hsi1 || !i2c) {
		printf("akita haptic: could not map its registers\n");
		return;
	}
	start = thread_call_allocate(akita_haptic_start, NULL);
	clock_interval_to_deadline(22, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_haptic_init);
