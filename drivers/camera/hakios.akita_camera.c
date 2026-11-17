/*
 * The Pixel 8a's cameras, from the start (11-12; devices/akita TODO 39). Android drives them from its camera HAL through
 * LWIS (Google's register-access framework: the sensors' sequences are in the closed HAL, not in Linux), so what is
 * known comes from the device tree (devices/akita/boot-info/fdt.dtb):
 *
 *   sensor-oksoko     the main camera: 0x1a on hsi2c_1 (0x10880000, USI1 of PERIC0, pins gpp2-0/1); 16-bit registers,
 *                     8-bit values; power: S2MPG15 LDO12 (1.8 V, "L12S_CAMIO"), the camera PMIC's ldo5 ldo4 ldo1 ldo3,
 *                     the S2MPG15's GPIO 2 (enable), 1 ms, gpp2-2 (reset), the clock CIS_CLK3 (26 MHz, pin gpp9-0
 *                     function 2), 2 ms. Its EEPROM and actuator at 0x24 on the same bus.
 *   sensor-leshen     the front camera: 0x1a on hsi2c_2 (0x10890000, USI2, gpp4-0/1); LDO12, ldo2, ldo7, 1 ms, gpp4-2,
 *                     CIS_CLK1 (24 MHz, gpp5-0). Its EEPROM at 0x50 (LDO12 alone).
 *   sensor-leshen-uw  the ultra-wide: 0x1a on hsi2c_3 (0x108a0000, USI3, gpp6-0/1); LDO12, ldo2, ldo6, 1 ms, gpp6-3,
 *                     CIS_CLK0 (24 MHz, gpp3-0), 1 ms. Its EEPROM at 0x50.
 *   the camera PMIC   a Dialog SLG51002 at 0x75 on hsi2c_14 (0x10980000, the flashlight's bus: hakios.akita_torch.c), its
 *                     chip select and buck-boost on the S2MPG15's GPIOs 0 and 1; ldo1 2.9 V, ldo2/4/5 2.8, ldo3 1.8,
 *                     ldo6/7 1.2, ldo8 1.1.
 * The I2C controllers are USIs of PERIC0 like the flashlight's (CMU_PERIC0 gates, sysreg_peric0 mode registers 0x00, 0x04,
 * 0x08: samsung,usi-offset); the sensors' clocks CMU_TOP's CIS_CLKn (Google's cal-if zuma cmucal-sfr.c: mux 0x1010 + 4n,
 * divider 0x1810 + 4n, gate 0x2038 + 4n); the S2MPG15 through ACPM's PMIC protocol (hakios.akita_thermal.c: bus 1; type 1
 * its PM block -- L12S_CTRL 0x3a, enable bits 7:6 --, type 0xc its GPIOs -- GPIOn_SET 0x8 + n).
 *
 *   camera=1   a look: the clocks, the USIs' modes, the pins and the S2MPG15's rail and GPIOs as found -- nothing written
 *   camera=2   ... then the camera PMIC woken (S2MPG15 GPIO1 = its buck-boost, GPIO0 = its chip select, as Google's
 *              slg51002-core.c does it) and read: its id, its enables (MATRIX_CONF_A), each LDO's range, setting and state;
 *              the camera I/O rail (S2MPG15 LDO12, 1.8 V) on, the front camera's bus (hsi2c_2) up and its EEPROM (0x50)
 *              read -- no LDO of the PMIC turned on
 *   camera=3   ... and the front camera powered as LWIS does it (sensor-leshen's power-up-seqs: LDO12, ldo2 2.8 V, ldo7
 *              1.2 V, 1 ms, reset gpp4-2 high, the clock CIS_CLK1 on gpp5-0) and its sensor's id read (0x1a: registers
 *              0x0000-0x0003 and 0x0016-0x0017); then powered down again in the reverse order
 * (The PMIC's registers: Google's include/linux/mfd/slg51002.h; LDOn's voltage 1.2 V + 10 mV a step for ldo1-5, 0.4 V +
 * 5 mV for ldo6-8 (slg51002-regulator.c), each within its MINV..MAXV; on/off: MATRIX_CONF_A bit n-1. The S2MPG15's GPIOs:
 * GPIOn_SET bit 6 output enable, bit 5 the level (s2mpg1415-gpio.c).)
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern int akita_acpm_xfer(unsigned channel, uint32_t cmd[4]);
extern int akita_i2c14_xfer(uint8_t addr, const uint8_t *w, unsigned wl, uint8_t *r, unsigned rl) __attribute__((weak));

#define CMU_TOP_PHYS            0x26040000ULL
#define CMU_TOP_CIS_MUX(n)      (0x1010 + 4 * (n))
#define CMU_TOP_CIS_DIV(n)      (0x1810 + 4 * (n))
#define CMU_TOP_CIS_GATE(n)     (0x2038 + 4 * (n))
#define CMU_PERIC0_PHYS         0x10800000ULL
#define SYSREG_PERIC0_PHYS      0x10821000ULL
#define GPIO_PERIC0_PHYS        0x10840000ULL
#define GPP(n)                  ((n) * 0x20)    /* gpp0..gpp15: CON +0, DAT +4, PUD +8, DRV +0xc */

#define PMIC_CHANNEL            2
#define SUBPMIC_BUS             1
#define S2MPG15_PM              0x1
#define S2MPG15_GPIO            0xc
#define S2MPG15_L12S_CTRL       0x3a
#define S2MPG15_GPIO_STATUS1    0x06
#define S2MPG15_GPIO0_SET       0x08
#define S2MPG15_GPIO_OEN        0x40
#define S2MPG15_GPIO_OUT        0x20
#define S2MPG15_L12S_ON         0xc0            /* L12S_CTRL bits 7:6 */

/* the camera PMIC */
#define SLG51002                0x75
#define SLG_PATN_ID_B0          0x1105
#define SLG_MATRIX_CONF_A       0x110d          /* LDOn on: bit n-1 */
#define SLG_FAULT_LOG1          0x1115
#define SLG_STATUS              0x1117
#define SLG_LDO_VSEL(n)         ((const uint16_t[]){ 0x2000, 0x2200, 0x2300, 0x2500, 0x2700, 0x2900, 0x3100, 0x3200 }[(n) - 1])
#define SLG_LDO_MINV(n)         (SLG_LDO_VSEL(n) + 0x60)
#define SLG_LDO_STATUS(n)       (SLG_LDO_VSEL(n) + 0xc1)
#define SLG_STA_VOUT_OK         0x02
#define SLG_STA_ILIM            0x01

/* the HSI2C (as hakios.akita_torch.c's; U-Boot's struct exynos5_hsi2c) */
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
#define HSI2C_BATTERY_PHYS      0x10cb0000ULL   /* U-Boot's bus: its timing copied */
#define CMU_GATE_ON             0x00300000
#define USI_MODE_I2C            0x4

/* the three camera buses: USI n of PERIC0 */
static const struct {
	const char *name;
	uint32_t usi, mux, div, ipclk, pclk, rstn, mode, qch;
	unsigned bank;
} buses[3] = {
	{ "hsi2c_1 (main)", 1, 0x650, 0x1810, 0x2070, 0x2074, 0x20d0, 0x00, 0x3074, 2 },
	{ "hsi2c_2 (front)", 2, 0x660, 0x1814, 0x2078, 0x207c, 0x20d4, 0x04, 0x3078, 4 },
	{ "hsi2c_3 (ultra-wide)", 3, 0x670, 0x1818, 0x2080, 0x2084, 0x20d8, 0x08, 0x307c, 6 },
};

static vm_offset_t cmu_top, cmu_peric0, sysreg, gpio0, hsi2c[3], hsi2c_battery;
static const uint64_t hsi2c_phys[3] = { 0x10880000ULL, 0x10890000ULL, 0x108a0000ULL };

static inline uint32_t
rd(vm_offset_t a)
{
	return *(volatile uint32_t *)a;
}

/* one S2MPG15 register through ACPM: read (WRITE false) or written; false if ACPM or the PMIC said no */
static bool
spmic(bool write, unsigned type, unsigned reg, uint8_t *value)
{
	uint32_t cmd[4] = { SUBPMIC_BUS << 12 | (type & 0xf) << 8 | (reg & 0xff), (write ? 1u : 0u) | (uint32_t)*value << 8 |
	    0xffu << 16, 0, 0 };

	if (akita_acpm_xfer(PMIC_CHANNEL, cmd) != 0 || (cmd[1] >> 24) != 0) {
		return false;
	}
	if (!write) {
		*value = (uint8_t)(cmd[1] >> 8);
	}
	return true;
}


static inline void
wr(vm_offset_t a, uint32_t v)
{
	*(volatile uint32_t *)a = v;
}

/* ---- the camera PMIC, through the flashlight's bus ----------------------------------------------------------------- */

static bool
slg_read(uint16_t reg, uint8_t *buf, unsigned n)
{
	uint8_t a[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };

	return akita_i2c14_xfer != NULL && akita_i2c14_xfer(SLG51002, a, 2, buf, n) == 0;
}

static bool
slg_write(uint16_t reg, uint8_t value)
{
	uint8_t a[3] = { (uint8_t)(reg >> 8), (uint8_t)reg, value }, back = 0;

	return akita_i2c14_xfer != NULL && akita_i2c14_xfer(SLG51002, a, 3, NULL, 0) == 0 && slg_read(reg, &back, 1) &&
	       back == value;
}

/* an S2MPG15 GPIO an output at LEVEL */
static bool
spmic_gpio(unsigned n, bool level)
{
	uint8_t v = S2MPG15_GPIO_OEN | (level ? S2MPG15_GPIO_OUT : 0);

	return spmic(true, S2MPG15_GPIO, S2MPG15_GPIO0_SET + n, &v);
}

/* the PMIC's buck-boost (GPIO1), 2 ms, its chip select (GPIO0), ~10 ms to be ready (slg51002-core.c's probe) */
static bool
slg_wake(void)
{
	uint8_t id[3] = { 0 };

	if (!spmic_gpio(1, true)) {
		return false;
	}
	delay(2000);
	if (!spmic_gpio(0, true)) {
		return false;
	}
	delay(12000);
	if (!slg_read(SLG_PATN_ID_B0, id, 3)) {
		printf("akita camera: SLG51002: no answer at %#x after its wake-up\n", SLG51002);
		return false;
	}
	printf("akita camera: SLG51002 answers: id %#x%s\n", id[2] << 16 | id[1] << 8 | id[0],
	    (id[2] << 16 | id[1] << 8 | id[0]) == 0xafda01 ? " (revision AA)" : "");
	return true;
}

static void
slg_look(const char *when)
{
	uint8_t en = 0, st = 0, flt = 0;

	slg_read(SLG_MATRIX_CONF_A, &en, 1);
	slg_read(SLG_STATUS, &st, 1);
	slg_read(SLG_FAULT_LOG1, &flt, 1);
	printf("akita camera: SLG51002 %s: enables %#x status %#x fault log %#x\n", when, en, st, flt);
	for (unsigned n = 1; n <= 8; n++) {
		uint8_t vsel = 0, mm[2] = { 0 }, ls = 0;
		bool ok = slg_read(SLG_LDO_VSEL(n), &vsel, 1) && slg_read(SLG_LDO_MINV(n), mm, 2) &&
		    slg_read(SLG_LDO_STATUS(n), &ls, 1);
		unsigned base = n <= 5 ? 1200 : 400, step = n <= 5 ? 10 : 5;

		printf("akita camera:   ldo%u: vsel %u (%u mV) range %u..%u (%u..%u mV) status %#x%s%s\n", n, vsel,
		    base + vsel * step, mm[0], mm[1], base + mm[0] * step, base + mm[1] * step, ls,
		    ok ? "" : " (NOT ALL READ)", (en >> (n - 1)) & 1 ? " ON" : "");
	}
}

/* LDOn at MV, then on; false (and left off) if MV is outside its range or it does not say it is up within 5 ms */
static bool
slg_ldo_on(unsigned n, unsigned mv)
{
	unsigned base = n <= 5 ? 1200 : 400, step = n <= 5 ? 10 : 5, code = (mv - base) / step;
	uint8_t mm[2] = { 0 }, en = 0, ls = 0;

	if (mv < base || !slg_read(SLG_LDO_MINV(n), mm, 2) || code < mm[0] || code > mm[1]) {
		printf("akita camera: ldo%u: %u mV is outside its range (%u..%u): not turned on\n", n, mv, mm[0], mm[1]);
		return false;
	}
	if (!slg_write(SLG_LDO_VSEL(n), (uint8_t)code) || !slg_read(SLG_MATRIX_CONF_A, &en, 1) ||
	    !slg_write(SLG_MATRIX_CONF_A, en | (uint8_t)(1u << (n - 1)))) {
		printf("akita camera: ldo%u: the PMIC did not take it\n", n);
		return false;
	}
	for (int i = 0; i < 5; i++) {
		delay(1000);
		if (slg_read(SLG_LDO_STATUS(n), &ls, 1) && (ls & (SLG_STA_VOUT_OK | SLG_STA_ILIM)) == SLG_STA_VOUT_OK) {
			printf("akita camera: ldo%u on at %u mV (code %u)\n", n, mv, code);
			return true;
		}
	}
	printf("akita camera: ldo%u: status %#x after 5 ms (no VOUT_OK, or the current limit): off again\n", n, ls);
	slg_read(SLG_MATRIX_CONF_A, &en, 1);
	slg_write(SLG_MATRIX_CONF_A, en & (uint8_t)~(1u << (n - 1)));
	return false;
}

static void
slg_ldo_off(unsigned n)
{
	uint8_t en = 0;

	if (slg_read(SLG_MATRIX_CONF_A, &en, 1)) {
		slg_write(SLG_MATRIX_CONF_A, en & (uint8_t)~(1u << (n - 1)));
	}
}

/* the camera I/O rail, S2MPG15 LDO12 (1.8 V as found: its voltage bits are left as they are) */
static bool
l12_set(bool on)
{
	uint8_t v = 0;

	if (!spmic(false, S2MPG15_PM, S2MPG15_L12S_CTRL, &v)) {
		return false;
	}
	v = on ? (v | S2MPG15_L12S_ON) : (v & (uint8_t)~S2MPG15_L12S_ON);
	if (!spmic(true, S2MPG15_PM, S2MPG15_L12S_CTRL, &v)) {
		return false;
	}
	printf("akita camera: S2MPG15 LDO12 (L12S_CTRL %#x) %s\n", v, on ? "on" : "off");
	return true;
}

/* ---- the camera buses: USI1-3 of PERIC0 as I2C (as the flashlight's bus is brought up) ----------------------------- */

static unsigned i2c_errors;

static bool
i2c_wait(vm_offset_t b, uint32_t off, uint32_t mask, uint32_t want)
{
	for (unsigned i = 0; i < 10000; i++) {
		if ((rd(b + off) & mask) == want) {
			return true;
		}
		delay(1);
	}
	return false;
}

static bool
i2c_transfer(int bus, uint8_t addr, bool read, uint8_t *buf, unsigned len, bool stop)
{
	vm_offset_t b = hsi2c[bus];
	uint32_t s = 0;
	bool ok = true, ended = false;

	wr(b + TIMEOUT, rd(b + TIMEOUT) & ~(1u << 31));
	wr(b + ADDR, (uint32_t)(addr & 0x3ff) << 10);
	wr(b + CTL, (read ? (1u << 6) : (1u << 7)) | (1u << 0) | (1u << 3));
	wr(b + INT_STAT, 0xf80);
	wr(b + AUTO_CONF, len | (1u << 31) | (stop ? (1u << 17) : 0) | (read ? (1u << 16) : 0));
	for (unsigned i = 0; i < len && ok; i++) {
		if (read) {
			ok = i2c_wait(b, FIFO_STAT, 1u << 24, 0);
			if (ok) {
				buf[i] = (uint8_t)rd(b + RX_DATA);
			}
		} else {
			ok = i2c_wait(b, FIFO_STAT, 1u << 7, 0);
			if (ok) {
				wr(b + TX_DATA, buf[i]);
			}
		}
	}
	for (unsigned t = 0; t < 25000 && !ended; t++) {
		s = rd(b + INT_STAT);
		if (s & 0xf80) {
			wr(b + INT_STAT, s);
			ended = true;
		} else {
			delay(2);
		}
	}
	if (stop) {
		i2c_wait(b, TRANS_STATUS, 1u << 17, 0);
	}
	wr(b + CTL, 1u << 0);
	if (!ok || !ended || (s & 0xf00)) {
		if (i2c_errors++ < 12) {
			printf("akita camera: %s: I2C %s of %u bytes at %#x failed (%s, INT_STAT %#x): FIFO_STAT %#x TRANS_STATUS %#x "
			    "ERROR_STAT %#x CTL %#x CONF %#x AUTO_CONF %#x INT_EN %#x TIMING %#x/%#x/%#x USI_CON %#x USI_OPTION %#x\n",
			    buses[bus].name, read ? "read" : "write", len, addr, !ok ? "the FIFO" : !ended ? "no end" : "an error", s,
			    rd(b + FIFO_STAT), rd(b + TRANS_STATUS), rd(b + ERROR_STAT), rd(b + CTL), rd(b + CONF), rd(b + AUTO_CONF),
			    rd(b + INT_EN), rd(b + TIMING_FS1), rd(b + TIMING_FS2), rd(b + TIMING_FS3), rd(b + USI_CON),
			    rd(b + USI_OPTION));
		}
		return false;
	}
	return true;
}

/* N bytes from 16-bit register REG of the device at ADDR on bus BUS */
static bool
i2c_read16(int bus, uint8_t addr, uint16_t reg, uint8_t *buf, unsigned n)
{
	uint8_t a[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };

	return i2c_transfer(bus, addr, false, a, 2, false) && i2c_transfer(bus, addr, true, buf, n, true);
}

static bool
i2c_bus_up(int bus)
{
	vm_offset_t p = gpio0 + GPP(buses[bus].bank), b;

	if (hsi2c[bus] != 0) {
		return true;                    /* (up already) */
	}
	hsi2c[bus] = b = ml_io_map(hsi2c_phys[bus], 0x1000);
	if (!b || !hsi2c_battery) {
		return false;
	}
	/*
	 * The USI's Q-channel as USI14's is (the flashlight's bus, which works): 0x2 -- CLOCK_REQ held, the handshake off
	 * (cmucal-sfr.c QCH_CON_USIn_USI_QCH: bit 0 ENABLE, 1 CLOCK_REQ, 2 IGNORE_FORCE_PM_EN). With it as found the
	 * controller took the bytes and never ended a transfer (11-17, run-cam2: "no end, INT_STAT 0").
	 */
	printf("akita camera: %s: Q-channel %#x as found\n", buses[bus].name, rd(cmu_peric0 + buses[bus].qch));
	wr(cmu_peric0 + buses[bus].qch, 0x2);
	wr(cmu_peric0 + buses[bus].pclk, CMU_GATE_ON);
	wr(cmu_peric0 + buses[bus].rstn, CMU_GATE_ON);
	wr(cmu_peric0 + buses[bus].ipclk, CMU_GATE_ON);
	delay(10);
	wr(sysreg + buses[bus].mode, USI_MODE_I2C);
	wr(b + USI_CON, rd(b + USI_CON) & ~1u);
	delay(10);
	wr(b + USI_OPTION, (rd(b + USI_OPTION) & ~(1u << 2)) | (1u << 1));
	/* SDA and SCL (pins 0 and 1 of the bank) the I2C's, function 3, pulled up (as found: pull-downs) -- their levels
	 * first, as inputs without pulls: both high if the bus's pull-ups have power (the camera I/O rail) */
	wr(p, rd(p) & ~0xffu);
	wr(p + 8, rd(p + 8) & ~0xffu);
	delay(100);
	printf("akita camera: %s: idle lines as inputs: SDA %u SCL %u (dat %#x)\n", buses[bus].name, rd(p + 4) & 1,
	    (rd(p + 4) >> 1) & 1, rd(p + 4));
	/* (function 3, pulled up, as the tree's hsi2cN-bus states have it: the flashlight's bus is function 2, and function
	 * 2 here left the controller stuck in its address phase, TRANS_STATUS 0xa0006 -- 11-17, run-cam4) */
	wr(p + 8, (rd(p + 8) & ~0xffu) | 0x33);
	wr(p, (rd(p) & ~0xffu) | 0x33);
	wr(b + CTL, rd(b + CTL) | (1u << 31));
	wr(b + CTL, rd(b + CTL) & ~(1u << 31));
	wr(b + TRAILING_CTL, rd(hsi2c_battery + TRAILING_CTL));
	wr(b + TIMEOUT, rd(b + TIMEOUT) & ~(1u << 31));
	wr(b + CONF, rd(hsi2c_battery + CONF) | (1u << 31));
	wr(b + INT_EN, rd(hsi2c_battery + INT_EN));
	wr(b + FIFO_CTL, 0x3);
	wr(b + TIMING_FS1, rd(hsi2c_battery + TIMING_FS1));
	wr(b + TIMING_FS2, rd(hsi2c_battery + TIMING_FS2));
	wr(b + TIMING_FS3, rd(hsi2c_battery + TIMING_FS3));
	wr(b + TIMING_SLA, rd(hsi2c_battery + TIMING_SLA));
	printf("akita camera: %s up: sysreg mode %#x, pins gpp%u con %#x pud %#x\n", buses[bus].name, rd(sysreg + buses[bus].mode),
	    buses[bus].bank, rd(p), rd(p + 8));
	return true;
}

/* a pin of PERIC0 an output at LEVEL (bank, pin) */
static void
pin_out(unsigned bank, unsigned pin, bool level)
{
	vm_offset_t p = gpio0 + GPP(bank);

	wr(p + 4, (rd(p + 4) & ~(1u << pin)) | (level ? 1u << pin : 0));
	wr(p, (rd(p) & ~(0xfu << (4 * pin))) | (1u << (4 * pin)));
}

/* CIS_CLKn from the crystal (mux 0, divider 1: ~24.6 MHz) held on, and its pin (bank, pin) given to it (function 2); or
 * the pin back to an input and the gate to as found */
static void
mclk(unsigned n, unsigned bank, unsigned pin, bool on)
{
	vm_offset_t p = gpio0 + GPP(bank);

	if (on) {
		wr(cmu_top + CMU_TOP_CIS_MUX(n), 0);
		wr(cmu_top + CMU_TOP_CIS_DIV(n), 0);
		wr(cmu_top + CMU_TOP_CIS_GATE(n), CMU_GATE_ON);
		wr(p, (rd(p) & ~(0xfu << (4 * pin))) | (2u << (4 * pin)));
		wr(p + 0xc, (rd(p + 0xc) & ~(0xfu << (4 * pin))) | (1u << (4 * pin)));        /* drive strength 1 (the tree's) */
	} else {
		wr(p, rd(p) & ~(0xfu << (4 * pin)));
		wr(cmu_top + CMU_TOP_CIS_GATE(n), 0x200000);
	}
	printf("akita camera: CIS_CLK%u %s (mux %#x div %#x gate %#x; gpp%u con %#x)\n", n, on ? "on" : "off",
	    rd(cmu_top + CMU_TOP_CIS_MUX(n)), rd(cmu_top + CMU_TOP_CIS_DIV(n)), rd(cmu_top + CMU_TOP_CIS_GATE(n)), bank, rd(p));
}

/* the front camera's EEPROM (0x50 on hsi2c_2): its first 32 bytes */
static void
front_eeprom(void)
{
	uint8_t e[32] = { 0 };

	if (!i2c_read16(1, 0x50, 0x0000, e, sizeof(e))) {
		printf("akita camera: the front camera's EEPROM does not answer\n");
		return;
	}
	printf("akita camera: front EEPROM 0x0000:");
	for (unsigned i = 0; i < sizeof(e); i++) {
		printf(" %02x", e[i]);
	}
	printf("\n");
}

/*
 * The three cameras as the tree's LWIS nodes power them (power-up-seqs; ldoN = the SLG51002's, after S2MPG15 LDO12):
 *   sensor-oksoko (main)        hsi2c_1: ldo5 2.8, ldo4 2.8, ldo1 2.9, ldo3 1.8 V, enable (S2MPG15 GPIO2), 1 ms, reset
 *                               gpp2-2, CIS_CLK3 (26 MHz in the tree) on gpp9-0, 2 ms
 *   sensor-leshen (front)       hsi2c_2: ldo2 2.8, ldo7 1.2 V, 1 ms, reset gpp4-2, CIS_CLK1 (24 MHz) on gpp5-0
 *   sensor-leshen-uw (ultra-w.) hsi2c_3: ldo2 2.8, ldo6 1.2 V, 1 ms, reset gpp6-3, CIS_CLK0 (24 MHz) on gpp3-0, 1 ms
 * (The clocks come from the crystal here, ~24.6 MHz: enough for the sensors to answer; the streaming modes will want
 * the tree's rates.)
 */
static const struct sensor {
	const char *name;
	int bus;
	unsigned ldo[4], mv[4];                 /* in power-up order; 0 ends */
	bool enable;                            /* the S2MPG15's GPIO2 */
	unsigned reset_bank, reset_pin, clk, clk_bank, clk_pin, settle_us;
} sensors[3] = {
	{ "main (oksoko)", 0, { 5, 4, 1, 3 }, { 2800, 2800, 2900, 1800 }, true, 2, 2, 3, 9, 0, 2000 },
	{ "front (leshen)", 1, { 2, 7 }, { 2800, 1200 }, false, 4, 2, 1, 5, 0, 0 },
	{ "ultra-wide (leshen-uw)", 2, { 2, 6 }, { 2800, 1200 }, false, 6, 3, 0, 3, 0, 1000 },
};

static void
sensor_probe(const struct sensor *c)
{
	uint8_t id[4] = { 0 }, id16[2] = { 0 };
	unsigned on = 0;
	bool a, b;

	/* (each step said and given 300 ms first, for the log to get out before a reset: run-cam5 reset somewhere here) */
#define STEP(what) do { printf("akita camera: %s: step: %s\n", c->name, what); delay_for_interval(300, NSEC_PER_MSEC); } while (0)
	if (!i2c_bus_up(c->bus)) {
		printf("akita camera: %s: its bus could not be brought up\n", c->name);
		return;
	}
	STEP("the rails");
	for (on = 0; on < 4 && c->ldo[on] != 0; on++) {
		if (!slg_ldo_on(c->ldo[on], c->mv[on])) {
			break;
		}
	}
	if (on < 4 && c->ldo[on] != 0) {
		goto down;
	}
	if (c->enable) {
		STEP("enable (S2MPG15 GPIO2 high)");
		spmic_gpio(2, true);
	}
	delay(1000);
	STEP("reset released");
	pin_out(c->reset_bank, c->reset_pin, true);
	STEP("the clock");
	mclk(c->clk, c->clk_bank, c->clk_pin, true);
	delay(c->settle_us + 10000);
	STEP("the sensor's id");
	a = i2c_read16(c->bus, 0x1a, 0x0000, id, 4);
	b = i2c_read16(c->bus, 0x1a, 0x0016, id16, 2);
	printf("akita camera: %s sensor at 0x1a: %s 0x0000-3 %02x %02x %02x %02x, 0x0016-7 %02x %02x\n", c->name,
	    a || b ? "ANSWERS" : "no answer", id[0], id[1], id[2], id[3], id16[0], id16[1]);
	/* power-down-seqs: the clock, reset, enable, the rails in reverse (LDO12 stays: the EEPROMs') */
	STEP("power down");
	mclk(c->clk, c->clk_bank, c->clk_pin, false);
	pin_out(c->reset_bank, c->reset_pin, false);
	if (c->enable) {
		spmic_gpio(2, false);
	}
down:
	while (on-- > 0) {
		slg_ldo_off(c->ldo[on]);
	}
#undef STEP
}

static unsigned camera_mode;

static void
look(void)
{
	uint8_t l12 = 0, st1 = 0, st2 = 0, g[3] = { 0 };
	bool ok;

	for (int n = 0; n < 4; n++) {
		printf("akita camera: CIS_CLK%d: mux %#x div %#x gate %#x\n", n, rd(cmu_top + CMU_TOP_CIS_MUX(n)),
		    rd(cmu_top + CMU_TOP_CIS_DIV(n)), rd(cmu_top + CMU_TOP_CIS_GATE(n)));
	}
	for (int i = 0; i < 3; i++) {
		vm_offset_t p = gpio0 + GPP(buses[i].bank);

		printf("akita camera: %s, USI%u: mux %#x/%#x div %#x gates ipclk %#x pclk %#x rstnsync %#x; sysreg mode %#x; pins "
		    "gpp%u con %#x dat %#x pud %#x drv %#x\n", buses[i].name, buses[i].usi, rd(cmu_peric0 + buses[i].mux),
		    rd(cmu_peric0 + buses[i].mux + 4), rd(cmu_peric0 + buses[i].div), rd(cmu_peric0 + buses[i].ipclk),
		    rd(cmu_peric0 + buses[i].pclk), rd(cmu_peric0 + buses[i].rstn), rd(sysreg + buses[i].mode), buses[i].bank,
		    rd(p), rd(p + 4), rd(p + 8), rd(p + 0xc));
	}
	for (unsigned b = 3; b <= 9; b += 2) {
		vm_offset_t p = gpio0 + GPP(b);

		printf("akita camera: gpp%u con %#x dat %#x pud %#x drv %#x\n", b, rd(p), rd(p + 4), rd(p + 8), rd(p + 0xc));
	}
	ok = spmic(false, S2MPG15_PM, S2MPG15_L12S_CTRL, &l12);
	ok = spmic(false, S2MPG15_GPIO, S2MPG15_GPIO_STATUS1, &st1) && ok;
	ok = spmic(false, S2MPG15_GPIO, S2MPG15_GPIO_STATUS1 + 1, &st2) && ok;
	for (int i = 0; i < 3; i++) {
		ok = spmic(false, S2MPG15_GPIO, S2MPG15_GPIO0_SET + i, &g[i]) && ok;
	}
	printf("akita camera: S2MPG15 (%s): L12S_CTRL %#x (the camera I/O rail: enable bits 7:6 %u); GPIO0_SET %#x GPIO1_SET %#x "
	    "GPIO2_SET %#x, status %#x %#x\n", ok ? "answered" : "SOME READS FAILED", l12, l12 >> 6, g[0], g[1], g[2], st1, st2);
}

static void
akita_camera_thread(__unused void *arg, __unused wait_result_t w)
{
	cmu_top = ml_io_map(CMU_TOP_PHYS, 0x8000);
	cmu_peric0 = ml_io_map(CMU_PERIC0_PHYS, 0x4000);
	sysreg = ml_io_map(SYSREG_PERIC0_PHYS, 0x1000);
	gpio0 = ml_io_map(GPIO_PERIC0_PHYS, 0x1000);
	if (!cmu_top || !cmu_peric0 || !sysreg || !gpio0) {
		printf("akita camera: cannot map the clocks, the sysreg or the pins\n");
		return;
	}
	look();
	if (camera_mode < 2) {
		return;
	}
	hsi2c_battery = ml_io_map(HSI2C_BATTERY_PHYS, 0x1000);
	if (akita_i2c14_xfer == NULL) {
		printf("akita camera: no flashlight driver (its bus is the PMIC's): nothing more\n");
		return;
	}
	/* (the flashlight driver brings that bus up in its own time: up to 30 s for it) */
	for (int i = 0; i < 300 && akita_i2c14_xfer(SLG51002, NULL, 0, NULL, 0) != 0; i++) {
		delay_for_interval(100, NSEC_PER_MSEC);
	}
	if (akita_i2c14_xfer(SLG51002, NULL, 0, NULL, 0) != 0) {
		printf("akita camera: the flashlight's bus (the PMIC's) is not up (torch=0?): nothing more\n");
		return;
	}
	if (!slg_wake()) {
		return;
	}
	slg_look("as found");
	if (!l12_set(true)) {
		printf("akita camera: LDO12 could not be turned on: nothing more\n");
		return;
	}
	delay(2000);
	if (!i2c_bus_up(1)) {
		printf("akita camera: hsi2c_2 could not be brought up\n");
		return;
	}
	front_eeprom();
	if (camera_mode >= 3) {
		for (int i = 0; i < 3; i++) {
			sensor_probe(&sensors[i]);
		}
		slg_look("after the three cameras");
	}
}

static void
akita_camera_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(akita_camera_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

__startup_func
static void
akita_camera_init(void)
{
	static thread_call_t start;
	uint32_t mode = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("camera", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	camera_mode = mode;
	start = thread_call_allocate(akita_camera_start, NULL);
	clock_interval_to_deadline(12, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_camera_init);
