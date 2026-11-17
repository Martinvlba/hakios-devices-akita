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
 *   camera=4   ... and the front camera's sensor (IMX712) set to the mode Android's camera uses for it (imx712_init[]:
 *              its writes as LWIS made them, traced on the phone's Android 11-17 -- i2c events, devices/akita notes) and
 *              streamed for 2 s: its frame counter (0x0005) read every 100 ms says whether frames come out
 *   camera=5   ... and the ISP front end (isp-fe): its power domain (pd-ispfe) -- on already when the kernel starts
 *              (configuration/status 0x1), and then left as it is; if it is off, turned on as Google's cal-if does it
 *              (ISPFE_CONFIGURATION 0x15462480 bit 0 written through the PRIV_REG SMC -- PMU_ALIVE takes no plain
 *              stores: one reset the phone --, ISPFE_STATUS bit 0 waited for, then the TZPC restore SMC) and off again
 *              after -- and its CMU, the MIPI link block (CSIS link 0) and the front end read, each step said and given
 *              300 ms first. The front end only from 0x20000 up, where Android's accesses are: a read at offset 0 reset
 *              the phone (run-cam12)
 *   camera=6   ... and the three paths of the front end's DMA looked at: each SysMMU (v9 at 0x1c110000, 0x1c150000,
 *              0x1c190000: CTRL, STATUS, VERSION, CTRL_VM -- what Google's samsung-iommu-v9.c reads) and each S2MPU behind
 *              it (0x1c140000, 0x1c180000, 0x1c1c0000: only VERSION and FAULT_STATUS -- other reads of an S2MPU were an
 *              SError on the GPU's, hakios.akita_mali.c); each read said first
 *   camera=7   (alone) one capture from the front camera into a buffer of ours: Android's PHY/CSIS/front-end setup and
 *              its per-frame channel-0 writes replayed by the CPU (front_capture())
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
#include <vm/vm_kern.h>

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

/*
 * The IMX712's mode as Android's camera HAL sets it for the front camera (traced 11-17 on the phone's Android: the i2c
 * writes to 0x1a on hsi2c_2 from power-up to stream-on): EXCK 24.57 MHz (0x0136/7 = 0x1893: the crystal clock this driver
 * gives it), RAW10 (0x0112/3), 4 lanes (0x0114), 4208 x 3072 (0x034c-f), line 5192 / frame 3128 (0x0342/0x0340), the
 * link at 4423 Mbit/s (0x0820/1). Each entry: N, the register (2 bytes), N values; 0 ends. After entry 104 (0x500f)
 * Android waited ~20 ms; then exposure/gain/frame length under the group hold (0x0104), the temperature sensor on
 * (0x0138), and ~7 ms later stream on (0x0100).
 */
static unsigned camera_mode;

/* ---- the ISP front end (isp-fe: the MIPI receivers, the front end and its DMA) ----------------------------------- */

#define PMU_ISPFE_CONF          0x15462480ULL   /* (+4 STATUS) */
#define CMU_ISPFE_PHYS          0x1c000000ULL
#define ISPFE_CSIS_PHYS         0x1c040000ULL   /* csis-link-phy: 12 links of 64 KB */
#define ISPFE_FE_PHYS           0x1c300000ULL
#define CMU_TOP_ISPFE_MUX       0x10bc
#define CMU_TOP_ISPFE_DIV       0x18b4
#define CMU_TOP_ISPFE_GATE      0x20e4
#define SMC_PREPARE_PD_ONOFF    0x82000410ULL
#define PD_GET_IN_DOWN          0
#define PD_WAKEUP_DOWN          1
#define PD_TZPC_GROUP           2
#define ISPFE_SMC_ID            0x1c010204ULL
#define SMC_PRIV_REG            0x82000504ULL   /* (address, 1 = write, value): PMU_ALIVE is written only through EL3 */
#define PRIV_REG_WRITE          1

static uint64_t
cam_smc(uint64_t f, uint64_t a1, uint64_t a2, uint64_t a3)
{
	register uint64_t x0 __asm__("x0") = f, x1 __asm__("x1") = a1, x2 __asm__("x2") = a2, x3 __asm__("x3") = a3;

	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : : "x4", "x5", "x6", "x7", "x8", "x9", "x10",
	    "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

static void
ispfe_dma_look(void)
{
	static const uint64_t sysmmu[3] = { 0x1c110000ULL, 0x1c150000ULL, 0x1c190000ULL };
	static const uint64_t s2mpu[3] = { 0x1c140000ULL, 0x1c180000ULL, 0x1c1c0000ULL };

	for (int i = 0; i < 3; i++) {
		vm_offset_t m = ml_io_map(sysmmu[i], 0x10000), p = ml_io_map(s2mpu[i], 0x3000);

		printf("akita camera: isp-fe S%d: reading its SysMMU (%#llx)\n", i, sysmmu[i]);
		delay_for_interval(300, NSEC_PER_MSEC);
		printf("akita camera: isp-fe S%d: SysMMU CTRL %#x STATUS %#x VERSION %#x CTRL_VM %#x FLPT_BASE_VM %#x\n", i, rd(m),
		    rd(m + 0x8), rd(m + 0x34), rd(m + 0x8000), rd(m + 0x8404));
		printf("akita camera: isp-fe S%d: reading its S2MPU (%#llx): VERSION, FAULT_STATUS\n", i, s2mpu[i]);
		delay_for_interval(300, NSEC_PER_MSEC);
		printf("akita camera: isp-fe S%d: S2MPU VERSION %#x FAULT_STATUS %#x\n", i, rd(p + 0x60), rd(p + 0x2000));
	}
}

static void
ispfe_test(void)
{
	vm_offset_t pmu = ml_io_map(PMU_ISPFE_CONF & ~0xfffULL, 0x1000) + (PMU_ISPFE_CONF & 0xfff), cmu, csis, fe;
	uint32_t st = 0;
	uint64_t r;
	bool was_on, up;

#define ISTEP(what) do { printf("akita camera: isp-fe: step: %s\n", what); delay_for_interval(300, NSEC_PER_MSEC); } while (0)
	printf("akita camera: isp-fe: CMU_TOP ISPFE_NOC mux %#x div %#x gate %#x; pd-ispfe configuration %#x status %#x\n",
	    rd(cmu_top + CMU_TOP_ISPFE_MUX), rd(cmu_top + CMU_TOP_ISPFE_DIV), rd(cmu_top + CMU_TOP_ISPFE_GATE), rd(pmu), rd(pmu + 4));
	was_on = up = (rd(pmu + 4) & 1u) != 0;
	if (!was_on) {
		/* (PMU_ALIVE, 0x15460000-, takes writes only through the secure monitor, as cal-if's pmucal_write_reg() does: a
		 * plain store reset the phone, 11-17 run-cam10) */
		ISTEP("power domain on (PRIV_REG SMC)");
		cam_smc(SMC_PRIV_REG, PMU_ISPFE_CONF, PRIV_REG_WRITE, rd(pmu) | 1u);
		for (int i = 0; i < 100 && !up; i++) {
			delay(100);
			st = rd(pmu + 4);
			up = (st & 1u) != 0;
		}
		if (!up) {
			printf("akita camera: isp-fe: the power domain did not come up (status %#x after 10 ms)\n", st);
			cam_smc(SMC_PRIV_REG, PMU_ISPFE_CONF, PRIV_REG_WRITE, rd(pmu) & ~1u);
			return;
		}
		ISTEP("TZPC restore SMC");
		r = cam_smc(SMC_PREPARE_PD_ONOFF, PD_WAKEUP_DOWN, ISPFE_SMC_ID, PD_TZPC_GROUP);
		printf("akita camera: isp-fe: power domain on (status %#x); TZPC restore SMC: %#llx\n", rd(pmu + 4), r);
	} else {
		printf("akita camera: isp-fe: the power domain is on already (left so: no SMCs, not turned off)\n");
	}
	cmu = ml_io_map(CMU_ISPFE_PHYS, 0x4000);
	csis = ml_io_map(ISPFE_CSIS_PHYS, 0x1000);
	fe = ml_io_map(ISPFE_FE_PHYS + 0x20000, 0x1000);       /* (offset 0 reset the phone, run-cam12: Android's
	                                                        * accesses start at 0x20000; its reads were at 0x2003c) */
	ISTEP("read CMU_ISPFE");
	printf("akita camera: isp-fe: CMU_ISPFE noc user mux %#x, div dcphy %#x nocp %#x, cmu gate %#x\n", rd(cmu + 0x600),
	    rd(cmu + 0x1800), rd(cmu + 0x1804), rd(cmu + 0x200c));
	ISTEP("read the CSIS link block");
	printf("akita camera: isp-fe: CSIS link 0: %#x %#x %#x %#x\n", rd(csis), rd(csis + 4), rd(csis + 8), rd(csis + 0xc));
	ISTEP("read the front end");
	printf("akita camera: isp-fe: front end +0x20000: %#x +0x20008: %#x +0x20010: %#x +0x2003c: %#x\n", rd(fe), rd(fe + 8),
	    rd(fe + 0x10), rd(fe + 0x3c));
	if (!was_on) {
		ISTEP("power domain off");
		r = cam_smc(SMC_PREPARE_PD_ONOFF, PD_GET_IN_DOWN, ISPFE_SMC_ID, PD_TZPC_GROUP);
		cam_smc(SMC_PRIV_REG, PMU_ISPFE_CONF, PRIV_REG_WRITE, rd(pmu) & ~1u);
		printf("akita camera: isp-fe: off again (TZPC save SMC %#llx; status %#x)\n", r, rd(pmu + 4));
	}
#undef ISTEP
}

static const uint8_t imx712_init[] = {
	1, 0x01, 0x36, 0x18,
	1, 0x01, 0x37, 0x93,
	1, 0x30, 0x40, 0x01,
	1, 0x30, 0x41, 0x01,
	1, 0x01, 0x01, 0x03,
	1, 0x30, 0x0b, 0x00,
	1, 0x43, 0x50, 0x3c,
	1, 0x45, 0x16, 0x3e,
	1, 0x45, 0x18, 0x56,
	1, 0x45, 0x1a, 0x02,
	1, 0x45, 0x1b, 0xdb,
	1, 0x45, 0x27, 0x00,
	1, 0x45, 0x28, 0xff,
	1, 0x45, 0xbe, 0x4b,
	1, 0x45, 0xbf, 0xcc,
	1, 0x45, 0xc6, 0x4b,
	1, 0x45, 0xc7, 0xcc,
	1, 0x47, 0x02, 0x0a,
	1, 0x48, 0x28, 0x08,
	1, 0x48, 0x4c, 0x2b,
	1, 0x48, 0x4d, 0x2b,
	1, 0x48, 0x4e, 0x2b,
	1, 0x48, 0x4f, 0x2b,
	1, 0x48, 0x50, 0x20,
	1, 0x48, 0x51, 0x20,
	1, 0x48, 0x52, 0x20,
	1, 0x48, 0x53, 0x20,
	1, 0x48, 0x56, 0x05,
	1, 0x48, 0x57, 0x05,
	1, 0x48, 0x58, 0x05,
	1, 0x48, 0x59, 0x05,
	1, 0x48, 0x5a, 0x04,
	1, 0x48, 0x5b, 0x04,
	1, 0x48, 0x5c, 0x04,
	1, 0x48, 0x5d, 0x04,
	1, 0x48, 0x79, 0x00,
	1, 0x48, 0x99, 0x0a,
	1, 0x48, 0xa9, 0x2e,
	1, 0x6a, 0x10, 0x0a,
	1, 0x70, 0xb0, 0x00,
	1, 0x70, 0xab, 0xff,
	1, 0x40, 0x12, 0x00,
	1, 0x40, 0x13, 0xdf,
	1, 0x70, 0x8b, 0x00,
	1, 0x41, 0x08, 0x01,
	1, 0x41, 0x09, 0x07,
	1, 0x30, 0xa1, 0x01,
	1, 0x01, 0x06, 0x01,
	1, 0x01, 0x12, 0x0a,
	1, 0x01, 0x13, 0x0a,
	1, 0x01, 0x14, 0x03,
	1, 0x03, 0x42, 0x14,
	1, 0x03, 0x43, 0x48,
	1, 0x03, 0x40, 0x0c,
	1, 0x03, 0x41, 0x38,
	1, 0x03, 0x44, 0x00,
	1, 0x03, 0x45, 0x00,
	1, 0x03, 0x46, 0x00,
	1, 0x03, 0x47, 0x18,
	1, 0x03, 0x48, 0x10,
	1, 0x03, 0x49, 0x6f,
	1, 0x03, 0x4a, 0x0c,
	1, 0x03, 0x4b, 0x17,
	1, 0x09, 0x00, 0x00,
	1, 0x09, 0x01, 0x11,
	1, 0x09, 0x02, 0x00,
	1, 0x31, 0x30, 0x01,
	1, 0x03, 0x4c, 0x10,
	1, 0x03, 0x4d, 0x70,
	1, 0x03, 0x4e, 0x0c,
	1, 0x03, 0x4f, 0x00,
	1, 0x03, 0x01, 0x04,
	1, 0x03, 0x03, 0x02,
	1, 0x03, 0x05, 0x03,
	1, 0x03, 0x06, 0x00,
	1, 0x03, 0x07, 0x77,
	1, 0x03, 0x0b, 0x01,
	1, 0x03, 0x0d, 0x03,
	1, 0x03, 0x0e, 0x00,
	1, 0x03, 0x0f, 0x87,
	1, 0x07, 0x00, 0x00,
	1, 0x07, 0x01, 0x30,
	1, 0x08, 0x20, 0x11,
	1, 0x08, 0x21, 0x47,
	1, 0x31, 0x00, 0x05,
	1, 0x70, 0x13, 0x01,
	1, 0x02, 0x02, 0x0c,
	1, 0x02, 0x03, 0x2e,
	1, 0x02, 0x04, 0x00,
	1, 0x02, 0x05, 0x00,
	1, 0x02, 0x0e, 0x01,
	1, 0x02, 0x0f, 0x00,
	1, 0x0b, 0x06, 0x01,
	1, 0x48, 0x4c, 0x54,
	1, 0x48, 0x4d, 0x54,
	1, 0x48, 0x4e, 0x54,
	1, 0x48, 0x4f, 0x54,
	1, 0x48, 0x50, 0x20,
	1, 0x48, 0x51, 0x20,
	1, 0x48, 0x52, 0x20,
	1, 0x48, 0x53, 0x20,
	1, 0x50, 0x0c, 0x01,
	1, 0x50, 0x0d, 0x72,
	1, 0x50, 0x0e, 0x00,
	1, 0x50, 0x0f, 0x28,
	1, 0x01, 0x04, 0x01,
	1, 0x02, 0x0e, 0x01,
	1, 0x02, 0x0f, 0x00,
	1, 0x02, 0x04, 0x03,
	1, 0x02, 0x05, 0xc0,
	1, 0x02, 0x02, 0x03,
	1, 0x02, 0x03, 0x0e,
	1, 0x03, 0x40, 0x0c,
	1, 0x03, 0x41, 0x73,
	1, 0x30, 0x04, 0x00,
	1, 0x01, 0x04, 0x00,
	1, 0x01, 0x38, 0x01,
	0
};

static bool
i2c_write16(int bus, uint8_t addr, uint16_t reg, const uint8_t *vals, unsigned n)
{
	uint8_t b[8];

	if (n > sizeof(b) - 2) {
		return false;
	}
	b[0] = (uint8_t)(reg >> 8);
	b[1] = (uint8_t)reg;
	memcpy(b + 2, vals, n);
	return i2c_transfer(bus, addr, false, b, 2 + n, true);
}

/* the sensor (powered, out of reset, clocked) set to Android's mode, streamed 2 s with its frame counter read, stopped */
/* the IMX712's mode written (not streaming yet); false if any write failed */
static bool
imx712_mode(const struct sensor *c)
{
	unsigned k = 0, i = 0, failed = 0;

	while (imx712_init[i] != 0) {
		unsigned n = imx712_init[i];
		uint16_t reg = (uint16_t)(imx712_init[i + 1] << 8 | imx712_init[i + 2]);

		if (!i2c_write16(c->bus, 0x1a, reg, &imx712_init[i + 3], n)) {
			failed++;
		}
		i += 3 + n;
		if (++k == 105) {
			delay(20000);           /* (Android's pause after 0x500f) */
		}
	}
	printf("akita camera: %s: the IMX712's mode written (%u registers, %u failed)\n", c->name, k, failed);
	return failed == 0;
}

static void
imx712_stream_test(const struct sensor *c)
{
	uint8_t one = 1, zero = 0, fc = 0, last = 0;

	imx712_mode(c);
	delay(7000);
	if (!i2c_write16(c->bus, 0x1a, 0x0100, &one, 1)) {
		printf("akita camera: %s: stream on failed\n", c->name);
		return;
	}
	for (int t = 0; t < 20; t++) {
		delay_for_interval(100, NSEC_PER_MSEC);
		if (i2c_read16(c->bus, 0x1a, 0x0005, &fc, 1)) {
			printf("akita camera: %s: streaming %d ms: frame counter %u%s\n", c->name, (t + 1) * 100, fc,
			    t > 0 && fc != last ? "" : " (not moving)");
			last = fc;
		}
	}
	i2c_write16(c->bus, 0x1a, 0x0100, &zero, 1);
	printf("akita camera: %s: stream off\n", c->name);
}

/*
 * camera=7: one capture from the front camera into memory, the way Android's camera does it (traced 11-17 on the
 * phone's Android, devices/akita notes): the MIPI PHY and CSIS link 6 and the front end set up as Android did before
 * stream-on (ispfe_front_setup[], its writes in order with its pauses), then what Android's PDMA writes each frame for
 * the front camera -- WDMA channel 0 (fe+0x20500: +0x140 enable, +0x150 format 4 = RAW10, +0x154 stride 0x14c0 = 4208
 * px, +0x1c0 the frame's address, +0x110 a small second buffer) and the 0x37 block (size 0x0c001070 = 3072 x 4208, crop,
 * the start) -- written here by the CPU, with OUR buffers' physical addresses (the SysMMUs are off: no translation).
 * Android's processing blocks (fe 0x54xxx/0x55xxx, 1279 writes) are left out: some of them hold Android's buffer
 * addresses, which here would be raw physical addresses. Then the sensor streams 300 ms, and the buffer is examined.
 */
struct ispfe_write { uint32_t pa, val, delay_us; };
static const struct ispfe_write ispfe_front_setup[] = {
	{ 0x1c040004, 0x00005002, 0 },
	{ 0x1c050004, 0x00005002, 0 },
	{ 0x1c060004, 0x00005002, 0 },
	{ 0x1c070004, 0x00005002, 0 },
	{ 0x1c080004, 0x00005002, 0 },
	{ 0x1c090004, 0x00005002, 0 },
	{ 0x1c0a0004, 0x00005002, 0 },
	{ 0x1c0b0004, 0x00005002, 0 },
	{ 0x1c0c0004, 0x00005002, 0 },
	{ 0x1c0d0004, 0x00005002, 0 },
	{ 0x1c0e0004, 0x00005002, 0 },
	{ 0x1c0f0004, 0x00005002, 0 },
	{ 0x1c320008, 0x0000000f, 0 },
	{ 0x1c320010, 0x00000001, 0 },
	{ 0x1c35000c, 0x00000000, 1694 },
	{ 0x1c350000, 0x00000000, 0 },
	{ 0x1c350004, 0x00000000, 0 },
	{ 0x1c350008, 0x00000000, 0 },
	{ 0x1c35000c, 0x00000c10, 0 },
	{ 0x1c350010, 0x00dddead, 0 },
	{ 0x1c350014, 0x00000000, 0 },
	{ 0x1c350018, 0x00000000, 0 },
	{ 0x1c35001c, 0x00000000, 0 },
	{ 0x1c350020, 0x00020002, 0 },
	{ 0x1c350024, 0x00000000, 0 },
	{ 0x1c350028, 0x00000000, 0 },
	{ 0x1c35002c, 0x00000000, 0 },
	{ 0x1c350030, 0x00000000, 0 },
	{ 0x1c350034, 0x00000000, 0 },
	{ 0x1c350038, 0x00000000, 0 },
	{ 0x1c35003c, 0x00000000, 0 },
	{ 0x1c350040, 0x00000000, 0 },
	{ 0x1c350044, 0x00000000, 0 },
	{ 0x1c350048, 0x00000000, 0 },
	{ 0x1c35004c, 0x00000000, 0 },
	{ 0x1c350050, 0x00000000, 0 },
	{ 0x1c350054, 0x00000000, 0 },
	{ 0x1c350294, 0x00000006, 0 },
	{ 0x1c350298, 0x00000064, 0 },
	{ 0x1c35029c, 0x000001f4, 0 },
	{ 0x1c3502a0, 0x00001388, 0 },
	{ 0x1c3502a4, 0x00000000, 0 },
	{ 0x1c3502a8, 0x0000005a, 0 },
	{ 0x1c3502ac, 0x00000190, 0 },
	{ 0x1c35000c, 0x00000c11, 0 },
	{ 0x1c350058, 0x00000000, 0 },
	{ 0x1c35005c, 0x00000000, 0 },
	{ 0x1c350060, 0x00000000, 0 },
	{ 0x1c350064, 0x3fffffff, 0 },
	{ 0x1c350068, 0x00000000, 0 },
	{ 0x1c350088, 0x00000000, 0 },
	{ 0x1c35008c, 0x00000000, 0 },
	{ 0x1c350090, 0x00000000, 0 },
	{ 0x1c350094, 0x3fffffff, 0 },
	{ 0x1c350098, 0x00000000, 0 },
	{ 0x1c3500b8, 0x00000000, 0 },
	{ 0x1c3500bc, 0x00000000, 0 },
	{ 0x1c3500c0, 0x00000000, 0 },
	{ 0x1c3500c4, 0x3fffffff, 0 },
	{ 0x1c3500c8, 0x00000000, 0 },
	{ 0x1c3500e8, 0x00000000, 0 },
	{ 0x1c3500ec, 0x00000000, 0 },
	{ 0x1c3500f0, 0x00000000, 0 },
	{ 0x1c3500f4, 0x3fffffff, 0 },
	{ 0x1c3500f8, 0x00000000, 0 },
	{ 0x1c350118, 0x00000000, 0 },
	{ 0x1c35011c, 0x00000000, 0 },
	{ 0x1c350120, 0x00000000, 0 },
	{ 0x1c350124, 0x3fffffff, 0 },
	{ 0x1c350128, 0x00000000, 0 },
	{ 0x1c350148, 0x00000000, 0 },
	{ 0x1c35014c, 0x00000000, 0 },
	{ 0x1c350150, 0x00000000, 0 },
	{ 0x1c350154, 0x000003ff, 0 },
	{ 0x1c350158, 0x00000000, 0 },
	{ 0x1c350160, 0x00000000, 0 },
	{ 0x1c350164, 0x00000000, 0 },
	{ 0x1c350168, 0x00000000, 0 },
	{ 0x1c35016c, 0x000003ff, 0 },
	{ 0x1c350170, 0x00000000, 0 },
	{ 0x1c350178, 0x00000000, 0 },
	{ 0x1c35017c, 0x00000000, 0 },
	{ 0x1c350180, 0x00000000, 0 },
	{ 0x1c350184, 0x000003ff, 0 },
	{ 0x1c350188, 0x00000000, 0 },
	{ 0x1c350190, 0x00000000, 0 },
	{ 0x1c350194, 0x00000000, 0 },
	{ 0x1c350198, 0x00000000, 0 },
	{ 0x1c35019c, 0x000003ff, 0 },
	{ 0x1c3501a0, 0x00000000, 0 },
	{ 0x1c330f00, 0x00000001, 0 },
	{ 0x1c330c00, 0x00000001, 0 },
	{ 0x1c330c04, 0x00000001, 0 },
	{ 0x1c330c0c, 0x00000000, 0 },
	{ 0x1c330d00, 0x00000001, 0 },
	{ 0x1c330d04, 0x00000001, 0 },
	{ 0x1c330d0c, 0x00000000, 0 },
	{ 0x1c330100, 0x00000053, 0 },
	{ 0x1c320094, 0x3fffffff, 1145 },
	{ 0x1c32011c, 0x00000001, 0 },
	{ 0x1c040004, 0x00005002, 0 },
	{ 0x1c050004, 0x00005002, 0 },
	{ 0x1c060004, 0x00005002, 0 },
	{ 0x1c070004, 0x00005002, 0 },
	{ 0x1c080004, 0x00005002, 0 },
	{ 0x1c090004, 0x00005002, 0 },
	{ 0x1c0a0004, 0x00005002, 0 },
	{ 0x1c0b0004, 0x00005002, 0 },
	{ 0x1c0c0004, 0x00005002, 0 },
	{ 0x1c0d0004, 0x00005002, 0 },
	{ 0x1c0e0004, 0x00005002, 0 },
	{ 0x1c0f0004, 0x00005002, 0 },
	{ 0x1c10131c, 0x00000200, 0 },
	{ 0x1c10141c, 0x00000200, 0 },
	{ 0x1c10151c, 0x00000200, 0 },
	{ 0x1c10181c, 0x00000200, 0 },
	{ 0x1c10191c, 0x00000200, 0 },
	{ 0x1c101a1c, 0x00000200, 0 },
	{ 0x1c10231c, 0x00000200, 0 },
	{ 0x1c10241c, 0x00000200, 0 },
	{ 0x1c10251c, 0x00000200, 0 },
	{ 0x1c10281c, 0x00000200, 0 },
	{ 0x1c10291c, 0x00000200, 0 },
	{ 0x1c102a1c, 0x00000200, 0 },
	{ 0x1c10321c, 0x00000200, 0 },
	{ 0x1c10331c, 0x00000200, 0 },
	{ 0x1c10341c, 0x00000200, 0 },
	{ 0x1c10381c, 0x00000200, 0 },
	{ 0x1c10391c, 0x00000200, 0 },
	{ 0x1c103a1c, 0x00000200, 0 },
	{ 0x1c337308, 0x000007fc, 22984 },
	{ 0x1c33730c, 0x000007f8, 0 },
	{ 0x1c330c38, 0x00000040, 0 },
	{ 0x1c330c3c, 0x00000020, 0 },
	{ 0x1c330d38, 0x00000040, 0 },
	{ 0x1c330d3c, 0x00000020, 0 },
	{ 0x1c330c08, 0x0000001a, 0 },
	{ 0x1c330d08, 0x00000002, 0 },
	{ 0x1c3500f0, 0x103ffffe, 0 },
	{ 0x1c320410, 0x00000007, 764 },
	{ 0x1c320430, 0x00000012, 0 },
	{ 0x1c320444, 0x00003fff, 0 },
	{ 0x1c320458, 0x00000003, 0 },
	{ 0x1c3200b0, 0x00000001, 0 },
	{ 0x1c320478, 0x0007ca00, 0 },
	{ 0x1c32047c, 0x000015f0, 0 },
	{ 0x1c320460, 0x00000001, 0 },
	{ 0x1c320464, 0x00000001, 0 },
	{ 0x1c32046c, 0x00000001, 0 },
	{ 0x1c320470, 0x00000001, 0 },
	{ 0x1c32048c, 0x06000200, 0 },
	{ 0x1c320488, 0x00000000, 0 },
	{ 0x1c320474, 0x0c001070, 0 },
	{ 0x1c32042c, 0x0000001b, 0 },
	{ 0x1c328804, 0x00000000, 0 },
	{ 0x1c328800, 0x00000001, 0 },
	{ 0x1c320418, 0x00000001, 0 },
	{ 0x1c103100, 0x00000000, 0 },
	{ 0x1c103600, 0x00000000, 0 },
	{ 0x1c103200, 0x00000000, 0 },
	{ 0x1c103300, 0x00000000, 0 },
	{ 0x1c103400, 0x00000000, 0 },
	{ 0x1c103500, 0x00000000, 0 },
	{ 0x1c101000, 0x00000010, 0 },
	{ 0x1c101004, 0x00000110, 0 },
	{ 0x1c101008, 0x00003223, 0 },
	{ 0x1c10100c, 0x00000000, 0 },
	{ 0x1c101010, 0x00000200, 0 },
	{ 0x1c101014, 0x00000000, 0 },
	{ 0x1c103104, 0x00001334, 0 },
	{ 0x1c103108, 0x00000005, 0 },
	{ 0x1c10310c, 0x0000f0ae, 0 },
	{ 0x1c103110, 0x00000002, 0 },
	{ 0x1c103114, 0x00008600, 0 },
	{ 0x1c103118, 0x00004000, 0 },
	{ 0x1c103130, 0x00000103, 0 },
	{ 0x1c103140, 0x00000001, 0 },
	{ 0x1c103100, 0x00000003, 0 },
	{ 0x1c103600, 0x00000003, 0 },
	{ 0x1c103204, 0x00001334, 0 },
	{ 0x1c103208, 0x0000000d, 0 },
	{ 0x1c10320c, 0x000020ae, 0 },
	{ 0x1c103210, 0x00000302, 0 },
	{ 0x1c103214, 0x00008600, 0 },
	{ 0x1c103218, 0x00004000, 0 },
	{ 0x1c103230, 0x00000137, 0 },
	{ 0x1c103234, 0x00000003, 0 },
	{ 0x1c103240, 0x00000001, 0 },
	{ 0x1c103250, 0x0000081a, 0 },
	{ 0x1c103200, 0x00000001, 0 },
	{ 0x1c103304, 0x00001334, 0 },
	{ 0x1c103308, 0x0000000d, 0 },
	{ 0x1c10330c, 0x000020ae, 0 },
	{ 0x1c103310, 0x00000302, 0 },
	{ 0x1c103314, 0x00008600, 0 },
	{ 0x1c103318, 0x00004000, 0 },
	{ 0x1c103330, 0x00000137, 0 },
	{ 0x1c103334, 0x00000003, 0 },
	{ 0x1c103340, 0x00000001, 0 },
	{ 0x1c103350, 0x0000081a, 0 },
	{ 0x1c103300, 0x00000001, 0 },
	{ 0x1c103404, 0x00001334, 0 },
	{ 0x1c103408, 0x0000000d, 0 },
	{ 0x1c10340c, 0x000020ae, 0 },
	{ 0x1c103410, 0x00000302, 0 },
	{ 0x1c103414, 0x00008600, 0 },
	{ 0x1c103418, 0x00004000, 0 },
	{ 0x1c103430, 0x00000137, 0 },
	{ 0x1c103434, 0x00000003, 0 },
	{ 0x1c103440, 0x00000001, 0 },
	{ 0x1c103450, 0x0000081a, 0 },
	{ 0x1c103400, 0x00000001, 0 },
	{ 0x1c103504, 0x00001334, 0 },
	{ 0x1c103508, 0x0000000d, 0 },
	{ 0x1c10350c, 0x000020ae, 0 },
	{ 0x1c103510, 0x00000302, 0 },
	{ 0x1c103514, 0x00008600, 0 },
	{ 0x1c103518, 0x00004000, 0 },
	{ 0x1c103530, 0x00000137, 0 },
	{ 0x1c103534, 0x00000003, 0 },
	{ 0x1c103540, 0x00000001, 0 },
	{ 0x1c103550, 0x0000081a, 0 },
	{ 0x1c103500, 0x00000001, 0 },
	{ 0x1c0a0690, 0x00000000, 0 },
	{ 0x1c0a0040, 0x000030ac, 0 },
	{ 0x1c0a0044, 0x0c001070, 0 },
	{ 0x1c0a0050, 0x0000b048, 0 },
	{ 0x1c0a0054, 0x00011070, 0 },
	{ 0x1c0a0600, 0x7fff7fff, 0 },
	{ 0x1c0a000c, 0x0000000f, 0 },
	{ 0x1c0a0004, 0x00005f01, 0 },
	{ 0x1c0a0020, 0x00000000, 0 },
	{ 0x1c0a0028, 0x00000000, 0 },
	{ 0x1c0a0010, 0x01fff1ff, 0 },
	{ 0x1c0a0018, 0x0000007e, 0 },
	{ 0x1c33730c, 0x000007fc, 0 },
	{ 0x1c3500f0, 0x103fffff, 0 },
	{ 0x1c320430, 0x0000001a, 0 },
	{ 0x1c320430, 0x0000001b, 0 },
	{ 0x1c32008c, 0x00000001, 3784 },
	{ 0x1c320404, 0x00000000, 0 },
	{ 0x1c320424, 0x00000008, 0 },
	{ 0x1c320438, 0x00000000, 0 },
	{ 0x1c32044c, 0x00000000, 0 },
	{ 0x1c320110, 0x00000000, 0 },
	{ 0x1c350058, 0x00000000, 0 },
	{ 0x1c350088, 0x00000000, 0 },
	{ 0x1c3500b8, 0x00000000, 0 },
	{ 0x1c3500e8, 0x00000001, 0 },
	{ 0x1c350118, 0x00000000, 0 },
	{ 0x1c3501ac, 0x00000000, 0 },
	{ 0x1c331300, 0x00000000, 0 },
	{ 0x1c333300, 0x00000000, 0 },
	{ 0x1c335300, 0x00000000, 0 },
	{ 0x1c337300, 0x00000004, 0 },
	{ 0x1c339300, 0x00000000, 0 },
};
static const struct { uint32_t off, val; } ispfe_front_37[] = {
	{ 0x37050, 0x00000000 },
	{ 0x37400, 0x00000014 },
	{ 0x37404, 0x00000030 },
	{ 0x37500, 0x00000002 },
	{ 0x37504, 0x00000000 },
	{ 0x37508, 0x04000400 },
	{ 0x3750c, 0x04000400 },
	{ 0x37510, 0x04000400 },
	{ 0x37514, 0x04000400 },
	{ 0x37518, 0x04000400 },
	{ 0x3751c, 0x04000400 },
	{ 0x37520, 0x04000400 },
	{ 0x37524, 0x04000400 },
	{ 0x37528, 0x04000400 },
	{ 0x3752c, 0x04000400 },
	{ 0x37530, 0x04000400 },
	{ 0x37534, 0x04000400 },
	{ 0x37538, 0x04000400 },
	{ 0x3753c, 0x04000400 },
	{ 0x37540, 0x04000400 },
	{ 0x37544, 0x04000400 },
	{ 0x37548, 0x04000400 },
	{ 0x3754c, 0x04000400 },
	{ 0x37550, 0x04000400 },
	{ 0x37554, 0x04000400 },
	{ 0x37558, 0x00000100 },
	{ 0x3755c, 0x00000100 },
	{ 0x37560, 0x00000100 },
	{ 0x37564, 0x00000100 },
	{ 0x37568, 0x00000f00 },
	{ 0x3756c, 0x0000000d },
	{ 0x37570, 0x00000000 },
	{ 0x37574, 0x00b40078 },
	{ 0x37578, 0x06080300 },
	{ 0x3757c, 0x00001000 },
	{ 0x37580, 0x00001000 },
	{ 0x37584, 0x00001000 },
	{ 0x37588, 0x00001000 },
	{ 0x3758c, 0x0000000c },
	{ 0x37590, 0x00100008 },
	{ 0x37594, 0x005000b4 },
	{ 0x37598, 0x1d380dff },
	{ 0x3759c, 0x01480100 },
	{ 0x375a0, 0x00000000 },
	{ 0x375a4, 0x00180006 },
	{ 0x375a8, 0x00230023 },
	{ 0x375ac, 0x008c0065 },
	{ 0x375b0, 0x04000040 },
	{ 0x375b4, 0x10000080 },
	{ 0x375b8, 0x08080403 },
	{ 0x375bc, 0x00100008 },
	{ 0x375c0, 0x0028000a },
	{ 0x375c4, 0x0eff062e },
	{ 0x375c8, 0x0cd30eff },
	{ 0x375cc, 0x1dff0bf0 },
	{ 0x375d0, 0x19a71dff },
	{ 0x375d4, 0x00000050 },
	{ 0x375d8, 0x0493002d },
	{ 0x375dc, 0x00000000 },
	{ 0x375e0, 0x00c00700 },
	{ 0x375e4, 0x005500aa },
	{ 0x375e8, 0x00000000 },
	{ 0x375ec, 0x00000000 },
	{ 0x375f0, 0x00000000 },
	{ 0x375f4, 0x00000000 },
	{ 0x375f8, 0x00000000 },
	{ 0x375fc, 0x00000000 },
	{ 0x37600, 0x00000000 },
	{ 0x37604, 0x00000000 },
	{ 0x37608, 0x00000000 },
	{ 0x3760c, 0x10700c00 },
	{ 0x37700, 0x00000000 },
	{ 0x37900, 0x00000000 },
	{ 0x37904, 0x0c001070 },
	{ 0x37b00, 0x00000000 },
	{ 0x37008, 0x00000000 },
	{ 0x3700c, 0x00000071 },
	{ 0x37a00, 0x00000000 },
	{ 0x37b04, 0x00000003 },
	{ 0x37028, 0x00000001 },
};

#define FRONT_STRIDE            0x14c0u
#define FRONT_HEIGHT            3072u
#define FRONT_FRAME             (FRONT_STRIDE * FRONT_HEIGHT)
#define FRONT_BUF               0x1080000u      /* the frame, rounded up */
#define FRONT_SIDE              0x100000u       /* the second buffer (+0x110: its size not known: 1 MiB) */

/* CMU_ISPFE's registers (Google's cal-if zuma cmucal-sfr.c; the block has holes that reset the phone: only these are
 * read), said while the front camera streams, to be set beside Android's (~/akita/ref/camera-android/live-regs-front.txt) */
static const uint16_t cmu_ispfe_regs[] = {
	0x0600, 0x0604, 0x0800, 0x1800, 0x1804, 0x2000, 0x2004, 0x2008, 0x200c, 0x2010,
	0x2014, 0x2018, 0x201c, 0x2020, 0x2024, 0x2028, 0x202c, 0x2030, 0x2034, 0x2038,
	0x203c, 0x2040, 0x2044, 0x2048, 0x204c, 0x2050, 0x2054, 0x2058, 0x205c, 0x2060,
	0x2064, 0x2068, 0x206c, 0x2070, 0x2074, 0x2078, 0x207c, 0x2080, 0x2084, 0x2088,
	0x208c, 0x2090, 0x2094, 0x2098, 0x209c, 0x20a0, 0x20a4, 0x20a8, 0x20ac, 0x20b0,
	0x20b4, 0x20b8, 0x20bc, 0x20c0, 0x20c4, 0x20c8, 0x20cc, 0x20d0, 0x20d4, 0x20d8,
	0x20dc, 0x20e0, 0x20e4, 0x20e8, 0x20ec, 0x20f0, 0x20f4, 0x20f8, 0x20fc, 0x2100,
	0x2104, 0x2108, 0x210c, 0x2110, 0x2114, 0x2118, 0x211c, 0x2120, 0x2124, 0x2128,
	0x212c, 0x2130, 0x2134, 0x2138, 0x213c, 0x2140, 0x2144, 0x2148, 0x214c, 0x2150,
	0x2154, 0x301c, 0x3020, 0x3024, 0x3028, 0x302c, 0x3030, 0x3034, 0x3038, 0x303c,
	0x3040, 0x3044, 0x3048, 0x304c, 0x3050, 0x3054, 0x3058, 0x305c, 0x3060, 0x3064,
	0x3068, 0x306c, 0x3070, 0x3074, 0x3078, 0x307c, 0x3080, 0x3084, 0x3088, 0x308c,
	0x3090, 0x3094, 0x3098, 0x309c, 0x30a0, 0x30a4, 0x30a8, 0x30ac, 0x30b0, 0x30b4,
	0x30b8, 0x30bc, 0x30c0, 0x30c4, 0x30cc, 0x30d0,
};
#define ISPFE_SYSREG_PHYS       0x1c020000ULL   /* SYSREG_ISPFE: +0x500 the D-PHYs' resets (bit n = D-PHY n, 1 = running);
                                                * a sweep of the page from Android reset it within its first 16 words: read only +0x500 */
#define FRONT_DPHY_RESET_BITS   0x60u
#define PMU_CSI_PHY_ISO         0x15463ebcULL   /* the CSIS D-PHYs' isolation (device tree "isolation" <0x3ebc>): bit0 1 = bypass */

extern uint64_t kvtophys(vm_offset_t va);
extern void flush_dcache(vm_offset_t addr, unsigned count, int phys);

static void
front_capture(void)
{
	const struct sensor *c = &sensors[1];
	vm_offset_t fe = 0, csis = 0, frame = 0, side = 0, sysreg = 0, pmu = ml_io_map(0x15462000ULL, 0x2000);
	uint64_t frame_pa, side_pa;
	uint8_t one = 1, zero = 0, fc = 0;
	unsigned changed = 0, first = ~0u;
	unsigned on = 0;

	if (!(rd(pmu + 0x484) & 1u)) {
		printf("akita camera: capture: pd-ispfe is off: not trying\n");
		return;
	}
	if (kmem_alloc_contig(kernel_map, &frame, FRONT_BUF, 0, 0, 0, KMA_KOBJECT | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS ||
	    kmem_alloc_contig(kernel_map, &side, FRONT_SIDE, 0, 0, 0, KMA_KOBJECT | KMA_DATA, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("akita camera: capture: no contiguous memory for the frame\n");
		return;
	}
	frame_pa = kvtophys(frame);
	side_pa = kvtophys(side);
	memset((void *)frame, 0xa5, FRONT_BUF);
	memset((void *)side, 0xa5, FRONT_SIDE);
	flush_dcache(frame, FRONT_BUF, FALSE);
	flush_dcache(side, FRONT_SIDE, FALSE);
	printf("akita camera: capture: frame buffer %#llx (%u bytes), second buffer %#llx\n", frame_pa, FRONT_BUF, side_pa);
	fe = ml_io_map(ISPFE_FE_PHYS, 0x100000);
	csis = ml_io_map(ISPFE_CSIS_PHYS, 0xd0000);

	/* the front camera powered and set to its mode (not streaming) */
	if (!i2c_bus_up(c->bus)) {
		return;
	}
	for (on = 0; on < 4 && c->ldo[on] != 0; on++) {
		if (!slg_ldo_on(c->ldo[on], c->mv[on])) {
			goto down;
		}
	}
	delay(1000);
	pin_out(c->reset_bank, c->reset_pin, true);
	mclk(c->clk, c->clk_bank, c->clk_pin, true);
	delay(10000);
	if (!imx712_mode(c)) {
		goto down;
	}
	/* the D-PHYs out of isolation, as Linux's phy-exynos-mipi power_on does (exynos_pmu_update: here the PRIV_REG SMC);
	 * Android 11-17: 1 while a camera streams, 0 when closed -- camera=7 before this left it 0 and no bytes came */
	{
		uint64_t r = cam_smc(SMC_PRIV_REG, PMU_CSI_PHY_ISO, PRIV_REG_WRITE, rd(pmu + 0x1ebc) | 1u);

		printf("akita camera: capture: CSI PHY isolation bypass: %#x (SMC %#llx)\n", rd(pmu + 0x1ebc), r);
	}
	/* ... and D-PHYs 5 and 6 (0x1c103100: the clock lane and 4 data lanes; 0x1c103600) out of reset: SYSREG_ISPFE +0x500
	 * (device tree system-controller@1C020500 "samsung,exynos-csis", each dcphy node's "reset" = its bit). Android 11-17:
	 * 0x60 while the front camera streams, 0x2 for the main one (D-PHY 1) */
	sysreg = ml_io_map(ISPFE_SYSREG_PHYS, 0x1000);
	wr(sysreg + 0x500, rd(sysreg + 0x500) | FRONT_DPHY_RESET_BITS);
	printf("akita camera: capture: D-PHY reset release: %#x\n", rd(sysreg + 0x500));
	/* Android's setup of the PHY, CSIS link 6 and the front end */
	for (unsigned i = 0; i < sizeof(ispfe_front_setup) / sizeof(ispfe_front_setup[0]); i++) {
		const struct ispfe_write *w = &ispfe_front_setup[i];
		vm_offset_t va = w->pa >= ISPFE_FE_PHYS ? fe + (w->pa - ISPFE_FE_PHYS) : csis + (w->pa - ISPFE_CSIS_PHYS);

		if (w->delay_us) {
			delay(w->delay_us);
		}
		wr(va, w->val);
	}
	printf("akita camera: capture: the PHY, CSIS link 6 and the front end set up (%u writes)\n",
	    (unsigned)(sizeof(ispfe_front_setup) / sizeof(ispfe_front_setup[0])));
	/* the frame's channel and the 0x37 block, as the PDMA's list would, with our buffers */
	wr(fe + 0x20554, FRONT_STRIDE);
	wr(fe + 0x20550, 4);
	wr(fe + 0x20544, 0);
	wr(fe + 0x20540, 1);
	wr(fe + 0x205c0, (uint32_t)frame_pa);
	wr(fe + 0x205c4, (uint32_t)(frame_pa >> 32));
	wr(fe + 0x20508, 0x71);
	wr(fe + 0x2050c, 0);
	wr(fe + 0x20510, (uint32_t)side_pa);
	wr(fe + 0x20514, (uint32_t)(side_pa >> 32));
	for (unsigned i = 0; i < sizeof(ispfe_front_37) / sizeof(ispfe_front_37[0]); i++) {
		wr(fe + ispfe_front_37[i].off, ispfe_front_37[i].val);
	}
	printf("akita camera: capture: channel 0 -> %#llx; stream on\n", frame_pa);
	i2c_write16(c->bus, 0x1a, 0x0100, &one, 1);
	wr(fe + 0x200a0, 1);                    /* (Android, 34 us after stream on: the front end's start) */
	delay_for_interval(150, NSEC_PER_MSEC);
	/* while streaming: CSIS link 6, the front end's top page, channel 0 (the words that are not 0) */
	{
		static const struct { const char *name; uint32_t pa, len; } dumps[] = {
			{ "CSIS link 6", 0x1c0a0000, 0x100 }, { "CSIS link 6 +0x600", 0x1c0a0600, 0x100 },
			{ "D-PHY 5 clock", 0x1c103100, 0x60 }, { "D-PHY 5 lane 0", 0x1c103200, 0x60 },
			{ "D-PHY 5 lane 3", 0x1c103500, 0x60 }, { "fe +0x20000", 0x1c320000, 0x100 }, { "fe +0x20400", 0x1c320400, 0x100 }, { "fe ch0", 0x1c320500, 0x100 },
		};

		for (unsigned d = 0; d < sizeof(dumps) / sizeof(dumps[0]); d++) {
			vm_offset_t base = dumps[d].pa >= ISPFE_FE_PHYS ? fe + (dumps[d].pa - ISPFE_FE_PHYS) : csis + (dumps[d].pa - ISPFE_CSIS_PHYS);
			char line[512];
			int n = 0;

			for (uint32_t o = 0; o < dumps[d].len && n < (int)sizeof(line) - 24; o += 4) {
				uint32_t v = rd(base + o);

				if (v) {
					n += snprintf(line + n, sizeof(line) - (size_t)n, " +%02x:%x", o, v);
				}
			}
			line[n] = '\0';
			printf("akita camera: capture: streaming: %s:%s\n", dumps[d].name, line);
		}
	}
	{
		vm_offset_t cmu = ml_io_map(CMU_ISPFE_PHYS, 0x4000);
		char line[256];
		int n = 0;

		for (unsigned i = 0; i < sizeof(cmu_ispfe_regs) / sizeof(cmu_ispfe_regs[0]); i++) {
			n += snprintf(line + n, sizeof(line) - (size_t)n, " %x:%x", cmu_ispfe_regs[i], rd(cmu + cmu_ispfe_regs[i]));
			if (n > 200 || i + 1 == sizeof(cmu_ispfe_regs) / sizeof(cmu_ispfe_regs[0])) {
				printf("akita camera: capture: CMU_ISPFE:%s\n", line);
				n = 0;
			}
		}
	}
	delay_for_interval(150, NSEC_PER_MSEC);
	i2c_read16(c->bus, 0x1a, 0x0005, &fc, 1);
	i2c_write16(c->bus, 0x1a, 0x0100, &zero, 1);
	delay(50000);
	printf("akita camera: capture: stream off (frame counter %u); channel 0: enable %#x addr %#x %#x, +0x37904 %#x\n", fc,
	    rd(fe + 0x20540), rd(fe + 0x205c0), rd(fe + 0x205c4), rd(fe + 0x37904));
	wr(fe + 0x20540, 0);
	/* what came: the frame's bytes no longer 0xa5 */
	flush_dcache(frame, FRONT_BUF, FALSE);
	flush_dcache(side, FRONT_SIDE, FALSE);
	for (unsigned i = 0; i < FRONT_BUF; i++) {
		if (((uint8_t *)frame)[i] != 0xa5) {
			changed++;
			first = first == ~0u ? i : first;
		}
	}
	printf("akita camera: capture: %u of %u bytes written (first at %#x); second buffer: %s\n", changed, FRONT_FRAME,
	    first, ((uint8_t *)side)[0] != 0xa5 || ((uint8_t *)side)[4] != 0xa5 ? "written" : "untouched");
	for (unsigned row = 0; row < FRONT_HEIGHT && changed; row += 1024) {
		const uint8_t *b = (const uint8_t *)frame + row * FRONT_STRIDE + 2000;

		printf("akita camera: capture: row %u: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", row, b[0], b[1], b[2], b[3],
		    b[4], b[5], b[6], b[7], b[8], b[9]);
	}
	{
		vm_offset_t s2 = ml_io_map(0x1c140000ULL, 0x3000);

		printf("akita camera: capture: S2MPU S0 FAULT_STATUS %#x\n", rd(s2 + 0x2000));
	}
down:
	if (sysreg) {
		wr(sysreg + 0x500, rd(sysreg + 0x500) & ~FRONT_DPHY_RESET_BITS);
	}
	cam_smc(SMC_PRIV_REG, PMU_CSI_PHY_ISO, PRIV_REG_WRITE, rd(pmu + 0x1ebc) & ~1u);
	mclk(c->clk, c->clk_bank, c->clk_pin, false);
	pin_out(c->reset_bank, c->reset_pin, false);
	while (on-- > 0) {
		slg_ldo_off(c->ldo[on]);
	}
	printf("akita camera: capture: front camera powered down\n");
}

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
	if (camera_mode >= 4 && c->bus == 1 && id16[0] == 0x07 && id16[1] == 0x12) {
		imx712_stream_test(c);
	}
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
	if (camera_mode == 7) {
		front_capture();
		return;
	}
	if (camera_mode >= 3) {
		for (int i = 0; i < 3; i++) {
			sensor_probe(&sensors[i]);
		}
		slg_look("after the three cameras");
	}
	if (camera_mode >= 5) {
		ispfe_test();
	}
	if (camera_mode >= 6) {
		ispfe_dma_look();
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
