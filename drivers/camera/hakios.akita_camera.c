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
 */
#include <kern/clock.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern int akita_acpm_xfer(unsigned channel, uint32_t cmd[4]);

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

/* the three camera buses: USI n of PERIC0 */
static const struct {
	const char *name;
	uint32_t usi, mux, div, ipclk, pclk, rstn, mode;
	unsigned bank;
} buses[3] = {
	{ "hsi2c_1 (main)", 1, 0x650, 0x1810, 0x2070, 0x2074, 0x20d0, 0x00, 2 },
	{ "hsi2c_2 (front)", 2, 0x660, 0x1814, 0x2078, 0x207c, 0x20d4, 0x04, 4 },
	{ "hsi2c_3 (ultra-wide)", 3, 0x670, 0x1818, 0x2080, 0x2084, 0x20d8, 0x08, 6 },
};

static vm_offset_t cmu_top, cmu_peric0, sysreg, gpio0;

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
akita_camera_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
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
	start = thread_call_allocate(akita_camera_start, NULL);
	clock_interval_to_deadline(12, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_camera_init);
