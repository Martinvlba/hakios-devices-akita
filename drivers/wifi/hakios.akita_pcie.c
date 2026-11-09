/*
 * The PCIe link to the Pixel 8a's Wi-Fi chip: zuma's PCIe channel 1 (a Samsung-wrapped DesignWare root complex at
 * 0x13120000, one lane, Gen2 at most) and, on its far end, the Broadcom BCM4383 (rev a3; 14e4:4449) that does Wi-Fi
 * and Bluetooth. Channel 0 is the modem's and is never touched here.
 *
 * Nothing here is touched unless the boot-arg asks: pcie=N performs steps 1 to N and stops, each saying what it is
 * about to touch before it does and what it found after; a step that does not find what it needs ends the sequence.
 *
 *   1  reads   the PHY isolation bits in the PMU (through the secure-register SMC), the pins (gph3 of the HSI2 pin
 *              controller, gpa5 of the far-alive one) as the bootloader left them
 *   2  writes  the pins as Android's "active" state has them: PERST# (gph3-0) an output, low; CLKREQ# (gph3-1) its
 *              PCIe function; WL_DEV_WAKE (gph3-5) an input; then WL_REG_ON (gph3-4) high -- the chip's power -- and
 *              200 ms for its supplies to settle (Google's WIFI_TURNON_DELAY)
 *   3  writes  the PHY: isolation off, the PHY powered down and up again (as Android's probe and power-on do, the
 *              unused second PHY of HSI2 off), then zuma's PHY settings for this channel with a Broadcom chip on it;
 *              the PLL, CDR and offset calibration waited for and said
 *   4  writes  the root complex set up (DesignWare's and Samsung's settings), PERST# let go, link training started
 *              and waited for (LTSSM in L0); the speed and width reached are said. Ten tries, as Android
 *   5  reads   the chip: the address translation for configuration and memory accesses, the chip's configuration
 *              space (vendor, device, class, the sizes of its BARs), its BARs placed where Android places them, and
 *              through BAR0 its chipcommon core's identity register (chip 0x4383, revision 2 = a3)
 * After the last step everything is left as it is (the chip powered, the link up): the next driver, the Broadcom one,
 * takes over from there; a test build restarts by itself anyway.
 *
 * What the hardware is, and where each value comes from: Google's Android kernel for this phone (google-modules/soc/gs,
 * android-gs-akita-android16: drivers/pci/controller/dwc/pcie-exynos-rc.c, pcie-exynos-zuma-rc-cal.c, the pin banks
 * in drivers/pinctrl/gs/pinctrl-gs.c; google-modules/wlan/bcmdhd/bcm4383: dhd_custom_google.c's power sequence), the
 * phone's device tree (pcie@13120000: reg names elbi soc udbg phy sysreg dbi pcs config ia, ip-ver 0x986500,
 * pmu-offset 0x3ec8, max-link-speed 2, use-sysmmu disabled, ep-device-type 1 = Broadcom Wi-Fi, perst-delay-us
 * 15000) and Linux's DesignWare core (dw_pcie_setup_rc). That code is GPL and none of it is here: the register
 * values are the hardware's facts. Android's own log of the same bring-up (11-09): link at Gen2 x1, the chip's BARs
 * 0x60200000 (64 KB), 0x60400000 (4 MB), 0x60800000 (1 MB, prefetchable), behind the root port's window
 * 0x60200000-0x608fffff; the chip has no SysMMU in front of it (its DMA addresses are physical, 64-bit).
 *
 * More boot-args: pcie_delay=SECONDS (8) after the kernel's start before step 1 runs.
 */
#include <kern/clock.h>
#include <kern/sched_prim.h>
#include <kern/startup.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* ---- where things are ---- */

/* channel 1's blocks (the device tree's reg, by name) */
#define PCIE1_ELBI              0x13120000ULL   /* Samsung's wrapper: resets, LTSSM, link state (0x2000) */
#define PCIE1_SOC               0x13134000ULL   /* "soc" 0x13130000 + 0x4000: the PHY's input clock select */
#define PCIE1_UDBG              0x1315c000ULL   /* "udbg" 0x13150000 + 0xc000: the external PLL (0xc700...) */
#define PCIE1_PHY               0x13160000ULL   /* the PHY's PMA (0x3000) */
#define PCIE1_PCS               0x13140000ULL   /* the PHY's PCS (offsets below 0x1000 used) */
#define PCIE1_DBI               0x13400000ULL   /* DesignWare's registers: the root port's configuration space + port logic */
#define PCIE1_ATU               0x13700000ULL   /* "dbi" + 0x300000: the address translation unit ("unrolled") */
#define PCIE1_CFG               0x60ffe000ULL   /* the window configuration accesses go out through (0x2000) */
#define PCIE1_CFG_SIZE          0x2000u
#define PCIE1_MEM               0x60000000ULL   /* the window memory accesses go out through, 1:1 (to 0x60feffff) */
#define PCIE1_MEM_LIMIT         0x60fdffffULL   /* (below the configuration window) */

/* HSI2's second PHY (Android powers it down at its probe of channel 1) */
#define PCIE2_PHY               0x131b0000ULL
#define PCIE2_UDBG              0x131ac000ULL   /* + 0x700 */

/* the PMU: the PHYs' isolation, written through the secure world */
#define PMU_BASE                0x15460000ULL
#define PMU_PCIE1_PHY           (PMU_BASE + 0x3ec8)     /* bit 0: 1 = bypass (isolation off); bit 10 set before training */
#define PMU_PCIE2_PHY           (PMU_BASE + 0x3ecc)
#define TENSOR_SMC_PMU_SEC_REG  0x82000504u
#define PMU_REG_READ            0
#define PMU_REG_RMW             2       /* (mask, value): Android's rmw_priv_reg */

/* the pins: zuma's HSI2 pin controller (gph3 at bank 0) and the far-alive one (gpa5 at bank 0) */
#define PIN_HSI2                0x13040000ULL
#define PIN_FAR_ALIVE           0x154e0000ULL
#define PIN_CON                 0x00            /* 4 bits a pin: 0 input, 1 output, 2.. a function, 0xf interrupt */
#define PIN_DAT                 0x04            /* a bit a pin */
#define PIN_PUD                 0x08            /* 4 bits a pin: 0 none, 1 down, 3 up */
#define PIN_DRV                 0x0c
#define GPH3_PERST              0
#define GPH3_CLKREQ             1
#define GPH3_WL_REG_ON          4
#define GPH3_WL_DEV_WAKE        5
#define GPA5_WL_HOST_WAKE       1

/* the wrapper's registers (ELBI) */
#define ELBI_IRQ0               0x000
#define ELBI_IRQ1               0x004
#define ELBI_IRQ2               0x008
#define ELBI_LTSSM_ENABLE       0x054
#define ELBI_XFER_PENDING       0x074
#define ELBI_DEVICE_TYPE        0x080           /* 4: root port */
#define ELBI_LINKUP             0x2c8           /* LTSSM state in bits 5:0 */
#define ELBI_PM_DSTATE          0x2e8
#define ELBI_LINKDOWN_RST_CTRL  0x3a0           /* 2: manual */
#define ELBI_SOFT_RESET         0x3a4
#define ELBI_QCH_SEL            0x3a8
#define ELBI_REQ_EXIT_L1_MODE   0x3bc
#define ELBI_SLV_PEND_SEL_NAK   0x3d8
#define ELBI_MSTR_PEND_SEL_NAK  0x474
#define ELBI_HISTORY_CHECK      0xc00
#define ELBI_STATE_POWER_S      0xc04
#define ELBI_STATE_POWER_M      0xc08
#define ELBI_HISTORY(n)         (0xc0c + 4 * (n))
#define ELBI_DBI_L1_EXIT_DIS    0x1078
#define ELBI_PMA_RST0           0x1400
#define ELBI_PMA_RST1           0x1404
#define ELBI_PMA_RST2           0x1408

#define LTSSM_L0                0x11
#define LTSSM_L1_IDLE           0x14

/* the root port's configuration space and DesignWare's port logic (DBI) */
#define CFG_ID                  0x00
#define CFG_COMMAND             0x04
#define CFG_CLASS_REV           0x08
#define CFG_BAR(n)              (0x10 + 4 * (n))
#define CFG_BUSES               0x18
#define CFG_MEM_WINDOW          0x20
#define CFG_PREF_WINDOW         0x24
#define CFG_PREF_BASE_HI        0x28
#define CFG_PREF_LIMIT_HI       0x2c
#define CFG_INTERRUPT           0x3c
#define RC_PM_CAP               0x40
#define RC_EXP_CAP              0x70
#define EXP_LNKCAP              0x0c
#define EXP_LNKCTL_STA          0x10
#define EXP_DEVCTL2             0x28
#define EXP_LNKCTL2             0x30
#define DBI_PORT_LINK_CTRL      0x710
#define DBI_GEN2_CTRL           0x80c           /* LINK_WIDTH_SPEED_CONTROL */
#define DBI_GEN3_EQ_CTRL        0x890
#define DBI_MISC_CONTROL        0x8bc           /* bit 0: read-only registers writable */
#define DBI_AUX_CLK_FREQ        0xb40
#define DBI_L1_SUBSTATES        0xb44

/* the address translation unit, outbound region n (0x200 apart) */
#define ATU_CR1                 0x00
#define ATU_CR2                 0x04            /* bit 31: on */
#define ATU_BASE_LO             0x08
#define ATU_BASE_HI             0x0c
#define ATU_LIMIT               0x10
#define ATU_TARGET_LO           0x14
#define ATU_TARGET_HI           0x18
#define ATU_TYPE_MEM            0
#define ATU_TYPE_CFG0           4

/* the chip */
#define BCM_VENDOR              0x14e4
#define BCM_BAR0_WIN            0x80            /* configuration register: the backplane address BAR0's first 4 KB show */
#define BCM_ENUM_BASE           0x18000000u     /* chipcommon, the first core */
#define EP_BAR0                 0x60200000ULL   /* where Android puts the BARs */
#define EP_BAR2                 0x60400000ULL
#define EP_BAR4                 0x60800000ULL

static unsigned int steps, step;
static volatile uint8_t *elbi, *socr, *udbg, *phy, *pcs, *dbi, *atu, *cfg, *pin_hsi2, *pin_far, *phy2, *udbg2;
static volatile uint8_t *bar0;

static inline uint32_t
rd(volatile uint8_t *base, uint32_t off)
{
	return *(volatile uint32_t *)(base + off);
}

static inline void
wr(volatile uint8_t *base, uint32_t off, uint32_t value)
{
	*(volatile uint32_t *)(base + off) = value;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static inline void
set_bits(volatile uint8_t *base, uint32_t off, uint32_t bits)
{
	wr(base, off, rd(base, off) | bits);
}

static inline void
clear_bits(volatile uint8_t *base, uint32_t off, uint32_t bits)
{
	wr(base, off, rd(base, off) & ~bits);
}

static uint64_t
smc(uint64_t function, uint64_t a1, uint64_t a2, uint64_t a3, uint64_t a4)
{
	register uint64_t x0 __asm__("x0") = function, x1 __asm__("x1") = a1, x2 __asm__("x2") = a2, x3 __asm__("x3") = a3;
	register uint64_t x4 __asm__("x4") = a4;

	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3), "+r" (x4) : :
	    "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

static uint32_t
pmu_read(uint64_t address)
{
	return (uint32_t)smc(TENSOR_SMC_PMU_SEC_REG, address, PMU_REG_READ, 0, 0);
}

/* the bits of `mask` set to `value`, by the secure monitor (Android's rmw_priv_reg); says so if it refuses */
static void
pmu_update(uint64_t address, uint32_t mask, uint32_t value)
{
	uint64_t r = smc(TENSOR_SMC_PMU_SEC_REG, address, PMU_REG_RMW, mask, value);

	if (r != 0) {
		printf("pcie: %u: the secure monitor refused PMU 0x%llx (mask 0x%x): %lld\n", step, address, mask, (long long)r);
	}
}

static void
sleep_ms(uint32_t ms)
{
	assert_wait_timeout((event_t)&steps, THREAD_UNINT, ms, NSEC_PER_MSEC);
	thread_block(THREAD_CONTINUE_NULL);
}

static bool
announce(unsigned int n, const char *what)
{
	if (n > steps) {
		return false;
	}
	step = n;
	printf("pcie: %u: %s\n", n, what);
	return true;
}

/* ---- pins ---- */

static void
pin_set(volatile uint8_t *bank, unsigned pin, uint32_t function, int pud, int level)
{
	if (level >= 0) {
		/* the level first, so that an output starts at it */
		uint32_t dat = rd(bank, PIN_DAT) & ~(1u << pin);

		wr(bank, PIN_DAT, dat | ((uint32_t)level << pin));
	}
	if (pud >= 0) {
		uint32_t p = rd(bank, PIN_PUD) & ~(0xfu << (4 * pin));

		wr(bank, PIN_PUD, p | ((uint32_t)pud << (4 * pin)));
	}
	uint32_t con = rd(bank, PIN_CON) & ~(0xfu << (4 * pin));

	wr(bank, PIN_CON, con | (function << (4 * pin)));
}

static void
pins_say(const char *when)
{
	printf("pcie: %u: %s: gph3 CON 0x%08x DAT 0x%02x PUD 0x%08x DRV 0x%08x; gpa5 CON 0x%08x DAT 0x%02x PUD 0x%08x\n",
	    step, when, rd(pin_hsi2, PIN_CON), rd(pin_hsi2, PIN_DAT), rd(pin_hsi2, PIN_PUD), rd(pin_hsi2, PIN_DRV),
	    rd(pin_far, PIN_CON), rd(pin_far, PIN_DAT), rd(pin_far, PIN_PUD));
}

/* ---- the PHY (zuma, channel 1: "PCIe Gen3 1 lane"; a Broadcom chip on it) ---- */

static bool
wait_bit(volatile uint8_t *base, uint32_t off, unsigned bit, const char *what)
{
	for (unsigned i = 0; i < 10000; i++) {
		delay(10);
		if (rd(base, off) & (1u << bit)) {
			printf("pcie: %u: %s after %u us\n", step, what, 10 * (i + 1));
			return true;
		}
	}
	printf("pcie: %u: %s: NOT after 100 ms (0x%x = 0x%08x)\n", step, what, off, rd(base, off));
	return false;
}

static void
phy_power_down(void)
{
	clear_bits(phy, 0x204, 0x3u << 2);
	wr(phy, 0x1044, 0x2a);
	wr(phy, 0x1048, 0xaa);
	wr(phy, 0x104c, 0xa8);
	wr(phy, 0x1050, 0x80);
	wr(phy, 0x185c, 0x0a);
	delay(1);
	wr(phy, 0x0208, 0xff);
	delay(1);
	wr(phy, 0x0580, 0x0a);
	wr(phy, 0x0928, 0xaa);
	wr(phy, 0x000c, 0x0a);         /* common bias, PLL off */
	delay(50);
	clear_bits(udbg, 0x700, 1u << 0);       /* the external PLL gated */
	delay(10);
}

/* HSI2's other PHY off, isolated again (Android's probe of channel 1 does this) */
static void
phy2_disable(void)
{
	pmu_update(PMU_PCIE2_PHY, 1, 1);
	clear_bits(phy2, 0x204, 0x3u << 2);
	wr(phy2, 0x1044, 0x2a);
	wr(phy2, 0x1048, 0xaa);
	wr(phy2, 0x104c, 0xa8);
	wr(phy2, 0x1050, 0x80);
	wr(phy2, 0x185c, 0x0a);
	delay(1);
	wr(phy2, 0x0208, 0xff);
	delay(1);
	wr(phy2, 0x0580, 0x0a);
	wr(phy2, 0x0928, 0xaa);
	wr(phy2, 0x000c, 0x0a);
	delay(50);
	clear_bits(udbg2, 0x700, 1u << 0);
	delay(10);
	pmu_update(PMU_PCIE2_PHY, 1, 0);
}

static void
phy_power_up(void)
{
	set_bits(udbg, 0x700, 1u << 0);         /* the external PLL ungated */
	delay(100);
	wr(phy, 0x000c, 0x00);
	wr(phy, 0x0928, 0x55);
	wr(phy, 0x0580, 0x02);
	wr(phy, 0x0208, 0x00);
	wr(phy, 0x1044, 0x00);
	wr(phy, 0x1048, 0x00);
	wr(phy, 0x104c, 0x00);
	wr(phy, 0x1050, 0x00);
	wr(phy, 0x185c, 0x00);
	delay(10);
}

static const struct { uint16_t off; uint8_t value; } phy_common[] = {
	{ 0x0000, 0x88 }, { 0x001c, 0x66 }, { 0x01f4, 0x00 }, { 0x0514, 0x59 }, { 0x051c, 0x11 }, { 0x062c, 0x0e },
	{ 0x0644, 0x22 }, { 0x0688, 0x03 }, { 0x06d4, 0x28 }, { 0x0788, 0x64 }, { 0x078c, 0x64 }, { 0x0790, 0x50 },
	{ 0x0794, 0x50 }, { 0x0944, 0x05 }, { 0x0948, 0x05 }, { 0x094c, 0x05 }, { 0x0950, 0x05 },
	/* the 38.4 MHz reference clock to the external PLL */
	{ 0x0590, 0x02 }, { 0x07f8, 0xb0 }, { 0x0730, 0x08 },
	{ 0x0344, 0xc0 },               /* delay in L1.2: 7.68 us */
	{ 0x0040, 0x04 }, { 0x0204, 0x03 }, { 0x02d4, 0x1f }, { 0x0358, 0x10 },  /* CLKREQ# not followed in L1.2 */
	{ 0x0018, 0x01 },               /* ERIO on */
	{ 0x0514, 0x5b }, { 0x0608, 0x0c }, { 0x060c, 0x0f }, { 0x0610, 0x0f }, { 0x0614, 0x0f }, { 0x0618, 0x0f },
	{ 0x0510, 0x80 }, { 0x0688, 0x03 }, { 0x0644, 0x23 }, { 0x0624, 0x11 },
	{ 0x0630, 0x0f }, { 0x06d0, 0x53 },     /* PLL margin for ERIO (Gen1, Gen2) */
	/* the lane */
	{ 0x1140, 0x04 }, { 0x1144, 0x04 }, { 0x1148, 0x04 }, { 0x114c, 0x02 }, { 0x1150, 0x00 }, { 0x1154, 0x00 },
	{ 0x1158, 0x00 }, { 0x115c, 0x00 }, { 0x12cc, 0x1c }, { 0x12dc, 0x6c }, { 0x130c, 0x29 }, { 0x13b4, 0x2f },
	{ 0x1a64, 0x05 }, { 0x1a68, 0x05 }, { 0x1a84, 0x05 }, { 0x1a88, 0x05 }, { 0x1a98, 0x00 }, { 0x1a9c, 0x00 },
	{ 0x1aa8, 0x07 }, { 0x1ab8, 0x00 }, { 0x1abc, 0x00 }, { 0x1af8, 0x90 }, { 0x1b34, 0x03 }, { 0x1bb0, 0x03 },
	{ 0x1bb4, 0x03 }, { 0x1bc0, 0x06 }, { 0x1bc4, 0x06 }, { 0x1be8, 0x01 }, { 0x1bf8, 0x04 }, { 0x1c98, 0x00 },
	{ 0x1ca4, 0x81 },
	/* a Broadcom chip: the link going down on leaving L1 substates */
	{ 0x1b20, 0x02 }, { 0x1340, 0x23 }, { 0x002c, 0x3c },
};

static bool
phy_config(void)
{
	bool pll, cdr, oc;

	wr(phy, 0x032c, 0x3);                   /* the PHY's input clock ungated */
	set_bits(udbg, 0x700, 1u << 0);         /* the external PLL out of its init */
	clear_bits(udbg, 0x700, 1u << 1);       /* ... and its RESETB overridden */
	(void)wait_bit(udbg, 0x734, 2, "external PLL locked");
	clear_bits(socr, 0x4, 1u << 2);         /* the PHY's input clock: the external PLL, not TCXO */
	set_bits(socr, 0x4, 0x3u << 4);
	wr(elbi, ELBI_DEVICE_TYPE, 0x4);        /* root port */
	wr(elbi, ELBI_SOFT_RESET, 0xf);         /* soft power reset */
	wr(elbi, ELBI_SOFT_RESET, 0xd);
	delay(10);
	wr(elbi, ELBI_SOFT_RESET, 0xf);
	delay(10);
	wr(elbi, ELBI_PMA_RST1, 1);             /* the PMA's resets asserted */
	wr(elbi, ELBI_PMA_RST2, 1);
	wr(elbi, ELBI_PMA_RST0, 1);
	wr(elbi, ELBI_PMA_RST1, 0);
	wr(elbi, ELBI_PMA_RST2, 0);
	wr(elbi, ELBI_PMA_RST0, 0);
	wr(elbi, ELBI_SLV_PEND_SEL_NAK, 1);     /* a pending slave access is answered (NAK), not left hanging */

	for (unsigned i = 0; i < sizeof(phy_common) / sizeof(phy_common[0]); i++) {
		wr(phy, phy_common[i].off, phy_common[i].value);
	}
	set_bits(phy, 0x1350, 0x1);

	/* PCS */
	wr(pcs, 0x190, 0x100b0604);
	wr(pcs, 0x154, 0x000700d5);
	wr(pcs, 0x100, 0x16400000);
	wr(pcs, 0x104, 0x08600000);
	wr(pcs, 0x114, 0x18500000);
	wr(pcs, 0x124, 0x60700000);
	wr(pcs, 0x174, 0x00000007);
	wr(pcs, 0x178, 0x00000100);
	wr(pcs, 0x17c, 0x00000700);             /* (a Broadcom chip) */
	wr(pcs, 0x110, 0x01600202);             /* (a Broadcom chip) */

	wr(elbi, ELBI_PMA_RST1, 1);             /* the PMA's resets let go */
	delay(10);
	wr(elbi, ELBI_PMA_RST2, 1);
	wr(elbi, ELBI_PMA_RST0, 1);

	/* (Android waits for all three and only logs what it found; so here: the PLL is what has to lock) */
	pll = wait_bit(phy, 0x0a80, 0, "PLL locked");
	cdr = wait_bit(phy, 0x15c0, 4, "CDR locked");
	oc = wait_bit(phy, 0x140c, 7, "offset calibration done");
	if (!cdr || !oc) {
		printf("pcie: %u: (CDR %s, offset calibration %s: Android goes on all the same)\n", step,
		    cdr ? "locked" : "not locked", oc ? "done" : "not done");
	}

	wr(phy, 0x032c, 0x0);
	wr(elbi, ELBI_DBI_L1_EXIT_DIS, 1);      /* L1 exit off by DBI */
	return pll;
}

/* ---- the root complex ---- */

static void
setup_rc(void)
{
	uint32_t v;

	/* DesignWare's (Linux's dw_pcie_setup_rc, dw_pcie_setup) */
	set_bits(dbi, DBI_MISC_CONTROL, 1);     /* read-only registers writable */
	v = rd(dbi, RC_EXP_CAP + EXP_LNKCTL2) & ~0xfu;
	wr(dbi, RC_EXP_CAP + EXP_LNKCTL2, v | 2);       /* max-link-speed 2: 5 GT/s */
	v = rd(dbi, DBI_PORT_LINK_CTRL);
	v &= ~(1u << 7);                        /* no fast link mode */
	v |= 1u << 5;                           /* DLL link enable */
	v = (v & ~(0x3fu << 16)) | (0x1u << 16);        /* one lane */
	wr(dbi, DBI_PORT_LINK_CTRL, v);
	v = rd(dbi, DBI_GEN2_CTRL);
	wr(dbi, DBI_GEN2_CTRL, (v & ~(0x1fu << 8)) | (0x1u << 8));
	wr(dbi, CFG_BAR(0), 0x00000004);
	wr(dbi, CFG_BAR(1), 0x00000000);
	v = rd(dbi, CFG_INTERRUPT);
	wr(dbi, CFG_INTERRUPT, (v & 0xffff00ffu) | 0x00000100u);
	v = rd(dbi, CFG_BUSES);
	wr(dbi, CFG_BUSES, (v & 0xff000000u) | 0x00ff0100u);    /* primary 0, secondary 1, subordinate 0xff */
	v = rd(dbi, CFG_COMMAND);
	wr(dbi, CFG_COMMAND, (v & 0xffff0000u) | 0x1u | 0x2u | 0x4u | 0x100u);  /* I/O, memory, bus master, SERR */
	wr(dbi, CFG_BAR(0), 0);
	v = rd(dbi, CFG_CLASS_REV);
	wr(dbi, CFG_CLASS_REV, (v & 0xffffu) | (0x0604u << 16));        /* a PCI-to-PCI bridge */
	set_bits(dbi, DBI_GEN2_CTRL, 1u << 17); /* speed change */
	clear_bits(dbi, DBI_MISC_CONTROL, 1);

	/* Samsung's (exynos_pcie_setup_rc) */
	wr(dbi, DBI_MISC_CONTROL, 1);
	wr(dbi, CFG_ID, 0x144du | ((0xececu + 1) << 16));        /* Samsung, 0xeced (channel 1) */
	v = rd(dbi, RC_EXP_CAP + EXP_LNKCAP);
	v &= ~((0x7u << 15) | 0xfu);
	v |= (0x7u << 15) | 3;                  /* L1 exit latency 64 us; speeds to 8 GT/s */
	wr(dbi, RC_EXP_CAP + EXP_LNKCAP, v);
	wr(dbi, DBI_AUX_CLK_FREQ, 0x1a);        /* 26 MHz */
	wr(dbi, DBI_L1_SUBSTATES, 0xea);
	wr(dbi, RC_PM_CAP + 4, 0);              /* power management control and status */
	v = rd(dbi, RC_EXP_CAP + EXP_LNKCTL2) & ~0xfu;
	wr(dbi, RC_EXP_CAP + EXP_LNKCTL2, v | 2);       /* the target speed: Gen2 */
	v = rd(dbi, RC_EXP_CAP + EXP_DEVCTL2) & ~0xfu;
	wr(dbi, RC_EXP_CAP + EXP_DEVCTL2, v | 5);       /* completion timeout 16-55 ms */
}

static void
history_say(void)
{
	printf("pcie: %u: LTSSM history:", step);
	for (unsigned i = 0; i < 32; i++) {
		uint32_t h = rd(elbi, ELBI_HISTORY(i));

		printf(" %02x/%u/%u", (h >> 16) & 0x3f, (h >> 8) & 7, h & 7);
	}
	printf("\n");
}

static bool
establish_link(void)
{
	for (unsigned attempt = 1; attempt <= 10; attempt++) {
		uint32_t state = 0, speed, width, v;
		unsigned count;

		if (attempt > 1) {
			if (!phy_config()) {
				printf("pcie: %u: the PHY did not lock again\n", step);
			}
		}
		pmu_update(PMU_PCIE1_PHY, 0x400, 0x400);
		wr(udbg, 0x800, 0x421);
		wr(phy, 0x032c, 0x0);
		wr(dbi, DBI_GEN3_EQ_CTRL, 0x12000);     /* equalisation off */

		pin_set(pin_hsi2, GPH3_PERST, 1, -1, 1);        /* PERST# let go */
		sleep_ms(16);                   /* perst-delay-us 15000 */

		set_bits(elbi, ELBI_REQ_EXIT_L1_MODE, 0x1u | (0x1u << 4));
		wr(elbi, ELBI_LINKDOWN_RST_CTRL, 0x2);
		wr(elbi, ELBI_XFER_PENDING, 0x1);
		clear_bits(elbi, ELBI_QCH_SEL, 0xfffu);         /* no clock gating (ip-ver 0x986500) */
		wr(elbi, ELBI_MSTR_PEND_SEL_NAK, 1);
		wr(elbi, ELBI_DBI_L1_EXIT_DIS, 1);

		setup_rc();

		printf("pcie: %u: try %u: D state %u, LTSSM 0x%02x before training\n", step, attempt,
		    rd(elbi, ELBI_PM_DSTATE) & 3, rd(elbi, ELBI_LINKUP) & 0x3f);
		clear_bits(elbi, ELBI_HISTORY_CHECK, 1u << 2);
		wr(elbi, ELBI_STATE_POWER_S, 0xffffffffu);
		wr(elbi, ELBI_STATE_POWER_M, 0xffffffffu);
		set_bits(elbi, ELBI_HISTORY_CHECK, 0x3);

		wr(elbi, ELBI_LTSSM_ENABLE, 1);
		for (count = 0; count < 4800; count++) {
			state = rd(elbi, ELBI_LINKUP) & 0x3f;
			if (state == LTSSM_L0) {
				break;
			}
			delay(10);
		}
		if (state != LTSSM_L0) {
			printf("pcie: %u: try %u: no link (LTSSM 0x%02x)\n", step, attempt, state);
			history_say();
			pin_set(pin_hsi2, GPH3_PERST, 1, -1, 0);
			wr(elbi, ELBI_LTSSM_ENABLE, 0);
			continue;
		}
		printf("pcie: %u: try %u: L0 after ~%u us\n", step, attempt, 10 * count);
		sleep_ms(3);                    /* from Gen1 to the target speed */
		v = rd(dbi, RC_EXP_CAP + EXP_LNKCTL_STA);
		speed = (v >> 16) & 0xf;
		width = (v >> 20) & 0x3f;
		printf("pcie: %u: link up: Gen%u x%u (link status 0x%08x)\n", step, speed, width, v);
		if (speed < 2) {
			printf("pcie: %u: below Gen2: again\n", step);
			pin_set(pin_hsi2, GPH3_PERST, 1, -1, 0);
			wr(elbi, ELBI_LTSSM_ENABLE, 0);
			continue;
		}
		for (count = 0; count < 4800; count++) {
			state = rd(elbi, ELBI_LINKUP) & 0x3f;
			if (state >= LTSSM_L0 && state <= LTSSM_L1_IDLE) {
				break;
			}
			delay(10);
		}
		wr(elbi, ELBI_IRQ0, rd(elbi, ELBI_IRQ0));
		wr(elbi, ELBI_IRQ1, rd(elbi, ELBI_IRQ1));
		wr(elbi, ELBI_IRQ2, rd(elbi, ELBI_IRQ2));
		return true;
	}
	return false;
}

/* ---- the chip ---- */

static void
atu_region(unsigned n, uint32_t type, uint64_t base, uint64_t limit, uint64_t target)
{
	volatile uint8_t *r = atu + 0x200 * n;

	wr(r, ATU_CR1, type);
	wr(r, ATU_BASE_LO, (uint32_t)base);
	wr(r, ATU_BASE_HI, (uint32_t)(base >> 32));
	wr(r, ATU_LIMIT, (uint32_t)limit);
	wr(r, ATU_TARGET_LO, (uint32_t)target);
	wr(r, ATU_TARGET_HI, (uint32_t)(target >> 32));
	wr(r, ATU_CR2, 1u << 31);
}

/* the size a BAR asks for: all ones written, read back, the old value put back */
static uint64_t
bar_size(unsigned n, bool *is64)
{
	uint32_t old = rd(cfg, CFG_BAR(n)), lo, hi = 0xffffffffu, oldhi = 0;

	*is64 = ((old >> 1) & 3) == 2;
	wr(cfg, CFG_BAR(n), 0xffffffffu);
	lo = rd(cfg, CFG_BAR(n)) & ~0xfu;
	wr(cfg, CFG_BAR(n), old);
	if (*is64) {
		oldhi = rd(cfg, CFG_BAR(n + 1));
		wr(cfg, CFG_BAR(n + 1), 0xffffffffu);
		hi = rd(cfg, CFG_BAR(n + 1));
		wr(cfg, CFG_BAR(n + 1), oldhi);
	}
	if (lo == 0 && (!*is64 || hi == 0)) {
		return 0;
	}
	return ~(((uint64_t)hi << 32) | lo) + 1;
}

static void
bar_place(unsigned n, uint64_t address)
{
	wr(cfg, CFG_BAR(n), (uint32_t)address | (rd(cfg, CFG_BAR(n)) & 0xfu));
	if (((rd(cfg, CFG_BAR(n)) >> 1) & 3) == 2) {
		wr(cfg, CFG_BAR(n + 1), (uint32_t)(address >> 32));
	}
}

static bool
chip(void)
{
	uint32_t id, v;

	/* region 0: configuration accesses to bus 1, device 0; region 1: memory, 1:1 */
	atu_region(0, ATU_TYPE_CFG0, PCIE1_CFG, PCIE1_CFG + PCIE1_CFG_SIZE - 1, 1u << 24);
	atu_region(1, ATU_TYPE_MEM, PCIE1_MEM, PCIE1_MEM_LIMIT, PCIE1_MEM);
	wr(elbi, ELBI_XFER_PENDING, 0x0);

	id = rd(cfg, CFG_ID);
	printf("pcie: %u: the chip: %04x:%04x, class/rev 0x%08x, header 0x%02x, status/command 0x%08x\n", step,
	    id & 0xffff, id >> 16, rd(cfg, CFG_CLASS_REV), (rd(cfg, 0x0c) >> 16) & 0xff, rd(cfg, CFG_COMMAND));
	if ((id & 0xffff) != BCM_VENDOR) {
		printf("pcie: %u: not Broadcom's: stopped\n", step);
		return false;
	}
	for (unsigned n = 0; n < 6; n++) {
		bool is64;
		uint64_t size = bar_size(n, &is64);

		if (size) {
			printf("pcie: %u: BAR%u: %llu KB%s%s\n", step, n, size >> 10, is64 ? ", 64-bit" : "",
			    (rd(cfg, CFG_BAR(n)) & 0x8) ? ", prefetchable" : "");
		}
		if (is64) {
			n++;
		}
	}

	/* the root port's windows, then the chip's BARs, as Android had them */
	wr(dbi, CFG_MEM_WINDOW, 0x6020u | (0x6070u << 16));     /* 0x60200000 - 0x607fffff */
	wr(dbi, CFG_PREF_WINDOW, 0x6081u | (0x6081u << 16));    /* 0x60800000 - 0x608fffff, 64-bit */
	wr(dbi, CFG_PREF_BASE_HI, 0);
	wr(dbi, CFG_PREF_LIMIT_HI, 0);
	bar_place(0, EP_BAR0);
	bar_place(2, EP_BAR2);
	bar_place(4, EP_BAR4);
	v = rd(cfg, CFG_COMMAND);
	wr(cfg, CFG_COMMAND, (v & 0xffff0000u) | 0x2u | 0x4u);  /* memory, bus master */
	printf("pcie: %u: BARs 0x%08x%08x 0x%08x%08x 0x%08x%08x, command 0x%04x\n", step, rd(cfg, CFG_BAR(1)),
	    rd(cfg, CFG_BAR(0)), rd(cfg, CFG_BAR(3)), rd(cfg, CFG_BAR(2)), rd(cfg, CFG_BAR(5)), rd(cfg, CFG_BAR(4)),
	    rd(cfg, CFG_COMMAND) & 0xffff);

	/* BAR0's first 4 KB on chipcommon: its identity register */
	wr(cfg, BCM_BAR0_WIN, BCM_ENUM_BASE);
	printf("pcie: %u: BAR0 window 0x%08x\n", step, rd(cfg, BCM_BAR0_WIN));
	bar0 = (volatile uint8_t *)ml_io_map(EP_BAR0, 0x4000);
	id = rd(bar0, 0);
	printf("pcie: %u: chipcommon: chip 0x%x, revision %u, package %u, %u cores, interconnect type %u (id 0x%08x)%s\n",
	    step, id & 0xffff, (id >> 16) & 0xf, (id >> 20) & 0xf, (id >> 24) & 0xf, id >> 28, id,
	    (id & 0xffff) == 0x4383 ? "" : " -- NOT the BCM4383");
	return (id & 0xffff) == 0x4383;
}

/* ---- the sequence ---- */

static void
pcie_run(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	if (!announce(1, "reads: the PHYs' isolation (PMU), the pins as the bootloader left them")) {
		return;
	}
	pin_hsi2 = (volatile uint8_t *)ml_io_map(PIN_HSI2, 0x1000);
	pin_far = (volatile uint8_t *)ml_io_map(PIN_FAR_ALIVE, 0x1000);
	printf("pcie: 1: PMU: channel 1's PHY control 0x%08x, the second PHY's 0x%08x (bit 0: 1 = isolation off)\n",
	    pmu_read(PMU_PCIE1_PHY), pmu_read(PMU_PCIE2_PHY));
	pins_say("pins");

	if (!announce(2, "writes: the pins (PERST# low, CLKREQ#, WL_DEV_WAKE), then WL_REG_ON high: the chip's power")) {
		return;
	}
	pin_set(pin_hsi2, GPH3_PERST, 1, -1, 0);
	pin_set(pin_hsi2, GPH3_CLKREQ, 2, 3, -1);
	pin_set(pin_hsi2, GPH3_WL_DEV_WAKE, 0, 1, -1);
	pin_set(pin_far, GPA5_WL_HOST_WAKE, 0, 1, -1);  /* an input for now (Android: its interrupt) */
	pin_set(pin_hsi2, GPH3_WL_REG_ON, 1, 0, 1);
	sleep_ms(200);
	pins_say("after");

	if (!announce(3, "writes: the PHY (isolation off, powered down and up, zuma's settings for a Broadcom chip)")) {
		return;
	}
	elbi = (volatile uint8_t *)ml_io_map(PCIE1_ELBI, 0x2000);
	socr = (volatile uint8_t *)ml_io_map(PCIE1_SOC, 0x1000);
	udbg = (volatile uint8_t *)ml_io_map(PCIE1_UDBG, 0x1000);
	phy = (volatile uint8_t *)ml_io_map(PCIE1_PHY, 0x3000);
	pcs = (volatile uint8_t *)ml_io_map(PCIE1_PCS, 0x1000);
	dbi = (volatile uint8_t *)ml_io_map(PCIE1_DBI, 0x1000);
	atu = (volatile uint8_t *)ml_io_map(PCIE1_ATU, 0x1000);
	phy2 = (volatile uint8_t *)ml_io_map(PCIE2_PHY, 0x2000);
	udbg2 = (volatile uint8_t *)ml_io_map(PCIE2_UDBG, 0x1000);
	pmu_update(PMU_PCIE1_PHY, 1, 1);
	printf("pcie: 3: isolation off (PMU 0x%08x); ELBI: LTSSM 0x%02x, soft reset 0x%x, D state %u; root port %08x\n",
	    pmu_read(PMU_PCIE1_PHY), rd(elbi, ELBI_LINKUP) & 0x3f, rd(elbi, ELBI_SOFT_RESET), rd(elbi, ELBI_PM_DSTATE) & 3,
	    rd(dbi, CFG_ID));
	phy_power_down();
	phy2_disable();
	wr(socr, 0x0, 0x15);
	phy_power_up();
	if (!phy_config()) {
		printf("pcie: 3: the PHY did not lock: stopped\n");
		return;
	}

	if (!announce(4, "writes: the root complex, PERST# let go, link training")) {
		return;
	}
	if (!establish_link()) {
		printf("pcie: 4: no link after ten tries: stopped\n");
		return;
	}

	if (!announce(5, "reads: the chip's configuration space, its BARs placed, its identity through BAR0")) {
		return;
	}
	cfg = (volatile uint8_t *)ml_io_map(PCIE1_CFG, PCIE1_CFG_SIZE);
	if (chip()) {
		printf("pcie: 5: the BCM4383 answers\n");
	}
}

static void
akita_pcie_init(void)
{
	static thread_call_t call;
	uint32_t delay_s = 8;
	uint64_t deadline;

	if (!PE_parse_boot_argn("pcie", &steps, sizeof(steps)) || steps == 0) {
		return;
	}
	PE_parse_boot_argn("pcie_delay", &delay_s, sizeof(delay_s));
	printf("pcie: steps 1 to %u in %u s\n", steps, delay_s);
	call = thread_call_allocate(pcie_run, NULL);
	clock_interval_to_deadline(delay_s, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_pcie_init);
