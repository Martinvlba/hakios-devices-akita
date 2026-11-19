/*
 * The PCIe link to the Pixel 8a's modem: zuma's PCIe channel 0 (the same Samsung-wrapped DesignWare root complex as
 * channel 1, the Wi-Fi chip's -- devices/akita/drivers/wifi/hakios.akita_pcie.c -- at 0x12100000 in HSI1: two lanes,
 * Gen3) with the Exynos 5300 (144d:a5a5) on its far end. hakios.akita_modem.c asks for the link: up at Gen1 on one
 * lane for the modem's ROM, down, up again at Gen3 on two lanes for its loader and firmware -- what Android does.
 *
 * Nothing here runs by itself. Each call says what it is about to touch and what it found.
 *
 * What the hardware is, and where each value comes from: Google's Android kernel for this phone (google-modules/soc/gs:
 * drivers/pci/controller/dwc/pcie-exynos-rc.c and pcie-exynos-zuma-rc-cal.c -- channel 0 is the "else" of its
 * channel-1 branches --, drivers/iommu/exynos-pcie-iommu-zuma.c, the pin banks of drivers/pinctrl/gs/pinctrl-gs.c),
 * the phone's device tree (pcie@12100000: ip-ver 0x986500, num-lanes 2, max-link-speed 3, pmu-offset 0x3ec4,
 * perst-delay-us 18000, ep-device-type 4, use-sysmmu, separated-msi; ranges: PCI 0x14e00000 = CPU 0x40000000), Linux's
 * DesignWare core, and a trace of Android's own register writes of two power-ons (11-19: Gen1 x1, then Gen3 x2), which
 * decided where the source can be read two ways. That code is GPL and none of it is here: the register values are the
 * hardware's facts.
 *
 * Three things Android has between the modem and memory, and what is done about each:
 *  - the PCIe SysMMU (sysmmu@12060000): Android turns its translation on with a table that maps the shared regions
 *    to themselves. Here it is left as found, OFF, and said: a SysMMU that is off passes addresses through (channel
 *    1's is never programmed by Android either, and the Wi-Fi chip's DMA works there).
 *  - the S2MPU of HSI1 (0x12090000): the hypervisor's under Android. Here there is none: its state is said, and if
 *    it protects any stream it is opened the way Linux does without a hypervisor (0xff to +0x54).
 *  - cache coherency: the two writes Android makes (DBI 0x8e8, SYSREG 0x704) are made; the shared memory is mapped
 *    uncached all the same.
 */
#include <kern/clock.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* ---- where things are ---- */

#define PCIE0_ELBI              0x12100000ULL
#define PCIE0_SOC               0x12114000ULL   /* "soc" 0x12110000 + 0x4000 */
#define PCIE0_UDBG              0x1213c000ULL   /* "udbg" 0x12130000 + 0xc000 */
#define PCIE0_PHY               0x12140000ULL   /* common 0x000.., lane 0 0x1000.., lane 1 0x2000.. */
#define PCIE0_PCS               0x12120000ULL
#define PCIE0_SYSREG            0x12020000ULL
#define PCIE0_DBI               0x12400000ULL
#define PCIE0_DBI2              0x12500000ULL   /* the shadow registers (BAR masks) */
#define PCIE0_ATU               0x12700000ULL
#define PCIE0_CFG               0x40ffe000ULL
#define PCIE0_CFG_SIZE          0x2000u
#define PCIE0_MEM               0x40000000ULL   /* = bus 0x14e00000; Android opens 2 MB of it */
#define PCIE0_MEM_BUS           0x14e00000u
#define PCIE0_MEM_SIZE          0x00200000u
#define PCIE0_SYSMMU            0x12060000ULL
#define PCIE0_S2MPU             0x12090000ULL
#define PCIE0_MSI_TARGET        0xf6200000u     /* cp_msi_rmem: the address the modem's MSIs are written to */

#define PMU_BASE                0x15460000ULL
#define PMU_HSI1_STATUS         (PMU_BASE + 0x2b04)     /* bit 0: the HSI1 block powered */
#define PMU_PCIE0_PHY           (PMU_BASE + 0x3ec4)     /* bit 0: 1 = isolation off; bit 10 set before training */
#define TENSOR_SMC_PMU_SEC_REG  0x82000504u
#define PMU_REG_READ            0
#define PMU_REG_RMW             2

#define PIN_HSI1                0x12040000ULL   /* gph0 at 0: PERST# pin 0, CLKREQ# pin 1 (pins 2, 3: the modem driver's) */
#define PIN_CON                 0x00
#define PIN_DAT                 0x04
#define PIN_PUD                 0x08
#define PIN_DRV                 0x0c

#define ELBI_IRQ0               0x000
#define ELBI_IRQ1               0x004
#define ELBI_IRQ2               0x008
#define ELBI_IRQ0_EN            0x010
#define ELBI_IRQ1_EN            0x014
#define ELBI_IRQ2_EN            0x018
#define ELBI_LTSSM_ENABLE       0x054
#define ELBI_XFER_PENDING       0x074
#define ELBI_DEVICE_TYPE        0x080
#define ELBI_LINKUP             0x2c8
#define ELBI_LINKDOWN_RST_CTRL  0x3a0
#define ELBI_SOFT_RESET         0x3a4
#define ELBI_QCH_SEL            0x3a8
#define ELBI_REQ_EXIT_L1_MODE   0x3bc
#define ELBI_SLV_PEND_SEL_NAK   0x3d8
#define ELBI_MSTR_PEND_SEL_NAK  0x474
#define ELBI_HISTORY_CHECK      0xc00
#define ELBI_STATE_POWER_S      0xc04
#define ELBI_STATE_POWER_M      0xc08
#define ELBI_DBI_L1_EXIT_DIS    0x1078
#define ELBI_PMA_RST0           0x1400
#define ELBI_PMA_RST1           0x1404
#define ELBI_PMA_RST2           0x1408
#define LTSSM_L0                0x11

#define CFG_ID                  0x00
#define CFG_COMMAND             0x04
#define CFG_CLASS_REV           0x08
#define CFG_BAR(n)              (0x10 + 4 * (n))
#define CFG_BUSES               0x18
#define CFG_INTERRUPT           0x3c
#define RC_PM_CAP               0x40
#define RC_EXP_CAP              0x70
#define EXP_LNKCAP              0x0c
#define EXP_LNKCTL_STA          0x10
#define EXP_DEVCTL2             0x28
#define EXP_LNKCTL2             0x30
#define DBI_PORT_LINK_CTRL      0x710
#define DBI_GEN2_CTRL           0x80c
#define DBI_MSI_ADDR_LO         0x820
#define DBI_MSI_ADDR_HI         0x824
#define DBI_MSI_ENABLE(n)       (0x828 + 12 * (n))
#define DBI_MSI_MASK(n)         (0x82c + 12 * (n))
#define DBI_MSI_STATUS(n)       (0x830 + 12 * (n))
#define DBI_GEN3_RELATED        0x890
#define DBI_MISC_CONTROL        0x8bc
#define DBI_COHERENCY           0x8e8
#define DBI_AUX_CLK_FREQ        0xb40
#define DBI_L1_SUBSTATES        0xb44

#define ATU_CR1                 0x00
#define ATU_CR2                 0x04
#define ATU_BASE_LO             0x08
#define ATU_BASE_HI             0x0c
#define ATU_LIMIT               0x10
#define ATU_TARGET_LO           0x14
#define ATU_TARGET_HI           0x18
#define ATU_TYPE_MEM            0
#define ATU_TYPE_CFG0           4

static volatile uint8_t *elbi, *socr, *udbg, *phy, *pcs, *sysreg, *dbi, *dbi2, *atu, *cfg, *pin, *sysmmu, *sysmmu_vid, *s2mpu;
static bool prepared, link_is_up;

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

static void
pmu_update(uint64_t address, uint32_t mask, uint32_t value)
{
	uint64_t r = smc(TENSOR_SMC_PMU_SEC_REG, address, PMU_REG_RMW, mask, value);

	if (r != 0) {
		printf("pcie0: the secure monitor refused PMU 0x%llx (mask 0x%x): %lld\n", address, mask, (long long)r);
	}
}

static void
sleep_ms(uint32_t ms)
{
	assert_wait_timeout((event_t)&prepared, THREAD_UNINT, ms, NSEC_PER_MSEC);
	thread_block(THREAD_CONTINUE_NULL);
}

static void
perst(int level)
{
	wr(pin, PIN_DAT, (rd(pin, PIN_DAT) & ~1u) | (uint32_t)level);
}

static bool
wait_bit(volatile uint8_t *base, uint32_t off, unsigned bit, const char *what)
{
	for (unsigned i = 0; i < 10000; i++) {
		delay(10);
		if (rd(base, off) & (1u << bit)) {
			printf("pcie0: %s after %u us\n", what, 10 * (i + 1));
			return true;
		}
	}
	printf("pcie0: %s: NOT after 100 ms (0x%x = 0x%08x)\n", what, off, rd(base, off));
	return false;
}

/* ---- the PHY (zuma, channel 0: two lanes) ---- */

static void
phy_power_down(void)
{
	clear_bits(phy, 0x204, 0x3u << 2);
	wr(pcs, 0x150, 0x300d9);                /* PLL off, bias on */
	delay(5);
	for (uint32_t lane = 0x1000; lane <= 0x2000; lane += 0x1000) {
		wr(phy, lane + 0x044, 0x2a);
		wr(phy, lane + 0x048, 0xaa);
		wr(phy, lane + 0x04c, 0xa8);
		wr(phy, lane + 0x050, 0x80);
		wr(phy, lane + 0x85c, 0x0a);
		delay(1);
	}
	wr(phy, 0x208, 0xff);
	wr(phy, 0x228, 0xff);
	delay(1);
	wr(phy, 0x580, 0x0a);
	wr(phy, 0x928, 0xaa);
	wr(phy, 0x00c, 0x0a);
	set_bits(phy, 0x600, 0x20);             /* the PLL's lock signal forced off */
	wr(pcs, 0x150, 0x300de);                /* PLL off, bias off */
	delay(5);
	for (unsigned i = 0; i < 100 && (rd(phy, 0x260) & 0x4) != 0; i++) {
		delay(1);
	}
	if ((rd(phy, 0x260) & 0x4) != 0) {
		printf("pcie0: the PHY's PLL did not stop (0x260 = 0x%x): the external PLL is left on\n", rd(phy, 0x260));
		return;
	}
	set_bits(udbg, 0x700, 1u << 1);         /* the external PLL off */
	delay(10);
}

static void
phy_power_up(void)
{
	clear_bits(udbg, 0x700, 1u << 1);
	delay(100);
	clear_bits(phy, 0x600, 0x30);
	wr(phy, 0x00c, 0x00);
	wr(phy, 0x928, 0x55);
	wr(phy, 0x580, 0x02);
	wr(phy, 0x208, 0x00);
	wr(phy, 0x228, 0x00);
	for (uint32_t lane = 0x1000; lane <= 0x2000; lane += 0x1000) {
		wr(phy, lane + 0x044, 0x00);
		wr(phy, lane + 0x048, 0x00);
		wr(phy, lane + 0x04c, 0x00);
		wr(phy, lane + 0x050, 0x00);
		wr(phy, lane + 0x85c, 0x00);
	}
	delay(10);
}

static const struct { uint16_t off; uint8_t value; } phy_common[] = {
	{ 0x0000, 0x88 }, { 0x001c, 0x66 }, { 0x01f4, 0x00 }, { 0x0510, 0x81 }, { 0x0514, 0x59 }, { 0x051c, 0x11 },
	{ 0x062c, 0x0e }, { 0x0644, 0x22 }, { 0x0688, 0x03 }, { 0x06d4, 0x28 }, { 0x0788, 0x64 }, { 0x078c, 0x64 },
	{ 0x0790, 0x50 }, { 0x0794, 0x50 }, { 0x0944, 0x05 }, { 0x0948, 0x05 }, { 0x094c, 0x05 }, { 0x0950, 0x05 },
	/* the 38.4 MHz reference clock to the external PLL */
	{ 0x0590, 0x02 }, { 0x07f8, 0xb0 }, { 0x0730, 0x08 },
	{ 0x0344, 0xc0 },               /* delay in L1.2 */
	{ 0x0040, 0x04 }, { 0x0204, 0x03 }, { 0x02d4, 0x1f }, { 0x0358, 0x10 },  /* CLKREQ# not followed in L1.2 */
	{ 0x0018, 0x01 },               /* ERIO on */
	{ 0x0514, 0x5b }, { 0x0608, 0x0c }, { 0x060c, 0x0f }, { 0x0610, 0x0f }, { 0x0614, 0x0f }, { 0x0618, 0x0f },
	{ 0x0510, 0x80 }, { 0x0688, 0x03 }, { 0x0644, 0x23 }, { 0x0624, 0x11 },
	{ 0x0630, 0x0f }, { 0x06d0, 0x53 },
}, phy_lane[] = {
	{ 0x140, 0x04 }, { 0x144, 0x04 }, { 0x148, 0x04 }, { 0x14c, 0x02 }, { 0x150, 0x00 }, { 0x154, 0x00 },
	{ 0x158, 0x00 }, { 0x15c, 0x00 }, { 0x2cc, 0x1c }, { 0x2dc, 0x6c }, { 0x30c, 0x29 }, { 0x3b4, 0x2f },
	{ 0xa64, 0x05 }, { 0xa68, 0x05 }, { 0xa84, 0x05 }, { 0xa88, 0x05 }, { 0xa98, 0x00 }, { 0xa9c, 0x00 },
	{ 0xaa8, 0x07 }, { 0xab8, 0x00 }, { 0xabc, 0x00 }, { 0xaf8, 0x90 }, { 0xb34, 0x03 }, { 0xbb0, 0x03 },
	{ 0xbb4, 0x03 }, { 0xbc0, 0x06 }, { 0xbc4, 0x06 }, { 0xbe8, 0x01 }, { 0xbf8, 0x04 }, { 0xc98, 0x00 },
	{ 0xca4, 0x81 },
};

static void
phy_config(void)
{
	bool pll, cdr, oc;

	wr(phy, 0x032c, 0x3);                   /* the PHY's input clock ungated */
	clear_bits(udbg, 0x710, 1u << 1);       /* the external PLL out of its init */
	clear_bits(udbg, 0x700, 1u << 1);       /* ... and its RESETB overridden */
	(void)wait_bit(udbg, 0x704, 3, "external PLL locked");
	clear_bits(socr, 0x4, 1u << 2);         /* the PHY's input clock: the external PLL, not TCXO */
	set_bits(socr, 0x4, 0x3u << 4);
	wr(elbi, ELBI_DEVICE_TYPE, 0x4);        /* root port */
	wr(elbi, ELBI_SOFT_RESET, 0xf);
	wr(elbi, ELBI_SOFT_RESET, 0xd);
	delay(10);
	wr(elbi, ELBI_SOFT_RESET, 0xf);
	delay(10);
	wr(elbi, ELBI_PMA_RST1, 1);
	wr(elbi, ELBI_PMA_RST2, 1);
	wr(elbi, ELBI_PMA_RST0, 1);
	wr(elbi, ELBI_PMA_RST1, 0);
	wr(elbi, ELBI_PMA_RST2, 0);
	wr(elbi, ELBI_PMA_RST0, 0);
	wr(elbi, ELBI_SLV_PEND_SEL_NAK, 1);

	for (unsigned i = 0; i < sizeof(phy_common) / sizeof(phy_common[0]); i++) {
		wr(phy, phy_common[i].off, phy_common[i].value);
	}
	for (uint32_t lane = 0x1000; lane <= 0x2000; lane += 0x1000) {
		for (unsigned i = 0; i < sizeof(phy_lane) / sizeof(phy_lane[0]); i++) {
			wr(phy, lane + phy_lane[i].off, phy_lane[i].value);
		}
	}

	wr(pcs, 0x190, 0x100b0604);
	wr(pcs, 0x154, 0x000700d5);
	wr(pcs, 0x100, 0x16400000);
	wr(pcs, 0x104, 0x08600000);
	wr(pcs, 0x114, 0x18500000);
	wr(pcs, 0x124, 0x60700000);
	wr(pcs, 0x174, 0x00000007);
	wr(pcs, 0x178, 0x00000100);
	wr(pcs, 0x17c, 0x00000010);

	wr(elbi, ELBI_PMA_RST1, 1);             /* the PMA's resets let go */
	delay(10);
	wr(elbi, ELBI_PMA_RST2, 1);
	wr(elbi, ELBI_PMA_RST0, 1);

	/*
	 * Android's code for this channel reads these three a lane's width (0x1000) further than channel 1's: its lane loop
	 * leaves the base moved. The trace shows them answer there, so the same are read (and, as there, only said).
	 */
	pll = wait_bit(phy, 0x1a80, 0, "PLL locked");
	cdr = wait_bit(phy, 0x25c0, 4, "CDR locked");
	oc = wait_bit(phy, 0x240c, 7, "offset calibration done");
	if (!pll || !cdr || !oc) {
		printf("pcie0: (PLL %d, CDR %d, offset calibration %d; the unshifted ones read 0x%x 0x%x 0x%x: Android goes on all the same)\n",
		    pll, cdr, oc, rd(phy, 0x0a80), rd(phy, 0x15c0), rd(phy, 0x140c));
	}
	set_bits(udbg, 0x710, 0x3u << 8);       /* the external PLL gated in L2 */
	wr(phy, 0x132c, 0x0);                   /* (the same shift) */
	wr(elbi, ELBI_DBI_L1_EXIT_DIS, 1);
}

/* ---- the root complex ---- */

static void
setup_rc(unsigned gen, unsigned lanes)
{
	uint32_t v;

	/* DesignWare's (Linux's dw_pcie_setup_rc, dw_pcie_setup) */
	set_bits(dbi, DBI_MISC_CONTROL, 1);     /* read-only registers writable */
	v = rd(dbi, RC_EXP_CAP + EXP_LNKCTL2) & ~0xfu;
	wr(dbi, RC_EXP_CAP + EXP_LNKCTL2, v | 3);
	v = rd(dbi, RC_EXP_CAP + EXP_LNKCAP) & ~0xfu;
	wr(dbi, RC_EXP_CAP + EXP_LNKCAP, v | 3);
	v = rd(dbi, DBI_PORT_LINK_CTRL);
	v &= ~(1u << 7);
	v |= 1u << 5;
	v = (v & ~(0x3fu << 16)) | (0x3u << 16);        /* two lanes */
	wr(dbi, DBI_PORT_LINK_CTRL, v);
	v = rd(dbi, DBI_GEN2_CTRL);
	wr(dbi, DBI_GEN2_CTRL, (v & ~(0x1fu << 8)) | (0x2u << 8));
	wr(dbi, DBI_MSI_MASK(0), 0xfffffffd);
	wr(dbi, DBI_MSI_ENABLE(0), 0xffffffff);
	wr(dbi, DBI_MSI_ADDR_LO, PCIE0_MSI_TARGET);
	wr(dbi, DBI_MSI_ADDR_HI, 0);
	wr(dbi, CFG_BAR(0), 0x00000004);
	wr(dbi, CFG_BAR(1), 0x00000000);
	v = rd(dbi, CFG_INTERRUPT);
	wr(dbi, CFG_INTERRUPT, (v & 0xffff00ffu) | 0x00000100u);
	v = rd(dbi, CFG_BUSES);
	wr(dbi, CFG_BUSES, (v & 0xff000000u) | 0x00ff0100u);
	v = rd(dbi, CFG_COMMAND);
	wr(dbi, CFG_COMMAND, (v & 0xffff0000u) | 0x107u);
	wr(dbi, CFG_BAR(0), 0);
	v = rd(dbi, CFG_CLASS_REV);
	wr(dbi, CFG_CLASS_REV, (v & 0xffffu) | (0x0604u << 16));
	set_bits(dbi, DBI_GEN2_CTRL, 1u << 17);
	clear_bits(dbi, DBI_MISC_CONTROL, 1);

	/* Samsung's (exynos_pcie_setup_rc) */
	wr(dbi, DBI_MISC_CONTROL, 1);
	wr(dbi2, CFG_BAR(0), 0);                /* the modem's channel only: the root port's own BAR0 and ROM BAR off */
	wr(dbi2, 0x30, 0);
	wr(dbi, CFG_ID, 0x144du | (0xececu << 16));
	v = rd(dbi, RC_EXP_CAP + EXP_LNKCAP);
	v &= ~((0x7u << 15) | 0xfu);
	v |= (0x7u << 15) | 3;
	wr(dbi, RC_EXP_CAP + EXP_LNKCAP, v);
	wr(dbi, DBI_AUX_CLK_FREQ, 0x1a);
	wr(dbi, DBI_L1_SUBSTATES, 0xea);
	wr(dbi, RC_PM_CAP + 4, 0);
	v = rd(dbi, RC_EXP_CAP + EXP_LNKCTL2) & ~0xfu;
	wr(dbi, RC_EXP_CAP + EXP_LNKCTL2, v | gen);
	v = rd(dbi, RC_EXP_CAP + EXP_DEVCTL2) & ~0xfu;
	wr(dbi, RC_EXP_CAP + EXP_DEVCTL2, v | 5);

	/* cache coherency, as Android has it for this channel */
	wr(dbi, DBI_COHERENCY, 0x10101010);
	set_bits(sysreg, 0x704, 0x3);

	clear_bits(elbi, ELBI_HISTORY_CHECK, 1u << 2);
	wr(elbi, ELBI_STATE_POWER_S, 0xffffffff);
	wr(elbi, ELBI_STATE_POWER_M, 0xffffffff);
	set_bits(elbi, ELBI_HISTORY_CHECK, 0x3);

	/* the width asked for: one lane for the ROM, two after */
	v = rd(dbi, DBI_PORT_LINK_CTRL);
	wr(dbi, DBI_PORT_LINK_CTRL, (v & ~(0x3fu << 16)) | ((lanes == 2 ? 0x3u : 0x1u) << 16));
	v = rd(dbi, DBI_GEN2_CTRL);
	wr(dbi, DBI_GEN2_CTRL, (v & ~(0x1fu << 8)) | ((uint32_t)lanes << 8));
}

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

static bool
train(unsigned gen, unsigned lanes)
{
	for (unsigned try = 1; try <= 10; try++) {
		uint32_t state = 0, status;
		unsigned waited = 0;

		phy_config();
		pmu_update(PMU_PCIE0_PHY, 1u << 10, 1u << 10);
		clear_bits(udbg, 0x800, 0x60);
		wr(phy, 0x032c, 0x0);
		wr(dbi, DBI_GEN3_RELATED, 0x12000);     /* no equalization (as Android: none of its presets is programmed) */
		perst(1);
		sleep_ms(19);
		set_bits(elbi, ELBI_REQ_EXIT_L1_MODE, 0x11);
		wr(elbi, ELBI_LINKDOWN_RST_CTRL, 2);
		wr(elbi, ELBI_XFER_PENDING, 1);
		clear_bits(elbi, ELBI_QCH_SEL, 0xfff);
		wr(elbi, ELBI_MSTR_PEND_SEL_NAK, 1);
		wr(elbi, ELBI_DBI_L1_EXIT_DIS, 1);
		setup_rc(gen, lanes);
		wr(elbi, ELBI_LTSSM_ENABLE, 1);
		for (waited = 0; waited < 4800; waited++) {
			state = rd(elbi, ELBI_LINKUP) & 0x3f;
			if (state == LTSSM_L0) {
				break;
			}
			delay(11);
		}
		if (state == LTSSM_L0) {
			if (gen != 1) {
				sleep_ms(3);    /* the change of speed happens by itself */
			}
			status = rd(dbi, RC_EXP_CAP + EXP_LNKCTL_STA);
			printf("pcie0: try %u: L0 after %u us: Gen%u x%u (asked: Gen%u x%u), LTSSM 0x%02x now\n", try, 11 * waited,
			    (status >> 16) & 0xf, (status >> 20) & 0x3f, gen, lanes, rd(elbi, ELBI_LINKUP) & 0x3f);
			if (((status >> 16) & 0xf) >= gen) {
				return true;
			}
		} else {
			printf("pcie0: try %u: no L0 in 53 ms: LTSSM 0x%02x\n", try, state);
		}
		perst(0);
		wr(elbi, ELBI_LTSSM_ENABLE, 0);
	}
	return false;
}

/* The blocks mapped, what stands between the modem and memory said, the pins, the PHY powered down as Android's probe. */
static bool
prepare(void)
{
	/* (read as memory, as the camera driver reads its block's status: the secure monitor refuses this one, -22) */
	uint32_t hsi1 = rd((volatile uint8_t *)ml_io_map(PMU_HSI1_STATUS & ~0xfffULL, 0x1000), PMU_HSI1_STATUS & 0xfff), id;

	printf("pcie0: PMU: HSI1's power status 0x%x (bit 0: on), channel 0's PHY control 0x%08x (bit 0: isolation off)\n", hsi1,
	    pmu_read(PMU_PCIE0_PHY));
	if ((hsi1 & 1) == 0) {
		printf("pcie0: HSI1 is not powered: nothing of it is touched: stopped\n");
		return false;
	}
	elbi = (volatile uint8_t *)ml_io_map(PCIE0_ELBI, 0x2000);
	socr = (volatile uint8_t *)ml_io_map(PCIE0_SOC, 0x1000);
	udbg = (volatile uint8_t *)ml_io_map(PCIE0_UDBG, 0x1000);
	phy = (volatile uint8_t *)ml_io_map(PCIE0_PHY, 0x3000);
	pcs = (volatile uint8_t *)ml_io_map(PCIE0_PCS, 0x1000);
	sysreg = (volatile uint8_t *)ml_io_map(PCIE0_SYSREG, 0x1000);
	dbi = (volatile uint8_t *)ml_io_map(PCIE0_DBI, 0x1000);
	dbi2 = (volatile uint8_t *)ml_io_map(PCIE0_DBI2, 0x1000);
	atu = (volatile uint8_t *)ml_io_map(PCIE0_ATU, 0x1000);
	cfg = (volatile uint8_t *)ml_io_map(PCIE0_CFG, PCIE0_CFG_SIZE);
	pin = (volatile uint8_t *)ml_io_map(PIN_HSI1, 0x1000);
	sysmmu = (volatile uint8_t *)ml_io_map(PCIE0_SYSMMU, 0x1000);
	sysmmu_vid = (volatile uint8_t *)ml_io_map(PCIE0_SYSMMU + 0x8000, 0x1000);
	s2mpu = (volatile uint8_t *)ml_io_map(PCIE0_S2MPU, 0x1000);

	id = rd(dbi, CFG_ID);
	printf("pcie0: the root port 0x%08x (0xa5a5144d out of reset); ELBI: LTSSM 0x%02x, soft reset 0x%x; SYSREG 0x704 0x%x\n", id,
	    rd(elbi, ELBI_LINKUP) & 0x3f, rd(elbi, ELBI_SOFT_RESET), rd(sysreg, 0x704));
	printf("pcie0: the SysMMU: control 0x%x, config 0x%x; stream 0's control 0x%x (bit 0: translating), table 0x%x, fault 0x%x\n",
	    rd(sysmmu, 0x0), rd(sysmmu, 0x8), rd(sysmmu_vid, 0x0), rd(sysmmu_vid, 0x404), rd(sysmmu_vid, 0x60));
	printf("pcie0: the S2MPU of HSI1: control 0x%x, protection per stream 0x%x\n", rd(s2mpu, 0x0), rd(s2mpu, 0x50));
	if ((rd(sysmmu_vid, 0x0) & 1) != 0) {
		printf("pcie0: the SysMMU translates already (nobody here set it up): its stream 0 is turned off\n");
		clear_bits(sysmmu_vid, 0x0, 1);
		wr(sysmmu_vid, 0x10, 1);
	}
	if ((rd(s2mpu, 0x50) & 0xff) != 0) {
		wr(s2mpu, 0x54, 0xff);          /* Linux's way without a hypervisor: no stream protected */
		printf("pcie0: the S2MPU protected streams: opened, now 0x%x\n", rd(s2mpu, 0x50));
	}

	/* the pins: PERST# an output, low; CLKREQ# its function, pulled up (gph0-2, -3 are left alone) */
	wr(pin, PIN_DAT, rd(pin, PIN_DAT) & ~1u);
	wr(pin, PIN_DRV, (rd(pin, PIN_DRV) & ~0xffu) | 0x33);
	wr(pin, PIN_PUD, (rd(pin, PIN_PUD) & ~0xf0u) | 0x30);
	wr(pin, PIN_CON, (rd(pin, PIN_CON) & ~0xffu) | 0x21);
	printf("pcie0: gph0 CON 0x%08x DAT 0x%02x PUD 0x%08x DRV 0x%08x\n", rd(pin, PIN_CON), rd(pin, PIN_DAT), rd(pin, PIN_PUD),
	    rd(pin, PIN_DRV));

	/* Android's probe: no interrupt of the wrapper, the PHY's isolation off, the PHY powered down */
	wr(elbi, ELBI_IRQ0_EN, 0);
	wr(elbi, ELBI_IRQ1_EN, 0);
	wr(elbi, ELBI_IRQ2_EN, 0);
	pmu_update(PMU_PCIE0_PHY, 1, 1);
	phy_power_down();
	prepared = true;
	return true;
}

/*
 * The link up: gen 1 for the modem's ROM (one lane, as Android's first power-on turns out), gen 3 after (two lanes).
 * True with the link in L0 at that speed, the configuration and memory windows open, the MSI registers set.
 */
bool
akita_pcie0_up(unsigned gen)
{
	unsigned lanes = gen == 1 ? 1 : 2;

	printf("pcie0: the link up: Gen%u, %u lane%s\n", gen, lanes, lanes == 1 ? "" : "s");
	if (!prepared && !prepare()) {
		return false;
	}
	wr(socr, 0x0, 0x15);
	phy_power_up();
	if (!train(gen, lanes)) {
		printf("pcie0: no link after ten tries\n");
		return false;
	}
	wr(elbi, ELBI_IRQ0, rd(elbi, ELBI_IRQ0));       /* what the wrapper noted meanwhile, cleared */
	wr(elbi, ELBI_IRQ1, rd(elbi, ELBI_IRQ1));
	wr(elbi, ELBI_IRQ2, rd(elbi, ELBI_IRQ2));
	atu_region(0, ATU_TYPE_CFG0, PCIE0_CFG, PCIE0_CFG + PCIE0_CFG_SIZE - 1, 0x01000000);       /* bus 1, device 0 */
	atu_region(1, ATU_TYPE_MEM, PCIE0_MEM, PCIE0_MEM + PCIE0_MEM_SIZE - 1, PCIE0_MEM_BUS);
	wr(elbi, ELBI_XFER_PENDING, 0);
	/* the MSIs: where the modem's writes are caught, no MSI on the wrapper's own line, vectors 0 and 4 to 7 of group 0 */
	wr(dbi, DBI_MSI_ADDR_LO, PCIE0_MSI_TARGET);
	wr(dbi, DBI_MSI_ADDR_HI, 0);
	clear_bits(elbi, ELBI_IRQ2_EN, 1u << 17);
	wr(dbi, DBI_MSI_ENABLE(0), 0xf1);
	wr(dbi, DBI_MSI_MASK(0), 0xffffff0c);
	wr(pcs, 0x150, 0x300d5);                /* this channel only: PLL and bias always on */
	link_is_up = true;
	return true;
}

/*
 * The link down. Android asks the far end first (PME_Turn_Off, then waits for its L23 ready -- which the modem's ROM
 * never sends: "cannot receive L23_READY" is in Android's log at every boot); here it is simply taken down: LTSSM
 * off, PERST# low, the PHY powered down.
 */
void
akita_pcie0_down(void)
{
	if (!prepared) {
		return;
	}
	printf("pcie0: the link down (LTSSM 0x%02x)\n", rd(elbi, ELBI_LINKUP) & 0x3f);
	link_is_up = false;
	clear_bits(elbi, ELBI_HISTORY_CHECK, 0x3);
	perst(0);
	clear_bits(socr, 0x0, 0x3);
	wr(elbi, ELBI_LTSSM_ENABLE, 0);
	clear_bits(elbi, ELBI_SOFT_RESET, 0x2);         /* soft power reset */
	delay(20);
	set_bits(elbi, ELBI_SOFT_RESET, 0x2);
	phy_power_down();
}

/* the endpoint's configuration space (bus 1, device 0, function 0) through region 0 */
uint32_t
akita_pcie0_ep_read(uint32_t reg)
{
	uint32_t v;

	if (!link_is_up) {
		return 0xffffffff;
	}
	wr(elbi, ELBI_XFER_PENDING, 1);
	v = rd(cfg, reg & 0xffc);
	wr(elbi, ELBI_XFER_PENDING, 0);
	return v;
}

void
akita_pcie0_ep_write(uint32_t reg, uint32_t value)
{
	if (!link_is_up) {
		return;
	}
	wr(elbi, ELBI_XFER_PENDING, 1);
	wr(cfg, reg & 0xffc, value);
	wr(elbi, ELBI_XFER_PENDING, 0);
}

/* a piece of the memory window (bus 0x14e00000 + offset), mapped */
volatile uint8_t *
akita_pcie0_window(uint32_t bus_offset, uint32_t size)
{
	if (bus_offset + size > PCIE0_MEM_SIZE) {
		return NULL;
	}
	return (volatile uint8_t *)ml_io_map(PCIE0_MEM + bus_offset, size);
}

/* the MSIs that arrived (group 0's status: bit 4 the modem's message, bit 5 its status), read and cleared */
uint32_t
akita_pcie0_msi_take(void)
{
	uint32_t s;

	if (!link_is_up) {
		return 0;
	}
	s = rd(dbi, DBI_MSI_STATUS(0));
	if (s != 0) {
		wr(dbi, DBI_MSI_STATUS(0), s);
	}
	return s;
}
