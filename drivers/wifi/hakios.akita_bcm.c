/*
 * The Pixel 8a's Wi-Fi chip, the Broadcom BCM4383 (rev a3), from the far end of the PCIe link (hakios.akita_pcie.c
 * brings the link up; this file starts at its step 5): the chip's cores found, its ARM Cortex-R4 halted, the radio's
 * firmware and the board's calibration written into the chip's memory, the ARM started, and the firmware's answer
 * awaited -- the address of the area it shares with the host ("pcie_shared"), written at the top of its memory.
 *
 * Only with boot-arg wifi=N (and pcie=5 before it): steps 1 to N, each saying what it is about to touch before it
 * does and what it found after; a step that does not find what it needs ends the sequence.
 *
 *   1  reads   the cores, from the chip's enumeration ROM (chipcommon's EROM pointer): their ids, revisions,
 *              register and wrapper addresses; the PCIe core's revision
 *   2  writes  the chip's completion timeout on (an access the chip cannot complete is answered with an error,
 *              not left hanging: Android's 0x8a058000), the ARM's backplane powered (the PCIe core's power request,
 *              a multi-backplane chip), the bootloader's scratch registers cleared; then the ARM's state, the ARM
 *              halted, the size of its memory (the CR4's banks: 2228224 bytes at 0x6e0000 on Android), the word at
 *              the top of memory zeroed
 *   3  writes  the firmware (fw/fw_bcmdhd.bin_4383_a3) at the bottom of the memory and read back; the calibration
 *              (fw/bcmdhd.cal_4383_a3, its text turned into a list of NUL-ended "name=value" as Google's driver does)
 *              below the top with its length token in the last word (Android: 11716 bytes, token 0xf48e0b71);
 *              128 random bytes below it (a reversed TLV of type 0xfeedc0de: the firmware's heap randomisation);
 *              the firmware's first word -- its reset vector -- at address 0
 *   4  runs    the ARM let go; the word at the top of memory awaited until it is no longer the length token: it is
 *              then the address of pcie_shared (Android: 0x8d7a98 after ~770 ms). Its flags (protocol version...)
 *              and the firmware's console (its own log, in its memory) are read into the log
 * The chip is left running: the next steps (the message rings) build on it.
 *
 * How the chip is reached: BAR0 (64 KB at 0x60200000) shows 4 KB of the chip's backplane at the address in its
 * configuration register 0x80 (the "BAR0 window"); BAR0 + 0x2000 is always the PCIe core's registers. BAR2 (4 MB at
 * 0x60400000) shows 4 MB of the backplane from the 4 MB boundary in configuration register 0x84 (the "BAR1 window":
 * Broadcom counts its BARs in pairs); the memory 0x6e0000-0x8fffff crosses one, so writes move the window.
 *
 * Where each value comes from: Google's driver for this chip (google-modules/wlan/bcmdhd/bcm4383, android-gs-akita-
 * android16: dhd_pcie.c's dongle attach and download, siutils/aiutils, the headers; GPL, read for the facts, none of
 * it here) and Linux's brcmfmac (ISC: the EROM scan and the core reset are done its way); Android's log of the same
 * start on this phone (11-09). The firmware files are the phone's own (/vendor/firmware of its Android, fw-wifi/).
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/sched_prim.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/random.h>
#include <sys/systm.h>
#include <vm/memory_types.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, unsigned int flags);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
extern uint64_t mach_absolute_time(void);
extern void absolutetime_to_nanoseconds(uint64_t abstime, uint64_t *result);

/* from hakios.akita_pcie.c: the chip's configuration space (through the root complex's window) and its BARs */
volatile uint8_t *akita_pcie_config(void);
uint64_t akita_pcie_bar(unsigned n);
void akita_bcm_run(void);

#define FW_NAME                 "fw_bcmdhd.bin_4383_a3"
#define NVRAM_NAME              "bcmdhd.cal_4383_a3"

/* the chip's configuration registers */
#define CFG_BAR0_WIN            0x80
#define CFG_BAR1_WIN            0x84            /* BAR2's window (4 MB aligned) */
#define CFG_INT_MASK            0x94
#define CFG_INT_MASK_CTO        (1u << 16)
#define CFG_INT_MASK_SERR       0x4u

#define BAR0_SIZE               0x4000          /* what is mapped of BAR0: the window, the PCIe core at +0x2000 */
#define BAR0_PCIE_CORE          0x2000
#define BAR2_SIZE               0x400000u

/* the PCIe core's registers (BAR0 + 0x2000) */
#define PCIE_CPL_TO_CTRL        0x3c            /* completion timeout */
#define PCIE_CONFIG_IND_ADDR    0x120
#define PCIE_CONFIG_IND_DATA    0x124
#define PCIE_POWER_CONTROL      0x1e8           /* the power requests of a multi-backplane chip */
#define SRPWR_ARM_DOMAIN        0x2u            /* domain 1: ARM, backplane, SDIO */
#define SRPWR_REQ_SHIFT         8
#define SRPWR_STATUS_SHIFT      16
#define CTO_CTRL_ANDROID        0x8a058000u     /* what Android's dhdpcie_cto_init left (threshold, clock check, on) */

/* chipcommon (the first core, at the enumeration base) */
#define CC_BASE                 0x18000000u
#define CC_CHIPID               0x000
#define CC_BL_SCRATCH_TO_BL     0x0d0           /* BackplaneAddrLow, used as the host -> bootloader word */
#define CC_BL_SCRATCH_FROM_BL   0x0d4
#define CC_EROMPTR              0x0fc

/* the enumeration ROM's descriptors (brcmfmac's names) */
#define DMP_DESC_TYPE_MSK       0x0000000fu
#define DMP_DESC_EMPTY          0x0u
#define DMP_DESC_VALID          0x1u
#define DMP_DESC_COMPONENT      0x1u
#define DMP_DESC_MASTER_PORT    0x3u
#define DMP_DESC_ADDRESS        0x5u
#define DMP_DESC_ADDRSIZE_GT32  0x8u
#define DMP_DESC_EOT            0xfu
#define DMP_COMP_PARTNUM(v)     (((v) >> 8) & 0xfffu)
#define DMP_COMP_REVISION(v)    (((v) >> 24) & 0xffu)
#define DMP_COMP_NUM_SWRAP(v)   (((v) >> 19) & 0x1fu)
#define DMP_COMP_NUM_MWRAP(v)   (((v) >> 14) & 0x1fu)
#define DMP_SLAVE_ADDR_BASE     0xfffff000u
#define DMP_SLAVE_TYPE(v)       (((v) >> 6) & 3u)
#define DMP_SLAVE_TYPE_SLAVE    0
#define DMP_SLAVE_TYPE_SWRAP    2
#define DMP_SLAVE_TYPE_MWRAP    3
#define DMP_SLAVE_SIZE_TYPE(v)  (((v) >> 4) & 3u)
#define DMP_SLAVE_SIZE_4K       0
#define DMP_SLAVE_SIZE_8K       1
#define DMP_SLAVE_SIZE_DESC     3

/* core ids */
#define CORE_CC                 0x800
#define CORE_80211              0x812
#define CORE_PMU                0x827
#define CORE_PCIE2              0x83c
#define CORE_ARM_CR4            0x83e
#define CORE_GCI                0x840
#define CORE_ARM_CA7            0x847
#define CORE_SYSMEM             0x849

/* a core's wrapper (AI) */
#define AI_IOCTRL               0x408
#define AI_IOSTATUS             0x500
#define AI_RESETCTRL            0x800
#define AI_RESETSTATUS          0x804
#define AIRC_RESET              1u
#define SICF_CLOCK_EN           0x1u
#define SICF_FGC                0x2u
#define SICF_CPUHALT            0x20u

/* the ARM CR4's registers */
#define CR4_CAP                 0x04
#define CR4_BANKIDX             0x40
#define CR4_BANKINFO            0x44

#define RAM_BASE_4383           0x6e0000u       /* Google's CR4_4383_RAM_BASE */
#define RNG_TLV_TYPE            0xfeedc0deu
#define RNG_BYTES               128u

/* pcie_shared, at the address the firmware leaves at the top of its memory */
#define SHARED_FLAGS            0
#define SHARED_CONSOLE_ADDR     20
#define SHARED_MAX_RXBUFPOST    34
#define SHARED_RX_DATAOFFSET    36
#define SHARED_H2D_MB_ADDR      40
#define SHARED_D2H_MB_ADDR      44
#define SHARED_RING_INFO_ADDR   48
#define CONSOLE_BUF_ADDR        8
#define CONSOLE_BUF_SIZE        12
#define CONSOLE_WRITE_IDX       16

static unsigned int steps, step;
static volatile uint8_t *cfg, *bar0, *bar2;
static uint32_t bar0_win = ~0u, bar2_win = ~0u;

#define CORES_MOST              32
static struct core {
	uint16_t id;
	uint8_t rev;
	uint32_t base, wrap;
} cores[CORES_MOST];
static unsigned core_count;
static uint32_t ram_base, ram_size;

/* ---- access ---- */

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

/* any backplane address through BAR0's 4 KB window */
static uint32_t
bp_rd(uint32_t address)
{
	uint32_t win = address & ~0xfffu;

	if (win != bar0_win) {
		wr(cfg, CFG_BAR0_WIN, win);
		bar0_win = win;
	}
	return rd(bar0, address & 0xfff);
}

static void
bp_wr(uint32_t address, uint32_t value)
{
	uint32_t win = address & ~0xfffu;

	if (win != bar0_win) {
		wr(cfg, CFG_BAR0_WIN, win);
		bar0_win = win;
	}
	wr(bar0, address & 0xfff, value);
}

static inline uint32_t
pcie_rd(uint32_t off)
{
	return rd(bar0, BAR0_PCIE_CORE + off);
}

static inline void
pcie_wr(uint32_t off, uint32_t value)
{
	wr(bar0, BAR0_PCIE_CORE + off, value);
}

/* the chip's memory (any backplane address) through BAR2's 4 MB window */
static volatile uint8_t *
mem_at(uint32_t address)
{
	uint32_t win = address & ~(BAR2_SIZE - 1);

	if (win != bar2_win) {
		wr(cfg, CFG_BAR1_WIN, win);
		bar2_win = win;
	}
	return bar2 + (address - win);
}

static uint32_t
mem_rd(uint32_t address)
{
	return *(volatile uint32_t *)mem_at(address);
}

static void
mem_wr(uint32_t address, uint32_t value)
{
	*(volatile uint32_t *)mem_at(address) = value;
}

/* `length` bytes (rounded up to words, the tail zero-filled) from `data` to the chip's `address` */
static void
mem_write(uint32_t address, const uint8_t *data, uint32_t length)
{
	for (uint32_t i = 0; i < length; i += 4) {
		uint32_t w = 0;

		memcpy(&w, data + i, length - i >= 4 ? 4 : length - i);
		mem_wr(address + i, w);
	}
	__asm__ volatile ("dsb sy" ::: "memory");
}

/* how many words differ between `data` and the chip's `address` (the first ones said) */
static uint32_t
mem_compare(uint32_t address, const uint8_t *data, uint32_t length, const char *what)
{
	uint32_t bad = 0;

	for (uint32_t i = 0; i < length; i += 4) {
		uint32_t w = 0, got = mem_rd(address + i);

		memcpy(&w, data + i, length - i >= 4 ? 4 : length - i);
		if (got != w) {
			if (bad < 4) {
				printf("wifi: %u: %s: 0x%08x is 0x%08x, not 0x%08x\n", step, what, address + i, got, w);
			}
			bad++;
		}
	}
	return bad;
}

static void
sleep_ms(uint32_t ms)
{
	assert_wait_timeout((event_t)&steps, THREAD_UNINT, ms, NSEC_PER_MSEC);
	thread_block(THREAD_CONTINUE_NULL);
}

static uint64_t
micros_since(uint64_t since)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time() - since, &ns);
	return ns / 1000;
}

static bool
announce(unsigned int n, const char *what)
{
	if (n > steps) {
		return false;
	}
	step = n;
	printf("wifi: %u: %s\n", n, what);
	return true;
}

/* ---- the cores ---- */

static const char *
core_name(uint16_t id)
{
	switch (id) {
	case CORE_CC: return "chipcommon";
	case CORE_80211: return "802.11";
	case CORE_PMU: return "PMU";
	case CORE_PCIE2: return "PCIe2";
	case CORE_ARM_CR4: return "ARM CR4";
	case CORE_GCI: return "GCI";
	case CORE_ARM_CA7: return "ARM CA7";
	case CORE_SYSMEM: return "sysmem";
	default: return "";
	}
}

static struct core *
core_find(uint16_t id)
{
	for (unsigned i = 0; i < core_count; i++) {
		if (cores[i].id == id) {
			return &cores[i];
		}
	}
	return NULL;
}

static uint32_t
erom_next(uint32_t *at, uint32_t *type)
{
	uint32_t v = bp_rd(*at);

	*at += 4;
	if (type) {
		*type = v & DMP_DESC_TYPE_MSK;
		if ((*type & ~DMP_DESC_ADDRSIZE_GT32) == DMP_DESC_ADDRESS) {
			*type = DMP_DESC_ADDRESS;
		}
	}
	return v;
}

/* a component's register and wrapper addresses (brcmf_chip_dmp_get_regaddr) */
static bool
erom_regaddr(uint32_t *at, uint32_t *base, uint32_t *wrap)
{
	uint32_t type, v, wraptype;

	*base = 0;
	*wrap = 0;
	v = erom_next(at, &type);
	if (type == DMP_DESC_MASTER_PORT) {
		wraptype = DMP_SLAVE_TYPE_MWRAP;
	} else if (type == DMP_DESC_ADDRESS) {
		*at -= 4;
		wraptype = DMP_SLAVE_TYPE_SWRAP;
	} else {
		*at -= 4;
		return false;
	}
	do {
		do {
			v = erom_next(at, &type);
			if (type == DMP_DESC_EOT) {
				*at -= 4;
				return false;
			}
		} while (type != DMP_DESC_ADDRESS && type != DMP_DESC_COMPONENT);
		if (type == DMP_DESC_COMPONENT) {
			*at -= 4;
			return true;
		}
		if (v & DMP_DESC_ADDRSIZE_GT32) {
			(void)erom_next(at, NULL);
		}
		uint32_t sztype = DMP_SLAVE_SIZE_TYPE(v);

		if (sztype == DMP_SLAVE_SIZE_DESC) {
			uint32_t sz = erom_next(at, NULL);

			if (sz & DMP_DESC_ADDRSIZE_GT32) {
				(void)erom_next(at, NULL);
			}
		}
		if (sztype != DMP_SLAVE_SIZE_4K && sztype != DMP_SLAVE_SIZE_8K) {
			continue;
		}
		uint32_t stype = DMP_SLAVE_TYPE(v);

		if (*base == 0 && stype == DMP_SLAVE_TYPE_SLAVE) {
			*base = v & DMP_SLAVE_ADDR_BASE;
		}
		if (*wrap == 0 && stype == wraptype) {
			*wrap = v & DMP_SLAVE_ADDR_BASE;
		}
	} while (*base == 0 || *wrap == 0);
	return true;
}

static bool
erom_scan(void)
{
	uint32_t at = bp_rd(CC_BASE + CC_EROMPTR), start = at, type = 0;

	printf("wifi: 1: the enumeration ROM at 0x%08x\n", at);
	core_count = 0;
	while (type != DMP_DESC_EOT && at - start < 0x1000) {
		uint32_t v = erom_next(&at, &type), base, wrap;

		if (!(v & DMP_DESC_VALID) || type == DMP_DESC_EMPTY || type != DMP_DESC_COMPONENT) {
			continue;
		}
		uint16_t id = DMP_COMP_PARTNUM(v);

		v = erom_next(&at, &type);
		if ((v & DMP_DESC_TYPE_MSK) != DMP_DESC_COMPONENT) {
			printf("wifi: 1: the ROM's second component word is 0x%08x: stopped\n", v);
			return false;
		}
		unsigned nmw = DMP_COMP_NUM_MWRAP(v), nsw = DMP_COMP_NUM_SWRAP(v);
		uint8_t rev = DMP_COMP_REVISION(v);

		if (nmw + nsw == 0 && id != CORE_PMU && id != CORE_GCI) {
			continue;
		}
		if (!erom_regaddr(&at, &base, &wrap)) {
			continue;
		}
		if (core_count < CORES_MOST) {
			cores[core_count++] = (struct core){ .id = id, .rev = rev, .base = base, .wrap = wrap };
		}
		printf("wifi: 1: core 0x%03x %-10s rev %3u: registers 0x%08x, wrapper 0x%08x\n", id, core_name(id), rev,
		    base, wrap);
	}
	return core_count > 0;
}

/* (aiutils' _ai_core_reset: Google's driver resets the ARM this way) */
static void
core_reset(const struct core *c, uint32_t bits, uint32_t resetbits)
{
	unsigned i;

	for (i = 0; i < 300 && bp_rd(c->wrap + AI_RESETSTATUS) != 0; i++) {
		delay(1);
	}
	bp_wr(c->wrap + AI_RESETCTRL, AIRC_RESET);
	delay(10);
	for (i = 0; i < 300 && bp_rd(c->wrap + AI_RESETSTATUS) != 0; i++) {
		delay(1);
	}
	bp_wr(c->wrap + AI_IOCTRL, bits | resetbits | SICF_FGC | SICF_CLOCK_EN);
	(void)bp_rd(c->wrap + AI_IOCTRL);
	for (i = 0; i < 300 && bp_rd(c->wrap + AI_RESETSTATUS) != 0; i++) {
		delay(1);
	}
	for (i = 0; i < 10 && bp_rd(c->wrap + AI_RESETCTRL) != 0; i++) {
		bp_wr(c->wrap + AI_RESETCTRL, 0);
		for (unsigned j = 0; j < 300 && bp_rd(c->wrap + AI_RESETSTATUS) != 0; j++) {
			delay(1);
		}
	}
	bp_wr(c->wrap + AI_IOCTRL, bits | SICF_CLOCK_EN);
	(void)bp_rd(c->wrap + AI_IOCTRL);
}

static void
core_say(const struct core *c, const char *when)
{
	printf("wifi: %u: %s %s: ioctrl 0x%x, iostatus 0x%x, resetctrl 0x%x, resetstatus 0x%x\n", step, core_name(c->id),
	    when, bp_rd(c->wrap + AI_IOCTRL), bp_rd(c->wrap + AI_IOSTATUS), bp_rd(c->wrap + AI_RESETCTRL),
	    bp_rd(c->wrap + AI_RESETSTATUS));
}

/* the CR4's memory: the sum of its banks (si_tcm_size) */
static uint32_t
cr4_ram_size(const struct core *arm)
{
	uint32_t cap = bp_rd(arm->base + CR4_CAP), nab = cap & 0xf, nbb = (cap >> 4) & 0xf, size = 0;

	for (uint32_t i = 0; i < nab + nbb; i++) {
		bp_wr(arm->base + CR4_BANKIDX, i);
		uint32_t info = bp_rd(arm->base + CR4_BANKINFO);

		size += ((info & 0x7f) + 1) * ((info & 0x200) ? 1024 : 8192);
	}
	printf("wifi: %u: ARM CR4: capabilities 0x%08x, %u + %u banks, %u bytes\n", step, cap, nab, nbb, size);
	return size;
}

/* ---- the calibration ---- */

/*
 * The board's file as the firmware wants it (Google's process_nvram_vars + dhdpcie_download_nvram): '\r' dropped,
 * '#' to the end of the line dropped, empty lines dropped, each line ended by a NUL; padded to a word with NULs, then
 * one more NUL (the list's end), the whole rounded up to a word. Returns the length; *out is kalloc'ed.
 */
static uint32_t
nvram_vars(const uint8_t *text, uint64_t length, uint8_t **out, uint32_t *allocated)
{
	uint32_t size = (uint32_t)length + 8, n = 0, column = 0;
	uint8_t *v = kalloc_data(size, Z_WAITOK | Z_ZERO);
	bool comment = false;

	if (v == NULL) {
		return 0;
	}
	for (uint64_t i = 0; i < length; i++) {
		uint8_t c = text[i];

		if (c == '\r') {
			continue;
		}
		if (comment && c != '\n') {
			continue;
		}
		comment = false;
		if (c == '#') {
			comment = true;
			continue;
		}
		if (c == '\n') {
			if (column == 0) {
				continue;
			}
			v[n++] = 0;
			column = 0;
			continue;
		}
		v[n++] = c;
		column++;
	}
	if (column != 0) {
		v[n++] = 0;             /* a last line with no newline */
	}
	n = (n + 3) & ~3u;
	n += 1;                         /* the list's end */
	n = (n + 3) & ~3u;
	*out = v;
	*allocated = size;
	return n;
}

/* ---- the sequence ---- */

static void
console_say(uint32_t shared)
{
	uint32_t console = mem_rd(shared + SHARED_CONSOLE_ADDR), buf, size, widx;

	if (console < ram_base || console >= ram_base + ram_size) {
		printf("wifi: %u: no console (0x%08x)\n", step, console);
		return;
	}
	buf = mem_rd(console + CONSOLE_BUF_ADDR);
	size = mem_rd(console + CONSOLE_BUF_SIZE);
	widx = mem_rd(console + CONSOLE_WRITE_IDX);
	printf("wifi: %u: the firmware's console at 0x%08x: buffer 0x%08x, %u bytes, written to %u\n", step, console, buf,
	    size, widx);
	if (buf < ram_base || size == 0 || size > 0x10000 || buf + size > ram_base + ram_size || widx >= size) {
		return;
	}
	char line[160];
	unsigned n = 0;

	for (uint32_t i = 0; i < widx; i++) {
		uint32_t w = mem_rd(buf + (i & ~3u));
		char c = (char)(w >> (8 * (i & 3)));

		if (c == '\n' || c == 0 || n == sizeof(line) - 1) {
			if (n) {
				line[n] = 0;
				printf("wifi: fw: %s\n", line);
				n = 0;
			}
			continue;
		}
		if (c != '\r') {
			line[n++] = c;
		}
	}
	if (n) {
		line[n] = 0;
		printf("wifi: fw: %s\n", line);
	}
}

void
akita_bcm_run(void)
{
	struct core *arm, *pcie;
	const uint8_t *fw, *nv;
	uint64_t fw_size = 0, nv_size = 0;
	uint8_t *vars = NULL;
	uint32_t vars_len = 0, vars_alloc = 0, token = 0, top, shared = 0;

	if (!PE_parse_boot_argn("wifi", &steps, sizeof(steps)) || steps == 0) {
		return;
	}
	cfg = akita_pcie_config();
	bar0 = (volatile uint8_t *)ml_io_map(akita_pcie_bar(0), BAR0_SIZE);
	bar0_win = rd(cfg, CFG_BAR0_WIN);

	if (!announce(1, "reads: the chip's cores (enumeration ROM)")) {
		return;
	}
	printf("wifi: 1: chipcommon id 0x%08x; PCIe core: power control 0x%08x, completion timeout 0x%08x\n",
	    bp_rd(CC_BASE + CC_CHIPID), pcie_rd(PCIE_POWER_CONTROL), pcie_rd(PCIE_CPL_TO_CTRL));
	if (!erom_scan()) {
		printf("wifi: 1: no cores: stopped\n");
		return;
	}
	arm = core_find(CORE_ARM_CR4);
	pcie = core_find(CORE_PCIE2);
	if (arm == NULL || pcie == NULL) {
		printf("wifi: 1: no %s core: stopped (an ARM CA7 or sysmem chip is not what Google's driver says the 4383 "
		    "is)\n", arm == NULL ? "ARM CR4" : "PCIe2");
		return;
	}
	printf("wifi: 1: PCIe core revision %u (Android: buscorerev 77)\n", pcie->rev);

	/* (the ARM's registers only after its domain is powered and the completion timeout is on: step 2) */
	if (!announce(2, "writes: completion timeout on, the ARM's backplane powered, the ARM halted, its memory sized")) {
		return;
	}
	wr(cfg, CFG_INT_MASK, CFG_INT_MASK_CTO | CFG_INT_MASK_SERR);
	pcie_wr(PCIE_CPL_TO_CTRL, CTO_CTRL_ANDROID);
	{
		uint32_t r = pcie_rd(PCIE_POWER_CONTROL);
		unsigned i;

		if ((r & (SRPWR_ARM_DOMAIN << SRPWR_REQ_SHIFT)) == 0) {
			pcie_wr(PCIE_POWER_CONTROL, r | (SRPWR_ARM_DOMAIN << SRPWR_REQ_SHIFT));
		}
		for (i = 0; i < 1000 && !(pcie_rd(PCIE_POWER_CONTROL) & (SRPWR_ARM_DOMAIN << SRPWR_STATUS_SHIFT)); i++) {
			delay(10);
		}
		printf("wifi: 2: power control 0x%08x -> 0x%08x%s; completion timeout 0x%08x\n", r,
		    pcie_rd(PCIE_POWER_CONTROL), i == 1000 ? " (the ARM domain did NOT come on)" : "",
		    pcie_rd(PCIE_CPL_TO_CTRL));
	}
	bp_wr(CC_BASE + CC_BL_SCRATCH_TO_BL, 0);
	bp_wr(CC_BASE + CC_BL_SCRATCH_FROM_BL, 0);
	pcie_wr(PCIE_CONFIG_IND_ADDR, 0x4e0);   /* (Google's attach writes this word of the core's config back) */
	pcie_wr(PCIE_CONFIG_IND_DATA, pcie_rd(PCIE_CONFIG_IND_DATA));

	bar2 = (volatile uint8_t *)ml_io_map_unmappable(akita_pcie_bar(2), BAR2_SIZE, VM_WIMG_IO);
	wr(cfg, CFG_BAR1_WIN, 0);
	bar2_win = 0;
	core_say(arm, "as found");
	core_reset(arm, SICF_CPUHALT, SICF_CPUHALT);
	core_say(arm, "halted");
	ram_base = RAM_BASE_4383;
	ram_size = cr4_ram_size(arm);
	if (ram_size == 0 || ram_size > 0x800000) {
		printf("wifi: 2: a memory of %u bytes cannot be: stopped\n", ram_size);
		return;
	}
	printf("wifi: 2: memory 0x%x - 0x%x (Android: 2228224 bytes at 0x6e0000)\n", ram_base, ram_base + ram_size);
	top = ram_base + ram_size - 4;
	mem_wr(top, 0);
	printf("wifi: 2: the top word 0x%08x: 0x%08x; address 0: 0x%08x\n", top, mem_rd(top), mem_rd(0));

	if (!announce(3, "writes: the firmware, the calibration with its token, random bytes, the reset vector")) {
		return;
	}
	fw = hakios_image_file("fw", FW_NAME, &fw_size, NULL);
	nv = hakios_image_file("fw", NVRAM_NAME, &nv_size, NULL);
	if (fw == NULL || nv == NULL) {
		printf("wifi: 3: fw/%s or fw/%s not in the small image (fw-wifi/ of the build): stopped\n", FW_NAME,
		    NVRAM_NAME);
		return;
	}
	if (fw_size < 64 || fw_size + nv_size + 0x10000 > ram_size) {
		printf("wifi: 3: fw/%s (%llu bytes) and fw/%s (%llu) do not fit a memory of %u bytes: stopped\n", FW_NAME,
		    fw_size, NVRAM_NAME, nv_size, ram_size);
		return;
	}
	{
		uint64_t t0 = mach_absolute_time();
		uint32_t bad;

		mem_write(ram_base, fw, (uint32_t)fw_size);
		printf("wifi: 3: fw/%s: %llu bytes to 0x%x in %llu us\n", FW_NAME, fw_size, ram_base, micros_since(t0));
		t0 = mach_absolute_time();
		bad = mem_compare(ram_base, fw, (uint32_t)fw_size, "firmware");
		printf("wifi: 3: read back in %llu us: %u words differ\n", micros_since(t0), bad);
		if (bad) {
			printf("wifi: 3: stopped\n");
			return;
		}
	}
	vars_len = nvram_vars(nv, nv_size, &vars, &vars_alloc);
	if (vars_len == 0 || vars_len > 0x10000) {
		printf("wifi: 3: the calibration came to %u bytes: stopped\n", vars_len);
		return;
	}
	{
		uint32_t varaddr = top - vars_len, words = vars_len / 4, rnd_addr, bad;
		uint8_t rnd[RNG_BYTES];

		mem_write(varaddr, vars, vars_len);
		bad = mem_compare(varaddr, vars, vars_len, "calibration");
		token = (~words << 16) | (words & 0xffff);
		mem_wr(top, token);
		printf("wifi: 3: fw/%s: %llu bytes of text, %u as the firmware wants them, at 0x%x; token 0x%08x (read back "
		    "0x%08x)%s (Android: 11716, 0xf48e0b71)\n", NVRAM_NAME, nv_size, vars_len, varaddr, token, mem_rd(top),
		    bad ? " -- DIFFERS" : "");
		read_random(rnd, sizeof(rnd));
		rnd_addr = varaddr - 8 - RNG_BYTES;
		mem_write(rnd_addr, rnd, RNG_BYTES);
		mem_wr(rnd_addr + RNG_BYTES, RNG_BYTES);        /* (this first TLV's length has no checksum) */
		mem_wr(rnd_addr + RNG_BYTES + 4, RNG_TLV_TYPE);
		printf("wifi: 3: %u random bytes at 0x%x, length 0x%x, type 0x%08x\n", RNG_BYTES, rnd_addr,
		    mem_rd(rnd_addr + RNG_BYTES), mem_rd(rnd_addr + RNG_BYTES + 4));
		bzero(rnd, sizeof(rnd));
	}
	{
		uint32_t vector, before = mem_rd(0);

		memcpy(&vector, fw, 4);
		if (before == 0xffffffffu) {
			printf("wifi: 3: address 0 reads 0xffffffff: the reset vector is locked (secure boot?): stopped\n");
			return;
		}
		mem_wr(0, vector);
		printf("wifi: 3: reset vector 0x%08x at address 0 (was 0x%08x, reads 0x%08x)\n", vector, before, mem_rd(0));
	}
	kfree_data(vars, vars_alloc);

	if (!announce(4, "runs: the ARM let go; the firmware's shared area awaited")) {
		return;
	}
	{
		uint64_t t0 = mach_absolute_time(), us = 0;
		uint32_t v = token;

		core_reset(arm, 0, 0);
		while ((us = micros_since(t0)) < 5000000) {
			v = mem_rd(top);
			if (v != token) {
				break;
			}
			sleep_ms(10);
		}
		core_say(arm, "running");
		if (v == token) {
			printf("wifi: 4: the top word still the token after 5 s: the firmware did not come up\n");
			return;
		}
		printf("wifi: 4: the top word 0x%08x after %llu ms\n", v, us / 1000);
		if (v < ram_base || v >= ram_base + ram_size) {
			printf("wifi: 4: not an address in the chip's memory: stopped\n");
			return;
		}
		shared = v;
	}
	{
		uint32_t flags = mem_rd(shared + SHARED_FLAGS);

		printf("wifi: 4: pcie_shared at 0x%08x: flags 0x%08x (protocol version %u%s%s), max rx buffers %u, rx data "
		    "offset %u, mailboxes 0x%08x / 0x%08x, ring info 0x%08x\n", shared, flags, flags & 0xff,
		    (flags & 0x10000) ? ", DMA indices" : "", (flags & 0x100000) ? " of 2 bytes" : "",
		    *(volatile uint16_t *)mem_at(shared + SHARED_MAX_RXBUFPOST), mem_rd(shared + SHARED_RX_DATAOFFSET),
		    mem_rd(shared + SHARED_H2D_MB_ADDR), mem_rd(shared + SHARED_D2H_MB_ADDR),
		    mem_rd(shared + SHARED_RING_INFO_ADDR));
	}
	sleep_ms(500);
	console_say(shared);
	printf("wifi: 4: the BCM4383's firmware runs\n");
}
