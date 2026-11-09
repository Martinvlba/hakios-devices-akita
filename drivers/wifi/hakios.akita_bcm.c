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
 *   5  writes  the message rings (msgbuf, protocol 7) in host memory -- the five common rings (H2D control and rx
 *              post, D2H control, tx and rx completions), the index arrays the chip DMAs its read and write indices
 *              to and from (the "DMA indices" its flags offer), brcmfmac's scratch and ring-update buffers --, all
 *              described in the chip's ring table; "host ready" (doorbell 1); 8 ioctl-answer and 8 event buffers
 *              posted on the control ring
 *   6  talks   the first ioctls: "ver" (the firmware's version, the line Android logs), "cur_etheraddr" (only its
 *              first and last bytes in the log); then the regulatory database (fw/bcmdhd_clm.blob_4383_a3, "clmload")
 *              and the transmit caps (fw/bcmdhd_txcap.blob_4383_a3, "txcapload") in chunks of 1400 bytes; events
 *              the firmware sends meanwhile are said by number
 *   7  talks   the events this driver wants ("event_msgs"), power saving off ("mpc" 0), infrastructure mode, the
 *              radio up (WLC_UP), an escan of every channel (scan parameters version 2 if "scan_ver" says so), the
 *              access points it heard: name, band, channel, signal, protection (BSSIDs shortened in the log)
 *   8  talks   the network of fw/network (a name and a passphrase, the tester's; only their lengths in the log)
 *              joined as WPA2-PSK with AES, the firmware's own supplicant ("sup_wpa", WLC_SET_WSEC_PMK with the
 *              passphrase), WLC_SET_SSID; the events awaited: set SSID, link, the supplicant's "keyed"
 * The chip is left running, polled (no interrupts yet); the chip's deep-sleep requests are acked as brcmfmac does.
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
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_unmappable(vm_offset_t phys_addr, vm_size_t size, unsigned int flags);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
extern uint64_t mach_absolute_time(void);
extern uint64_t kvtophys(vm_offset_t va);
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

/* the firmware's console: what it has written since the last look (its buffer is a ring) */
static uint32_t console_read;

static void
console_say(uint32_t shared)
{
	static uint32_t console, buf, size;
	uint32_t widx;

	if (console == 0) {
		console = mem_rd(shared + SHARED_CONSOLE_ADDR);
		if (console < ram_base || console >= ram_base + ram_size) {
			printf("wifi: %u: no console (0x%08x)\n", step, console);
			console = 0;
			return;
		}
		buf = mem_rd(console + CONSOLE_BUF_ADDR);
		size = mem_rd(console + CONSOLE_BUF_SIZE);
		printf("wifi: %u: the firmware's console at 0x%08x: buffer 0x%08x, %u bytes\n", step, console, buf, size);
		if (buf < ram_base || size == 0 || size > 0x10000 || buf + size > ram_base + ram_size) {
			console = 0;
			return;
		}
	}
	widx = mem_rd(console + CONSOLE_WRITE_IDX);
	if (widx >= size) {
		return;
	}
	char line[160];
	unsigned n = 0;

	while (console_read != widx) {
		uint32_t i = console_read;
		uint32_t w = mem_rd(buf + (i & ~3u));
		char c = (char)(w >> (8 * (i & 3)));

		console_read = (i + 1) % size;
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

/* ---- the message rings (msgbuf, protocol 7: brcmfmac's pcie.c and msgbuf.c, Google's bcmpcie.h) ---- */

#define CFG_SBMBX               0x98            /* host -> chip mailbox interrupt (configuration space) */
#define PCIE_H2D_MAILBOX_0      0xa20           /* the doorbell: "a ring has new items" (core rev >= 64) */
#define PCIE_H2D_MAILBOX_1      0xa24           /* "the host is ready" */
#define PCIE_MAILBOX_INT        0xc30
#define PCIE_MAILBOX_MASK       0xc34
#define SHARED_FLAG_HOSTRDY     0x10000000u
#define SHARED_DMA_SCRATCH_LEN  52
#define SHARED_DMA_SCRATCH_ADDR 56
#define SHARED_DMA_RINGUPD_LEN  64
#define SHARED_DMA_RINGUPD_ADDR 68
#define D2H_MB_DS_ENTER_REQ     0x2u
#define D2H_MB_DS_EXIT_NOTE     0x4u
#define D2H_MB_FWHALT           0x10000000u
#define H2D_MB_DS_ACK           0x2u

#define MSG_GEN_STATUS          0x1
#define MSG_RING_STATUS         0x2
#define MSG_IOCTLPTR_REQ        0x9
#define MSG_IOCTLPTR_REQ_ACK    0xa
#define MSG_IOCTLRESP_BUF_POST  0xb
#define MSG_IOCTL_CMPLT         0xc
#define MSG_EVENT_BUF_POST      0xd
#define MSG_WL_EVENT            0xe

#define WLC_GET_VAR             262
#define WLC_SET_VAR             263
#define IOCTL_REQ_ID            0xfffeu
#define IOCTL_MAX               1518            /* what one request may carry (an Ethernet frame) */

#define RING_H2D_CTRL           0               /* the five common rings, in this order in the chip's table */
#define RING_H2D_RXPOST         1
#define RING_D2H_CTRL           2
#define RING_D2H_TXCPL          3
#define RING_D2H_RXCPL          4
#define RINGS                   5
#define CTRL_BUFS               8               /* ioctl response buffers posted, and as many for events */
#define CTRL_BUF_SIZE           8192

static const uint16_t ring_items[RINGS] = { 64, 1024, 64, 1024, 1024 };
static const uint16_t ring_item_size[RINGS] = { 40, 32, 24, 24, 40 };

static struct {
	vm_offset_t va;                 /* the uncached view */
	uint64_t pa;
	uint32_t size, used;
} dma;

static struct {
	uint8_t *ring[RINGS];
	uint64_t ring_pa[RINGS];
	uint16_t w[RINGS], r[RINGS];    /* ours: H2D write, D2H read */
	volatile uint32_t *idx;         /* the index arrays the chip DMAs to and from (4 bytes each) */
	unsigned h2d_w, h2d_r, d2h_w, d2h_r;    /* where each array starts in idx[] */
	uint8_t *ioctl_req;
	uint64_t ioctl_req_pa;
	uint8_t *ctrl_buf[2 * CTRL_BUFS];       /* request ids 1..8 ioctl responses, 9..16 events */
	uint64_t ctrl_buf_pa[2 * CTRL_BUFS];
	uint16_t trans_id;
	bool ioctl_done;
	int16_t ioctl_status;
	uint16_t ioctl_len;
	uint32_t ioctl_buf;             /* the request id of the buffer holding the answer */
	uint32_t shared, h2d_mb, d2h_mb, rx_dataoffset;
	uint32_t tcm_h2d_r;             /* the chip's own array of H2D read indices (4 bytes apart) */
	unsigned events;
} mb;

/* how far the chip has read an H2D ring: from the array it DMAs to us, or (if that has not moved) its own */
static uint16_t
h2d_read_index(unsigned ring)
{
	uint16_t r = (uint16_t)mb.idx[mb.h2d_r + ring];

	if (r == mb.w[ring] || mb.tcm_h2d_r == 0) {
		return r;
	}
	return (uint16_t)mem_rd(mb.tcm_h2d_r + 4 * ring);
}

/* one physically contiguous piece, its cache flushed and seen through an uncached mapping from then on */
static bool
dma_init(uint32_t size)
{
	vm_offset_t cached = 0;

	size = (uint32_t)round_page(size);
	if (kmem_alloc_contig(kernel_map, &cached, size, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return false;
	}
	bzero((void *)cached, size);
	for (vm_offset_t line = cached; line < cached + size; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	dma.pa = kvtophys(cached);
	dma.va = ml_io_map_unmappable(dma.pa, size, VM_WIMG_WCOMB);
	dma.size = size;
	dma.used = 0;
	return dma.va != 0;
}

static void *
dma_take(uint32_t size, uint64_t *pa)
{
	uint32_t at = (dma.used + 4095) & ~4095u;

	if (at + size > dma.size) {
		return NULL;
	}
	dma.used = at + size;
	*pa = dma.pa + at;
	return (void *)(dma.va + at);
}

static inline void
put16(uint8_t *p, uint32_t off, uint16_t v)
{
	*(volatile uint16_t *)(p + off) = v;
}

static inline void
put32(uint8_t *p, uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(p + off) = v;
}

static inline uint16_t
get16(const uint8_t *p, uint32_t off)
{
	return *(const volatile uint16_t *)(p + off);
}

static inline uint32_t
get32(const uint8_t *p, uint32_t off)
{
	return *(const volatile uint32_t *)(p + off);
}

static void
mb_data_send(uint32_t value)
{
	for (unsigned i = 0; i < 100 && mem_rd(mb.h2d_mb) != 0; i++) {
		sleep_ms(10);
	}
	mem_wr(mb.h2d_mb, value);
	wr(cfg, CFG_SBMBX, 1);
}

/* the chip's mailbox word: deep sleep requests answered (as brcmfmac), a halt said */
static void
mb_data_check(void)
{
	uint32_t d = mem_rd(mb.d2h_mb);

	if (d == 0) {
		return;
	}
	mem_wr(mb.d2h_mb, 0);
	printf("wifi: %u: the chip's mailbox: 0x%08x%s%s%s\n", step, d, (d & D2H_MB_DS_ENTER_REQ) ? " (deep sleep asked: acked)" :
	    "", (d & D2H_MB_DS_EXIT_NOTE) ? " (deep sleep over)" : "", (d & D2H_MB_FWHALT) ? " (THE FIRMWARE HALTED)" : "");
	if (d & D2H_MB_DS_ENTER_REQ) {
		mb_data_send(H2D_MB_DS_ACK);
	}
}

/* room for one item on an H2D ring, or NULL */
static uint8_t *
h2d_reserve(unsigned ring)
{
	unsigned h = ring;              /* the H2D rings' index in the arrays */
	uint16_t next = (uint16_t)((mb.w[ring] + 1) % ring_items[ring]);

	if (next == (uint16_t)mb.idx[mb.h2d_r + h] && next == h2d_read_index(ring)) {
		return NULL;
	}
	uint8_t *item = mb.ring[ring] + (uint32_t)mb.w[ring] * ring_item_size[ring];

	bzero(item, ring_item_size[ring]);
	return item;
}

static void
h2d_commit(unsigned ring, unsigned count)
{
	mb.w[ring] = (uint16_t)((mb.w[ring] + count) % ring_items[ring]);
	mb.idx[mb.h2d_w + ring] = mb.w[ring];
	__asm__ volatile ("dsb sy" ::: "memory");
	pcie_wr(PCIE_H2D_MAILBOX_0, 1);
}

/* ---- events ---- */

#define EV_SET_SSID             0
#define EV_AUTH                 3
#define EV_DEAUTH               5
#define EV_DEAUTH_IND           6
#define EV_ASSOC                7
#define EV_DISASSOC_IND         12
#define EV_LINK                 16
#define EV_PSK_SUP              46
#define EV_COUNTRY_CODE_CHANGED 47
#define EV_IF                   54
#define EV_ESCAN_RESULT         69
#define EV_STATUS_PARTIAL       8
#define EVENT_HDR               (14 + 10)       /* Ethernet, then Broadcom's header */
#define EVENT_MSG               48              /* the event itself, big-endian */

static inline uint32_t
be32(const uint8_t *p)
{
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}

#define HEARD_MOST              64
static struct heard {
	uint8_t bssid[6], ssid[33], ssid_len;
	uint16_t chanspec, capability;
	int16_t rssi;
} heard[HEARD_MOST];
static unsigned heard_count;
static bool escan_done;
static int escan_status = -1;
static int join_status = -1;            /* SET_SSID's status (0: joined), -1: not yet */
static bool link_up, keyed;
static int psk_status = -1;

static void
escan_result(const uint8_t *d, uint32_t len)
{
	if (len < 12) {
		return;
	}
	uint32_t buflen = get32(d, 0), at = 12;
	uint16_t count = get16(d, 10);

	if (buflen > len) {
		buflen = len;
	}
	for (unsigned n = 0; n < count && at + 90 <= buflen; n++) {
		const uint8_t *b = d + at;
		uint32_t blen = get32(b, 4);
		struct heard h;

		if (blen < 90 || at + blen > buflen) {
			break;
		}
		bzero(&h, sizeof(h));
		memcpy(h.bssid, b + 8, 6);
		h.capability = get16(b, 16);
		h.ssid_len = b[18] > 32 ? 32 : b[18];
		memcpy(h.ssid, b + 19, h.ssid_len);
		h.chanspec = get16(b, 72);
		h.rssi = (int16_t)get16(b, 78);
		unsigned i;

		for (i = 0; i < heard_count; i++) {
			if (memcmp(heard[i].bssid, h.bssid, 6) == 0) {
				if (h.rssi > heard[i].rssi) {
					heard[i] = h;
				}
				break;
			}
		}
		if (i == heard_count && heard_count < HEARD_MOST) {
			heard[heard_count++] = h;
		}
		at += (blen + 3) & ~3u;
	}
}

static void
event_handle(const uint8_t *frame, uint32_t length)
{
	if (length < EVENT_HDR + EVENT_MSG) {
		printf("wifi: %u: an event of %u bytes\n", step, length);
		return;
	}
	const uint8_t *e = frame + EVENT_HDR;
	uint32_t type = be32(e + 4), status = be32(e + 8), reason = be32(e + 12), datalen = be32(e + 20);
	const uint8_t *data = e + EVENT_MSG;

	if (datalen > length - EVENT_HDR - EVENT_MSG) {
		datalen = length - EVENT_HDR - EVENT_MSG;
	}
	if (type == EV_ESCAN_RESULT) {
		escan_result(data, datalen);
		if (status != EV_STATUS_PARTIAL) {
			escan_done = true;
			escan_status = (int)status;
		}
		return;
	}
	uint16_t flags = (uint16_t)(e[2] << 8 | e[3]);
	const char *name = "";

	switch (type) {
	case EV_SET_SSID:
		name = " (set SSID: 0 joined, 1 failed, 2 no network, 3 timeout)";
		join_status = (int)status;
		break;
	case EV_AUTH: name = " (authentication)"; break;
	case EV_ASSOC: name = " (association)"; break;
	case EV_DEAUTH: case EV_DEAUTH_IND: name = " (deauthenticated)"; link_up = keyed = false; break;
	case EV_DISASSOC_IND: name = " (disassociated)"; link_up = keyed = false; break;
	case EV_LINK:
		link_up = (flags & 1) != 0;
		name = link_up ? " (link up)" : " (link down)";
		if (!link_up) {
			keyed = false;
		}
		break;
	case EV_PSK_SUP:
		psk_status = (int)status;
		keyed = status == 6;
		name = status == 6 ? " (supplicant: keys set)" : " (supplicant: 4 authenticating, 5 key exchange, 6 keyed, 7 timeout)";
		break;
	case EV_COUNTRY_CODE_CHANGED: name = " (country code changed)"; break;
	case EV_IF: name = " (interface)"; break;
	}
	printf("wifi: %u: event %u%s, status %u, reason %u, flags 0x%x, %u bytes\n", step, type, name, status, reason, flags,
	    datalen);
}

static void
ctrl_buf_post(uint32_t id)
{
	uint8_t *item = h2d_reserve(RING_H2D_CTRL);

	if (item == NULL) {
		printf("wifi: %u: the control ring is full\n", step);
		return;
	}
	item[0] = id <= CTRL_BUFS ? MSG_IOCTLRESP_BUF_POST : MSG_EVENT_BUF_POST;
	put32(item, 4, id);
	put16(item, 8, CTRL_BUF_SIZE);
	put32(item, 16, (uint32_t)mb.ctrl_buf_pa[id - 1]);
	put32(item, 20, (uint32_t)(mb.ctrl_buf_pa[id - 1] >> 32));
	h2d_commit(RING_H2D_CTRL, 1);
}

/* what the chip has put on its control completion ring */
static unsigned
d2h_ctrl_poll(void)
{
	unsigned n = 0;
	uint16_t w = (uint16_t)mb.idx[mb.d2h_w + (RING_D2H_CTRL - 2)];

	if (w >= ring_items[RING_D2H_CTRL]) {
		return 0;
	}
	while (mb.r[RING_D2H_CTRL] != w) {
		const uint8_t *m = mb.ring[RING_D2H_CTRL] + (uint32_t)mb.r[RING_D2H_CTRL] * ring_item_size[RING_D2H_CTRL];
		uint8_t type = m[0];
		uint32_t id = get32(m, 4);

		switch (type) {
		case MSG_IOCTLPTR_REQ_ACK:
			if ((int16_t)get16(m, 8) != 0) {
				printf("wifi: %u: ioctl not taken: status %d\n", step, (int16_t)get16(m, 8));
			}
			break;
		case MSG_IOCTL_CMPLT:
			mb.ioctl_status = (int16_t)get16(m, 8);
			mb.ioctl_len = get16(m, 12);
			mb.ioctl_buf = id;
			mb.ioctl_done = true;
			break;
		case MSG_WL_EVENT:
			if (id > CTRL_BUFS && id <= 2 * CTRL_BUFS) {
				event_handle(mb.ctrl_buf[id - 1] + mb.rx_dataoffset, get16(m, 12));
				mb.events++;
				ctrl_buf_post(id);
			}
			break;
		case MSG_GEN_STATUS:
		case MSG_RING_STATUS:
			printf("wifi: %u: %s status %d (ring %u)\n", step, type == MSG_GEN_STATUS ? "general" : "ring",
			    (int16_t)get16(m, 8), get16(m, 10));
			break;
		default:
			printf("wifi: %u: message type 0x%x on the control completion ring\n", step, type);
		}
		mb.r[RING_D2H_CTRL] = (uint16_t)((mb.r[RING_D2H_CTRL] + 1) % ring_items[RING_D2H_CTRL]);
		mb.idx[mb.d2h_r + (RING_D2H_CTRL - 2)] = mb.r[RING_D2H_CTRL];
		n++;
	}
	mb_data_check();
	return n;
}

/*
 * One ioctl: `cmd`, `in` (of `in_len`, the input) and room for `out_len` in the answer; the answer is copied into
 * `out` (up to out_len). Returns the firmware's status (0: done), or -1000 for no answer within 2 s.
 */
static int
bcm_ioctl(uint32_t cmd, const void *in, uint32_t in_len, void *out, uint32_t out_len)
{
	uint8_t *item;
	uint32_t len = in_len > out_len ? in_len : out_len;

	if (len > IOCTL_MAX) {
		len = IOCTL_MAX;
	}
	if ((item = h2d_reserve(RING_H2D_CTRL)) == NULL) {
		return -1001;
	}
	bzero(mb.ioctl_req, IOCTL_MAX);
	memcpy(mb.ioctl_req, in, in_len > IOCTL_MAX ? IOCTL_MAX : in_len);
	mb.trans_id++;
	mb.ioctl_done = false;
	item[0] = MSG_IOCTLPTR_REQ;
	put32(item, 4, IOCTL_REQ_ID);
	put32(item, 8, cmd);
	put16(item, 12, mb.trans_id);
	put16(item, 14, (uint16_t)len);
	/* the answer's room: never less than the request (the firmware answers in place: brcmfmac sends the longer) */
	put16(item, 16, (uint16_t)(out_len > len ? (out_len > CTRL_BUF_SIZE ? CTRL_BUF_SIZE : out_len) : len));
	put32(item, 24, (uint32_t)mb.ioctl_req_pa);
	put32(item, 28, (uint32_t)(mb.ioctl_req_pa >> 32));
	h2d_commit(RING_H2D_CTRL, 1);

	uint64_t t0 = mach_absolute_time();

	while (!mb.ioctl_done && micros_since(t0) < 2000000) {
		if (d2h_ctrl_poll() == 0) {
			delay(200);
		}
	}
	if (!mb.ioctl_done) {
		return -1000;
	}
	if (mb.ioctl_buf >= 1 && mb.ioctl_buf <= CTRL_BUFS) {
		if (out && out_len) {
			uint32_t n = mb.ioctl_len < out_len ? mb.ioctl_len : out_len;

			memcpy(out, mb.ctrl_buf[mb.ioctl_buf - 1] + mb.rx_dataoffset, n);
		}
		ctrl_buf_post(mb.ioctl_buf);
	}
	return mb.ioctl_status;
}

/* an iovar: its name, a NUL, the data */
static int
iovar_get(const char *name, void *out, uint32_t out_len)
{
	uint8_t req[IOCTL_MAX];
	uint32_t n = (uint32_t)strlen(name) + 1;

	bzero(req, sizeof(req));
	memcpy(req, name, n);
	return bcm_ioctl(WLC_GET_VAR, req, n > out_len ? n : out_len, out, out_len);
}

static int
iovar_set(const char *name, const void *data, uint32_t len)
{
	uint8_t req[IOCTL_MAX];
	uint32_t n = (uint32_t)strlen(name) + 1;

	if (n + len > sizeof(req)) {
		return -1002;
	}
	memcpy(req, name, n);
	memcpy(req + n, data, len);
	return bcm_ioctl(WLC_SET_VAR, req, n + len, NULL, n + len);
}

/* a blob (the CLM's regulatory data, the transmit caps) in chunks of 1400 bytes, as brcmfmac's brcmf_c_download_blob */
static int
blob_load(const char *file, const char *var, const char *status_var)
{
	uint64_t size = 0;
	const uint8_t *data = hakios_image_file("fw", file, &size, NULL);
	uint8_t chunk[12 + 1400];
	uint32_t done = 0, status = 0xffffffffu;
	int err = 0;

	if (data == NULL || size == 0) {
		printf("wifi: %u: no fw/%s\n", step, file);
		return -1;
	}
	while (done < size && err == 0) {
		uint32_t n = size - done > 1400 ? 1400 : (uint32_t)(size - done);
		uint16_t flag = (uint16_t)(1u << 12);   /* the downloader's version, 1 */

		if (done == 0) {
			flag |= 0x2;            /* begin */
		}
		if (done + n == size) {
			flag |= 0x4;            /* end */
		}
		memcpy(chunk, &flag, 2);
		uint16_t type = 2;      /* (the CLM's type; the transmit caps are loaded the same way) */
		uint32_t len32 = n, crc = 0;

		memcpy(chunk + 2, &type, 2);
		memcpy(chunk + 4, &len32, 4);
		memcpy(chunk + 8, &crc, 4);
		memcpy(chunk + 12, data + done, n);
		err = iovar_set(var, chunk, 12 + n);
		done += n;
	}
	(void)iovar_get(status_var, &status, sizeof(status));
	printf("wifi: %u: fw/%s: %llu bytes by \"%s\": %d, \"%s\" %u%s\n", step, file, size, var, err, status_var, status,
	    err == 0 && status == 0 ? " (loaded)" : "");
	return err != 0 ? err : (int)status;
}

/* step 5: the rings set up, the chip told the host is ready, buffers posted for answers and events */
static bool
rings_up(uint32_t shared)
{
	uint32_t ringinfo = mem_rd(shared + SHARED_RING_INFO_ADDR), ringmem, max_sub, max_cpl, flags;
	uint64_t pa;

	mb.shared = shared;
	mb.h2d_mb = mem_rd(shared + SHARED_H2D_MB_ADDR);
	mb.d2h_mb = mem_rd(shared + SHARED_D2H_MB_ADDR);
	mb.rx_dataoffset = mem_rd(shared + SHARED_RX_DATAOFFSET);
	flags = mem_rd(shared + SHARED_FLAGS);
	ringmem = mem_rd(ringinfo + 0);
	mb.tcm_h2d_r = mem_rd(ringinfo + 8);
	max_sub = mem_rd(ringinfo + 52) >> 16;          /* (max_tx_flowrings at +52, max_submission_queues +54) */
	max_cpl = mem_rd(ringinfo + 56) & 0xffff;
	printf("wifi: 5: ring info 0x%08x: ring memory 0x%08x, %u flow rings, %u submission, %u completion rings, %u "
	    "virtual interfaces; TCM index arrays 0x%08x 0x%08x 0x%08x 0x%08x\n", ringinfo, ringmem, mem_rd(ringinfo + 52) & 0xffff,
	    max_sub, max_cpl, mem_rd(ringinfo + 56) >> 16, mem_rd(ringinfo + 4), mem_rd(ringinfo + 8), mem_rd(ringinfo + 12),
	    mem_rd(ringinfo + 16));
	if (max_sub < 2 || max_sub > 600 || max_cpl < 3 || max_cpl > 64 || ringmem < ram_base || ringmem >= ram_base + ram_size) {
		printf("wifi: 5: not what a ring table looks like: stopped\n");
		return false;
	}
	if (!dma_init(512 * 1024)) {
		printf("wifi: 5: no memory in one piece for the rings: stopped\n");
		return false;
	}

	/* the index arrays: H2D write, H2D read, D2H write, D2H read, 4 bytes an entry */
	mb.idx = dma_take((2 * max_sub + 2 * max_cpl) * 4, &pa);
	mb.h2d_w = 0;
	mb.h2d_r = max_sub;
	mb.d2h_w = 2 * max_sub;
	mb.d2h_r = 2 * max_sub + max_cpl;
	mem_wr(ringinfo + 20, (uint32_t)pa);
	mem_wr(ringinfo + 24, (uint32_t)(pa >> 32));
	mem_wr(ringinfo + 28, (uint32_t)(pa + max_sub * 4));
	mem_wr(ringinfo + 32, (uint32_t)((pa + max_sub * 4) >> 32));
	mem_wr(ringinfo + 36, (uint32_t)(pa + 2 * max_sub * 4));
	mem_wr(ringinfo + 40, (uint32_t)((pa + 2 * max_sub * 4) >> 32));
	mem_wr(ringinfo + 44, (uint32_t)(pa + (2 * max_sub + max_cpl) * 4));
	mem_wr(ringinfo + 48, (uint32_t)((pa + (2 * max_sub + max_cpl) * 4) >> 32));

	/* the five common rings, each described in the chip's ring memory (16 bytes a ring) */
	for (unsigned i = 0; i < RINGS; i++) {
		mb.ring[i] = dma_take((uint32_t)ring_items[i] * ring_item_size[i], &mb.ring_pa[i]);
		if (mb.ring[i] == NULL) {
			printf("wifi: 5: no room for ring %u: stopped\n", i);
			return false;
		}
		uint32_t d = ringmem + 16 * i;

		mem_wr(d + 4, (uint32_t)ring_items[i] | (uint32_t)ring_item_size[i] << 16);
		mem_wr(d + 8, (uint32_t)mb.ring_pa[i]);
		mem_wr(d + 12, (uint32_t)(mb.ring_pa[i] >> 32));
		mb.w[i] = mb.r[i] = 0;
	}

	/* brcmfmac's scratch and ring-update buffers */
	{
		void *scratch = dma_take(8, &pa);

		(void)scratch;
		mem_wr(shared + SHARED_DMA_SCRATCH_ADDR, (uint32_t)pa);
		mem_wr(shared + SHARED_DMA_SCRATCH_ADDR + 4, (uint32_t)(pa >> 32));
		mem_wr(shared + SHARED_DMA_SCRATCH_LEN, 8);
		void *ringupd = dma_take(1024, &pa);

		(void)ringupd;
		mem_wr(shared + SHARED_DMA_RINGUPD_ADDR, (uint32_t)pa);
		mem_wr(shared + SHARED_DMA_RINGUPD_ADDR + 4, (uint32_t)(pa >> 32));
		mem_wr(shared + SHARED_DMA_RINGUPD_LEN, 1024);
	}
	mb.ioctl_req = dma_take(IOCTL_MAX + 2, &mb.ioctl_req_pa);
	for (unsigned i = 0; i < 2 * CTRL_BUFS; i++) {
		mb.ctrl_buf[i] = dma_take(CTRL_BUF_SIZE, &mb.ctrl_buf_pa[i]);
		if (mb.ctrl_buf[i] == NULL) {
			printf("wifi: 5: no room for the control buffers: stopped\n");
			return false;
		}
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	printf("wifi: 5: rings in host memory at 0x%llx (%u KB used): index arrays %u+%u+%u+%u, control %u x %u, rx post %u "
	    "x %u, completions %u x %u / %u x %u / %u x %u\n", dma.pa, dma.used >> 10, max_sub, max_sub, max_cpl, max_cpl,
	    ring_items[0], ring_item_size[0], ring_items[1], ring_item_size[1], ring_items[2], ring_item_size[2], ring_items[3],
	    ring_item_size[3], ring_items[4], ring_item_size[4]);

	pcie_wr(PCIE_MAILBOX_INT, pcie_rd(PCIE_MAILBOX_INT));
	if (flags & SHARED_FLAG_HOSTRDY) {
		pcie_wr(PCIE_H2D_MAILBOX_1, 1);
		printf("wifi: 5: host ready (doorbell 1)\n");
	}
	for (uint32_t id = 1; id <= 2 * CTRL_BUFS; id++) {
		ctrl_buf_post(id);
	}
	sleep_ms(50);
	(void)d2h_ctrl_poll();
	printf("wifi: 5: %u ioctl and %u event buffers posted; the chip has read the control ring to %u (of %u written); "
	    "mailbox interrupts 0x%08x\n", CTRL_BUFS, CTRL_BUFS, mb.idx[mb.h2d_r + RING_H2D_CTRL], mb.w[RING_H2D_CTRL],
	    pcie_rd(PCIE_MAILBOX_INT));
	return true;
}

static int
cmd_int(uint32_t cmd, uint32_t value)
{
	return bcm_ioctl(cmd, &value, 4, NULL, 4);
}

static int
iovar_int(const char *name, uint32_t value)
{
	return iovar_set(name, &value, 4);
}

static const char *
band_of(uint16_t chanspec)
{
	switch (chanspec & 0xc000) {
	case 0x0000: return "2.4";
	case 0x4000: return "4.9";
	case 0x8000: return "6";
	default: return "5";
	}
}

/* step 7: the events this driver wants, the radio up, an escan of every channel, what it heard */
static void
scan_all(void)
{
	uint8_t mask[16];
	uint32_t scan_ver = 0;
	int err;
	static const uint8_t wanted[] = { EV_SET_SSID, EV_AUTH, EV_DEAUTH, EV_DEAUTH_IND, EV_ASSOC, EV_DISASSOC_IND, EV_LINK,
		                          EV_PSK_SUP, EV_COUNTRY_CODE_CHANGED, EV_IF, EV_ESCAN_RESULT };

	bzero(mask, sizeof(mask));
	err = iovar_get("event_msgs", mask, sizeof(mask));
	for (unsigned i = 0; i < sizeof(wanted); i++) {
		mask[wanted[i] / 8] |= (uint8_t)(1u << (wanted[i] % 8));
	}
	printf("wifi: 7: \"event_msgs\" read: %d; set: %d\n", err, iovar_set("event_msgs", mask, sizeof(mask)));
	printf("wifi: 7: \"mpc\" 0: %d; infrastructure: %d; up: %d\n", iovar_int("mpc", 0), cmd_int(20, 1), cmd_int(2, 1));
	err = iovar_get("scan_ver", &scan_ver, sizeof(scan_ver));
	printf("wifi: 7: \"scan_ver\": %d, %u\n", err, scan_ver);

	/* the escan's parameters: version, action (1: start), sync id; then the scan's (version 2 if the firmware says so) */
	uint8_t p[8 + 72];
	uint32_t size;

	bzero(p, sizeof(p));
	put16(p, 4, 1);
	put16(p, 6, 0x1234);
	if (err == 0 && scan_ver >= 2) {
		uint8_t *v = p + 8;

		put32(p, 0, 2);
		put16(v, 0, 2);                 /* version */
		put16(v, 2, 72);                /* length */
		memset(v + 40, 0xff, 6);        /* any BSSID */
		v[46] = 2;                      /* any BSS type */
		put32(v, 48, 0);                /* active */
		put32(v, 52, 0xffffffffu);      /* the firmware's own probes and dwell times */
		put32(v, 56, 0xffffffffu);
		put32(v, 60, 0xffffffffu);
		put32(v, 64, 0xffffffffu);
		put32(v, 68, 0);                /* every channel */
		size = 8 + 72;
	} else {
		uint8_t *v = p + 8;             /* version 1: ssid 36, bssid 6, bss type, scan type, four times, channels */

		put32(p, 0, 1);
		memset(v + 36, 0xff, 6);
		v[42] = 2;
		v[43] = 0;
		put32(v, 44, 0xffffffffu);
		put32(v, 48, 0xffffffffu);
		put32(v, 52, 0xffffffffu);
		put32(v, 56, 0xffffffffu);
		put32(v, 60, 0);
		size = 8 + 64;
	}
	heard_count = 0;
	escan_done = false;
	escan_status = -1;
	err = iovar_set("escan", p, size);
	printf("wifi: 7: \"escan\" (version %u): %d\n", get32(p, 0), err);
	if (err != 0) {
		return;
	}
	uint64_t t0 = mach_absolute_time();

	while (!escan_done && micros_since(t0) < 15000000) {
		if (d2h_ctrl_poll() == 0) {
			sleep_ms(5);
		}
	}
	printf("wifi: 7: the scan %s after %llu ms (status %d): %u access points\n", escan_done ? "ended" : "did NOT end",
	    micros_since(t0) / 1000, escan_status, heard_count);
	for (unsigned i = 0; i < heard_count; i++) {
		struct heard *h = &heard[i];
		char name[33];

		for (unsigned j = 0; j < h->ssid_len; j++) {
			name[j] = (h->ssid[j] >= 0x20 && h->ssid[j] < 0x7f) ? (char)h->ssid[j] : '?';
		}
		name[h->ssid_len] = 0;
		printf("wifi: 7: %4d dBm  %3s GHz channel %3u  %s  xx:xx:xx:xx:%02x:%02x  \"%s\"\n", h->rssi,
		    band_of(h->chanspec), h->chanspec & 0xff, (h->capability & 0x10) ? "protected" : "open     ", h->bssid[4],
		    h->bssid[5], h->ssid_len ? name : "(hidden)");
	}
}

/*
 * The network to join: fw/network of the small image (fw-wifi/network of the build, the tester's, never in a
 * repository): its name on the first line, its passphrase on the second (as upstream's hakios_wifi_network). Only
 * their lengths go into the log.
 */
static bool
network_file(uint8_t *name, unsigned *name_length, uint8_t *passphrase, unsigned *passphrase_length)
{
	uint64_t size = 0;
	const uint8_t *file = hakios_image_file("fw", "network", &size, NULL), *end, *line;
	unsigned lengths[2] = { 0, 0 }, sizes[2] = { *name_length, *passphrase_length };
	uint8_t *where[2] = { name, passphrase };

	if (file == NULL) {
		printf("wifi: %u: no fw/network in the small image (fw-wifi/network: a name, a passphrase): nothing to join\n",
		    step);
		return false;
	}
	end = file + size;
	line = file;
	for (unsigned n = 0; n < 2 && line < end; n++) {
		const uint8_t *stop = line;

		while (stop < end && *stop != '\n') {
			stop++;
		}
		lengths[n] = (unsigned)(stop - line) - (stop > line && stop[-1] == '\r' ? 1 : 0);
		if (lengths[n] > sizes[n]) {
			lengths[n] = 0;
		} else {
			memcpy(where[n], line, lengths[n]);
		}
		line = stop + 1;
	}
	printf("wifi: %u: fw/network: a name of %u bytes and a passphrase of %u bytes%s\n", step, lengths[0], lengths[1],
	    lengths[0] == 0 || lengths[1] < 8 || lengths[1] > 63 ? ": NOT A NAME OF 1 TO 32 BYTES AND A PASSPHRASE OF 8 TO 63" :
	    "");
	if (lengths[0] == 0 || lengths[1] < 8 || lengths[1] > 63) {
		return false;
	}
	*name_length = lengths[0];
	*passphrase_length = lengths[1];
	return true;
}

/* step 8: WPA2-PSK (AES) with the firmware's supplicant: the passphrase handed over, the SSID set, the events awaited */
static void
join_network(void)
{
	uint8_t name[32], pass[64], pmk[2 + 2 + 256 + 2], params[52];
	unsigned name_len = sizeof(name), pass_len = sizeof(pass);
	int err;

	bzero(name, sizeof(name));
	bzero(pass, sizeof(pass));
	if (!network_file(name, &name_len, pass, &pass_len)) {
		return;
	}
	printf("wifi: 8: \"wpa_auth\" WPA2-PSK: %d; \"auth\" open: %d; \"wsec\" AES: %d; \"sup_wpa\" 1: %d\n",
	    iovar_int("wpa_auth", 0x80), iovar_int("auth", 0), iovar_int("wsec", 4), iovar_int("sup_wpa", 1));

	/* WLC_SET_WSEC_PMK: the passphrase itself (flag 1); this firmware's structure has room for 256 bytes and a TLV length */
	bzero(pmk, sizeof(pmk));
	put16(pmk, 0, (uint16_t)pass_len);
	put16(pmk, 2, 1);
	memcpy(pmk + 4, pass, pass_len);
	err = bcm_ioctl(268, pmk, sizeof(pmk), NULL, sizeof(pmk));
	bzero(pmk, sizeof(pmk));
	bzero(pass, sizeof(pass));
	bzero(mb.ioctl_req, IOCTL_MAX);
	printf("wifi: 8: the passphrase handed over: %d\n", err);

	/* WLC_SET_SSID: the name, any BSSID, every channel */
	bzero(params, sizeof(params));
	put32(params, 0, name_len);
	memcpy(params + 4, name, name_len);
	memset(params + 36, 0xff, 6);
	put32(params, 44, 0);
	join_status = -1;
	psk_status = -1;
	link_up = keyed = false;
	uint64_t t0 = mach_absolute_time();

	err = bcm_ioctl(26, params, sizeof(params), NULL, sizeof(params));
	printf("wifi: 8: join asked: %d\n", err);
	if (err != 0) {
		return;
	}
	while (!keyed && micros_since(t0) < 20000000 && (join_status <= 0)) {
		if (d2h_ctrl_poll() == 0) {
			sleep_ms(5);
		}
	}
	printf("wifi: 8: after %llu ms: join %d, link %s, supplicant %d: %s\n", micros_since(t0) / 1000, join_status,
	    link_up ? "up" : "down", psk_status, keyed ? "JOINED, the keys set" : "not joined");
	/* a while longer, to see whether the access point keeps us */
	for (unsigned i = 0; i < 100; i++) {
		(void)d2h_ctrl_poll();
		sleep_ms(50);
	}
	printf("wifi: 8: 5 s later: link %s, keys %s\n", link_up ? "up" : "down", keyed ? "set" : "not set");
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

	if (!announce(5, "writes: the message rings in host memory, the chip told the host is ready, buffers posted")) {
		return;
	}
	if (!rings_up(shared)) {
		console_say(shared);
		return;
	}
	console_say(shared);

	if (!announce(6, "talks: the first ioctls (\"ver\", \"cur_etheraddr\"), the regulatory and transmit-cap blobs")) {
		return;
	}
	{
		char ver[256];
		uint8_t mac[6];
		int err;

		bzero(ver, sizeof(ver));
		err = iovar_get("ver", ver, sizeof(ver) - 1);
		for (unsigned i = 0; i < sizeof(ver) && ver[i]; i++) {
			if (ver[i] == '\n') {
				ver[i] = 0;
			}
		}
		printf("wifi: 6: \"ver\": %d: %s\n", err, err == 0 ? ver : "(no answer)");
		bzero(mac, sizeof(mac));
		err = iovar_get("cur_etheraddr", mac, sizeof(mac));
		/* (the address is the phone's: only its first and last bytes go into the log, as Android's driver does) */
		printf("wifi: 6: \"cur_etheraddr\": %d: %02x:xx:xx:xx:xx:%02x\n", err, mac[0], mac[5]);
		console_say(shared);
		if (strncmp(ver, "wl0:", 4) == 0) {
			(void)blob_load("bcmdhd_clm.blob_4383_a3", "clmload", "clmload_status");
			(void)blob_load("bcmdhd_txcap.blob_4383_a3", "txcapload", "txcapload_status");
		}
		sleep_ms(200);
		(void)d2h_ctrl_poll();
		console_say(shared);
		printf("wifi: 6: control ring: written to %u; read to %u by the array in host memory, %u by the chip's own\n",
		    mb.w[RING_H2D_CTRL], mb.idx[mb.h2d_r + RING_H2D_CTRL], mem_rd(mb.tcm_h2d_r + 4 * RING_H2D_CTRL) & 0xffff);
		printf("wifi: 6: %u events so far; the firmware answers\n", mb.events);
	}

	if (!announce(7, "talks: the events wanted, the radio up, a scan of every channel")) {
		return;
	}
	scan_all();
	console_say(shared);

	if (!announce(8, "talks: the network of fw/network joined, WPA2-PSK by the firmware's own supplicant")) {
		return;
	}
	join_network();
	console_say(shared);
}
