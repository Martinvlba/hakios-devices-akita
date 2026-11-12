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
 *              posted on the control ring; 256 data buffers of 2 KB on the rx post ring (the firmware's SPLITRX puts
 *              received frames' payloads into host buffers), given back on the rx completion ring and posted again
 *   6  talks   the first ioctls: "ver" (the firmware's version, the line Android logs), "cur_etheraddr" (only its
 *              first and last bytes in the log); then the regulatory database (fw/bcmdhd_clm.blob_4383_a3, "clmload")
 *              and the transmit caps (fw/bcmdhd_txcap.blob_4383_a3, "txcapload") in chunks of 1400 bytes; events
 *              the firmware sends meanwhile are said by number
 *   7  talks   the events this driver wants ("event_msgs"), power saving off ("mpc" 0), infrastructure mode, the
 *              radio up (WLC_UP), an escan of every channel (scan parameters version 2 if "scan_ver" says so), the
 *              access points it heard: name, band, channel, signal, protection (BSSIDs shortened in the log)
 *   8  carries en0 (drivers/net/hakios.net.c) on the chip: frames received handed to the network stack, frames sent
 *              on a flow ring of our own (created with the first one); link up for the stack when associated and keyed.
 *              Then the station (sys/hakios.sta.h, drivers/wifi/hakios.sta.c): Darwin's Wi-Fi side -- wifid, Settings
 *              -- asks it for the radio's power, scans, joins and leaves; and the network of fw/network (a name and a
 *              passphrase, the tester's; only their lengths in the log) asked of it as a standing request: WPA2-PSK
 *              with AES by the firmware's own supplicant ("sup_wpa", WLC_SET_WSEC_PMK with the passphrase),
 *              WLC_SET_SSID, the events awaited (11-10: keyed 7.4 s after the request, once rx buffers were posted)
 * The chip is left running, polled (no interrupts yet: en0's thread polls); the chip's deep-sleep requests are acked as
 * brcmfmac does.
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
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/random.h>
#include <sys/systm.h>
#include <sys/hakios.net.h>
#include <sys/hakios.sta.h>
#include <pexpert/device_tree.h>
#include <sys/errno.h>
#include <kern/locks.h>
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
bool akita_pcie_restart(void);                     /* the chip power-cycled, its link again (bcm_recover) */
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
static bool console_forget;                     /* a new firmware (bcm_recover): its console is somewhere else */

static void
console_say(uint32_t shared)
{
	static uint32_t console, buf, size;
	uint32_t widx;

	if (console_forget) {
		console_forget = false;
		console = 0;
		console_read = 0;
	}
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
#define SHARED_HOST_SCB_ADDR    64              /* (bcmdhd's layout, protocol 6+: what brcmfmac's ring update overlaps) */
#define SHARED_HOST_SCB_SIZE    72
#define SHARED_FLAGS2           80
#define SHARED_HOST_CAP         84
#define SHARED_HOST_TRAP_ADDR   88              /* sh_addr_t: where the firmware writes its extended trap data */
/* host_cap (bcmpcie.h HOSTCAP_*): what Android's dhd_set_host_cap() offers this firmware, less in-band device wake */
#define HOSTCAP_H2D_ENABLE_HOSTRDY      0x00000400
#define HOSTCAP_DS_NO_OOB_DW            0x00001000
#define HOSTCAP_EXTENDED_TRAP_DATA      0x00020000
#define HOSTCAP_UR_FW_NO_TRAP           0x00800000
#define HOSTCAP_EDL_RING                0x10000000
#define EXT_TRAP_DATA_MAXLEN            4096    /* BCMPCIE_EXT_TRAP_DATA_MAXLEN */
static uint8_t *trap_data;                      /* the extended trap data, when the firmware has written some */
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
#define MSG_FLOW_RING_CREATE    0x3
#define MSG_FLOW_RING_CREATE_CMPLT 0x4
#define MSG_FLOW_RING_DELETE    0x5
#define MSG_FLOW_RING_DELETE_CMPLT 0x6
#define MSG_TX_POST             0xf
#define MSG_TX_STATUS           0x10
#define MSG_RXBUF_POST          0x11
#define MSG_RX_CMPLT            0x12

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
#define CTRL_BUFS               8               /* ioctl response buffers posted (request ids 1..8) */
/*
 * EVENT BUFFERS (11-11): 32 -- Android's bcmdhd posts at least EVENT_BUF_POOL_LOW (32; 64-256 when the firmware asks
 * through the shared flags2), this driver posted 8. The firmware traps out of packet ids ("hnd_pktid_free_cnt 0",
 * TODO 35f) seconds after a failed handshake or a re-association -- bursts of events, which wait in its own pool for a
 * host buffer to go into. wifi_evbufs=N (1-32) for another count.
 */
#define EVENT_BUFS_MAX          32
#define ALL_CTRL_BUFS           (CTRL_BUFS + EVENT_BUFS_MAX)
static uint32_t event_bufs = EVENT_BUFS_MAX;
#define CTRL_BUF_SIZE           8192
#define RX_BUFS                 256             /* data buffers posted on the rx post ring (brcmfmac: max_rxbufpost) */
#define RX_BUF_SIZE             2048            /* brcmfmac's BRCMF_MSGBUF_MAX_PKT_SIZE */
#define RX_ID_BASE              0x100           /* their request ids: RX_ID_BASE + n (the control buffers' are 1..40) */

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
	uint8_t *ctrl_buf[ALL_CTRL_BUFS];       /* request ids 1..8 ioctl responses, 9..40 events */
	uint64_t ctrl_buf_pa[ALL_CTRL_BUFS];
	uint16_t trans_id;
	bool ioctl_done;
	int16_t ioctl_status;
	uint16_t ioctl_len;
	uint32_t ioctl_buf;             /* the request id of the buffer holding the answer */
	uint32_t shared, h2d_mb, d2h_mb, rx_dataoffset;
	uint32_t tcm_h2d_r;             /* the chip's own array of H2D read indices (4 bytes apart) */
	uint32_t tcm_d2h_w;             /* ... and of its D2H write indices (what it has written, whatever it DMAed to us) */
	uint32_t tcm_d2h_r;             /* ... and of the D2H read indices (ours, as far as it has fetched them?) */
	unsigned events;
	uint8_t *rx_buf;                /* RX_BUFS data buffers of RX_BUF_SIZE in one piece of their own (uncached view) */
	uint64_t rx_buf_pa;
	unsigned rx_posted;             /* on the rx post ring, not yet given back */
	unsigned rx_frames, rx_eapol, rx_bad, rx_bytes;
	uint16_t rx_types[8];           /* the first ethertypes seen, for the log */
	unsigned rx_type_count[8];
} mb;

/*
 * The rings' lock, from the start of the steps: once en0 is up, its thread (the completion rings, every 2-10 ms), the
 * stack's sender and the station's thread (the ioctls of a scan or a join) all reach them. bcm_ioctl takes it itself;
 * the long waits (a scan, a join) take it for each look and let go between.
 */
static lck_grp_t *wl_group;
static lck_mtx_t *wl_lock;

static inline void
wl_take(void)
{
	if (wl_lock != NULL) {
		lck_mtx_lock(wl_lock);
	}
}

static inline void
wl_give(void)
{
	if (wl_lock != NULL) {
		lck_mtx_unlock(wl_lock);
	}
}

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
static vm_offset_t
dma_piece(uint32_t size, uint64_t *pa)
{
	vm_offset_t cached = 0;

	size = (uint32_t)round_page(size);
	if (kmem_alloc_contig(kernel_map, &cached, size, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return 0;
	}
	bzero((void *)cached, size);
	for (vm_offset_t line = cached; line < cached + size; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	*pa = kvtophys(cached);
	return ml_io_map_unmappable(*pa, size, VM_WIMG_WCOMB);
}

static bool
dma_init(uint32_t size)
{
	if (dma.va != 0 && dma.size >= round_page(size)) {     /* (a recovery: the same piece again, cleared) */
		bzero((void *)dma.va, dma.size);
		dma.used = 0;
		return true;
	}
	dma.va = dma_piece(size, &dma.pa);
	dma.size = (uint32_t)round_page(size);
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

static bool fw_halted;                          /* the firmware trapped: nothing more is asked of it until bcm_recover */
static volatile bool recovering;                /* bcm_recover at work: the station refuses work */
static thread_call_t recover_call;
static uint32_t recover_max = 3;                /* boot-arg wifi_recover=N: at most N recoveries in 5 minutes (0: none) */
static void bcm_recover(thread_call_param_t p0, thread_call_param_t p1);
static bool bcm_steps(void);
static void fw_halt_said(void);

/* the chip's mailbox word: deep sleep requests answered (as brcmfmac), a halt said -- with the firmware's console, where
 * it prints its trap (11-10, rejoin1: it halted during the fourth handshake with a wrong passphrase) */
static void
mb_data_check(void)
{
	uint32_t d;

	if (fw_halted) {
		return;
	}
	d = mem_rd(mb.d2h_mb);
	if (d == 0) {
		return;
	}
	mem_wr(mb.d2h_mb, 0);
	printf("wifi: %u: the chip's mailbox: 0x%08x%s%s%s\n", step, d, (d & D2H_MB_DS_ENTER_REQ) ? " (deep sleep asked)" :
	    "", (d & D2H_MB_DS_EXIT_NOTE) ? " (deep sleep over)" : "", (d & D2H_MB_FWHALT) ? " (THE FIRMWARE HALTED)" : "");
	if (d & D2H_MB_FWHALT) {
		fw_halted = true;
		/* where every ring stood: the firmware's write against our read on the completion rings, ours against its on
		 * the submission rings (control, rx post, the flow ring) -- which one was it waiting on? */
		printf("wifi: at the halt: D2H ctrl w %u r %u, txcpl w %u r %u, rxcpl w %u r %u; H2D ctrl w %u r %u, rxpost w %u r %u, "
		    "flow w %u r %u; %u rx buffers posted, %u frames received, %u events\n",
		    mb.idx[mb.d2h_w + 0], mb.r[RING_D2H_CTRL], mb.idx[mb.d2h_w + 1], mb.r[RING_D2H_TXCPL], mb.idx[mb.d2h_w + 2],
		    mb.r[RING_D2H_RXCPL], mb.w[RING_H2D_CTRL], h2d_read_index(RING_H2D_CTRL), mb.w[RING_H2D_RXPOST],
		    h2d_read_index(RING_H2D_RXPOST), mb.idx[mb.h2d_w + 2], mb.idx[mb.h2d_r + 2], mb.rx_posted, mb.rx_frames,
		    mb.events);
		printf("wifi: the firmware's console, its last words (pcie_shared flags 0x%08x):\n", mem_rd(mb.shared + SHARED_FLAGS));
		console_say(mb.shared);
		if (recover_call != NULL && recover_max != 0) {
			recovering = true;
			thread_call_enter(recover_call);        /* (outside this lock: the chip started again from scratch) */
		}
		if (trap_data != NULL && *(volatile uint32_t *)trap_data != 0) {
			printf("wifi: its extended trap data (the first 64 bytes):");
			for (int i = 0; i < 16; i++) {
				printf(" %08x", ((volatile uint32_t *)trap_data)[i]);
			}
			printf("\n");
		}
		fw_halt_said();
		return;
	}
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

static void en0_link_update(void);
static bool en0_attached;
static bool en0_live;                           /* the rings and the flow ring there: en0 carries frames (step 8) */
static void sta_heard_add(const uint8_t *b, uint32_t blen);
static void sta_link_check(void);
static bool sta_collect;
static volatile int sta_phase;                  /* (the station's state: defined with the station, below) */
static struct hakios_sta_bss sta_bss;
static bool bss_now(struct hakios_sta_bss *x, bool locked);

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
		if (sta_collect) {
			sta_heard_add(b, blen);
		}
		at += (blen + 3) & ~3u;
	}
}

static void
event_handle(const uint8_t *frame, uint32_t length)
{
	if (length < EVENT_HDR + EVENT_MSG) {
		/* (11-09: three of 54 bytes during the join -- shorter than a WLC event: from the ethertype on, to decode) */
		char hex[3 * 64 + 1];
		unsigned at = 0;

		hex[0] = 0;
		for (uint32_t i = 12; i < length && i < 12 + 64; i++) {
			at += (unsigned)snprintf(hex + at, sizeof(hex) - at, " %02x", frame[i]);
		}
		printf("wifi: %u: an event of %u bytes; from byte 12:%s\n", step, length, hex);
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
	case 75: {
		/*
		 * WLC_E_ACTION_FRAME_RX: an 802.11 action frame the firmware hands up (wl_event_rx_frame_data_t, then the frame
		 * from its 802.11 header). (11-12, TODO 35e: three, 3 s apart, right after the join -- and frames for the host
		 * held until ~25 s later or for minutes: which kind are they? 3 block ack, 8 SA query, 5 radio measurement, 10
		 * WNM.) The first ten said, all of them.
		 */
		static unsigned said;
		char hex[3 * 72 + 1];
		unsigned at = 0;

		name = " (action frame received)";
		hex[0] = 0;
		for (uint32_t i = 0; i < datalen && i < 72; i++) {
			at += (unsigned)snprintf(hex + at, sizeof(hex) - at, " %02x", data[i]);
		}
		if (said++ < 10) {
			printf("wifi: %u: action frame:%s\n", step, hex);
		}
		break;
	}
	}
	printf("wifi: %u: event %u%s, status %u, reason %u, flags 0x%x, %u bytes\n", step, type, name, status, reason, flags,
	    datalen);
	en0_link_update();
	sta_link_check();
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

/* ---- received frames: data buffers on the rx post ring, given back on the rx completion ring ---- */

/*
 * Data buffers for received frames (brcmfmac's rxbuf_data_post): the firmware runs SPLITRX (its boot log: mode 4) --
 * a received data frame's payload goes straight into a HOST buffer from the rx post ring. With none posted (11-09) the
 * join associated but the WPA2 handshake never started: EAPOL's first message had nowhere to land. Posted here until
 * RX_BUFS are out (or the ring is full); each comes back with a frame on the rx completion ring and is posted again.
 */
static void
rx_post(unsigned n)
{
	uint8_t *item = h2d_reserve(RING_H2D_RXPOST);

	if (item == NULL) {
		return;
	}
	uint64_t pa = mb.rx_buf_pa + (uint64_t)n * RX_BUF_SIZE;
	item[0] = MSG_RXBUF_POST;
	put32(item, 4, RX_ID_BASE + n);	/* (a fresh id each time -- RX_ID_BASE + g * 256 + n -- delivered one frame and no more, 11-11) */
	put16(item, 10, RX_BUF_SIZE);
	put32(item, 24, (uint32_t)pa);
	put32(item, 28, (uint32_t)(pa >> 32));
	h2d_commit(RING_H2D_RXPOST, 1);
	mb.rx_posted++;
}

/* (boot-arg wifi_rxbufs=N: fewer posted, a test -- the firmware's packet ids ran out after a few associations, 11-10) */
static uint32_t rx_bufs = RX_BUFS;

static bool
rx_setup(void)
{
	PE_parse_boot_argn("wifi_rxbufs", &rx_bufs, sizeof(rx_bufs));
	if (rx_bufs == 0 || rx_bufs > RX_BUFS) {
		rx_bufs = RX_BUFS;
	}
	static uint8_t *rx_area;                /* (one piece for the run: a recovery posts the same buffers again) */
	static uint64_t rx_area_pa;

	if (rx_area == NULL) {
		rx_area = (uint8_t *)dma_piece(RX_BUFS * RX_BUF_SIZE, &rx_area_pa);
	}
	mb.rx_buf = rx_area;
	mb.rx_buf_pa = rx_area_pa;
	if (mb.rx_buf == NULL) {
		return false;
	}
	for (unsigned n = 0; n < rx_bufs; n++) {
		rx_post(n);
	}
	return true;
}

/* one received frame (Ethernet: the firmware hands 802.3 frames): counted by ethertype, EAPOL said */
static void
rx_frame(const uint8_t *f, uint32_t len, uint16_t flags, uint8_t ifidx)
{
	uint16_t type = len >= 14 ? (uint16_t)(f[12] << 8 | f[13]) : 0;
	unsigned i;

	mb.rx_frames++;
	mb.rx_bytes += len;
	for (i = 0; i < 8 && mb.rx_type_count[i] != 0 && mb.rx_types[i] != type; i++) {
	}
	if (i < 8) {
		mb.rx_types[i] = type;
		mb.rx_type_count[i]++;
	}
	if (en0_attached && en0_live && ifidx == 0 && len >= 14 && len <= HAKIOS_NET_FRAME_MAX) {
		hakios_net_input(f, len);
	}
	if (type == 0x888e) {
		/* EAPOL: version, type (3: key), length; the key's descriptor type and its key information (which message) */
		mb.rx_eapol++;
		printf("wifi: %u: EAPOL frame received: %u bytes, version %u, type %u, key descriptor %u, key info 0x%04x "
		    "(flags 0x%x, interface %u)\n", step, len, len > 14 ? f[14] : 0, len > 15 ? f[15] : 0, len > 18 ? f[18] : 0,
		    len > 20 ? (unsigned)(f[19] << 8 | f[20]) : 0, flags, ifidx);
	} else if (mb.rx_frames <= 8) {
		printf("wifi: %u: frame received: %u bytes, ethertype 0x%04x (flags 0x%x, interface %u)\n", step, len, type, flags,
		    ifidx);
	}
}

/* what the chip has put on the rx completion ring (and the tx completion ring: nothing sent by the host yet) */
/*
 * The D2H read doorbell (11-12, TODO 35e/35f): with the ring indices DMA'd (ours in host memory), bcmdhd does not only
 * write its read index there -- after each batch it read it RINGS H2D doorbell 0 with 0xDD000000 | ring << 16 | rd
 * (__dhd_prot_upd_read_idx: "for DMA indices case ring doorbell once n items are read to sync with dongle"); this driver
 * never did, and the firmware learned how far the host had read only as a side effect of other work -- the 11-12
 * night's "rx held unless the event logs keep it busy" and, perhaps, the old mode's trap (packet ids held by
 * completions it thinks unread). wifi_rddb=0: as before. RING: the ring's id (2-4 the common completion rings; a
 * later D2H ring: max submission rings + its D2H offset, as DHD_D2H_RING_OFFSET reads it).
 */
static bool
wifi_rddb(void)
{
	static uint32_t v = ~0u;

	if (v == ~0u) {
		v = 1;
		PE_parse_boot_argn("wifi_rddb", &v, sizeof(v));
	}
	return v != 0;
}

static void
d2h_read_doorbell(unsigned ring, uint16_t rd)
{
	if (!wifi_rddb()) {
		return;
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	pcie_wr(PCIE_H2D_MAILBOX_0, 0xdd000000u | ((uint32_t)(ring & 0xff) << 16) | rd);
}

static unsigned
d2h_rx_poll(void)
{
	unsigned n = 0;
	uint16_t w = (uint16_t)mb.idx[mb.d2h_w + (RING_D2H_RXCPL - 2)];

	if (mb.rx_buf == NULL || w >= ring_items[RING_D2H_RXCPL]) {
		return 0;
	}
	while (mb.r[RING_D2H_RXCPL] != w) {
		const uint8_t *m = mb.ring[RING_D2H_RXCPL] + (uint32_t)mb.r[RING_D2H_RXCPL] * ring_item_size[RING_D2H_RXCPL];
		uint32_t id = get32(m, 4);
		uint16_t len = get16(m, 14), offset = get16(m, 16), flags = get16(m, 18), status = get16(m, 8);

		if (m[0] == MSG_RX_CMPLT && status != 0) {
			/* (a buffer given back without a frame: flushed by the firmware? -- counted, posted again below) */
			static unsigned said;

			if (said++ < 8) {
				printf("wifi: rx completion with status %u (id 0x%x, %u bytes)\n", status, id, len);
			}
			len = 0;
		}
		if (m[0] == MSG_RX_CMPLT && id >= RX_ID_BASE && id < RX_ID_BASE + RX_BUFS) {
			unsigned b = id - RX_ID_BASE;

			if (offset == 0) {
				offset = (uint16_t)mb.rx_dataoffset;
			}
			if (offset + (uint32_t)len <= RX_BUF_SIZE) {
				rx_frame(mb.rx_buf + (uint32_t)b * RX_BUF_SIZE + offset, len, flags, m[1]);
			} else {
				mb.rx_bad++;
			}
			mb.rx_posted--;
			rx_post(b);
		} else {
			mb.rx_bad++;
			if (mb.rx_bad <= 4) {
				printf("wifi: %u: rx completion ring: message type 0x%x, id 0x%x\n", step, m[0], id);
			}
		}
		mb.r[RING_D2H_RXCPL] = (uint16_t)((mb.r[RING_D2H_RXCPL] + 1) % ring_items[RING_D2H_RXCPL]);
		mb.idx[mb.d2h_r + (RING_D2H_RXCPL - 2)] = mb.r[RING_D2H_RXCPL];
		n++;
	}
	if (n) {
		d2h_read_doorbell(RING_D2H_RXCPL, mb.r[RING_D2H_RXCPL]);
	}
	return n;
}

static void
rx_say(void)
{
	char types[8 * 16];
	unsigned at = 0;

	types[0] = 0;
	for (unsigned i = 0; i < 8 && mb.rx_type_count[i] != 0; i++) {
		at += (unsigned)snprintf(types + at, sizeof(types) - at, " 0x%04x:%u", mb.rx_types[i], mb.rx_type_count[i]);
	}
	printf("wifi: %u: received %u frames (%u bytes), %u EAPOL, %u bad; by ethertype:%s; %u buffers posted\n", step,
	    mb.rx_frames, mb.rx_bytes, mb.rx_eapol, mb.rx_bad, at ? types : " none", mb.rx_posted);
}

/*
 * ---- the firmware's debug info rings (bcmdhd's dhd_prot_init_info_rings; 11-11 night, TODO 35f) ----
 * The firmware keeps its event-log records (the console's "EL: <tag> <fmt>", its statistics) for the host, which
 * takes them through a pair of rings Android creates at the start: an H2D "debug buffer submit" ring (type 5) of
 * buffers posted for them, and a D2H "debug buffer completion" ring (type 4) where they come back filled. This driver
 * made neither -- and the firmware leaked them: its heap and packet ids ran out ~130 s after its start (THE TRAP).
 * Here both are made (bcmdhd's ids: the last H2D slot, max_sub - 1; the spare D2H slot, max_cpl - 1, which the
 * submit ring names as its completion ring by bcmdhd's global number max_sub + it), buffers posted, and every
 * completion posted again at once -- the records themselves are not read.
 * BUT this firmware supports the EDL ring (flags2 PCIE_SHARED2_EDL_RING) and then logs only there, as Android builds
 * bcmdhd (EWP_EDL): with the info rings made it sent nothing through them and leaked the same (info1). So by DEFAULT
 * (wifi_inforing=3) the EDL ring: one D2H ring of type 7, 256 items of 2 KB in a piece of its own, which the firmware
 * fills (host_cap HOSTCAP_EDL_RING) and this driver reads past -- THE LEAK STOPS: its free packet ids flat for minutes
 * (edl1). wifi_inforing=1: the info rings (2: their first buffers said); 4: the EDL ring, its first items said;
 * 0: neither.
 */
#define MSG_INFO_BUF_POST       0x19
#define MSG_INFO_BUF_CMPLT      0x1a
#define MSG_H2D_RING_CREATE     0x1b
#define MSG_D2H_RING_CREATE     0x1c
#define MSG_H2D_RING_CREATE_CMPLT 0x1d
#define MSG_D2H_RING_CREATE_CMPLT 0x1e
#define INFO_ITEMS              32              /* H2DRING_DYNAMIC_INFO_MAX_ITEM, D2HRING_DYNAMIC_INFO_MAX_ITEM */
#define INFO_SUB_SIZE           40              /* H2DRING_INFO_BUFPOST_ITEMSIZE */
#define INFO_CPL_SIZE           24              /* D2HRING_INFO_BUFCMPLT_ITEMSIZE */
#define INFO_BUFS               16
#define INFO_BUF_SIZE           2048
#define INFO_ID_BASE            0x3000
static struct { unsigned max_sub, max_cpl; } ring_counts;
static struct {
	uint8_t *sub, *cpl, *buf;
	uint64_t sub_pa, cpl_pa, buf_pa;
	unsigned h2d, d2h;              /* their places in the index arrays */
	uint16_t w, r;
	int d2h_status, h2d_status;     /* 1 asked, then the firmware's status (0 made) */
	unsigned completions, bytes;
	bool up;
	uint32_t mode;
	bool edl;                       /* wifi_inforing=3: the EDL ring instead (one D2H ring the firmware writes into) */
} info;
#define EDL_ITEMS               256             /* D2HRING_EDL_MAX_ITEM */
#define EDL_ITEM_SIZE           2048            /* D2HRING_EDL_ITEMSIZE */
static uint8_t *edl_ring;                       /* (its own piece, 512 KB, kept across recoveries) */
static uint64_t edl_ring_pa;

static void
info_buf_post(uint32_t n)
{
	uint8_t *item = info.sub + (uint32_t)info.w * INFO_SUB_SIZE;
	uint64_t pa = info.buf_pa + (uint64_t)n * INFO_BUF_SIZE;

	bzero(item, INFO_SUB_SIZE);
	item[0] = MSG_INFO_BUF_POST;
	put32(item, 4, INFO_ID_BASE + n);
	put16(item, 8, INFO_BUF_SIZE);
	put32(item, 16, (uint32_t)pa);
	put32(item, 20, (uint32_t)(pa >> 32));
	info.w = (uint16_t)((info.w + 1) % INFO_ITEMS);
	mb.idx[mb.h2d_w + info.h2d] = info.w;
	__asm__ volatile ("dsb sy" ::: "memory");
	pcie_wr(PCIE_H2D_MAILBOX_0, 1);
}

/* what the firmware has put on the debug info completion ring: each buffer posted again (the rings' lock held) */
static unsigned
info_poll(void)
{
	unsigned n = 0;

	if (!info.up) {
		return 0;
	}
	uint16_t w = (uint16_t)mb.idx[mb.d2h_w + info.d2h];

	if (info.edl) {                 /* (the EDL ring: its items are the logs themselves -- read past, counted) */
		if (w >= EDL_ITEMS) {
			return 0;
		}
		while (info.r != w) {
			if (info.mode == 4 && (info.completions < 6 || (info.completions >= 40 && info.completions < 70))) {
				/* (wifi_inforing=4: what EDL items hold -- the first few, and some after the join: 48 bytes each) */
				const uint8_t *m = edl_ring + (uint32_t)info.r * EDL_ITEM_SIZE;
				char hex[48 * 3 + 1];

				for (unsigned i = 0; i < 48; i++) {
					snprintf(hex + 3 * i, 4, " %02x", m[i]);
				}
				printf("wifi: edl: item %u (#%u):%s\n", info.r, info.completions, hex);
			}
			info.completions++;
			info.r = (uint16_t)((info.r + 1) % EDL_ITEMS);
			mb.idx[mb.d2h_r + info.d2h] = info.r;
			n++;
		}
		if (n) {
			d2h_read_doorbell(ring_counts.max_sub + info.d2h, info.r);
		}
		return n;
	}
	if (w >= INFO_ITEMS) {
		return 0;
	}
	while (info.r != w) {
		const uint8_t *m = info.cpl + (uint32_t)info.r * INFO_CPL_SIZE;
		uint32_t id = get32(m, 4);
		uint16_t len = get16(m, 12);

		if (m[0] == MSG_INFO_BUF_CMPLT && id >= INFO_ID_BASE && id < INFO_ID_BASE + INFO_BUFS) {
			info.completions++;
			info.bytes += len;
			if (info.mode == 2 && info.completions <= 20) {
				printf("wifi: info: a buffer back with %u bytes of the firmware's logs (status %d)\n", len,
				    (int16_t)get16(m, 8));
			}
			info_buf_post(id - INFO_ID_BASE);
		}
		info.r = (uint16_t)((info.r + 1) % INFO_ITEMS);
		mb.idx[mb.d2h_r + info.d2h] = info.r;
		n++;
	}
	if (n) {
		d2h_read_doorbell(ring_counts.max_sub + info.d2h, info.r);
	}
	return n;
}

static unsigned d2h_ctrl_poll(void);

/*
 * WHICH MODE (11-12 night, TODO 35e/35f) -- wifi_edl=1: the EDL ring (host_cap HOSTCAP_EDL_RING and a D2H ring of type
 * 7): the firmware's event logs go there and it never runs out of packet ids (35f's trap fixed) -- BUT it hands the
 * host one received frame after the join and then holds them all (it releases held rx only as a side effect of its
 * event logs going the old way): no ARP answer, no DNS, iOS "offline". Default (0), the old mode: no host_cap, no
 * EDL ring, event-log tags 121-482 off (0-120 left on: with them off rx is held too -- the volume, no single tag):
 * rx flows ~25 s after the join, and the firmware traps every ~2-3 minutes (its packet ids gone into event logs no
 * ring takes) and is recovered in 7.6 s -- the internet there ~75% of the time (uptime-legacy 25/35 fetches,
 * ev121-482 24/32) against ~0% with the EDL ring. Until the rx hold is understood, the mode with an internet.
 */
static bool
wifi_edl(void)
{
	static uint32_t edl = ~0u;

	if (edl == ~0u) {
		edl = 0;
		PE_parse_boot_argn("wifi_edl", &edl, sizeof(edl));
	}
	return edl != 0;
}

/* the two rings asked for and the buffers posted (step 6, the rings' lock held) */
static void
info_rings_up(void)
{
	uint64_t t0;

	info.mode = wifi_edl() ? 3 : 0; /* (the EDL ring with wifi_edl=1, with host_cap's HOSTCAP_EDL_RING: rings_up) */
	PE_parse_boot_argn("wifi_inforing", &info.mode, sizeof(info.mode));
	if (info.mode == 0) {
		return;
	}
	if (ring_counts.max_cpl < 4 || ring_counts.max_sub < 3) {
		printf("wifi: 6: no spare ring slots for the debug info rings (%u submission, %u completion)\n", ring_counts.max_sub,
		    ring_counts.max_cpl);
		return;
	}
	info.h2d = ring_counts.max_sub - 1;
	info.d2h = ring_counts.max_cpl - 1;
	if (info.mode >= 3) {
		/* the EDL ring (bcmdhd's dhd_prot_init_edl_rings, used when the firmware says PCIE_SHARED2_EDL_RING and the
		 * host_cap has HOSTCAP_EDL_RING: wifi_hostcap=0x10000000): one D2H ring, type 7, 256 items of 2 KB */
		if (edl_ring == NULL) {
			edl_ring = (uint8_t *)dma_piece(EDL_ITEMS * EDL_ITEM_SIZE, &edl_ring_pa);
		}
		if (edl_ring == NULL) {
			printf("wifi: 6: no 512 KB in one piece for the EDL ring\n");
			return;
		}
		bzero(edl_ring, EDL_ITEMS * EDL_ITEM_SIZE);
		uint8_t *it = h2d_reserve(RING_H2D_CTRL);
		if (it == NULL) {
			return;
		}
		it[0] = MSG_D2H_RING_CREATE;
		put32(it, 4, 0xfff9);
		put16(it, 8, (uint16_t)info.d2h);
		put16(it, 10, 7);                       /* BCMPCIE_D2H_RING_TYPE_EDL */
		put32(it, 16, (uint32_t)edl_ring_pa);
		put32(it, 20, (uint32_t)(edl_ring_pa >> 32));
		put16(it, 24, EDL_ITEMS);
		put16(it, 26, EDL_ITEM_SIZE);
		info.d2h_status = 1;
		h2d_commit(RING_H2D_CTRL, 1);
		for (t0 = mach_absolute_time(); info.d2h_status == 1 && micros_since(t0) < 1000000;) {
			if (d2h_ctrl_poll() == 0) {
				delay(200);
			}
		}
		printf("wifi: 6: the EDL ring (D2H %u, %u x %u bytes): %s %d\n", info.d2h, EDL_ITEMS, EDL_ITEM_SIZE,
		    info.d2h_status == 0 ? "MADE" : info.d2h_status == 1 ? "no answer" : "refused", info.d2h_status);
		if (info.d2h_status == 0) {
			info.edl = true;
			info.up = true;
		}
		return;
	}
	info.sub = dma_take(INFO_ITEMS * INFO_SUB_SIZE, &info.sub_pa);
	info.cpl = dma_take(INFO_ITEMS * INFO_CPL_SIZE, &info.cpl_pa);
	info.buf = dma_take(INFO_BUFS * INFO_BUF_SIZE, &info.buf_pa);
	if (info.sub == NULL || info.cpl == NULL || info.buf == NULL) {
		printf("wifi: 6: no room for the debug info rings\n");
		return;
	}
	bzero(info.sub, INFO_ITEMS * INFO_SUB_SIZE);
	bzero(info.cpl, INFO_ITEMS * INFO_CPL_SIZE);

	/* the completion ring first (d2h_ring_create_req_t), then the submit ring naming it (h2d_ring_create_req_t) */
	uint8_t *item = h2d_reserve(RING_H2D_CTRL);
	if (item == NULL) {
		return;
	}
	item[0] = MSG_D2H_RING_CREATE;
	put32(item, 4, 0xfff9);                         /* (DHD_D2H_DBGRING_REQ_PKTID's like) */
	put16(item, 8, (uint16_t)info.d2h);
	put16(item, 10, 4);                             /* BCMPCIE_D2H_RING_TYPE_DBGBUF_CPL */
	put32(item, 16, (uint32_t)info.cpl_pa);
	put32(item, 20, (uint32_t)(info.cpl_pa >> 32));
	put16(item, 24, INFO_ITEMS);
	put16(item, 26, INFO_CPL_SIZE);
	info.d2h_status = 1;
	h2d_commit(RING_H2D_CTRL, 1);
	for (t0 = mach_absolute_time(); info.d2h_status == 1 && micros_since(t0) < 1000000;) {
		if (d2h_ctrl_poll() == 0) {
			delay(200);
		}
	}
	if (info.d2h_status != 0) {
		printf("wifi: 6: the debug info completion ring (D2H %u): %s %d\n", info.d2h, info.d2h_status == 1 ? "no answer" :
		    "refused", info.d2h_status);
		return;
	}
	if ((item = h2d_reserve(RING_H2D_CTRL)) == NULL) {
		return;
	}
	item[0] = MSG_H2D_RING_CREATE;
	put32(item, 4, 0xfffa);
	put16(item, 8, (uint16_t)info.h2d);
	item[10] = 5;                                   /* BCMPCIE_H2D_RING_TYPE_DBGBUF_SUBMIT */
	item[11] = 1;                                   /* one completion ring */
	put32(item, 16, (uint32_t)info.sub_pa);
	put32(item, 20, (uint32_t)(info.sub_pa >> 32));
	put16(item, 24, INFO_ITEMS);
	put16(item, 26, INFO_SUB_SIZE);
	put16(item, 28, (uint16_t)(ring_counts.max_sub + info.d2h));
	info.h2d_status = 1;
	h2d_commit(RING_H2D_CTRL, 1);
	for (t0 = mach_absolute_time(); info.h2d_status == 1 && micros_since(t0) < 1000000;) {
		if (d2h_ctrl_poll() == 0) {
			delay(200);
		}
	}
	if (info.h2d_status != 0) {
		printf("wifi: 6: the debug info submit ring (H2D %u): %s %d\n", info.h2d, info.h2d_status == 1 ? "no answer" :
		    "refused", info.h2d_status);
		return;
	}
	info.up = true;
	for (uint32_t i = 0; i < INFO_BUFS; i++) {
		info_buf_post(i);
	}
	printf("wifi: 6: THE DEBUG INFO RINGS are made (H2D %u, D2H %u: the firmware's logs have somewhere to go), %u buffers "
	    "of %u bytes posted\n", info.h2d, info.d2h, INFO_BUFS, INFO_BUF_SIZE);
}

static void flow_created(int16_t status, uint16_t ring);
static void flow_deleted(int16_t status, uint16_t ring);
static unsigned d2h_tx_poll(void);

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
			if (id > CTRL_BUFS && id <= CTRL_BUFS + event_bufs) {
				event_handle(mb.ctrl_buf[id - 1] + mb.rx_dataoffset, get16(m, 12));
				mb.events++;
				ctrl_buf_post(id);
			}
			break;
		case MSG_FLOW_RING_CREATE_CMPLT:
			flow_created((int16_t)get16(m, 8), get16(m, 10));
			break;
		case MSG_D2H_RING_CREATE_CMPLT:
			info.d2h_status = (int16_t)get16(m, 8);
			break;
		case MSG_H2D_RING_CREATE_CMPLT:
			info.h2d_status = (int16_t)get16(m, 8);
			break;
		case MSG_FLOW_RING_DELETE_CMPLT:
			flow_deleted((int16_t)get16(m, 8), get16(m, 10));
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
	if (n) {
		d2h_read_doorbell(RING_D2H_CTRL, mb.r[RING_D2H_CTRL]);
	}
	n += d2h_rx_poll();
	n += d2h_tx_poll();
	n += info_poll();
	mb_data_check();
	return n;
}

/*
 * One ioctl: `cmd`, `in` (of `in_len`, the input) and room for `out_len` in the answer; the answer is copied into
 * `out` (up to out_len). Returns the firmware's status (0: done), or -1000 for no answer within 2 s.
 */
static int bcm_ioctl_locked(uint32_t cmd, const void *in, uint32_t in_len, void *out, uint32_t out_len);

static int
bcm_ioctl(uint32_t cmd, const void *in, uint32_t in_len, void *out, uint32_t out_len)
{
	int err;

	wl_take();
	err = bcm_ioctl_locked(cmd, in, in_len, out, out_len);
	wl_give();
	return err;
}

static int
bcm_ioctl_locked(uint32_t cmd, const void *in, uint32_t in_len, void *out, uint32_t out_len)
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

/*
 * (11-12, TODO 35e, a test) wifi_msi=1: the chip's MSI enabled in its configuration space (capability 5), its
 * address a word of our memory -- an interrupt is then just a write of the data there, counted by en0's thread
 * (msi_seen), no handler needed. Android's host takes the chip's interrupts as MSIs; this one polls with MSI off
 * (INTx): does the firmware hold received frames for a host that cannot be interrupted?
 */
static volatile uint32_t *msi_word;
static unsigned msi_seen;

static void
msi_setup(void)
{
	uint32_t want = 0;
	static uint64_t pa;

	if (!PE_parse_boot_argn("wifi_msi", &want, sizeof(want)) || want == 0) {
		return;
	}
	uint32_t at = rd(cfg, 0x34) & 0xfc;
	for (unsigned hops = 0; at >= 0x40 && at < 0x100 && hops < 32; hops++) {
		uint32_t head = rd(cfg, at);

		if ((head & 0xff) == 0x05) {
			uint16_t control = (uint16_t)(head >> 16);
			bool wide = (control & 0x80) != 0;

			if (msi_word == NULL) {
				msi_word = (volatile uint32_t *)dma_piece(4096, &pa);
			}
			if (msi_word == NULL) {
				printf("wifi: 5: MSI: no memory\n");
				return;
			}
			*msi_word = 0;
			wr(cfg, at + 4, (uint32_t)pa);
			if (wide) {
				wr(cfg, at + 8, (uint32_t)(pa >> 32));
				wr(cfg, at + 12, 0x5a5a);
			} else {
				wr(cfg, at + 8, 0x5a5a);
			}
			wr(cfg, at, (head & 0xff8fffffu) | (1u << 16));        /* enable, one vector */
			wr(cfg, 0x04, rd(cfg, 0x04) | (1u << 10) | (1u << 2));  /* INTx off; bus master on */
			printf("wifi: 5: MSI enabled at 0x%02x (control 0x%04x -> 0x%04x, %s), its writes to 0x%llx\n", at, control,
			    rd(cfg, at) >> 16, wide ? "64-bit" : "32-bit", pa);
			return;
		}
		at = (head >> 8) & 0xfc;
	}
	printf("wifi: 5: MSI: no capability 5 in the chip's configuration space\n");
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
	mb.tcm_d2h_w = mem_rd(ringinfo + 12);
	mb.tcm_d2h_r = mem_rd(ringinfo + 16);
	max_sub = mem_rd(ringinfo + 52) >> 16;          /* (max_tx_flowrings at +52, max_submission_queues +54) */
	max_cpl = mem_rd(ringinfo + 56) & 0xffff;
	ring_counts.max_sub = max_sub;
	ring_counts.max_cpl = max_cpl;
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
		/*
		 * NOT brcmfmac's ring-update buffer (11-11 night, TODO 35f): in this firmware's pcie_shared (protocol 7, bcmdhd's
		 * bcmpcie.h) offsets 64-75 are host_scb_addr (8 bytes) and host_scb_size -- memory the host lends the firmware
		 * for its station control blocks (HOSTCAP_HSCB) -- and brcmfmac's "ring update" length and address written there
		 * read as an SCB area at (address << 32 | 1024) of 8 bytes. Zero, as a host without HSCB leaves them.
		 * wifi_ringupd=1: as before.
		 */
		uint32_t ringupd_old = 0;

		PE_parse_boot_argn("wifi_ringupd", &ringupd_old, sizeof(ringupd_old));
		if (ringupd_old) {
			void *ringupd = dma_take(1024, &pa);

			(void)ringupd;
			mem_wr(shared + SHARED_DMA_RINGUPD_ADDR, (uint32_t)pa);
			mem_wr(shared + SHARED_DMA_RINGUPD_ADDR + 4, (uint32_t)(pa >> 32));
			mem_wr(shared + SHARED_DMA_RINGUPD_LEN, 1024);
		} else {
			mem_wr(shared + SHARED_HOST_SCB_ADDR, 0);
			mem_wr(shared + SHARED_HOST_SCB_ADDR + 4, 0);
			mem_wr(shared + SHARED_HOST_SCB_SIZE, 0);
		}
		/*
		 * The host's capabilities (11-11 night, TODO 35f): this driver wrote none -- every capability off, the IPC version
		 * 0. Android's dhd_set_host_cap() (dhd_msgbuf.c) writes, for this firmware (flags 0x70050107: protocol 7, host
		 * ready, in-band and no-OOB deep sleep offered): the version (7), HOSTRDY (it rings doorbell 1 after the rings,
		 * as this driver does), extended trap data (into a host buffer whose address it gives), UR_FW_NO_TRAP, and for
		 * deep sleep either in-band device wake (Pixel's build) or, without that, DS_NO_OOB_DW: "Disable DS altogether"
		 * -- (this driver has no device wake; it acks the firmware's deep-sleep requests).
		 * TRIED (11-11 night, hc-*, rejoin-ds0): that whole set halted the chip at once (the PCIe link gone after host
		 * ready); the version alone (7) -- the firmware never read the control ring: protocol 7 wants more of the host
		 * than these bits; DS_NO_OOB_DW alone (0x1000) -- joins as before, and the trap after a failed handshake is the
		 * same.
		 * WHAT IS WRITTEN WITH wifi_edl=1 (11-11 late, THE FIX OF THE TRAP; not the default since 11-12: wifi_edl()): HOSTCAP_EDL_RING alone -- the firmware's event logs and
		 * statistics then go into the EDL ring step 6 makes (info_rings_up), as on Android (its Kbuild: EWP_EDL,
		 * EWP_ECNTRS_LOGGING); without it the firmware kept them for a host ring that never came and ran out of memory
		 * and packet ids ~130 s after its start, every time. It honours the bit with the version 0.
		 * wifi_hostcap=N: N written instead (with EXTENDED_TRAP_DATA, a buffer for it given too); 1: nothing.
		 */
		uint32_t hostcap = wifi_edl() ? HOSTCAP_EDL_RING : 1;  /* (1: none written -- the old mode, 11-12: see wifi_edl) */
		(void)(HOSTCAP_DS_NO_OOB_DW | HOSTCAP_UR_FW_NO_TRAP | HOSTCAP_H2D_ENABLE_HOSTRDY);

		PE_parse_boot_argn("wifi_hostcap", &hostcap, sizeof(hostcap));
		if (hostcap != 1) {
			if (hostcap & HOSTCAP_EXTENDED_TRAP_DATA) {
				trap_data = dma_take(EXT_TRAP_DATA_MAXLEN, &pa);
				if (trap_data != NULL) {
					bzero(trap_data, EXT_TRAP_DATA_MAXLEN);
					mem_wr(shared + SHARED_HOST_TRAP_ADDR, (uint32_t)pa);
					mem_wr(shared + SHARED_HOST_TRAP_ADDR + 4, (uint32_t)(pa >> 32));
				} else {
					hostcap &= ~HOSTCAP_EXTENDED_TRAP_DATA;
				}
			}
			mem_wr(shared + SHARED_HOST_CAP, hostcap);
		}
		printf("wifi: 5: pcie_shared flags2 0x%08x, host_cap 0x%08x (%s), the firmware's host SCB size %u (%s)\n",
		    mem_rd(shared + SHARED_FLAGS2), mem_rd(shared + SHARED_HOST_CAP), hostcap == 1 ? "none written" :
		    "wifi_hostcap's", mem_rd(shared + SHARED_HOST_SCB_SIZE),
		    ringupd_old ? "brcmfmac's ring-update buffer written over it" : "zeroed");
	}
	mb.ioctl_req = dma_take(IOCTL_MAX + 2, &mb.ioctl_req_pa);
	PE_parse_boot_argn("wifi_evbufs", &event_bufs, sizeof(event_bufs));
	if (event_bufs < 1 || event_bufs > EVENT_BUFS_MAX) {
		event_bufs = EVENT_BUFS_MAX;
	}
	for (unsigned i = 0; i < CTRL_BUFS + event_bufs; i++) {
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
	{
		/*
		 * (11-12, TODO 35e, a test) wifi_intmask=N: the PCIe core's MailboxIntMask set to N -- bcmdhd's def_intmask for a
		 * core of revision 64 or later is PCIE_MB_D2H_MB_MASK, 0xffff (the D2H doorbells; bit 0, D2H doorbell 0, is the
		 * status the firmware sets after it puts work on the completion rings). This driver polls and has no
		 * interrupt handler: left 0 the host looks like one with its interrupts off -- a host asleep?
		 */
		uint32_t intmask = ~0u;

		if (PE_parse_boot_argn("wifi_intmask", &intmask, sizeof(intmask)) && intmask != ~0u) {
			pcie_wr(PCIE_MAILBOX_MASK, intmask);
			printf("wifi: 5: interrupt mask 0x%08x set (reads 0x%08x)\n", intmask, pcie_rd(PCIE_MAILBOX_MASK));
		} else {
			printf("wifi: 5: interrupt mask as found: 0x%08x\n", pcie_rd(PCIE_MAILBOX_MASK));
		}
	}
	msi_setup();
	if (flags & SHARED_FLAG_HOSTRDY) {
		pcie_wr(PCIE_H2D_MAILBOX_1, 1);
		printf("wifi: 5: host ready (doorbell 1)\n");
	}
	for (uint32_t id = 1; id <= CTRL_BUFS + event_bufs; id++) {
		ctrl_buf_post(id);
	}
	if (!rx_setup()) {
		printf("wifi: 5: no memory in one piece for the rx buffers (%u KB): frames cannot be received\n",
		    RX_BUFS * RX_BUF_SIZE / 1024);
	}
	sleep_ms(50);
	(void)d2h_ctrl_poll();
	printf("wifi: 5: %u ioctl and %u event buffers posted; the chip has read the control ring to %u (of %u written); "
	    "%u rx buffers of %u bytes posted at 0x%llx, the rx post ring read to %u (of %u); mailbox interrupts 0x%08x\n",
	    CTRL_BUFS, event_bufs, mb.idx[mb.h2d_r + RING_H2D_CTRL], mb.w[RING_H2D_CTRL], mb.rx_posted, RX_BUF_SIZE,
	    mb.rx_buf_pa, h2d_read_index(RING_H2D_RXPOST), mb.w[RING_H2D_RXPOST], pcie_rd(PCIE_MAILBOX_INT));
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
	{
		/*
		 * The address: the one iBoot reports, the loader's /chosen mac-address-wifi0 (what MobileGestalt builds the
		 * phone's identity from), set before the radio comes up -- en0's on an iPhone is that one. Else the chip makes
		 * up a new one at every start, and configd's interfaces (NetworkInterfaces.plist, by address) see a new radio
		 * each time. (The loader's is a fixed local one today; the phone's own is wlan_mac1 of ABL's device tree.)
		 */
		DTEntry chosen;
		void const *mac = NULL;
		unsigned int size = 0;

		uint32_t own = 1;               /* (wifi_mac=0, a test of TODO 35e: the chip's own address, en0 made at step 8) */

		PE_parse_boot_argn("wifi_mac", &own, sizeof(own));
		if (own != 0 && SecureDTLookupEntry(NULL, "/chosen", &chosen) == kSuccess &&
		    SecureDTGetProperty(chosen, "mac-address-wifi0", &mac, &size) == kSuccess && size == 6) {
			const uint8_t *m = mac;

			printf("wifi: 7: \"cur_etheraddr\" set to the device tree's mac-address-wifi0 (%02x:xx:xx:xx:xx:%02x): %d\n",
			    m[0], m[5], iovar_set("cur_etheraddr", m, 6));
		} else {
			printf("wifi: 7: no mac-address-wifi0 in the device tree's /chosen: the chip's own address\n");
		}
	}
	{
		/*
		 * (11-12, TODO 35e, a test) wifi_ampdurx=N: "ampdu_rx" N before the radio is up (0: no block-ack sessions taken
		 * from the access point). With the EDL ring the firmware took in and decrypted ~1 frame a second after the join
		 * (its counters) and handed the host none for minutes -- then, once, hundreds at a time: frames held somewhere,
		 * released together; a reorder queue waiting on a hole is one such place.
		 */
		uint32_t ampdurx = ~0u;

		if (PE_parse_boot_argn("wifi_ampdurx", &ampdurx, sizeof(ampdurx)) && ampdurx <= 1) {
			printf("wifi: 7: \"ampdu_rx\" %u: %d\n", ampdurx, iovar_int("ampdu_rx", ampdurx));
		}
	}
	{
		/*
		 * "bus:host_access" 1 right before WLC_UP, as Android does (its Kbuild has WLAN_ACCEL_BOOT: dhd_cfg80211.c's
		 * wl_dongle_up; 0 again after WLC_DOWN): the host's memory there for the firmware to use. (11-12, TODO 35e: being
		 * tested.) wifi_hostaccess=0: not said.
		 */
		uint32_t ha = 1;

		PE_parse_boot_argn("wifi_hostaccess", &ha, sizeof(ha));
		if (ha != 0) {
			printf("wifi: 7: \"bus:host_access\" 1: %d\n", iovar_int("bus:host_access", 1));
		}
	}
	printf("wifi: 7: \"mpc\" 0: %d; infrastructure: %d; up: %d\n", iovar_int("mpc", 0), cmd_int(20, 1), cmd_int(2, 1));
	{
		/* (11-11, a test of TODO 35f) wifi_roamoff=1: the firmware's own roaming off -- after a failed handshake it may
		 * try other access points or re-associate by itself, and its packet pool ran dry ~40 ms after such a failure */
		uint32_t roamoff = 0;

		if (PE_parse_boot_argn("wifi_roamoff", &roamoff, sizeof(roamoff)) && roamoff) {
			printf("wifi: 7: \"roam_off\" 1: %d\n", iovar_int("roam_off", 1));
		}
	}
	{
		/*
		 * The radio's power saving (WLC_GET_PM 85 / WLC_SET_PM 86: 0 off, 1 PM_MAX -- awake for the DTIM beacons only, 2
		 * PM_FAST -- awake while traffic flows; Android's dhd sets 2). Not set by this driver until 11-12: the firmware's
		 * own default. (11-12, TODO 35e: after the join one frame comes in, then none for minutes while ~150 go out --
		 * the chip's own rx completion index does not move: the firmware takes nothing in.) wifi_pm=N: N set, a test.
		 */
		uint32_t pm = ~0u, now = ~0u;
		int got = bcm_ioctl(85, &now, sizeof(now), &now, sizeof(now));

		if (PE_parse_boot_argn("wifi_pm", &pm, sizeof(pm)) && pm <= 2) {
			printf("wifi: 7: power saving (PM) %u as found (%d); %u set: %d\n", now, got, pm, cmd_int(86, pm));
		} else {
			printf("wifi: 7: power saving (PM) %u as found (%d), left so\n", now, got);
		}
	}
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

/*
 * ---- step 8: en0 on the chip ----
 *
 * The interface (drivers/net/hakios.net.c, sys/hakios.net.h: Ethernet to the stack, "Wi-Fi" by its subfamily) on top of
 * this driver's rings: received frames (rx_frame, interface 0) handed to hakios_net_input; frames to send copied into
 * buffers of our own and posted on ONE flow ring (a station sends everything to its access point: brcmfmac's STA flow
 * hash, one ring per priority -- one for all here), created with the first frame's destination as brcmfmac does;
 * TX_STATUS gives the buffer back. Link up for the stack once associated AND keyed (configd starts DHCP then). The
 * stack's sender and the interface's thread (its poll: the completion rings) take wl_lock; the one-shot steps before
 * are done by then. The address: the chip's own for now (it changes each start: the phone's is wlan_mac1 of the device
 * tree, to set with "cur_etheraddr" before the radio comes up -- a phone identifier, never in git).
 */
#define FLOW_ITEMS              512             /* brcmfmac's BRCMF_H2D_TXFLOWRING_MAX_ITEM and ITEMSIZE */
#define FLOW_ITEM_SIZE          48
#define FLOW_H2D_INDEX          2               /* the first flow ring: H2D ring 2 (after control and rx post) */
#define TX_BUFS                 128
#define TX_BUF_SIZE             2048
#define TX_ID_BASE              0x1000

static struct {
	uint8_t *ring, *buf;
	uint64_t ring_pa, buf_pa;
	uint16_t w;
	int state;                      /* 0 none, 1 asked, 2 ready, 3 being deleted, -1 refused */
	uint32_t busy[TX_BUFS / 32];
	unsigned sent, done, dropped, outstanding;
	uint8_t address[6];
} tx;
static bool en0_link_said;

static void flow_create(const uint8_t *da);
static void flow_delete(void);

static void
en0_link_update(void)
{
	static const uint8_t everyone[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	bool up = link_up && keyed;

	/* the network left or lost: its flow ring deleted, as brcmfmac deletes a peer's (a new one at the next join);
	 * boot-arg wifi_flow_keep=1: kept for the next network (a test) */
	static int flow_keep = -1;

	if (flow_keep < 0) {
		uint32_t keep = 0;

		PE_parse_boot_argn("wifi_flow_keep", &keep, sizeof(keep));
		flow_keep = keep != 0;
	}
	if (!up && en0_live && tx.state == 2 && !flow_keep) {
		flow_delete();
	}
	/*
	 * The flow ring made as soon as frames can pass, not by the first frame (which was dropped while it was being
	 * made, and the ones right after it: sta5, the first DHCP request lost -- the service published 21 s after the
	 * address). A station's frames all go to its access point: any destination will do (brcmfmac: the first frame's).
	 */
	if (up && en0_live && tx.state == 0) {
		flow_create(everyone);
	}
	if (en0_attached && en0_live && up != en0_link_said) {
		en0_link_said = up;
		hakios_net_link_state(up);
		printf("wifi: en0: link %s\n", up ? "UP (associated, keys set): frames pass" : "down");
	}
}

static void
flow_created(int16_t status, uint16_t ring)
{
	tx.state = status == 0 ? 2 : -1;
	printf("wifi: en0: flow ring %u %s (status %d)\n", ring, status == 0 ? "created" : "REFUSED", status);
}

/* FLOW_RING_DELETE (brcmfmac's msgbuf_tx_flowring_delete_req: the ring's id at +8, a reason at +10) */
static void
flow_delete(void)
{
	uint8_t *item = h2d_reserve(RING_H2D_CTRL);

	if (item == NULL) {
		return;
	}
	item[0] = MSG_FLOW_RING_DELETE;
	item[1] = 0;
	put16(item, 8, FLOW_H2D_INDEX);
	tx.state = 3;
	h2d_commit(RING_H2D_CTRL, 1);
	printf("wifi: en0: flow ring %u to be deleted (the link is down)\n", FLOW_H2D_INDEX);
}

/* the ring gone: our side of it emptied (frames never sent are given back), ready for a new one at the next join */
static void
flow_deleted(int16_t status, uint16_t ring)
{
	tx.w = 0;
	mb.idx[mb.h2d_w + FLOW_H2D_INDEX] = 0;
	bzero(tx.busy, sizeof(tx.busy));
	tx.outstanding = 0;
	tx.state = 0;
	printf("wifi: en0: flow ring %u deleted (status %d)\n", ring, status);
	en0_link_update();                      /* (up again meanwhile: a new ring at once) */
}

/* the flow ring's room for one more (its read index: the array the chip DMAs, else the chip's own) */
static uint8_t *
flow_reserve(void)
{
	uint16_t next = (uint16_t)((tx.w + 1) % FLOW_ITEMS), r = (uint16_t)mb.idx[mb.h2d_r + FLOW_H2D_INDEX];

	if (next == r && (mb.tcm_h2d_r == 0 || next == (uint16_t)mem_rd(mb.tcm_h2d_r + 4 * FLOW_H2D_INDEX))) {
		return NULL;
	}
	uint8_t *item = tx.ring + (uint32_t)tx.w * FLOW_ITEM_SIZE;

	bzero(item, FLOW_ITEM_SIZE);
	return item;
}

static void
flow_create(const uint8_t *da)
{
	uint8_t *item = h2d_reserve(RING_H2D_CTRL);

	if (item == NULL) {
		return;
	}
	item[0] = MSG_FLOW_RING_CREATE;
	item[1] = 0;                                    /* interface 0 */
	memcpy(item + 8, da, 6);
	memcpy(item + 14, tx.address, 6);
	item[20] = 0;                                   /* tid: best effort */
	put16(item, 22, FLOW_H2D_INDEX);
	put16(item, 28, FLOW_ITEMS);
	put16(item, 30, FLOW_ITEM_SIZE);
	put32(item, 32, (uint32_t)tx.ring_pa);
	put32(item, 36, (uint32_t)(tx.ring_pa >> 32));
	tx.state = 1;
	h2d_commit(RING_H2D_CTRL, 1);
	printf("wifi: en0: flow ring %u asked for (%u x %u bytes at 0x%llx)\n", FLOW_H2D_INDEX, FLOW_ITEMS, FLOW_ITEM_SIZE,
	    tx.ring_pa);
}

static bool
en0_transmit(mbuf_t packet, size_t length)
{
	uint8_t head[14], *item, *buf;
	unsigned b;
	bool ok = false;

	if (!en0_live || length < 14 || length > TX_BUF_SIZE || mbuf_copydata(packet, 0, 14, head) != 0) {
		return false;
	}
	lck_mtx_lock(wl_lock);
	if (tx.state == 0 && link_up) {
		flow_create(head);                      /* (the first frame waits for the ring: dropped, the stack retries) */
	}
	if (tx.state != 2) {
		goto out;
	}
	for (b = 0; b < TX_BUFS && (tx.busy[b / 32] & (1u << (b % 32))); b++) {
	}
	if (b == TX_BUFS || (item = flow_reserve()) == NULL) {
		goto out;
	}
	buf = tx.buf + (uint32_t)b * TX_BUF_SIZE;
	if (mbuf_copydata(packet, 0, length, buf) != 0) {
		goto out;
	}
	__asm__ volatile ("dsb sy" ::: "memory");
	tx.busy[b / 32] |= 1u << (b % 32);
	item[0] = MSG_TX_POST;
	item[1] = 0;
	put32(item, 4, TX_ID_BASE + b);
	memcpy(item + 8, buf, 14);
	item[22] = 0x01;                                /* an 802.3 frame, priority 0 */
	item[23] = 1;                                   /* one segment */
	put32(item, 32, (uint32_t)(tx.buf_pa + (uint64_t)b * TX_BUF_SIZE + 14));
	put32(item, 36, (uint32_t)((tx.buf_pa + (uint64_t)b * TX_BUF_SIZE + 14) >> 32));
	put16(item, 42, (uint16_t)(length - 14));
	tx.w = (uint16_t)((tx.w + 1) % FLOW_ITEMS);
	mb.idx[mb.h2d_w + FLOW_H2D_INDEX] = tx.w;
	__asm__ volatile ("dsb sy" ::: "memory");
	pcie_wr(PCIE_H2D_MAILBOX_0, 1);
	tx.sent++;
	tx.outstanding++;
	ok = true;
out:
	if (!ok) {
		tx.dropped++;
	}
	lck_mtx_unlock(wl_lock);
	return ok;
}

/* the tx completion ring: each sent frame's buffer given back */
static unsigned
d2h_tx_poll(void)
{
	unsigned n = 0;
	uint16_t w = (uint16_t)mb.idx[mb.d2h_w + (RING_D2H_TXCPL - 2)];

	if (tx.buf == NULL || w >= ring_items[RING_D2H_TXCPL]) {
		return 0;
	}
	while (mb.r[RING_D2H_TXCPL] != w) {
		const uint8_t *m = mb.ring[RING_D2H_TXCPL] + (uint32_t)mb.r[RING_D2H_TXCPL] * ring_item_size[RING_D2H_TXCPL];
		uint32_t id = get32(m, 4);

		if (m[0] == MSG_TX_STATUS && id >= TX_ID_BASE && id < TX_ID_BASE + TX_BUFS) {
			unsigned b = id - TX_ID_BASE;

			if (tx.busy[b / 32] & (1u << (b % 32))) {
				tx.busy[b / 32] &= ~(1u << (b % 32));
				tx.outstanding--;
			}
			tx.done++;
		}
		mb.r[RING_D2H_TXCPL] = (uint16_t)((mb.r[RING_D2H_TXCPL] + 1) % ring_items[RING_D2H_TXCPL]);
		mb.idx[mb.d2h_r + (RING_D2H_TXCPL - 2)] = mb.r[RING_D2H_TXCPL];
		n++;
	}
	if (n) {
		d2h_read_doorbell(RING_D2H_TXCPL, mb.r[RING_D2H_TXCPL]);
	}
	return n;
}

/*
 * wifi_counters=N (a diagnosis, 11-12, TODO 35e): every N seconds the firmware's own "counters" (wl_cnt_info_t: xtlvs, the
 * WLC layer's wl_cnt_wlc_t at id 0x100, 32-bit words) -- what the radio took in and why it dropped it: no buffer, not
 * decrypted, filtered. Called on en0's thread without the rings' lock (the ioctl takes it).
 */
static void
counters_watch(void)
{
	static uint32_t every = ~0u;
	static uint64_t at;
	static uint8_t buf[4096];

	if (every == ~0u) {
		every = 0;
		PE_parse_boot_argn("wifi_counters", &every, sizeof(every));
	}
	if (every == 0 || fw_halted || recovering || micros_since(at) < every * 1000000ull) {
		return;
	}
	at = mach_absolute_time();
	bzero(buf, sizeof(buf));
	int err = iovar_get("counters", buf, sizeof(buf));
	uint32_t datalen = get16(buf, 2), off = 4;

	while (err == 0 && off + 4 <= 4 + datalen && off + 4 <= sizeof(buf)) {
		uint32_t id = get16(buf, off), len = get16(buf, off + 2);

		if (id == 0x100 && len >= 72 * 4 && off + 4 + len <= sizeof(buf)) {
			const uint8_t *c = buf + off + 4;
#define CNT(i) get32(c, 4 * (i))
			printf("wifi: counters: tx %u (fail %u, retrans %u, nobuf %u); rx %u frames %u bytes, error %u, ctl %u, NOBUF %u, "
			    "nondata %u, bad da %u, filter %u, overflow %u, frag %u, multi %u, crc %u, dup %u; UNDECRYPTED %u (ccmp %u), "
			    "decrypted %u\n", CNT(0), CNT(50), CNT(2), CNT(7), CNT(15), CNT(16), CNT(17), CNT(18), CNT(19), CNT(20),
			    CNT(29), CNT(30), CNT(31), CNT(57), CNT(58), CNT(59), CNT(53), CNT(61), CNT(67), CNT(71));
#undef CNT
			return;
		}
		off += 4 + ((len + 3) & ~3u);
	}
	printf("wifi: counters: %d, version %u, %u bytes: no WLC counters found\n", err, get16(buf, 0), datalen);
}

static bool intclear_masked_now;

/* (wifi_intclear=2: whether this poll masked the chip's interrupts as bcmdhd's ISR does, to unmask after the rings) */
static bool
intclear_masked(void)
{
	bool m = intclear_masked_now;

	intclear_masked_now = false;
	return m;
}

/*
 * wifi_dumps=a,b,c (a diagnosis, 11-12, TODO 35e): the firmware's own "dump" iovar for each named section ("wl dump
 * NAME": its state as text), once, 60 s after en0 carries frames -- printed line by line. Called without the lock.
 */
static void
dumps_once(void)
{
	static char names[96];
	static int state;                       /* 0 not read, 1 waiting, 2 done */
	static uint64_t live_at;
	static uint8_t out[4096];

	if (state == 0) {
		state = PE_parse_boot_arg_str("wifi_dumps", names, sizeof(names)) && names[0] ? 1 : 2;
		live_at = mach_absolute_time();
	}
	if (state != 1 || fw_halted || recovering || micros_since(live_at) < 120000000ull) {
		return;
	}
	state = 2;
	for (char *name = names; *name != 0;) {
		char *end = name;
		uint8_t req[64];

		while (*end != 0 && *end != ',') {
			end++;
		}
		size_t n = (size_t)(end - name);
		if (n > 0 && n < 40) {
			bzero(req, sizeof(req));
			memcpy(req, "dump", 5);
			memcpy(req + 5, name, n);
			bzero(out, sizeof(out));
			int err = bcm_ioctl(WLC_GET_VAR, req, sizeof(out), out, sizeof(out) - 1);
			printf("wifi: dump %.*s: %d\n", (int)n, name, err);
			if (err == 0) {
				char *line = (char *)out;
				for (unsigned k = 0; k < 120 && *line != 0; k++) {
					char *nl = line;
					while (*nl != 0 && *nl != '\n') {
						nl++;
					}
					printf("wifi: dump %.*s| %.*s\n", (int)n, name, (int)(nl - line > 200 ? 200 : nl - line), line);
					line = *nl ? nl + 1 : nl;
				}
			}
		}
		name = *end ? end + 1 : end;
	}
}

/* the interface's thread, every 2-10 ms: the completion rings (events, received frames, sent ones); a line every 30 s */
static unsigned
en0_poll(void)
{
	static uint64_t said_at;
	unsigned n;

	if (!en0_live) {
		return 0;                       /* (en0 from the start: the steps poll the rings themselves until step 8) */
	}
	lck_mtx_lock(wl_lock);
	{
		/*
		 * The chip's interrupt status (PCIe core MailboxInt), answered as an interrupt handler would: read, written back
		 * (11-12, TODO 35e, wifi_intclear=0: not). Without it the status set at the first doorbell stayed set for the
		 * whole run -- and after the join the firmware handed up one frame, then none for minutes while it took in and
		 * decrypted ~1 a second (its counters), until other traffic of its own (its event logs, the legacy way) pushed
		 * them out; with the EDL ring, never.
		 */
		static uint32_t intclear = ~0u;
		static unsigned cleared;

		if (intclear == ~0u) {
			intclear = 1;
			PE_parse_boot_argn("wifi_intclear", &intclear, sizeof(intclear));
		}
		static uint32_t st;

		st = 0;
		if (intclear != 0 && !fw_halted) {
			st = pcie_rd(PCIE_MAILBOX_INT);
			if (st != 0 && st != ~0u) {
				if (intclear == 2) {
					pcie_wr(PCIE_MAILBOX_MASK, 0);  /* (2: bcmdhd's ISR -- mask off, status cleared, DPC, mask on) */
					intclear_masked_now = true;
				}
				pcie_wr(PCIE_MAILBOX_INT, st);
				if (cleared++ < 4 || (cleared & (cleared - 1)) == 0) {
					printf("wifi: en0: the chip's interrupt status 0x%08x cleared (%u times)\n", st, cleared);
				}
			} else {
				st = 0;
			}
		}
	}
	if (msi_word != NULL && *msi_word != 0) {
		if (msi_seen++ < 3) {
			printf("wifi: en0: an MSI (0x%x)\n", *msi_word);
		}
		*msi_word = 0;
	}
	n = d2h_ctrl_poll();
	if (intclear_masked()) {
		pcie_wr(PCIE_MAILBOX_MASK, 0xffff);     /* (the DPC done: PCIE_MB_D2H_MB_MASK on again) */
	}
	{
		/* the signal of the network joined, every 5 s (wifid's bars, RSSI) */
		static uint64_t looked_at;
		static struct hakios_sta_bss now;

		if (sta_phase == HAKIOS_STA_JOINED && micros_since(looked_at) > 5000000) {
			looked_at = mach_absolute_time();
			if (bss_now(&now, true) && memcmp(now.bssid, sta_bss.bssid, 6) == 0) {
				sta_bss.rssi = now.rssi;
			}
		}
	}
	{
		/*
		 * Diagnosis (11-11, TODO 35f: the firmware runs out of packet ids ~130 s after it starts, idle or not):
		 * wifi_poolwatch=N -- every N seconds the firmware's own packet-id variables read straight from its memory
		 * (fw_bcmdhd.map_4383_a3, THIS build only: total_pool_pktid_count, hnd_pktid_max, the failure count, the map's
		 * first words); wifi_console=N -- its console's new lines every N seconds.
		 */
		static uint32_t poolwatch = ~0u, console_every;
		static uint64_t pool_at, console_at;

		if (poolwatch == ~0u) {
			poolwatch = 0;
			PE_parse_boot_argn("wifi_poolwatch", &poolwatch, sizeof(poolwatch));
			PE_parse_boot_argn("wifi_console", &console_every, sizeof(console_every));
		}
		if (poolwatch != 0 && !fw_halted && micros_since(pool_at) > poolwatch * 1000000ull) {
			uint32_t map = mem_rd(0x006e01d8), w[6] = { 0 };

			pool_at = mach_absolute_time();
			if (map >= ram_base && map + 24 <= ram_base + ram_size) {
				for (int i = 0; i < 6; i++) {
					w[i] = mem_rd(map + 4 * i);
				}
			}
			printf("wifi: pool: total_pool_pktid_count %u, hnd_pktid_max %u, failures %u, map at 0x%08x: %08x %08x %08x "
			    "%08x %08x %08x; %u frames in, %u events\n", mem_rd(0x006e0108), mem_rd(0x006e01dc), mem_rd(0x006e01d4), map,
			    w[0], w[1], w[2], w[3], w[4], w[5], mb.rx_frames, mb.events);
			/* the firmware's packet pools (pktpool_t, hnd_pktpool.h: inited/type/id/istx, a mutex, freelist, avail,
			 * n_pkts, maxlen, max_pkt_bytes ...): which one grows as the free ids fall? and its heap's free memory */
			static const struct { uint32_t at; const char *name; } pools[] = {
				{ 0x006e01f0, "resv_lfrag" }, { 0x006e01f4, "shared" }, { 0x006e01f8, "shared_lfrag" },
				{ 0x006e01fc, "shared_rxlfrag" }, { 0x006e02d8, "pktfetch" },
			};
			for (unsigned k = 0; k < sizeof(pools) / sizeof(pools[0]); k++) {
				uint32_t pp = mem_rd(pools[k].at), v[8] = { 0 };

				if (pp >= ram_base && pp + 32 <= ram_base + ram_size) {
					for (int i = 0; i < 8; i++) {
						v[i] = mem_rd(pp + 4 * i);
					}
				}
				printf("wifi: pool %s at 0x%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n", pools[k].name, pp, v[0], v[1],
				    v[2], v[3], v[4], v[5], v[6], v[7]);
			}
			printf("wifi: heap: in use %u, free %u, in use high-water %u\n", mem_rd(0x006e029c), mem_rd(0x006e02a8),
			    mem_rd(0x006e0294));
			{
				/*
				 * (11-12, TODO 35e) the firmware's rx completion ids (g_rxcplid_list, a pointer: max, avail, ... --
				 * bcm_rxcplid_list_t): each frame handed up takes one until the firmware sees our read index pass its
				 * completion (pciedev_freeup_rxcplid). Do they run out after the join? And our D2H read indices as
				 * the chip's own array has them (ctrl, txcpl, rxcpl).
				 */
				uint32_t l = mem_rd(0x006e01b4), v[6] = { 0 };

				if (l >= ram_base && l + 24 <= ram_base + ram_size) {
					for (int i = 0; i < 6; i++) {
						v[i] = mem_rd(l + 4 * i);
					}
				}
				printf("wifi: the firmware's _pciedev_hostready %08x, _pciedevenab byte %02x, dongle_bus 0x%08x, logtrace "
				    "0x%08x\n", mem_rd(0x006e0118), (mem_rd(0x006e0134) >> 8) & 0xff, mem_rd(0x006e0220), mem_rd(0x006e0180));
				printf("wifi: rxcplids at 0x%08x: %08x %08x %08x %08x %08x %08x; the chip's D2H read indices %u %u %u "
				    "(ours %u %u %u)\n", l, v[0], v[1], v[2], v[3], v[4], v[5],
				    mb.tcm_d2h_r ? mem_rd(mb.tcm_d2h_r + 0) & 0xffff : 0, mb.tcm_d2h_r ? mem_rd(mb.tcm_d2h_r + 4) & 0xffff : 0,
				    mb.tcm_d2h_r ? mem_rd(mb.tcm_d2h_r + 8) & 0xffff : 0, mb.r[RING_D2H_CTRL], mb.r[RING_D2H_TXCPL],
				    mb.r[RING_D2H_RXCPL]);
			}
			/* every common ring's indices as the chip leaves them in host memory (and ours): where do frames go? */
			printf("wifi: rings: D2H write ctrl %u txcpl %u rxcpl %u (+%u), ours %u %u %u; H2D read ctrl %u rxpost %u flow %u, "
			    "ours %u %u %u; rx posted %u, rx bad %u; debug info %u back (%u bytes)\n", mb.idx[mb.d2h_w + 0], mb.idx[mb.d2h_w + 1], mb.idx[mb.d2h_w + 2],
			    mb.idx[mb.d2h_w + 3], mb.r[RING_D2H_CTRL], mb.r[RING_D2H_TXCPL], mb.r[RING_D2H_RXCPL],
			    mb.idx[mb.h2d_r + RING_H2D_CTRL], mb.idx[mb.h2d_r + RING_H2D_RXPOST], mb.idx[mb.h2d_r + 2],
			    mb.w[RING_H2D_CTRL], mb.w[RING_H2D_RXPOST], mb.idx[mb.h2d_w + 2], mb.rx_posted, mb.rx_bad, info.completions,
			    info.bytes);
		}
		if (console_every != 0 && !fw_halted && micros_since(console_at) > console_every * 1000000ull) {
			console_at = mach_absolute_time();
			console_say(mb.shared);
		}
	}
	if (micros_since(said_at) > 30000000) {
		said_at = mach_absolute_time();
		printf("wifi: en0: %u frames sent (%u done, %u dropped, %u out), %u received (%u bytes); link %s, keys %s%s\n",
		    tx.sent, tx.done, tx.dropped, tx.outstanding, mb.rx_frames, mb.rx_bytes, link_up ? "up" : "down",
		    keyed ? "set" : "not set", msi_word != NULL ? " (MSIs counted)" : "");
		if (msi_word != NULL) {
			printf("wifi: en0: %u MSIs so far\n", msi_seen);
		}
		if (!fw_halted && mb.tcm_d2h_w != 0) {
			/*
			 * (11-12, TODO 35e: since the EDL ring, ~10 frames a minute come in where 400 did before -- iOS "offline":
			 * no ARP answer, no DNS. Whether the chip has written rx completions it has not DMAed the index of: its
			 * own write index of the rx completion ring beside the copy in our memory; the rx post ring likewise.)
			 */
			char types[8 * 16];
			unsigned at = 0;

			types[0] = 0;
			for (unsigned i = 0; i < 8 && mb.rx_type_count[i] != 0; i++) {
				at += (unsigned)snprintf(types + at, sizeof(types) - at, " 0x%04x:%u", mb.rx_types[i], mb.rx_type_count[i]);
			}
			printf("wifi: en0: rx completions: the chip's write index %u, its copy here %u, ours read %u; rx post: "
			    "written %u, read %u (the chip's own %u); %u posted, %u bad; ctrl %u/%u; by ethertype:%s\n",
			    mem_rd(mb.tcm_d2h_w + 4 * (RING_D2H_RXCPL - 2)) & 0xffff, mb.idx[mb.d2h_w + (RING_D2H_RXCPL - 2)],
			    mb.r[RING_D2H_RXCPL], mb.w[RING_H2D_RXPOST], mb.idx[mb.h2d_r + RING_H2D_RXPOST],
			    mem_rd(mb.tcm_h2d_r + 4 * RING_H2D_RXPOST) & 0xffff, mb.rx_posted, mb.rx_bad,
			    mem_rd(mb.tcm_d2h_w + 4 * (RING_D2H_CTRL - 2)) & 0xffff, mb.idx[mb.d2h_w + (RING_D2H_CTRL - 2)],
			    at ? types : " none");
		}
	}
	lck_mtx_unlock(wl_lock);
	counters_watch();
	dumps_once();
	return n;
}

static struct hakios_net_link en0_link = {
	.name = "BCM4383",
	.unit = 0,
	.wireless = true,
	.up = false,
	.transmit = en0_transmit,
	.poll = en0_poll,
};

/*
 * EN0 FROM THE START (wifi=8 or more; boot-arg wifi_early_en0=0: at step 8 instead): hakios_net_init calls this as it
 * makes the interfaces there are, long before the radio's firmware is up -- an iPhone's en0 is there before configd
 * starts, and configd makes its network services of the interfaces it finds (11-10, sta1/sta2: with en0 coming at
 * ~87 s, configd's store had no network service at all and no global IPv4). The address the device tree's (step 7
 * gives the chip the same); the link down, nothing polled or sent until step 8 has the rings and the flow ring
 * (en0_live).
 */
void hakios_akita_wifi_early(void);

void
hakios_akita_wifi_early(void)
{
	uint32_t early = 1;
	DTEntry chosen;
	void const *mac = NULL;
	unsigned int size = 0;

	if (!PE_parse_boot_argn("wifi", &steps, sizeof(steps)) || steps < 8 ||
	    (PE_parse_boot_argn("wifi_early_en0", &early, sizeof(early)) && early == 0) ||
	    (PE_parse_boot_argn("wifi_mac", &early, sizeof(early)) && early == 0)) {
		return;
	}
	if (SecureDTLookupEntry(NULL, "/chosen", &chosen) != kSuccess ||
	    SecureDTGetProperty(chosen, "mac-address-wifi0", &mac, &size) != kSuccess || size != 6) {
		printf("wifi: en0: no mac-address-wifi0 in the device tree: en0 comes at step 8\n");
		return;
	}
	if (wl_lock == NULL) {
		wl_group = lck_grp_alloc_init("akita_wifi", LCK_GRP_ATTR_NULL);
		wl_lock = lck_mtx_alloc_init(wl_group, LCK_ATTR_NULL);
	}
	memcpy(tx.address, mac, 6);
	memcpy(en0_link.address, mac, 6);
	if (!hakios_net_attach(&en0_link)) {
		printf("wifi: en0: taken already\n");
		return;
	}
	en0_attached = true;
	printf("wifi: en0 there from the start (the device tree's address), the radio behind it from step 8\n");
}

static void
en0_attach(void)
{
	uint8_t mac[6];

	bzero(mac, sizeof(mac));
	if (iovar_get("cur_etheraddr", mac, sizeof(mac)) != 0) {
		printf("wifi: 8: no address from the chip: stopped\n");
		return;
	}
	if (tx.ring == NULL) {
		tx.ring = (uint8_t *)dma_piece(FLOW_ITEMS * FLOW_ITEM_SIZE, &tx.ring_pa);
	}
	if (tx.buf == NULL) {
		tx.buf = (uint8_t *)dma_piece(TX_BUFS * TX_BUF_SIZE, &tx.buf_pa);
	}
	if (tx.ring == NULL || tx.buf == NULL) {
		printf("wifi: 8: no memory in one piece for the flow ring and the send buffers: stopped\n");
		return;
	}
	if (en0_attached) {
		if (memcmp(tx.address, mac, 6) != 0) {
			printf("wifi: 8: the chip's address is not en0's (%02x:..:%02x, en0 %02x:..:%02x)\n", mac[0], mac[5], tx.address[0],
			    tx.address[5]);
		}
	} else {
		memcpy(tx.address, mac, 6);
		memcpy(en0_link.address, mac, 6);
		if (!hakios_net_attach(&en0_link)) {
			printf("wifi: 8: en0 is taken already: stopped\n");
			return;
		}
		en0_attached = true;
	}
	lck_mtx_lock(wl_lock);
	en0_live = true;
	en0_link_update();
	lck_mtx_unlock(wl_lock);
	printf("wifi: 8: en0 on the BCM4383 (address %02x:xx:xx:xx:xx:%02x), %u send buffers; link %s\n", tx.address[0],
	    tx.address[5], TX_BUFS, link_up && keyed ? "up" : "down until joined");
}

/*
 * ---- the station: what Darwin's Wi-Fi side asks for (sys/hakios.sta.h, through drivers/wifi/hakios.sta.c) ----
 *
 * The same thing the WCN6750's station does on redwood, for this chip: the radio on and off (WLC_UP, WLC_DOWN), a scan
 * (the escan of step 7, for a name or some channels if asked, every access point kept with its information elements --
 * wifid reads a network's protection out of them), a join (the firmware's own supplicant, as step 8 did: WPA2-PSK with
 * the passphrase -- or a PMK, handed over as its 64 hex digits, which the firmware takes for a key -- or an open
 * network), a leave (WLC_DISASSOC). All called on hakios.sta.c's thread, one at a time; the rings' lock taken for each
 * look, not across a whole scan or join (en0's frames keep coming meanwhile). A join's outcome as apple80211.c reads it:
 * 0; EAUTH when the access point took us but the keys were never set (a wrong passphrase); ETIMEDOUT; ENOENT for no
 * such network; EIO otherwise. A network lost by itself (deauthenticated, the link down) is said with hakios_sta_lost.
 */
#define STA_HEARD_MOST          64

static struct hakios_sta_bss *sta_heard;
static unsigned sta_heard_count;
/* (sta_collect, declared above: an escan of the station's, its results kept whole) */
static volatile int sta_phase = HAKIOS_STA_OFF;
static struct hakios_sta_bss sta_bss;           /* the network joined or being joined */
static int sta_last_error;
static bool sta_leaving;                        /* a leave asked: the link going down is not a loss */
static bool radio_up;

static unsigned
chan_to_freq(unsigned ch, uint16_t band)
{
	return band == 0 ? (ch == 14 ? 2484 : 2407 + 5 * ch) : band == 0x8000 ? 5950 + 5 * ch : 5000 + 5 * ch;
}

/* one access point of an escan's result (a bss_info: control channel at 88 when n_cap at 81, IEs at 116/120) */
static void bss_fill(struct hakios_sta_bss *x, const uint8_t *b, uint32_t blen);

static void
sta_heard_add(const uint8_t *b, uint32_t blen)
{
	struct hakios_sta_bss *x = NULL;
	int16_t rssi = (int16_t)get16(b, 78);
	unsigned i;

	for (i = 0; i < sta_heard_count; i++) {
		if (memcmp(sta_heard[i].bssid, b + 8, 6) == 0) {
			if (rssi <= sta_heard[i].rssi) {
				return;
			}
			x = &sta_heard[i];
			break;
		}
	}
	if (x == NULL) {
		if (sta_heard_count >= STA_HEARD_MOST) {
			return;
		}
		x = &sta_heard[sta_heard_count++];
	}
	bss_fill(x, b, blen);
}

/* a bss_info (escan result, WLC_GET_BSS_INFO) as the station's record */
static void
bss_fill(struct hakios_sta_bss *x, const uint8_t *b, uint32_t blen)
{
	uint16_t chanspec = get16(b, 72), ie_off = get16(b, 116);
	uint32_t ie_len = get32(b, 120);

	bzero(x, sizeof(*x));
	memcpy(x->bssid, b + 8, 6);
	x->ssid_length = b[18] > 32 ? 32 : b[18];
	memcpy(x->ssid, b + 19, x->ssid_length);
	x->frequency = (uint16_t)chan_to_freq(b[81] != 0 ? b[88] : (chanspec & 0xff), chanspec & 0xc000);
	x->rssi = (int16_t)get16(b, 78);
	x->beacon_interval = get16(b, 14);
	x->capability = get16(b, 16);
	if (ie_off >= 90 && ie_off <= blen && ie_len <= blen - ie_off && ie_len <= HAKIOS_STA_ELEMENTS_MOST) {
		memcpy(x->elements, b + ie_off, ie_len);
		x->elements_length = (uint16_t)ie_len;
	}
}

/* the network joined, as the chip has it now (WLC_GET_BSS_INFO: a length, then its bss_info); LOCKED: the rings' lock held */
static bool
bss_now(struct hakios_sta_bss *x, bool locked)
{
	static uint8_t buf[2048];               /* (one for the station's thread and en0's poll: used under the lock only) */
	uint32_t blen;
	int err;

	if (!locked) {
		bool ok;

		wl_take();
		ok = bss_now(x, true);
		wl_give();
		return ok;
	}
	bzero(buf, sizeof(buf));
	put32(buf, 0, sizeof(buf));
	err = bcm_ioctl_locked(136, buf, 4, buf, sizeof(buf));
	blen = get32(buf, 4 + 4);
	if (err != 0 || blen < 128 || blen > sizeof(buf) - 4) {
		return false;
	}
	bss_fill(x, buf + 4, blen);
	return true;
}

/* frequency -> a 20 MHz chanspec (2.4 GHz 0x0000, 5 GHz 0xc000, 6 GHz 0x8000; bandwidth 20 0x1000) */
static uint16_t
freq_to_chanspec(unsigned f)
{
	if (f >= 2412 && f <= 2484) {
		return (uint16_t)(0x1000 | (f == 2484 ? 14 : (f - 2407) / 5));
	}
	if (f >= 5955 && f <= 7115) {
		return (uint16_t)(0x8000 | 0x1000 | (f - 5950) / 5);
	}
	return (uint16_t)(0xc000 | 0x1000 | ((f - 5000) / 5 & 0xff));
}

/* an escan (scan parameters v2), its results awaited; WHAT NULL: every channel, every network (step 7) */
static int
escan_run(const struct hakios_sta_scan *what)
{
	uint8_t p[8 + 72 + 2 * 64];
	uint8_t *v = p + 8;
	unsigned nch = 0;
	int err;

	bzero(p, sizeof(p));
	put32(p, 0, 2);
	put16(p, 4, 1);                 /* start */
	put16(p, 6, 0x1234);
	put16(v, 0, 2);
	if (what != NULL && what->ssid_length != 0 && what->ssid_length <= 32) {
		put32(v, 4, what->ssid_length);
		memcpy(v + 8, what->ssid, what->ssid_length);
	}
	memset(v + 40, 0xff, 6);        /* any BSSID */
	v[46] = 2;                      /* any BSS type */
	put32(v, 48, what != NULL && what->passive ? 1 : 0);
	put32(v, 52, 0xffffffffu);
	put32(v, 56, what != NULL && what->dwell_ms && !what->passive ? what->dwell_ms : 0xffffffffu);
	put32(v, 60, what != NULL && what->dwell_ms && what->passive ? what->dwell_ms : 0xffffffffu);
	put32(v, 64, 0xffffffffu);
	for (unsigned i = 0; what != NULL && i < what->frequency_count && i < 64; i++) {
		put16(v, 72 + 2 * nch, freq_to_chanspec(what->frequencies[i]));
		nch++;
	}
	put32(v, 68, nch);
	put16(v, 2, (uint16_t)(72 + 2 * nch));
	wl_take();
	heard_count = 0;
	sta_heard_count = 0;
	sta_collect = sta_heard != NULL;
	escan_done = false;
	escan_status = -1;
	wl_give();
	err = iovar_set("escan", p, 8 + 72 + 2 * nch);
	if (err != 0) {
		sta_collect = false;
		return EIO;
	}
	uint64_t t0 = mach_absolute_time();

	while (!escan_done && micros_since(t0) < 15000000) {
		unsigned n;

		wl_take();
		n = d2h_ctrl_poll();
		wl_give();
		if (n == 0) {
			sleep_ms(5);
		}
	}
	sta_collect = false;
	return escan_done ? (escan_status == 0 ? 0 : EIO) : ETIMEDOUT;
}

/* boot-arg wifi_soft_power=1: the radio never WLC_DOWN (off = out of the network only), a test -- does WLC_DOWN drop the
 * rx buffers we posted (11-10: the firmware ran out of packet ids seconds after every re-join)? */
static int soft_power = -1;

static bool
soft(void)
{
	if (soft_power < 0) {
		uint32_t v = 0;

		PE_parse_boot_argn("wifi_soft_power", &v, sizeof(v));
		soft_power = v != 0;
	}
	return soft_power;
}

static int
sta_power(bool on)
{
	int err;

	if (fw_halted || recovering) {
		return on ? EIO : 0;
	}

	if (on == radio_up) {
		return 0;
	}
	if (!on) {
		sta_leaving = true;
		(void)cmd_int(52, 0);           /* WLC_DISASSOC */
	}
	err = soft() ? 0 : cmd_int(on ? 2 : 3, 1);     /* WLC_UP / WLC_DOWN */
	sta_leaving = false;
	if (err != 0) {
		return EIO;
	}
	radio_up = on;
	sta_phase = on ? HAKIOS_STA_IDLE : HAKIOS_STA_OFF;
	printf("wifi: station: the radio %s\n", on ? "on" : "off");
	return 0;
}

static int
sta_scan(const struct hakios_sta_scan *what)
{
	int err, was = sta_phase;

	if (fw_halted || recovering) {
		return EIO;
	}
	if (!radio_up) {
		return ENETDOWN;
	}
	if (was == HAKIOS_STA_IDLE) {
		sta_phase = HAKIOS_STA_SCANNING;
	}
	err = escan_run(what);
	if (sta_phase == HAKIOS_STA_SCANNING) {
		sta_phase = was;
	}
	printf("wifi: station: a scan%s%s: %d, %u access points\n", what->ssid_length ? " for a name" : "",
	    what->frequency_count ? " on some channels" : "", err, sta_heard_count);
	return err;
}

static void
sta_heard_each(void (*each)(const struct hakios_sta_bss *bss, void *context), void *context)
{
	for (unsigned i = 0; i < sta_heard_count; i++) {
		each(&sta_heard[i], context);
	}
}

/*
 * Out of the network, and the radio down and up again: a new association right after a WLC_DISASSOC made the firmware
 * run out of packet ids ("lb_alloc: hnd_pktid_free_cnt 0") and trap, three times on 11-10 (a wrong passphrase, a second
 * join of the same network); off and on first, the way the radio's switch goes, it joins every time (toggle1).
 */
static void
sta_radio_cycle(void)
{
	sta_leaving = true;
	(void)cmd_int(52, 0);                   /* WLC_DISASSOC */
	for (unsigned i = 0; i < 100 && link_up; i++) {
		wl_take();
		(void)d2h_ctrl_poll();
		wl_give();
		sleep_ms(10);
	}
	if (!soft()) {
		(void)cmd_int(3, 1);            /* WLC_DOWN */
		(void)cmd_int(2, 1);            /* WLC_UP */
	}
	sta_leaving = false;
}

static struct hakios_sta_join sta_joined_with;  /* the request the network joined came from (to know a repeat of it) */

static int
sta_join(const struct hakios_sta_join *j)
{
	uint8_t pmk[2 + 2 + 256 + 2], params[52];
	bool psk = j->security == HAKIOS_STA_WPA2_PSK, any_bssid = true;
	int err, result;

	if (fw_halted || recovering) {
		return EIO;
	}
	if (!radio_up) {
		return ENETDOWN;
	}
	if (j->ssid_length == 0 || j->ssid_length > 32) {
		return EINVAL;
	}
	/* the network we are on, asked for again with the same key (the standing request at every power-on, wifid's own
	 * join of it): nothing to do -- leaving and joining again is what made the firmware trap (toggle1) */
	if (sta_phase == HAKIOS_STA_JOINED && link_up && keyed && memcmp(j, &sta_joined_with, sizeof(*j)) == 0) {
		printf("wifi: station: join of the network we are on, with the same key: nothing to do\n");
		return 0;
	}
	if (sta_phase == HAKIOS_STA_JOINED || link_up) {
		sta_radio_cycle();
	}
	sta_phase = HAKIOS_STA_JOINING;
	bzero(&sta_bss, sizeof(sta_bss));
	memcpy(sta_bss.ssid, j->ssid, j->ssid_length);
	sta_bss.ssid_length = j->ssid_length;
	for (unsigned i = 0; i < 6; i++) {
		any_bssid = any_bssid && j->bssid[i] == 0;
	}
	err = iovar_int("wpa_auth", psk ? 0x80 : 0) | iovar_int("auth", 0) | iovar_int("wsec", psk ? 4 : 0) |
	    iovar_int("sup_wpa", psk ? 1 : 0);
	if (err == 0 && psk) {
		/*
		 * WLC_SET_WSEC_PMK (wsec_pmk_t: key_len, flags, key): the passphrase with WSEC_PASSPHRASE (1: the firmware
		 * hashes it with the SSID), or a PMK as its 32 raw bytes with flags 0 -- what bcmdhd's wl_cfg80211 does with
		 * the PMK a supplicant hands it. wifid joins from Settings with the PMK it derived (32 bytes, "kind 6"): this
		 * driver sent it as 64 hex digits WITH flag 1, so the firmware hashed the digits as if they were a passphrase,
		 * got another key, and the handshake failed -- "incorrect password" for the right one (the tester, 11-11).
		 */
		bzero(pmk, sizeof(pmk));
		if (j->passphrase_length >= 8 && j->passphrase_length <= 63) {
			put16(pmk, 0, j->passphrase_length);
			put16(pmk, 2, 1);               /* WSEC_PASSPHRASE */
			memcpy(pmk + 4, j->passphrase, j->passphrase_length);
		} else {
			put16(pmk, 0, 32);
			put16(pmk, 2, 0);
			memcpy(pmk + 4, j->pmk, 32);
		}
		err = bcm_ioctl(268, pmk, sizeof(pmk), NULL, sizeof(pmk));
		bzero(pmk, sizeof(pmk));
		wl_take();
		bzero(mb.ioctl_req, IOCTL_MAX);
		wl_give();
	}
	if (err != 0) {
		sta_phase = HAKIOS_STA_IDLE;
		sta_last_error = EIO;
		return EIO;
	}
	bzero(params, sizeof(params));
	put32(params, 0, j->ssid_length);
	memcpy(params + 4, j->ssid, j->ssid_length);
	if (any_bssid) {
		memset(params + 36, 0xff, 6);
	} else {
		memcpy(params + 36, j->bssid, 6);
	}
	wl_take();
	join_status = -1;
	psk_status = -1;
	link_up = keyed = false;
	wl_give();
	uint64_t t0 = mach_absolute_time();

	err = bcm_ioctl(26, params, sizeof(params), NULL, sizeof(params));     /* WLC_SET_SSID */
	if (err != 0) {
		sta_phase = HAKIOS_STA_IDLE;
		sta_last_error = EIO;
		return EIO;
	}
	/* until the keys are set (or, open, the link is up), the join fails, or 20 s */
	while (micros_since(t0) < 20000000) {
		unsigned n;

		wl_take();
		n = d2h_ctrl_poll();
		wl_give();
		if ((psk && keyed) || (!psk && link_up && join_status == 0) || join_status > 0) {
			break;
		}
		/* the supplicant gave up (PSK_SUP 7 timeout, 8 failed -- reason 14: the access point deauthenticated us): over
		 * now, not after the firmware's own retries -- at the second retry with a wrong passphrase it ran out of packet
		 * ids ("lb_alloc: hnd_pktid_free_cnt 0") and trapped (rejoin2) */
		if (psk && psk_status >= 7) {
			break;
		}
		if (n == 0) {
			sleep_ms(5);
		}
	}
	if ((psk && keyed) || (!psk && link_up && join_status == 0)) {
		result = 0;
	} else if (join_status == 2) {
		result = ENOENT;
	} else if (join_status == 0 && psk && !keyed) {
		result = EAUTH;                 /* associated, the 4-way handshake never ended: the passphrase */
	} else if (join_status == 3 || join_status < 0) {
		result = ETIMEDOUT;
	} else {
		result = EIO;
	}
	if (result != 0) {
		sta_radio_cycle();              /* (out, and the radio fresh for whatever is joined next) */
		sta_phase = HAKIOS_STA_IDLE;
		sta_last_error = result;
		bzero(&sta_joined_with, sizeof(sta_joined_with));
	} else {
		sta_joined_with = *j;
		uint8_t bssid[6];

		bzero(bssid, sizeof(bssid));
		(void)bcm_ioctl(23, NULL, 0, bssid, 6); /* WLC_GET_BSSID */
		memcpy(sta_bss.bssid, bssid, 6);
		for (unsigned i = 0; i < sta_heard_count; i++) {
			if (memcmp(sta_heard[i].bssid, bssid, 6) == 0) {
				sta_bss = sta_heard[i];
				break;
			}
		}
		/* the network as the chip has it (WLC_GET_BSS_INFO: channel, signal, beacon, IEs) -- a join at the start comes
		 * before any scan of the station's, and wifid and configd read them (sta3: CHANNEL 858992977, RSSI 0) */
		{
			static struct hakios_sta_bss now;

			if (bss_now(&now, false)) {
				sta_bss = now;
			}
		}
		if (sta_bss.frequency == 0) {
			uint32_t cs = 0;

			if (iovar_get("chanspec", &cs, sizeof(cs)) == 0 && cs != 0) {
				unsigned centre = cs & 0xff, sb = (cs >> 8) & 7, bw = cs & 0x3800, ch = centre;

				ch = bw == 0x1800 ? centre - 2 + 4 * sb : bw == 0x2000 ? centre - 6 + 4 * sb : bw == 0x2800 ? centre - 14 +
				    4 * sb : centre;
				sta_bss.frequency = (uint16_t)chan_to_freq(ch, cs & 0xc000);
			}
		}

		sta_last_error = 0;
		sta_phase = HAKIOS_STA_JOINED;
	}
	printf("wifi: station: join (%s, a name of %u bytes) after %llu ms: %d (set SSID %d, link %s, supplicant %d); %u MHz, "
	    "%d dBm\n", psk ? "WPA2-PSK" : "open", j->ssid_length, micros_since(t0) / 1000, result, join_status,
	    link_up ? "up" : "down", psk_status, sta_bss.frequency, sta_bss.rssi);
	return result;
}

static int
sta_leave(void)
{
	if (fw_halted || recovering) {
		return 0;
	}
	sta_radio_cycle();                      /* (out, the radio down and up: the next join comes after a fresh start) */
	bzero(&sta_joined_with, sizeof(sta_joined_with));
	if (radio_up) {
		sta_phase = HAKIOS_STA_IDLE;
	}
	printf("wifi: station: left the network\n");
	return 0;
}

static void
sta_state(struct hakios_sta_state *st)
{
	st->phase = (enum hakios_sta_phase)sta_phase;
	st->bss = sta_bss;
	st->last_error = sta_last_error;
}

/* the channels: 2.4 GHz 1-13; 5 GHz 36-64 (52-64 radar), 100-144 (radar), 149-165 -- the EU's (the CLM decides what it sends) */
static unsigned
sta_channels(struct hakios_sta_channel *c, unsigned most)
{
	static const uint8_t five[] = { 36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140,
		                        144, 149, 153, 157, 161, 165 };
	unsigned n = 0;

	for (unsigned ch = 1; ch <= 13 && n < most; ch++, n++) {
		c[n].frequency = (uint16_t)(2407 + 5 * ch);
		c[n].flags = 0;
	}
	for (unsigned i = 0; i < sizeof(five) && n < most; i++, n++) {
		c[n].frequency = (uint16_t)(5000 + 5 * five[i]);
		c[n].flags = five[i] >= 52 && five[i] <= 144 ? HAKIOS_STA_CHANNEL_PASSIVE | HAKIOS_STA_CHANNEL_RADAR : 0;
	}
	return n;
}

static struct hakios_sta_ops sta_ops_bcm = {
	.power = sta_power,
	.scan = sta_scan,
	.heard = sta_heard_each,
	.join = sta_join,
	.leave = sta_leave,
	.state = sta_state,
	.channels = sta_channels,
};

/* the firmware halted (mb_data_check, the rings' lock held): the link down, the station's radio off */
static void
fw_halt_said(void)
{
	link_up = keyed = false;
	en0_link_update();
	if (sta_phase == HAKIOS_STA_JOINED) {
		hakios_sta_lost(EIO);
	}
	sta_phase = HAKIOS_STA_OFF;
	radio_up = false;
}

/*
 * RECOVERY (11-11, TODO 35f): the firmware has halted -- it traps a few seconds after ANY failed WPA handshake (a wrong
 * passphrase: its own supplicant's failure path runs out of packet ids), and nothing else brings it back. As Android's
 * driver does after a trap: the chip's power off and on and its link again (akita_pcie_restart), steps 1 to 7 from
 * scratch (the firmware, the rings -- the same memory again --, the blobs, the events, the radio), en0 carrying again
 * (the same interface: Darwin's network stack never sees it go), the station's radio on -- which joins the standing
 * network again (hakios.sta.c: the last network asked for, or fw/network's). A wrong passphrase costs a few seconds,
 * not the Wi-Fi until the next boot. At most recover_max times in 5 minutes (wifi_recover=N; 0: none).
 */
static void
bcm_recover(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	static uint64_t window_start;
	static unsigned in_window, total;
	uint64_t t0 = mach_absolute_time();
	bool ok;

	if (micros_since(window_start) > 300000000ull || window_start == 0) {
		window_start = t0;
		in_window = 0;
	}
	if (in_window >= recover_max) {
		printf("wifi: recovery: %u already in 5 minutes: the firmware stays halted (Wi-Fi off until the next boot)\n",
		    in_window);
		return;
	}
	in_window++;
	total++;
	printf("wifi: recovery %u: the firmware halted -- the chip started again from scratch\n", total);

	/* en0 quiet: nothing polled or sent; the link down (Darwin was told: fw_halt_said) */
	lck_mtx_lock(wl_lock);
	en0_live = false;
	link_up = keyed = false;
	lck_mtx_unlock(wl_lock);
	/* the station's call under way (the failed join that brought this on) let finish: it fails at once now */
	for (unsigned i = 0; i < 200 && hakios_sta_busy(); i++) {
		sleep_ms(100);
	}

	ok = akita_pcie_restart();
	if (ok) {
		lck_mtx_lock(wl_lock);
		bzero(&mb, sizeof(mb));
		bzero(&info, sizeof(info));
		trap_data = NULL;
		console_forget = true;
		tx.state = 0;
		tx.w = 0;
		tx.outstanding = 0;
		bzero(tx.busy, sizeof(tx.busy));
		fw_halted = false;
		lck_mtx_unlock(wl_lock);
		bar0_win = rd(cfg, CFG_BAR0_WIN);
		bar2_win = ~0u;                 /* (unknown after the reset: set at the next access) */
		ok = bcm_steps();
	}
	if (ok) {
		en0_attach();                   /* (en0 there already: the flow ring and send buffers again, en0_live) */
		ok = en0_live;
	}
	if (!ok) {
		fw_halted = true;
		recovering = false;
		printf("wifi: recovery %u FAILED after %llu ms: Wi-Fi stays off until the next boot\n", total, micros_since(t0) / 1000);
		return;
	}
	bzero(&sta_joined_with, sizeof(sta_joined_with));       /* (the next join is a fresh one, not "the same again") */
	radio_up = true;                        /* (WLC_UP at step 7) */
	sta_phase = HAKIOS_STA_IDLE;
	recovering = false;
	printf("wifi: recovery %u: the firmware runs again after %llu ms; the station's radio %s\n", total, micros_since(t0) / 1000,
	    hakios_sta_powered() ? "on: the standing network joined again" : "left off (Darwin has it off)");
	if (hakios_sta_powered()) {
		(void)hakios_sta_ask_power(true);
	} else {
		(void)sta_power(false);
	}
}

/* the link's events, under the rings' lock (event_handle): a network joined and lost by itself is said */
static void
sta_link_check(void)
{
	if (sta_phase == HAKIOS_STA_JOINED && !sta_leaving && (!link_up || !keyed)) {
		sta_phase = radio_up ? HAKIOS_STA_IDLE : HAKIOS_STA_OFF;
		sta_last_error = ECONNRESET;
		printf("wifi: station: the network is lost (link %s, keys %s)\n", link_up ? "up" : "down", keyed ? "set" : "not set");
		hakios_sta_lost(ECONNRESET);
	}
}

/* step 8, after en0: the station attached (Darwin's Wi-Fi side can ask from now on), the network of fw/network joined
 * through it as a standing request -- again whenever the radio comes back on -- as redwood's join_given does */
static void
station_start(void)
{
	static struct hakios_sta_join what;
	unsigned name_len = sizeof(what.ssid), pass_len = sizeof(what.passphrase);
	struct hakios_sta_state *st;
	int err;

	if ((sta_heard = kalloc_data(STA_HEARD_MOST * sizeof(*sta_heard), Z_WAITOK | Z_ZERO)) == NULL) {
		printf("wifi: 8: no memory for what a scan hears: no station\n");
		return;
	}
	radio_up = true;                        /* (WLC_UP at step 7) */
	sta_phase = HAKIOS_STA_IDLE;
	memcpy(sta_ops_bcm.address, tx.address, 6);
	hakios_sta_attach(&sta_ops_bcm);
	printf("wifi: 8: the station attached: Darwin's Wi-Fi side (wifid, Settings) can ask for scans and networks\n");
	bzero(&what, sizeof(what));
	if (!network_file(what.ssid, &name_len, what.passphrase, &pass_len)) {
		return;
	}
	what.ssid_length = (uint8_t)name_len;
	what.passphrase_length = (uint8_t)pass_len;
	what.security = HAKIOS_STA_WPA2_PSK;
	err = hakios_sta_ask_join_standing(&what);
	bzero(&what, sizeof(what));
	for (unsigned waited = 0; err == 0 && hakios_sta_busy() && waited < 300; waited++) {
		sleep_ms(100);
	}
	if ((st = kalloc_data(sizeof(*st), Z_WAITOK | Z_ZERO)) != NULL) {
		hakios_sta_state(st);
		printf("wifi: 8: the network of fw/network %s (%d, %d)\n", st->phase == HAKIOS_STA_JOINED ? "IS JOINED" :
		    "is NOT joined", err, st->last_error);
		kfree_data(st, sizeof(*st));
	}
	rx_say();
}

/* steps 1 to 7: the chip's cores, its ARM halted, the firmware and calibration written, the ARM let go, the rings, the
 * first ioctls and blobs, the events and the radio -- at the start, and again after a recovery (bcm_recover) */
static bool
bcm_steps(void)
{
	struct core *arm, *pcie;
	const uint8_t *fw, *nv;
	uint64_t fw_size = 0, nv_size = 0;
	uint8_t *vars = NULL;
	uint32_t vars_len = 0, vars_alloc = 0, token = 0, top, shared = 0;

	if (!announce(1, "reads: the chip's cores (enumeration ROM)")) {
		return false;
	}
	printf("wifi: 1: chipcommon id 0x%08x; PCIe core: power control 0x%08x, completion timeout 0x%08x\n",
	    bp_rd(CC_BASE + CC_CHIPID), pcie_rd(PCIE_POWER_CONTROL), pcie_rd(PCIE_CPL_TO_CTRL));
	if (!erom_scan()) {
		printf("wifi: 1: no cores: stopped\n");
		return false;
	}
	arm = core_find(CORE_ARM_CR4);
	pcie = core_find(CORE_PCIE2);
	if (arm == NULL || pcie == NULL) {
		printf("wifi: 1: no %s core: stopped (an ARM CA7 or sysmem chip is not what Google's driver says the 4383 "
		    "is)\n", arm == NULL ? "ARM CR4" : "PCIe2");
		return false;
	}
	printf("wifi: 1: PCIe core revision %u (Android: buscorerev 77)\n", pcie->rev);

	/* (the ARM's registers only after its domain is powered and the completion timeout is on: step 2) */
	if (!announce(2, "writes: completion timeout on, the ARM's backplane powered, the ARM halted, its memory sized")) {
		return false;
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

	if (bar2 == NULL) {                     /* (mapped once: a recovery finds the same place) */
		bar2 = (volatile uint8_t *)ml_io_map_unmappable(akita_pcie_bar(2), BAR2_SIZE, VM_WIMG_IO);
	}
	wr(cfg, CFG_BAR1_WIN, 0);
	bar2_win = 0;
	core_say(arm, "as found");
	core_reset(arm, SICF_CPUHALT, SICF_CPUHALT);
	core_say(arm, "halted");
	ram_base = RAM_BASE_4383;
	ram_size = cr4_ram_size(arm);
	if (ram_size == 0 || ram_size > 0x800000) {
		printf("wifi: 2: a memory of %u bytes cannot be: stopped\n", ram_size);
		return false;
	}
	printf("wifi: 2: memory 0x%x - 0x%x (Android: 2228224 bytes at 0x6e0000)\n", ram_base, ram_base + ram_size);
	top = ram_base + ram_size - 4;
	mem_wr(top, 0);
	printf("wifi: 2: the top word 0x%08x: 0x%08x; address 0: 0x%08x\n", top, mem_rd(top), mem_rd(0));

	if (!announce(3, "writes: the firmware, the calibration with its token, random bytes, the reset vector")) {
		return false;
	}
	fw = hakios_image_file("fw", FW_NAME, &fw_size, NULL);
	nv = hakios_image_file("fw", NVRAM_NAME, &nv_size, NULL);
	if (fw == NULL || nv == NULL) {
		printf("wifi: 3: fw/%s or fw/%s not in the small image (fw-wifi/ of the build): stopped\n", FW_NAME,
		    NVRAM_NAME);
		return false;
	}
	if (fw_size < 64 || fw_size + nv_size + 0x10000 > ram_size) {
		printf("wifi: 3: fw/%s (%llu bytes) and fw/%s (%llu) do not fit a memory of %u bytes: stopped\n", FW_NAME,
		    fw_size, NVRAM_NAME, nv_size, ram_size);
		return false;
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
			return false;
		}
	}
	vars_len = nvram_vars(nv, nv_size, &vars, &vars_alloc);
	if (vars_len == 0 || vars_len > 0x10000) {
		printf("wifi: 3: the calibration came to %u bytes: stopped\n", vars_len);
		return false;
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
			return false;
		}
		mem_wr(0, vector);
		printf("wifi: 3: reset vector 0x%08x at address 0 (was 0x%08x, reads 0x%08x)\n", vector, before, mem_rd(0));
	}
	kfree_data(vars, vars_alloc);

	{
		/*
		 * (11-12, TODO 35e, a test) wifi_splitrx=0: the firmware's split rx off before it runs -- _bcmsplitrx, the byte at
		 * 0x6e0133 of THIS build (fw_bcmdhd.map_4383_a3; the word at 0x6e0130 reads 0x01010100 in its image: _bcmfrwdpkt
		 * 0, 1, 1, _bcmsplitrx 1; _bcmsplitrx_mode 4 next). With split rx a frame's payload goes straight into a host
		 * buffer and the firmware fetches it back to look at it (pktfetch): under the EDL ring it decrypts frames and
		 * drops them -- is it that path? Patched only if the word is what this build has.
		 */
		uint32_t split = 1;

		if (PE_parse_boot_argn("wifi_splitrx", &split, sizeof(split)) && split == 0) {
			uint32_t w = mem_rd(0x006e0130);

			if (w == 0x01010100) {
				mem_wr(0x006e0130, w & 0x00ffffffu);
				printf("wifi: 3: split rx off (_bcmsplitrx 0: 0x%08x -> 0x%08x)\n", w, mem_rd(0x006e0130));
			} else {
				printf("wifi: 3: split rx left: the word at 0x6e0130 is 0x%08x, not this build's\n", w);
			}
		}
	}
	if (!announce(4, "runs: the ARM let go; the firmware's shared area awaited")) {
		return false;
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
			return false;
		}
		printf("wifi: 4: the top word 0x%08x after %llu ms\n", v, us / 1000);
		if (v < ram_base || v >= ram_base + ram_size) {
			printf("wifi: 4: not an address in the chip's memory: stopped\n");
			return false;
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
		return false;
	}
	if (!rings_up(shared)) {
		console_say(shared);
		return false;
	}
	console_say(shared);

	if (!announce(6, "talks: the first ioctls (\"ver\", \"cur_etheraddr\"), the regulatory and transmit-cap blobs")) {
		return false;
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
		wl_take();
		info_rings_up();
		wl_give();
		/*
		 * The firmware's event logs (11-11 night, TODO 35f -- THE TRAP): its packet-id pool (hnd_pktid_map's free count,
		 * read from its memory with wifi_poolwatch) fell from ~254 to 0 in ~125 s from its start, idle or not, rx buffers
		 * 256 or 64, and it trapped there (hnd_pkt_get -> _hnd_die). Its console meanwhile: "EL: <tag> <fmt>" every
		 * second -- event-log records it makes for the host, which takes them through a debug info ring (or the EDL
		 * ring) that this driver never creates. "logtrace" 0 (what Android sets to keep it from waking a sleeping host)
		 * stops it sending them up. wifi_logtrace=1: left as the firmware has it.
		 */
		{
			/*
			 * ... "logtrace" is not in this firmware (-23). What stopped the leak instead: every event-log tag off
			 * ("event_log_tag_control", wl_el_tag_params_t {tag, set, flags}: flags 0, neither LOG (0x80: kept for the
			 * host) nor PRINT (0x40: the console), tags 0 to EVENT_LOG_TAG_MAX 482). Its heap grew ~900 bytes a record
			 * (wifi_poolwatch: 1159 KB -> 1389 KB in 120 s, the pools themselves unchanged) and each took a packet id.
			 * wifi_evlog=1: left on.
			 */
			uint32_t evlog = 0, failed = 0;
			uint64_t t0 = mach_absolute_time();

			evlog = wifi_edl() ? 0 : 1;     /* (the old mode: tags 121-482 off by default -- see wifi_edl) */
			PE_parse_boot_argn("wifi_evlog_off", &evlog, sizeof(evlog));
			if (evlog) {    /* (OFF by default: with every tag off, rx stopped -- 39 frames, then 1 (evlog0, soak1)) */
				/* (11-12, TODO 35e) wifi_evlog_lo/hi=N: only the tags from lo to hi off -- which ones does rx need? */
				uint32_t lo = wifi_edl() ? 0 : 121, hi = 482;

				PE_parse_boot_argn("wifi_evlog_lo", &lo, sizeof(lo));
				PE_parse_boot_argn("wifi_evlog_hi", &hi, sizeof(hi));
				printf("wifi: 6: event-log tags %u to %u to be turned off\n", lo, hi);
				for (uint16_t tag = (uint16_t)lo; tag <= hi && tag <= 482; tag++) {
					uint8_t el[4] = { (uint8_t)tag, (uint8_t)(tag >> 8), 0, 0 };

					if (iovar_set("event_log_tag_control", el, sizeof(el)) != 0) {
						failed++;
					}
				}
				printf("wifi: 6: the firmware's event logs off (tags %u-%u, %u refused) in %llu ms\n", lo, hi, failed,
				    micros_since(t0) / 1000);
				/*
				 * wifi_evlog_print=1 (11-12): the tags left on (0 to lo-1) to the firmware's CONSOLE only (PRINT, 0x40)
				 * instead of LOG (0x80, records kept for a host that has no ring for them: the old mode's leak) -- the
				 * firmware keeps logging (rx flowed only while it did) without a packet id a record?
				 */
				uint32_t print_only = 0;

				PE_parse_boot_argn("wifi_evlog_print", &print_only, sizeof(print_only));
				if (print_only) {
					failed = 0;
					for (uint16_t tag = 0; tag < lo && tag <= 482; tag++) {
						uint8_t el[4] = { (uint8_t)tag, (uint8_t)(tag >> 8), 0, 0x40 };

						if (iovar_set("event_log_tag_control", el, sizeof(el)) != 0) {
							failed++;
						}
					}
					printf("wifi: 6: event-log tags 0-%u to the console only (PRINT), %u refused\n", lo ? lo - 1 : 0, failed);
				}
				/* ("ecounters_suspend", to stop its statistics reports too: -47, not taken) */
			}
		}
		{
			uint32_t keep = 0, off = 0, now = ~0u;

			PE_parse_boot_argn("wifi_logtrace", &keep, sizeof(keep));
			if (!keep) {
				int e = iovar_set("logtrace", &off, sizeof(off));

				(void)iovar_get("logtrace", &now, sizeof(now));
				printf("wifi: 6: \"logtrace\" 0 (its event logs not sent up): %d, now %u\n", e, now);
			}
		}
		sleep_ms(200);
		(void)d2h_ctrl_poll();
		console_say(shared);
		printf("wifi: 6: control ring: written to %u; read to %u by the array in host memory, %u by the chip's own\n",
		    mb.w[RING_H2D_CTRL], mb.idx[mb.h2d_r + RING_H2D_CTRL], mem_rd(mb.tcm_h2d_r + 4 * RING_H2D_CTRL) & 0xffff);
		printf("wifi: 6: %u events so far; the firmware answers\n", mb.events);
	}

	if (!announce(7, "talks: the events wanted, the radio up, a scan of every channel")) {
		return false;
	}
	scan_all();
	console_say(shared);

	return true;
}

void
akita_bcm_run(void)
{
	if (!PE_parse_boot_argn("wifi", &steps, sizeof(steps)) || steps == 0) {
		return;
	}
	if (wl_lock == NULL) {                  /* (made already when en0 came at the start) */
		wl_group = lck_grp_alloc_init("akita_wifi", LCK_GRP_ATTR_NULL);
		wl_lock = lck_mtx_alloc_init(wl_group, LCK_ATTR_NULL);
	}
	cfg = akita_pcie_config();
	bar0 = (volatile uint8_t *)ml_io_map(akita_pcie_bar(0), BAR0_SIZE);
	bar0_win = rd(cfg, CFG_BAR0_WIN);

	PE_parse_boot_argn("wifi_recover", &recover_max, sizeof(recover_max));
	recover_call = thread_call_allocate(bcm_recover, NULL);
	if (!bcm_steps()) {
		return;
	}
	if (!announce(8, "carries: en0 on the chip, the station Darwin's Wi-Fi side talks to, the network of fw/network joined")) {
		return;
	}
	en0_attach();
	if (en0_attached) {
		station_start();
	}
	console_say(mb.shared);
	{
		/* diagnosis (11-11, TODO 35f): wifi_ioctltest=N -- the firmware's free packet ids (hnd_pktid_map +4, THIS build)
		 * before and after N "ver" iovars, then N WLC_GET_RSSI: does an ioctl cost one for good? */
		uint32_t n = 0;

		PE_parse_boot_argn("wifi_ioctltest", &n, sizeof(n));
		if (n != 0) {
			uint32_t map = mem_rd(0x006e01d8), f0, f1, f2;
			char ver[64];
			int32_t rssi;

			sleep_ms(5000);
			f0 = mem_rd(map + 4);
			for (uint32_t i = 0; i < n; i++) {
				(void)iovar_get("ver", ver, sizeof(ver));
			}
			f1 = mem_rd(map + 4);
			for (uint32_t i = 0; i < n; i++) {
				(void)bcm_ioctl(127, NULL, 0, &rssi, sizeof(rssi));     /* WLC_GET_RSSI */
			}
			f2 = mem_rd(map + 4);
			printf("wifi: ioctl test: free packet ids %u, after %u \"ver\" %u, after %u WLC_GET_RSSI %u\n", f0, n, f1, n, f2);
		}
	}
}
