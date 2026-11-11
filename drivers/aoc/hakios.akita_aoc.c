/*
 * The Pixel 8a's AoC ("always-on compute", Google's DSP subsystem on zuma): audio (the two CS35L41 speaker amplifiers'
 * data comes from its TDM ports) and the sensors (the IMU, proximity, light: CHRE) are behind it -- TODO 33 and 34.
 * Android's way to it (google-modules/aoc, aoc.c and aoc_v1.c; soc/gs drivers/soc/google/gsa): the AoC asked on
 * (aoc_req 0x154b0000: write 1, status at +0x40), its signed firmware (aoc.bin) copied into the AoC's DRAM carveout
 * (0x94000000, 48 MB), then the GSA -- Google's security co-processor, through its non-secure mailbox (gsa-ns,
 * 0x16490000) -- authenticates the image (GSA_MB_CMD_LOAD_AOC_FW_IMG, 90: the header's and the body's physical
 * addresses) and starts the AoC (GSA_MB_CMD_AOC_CMD, 91: GSA_AOC_START); the AoC then publishes its services in an IPC
 * block in its DRAM, reached through mailboxes.
 *
 * The GSA mailbox (gsa_mbox.c): 16 shared registers SR0-15 at +0x80; a request writes the command into SR0, the
 * argument count into SR1 and the arguments after; the response interrupt (bit 0) is cleared (INTCR0 +0x24) and
 * unmasked (INTMR0 +0x28), the request raised (INTGR1 +0x40, bit 0), and the GSA answers in SR0 (the command | bit 31),
 * SR1 (an error: 0 OK), SR2 (its argument count) and the arguments after, the response bit then set in INTSR0 (+0x2c).
 *
 *   aoc=1   a look: the AoC's power request and status read, the GSA mailbox's registers, and one harmless request
 *           (GSA_MB_CMD_GET_CHIP_ID, 3) -- whether this kernel can talk to the GSA at all. Nothing switched on,
 *           nothing written but the mailbox.
 *   aoc=2   ... and one word of the AoC's DRAM carveout (0x94000000, 48 MB; U-Boot's tree keeps it no-map, so XNU never
 *           has it): 6 MB in, where a running firmware's IPC block (aoc-ipc's struct aoc_control_block, magic
 *           0xA0C00A0C) is by default -- whether Android's AoC firmware still runs when this kernel starts.
 *
 *   aoc=3   THE AoC STARTED, as Android's aoc_fw_callback does with the GSA: its state asked first (only from 0,
 *           inactive); the image (fw/aoc.bin: Google's signed one, ../fw-aoc in the build) checked -- the 4 KB auth
 *           header (magic "AOC", generation 2: its image_config says where the GSA locks it, 0x94000000 + 28 MB) and the
 *           superbin after it (0xaabbccdd; image_size = the IPC block's offset) --, the body copied to the carveout's
 *           start, the header into a page of ours, GSA_MB_CMD_LOAD_AOC_FW_IMG (90: header lo/hi, body lo/hi), the
 *           firmware's parameters written at the IPC block (aoc_pass_fw_information: AOC_PARAMETER_MAGIC, the board,
 *           the carveout, the heaps at its top, the chip), GSA_AOC_START through Trusty (devices/akita/drivers/trusty,
 *           akita_trusty_aoc_cmd), and the IPC block's magic (0xA0C00A0C) awaited.
 *
 * Starting the AoC is the GSA's: the image's load and authentication through the mailbox (90), but GSA_AOC_START,
 * GET_STATE and the unload through Trusty (gsa_tz.c: a TIPC channel to the secure world's "hwmgr" service) --
 * devices/akita/drivers/trusty (11-11: it works; the AoC said INACTIVE).
 *
 * Found 11-11 (aoc=1): the AoC is ON when this kernel starts (aoc_req 1, state 1: left so by Android's boot or the
 * bootloader); the GSA answers (GET_CHIP_ID: error 1, invalid arguments -- the ROM's form wants a buffer).
 */
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_map_address_t pmap_map(vm_map_address_t virt, vm_offset_t start, vm_offset_t end, vm_prot_t prot, unsigned int flags);
extern uint64_t kvtophys(vm_offset_t va);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
extern int akita_trusty_aoc_cmd(uint32_t cmd, uint32_t *state);

#define AOC_REQ_PHYS            0x154b0000ULL   /* aoc_req: +0 the request, +0x40 the AoC's state */
#define AOC_DRAM_PHYS           0x94000000ULL   /* the AoC's carveout (aoc@94000000), 48 MB */
#define AOC_DRAM_SIZE           (48u << 20)
#define AOC_MAGIC               0xA0C00A0Cu
#define GSA_MBOX_PHYS           0x16490000ULL   /* gsa-ns */

#define MBOX_INTCR0             0x0024
#define MBOX_INTMR0             0x0028
#define MBOX_INTSR0             0x002c
#define MBOX_INTMSR0            0x0030
#define MBOX_INTGR1             0x0040
#define MBOX_SR(n)              (0x0080 + (n) * 4)
#define MBOX_SR_NUM             16

#define GSA_MB_CMD_RSP          (1u << 31)
#define GSA_MB_CMD_GET_CHIP_ID  3

static vm_offset_t aoc_req, gsa;
static uint32_t mode;

static inline uint32_t
rd(vm_offset_t base, uint32_t off)
{
	return *(volatile uint32_t *)(base + off);
}

static inline void
wr(vm_offset_t base, uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(base + off) = v;
}

/* one GSA request, polled (no interrupt): 0 and the response's words, or -1 when the GSA did not answer */
static int gsa_request_ms(uint32_t cmd, const uint32_t *args, uint32_t argc, uint32_t *rsp, uint32_t *rsp_n, uint32_t *err,
    int timeout_ms);

static int
gsa_request(uint32_t cmd, const uint32_t *args, uint32_t argc, uint32_t *rsp, uint32_t *rsp_n, uint32_t *err)
{
	return gsa_request_ms(cmd, args, argc, rsp, rsp_n, err, 500);
}

static int
gsa_request_ms(uint32_t cmd, const uint32_t *args, uint32_t argc, uint32_t *rsp, uint32_t *rsp_n, uint32_t *err,
    int timeout_ms)
{
	if (argc + 2 > MBOX_SR_NUM) {
		return -1;
	}
	wr(gsa, MBOX_SR(0), cmd);
	wr(gsa, MBOX_SR(1), argc);
	for (uint32_t i = 0; i < argc; i++) {
		wr(gsa, MBOX_SR(i + 2), args[i]);
	}
	wr(gsa, MBOX_INTCR0, 1u);
	wr(gsa, MBOX_INTMR0, rd(gsa, MBOX_INTMR0) & ~1u);
	__builtin_arm_dsb(0xf);
	wr(gsa, MBOX_INTGR1, 1u);
	int answered = 0;
	for (int ms = 0; ms < timeout_ms; ms++) {
		if (rd(gsa, MBOX_INTSR0) & 1u) {
			answered = 1;
			break;
		}
		delay(1000);
	}
	uint32_t got = rd(gsa, MBOX_SR(0));
	*err = rd(gsa, MBOX_SR(1));
	uint32_t n = rd(gsa, MBOX_SR(2));
	*rsp_n = 0;
	if (answered && got == (cmd | GSA_MB_CMD_RSP) && n + 3 <= MBOX_SR_NUM) {
		for (uint32_t i = 0; i < n; i++) {
			rsp[i] = rd(gsa, MBOX_SR(i + 3));
		}
		*rsp_n = n;
	}
	wr(gsa, MBOX_INTCR0, 1u);
	wr(gsa, MBOX_INTMR0, rd(gsa, MBOX_INTMR0) | 1u);
	printf("akita aoc: GSA request %u: %s (SR0 0x%x, error %u, %u words)\n", cmd, answered ? "answered" : "NO ANSWER",
	    got, *err, n);
	return answered && got == (cmd | GSA_MB_CMD_RSP) ? 0 : -1;
}

/*
 * The carveout mapped once, all 48 MB (write-combined), with kernel addresses of its own and pmap_map: not ml_io_map,
 * whose whole area is 8 MB (io_map.c's IO_MAP_SIZE) -- a 48 MB ml_io_map, then the second of 1 MB megabyte pieces, each
 * killed the kernel (11-11); hakios.akita_display.c maps its buffers so for the same reason
 */
static vm_offset_t
carveout_map(void)
{
	static vm_offset_t va;

	if (va == 0) {
		vm_offset_t v = 0;

		if (kmem_alloc(kernel_map, &v, AOC_DRAM_SIZE, KMA_VAONLY | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
			return 0;
		}
		pmap_map(v, (vm_offset_t)AOC_DRAM_PHYS, (vm_offset_t)(AOC_DRAM_PHYS + AOC_DRAM_SIZE), VM_PROT_READ | VM_PROT_WRITE,
		    VM_WIMG_WCOMB);
		va = v;
	}
	return va;
}

#define GSA_MB_CMD_LOAD_AOC_FW_IMG      90
#define AOC_AUTH_HEADER_SIZE            4096u
#define AOC_PARAMETER_MAGIC             0x0a0cda7au
#define AOC_BINARY_DRAM_BASE            0x98000000u     /* the carveout as the AoC sees it */
#define CHIPID_PHYS                     0x10000000ULL

/* the AoC's firmware loaded by the GSA and started through Trusty (aoc=3) */
static void
aoc_boot(void)
{
	uint64_t size = 0;
	const uint8_t *f = hakios_image_file("fw", "aoc.bin", &size, NULL);
	uint32_t state = 0xff;

	if (f == NULL || size < AOC_AUTH_HEADER_SIZE + 0x1000) {
		printf("akita aoc: no fw/aoc.bin in the image (the build's ../fw-aoc)\n");
		return;
	}
	uint32_t auth_magic = *(const uint32_t *)(f + 1024), generation = *(const uint32_t *)(f + 1028);
	const uint8_t *cfg = f + 1024 + (generation == 2 ? 384 : 352);         /* header_v2: a 64-byte hash */
	uint32_t fw_start = *(const uint32_t *)(cfg + 4), fw_size = *(const uint32_t *)(cfg + 8);
	const uint8_t *sb = f + AOC_AUTH_HEADER_SIZE;
	uint32_t sb_magic = *(const uint32_t *)sb, image_size = *(const uint32_t *)(sb + 12);
	char version[65];

	memcpy(version, sb + 48, 64);
	version[64] = 0;
	printf("akita aoc: fw/aoc.bin: %llu bytes, auth magic 0x%x generation %u, locked at 0x%x + 0x%x; superbin 0x%x, image 0x%x, "
	    "\"%s\"\n", size, auth_magic, generation, fw_start, fw_size, sb_magic, image_size, version);
	if (auth_magic != 0x00434f41 || sb_magic != 0xaabbccddu || fw_start != AOC_DRAM_PHYS ||
	    image_size + 0x10000 > AOC_DRAM_SIZE || size - AOC_AUTH_HEADER_SIZE > image_size) {
		printf("akita aoc: not an image this driver knows how to load\n");
		return;
	}
	if (akita_trusty_aoc_cmd(0, &state) != 0) {
		printf("akita aoc: the GSA's AoC service not reached (Trusty): not loaded\n");
		return;
	}
	printf("akita aoc: the AoC's state %u; copying the image ...\n", state);
	delay(300000);
	if (state != 0) {
		printf("akita aoc: the AoC is not inactive (%u): not loaded again\n", state);
		return;
	}
	/* the body at the carveout's start (aligned 64-bit stores, no memcpy/bzero into it), the IPC block's area cleared */
	vm_offset_t dram = carveout_map();
	if (!dram) {
		printf("akita aoc: the carveout could not be mapped\n");
		return;
	}
	uint64_t body = size - AOC_AUTH_HEADER_SIZE;
	const uint8_t *src = f + AOC_AUTH_HEADER_SIZE;
	uint64_t i = 0;
	for (; i + 8 <= body; i += 8) {
		uint64_t w;
		__builtin_memcpy(&w, src + i, 8);
		*(volatile uint64_t *)(dram + i) = w;
		if ((i & ((8u << 20) - 1)) == 0 && i != 0) {
			printf("akita aoc: ... %llu MB\n", i >> 20);
		}
	}
	for (; i < body; i++) {
		*(volatile uint8_t *)(dram + i) = src[i];
	}
	for (uint32_t k = 0; k < 0x10000; k += 8) {
		*(volatile uint64_t *)(dram + image_size + k) = 0;
	}
	__builtin_arm_dsb(0xf);
	/* the header in a page of ours, cleaned to memory for the GSA to read */
	vm_offset_t hdr = 0;
	if (kmem_alloc_contig(kernel_map, &hdr, PAGE_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		printf("akita aoc: no memory for the header\n");
		return;
	}
	memcpy((void *)hdr, f, AOC_AUTH_HEADER_SIZE);
	for (vm_offset_t line = hdr; line < hdr + PAGE_SIZE; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__builtin_arm_dsb(0xf);
	uint64_t hdr_pa = kvtophys(hdr);
	uint32_t args[4] = { (uint32_t)hdr_pa, (uint32_t)(hdr_pa >> 32), (uint32_t)AOC_DRAM_PHYS, (uint32_t)(AOC_DRAM_PHYS >> 32) };
	uint32_t rsp[MBOX_SR_NUM], n = 0, err = 0;
	uint64_t t0 = mach_absolute_time(), ns;

	printf("akita aoc: the image copied (%llu MB); the GSA asked to authenticate and load it (header at 0x%llx)\n",
	    (size - AOC_AUTH_HEADER_SIZE) >> 20, hdr_pa);
	delay(300000);
	if (gsa_request_ms(GSA_MB_CMD_LOAD_AOC_FW_IMG, args, 4, rsp, &n, &err, 20000) != 0 || err != 0) {
		printf("akita aoc: THE GSA REFUSED THE IMAGE: error %u (1 invalid args, 2 auth failed, 3 busy, 4 running, 5 resources)\n",
		    err);
		return;
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	printf("akita aoc: THE GSA LOADED THE IMAGE in %llu ms\n", ns / 1000000);
	/* the firmware's parameters at the IPC block (aoc_pass_fw_information), as Android passes them */
	vm_offset_t chip = ml_io_map(CHIPID_PHYS, 0x1000);
	uint32_t chip_type = chip ? rd(chip, 0) & 0xf : 0, r10 = chip ? rd(chip, 0x10) : 0;
	uint32_t chip_rev = (((r10 >> 20) & 0xf) << 4) | ((r10 >> 16) & 0xf);
	uint32_t top = AOC_DRAM_SIZE, sensor = top - (4u << 20), playback = sensor - 0x4000, capture = playback - 0x10000;
	const uint32_t kv[][2] = {
		{ 0x1001, 0x40402 }, { 0x1002, 0x10000 },               /* the board (aoc-board-id, -rev: "AK3") */
		{ 0x1003, 0 },                                          /* SRAM repaired */
		{ 0x1005, (uint32_t)AOC_DRAM_PHYS }, { 0x1006, AOC_DRAM_SIZE },
		{ 0x1007, AOC_BINARY_DRAM_BASE + sensor }, { 0x1008, 4u << 20 },
		{ 0x100C, AOC_BINARY_DRAM_BASE + playback }, { 0x100D, 0x4000 },
		{ 0x100E, AOC_BINARY_DRAM_BASE + capture }, { 0x100F, 0x10000 },
		{ 0x1009, 0 }, { 0x100A, 0 }, { 0x100B, 0 }, { 0x1010, 0 },     /* VNOM, monitor mode, UART, ultrasonic */
		{ 0x1011, (uint32_t)mach_absolute_time() },             /* a seed */
		{ 0x1012, chip_rev }, { 0x1013, chip_type }, { 0x1014, 1 }, { 0x1015, 0 },   /* the chip, gnss-type 1 */
	};
	volatile uint32_t *p = (volatile uint32_t *)(dram + image_size);
	uint32_t num = sizeof(kv) / sizeof(kv[0]);
	p[0] = AOC_PARAMETER_MAGIC;
	p[1] = num;
	p[2] = 12 + num * 12;
	for (uint32_t i = 0; i < num; i++) {
		p[3 + 3 * i] = kv[i][0];
		p[4 + 3 * i] = 4;
		p[5 + 3 * i] = kv[i][1];
	}
	__builtin_arm_dsb(0xf);
	printf("akita aoc: parameters written (chip type %u revision 0x%x); the AoC asked on (aoc_req), started through the GSA\n",
	    chip_type, chip_rev);
	delay(300000);
	wr(aoc_req, 0, 1);
	if (akita_trusty_aoc_cmd(1, &state) != 0) {         /* GSA_AOC_START */
		printf("akita aoc: GSA_AOC_START refused\n");
		return;
	}
	for (int ms = 0; ms < 10000; ms += 100) {
		if (rd(dram, image_size) == AOC_MAGIC) {
			char fw[33] = "";
			for (int i = 0; i < 32; i++) {
				fw[i] = (char)*(volatile uint8_t *)(dram + image_size + 36 + i);
				if (fw[i] && (fw[i] < 32 || fw[i] > 126)) {
					fw[i] = 0;
					break;
				}
			}
			printf("akita aoc: THE AoC IS UP after %d ms: its IPC block, fw 0x%x hw 0x%x status 0x%x, %u services of %u bytes at "
			    "+0x%x, \"%s\"\n", ms, rd(dram, image_size + 4), rd(dram, image_size + 8), rd(dram, image_size + 12),
			    rd(dram, image_size + 24), rd(dram, image_size + 28), rd(dram, image_size + 32), fw);
			/* its services by name (aoc-ipc's struct aoc_ipc_service_header: name[32], flags, two rings), a line of a few */
			uint32_t count = rd(dram, image_size + 24), each = rd(dram, image_size + 28), at = rd(dram, image_size + 32);
			char line[200];
			int len = 0;
			for (uint32_t k = 0; k < count && k < 128 && each >= 32; k++) {
				char nm[33];
				for (int c = 0; c < 32; c++) {
					nm[c] = (char)*(volatile uint8_t *)(dram + image_size + at + k * each + c);
				}
				nm[32] = 0;
				if (len + (int)strlen(nm) + 3 > (int)sizeof(line) - 1) {
					printf("akita aoc: services:%s\n", line);
					len = 0;
				}
				len += snprintf(line + len, sizeof(line) - len, " %s", nm);
			}
			if (len) {
				printf("akita aoc: services:%s\n", line);
			}
			return;
		}
		delay(100000);
	}
	printf("akita aoc: no IPC block in 10 s (word 0x%x, state %u)\n", rd(dram, image_size), state);
}

static void
akita_aoc_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint32_t rsp[MBOX_SR_NUM], n = 0, err = 0;

	printf("akita aoc: aoc_req 0x%x, the AoC's state 0x%x\n", rd(aoc_req, 0), rd(aoc_req, 0x40));
	printf("akita aoc: GSA mailbox: INTMR0 0x%x INTSR0 0x%x INTMSR0 0x%x; SR0-3 0x%x 0x%x 0x%x 0x%x\n", rd(gsa, MBOX_INTMR0),
	    rd(gsa, MBOX_INTSR0), rd(gsa, MBOX_INTMSR0), rd(gsa, MBOX_SR(0)), rd(gsa, MBOX_SR(1)), rd(gsa, MBOX_SR(2)),
	    rd(gsa, MBOX_SR(3)));
	if (gsa_request(GSA_MB_CMD_GET_CHIP_ID, NULL, 0, rsp, &n, &err) == 0) {
		printf("akita aoc: THE GSA ANSWERS: chip id (%u words) %08x %08x %08x %08x\n", n, n > 0 ? rsp[0] : 0, n > 1 ? rsp[1] : 0,
		    n > 2 ? rsp[2] : 0, n > 3 ? rsp[3] : 0);
	}
	/*
	 * The carveout: the IPC block a running firmware keeps, 6 MB in (aoc.c's default when it loads no firmware). ONE
	 * word, and only there: the firmware body the GSA authenticated (from the carveout's start) is LOCKED against every
	 * other master -- the first version of this probe read the whole carveout from 0 and the kernel died at once (the
	 * phone came back in its flashed slot, 11-11). aoc=2: the look at 6 MB.
	 */
	if (mode == 5) {
		/* aoc=5 (a test, 11-11): may the CPU WRITE the carveout? aoc=3 died at its first copy; reads are fine. One word
		 * at the start, at the IPC block (28 MB) and in the heaps (47 MB), said before, read back after */
		const uint32_t at[3] = { 0, 28, 47 };
		for (int i = 0; i < 3; i++) {
			vm_offset_t pg = ml_io_map(AOC_DRAM_PHYS + ((uint64_t)at[i] << 20), 0x1000);
			printf("akita aoc: writing a word at +%u MB (was 0x%08x) ...\n", at[i], pg ? rd(pg, 0x100) : 0);
			delay(300000);
			if (pg) {
				wr(pg, 0x100, 0x5a5a1234u + (uint32_t)i);
				__builtin_arm_dsb(0xf);
			}
			printf("akita aoc: +%u MB written: reads back 0x%08x\n", at[i], pg ? rd(pg, 0x100) : 0);
			delay(300000);
		}
		return;
	}
	if (mode == 4) {
		/*
		 * aoc=4 (a test, 11-11): which of the carveout the CPU may touch -- aoc=3's copy and the first probe's read both
		 * killed the kernel, one word 6 MB in did not. One word a megabyte, said BEFORE it is read, slowly (the log
		 * must get out before an abort does)
		 */
		for (uint32_t mb = 0; mb < (AOC_DRAM_SIZE >> 20); mb++) {
			printf("akita aoc: reading +%u MB ...\n", mb);
			delay(150000);
			vm_offset_t pg = ml_io_map(AOC_DRAM_PHYS + ((uint64_t)mb << 20), 0x1000);
			uint32_t w = pg ? rd(pg, 0) : 0xdeadbeef;
			printf("akita aoc: +%u MB: 0x%08x\n", mb, w);
		}
		printf("akita aoc: the whole carveout read, one word a megabyte\n");
		return;
	}
	if (mode >= 3) {
		aoc_boot();
		return;
	}
	if (mode < 2) {
		return;
	}
	vm_offset_t ipc = ml_io_map(AOC_DRAM_PHYS + (6u << 20), 0x1000);

	if (!ipc) {
		printf("akita aoc: 6 MB into the carveout could not be mapped\n");
		return;
	}
	uint32_t magic = rd(ipc, 0);

	printf("akita aoc: 6 MB into the carveout: 0x%08x%s\n", magic, magic == AOC_MAGIC ? " -- AN IPC BLOCK" : "");
	if (magic == AOC_MAGIC) {
		char fw[33] = "";

		for (int i = 0; i < 32; i++) {
			fw[i] = (char)*(volatile uint8_t *)(ipc + 36 + i);
			if (fw[i] && (fw[i] < 32 || fw[i] > 126)) {
				fw[i] = 0;
				break;
			}
		}
		printf("akita aoc: fw_version 0x%x hw_version 0x%x status 0x%x, %u services of %u bytes at +0x%x; \"%s\"\n",
		    rd(ipc, 4), rd(ipc, 8), rd(ipc, 12), rd(ipc, 24), rd(ipc, 28), rd(ipc, 32), fw);
	}
}

__startup_func
static void
akita_aoc_init(void)
{
	static thread_call_t start;
	uint64_t deadline;

	if (!PE_parse_boot_argn("aoc", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	aoc_req = ml_io_map(AOC_REQ_PHYS, 0x1000);
	gsa = ml_io_map(GSA_MBOX_PHYS, 0x1000);
	if (!aoc_req || !gsa) {
		printf("akita aoc: could not map its registers\n");
		return;
	}
	start = thread_call_allocate(akita_aoc_start, NULL);
	/* (aoc=3 at 25 s: after the USB log is up -- a load that killed the kernel at 3 s left no line of it, 11-11) */
	clock_interval_to_deadline(mode >= 3 ? 25 : 3, NSEC_PER_SEC, &deadline);     /* (aoc=4 and 5 too) */
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_aoc_init);
