/*
 * The GPU of the Pixel 8a (akita): zuma's Mali-G715 (arch 11.8, r1p2, 7 shader cores), woken step by step -- what
 * drivers/gpu/hakios.adreno.c is for redwood's Adreno, here for a Mali with its command-stream front end (CSF): a
 * small processor of its own, the MCU, runs Arm's firmware (mali_csffw-r54p2.bin, the phone's own, /vendor/firmware of
 * its Android, in fw/ of the small image from fw-gpu/), and everything the GPU is given to do later goes through that
 * firmware's interface in memory both sides share.
 *
 * Nothing here is touched unless the boot-arg asks: mali=N performs steps 1 to N and stops, each saying what it is about
 * to touch before it does and what it found after; a step that does not find what it needs ends the sequence.
 *
 *   1  reads   the power domains (PMU), the GPU's identity and state (not the S2MPU in front of its memory accesses at
 *              0x1ee70000: a read of it from here is refused by the bus, an SError: it is the secure world's)
 *   2  writes  a soft reset of the GPU, waited for; coherency "none" (the GPU does not snoop the processors' caches);
 *              the L2's slice hash (zuma's values: without them most of memory is a bus fault for the GPU)
 *   3  writes  the L2 cache's power on, waited for (the MCU's firmware powers the shader cores itself)
 *   4  writes  the firmware's sections into memory of ours, with page tables for them, and address space 0 (the
 *              MCU's) pointed at those tables
 *   5  writes  the MCU started; its state, the firmware's first words in the shared interface (GLB_VERSION ...), and
 *              any fault, are read; then the MCU is stopped again and the GPU reset: nothing outlives the run
 *
 * What the hardware is, and where each value comes from: Google's Android kernel for this phone (google-modules/gpu,
 * android-gs-akita-android16: kbase r54p2; google-modules/soc/gs: the PMU's sequences), read for how the hardware
 * behaves -- that code is GPL and none of it is here. The values were checked on the phone with U-Boot first
 * (the port's log: the bootloader leaves the GPU's power domains on and its clocks running).
 *
 * More boot-args: mali_delay=SECONDS (8) after the kernel's start before step 1 runs (the small image has to be there
 * for the firmware).
 */
#include <kern/clock.h>
#include <kern/startup.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <mach/vm_param.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern uint64_t kvtophys(vm_offset_t va);
extern vm_offset_t phystokv(uint64_t pa);       /* pmap_paddr_t to the physical aperture */
extern void clean_dcache(vm_offset_t addr, unsigned count, int phys);
extern void flush_dcache(vm_offset_t addr, unsigned count, int phys);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

#define FW_NAME                 "mali_csffw-r54p2.bin"

/* the GPU's registers (0x1f000000, 0x480000 in Android's tree) */
#define GPU_PHYS                0x1f000000ULL
#define GPU_SIZE                0x10000         /* control, job, MMU: all of it below 0x10000 */
#define GPU_ID                  0x000
#define L2_FEATURES             0x004
#define CORE_FEATURES           0x008
#define MMU_FEATURES            0x014
#define AS_PRESENT              0x018
#define CSF_ID                  0x01c
#define GPU_IRQ_RAWSTAT         0x020
#define GPU_IRQ_CLEAR           0x024
#define GPU_IRQ_MASK            0x028
#define GPU_COMMAND             0x030
#define GPU_STATUS              0x034
#define GPU_FAULTSTATUS         0x03c
#define GPU_FAULTADDRESS        0x040           /* 64 bits */
#define L2_CONFIG               0x048
#define CYCLE_COUNT             0x090
#define SHADER_PRESENT          0x100
#define TILER_PRESENT           0x110
#define L2_PRESENT              0x120
#define SHADER_READY            0x140
#define TILER_READY             0x150
#define L2_READY                0x160
#define L2_PWRON                0x1a0
#define L2_PWRTRANS             0x220
#define ASN_HASH(n)             (0x2c0 + 4 * (n))   /* the L2's slice hash, v11's name */
#define COHERENCY_FEATURES      0x300
#define COHERENCY_ENABLE        0x304
#define MCU_CONTROL             0x700
#define MCU_STATUS              0x704
#define JOB_IRQ_RAWSTAT         0x1000
#define JOB_IRQ_CLEAR           0x1004
#define MMU_IRQ_RAWSTAT         0x2000
#define MMU_IRQ_CLEAR           0x2004
#define AS0                     0x2400          /* address space 0: the MCU's */
#define AS_TRANSTAB             0x00            /* 64 bits */
#define AS_MEMATTR              0x08            /* 64 bits */
#define AS_COMMAND              0x18
#define AS_FAULTSTATUS          0x1c
#define AS_FAULTADDRESS         0x20            /* 64 bits */
#define AS_STATUS               0x28
#define AS_TRANSCFG             0x30            /* 64 bits */

/*
 * Which L2 slice (and so which of the GPU's bus ports) serves an address: zuma's values, Android's tree
 * ("l2-hash-values" of mali@1F000000), enabled by L2_CONFIG bit 24, after every reset and before the L2 is powered
 * (Google's driver: kbase_pm_l2_config_override). Without them the GPU's reads of four pages in five ended in bus
 * faults, in a pattern of the address (the scan below).
 */
#define L2_HASH_ENABLE          (1u << 24)
static const uint32_t l2_hash_values[3] = { 0x36db6d, 0x5b6db6, 0x0 };
#define CMD_SOFT_RESET          0x101           /* GPU_COMMAND: code 1 (reset), payload 1 (soft) */
#define IRQ_RESET_COMPLETED     (1u << 8)
#define COHERENCY_NONE          31
#define AS_CMD_UPDATE           1
#define AS_STATUS_ACTIVE        1u
#define MCU_REQ_DISABLE         0
#define MCU_REQ_AUTO            2               /* as Google's driver starts it */
#define MCU_ENABLED             1

/*
 * The memory attributes of an address space, eight of one byte, and which a page uses (bits 4:2 of its entry): Google's
 * table for a CSF GPU. 0x88 write-back, cache policy the GPU's own; 0x8f write-back allocating; 0x8d write-allocate;
 * 0x88 outer; 0x8d outer write-allocate; 0x4c non-cacheable; 0x08 shared with the host.
 */
#define MEMATTR_TABLE           0x00084c8d888d8f88ULL
#define ATTR_CACHED             0
#define ATTR_NON_CACHEABLE      5
#define ATTR_SHARED             6
/* TRANSCFG: AArch64 tables with a 4 KiB granule (6), the walk's reads write-back (2 << 24), read-allocating (1 << 30) */
#define TRANSCFG_VALUE          (6ULL | 2ULL << 24 | 1ULL << 30)

/* the GPU's page tables: the processors' AArch64 format, 4 KiB pages, four levels for 48 bits */
#define GPU_PAGE                4096u
#define PTE_VALID_TABLE         3ULL            /* a table at levels 0-2, a page at level 3 */
#define PTE_AF                  (1ULL << 10)
#define PTE_RW                  (1ULL << 6)
#define PTE_RO                  (3ULL << 6)
#define PTE_NX                  (1ULL << 54)
#define PTE_ADDR                0x000000fffffff000ULL

/* the firmware file (Google's loader: magic, header version 0.3, entries of a type, a size, and words) */
#define FW_MAGIC                0xc3f13a6eu
#define ENTRY_INTERFACE         0
#define IF_READ                 (1u << 0)
#define IF_WRITE                (1u << 1)
#define IF_EXECUTE              (1u << 2)
#define IF_CACHE(f)             (((f) >> 3) & 3)
#define IF_PROTECTED            (1u << 5)
#define IF_SHARED               (1u << 30)
#define IF_ZERO                 (1u << 31)
#define SHARED_IFACE_VA         0x04000000u     /* fixed by the firmware's specification */

/* the PMU (the power domains, read only here) */
#define PMU_G3D_STATUS          0x15461a84ULL
#define PMU_EMBEDDED_G3D_STATUS 0x15461c84ULL

static unsigned int steps, step;
static volatile uint8_t *gpu;

static uint32_t
rd(uint32_t off)
{
	return *(volatile uint32_t *)(gpu + off);
}

static void
wr(uint32_t off, uint32_t value)
{
	*(volatile uint32_t *)(gpu + off) = value;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static uint64_t
rd64(uint32_t off)
{
	return (uint64_t)rd(off) | (uint64_t)rd(off + 4) << 32;
}

static void
wr64(uint32_t off, uint64_t value)
{
	wr(off, (uint32_t)value);
	wr(off + 4, (uint32_t)(value >> 32));
}

/* waits up to max_us for (register & mask) == want; true if it came */
static bool
wait_for(uint32_t off, uint32_t mask, uint32_t want, unsigned int max_us)
{
	for (unsigned int t = 0; t < max_us; t += 10) {
		if ((rd(off) & mask) == want) {
			return true;
		}
		delay(10);
	}
	return (rd(off) & mask) == want;
}

/* after a reset, before the L2's power: coherency none, and the L2's slice hash */
static void
configure(void)
{
	wr(COHERENCY_ENABLE, COHERENCY_NONE);
	for (int i = 0; i < 3; i++) {
		wr(ASN_HASH(i), l2_hash_values[i]);
	}
	wr(L2_CONFIG, rd(L2_CONFIG) | L2_HASH_ENABLE);
}

static bool
announce(unsigned int n, const char *what)
{
	if (n > steps) {
		return false;
	}
	step = n;
	printf("mali: %u: %s\n", n, what);
	return true;
}

static void
faults(const char *when)
{
	printf("mali: %u: %s: GPU_STATUS 0x%x, GPU_IRQ_RAWSTAT 0x%x, GPU_FAULTSTATUS 0x%x at 0x%llx, MMU_IRQ_RAWSTAT 0x%x, "
	    "AS0 status 0x%x, fault 0x%x at 0x%llx, JOB_IRQ_RAWSTAT 0x%x, MCU_STATUS 0x%x\n", step, when, rd(GPU_STATUS),
	    rd(GPU_IRQ_RAWSTAT), rd(GPU_FAULTSTATUS), rd64(GPU_FAULTADDRESS), rd(MMU_IRQ_RAWSTAT), rd(AS0 + AS_STATUS),
	    rd(AS0 + AS_FAULTSTATUS), rd64(AS0 + AS_FAULTADDRESS), rd(JOB_IRQ_RAWSTAT), rd(MCU_STATUS));
}

/* ---- memory for the GPU: allocated in the kernel's pages (16 KiB), handed to the GPU in its own (4 KiB) ---- */

static vm_offset_t
gpu_memory(vm_size_t size)
{
	vm_offset_t va = 0;

	if (kmem_alloc(kernel_map, &va, round_page(size), KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		return 0;
	}
	return va;
}

/* the page tables: level 0 and every table under it, taken from pages of ours one 4 KiB table at a time */
static uint64_t *level0;
static vm_offset_t table_pool;
static vm_size_t table_left;
static unsigned int table_count;

static uint64_t *
new_table(void)
{
	uint64_t *table;

	if (table_left < GPU_PAGE) {
		table_pool = gpu_memory(PAGE_SIZE);
		table_left = table_pool ? PAGE_SIZE : 0;
		if (table_pool == 0) {
			return NULL;
		}
	}
	table = (uint64_t *)table_pool;
	table_pool += GPU_PAGE;
	table_left -= GPU_PAGE;
	table_count++;
	return table;
}

/* one 4 KiB page of the GPU's address space at the physical page phys */
static bool
map_page(uint64_t va, uint64_t phys, uint64_t attributes)
{
	uint64_t *table = level0;

	for (int level = 0; level < 3; level++) {
		uint64_t *entry = &table[(va >> (39 - 9 * level)) & 511];

		if ((*entry & 3) == 0) {
			uint64_t *next = new_table();

			if (next == NULL) {
				return false;
			}
			*entry = kvtophys((vm_offset_t)next) | PTE_VALID_TABLE;
		}
		table = (uint64_t *)phystokv(*entry & PTE_ADDR);
	}
	table[(va >> 12) & 511] = (phys & PTE_ADDR) | attributes | PTE_AF | PTE_VALID_TABLE;
	return true;
}

static void
clean_tables(void)
{
	/* walk level 0 to 2 and clean every table found, level 0 too */
	clean_dcache((vm_offset_t)level0, GPU_PAGE, FALSE);
	for (int i = 0; i < 512; i++) {
		if ((level0[i] & 3) == 0) {
			continue;
		}
		uint64_t *l1 = (uint64_t *)phystokv(level0[i] & PTE_ADDR);
		clean_dcache((vm_offset_t)l1, GPU_PAGE, FALSE);
		for (int j = 0; j < 512; j++) {
			if ((l1[j] & 3) == 0) {
				continue;
			}
			uint64_t *l2 = (uint64_t *)phystokv(l1[j] & PTE_ADDR);
			clean_dcache((vm_offset_t)l2, GPU_PAGE, FALSE);
			for (int k = 0; k < 512; k++) {
				if ((l2[k] & 3) != 0) {
					clean_dcache(phystokv(l2[k] & PTE_ADDR), GPU_PAGE, FALSE);
				}
			}
		}
	}
}

/* ---- the firmware ---- */

static vm_offset_t shared_iface;        /* the kernel's address of the firmware's shared interface, 0x04000000 */
static unsigned int section_count;

static bool
load_section(const uint8_t *fw, uint64_t fw_size, const uint32_t *w)
{
	uint32_t flags = w[0], va = w[1], va_end = w[2], data = w[3], data_end = w[4];
	vm_size_t size = va_end - va;
	vm_offset_t memory;
	uint64_t attributes;
	unsigned int cache = IF_CACHE(flags);

	if (va_end < va || data_end < data || data_end > fw_size || (va | va_end) & (GPU_PAGE - 1)) {
		printf("mali: %u: a section that does not hold together: va 0x%x-0x%x, file 0x%x-0x%x\n", step, va, va_end,
		    data, data_end);
		return false;
	}
	if (flags & IF_PROTECTED) {
		printf("mali: %u: va 0x%08x-0x%08x: protected, left out (no protected memory here, as Google's driver without its "
		    "allocator)\n", step, va, va_end);
		return true;
	}
	if (data_end - data > size) {
		printf("mali: %u: va 0x%08x: more data (%u) than room (%lu)\n", step, va, data_end - data, (unsigned long)size);
		return false;
	}
	memory = gpu_memory(size);
	if (memory == 0) {
		printf("mali: %u: no memory for va 0x%08x (%lu bytes)\n", step, va, (unsigned long)size);
		return false;
	}
	memcpy((void *)memory, fw + data, data_end - data);    /* the rest stays zero (kmem_alloc's KMA_ZERO) */
	clean_dcache(memory, (unsigned)round_page(size), FALSE);

	attributes = (flags & IF_WRITE) ? PTE_RW : PTE_RO;
	if (!(flags & IF_EXECUTE)) {
		attributes |= PTE_NX;
	}
	/* the cache mode: 0 none, 1 cached, 2 uncached coherent, 3 cached coherent (the shared interface) */
	attributes |= (uint64_t)(cache == 1 ? ATTR_CACHED : cache == 3 ? ATTR_SHARED : ATTR_NON_CACHEABLE) << 2;
	for (vm_size_t off = 0; off < size; off += GPU_PAGE) {
		if (!map_page(va + off, kvtophys(memory + off), attributes)) {
			printf("mali: %u: no memory for page tables\n", step);
			return false;
		}
	}
	if (va == SHARED_IFACE_VA) {
		shared_iface = memory;
	}
	section_count++;
	printf("mali: %u: va 0x%08x-0x%08x %c%c%c cache %u%s%s: %u bytes of the file, at 0x%llx\n", step, va, va_end,
	    flags & IF_READ ? 'r' : '-', flags & IF_WRITE ? 'w' : '-', flags & IF_EXECUTE ? 'x' : '-', cache,
	    flags & IF_SHARED ? " shared" : "", flags & IF_ZERO ? " zeroed" : "", data_end - data, kvtophys(memory));
	return true;
}

static bool
load_firmware(void)
{
	uint64_t size = 0;
	const uint8_t *fw = hakios_image_file("fw", FW_NAME, &size, NULL);
	uint32_t magic, end, hash;

	if (fw == NULL) {
		printf("mali: %u: no fw/%s in the small image: put the phone's /vendor/firmware/%s into fw-gpu/ and build the "
		    "user space again\n", step, FW_NAME, FW_NAME);
		return false;
	}
	if (size < 0x14) {
		return false;
	}
	memcpy(&magic, fw, 4);
	memcpy(&hash, fw + 8, 4);
	memcpy(&end, fw + 0x10, 4);
	printf("mali: %u: fw/%s: %llu bytes, magic 0x%08x, header %u.%u, hash 0x%08x, entries to 0x%x\n", step, FW_NAME,
	    size, magic, fw[5], fw[4], hash, end);
	if (magic != FW_MAGIC || fw[5] != 0 || fw[4] != 3 || end > size) {
		printf("mali: %u: not a firmware image this driver reads\n", step);
		return false;
	}
	level0 = new_table();
	if (level0 == NULL) {
		return false;
	}
	for (uint32_t off = 0x14; off < end;) {
		uint32_t header, entry_size;

		memcpy(&header, fw + off, 4);
		entry_size = (header >> 8) & 0xff;
		if (entry_size < 4 || off + entry_size > end) {
			printf("mali: %u: entry at 0x%x: size %u\n", step, off, entry_size);
			return false;
		}
		if ((header & 0xff) == ENTRY_INTERFACE) {
			uint32_t w[5];

			if (entry_size < 4 + sizeof(w)) {
				return false;
			}
			memcpy(w, fw + off + 4, sizeof(w));
			if (!load_section(fw, size, w)) {
				return false;
			}
		}
		off += entry_size;
	}
	if (shared_iface == 0) {
		printf("mali: %u: no section at 0x%08x, the shared interface\n", step, SHARED_IFACE_VA);
		return false;
	}
	clean_tables();
	printf("mali: %u: %u sections, %u page tables; level 0 at 0x%llx\n", step, section_count, table_count,
	    kvtophys((vm_offset_t)level0));
	return true;
}

/*
 * Which of our pages may the GPU read? (mali_scan=N, after step 3.) The first run's MCU started, and the walk of its
 * address space read the level-0 table and then took a bus fault (GPU_FAULTSTATUS 0x80, GPU_BUS_FAULT) on the level-1
 * table, 4 KiB further in the same page of the kernel's: what an S2MPU (the stage-2 protection in front of the GPU's
 * accesses, 0x1ee70000, unreadable from here) allowing some 4 KiB pages and not others would do. So each 4 KiB piece of
 * N fresh pages of ours is made the level-0 table of address space 0, zeroed, and the MCU started: a readable table is
 * a translation fault at level 0 (its entry invalid); one the bus refuses is a GPU bus fault at its address.
 */
static int
probe_page(uint64_t phys)
{
	int result;

	if (rd(MCU_STATUS) != 0) {
		wr(MCU_CONTROL, MCU_REQ_DISABLE);
	}
	wr(GPU_IRQ_CLEAR, 0xffffffff);
	wr(GPU_COMMAND, CMD_SOFT_RESET);
	if (!wait_for(GPU_IRQ_RAWSTAT, IRQ_RESET_COMPLETED, IRQ_RESET_COMPLETED, 100000)) {
		return -1;
	}
	wr(GPU_IRQ_CLEAR, 0xffffffff);
	configure();
	wr64(L2_PWRON, rd64(L2_PRESENT));
	if (!wait_for(L2_READY, 1, 1, 100000)) {
		return -2;
	}
	wr64(AS0 + AS_TRANSCFG, TRANSCFG_VALUE);
	wr64(AS0 + AS_TRANSTAB, phys);
	wr64(AS0 + AS_MEMATTR, MEMATTR_TABLE);
	wr(AS0 + AS_COMMAND, AS_CMD_UPDATE);
	if (!wait_for(AS0 + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000)) {
		return -3;
	}
	wr(MMU_IRQ_CLEAR, 0xffffffff);
	wr(MCU_CONTROL, MCU_REQ_AUTO);
	for (int t = 0; t < 200 && rd(MMU_IRQ_RAWSTAT) == 0 && !(rd(GPU_IRQ_RAWSTAT) & 1); t++) {
		delay(10);
	}
	if ((rd(GPU_FAULTSTATUS) & 0xff) == 0x80 && (rd64(GPU_FAULTADDRESS) & ~0xfffULL) == phys) {
		result = 0;     /* refused */
	} else if ((rd(AS0 + AS_FAULTSTATUS) & 0xff) == 0xc0) {
		result = 1;     /* read, and its entry found invalid */
	} else {
		printf("mali: scan: 0x%llx: neither: ", phys);
		faults("scan");
		result = 2;
	}
	wr(MCU_CONTROL, MCU_REQ_DISABLE);
	return result;
}

static void
scan(unsigned int pages)
{
	unsigned int readable = 0, refused = 0, other = 0;

	printf("mali: scan: %u pages of ours, each 4 KiB of them as the MCU's level-0 table\n", pages);
	for (unsigned int i = 0; i < pages; i++) {
		vm_offset_t page = gpu_memory(PAGE_SIZE);
		char line[8];

		if (page == 0) {
			break;
		}
		clean_dcache(page, PAGE_SIZE, FALSE);
		for (unsigned int k = 0; k < PAGE_SIZE / GPU_PAGE; k++) {
			int r = probe_page(kvtophys(page + k * GPU_PAGE));

			line[k] = r == 1 ? 'r' : r == 0 ? '-' : '?';
			readable += r == 1;
			refused += r == 0;
			other += r != 0 && r != 1;
		}
		line[PAGE_SIZE / GPU_PAGE] = 0;
		printf("mali: scan: 0x%llx: %s\n", kvtophys(page), line);
	}
	printf("mali: scan: %u readable by the GPU, %u refused, %u neither (r: read, -: refused)\n", readable, refused,
	    other);
}

/*
 * The S2MPU in front of the GPU (google,s2mpu-v9 at 0x1ee70000, Android's tree): Google's stage-2 protection, which
 * on Android only its hypervisor (pKVM) programs. Its registers as that driver names them (google-modules/soc/gs,
 * drivers/soc/google/pkvm-s2mpu/common/kvm_s2mpu.h); an S2MPU resets to blocking every access, and v9's protection is
 * switched on per VID (CTRL_PROT_EN_PER_VID_SET 0x50, ..._CLR 0x54): off, everything passes. The scan above found it
 * on with a table that lets a fifth of our pages through. An earlier read of every word from 0 to 0x1c ended the run
 * with an SError, and so did a read of CTRL0..STATUS: only what Google's v9 code reads is read here (VERSION, the faults).
 *   mali_s2mpu=1  read them; 2  also switch the protection off for every VID and invalidate its caches
 */
#define S2MPU_G3D_PHYS          0x1ee70000ULL
#define S2MPU_CTRL0             0x0
#define S2MPU_CTRL1             0x4
#define S2MPU_CFG               0x10
#define S2MPU_VERSION           0x60
#define S2MPU_INFO              0x64
#define S2MPU_STATUS            0x68
#define S2MPU_PROT_EN_CLR       0x54
#define S2MPU_ALL_INVALIDATION  0x1000
#define S2MPU_FAULT_STATUS      0x2000
#define S2MPU_L1ENTRY_ADDR(vid, gb)     (0x4000 + (vid) * 0x200 + (gb) * 8)
#define S2MPU_L1ENTRY_ATTR(vid, gb)     (0x4004 + (vid) * 0x200 + (gb) * 8)

static void
s2mpu(unsigned int level)
{
	volatile uint8_t *s = (volatile uint8_t *)ml_io_map(S2MPU_G3D_PHYS, 0x10000);
#define S2(off) (*(volatile uint32_t *)(s + (off)))

	printf("mali: s2mpu: reading its VERSION (0x60)\n");
	printf("mali: s2mpu: VERSION 0x%08x\n", S2(S2MPU_VERSION));
	/* only what Google's v9 code reads itself: a read of CTRL0, CTRL1, CFG, INFO or STATUS ended a run (an SError) */
	printf("mali: s2mpu: FAULT_STATUS 0x%x (a bit per VID that faulted)\n", S2(S2MPU_FAULT_STATUS));
	for (unsigned int vid = 0; vid < 8; vid++) {
		if (S2(S2MPU_FAULT_STATUS) & (1u << vid)) {
			printf("mali: s2mpu: vid %u: fault at 0x%x%08x, info 0x%x\n", vid, S2(0x2008 + vid * 0x20),
			    S2(0x2004 + vid * 0x20), S2(0x2010 + vid * 0x20));
		}
	}
	if (level < 2) {
		return;
	}
	printf("mali: s2mpu: switching the protection off for every VID\n");
	S2(S2MPU_PROT_EN_CLR) = 0xff;
	S2(S2MPU_ALL_INVALIDATION) = 1;
	__asm__ volatile ("dsb sy" ::: "memory");
	delay(100);
	printf("mali: s2mpu: done; FAULT_STATUS 0x%x\n", S2(S2MPU_FAULT_STATUS));
#undef S2
}

/* ---- the steps ---- */

static void
stop(void)
{
	if (rd(MCU_STATUS) != 0) {
		wr(MCU_CONTROL, MCU_REQ_DISABLE);
		if (!wait_for(MCU_STATUS, 3, 0, 100000)) {
			printf("mali: stop: the MCU did not stop (MCU_STATUS 0x%x)\n", rd(MCU_STATUS));
		}
	}
	wr(GPU_IRQ_CLEAR, 0xffffffff);
	wr(GPU_COMMAND, CMD_SOFT_RESET);
	printf("mali: stop: the GPU reset again: %s\n", wait_for(GPU_IRQ_RAWSTAT, IRQ_RESET_COMPLETED, IRQ_RESET_COMPLETED,
	    100000) ? "done" : "NOT COMPLETED");
	wr(GPU_IRQ_CLEAR, 0xffffffff);
}

static void
mali_run(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	volatile uint32_t *pmu_g3d = (volatile uint32_t *)ml_io_map(PMU_G3D_STATUS & ~0xfffULL, 0x1000);
	bool started = false;

	if (!announce(1, "reading the power domains and the GPU")) {
		return;
	}
	printf("mali: 1: PMU: G3D status 0x%x, embedded G3D status 0x%x\n", pmu_g3d[(PMU_G3D_STATUS & 0xfff) / 4],
	    pmu_g3d[(PMU_EMBEDDED_G3D_STATUS & 0xfff) / 4]);
	if (!(pmu_g3d[(PMU_G3D_STATUS & 0xfff) / 4] & 1) || !(pmu_g3d[(PMU_EMBEDDED_G3D_STATUS & 0xfff) / 4] & 1)) {
		printf("mali: 1: the GPU's power is off (the bootloader left it on so far): not reading it\n");
		return;
	}
	gpu = (volatile uint8_t *)ml_io_map(GPU_PHYS, GPU_SIZE);
	printf("mali: 1: GPU_ID 0x%08x (arch %u.%u, rev %u, product %u, r%up%u), L2_FEATURES 0x%08x, CORE_FEATURES 0x%x, "
	    "MMU_FEATURES 0x%x, AS_PRESENT 0x%x, CSF_ID 0x%08x\n", rd(GPU_ID), rd(GPU_ID) >> 28, rd(GPU_ID) >> 24 & 15,
	    rd(GPU_ID) >> 20 & 15, rd(GPU_ID) >> 16 & 15, rd(GPU_ID) >> 12 & 15, rd(GPU_ID) >> 4 & 0xff, rd(L2_FEATURES),
	    rd(CORE_FEATURES), rd(MMU_FEATURES), rd(AS_PRESENT), rd(CSF_ID));
	printf("mali: 1: present: shaders 0x%llx, tiler 0x%llx, L2 0x%llx; ready: shaders 0x%llx, tiler 0x%llx, L2 0x%llx; "
	    "coherency features 0x%x, enabled 0x%x; L2_CONFIG 0x%x\n", rd64(SHADER_PRESENT), rd64(TILER_PRESENT),
	    rd64(L2_PRESENT), rd64(SHADER_READY), rd64(TILER_READY), rd64(L2_READY), rd(COHERENCY_FEATURES),
	    rd(COHERENCY_ENABLE), rd(L2_CONFIG));
	faults("as found");
	{
		uint32_t c0 = rd(CYCLE_COUNT);

		delay(1000);
		printf("mali: 1: the cycle counter: %u in a millisecond (the GPU's clock, in kHz)\n", rd(CYCLE_COUNT) - c0);
	}

	if (!announce(2, "a soft reset of the GPU")) {
		return;
	}
	wr(GPU_IRQ_MASK, 0);
	wr(GPU_IRQ_CLEAR, 0xffffffff);
	wr(GPU_COMMAND, CMD_SOFT_RESET);
	if (!wait_for(GPU_IRQ_RAWSTAT, IRQ_RESET_COMPLETED, IRQ_RESET_COMPLETED, 100000)) {
		faults("the reset did not complete");
		return;
	}
	wr(GPU_IRQ_CLEAR, 0xffffffff);
	configure();
	printf("mali: 2: reset; coherency enabled now 0x%x; L2_CONFIG 0x%x, slice hash %x %x %x; GPU_ID still 0x%08x\n",
	    rd(COHERENCY_ENABLE), rd(L2_CONFIG), rd(ASN_HASH(0)), rd(ASN_HASH(1)), rd(ASN_HASH(2)), rd(GPU_ID));

	if (!announce(3, "the L2 cache's power on")) {
		return;
	}
	wr64(L2_PWRON, rd64(L2_PRESENT));
	if (!wait_for(L2_READY, (uint32_t)rd64(L2_PRESENT), (uint32_t)rd64(L2_PRESENT), 100000)) {
		printf("mali: 3: L2 not ready: ready 0x%llx, in transition 0x%llx\n", rd64(L2_READY), rd64(L2_PWRTRANS));
		faults("L2");
		return;
	}
	printf("mali: 3: L2 ready 0x%llx\n", rd64(L2_READY));
	{
		unsigned int scan_pages = 0;

		unsigned int open_s2mpu = 0;

		if (PE_parse_boot_argn("mali_s2mpu", &open_s2mpu, sizeof(open_s2mpu)) && open_s2mpu > 0) {
			s2mpu(open_s2mpu);
		}
		if (PE_parse_boot_argn("mali_scan", &scan_pages, sizeof(scan_pages)) && scan_pages > 0) {
			scan(scan_pages > 256 ? 256 : scan_pages);
			goto out;
		}
	}

	if (!announce(4, "the firmware into memory, page tables, address space 0")) {
		goto out;
	}
	if (!load_firmware()) {
		goto out;
	}
	if (!wait_for(AS0 + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000)) {
		faults("address space 0 busy");
		goto out;
	}
	wr64(AS0 + AS_TRANSCFG, TRANSCFG_VALUE);
	wr64(AS0 + AS_TRANSTAB, kvtophys((vm_offset_t)level0));
	wr64(AS0 + AS_MEMATTR, MEMATTR_TABLE);
	wr(AS0 + AS_COMMAND, AS_CMD_UPDATE);
	if (!wait_for(AS0 + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000)) {
		faults("the address space update did not complete");
		goto out;
	}
	printf("mali: 4: address space 0: transtab 0x%llx, memattr 0x%llx, transcfg 0x%llx\n", rd64(AS0 + AS_TRANSTAB),
	    rd64(AS0 + AS_MEMATTR), rd64(AS0 + AS_TRANSCFG));

	if (!announce(5, "the MCU started")) {
		goto out;
	}
	wr(JOB_IRQ_CLEAR, 0xffffffff);
	wr(MMU_IRQ_CLEAR, 0xffffffff);
	wr(MCU_CONTROL, MCU_REQ_AUTO);
	started = true;
	if (!wait_for(MCU_STATUS, 3, MCU_ENABLED, 1000000)) {
		faults("the MCU did not come up");
		goto out;
	}
	printf("mali: 5: the MCU is enabled\n");
	for (int i = 0; i < 100; i++) {         /* up to a second for the firmware to fill in its interface */
		flush_dcache(shared_iface, GPU_PAGE, FALSE);
		if (*(volatile uint32_t *)shared_iface != 0) {
			break;
		}
		delay(10000);
	}
	{
		volatile uint32_t *g = (volatile uint32_t *)shared_iface;

		printf("mali: 5: GLB_VERSION 0x%08x (%u.%u.%u), GLB_FEATURES 0x%x, input at 0x%08x, output at 0x%08x, "
		    "groups %u, ... %08x %08x %08x %08x\n", g[0], g[0] >> 24, g[0] >> 16 & 0xff, g[0] & 0xffff, g[1], g[2], g[3],
		    g[4], g[5], g[6], g[7], g[8]);
		printf("mali: 5: %s\n", g[0] != 0 ? "THE FIRMWARE RUNS: its global interface is filled in" :
		    "the shared interface stayed empty");
	}
	faults("after the start");
	printf("mali: 5: shaders ready 0x%llx, tiler ready 0x%llx\n", rd64(SHADER_READY), rd64(TILER_READY));

out:
	if (step >= 2) {
		stop();
	}
	(void)started;
}

static void
akita_mali_init(void)
{
	static thread_call_t call;
	uint32_t delay_s = 8;
	uint64_t deadline;

	if (!PE_parse_boot_argn("mali", &steps, sizeof(steps)) || steps == 0) {
		return;
	}
	PE_parse_boot_argn("mali_delay", &delay_s, sizeof(delay_s));
	printf("mali: steps 1 to %u in %u s\n", steps, delay_s);
	call = thread_call_allocate(mali_run, NULL);
	clock_interval_to_deadline(delay_s, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_mali_init);
