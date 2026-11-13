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
 *              any fault, are read
 *   6  talks   to the firmware's global interface: a ping, the shader cores allowed
 *   7  writes  a command stream group (slot 0) with one queue, in address space 1 (the same tables), started
 *   8  runs    the first job: four command-stream instructions storing two words into a page of ours, checked
 *   9  runs    the first compute job: a four-instruction shader (assembled by Mesa) storing a word, then a marker
 * After the last step the MCU is stopped again and the GPU reset: nothing outlives the run.
 *
 * What the hardware is, and where each value comes from: Google's Android kernel for this phone (google-modules/gpu,
 * android-gs-akita-android16: kbase r54p2; google-modules/soc/gs: the PMU's sequences), read for how the hardware
 * behaves -- that code is GPL and none of it is here. The values were checked on the phone with U-Boot first
 * (the port's log: the bootloader leaves the GPU's power domains on and its clocks running).
 *
 * More boot-args: mali_delay=SECONDS (8) after the kernel's start before step 1 runs (the small image has to be there
 * for the firmware).
 */
#include <os/atomic_private.h>
#include <kern/clock.h>
#include <kern/startup.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <libkern/OSAtomic.h>
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
#define SHADER_PWRON            0x180
#define TILER_PWRON             0x190
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
#define AS_CMD_LOCK             2
#define AS_CMD_UNLOCK           3
#define AS_LOCKADDR             0x10            /* 64 bits: a region's base | log2(its size) - 1 */
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
/* outer shareable, as Linux's io-pgtable maps a Mali's pages (ARM_LPAE_PTE_SH_OS): Mali's inner domain is the GPU's
 * own cores, and memory of neither domain was not even coherent among those -- the tiler read what the shader cores
 * had written to a geometry buffer or tiler heap as memory held it before */
#define PTE_SH_OS               (2ULL << 8)
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
	    "AS0 status 0x%x, fault 0x%x at 0x%llx, AS1 fault 0x%x at 0x%llx, JOB_IRQ_RAWSTAT 0x%x, MCU_STATUS 0x%x\n", step,
	    when, rd(GPU_STATUS), rd(GPU_IRQ_RAWSTAT), rd(GPU_FAULTSTATUS), rd64(GPU_FAULTADDRESS), rd(MMU_IRQ_RAWSTAT),
	    rd(AS0 + AS_STATUS), rd(AS0 + AS_FAULTSTATUS), rd64(AS0 + AS_FAULTADDRESS), rd(AS0 + 0x40 + AS_FAULTSTATUS),
	    rd64(AS0 + 0x40 + AS_FAULTADDRESS), rd(JOB_IRQ_RAWSTAT), rd(MCU_STATUS));
}

/* ---- memory for the GPU: allocated in the kernel's pages (16 KiB), handed to the GPU in its own (4 KiB) ---- */

static uint64_t gpu_memory_bytes;      /* (11-10) what gpu_memory holds now: firmware, page tables, heaps, rings */

static vm_offset_t
gpu_memory(vm_size_t size)
{
	vm_offset_t va = 0;

	if (kmem_alloc(kernel_map, &va, round_page(size), KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		return 0;
	}
	os_atomic_add(&gpu_memory_bytes, (uint64_t)round_page(size), relaxed);
	return va;
}

/* the driver's own memory for the GPU (not the programs' buffers: HakiosGPU counts those) */
uint64_t
akita_mali_kernel_bytes(void)
{
	return os_atomic_load(&gpu_memory_bytes, relaxed);
}

/* the page tables: level 0 and every table under it, taken from pages of ours one 4 KiB table at a time */
static uint64_t *level0;
static vm_offset_t table_pool;
static vm_size_t table_left;
static unsigned int table_count;

/* tables given back (a VM that went: free_tables), linked through their first word */
static uint64_t *table_free;

static uint64_t *
new_table(void)
{
	uint64_t *table;

	if (table_free != NULL) {
		table = table_free;
		table_free = (uint64_t *)(uintptr_t)table[0];
		bzero(table, GPU_PAGE);
		return table;
	}
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
/* every region mapped for the GPU: its GPU address, size and where the kernel sees it (the MCU's addresses in the
 * firmware's interface blocks are looked up here) */
static struct { uint64_t va; vm_size_t size; vm_offset_t kva; } regions[16];
static unsigned int region_count;

static void
remember(uint64_t va, vm_size_t size, vm_offset_t kva)
{
	if (region_count < 16) {
		regions[region_count].va = va;
		regions[region_count].size = size;
		regions[region_count].kva = kva;
		region_count++;
	}
}

static void *
kernel_address(uint64_t va)
{
	for (unsigned int i = 0; i < region_count; i++) {
		if (va >= regions[i].va && va - regions[i].va < regions[i].size) {
			return (void *)(regions[i].kva + (va - regions[i].va));
		}
	}
	return NULL;
}

/*
 * The firmware's interface (steps 6 to 8), as Arm's CSF defines it and Google's driver uses it: blocks in memory the
 * MCU and we share -- the global control block at 0x04000000 names its input (ours to write) and output (the
 * firmware's) blocks and the groups'; a request is a bit of an input word toggled against the same bit of the output
 * word, followed by a doorbell, and done when the firmware has toggled the output bit to match.
 */
#define DOORBELL_PHYS           (GPU_PHYS + 0x80000)    /* doorbell n at + n * 0x10000; 0 is the kernel's */
#define USER_DOORBELL           1
/* global control */
#define GLB_VERSION_W           0
#define GLB_INPUT_VA_W          2
#define GLB_OUTPUT_VA_W         3
#define GLB_GROUP_NUM_W         4
#define GLB_GROUP_STRIDE_W      5
#define GROUP_CONTROL_0         0x1000
/* global input / output */
#define GLB_REQ                 0x00
#define GLB_ACK_IRQ_MASK        0x04
#define GLB_DB_REQ              0x08
#define GLB_ALLOC_EN_LO         0x18
#define GLB_ALLOC_EN_HI         0x1c
#define GLB_ACK                 0x00
#define GLB_DB_ACK              0x08
#define GLB_FATAL_STATUS        0x10
#define GLB_REQ_CFG_ALLOC_EN    (1u << 2)
#define GLB_REQ_CFG_PWROFF_TIMER (1u << 3)
#define GLB_REQ_CFG_PROGRESS_TIMER (1u << 1)
#define GLB_PROGRESS_TIMER      0x10
#define GLB_PWROFF_TIMER        0x14
#define GLB_REQ_PING            (1u << 8)
/* group control, input, output */
#define GROUP_FEATURES          0x00
#define GROUP_INPUT_VA          0x04
#define GROUP_OUTPUT_VA         0x08
#define GROUP_SUSPEND_SIZE      0x0c
#define GROUP_STREAM_NUM        0x14
#define GROUP_STREAM_STRIDE     0x18
#define STREAM_CONTROL_0        0x40
#define CSG_REQ                 0x00
#define CSG_ACK_IRQ_MASK        0x04
#define CSG_DB_REQ              0x08
#define CSG_ALLOW_COMPUTE_LO    0x20
#define CSG_ALLOW_COMPUTE_HI    0x24
#define CSG_ALLOW_FRAGMENT_LO   0x28
#define CSG_ALLOW_FRAGMENT_HI   0x2c
#define CSG_ALLOW_OTHER         0x30
#define CSG_EP_REQ_LO           0x34
#define CSG_EP_REQ_HI           0x38
#define CSG_SUSPEND_BUF_LO      0x40
#define CSG_SUSPEND_BUF_HI      0x44
#define CSG_CONFIG              0x50
#define CSG_ACK                 0x00
#define CSG_IRQ_REQ             0x0c    /* (output) */
#define CSG_IRQ_ACK             0x0c    /* (input) */
#define CSG_REQ_IDLE            (1u << 29)
#define CSG_REQ_EP_CFG          (1u << 4)
#define CSG_DB_ACK              0x08
#define CSG_STATE_START         1
/* stream control, input, output */
#define STREAM_FEATURES         0x00
#define STREAM_INPUT_VA         0x04
#define STREAM_OUTPUT_VA        0x08
#define CS_REQ                  0x00
#define CS_CONFIG               0x04
#define CS_ACK_IRQ_MASK         0x0c
#define CS_BASE_LO              0x10
#define CS_BASE_HI              0x14
#define CS_SIZE                 0x18
#define CS_USER_INPUT_LO        0x30
#define CS_USER_INPUT_HI        0x34
#define CS_USER_OUTPUT_LO       0x38
#define CS_USER_OUTPUT_HI       0x3c
#define CS_ACK                  0x00
#define CS_STATUS_CMD_PTR_LO    0x40
#define CS_STATUS_WAIT          0x48
#define CS_STATUS_BLOCKED       0x60
#define CS_FAULT                0x80
#define CS_FATAL                0x84
#define CS_STATE_START          1
#define CS_REQ_IDLE_BITS        ((1u << 8) | (1u << 10) | (1u << 12))
/* the queue's user pages: input (CS_INSERT, CS_EXTRACT_INIT), output (CS_EXTRACT, CS_ACTIVE) */
#define CS_INSERT               0x00
#define CS_EXTRACT              0x00
#define CS_ACTIVE               0x08
/*
 * Our own region of the GPU's address space, above the firmware's sections and within the MCU's: it is a 32-bit
 * processor, and the queue's user pages, the suspend buffer and the log are reached by it (Google's driver keeps
 * them in a zone from 0x04000000). At 0x1_0000_0000 the log's address reached the firmware cut to 32 bits and it
 * died at boot; the group's start had ended in an AS1 bus error and the MCU fatal there too.
 */
#define OURS_VA                 0x05000000ULL   /* and within the MCU's 28 bits: at 0x10000000 the log's address
                                                 * reached it as 0x00220000 (an AS0 translation fault there) */
#define RING_VA                 (OURS_VA + 0x00000)
#define TARGET_VA               (OURS_VA + 0x10000)
#define USER_IO_VA              (OURS_VA + 0x20000)     /* input page, then output page */
#define SUSPEND_VA              (OURS_VA + 0x100000)
#define SUSPEND_SIZE            0x10000u                /* group 0 asked for 26368 bytes */
#define CODE_VA                 (OURS_VA + 0x30000)     /* shaders: executable, read only for the GPU */
#define DESC_VA                 (OURS_VA + 0x40000)     /* a compute job's descriptors and push constants */
#define FWLOG_VA                (OURS_VA + 0x200000)    /* the firmware's log: its text */
#define FWLOG_SIZE              0x10000u
#define FWLOG_PTRS_VA           (OURS_VA + 0x220000)    /* its insert (the firmware's) and extract (ours) words */

static volatile uint32_t *glb_in, *glb_out, *csg_in, *csg_out, *cs_in, *cs_out, *user_in, *user_out;
static uint32_t *glb_ctl;
static uint32_t *ring, *target, *code, *desc;
static unsigned int ring_used;     /* bytes of the ring the jobs so far took */
static volatile uint8_t *doorbells;
static void fwlog_dump(const char *when);
#define MCU_ZONE_BASE           0x05400000u             /* the MCU's: groups' user pages and suspend buffers */
#define MCU_POOL_SIZE           0xc0000u                /* 8 groups x (64 KiB + 4 queues x 8 KiB) */
static vm_offset_t mcu_pool;
static char *fwlog;                     /* the firmware's log text, and its two pointers */
static volatile uint32_t *fwlog_insert, *fwlog_extract;

/* the processors' caches against memory the MCU reads or writes without them */
/* (whole cache lines from the first to the last byte: no further, a region may end at a guard page) */
static void
to_gpu(volatile void *p, unsigned int len)
{
	vm_offset_t start = (vm_offset_t)p & ~63UL, end = ((vm_offset_t)p + len + 63) & ~63UL;

	clean_dcache(start, (unsigned)(end - start), FALSE);
	__asm__ volatile ("dsb sy" ::: "memory");
}

static void
from_gpu(volatile void *p, unsigned int len)
{
	vm_offset_t start = (vm_offset_t)p & ~63UL, end = ((vm_offset_t)p + len + 63) & ~63UL;

	__asm__ volatile ("dsb sy" ::: "memory");
	flush_dcache(start, (unsigned)(end - start), FALSE);
}

static void
ring_doorbell(unsigned int n)
{
	__asm__ volatile ("dsb sy" ::: "memory");
	*(volatile uint32_t *)(doorbells + n * 0x10000) = 1;
	__asm__ volatile ("dsb sy" ::: "memory");
}

/* the GPU's L2 and load/store caches cleaned and invalidated, waited for */
#define CMD_FLUSH_L2_LSC        0x23304         /* code 4 (flush caches): L2 and LSC clean+invalidate, and the "other"
                                                 * caches invalidated -- among them the command stream's own fetches:
                                                 * without it a submission's first instructions, written into a ring
                                                 * line the stream had already fetched (as empty ones), were lost */
#define IRQ_CLEAN_CACHES_DONE   (1u << 17)
static void
flush_gpu(void)
{
	wr(GPU_IRQ_CLEAR, IRQ_CLEAN_CACHES_DONE);
	wr(GPU_COMMAND, CMD_FLUSH_L2_LSC);
	if (!wait_for(GPU_IRQ_RAWSTAT, IRQ_CLEAN_CACHES_DONE, IRQ_CLEAN_CACHES_DONE, 100000)) {
		printf("mali: %u: the GPU's cache flush did not complete\n", step);
	}
	wr(GPU_IRQ_CLEAR, IRQ_CLEAN_CACHES_DONE);
}

/* a region of ours mapped for the GPU as the firmware's own sections are (index 0, the GPU's cache policy: index 5,
 * non-cacheable, and 6, shared, both ended the first group's start in an AS1 bus error and the MCU fatal); what the
 * GPU writes is read after its caches are flushed (flush_gpu); the kernel's address */
static vm_offset_t
map_region_x(uint64_t va, vm_size_t size, bool exec)
{
	vm_offset_t memory = gpu_memory(size);

	if (memory == 0) {
		return 0;
	}
	clean_dcache(memory, (unsigned)round_page(size), FALSE);
	for (vm_size_t off = 0; off < size; off += GPU_PAGE) {
		if (!map_page(va + off, kvtophys(memory + off), (exec ? PTE_RO : PTE_RW | PTE_NX) |
		    (uint64_t)ATTR_CACHED << 2)) {
			return 0;
		}
	}
	remember(va, size, memory);
	return memory;
}

static vm_offset_t
map_region(uint64_t va, vm_size_t size)
{
	return map_region_x(va, size, false);
}

/* waits for the output word's bits under mask to equal the input word's: the firmware has done what was asked */
static bool
acked(volatile uint32_t *in, volatile uint32_t *out, uint32_t mask, unsigned int max_ms)
{
	for (unsigned int t = 0; t <= max_ms; t++) {
		from_gpu(out, 4);
		if ((*out & mask) == (*in & mask)) {
			return true;
		}
		delay(1000);
	}
	return false;
}

static void
fw_state(const char *when)
{
	from_gpu(glb_out, 64);
	printf("mali: %u: %s: GLB_REQ 0x%x ACK 0x%x DB_REQ 0x%x DB_ACK 0x%x FATAL 0x%x", step, when, glb_in[0], glb_out[0],
	    glb_in[GLB_DB_REQ / 4], glb_out[GLB_DB_ACK / 4], glb_out[GLB_FATAL_STATUS / 4]);
	if (csg_out != NULL) {
		from_gpu(csg_out, 64);
		printf("; CSG_REQ 0x%x ACK 0x%x DB_REQ 0x%x DB_ACK 0x%x", csg_in[0], csg_out[0], csg_in[CSG_DB_REQ / 4],
		    csg_out[CSG_DB_ACK / 4]);
	}
	if (cs_out != NULL) {
		from_gpu(cs_out, 256);
		printf("; CS_REQ 0x%x ACK 0x%x cmd at 0x%x wait 0x%x blocked 0x%x fault 0x%x fatal 0x%x", cs_in[0], cs_out[0],
		    cs_out[CS_STATUS_CMD_PTR_LO / 4], cs_out[CS_STATUS_WAIT / 4], cs_out[CS_STATUS_BLOCKED / 4],
		    cs_out[CS_FAULT / 4], cs_out[CS_FATAL / 4]);
	}
	printf("\n");
	faults(when);
	fwlog_dump(when);
}

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
	remember(va, size, memory);
	if (va == SHARED_IFACE_VA) {
		shared_iface = memory;
	}
	section_count++;
	printf("mali: %u: va 0x%08x-0x%08x %c%c%c cache %u%s%s: %u bytes of the file, at 0x%llx\n", step, va, va_end,
	    flags & IF_READ ? 'r' : '-', flags & IF_WRITE ? 'w' : '-', flags & IF_EXECUTE ? 'x' : '-', cache,
	    flags & IF_SHARED ? " shared" : "", flags & IF_ZERO ? " zeroed" : "", data_end - data, kvtophys(memory));
	return true;
}

/*
 * The firmware's log ("fwlog", a trace buffer entry of the image: Google's driver, mali_kbase_csf_trace_buffer.c):
 * the entry names five words in the firmware's own data where the host puts, before the MCU starts, the buffer's
 * size, the addresses of the insert word (the firmware's) and of the extract word (ours), of the text, and the
 * enable bits (Pixel's driver turns all on); the "Log verbosity" configuration entry is set to its maximum.
 */
static void
fw_poke(uint32_t mcu_va, uint32_t value)
{
	uint32_t *p = kernel_address(mcu_va);

	if (p != NULL) {
		*p = value;
		clean_dcache((vm_offset_t)p & ~63UL, 64, FALSE);
	}
}

static void
fwlog_setup(const uint8_t *fw, uint32_t end)
{
	for (uint32_t off = 0x14; off < end;) {
		uint32_t header, w[8], size;

		memcpy(&header, fw + off, 4);
		size = (header >> 8) & 0xff;
		if (size < 4) {
			break;
		}
		memset(w, 0, sizeof(w));
		memcpy(w, fw + off + 4, MIN(size - 4, sizeof(w)));
		if ((header & 0xff) == 3 && size >= 4 + 7 * 4 && strncmp((const char *)fw + off + 4 + 7 * 4, "fwlog", 5) == 0) {
			vm_offset_t text = map_region(FWLOG_VA, FWLOG_SIZE);
			vm_offset_t ptrs = map_region(FWLOG_PTRS_VA, 0x2000);

			if (text != 0 && ptrs != 0) {
				fwlog = (char *)text;
				fwlog_insert = (volatile uint32_t *)ptrs;
				fwlog_extract = (volatile uint32_t *)(ptrs + GPU_PAGE);
				fw_poke(w[1], FWLOG_SIZE);
				fw_poke(w[2], (uint32_t)FWLOG_PTRS_VA);                 /* insert: the firmware writes it */
				fw_poke(w[3], (uint32_t)(FWLOG_PTRS_VA + GPU_PAGE));    /* extract: ours */
				fw_poke(w[4], (uint32_t)FWLOG_VA);
				for (uint32_t i = 0; i < (w[6] + 31) / 32; i++) {
					fw_poke(w[5] + 4 * i, 0xffffffff);
				}
				printf("mali: %u: the firmware's log: %u bytes at 0x%llx (%u enable bits)\n", step, FWLOG_SIZE,
				    FWLOG_VA, w[6]);
			}
		}
		if ((header & 0xff) == 1 && size >= 4 + 3 * 4 && strncmp((const char *)fw + off + 4 + 3 * 4, "Log verbosity", 13) == 0) {
			fw_poke(w[0], w[2]);
			printf("mali: %u: log verbosity %u (of %u..%u)\n", step, w[2], w[1], w[2]);
		}
		off += size;
	}
}

/* what the firmware logged since the last look, line by line */
static void
fwlog_dump(const char *when)
{
	uint32_t insert, extract, n = 0;
	char line[160];
	unsigned int len = 0;

	if (fwlog == NULL) {
		return;
	}
	flush_gpu();
	from_gpu(fwlog_insert, 8);
	from_gpu(fwlog_extract, 8);
	insert = *fwlog_insert;
	extract = *fwlog_extract;
	printf("mali: fwlog (%s): insert 0x%x, extract 0x%x\n", when, insert, extract);
	from_gpu(fwlog, FWLOG_SIZE);
	while (extract != insert && n++ < 8192) {
		char c = fwlog[extract % FWLOG_SIZE];

		extract = (extract + 1) % FWLOG_SIZE;
		if (c == '\n' || len == sizeof(line) - 1) {
			line[len] = 0;
			printf("mali: fw| %s\n", line);
			len = 0;
		} else if (c >= 32 && c < 127) {
			line[len++] = c;
		} else if (c != '\r' && c != 0) {
			len += snprintf(line + len, sizeof(line) - len, "\\x%02x", (uint8_t)c) > 0 ? 4 : 0;
			if (len >= sizeof(line) - 5) {
				len = sizeof(line) - 1;
			}
		}
	}
	if (len) {
		line[len] = 0;
		printf("mali: fw| %s\n", line);
	}
	*fwlog_extract = extract;
	to_gpu(fwlog_extract, 8);
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
	fwlog_setup(fw, end);
	/* the MCU's zone for groups (suspend buffers, queues' user pages): mapped now, before the MCU walks these tables --
	 * changing the MCU's own address space under it (as_update(0)) left a group unstarted */
	mcu_pool = map_region(MCU_ZONE_BASE, MCU_POOL_SIZE);
	if (mcu_pool == 0) {
		printf("mali: %u: no memory for the MCU's zone\n", step);
		return false;
	}
	/* our own regions too, before the MCU walks these tables: a table changed under it may stay stale in the L2 */
	ring = (uint32_t *)map_region(RING_VA, 0x10000);
	target = (uint32_t *)map_region(TARGET_VA, 0x1000);
	user_in = (volatile uint32_t *)map_region(USER_IO_VA, 0x2000);
	user_out = user_in ? user_in + GPU_PAGE / 4 : NULL;
	code = (uint32_t *)map_region_x(CODE_VA, 0x1000, true);
	desc = (uint32_t *)map_region(DESC_VA, 0x1000);
	if (ring == NULL || target == NULL || user_in == NULL || code == NULL || desc == NULL ||
	    map_region(SUSPEND_VA, SUSPEND_SIZE) == 0) {
		printf("mali: %u: no memory for the queue's regions\n", step);
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

static bool
global_setup(void)
{
	uint32_t *g = (uint32_t *)shared_iface;

	glb_ctl = g;
	from_gpu(g, GPU_PAGE);
	glb_in = kernel_address(g[GLB_INPUT_VA_W]);
	glb_out = kernel_address(g[GLB_OUTPUT_VA_W]);
	doorbells = (volatile uint8_t *)ml_io_map(DOORBELL_PHYS, 9 * 0x10000);  /* doorbell 0 and 1 + each of 8 groups (two only:
	                                                                     * group 1's doorbell 2 was a kernel data abort) */
	if (glb_in == NULL || glb_out == NULL) {
		printf("mali: 6: the global input/output blocks (0x%x, 0x%x) are in no section of ours\n", g[GLB_INPUT_VA_W],
		    g[GLB_OUTPUT_VA_W]);
		return false;
	}
	from_gpu(glb_in, GPU_PAGE);
	printf("mali: 6: version 0x%x, %u groups every 0x%x bytes; input 0x%x, output 0x%x; GLB_REQ 0x%x, GLB_ACK 0x%x\n",
	    g[GLB_VERSION_W], g[GLB_GROUP_NUM_W], g[GLB_GROUP_STRIDE_W], g[GLB_INPUT_VA_W], g[GLB_OUTPUT_VA_W], glb_in[0],
	    glb_out[0]);
	glb_in[GLB_ACK_IRQ_MASK / 4] = 0xffffffff;
	glb_in[GLB_REQ / 4] ^= GLB_REQ_PING;
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	if (!acked(&glb_in[GLB_REQ / 4], &glb_out[GLB_ACK / 4], GLB_REQ_PING, 100)) {
		fw_state("NO ANSWER to the ping");
		return false;
	}
	printf("mali: 6: the firmware answered the ping\n");
	glb_in[GLB_ALLOC_EN_LO / 4] = (uint32_t)rd64(SHADER_PRESENT);
	glb_in[GLB_ALLOC_EN_HI / 4] = (uint32_t)(rd64(SHADER_PRESENT) >> 32);
	glb_in[GLB_REQ / 4] ^= GLB_REQ_CFG_ALLOC_EN;
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	if (!acked(&glb_in[GLB_REQ / 4], &glb_out[GLB_ACK / 4], GLB_REQ_CFG_ALLOC_EN, 100)) {
		fw_state("the shader cores' allocation not acknowledged");
		return false;
	}
	printf("mali: 6: shader cores 0x%llx allowed\n", rd64(SHADER_PRESENT));
	/* the progress timer, the firmware's watchdog for work that stops moving (Google's set_timeout_global: its
	 * default 2.5e9 cycles, in units of 1024) -- left at 0 it may be "no time at all" for the firmware's scheduler */
	glb_in[GLB_PROGRESS_TIMER / 4] = (uint32_t)(2500000000ULL / 1024);
	glb_in[GLB_REQ / 4] ^= GLB_REQ_CFG_PROGRESS_TIMER;
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	if (!acked(&glb_in[GLB_REQ / 4], &glb_out[GLB_ACK / 4], GLB_REQ_CFG_PROGRESS_TIMER, 100)) {
		fw_state("the progress timer not acknowledged");
		return false;
	}
	/*
	 * THE SHADER CORES' POWER (TODO 15): left to the firmware -- its power-off timer turns the cores off when the GPU
	 * has been idle that long, and on again for work (Android's default policy; panthor's PWROFF_HYSTERESIS_US 10 ms).
	 * (Before the global setup was whole -- progress timer, allocation -- the cores stayed off and the first compute
	 * job waited for good; with it, since 11-09, it works.) Boot-args: mali_pwroff_us=N the idle time before the cores
	 * go off (default 10000); mali_hctl=1 the old way, the host powers them on once and they stay on.
	 */
	{
		unsigned int hctl = 0, pwroff_us = 10000;

		PE_parse_boot_argn("mali_hctl", &hctl, sizeof(hctl));
		PE_parse_boot_argn("mali_pwroff_us", &pwroff_us, sizeof(pwroff_us));
		if (!hctl) {
			/* in units of 1024 system timestamp ticks (24.576 MHz), source "timestamp" (bit 31 0) -- panthor's
			 * panthor_fw_conv_timeout: timeout * rate / (1e6 << 10). (Counted in plain ticks, "10 ms" was 10 s.) */
			uint64_t ticks = ((uint64_t)pwroff_us * 24576 / 1000 + 1023) >> 10;

			glb_in[GLB_PWROFF_TIMER / 4] = (uint32_t)(ticks > 0x7fffffff ? 0x7fffffff : ticks ? ticks : 1);
			glb_in[GLB_REQ / 4] ^= GLB_REQ_CFG_PWROFF_TIMER;
			to_gpu(glb_in, 64);
			ring_doorbell(0);
			if (!acked(&glb_in[GLB_REQ / 4], &glb_out[GLB_ACK / 4], GLB_REQ_CFG_PWROFF_TIMER, 100)) {
				fw_state("the power-off timer not acknowledged");
				return false;
			}
			printf("mali: 6: the shader cores left to the firmware: off after %u us idle\n", pwroff_us);
			return true;
		}
	}
	glb_in[GLB_PWROFF_TIMER / 4] = 0;
	glb_in[GLB_REQ / 4] ^= GLB_REQ_CFG_PWROFF_TIMER;
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	if (!acked(&glb_in[GLB_REQ / 4], &glb_out[GLB_ACK / 4], GLB_REQ_CFG_PWROFF_TIMER, 100)) {
		fw_state("the power-off timer not acknowledged");
		return false;
	}
	wr64(TILER_PWRON, rd64(TILER_PRESENT));        /* the tiler too: it stayed off (tiler ready 0) */
	wr64(SHADER_PWRON, rd64(SHADER_PRESENT));
	if (!wait_for(SHADER_READY, (uint32_t)rd64(SHADER_PRESENT), (uint32_t)rd64(SHADER_PRESENT), 100000)) {
		printf("mali: 6: shader cores not ready: 0x%llx, in transition 0x%llx\n", rd64(SHADER_READY), rd64(0x200));
		return false;
	}
	/* and the firmware told the stable mask again, now that the cores are on (Google's KBASE_MCU_HCTL_SHADERS_PEND_ON:
	 * "notify MCU the stable mask"): without it the group asked for endpoints (0x70707) and got none */
	glb_in[GLB_ALLOC_EN_LO / 4] = (uint32_t)rd64(SHADER_READY);
	glb_in[GLB_ALLOC_EN_HI / 4] = (uint32_t)(rd64(SHADER_READY) >> 32);
	glb_in[GLB_REQ / 4] ^= GLB_REQ_CFG_ALLOC_EN;
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	if (!acked(&glb_in[GLB_REQ / 4], &glb_out[GLB_ACK / 4], GLB_REQ_CFG_ALLOC_EN, 100)) {
		fw_state("the stable core mask not acknowledged");
		return false;
	}
	wait_for(TILER_READY, 1, 1, 100000);
	printf("mali: 6: shader cores powered: ready 0x%llx, told to the firmware; tiler ready 0x%llx\n", rd64(SHADER_READY),
	    rd64(TILER_READY));
	return true;
}

static bool
group_start(void)
{
	uint32_t *gc = glb_ctl + (GROUP_CONTROL_0 + 0 * glb_ctl[GLB_GROUP_STRIDE_W]) / 4;     /* group 0 */
	uint32_t *sc;
	uint32_t suspend_size, stream_stride, compute = 0;
	uint64_t shaders = rd64(SHADER_PRESENT);

	from_gpu((void *)shared_iface, 0x10000);       /* every control block, not only the first page's */
	csg_in = kernel_address(gc[GROUP_INPUT_VA / 4]);
	csg_out = kernel_address(gc[GROUP_OUTPUT_VA / 4]);
	suspend_size = gc[GROUP_SUSPEND_SIZE / 4];
	stream_stride = gc[GROUP_STREAM_STRIDE / 4];
	sc = gc + (STREAM_CONTROL_0 + 0 * stream_stride) / 4;                                /* stream 0 */
	cs_in = kernel_address(sc[STREAM_INPUT_VA / 4]);
	cs_out = kernel_address(sc[STREAM_OUTPUT_VA / 4]);
	printf("mali: 7: group 0: features 0x%x, %u streams every %u bytes, suspend buffer %u bytes; stream 0: features "
	    "0x%x\n", gc[GROUP_FEATURES / 4], gc[GROUP_STREAM_NUM / 4], stream_stride, suspend_size, sc[STREAM_FEATURES / 4]);
	if (csg_in == NULL || csg_out == NULL || cs_in == NULL || cs_out == NULL) {
		printf("mali: 7: a block outside our sections (group 0x%x/0x%x, stream 0x%x/0x%x)\n", gc[GROUP_INPUT_VA / 4],
		    gc[GROUP_OUTPUT_VA / 4], sc[STREAM_INPUT_VA / 4], sc[STREAM_OUTPUT_VA / 4]);
		return false;
	}
	if (suspend_size > SUSPEND_SIZE) {
		printf("mali: 7: the suspend buffer wants %u bytes, %u are mapped\n", suspend_size, SUSPEND_SIZE);
		return false;
	}
	/* address space 1: the same tables (the group's memory and the MCU's in one space, for now) */
	if (!wait_for(0x2440 + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000)) {
		return false;
	}
	wr64(0x2440 + AS_TRANSCFG, TRANSCFG_VALUE);
	wr64(0x2440 + AS_TRANSTAB, kvtophys((vm_offset_t)level0));
	wr64(0x2440 + AS_MEMATTR, MEMATTR_TABLE);
	wr(0x2440 + AS_COMMAND, AS_CMD_UPDATE);
	if (!wait_for(0x2440 + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000)) {
		faults("address space 1");
		return false;
	}
	/* the queue: ring, user pages, doorbell, started (Google's program_cs) */
	from_gpu(cs_in, 128);
	from_gpu(cs_out, 128);
	cs_in[CS_BASE_LO / 4] = (uint32_t)RING_VA;
	cs_in[CS_BASE_HI / 4] = (uint32_t)(RING_VA >> 32);
	cs_in[CS_SIZE / 4] = 0x10000;
	cs_in[CS_USER_INPUT_LO / 4] = (uint32_t)USER_IO_VA;
	cs_in[CS_USER_INPUT_HI / 4] = (uint32_t)(USER_IO_VA >> 32);
	cs_in[CS_USER_OUTPUT_LO / 4] = (uint32_t)(USER_IO_VA + GPU_PAGE);
	cs_in[CS_USER_OUTPUT_HI / 4] = (uint32_t)((USER_IO_VA + GPU_PAGE) >> 32);
	cs_in[CS_CONFIG / 4] = USER_DOORBELL << 8 | 1;          /* doorbell 1, priority 1 */
	cs_in[CS_ACK_IRQ_MASK / 4] = 0xffffffff;
	cs_in[CS_REQ / 4] = (cs_out[CS_ACK / 4] & 0xc0000000u) | CS_REQ_IDLE_BITS | CS_STATE_START;
	to_gpu(cs_in, 128);
	from_gpu(csg_out, 64);
	csg_in[CSG_DB_REQ / 4] = csg_out[CSG_DB_ACK / 4] ^ 1u;  /* stream 0's kernel doorbell */
	/* the group: endpoints, address space, suspend buffer, started (Google's program_csg_slot) */
	for (uint64_t m = shaders; m; m &= m - 1) {
		compute++;
	}
	csg_in[CSG_ALLOW_COMPUTE_LO / 4] = (uint32_t)shaders;
	csg_in[CSG_ALLOW_COMPUTE_HI / 4] = (uint32_t)(shaders >> 32);
	csg_in[CSG_ALLOW_FRAGMENT_LO / 4] = (uint32_t)shaders;
	csg_in[CSG_ALLOW_FRAGMENT_HI / 4] = (uint32_t)(shaders >> 32);
	csg_in[CSG_ALLOW_OTHER / 4] = (uint32_t)rd64(TILER_PRESENT);
	{
		unsigned int ep = 0;            /* mali_ep=1: compute endpoints only (all a compute job needs) */

		PE_parse_boot_argn("mali_ep", &ep, sizeof(ep));
		csg_in[CSG_EP_REQ_LO / 4] = ep == 1 ? compute | 8u << 28 : compute | compute << 8 | 1u << 16 | 8u << 28;
	}
	csg_in[CSG_EP_REQ_HI / 4] = 0;
	csg_in[CSG_CONFIG / 4] = 1;                             /* address space 1 */
	csg_in[CSG_SUSPEND_BUF_LO / 4] = (uint32_t)SUSPEND_VA;
	csg_in[CSG_SUSPEND_BUF_HI / 4] = (uint32_t)(SUSPEND_VA >> 32);
	csg_in[CSG_ACK_IRQ_MASK / 4] = 0xffffffff;
	csg_in[CSG_REQ / 4] = (csg_in[CSG_REQ / 4] & ~7u) | CSG_STATE_START;
	to_gpu(csg_in, 128);
	from_gpu(glb_out, 64);
	glb_in[GLB_DB_REQ / 4] = glb_out[GLB_DB_ACK / 4] ^ 1u;  /* group 0's doorbell */
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	if (!acked(&csg_in[CSG_REQ / 4], &csg_out[CSG_ACK / 4], 7, 200)) {
		fw_state("the group did not start");
		return false;
	}
	if (!acked(&cs_in[CS_REQ / 4], &cs_out[CS_ACK / 4], 7, 200)) {
		fw_state("the queue did not start");
		return false;
	}
	fw_state("group 0 and its queue started");
	return true;
}

static void
first_job(void)
{
	unsigned int n = 0;
	uint64_t *cmd = (uint64_t *)ring;

	target[0] = 0x11111111;
	target[1] = 0x22222222;
	to_gpu(target, 16);
	/* MOVE48 r0 <- the target; MOVE32 r4, r5 <- two words; STORE_MULTIPLE r4..r5 -> [r0 + 0] (Mesa's v10/v12 genxml) */
	{
		/*
		 * The stream holds its compute request from its very start (as Google's and panvk's streams do): a request
		 * made only after the group had gone idle once never took effect (the stream's requested resources stayed 0,
		 * no endpoints, the next job waiting at RUN_COMPUTE). mali_firstreq=0 leaves it out, to see that again.
		 */
		unsigned int req = 1;

		PE_parse_boot_argn("mali_firstreq", &req, sizeof(req));
		if (req) {
			cmd[n++] = 34ULL << 56 | 1;
		}
	}
	cmd[n++] = 1ULL << 56 | 0ULL << 48 | (TARGET_VA & 0xffffffffffffULL);
	cmd[n++] = 2ULL << 56 | 4ULL << 48 | 0xc0ffee11ULL;
	cmd[n++] = 2ULL << 56 | 5ULL << 48 | 0x5a5a1234ULL;
	cmd[n++] = 21ULL << 56 | 4ULL << 48 | 0ULL << 40 | 0x3ULL << 16 | 0;
	to_gpu(ring, n * 8);
	ring_used = n * 8;
	user_in[CS_INSERT / 4] = n * 8;
	user_in[CS_INSERT / 4 + 1] = 0;
	to_gpu(user_in, 16);
	printf("mali: 8: %u instructions in the ring; the queue's doorbell rung\n", n);
	ring_doorbell(USER_DOORBELL);
	for (int t = 0; t < 200; t++) {
		flush_gpu();
		from_gpu(user_out, 16);
		from_gpu(target, 16);
		if (user_out[CS_EXTRACT / 4] >= n * 8 && target[0] != 0x11111111) {
			break;
		}
		delay(1000);
	}
	printf("mali: 8: the queue read to 0x%x (of 0x%x), active %u; the target holds 0x%08x 0x%08x\n",
	    user_out[CS_EXTRACT / 4], n * 8, user_out[CS_ACTIVE / 4], target[0], target[1]);
	printf("mali: 8: %s\n", target[0] == 0xc0ffee11 && target[1] == 0x5a5a1234 ?
	    "THE GPU RAN OUR COMMANDS: both words are what the instructions stored" :
	    "the target was not written as asked");
	fw_state("after the job");
}

/*
 * Step 9: the first compute job -- a shader on the shader cores. Its four instructions, Valhall assembly put
 * together with Mesa's own assembler (src/panfrost/compiler/bifrost/valhall/asm.py, 26.2.5; only instructions from
 * its test cases, none of those v11 removed):
 *     MOV.i32 r2, u0.w0                           the target's address, from push constant 0
 *     MOV.i32 r3, u0.w1
 *     IADD_IMM.i32 r0, 0x0, #0x600DF00D           the value
 *     STORE.i32.slot0.end @r0, [r2^:r3^], offset:0
 * The descriptors as Mesa's v10 genxml has them (v12 only adds fields, left zero here): the shader program
 * (type 8, stage 1 compute, the binary's address), the local storage (no thread or workgroup memory); the command
 * stream's compute resource requested (REQ_RESOURCE), its compute registers (COMPUTE SR: FAU r8, SPD r16, TSD r24,
 * WG_SIZE r33, JOB_SIZE r37-39), RUN_COMPUTE,
 * a WAIT on every scoreboard, and a marker stored after it: the marker says the compute work is over.
 */
static const uint8_t store_shader[32] = {
	0x80, 0x00, 0x00, 0x00, 0x00, 0xc2, 0x91, 0x00, 0x81, 0x00, 0x00, 0x00, 0x00, 0xc3, 0x91, 0x00,
	0xc0, 0x0d, 0xf0, 0x0d, 0x60, 0xc0, 0x10, 0x01, 0x42, 0x00, 0x00, 0x18, 0x02, 0x40, 0x61, 0x78,
};

static void
compute_job(void)
{
	uint64_t *cmd = (uint64_t *)((uint8_t *)ring + ring_used);
	unsigned int n = 0;
	const uint64_t spd = DESC_VA, tsd = DESC_VA + 0x40, fau = DESC_VA + 0x80;

	memcpy(code, store_shader, sizeof(store_shader));
	to_gpu(code, 128);
	memset(desc, 0, 0x100);
	desc[0] = 8 | 1 << 4;                   /* shader program: type shader, stage compute, 64 registers a thread */
	desc[2] = (uint32_t)CODE_VA;
	desc[3] = (uint32_t)(CODE_VA >> 32);
	desc[0x40 / 4 + 1] = 31;                /* local storage: WLS instances "no workgroup memory" (log2 of 1 << 31) */
	*(uint64_t *)&desc[0x80 / 4] = TARGET_VA + 0x10;        /* push constant 0: where the shader stores */
	to_gpu(desc, 0x100);
	target[4] = 0x33333333;                 /* the shader's word */
	target[8] = 0x44444444;                 /* the marker after the wait */
	to_gpu(target, 64);

	cmd[n++] = 34ULL << 56 | 1;                                     /* REQ_RESOURCE compute (without it the stream
	                                                                 * stopped at RUN_COMPUTE, waiting for nothing said) */
	/*
	 * v11's scoreboard state, as panvk sets it up for a queue (panvk_vX_gpu_queue.c, PAN_ARCH >= 11; Google's own
	 * streams on Android start the same way): SET_STATE_IMM32 (opcode 28, the state in bits 39:32) -- the entry that
	 * tracks endpoint tasks (3), the entries WAIT waits on (0x8), the load/store one (0), the deferred one (1), and the
	 * entries the stream may use (0xf0). Without it RUN_COMPUTE waited for ever, saying nothing (v10's way,
	 * SET_SB_ENTRY, is not what this GPU has).
	 */
	cmd[n++] = 28ULL << 56 | 0ULL << 32 | 3;                        /* SB_SEL_ENDPOINT 3 */
	cmd[n++] = 28ULL << 56 | 9ULL << 32 | 0x8;                      /* SB_MASK_WAIT 0x8 */
	cmd[n++] = 28ULL << 56 | 1ULL << 32 | 0;                        /* SB_SEL_OTHER 0 */
	cmd[n++] = 28ULL << 56 | 2ULL << 32 | 1;                        /* SB_SEL_DEFERRED 1 */
	cmd[n++] = 28ULL << 56 | 8ULL << 32 | 0xf0;                     /* SB_MASK_STREAM 0xf0 */
	cmd[n++] = 2ULL << 56 | 62ULL << 48 | 0;                        /* iteration trace ids, as Google's streams */
	cmd[n++] = 2ULL << 56 | 63ULL << 48 | 0;
	cmd[n++] = 1ULL << 56 | 16ULL << 48 | spd;                      /* MOVE48 r16:r17 <- the shader program */
	cmd[n++] = 1ULL << 56 | 24ULL << 48 | tsd;                      /* MOVE48 r24:r25 <- the local storage */
	cmd[n++] = 1ULL << 56 | 8ULL << 48 | fau;                       /* MOVE48 r8:r9 <- the push constants ... */
	cmd[n++] = 2ULL << 56 | 9ULL << 48 | (uint32_t)(fau >> 32) | 1u << 24;   /* ... one of them (count << 56) */
	cmd[n++] = 1ULL << 56 | 0ULL << 48 | 0;                         /* MOVE48 r0:r1 <- no resource table */
	cmd[n++] = 2ULL << 56 | 32ULL << 48 | 0;                        /* global attribute offset */
	cmd[n++] = 2ULL << 56 | 33ULL << 48 | 0;                        /* workgroup size 1x1x1 (each minus 1) */
	for (int r = 34; r <= 36; r++) {
		cmd[n++] = 2ULL << 56 | (uint64_t)r << 48 | 0;          /* job offset */
	}
	for (int r = 37; r <= 39; r++) {
		cmd[n++] = 2ULL << 56 | (uint64_t)r << 48 | 1;          /* job size: one workgroup */
	}
	cmd[n++] = 4ULL << 56 | 1;                                      /* RUN_COMPUTE: a workgroup a task, along X */
	{
		unsigned int nowait = 0;

		PE_parse_boot_argn("mali_nowait", &nowait, sizeof(nowait));
		if (!nowait) {
			cmd[n++] = 3ULL << 56 | 0xffffULL << 16;        /* WAIT on every scoreboard */
		}
	}
	cmd[n++] = 1ULL << 56 | 0ULL << 48 | (TARGET_VA + 0x20);        /* the marker: MOVE48 r0, MOVE32 r4, STORE */
	cmd[n++] = 2ULL << 56 | 4ULL << 48 | 0x9a55ed00u;
	cmd[n++] = 21ULL << 56 | 4ULL << 48 | 0ULL << 40 | 0x1ULL << 16 | 0;
	to_gpu(cmd, n * 8);
	ring_used += n * 8;
	user_in[CS_INSERT / 4] = ring_used;
	to_gpu(user_in, 16);
	printf("mali: 9: %u instructions; the shader at 0x%llx, its descriptors at 0x%llx; the doorbell rung\n", n,
	    CODE_VA, DESC_VA);
	/* the GPU's L2 still holds what the firmware read for the first job (the old insert word among it): flushed, it
	 * reads what we wrote. (Without this the queue stayed at the first job's end, 0x20.) */
	flush_gpu();
	/* the first job left group 0 idle, and the firmware said so (CSG_ACK bit 29 toggled, JOB_IRQ bit 0): Google's
	 * driver acknowledges that (CSG_REQ's bit set to the ACK's, the CS interrupts acked, JOB_IRQ cleared) before it
	 * kicks a queue again -- through the group's kernel doorbell bit for the stream and doorbell 0 -- and so does this */
	from_gpu(csg_out, 64);
	csg_in[CSG_REQ / 4] = (csg_in[CSG_REQ / 4] & ~CSG_REQ_IDLE) | (csg_out[CSG_ACK / 4] & CSG_REQ_IDLE);
	/* and its endpoints asked for again (CSG_REQ EP_CFG toggled): an idle group's are taken back, and were not given
	 * again for the next job without it */
	csg_in[CSG_REQ / 4] = (csg_in[CSG_REQ / 4] & ~CSG_REQ_EP_CFG) | ((csg_out[CSG_ACK / 4] ^ CSG_REQ_EP_CFG) & CSG_REQ_EP_CFG);
	csg_in[CSG_IRQ_ACK / 4] = csg_out[CSG_IRQ_REQ / 4];
	csg_in[CSG_DB_REQ / 4] = csg_out[CSG_DB_ACK / 4] ^ 1u;
	to_gpu(csg_in, 64);
	wr(JOB_IRQ_CLEAR, 0xffffffff);
	from_gpu(glb_out, 64);
	glb_in[GLB_DB_REQ / 4] = glb_out[GLB_DB_ACK / 4] ^ 1u;
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	ring_doorbell(USER_DOORBELL);
	for (int t = 0; t < 500; t++) {
		flush_gpu();
		from_gpu(user_out, 16);
		from_gpu(target, 64);
		if (target[8] != 0x44444444) {
			break;
		}
		delay(1000);
	}
	printf("mali: 9: the queue read to 0x%x (of 0x%x); the shader's word 0x%08x, the marker 0x%08x\n",
	    user_out[CS_EXTRACT / 4], ring_used, target[4], target[8]);
	printf("mali: 9: %s\n", target[4] == 0x600df00d ? "A SHADER RAN ON THE SHADER CORES: it stored 0x600df00d" :
	    target[8] == 0x9a55ed00 ? "the job was passed, but the shader's word was not written" :
	    "the command stream did not get past the compute job");
	/* the status words are filled in on request: CSG_REQ's STATUS_UPDATE toggled, the group's doorbell */
	from_gpu(csg_out, 64);
	csg_in[CSG_REQ / 4] = (csg_in[CSG_REQ / 4] & ~(1u << 5)) | ((csg_out[CSG_ACK / 4] ^ (1u << 5)) & (1u << 5));
	to_gpu(csg_in, 64);
	from_gpu(glb_out, 64);
	glb_in[GLB_DB_REQ / 4] = glb_out[GLB_DB_ACK / 4] ^ 1u;
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	printf("mali: 9: status update %s\n", acked(&csg_in[CSG_REQ / 4], &csg_out[CSG_ACK / 4], 1u << 5, 100) ?
	    "acknowledged" : "NOT acknowledged");
	flush_gpu();
	from_gpu(csg_out, 64);
	printf("mali: 9: group 0: endpoints current 0x%x, requested 0x%x, progress timer state 0x%x; EP_REQ asked 0x%x; "
	    "stream blocked reason 0x%x, cmd at 0x%x%08x\n", csg_out[0x10 / 4], csg_out[0x14 / 4], csg_out[0x1c / 4],
	    csg_in[CSG_EP_REQ_LO / 4], cs_out[0x60 / 4], cs_out[0x44 / 4], cs_out[0x40 / 4]);
	from_gpu(cs_out, 256);
	printf("mali: 9: shaders ready 0x%llx (in transition 0x%llx), tiler ready 0x%llx; the stream: requested resources "
	    "0x%x, scoreboards 0x%x, wait sync value 0x%x\n", rd64(SHADER_READY), rd64(0x200), rd64(TILER_READY),
	    cs_out[0x4c / 4], cs_out[0x5c / 4], cs_out[0x58 / 4]);
	fw_state("after the compute job");
}

/*
 * ---- The GPU kept on (mali_keep=1): what the service HakiosGPU (hakios.akita_gpu.cpp) gives programs ----
 *
 * Steps 1 to 6 bring the GPU up and the firmware's global interface; then the GPU stays on, and HakiosGPU speaks
 * Linux's panthor interface (the one Mesa's panfrost and panvk know) over these functions. Every one of them is called
 * with the service's lock held: nothing here locks.
 *
 *   address spaces  one per program VM, slots 1..7 of the GPU's MMU (0 is the MCU's), each with its own tables; the
 *                   upper half of its 48 bits is the kernel's (Mesa's split): rings and sequence pages go there
 *   groups          command stream groups 0..7, each with queues (its streams); a queue's ring and its user pages
 *                   are the kernel's, its doorbell is 1 + the group's slot
 *   submissions     into a queue's ring: CALL the program's stream, WAIT every scoreboard, then store the queue's
 *                   new sequence number into its sequence page (r92..r95, the registers Mesa leaves the kernel); done
 *                   is that number having reached the page
 *
 * The stream holds REQ_RESOURCE compute from the start of its life (a request made only after the group had been
 * idle never took effect, step 9's lesson); the scoreboards each program sets up itself (panvk does).
 */
#define KVM_BASE                0x800000000000ULL       /* the kernel's half of a program's address space */
/*
 * MORE CLIENTS THAN THE GPU HAS PLACES (11-12, Metal for every process): the Mali-G715 has 8 address spaces (0 the
 * MCU's) and its firmware 8 group slots; a VM and a group each took one for good, so the eighth process to open the
 * device got none ("no VM": Maps, Safari, WebKit's GPU process ran without, the watch faces' snapshot service died of
 * it). Now a VM and a group are the driver's own (MAX_VMS, MAX_GROUPS), and take a place only while they work: a group
 * is given a slot at its submission (group_bind) -- a free one, or that of the group idle longest (everything it was
 * given done: ended with CSG TERMINATE as a restart ends one, and started again later with empty rings, group_restart's
 * way, which loses nothing of an idle group) -- and its VM an address space the same way (vm_bind: one whose VM has no
 * group in a slot is turned off and handed over). panthor does this with its scheduler's ticks; here it is done at
 * the submission, under HakiosGPU's lock.
 */
#define MAX_VMS                 48
#define MAX_GROUPS              48
#define MAX_QUEUES              4
#define CSG_SLOTS               8
#define AS_SLOTS                8

struct mali_queue {
	vm_offset_t ring;               /* the kernel's address */
	uint64_t ring_va;               /* the group's address (the kernel's half of its VM) */
	uint32_t ring_size;
	uint64_t insert;                /* bytes written, ever */
	volatile uint32_t *user_in, *user_out;
	volatile uint64_t *seqno;       /* the sequence page: the last number the queue reached */
	uint64_t seqno_va, submitted;
	uint64_t seen;                  /* the last sequence number a completion check saw */
	/* submissions a restart counted done without the GPU having done them (group_restart): the last LOST_RANGES
	 * runs, each (after, upto] -- akita_mali_lost says so to whoever waited for one */
#define LOST_RANGES             4
	struct { uint64_t after, upto; } lost[LOST_RANGES];
	unsigned int lost_next;
};

static struct mali_vm {
	bool used;
	uint64_t *root;
	uint64_t kernel_next;           /* the next free address of the kernel's half */
	bool flush_pending;             /* pages mapped since its last update: flushed before its next submission */
	unsigned int as;                /* the address space it has now (1..7), 0: none */
	uint64_t last_used;             /* mach absolute time of its groups' last submission */
	uint64_t flushed;               /* a number that changes whenever the GPU is given its tables anew (11-13) */
} vms[MAX_VMS];
static uint64_t flush_stamp;

static struct mali_group {
	bool used;
	unsigned int vm, queues;
	struct mali_queue q[MAX_QUEUES];
	uint8_t priority;
	unsigned int restarts;
	int slot;                       /* the firmware's group slot it has now, -1: none */
	uint64_t last_used;             /* mach absolute time of its last submission */
} groups[MAX_GROUPS];

/* the places: a slot's group (its number + 1, 0: free), an address space's VM (0: free; address space 0 is the MCU's) */
static unsigned int slot_group[CSG_SLOTS], as_vm[AS_SLOTS];
/* what the MCU reaches of a slot, made once and used by whichever group has the slot: the suspend buffer and each
 * queue's two user pages (the MCU's zone has room for exactly eight slots' worth and gives nothing back: made a group,
 * as before, the tenth group ever made found it empty) */
static struct {
	bool made;
	uint32_t suspend_va, user_va[MAX_QUEUES];
	vm_offset_t user[MAX_QUEUES];
} slots[CSG_SLOTS];

static uint32_t mcu_next = MCU_ZONE_BASE;
static bool kept;

/*
 * How busy the GPU is, for the clocks' governor (hakios.akita_dvfs.c): the time any queue had work submitted and not
 * yet seen done -- from the submission that found every queue idle to the completion check that found them all caught
 * up (waiters check often: HakiosGPU's SYNCOBJ_WAIT, hakios-metal's completion thread). The shader cores' power is no
 * measure here: the host keeps them on (mali_hctl=1).
 */
static uint64_t busy_since, busy_total;         /* mach absolute time */

static void
busy_update(void)
{
	bool busy = false;
	uint64_t now = mach_absolute_time();

	for (unsigned int g = 0; g < MAX_GROUPS && !busy; g++) {
		if (!groups[g].used) {
			continue;
		}
		for (unsigned int q = 0; q < groups[g].queues; q++) {
			if (groups[g].q[q].submitted > groups[g].q[q].seen) {
				busy = true;
				break;
			}
		}
	}
	if (busy && busy_since == 0) {
		busy_since = now;
	} else if (!busy && busy_since != 0) {
		busy_total += now - busy_since;
		busy_since = 0;
	}
}

/* the GPU's busy time so far, in mach absolute time units */
uint64_t akita_mali_busy_abs(void);
uint64_t
akita_mali_busy_abs(void)
{
	uint64_t since = busy_since;

	/* (boot-arg mali_pwr_say: how often the shader cores were found off -- the firmware's power-off timer at work --
	 * said every 300 looks, ~30 s at the governor's 100 ms) */
	static int pwr_say = -1;
	static unsigned looks, off;
	if (pwr_say < 0) {
		int v = 0;
		PE_parse_boot_argn("mali_pwr_say", &v, sizeof(v));
		pwr_say = v;
	}
	if (pwr_say && kept) {
		looks++;
		off += rd(SHADER_READY) == 0;
		if (looks == 300) {
			printf("mali: shader cores off in %u of %u looks (L2 ready 0x%x, tiler ready 0x%x now)\n", off, looks,
			    rd(L2_READY), rd(TILER_READY));
			looks = off = 0;
		}
	}

	return busy_total + (since ? mach_absolute_time() - since : 0);
}

bool akita_mali_ready(void);
bool
akita_mali_ready(void)
{
	return kept && rd(MCU_STATUS) == MCU_ENABLED;
}

/* a page into a table tree, every entry written cleaned to memory for the GPU's walk */
static bool
map_page_in(uint64_t *root, uint64_t va, uint64_t phys, uint64_t attributes)
{
	uint64_t *table = root;

	for (int level = 0; level < 3; level++) {
		uint64_t *entry = &table[(va >> (39 - 9 * level)) & 511];

		if ((*entry & 3) == 0) {
			uint64_t *next = new_table();

			if (next == NULL) {
				return false;
			}
			clean_dcache((vm_offset_t)next, GPU_PAGE, FALSE);
			*entry = kvtophys((vm_offset_t)next) | PTE_VALID_TABLE;
			clean_dcache((vm_offset_t)entry & ~63UL, 64, FALSE);
		}
		table = (uint64_t *)phystokv(*entry & PTE_ADDR);
	}
	table[(va >> 12) & 511] = (phys & PTE_ADDR) | attributes | PTE_SH_OS | PTE_AF | PTE_VALID_TABLE;
	clean_dcache((vm_offset_t)&table[(va >> 12) & 511] & ~63UL, 64, FALSE);
	return true;
}

static void
unmap_page_in(uint64_t *root, uint64_t va)
{
	uint64_t *table = root;

	for (int level = 0; level < 3; level++) {
		uint64_t entry = table[(va >> (39 - 9 * level)) & 511];

		if ((entry & 3) == 0) {
			return;
		}
		table = (uint64_t *)phystokv(entry & PTE_ADDR);
	}
	table[(va >> 12) & 511] = 0;
	clean_dcache((vm_offset_t)&table[(va >> 12) & 511] & ~63UL, 64, FALSE);
}

/* an address space's registers pointed at tables: the TLB and the walk's caches see what was written */
static void
as_program(unsigned int as, uint64_t *root)
{
	uint32_t base = AS0 + as * 0x40;

	flush_gpu();
	wait_for(base + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000);
	wr64(base + AS_TRANSCFG, TRANSCFG_VALUE);
	wr64(base + AS_TRANSTAB, kvtophys((vm_offset_t)root));
	wr64(base + AS_MEMATTR, MEMATTR_TABLE);
	wr(base + AS_COMMAND, AS_CMD_UPDATE);
	wait_for(base + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000);
	/* the MMU's cached translations dropped (kbase's mmu_hw_do_flush on GPUs with cache flush commands: LOCK the
	 * region, the GPU's caches flushed, UNLOCK -- which forgets the region's walks): the whole 48-bit space. UPDATE
	 * alone did not: a second client's grown heap chunks (fresh level-3 tables in its VM) faulted at level 2 one
	 * after the other, 120 group restarts */
	wr64(base + AS_LOCKADDR, 47);
	wr(base + AS_COMMAND, AS_CMD_LOCK);
	wait_for(base + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000);
	flush_gpu();
	wr(base + AS_COMMAND, AS_CMD_UNLOCK);
	wait_for(base + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000);
}

/* an address space turned off: what uses it fails (a page fault's answer; a VM losing its place) */
#define TRANSCFG_UNMAPPED       1ULL

static void
as_off(unsigned int as)
{
	uint32_t base = AS0 + as * 0x40;

	flush_gpu();
	wait_for(base + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000);
	wr64(base + AS_TRANSCFG, TRANSCFG_UNMAPPED);
	wr(base + AS_COMMAND, AS_CMD_UPDATE);
	wait_for(base + AS_STATUS, AS_STATUS_ACTIVE, 0, 100000);
}

/* a VM's tables as they are now, for the GPU -- if the VM has an address space (one without gets it at its group's
 * next submission, programmed then); 0: the MCU's own */
static void
as_update(unsigned int vm)
{
	if (vm == 0) {
		as_program(0, level0);
	} else if (vm < MAX_VMS && vms[vm].as != 0) {
		as_program(vms[vm].as, vms[vm].root);
		vms[vm].flushed = ++flush_stamp;
	}
}

/* an address space for the VM: a free one, or the one of the VM used longest ago that has no group in a slot */
static bool
vm_bind(unsigned int vm)
{
	unsigned int as = 0;

	if (vms[vm].as != 0) {
		return true;
	}
	for (unsigned int a = 1; a < AS_SLOTS && as == 0; a++) {
		if (as_vm[a] == 0) {
			as = a;
		}
	}
	if (as == 0) {
		uint64_t oldest = ~0ULL;

		for (unsigned int a = 1; a < AS_SLOTS; a++) {
			unsigned int v = as_vm[a];
			bool working = false;

			for (unsigned int g = 0; g < MAX_GROUPS && !working; g++) {
				working = groups[g].used && groups[g].vm == v && groups[g].slot >= 0;
			}
			if (!working && vms[v].last_used < oldest) {
				oldest = vms[v].last_used;
				as = a;
			}
		}
		if (as == 0) {
			return false;
		}
		as_off(as);
		vms[as_vm[as]].as = 0;
	}
	as_vm[as] = vm;
	vms[vm].as = as;
	vms[vm].flush_pending = false;
	as_program(as, vms[vm].root);
	vms[vm].flushed = ++flush_stamp;
	return true;
}

/* a VM's tables given back (not what they map) */
static void
free_tables(uint64_t *table, int level)
{
	if (level < 3) {
		for (unsigned int i = 0; i < 512; i++) {
			if ((table[i] & 3) != 0) {
				free_tables((uint64_t *)phystokv(table[i] & PTE_ADDR), level + 1);
			}
		}
	}
	table[0] = (uint64_t)(uintptr_t)table_free;
	table_free = table;
}

int akita_mali_vm_new(void);
int
akita_mali_vm_new(void)
{
	for (unsigned int i = 1; i < MAX_VMS; i++) {
		if (!vms[i].used) {
			vms[i].root = new_table();
			if (vms[i].root == NULL) {
				return -1;
			}
			clean_dcache((vm_offset_t)vms[i].root, GPU_PAGE, FALSE);
			vms[i].used = true;
			vms[i].kernel_next = KVM_BASE;
			vms[i].as = 0;                  /* (its address space at its group's first submission: vm_bind) */
			vms[i].flush_pending = false;
			vms[i].flushed = ++flush_stamp;
			vms[i].last_used = mach_absolute_time();
			return (int)i;
		}
	}
	return -1;
}

void akita_mali_vm_free(int vm);
void
akita_mali_vm_free(int vm)
{
	if (vm > 0 && vm < MAX_VMS && vms[vm].used) {
		if (vms[vm].as != 0) {
			as_off(vms[vm].as);
			as_vm[vms[vm].as] = 0;
			vms[vm].as = 0;
		}
		vms[vm].used = false;
		free_tables(vms[vm].root, 0);   /* (11-12: they stayed for good, a tree a client that ever was) */
		vms[vm].root = NULL;
	}
}

/* flags: 1 read only, 2 not executable, 4 uncached (panthor's DRM_PANTHOR_VM_BIND_OP_MAP_*) */
bool akita_mali_vm_map(int vm, uint64_t va, uint64_t phys, uint32_t flags);
bool
akita_mali_vm_map(int vm, uint64_t va, uint64_t phys, uint32_t flags)
{
	uint64_t attributes = (flags & 1) ? PTE_RO : PTE_RW;

	if (vm <= 0 || vm >= MAX_VMS || !vms[vm].used || (va & (GPU_PAGE - 1)) || va >= KVM_BASE) {
		return false;
	}
	if (flags & 2) {
		attributes |= PTE_NX;
	}
	attributes |= (uint64_t)((flags & 4) ? ATTR_NON_CACHEABLE : ATTR_CACHED) << 2;
	return map_page_in(vms[vm].root, va, phys, attributes);
}

void akita_mali_vm_unmap(int vm, uint64_t va, uint64_t size);
void
akita_mali_vm_unmap(int vm, uint64_t va, uint64_t size)
{
	if (vm <= 0 || vm >= MAX_VMS || !vms[vm].used) {
		return;
	}
	for (uint64_t off = 0; off < size; off += GPU_PAGE) {
		unmap_page_in(vms[vm].root, va + off);
	}
}

void akita_mali_vm_flush(int vm);
void
akita_mali_vm_flush(int vm)
{
	if (vm > 0 && vm < MAX_VMS && vms[vm].used) {
		as_update((unsigned int)vm);
		vms[vm].flush_pending = false;
	}
}

/*
 * ... or before the VM's next submission (11-10): pages newly MAPPED are only reached by work submitted after them, so
 * the update -- two whole-GPU cache flushes, the address space reprogrammed and its whole range locked meanwhile -- can
 * wait for that submission, and a burst of binds costs one. HakiosMetal binds each buffer apart: CoreAnimation's new
 * surfaces (an App Library page, a folder opening) came in bursts of ~30 binds in 60 ms, 60 flushes under the GPU's
 * running work, and the panel's frame stood still 25-46 ms each time (the tester's slow-motion clip, sb_debug's counter
 * against the log). An unmap is still flushed at once (its memory may be freed right after).
 */
/*
 * UNMAPS WAIT TOO (11-13), their memory kept by HakiosGPU until the update: the update at once cost ~10 ms each under
 * a busy GPU, and WebKit's WebGL (ANGLE) gives a buffer back for every draw -- the aquarium's 611 draws a frame took 12
 * s. akita_mali_vm_stale: the GPU may still hold translations older than STAMP for this VM (it has an address space,
 * binds wait, and no update came since); *STAMP_NOW for the asker to keep.
 */
int akita_mali_vm_unflushed(int vm, uint64_t *stamp_now);
int
akita_mali_vm_unflushed(int vm, uint64_t *stamp_now)
{
	if (vm <= 0 || vm >= MAX_VMS || !vms[vm].used) {
		return 0;
	}
	*stamp_now = vms[vm].flushed;
	return vms[vm].as != 0 && vms[vm].flush_pending;
}

int akita_mali_vm_stale(int vm, uint64_t stamp);
int
akita_mali_vm_stale(int vm, uint64_t stamp)
{
	return vm > 0 && vm < MAX_VMS && vms[vm].used && vms[vm].as != 0 && vms[vm].flushed == stamp;
}

void akita_mali_vm_flush_later(int vm);
void
akita_mali_vm_flush_later(int vm)
{
	if (vm > 0 && vm < MAX_VMS && vms[vm].used) {
		vms[vm].flush_pending = true;
	}
}

/* memory of the kernel's in a VM's kernel half (rings, sequence pages); its kernel address. UNCACHED: the GPU's view
 * of it uncached too (tiler heaps: what the shader cores write there, the tiler read stale through the cached view) */
static vm_offset_t
vm_kernel_region_attr(unsigned int vm, vm_size_t size, uint64_t *va, bool exec, bool uncached)
{
	vm_offset_t memory = gpu_memory(size);

	if (memory == 0) {
		return 0;
	}
	clean_dcache(memory, (unsigned)round_page(size), FALSE);
	*va = vms[vm].kernel_next;
	for (vm_size_t off = 0; off < size; off += GPU_PAGE) {
		if (!map_page_in(vms[vm].root, *va + off, kvtophys(memory + off), (exec ? PTE_RO : PTE_RW | PTE_NX) |
		    (uint64_t)(uncached ? ATTR_NON_CACHEABLE : ATTR_CACHED) << 2)) {
			return 0;
		}
	}
	vms[vm].kernel_next += round_page(size) + PAGE_SIZE;   /* a page between them */
	return memory;
}

static vm_offset_t
vm_kernel_region(unsigned int vm, vm_size_t size, uint64_t *va, bool exec)
{
	return vm_kernel_region_attr(vm, size, va, exec, false);
}

/* a kernel region taken back: out of the VM's tables, then freed */
static void
vm_kernel_free(unsigned int vm, uint64_t va, vm_offset_t memory, vm_size_t size)
{
	if (memory == 0) {
		return;
	}
	if (vms[vm].used) {
		for (vm_size_t off = 0; off < size; off += GPU_PAGE) {
			unmap_page_in(vms[vm].root, va + off);
		}
		as_update(vm);
	}
	kmem_free(kernel_map, memory, round_page(size));
	os_atomic_sub(&gpu_memory_bytes, (uint64_t)round_page(size), relaxed);
}

/*
 * Tiler heaps (panthor's TILER_HEAP_CREATE): what the tiler writes a render pass's lists into. A heap is a context,
 * zeroed, that the firmware keeps its state in (32 bytes; a page here), and chunks in a list: each chunk's first
 * 64 bytes its header, whose first word is the next chunk's address | its size >> 12 (0: the last), the newest
 * chunk first -- as Linux's panthor_heap.c makes them. All of a heap is in the VM's kernel half. When the chunks are
 * used up the firmware raises the queue's tiler out-of-memory event and waits: tiler_oom() gives it one chunk more
 * (heap_grow, panthor_heap_grow's), up to the heap's max_chunks.
 */
#define MAX_HEAPS               48      /* (11-13, was 16: a heap a client, and there are up to MAX_VMS of those) */
#define MAX_HEAP_CHUNKS         64
#define HEAP_INITIAL_BYTES      (64u << 20)     /* the most of a heap's initial chunks together */

static struct {
	bool used, told;                        /* (told: "at its most" said once) */
	unsigned int vm, chunks, max_chunks, at_most;
	uint32_t chunk_size;
	uint64_t ctx_va, chunk_va[MAX_HEAP_CHUNKS];
	vm_offset_t ctx, chunk[MAX_HEAP_CHUNKS];
} heaps[MAX_HEAPS];

void akita_mali_heap_free(int h);
void
akita_mali_heap_free(int h)
{
	if (h < 0 || h >= MAX_HEAPS || !heaps[h].used) {
		return;
	}
	if (heaps[h].at_most != 0) {
		printf("mali: tiler heap %d freed: %u chunks; asked %u time(s) more at its most\n", h, heaps[h].chunks,
		    heaps[h].at_most);
	}
	for (unsigned int i = 0; i < heaps[h].chunks; i++) {
		vm_kernel_free(heaps[h].vm, heaps[h].chunk_va[i], heaps[h].chunk[i], heaps[h].chunk_size);
	}
	vm_kernel_free(heaps[h].vm, heaps[h].ctx_va, heaps[h].ctx, GPU_PAGE);
	bzero(&heaps[h], sizeof(heaps[h]));
}

/* the heap's slot, or -1: bad arguments (as Linux checks them) or no memory */
int akita_mali_heap_new(int vm, uint32_t chunk_size, uint32_t initial, uint32_t max_chunks, uint32_t in_flight,
    uint64_t *ctx_va, uint64_t *first_chunk_va);
int
akita_mali_heap_new(int vm, uint32_t chunk_size, uint32_t initial, uint32_t max_chunks, uint32_t in_flight,
    uint64_t *ctx_va, uint64_t *first_chunk_va)
{
	int h;

	if (vm <= 0 || vm >= MAX_VMS || !vms[vm].used || chunk_size < (128u << 10) || chunk_size > (8u << 20) ||
	    (chunk_size & (chunk_size - 1)) != 0 || initial == 0 || initial > max_chunks || in_flight == 0 ||
	    in_flight > 65535 || initial > MAX_HEAP_CHUNKS || (uint64_t)initial * chunk_size > HEAP_INITIAL_BYTES) {
		return -1;
	}
	for (h = 0; h < MAX_HEAPS && heaps[h].used; h++) {
	}
	if (h == MAX_HEAPS) {
		return -1;
	}
	heaps[h].used = true;
	heaps[h].vm = (unsigned int)vm;
	heaps[h].chunk_size = chunk_size;
	heaps[h].max_chunks = max_chunks < MAX_HEAP_CHUNKS ? max_chunks : MAX_HEAP_CHUNKS;
	heaps[h].ctx = vm_kernel_region((unsigned int)vm, GPU_PAGE, &heaps[h].ctx_va, false);
	if (heaps[h].ctx == 0) {
		akita_mali_heap_free(h);
		return -1;
	}
	for (unsigned int i = 0; i < initial; i++) {
		uint64_t *header;

		/* each chunk at an address that is a multiple of its size, as panthor's allocator gives large ones */
		vms[vm].kernel_next = (vms[vm].kernel_next + chunk_size - 1) & ~(uint64_t)(chunk_size - 1);
		heaps[h].chunk[i] = vm_kernel_region((unsigned int)vm, chunk_size, &heaps[h].chunk_va[i], false);
		if (heaps[h].chunk[i] == 0) {
			akita_mali_heap_free(h);
			return -1;
		}
		heaps[h].chunks = i + 1;
		header = (uint64_t *)heaps[h].chunk[i];
		header[0] = i == 0 ? 0 : (heaps[h].chunk_va[i - 1] & ~0xfffULL) | (chunk_size >> 12);
		clean_dcache((vm_offset_t)header, 64, FALSE);
	}
	as_update((unsigned int)vm);
	*ctx_va = heaps[h].ctx_va;
	*first_chunk_va = heaps[h].chunk_va[initial - 1];
	printf("mali: tiler heap %d in VM %d: context at %#llx, %u chunk(s) of %u KiB, first at %#llx\n", h, vm,
	    *ctx_va, initial, chunk_size >> 10, *first_chunk_va);
	return h;
}

/* memory the MCU reaches (in its 28 bits): queues' user pages, suspend buffers */
static vm_offset_t
mcu_region(vm_size_t size, uint32_t *va)
{
	vm_offset_t memory;

	size = round_page(size);
	if (mcu_pool == 0 || mcu_next + size > MCU_ZONE_BASE + MCU_POOL_SIZE) {
		return 0;
	}
	memory = mcu_pool + (mcu_next - MCU_ZONE_BASE);
	bzero((void *)memory, size);
	clean_dcache(memory, (unsigned)size, FALSE);
	*va = mcu_next;
	mcu_next += size;
	return memory;
}

static void
group_blocks(unsigned int g, volatile uint32_t **in, volatile uint32_t **out, uint32_t **control)
{
	uint32_t *gc = glb_ctl + (GROUP_CONTROL_0 + g * glb_ctl[GLB_GROUP_STRIDE_W]) / 4;

	*control = gc;
	*in = kernel_address(gc[GROUP_INPUT_VA / 4]);
	*out = kernel_address(gc[GROUP_OUTPUT_VA / 4]);
}

static void
stream_blocks(uint32_t *gc, unsigned int q, volatile uint32_t **in, volatile uint32_t **out)
{
	uint32_t *sc = gc + (STREAM_CONTROL_0 + q * gc[GROUP_STREAM_STRIDE / 4]) / 4;

	*in = kernel_address(sc[STREAM_INPUT_VA / 4]);
	*out = kernel_address(sc[STREAM_OUTPUT_VA / 4]);
}

/* a slot's own memory in the MCU's zone, made the first time the slot is used */
static bool
slot_make(unsigned int s)
{
	volatile uint32_t *gin, *gout;
	uint32_t *gc;

	if (slots[s].made) {
		return true;
	}
	group_blocks(s, &gin, &gout, &gc);
	if (gin == NULL || gout == NULL || gc[GROUP_SUSPEND_SIZE / 4] > 0x40000 ||
	    mcu_region(gc[GROUP_SUSPEND_SIZE / 4], &slots[s].suspend_va) == 0) {
		return false;
	}
	for (unsigned int q = 0; q < MAX_QUEUES; q++) {
		slots[s].user[q] = mcu_region(2 * GPU_PAGE, &slots[s].user_va[q]);
		if (slots[s].user[q] == 0) {
			return false;
		}
	}
	slots[s].made = true;
	return true;
}

/* the slot's group ended (CSG TERMINATE): the slot is the firmware's to start again */
static bool
slot_stop(unsigned int s)
{
	volatile uint32_t *gin, *gout;
	uint32_t *gc;

	group_blocks(s, &gin, &gout, &gc);
	from_gpu(gout, 64);
	gin[CSG_REQ / 4] = (gin[CSG_REQ / 4] & ~7u);            /* TERMINATE */
	to_gpu(gin, 64);
	from_gpu(glb_out, 64);
	glb_in[GLB_DB_REQ / 4] = glb_out[GLB_DB_ACK / 4] ^ (1u << s);
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	return acked(&gin[CSG_REQ / 4], &gout[CSG_ACK / 4], 7, 200);
}

/*
 * Group G started in slot S, its VM in an address space already: its rings from their start (REQ_RESOURCE held from the
 * stream's first word -- a request made only after the group had been idle never took effect, step 9's lesson -- and the
 * VM's tiler heap set: hakios-metal sets it once, in its first command buffer, and a queue started again without it
 * ended its next draw in an invalid instruction fault, 0x49), everything it was given counted reached, the slot's user
 * pages and suspend buffer its own for now.
 */
static bool
slot_start(unsigned int s, unsigned int g)
{
	struct mali_group *grp = &groups[g];
	volatile uint32_t *gin, *gout, *sin, *sout;
	uint32_t *gc;
	uint64_t shaders = rd64(SHADER_PRESENT);
	unsigned int compute = 0;

	group_blocks(s, &gin, &gout, &gc);
	if (gin == NULL || gout == NULL || grp->queues > gc[GROUP_STREAM_NUM / 4]) {
		return false;
	}
	for (unsigned int q = 0; q < grp->queues; q++) {
		struct mali_queue *mq = &grp->q[q];
		uint64_t *r = (uint64_t *)mq->ring;

		*mq->seqno = mq->submitted;
		to_gpu((void *)mq->seqno, 8);
		mq->seen = mq->submitted;
		bzero((void *)mq->ring, mq->ring_size);
		r[0] = 34ULL << 56 | 0xf;
		for (int h = 0; h < MAX_HEAPS; h++) {
			if (heaps[h].used && heaps[h].vm == grp->vm) {
				r[1] = 1ULL << 56 | 0ULL << 48 | (heaps[h].ctx_va & 0xffffffffffffULL);     /* MOVE48 r0 <- context */
				r[2] = 48ULL << 56;                                                      /* HEAP_SET */
				break;
			}
		}
		mq->insert = 64;
		to_gpu((void *)mq->ring, mq->ring_size);
		mq->user_in = (volatile uint32_t *)slots[s].user[q];
		mq->user_out = (volatile uint32_t *)(slots[s].user[q] + GPU_PAGE);
		bzero((void *)mq->user_in, GPU_PAGE);
		bzero((void *)mq->user_out, GPU_PAGE);
		mq->user_in[CS_INSERT / 4] = (uint32_t)mq->insert;
		to_gpu(mq->user_in, GPU_PAGE);
		to_gpu(mq->user_out, GPU_PAGE);
		stream_blocks(gc, q, &sin, &sout);
		if (sin == NULL || sout == NULL) {
			return false;
		}
		from_gpu(sin, 128);
		from_gpu(sout, 128);
		sin[CS_BASE_LO / 4] = (uint32_t)mq->ring_va;
		sin[CS_BASE_HI / 4] = (uint32_t)(mq->ring_va >> 32);
		sin[CS_SIZE / 4] = mq->ring_size;
		sin[CS_USER_INPUT_LO / 4] = slots[s].user_va[q];
		sin[CS_USER_INPUT_HI / 4] = 0;
		sin[CS_USER_OUTPUT_LO / 4] = slots[s].user_va[q] + GPU_PAGE;
		sin[CS_USER_OUTPUT_HI / 4] = 0;
		sin[CS_CONFIG / 4] = (1 + s) << 8 | 1;
		sin[CS_ACK_IRQ_MASK / 4] = 0xffffffff;
		sin[CS_REQ / 4] = (sout[CS_ACK / 4] & 0xc0000000u) | CS_REQ_IDLE_BITS | CS_STATE_START;
		to_gpu(sin, 128);
	}
	for (uint64_t m = shaders; m; m &= m - 1) {
		compute++;
	}
	from_gpu(gout, 64);
	gin[CSG_DB_REQ / 4] = gout[CSG_DB_ACK / 4] ^ ((1u << grp->queues) - 1);
	gin[CSG_ALLOW_COMPUTE_LO / 4] = (uint32_t)shaders;
	gin[CSG_ALLOW_COMPUTE_HI / 4] = (uint32_t)(shaders >> 32);
	gin[CSG_ALLOW_FRAGMENT_LO / 4] = (uint32_t)shaders;
	gin[CSG_ALLOW_FRAGMENT_HI / 4] = (uint32_t)(shaders >> 32);
	gin[CSG_ALLOW_OTHER / 4] = (uint32_t)rd64(TILER_PRESENT);
	/* SEVERAL GROUPS (TODO 16): each its own CSG priority, 15 - its slot (panthor gives every slot a priority of its
	 * own). With the same one (every hakios-metal client asks 1) the firmware handed the endpoints out in pieces -- one
	 * group the compute ones, another the fragment and tiler -- and each waited for the other's for good (CS blocked
	 * on resources, 0x5). The first slot, backboardd's as a rule, ranks first. (The client's PRIORITY is kept.) */
	gin[CSG_EP_REQ_LO / 4] = compute | compute << 8 | 1u << 16 | (uint32_t)(15 - s) << 28;
	gin[CSG_EP_REQ_HI / 4] = 0;
	gin[CSG_CONFIG / 4] = vms[grp->vm].as;  /* the group's address space: its VM's, for now */
	gin[CSG_SUSPEND_BUF_LO / 4] = slots[s].suspend_va;
	gin[CSG_SUSPEND_BUF_HI / 4] = 0;
	gin[CSG_ACK_IRQ_MASK / 4] = 0xffffffff;
	gin[CSG_REQ / 4] = (gin[CSG_REQ / 4] & ~7u) | CSG_STATE_START;
	to_gpu(gin, 128);
	from_gpu(glb_out, 64);
	glb_in[GLB_DB_REQ / 4] = glb_out[GLB_DB_ACK / 4] ^ (1u << s);
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	if (!acked(&gin[CSG_REQ / 4], &gout[CSG_ACK / 4], 7, 200)) {
		printf("mali: group %u did not start in slot %u (CSG_REQ 0x%x, ACK 0x%x)\n", g, s, gin[CSG_REQ / 4],
		    gout[CSG_ACK / 4]);
		faults("a group's start");
		return false;
	}
	return true;
}

/* has the GPU done everything the group was given (its sequence pages, read past the caches) */
static bool
group_idle(unsigned int g)
{
	for (unsigned int q = 0; q < groups[g].queues; q++) {
		from_gpu(groups[g].q[q].seqno, 8);
		if (*groups[g].q[q].seqno < groups[g].q[q].submitted) {
			return false;
		}
	}
	return true;
}

static unsigned int binds, evictions;

/* the slot of the idle group used longest ago made free (not KEEP's; OTHER_VM: nor one of KEEP's VM, when it is an
 * address space that is wanted); false: every group in a slot is at work */
static bool
evict_one(unsigned int keep, bool other_vm)
{
	int victim = -1;
	uint64_t oldest = ~0ULL;

	flush_gpu();
	for (unsigned int s = 0; s < CSG_SLOTS; s++) {
		unsigned int v;

		if (slot_group[s] == 0) {
			continue;
		}
		v = slot_group[s] - 1;
		if (v == keep || (other_vm && groups[v].vm == groups[keep].vm) || !group_idle(v)) {
			continue;
		}
		if (groups[v].last_used < oldest) {
			oldest = groups[v].last_used;
			victim = (int)s;
		}
	}
	if (victim < 0) {
		return false;
	}
	if (!slot_stop((unsigned int)victim)) {
		printf("mali: group %u in slot %d did not end for another's turn\n", slot_group[victim] - 1, victim);
	}
	groups[slot_group[victim] - 1].slot = -1;
	slot_group[victim] = 0;
	evictions++;
	return true;
}

/* a slot (and its VM an address space) for the group, which is about to be given work; false: none to be had now */
static bool
group_bind(unsigned int g)
{
	struct mali_group *grp = &groups[g];
	unsigned int slots_there = glb_ctl[GLB_GROUP_NUM_W] < CSG_SLOTS ? glb_ctl[GLB_GROUP_NUM_W] : CSG_SLOTS;
	int s = -1;

	if (grp->slot >= 0) {
		return true;
	}
	while (!vm_bind(grp->vm)) {
		if (!evict_one(g, true)) {
			return false;
		}
	}
	for (;;) {
		for (unsigned int i = 0; i < slots_there && s < 0; i++) {
			if (slot_group[i] == 0) {
				s = (int)i;
			}
		}
		if (s >= 0) {
			break;
		}
		if (!evict_one(g, false)) {
			return false;
		}
	}
	if (!slot_make((unsigned int)s) || !slot_start((unsigned int)s, g)) {
		return false;
	}
	slot_group[s] = g + 1;
	grp->slot = s;
	binds++;
	if (binds <= 12 || binds % 200 == 0) {
		printf("mali: group %u (VM %u) in slot %d, address space %u (%u turns given, %u taken from idle groups)\n", g, grp->vm,
		    s, vms[grp->vm].as, binds, evictions);
	}
	return true;
}

/* ring sizes in bytes (powers of two, 4 KiB to 1 MiB); the group's number or -1. It takes a slot of the firmware's
 * only at its first submission (group_bind). */
static void mali_irq_attach(void);
int akita_mali_group_new(int vm, unsigned int nqueues, const uint32_t *ring_sizes, uint8_t priority);
int
akita_mali_group_new(int vm, unsigned int nqueues, const uint32_t *ring_sizes, uint8_t priority)
{
	unsigned int g;
	struct mali_group *grp;

	if (vm <= 0 || vm >= MAX_VMS || !vms[vm].used || nqueues == 0 || nqueues > MAX_QUEUES) {
		return -1;
	}
	for (g = 0; g < MAX_GROUPS && groups[g].used; g++) {
	}
	if (g == MAX_GROUPS) {
		return -1;
	}
	grp = &groups[g];
	bzero(grp, sizeof(*grp));
	grp->slot = -1;
	grp->vm = (unsigned)vm;
	from_gpu((void *)shared_iface, 0x10000);
	{
		volatile uint32_t *gin, *gout;
		uint32_t *gc;

		group_blocks(0, &gin, &gout, &gc);      /* (the slots are alike: what one takes, each does) */
		if (gin == NULL || gout == NULL || nqueues > gc[GROUP_STREAM_NUM / 4]) {
			return -1;
		}
	}
	for (unsigned int q = 0; q < nqueues; q++) {
		struct mali_queue *mq = &grp->q[q];
		uint32_t size = ring_sizes[q];

		if (size < 0x1000 || size > 0x100000 || (size & (size - 1))) {
			size = 0;
		} else {
			mq->ring = vm_kernel_region((unsigned)vm, size, &mq->ring_va, false);
			mq->seqno = (volatile uint64_t *)vm_kernel_region((unsigned)vm, GPU_PAGE, &mq->seqno_va, false);
		}
		if (size == 0 || mq->ring == 0 || mq->seqno == NULL) {
			/* what was made so far given back */
			for (unsigned int k = 0; k <= q; k++) {
				vm_kernel_free((unsigned)vm, grp->q[k].ring_va, grp->q[k].ring, grp->q[k].ring_size);
				vm_kernel_free((unsigned)vm, grp->q[k].seqno_va, (vm_offset_t)grp->q[k].seqno, GPU_PAGE);
			}
			return -1;
		}
		mq->ring_size = size;
		*mq->seqno = 0;
		to_gpu((void *)mq->seqno, 8);
	}
	as_update((unsigned int)vm);            /* the rings and pages just mapped into it (if it has an address space) */
	grp->used = true;
	grp->queues = nqueues;
	grp->priority = priority;
	grp->last_used = mach_absolute_time();
	mali_irq_attach();
	printf("mali: group %u made in VM %d, %u queue(s)\n", g, vm, nqueues);
	return (int)g;
}

void akita_mali_group_free(int g);
void
akita_mali_group_free(int g)
{
	struct mali_group *grp;

	if (g < 0 || g >= MAX_GROUPS || !groups[g].used) {
		return;
	}
	grp = &groups[g];
	if (grp->slot >= 0) {
		slot_stop((unsigned int)grp->slot);
		slot_group[grp->slot] = 0;
		grp->slot = -1;
	}
	grp->used = false;
	/* (11-12) its rings and sequence pages given back: they stayed for good, a set a client that ever was */
	for (unsigned int q = 0; q < grp->queues; q++) {
		vm_kernel_free(grp->vm, grp->q[q].ring_va, grp->q[q].ring, grp->q[q].ring_size);
		vm_kernel_free(grp->vm, grp->q[q].seqno_va, (vm_offset_t)grp->q[q].seqno, GPU_PAGE);
		grp->q[q].ring = 0;
		grp->q[q].seqno = NULL;
	}
	busy_update();
}

/*
 * THE JOB INTERRUPT (the device tree's "JOB", SPI 0x15b: GIC intid 379, level): the firmware raises it for a group's
 * events -- SYNC_UPDATE when a ring's SYNC_ADD64 counted a submission done. The handler only quiets it and wakes the
 * waiters (HakiosGPU's syncobj wait sleeps on mali_irq_event, a millisecond at most as before); the events are acked
 * by akita_mali_reached (mali_ack_events).
 */
#define JOB_IRQ_STATUS          0x100c
#define JOB_IRQ_MASK            0x1008
#define MALI_JOB_INTID          (32 + 0x15b)
#define CSG_SYNC_UPDATE         (1u << 28)
int mali_irq_event;
static bool mali_irq_on;
extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *context, uint32_t intid), void *context);
extern uint64_t gic_v3_spi_count(uint32_t intid);

static void
mali_irq(__unused void *context, __unused uint32_t intid)
{
	wr(JOB_IRQ_CLEAR, rd(JOB_IRQ_STATUS) | rd(JOB_IRQ_RAWSTAT));
	wakeup((void *)&mali_irq_event);       /* (BSD wakeup: thread_wakeup on that event) */
}

static void
mali_irq_attach(void)
{
	unsigned int off = 0;

	if (mali_irq_on || (PE_parse_boot_argn("mali_irq", &off, sizeof(off)) && off == 0)) {
		return;
	}
	wr(JOB_IRQ_CLEAR, 0xffffffff);
	wr(MMU_IRQ_CLEAR, 0xffffffff);          /* (an AS1 bus fault left from start-up work: mmu_faults would act on it) */
	if (gic_v3_spi_attach(MALI_JOB_INTID, false, mali_irq, NULL) == KERN_SUCCESS) {
		wr(JOB_IRQ_MASK, 0xffffffff);
		mali_irq_on = true;
		printf("mali: the JOB interrupt (intid %u) taken: waits wake on it\n", MALI_JOB_INTID);
	} else {
		printf("mali: the JOB interrupt (intid %u) could not be taken: waits poll\n", MALI_JOB_INTID);
	}
}

/* how many JOB interrupts so far (sysctl-free check from the service's log) */
uint64_t akita_mali_irq_count(void);
uint64_t
akita_mali_irq_count(void)
{
	return mali_irq_on ? gic_v3_spi_count(MALI_JOB_INTID) : 0;
}

/* a group's events acked -- SYNC_UPDATE and IDLE in CSG_REQ set to CSG_ACK's, the CS interrupts acked -- and the
 * firmware told (panthor's sched_process_csg_irq_locked), so that the next one raises the interrupt again */
static void
mali_ack_events(unsigned int g)
{
	volatile uint32_t *gin, *gout;
	uint32_t *gc, mask = CSG_SYNC_UPDATE | CSG_REQ_IDLE;

	group_blocks(g, &gin, &gout, &gc);
	from_gpu(gout, 64);
	if (((gin[CSG_REQ / 4] ^ gout[CSG_ACK / 4]) & mask) == 0 && gin[CSG_IRQ_ACK / 4] == gout[CSG_IRQ_REQ / 4]) {
		return;
	}
	gin[CSG_IRQ_ACK / 4] = gout[CSG_IRQ_REQ / 4];
	gin[CSG_REQ / 4] = (gin[CSG_REQ / 4] & ~mask) | (gout[CSG_ACK / 4] & mask);
	to_gpu(gin, 64);
	from_gpu(glb_out, 64);
	glb_in[GLB_DB_REQ / 4] = (glb_in[GLB_DB_REQ / 4] & ~(1u << g)) | ((glb_out[GLB_DB_ACK / 4] ^ (1u << g)) & (1u << g));
	to_gpu(glb_in, 64);
	ring_doorbell(0);
}

/*
 * A program's stream into a queue: the ring's instructions (CALL it; WAIT; store the new sequence number), then the
 * group woken as step 9 found it must be. The sequence number (1, 2, ...) or 0 if there is no room or no queue.
 */
static void mmu_faults(void);
static void tiler_oom(void);
uint64_t akita_mali_submit(int g, unsigned int q, uint64_t stream, uint32_t size);
uint64_t
akita_mali_submit(int g, unsigned int q, uint64_t stream, uint32_t size)
{
	struct mali_queue *mq;
	volatile uint32_t *gin, *gout;
	uint32_t *gc;
	uint64_t cmd[24];
	unsigned int n = 0, s;

	if (g < 0 || g >= MAX_GROUPS || !groups[g].used || q >= groups[g].queues || (size & 7) || size == 0) {
		return 0;
	}
	/* A SLOT FOR THE GROUP, if it has none (its first work, or its slot went to another while it was idle): a free
	 * one, or an idle group's; with every slot's group at work, waited for -- their work goes on meanwhile (the
	 * heaps' and faults' events are looked after here: the waiters that do it otherwise stand at HakiosGPU's lock) */
	uint64_t bind_deadline;

	clock_interval_to_deadline(3000, 1000 * 1000, &bind_deadline);
	while (!group_bind((unsigned int)g)) {
		if (mach_absolute_time() >= bind_deadline) {
			printf("mali: group %d: no slot in 3 s (every slot's group at work, or it did not start)\n", g);
			return 0;
		}
		flush_gpu();
		mmu_faults();
		tiler_oom();
		delay_for_interval(1, 1000 * 1000);
	}
	s = (unsigned int)groups[g].slot;
	groups[g].last_used = vms[groups[g].vm].last_used = mach_absolute_time();
	mq = &groups[g].q[q];
	from_gpu(mq->user_out, 16);
	if (mq->insert + sizeof(cmd) - *(volatile uint64_t *)mq->user_out > mq->ring_size) {
		return 0;                       /* the ring is full: the GPU is that far behind */
	}
	if (vms[groups[g].vm].flush_pending) {
		as_update(groups[g].vm);        /* (the binds since its last update: akita_mali_vm_flush_later) */
		vms[groups[g].vm].flush_pending = false;
	}
	mq->submitted++;
	busy_update();
	{
		unsigned int calltest = 0;      /* mali_calltest=1: the ring stores 0xbbbb at stream - 0x800 + 0x130 first */

		PE_parse_boot_argn("mali_calltest", &calltest, sizeof(calltest));
		if (calltest) {
			cmd[n++] = 1ULL << 56 | 92ULL << 48 | ((stream - 0x800 + 0x130) & 0xffffffffffffULL);
			cmd[n++] = 2ULL << 56 | 94ULL << 48 | 0xbbbb;
			cmd[n++] = 21ULL << 56 | 94ULL << 48 | 92ULL << 40 | 0x1ULL << 16;
		}
	}
	cmd[n++] = 1ULL << 56 | 92ULL << 48 | (stream & 0xffffffffffffULL);     /* MOVE48 r92:r93 <- the stream */
	cmd[n++] = 2ULL << 56 | 94ULL << 48 | size;                             /* MOVE32 r94 <- its length */
	cmd[n++] = 32ULL << 56 | 92ULL << 40 | 94ULL << 32;                     /* CALL [r92], r94 */
	cmd[n++] = 3ULL << 56 | 0xffffULL << 16;                                /* WAIT on every scoreboard */
	cmd[n++] = 1ULL << 56 | 92ULL << 48 | mq->seqno_va;                     /* MOVE48 r92 <- the sequence page */
	cmd[n++] = 1ULL << 56 | 94ULL << 48 | 1;                                /* MOVE48 r94:r95 <- 1 */
	/* SYNC_ADD64 [r92] += r94, scope CSG: the firmware's sync update event, the JOB interrupt (mali_irq) -- the
	 * page counts the submissions done, seeded with the count at the group's start */
	cmd[n++] = 51ULL << 56 | 92ULL << 40 | 94ULL << 32 | 2ULL << 1;
	cmd[n++] = 0;                                                           /* NOP */
	while ((mq->insert + n * 8) & 63) {
		cmd[n++] = 0;                   /* NOPs to the end of the line: each submission in whole lines of the ring */
	}
	for (unsigned int i = 0; i < n; i++) {
		*(uint64_t *)(mq->ring + ((mq->insert + i * 8) & (mq->ring_size - 1))) = cmd[i];
	}
	{
		/* only what was written to the GPU (the whole ring was cleaned at every submission), in two pieces when it
		 * wraps around the ring's end */
		uint64_t at = mq->insert & (mq->ring_size - 1), len = (uint64_t)n * 8;

		if (at + len <= mq->ring_size) {
			to_gpu((void *)(mq->ring + at), (size_t)len);
		} else {
			to_gpu((void *)(mq->ring + at), (size_t)(mq->ring_size - at));
			to_gpu((void *)mq->ring, (size_t)(at + len - mq->ring_size));
		}
	}
	mq->insert += n * 8;
	*(volatile uint64_t *)mq->user_in = mq->insert;
	to_gpu(mq->user_in, 16);
	/* the wake (step 9): caches, the group's idle event acked and its endpoints asked for again, the doorbells */
	flush_gpu();
	group_blocks(s, &gin, &gout, &gc);
	from_gpu(gout, 64);
	gin[CSG_REQ / 4] = (gin[CSG_REQ / 4] & ~CSG_REQ_IDLE) | (gout[CSG_ACK / 4] & CSG_REQ_IDLE);
	gin[CSG_REQ / 4] = (gin[CSG_REQ / 4] & ~CSG_REQ_EP_CFG) | ((gout[CSG_ACK / 4] ^ CSG_REQ_EP_CFG) & CSG_REQ_EP_CFG);
	gin[CSG_IRQ_ACK / 4] = gout[CSG_IRQ_REQ / 4];
	gin[CSG_DB_REQ / 4] = (gin[CSG_DB_REQ / 4] & ~(1u << q)) | ((gout[CSG_DB_ACK / 4] ^ (1u << q)) & (1u << q));
	to_gpu(gin, 64);
	wr(JOB_IRQ_CLEAR, 0xffffffff);
	from_gpu(glb_out, 64);
	glb_in[GLB_DB_REQ / 4] = (glb_in[GLB_DB_REQ / 4] & ~(1u << s)) | ((glb_out[GLB_DB_ACK / 4] ^ (1u << s)) & (1u << s));
	to_gpu(glb_in, 64);
	ring_doorbell(0);
	ring_doorbell(1 + s);
	return mq->submitted;
}

/*
 * TILER OUT OF MEMORY (panthor_sched.c's group_process_tiler_oom, panthor_heap.c's panthor_heap_grow): a queue whose
 * render passes filled its heap raises CS_REQ/ACK bit 26 and stops. The answer: one more chunk, zeroed, its header's
 * next 0, given in the queue's input as heap_start = heap_end = its address | its size >> 12, the event acked (REQ's bit
 * set to ACK's) and the group's doorbell rung. A heap at its most chunks gets 0: the firmware then waits for passes in
 * flight to finish and give chunks back. Checked wherever a queue's progress is (akita_mali_reached, polled by every
 * waiter): the GPU raises no interrupt here that anyone takes.
 */
#define CS_TILER_OOM            (1u << 26)
#define CS_HEAP_START           0x20
#define CS_HEAP_END             0x28
#define CS_HEAP_ADDRESS         0xd0

static uint64_t
heap_grow(unsigned int vm, uint64_t ctx_va)
{
	/* (the heap by its context AND the VM of the group that ran out: every client's first heap context is at the same
	 * address of its own VM -- found by address alone, a second client's OOM grew backboardd's heap, and its own tiler
	 * wrote to chunks its VM never had: a page fault a chunk) */
	for (int h = 0; h < MAX_HEAPS; h++) {
		if (!heaps[h].used || heaps[h].ctx_va != ctx_va || heaps[h].vm != vm) {
			continue;
		}
		unsigned int i = heaps[h].chunks, vm = heaps[h].vm;
		uint32_t size = heaps[h].chunk_size;

		if (i >= heaps[h].max_chunks) {
			/* no chunk: the firmware runs the queue's tiler OOM handler (hakios-metal's incremental rendering: the lists
			 * so far drawn, their chunks given back), or with none waits for passes in flight to give chunks back */
			heaps[h].at_most++;
			if (!heaps[h].told) {
				heaps[h].told = true;
				printf("mali: tiler heap %d: out of memory at %u chunks of %u KiB, its most: no chunk given (the queue's "
				    "handler draws what is binned so far)\n", h, i, size >> 10);
			}
			return 0;
		}
		vms[vm].kernel_next = (vms[vm].kernel_next + size - 1) & ~(uint64_t)(size - 1);
		heaps[h].chunk[i] = vm_kernel_region(vm, size, &heaps[h].chunk_va[i], false);
		if (heaps[h].chunk[i] == 0) {
			printf("mali: tiler heap %d: no memory for chunk %u\n", h, i);
			return 0;
		}
		*(uint64_t *)heaps[h].chunk[i] = 0;
		clean_dcache(heaps[h].chunk[i], 64, FALSE);
		heaps[h].chunks = i + 1;
		as_update(vm);
		printf("mali: tiler heap %d: grown to %u chunks (%u MiB)\n", h, i + 1, (i + 1) * (size >> 10) >> 10);
		return (heaps[h].chunk_va[i] & ~0xfffULL) | (size >> 12);
	}
	printf("mali: tiler out of memory for heap context %#llx, which is none of ours\n", ctx_va);
	return 0;
}

/*
 * A QUEUE'S FATAL ERROR (CS_REQ/ACK bit 31: a GPU page fault, an invalid instruction...): the queue stops for good, and
 * every later wait on the group timed out -- the screen went black. panthor kills the group and fails its jobs; here
 * the group is ended (CSG TERMINATE) and started again in place with empty rings, everything submitted to it counted
 * as reached (the waiters go on; the frames in it are lost), so that the window server keeps drawing. A fault that is
 * not fatal (bit 30) is acknowledged and the queue goes on, as panthor does.
 */
#define CS_FAULT_BIT            (1u << 30)
#define CS_FATAL_BIT            (1u << 31)

static void
group_restart(unsigned int g)
{
	struct mali_group *grp = &groups[g];
	unsigned int s;

	if (grp->slot < 0) {
		return;                         /* (not in a slot: nothing of it is with the firmware) */
	}
	s = (unsigned int)grp->slot;
	grp->restarts++;
	if (!slot_stop(s)) {
		printf("mali: group %u: not ended for its restart (slot %u)\n", g, s);
	}
	as_update(grp->vm);             /* (its address space back on, if a page fault turned it off: mmu_faults) */
	for (unsigned int q = 0; q < grp->queues; q++) {
		struct mali_queue *mq = &grp->q[q];

		/* everything it was given counted done (slot_start); what the GPU had not reached is kept as lost (the
		 * faulting submission and those queued behind it) */
		from_gpu((void *)mq->seqno, 8);
		if (*mq->seqno < mq->submitted) {
			mq->lost[mq->lost_next % LOST_RANGES].after = *mq->seqno;
			mq->lost[mq->lost_next % LOST_RANGES].upto = mq->submitted;
			mq->lost_next++;
			printf("mali: group %u queue %u: submissions %llu..%llu are lost with the restart\n", g, q, *mq->seqno + 1,
			    mq->submitted);
		}
	}
	if (!slot_start(s, g)) {
		/* (the slot is given up: the group takes one again at its next submission) */
		slot_group[s] = 0;
		grp->slot = -1;
		for (unsigned int q = 0; q < grp->queues; q++) {
			*grp->q[q].seqno = grp->q[q].submitted;
			to_gpu((void *)grp->q[q].seqno, 8);
			grp->q[q].seen = grp->q[q].submitted;
		}
		return;
	}
	wr(MMU_IRQ_CLEAR, 0xffffffff);
	printf("mali: group %u started again after a fatal error (restart %u)\n", g, grp->restarts);
}

/*
 * A GPU PAGE FAULT (MMU_IRQ_RAWSTAT bit n, a bus fault bit 16 + n, for address space n -- a VM's AS is its number): the
 * address space stalls the access and waits; nothing in the queue reports it (the CS just stays blocked: the 10 s
 * timeout). As panthor: the fault cleared and the address space turned off (unmapped), so that what stalled fails;
 * then the groups on it restarted (group_restart turns it on again).
 */
static void
mmu_faults(void)
{
	uint32_t raw = rd(MMU_IRQ_RAWSTAT);

	if (raw == 0) {
		return;
	}
	for (unsigned int as = 1; as < AS_SLOTS; as++) {
		uint32_t base = AS0 + as * 0x40, status = rd(base + AS_FAULTSTATUS);
		/* page faults only: AS1's bus fault bit comes back at once after every clear, from start-up on, and the fault
		 * status keeps the last page fault's -- acting on it restarted the group in a loop */
		uint32_t bits = 1u << as;
		unsigned int vm = as_vm[as];
		bool any = false;

		if (vm == 0 || !vms[vm].used || !(raw & bits)) {
			continue;
		}
		printf("mali: address space %u (VM %u): %s fault 0x%x at 0x%llx; it is turned off and its groups restart\n", as, vm,
		    "page", status, rd64(base + AS_FAULTADDRESS));
		faults("a GPU page fault");
		wr(MMU_IRQ_CLEAR, bits);
		as_off(as);
		for (unsigned int g = 0; g < MAX_GROUPS; g++) {
			if (groups[g].used && groups[g].vm == vm && groups[g].slot >= 0) {
				group_restart(g);
				any = true;
			}
		}
		if (!any) {
			as_update(vm);
		}
	}
}

static int mali_heap_say = -1;      /* boot-arg mali_heap_say: each tiler OOM said */

static void
tiler_oom(void)
{
	static volatile UInt32 busy;

	if (!OSCompareAndSwap(0, 1, &busy)) {
		return;                         /* (another waiter is at it) */
	}
	if (mali_heap_say < 0) {
		int say = 0;
		PE_parse_boot_argn("mali_heap_say", &say, sizeof(say));
		mali_heap_say = say;
	}
	for (unsigned int s = 0; s < CSG_SLOTS; s++) {
		volatile uint32_t *gin, *gout, *sin, *sout;
		uint32_t *gc;
		bool rang = false;
		unsigned int g;

		if (slot_group[s] == 0) {
			continue;
		}
		g = slot_group[s] - 1;          /* (the group that has the slot) */
		group_blocks(s, &gin, &gout, &gc);
		for (unsigned int q = 0; q < groups[g].queues; q++) {
			stream_blocks(gc, q, &sin, &sout);
			from_gpu(sout, 256);
			uint32_t events = sin[CS_REQ / 4] ^ sout[CS_ACK / 4];
			if (events & CS_FATAL_BIT) {
				printf("mali: group %u queue %u: fatal 0x%x (info 0x%x%08x), cmd at 0x%x%08x; the group restarts\n", g, q,
				    sout[CS_FATAL / 4], sout[0x94 / 4], sout[0x90 / 4], sout[0x44 / 4], sout[0x40 / 4]);
				faults("a queue's fatal error");
				group_restart(g);
				rang = false;
				break;
			}
			if (events & CS_FAULT_BIT) {
				printf("mali: group %u queue %u: fault 0x%x (info 0x%x%08x), acknowledged\n", g, q, sout[CS_FAULT / 4],
				    sout[0x8c / 4], sout[0x88 / 4]);
				sin[CS_REQ / 4] = (sin[CS_REQ / 4] & ~CS_FAULT_BIT) | (sout[CS_ACK / 4] & CS_FAULT_BIT);
				to_gpu(sin, 64);
				from_gpu(gout, 64);
				gin[CSG_DB_REQ / 4] = (gin[CSG_DB_REQ / 4] & ~(1u << q)) | ((gout[CSG_DB_ACK / 4] ^ (1u << q)) & (1u << q));
				rang = true;
			}
			if (!(events & CS_TILER_OOM)) {
				continue;
			}
			uint64_t ctx = (uint64_t)sout[CS_HEAP_ADDRESS / 4] | (uint64_t)sout[CS_HEAP_ADDRESS / 4 + 1] << 32;
			uint64_t chunk = heap_grow(groups[g].vm, ctx);

			if (mali_heap_say) {
				from_gpu(gout, 64);
				printf("mali: group %u queue %u: tiler OOM: passes started %u, tiled %u, fragment done %u; CS_REQ 0x%x ACK 0x%x, "
				    "CSG_IRQ_REQ 0x%x ACK 0x%x; wait 0x%x blocked 0x%x; chunk given 0x%llx\n", g, q, sout[0xc0 / 4],
				    sout[0xc4 / 4], sout[0xcc / 4], sin[CS_REQ / 4], sout[CS_ACK / 4], gout[CSG_IRQ_REQ / 4], gin[CSG_IRQ_ACK / 4],
				    sout[0x48 / 4], sout[0x60 / 4], chunk);
			}

			*(volatile uint64_t *)&sin[CS_HEAP_START / 4] = chunk;
			*(volatile uint64_t *)&sin[CS_HEAP_END / 4] = chunk;
			sin[CS_REQ / 4] = (sin[CS_REQ / 4] & ~CS_TILER_OOM) | (sout[CS_ACK / 4] & CS_TILER_OOM);
			to_gpu(sin, 64);
			/* the queue's doorbell request toggled from its ACK as the firmware has it NOW: from a stale copy (the
			 * firmware had acked the last OOM's doorbell since) the "toggle" left REQ = ACK, no doorbell -- the
			 * firmware never read the new chunk, and a pass needing more than ~3 grows stalled for good */
			from_gpu(gout, 64);
			gin[CSG_DB_REQ / 4] = (gin[CSG_DB_REQ / 4] & ~(1u << q)) | ((gout[CSG_DB_ACK / 4] ^ (1u << q)) & (1u << q));
			rang = true;
		}
		if (rang) {
			from_gpu(gout, 64);
			gin[CSG_IRQ_ACK / 4] = gout[CSG_IRQ_REQ / 4];
			to_gpu(gin, 64);
			from_gpu(glb_out, 64);
			glb_in[GLB_DB_REQ / 4] = (glb_in[GLB_DB_REQ / 4] & ~(1u << s)) | ((glb_out[GLB_DB_ACK / 4] ^ (1u << s)) & (1u << s));
			to_gpu(glb_in, 64);
			ring_doorbell(0);
			ring_doorbell(1 + s);
		}
	}
	busy = 0;
}

/* was the submission counted done by a restart after a fault, its work never done (one of the queue's last
 * LOST_RANGES such runs) */
bool akita_mali_lost(int g, unsigned int q, uint64_t seqno);
bool
akita_mali_lost(int g, unsigned int q, uint64_t seqno)
{
	if (g < 0 || g >= MAX_GROUPS || !groups[g].used || q >= groups[g].queues) {
		return false;
	}
	for (unsigned int i = 0; i < LOST_RANGES; i++) {
		if (seqno > groups[g].q[q].lost[i].after && seqno <= groups[g].q[q].lost[i].upto) {
			return true;
		}
	}
	return false;
}

/* the last sequence number the queue reached (what the GPU wrote is read after its caches are flushed) */
uint64_t akita_mali_reached(int g, unsigned int q);
uint64_t
akita_mali_reached(int g, unsigned int q)
{
	if (g < 0 || g >= MAX_GROUPS || !groups[g].used || q >= groups[g].queues) {
		return 0;
	}
	if (groups[g].slot < 0) {
		/* (in no slot: it was idle when its slot went, or has had no work yet -- its page says where it stands) */
		from_gpu(groups[g].q[q].seqno, 8);
		groups[g].q[q].seen = *groups[g].q[q].seqno;
		busy_update();
		return groups[g].q[q].seen;
	}
	if (mali_irq_on) {
		/* nothing new since the last look -- no interrupt (every event raises one), and the page (read past the CPU's
		 * cache, possibly behind the GPU's but never ahead) already says all done: no cache flush, no events. The
		 * flush cost most of the time a short command buffer waited. */
		static uint64_t irqs_seen;
		uint64_t irqs = akita_mali_irq_count();

		from_gpu(groups[g].q[q].seqno, 8);
		if (irqs == irqs_seen && *groups[g].q[q].seqno >= groups[g].q[q].submitted) {
			groups[g].q[q].seen = *groups[g].q[q].seqno;
			busy_update();
			return groups[g].q[q].seen;
		}
		irqs_seen = irqs;
	}
	flush_gpu();
	mmu_faults();
	tiler_oom();
	if (mali_irq_on) {
		for (unsigned int i = 0; i < CSG_SLOTS; i++) {
			if (slot_group[i] != 0) {
				mali_ack_events(i);     /* (a slot's events) */
			}
		}
	}
	from_gpu(groups[g].q[q].seqno, 8);
	groups[g].q[q].seen = *groups[g].q[q].seqno;
	busy_update();
	return groups[g].q[q].seen;
}

/*
 * For the display (MiniIOSurfaceDrawn, hakios.akita_gpu.cpp): what every queue has been given so far, and whether the GPU
 * has done that much. A surface handed to the panel was drawn by work submitted before its swap; until that work is done
 * the panel must not show it (11-10, the tester's videos: half the old picture and half the new, streaks in tile order --
 * the Mali was still drawing the surface it showed). Under HakiosGPU's lock, as every other call here.
 */
unsigned akita_mali_snapshot(uint64_t *snap, unsigned most);
bool akita_mali_done(const uint64_t *snap, unsigned n);

unsigned
akita_mali_snapshot(uint64_t *snap, unsigned most)
{
	unsigned n = 0;

	for (unsigned g = 0; g < MAX_GROUPS; g++) {
		for (unsigned q = 0; q < MAX_QUEUES && n < most; q++, n++) {
			snap[n] = groups[g].used && q < groups[g].queues ? groups[g].q[q].submitted : 0;
		}
	}
	return n;
}

bool
akita_mali_done(const uint64_t *snap, unsigned n)
{
	for (unsigned i = 0; i < n; i++) {
		unsigned g = i / MAX_QUEUES, q = i % MAX_QUEUES;

		if (snap[i] != 0 && groups[g].used && q < groups[g].queues && akita_mali_reached((int)g, q) < snap[i]) {
			return false;
		}
	}
	return true;
}

/* every group's queues as the firmware sees them (a wait that timed out asks for this) */
void akita_mali_debug(void);
void
akita_mali_debug(void)
{
	for (unsigned int s = 0; s < CSG_SLOTS; s++) {
		volatile uint32_t *gin, *gout, *sin, *sout;
		uint32_t *gc;
		unsigned int g;

		if (slot_group[s] == 0) {
			continue;
		}
		g = slot_group[s] - 1;          /* (the groups in slots: the others have nothing with the firmware) */
		printf("mali: slot %u: group %u, VM %u in address space %u\n", s, g, groups[g].vm, vms[groups[g].vm].as);
		group_blocks(s, &gin, &gout, &gc);
		from_gpu(gout, 64);
		gin[CSG_REQ / 4] = (gin[CSG_REQ / 4] & ~(1u << 5)) | ((gout[CSG_ACK / 4] ^ (1u << 5)) & (1u << 5));
		to_gpu(gin, 64);
		from_gpu(glb_out, 64);
		glb_in[GLB_DB_REQ / 4] = (glb_in[GLB_DB_REQ / 4] & ~(1u << s)) | ((glb_out[GLB_DB_ACK / 4] ^ (1u << s)) & (1u << s));
		to_gpu(glb_in, 64);
		ring_doorbell(0);
		acked(&gin[CSG_REQ / 4], &gout[CSG_ACK / 4], 1u << 5, 100);
		flush_gpu();
		from_gpu(gout, 64);
		printf("mali: group %u: CSG_REQ 0x%x ACK 0x%x, DB_REQ 0x%x ACK 0x%x, IRQ_REQ 0x%x ACK 0x%x, endpoints 0x%x of 0x%x\n", g,
		    gin[CSG_REQ / 4], gout[CSG_ACK / 4], gin[CSG_DB_REQ / 4], gout[CSG_DB_ACK / 4], gout[CSG_IRQ_REQ / 4],
		    gin[CSG_IRQ_ACK / 4], gout[0x10 / 4], gout[0x14 / 4]);
		for (unsigned int q = 0; q < groups[g].queues; q++) {
			struct mali_queue *mq = &groups[g].q[q];

			stream_blocks(gc, q, &sin, &sout);
			from_gpu(sout, 256);
			from_gpu(mq->user_out, 16);
			from_gpu(mq->seqno, 8);
			printf("mali: group %u queue %u: insert 0x%llx, extract 0x%llx, active %u; cmd at 0x%x%08x, CS_REQ 0x%x ACK "
			    "0x%x, wait 0x%x, blocked 0x%x, requested 0x%x, fault 0x%x fatal 0x%x (info 0x%x%08x); seqno %llu of %llu, "
			    "the start's check word 0x%llx\n",
			    g, q, mq->insert, *(volatile uint64_t *)mq->user_out, mq->user_out[CS_ACTIVE / 4], sout[0x44 / 4],
			    sout[0x40 / 4], sin[CS_REQ / 4], sout[CS_ACK / 4], sout[0x48 / 4], sout[0x60 / 4], sout[0x4c / 4],
			    sout[CS_FAULT / 4], sout[CS_FATAL / 4], sout[0x8c / 4], sout[0x88 / 4], *mq->seqno, mq->submitted,
			    mq->seqno[1]);	/* the firmware's count of the queue's render passes on its tiler heap (panthor_fw.h: heap_vt_start, heap_vt_end,
			 * heap_frag_end, heap_address), its scoreboards, and the heap chunk it was last given (heap_start, heap_end) */
			printf("mali: group %u queue %u: heap passes started %u, tiled %u, fragment done %u; heap 0x%x%08x; scoreboards "
			    "0x%x; given chunks 0x%x%08x..0x%x%08x\n", g, q, sout[0xc0 / 4], sout[0xc4 / 4], sout[0xcc / 4], sout[0xd4 / 4],
			    sout[0xd0 / 4], sout[0x5c / 4], sin[0x24 / 4], sin[0x20 / 4], sin[0x2c / 4], sin[0x28 / 4]);
		}
	}
	faults("the service asked");
	fwlog_dump("the service asked");
	/* the tiler heaps as the GPU left them: each context, and its newest chunk's header and top (the polygon lists) */
	for (int h = 0; h < MAX_HEAPS; h++) {
		if (!heaps[h].used) {
			continue;
		}
		uint32_t *ctx = (uint32_t *)heaps[h].ctx;
		vm_offset_t top = heaps[h].chunk[heaps[h].chunks - 1];
		uint32_t *head = (uint32_t *)top, *end = (uint32_t *)(top + heaps[h].chunk_size - 0x200);
		unsigned nonzero = 0, first = ~0u;

		flush_dcache((vm_offset_t)ctx, 64, FALSE);
		flush_dcache(top, 256, FALSE);
		flush_dcache((vm_offset_t)end, 0x200, FALSE);
		for (unsigned int i = 0; i < 0x200 / 4; i++) {
			if (end[i] != 0) {
				nonzero++;
				first = first == ~0u ? i * 4 : first;
			}
		}
		printf("mali: tiler heap %d: context %08x %08x %08x %08x %08x %08x %08x %08x; chunk 0x%llx: %08x %08x %08x %08x "
		    "%08x %08x %08x %08x %08x %08x %08x %08x; its last 0x200 bytes: %u words set, the first at +0x%x\n", h, ctx[0],
		    ctx[1], ctx[2], ctx[3], ctx[4], ctx[5], ctx[6], ctx[7], heaps[h].chunk_va[heaps[h].chunks - 1], head[0], head[1],
		    head[2], head[3], head[4], head[5], head[6], head[7], head[16], head[17], head[18], head[19], nonzero, first);
		if (first != ~0u) {
			uint32_t *w = &end[first / 4];

			printf("mali: tiler heap %d: at +0x%x of the last 0x200: %08x %08x %08x %08x %08x %08x %08x %08x\n", h, first, w[0],
			    w[1], w[2], w[3], w[4], w[5], w[6], w[7]);
		}
	}
	{
		volatile uint8_t *m = (volatile uint8_t *)ml_io_map(S2MPU_G3D_PHYS, 0x10000);
		uint32_t st = *(volatile uint32_t *)(m + S2MPU_FAULT_STATUS);

		printf("mali: S2MPU fault status 0x%x\n", st);
		for (unsigned int vid = 0; vid < 8; vid++) {
			if (st & (1u << vid)) {
				printf("mali: S2MPU vid %u: fault at 0x%x%08x, info 0x%x\n", vid, *(volatile uint32_t *)(m + 0x2008 + vid * 0x20),
				    *(volatile uint32_t *)(m + 0x2004 + vid * 0x20), *(volatile uint32_t *)(m + 0x2010 + vid * 0x20));
			}
		}
	}
}

/* the GPU's caches flushed: what it wrote into programs' buffers is in memory for the processors */
void akita_mali_flush(void);
void
akita_mali_flush(void)
{
	flush_gpu();
}

/* what DRM_PANTHOR_DEV_QUERY tells: register values, and the command stream interface's sizes */
void akita_mali_info(uint32_t *regs, uint32_t *csif);
void
akita_mali_info(uint32_t *regs, uint32_t *csif)
{
	uint32_t *gc = glb_ctl + GROUP_CONTROL_0 / 4;
	uint32_t *sc = gc + STREAM_CONTROL_0 / 4;
	uint32_t features;

	for (unsigned int i = 0; i < 0x130 / 4; i++) {          /* regs: 256 words; only registers there are read */
		regs[i] = rd(i * 4);
	}
	regs[COHERENCY_FEATURES / 4] = rd(COHERENCY_FEATURES);
	from_gpu((void *)shared_iface, 0x10000);
	features = sc[STREAM_FEATURES / 4];
	csif[0] = glb_ctl[GLB_GROUP_NUM_W];             /* csg_slot_count */
	csif[1] = gc[GROUP_STREAM_NUM / 4];             /* cs_slot_count */
	csif[2] = (features & 0xff) + 1;                /* cs_reg_count: CS_FEATURES work registers */
	csif[3] = (features >> 8) & 0xff;               /* scoreboard_slot_count */
	csif[4] = 4;                                    /* unpreserved_cs_reg_count: r92..r95 are the kernel's */
}

void akita_gpu_start(void);     /* hakios.akita_gpu.cpp: the service */

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
		unsigned int probe_regions = 0;

		/* mali_probe=1: can the GPU read each of the phone's four RAM regions (Android's tree: 0x8000_0000,
		 * 0x8_8000_0000, 0x9_0000_0000, 0x9_8000_0000, 2 GiB each)? -- buffers from 0x9_9eaa_4000 read as nothing */
		if (PE_parse_boot_argn("mali_probe", &probe_regions, sizeof(probe_regions)) && probe_regions) {
			static const uint64_t at[] = { 0x80000000ULL, 0xc0000000ULL, 0xfff00000ULL, 0x880000000ULL, 0x8f0000000ULL,
			    0x8fff00000ULL, 0x900000000ULL, 0x940000000ULL, 0x980000000ULL, 0x99eaa4000ULL, 0x9fff00000ULL };

			for (unsigned int i = 0; i < sizeof(at) / sizeof(at[0]); i++) {
				int r = probe_page(at[i]);

				printf("mali: probe: 0x%llx: %s\n", at[i], r == 0 ? "REFUSED (a bus fault)" : r == 1 ?
				    "read (an invalid entry)" : r == 2 ? "read (something walked)" : "no answer");
			}
			goto out;
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

	if (!announce(6, "the firmware's global interface: a ping, the shader cores allowed, what it offers")) {
		goto out;
	}
	if (!global_setup()) {
		goto out;
	}
	{
		unsigned int keep = 0;

		PE_parse_boot_argn("mali_keep", &keep, sizeof(keep));
		if (keep) {
			kept = true;
			printf("mali: the GPU kept on: HakiosGPU (panthor's interface) for programs\n");
			akita_gpu_start();
			return;
		}
	}
	if (!announce(7, "a command stream group (slot 0) with one queue, in address space 1")) {
		goto out;
	}
	if (!group_start()) {
		goto out;
	}
	if (!announce(8, "the first job: four instructions that store two words into a page of ours")) {
		goto out;
	}
	{
		/*
		 * mali_skipfirst=1: the compute job as the queue's first work (without step 8's store job before it).
		 */
		unsigned int skip = 0;

		PE_parse_boot_argn("mali_skipfirst", &skip, sizeof(skip));
		if (!skip) {
			first_job();
		}
	}
	if (!announce(9, "the first compute job: a shader of four instructions on the shader cores")) {
		goto out;
	}
	{
		unsigned int repeat = 1;        /* mali_repeat=N: the compute job N times, the group idle in between */

		PE_parse_boot_argn("mali_repeat", &repeat, sizeof(repeat));
		for (unsigned int i = 0; i < repeat && i < 16; i++) {
			compute_job();
		}
	}

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
		uint32_t keep = 0;

		if (!PE_parse_boot_argn("mali_keep", &keep, sizeof(keep)) || keep == 0) {
			return;
		}
		steps = 6;              /* mali_keep=1 alone: up to the global interface, then kept */
	}
	PE_parse_boot_argn("mali_delay", &delay_s, sizeof(delay_s));
	printf("mali: steps 1 to %u in %u s\n", steps, delay_s);
	call = thread_call_allocate(mali_run, NULL);
	clock_interval_to_deadline(delay_s, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_mali_init);
