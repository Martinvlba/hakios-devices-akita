/*
 * The Pixel 8a's display pipeline, as Google's bootloader leaves it (display_probe=1): a look before driving it.
 *
 * The panel (google-ak3b, 1080 x 2400, DSI command mode) is refreshed by DECON0 at every TE from what DPP0's DMA reads
 * (U-Boot's environment unmasks the trigger; MiniIOMobileFramebuffer draws into that buffer, 0xfac00000, by the
 * processor). To flip between buffers, scale in the DPP and read iOS's surfaces directly, the registers that do it are
 * read here first, nothing written: DECON0's main, window and window-control blocks, DPP0's DMA (RDMA) and DPP (scaler)
 * blocks, and the SysMMU in front of DPUF0 (whether the DMA's addresses are physical or translated).
 *
 * Addresses: Android's device tree for akita (devices/akita/boot-info/fdt.dtb: drmdecon@19470000, drmdpp@19900000,
 * sysmmu@19840000); register names: Google's display driver for zuma (kernel/google-modules/display, samsung/cal_9865:
 * regs-decon.h, regs-dpp.h). Not every block need be readable from here (secure, or clocked apart): see below.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <sys/systm.h>
#include <kern/startup.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <mach/vm_param.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

static const struct {
	const char *name;
	uint64_t phys;
	uint32_t from, to;      /* byte offsets dumped, [from, to) */
} blocks[] = {
	/* the safest first: DECON0's TRIG_CON is read (and written) by U-Boot already */
	{ "decon0.main", 0x19470000, 0x000, 0x100 },
	{ "dpp0.rdma", 0x19900000, 0x000, 0x0a0 },
	{ "decon0.win", 0x19480000, 0x000, 0x080 },
	{ "decon0.wincon", 0x194a0000, 0x000, 0x080 },
	{ "dpp0.dpp", 0x19930000, 0x000, 0x0c0 },
	{ "decon0.main", 0x19470000, 0x400, 0x440 },
	{ "dpp0.rdma", 0x19900000, 0x400, 0x4a0 },
	/* not the SysMMU in front of DPUF0 (0x19840000): reading it restarts the phone (secure, or closed to the kernel); DPP0
	 * reads physical addresses as the bootloader left it (RDMA_BASEADDR_P0 = 0xfac00000), so it is bypassed */
};
static uint32_t probe_blocks = sizeof(blocks) / sizeof(blocks[0]);

/*
 * The first try (all the blocks 5 s after the start) left the phone restarting: one read faulted, and its log was lost
 * with it. So now: late (display_probe_delay= seconds, 60, when the log reaches the computer over USB), each block named
 * before it is read, a second between blocks, and display_probe=N reads only the first N blocks.
 */
static void
akita_display_probe(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	for (unsigned b = 0; b < probe_blocks; b++) {
		printf("akita_display: reading %s at %#llx (+%#x)\n", blocks[b].name, (unsigned long long)blocks[b].phys,
		    blocks[b].from);
		delay(1000000);
		vm_offset_t base = ml_io_map(blocks[b].phys + (blocks[b].from & ~0xfffu), 0x1000);
		uint32_t at = blocks[b].from & 0xfff;

		for (uint32_t off = blocks[b].from; off < blocks[b].to; off += 16, at += 16) {
			volatile uint32_t *r = (volatile uint32_t *)(base + at);

			printf("akita_display: %-13s %#llx: %08x %08x %08x %08x\n", blocks[b].name,
			    (unsigned long long)(blocks[b].phys + off), r[0], r[1], r[2], r[3]);
		}
	}
	printf("akita_display: done\n");
}

/* the two buffers: the bootloader's, and RAM nobody uses after it (zuma-akita.dts: free between abl@f8800000 and
 * dss_log_reserved@fd3f0000; the loader puts XNU elsewhere, at 0x9_xxxxxxxx); the DMA's addresses are 32-bit */
#define FB_FRONT        0xfac00000ULL
#define FB_BACK         0xfb800000ULL
#define FB_WIDTH        1080
#define FB_HEIGHT       2400
#define FB_BYTES        (FB_WIDTH * FB_HEIGHT * 4)
#define DECON0          0x19470000ULL
#define DPP0_RDMA       0x19900000ULL
#define TRIG_CON        0x030
#define RDMA_ENABLE     0x000
#define RDMA_BASEADDR   0x040
#define RDMA_SHADOW     0x400
#define SFR_UPDATE_FORCE (1u << 4)

extern vm_map_address_t pmap_map(vm_map_address_t virt, vm_offset_t start, vm_offset_t end, vm_prot_t prot, unsigned int flags);
extern uint64_t mach_absolute_time(void);
extern void absolutetime_to_nanoseconds(uint64_t abstime, uint64_t *result);

static volatile uint32_t *decon, *rdma;

/*
 * A buffer's 10 MB mapped write-combined: not by ml_io_map, whose whole area is 8 MB (io_map.c's IO_MAP_SIZE, allocated
 * KMA_NOFAIL -- the first try of the flip ended the kernel there), but kernel addresses of its own and pmap_map page by page.
 */
static volatile uint32_t *
map_buffer(uint64_t phys, vm_size_t size)
{
	vm_offset_t va = 0;

	size = round_page(size);
	if (kmem_alloc(kernel_map, &va, size, KMA_VAONLY | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return NULL;
	}
	pmap_map(va, (vm_offset_t)phys, (vm_offset_t)(phys + size), VM_PROT_READ | VM_PROT_WRITE, VM_WIMG_WCOMB);
	return (volatile uint32_t *)va;
}

static uint64_t
micros(uint64_t since)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time() - since, &ns);
	return ns / 1000;
}

/*
 * Triple buffering for MiniIOMobileFramebuffer (display_flip=1; display_buffers=2 for double): the panel shows one
 * buffer, one may be queued, the framebuffer driver draws into the third and never waits. DECON0 sends a frame at every
 * TE (60 Hz); DPP0 says when it has read one out (RDMA_IRQ's frame done, bit 16, cleared by writing it). A thread of
 * this driver's waits for a queued buffer, clears frame done, waits for the next, and switches then -- the rest of the
 * refresh is left before the next TE starts a read -- by the base address and IDMA_SFR_UPDATE_FORCE (DECON0's update
 * request does not carry DPP0's address: seen on the phone). A buffer queued while another still waits replaces it (the
 * newest frame is the one shown; the other is counted as dropped). With two buffers, flip() waits until it is shown.
 * No frame done in 50 ms: switched anyway, counted.
 */
#define RDMA_IRQ        0x004
#define FRAME_DONE      (1u << 16)
#define NO_BUFFER       (~0u)

static const uint64_t buffer_phys[3] = { FB_FRONT, FB_BACK, FB_BACK + 0xa00000 };     /* the third: 0xfc200000 */
static uint32_t nbuffers = 3;
static volatile uint32_t shown_index, queued = NO_BUFFER;
static uint32_t flips, flips_late, flips_dropped;
static uint64_t flip_wait_us;
static lck_grp_t *flip_group;
static lck_mtx_t *flip_lock;

static void flipper(void *arg, wait_result_t wr);

static int
display_buffers(vm_offset_t va[3], uint32_t *count, uint32_t *width, uint32_t *height, uint32_t *row_bytes)
{
	decon = (volatile uint32_t *)ml_io_map(DECON0, 0x1000);
	rdma = (volatile uint32_t *)ml_io_map(DPP0_RDMA, 0x1000);
	if (rdma[RDMA_BASEADDR / 4] != (uint32_t)FB_FRONT || decon[TRIG_CON / 4] != 0x3061 ||
	    rdma[0x1c / 4] != (FB_HEIGHT << 16 | FB_WIDTH)) {
		printf("akita_display: no flips: the pipeline is not as the bootloader leaves it (RDMA_BASEADDR_P0 %#x, "
		    "TRIG_CON %#x, RDMA_IMG_SIZE %#x)\n", rdma[RDMA_BASEADDR / 4], decon[TRIG_CON / 4], rdma[0x1c / 4]);
		return -1;
	}
	for (uint32_t i = 0; i < nbuffers; i++) {
		if ((va[i] = (vm_offset_t)map_buffer(buffer_phys[i], FB_BYTES)) == 0) {
			return -1;
		}
	}
	/* (the thread that flips: started here, from IOKit, not at EARLY_BOOT) */
	thread_t thread;
	flip_group = lck_grp_alloc_init("akita_display", LCK_GRP_ATTR_NULL);
	flip_lock = lck_mtx_alloc_init(flip_group, LCK_ATTR_NULL);
	if (kernel_thread_start(flipper, NULL, &thread) != KERN_SUCCESS) {
		printf("akita_display: no flips: the flipping thread could not start\n");
		return -1;
	}
	thread_set_thread_name(thread, "akita_display: flips");
	thread_deallocate(thread);
	*count = nbuffers;
	*width = FB_WIDTH;
	*height = FB_HEIGHT;
	*row_bytes = FB_WIDTH * 4;
	printf("akita_display: %u buffers from %#llx, flipped by DPP0 between the panel's frames\n", nbuffers, FB_FRONT);
	return 0;
}

static uint32_t
display_shown(void)
{
	return shown_index;
}

static void
display_flip(unsigned index)
{
	lck_mtx_lock(flip_lock);
	if (queued != NO_BUFFER) {
		flips_dropped++;
	}
	queued = index;
	thread_wakeup((event_t)&queued);
	if (nbuffers == 2) {
		while (shown_index != index) {
			assert_wait((event_t)&shown_index, THREAD_UNINT);
			lck_mtx_unlock(flip_lock);
			thread_block(THREAD_CONTINUE_NULL);
			lck_mtx_lock(flip_lock);
		}
	}
	lck_mtx_unlock(flip_lock);
}

static void
flipper(__unused void *arg, __unused wait_result_t wr)
{
	for (;;) {
		uint64_t t0;
		uint32_t index;

		lck_mtx_lock(flip_lock);
		while (queued == NO_BUFFER) {
			assert_wait((event_t)&queued, THREAD_UNINT);
			lck_mtx_unlock(flip_lock);
			thread_block(THREAD_CONTINUE_NULL);
			lck_mtx_lock(flip_lock);
		}
		lck_mtx_unlock(flip_lock);
		t0 = mach_absolute_time();
		__asm__ volatile("dsb sy" ::: "memory");        /* (the frame's last writes out of the write buffers) */
		rdma[RDMA_IRQ / 4] = FRAME_DONE;
		while ((rdma[RDMA_IRQ / 4] & FRAME_DONE) == 0) {
			if (micros(t0) > 50000) {
				flips_late++;
				break;
			}
			delay(100);
		}
		lck_mtx_lock(flip_lock);
		index = queued;                                 /* (the newest, if another came while waiting) */
		queued = NO_BUFFER;
		rdma[RDMA_BASEADDR / 4] = (uint32_t)buffer_phys[index];
		__asm__ volatile("dsb sy" ::: "memory");
		rdma[RDMA_ENABLE / 4] |= SFR_UPDATE_FORCE;
		shown_index = index;
		thread_wakeup((event_t)&shown_index);
		lck_mtx_unlock(flip_lock);
		flip_wait_us += micros(t0);
		if (++flips % 600 == 0) {
			printf("akita_display: %u flips, %llu us from queued to shown on average, %u frames replaced by a newer "
			    "before being shown, %u flips without a frame done in 50 ms\n", flips, flip_wait_us / 600, flips_dropped,
			    flips_late);
			flip_wait_us = 0;
		}
	}
}

struct hakios_display_ops {
	int (*buffers)(vm_offset_t va[3], uint32_t *count, uint32_t *width, uint32_t *height, uint32_t *row_bytes);
	void (*flip)(unsigned index);
	uint32_t (*shown)(void);
};
extern const struct hakios_display_ops *hakios_display_ops;     /* MiniIOMobileFramebuffer's */
static const struct hakios_display_ops akita_display_ops = { display_buffers, display_flip, display_shown };

__startup_func
static void
akita_display_init(void)
{
	uint32_t enabled = 0, seconds = 60;
	uint64_t deadline;
	static thread_call_t probe;

	if (PE_parse_boot_argn("display_flip", &enabled, sizeof(enabled)) && enabled) {
		if (PE_parse_boot_argn("display_buffers", &seconds, sizeof(seconds)) && seconds == 2) {
			nbuffers = 2;
		}
		seconds = 60;
		hakios_display_ops = &akita_display_ops;
	}
	enabled = 0;
	if (!PE_parse_boot_argn("display_probe", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	if (enabled < probe_blocks) {
		probe_blocks = enabled;
	}
	PE_parse_boot_argn("display_probe_delay", &seconds, sizeof(seconds));
	probe = thread_call_allocate(akita_display_probe, NULL);
	clock_interval_to_deadline(seconds, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(probe, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_display_init);
