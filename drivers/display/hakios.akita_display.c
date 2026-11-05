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
#define SHD_REG_UP_REQ  0x050
#define RDMA_ENABLE     0x000
#define RDMA_BASEADDR   0x040
#define RDMA_SHADOW     0x400
#define UP_REQ_GLOBAL   (1u << 31)
#define UP_REQ_WIN0     (1u << 0)
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
 * Double buffering for MiniIOMobileFramebuffer (display_flip=1): the two buffers, and a flip that is taken between two
 * of the panel's frames. DECON0 sends a frame at every TE (60 Hz); DPP0 says when it has read one out (RDMA_IRQ's
 * frame done, bit 16, cleared by writing it). The flip clears it, waits for the next, and switches then -- the rest of
 * the refresh is left before the next TE starts a read -- by the base address and IDMA_SFR_UPDATE_FORCE (DECON0's
 * update request does not carry DPP0's address: seen on the phone). No frame done in 50 ms: switched anyway, counted.
 */
#define RDMA_IRQ        0x004
#define FRAME_DONE      (1u << 16)

static const uint64_t buffer_phys[2] = { FB_FRONT, FB_BACK };
static uint32_t flips, flips_late;
static uint64_t flip_wait_us;

static int
display_buffers(vm_offset_t va[2], uint32_t *width, uint32_t *height, uint32_t *row_bytes)
{
	decon = (volatile uint32_t *)ml_io_map(DECON0, 0x1000);
	rdma = (volatile uint32_t *)ml_io_map(DPP0_RDMA, 0x1000);
	if (rdma[RDMA_BASEADDR / 4] != (uint32_t)FB_FRONT || decon[TRIG_CON / 4] != 0x3061 ||
	    rdma[0x1c / 4] != (FB_HEIGHT << 16 | FB_WIDTH)) {
		printf("akita_display: no double buffering: the pipeline is not as the bootloader leaves it (RDMA_BASEADDR_P0 "
		    "%#x, TRIG_CON %#x, RDMA_IMG_SIZE %#x)\n", rdma[RDMA_BASEADDR / 4], decon[TRIG_CON / 4], rdma[0x1c / 4]);
		return -1;
	}
	for (int i = 0; i < 2; i++) {
		if ((va[i] = (vm_offset_t)map_buffer(buffer_phys[i], FB_BYTES)) == 0) {
			return -1;
		}
	}
	*width = FB_WIDTH;
	*height = FB_HEIGHT;
	*row_bytes = FB_WIDTH * 4;
	printf("akita_display: double buffering: %#llx and %#llx, flipped by DPP0 between the panel's frames\n", FB_FRONT, FB_BACK);
	return 0;
}

static void
display_flip(unsigned index)
{
	uint64_t t0 = mach_absolute_time(), us;

	__asm__ volatile("dsb sy" ::: "memory");        /* (the frame's last writes out of the write buffers) */
	rdma[RDMA_IRQ / 4] = FRAME_DONE;
	while ((rdma[RDMA_IRQ / 4] & FRAME_DONE) == 0) {
		if (micros(t0) > 50000) {
			flips_late++;
			break;
		}
		delay(20);
	}
	rdma[RDMA_BASEADDR / 4] = (uint32_t)buffer_phys[index & 1];
	__asm__ volatile("dsb sy" ::: "memory");
	rdma[RDMA_ENABLE / 4] |= SFR_UPDATE_FORCE;
	us = micros(t0);
	flip_wait_us += us;
	if (++flips % 600 == 0) {
		printf("akita_display: %u flips, %llu us waited for the panel's frame on average, %u without one in 50 ms\n",
		    flips, flip_wait_us / 600, flips_late);
		flip_wait_us = 0;
	}
}

struct hakios_display_ops {
	int (*buffers)(vm_offset_t va[2], uint32_t *width, uint32_t *height, uint32_t *row_bytes);
	void (*flip)(unsigned index);
};
extern const struct hakios_display_ops *hakios_display_ops;     /* MiniIOMobileFramebuffer's */
static const struct hakios_display_ops akita_display_ops = { display_buffers, display_flip };

__startup_func
static void
akita_display_init(void)
{
	uint32_t enabled = 0, seconds = 60;
	uint64_t deadline;
	static thread_call_t probe;

	if (PE_parse_boot_argn("display_flip", &enabled, sizeof(enabled)) && enabled) {
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
