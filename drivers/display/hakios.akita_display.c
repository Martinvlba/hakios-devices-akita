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

/*
 * The flip (display_flip=1, display_flip_delay= seconds, 60): DPP0 shown a second buffer for 5 s, then the first again
 * -- what page flips will do for every frame. The second buffer is RAM nobody uses, after the bootloader's (zuma-akita.dts:
 * free between abl@f8800000 and dss_log_reserved@fd3f0000; the loader puts XNU elsewhere): the current picture copied in
 * with a band across its top, so the flip can be seen. A flip, as Google's driver does it (cal_9865): the new address
 * into RDMA_BASEADDR_P0, then an update request for window 0 and the global one in DECON0's SHD_REG_UP_REQ; the hardware
 * takes it at the next frame start (DSI command mode, TE-triggered), and its shadow copy (+0x400) says when. If it does
 * not, IDMA_SFR_UPDATE_FORCE (RDMA_ENABLE bit 4) is tried, and the log says which did it.
 */
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

/* the shadow copy at ADDR within 200 ms: how long it took, or -1 */
static int64_t
latched(uint32_t addr)
{
	uint64_t t0 = mach_absolute_time();

	while (micros(t0) < 200000) {
		if (rdma[(RDMA_SHADOW + RDMA_BASEADDR) / 4] == addr) {
			return (int64_t)micros(t0);
		}
		delay(50);
	}
	return -1;
}

static int
flip(uint32_t addr)
{
	int64_t us;

	rdma[RDMA_BASEADDR / 4] = addr;
	__asm__ volatile("dsb sy" ::: "memory");
	decon[SHD_REG_UP_REQ / 4] = UP_REQ_WIN0 | UP_REQ_GLOBAL;
	if ((us = latched(addr)) >= 0) {
		printf("akita_display: flip to %#x by the update request: taken after %lld us (SHD_REG_UP_REQ now %#x)\n", addr,
		    (long long)us, decon[SHD_REG_UP_REQ / 4]);
		return 0;
	}
	printf("akita_display: flip to %#x: the update request was not taken in 200 ms (SHD_REG_UP_REQ %#x, shadow %#x); "
	    "forcing\n", addr, decon[SHD_REG_UP_REQ / 4], rdma[(RDMA_SHADOW + RDMA_BASEADDR) / 4]);
	rdma[RDMA_ENABLE / 4] |= SFR_UPDATE_FORCE;
	if ((us = latched(addr)) >= 0) {
		printf("akita_display: flip to %#x by IDMA_SFR_UPDATE_FORCE: taken after %lld us\n", addr, (long long)us);
		return 0;
	}
	printf("akita_display: flip to %#x: not taken at all (shadow %#x)\n", addr, rdma[(RDMA_SHADOW + RDMA_BASEADDR) / 4]);
	return -1;
}

static void
akita_display_flip(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	volatile uint32_t *front, *back;

	decon = (volatile uint32_t *)ml_io_map(DECON0, 0x1000);
	rdma = (volatile uint32_t *)ml_io_map(DPP0_RDMA, 0x1000);
	if (rdma[RDMA_BASEADDR / 4] != (uint32_t)FB_FRONT || decon[TRIG_CON / 4] != 0x3061) {
		printf("akita_display: flip: not as the bootloader leaves it (RDMA_BASEADDR_P0 %#x, TRIG_CON %#x): not tried\n",
		    rdma[RDMA_BASEADDR / 4], decon[TRIG_CON / 4]);
		return;
	}
	/* (each step said, and a moment to reach the computer: the first run of this ended the kernel between them) */
	printf("akita_display: flip: mapping the front buffer %#llx\n", FB_FRONT);
	delay(500000);
	front = map_buffer(FB_FRONT, FB_BYTES);
	if (front == NULL) {
		printf("akita_display: flip: no kernel addresses for the front buffer\n");
		return;
	}
	printf("akita_display: flip: front at %p, first word %#x; mapping the back buffer %#llx\n", (void *)front, front[0], FB_BACK);
	delay(500000);
	back = map_buffer(FB_BACK, FB_BYTES);
	if (back == NULL) {
		printf("akita_display: flip: no kernel addresses for the back buffer\n");
		return;
	}
	printf("akita_display: flip: back at %p; reading its first word\n", (void *)back);
	delay(500000);
	printf("akita_display: flip: back's first word %#x; writing it\n", back[0]);
	delay(500000);
	back[0] = 0xffff00ffu;
	__asm__ volatile("dsb sy" ::: "memory");
	printf("akita_display: flip: written (%#x); copying the picture\n", back[0]);
	delay(500000);
	for (unsigned i = 0; i < FB_WIDTH * FB_HEIGHT; i++) {
		back[i] = i < FB_WIDTH * 240 ? 0xffff00ffu : front[i];     /* magenta across the top 240 rows */
	}
	__asm__ volatile("dsb sy" ::: "memory");
	printf("akita_display: flip: a second buffer at %#llx (the picture with a magenta band); RDMA_ENABLE %#x\n",
	    FB_BACK, rdma[RDMA_ENABLE / 4]);
	if (flip((uint32_t)FB_BACK) == 0) {
		delay(5000000);
	}
	flip((uint32_t)FB_FRONT);
}

__startup_func
static void
akita_display_init(void)
{
	uint32_t enabled = 0, seconds = 60;
	uint64_t deadline;
	static thread_call_t probe, flipper;

	if (PE_parse_boot_argn("display_flip", &enabled, sizeof(enabled)) && enabled) {
		PE_parse_boot_argn("display_flip_delay", &seconds, sizeof(seconds));
		flipper = thread_call_allocate(akita_display_flip, NULL);
		clock_interval_to_deadline(seconds, NSEC_PER_SEC, &deadline);
		thread_call_enter_delayed(flipper, deadline);
		seconds = 60;
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
