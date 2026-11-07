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
static void panel_set_hz(uint32_t hz);
static uint32_t display_refresh_hz(void);
static uint32_t measure_hz(void);
static volatile uint32_t vblank_caught_hz;        /* frame dones the vblank thread caught in the last second */
static volatile uint64_t vblank_at;
static volatile uint32_t vblank_hz;
static volatile uint64_t vblank_period;         /* the refresh's length (mach_absolute_time units): the vblank thread's sleep */
static volatile bool vblank_paused;             /* measure_hz has the frame-done bit to itself */

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
	thread_set_thread_name(thread, "akita_display: vblank");
	thread_deallocate(thread);
	{
		/* the panel's rate measured (a second and a bit), and display_hz= set before the framebuffer tells
		 * CoreAnimation its refresh */
		uint32_t hz = 0;

		if (PE_parse_boot_argn("display_hz", &hz, sizeof(hz)) && (hz == 60 || hz == 120)) {
			panel_set_hz(hz);
		} else {
			measure_hz();
		}
		printf("akita_display: the panel refreshes at %u Hz (counted %u)\n", display_refresh_hz(), vblank_hz);
	}
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

/*
 * The panel's beat (the vblank thread, in place of a thread that woke only to flip): DPP0's frame done at every refresh
 * -- DECON0 sends a frame at each of the panel's TE, 60 or 120 a second (display_hz=) -- each one timestamped, the rate
 * measured over the last second, and a queued buffer switched to right there, in the rest of the refresh before the
 * next TE starts a read. No interrupt is wired: the thread sleeps until just before the frame done it expects (the last
 * one plus a period, less 1.5 ms) and then looks every 50 us; a frame done missed for 50 ms is counted, and a queued
 * buffer is switched anyway.
 */
/* vblank_at: mach_absolute_time of the last frame done; vblank_hz: refreshes in the last second (declared above) */
static uint64_t vblank_count;

static void
flipper(__unused void *arg, __unused wait_result_t wr)
{
	uint64_t second_start = mach_absolute_time(), period_abs, margin_abs, window_abs;
	uint32_t in_second = 0;

	nanoseconds_to_absolutetime(NSEC_PER_SEC / 60, &period_abs);
	vblank_period = period_abs;
	nanoseconds_to_absolutetime(700 * NSEC_PER_USEC, &margin_abs);
	nanoseconds_to_absolutetime(50 * NSEC_PER_MSEC, &window_abs);
	rdma[RDMA_IRQ / 4] = FRAME_DONE;
	for (;;) {
		uint64_t now = mach_absolute_time(), t0;
		uint32_t index;
		bool seen = false;

		/* asleep until a quarter of a refresh (1.5 ms at least) before the frame done that is due; the period is
		 * measure_hz's, not guessed from the gaps seen here */
		while (vblank_paused) {
			assert_wait_timeout((event_t)&vblank_paused, THREAD_UNINT, 5, NSEC_PER_MSEC);
			thread_block(THREAD_CONTINUE_NULL);
			now = mach_absolute_time();
		}
		period_abs = vblank_period;
		margin_abs = period_abs / 4;
		{
			uint64_t least;
			nanoseconds_to_absolutetime(1500 * NSEC_PER_USEC, &least);
			if (margin_abs < least) margin_abs = least;
		}
		if (vblank_at != 0 && vblank_at + period_abs > now + margin_abs) {
			assert_wait_deadline((event_t)&vblank_at, THREAD_UNINT, vblank_at + period_abs - margin_abs);
			thread_block(THREAD_CONTINUE_NULL);
		}
		t0 = mach_absolute_time();
		while (mach_absolute_time() - t0 < window_abs) {
			if (rdma[RDMA_IRQ / 4] & FRAME_DONE) {
				seen = true;
				break;
			}
			delay(50);
		}
		now = mach_absolute_time();
		rdma[RDMA_IRQ / 4] = FRAME_DONE;
		if (seen) {
			vblank_at = now;
			vblank_count++;
			in_second++;
		} else {
			flips_late += queued != NO_BUFFER;
		}
		{
			uint64_t ns;
			absolutetime_to_nanoseconds(now - second_start, &ns);
			if (ns >= NSEC_PER_SEC) {
				vblank_caught_hz = (uint32_t)((uint64_t)in_second * NSEC_PER_SEC / ns);
				second_start = now;
				in_second = 0;
			}
		}
		lck_mtx_lock(flip_lock);
		if (queued == NO_BUFFER) {
			lck_mtx_unlock(flip_lock);
			continue;
		}
		__asm__ volatile("dsb sy" ::: "memory");        /* (the frame's last writes out of the write buffers) */
		index = queued;                                 /* (the newest, if another came while waiting) */
		queued = NO_BUFFER;
		rdma[RDMA_BASEADDR / 4] = (uint32_t)buffer_phys[index];
		__asm__ volatile("dsb sy" ::: "memory");
		rdma[RDMA_ENABLE / 4] |= SFR_UPDATE_FORCE;
		shown_index = index;
		thread_wakeup((event_t)&shown_index);
		lck_mtx_unlock(flip_lock);
		if (++flips % 600 == 0) {
			printf("akita_display: %u flips; the panel at %u Hz, %u frame dones caught in the last second (%llu in all); %u "
			    "frames replaced by a newer before being shown, %u flips without a frame done in 50 ms\n", flips, vblank_hz,
			    vblank_caught_hz, vblank_count, flips_dropped, flips_late);
		}
	}
}

/* the panel's rate as its mode's (60 or 120) when the count is near it: a refresh missed now and then is the polling's */
static uint32_t
display_refresh_hz(void)
{
	uint32_t hz = vblank_hz;

	return hz >= 112 && hz <= 128 ? 120 : hz >= 55 && hz <= 65 ? 60 : hz;
}
static uint64_t display_vblank_time(void) { return vblank_at; }

/*
 * The panel's commands (display_hz=120 or 60): MIPI DCS writes through DSIM0 (drmdsim@19440000 in the phone's tree), as
 * Google's exynos_drm_dsim.c sends one without batching -- the payload's words into DSIM_PAYLOAD, then the header into
 * DSIM_PKTHDR (data type 0x05 or 0x15 for one or two bytes, 0x39 and the length for more), and the FIFOs waited for.
 * The sequence for a rate is Google's panel driver's for this panel (kernel/devices/google/akita display/
 * panel-google-ak3b.c, ak3b_change_frequency): the test key, 60 00 00 (HS 120) or 60 08 00 (HS 60), B9 31 or 30 (the
 * TE), F7 0F (the update), the test key off. In command mode the panel's TE then comes at that rate, and DECON0's
 * hardware trigger with it: nothing else changes on this side.
 */
#define DSIM0           0x19440000ULL
#define DSIM_PKTHDR     0x058
#define DSIM_PAYLOAD    0x05c
#define DSIM_FIFOCTRL   0x068
#define DSIM_CMD_CONFIG 0x080
#define FIFO_PH_EMPTY   (1u << 10)
#define FIFO_PL_EMPTY   (1u << 8)
#define PKT_GO_EN       (1u << 16)
#define PKT_GO_RDY      (1u << 17)

static volatile uint32_t *dsim;

static bool
dsim_fifo_empty(uint32_t which)
{
	uint64_t t0 = mach_absolute_time();

	while ((dsim[DSIM_FIFOCTRL / 4] & which) != which) {
		if (micros(t0) > 20000) {
			return false;
		}
		delay(20);
	}
	return true;
}

static bool
dsim_dcs(const uint8_t *b, uint32_t n)
{
	if (!dsim_fifo_empty(FIFO_PH_EMPTY | FIFO_PL_EMPTY)) {
		return false;
	}
	if (n == 1) {
		dsim[DSIM_PKTHDR / 4] = 0x05 | (uint32_t)b[0] << 8;
	} else if (n == 2) {
		dsim[DSIM_PKTHDR / 4] = 0x15 | (uint32_t)b[0] << 8 | (uint32_t)b[1] << 16;
	} else {
		for (uint32_t i = 0; i < n; i += 4) {
			uint32_t w = 0;
			for (uint32_t k = 0; k < 4 && i + k < n; k++) {
				w |= (uint32_t)b[i + k] << (8 * k);
			}
			dsim[DSIM_PAYLOAD / 4] = w;
		}
		dsim[DSIM_PKTHDR / 4] = 0x39 | (n & 0xff) << 8 | (n >> 8 & 0xff) << 16;
	}
	if (dsim[DSIM_CMD_CONFIG / 4] & PKT_GO_EN) {
		dsim[DSIM_CMD_CONFIG / 4] |= PKT_GO_RDY;
	}
	return dsim_fifo_empty(FIFO_PH_EMPTY | FIFO_PL_EMPTY);
}

/*
 * The panel's true rate: the vblank thread paused, frame done's edges counted for half a second by polling every 20 us
 * (no sleeping in between, so none is missed); the period set from it for the vblank thread
 */
static uint32_t
measure_hz(void)
{
	uint64_t t0, first = 0, last = 0;
	uint32_t edges = 0;

	vblank_paused = true;
	delay(20000);                           /* (the thread out of its poll) */
	rdma[RDMA_IRQ / 4] = FRAME_DONE;
	t0 = mach_absolute_time();
	while (micros(t0) < 500000) {
		if (rdma[RDMA_IRQ / 4] & FRAME_DONE) {
			uint64_t now = mach_absolute_time();
			rdma[RDMA_IRQ / 4] = FRAME_DONE;
			if (edges++ == 0) first = now;
			last = now;
		}
		delay(20);
	}
	if (edges >= 2) {
		vblank_period = (last - first) / (edges - 1);
		vblank_at = last;
	}
	vblank_paused = false;
	thread_wakeup((event_t)&vblank_paused);
	{
		uint64_t ns;
		absolutetime_to_nanoseconds(vblank_period, &ns);
		vblank_hz = ns ? (uint32_t)((NSEC_PER_SEC + ns / 2) / ns) : 0;
	}
	return vblank_hz;
}

static void
panel_set_hz(uint32_t hz)
{
	const uint8_t key_on[] = { 0xf0, 0x5a, 0x5a }, key_off[] = { 0xf0, 0xa5, 0xa5 }, update[] = { 0xf7, 0x0f };
	const uint8_t hs120[] = { 0x60, 0x00, 0x00 }, hs60[] = { 0x60, 0x08, 0x00 };
	const uint8_t te120[] = { 0xb9, 0x31 }, te60[] = { 0xb9, 0x30 };
	uint32_t before = measure_hz();
	bool ok;

	dsim = (volatile uint32_t *)ml_io_map(DSIM0, 0x1000);
	printf("akita_display: the panel to %u Hz (it refreshes at %u now); DSIM0 CMD_CONFIG %#x FIFOCTRL %#x\n", hz, before,
	    dsim[DSIM_CMD_CONFIG / 4], dsim[DSIM_FIFOCTRL / 4]);
	/* right after a frame done: the rest of the refresh is free for commands */
	{
		uint64_t at = vblank_at, t0 = mach_absolute_time();
		while (vblank_at == at && micros(t0) < 50000) {
			delay(100);
		}
	}
	ok = dsim_dcs(key_on, sizeof(key_on)) && dsim_dcs(hz == 120 ? hs120 : hs60, 3) &&
	    dsim_dcs(hz == 120 ? te120 : te60, 2) && dsim_dcs(update, sizeof(update)) && dsim_dcs(key_off, sizeof(key_off));
	/* the new rate measured afresh */
	delay(200000);
	measure_hz();
	printf("akita_display: the panel's commands %s; it refreshes at %u Hz now (FIFOCTRL %#x)\n", ok ? "sent" : "NOT all sent",
	    vblank_hz, dsim[DSIM_FIFOCTRL / 4]);
}

struct hakios_display_ops {
	int (*buffers)(vm_offset_t va[3], uint32_t *count, uint32_t *width, uint32_t *height, uint32_t *row_bytes);
	void (*flip)(unsigned index);
	uint32_t (*shown)(void);
	uint32_t (*refresh_hz)(void);
	uint64_t (*vblank_time)(void);
};
extern const struct hakios_display_ops *hakios_display_ops;     /* MiniIOMobileFramebuffer's */
static const struct hakios_display_ops akita_display_ops = { display_buffers, display_flip, display_shown, display_refresh_hz,
	                                                     display_vblank_time };

/*
 * The DPP's scaler, tried on its own (display_scale=1, without display_flip; display_scale_delay= seconds, 60): DPP0 made
 * to read a 1125 x 2436 picture -- the iPhone X's, what iOS draws -- and scale it to the panel's 1080 x 2400, for 8 s,
 * then the bootloader's setting back. The picture: the screen copied in, a red border around all of 1125 x 2436 and
 * green lines where the panel's edges fell (x 1080, y 2400): scaled right, the whole red border is on the panel.
 * Registers (Google's names, cal_9865): RDMA_SRC_WIDTH/HEIGHT (+0x10/0x14), RDMA_IMG_SIZE (+0x1c); DPP_COM_IMG_SIZE
 * (+0x3c), DPP_COM_SCL_CTRL (+0x80: enable, coefficient set 0), SCALED_IMG_SIZE (+0x84), H/V_RATIO (+0x88/0x8c: source
 * over destination, 20 fraction bits); the filter's coefficients in the scl_coef block (0x19940000): 9 phases of 8 taps
 * across, 4 down, 11-bit signed, each phase summing to 512 -- here Catmull-Rom's cubic, worked out below (ours: Google's
 * tables are not taken). The log says what the DPP took (its shadow registers, +0x100) and its configuration errors.
 *
 * Tried on the phone (11-05): it does NOT work and leaves the screen striped until a restart. DPP0's DMA takes the new
 * size and address, but DPP0's own block (DPP_COM: sizes, scaler, ratios) takes nothing -- neither with
 * IDMA_SFR_UPDATE_FORCE nor with DECON0's update request -- so 1125-pixel lines reach a stage set for 1080 and the DMA
 * reports a deadlock (RDMA_IRQ bit 17). DPP_COM_TZPC (0x19930034) reads 1: DPP0's block is secure, its registers are
 * not the kernel's to write. Scaling in hardware would need another DPP channel (and DECON0's window moved to it).
 */
#define DPP0_COM        0x19930000ULL
#define DPP0_COEF       0x19940000ULL
#define SRC_W           1125
#define SRC_H           2436
#define SRC_BYTES       (SRC_W * SRC_H * 4)
#define DPP_IMG_SIZE    0x03c
#define DPP_SCL_CTRL    0x080
#define DPP_SCALED_SIZE 0x084
#define DPP_H_RATIO     0x088
#define DPP_V_RATIO     0x08c
#define DPP_CFG_ERROR   0x01c
#define DPP_SHADOW      0x100
#define RDMA_SRC_WIDTH  0x010
#define RDMA_SRC_HEIGHT 0x014
#define RDMA_IMG_SIZE   0x01c
#define RDMA_IRQ_REG    0x004
#define RDMA_DONE       (1u << 16)

static volatile uint32_t *dpp, *coef;

/* Catmull-Rom's four weights at phase P (0..8 of 16), as 512ths that sum to 512 */
static void
cubic(unsigned phase, int w[4])
{
	/* in 1/4096ths of a pixel: t = phase/16 */
	int64_t t = phase * 256, t2 = t * t / 4096, t3 = t2 * t / 4096;
	int64_t a = (-t3 + 2 * t2 - t) / 2, c = (-3 * t3 + 4 * t2 + t) / 2, d = (t3 - t2) / 2;      /* (the pixel's own: what is left of 512) */

	w[0] = (int)(a * 512 / 4096);
	w[2] = (int)(c * 512 / 4096);
	w[3] = (int)(d * 512 / 4096);
	w[1] = 512 - w[0] - w[2] - w[3];
}

static void
write_coefficients(void)
{
	for (unsigned n = 0; n < 9; n++) {
		int w[4];

		cubic(n, w);
		for (unsigned s = 0; s < 8; s++) {              /* across: 8 taps, the pixel at tap 3 */
			int v = s >= 2 && s <= 5 ? w[s - 2] : 0;

			coef[(0x90 + n * 4 + s * 0x24) / 4] = (uint32_t)v & 0x7ff;
		}
		for (unsigned s = 0; s < 4; s++) {              /* down: 4 taps, the pixel at tap 1 */
			coef[(n * 4 + s * 0x24) / 4] = (uint32_t)w[s] & 0x7ff;
		}
	}
}

static void
wait_frame_done(void)
{
	uint64_t t0 = mach_absolute_time();

	rdma[RDMA_IRQ_REG / 4] = RDMA_DONE;
	while ((rdma[RDMA_IRQ_REG / 4] & RDMA_DONE) == 0 && micros(t0) < 50000) {
		delay(50);
	}
}

static void
say_dpp(const char *when)
{
	printf("akita_display: scale %s: DPP shadow IMG_SIZE %#x SCL_CTRL %#x SCALED %#x RATIO %#x/%#x, CFG_ERROR %#x, DPP IRQ %#x; "
	    "RDMA shadow SRC %u x %u IMG %#x base %#x, RDMA_IRQ %#x\n", when, dpp[(DPP_SHADOW + DPP_IMG_SIZE) / 4],
	    dpp[(DPP_SHADOW + DPP_SCL_CTRL) / 4], dpp[(DPP_SHADOW + DPP_SCALED_SIZE) / 4], dpp[(DPP_SHADOW + DPP_H_RATIO) / 4],
	    dpp[(DPP_SHADOW + DPP_V_RATIO) / 4], dpp[DPP_CFG_ERROR / 4], dpp[0x18 / 4], rdma[(RDMA_SHADOW + RDMA_SRC_WIDTH) / 4],
	    rdma[(RDMA_SHADOW + RDMA_SRC_HEIGHT) / 4], rdma[(RDMA_SHADOW + RDMA_IMG_SIZE) / 4],
	    rdma[(RDMA_SHADOW + RDMA_BASEADDR) / 4], rdma[RDMA_IRQ_REG / 4]);
}

/* the pipeline set for a source of W x H at BASE, scaled to the panel when it is not the panel's size */
static void
set_source(uint32_t w, uint32_t h, uint32_t base)
{
	bool scaled = w != FB_WIDTH || h != FB_HEIGHT;

	wait_frame_done();
	rdma[RDMA_SRC_WIDTH / 4] = w;
	rdma[RDMA_SRC_HEIGHT / 4] = h;
	rdma[RDMA_IMG_SIZE / 4] = h << 16 | w;
	rdma[RDMA_BASEADDR / 4] = base;
	dpp[DPP_IMG_SIZE / 4] = h << 16 | w;
	dpp[DPP_SCALED_SIZE / 4] = scaled ? (uint32_t)(FB_HEIGHT << 16 | FB_WIDTH) : 0;
	dpp[DPP_H_RATIO / 4] = (uint32_t)(((uint64_t)w << 20) / FB_WIDTH);
	dpp[DPP_V_RATIO / 4] = (uint32_t)(((uint64_t)h << 20) / FB_HEIGHT);
	dpp[DPP_SCL_CTRL / 4] = scaled ? 1 : 0;
	__asm__ volatile("dsb sy" ::: "memory");
	rdma[RDMA_ENABLE / 4] |= SFR_UPDATE_FORCE;
}

static void
akita_display_scale(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	volatile uint32_t *pic, *front;

	decon = (volatile uint32_t *)ml_io_map(DECON0, 0x1000);
	rdma = (volatile uint32_t *)ml_io_map(DPP0_RDMA, 0x1000);
	dpp = (volatile uint32_t *)ml_io_map(DPP0_COM, 0x1000);
	coef = (volatile uint32_t *)ml_io_map(DPP0_COEF, 0x1000);
	if (rdma[RDMA_BASEADDR / 4] != (uint32_t)FB_FRONT || decon[TRIG_CON / 4] != 0x3061) {
		printf("akita_display: scale: the pipeline is not as the bootloader leaves it: not tried\n");
		return;
	}
	say_dpp("before");
	front = map_buffer(FB_FRONT, FB_BYTES);
	pic = map_buffer(FB_BACK, SRC_BYTES);
	if (front == NULL || pic == NULL) {
		printf("akita_display: scale: no kernel addresses for the buffers\n");
		return;
	}
	for (uint32_t y = 0; y < SRC_H; y++) {
		for (uint32_t x = 0; x < SRC_W; x++) {
			uint32_t v = x < FB_WIDTH && y < FB_HEIGHT ? front[y * FB_WIDTH + x] : 0xff202020u;

			if (x < 6 || y < 6 || x >= SRC_W - 6 || y >= SRC_H - 6) {
				v = 0xffff0000u;                /* red: the border of all 1125 x 2436 */
			} else if ((x >= FB_WIDTH - 2 && x < FB_WIDTH + 2) || (y >= FB_HEIGHT - 2 && y < FB_HEIGHT + 2)) {
				v = 0xff00ff00u;                /* green: where the panel's edges fell */
			}
			pic[y * SRC_W + x] = v;
		}
	}
	__asm__ volatile("dsb sy" ::: "memory");
	write_coefficients();
	set_source(SRC_W, SRC_H, (uint32_t)FB_BACK);
	delay(100000);
	say_dpp("1125x2436 at 0xfb800000, forced");
	if (dpp[(DPP_SHADOW + DPP_SCL_CTRL) / 4] != 1) {
		decon[0x050 / 4] = 0x3fff | (1u << 31);         /* SHD_REG_UP_REQ: every window, and the global one */
		delay(100000);
		say_dpp("and after DECON0's update request");
	}
	delay(8000000);
	set_source(FB_WIDTH, FB_HEIGHT, (uint32_t)FB_FRONT);
	delay(100000);
	if (dpp[(DPP_SHADOW + DPP_SCL_CTRL) / 4] != 0) {
		decon[0x050 / 4] = 0x3fff | (1u << 31);
		delay(100000);
	}
	say_dpp("back to the bootloader's");
}

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
	if (PE_parse_boot_argn("display_scale", &enabled, sizeof(enabled)) && enabled) {
		static thread_call_t scaler;
		uint32_t at = 60;

		PE_parse_boot_argn("display_scale_delay", &at, sizeof(at));
		scaler = thread_call_allocate(akita_display_scale, NULL);
		clock_interval_to_deadline(at, NSEC_PER_SEC, &deadline);
		thread_call_enter_delayed(scaler, deadline);
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
