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
#include <sys/sysctl.h>
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
 *
 * IDMA_SFR_UPDATE_FORCE takes effect AT ONCE: written while DPP0 reads a frame, the DMA switches in the middle of it
 * (the tester's videos, 11-10: thin white lines across the whole width in bursts at every flip, two pictures in one
 * frame). Frame done stays set until cleared, so a thread that wakes late finds an old one and wrote into the next
 * read. So now (11-10): the switch only while DPP0 is idle -- RDMA_ENABLE's IDMA_OP_STATUS (bit 2, Google's
 * OP_STATUS_BUSY) clear, and for a frame done not seen arrive, the time since it (from the beat) inside the idle gap
 * measure_hz found. Measured (11-10, flip1): DPP0 reads a frame in 4.76 ms and idles 3.6 ms before the next at 120 Hz
 * (11.9 ms at 60), and OP_STATUS follows it exactly; wake-ups from a deadline come up to 1.1 ms late (flip2), and a
 * beat kept by timing alone drifts more than a millisecond now and then (flip3). So (flip_pending) the switch goes by
 * what DPP0 is seen doing: busy, then idle within the gap less 300 us of that -- provably between two reads -- and
 * never while it reads (flip2's "forced after 4 put off" and flip3's beat-timed waits did so); at once on arrival only
 * when idle and the beat was caught exactly within two refreshes (scanout() wakes the thread: the frame on the panel
 * up to a refresh sooner); forced only when nothing was read for 50 ms. The beat (vblank_time) is kept by looking at
 * frame done just after it is due, and every 8th refresh catching it as it comes (a spin: delay() of a few us SLEEPS
 * on this kernel, cpu_idle_latency is 0); the thread above the kernel's normal priority. display_say=0: no
 * statistics line (every 30 s while flipping).
 */
#define RDMA_IRQ        0x004
#define FRAME_DONE      (1u << 16)
#define IDMA_OP_STATUS  (1u << 2)       /* RDMA_ENABLE: DPP0's DMA busy reading a frame */
#define NO_BUFFER       (~0u)
/* the pipeline's error flags (health_check, below) */
#define RDMA_ERR_BITS   ((1u << 17) | (1u << 19) | (1u << 21) | (1u << 22) | (1u << 24) | (1u << 26))
#define DSIM_INTSRC     0x050
#define DSIM_ERR_BITS   ((1u << 19) | (1u << 23) | (1u << 27))     /* (not 22, abnormal command state: every frame, at 60 Hz too) */
#define DECON_INT_PEND_EXTRA 0x074
#define DECON_ERR_BITS  ((1u << 0) | (1u << 4) | (1u << 12))
static volatile uint32_t *dsim_regs;
#define DSIM0           0x19440000ULL
static volatile uint32_t *dsim;                 /* DSIM0, for the panel's commands (dsim_dcs) */
static uint32_t panel_on = 1, max_nits = 800;  /* display_panel=, display_max_nits= (panel_commands) */
static uint32_t dimming = 1;                    /* display_dimming=0: the panel's smooth dimming not turned on */
static uint32_t health_on = 1;                 /* display_health=0: the error flags not read (a hang's suspect, 11-10) */

extern kern_return_t kernel_thread_start_priority(thread_continue_t continuation, void *parameter, integer_t priority,
    thread_t *new_thread);

static const uint64_t buffer_phys[3] = { FB_FRONT, FB_BACK, FB_BACK + 0xa00000 };     /* the third: 0xfc200000 */
/*
 * The scan-out slots (zero copy, MiniIOMobileFramebuffer's fb_zc=1): the same free RAM cut into four pieces of
 * 0x9f8000 bytes from 0xfac00000 -- room for 2400 rows of up to 4352 bytes each and a page, the last ending at 0xfd3e0000 below
 * dss_log -- the first the framebuffer's own buffer, the others iOS's display surfaces, scanned out where they are.
 */
#define SLOT_SIZE       0x9f8000ULL      /* (2400 rows of 4352 bytes and a page more: the picture starts on a page) */
#define SLOTS           4
#define RDMA_SRC_WIDTH_REG 0x010              /* the source's width in pixels: its row stride */
static volatile uint64_t scan_phys;            /* a slot queued to be scanned out (0: none) ... */
static volatile uint32_t scan_stride;          /* ... and its rows' bytes */
static volatile uint64_t scanned_phys;         /* what DPP0 reads now */
static uint32_t nbuffers = 3;
static volatile uint32_t shown_index, queued = NO_BUFFER;
static uint32_t flips, flips_dropped;
/* (for a frame counter in user space: inject_sb.c's cube, sb_cube=1 -- the panel's real rate, not what was drawn) */
SYSCTL_DECL(_kern_hakios);
SYSCTL_UINT(_kern_hakios, OID_AUTO, display_flips, CTLFLAG_RD | CTLFLAG_LOCKED, &flips, 0,
    "surfaces put on the panel since boot");
SYSCTL_UINT(_kern_hakios, OID_AUTO, display_dropped, CTLFLAG_RD | CTLFLAG_LOCKED, &flips_dropped, 0,
    "surfaces replaced by a newer one before they reached the panel");
/* (for the governor: frames reaching the panel keep the clocks up as a touch does, hakios.akita_dvfs.c) */
uint32_t akita_display_flip_count(void);
uint32_t
akita_display_flip_count(void)
{
	return flips;
}
static uint64_t flip_wait_us;
/* the switch's timing (display_say): */
static volatile bool op_status_works;          /* measure_hz saw DPP0 go busy and idle at every refresh */
static volatile uint32_t read_us, idle_us;     /* measure_hz: DPP0 reading a frame; the least gap from frame done to the next read */
static uint32_t flips_forced, flips_raced, flips_stale, flips_fresh, flips_at_once;
static uint32_t fd_after_hist[5];              /* frame done -> base written: < 100, 300, 1000, 3000 us, more */
static uint64_t fd_after_max_us, wake_late_max_us, spin_us_total;
static uint32_t say_flips = 1;
static lck_grp_t *flip_group;
static lck_mtx_t *flip_lock;

static void flipper(void *arg, wait_result_t wr);
static void panel_set_hz(uint32_t hz);
static void panel_commands(void);
static uint32_t display_refresh_hz(void);
static uint64_t display_slot(unsigned i, uint64_t *size);
static void display_scanout(uint64_t phys, uint32_t row_bytes);
static uint64_t display_scanned(void);
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
	PE_parse_boot_argn("display_say", &say_flips, sizeof(say_flips));
	PE_parse_boot_argn("display_health", &health_on, sizeof(health_on));
	PE_parse_boot_argn("display_panel", &panel_on, sizeof(panel_on));
	PE_parse_boot_argn("display_max_nits", &max_nits, sizeof(max_nits));
	PE_parse_boot_argn("display_dimming", &dimming, sizeof(dimming));
	max_nits = max_nits < 100 ? 100 : max_nits > 1400 ? 1400 : max_nits;
	dsim = (volatile uint32_t *)ml_io_map(DSIM0, 0x1000);     /* (the panel's commands: panel_commands) */
	if (health_on) {
		dsim_regs = dsim;       /* DSIM0 (its error flags: health_check) */
		printf("akita_display: error flags as found: DPP0 RDMA_IRQ %#x, DSIM0 INTSRC %#x, DECON0 INT_PEND_EXTRA %#x\n",
		    rdma[RDMA_IRQ / 4], dsim_regs[DSIM_INTSRC / 4], decon[DECON_INT_PEND_EXTRA / 4]);
		rdma[RDMA_IRQ / 4] = RDMA_ERR_BITS;
		dsim_regs[DSIM_INTSRC / 4] = DSIM_ERR_BITS;
		decon[DECON_INT_PEND_EXTRA / 4] = DECON_ERR_BITS;
	}
	/* 93 (BASEPRI_PREEMPT_HIGH): above the kernel's own threads (81) and every user thread but real-time ones */
	if (kernel_thread_start_priority(flipper, NULL, 93, &thread) != KERN_SUCCESS) {
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
	scanned_phys = FB_FRONT;
	printf("akita_display: %u buffers from %#llx, flipped by DPP0 between the panel's frames (RDMA_SRC_WIDTH %u)\n", nbuffers,
	    FB_FRONT, rdma[RDMA_SRC_WIDTH_REG / 4]);
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
	thread_wakeup((event_t)&vblank_at);             /* (the flipper: at once if DPP0 idles now) */
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
 * one plus a period, plus 0.2 ms -- or from 1.2 ms before it, every 8th refresh and until one is caught as it comes);
 * a switch waiting is flip_pending's. The switch's rule (dpp_idle) and why: above, at RDMA_IRQ.
 */
/* vblank_at: mach_absolute_time of the last frame done; vblank_hz: refreshes in the last second (declared above) */
static uint64_t vblank_count;

static uint64_t
abs_us(uint64_t abs)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(abs, &ns);
	return ns / 1000;
}

/* a busy wait of ABS (mach_absolute_time units): delay() of a few us would sleep */
static void
spin_abs(uint64_t abs)
{
	uint64_t t0 = mach_absolute_time();

	while (mach_absolute_time() - t0 < abs) {
		__asm__ volatile ("yield");
	}
}

/*
 * May the base be switched now, SINCE_FD after the frame done (FRESH: seen arrive by the poll)? Not while DPP0 reads
 * (its OP_STATUS, when measure_hz found it working); not when the frame done was found already set and the time since
 * it (from the beat) is past the idle gap less 300 us.
 */
static bool
dpp_idle(bool fresh, uint64_t since_fd_abs)
{
	if (op_status_works && (rdma[RDMA_ENABLE / 4] & IDMA_OP_STATUS)) {
		return false;
	}
	if (fresh) {
		return true;
	}
	/* (no gap measured: 800 us assumed) */
	return abs_us(since_fd_abs) + 300 < (idle_us > 300 ? idle_us : 800);
}

/*
 * The pipeline's own error flags, sampled at every frame done (the tester's thin white lines, 11-10: transient, at fixed
 * heights, in bursts with CoreAnimation's GPU work -- what a display DMA starved of memory bandwidth does; MIF drops to
 * 421 MHz at idle on this port, Android's display votes a floor through its BTS): DPP0's RDMA_IRQ error bits (deadlock
 * 17, read slave error 19, config error 21, recovery 22, MO conflict 24, AXI address error 26), DSIM0's INTSRC
 * (underrun 19, invalid SFR 23, PH FIFO overflow 27; not 22, abnormal command state, set at every frame since the
 * bootloader, 60 Hz too) and DECON0's INT_PEND_EXTRA (time out 0, resource conflict 4, DSIM late start 12) -- each
 * counted and cleared (write 1 to clear); the first 16 said with the time. flip3 (11-10, swipes and idle, 140 s): none
 * at all -- the display DMA is not starved.
 */
static uint32_t err_rdma, err_dsim_underrun, err_dsim_other, err_decon, errs_said;

static void
health_check(void)
{
	if (!health_on) {
		return;
	}
	uint32_t r = rdma[RDMA_IRQ / 4] & RDMA_ERR_BITS, d = dsim_regs ? dsim_regs[DSIM_INTSRC / 4] & DSIM_ERR_BITS : 0;
	uint32_t e = decon[DECON_INT_PEND_EXTRA / 4] & DECON_ERR_BITS;

	if ((r | d | e) == 0) {
		return;
	}
	if (r) {
		rdma[RDMA_IRQ / 4] = r;
		err_rdma++;
	}
	if (d) {
		dsim_regs[DSIM_INTSRC / 4] = d;
		if (d & (1u << 19)) {
			err_dsim_underrun++;
		}
		if (d & ~(1u << 19)) {
			err_dsim_other++;
		}
	}
	if (e) {
		decon[DECON_INT_PEND_EXTRA / 4] = e;
		err_decon++;
	}
	if (errs_said < 16) {
		uint64_t ns;

		errs_said++;
		absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
		printf("akita_display: pipeline error at %llu.%03llu s (flip %u): DPP0 RDMA_IRQ %#x, DSIM0 INTSRC %#x, DECON0 "
		    "INT_PEND_EXTRA %#x\n", ns / NSEC_PER_SEC, ns / NSEC_PER_MSEC % 1000, flips, r, d, e);
	}
}

static void
say_flipper(void)
{
	printf("akita_display: errors: DPP0 %u, DSIM0 underruns %u (other %u), DECON0 %u\n", err_rdma, err_dsim_underrun,
	    err_dsim_other, err_decon);
	printf("akita_display: %u flips (%u at once in DPP0's idle gap, %u at a read's end seen, %u by the beat alone); "
	    "%u forced (no read for 50 ms), %u raced (DPP0 busy right after the write); frame done -> write <100 us %u, <300 "
	    "%u, <1 ms %u, <3 ms %u, more %u, longest %llu us; woke up to %llu us late; %llu us spun; %u frame dones caught in "
	    "the last second, %u replaced before shown\n", flips, flips_at_once, flips_fresh, flips_stale, flips_forced,
	    flips_raced, fd_after_hist[0], fd_after_hist[1], fd_after_hist[2], fd_after_hist[3], fd_after_hist[4],
	    fd_after_max_us, wake_late_max_us, spin_us_total, vblank_caught_hz, flips_dropped);
	fd_after_max_us = wake_late_max_us = 0;
}

/* the queued surface or buffer onto the panel (flip_lock held); RACE_CHECK: count a read that started meanwhile */
static void
do_switch(void)
{
	__asm__ volatile ("dsb sy" ::: "memory");        /* (the frame's last writes out of the write buffers) */
	if (scan_phys != 0) {
		/* a surface scanned out where it is (zero copy): its address and its rows' stride */
		rdma[RDMA_BASEADDR / 4] = (uint32_t)scan_phys;
		rdma[RDMA_SRC_WIDTH_REG / 4] = scan_stride / 4;
		__asm__ volatile ("dsb sy" ::: "memory");
		rdma[RDMA_ENABLE / 4] |= SFR_UPDATE_FORCE;
		scanned_phys = scan_phys;
		scan_phys = 0;
		thread_wakeup((event_t)&scanned_phys);
	} else {
		uint32_t index = queued;                /* (the newest, if another came while waiting) */

		queued = NO_BUFFER;
		rdma[RDMA_BASEADDR / 4] = (uint32_t)buffer_phys[index];
		rdma[RDMA_SRC_WIDTH_REG / 4] = FB_WIDTH;
		__asm__ volatile ("dsb sy" ::: "memory");
		rdma[RDMA_ENABLE / 4] |= SFR_UPDATE_FORCE;
		scanned_phys = buffer_phys[index];
		shown_index = index;
		thread_wakeup((event_t)&shown_index);
	}
	if (op_status_works && (rdma[RDMA_ENABLE / 4] & IDMA_OP_STATUS)) {
		flips_raced++;
	}
	flips++;
}

/* the last frame done seen arrive (not estimated): the beat's phase is only trusted for 12 refreshes after one */
static volatile uint64_t vblank_sync_at;

/* the frame done last due before NOW by the beat (0: no recent beat -- four refreshes without one, or none seen
 * arrive for 12: flip2 showed the phase drifting off when only estimated, and switches "in the gap" landed in a read) */
static uint64_t
beat_fd(uint64_t now, uint64_t period)
{
	uint64_t at = vblank_at;

	if (at == 0 || period == 0 || now < at || now - at > 4 * period || now - vblank_sync_at > 12 * period) {
		return 0;
	}
	return at + (now - at) / period * period;
}

static void
flip_count(uint64_t now, uint64_t fd_at, uint32_t *kind)
{
	uint64_t after = abs_us(now - fd_at);

	fd_after_hist[after < 100 ? 0 : after < 300 ? 1 : after < 1000 ? 2 : after < 3000 ? 3 : 4]++;
	if (after > fd_after_max_us) {
		fd_after_max_us = after;
	}
	(*kind)++;
}

/*
 * A queued switch, made only where DPP0 is provably idle with time left before its next read -- never while it reads,
 * whatever the beat says (flip3: the beat's phase drifts more than a millisecond now and then, and a switch timed by it
 * landed in reads): DPP0 seen BUSY at B and then IDLE at I with I - B < the idle gap less 300 us means its frame done
 * was in (B, I] and the next read is at least a gap after B -- switched at I. Looked at every ~1 ms asleep (wake-ups
 * come up to ~1.1 ms late here: I - B stays near 2 ms), spinning instead when the beat puts the read's end within
 * 1.5 ms (2 ms of spin at most a switch). At once on arrival only when idle and the beat was caught exactly within
 * the last two refreshes. Nothing read for 50 ms (no TE: nothing to tear): switched anyway, counted as forced.
 */
static void
flip_pending(uint64_t period)
{
	uint64_t t0 = mach_absolute_time(), now, fd, last_busy = 0, spun = 0, step, near, safe, give_up, spin_most;
	bool done = false, first = true;

	nanoseconds_to_absolutetime(5 * NSEC_PER_USEC, &step);
	nanoseconds_to_absolutetime(1500 * NSEC_PER_USEC, &near);
	nanoseconds_to_absolutetime(((idle_us > 600 ? idle_us : 900) - 300) * NSEC_PER_USEC, &safe);
	nanoseconds_to_absolutetime(50 * NSEC_PER_MSEC, &give_up);
	nanoseconds_to_absolutetime(2 * NSEC_PER_MSEC, &spin_most);
	while (!done) {
		bool busy, spin = false;

		now = mach_absolute_time();
		busy = op_status_works && (rdma[RDMA_ENABLE / 4] & IDMA_OP_STATUS) != 0;
		lck_mtx_lock(flip_lock);
		if (scan_phys == 0 && queued == NO_BUFFER) {
			lck_mtx_unlock(flip_lock);
			return;                         /* (nothing queued after all) */
		}
		if (busy) {
			last_busy = now;
			fd = beat_fd(now, period);
			spin = spun < spin_most && (fd == 0 || fd + period < now + near);
		} else if (last_busy != 0 && now - last_busy < safe) {
			/* a read seen end: its frame done in (last_busy, now] */
			rdma[RDMA_IRQ / 4] = FRAME_DONE;
			vblank_at = now;
			if (now - last_busy < step * 8) {
				vblank_sync_at = now;   /* (caught as it came: the beat exact) */
			}
			vblank_count++;
			flip_count(now, last_busy, &flips_fresh);
			do_switch();
			done = true;
		} else if (first && op_status_works && now - vblank_sync_at < 2 * period && (fd = beat_fd(now, period)) != 0 &&
		    dpp_idle(false, now - fd)) {
			flip_count(now, fd, &flips_at_once);
			do_switch();
			done = true;
		} else {
			/* idle, not provably in the gap: the next read awaited (its start, then its end) */
			last_busy = 0;
			fd = beat_fd(now, period);
			spin = spun < spin_most && fd != 0 && fd + period > now && fd + period < now + near / 2;
		}
		if (!done && (now - t0 > give_up || !op_status_works)) {
			/* (OP_STATUS not trusted: by the beat alone, as before 11-10) */
			if (op_status_works || (fd = beat_fd(now, period)) == 0 || dpp_idle(false, now - fd) || now - t0 > give_up) {
				if (op_status_works || now - t0 > give_up) {
					flips_forced++;
				} else {
					flip_count(now, fd, &flips_stale);
				}
				do_switch();
				done = true;
			}
		}
		lck_mtx_unlock(flip_lock);
		first = false;
		if (done) {
			break;
		}
		if (spin) {
			spin_abs(step);
			spun += step;
		} else {
			assert_wait_timeout((event_t)&flip_pending, THREAD_UNINT, 1000, NSEC_PER_USEC);
			thread_block(THREAD_CONTINUE_NULL);
		}
	}
	spin_us_total += abs_us(spun);
	panel_commands();
	health_check();
}

static void
flipper(__unused void *arg, __unused wait_result_t wr)
{
	uint64_t second_start = mach_absolute_time(), say_at = second_start, period_abs, window_abs, after_abs, catch_abs;
	uint64_t ahead_abs, step_abs, say_abs, last_count = 0, short_abs;
	uint32_t said_flips = 0, beat = 0;
	bool need_sync = true;

	nanoseconds_to_absolutetime(NSEC_PER_SEC / 60, &period_abs);
	vblank_period = period_abs;
	nanoseconds_to_absolutetime(50 * NSEC_PER_MSEC, &window_abs);
	nanoseconds_to_absolutetime(200 * NSEC_PER_USEC, &after_abs);
	nanoseconds_to_absolutetime(1200 * NSEC_PER_USEC, &ahead_abs);
	nanoseconds_to_absolutetime(2600 * NSEC_PER_USEC, &catch_abs);
	nanoseconds_to_absolutetime(500 * NSEC_PER_USEC, &short_abs);
	nanoseconds_to_absolutetime(5 * NSEC_PER_USEC, &step_abs);
	nanoseconds_to_absolutetime(30 * NSEC_PER_SEC, &say_abs);
	rdma[RDMA_IRQ / 4] = FRAME_DONE;
	for (;;) {
		uint64_t now = mach_absolute_time(), t0, next, at;
		bool seen = false, fresh = false, precise;
		wait_result_t w = THREAD_TIMED_OUT;

		while (vblank_paused) {
			assert_wait_timeout((event_t)&vblank_paused, THREAD_UNINT, 5, NSEC_PER_MSEC);
			thread_block(THREAD_CONTINUE_NULL);
			now = mach_absolute_time();
		}
		period_abs = vblank_period;
		if (scan_phys != 0 || queued != NO_BUFFER) {
			flip_pending(period_abs);
			continue;
		}

		/* the beat, no switch waiting: asleep until just after the frame done due, the bit looked at (a frame done not
		 * yet there is caught as it comes, within 0.5 ms); every 8th refresh, and until one succeeds, from 1.2 ms before
		 * it (wake-ups come up to ~1.1 ms late) -- the phase kept exact; woken by a queued switch */
		next = vblank_at != 0 && period_abs != 0 ? (now > vblank_at ? vblank_at + ((now - vblank_at) / period_abs + 1) *
		    period_abs : vblank_at + period_abs) : now;
		precise = need_sync || (++beat & 7) == 0 || vblank_at == 0;
		at = precise ? next - ahead_abs : next + after_abs;
		if (at > now) {
			assert_wait_deadline((event_t)&vblank_at, THREAD_UNINT, at);
			w = thread_block(THREAD_CONTINUE_NULL);
			if (w == THREAD_AWAKENED) {
				continue;               /* (a switch queued) */
			}
		}
		t0 = mach_absolute_time();
		while (mach_absolute_time() - t0 < window_abs) {
			if (rdma[RDMA_IRQ / 4] & FRAME_DONE) {
				seen = true;
				fresh = mach_absolute_time() != t0 && mach_absolute_time() - t0 > step_abs;
				break;
			}
			if (mach_absolute_time() - t0 < (precise ? catch_abs : short_abs)) {
				spin_abs(step_abs);
			} else {
				delay(50);
			}
		}
		now = mach_absolute_time();
		rdma[RDMA_IRQ / 4] = FRAME_DONE;
		spin_us_total += abs_us((now - t0) < catch_abs ? now - t0 : catch_abs);
		if (seen) {
			/* when it was: now if seen arrive, else the beat's (the last one due before now) */
			if (fresh) {
				vblank_at = vblank_sync_at = now;
				need_sync = false;
			} else {
				vblank_at = vblank_at == 0 || period_abs == 0 || now < vblank_at + period_abs ? now :
				    vblank_at + (now - vblank_at) / period_abs * period_abs;
				need_sync = need_sync || precise;
			}
			vblank_count++;
			panel_commands();
		}
		health_check();
		{
			uint64_t ns;
			absolutetime_to_nanoseconds(now - second_start, &ns);
			if (ns >= NSEC_PER_SEC) {
				vblank_caught_hz = (uint32_t)((vblank_count - last_count) * NSEC_PER_SEC / ns);
				last_count = vblank_count;
				second_start = now;
			}
		}
		if (say_flips && flips != said_flips && now - say_at >= say_abs) {
			say_at = now;
			said_flips = flips;
			say_flipper();
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
/* (for the governor's frame boost: the panel's measured rate, hakios.akita_dvfs.c) */
uint32_t akita_display_refresh(void);
uint32_t akita_display_refresh(void) { return vblank_hz; }

/* a scan-out slot's physical address and size (0 past the last) */
static uint64_t
display_slot(unsigned i, uint64_t *size)
{
	*size = SLOT_SIZE;
	return i < SLOTS ? FB_FRONT + i * SLOT_SIZE : 0;
}

/* PHYS (inside a slot, its rows ROW_BYTES apart) scanned out from the panel's next refresh on; returns at once */
static void
display_scanout(uint64_t phys, uint32_t row_bytes)
{
	if (phys < FB_FRONT || phys + (uint64_t)row_bytes * FB_HEIGHT > FB_FRONT + SLOTS * SLOT_SIZE || (row_bytes & 3) != 0 ||
	    row_bytes < FB_WIDTH * 4) {
		return;
	}
	lck_mtx_lock(flip_lock);
	scan_stride = row_bytes;
	scan_phys = phys;
	lck_mtx_unlock(flip_lock);
	thread_wakeup((event_t)&vblank_at);             /* (the flipper: at once if DPP0 idles now) */
}

static uint64_t display_scanned(void) { return scanned_phys; }

/* until DPP0 reads BASE (true), or US microseconds (false): woken by the flipper's switch, no polling */
static bool
display_wait_scanned(uint64_t base, uint32_t us)
{
	uint64_t deadline;

	clock_interval_to_deadline(us, NSEC_PER_USEC, &deadline);
	while (scanned_phys != base) {
		assert_wait_deadline((event_t)&scanned_phys, THREAD_UNINT, deadline);
		if (scanned_phys == base) {
			clear_wait(current_thread(), THREAD_AWAKENED);
			break;
		}
		if (thread_block(THREAD_CONTINUE_NULL) == THREAD_TIMED_OUT) {
			break;
		}
	}
	return scanned_phys == base;
}

/*
 * What the panel shows, for a picture of it (hakios.akita_usbmux.c's port 6): what DPP0 reads -- its base address, its
 * stride and size from the DMA's own registers, not what any driver believes it set -- and a kernel address for those
 * bytes. The scan-out area (the four slots from 0xfac00000) is mapped once, the first time; 0 when the base is outside it.
 */
int akita_display_snapshot(const uint8_t **pixels, uint64_t *phys, uint32_t *width, uint32_t *height, uint32_t *row_bytes);

int
akita_display_snapshot(const uint8_t **pixels, uint64_t *phys, uint32_t *width, uint32_t *height, uint32_t *row_bytes)
{
	static volatile uint32_t *area;

	if (rdma == NULL) {
		rdma = (volatile uint32_t *)ml_io_map(DPP0_RDMA, 0x1000);
	}
	*phys = rdma[RDMA_BASEADDR / 4];
	*row_bytes = rdma[RDMA_SRC_WIDTH_REG / 4] * 4;
	*width = rdma[0x1c / 4] & 0xffff;                     /* RDMA_IMG_SIZE */
	*height = rdma[0x1c / 4] >> 16;
	if (area == NULL) {
		area = map_buffer(FB_FRONT, SLOTS * SLOT_SIZE);
	}
	if (area == NULL || *phys < FB_FRONT || *phys + (uint64_t)*row_bytes * *height > FB_FRONT + SLOTS * SLOT_SIZE) {
		return -1;
	}
	*pixels = (const uint8_t *)area + (*phys - FB_FRONT);
	return 0;
}

/*
 * The panel's commands (display_hz=120 or 60): MIPI DCS writes through DSIM0 (drmdsim@19440000 in the phone's tree), as
 * Google's exynos_drm_dsim.c sends one without batching -- the payload's words into DSIM_PAYLOAD, then the header into
 * DSIM_PKTHDR (data type 0x05 or 0x15 for one or two bytes, 0x39 and the length for more), and the FIFOs waited for.
 * The sequence for a rate is Google's panel driver's for this panel (kernel/devices/google/akita display/
 * panel-google-ak3b.c, ak3b_change_frequency): the test key, 60 00 00 (HS 120) or 60 08 00 (HS 60), B9 31 or 30 (the
 * TE), F7 0F (the update), the test key off. In command mode the panel's TE then comes at that rate, and DECON0's
 * hardware trigger with it: nothing else changes on this side.
 */
#define DSIM_PKTHDR     0x058
#define DSIM_PAYLOAD    0x05c
#define DSIM_FIFOCTRL   0x068
#define DSIM_CMD_CONFIG 0x080
#define FIFO_PH_EMPTY   (1u << 10)
#define FIFO_PL_EMPTY   (1u << 8)
#define PKT_GO_EN       (1u << 16)
#define PKT_GO_RDY      (1u << 17)

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
 * The panel's true rate: the vblank thread paused, frame done's edges counted for half a second by polling every ~2 us
 * (a spin: no sleeping in between, so none is missed); the period set from it for the vblank thread. Also DPP0's DMA
 * watched (RDMA_ENABLE's OP_STATUS): how long it reads a frame, and the least gap from a frame done to the next read --
 * the time the base may be switched in (the flipper's rule: op_status_works, idle_us).
 */
static uint32_t
measure_hz(void)
{
	uint64_t t0, first = 0, last = 0, step, busy_at = 0, fd_seen = 0, busy_sum = 0, idle_least = ~0ULL, idle_most = 0;
	uint32_t edges = 0, reads = 0, gaps = 0;
	bool busy;

	vblank_paused = true;
	delay(20000);                           /* (the thread out of its poll) */
	nanoseconds_to_absolutetime(2 * NSEC_PER_USEC, &step);
	rdma[RDMA_IRQ / 4] = FRAME_DONE;
	busy = (rdma[RDMA_ENABLE / 4] & IDMA_OP_STATUS) != 0;
	t0 = mach_absolute_time();
	while (micros(t0) < 500000) {
		uint64_t now = mach_absolute_time();
		bool b = (rdma[RDMA_ENABLE / 4] & IDMA_OP_STATUS) != 0;

		if (b && !busy) {                       /* a read starts */
			busy_at = now;
			if (fd_seen != 0) {
				uint64_t gap = now - fd_seen;
				if (gap < idle_least) idle_least = gap;
				if (gap > idle_most) idle_most = gap;
				gaps++;
				fd_seen = 0;
			}
		} else if (!b && busy && busy_at != 0) {  /* a read ends */
			busy_sum += now - busy_at;
			reads++;
		}
		busy = b;
		if (rdma[RDMA_IRQ / 4] & FRAME_DONE) {
			now = mach_absolute_time();
			rdma[RDMA_IRQ / 4] = FRAME_DONE;
			if (edges++ == 0) first = now;
			last = now;
			fd_seen = now;
		}
		spin_abs(step);
	}
	if (edges >= 2) {
		vblank_period = (last - first) / (edges - 1);
		vblank_at = vblank_sync_at = last;
	}
	/* OP_STATUS trusted when it went busy and idle about once a refresh */
	op_status_works = edges >= 10 && reads + 3 >= edges && reads <= edges + 3 && gaps + 3 >= edges;
	read_us = reads ? (uint32_t)abs_us(busy_sum / reads) : 0;
	idle_us = gaps ? (uint32_t)abs_us(idle_least) : 0;
	vblank_paused = false;
	thread_wakeup((event_t)&vblank_paused);
	{
		uint64_t ns;
		absolutetime_to_nanoseconds(vblank_period, &ns);
		vblank_hz = ns ? (uint32_t)((NSEC_PER_SEC + ns / 2) / ns) : 0;
	}
	printf("akita_display: %u frame dones in 0.5 s (%u Hz); DPP0 read %u frames, %u us each, the gap from frame done to "
	    "the next read %u-%llu us (%u seen): OP_STATUS %s\n", edges, vblank_hz, reads, read_us, idle_us,
	    gaps ? abs_us(idle_most) : 0, gaps, op_status_works ? "trusted" : "NOT trusted (the switch goes by the beat alone)");
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

/*
 * The panel's BRIGHTNESS and POWER as iOS asks for them (MiniIOMobileFramebuffer: CoreBrightness's IOMFBBrightnessLevel,
 * backboardd's RequestPowerChange -- the screen locked, auto-lock): wanted here from any thread, sent by the vblank thread
 * right after a frame done (the rest of the refresh is free for commands, as panel_set_hz sends them), the newest only.
 *   Brightness: DCS 0x51 and the 12-bit level, high byte first (panel-google-ak3b.c, ak3b_set_brightness): 209 (2 nits)
 *   to 3175 (800 nits), 3176-4095 the panel's HBM (to 1400 nits) -- ak3b_brightness_capability. iOS asks in nits (16.16);
 *   the level is put between the table's ends linearly, and kept to display_max_nits= (800: no HBM unless asked).
 *   Power: off is DCS 0x28 (display off) alone -- the panel stays out of sleep and its TE goes on, so DECON0's frames,
 *   this thread, the flips and the GPU's submission gate see no difference; nothing is lit. On is 0x29. (Google's off adds
 *   sleep in, 0x10, and its on the whole init sequence again: more saved, more to go wrong -- not yet.)
 * display_panel=0: neither sent (the panel as the bootloader left it).
 */
static volatile uint32_t want_level, sent_level;       /* the panel's level; 0: none asked for yet */
static volatile uint32_t want_on = 1, sent_on = 1;
static uint32_t levels_sent, levels_said, panel_fails;
SYSCTL_UINT(_kern_hakios, OID_AUTO, display_level, CTLFLAG_RD | CTLFLAG_LOCKED, (uint32_t *)&sent_level, 0,
    "the panel's brightness level last sent (209-4095; 0: the bootloader's)");
SYSCTL_UINT(_kern_hakios, OID_AUTO, display_on, CTLFLAG_RD | CTLFLAG_LOCKED, (uint32_t *)&sent_on, 0,
    "the panel lit (1) or not (0)");

static uint32_t
nits_to_level(uint32_t nits16)
{
	uint64_t n = nits16, lo = 2ull << 16, mid = 800ull << 16, top = (uint64_t)max_nits << 16;

	n = n < lo ? lo : n > top ? top : n;
	if (n <= mid) {
		return 209 + (uint32_t)((n - lo) * (3175 - 209) / (mid - lo));
	}
	return 3176 + (uint32_t)((n - mid) * (4095 - 3176) / ((1400ull << 16) - mid));
}

static void
display_brightness(uint32_t nits16)
{
	static uint64_t said_at;
	static uint32_t said_level;
	uint64_t now = mach_absolute_time();
	uint32_t level = nits_to_level(nits16);

	/* 1 nit or less is never a setting -- the slider's least is ~2 -- but CoreBrightness's while the backlight's factor
	 * is 0: 1 at a boot and a wake, 0 at a lock, the real value coming ~0.7 s later (the brightness bridge, inject.c):
	 * applied, they left the panel at its least for that long (and a boot's for good, before the bridge's replay). Not
	 * applied: the panel keeps its level through the dark and lights at it again. */
	if (nits16 <= 0x10000) {
		return;
	}
	want_level = level;
	/* the first 40, then at most one a second while it changes (a ramp sends ~60 a second) */
	if (level != said_level && (levels_said < 40 || abs_us(now - said_at) >= 1000000)) {
		levels_said++;
		said_at = now;
		said_level = level;
		printf("akita_display: brightness %u.%02u nits asked -> level %u\n", nits16 >> 16, (nits16 & 0xffff) * 100 >> 16,
		    level);
	}
}

static void
display_power(bool on)
{
	want_on = on ? 1 : 0;
	printf("akita_display: the panel to be %s\n", on ? "lit" : "dark");
}

static void
panel_commands(void)
{
	uint32_t level = want_level, on = want_on;
	bool ok;

	if (!panel_on || dsim == NULL || (level == sent_level && on == sent_on)) {
		return;
	}
	if (!on && sent_on) {
		const uint8_t off = 0x28;
		ok = dsim_dcs(&off, 1);
		sent_on = 0;
		panel_fails += !ok;
		printf("akita_display: the panel dark (display off %s)\n", ok ? "sent" : "NOT sent");
	}
	/*
	 * THE PANEL'S OWN DIMMING (11-11, the tester: the brightness slider "jumps"): DCS 0x53, WRCTRLD, BCTRL (0x20) and
	 * DIMMING (0x08) -- the panel then goes from one level to the next smoothly over a few frames, as on Android
	 * (panel-google-ak3b.c ak3b_update_wrctrld with dimming_on); without it each of the slider's steps was a jump of the
	 * light. Once, before the first level.
	 */
	if (level != 0 && dimming == 1) {
		const uint8_t w[2] = { 0x53, 0x28 };
		ok = dsim_dcs(w, sizeof(w));
		dimming = 2;
		printf("akita_display: the panel's smooth dimming on (WRCTRLD 0x28 %s)\n", ok ? "sent" : "NOT sent");
	}
	if (level != 0 && level != sent_level) {
		const uint8_t b[3] = { 0x51, (uint8_t)(level >> 8), (uint8_t)level };
		ok = dsim_dcs(b, sizeof(b));
		sent_level = level;
		levels_sent++;
		if (!ok && panel_fails++ < 10) {
			printf("akita_display: brightness level %u NOT sent (FIFOCTRL %#x)\n", level, dsim[DSIM_FIFOCTRL / 4]);
		}
	}
	if (on && !sent_on) {
		const uint8_t lit = 0x29;
		ok = dsim_dcs(&lit, 1);
		sent_on = 1;
		panel_fails += !ok;
		printf("akita_display: the panel lit (display on %s, level %u)\n", ok ? "sent" : "NOT sent", sent_level);
	}
}

struct hakios_display_ops {
	int (*buffers)(vm_offset_t va[3], uint32_t *count, uint32_t *width, uint32_t *height, uint32_t *row_bytes);
	void (*flip)(unsigned index);
	uint32_t (*shown)(void);
	uint32_t (*refresh_hz)(void);
	uint64_t (*vblank_time)(void);
	uint64_t (*slot)(unsigned i, uint64_t *size);
	void (*scanout)(uint64_t phys, uint32_t row_bytes);
	uint64_t (*scanned)(void);
	bool (*wait_scanned)(uint64_t base, uint32_t us);
	void (*brightness)(uint32_t nits16);
	void (*power)(bool on);
};
extern const struct hakios_display_ops *hakios_display_ops;     /* MiniIOMobileFramebuffer's */
static const struct hakios_display_ops akita_display_ops = { display_buffers, display_flip, display_shown, display_refresh_hz,
	                                                     display_vblank_time, display_slot, display_scanout, display_scanned,
	                                                     display_wait_scanned, display_brightness, display_power };

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
