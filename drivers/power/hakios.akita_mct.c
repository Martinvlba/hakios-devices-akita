/*
 * The Pixel 8a's MCT -- zuma's multi-core timer v3 (mct-v3@100D0000, Google's drivers/clocksource/exynos_mct_v3.c) --
 * as the wake of idle cores that power down (boot-arg cpuidle=1, osfmk/arm64/psci_smp.c; devices/akita TODO 27).
 *
 * Such a core (the device tree's idle state "c2", PSCI power state 0x10000) loses its own timer meanwhile
 * (local-timer-stop): Android wakes it with this one, which keeps running. The MCT has a free-running counter (FRC)
 * and twelve comparators, each with its own interrupt; Android gives comparator N to core N (its interrupt-map: the
 * MCT's interrupt N is GIC SPI 758 + N). arm(cpu, deadline) loads that core's comparator with the time to its deadline
 * in MCT cycles (COMP_PERIOD is relative: the comparator fires that many cycles after it is enabled) and enables it;
 * the interrupt, routed to that core (attached the first time it idles, from it), wakes it out of the power-down;
 * disarm (and the interrupt) stops it again. The counter's rate is measured against the system's timebase at start.
 * Every 30 s a line: how often the cores powered down, came back at once, were refused; how often the MCT woke one.
 */
#include <kern/clock.h>
#include <kern/sched_prim.h>
#include <kern/thread.h>
#include <os/atomic_private.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

/* osfmk/arm64/psci_smp.h (not reachable from bsd/): the idle cores' power-down and its wake */
typedef bool (*psci_idle_arm_t)(int cpu, uint64_t deadline);
typedef void (*psci_idle_disarm_t)(int cpu);
extern void psci_idle_set_wake(psci_idle_arm_t arm, psci_idle_disarm_t disarm);
extern bool psci_idle_wanted(void);
extern void psci_idle_totals(uint64_t *tries, uint64_t *downs, uint64_t *back, uint64_t *refused);
extern int64_t psci_idle_error(void);
extern uint32_t psci_idle_states(uint32_t *taken);

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *context, uint32_t intid),
    void *context);
extern unsigned int real_ncpus;

#define MCT_PHYS                0x100d0000ULL
#define MCT_CFG                 0x000
#define MCT_FRC_ENABLE          0x100
#define MCT_CNT_L               0x110
#define MCT_CNT_U               0x114
#define MCT_VER                 0x128
#define MCT_COMP_MODE(i)        (0x208 + (i) * 0x100)
#define MCT_COMP_PERIOD(i)      (0x20c + (i) * 0x100)
#define MCT_COMP_ENABLE(i)      (0x210 + (i) * 0x100)
#define MCT_INT_ENB(i)          (0x214 + (i) * 0x100)
#define MCT_INT_CSTAT(i)        (0x218 + (i) * 0x100)
#define MCT_DIV_REQ             (1u << 8)
#define MCT_SPI_INTID(i)        (32 + 758 + (i))        /* the device tree's interrupt-map: SPI 0x2f6 + N */
#define MCT_CPUS                9
/*
 * The hint the firmware wants as well (11-12: CPU_SUSPEND refused, -2, for every power state, from EL1 and EL2 alike):
 * Linaro's "soc: samsung: exynos-pmu: Enable CPU Idle for gs101" -- "the C2 hint is required to actually enter the C2
 * idle state in addition to the PSCI calls due to limitations in the el3mon/ACPM firmware": the PMU's CPU_INFORM
 * register of the core (PMU + 0x860 + 4 x cpu) set to 1 (C2) before the call and back to 0 after; the PMU written
 * through Google's secure-register SMC, as hakios.akita_power.c writes its reboot command.
 */
#define PMU_CPU_INFORM(cpu)     (0x15460000ULL + 0x860 + 4 * (uint64_t)(cpu))
#define CPU_INFORM_CLEAR        0
#define CPU_INFORM_C2           1
#define TENSOR_SMC_PMU_SEC_REG  0x82000504
static uint32_t mct_hint = 1;                   /* cpuidle_hint=0: not written (as before) */

static uint64_t
pmu_smc(uint64_t function, uint64_t a1, uint64_t a2, uint64_t a3)
{
	register uint64_t x0 __asm__("x0") = function, x1 __asm__("x1") = a1, x2 __asm__("x2") = a2, x3 __asm__("x3") = a3;
	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : :
	    "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

static vm_offset_t mct;
static uint64_t mct_per_abs_q32;                /* MCT cycles per timebase tick, Q32 */
static volatile uint32_t mct_attached;          /* a bit per core whose comparator's interrupt is routed to it */
static uint64_t mct_fires[MCT_CPUS], mct_arms[MCT_CPUS];

static inline uint32_t
rd(uint32_t off)
{
	return *(volatile uint32_t *)(mct + off);
}

static inline void
wr(uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(mct + off) = v;
}

static uint64_t
mct_count(void)
{
	uint32_t hi, lo;

	do {
		hi = rd(MCT_CNT_U);
		lo = rd(MCT_CNT_L);
	} while (hi != rd(MCT_CNT_U));
	return ((uint64_t)hi << 32) | lo;
}

/* the comparator stopped, as exynos_mct_comp_stop: disabled (and seen so), one-shot, its interrupt off and cleared */
static void
comp_stop(int i)
{
	wr(MCT_COMP_ENABLE(i), 0);
	for (int k = 0; k < 2000 && rd(MCT_COMP_ENABLE(i)) != 0; k++) {
	}
	wr(MCT_COMP_MODE(i), 0);
	wr(MCT_INT_ENB(i), 0);
	wr(MCT_INT_CSTAT(i), 1);
}

static void
mct_interrupt(void *context, __unused uint32_t intid)
{
	int i = (int)(uintptr_t)context;

	comp_stop(i);
	if (i < MCT_CPUS) {
		mct_fires[i]++;
	}
}

static bool
mct_arm(int cpu, uint64_t deadline)
{
	uint64_t now = mach_absolute_time(), cycles;

	if (mct == 0 || cpu >= MCT_CPUS || deadline <= now) {
		return false;
	}
	if (!(mct_attached & (1u << cpu))) {
		/* (from this core: the interrupt goes to the processor that attaches it) */
		if (gic_v3_spi_attach(MCT_SPI_INTID(cpu), false, mct_interrupt, (void *)(uintptr_t)cpu) != KERN_SUCCESS) {
			return false;
		}
		os_atomic_or(&mct_attached, 1u << cpu, relaxed);
	}
	cycles = ((deadline - now) * mct_per_abs_q32) >> 32;
	if (cycles < 64 || cycles > 0xffffffffULL) {
		return false;
	}
	comp_stop(cpu);
	wr(MCT_COMP_PERIOD(cpu), (uint32_t)cycles);
	wr(MCT_INT_ENB(cpu), 1);
	wr(MCT_COMP_ENABLE(cpu), 1);
	for (int k = 0; k < 2000 && rd(MCT_COMP_ENABLE(cpu)) == 0; k++) {
	}
	mct_arms[cpu]++;
	if (mct_hint) {
		(void)pmu_smc(TENSOR_SMC_PMU_SEC_REG, PMU_CPU_INFORM(cpu), 1, CPU_INFORM_C2);
	}
	return true;
}

static void
mct_disarm(int cpu)
{
	if (mct != 0 && cpu < MCT_CPUS) {
		if (mct_hint) {
			(void)pmu_smc(TENSOR_SMC_PMU_SEC_REG, PMU_CPU_INFORM(cpu), 1, CPU_INFORM_CLEAR);
		}
		comp_stop(cpu);
	}
}

static void
mct_report(__unused void *p, __unused wait_result_t w)
{
	for (;;) {
		uint64_t tries, downs, back, refused, fires = 0, arms = 0;

		assert_wait_timeout((event_t)&mct_report, THREAD_UNINT, 30, NSEC_PER_SEC);
		thread_block(THREAD_CONTINUE_NULL);
		uint32_t taken, asking = psci_idle_states(&taken);

		psci_idle_totals(&tries, &downs, &back, &refused);
		for (int i = 0; i < MCT_CPUS; i++) {
			fires += mct_fires[i];
			arms += mct_arms[i];
		}
		printf("akita mct: idle so far: %llu tries -- %llu powered down, %llu back at once, %llu refused; the MCT armed %llu "
		    "times, woke a core %llu times (cores attached 0x%x); the firmware's last refusal %lld; state 0x%x asked, 0x%x "
		    "taken\n", tries, downs, back, refused, arms, fires, mct_attached, psci_idle_error(), asking, taken);
	}
}

static void
akita_mct_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint64_t c0, c1, t0, t1, ns;
	thread_t thread;

	PE_parse_boot_argn("cpuidle_hint", &mct_hint, sizeof(mct_hint));
	mct = ml_io_map(MCT_PHYS, 0x1000);
	if (mct == 0) {
		printf("akita mct: cannot map 0x%llx\n", MCT_PHYS);
		return;
	}
	if (rd(MCT_FRC_ENABLE) == 0) {
		wr(MCT_CFG, MCT_DIV_REQ | 0);   /* (the device tree's div = 1) */
		wr(MCT_FRC_ENABLE, 1);
	}
	for (int i = 0; i < MCT_CPUS; i++) {
		comp_stop(i);
	}
	c0 = mct_count();
	t0 = mach_absolute_time();
	delay(20000);
	c1 = mct_count();
	t1 = mach_absolute_time();
	if (c1 <= c0 || t1 <= t0) {
		printf("akita mct: its counter does not run (version 0x%x, FRC enable %u, %llu -> %llu): no wake for idle cores\n",
		    rd(MCT_VER), rd(MCT_FRC_ENABLE), c0, c1);
		mct = 0;
		return;
	}
	mct_per_abs_q32 = ((c1 - c0) << 32) / (t1 - t0);
	absolutetime_to_nanoseconds(t1 - t0, &ns);
	printf("akita mct: version 0x%x, its counter at %llu kHz (%llu cycles in %llu us); the wake for idle cores (cpuidle)%s; "
	    "CPU_INFORM of core 4 reads 0x%llx\n", rd(MCT_VER), (c1 - c0) * 1000000ULL / (ns ? ns : 1), c1 - c0, ns / 1000,
	    mct_hint ? ", with the PMU's C2 hint" : "", pmu_smc(TENSOR_SMC_PMU_SEC_REG, PMU_CPU_INFORM(4), 0, 0));
	psci_idle_set_wake(mct_arm, mct_disarm);
	if (kernel_thread_start(mct_report, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

__startup_func
static void
akita_mct_init(void)
{
	static thread_call_t start;
	uint64_t deadline;

	uint32_t want = 0;

	/* (read here: psci_smp_init(), which reads it for the idle path, runs later than this -- the platform expert) */
	if (!PE_parse_boot_argn("cpuidle", &want, sizeof(want)) || want == 0) {
		return;
	}
	start = thread_call_allocate(akita_mct_start, NULL);
	clock_interval_to_deadline(8, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_mct_init);
