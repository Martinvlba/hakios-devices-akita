/*
 * The Pixel 8a's processor clocks (dvfs=1): zuma's three CPU clusters set through ACPM (hakios.akita_acpm.c), by a
 * governor of the kind of Linux's schedutil, and held down when the phone is hot (hakios.akita_thermal.c).
 *
 * ACPM's DVFS (Android's tree: cal_if and the zuma clock driver on IPC channel 0; exynos-acme's domains cal-id
 * 0xb040002 LITTLE cpus 0-3, 0xb040003 MID 4-7, 0xb040004 BIG 8): a request is word 0 the domain (the cal-id's low half:
 * 2, 3, 4), word 1 the clock in kHz, word 2 0 to set it or 1 to read it; the answer has the clock in word 1. Found on
 * the phone with U-Boot's `acpm raw 0 ...`: the bootloader leaves LITTLE 1328 MHz, MID 1065, BIG 1164; each set was
 * read back as set (LITTLE 324000, MID 402000, BIG 500000 and 2914000). The steps are each cluster's as Android lists
 * them (its CL0/CL1/CL2-DVFS states).
 *
 * The governor, every 100 ms: each processor's busy share since the last look (processor_info's tick counts), the
 * busiest of a cluster's taken, and the lowest step at or above 1.25 times that share of the cluster's top -- at once
 * upwards, downwards only after three looks in a row that want less. Hot: from 90 C on the clusters go no higher than
 * the middle of their steps, from 95 C no higher than the lowest, and that is lifted again below 85 C. BIG's one core
 * is not started by this kernel (cpus=8), so BIG is put at its lowest step and left there.
 *
 * kern.hakios.dvfs_mhz ("LITTLE=1328 MID=1065 BIG=500"), kern.hakios.dvfs_changes. dvfs_period_ms= (100).
 */
#include <kern/clock.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/host_info.h>
#include <mach/mach_types.h>
#include <mach/processor_info.h>
#include <pexpert/pexpert.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

extern int akita_acpm_xfer(unsigned channel, uint32_t cmd[4]);
extern int akita_thermal_hottest_c(void);
extern processor_t cpu_to_processor(int cpu);
extern kern_return_t processor_info(processor_t processor, processor_flavor_t flavor, host_t *host,
    processor_info_t info, mach_msg_type_number_t *count);
extern unsigned int real_ncpus;

#define DVFS_CHANNEL            0
#define FREQ_SET                0
#define FREQ_GET                1

struct cluster {
	const char *name;
	uint32_t domain, first_cpu, ncpus;
	const uint32_t *steps;          /* MHz, lowest first */
	unsigned nsteps;
	unsigned now;                   /* the step it is at */
	unsigned lower_wanted;          /* looks in a row that wanted a lower step */
	uint32_t last_busy[4], last_total[4];
	int share;                      /* the busiest core's share at the last look, percent */
	int core_share[4];              /* each core's */
};

static const uint32_t little_steps[] = { 324, 610, 820, 955, 1098, 1197, 1328, 1425, 1548, 1704 };
static const uint32_t mid_steps[] = { 402, 578, 697, 712, 910, 1065, 1221, 1328, 1418, 1572, 1836, 1945, 2130, 2245,
	                              2367 };
static const uint32_t big_steps[] = { 500, 880, 1164, 1298, 1557, 1745, 1885, 2049, 2147, 2294, 2363, 2556, 2687,
	                              2850, 2914 };

static struct cluster clusters[] = {
	{ "LITTLE", 2, 0, 4, little_steps, sizeof(little_steps) / sizeof(little_steps[0]), 0, 0, { 0 }, { 0 } },
	{ "MID", 3, 4, 4, mid_steps, sizeof(mid_steps) / sizeof(mid_steps[0]), 0, 0, { 0 }, { 0 } },
	{ "BIG", 4, 8, 1, big_steps, sizeof(big_steps) / sizeof(big_steps[0]), 0, 0, { 0 }, { 0 } },
};
#define NCLUSTERS (sizeof(clusters) / sizeof(clusters[0]))

static uint32_t period_ms = 100;
static int dvfs_changes_value;
static char dvfs_mhz[64] = "";
static int hot_cap = -1;                /* the highest step allowed while hot, as a fraction: -1 none, 0 lowest, 1 middle */

SYSCTL_DECL(_kern_hakios);
SYSCTL_STRING(_kern_hakios, OID_AUTO, dvfs_mhz, CTLFLAG_RD | CTLFLAG_LOCKED, dvfs_mhz, 0,
    "the CPU clusters' clocks, in MHz");
SYSCTL_INT(_kern_hakios, OID_AUTO, dvfs_changes, CTLFLAG_RD | CTLFLAG_LOCKED, &dvfs_changes_value, 0,
    "how many times a cluster's clock was changed");

static bool
set_mhz(struct cluster *c, uint32_t mhz)
{
	uint32_t cmd[4] = { c->domain, mhz * 1000, FREQ_SET, 0 };

	return akita_acpm_xfer(DVFS_CHANNEL, cmd) == 0;
}

static uint32_t
get_mhz(struct cluster *c)
{
	uint32_t cmd[4] = { c->domain, 0, FREQ_GET, 0 };

	return akita_acpm_xfer(DVFS_CHANNEL, cmd) == 0 ? cmd[1] / 1000 : 0;
}

/* the busiest of the cluster's processors since the last look, in percent; -1 if none could be read */
static int
busiest(struct cluster *c)
{
	int most = -1;

	for (uint32_t i = 0; i < c->ncpus && c->first_cpu + i < real_ncpus; i++) {
		struct processor_cpu_load_info load;
		mach_msg_type_number_t count = PROCESSOR_CPU_LOAD_INFO_COUNT;
		processor_t p = cpu_to_processor((int)(c->first_cpu + i));
		host_t host;
		uint32_t busy, total, d_busy, d_total;

		if (p == PROCESSOR_NULL || processor_info(p, PROCESSOR_CPU_LOAD_INFO, &host, (processor_info_t)&load,
		    &count) != KERN_SUCCESS) {
			continue;
		}
		busy = load.cpu_ticks[CPU_STATE_USER] + load.cpu_ticks[CPU_STATE_SYSTEM] + load.cpu_ticks[CPU_STATE_NICE];
		total = busy + load.cpu_ticks[CPU_STATE_IDLE];
		d_busy = busy - c->last_busy[i];
		d_total = total - c->last_total[i];
		c->last_busy[i] = busy;
		c->last_total[i] = total;
		if (d_total > 0) {
			int share = (int)(100 * (uint64_t)d_busy / d_total);

			c->core_share[i] = share;
			most = share > most ? share : most;
		}
	}
	return most;
}

static void
describe(void)
{
	int n = 0;

	for (unsigned i = 0; i < NCLUSTERS; i++) {
		n += snprintf(dvfs_mhz + n, sizeof(dvfs_mhz) - n, "%s%s=%u", i ? " " : "", clusters[i].name,
		    clusters[i].steps[clusters[i].now]);
	}
}

static void
govern(struct cluster *c)
{
	int share = busiest(c);
	unsigned want = 0, cap = c->nsteps - 1;
	uint32_t target;

	if (share < 0) {
		return;
	}
	c->share = share;
	/* the lowest step at or above 1.25 times the busy share of the top */
	target = (uint32_t)((uint64_t)c->steps[c->nsteps - 1] * (uint32_t)share * 125 / 10000);
	while (want < c->nsteps - 1 && c->steps[want] < target) {
		want++;
	}
	if (hot_cap == 0) {
		cap = 0;
	} else if (hot_cap == 1) {
		cap = (c->nsteps - 1) / 2;
	}
	want = want > cap ? cap : want;
	if (want < c->now && ++c->lower_wanted < 3 && c->now <= cap) {
		return;                 /* down only after three looks in a row (unless the cap demands it) */
	}
	c->lower_wanted = 0;
	if (want != c->now && set_mhz(c, c->steps[want])) {
		c->now = want;
		dvfs_changes_value++;
		describe();
	}
}

static void
akita_dvfs_thread(__unused void *parameter, __unused wait_result_t result)
{
	uint64_t next_log = 0, minute;

	nanoseconds_to_absolutetime(20 * NSEC_PER_SEC, &minute);
	for (unsigned i = 0; i < NCLUSTERS; i++) {
		struct cluster *c = &clusters[i];
		uint32_t was = get_mhz(c);

		/* where the bootloader left it: the step nearest to it */
		for (c->now = 0; c->now < c->nsteps - 1 && c->steps[c->now] < was; c->now++) {
		}
		if (c->first_cpu >= real_ncpus) {
			/* a cluster none of whose cores this kernel runs: its lowest step, for good */
			if (set_mhz(c, c->steps[0])) {
				c->now = 0;
			}
		}
		(void)busiest(c);       /* the first counts */
	}
	describe();
	printf("akita dvfs: the clusters' clocks every %u ms (cores 0-%u); now %s\n", period_ms, real_ncpus - 1, dvfs_mhz);
	for (;;) {
		int hot = akita_thermal_hottest_c();
		uint64_t now;

		if (hot >= 95) {
			hot_cap = 0;
		} else if (hot >= 90 && hot_cap != 0) {
			hot_cap = 1;
		} else if (hot < 85) {
			hot_cap = -1;
		}
		for (unsigned i = 0; i < NCLUSTERS; i++) {
			if (clusters[i].first_cpu < real_ncpus) {
				govern(&clusters[i]);
			}
		}
		now = mach_absolute_time();
		if (now >= next_log) {
			next_log = now + minute;
			printf("akita dvfs: %s (%d changes; %s); busy LITTLE %d %d %d %d, MID %d %d %d %d %%\n", dvfs_mhz,
			    dvfs_changes_value, hot_cap < 0 ? "not held down" : hot_cap ? "held to the middle: hot" :
			    "held to the lowest: hot", clusters[0].core_share[0], clusters[0].core_share[1],
			    clusters[0].core_share[2], clusters[0].core_share[3], clusters[1].core_share[0],
			    clusters[1].core_share[1], clusters[1].core_share[2], clusters[1].core_share[3]);
		}
		delay_for_interval(period_ms, NSEC_PER_MSEC);
	}
}

static void
akita_dvfs_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(akita_dvfs_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

__startup_func
static void
akita_dvfs_init(void)
{
	uint32_t enabled = 0, value;
	uint64_t deadline;
	static thread_call_t start;

	if (!PE_parse_boot_argn("dvfs", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	if (PE_parse_boot_argn("dvfs_period_ms", &value, sizeof(value)) && value >= 20 && value <= 2000) {
		period_ms = value;
	}
	start = thread_call_allocate(akita_dvfs_start, NULL);
	clock_interval_to_deadline(5, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_dvfs_init);
