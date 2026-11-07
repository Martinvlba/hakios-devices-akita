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
 * upwards, downwards only after three looks in a row that want less. Hot (the hottest sensor): from 75 C BIG, the
 * Cortex-X3, goes no higher than the middle of its steps (as Android holds the X3 back early: BIG's early throttle and
 * trip points from 70 C; at its top it took the phone from 45 to 81 C in a few minutes of start-up work), from 85 C
 * every cluster no higher than its middle, from 92 C no higher than its lowest; each lifted 5 degrees lower. A cluster
 * whose cores this kernel does not start (BIG with cpus=8) is put at its lowest step and left there.
 *
 * The skin (hakios.akita_thermal.c's VIRTUAL-SKIN): Android's own thermal policy for this phone, ported from its thermal
 * HAL with its numbers (see the controllers below): five PID controllers on the skin (CPU-LIGHT aiming at 39 C from 37,
 * CPU-MID at 41 from 39, CPU-HIGH at 43 from 41, SOC at 45 from 37, GPU at 45 from 43), each giving a power budget
 * (sustainable power + P + I, within its limits) shared by weight among its cooling devices -- the three clusters and
 * the GPU -- whose states come from the power each state costs; per severity a ceiling (and at 52-55 C a hard limit),
 * a step at a time; each device takes the most throttling request. A cluster's state n is a ceiling n steps under its
 * top; the GPU's state is a floor under its governor's row. thermal_test_shift=N reads the skin N degrees warmer.
 *
 * The GPU's clocks (domains 8 and 7, the Mali driver's 13 rows, 900/890 to 160/150 MHz): a governor with Android's
 * bounds per row (its dvfs_table's util min/max and hysteresis) on the GPU's busy share since the last look
 * (hakios.akita_mali.c: the time work was outstanding on any queue).
 *
 * kern.hakios.dvfs_mhz ("LITTLE=1328 MID=1065 BIG=500 GPU=160/150"), kern.hakios.dvfs_changes, dvfs_skin_caps (the
 * thermal ceilings).
 * dvfs_period_ms= (100).
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
extern int akita_thermal_skin_mc(void);
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
	int cdev;                       /* its cooling device (Android's thermal-cpufreq-n) */
};

static const uint32_t little_steps[] = { 324, 610, 820, 955, 1098, 1197, 1328, 1425, 1548, 1704 };
static const uint32_t mid_steps[] = { 402, 578, 697, 712, 910, 1065, 1221, 1328, 1418, 1572, 1836, 1945, 2130, 2245,
	                              2367 };
static const uint32_t big_steps[] = { 500, 880, 1164, 1298, 1557, 1745, 1885, 2049, 2147, 2294, 2363, 2556, 2687,
	                              2850, 2914 };

static struct cluster clusters[] = {
	{ "LITTLE", 2, 0, 4, little_steps, sizeof(little_steps) / sizeof(little_steps[0]), 0, 0, { 0 }, { 0 }, 0, { 0 }, 0 },
	{ "MID", 3, 4, 4, mid_steps, sizeof(mid_steps) / sizeof(mid_steps[0]), 0, 0, { 0 }, { 0 }, 0, { 0 }, 1 },
	{ "BIG", 4, 8, 1, big_steps, sizeof(big_steps) / sizeof(big_steps[0]), 0, 0, { 0 }, { 0 }, 0, { 0 }, 2 },
};

static char dvfs_skin_caps[96] = "";
static int skin_shift_mc;               /* thermal_test_shift=N: every skin target and threshold N degrees lower (testing) */
#define NCLUSTERS (sizeof(clusters) / sizeof(clusters[0]))

static uint32_t period_ms = 100;
static int dvfs_changes_value;
static char dvfs_mhz[96] = "";
static int hot_cap = -1;                /* every cluster's cap while hot: -1 none, 1 the middle step, 0 the lowest */
static bool big_cap;                    /* BIG's own, earlier: the middle step */

SYSCTL_DECL(_kern_hakios);
SYSCTL_STRING(_kern_hakios, OID_AUTO, dvfs_mhz, CTLFLAG_RD | CTLFLAG_LOCKED, dvfs_mhz, 0,
    "the CPU clusters' clocks, in MHz");
SYSCTL_INT(_kern_hakios, OID_AUTO, dvfs_changes, CTLFLAG_RD | CTLFLAG_LOCKED, &dvfs_changes_value, 0,
    "how many times a cluster's clock was changed");
SYSCTL_STRING(_kern_hakios, OID_AUTO, dvfs_skin_caps, CTLFLAG_RD | CTLFLAG_LOCKED, dvfs_skin_caps, 0,
    "the ceilings the skin's temperature puts on the clusters and the GPU, in MHz");

static bool
domain_set_khz(uint32_t domain, uint32_t khz)
{
	uint32_t cmd[4] = { domain, khz, FREQ_SET, 0 };

	return akita_acpm_xfer(DVFS_CHANNEL, cmd) == 0;
}

static uint32_t
domain_get_khz(uint32_t domain)
{
	uint32_t cmd[4] = { domain, 0, FREQ_GET, 0 };

	return akita_acpm_xfer(DVFS_CHANNEL, cmd) == 0 ? cmd[1] : 0;
}

/*
 * The GPU's clocks, as the Mali driver lists them on this phone (its dvfs_table: gpu0 the top level, ACPM domain 8;
 * gpu1 the shaders, domain 7), fastest first; a row is a state of Android's thermal-gpufreq-0.
 */
static const uint32_t gpu_rows[][2] = {
	{ 900000, 890000 }, { 900000, 850000 }, { 900000, 807000 }, { 900000, 723000 }, { 749000, 649000 },
	{ 749000, 580000 }, { 659000, 521000 }, { 659000, 467000 }, { 531000, 419000 }, { 531000, 376000 },
	{ 430000, 337000 }, { 386000, 302000 }, { 160000, 150000 },
};
#define NGPUROWS (sizeof(gpu_rows) / sizeof(gpu_rows[0]))
/* each row's utilisation bounds (percent) and how many looks under the lower one before going down: the Mali driver's
 * dvfs_table on this phone (Android's quickstep governor works from these) */
static const uint8_t gpu_util[][3] = {
	{ 85, 97, 1 }, { 76, 94, 1 }, { 85, 96, 2 }, { 83, 97, 3 }, { 78, 95, 3 }, { 80, 96, 3 }, { 76, 95, 3 },
	{ 72, 95, 3 }, { 67, 94, 2 }, { 65, 94, 1 }, { 60, 94, 1 }, { 50, 92, 1 }, { 0, 90, 1 },
};
static uint32_t gpu_boot[2];            /* kHz, as the bootloader left them (0: not readable: the GPU left alone) */
static int gpu_applied = -1;            /* the row last set (-1: the bootloader's clocks untouched) */
static int gpu_want;                    /* the governor's row (fastest 0), before the thermal floor */
static unsigned gpu_low_looks;          /* looks in a row under the row's lower bound */
static int gpu_util_pct;
static uint64_t gpu_busy_last, gpu_time_last;
extern uint64_t akita_mali_busy_abs(void);

/*
 * Android's thermal policy for this phone, from its thermal HAL (hardware/google/pixel thermal/utils/
 * thermal_throttling.cpp: updatePowerBudget, allocatePowerToCdev, updateCdevRequestByPower,
 * computeCoolingDevicesRequest) with the phone's own numbers: /vendor/etc/thermal_info_config.json's controllers on
 * VIRTUAL-SKIN, and the kernel's state2power_table of each cooling device (mW per state; a CPU cluster's state n is
 * n steps under its top, cpufreq_cooling's way). Without power rails (ODPM) the HAL shares the budget by weight
 * alone, which is what is done here.
 */
enum { CD_LITTLE, CD_MID, CD_BIG, CD_GPU, NCDEV };
static const int cd_power[NCDEV][15] = {
	{ 780, 620, 228, 180, 148, 124, 100, 76, 48, 44 },
	{ 6056, 5088, 4168, 3216, 2552, 1796, 1472, 1280, 1064, 832, 624, 456, 424, 340, 236 },
	{ 3714, 3175, 2415, 1911, 1533, 1363, 1145, 1034, 784, 675, 535, 391, 321, 213, 116 },
	{ 2161, 1994, 1802, 1475, 1230, 1041, 881, 758, 653, 561, 483, 414, 192 },
};
static const int cd_states[NCDEV] = { 10, 15, 15, 13 };
#define NOT_SET 0x7fffffff
#define ANY_STEP 1000

struct binding {
	int cdev, weight;               /* weight in thousandths */
	int throttle_step, release_step;
	int ceiling[7], limit[7];       /* the most and the least throttling at each severity */
};
struct controller {
	const char *name;
	int hot[7], hyst[7];            /* millidegrees; NOT_SET where Android has NAN */
	int target;                     /* the severity whose threshold the PID aims at */
	int k_po, k_pu, k_i, i_max, s_power, min_power, max_power, i_cutoff;   /* mW per degree, mW, degrees */
	int nbind;
	struct binding bind[4];
	/* state */
	int severity;
	int64_t i_budget_uw;            /* the integral, in microwatts */
	int prev_budget;
	int request[4];                 /* each binding's state */
};
#define N_ NOT_SET
static struct controller ctls[] = {
	{ "CPU-LIGHT", { N_, 37000, 39000, N_, N_, N_, N_ }, { 0, 1900, 1900, 0, 0, 0, 0 }, 2,
	  400, 400, 5, 2200, 800, 800, 4600, 4, 3, {
		{ CD_LITTLE, 1000, 1, 1, { 0, 2, 2, 2, 2, 2, 2 }, { 0 } },
		{ CD_MID, 1000, 2, 1, { 0, 6, 6, 6, 6, 6, 6 }, { 0 } },
		{ CD_BIG, 1000, 2, 1, { 0, 8, 8, 8, 8, 8, 8 }, { 0 } } }, 0, 0, 0, { 0 } },
	{ "CPU-MID", { N_, 39000, 41000, N_, N_, N_, N_ }, { 0, 0, 1900, 0, 0, 0, 0 }, 2,
	  400, 400, 5, 1500, 700, 0, 2800, 4, 3, {
		{ CD_LITTLE, 292, 1, 1, { 0, 6, 6, 6, 6, 6, 6 }, { 0 } },
		{ CD_MID, 804, 2, 1, { 0, 9, 9, 9, 9, 9, 9 }, { 0 } },
		{ CD_BIG, 342, 2, 1, { 0, 12, 12, 12, 12, 12, 12 }, { 0 } } }, 0, 0, 0, { 0 } },
	{ "CPU-HIGH", { N_, 41000, 43000, 52000, N_, N_, N_ }, { 0, 0, 1900, 1900, 0, 0, 0 }, 2,
	  400, 400, 5, 1000, 600, 0, 1600, 4, 3, {
		{ CD_LITTLE, 156, 1, 1, { 0, 8, 8, 8, 8, 8, 8 }, { 0 } },
		{ CD_MID, 428, 2, 1, { 0, 11, 11, 11, 11, 11, 11 }, { 0 } },
		{ CD_BIG, 225, 2, 1, { 0, 13, 13, 13, 13, 13, 13 }, { 0 } } }, 0, 0, 0, { 0 } },
	{ "SOC", { N_, 37000, 43000, 45000, 47000, 52000, 55000 }, { 0, 1900, 1900, 1900, 1400, 1900, 1900 }, 3,
	  300, 300, 5, 2600, 800, 800, 3900, 8, 4, {
		{ CD_LITTLE, 1000, 1, 1, { 0, 8, 8, 8, 8, 9, 9 }, { 0, 0, 0, 0, 0, 9, 9 } },
		{ CD_MID, 1000, 1, 1, { 0, 11, 11, 11, 11, 14, 14 }, { 0, 0, 0, 0, 0, 14, 14 } },
		{ CD_BIG, 1000, 1, 1, { 0, 13, 13, 13, 13, 14, 14 }, { 0, 0, 0, 0, 0, 14, 14 } },
		{ CD_GPU, 1000, 1, 1, { 0, 9, 9, 9, 9, 11, 11 }, { 0, 0, 0, 0, 0, 11, 11 } } }, 0, 0, 0, { 0 } },
	{ "GPU", { N_, 43000, 45000, 46500, 52000, N_, N_ }, { 0, 1900, 1900, 1400, 1900, 0, 0 }, 2,
	  700, 700, 5, 1723, 473, 0, 2500, 4, 1, {
		{ CD_GPU, 1000, ANY_STEP, ANY_STEP, { 0, 8, 8, 9, 11, 11, 11 }, { 0 } } }, 0, 0, 0, { 0 } },
};
#undef N_
#define NCTLS (sizeof(ctls) / sizeof(ctls[0]))
static int cd_state[NCDEV];             /* the result: each cooling device's state (0 none) */
static uint64_t pid_last;               /* when the controllers last ran */

/* the HAL's getSeverityFromThresholds (hot side): up at a threshold, down only under it less its hysteresis */
static int
severity_of(const struct controller *c, int mc)
{
	int hot = 0, hyst = 0;

	for (int s = 1; s < 7; s++) {
		if (c->hot[s] == NOT_SET) {
			continue;
		}
		if (mc >= c->hot[s]) {
			hot = s;
		}
		if (mc >= c->hot[s] - c->hyst[s]) {
			hyst = s;
		}
	}
	return hot > c->severity ? hot : (hyst < c->severity ? hyst : c->severity);
}

static int
state_for_power(int cdev, int64_t mw)
{
	int i;

	for (i = 0; i < cd_states[cdev] - 1; i++) {
		if (mw >= cd_power[cdev][i]) {
			break;
		}
	}
	return i;
}

/* one controller's update (updatePowerBudget + allocatePowerToCdev without power rails + updateCdevRequestByPower) */
static void
pid_update(struct controller *c, int mc)
{
	int sev = c->severity;

	if (sev == 0) {
		for (int b = 0; b < c->nbind; b++) {
			c->request[b] = 0;      /* (the ceiling at severity 0 is 0: all released) */
		}
		c->i_budget_uw = 0;
		return;
	}
	bool fully_throttled = true, fully_released = true;
	for (int b = 0; b < c->nbind; b++) {
		if (c->request[b] > c->bind[b].limit[sev]) {
			fully_released = false;
		}
		if (c->request[b] < c->bind[b].ceiling[sev]) {
			fully_throttled = false;
		}
	}
	int err = c->hot[c->target] - mc;                               /* millidegrees */
	int64_t p_uw = (int64_t)err * (err < 0 ? c->k_po : c->k_pu);    /* mdeg * mW/deg = uW */
	if (err < c->i_cutoff * 1000) {
		if ((err < 0 && c->prev_budget > c->min_power && !fully_throttled) ||
		    (err > 0 && c->prev_budget < c->max_power && !fully_released)) {
			c->i_budget_uw += (int64_t)err * c->k_i;
		}
	}
	if (c->i_budget_uw > (int64_t)c->i_max * 1000) {
		c->i_budget_uw = (int64_t)c->i_max * 1000;
	} else if (c->i_budget_uw < -(int64_t)c->i_max * 1000) {
		c->i_budget_uw = -(int64_t)c->i_max * 1000;
	}
	int64_t budget = c->s_power + (p_uw + c->i_budget_uw) / 1000;
	budget = budget < c->min_power ? c->min_power : budget > c->max_power ? c->max_power : budget;
	c->prev_budget = (int)budget;

	int total_weight = 0;
	for (int b = 0; b < c->nbind; b++) {
		total_weight += c->bind[b].weight;
	}
	for (int b = 0; b < c->nbind; b++) {
		const struct binding *d = &c->bind[b];
		int cd = d->cdev, cur = c->request[b];
		int64_t mine = budget * d->weight / total_weight;

		mine = mine > cd_power[cd][0] ? cd_power[cd][0] : mine < 0 ? 0 : mine;
		if (d->release_step != ANY_STEP) {                      /* no faster release than its step */
			int step = d->release_step;
			while (cur - step > d->limit[sev] && cur - step > 0 && cd_power[cd][cur - step] == cd_power[cd][cur]) {
				step++;
			}
			int to = cur - step > 0 ? cur - step : 0;
			mine = mine < cd_power[cd][to] ? mine : cd_power[cd][to];
		}
		if (d->throttle_step != ANY_STEP) {                     /* no faster throttling than its step */
			int step = d->throttle_step;
			while (cur + step < d->ceiling[sev] && cd_power[cd][cur + step] == cd_power[cd][cur]) {
				step++;
			}
			int to = cur + step < d->ceiling[sev] ? cur + step : d->ceiling[sev];
			mine = mine > cd_power[cd][to] ? mine : cd_power[cd][to];
		}
		c->request[b] = state_for_power(cd, mine);
	}
}

/*
 * Android's controllers on the skin (every 2 s a look; the PIDs every 7 s while any is above its first level, the
 * HAL's PassiveDelay, and at once when a level changes); each cooling device takes the most throttling of their
 * requests (computeCoolingDevicesRequest: max of the PID's and the hard limit, no more than the ceiling). True if a
 * cooling device's state changed.
 */
static bool
android_thermal_step(void)
{
	int skin = akita_thermal_skin_mc();
	uint64_t now = mach_absolute_time(), seven;
	bool level_changed = false, any = false, changed = false;
	int next[NCDEV] = { 0 };

	if (skin <= -273000) {
		return false;           /* no skin (the thermistors not read): the SoC sensors' caps alone */
	}
	skin += skin_shift_mc;
	for (unsigned i = 0; i < NCTLS; i++) {
		int sev = severity_of(&ctls[i], skin);
		if (sev != ctls[i].severity) {
			printf("akita dvfs: skin %d.%d C: %s at severity %d (was %d)\n", skin / 1000, (skin % 1000) / 100,
			    ctls[i].name, sev, ctls[i].severity);
			ctls[i].severity = sev;
			level_changed = true;
		}
		any |= sev > 0;
	}
	nanoseconds_to_absolutetime(7 * NSEC_PER_SEC, &seven);
	if (!level_changed && (!any || now < pid_last + seven)) {
		return false;
	}
	pid_last = now;
	for (unsigned i = 0; i < NCTLS; i++) {
		struct controller *c = &ctls[i];

		pid_update(c, skin);
		for (int b = 0; b < c->nbind; b++) {
			int want = c->request[b] > c->bind[b].limit[c->severity] ? c->request[b] : c->bind[b].limit[c->severity];
			want = want < c->bind[b].ceiling[c->severity] ? want : c->bind[b].ceiling[c->severity];
			next[c->bind[b].cdev] = want > next[c->bind[b].cdev] ? want : next[c->bind[b].cdev];
		}
	}
	for (int d = 0; d < NCDEV; d++) {
		changed |= next[d] != cd_state[d];
		cd_state[d] = next[d];
	}
	if (changed) {
		printf("akita dvfs: skin %d.%d C: states LITTLE %d MID %d BIG %d GPU %d (budgets", skin / 1000,
		    (skin % 1000) / 100, cd_state[CD_LITTLE], cd_state[CD_MID], cd_state[CD_BIG], cd_state[CD_GPU]);
		for (unsigned i = 0; i < NCTLS; i++) {
			if (ctls[i].severity) {
				printf(" %s %d mW", ctls[i].name, ctls[i].prev_budget);
			}
		}
		printf(")\n");
	}
	return changed;
}

/* the GPU's clocks: the governor's row, no faster than Android's controllers allow (their state is a floor) */
static void
gpu_apply(void)
{
	int row = gpu_want > cd_state[CD_GPU] ? gpu_want : cd_state[CD_GPU];

	if (!gpu_boot[0] || row == gpu_applied) {
		return;
	}
	if (domain_set_khz(8, gpu_rows[row][0]) && domain_set_khz(7, gpu_rows[row][1])) {
		gpu_applied = row;
		dvfs_changes_value++;
	}
}

/*
 * The GPU's governor (every look of the clusters'): its busy share since the last look (hakios.akita_mali.c's busy
 * time); over the row's upper bound a row faster (two when it is saturated), under its lower bound for the row's
 * count of looks a row slower -- the bounds Android's governor uses.
 */
static void
gpu_govern(void)
{
	uint64_t busy = akita_mali_busy_abs(), now = mach_absolute_time();

	if (!gpu_boot[0]) {
		return;
	}
	if (gpu_time_last != 0 && now > gpu_time_last) {
		gpu_util_pct = (int)((busy - gpu_busy_last) * 100 / (now - gpu_time_last));
		gpu_util_pct = gpu_util_pct > 100 ? 100 : gpu_util_pct;
		if (gpu_util_pct > gpu_util[gpu_want][1] && gpu_want > 0) {
			gpu_want -= gpu_util_pct >= 99 && gpu_want > 1 ? 2 : 1;
			gpu_low_looks = 0;
		} else if (gpu_util_pct < gpu_util[gpu_want][0] && gpu_want < (int)NGPUROWS - 1) {
			if (++gpu_low_looks >= gpu_util[gpu_want][2]) {
				gpu_want++;
				gpu_low_looks = 0;
			}
		} else {
			gpu_low_looks = 0;
		}
	}
	gpu_busy_last = busy;
	gpu_time_last = now;
	gpu_apply();
}

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
	if (gpu_boot[0]) {
		int st = gpu_applied;
		uint32_t a = st >= 0 ? gpu_rows[st][0] : gpu_boot[0];
		uint32_t b = st >= 0 ? gpu_rows[st][1] : gpu_boot[1];
		n += snprintf(dvfs_mhz + n, sizeof(dvfs_mhz) - n, " GPU=%u/%u", a / 1000, b / 1000);
	}
	n = 0;
	for (unsigned i = 0; i < NCLUSTERS; i++) {
		struct cluster *c = &clusters[i];
		n += snprintf(dvfs_skin_caps + n, sizeof(dvfs_skin_caps) - n, "%s%s=%u", i ? " " : "", c->name,
		    c->steps[c->nsteps - 1 - cd_state[c->cdev]]);
	}
	snprintf(dvfs_skin_caps + n, sizeof(dvfs_skin_caps) - n, " GPU=%u/%u", gpu_rows[cd_state[CD_GPU]][0] / 1000,
	    gpu_rows[cd_state[CD_GPU]][1] / 1000);
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
	} else if (hot_cap == 1 || (big_cap && c->domain == 4)) {
		cap = (c->nsteps - 1) / 2;
	}
	{
		unsigned thermal = c->nsteps - 1 - (unsigned)cd_state[c->cdev];        /* Android's controllers' ceiling */
		cap = cap > thermal ? thermal : cap;
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
	gpu_boot[0] = domain_get_khz(8);
	gpu_boot[1] = domain_get_khz(7);
	if (gpu_boot[0] == 0 || gpu_boot[1] == 0) {
		printf("akita dvfs: the GPU's clocks could not be read (%u, %u kHz): the GPU is left alone\n", gpu_boot[0],
		    gpu_boot[1]);
		gpu_boot[0] = gpu_boot[1] = 0;
	} else {
		/* the governor starts from the row of the bootloader's shader clock */
		for (gpu_want = 0; gpu_want < (int)NGPUROWS - 1 && gpu_rows[gpu_want][1] > gpu_boot[1]; gpu_want++) {
		}
		gpu_applied = gpu_rows[gpu_want][1] == gpu_boot[1] && gpu_rows[gpu_want][0] == gpu_boot[0] ? gpu_want : -1;
	}
	describe();
	printf("akita dvfs: the clusters' clocks every %u ms (cores 0-%u); now %s\n", period_ms, real_ncpus - 1, dvfs_mhz);
	for (unsigned tick = 0;; tick++) {
		int hot = akita_thermal_hottest_c();
		uint64_t now;

		if (tick % (2000 / period_ms ? 2000 / period_ms : 1) == 0 && android_thermal_step()) {
			gpu_apply();
			describe();
			printf("akita dvfs: ceilings %s\n", dvfs_skin_caps);
		}

		if (hot >= 92) {
			hot_cap = 0;
		} else if (hot >= 85 && hot_cap != 0) {
			hot_cap = 1;
		} else if ((hot_cap == 0 && hot < 87) || (hot_cap == 1 && hot < 80)) {
			hot_cap = hot >= 80 ? 1 : -1;
		}
		big_cap = hot >= 75 || (big_cap && hot >= 70);
		for (unsigned i = 0; i < NCLUSTERS; i++) {
			if (clusters[i].first_cpu < real_ncpus) {
				govern(&clusters[i]);
			}
		}
		{
			int was = gpu_applied;
			gpu_govern();
			if (gpu_applied != was) {
				describe();
			}
		}
		now = mach_absolute_time();
		if (now >= next_log) {
			next_log = now + minute;
			printf("akita dvfs: %s (%d changes; %s); busy LITTLE %d %d %d %d, MID %d %d %d %d, GPU %d %%\n", dvfs_mhz,
			    dvfs_changes_value, hot_cap < 0 ? (big_cap ? "BIG held to its middle: warm" : "not held down") : hot_cap ? "held to the middle: hot" :
			    "held to the lowest: hot", clusters[0].core_share[0], clusters[0].core_share[1],
			    clusters[0].core_share[2], clusters[0].core_share[3], clusters[1].core_share[0],
			    clusters[1].core_share[1], clusters[1].core_share[2], clusters[1].core_share[3], gpu_util_pct);
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
	if (PE_parse_boot_argn("thermal_test_shift", &value, sizeof(value)) && value <= 30) {
		skin_shift_mc = (int)value * 1000;
		printf("akita dvfs: thermal_test_shift=%u: the skin's targets %u degrees lower (a test)\n", value, value);
	}
	start = thread_call_allocate(akita_dvfs_start, NULL);
	clock_interval_to_deadline(5, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_dvfs_init);
