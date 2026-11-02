/*
 * The Pixel 8a's SoC temperatures (thermal=1): sysctl kern.hakios.thermal_*, a line in the log, and a stop for a phone
 * getting too hot -- what drivers/power/hakios.thermal.c is for the Qualcomm phones, here for zuma.
 *
 * The sensors: zuma's TMU blocks (Android's tree: samsung,gs-tmu-v3; BIG id 0, MID 1, LITTLE 2, G3D 3, ISP 4, TPU 5,
 * AUR 6) are read by ACPM, the power firmware, and asked over its IPC (acpm_tmu, channel 9; hakios.akita_acpm.c): a
 * request's word 1 is 0x02 | id << 16 (read the temperature), the answer's word 1 is type, result (0: good), id, and the
 * temperature in whole degrees in its top byte. Found on the phone with U-Boot's `acpm raw` (36-37 C at rest, Android
 * 35 C just before). Nothing is written: no threshold set, nothing throttled -- the firmware's own limits are as they
 * were.
 *
 *   - every thermal_poll milliseconds (2000) all seven are read;
 *   - kern.hakios.thermal_max_mc (the hottest, millidegrees), thermal_hottest (its name), thermal_sensors ("BIG=37
 *     MID=37 ..."), thermal_reads;
 *   - a line in the log every thermal_log seconds (60), and at once when the hottest crosses a multiple of 5 upwards
 *     from 60;
 *   - the stop: the hottest at or above thermal_stop degrees (100; 0: never) three readings in a row, and the phone is
 *     powered off -- the data volume synced first, then PSCI SYSTEM_OFF. Google slows the big cores at 90 and calls
 *     115 "hot" (BIG's trip points); nothing slows anything here yet, so the stop is the whole protection, for a phone
 *     left running. (Not the common driver's way back to the loader: on this phone that way does not end yet.)
 */
#include <kern/clock.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

extern int akita_acpm_xfer(unsigned channel, uint32_t cmd[4]);
extern int sync_internal(void);

#define TMU_CHANNEL             9
#define TMU_READ_TEMP           0x02
#define PSCI_SYSTEM_OFF         0x84000008
#define NSENSORS                7

static const char *const names[NSENSORS] = { "BIG", "MID", "LITTLE", "G3D", "ISP", "TPU", "AUR" };
static uint32_t poll_ms = 2000, log_s = 60, stop_c = 100;
static int temps[NSENSORS];
static int thermal_max_mc = -273000, thermal_reads_value;
static char thermal_hottest[16] = "", thermal_list[128] = "";
static int hot_in_a_row, last_band, failures, last_error;
static uint32_t last_answer[4];
static uint64_t next_log;

SYSCTL_DECL(_kern_hakios);
SYSCTL_INT(_kern_hakios, OID_AUTO, thermal_max_mc, CTLFLAG_RD | CTLFLAG_LOCKED, &thermal_max_mc, 0,
    "the hottest of the SoC's sensors, in millidegrees C");
SYSCTL_STRING(_kern_hakios, OID_AUTO, thermal_hottest, CTLFLAG_RD | CTLFLAG_LOCKED, thermal_hottest, 0,
    "the name of that sensor");
SYSCTL_STRING(_kern_hakios, OID_AUTO, thermal_sensors, CTLFLAG_RD | CTLFLAG_LOCKED, thermal_list, 0,
    "every sensor, in degrees C");
SYSCTL_INT(_kern_hakios, OID_AUTO, thermal_reads, CTLFLAG_RD | CTLFLAG_LOCKED, &thermal_reads_value, 0,
    "how many times the sensors were read");

/* for the processors' clocks (hakios.akita_dvfs.c): the hottest sensor in degrees, or -273 before the first reading */
int akita_thermal_hottest_c(void);

int
akita_thermal_hottest_c(void)
{
	return thermal_max_mc / 1000;
}

static uint64_t
smc(uint64_t function)
{
	register uint64_t x0 __asm__("x0") = function, x1 __asm__("x1") = 0, x2 __asm__("x2") = 0, x3 __asm__("x3") = 0;

	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : :
	    "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

/* one sensor in whole degrees, or false */
static bool
read_sensor(unsigned id, int *celsius)
{
	uint32_t cmd[4] = { 0, TMU_READ_TEMP | id << 16, 0, 0 };

	last_error = akita_acpm_xfer(TMU_CHANNEL, cmd);
	memcpy(last_answer, cmd, sizeof(last_answer));
	if (last_error != 0 || (cmd[1] & 0xff) != TMU_READ_TEMP || ((cmd[1] >> 8) & 0xff) != 0 ||
	    ((cmd[1] >> 16) & 0xff) != id) {
		return false;
	}
	*celsius = (int)(cmd[1] >> 24);
	return *celsius > 0 && *celsius < 160;
}

static void
power_off(int hottest)
{
	printf("\n--- akita thermal: %s at %d C, at or above thermal_stop=%u three times in a row: the phone is powered off "
	    "(the data volume synced first) ---\n", thermal_hottest, hottest, stop_c);
	sync_internal();
	delay_for_interval(500, NSEC_PER_MSEC);
	smc(PSCI_SYSTEM_OFF);
	printf("akita thermal: PSCI SYSTEM_OFF came back\n");
}

static void
look(void)
{
	int hottest = -273, which = -1, n = 0;
	char list[sizeof(thermal_list)];
	uint64_t now = mach_absolute_time(), interval;

	list[0] = 0;
	for (unsigned i = 0; i < NSENSORS; i++) {
		int t;

		if (!read_sensor(i, &t)) {
			if (failures++ < 5) {
				printf("akita thermal: sensor %s did not answer\n", names[i]);
			}
			continue;
		}
		temps[i] = t;
		n += snprintf(list + n, sizeof(list) - n, "%s%s=%d", n ? " " : "", names[i], t);
		if (t > hottest) {
			hottest = t;
			which = (int)i;
		}
	}
	if (which < 0) {
		nanoseconds_to_absolutetime((uint64_t)(log_s ? log_s : 60) * NSEC_PER_SEC, &interval);
		if (now >= next_log) {
			next_log = now + interval;
			printf("akita thermal: no sensor answered (error %d, the last answer %08x %08x %08x %08x)\n", last_error,
			    last_answer[0], last_answer[1], last_answer[2], last_answer[3]);
		}
		return;
	}
	strlcpy(thermal_list, list, sizeof(thermal_list));
	strlcpy(thermal_hottest, names[which], sizeof(thermal_hottest));
	thermal_max_mc = hottest * 1000;
	thermal_reads_value++;

	nanoseconds_to_absolutetime((uint64_t)log_s * NSEC_PER_SEC, &interval);
	if ((log_s && now >= next_log) || (hottest >= 60 && hottest / 5 > last_band)) {
		next_log = now + interval;
		printf("akita thermal: %s (hottest %s %d C)\n", thermal_list, thermal_hottest, hottest);
	}
	last_band = hottest >= 60 ? hottest / 5 : 0;
	hot_in_a_row = stop_c && hottest >= (int)stop_c ? hot_in_a_row + 1 : 0;
	if (hot_in_a_row >= 3) {
		power_off(hottest);
	}
}

static void
akita_thermal_thread(__unused void *parameter, __unused wait_result_t result)
{
	printf("akita thermal: the SoC's 7 sensors through ACPM every %u ms; the phone powered off at %u C%s\n", poll_ms,
	    stop_c, stop_c ? "" : " (never: thermal_stop=0)");
	for (;;) {
		look();
		delay_for_interval(poll_ms, NSEC_PER_MSEC);
	}
}

static void
akita_thermal_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(akita_thermal_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

__startup_func
static void
akita_thermal_init(void)
{
	uint32_t enabled = 0, value;
	uint64_t deadline;
	static thread_call_t start;

	if (!PE_parse_boot_argn("thermal", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	if (PE_parse_boot_argn("thermal_poll", &value, sizeof(value)) && value >= 200 && value <= 60000) {
		poll_ms = value;
	}
	if (PE_parse_boot_argn("thermal_log", &value, sizeof(value)) && value <= 86400) {
		log_s = value;
	}
	if (PE_parse_boot_argn("thermal_stop", &value, sizeof(value)) && value <= 150) {
		stop_c = value;
	}
	start = thread_call_allocate(akita_thermal_start, NULL);
	clock_interval_to_deadline(3, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_thermal_init);
