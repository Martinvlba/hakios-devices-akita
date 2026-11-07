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
 *     115 "hot" (BIG's trip points). (Not the common driver's way back to the loader: on this phone that way does not
 *     end yet.)
 *
 * The board's thermistors and the skin (what Android's thermal policy runs on, not the SoC's sensors): eight NTCs on
 * the sub-PMIC S2MPG15's meter (Google's s2mpg15_spmic_thermal.c; the device tree's s2mpg15_spmic_tm1 and its thermal
 * zones), channel 0 modem, 1 quiet, 2 rffe (RF front end), 3 usb_conn, 4 soc, 5 charger, 6 display, 7 gnss_tcxo. Read
 * through ACPM's PMIC protocol (channel 2, as hakios.akita_rtc.c does the main PMIC): the bus 1 (the sub-PMIC's speedy
 * channel), the type 0x0a (its meter), registers 0xd4 + 2n and the next (LPF_DATA_NTCn: 12 bits, the low 4 of the
 * second byte above the first), mapped to degrees by the driver's table. The meter's NTC channels are switched on
 * first if they are not (METER_CTRL3 = 0xff, as the driver does it: 0, CTRL5 0x4f, then 0xff).
 *   The skin is Android's VIRTUAL-SKIN (the phone's /vendor/etc/thermal_info_config.json): the largest of three
 * weighted sums, in millidegrees -- 0.05 quiet + 0.7 soc + 0.25 gnss_tcxo - 220; 0.626 modem + 0.203 rffe + 0.052 soc;
 * 0.15 quiet + 0.55 rffe + 0.15 usb_conn + 0.15 charger - 1000 -- and its severity (Android's 1..6) from 39, 43, 45,
 * 47, 52 and 55 C, each dropped 1.9 degrees lower. Android throttles the processors from 37-43 C of skin (PID
 * controllers aiming at 39, 41, 43; the GPU from 43) and shuts the phone down at severity 6 (55 C). The stops added
 * here, each three readings in a row: the skin at or above thermal_skin_stop (55), soc_therm at or above 58 (its own
 * critical trip in the device tree), the battery at or above 68 (Android's BatteryService). The throttling is
 * hakios.akita_dvfs.c's, from akita_thermal_skin_mc().
 *
 *   - kern.hakios.thermal_skin_mc, thermal_skin_level (0..6), thermal_board ("modem=29 quiet=28 ..."),
 *     thermal_battery (C)
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
#define PMIC_CHANNEL            2
#define SUBPMIC_BUS             1
#define METER_TYPE              0x0a
#define METER_CTRL3             0x0a
#define METER_CTRL5             0x0c
#define METER_NTC0              0xd4
#define NNTC                    8

static const char *const names[NSENSORS] = { "BIG", "MID", "LITTLE", "G3D", "ISP", "TPU", "AUR" };
static uint32_t poll_ms = 2000, log_s = 60, stop_c = 100;
static int temps[NSENSORS];
static int thermal_max_mc = -273000, thermal_reads_value;
static char thermal_hottest[16] = "", thermal_list[128] = "";
static int hot_in_a_row, last_band, failures, last_error;
static uint32_t last_answer[4];
static uint64_t next_log;

static const char *const ntc_names[NNTC] = { "modem", "quiet", "rffe", "usb_conn", "soc", "charger", "display",
	                                     "gnss_tcxo" };
static int ntc_mc[NNTC];                        /* millidegrees (the skin only once every channel was read) */
static bool ntc_ok;                             /* the meter answered and its channels are on */
static int thermal_skin_mc = -273000, thermal_skin_level, thermal_battery_c = -273;
static char thermal_board[160] = "";
static uint32_t skin_stop_c = 55;
static int skin_hot, soc_hot, battery_hot;
static const int skin_levels_mc[6] = { 39000, 43000, 45000, 47000, 52000, 55000 };

/* Google's table: the meter's 12-bit reading (descending) to millidegrees (ascending) */
static const struct { int volt, temp; } ntc_map[] = {
	{ 0xF8D, -26428 }, { 0xF6A, -21922 }, { 0xF29, -15958 }, { 0xEE4, -11060 }, { 0xE9D, -6890 }, { 0xE3F, -2264 },
	{ 0xDBF, 2961 }, { 0xD33, 7818 }, { 0xC97, 12525 }, { 0xBF5, 16945 }, { 0xB3A, 21623 }, { 0xA42, 27431 },
	{ 0x7F1, 40631 }, { 0x734, 44960 }, { 0x66B, 49757 }, { 0x5A3, 54854 }, { 0x4EE, 59898 }, { 0x446, 65076 },
	{ 0x43A, 65779 }, { 0x430, 65856 }, { 0x3C3, 69654 }, { 0x3BD, 69873 }, { 0x33B, 74910 }, { 0x2BB, 80691 },
	{ 0x259, 85844 }, { 0x206, 90915 }, { 0x1CE, 94873 }, { 0x191, 99720 }, { 0x160, 104216 }, { 0x12E, 109531 },
	{ 0xF9, 116445 }, { 0xD7, 121600 }, { 0x9F, 131839 },
};
#define NMAP (sizeof(ntc_map) / sizeof(ntc_map[0]))

SYSCTL_DECL(_kern_hakios);
SYSCTL_INT(_kern_hakios, OID_AUTO, thermal_max_mc, CTLFLAG_RD | CTLFLAG_LOCKED, &thermal_max_mc, 0,
    "the hottest of the SoC's sensors, in millidegrees C");
SYSCTL_STRING(_kern_hakios, OID_AUTO, thermal_hottest, CTLFLAG_RD | CTLFLAG_LOCKED, thermal_hottest, 0,
    "the name of that sensor");
SYSCTL_STRING(_kern_hakios, OID_AUTO, thermal_sensors, CTLFLAG_RD | CTLFLAG_LOCKED, thermal_list, 0,
    "every sensor, in degrees C");
SYSCTL_INT(_kern_hakios, OID_AUTO, thermal_reads, CTLFLAG_RD | CTLFLAG_LOCKED, &thermal_reads_value, 0,
    "how many times the sensors were read");
SYSCTL_INT(_kern_hakios, OID_AUTO, thermal_skin_mc, CTLFLAG_RD | CTLFLAG_LOCKED, &thermal_skin_mc, 0,
    "Android's VIRTUAL-SKIN from the board's thermistors, in millidegrees C");
SYSCTL_INT(_kern_hakios, OID_AUTO, thermal_skin_level, CTLFLAG_RD | CTLFLAG_LOCKED, &thermal_skin_level, 0,
    "the skin's severity, 0..6 (Android's: 6 shuts the phone down)");
SYSCTL_STRING(_kern_hakios, OID_AUTO, thermal_board, CTLFLAG_RD | CTLFLAG_LOCKED, thermal_board, 0,
    "the board's thermistors, in degrees C");
SYSCTL_INT(_kern_hakios, OID_AUTO, thermal_battery, CTLFLAG_RD | CTLFLAG_LOCKED, &thermal_battery_c, 0,
    "the battery's temperature, in degrees C (-273: not known)");

/* for the processors' clocks (hakios.akita_dvfs.c): the hottest sensor in degrees, or -273 before the first reading */
int akita_thermal_hottest_c(void);

int
akita_thermal_hottest_c(void)
{
	return thermal_max_mc / 1000;
}

/* for the clocks: the skin in millidegrees (-273000 while not known), one SoC sensor in degrees (BIG 0 .. AUR 6) */
int akita_thermal_skin_mc(void);
int akita_thermal_sensor_c(unsigned id);
extern int hakios_battery_temperature(void);

int
akita_thermal_skin_mc(void)
{
	return thermal_skin_mc;
}

int
akita_thermal_sensor_c(unsigned id)
{
	return id < NSENSORS && thermal_reads_value ? temps[id] : -273;
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

/* one meter register of the sub-PMIC: read (WRITE false) or written; false if ACPM or the PMIC said no */
static bool
meter_access(bool write, unsigned reg, uint8_t *value)
{
	uint32_t cmd[4] = { SUBPMIC_BUS << 12 | METER_TYPE << 8 | (reg & 0xff), (write ? 1u : 0u) | (uint32_t)*value << 8 |
	    0xffu << 16, 0, 0 };

	if (akita_acpm_xfer(PMIC_CHANNEL, cmd) != 0 || (cmd[1] >> 24) != 0) {
		return false;
	}
	if (!write) {
		*value = (uint8_t)(cmd[1] >> 8);
	}
	return true;
}

static int
ntc_to_mc(int raw)
{
	if (raw >= ntc_map[0].volt) {
		return ntc_map[0].temp;
	}
	for (unsigned i = 1; i < NMAP; i++) {
		if (raw >= ntc_map[i].volt) {
			return ntc_map[i].temp + (ntc_map[i - 1].temp - ntc_map[i].temp) * (raw - ntc_map[i].volt) /
			       (ntc_map[i - 1].volt - ntc_map[i].volt);
		}
	}
	return ntc_map[NMAP - 1].temp;
}

/* the meter's NTC channels on, as Google's driver turns them on; false if the meter does not answer */
static bool
ntc_start(void)
{
	uint8_t v = 0, zero = 0, ctrl5 = 0x4f, all = 0xff;

	if (!meter_access(false, METER_CTRL3, &v)) {
		printf("akita thermal: the sub-PMIC's meter did not answer: no board thermistors, no skin\n");
		return false;
	}
	if (v != 0xff) {
		if (!meter_access(true, METER_CTRL3, &zero)) {
			return false;
		}
		delay_for_interval(1, NSEC_PER_MSEC);
		if (!meter_access(true, METER_CTRL5, &ctrl5)) {
			return false;
		}
		delay_for_interval(1, NSEC_PER_MSEC);
		if (!meter_access(true, METER_CTRL3, &all)) {
			return false;
		}
		delay_for_interval(200, NSEC_PER_MSEC);         /* (the first conversions) */
	}
	printf("akita thermal: the board's 8 thermistors on the sub-PMIC's meter (CTRL3 was 0x%02x)\n", v);
	return true;
}

/* every thermistor, the skin and its level, the battery; their stops counted */
static void
look_board(void)
{
	int n = 0;
	char list[sizeof(thermal_board)];
	bool all = true;

	thermal_battery_c = hakios_battery_temperature();
	if (!ntc_ok) {
		return;
	}
	list[0] = 0;
	for (unsigned i = 0; i < NNTC; i++) {
		uint8_t lo = 0, hi = 0;
		int raw;

		if (!meter_access(false, METER_NTC0 + 2 * i, &lo) || !meter_access(false, METER_NTC0 + 2 * i + 1, &hi) ||
		    (raw = lo | (hi & 0x0f) << 8) == 0) {
			all = false;
			continue;
		}
		ntc_mc[i] = ntc_to_mc(raw);
		n += snprintf(list + n, sizeof(list) - n, "%s%s=%d", n ? " " : "", ntc_names[i], ntc_mc[i] / 1000);
	}
	strlcpy(thermal_board, list, sizeof(thermal_board));
	if (!all && thermal_skin_mc == -273000) {
		return;                 /* not every channel read yet: no skin from part of them */
	}
	/* VIRTUAL-SKIN (millidegrees; the weights in thousandths) */
	int s1 = (50 * ntc_mc[1] + 700 * ntc_mc[4] + 250 * ntc_mc[7]) / 1000 - 220;
	int s2 = (626 * ntc_mc[0] + 203 * ntc_mc[2] + 52 * ntc_mc[4]) / 1000;
	int s3 = (150 * ntc_mc[1] + 550 * ntc_mc[2] + 150 * ntc_mc[3] + 150 * ntc_mc[5]) / 1000 - 1000;
	int skin = s1 > s2 ? s1 : s2;
	skin = s3 > skin ? s3 : skin;
	thermal_skin_mc = skin;
	/* the severity: up at a threshold, down 1.9 degrees under it */
	while (thermal_skin_level < 6 && skin >= skin_levels_mc[thermal_skin_level]) {
		thermal_skin_level++;
	}
	while (thermal_skin_level > 0 && skin < skin_levels_mc[thermal_skin_level - 1] - 1900) {
		thermal_skin_level--;
	}
	skin_hot = skin_stop_c && skin >= (int)skin_stop_c * 1000 ? skin_hot + 1 : 0;
	soc_hot = ntc_mc[4] >= 58000 ? soc_hot + 1 : 0;
	battery_hot = thermal_battery_c >= 68 ? battery_hot + 1 : 0;
}

static void
power_off_because(const char *what, int celsius)
{
	printf("\n--- akita thermal: %s at %d C three times in a row: the phone is powered off (the data volume synced "
	    "first) ---\n", what, celsius);
	sync_internal();
	delay_for_interval(500, NSEC_PER_MSEC);
	smc(PSCI_SYSTEM_OFF);
	printf("akita thermal: PSCI SYSTEM_OFF came back\n");
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
	{
		int level_was = thermal_skin_level;

		look_board();
		if (thermal_skin_level != level_was) {
			printf("akita thermal: skin %d.%d C: severity %d (was %d)\n", thermal_skin_mc / 1000,
			    (thermal_skin_mc % 1000) / 100, thermal_skin_level, level_was);
		}
	}

	nanoseconds_to_absolutetime((uint64_t)log_s * NSEC_PER_SEC, &interval);
	if ((log_s && now >= next_log) || (hottest >= 60 && hottest / 5 > last_band)) {
		next_log = now + interval;
		printf("akita thermal: %s (hottest %s %d C); skin %d.%d C (severity %d), battery %d C; %s\n", thermal_list,
		    thermal_hottest, hottest, thermal_skin_mc / 1000, (thermal_skin_mc % 1000) / 100, thermal_skin_level,
		    thermal_battery_c, thermal_board);
	}
	if (skin_hot >= 3) {
		power_off_because("the skin", thermal_skin_mc / 1000);
	}
	if (soc_hot >= 3) {
		power_off_because("soc_therm (the board by the SoC)", ntc_mc[4] / 1000);
	}
	if (battery_hot >= 3) {
		power_off_because("the battery", thermal_battery_c);
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
	ntc_ok = ntc_start();
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
	if (PE_parse_boot_argn("thermal_skin_stop", &value, sizeof(value)) && value <= 80) {
		skin_stop_c = value;
	}
	start = thread_call_allocate(akita_thermal_start, NULL);
	clock_interval_to_deadline(3, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_thermal_init);
