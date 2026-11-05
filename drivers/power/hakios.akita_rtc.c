/*
 * The Pixel 8a's real-time clock, for the kernel's time of day: the main PMIC's (S2MPG14) RTC, read through ACPM
 * (hakios.akita_acpm.c) when the kernel first asks what time it is (GenericARMPE's getGMTTimeOfDay, when neither
 * epoch= nor the loader said). Without it a start with nobody at the computer began in 1970.
 *
 * ACPM's PMIC protocol (channel 2; U-Boot's cmd/hakios_acpm.c and Linux's exynos-acpm-pmic.c): word 0 the register
 * (bits 7:0), the access type (11:8: 2 the RTC) and the bus (15:12: 0 the main PMIC); word 1 the function (7:0: 0 read,
 * 2 update), the value (15:8), the mask (23:16), and in the answer the result (31:24, 0 good) and the value read.
 *
 * The RTC (Linux's rtc-s5m.c for s2mpg10 and its kind, and read on this phone from U-Boot): 0x00 CTRL (bit 1: 24-hour
 * mode, set), 0x01 UPDATE, 0x06 seconds, 0x07 minutes, 0x08 hours (bits 4:0; bit 6 the 24-hour flag), 0x09 the weekday
 * (a bit), 0x0a day, 0x0b month (1-12), 0x0c year since 2000, all binary. The time registers are a buffer: they held the
 * time last written until the running time was latched into them by RUDR (UPDATE bit 0), which is done before each
 * read. UTC: Android keeps it so (hwclock -u).
 *
 * Written back when the time is set (GenericARMPE's setGMTTimeOfDay -> hakios_rtc_set: settimeofday, Settings, network
 * time, idevicedate), from a thread call so that the caller does not wait on ACPM: the seven registers, then the write
 * update bit in UPDATE, which the PMIC clears once it has taken them (Linux's rtc-s5m.c). Which bit that is differs in
 * the family (bit 1 as on the S2MPS15, bit 4 as on the older S2MPS): bit 1 is tried, the running time read back, and
 * bit 4 tried if it did not take; the one that worked is kept.
 */
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/systm.h>

extern int akita_acpm_xfer(unsigned channel, uint32_t cmd[4]);

#define PMIC_CHANNEL            2
#define RTC_TYPE                2
#define PMIC_READ               0
#define PMIC_WRITE              1
#define PMIC_UPDATE             2
#define RTC_UPDATE              0x01
#define RTC_RUDR                0x01
#define RTC_SEC                 0x06

static bool
rtc_access(unsigned func, unsigned reg, unsigned value, unsigned mask, uint8_t *out)
{
	uint32_t cmd[4] = { RTC_TYPE << 8 | (reg & 0xff), func | (value & 0xff) << 8 | (mask & 0xff) << 16, 0, 0 };

	if (akita_acpm_xfer(PMIC_CHANNEL, cmd) != 0 || (cmd[1] >> 24) != 0) {
		return false;
	}
	if (out) {
		*out = (uint8_t)(cmd[1] >> 8);
	}
	return true;
}

/* days from 1970-01-01 to the given date (proleptic Gregorian) */
static int64_t
days_from_civil(int64_t y, unsigned m, unsigned d)
{
	y -= m <= 2;
	int64_t era = (y >= 0 ? y : y - 399) / 400;
	unsigned yoe = (unsigned)(y - era * 400);
	unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
	unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;

	return era * 146097 + (int64_t)doe - 719468;
}

/* the RTC's time in seconds since 1970 (UTC), or 0 if it could not be read or makes no sense */
uint64_t hakios_rtc_epoch(void);

uint64_t
hakios_rtc_epoch(void)
{
	uint8_t t[7];
	unsigned sec, min, hour, day, mon, year;

	if (!rtc_access(PMIC_UPDATE, RTC_UPDATE, RTC_RUDR, RTC_RUDR, NULL)) {
		printf("akita rtc: the PMIC's RTC did not answer through ACPM\n");
		return 0;
	}
	delay(100);
	for (unsigned i = 0; i < 7; i++) {
		if (!rtc_access(PMIC_READ, RTC_SEC + i, 0, 0, &t[i])) {
			printf("akita rtc: register %#x could not be read\n", RTC_SEC + i);
			return 0;
		}
	}
	sec = t[0] & 0x7f;
	min = t[1] & 0x7f;
	hour = t[2] & 0x1f;
	day = t[4] & 0x1f;
	mon = t[5] & 0x0f;
	year = 2000 + (t[6] & 0x7f);
	if (sec > 59 || min > 59 || hour > 23 || day < 1 || day > 31 || mon < 1 || mon > 12 || year < 2020) {
		printf("akita rtc: %04u-%02u-%02u %02u:%02u:%02u makes no sense: not used\n", year, mon, day, hour, min, sec);
		return 0;
	}
	printf("akita rtc: %04u-%02u-%02u %02u:%02u:%02u UTC from the PMIC's clock\n", year, mon, day, hour, min, sec);
	return (uint64_t)(days_from_civil(year, mon, day) * 86400 + hour * 3600 + min * 60 + sec);
}

/* the date of a time in seconds since 1970 (UTC): the reverse of days_from_civil */
static void
civil_from_days(int64_t z, unsigned *y, unsigned *m, unsigned *d)
{
	z += 719468;
	int64_t era = (z >= 0 ? z : z - 146096) / 146097;
	unsigned doe = (unsigned)(z - era * 146097);
	unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
	unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
	unsigned mp = (5 * doy + 2) / 153;

	*d = doy - (153 * mp + 2) / 5 + 1;
	*m = mp < 10 ? mp + 3 : mp - 9;
	*y = (unsigned)(yoe + era * 400 + (*m <= 2));
}

static uint64_t pending_epoch;
static unsigned write_bit;              /* 0 until a write has been seen to take */
static thread_call_t write_call;

static bool
rtc_write_with(uint64_t epoch, unsigned bit)
{
	int64_t days = (int64_t)(epoch / 86400);
	unsigned rest = (unsigned)(epoch % 86400), y, m, d;
	uint8_t t[7];

	civil_from_days(days, &y, &m, &d);
	t[0] = (uint8_t)(rest % 60);
	t[1] = (uint8_t)(rest / 60 % 60);
	t[2] = (uint8_t)(rest / 3600) | 0x40;                   /* 24-hour mode's flag, as the RTC keeps it */
	t[3] = (uint8_t)(1u << ((days + 4) % 7));               /* the weekday, a bit: 1970-01-01 was a Thursday */
	t[4] = (uint8_t)d;
	t[5] = (uint8_t)m;
	t[6] = (uint8_t)(y - 2000);
	for (unsigned i = 0; i < 7; i++) {
		if (!rtc_access(PMIC_WRITE, RTC_SEC + i, t[i], 0xff, NULL)) {
			return false;
		}
	}
	if (!rtc_access(PMIC_UPDATE, RTC_UPDATE, bit, bit, NULL)) {
		return false;
	}
	for (unsigned waited = 0; waited < 100; waited++) {     /* the PMIC clears the bit when it has the time */
		uint8_t u;

		if (rtc_access(PMIC_READ, RTC_UPDATE, 0, 0, &u) && !(u & bit)) {
			break;
		}
		delay(1000);
	}
	return true;
}

static void
rtc_write(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint64_t epoch = pending_epoch, now;
	const unsigned bits[2] = { 0x02, 0x10 };

	for (unsigned i = 0; i < 2; i++) {
		unsigned bit = write_bit ? write_bit : bits[i];

		if (!rtc_write_with(epoch, bit)) {
			printf("akita rtc: writing the time failed (ACPM)\n");
			return;
		}
		now = hakios_rtc_epoch();
		if (now >= epoch && now <= epoch + 5) {
			if (!write_bit) {
				printf("akita rtc: the time is written back to the PMIC's clock (write update bit %#x)\n", bit);
			}
			write_bit = bit;
			return;
		}
		if (write_bit) {
			break;
		}
	}
	printf("akita rtc: the PMIC's clock did not take the time written (it reads %llu, %llu written)\n", now, epoch);
}

/* the time of day was set: the RTC follows, soon, from a thread call */
void hakios_rtc_set(uint64_t epoch);

void
hakios_rtc_set(uint64_t epoch)
{
	if (epoch < 1577836800ULL) {                            /* before 2020: not a time to keep */
		return;
	}
	pending_epoch = epoch;
	if (write_call == NULL) {
		write_call = thread_call_allocate(rtc_write, NULL);
	}
	if (write_call != NULL) {
		thread_call_enter(write_call);
	}
}

/*
 * Before the phone is powered off (hakios.akita_power.c): the PMIC's restart features off, as Google's driver does in
 * its shutdown hook (rtc-s2mpg14.c, s2m_rtc_shutdown -> s2m_rtc_disable_wtsr_smpl): SMPL (sudden momentary power loss,
 * RTC register 0x02 bit 0) and WTSR and its cold reset (register 0x03, bits 0 and 4). Left on -- as the bootloader
 * leaves them -- the PMIC took the power-off for a fault and started the phone again: the tester saw iOS's Shut Down
 * come back to iOS, without a cable.
 */
#define RTC_SMPL                0x02
#define RTC_WTSR                0x03
#define SMPL_EN                 0x01
#define WTSR_EN                 0x01
#define COLDRST_EN              0x10

void hakios_rtc_before_power_off(void);

void
hakios_rtc_before_power_off(void)
{
	uint8_t smpl = 0, wtsr = 0, smpl_after = 0, wtsr_after = 0;
	bool read = rtc_access(PMIC_READ, RTC_SMPL, 0, 0, &smpl) && rtc_access(PMIC_READ, RTC_WTSR, 0, 0, &wtsr);
	bool cleared = rtc_access(PMIC_UPDATE, RTC_SMPL, 0, SMPL_EN, NULL) &&
	    rtc_access(PMIC_UPDATE, RTC_WTSR, 0, WTSR_EN | COLDRST_EN, NULL);

	rtc_access(PMIC_READ, RTC_SMPL, 0, 0, &smpl_after);
	rtc_access(PMIC_READ, RTC_WTSR, 0, 0, &wtsr_after);
	printf("akita rtc: before power off: SMPL %#x -> %#x, WTSR %#x -> %#x%s\n", smpl, smpl_after, wtsr, wtsr_after,
	    read && cleared ? "" : " (an ACPM request failed)");
}
