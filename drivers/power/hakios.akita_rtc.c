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
 */
#include <kern/clock.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/systm.h>

extern int akita_acpm_xfer(unsigned channel, uint32_t cmd[4]);

#define PMIC_CHANNEL            2
#define RTC_TYPE                2
#define PMIC_READ               0
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
