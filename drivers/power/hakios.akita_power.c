/*
 * The Pixel 8a's way down: iOS's Shut Down powers the phone off, Restart starts it again, instead of the way back to
 * the loader -- which on this phone stalls, and left the spinner on the screen for good.
 *
 * hakios_platform_halt (osfmk/arm64/machine_routines.c, halt_all_cpus, once the file systems are down): a restart sets
 * the bootloader's reboot command to a normal start (PMU + 0x810 = 0, through the PMU's secure-register SMC, as
 * hakios.akita_usbmux.c does for the bootloader) and resets warm (PSCI SYSTEM_RESET2 type 0, then SYSTEM_RESET if that
 * comes back); a halt turns the PMIC's restart features off (SMPL, WTSR: hakios.akita_rtc.c, as Google's PMIC RTC
 * driver does at shutdown -- without it the phone started again, with or without a cable) and is then PSCI SYSTEM_OFF, as
 * Android's on zuma is; if that comes back, the PMIC's power hold is let go (PMU + 0x3e9c bit 8, as Linux's gs101). With loader_return_after among the arguments (a run of xl's, from the computer) it returns, and the loader
 * gets the run and its log back as before.
 */
#include <kern/clock.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <kern/startup.h>
#include <kern/thread_call.h>

#define PMU_REBOOT_CMD          0x15460810UL
#define PMU_PS_HOLD_CONTROL     0x15463e9cUL    /* PMU 0x15460000 + 0x3e9c */
#define PS_HOLD_OUTPUT_HIGH     (1u << 8)
#define TENSOR_SMC_PMU_SEC_REG  0x82000504
#define PSCI_SYSTEM_OFF         0x84000008
#define PSCI_SYSTEM_RESET       0x84000009
#define PSCI_SYSTEM_RESET2      0xc4000012


/*
 * A TEST BUILD'S TIME LIMIT (the tester, 11-09: a phone left in iOS with its USB gone needed someone at home): boot-arg
 * test_reboot_after=SECONDS -- after that long the phone restarts, the file systems synced (reboot_kernel), into the
 * active slot, which is kept Android's: adb reaches it again. bootrun.sh passes 600 (ten minutes); 0 or absent: off.
 */
extern int reboot_kernel(int howto, char *message);

static void
test_time_up(__unused thread_call_param_t a, __unused thread_call_param_t b)
{
	printf("akita: test_reboot_after: the test build's time is up: restarting (to the active slot)\n");
	(void)reboot_kernel(0 /* RB_AUTOBOOT */, (char *)"akita: the test build's time limit");
}

static void
test_limit_init(void)
{
	static thread_call_t call;
	uint32_t secs = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("test_reboot_after", &secs, sizeof(secs)) || secs == 0) {
		return;
	}
	call = thread_call_allocate(test_time_up, NULL);
	clock_interval_to_deadline(secs, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(call, deadline);
	printf("akita: test_reboot_after=%u: the phone restarts after that many seconds\n", secs);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, test_limit_init);

static uint64_t
smc(uint64_t function, uint64_t a1, uint64_t a2, uint64_t a3)
{
	register uint64_t x0 __asm__("x0") = function, x1 __asm__("x1") = a1, x2 __asm__("x2") = a2, x3 __asm__("x3") = a3;

	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : :
	    "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

void hakios_platform_halt(int reboot);
void hakios_rtc_before_power_off(void);     /* hakios.akita_rtc.c */

void
hakios_platform_halt(int reboot)
{
	uint32_t after = 0;

	if (PE_parse_boot_argn("loader_return_after", &after, sizeof(after))) {
		return;                 /* xl's run: the loader takes it back */
	}
	if (reboot) {
		printf("akita: restart: a normal start next (reboot command 0), a warm reset\n");
		smc(TENSOR_SMC_PMU_SEC_REG, PMU_REBOOT_CMD, 1, 0);
		smc(PSCI_SYSTEM_RESET2, 0, 0, 0);
		smc(PSCI_SYSTEM_RESET, 0, 0, 0);
	} else {
		/* the PMIC's restart features off first (else it starts the phone again), then Android's own way off on zuma:
		 * PSCI SYSTEM_OFF (Google's exynos-zuma-reboot.c leaves the power-off to PSCI) */
		hakios_rtc_before_power_off();
		printf("akita: shut down: PSCI SYSTEM_OFF\n");
		smc(TENSOR_SMC_PMU_SEC_REG, PMU_REBOOT_CMD, 1, 0);     /* (a normal start when it is switched on again) */
		smc(PSCI_SYSTEM_OFF, 0, 0, 0);
		/* (came back: let go of the PMIC's power hold, PMU + 0x3e9c bit 8, as Linux's gs101 does) */
		uint64_t hold = smc(TENSOR_SMC_PMU_SEC_REG, PMU_PS_HOLD_CONTROL, 0, 0);

		printf("akita: shut down: PSCI SYSTEM_OFF came back; PS_HOLD let go (PMU + 0x3e9c %#llx)\n", hold);
		smc(TENSOR_SMC_PMU_SEC_REG, PMU_PS_HOLD_CONTROL, 1, hold & ~(uint64_t)PS_HOLD_OUTPUT_HIGH);
		delay(500000);
	}
	printf("akita: PSCI came back: the way back to the loader\n");
}
