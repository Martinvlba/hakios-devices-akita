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
#include <sys/sysctl.h>
#include <sys/proc.h>
#include <kern/startup.h>
#include <kern/thread_call.h>
extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define PMU_REBOOT_CMD          0x15460810UL
#define PMU_PS_HOLD_CONTROL     0x15463e9cUL    /* PMU 0x15460000 + 0x3e9c */
#define PS_HOLD_OUTPUT_HIGH     (1u << 8)
#define TENSOR_SMC_PMU_SEC_REG  0x82000504
#define PSCI_SYSTEM_OFF         0x84000008
#define PSCI_SYSTEM_RESET       0x84000009
#define PSCI_SYSTEM_RESET2      0xc4000012


static uint64_t
smc(uint64_t function, uint64_t a1, uint64_t a2, uint64_t a3)
{
	register uint64_t x0 __asm__("x0") = function, x1 __asm__("x1") = a1, x2 __asm__("x2") = a2, x3 __asm__("x3") = a3;

	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : :
	    "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

/*
 * THE START'S TIME BOMB (the tester, 11-14: "add timebomb where device self reboots for these cases" -- a phone whose
 * start never got as far as its home screen could only be got back with his hands on the buttons): boot-arg
 * boot_bomb=SECONDS -- unless user space says within that long that the system is up (sysctl kern.hakios.boot_bomb=0:
 * SpringBoard's inject library, once its main queue has run for 20 s), the phone restarts INTO THE BOOTLOADER
 * (fastboot: the computer reaches it there whatever the flashed image does; the reboot command as
 * hakios.akita_usbmux.c sets it). The file systems are synced first, and since a stuck file system may be why the
 * start stopped, a second timer resets 20 s later without them. Not for a kernel that is stuck whole (no timers run):
 * that needs the SoC's watchdog. The sysctl reads the seconds asked for while armed, 0 once disarmed.
 */
#define REBOOT_BOOTLOADER       0xfc
extern int sync_internal(void);
static thread_call_t bomb_call, bomb_hard_call;
static int bomb_seconds;

static void
bomb_reset(void)
{
	smc(TENSOR_SMC_PMU_SEC_REG, PMU_REBOOT_CMD, 1, REBOOT_BOOTLOADER);
	delay_for_interval(500, NSEC_PER_MSEC);         /* for the log to reach a host */
	smc(PSCI_SYSTEM_RESET2, 0, 0, 0);
	smc(PSCI_SYSTEM_RESET, 0, 0, 0);
}

static void
bomb_hard(__unused thread_call_param_t a, __unused thread_call_param_t b)
{
	if (bomb_seconds == 0) {
		return;
	}
	printf("akita: boot_bomb: the sync did not come back: into the bootloader without it\n");
	bomb_reset();
}

static void
bomb_fire(__unused thread_call_param_t a, __unused thread_call_param_t b)
{
	uint64_t deadline;

	if (bomb_seconds == 0) {
		return;
	}
	printf("akita: boot_bomb: user space did not say it is up within %d s: restarting into the bootloader\n",
	    bomb_seconds);
	clock_interval_to_deadline(20, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(bomb_hard_call, deadline);
	sync_internal();
	bomb_reset();
}

static int
bomb_sysctl SYSCTL_HANDLER_ARGS
{
#pragma unused(oidp, arg1, arg2)
	int value = bomb_seconds, error = sysctl_handle_int(oidp, &value, 0, req);

	if (error == 0 && req->newptr != USER_ADDR_NULL && value == 0 && bomb_seconds != 0) {
		bomb_seconds = 0;
		thread_call_cancel(bomb_call);
		printf("akita: boot_bomb: disarmed (%s says the system is up)\n", proc_best_name(current_proc()));
	}
	return error;
}
SYSCTL_DECL(_kern_hakios);
SYSCTL_PROC(_kern_hakios, OID_AUTO, boot_bomb, CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_ANYBODY | CTLFLAG_LOCKED, NULL, 0,
    bomb_sysctl, "I", "the start's time bomb: seconds asked for while armed; write 0 to disarm");

static void
bomb_init(void)
{
	uint32_t secs = 0;
	uint64_t deadline;

	if (!PE_parse_boot_argn("boot_bomb", &secs, sizeof(secs)) || secs == 0) {
		return;
	}
	bomb_call = thread_call_allocate(bomb_fire, NULL);
	bomb_hard_call = thread_call_allocate(bomb_hard, NULL);
	bomb_seconds = (int)secs;
	clock_interval_to_deadline(secs, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(bomb_call, deadline);
	printf("akita: boot_bomb=%u: into the bootloader after that many seconds unless user space says it is up\n", secs);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, bomb_init);

/*
 * A TEST BUILD'S TIME LIMIT (the tester, 11-09: a phone left in iOS with its USB gone needed someone at home): boot-arg
 * test_reboot_after=SECONDS -- after that long the phone restarts, the file systems synced (reboot_kernel), into the
 * active slot, which is kept Android's: adb reaches it again. bootrun.sh passes 600 (ten minutes); 0 or absent: off.
 */
extern int reboot_kernel(int howto, char *message);

/*
 * restart_fastboot=1 (11-15), from the kernel's start on: the bootloader's reboot command says "fastboot" all through
 * such a boot, not only when this driver restarts the phone -- a reset that comes another way (a watchdog while the
 * file systems are synced, a fault) then ends in the bootloader as well. In the night of 11-15 a quiet test boot
 * whose USB host had stalled was found in the installed image afterwards, the restart's own setting of the command
 * (hakios_platform_halt) notwithstanding.
 */
static void
restart_fastboot_init(void)
{
	uint32_t fastboot = 0;

	if (PE_parse_boot_argn("restart_fastboot", &fastboot, sizeof(fastboot)) && fastboot != 0) {
		smc(TENSOR_SMC_PMU_SEC_REG, PMU_REBOOT_CMD, 1, REBOOT_BOOTLOADER);
		printf("akita: restart_fastboot=1: the bootloader's reboot command is fastboot from now on (reads %#llx)\n",
		    smc(TENSOR_SMC_PMU_SEC_REG, PMU_REBOOT_CMD, 0, 0));
	}
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, restart_fastboot_init);

/*
 * THE POWER DOMAINS NOTHING HERE USES ARE TURNED OFF (11-15). The bootloader leaves all of zuma's power domains on,
 * and no driver of this port uses the 2D scaler (g2d), the camera's processing blocks (gdc, gse, mcsc, rgbp, tnr,
 * yuvp), the video codec (mfc), the two neural accelerators (aur, tpu) or bw: twenty seconds after the start they are
 * turned off as Google's cal-if does it (flexpmu_cal_local_zuma.h, NAME_off): bit 24 of the domain's clock
 * controller's CONTROLLER_OPTION cleared, bit 0 of the domain's CONFIGURATION in the PMU cleared -- the PMU is written
 * through the firmware alone (cal-if's set_priv_reg), and read as it is -- and bit 0 of STATUS waited for. Measured on
 * the battery (charge=0), the phone locked and dark: 506 mA before, 469 mA after (logs day15/pd-off1). Nothing turns
 * them on again in this boot. pd_off=MASK: other domains (a bit for each of the list); pd_off=0: none.
 * Left on: the GPU's, the display's (dpub, dpuf0, dpuf1), aoc (sound), eh, ispfe (the camera's sensors), hsi0 (USB),
 * hsi1 (the Wi-Fi's PCIe).
 */
#define PMU_ALIVE               0x15460000ULL
static const struct { const char *name; uint32_t at; uint64_t cmu; } unused_pd[] = {
	{ "g2d", 0x1e80, 0x1a800000 }, { "gdc", 0x1f00, 0x1d400000 }, { "gse", 0x1f80, 0x1d800000 },
	{ "mcsc", 0x2000, 0x1d000000 }, { "mfc", 0x2080, 0x1a400000 }, { "rgbp", 0x2100, 0x1c400000 },
	{ "tnr", 0x2180, 0x1cc00000 }, { "yuvp", 0x2200, 0x1c800000 }, { "aur", 0x2300, 0x20a00000 },
	{ "bw", 0x2380, 0x1a600000 }, { "tpu", 0x2500, 0x1a300000 },
};
static uint32_t pd_off_mask = 0x7ff;
static thread_call_t pd_off_call;

static void
pd_off_run(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	vm_offset_t pmu = ml_io_map(PMU_ALIVE, 0x4000);
	char line[256];
	int n = 0;

	if (pmu == 0) {
		return;
	}
	line[0] = 0;
	for (unsigned i = 0; i < sizeof(unused_pd) / sizeof(unused_pd[0]); i++) {
		volatile uint32_t *configuration = (volatile uint32_t *)(pmu + unused_pd[i].at), *status = configuration + 1;
		vm_offset_t cmu;
		int waited = 0;

		if ((pd_off_mask & (1u << i)) == 0 || (*status & 1) == 0) {
			continue;
		}
		cmu = ml_io_map(unused_pd[i].cmu, 0x1000);
		if (cmu == 0) {
			continue;
		}
		*(volatile uint32_t *)(cmu + 0x800) &= ~(1u << 24);
		(void)smc(TENSOR_SMC_PMU_SEC_REG, PMU_ALIVE + unused_pd[i].at, 1, *configuration & ~1u);
		while ((*status & 1) != 0 && waited < 5000) {
			delay(1);
			waited++;
		}
		n += snprintf(line + n, sizeof(line) - n, " %s%s", unused_pd[i].name, (*status & 1) ? " (STILL ON)" : "");
	}
	printf("akita: power domains nothing here uses, turned off:%s\n", n ? line : " none");
}

static void
pd_off_init(void)
{
	uint64_t deadline;

	PE_parse_boot_argn("pd_off", &pd_off_mask, sizeof(pd_off_mask));
	if (pd_off_mask == 0) {
		return;
	}
	pd_off_call = thread_call_allocate(pd_off_run, NULL);
	clock_interval_to_deadline(20, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(pd_off_call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, pd_off_init);

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
		/*
		 * Boot-arg restart_fastboot=1 (11-15; tools/bootrun.sh gives it with QUIET=1): every restart of this boot
		 * goes into the bootloader, not into the active slot -- a test boot that restarts by itself (its time limit,
		 * a USB host gone quiet) started the installed image, which plays its start sound and buzzes: in the night,
		 * twice. The bootloader is silent, and the computer reaches it.
		 */
		uint32_t fastboot = 0;

		if (PE_parse_boot_argn("restart_fastboot", &fastboot, sizeof(fastboot)) && fastboot != 0) {
			printf("akita: restart: into the bootloader (restart_fastboot=1), a warm reset\n");
			smc(TENSOR_SMC_PMU_SEC_REG, PMU_REBOOT_CMD, 1, REBOOT_BOOTLOADER);
			smc(PSCI_SYSTEM_RESET2, 0, 0, 0);
			smc(PSCI_SYSTEM_RESET, 0, 0, 0);
		}
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
