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
#include <sys/msgbuf.h>
#include <kern/thread.h>
#include <kern/startup.h>
#include <kern/thread_call.h>

extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);

#define PMU_REBOOT_CMD          0x15460810UL
#define PMU_PS_HOLD_CONTROL     0x15463e9cUL    /* PMU 0x15460000 + 0x3e9c */
#define PS_HOLD_OUTPUT_HIGH     (1u << 8)
#define TENSOR_SMC_PMU_SEC_REG  0x82000504
#define PSCI_SYSTEM_OFF         0x84000008
#define PSCI_SYSTEM_RESET       0x84000009
#define PSCI_SYSTEM_RESET2      0xc4000012


/*
 * THE KERNEL'S LOG KEPT FOR ANDROID (the tester, 11-09: "get logs on each reboot to Android"): the ramoops pmsg zone,
 * 0xfd5ff000, 2 MiB -- U-Boot's console is written there (cmd/hakios.c, hakios_pstore) and, after a warm restart into
 * Android, it is /sys/fs/pstore/pmsg-ramoops-0. Every 200 ms what is new in the kernel's message buffer goes after
 * U-Boot's and the loader's text, in Linux's persistent_ram_buffer layout (u32 "DBGC", u32 start, u32 size, the bytes),
 * a ring of the kernel's own after what was there, a marker line where the next byte goes (WRAP_MARK: what follows it is
 * the older part), and the last 64 KB left for the loader's echo of a panic (it appends as U-Boot does). A restart (the
 * USB watchdog, Restart, a panic's way back) keeps it; a power cycle does not. Boot-arg pstore_klog=0: off.
 */
#define PSTORE_PHYS             0xfd5ff000UL
#define PSTORE_SIZE             0x200000UL
#define PSTORE_SIG              0x43474244      /* "DBGC" */
#define PSTORE_KEEP             0x10000         /* for the loader's panic echo, at the end */
static const char WRAP_MARK[] = "\n~~~ HAKIOS KLOG: the older part follows ~~~\n";

struct pstore_buf {
	uint32_t sig, start, size;
	uint8_t data[];
};
static volatile struct pstore_buf *pbuf;
static uint32_t ring_from, ring_end, ring_pos, ring_high;
static int msg_seen = -1;
static bool wrapped;

static void
pstore_klog_copy(void)
{
	struct msgbuf *mb = msgbufp;

	if (pbuf == NULL || mb == NULL || mb->msg_magic != MSG_MAGIC || mb->msg_bufc == NULL || mb->msg_size <= 0) {
		return;
	}
	int x = mb->msg_bufx, n = 0;

	if (msg_seen < 0) {
		msg_seen = (x + 1) % mb->msg_size;    /* (from the oldest byte the ring holds: the whole log so far) */
		while (msg_seen != x && mb->msg_bufc[msg_seen] == 0) {
			msg_seen = (msg_seen + 1) % mb->msg_size;
		}
	}
	while (msg_seen != x && n < (int)(ring_end - ring_from)) {
		pbuf->data[ring_pos++] = (uint8_t)mb->msg_bufc[msg_seen];
		msg_seen = (msg_seen + 1) % mb->msg_size;
		n++;
		if (ring_pos >= ring_end) {
			ring_pos = ring_from;
			wrapped = true;
		}
	}
	if (n == 0) {
		return;
	}
	if (ring_pos > ring_high) {
		ring_high = ring_pos;
	}
	if (wrapped) {
		/* the marker where the next byte goes (overwritten as the log goes on); the zone's size its whole ring */
		uint32_t at = ring_pos;
		for (unsigned i = 0; i < sizeof(WRAP_MARK) - 1 && at < ring_end; i++) {
			pbuf->data[at++] = (uint8_t)WRAP_MARK[i];
		}
		ring_high = ring_end;
	}
	pbuf->size = ring_high;
	pbuf->start = ring_high;        /* (written straight on, as U-Boot leaves it: the reader takes it in order) */
	__asm__ volatile ("dsb sy" ::: "memory");
}

static void
pstore_klog_thread(__unused void *p, __unused wait_result_t w)
{
	for (;;) {
		pstore_klog_copy();
		delay_for_interval(200, NSEC_PER_MSEC);
	}
}

static void
pstore_klog_start(void)
{
	unsigned on = 1;
	thread_t th;

	PE_parse_boot_argn("pstore_klog", &on, sizeof(on));
	if (!on) {
		return;
	}
	pbuf = (volatile struct pstore_buf *)ml_io_map_wcomb(PSTORE_PHYS, PSTORE_SIZE);
	if (pbuf == NULL) {
		return;
	}
	uint32_t room = (uint32_t)(PSTORE_SIZE - sizeof(struct pstore_buf));
	if (pbuf->sig != PSTORE_SIG || pbuf->size > room) {
		pbuf->sig = PSTORE_SIG;         /* (no U-Boot text: a zone of our own from 0) */
		pbuf->size = pbuf->start = 0;
	}
	ring_from = ring_pos = ring_high = pbuf->size;
	ring_end = room - PSTORE_KEEP;
	if (ring_from + 65536 > ring_end) {
		printf("akita: pstore_klog: the zone is full (%u of %u bytes): off\n", ring_from, room);
		pbuf = NULL;
		return;
	}
	static const char head[] = "\n== HAKIOS KERNEL LOG (pstore_klog, every 200 ms) ==\n";
	for (unsigned i = 0; i < sizeof(head) - 1; i++) {
		pbuf->data[ring_pos++] = (uint8_t)head[i];
	}
	ring_from = ring_high = ring_pos;
	if (kernel_thread_start(pstore_klog_thread, NULL, &th) == KERN_SUCCESS) {
		thread_deallocate(th);
		printf("akita: pstore_klog: the kernel's log into the pmsg zone (%u KB for it): Android's pmsg-ramoops-0 after a "
		    "restart\n", (ring_end - ring_from) >> 10);
	}
}

/* (its thread once the kernel is up: from a thread call 2 s after the early start, as the battery's) */
static void
pstore_klog_call(__unused thread_call_param_t a, __unused thread_call_param_t b)
{
	pstore_klog_start();
}

static void
pstore_klog_init(void)
{
	static thread_call_t call;
	uint64_t deadline;

	call = thread_call_allocate(pstore_klog_call, NULL);
	clock_interval_to_deadline(2, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, pstore_klog_init);

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
		pstore_klog_copy();             /* (the last lines kept for Android) */
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
