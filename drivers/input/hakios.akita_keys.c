/*
 * The keys of the Pixel 8a (akita) -- power, volume up, volume down -- as a source of button records for
 * /dev/hakios-touch (sys/hakios.input.h). Only with boot-arg buttons=1.
 *
 * Unlike the reference phone's, all three are pins of the SoC (Android's tree: gpio_keys, button@1..3, gpios on
 * gpa4 and gpa6), of zuma's always-on pin controller (pinctrl@154D0000, google,zuma-pinctrl). Its banks follow one
 * another 0x20 apart in the tree's order (gpa0 gpa1 gpa2 gpa3 gpa4 gpa6 ...: there is no gpa5 and no gap for one), each
 * CON at +0 (4 bits a pin), DAT at +4 (a bit a pin):
 *
 *   power        gpa4-0   0x154d0084 bit 0
 *   volume down  gpa4-1   0x154d0084 bit 1
 *   volume up    gpa6-2   0x154d00a4 bit 2
 *
 * All three are active low (the tree's flags, and what U-Boot reads with nobody touching them: high). Seen from
 * U-Boot on the phone (2026-11-01): CON 0 (inputs) at that point, pull-ups on gpa4-0/1; Android's kernel later makes
 * them interrupt pins (function 0xf). Either way DAT has their level.
 *
 * THIS DRIVER ONLY READS: nothing of the pin controller is written. Before it believes a pin it checks that the pin is
 * an input or an interrupt pin; where it finds something else it says so and leaves that key alone.
 *
 * What a press is: a pin read low, twice in a row, 60 times a second (as the reference phone's driver: hakios.buttons.c).
 * A key that reads as held before it has ever been seen released is not believed. The mapping is the iPhone X's: power
 * is the side button (HAKIOS_BUTTON_LOCK).
 *
 * Status: written 2026-11-01 from the tree and U-Boot's view of the registers; not yet run with a person pressing.
 */
#include <stdint.h>
#include <kern/clock.h>
#include <kern/startup.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/hakios.input.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define ALIVE_PINCTRL           0x154d0000ULL
#define ALIVE_PINCTRL_SIZE      0x1000
#define BANK_GPA4               0x80
#define BANK_GPA6               0xa0
#define BANK_CON                0x0
#define BANK_DAT                0x4
#define POLL_MS                 16

static struct akita_key {
	const char *name;
	uint32_t record;        /* HAKIOS_BUTTON_* */
	uint32_t bank, pin;
	int usable;             /* the pin is an input or an interrupt pin */
	int seen_released;      /* not believed held before it has been seen up */
	int held;               /* as last reported */
	int last;               /* the last look */
} keys[] = {
	{ .name = "power", .record = HAKIOS_BUTTON_LOCK, .bank = BANK_GPA4, .pin = 0 },
	{ .name = "volume down", .record = HAKIOS_BUTTON_VOLUME_DOWN, .bank = BANK_GPA4, .pin = 1 },
	{ .name = "volume up", .record = HAKIOS_BUTTON_VOLUME_UP, .bank = BANK_GPA6, .pin = 2 },
};

static vm_offset_t pinctrl;
static thread_call_t call;

static uint32_t
reg(uint32_t offset)
{
	return *(volatile uint32_t *)(pinctrl + offset);
}

static void
tick(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint64_t deadline;

	for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		struct akita_key *k = &keys[i];
		int down;

		if (!k->usable) {
			continue;
		}
		down = (reg(k->bank + BANK_DAT) & (1u << k->pin)) == 0;        /* active low */
		if (down == k->last && down != k->held) {
			if (!down) {
				k->seen_released = 1;
			}
			if (k->seen_released) {
				k->held = down;
				hakios_touch_post(down ? HAKIOS_BUTTON_DOWN : HAKIOS_BUTTON_UP, k->record, 0, 0);
				printf("akita keys: %s %s\n", k->name, down ? "pressed" : "released");
			}
		}
		k->last = down;
	}
	clock_interval_to_deadline(POLL_MS, NSEC_PER_MSEC, &deadline);
	thread_call_enter_delayed(call, deadline);
}

__startup_func
static void
akita_keys_init(void)
{
	uint32_t enabled = 0;

	if (!PE_parse_boot_argn("buttons", &enabled, sizeof(enabled)) || !enabled) {
		return;                 /* nothing is touched unless a boot-argument asks */
	}
	pinctrl = ml_io_map(ALIVE_PINCTRL, ALIVE_PINCTRL_SIZE);
	if (pinctrl == 0) {
		printf("akita keys: could not map the pin controller at 0x%llx; off\n", ALIVE_PINCTRL);
		return;
	}
	for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); i++) {
		struct akita_key *k = &keys[i];
		uint32_t function = (reg(k->bank + BANK_CON) >> (4 * k->pin)) & 0xf;
		int level = (reg(k->bank + BANK_DAT) >> k->pin) & 1;

		k->usable = (function == 0x0 || function == 0xf);
		k->last = k->held = 0;
		k->seen_released = (level == 1);
		printf("akita keys: %s: bank 0x%x pin %u, function 0x%x, level %d%s\n", k->name, k->bank, k->pin, function,
		    level, k->usable ? "" : ": not an input, left alone");
	}
	call = thread_call_allocate(tick, NULL);
	tick(NULL, NULL);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_keys_init);
