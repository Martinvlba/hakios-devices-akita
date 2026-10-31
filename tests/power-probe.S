/*
 * devices/akita/tests/power-probe.S: does Google's bootloader run what `fastboot boot` gives it at all? A bare arm64
 * Image whose one act is a PSCI call by SMC: SYSTEM_OFF (-DPROBE_OFF, the default) or nothing but a loop (-DPROBE_HANG).
 *
 *   OFF:  the phone switches off and stays off            -> it ran (start it again with the power key)
 *         it is back in fastboot after a few seconds        -> it did not run, or the monitor refused the call
 *   HANG: the phone stays away (no USB) for minutes         -> it runs, and nothing resets it
 *         it is back in fastboot after a few seconds        -> something resets it: a watchdog the bootloader left on
 *
 * Build: devices/akita/tests/mk-screen-probe.sh (it makes these too).
 */
	.text
	.globl _start
_start:
	b	start
	.long	0
	.quad	0
	.quad	_end - _start
	.quad	0xa
	.quad	0, 0, 0
	.ascii	"ARM\x64"
	.long	0
start:
	msr	daifset, #0xf
#ifdef PROBE_HANG
1:	wfe
	b	1b
#else
	mov	x0, #0x0008
	movk	x0, #0x8400, lsl #16    /* PSCI SYSTEM_OFF */
	smc	#0
1:	b	1b                      /* if it returns: stay */
#endif
	.balign	4096
_end:
