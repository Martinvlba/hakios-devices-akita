/*
 * devices/akita/tests/screen-probe.S: the first thing to run on the Pixel 8a when HakiOS's U-Boot does not get as far
 * as its banner. A bare arm64 Image (Linux's header, as Google's bootloader wants) that says on the screen alone what
 * it found, and then stays: no reset, no return.
 *
 *   the whole screen   the exception level it was started at: green EL2, blue EL1, red anything else
 *   the top 300 rows   DECON0's TRIG_CON (0x19470030) as the bootloader left it: white 0x3070 (masked, as on the
 *                      Pixel 8 Pro; unmasked here with 0x3061), yellow anything else (left alone)
 *   a square, blinking it is still running
 *
 * Nothing on the screen but the phone does not come back by itself: it ran, and 0xfac00000 is not the splash buffer
 * (or the panel needs more than the trigger). The phone comes back by itself within seconds: the bootloader did not
 * start it. Hold the power key to restart (into fastboot, then Android with `fastboot reboot`).
 *
 * Writes nothing but the framebuffer's memory and that one display register. MMU and caches are off at entry (the
 * arm64 boot protocol), so the writes reach RAM, which the display reads.
 *
 * Build: devices/akita/tests/mk-screen-probe.sh
 */
#define FB      0xfac00000
#define WIDTH   1080
#define HEIGHT  2400
#define TRIG    0x19470030

	.text
	.globl _start
_start:
	b	start                   /* code0: past the header */
	.long	0                       /* code1 */
	.quad	0                       /* text_offset */
	.quad	_end - _start           /* image_size */
	.quad	0xa                     /* flags: little-endian, 4 KiB pages, anywhere in RAM */
	.quad	0, 0, 0                 /* reserved */
	.ascii	"ARM\x64"               /* magic */
	.long	0                       /* reserved */

start:
	msr	daifset, #0xf
	/* the colour of the exception level */
	mrs	x0, CurrentEL
	lsr	x0, x0, #2
	ldr	w1, =0xff0000ff         /* EL1: blue */
	cmp	x0, #1
	b.eq	1f
	ldr	w1, =0xff00ff00         /* EL2: green */
	cmp	x0, #2
	b.eq	1f
	ldr	w1, =0xffff0000         /* else: red */
1:
	ldr	x2, =FB
	ldr	x3, =(WIDTH * HEIGHT)
	mov	x4, x2
2:	str	w1, [x4], #4
	subs	x3, x3, #1
	b.ne	2b

	/* the trigger: unmasked only from the known masked value */
	ldr	x5, =TRIG
	ldr	w6, [x5]
	ldr	w7, =0x3070
	ldr	w1, =0xffffff00         /* yellow: something else, left alone */
	cmp	w6, w7
	b.ne	3f
	ldr	w1, =0xffffffff         /* white: 0x3070, unmasked */
3:	ldr	x3, =(WIDTH * 300)
	mov	x4, x2
4:	str	w1, [x4], #4
	subs	x3, x3, #1
	b.ne	4b
	dsb	sy
	cmp	w6, w7
	b.ne	5f
	ldr	w8, =0x3061
	str	w8, [x5]
	dsb	sy

	/* the blinking square, 200 x 200 pixels at row 1000, column 440: black, then magenta, for ever */
5:	ldr	w9, =0xff000000
	ldr	x16, =(WIDTH * 4)          /* a row, in bytes */
	ldr	w10, =0xffff00ff
6:	eor	w9, w9, w10             /* alternate: black <-> magenta... */
	eor	w10, w10, w9
	eor	w9, w9, w10
	ldr	x11, =(FB + (1000 * WIDTH + 440) * 4)
	mov	x12, #200
7:	mov	x13, #200
	mov	x14, x11
8:	str	w9, [x14], #4
	subs	x13, x13, #1
	b.ne	8b
	add	x11, x11, x16
	subs	x12, x12, #1
	b.ne	7b
	dsb	sy
	ldr	x15, =200000000         /* some hundreds of milliseconds, whatever the timer */
9:	subs	x15, x15, #1
	b.ne	9b
	b	6b

	.ltorg
	.balign	4096
_end:
