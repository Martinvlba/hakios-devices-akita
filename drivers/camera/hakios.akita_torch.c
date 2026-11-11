/*
 * The Pixel 8a's flashlight: a TI LM3644 (dual flash LED driver) at 0x63 on zuma's hsi2c@10980000 -- HSI2C14, USI14 of
 * PERIC0 -- as Android's device tree has it (devices/akita/boot-info/fdt.dtb: flash@0, "flash-lm3644", a LWIS I2C device
 * of the camera HAL; i2c-bus <&hsi2c_14>, i2c-addr 0x63, enable-gpios gpp8-2 active high and gpp24-0 active low).
 *
 * Android drives the flash from its camera HAL through LWIS, so nothing in Linux sets the bus up for us before the camera
 * does, and the bootloader does not either: here the clocks of USI14 are held on (CMU_PERIC0: Google's cal-if zuma
 * cmucal-sfr.c), the USI put in I2C mode (sysreg_peric0 + 0x28: samsung,usi-offset) and out of reset, its pins given to
 * the I2C (gpp16-0/1, function 2: pinctrl hsi2c14-bus) and the controller set up as U-Boot's exynos_hs_i2c.c does it (the
 * Exynos 7 variant, fast mode from the crystal, 24.576 MHz). Pin banks: Google's pinctrl-gs.c, zuma_pin_peric0/peric1.
 *
 * The LM3644 (TI's datasheet, SNVSA22): 0x01 ENABLE (bits 1:0 LED2/LED1 on, 3:2 the mode: 00 standby, 10 torch, 11
 * flash), 0x05/0x06 LED1/LED2 torch brightness (7 bits; bit 7 of 0x05: LED2 takes LED1's current), 0x08 timing (the torch
 * ramp in bits 6:4), 0x0a/0x0b the flags (read clears them: shorts, thermal shutdown, UVLO, timeouts), 0x0c the device
 * id. Torch current: code x 1.4 mA + 0.977 mA (LM3644; up to ~179 mA a LED).
 *
 * What Darwin's flashlight is: the loader says the camera has a flash when torch=N is given (/product/camera "flash":
 * Control Center's flashlight button) and libhakios.media.dylib in mediaserverd hands FigFlashlight a device whose torch
 * level goes to /dev/hakios-torch (userland/src/inject_media.c) -- the same node and the same 0..1000 as the POCO X5
 * Pro's driver (drivers/camera/hakios.torch.c), which this replaces on this phone.
 *
 * Nothing is done unless asked:
 *   torch=1    a look: the clocks, the USI's mode and the pins as found; then the bus brought up and the chip's registers
 *              read -- nothing written to the chip
 *   torch=2    ... and the light on once, LED1 at torch_ma= (default 50) for torch_secs (2), the flags read, then off
 *   torch=3    the light for Darwin until the run ends: /dev/hakios-torch takes 0 to 1000 (a 32-bit number, thousandths
 *              of torch_max_ma, default 100 mA, never more than 150) and the LED is set to it; a read gives what is lit
 * Limits whatever asks: at most torch_max_s (30, never more than 120) on at a time, then off until asked anew (0 first);
 * while on, the chip's flags ten times a second and the skin temperature (off above 45 C): a fault, a read or write
 * that fails, and the light goes off for the rest of the run. Off at the run's end and on a panic's way out.
 */
#include <kern/clock.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>
#include <sys/conf.h>
#include <sys/systm.h>
#include <sys/uio.h>
#include <sys/sysctl.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* ---- the addresses ------------------------------------------------------------------------------------------------ */

#define CMU_PERIC0_PHYS         0x10800000ULL
#define CMU_MUX_USI14           0x0640          /* PLL_CON0_MUX_CLKCMU_PERIC0_USI14_USI_USER: bit 4, 1 = CMU_TOP's clock */
#define CMU_MUX1_USI14          0x0644
#define CMU_DIV_USI14           0x180c          /* CLK_CON_DIV_DIV_CLK_PERIC0_USI14_USI: the ratio less one, bits 3:0 */
#define CMU_GATE_IPCLK_USI14    0x2068
#define CMU_GATE_PCLK_USI14     0x206c
#define CMU_GATE_RSTNSYNC_USI14 0x20cc
#define CMU_QCH_USI14           0x3070
#define CMU_GATE_ON             0x00300000      /* MANUAL (bit 20) and CG_VAL (bit 21): held on */
#define SYSREG_PERIC0_PHYS      0x10821000ULL
#define SYSREG_USI14_MODE       0x28
#define USI_MODE_I2C            0x4
#define GPIO_PERIC0_PHYS        0x10840000ULL
#define GPP8                    0x100           /* CON +0, DAT +4, PUD +8, DRV +0xc */
#define GPP16                   0x220
#define GPIO_PERIC1_PHYS        0x10c40000ULL
#define GPP24                   0x060
#define HSI2C14_PHYS            0x10980000ULL
#define HSI2C_BATTERY_PHYS      0x10cb0000ULL   /* (a bus U-Boot set up, the battery's: its registers for comparison) */
#define CRYSTAL_HZ              24576000u
#define BUS_HZ                  390000u         /* the tree's clock-frequency */

/* the HSI2C (U-Boot's struct exynos5_hsi2c) and its USI block */
#define CTL                     0x00
#define FIFO_CTL                0x04
#define TRAILING_CTL            0x08
#define INT_EN                  0x20
#define INT_STAT                0x24
#define FIFO_STAT               0x30
#define TX_DATA                 0x34
#define RX_DATA                 0x38
#define CONF                    0x40
#define AUTO_CONF               0x44
#define TIMEOUT                 0x48
#define TRANS_STATUS            0x50
#define TIMING_FS1              0x60
#define TIMING_FS2              0x64
#define TIMING_FS3              0x68
#define TIMING_SLA              0x6c
#define ADDR                    0x70
#define ERROR_STAT              0x2c
#define USI_CON                 0xc4
#define USI_OPTION              0xc8
#define USI_CON_RESET           (1u << 0)
#define USI_OPTION_CLKREQ_ON    (1u << 1)
#define USI_OPTION_CLKSTOP_ON   (1u << 2)
#define CTL_FUNC_MODE_I2C       (1u << 0)
#define CTL_MASTER              (1u << 3)
#define CTL_RXCHON              (1u << 6)
#define CTL_TXCHON              (1u << 7)
#define CTL_SW_RST              (1u << 31)
#define FIFO_RX_EN              (1u << 0)
#define FIFO_TX_EN              (1u << 1)
#define CONF_AUTO_MODE          (1u << 31)
#define AUTO_READ               (1u << 16)
#define AUTO_STOP               (1u << 17)
#define AUTO_RUN                (1u << 31)
#define TIMEOUT_EN              (1u << 31)
#define INT_TRANS_EN            0x1f80          /* the Exynos 7's completion and error reports (U-Boot HSI2C_INT_I2C_TRANS_EN) */
#define INT_DONE                (1u << 7)
#define INT_ABORT               (1u << 8)
#define INT_NO_ACK              (1u << 9)
#define INT_NO_DEV              (1u << 10)
#define INT_TIMEOUT             (1u << 11)
#define INT_ALL                 (INT_DONE | INT_ABORT | INT_NO_ACK | INT_NO_DEV | INT_TIMEOUT)
#define FIFO_TX_FULL            (1u << 7)
#define FIFO_RX_EMPTY           (1u << 24)
#define MASTER_BUSY             (1u << 17)
#define TRAILING_COUNT          0xf
#define WAIT_US                 10000

#define LM3644                  0x63
#define REG_ENABLE              0x01
#define REG_LED1_TORCH          0x05
#define REG_LED2_TORCH          0x06
#define REG_TIMING              0x08
#define REG_FLAGS1              0x0a
#define REG_FLAGS2              0x0b
#define REG_DEVICE_ID           0x0c
#define ENABLE_LED1             (1u << 0)
#define MODE_TORCH              (2u << 2)
#define ENABLE_DEFAULT          0x80            /* TX pin enabled, standby: the chip's reset value */

static vm_offset_t cmu, sysreg, gpio0, gpio1, i2c, i2c_battery;
static uint32_t level, errors;
static bool up, broken, lit;

static inline uint32_t rd(vm_offset_t a) { return *(volatile uint32_t *)a; }
static inline void wr(vm_offset_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

/* ---- the bus -------------------------------------------------------------------------------------------------------- */

static bool
wait_for(uint32_t off, uint32_t mask, uint32_t want)
{
	for (unsigned i = 0; i < WAIT_US; i++) {
		if ((rd(i2c + off) & mask) == want) {
			return true;
		}
		delay(1);
	}
	return false;
}

/* the end of a transfer: INT_STAT's report (done, or what went wrong), within 50 ms (TRANS_STATUS's low bits are the
 * Exynos 7 master's state, not a result -- 6 is ADDR1, 3 STOP: misread as errors once, 11-11) */
static uint32_t
transfer_end(void)
{
	uint64_t t0 = mach_absolute_time(), ns = 0;

	while (ns < 50 * NSEC_PER_MSEC) {
		uint32_t s = rd(i2c + INT_STAT);

		if (s & INT_ALL) {
			wr(i2c + INT_STAT, s);
			return s & (INT_ABORT | INT_NO_ACK | INT_NO_DEV | INT_TIMEOUT) ? s : 0;
		}
		delay(2);
		absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	}
	return ~0u;
}

static bool
transfer(uint8_t addr, bool read, uint8_t *buf, unsigned len, bool stop)
{
	uint32_t end;
	bool ok = true;

	wr(i2c + TIMEOUT, rd(i2c + TIMEOUT) & ~TIMEOUT_EN);
	wr(i2c + ADDR, (uint32_t)(addr & 0x3ff) << 10);
	wr(i2c + CTL, (read ? CTL_RXCHON : CTL_TXCHON) | CTL_FUNC_MODE_I2C | CTL_MASTER);
	wr(i2c + INT_STAT, INT_ALL);
	wr(i2c + AUTO_CONF, len | AUTO_RUN | (stop ? AUTO_STOP : 0) | (read ? AUTO_READ : 0));
	for (unsigned i = 0; i < len && ok; i++) {
		if (read) {
			ok = wait_for(FIFO_STAT, FIFO_RX_EMPTY, 0);
			if (ok) {
				buf[i] = (uint8_t)rd(i2c + RX_DATA);
			}
		} else {
			ok = wait_for(FIFO_STAT, FIFO_TX_FULL, 0);
			if (ok) {
				wr(i2c + TX_DATA, buf[i]);
			}
		}
	}
	end = transfer_end();
	if (stop) {
		wait_for(TRANS_STATUS, MASTER_BUSY, 0);
	}
	wr(i2c + CTL, CTL_FUNC_MODE_I2C);
	if (!ok || end) {
		if (errors++ < 8) {
			printf("akita torch: I2C %s of %u bytes at %#x failed (%s, INT_STAT %#x): now INT_STAT %#x FIFO_STAT %#x "
			    "TRANS_STATUS %#x ERROR_STAT %#x CTL %#x CONF %#x AUTO_CONF %#x\n", read ? "read" : "write", len, addr,
			    ok ? "the end" : "the FIFO", end, rd(i2c + INT_STAT), rd(i2c + FIFO_STAT), rd(i2c + TRANS_STATUS),
			    rd(i2c + ERROR_STAT), rd(i2c + CTL), rd(i2c + CONF), rd(i2c + AUTO_CONF));
		}
		return false;
	}
	return true;
}

static bool
get(uint8_t reg, uint8_t *value)
{
	return transfer(LM3644, false, &reg, 1, false) && transfer(LM3644, true, value, 1, true);
}

/* every write read back; a surprise ends writing for the run (but for the way to dark: force) */
static bool
put(uint8_t reg, uint8_t value, bool force)
{
	uint8_t b[2] = { reg, value }, back = 0;

	if (broken && !force) {
		return false;
	}
	if (!transfer(LM3644, false, b, 2, true) || !get(reg, &back) || back != value) {
		printf("akita torch: register %#x written %#x, reads %#x: given up\n", reg, value, back);
		broken = true;
		return false;
	}
	return true;
}

static void
say_regs(const char *when)
{
	uint8_t r[14] = { 0 };
	bool ok = true;

	for (unsigned i = 1; i < sizeof(r) && ok; i++) {
		ok = get((uint8_t)i, &r[i]);
	}
	printf("akita torch: LM3644 %s%s: enable %#x ivfm %#x flash %#x/%#x torch %#x/%#x boost %#x timing %#x temp %#x "
	    "flags %#x/%#x id %#x last %#x\n", when, ok ? "" : " (NOT ALL READ)", r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8],
	    r[9], r[10], r[11], r[12], r[13]);
}

static void
say_bus(const char *name, vm_offset_t b)
{
	printf("akita torch: %s: CTL %#x FIFO_CTL %#x TRAILING %#x INT_EN %#x CONF %#x TIMEOUT %#x timing fs %#x/%#x/%#x sla %#x "
	    "TRANS_STATUS %#x USI_CON %#x USI_OPTION %#x\n", name, rd(b + CTL), rd(b + FIFO_CTL), rd(b + TRAILING_CTL),
	    rd(b + INT_EN), rd(b + CONF), rd(b + TIMEOUT), rd(b + TIMING_FS1), rd(b + TIMING_FS2), rd(b + TIMING_FS3),
	    rd(b + TIMING_SLA), rd(b + TRANS_STATUS), rd(b + USI_CON), rd(b + USI_OPTION));
}

/* the clocks, the USI, the pins as found */
static void
look(void)
{
	if (i2c_battery) {
		say_bus("the battery's bus (U-Boot's)", i2c_battery);
	}
	printf("akita torch: CMU_PERIC0 USI14: mux %#x/%#x div %#x gates ipclk %#x pclk %#x rstnsync %#x qch %#x; sysreg mode %#x\n",
	    rd(cmu + CMU_MUX_USI14), rd(cmu + CMU_MUX1_USI14), rd(cmu + CMU_DIV_USI14), rd(cmu + CMU_GATE_IPCLK_USI14),
	    rd(cmu + CMU_GATE_PCLK_USI14), rd(cmu + CMU_GATE_RSTNSYNC_USI14), rd(cmu + CMU_QCH_USI14),
	    rd(sysreg + SYSREG_USI14_MODE));
	printf("akita torch: pins: gpp16 con %#x dat %#x pud %#x; gpp8 con %#x dat %#x pud %#x; gpp24 con %#x dat %#x pud %#x\n",
	    rd(gpio0 + GPP16), rd(gpio0 + GPP16 + 4), rd(gpio0 + GPP16 + 8), rd(gpio0 + GPP8), rd(gpio0 + GPP8 + 4),
	    rd(gpio0 + GPP8 + 8), rd(gpio1 + GPP24), rd(gpio1 + GPP24 + 4), rd(gpio1 + GPP24 + 8));
}

static bool
bus_up(void)
{
	uint32_t v, div = 0, cycle = 0;

	/* the clocks held on, from CMU_TOP as found (the "crystal" side of the mux is no clock to speak of here: a
	 * controller on it stalled in its first byte, 11-11) */
	wr(cmu + CMU_GATE_PCLK_USI14, CMU_GATE_ON);
	wr(cmu + CMU_GATE_RSTNSYNC_USI14, CMU_GATE_ON);
	wr(cmu + CMU_GATE_IPCLK_USI14, CMU_GATE_ON);
	delay(10);
	/* the USI an I2C, out of reset, its clock requested */
	wr(sysreg + SYSREG_USI14_MODE, USI_MODE_I2C);
	wr(i2c + USI_CON, rd(i2c + USI_CON) & ~USI_CON_RESET);
	delay(10);
	v = rd(i2c + USI_OPTION);
	wr(i2c + USI_OPTION, (v & ~USI_OPTION_CLKSTOP_ON) | USI_OPTION_CLKREQ_ON);
	/* the pins: SCL and SDA the I2C's (function 2, no pulls: the board has them); the enables, as LWIS sets them on
	 * power-up: gpp8-2 out high, gpp24-0 out low (active low) */
	wr(gpio0 + GPP16 + 8, rd(gpio0 + GPP16 + 8) & ~0xffu);
	wr(gpio0 + GPP16, (rd(gpio0 + GPP16) & ~0xffu) | 0x22);
	wr(gpio0 + GPP8 + 4, rd(gpio0 + GPP8 + 4) | (1u << 2));
	wr(gpio0 + GPP8, (rd(gpio0 + GPP8) & ~(0xfu << 8)) | (1u << 8));
	wr(gpio1 + GPP24 + 4, rd(gpio1 + GPP24 + 4) & ~1u);
	wr(gpio1 + GPP24, (rd(gpio1 + GPP24) & ~0xfu) | 1u);
	delay(2000);                            /* (the LM3644's start after its enable: well under a millisecond) */
	/* the controller: reset, and set up as the battery's bus is (U-Boot's, on PERIC1's CMU_TOP clock -- PERIC0's USI
	 * clock taken to be the same, 200 MHz in the tree): its timing copied */
	wr(i2c + CTL, rd(i2c + CTL) | CTL_SW_RST);
	wr(i2c + CTL, rd(i2c + CTL) & ~CTL_SW_RST);
	wr(i2c + TRAILING_CTL, rd(i2c_battery + TRAILING_CTL));
	wr(i2c + TIMEOUT, rd(i2c + TIMEOUT) & ~TIMEOUT_EN);
	wr(i2c + CONF, rd(i2c_battery + CONF) | CONF_AUTO_MODE);
	wr(i2c + INT_EN, rd(i2c_battery + INT_EN));
	wr(i2c + FIFO_CTL, FIFO_RX_EN | FIFO_TX_EN);
	wr(i2c + TIMING_FS1, rd(i2c_battery + TIMING_FS1));
	wr(i2c + TIMING_FS2, rd(i2c_battery + TIMING_FS2));
	wr(i2c + TIMING_FS3, rd(i2c_battery + TIMING_FS3));
	wr(i2c + TIMING_SLA, rd(i2c_battery + TIMING_SLA));
	printf("akita torch: the bus up (div %u, cycle %u); CMU mux %#x div %#x gates %#x/%#x/%#x\n", div, cycle,
	    rd(cmu + CMU_MUX_USI14), rd(cmu + CMU_DIV_USI14), rd(cmu + CMU_GATE_IPCLK_USI14), rd(cmu + CMU_GATE_PCLK_USI14),
	    rd(cmu + CMU_GATE_RSTNSYNC_USI14));
	say_bus("HSI2C14", i2c);
	return true;
}

/* ---- the light ------------------------------------------------------------------------------------------------------ */

static uint32_t max_ma = 100;

static uint8_t
code_for(uint32_t ma)
{
	/* code x 1.4 mA + 0.977 mA */
	uint32_t c = ma <= 1 ? 0 : (ma * 1000 - 977 + 700) / 1400;

	return (uint8_t)(c > 127 ? 127 : c);
}

/* off, whatever was refused before: standby, the current back to nothing */
static bool
dark(void)
{
	bool ok = put(REG_ENABLE, ENABLE_DEFAULT, true);

	ok = put(REG_LED1_TORCH, 0, true) && ok;
	lit = !ok && lit;
	level = ok ? 0 : level;
	return ok;
}

static bool
light(uint32_t ma)
{
	lit = true;     /* (before the first write: a write that lands but does not read back leaves it maybe on) */
	/* LED1 alone (bit 7 clear: LED2 does not take LED1's current) */
	return put(REG_LED1_TORCH, code_for(ma), false) && put(REG_ENABLE, ENABLE_DEFAULT | MODE_TORCH | ENABLE_LED1, false);
}

/* a fault: the flags (read clears them), the skin */
extern int akita_thermal_skin_mc(void) __attribute__((weak));

static bool
trouble(uint8_t *f1, uint8_t *f2)
{
	int skin = akita_thermal_skin_mc ? akita_thermal_skin_mc() : 0;

	if (!get(REG_FLAGS1, f1) || !get(REG_FLAGS2, f2)) {
		return true;
	}
	return (*f1 & 0x7e) != 0 || (*f2 & 0x1f) != 0 || skin > 45000;
}

/* ---- torch=3: the light for Darwin ---------------------------------------------------------------------------------- */

static volatile uint32_t wanted;                /* 0 to 1000 */
static uint32_t max_secs = 30;

static int
torch_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	uint32_t value;
	int error;

	while (uio_resid(uio) >= (user_ssize_t)sizeof(value)) {
		if ((error = uiomove((caddr_t)&value, sizeof(value), uio)) != 0) {
			return error;
		}
		wanted = value > 1000 ? 1000 : value;
	}
	return 0;
}

static int
torch_read(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	uint32_t value = level;

	return uiomove((caddr_t)&value, (int)MIN(sizeof(value), (size_t)uio_resid(uio)), uio);
}

static int
torch_open(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	return 0;
}

static const struct cdevsw torch_cdevsw = {
	.d_open = torch_open, .d_close = torch_open, .d_read = torch_read, .d_write = torch_write,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

static uint64_t
now_ms(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / NSEC_PER_MSEC;
}

/* /dev/hakios-torch, as early as devfs takes it (tried every 100 ms for 30 s): before the bus is up, so that it is there
 * when mediaserverd starts (~31 s into a boot; made after the bus it came at 33, 11-11); writes wait in `wanted` */
static bool
make_node(void)
{
	int major = cdevsw_add(-1, &torch_cdevsw);

	for (int i = 0; major >= 0 && i < 300; i++) {
		if (devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "hakios-torch") != NULL) {
			return true;
		}
		delay_for_interval(100, NSEC_PER_MSEC);
	}
	printf("akita torch: /dev/hakios-torch cannot be made\n");
	return false;
}

static void
serve(void)
{
	uint32_t applied = 0;
	uint64_t since = 0, looked = 0;
	bool cut = false;

	printf("akita torch: the light is Darwin's (/dev/hakios-torch): at most %u mA, %u s at a time\n", max_ma, max_secs);
	while (!broken) {
		uint32_t want = wanted;
		uint64_t now = now_ms();
		uint8_t f1 = 0, f2 = 0;

		if (want == 0) {
			cut = false;
		}
		if (cut) {
			want = 0;
		}
		if (want != 0 && applied != 0 && now - since >= max_secs * 1000ull) {
			printf("akita torch: on for %u s: off until asked anew\n", max_secs);
			cut = true;
			want = 0;
		}
		if (want != applied) {
			bool ok = want == 0 ? dark() : light((max_ma * want + 999) / 1000);
			if (!ok) {
				break;
			}
			if (applied == 0 && want != 0) {
				since = now;
			}
			printf("akita torch: %u of 1000 (%u mA)\n", want, (max_ma * want + 999) / 1000);
			applied = want;
			level = want;
		}
		if (applied != 0 && now - looked >= 100) {
			looked = now;
			if (trouble(&f1, &f2)) {
				printf("akita torch: flags %#x/%#x (or a read failed, or the skin too warm): off for the rest of the run\n",
				    f1, f2);
				(void)dark();
				broken = true;
			}
		}
		delay_for_interval(20, NSEC_PER_MSEC);
	}
	if (lit) {
		(void)dark();
	}
	printf("akita torch: the light given up on%s\n", lit ? " (NOT DARK)" : "");
}

/* ---- start, and the end ------------------------------------------------------------------------------------------- */

static uint32_t mode;

/* the two enables (gpp8-2, gpp24-0) set to these levels */
static void
enables(bool en1, bool en2)
{
	wr(gpio0 + GPP8 + 4, (rd(gpio0 + GPP8 + 4) & ~(1u << 2)) | (en1 ? 1u << 2 : 0));
	wr(gpio1 + GPP24 + 4, (rd(gpio1 + GPP24 + 4) & ~1u) | (en2 ? 1u : 0));
	delay(5000);
}

static void
akita_torch_thread(__unused void *arg, __unused wait_result_t wr)
{
	uint8_t id = 0;
	bool found = false;

	if (mode >= 3 && !make_node()) {
		return;
	}
	look();
	if (!bus_up()) {
		return;
	}
	up = true;
	/* LWIS's levels first (gpp8-2 high, gpp24-0 low: active low), then the other three: which one the chip answers */
	static const bool levels[4][2] = { { true, false }, { true, true }, { false, false }, { false, true } };
	for (unsigned k = 0; k < 4 && !found; k++) {
		enables(levels[k][0], levels[k][1]);
		found = get(REG_DEVICE_ID, &id);
		printf("akita torch: gpp8-2 %s, gpp24-0 %s (gpp8 con %#x dat %#x, gpp24 con %#x dat %#x): %s\n",
		    levels[k][0] ? "high" : "low", levels[k][1] ? "high" : "low", rd(gpio0 + GPP8), rd(gpio0 + GPP8 + 4),
		    rd(gpio1 + GPP24), rd(gpio1 + GPP24 + 4), found ? "THE LM3644 ANSWERS" : "no answer");
	}
	if (!found) {
		enables(false, true);           /* (as found: the flash's enable low) */
		printf("akita torch: no answer from the LM3644 at %#x: nothing more\n", LM3644);
		return;
	}
	say_regs("as found");
	if (mode == 2) {
		uint32_t ma = 50, secs = 2;
		uint8_t f1 = 0, f2 = 0;

		PE_parse_boot_argn("torch_ma", &ma, sizeof(ma));
		PE_parse_boot_argn("torch_secs", &secs, sizeof(secs));
		ma = ma > 150 ? 150 : ma;
		secs = secs < 1 ? 1 : secs > 10 ? 10 : secs;
		printf("akita torch: LED1 at %u mA (code %u) for %u s\n", ma, code_for(ma), secs);
		if (light(ma)) {
			for (uint32_t t = 0; t < secs * 10; t++) {
				delay_for_interval(100, NSEC_PER_MSEC);
				if (trouble(&f1, &f2)) {
					printf("akita torch: flags %#x/%#x after %u ms: off\n", f1, f2, t * 100);
					break;
				}
			}
			say_regs("lit");
		}
		printf("akita torch: dark %s\n", dark() ? "again" : "NOT SURE");
		say_regs("after");
	} else if (mode >= 3) {
		serve();
	}
}

static void
akita_torch_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(akita_torch_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

/* (drivers/core/hakios.stubs.c's names: the run's end and a panic's way out call these) */
void hakios_torch_init(void);
void hakios_torch_quiesce(void);

void
hakios_torch_init(void)
{
}

void
hakios_torch_quiesce(void)
{
	if (up && lit) {
		(void)dark();
		printf("akita torch: dark at the run's end\n");
	}
}

__startup_func
static void
akita_torch_init(void)
{
	static thread_call_t start;
	uint64_t deadline;

	if (!PE_parse_boot_argn("torch", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	mode = mode > 3 ? 3 : mode;
	PE_parse_boot_argn("torch_max_ma", &max_ma, sizeof(max_ma));
	max_ma = max_ma < 10 ? 10 : max_ma > 150 ? 150 : max_ma;
	PE_parse_boot_argn("torch_max_s", &max_secs, sizeof(max_secs));
	max_secs = max_secs < 1 ? 1 : max_secs > 120 ? 120 : max_secs;
	cmu = ml_io_map(CMU_PERIC0_PHYS, 0x4000);
	sysreg = ml_io_map(SYSREG_PERIC0_PHYS, 0x1000);
	gpio0 = ml_io_map(GPIO_PERIC0_PHYS, 0x1000);
	gpio1 = ml_io_map(GPIO_PERIC1_PHYS, 0x1000);
	i2c = ml_io_map(HSI2C14_PHYS, 0x1000);
	i2c_battery = ml_io_map(HSI2C_BATTERY_PHYS, 0x1000);
	if (!cmu || !sysreg || !gpio0 || !gpio1 || !i2c) {
		printf("akita torch: could not map its registers\n");
		return;
	}
	/* 2 s from now: /dev/hakios-torch first (make_node), then the bus -- the log keeps what is said before it reaches the
	 * computer */
	start = thread_call_allocate(akita_torch_start, NULL);
	clock_interval_to_deadline(2, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_torch_init);
