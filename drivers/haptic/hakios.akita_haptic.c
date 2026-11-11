/*
 * The Pixel 8a's vibration motor: a Cirrus Logic CS40L26A (a boosted haptic amplifier with a HALO DSP) at 0x43 on zuma's
 * hsi2c@10C80000 -- USI0 of PERIC1 -- driving a linear resonant actuator (Android's device tree, devices/akita/boot-info/
 * fdt.dtb: cs40l26a@43, reset-gpios gph2-3 active low, the interrupt on gpa6-3; f0-default 0x2a8000, redc-default
 * 0x68000, boost 10 V and 2.5 A peak).
 *
 * The chip boots its DSP from ROM; Google's driver (kernel/google-modules/amplifiers, cs40l26/cs40l26.c, akita 5.15
 * android15) then loads Cirrus's RAM firmware (cs40l26.wmfw and the tuning .bin files of /vendor/firmware) and plays
 * its wavetable's effects through the DSP's mailbox (CS40L26_DSP_VIRTUAL1_MBOX_1, 0x13020: 0x01000000 + n a RAM
 * effect, 0x01800000 + n a ROM one, 0x05000000 stop). Registers are 32-bit addresses and 32-bit values, both
 * big-endian on the bus.
 *
 * The bus is brought up here as the flashlight's is (hakios.akita_torch.c: USI clocks held on, the USI in I2C mode,
 * its pins the I2C's, the controller set up as U-Boot's exynos_hs_i2c.c does), with PERIC1's addresses: CMU_PERIC1
 * 0x10c00000 (USI0: Google's cal-if zuma cmucal-sfr.c), sysreg_peric1 0x10c21000 + 0 (samsung,usi-offset), pins
 * gpp19-0/1 function 3 (hsi2c0-bus; Google's pinctrl-gs.c zuma_pin_peric1).
 *
 *   haptic=1   a look: the chip out of reset, its ids and the DSP's state read -- nothing written to the chip
 *   haptic=2   ... and one of the ROM's effects played (haptic_effect=N, 0-38; 0 by default) through the mailbox, as
 *              Google's driver triggers one (hibernation prevented first, allowed after), with the ROM's own setup --
 *              no RAM firmware, no tuning: whether the motor moves at all
 *   haptic=3   ... instead: Cirrus's RAM firmware loaded as Google's driver loads it (cs40l26_fw_upload: the DSP stopped,
 *              fw/cs40l26.wmfw written into its memory, its algorithms' controls found, the wavetable fw/cs40l26.bin and
 *              the tunings -svc.bin and -dvl.bin written, the memory said ready, CALL_RAM_INIT, the core started) and
 *              checked running (HALO_STATE, the firmware's id and revision, its number of waves) -- nothing played
 *              (the files: the phone's /vendor/firmware, pulled into fw-haptic/ of the build, never in git)
 *   haptic=4   ... and the amplifier set up as Google's driver sets it (cs40l26_dsp_config, the device tree's values: the
 *              boost inductor's short test, the A1 errata, the registers' new defaults, boost 10 V at 2.5 A peak, DCM off,
 *              this phone's calibration -- f0 0x29b938, ReDC 0x6c792, Android's stored ones --, PWLE zero cross, f0
 *              compensation), written straight (hibernation stays prevented, so the DSP's power-on sequence, which
 *              Google's driver fills with them, is never run); then the wavetable's click (2) and a long vibration (0)
 *              of 300 ms played at haptic_gain= percent (50)
 *   haptic=5   the motor for Darwin until the run ends: what Darwin plays on its "Actuator" audio device (the system's
 *              feedbacks, the ringer's vibration, CoreHaptics, UIKit's feedbacks) the audio plug-in writes to
 *              /dev/hakios-haptic (userland/src/audio_plugin.c), and it is played as the POCO's driver plays it
 *              (drivers/haptic/hakios.haptic.c): a waveform's start a tap -- the wavetable's click --, one that goes on
 *              a vibration -- its long waveform, as long as Darwin plays, 1 s at most and then as long a rest --, as
 *              strong as Darwin's is loud, up to haptic_gain= percent (80); the chip's error flags looked at while it
 *              plays, and any of them stops it for the run. (No test effects.)
 * (Playing Darwin's haptics is for later: the RAM firmware's loader and its configuration first.)
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <miscfs/devfs/devfs.h>
#include <pexpert/pexpert.h>
#include <sys/conf.h>
#include <sys/systm.h>
#include <sys/uio.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

#define CMU_PERIC1_PHYS         0x10c00000ULL
#define CMU_MUX_USI0            0x0620          /* PLL_CON0_MUX_CLKCMU_PERIC1_USI0_USI_USER: bit 4, 1 = CMU_TOP's clock */
#define CMU_DIV_USI0            0x1808
#define CMU_GATE_IPCLK_USI0     0x2038
#define CMU_GATE_PCLK_USI0      0x203c
#define CMU_GATE_RSTNSYNC_USI0  0x208c
#define CMU_QCH_USI0            0x3038
#define CMU_GATE_ON             0x00300000
#define SYSREG_PERIC1_PHYS      0x10c21000ULL
#define SYSREG_USI0_MODE        0x00
#define USI_MODE_I2C            0x4
#define GPIO_PERIC1_PHYS        0x10c40000ULL
#define GPP19                   0x000
#define GPIO_HSI1_PHYS          0x12040000ULL
#define GPH2                    0x040           /* the reset: pin 3, active low */
#define HSI2C0_PHYS             0x10c80000ULL
#define HSI2C_BATTERY_PHYS      0x10cb0000ULL   /* (PERIC1's bus U-Boot set up, the battery's: timing to copy) */
#define CRYSTAL_HZ              24576000u
#define BUS_HZ                  400000u         /* (the tree says 1 MHz, 0xf4240: 400 kHz to start with) */

#define CTL                     0x00
#define FIFO_CTL                0x04
#define TRAILING_CTL            0x08
#define INT_EN                  0x20
#define INT_STAT                0x24
#define ERROR_STAT              0x2c
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
#define INT_TRANS_EN            0x0f80
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

#define CS40L26                 0x43
#define REG_DEVID               0x0000
#define REG_REVID               0x0004
#define REG_GLOBAL_ENABLES      0x2014
#define REG_PWRMGT_STS          0x290c
#define REG_IRQ1_STATUS         0x10004
#define REG_IRQ1_EINT_1         0x10010
#define REG_PM_CUR_STATE        0x02800370      /* (the A1 ROM's) */
#define REG_HALO_STATE          0x02800fa8
#define REG_REQ_ACTIVE          0x02800c08
#define DEVID_CS40L26A          0x40a260
#define REG_MBOX_1              0x13020         /* CS40L26_DSP_VIRTUAL1_MBOX_1: commands, read back as 0 once taken */
#define MBOX_PREVENT_HIBER      0x02000003
#define MBOX_ALLOW_HIBER        0x02000004
#define MBOX_ROM_EFFECT(n)      (0x01800000u + (n))

static vm_offset_t cmu, sysreg, gpio, gpio_hsi1, i2c, i2c_battery;
static uint32_t mode;
static uint32_t errors;
static bool reset_touched;                      /* gph2-3 driven by us (HSI1's pins: not read otherwise) */

static inline uint32_t rd(vm_offset_t a) { return *(volatile uint32_t *)a; }
static inline void wr(vm_offset_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

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

static void bus_reset(void);

static bool
transfer(uint8_t addr, bool read, uint8_t *buf, unsigned len, bool stop)
{
	uint32_t end;
	bool ok = true;

	for (int k = 0; k < 64 && !(rd(i2c + FIFO_STAT) & FIFO_RX_EMPTY); k++) {
		(void)rd(i2c + RX_DATA);        /* (bytes left from a transfer that failed) */
	}
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
			printf("akita haptic: I2C %s of %u bytes at %#x failed (%s, INT_STAT %#x): now INT_STAT %#x FIFO_STAT %#x "
			    "TRANS_STATUS %#x ERROR_STAT %#x\n", read ? "read" : "write", len, addr, ok ? "the end" : "the FIFO", end,
			    rd(i2c + INT_STAT), rd(i2c + FIFO_STAT), rd(i2c + TRANS_STATUS), rd(i2c + ERROR_STAT));
		}
		if (i2c_battery) {
			bus_reset();            /* (the FIFOs emptied: see bus_reset) */
		}
		return false;
	}
	return true;
}

/* a 32-bit register: its address written big-endian, then a repeated start and four bytes read, big-endian; the chip
 * may sleep (hibernate) and miss the first try, which wakes it */
static bool
reg_read(uint32_t reg, uint32_t *value)
{
	uint8_t a[4] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg }, v[4] = { 0 };

	for (int tries = 0; tries < 3; tries++) {
		if (transfer(CS40L26, false, a, 4, false) && transfer(CS40L26, true, v, 4, true)) {
			*value = (uint32_t)v[0] << 24 | (uint32_t)v[1] << 16 | (uint32_t)v[2] << 8 | v[3];
			return true;
		}
		delay(1000);
	}
	return false;
}

static void
say_bus(const char *name, vm_offset_t b)
{
	printf("akita haptic: %s: CTL %#x FIFO_CTL %#x TRAILING %#x INT_EN %#x CONF %#x TIMEOUT %#x timing fs %#x/%#x/%#x sla %#x "
	    "TRANS_STATUS %#x USI_CON %#x USI_OPTION %#x\n", name, rd(b + CTL), rd(b + FIFO_CTL), rd(b + TRAILING_CTL),
	    rd(b + INT_EN), rd(b + CONF), rd(b + TIMEOUT), rd(b + TIMING_FS1), rd(b + TIMING_FS2), rd(b + TIMING_FS3),
	    rd(b + TIMING_SLA), rd(b + TRANS_STATUS), rd(b + USI_CON), rd(b + USI_OPTION));
}

/*
 * The bus as the bootloader left it, if it did (the USI in I2C mode, the pins the I2C's: Google's bootloader uses this
 * one -- 11-11): its clock and timing kept, only what a transfer needs made sure of, as for the battery's bus. (Not so:
 * the crystal for its clock, as the flashlight's -- which stalled this controller, 11-11: its FIFO filled, nothing sent.)
 */
static bool
bus_as_found(void)
{
	say_bus("HSI2C0 as found", i2c);
	if (rd(sysreg + SYSREG_USI0_MODE) != USI_MODE_I2C || (rd(gpio + GPP19) & 0xff) != 0x33) {
		return false;
	}
	wr(i2c + CONF, rd(i2c + CONF) | CONF_AUTO_MODE);
	wr(i2c + FIFO_CTL, rd(i2c + FIFO_CTL) | FIFO_RX_EN | FIFO_TX_EN);
	return true;
}

/*
 * the battery's bus's timing, on the same CMU's clock, copied (its controller set up by U-Boot) -- after the USI's own
 * reset, which empties the FIFOs: the controller's (CTL_SW_RST) does not, and the 4 bytes a failed transfer left in the
 * transmit FIFO went out at the head of every transfer after, each read answering for the register before (11-11: REVID
 * read as DEVID, GLOBAL_ENABLES as the revision 0xa1)
 */
static void
bus_reset(void)
{
	wr(i2c + USI_CON, rd(i2c + USI_CON) | USI_CON_RESET);
	delay(10);
	wr(i2c + USI_CON, rd(i2c + USI_CON) & ~USI_CON_RESET);
	delay(10);
	wr(i2c + USI_OPTION, (rd(i2c + USI_OPTION) & ~USI_OPTION_CLKSTOP_ON) | USI_OPTION_CLKREQ_ON);
	wr(i2c + CTL, rd(i2c + CTL) | CTL_SW_RST);
	wr(i2c + CTL, rd(i2c + CTL) & ~CTL_SW_RST);
	wr(i2c + TRAILING_CTL, rd(i2c_battery + TRAILING_CTL));
	wr(i2c + CONF, rd(i2c_battery + CONF));
	wr(i2c + INT_EN, rd(i2c_battery + INT_EN));
	wr(i2c + FIFO_CTL, FIFO_RX_EN | FIFO_TX_EN);
	wr(i2c + TIMING_FS1, rd(i2c_battery + TIMING_FS1));
	wr(i2c + TIMING_FS2, rd(i2c_battery + TIMING_FS2));
	wr(i2c + TIMING_FS3, rd(i2c_battery + TIMING_FS3));
	wr(i2c + TIMING_SLA, rd(i2c_battery + TIMING_SLA));
}

static void
bus_timing_from_battery(void)
{
	bus_reset();
	wr(i2c + TRAILING_CTL, rd(i2c_battery + TRAILING_CTL));
	wr(i2c + CONF, rd(i2c_battery + CONF));
	wr(i2c + INT_EN, rd(i2c_battery + INT_EN));
	wr(i2c + FIFO_CTL, FIFO_RX_EN | FIFO_TX_EN);
	wr(i2c + TIMING_FS1, rd(i2c_battery + TIMING_FS1));
	wr(i2c + TIMING_FS2, rd(i2c_battery + TIMING_FS2));
	wr(i2c + TIMING_FS3, rd(i2c_battery + TIMING_FS3));
	wr(i2c + TIMING_SLA, rd(i2c_battery + TIMING_SLA));
	say_bus("HSI2C0 with the battery bus's timing", i2c);
}

static bool
reg_write(uint32_t reg, uint32_t value)
{
	uint8_t b[8] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg,
		         (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value };

	for (int tries = 0; tries < 3; tries++) {
		if (transfer(CS40L26, false, b, 8, true)) {
			return true;
		}
		delay(1000);
	}
	return false;
}

/* a mailbox command, and until the DSP has taken it (the mailbox reads 0 again: Google's cs40l26_ack_write), 100 ms */
static bool
mailbox(uint32_t command)
{
	uint32_t v = ~0u;

	if (!reg_write(REG_MBOX_1, command)) {
		return false;
	}
	for (int i = 0; i < 100; i++) {
		if (reg_read(REG_MBOX_1, &v) && v == 0) {
			return true;
		}
		delay(1000);
	}
	printf("akita haptic: mailbox command %#x not taken (%#x)\n", command, v);
	return false;
}

static __unused bool
bus_up_crystal(void)
{
	uint32_t v, t_ftl, cycles, div = 0, cycle = 0, scl;

	printf("akita haptic: CMU_PERIC1 USI0: mux %#x div %#x gates %#x/%#x/%#x qch %#x; sysreg mode %#x; gpp19 con %#x dat %#x\n",
	    rd(cmu + CMU_MUX_USI0), rd(cmu + CMU_DIV_USI0), rd(cmu + CMU_GATE_IPCLK_USI0), rd(cmu + CMU_GATE_PCLK_USI0),
	    rd(cmu + CMU_GATE_RSTNSYNC_USI0), rd(cmu + CMU_QCH_USI0), rd(sysreg + SYSREG_USI0_MODE), rd(gpio + GPP19),
	    rd(gpio + GPP19 + 4));
	wr(cmu + CMU_GATE_PCLK_USI0, CMU_GATE_ON);
	wr(cmu + CMU_GATE_RSTNSYNC_USI0, CMU_GATE_ON);
	wr(cmu + CMU_MUX_USI0, rd(cmu + CMU_MUX_USI0) & ~(1u << 4));
	wr(cmu + CMU_DIV_USI0, rd(cmu + CMU_DIV_USI0) & ~0xfu);
	wr(cmu + CMU_GATE_IPCLK_USI0, CMU_GATE_ON);
	delay(10);
	wr(sysreg + SYSREG_USI0_MODE, USI_MODE_I2C);
	wr(i2c + USI_CON, rd(i2c + USI_CON) & ~USI_CON_RESET);
	delay(10);
	v = rd(i2c + USI_OPTION);
	wr(i2c + USI_OPTION, (v & ~USI_OPTION_CLKSTOP_ON) | USI_OPTION_CLKREQ_ON);
	wr(gpio + GPP19 + 8, rd(gpio + GPP19 + 8) & ~0xffu);
	wr(gpio + GPP19, (rd(gpio + GPP19) & ~0xffu) | 0x33);
	wr(i2c + CTL, rd(i2c + CTL) | CTL_SW_RST);
	wr(i2c + CTL, rd(i2c + CTL) & ~CTL_SW_RST);
	t_ftl = (rd(i2c + CONF) >> 16) & 7;
	cycles = CRYSTAL_HZ / BUS_HZ - 8 - 2 * t_ftl;
	for (div = 0; div < 256; div++) {
		uint32_t c = cycles / (div + 1);
		if (c < 512 && c > 4) {
			cycle = c - 2;
			break;
		}
	}
	scl = cycle / 2;
	wr(i2c + TRAILING_CTL, TRAILING_COUNT);
	wr(i2c + TIMEOUT, rd(i2c + TIMEOUT) & ~TIMEOUT_EN);
	wr(i2c + CONF, rd(i2c + CONF) | CONF_AUTO_MODE);
	wr(i2c + INT_EN, INT_TRANS_EN);
	wr(i2c + FIFO_CTL, FIFO_RX_EN | FIFO_TX_EN);
	wr(i2c + TIMING_FS1, scl << 24 | scl << 16 | scl << 8);
	wr(i2c + TIMING_FS2, (scl / 2) << 24 | scl << 8 | scl);
	wr(i2c + TIMING_FS3, div << 16 | cycle);
	wr(i2c + TIMING_SLA, scl / 2);
	printf("akita haptic: the bus up (div %u, cycle %u): CONF %#x FIFO_CTL %#x USI_CON %#x USI_OPTION %#x\n", div, cycle,
	    rd(i2c + CONF), rd(i2c + FIFO_CTL), rd(i2c + USI_CON), rd(i2c + USI_OPTION));
	return true;
}


/* ---- haptic=3: the RAM firmware (Google's cs40l26.c and cl_dsp.c, akita 5.15 android15) ---------------------------- */

#define HALO_XM_PACKED          0x02000000
#define HALO_XM_UNPACKED24      0x02800000
#define HALO_YM_PACKED          0x02c00000
#define HALO_YM_UNPACKED24      0x03400000
#define HALO_PM                 0x03800000
#define HALO_FW_ID              0x0280000c      /* the algorithm list: the system's first (id, rev, XM/YM base and size) */
#define HALO_NUM_ALGOS          0x02800024
#define ALGO_LIST_TERM          0xbedead
#define T_XM_UNPACKED           0x05
#define T_YM_UNPACKED           0x06
#define T_PM_PACKED             0x10
#define T_XM_PACKED             0x11
#define T_YM_PACKED             0x12
#define T_ALGO_INFO             0xf2
#define T_INFO                  0xff
#define CCM_CORE_CONTROL        0x02bc1000
#define CCM_CORE_KILL           0x80
#define CCM_CORE_RESET          0x281
#define REG_PWRMGT_CTL          0x2900          /* bit 1: MEM_RDY */
#define REG_REFCLK_INPUT        0x2c04          /* bit 11: the PLL's open loop */
#define REG_EVENT_MAP_1         0x02806fc4      /* (the A1 ROM's GPI mappings: 0x1ff none) */
#define REG_EVENT_MAP_2         0x02806fc8
#define FW_ID                   0x1800d4
#define FW_MIN_REV              0x07022b
#define ALGO_FW                 0x1800d4
#define ALGO_VIBEGEN            0x000100bd
#define ALGO_PM                 0x0001f206
#define ALGO_DVL                0x00040140
#define ALGO_EXT                0x0004013c
#define REG_TEST_KEY            0x40
#define REG_ERROR_RELEASE       0x2034          /* bit 2: BST_SHORT_ERR_RLS */
#define REG_PLL_REFCLK_DETECT_0 0x2c28
#define REG_VBST_CTL_1          0x3800
#define REG_VBST_CTL_2          0x3804          /* bits 1:0 the boost's control (0 fixed, 1 class H), bit 2 its limit */
#define REG_BST_IPK_CTL         0x3808
#define REG_BST_DCM_CTL         0x381c
#define REG_TEST_LBST           0x391c
#define REG_NGATE1_INPUT        0x4c60
#define REG_TST_DAC_MSM_CONFIG  0x7404
#define REG_MIXER_NGATE_CH1_CFG 0x12004
#define BST_CTL_10V             150             /* (10 V - 2.55 V) / 50 mV + 1 */
#define BST_IPK_2500MA          34              /* 2500 mA / 50 - 16 */
#define F0_STORED               0x29b938        /* Android's (sysfs calibration/f0_stored, 11-11) */
#define REDC_STORED             0x6c792
#define MAX_COEFFS              768
#define MAX_ALGOS               33

struct coeff {
	char name[32];
	uint32_t algo;
	uint16_t type, offset;
	uint32_t reg;
};
static struct coeff *coeffs;
static unsigned ncoeffs;
static struct { uint32_t id, rev, xm_base, xm_size, ym_base, ym_size; } algos[MAX_ALGOS];
static unsigned nalgos;
static uint8_t *chunk;                          /* the address and up to 4096 bytes of a block, for one transfer */

static uint32_t
le(const uint8_t *p, unsigned n)
{
	uint32_t v = 0;

	for (unsigned i = 0; i < n; i++) {
		v |= (uint32_t)p[i] << (8 * i);
	}
	return v;
}

/* LEN bytes, as they are (the files are in the bus's order), from register REG on, 4096 at a time */
static bool
raw_write(uint32_t reg, const uint8_t *data, uint32_t len)
{
	for (uint32_t done = 0; done < len;) {
		uint32_t n = len - done > 4096 ? 4096 : len - done, at = reg + done;

		chunk[0] = (uint8_t)(at >> 24);
		chunk[1] = (uint8_t)(at >> 16);
		chunk[2] = (uint8_t)(at >> 8);
		chunk[3] = (uint8_t)at;
		memcpy(chunk + 4, data + done, n);
		if (!transfer(CS40L26, false, chunk, 4 + n, true)) {
			printf("akita haptic: %u bytes at %#x NOT written\n", n, at);
			return false;
		}
		done += n;
	}
	return true;
}

/* a control of the firmware by its name, block type and algorithm (cl_dsp_get_reg) */
static uint32_t
control(const char *name, uint32_t algo)
{
	for (unsigned i = 0; i < ncoeffs; i++) {
		if ((coeffs[i].algo & 0xffff) == (algo & 0xffff) && coeffs[i].type == T_XM_UNPACKED &&
		    strncmp(coeffs[i].name, name, sizeof(coeffs[i].name)) == 0) {
			return coeffs[i].reg;
		}
	}
	printf("akita haptic: no control %s of algorithm %#x\n", name, algo);
	return 0;
}

/* an ALGO_INFO block of the .wmfw: an algorithm's controls, their names, types and offsets (cl_dsp_algo_parse) */
static void
algo_info(const uint8_t *d, uint32_t len)
{
	uint32_t pos = 0, id, count;
	unsigned name_len, desc_len;

	if (len < 8) {
		return;
	}
	id = le(d, 4);
	pos = 4;
	name_len = d[pos];
	pos += 4 + (name_len / 4) * 4;
	desc_len = le(d + pos, 2);
	pos += 4 + (desc_len / 4) * 4;
	count = le(d + pos, 4);
	pos += 4;
	for (uint32_t i = 0; i < count && pos + 8 <= len; i++) {
		uint16_t offset = (uint16_t)le(d + pos, 2), type = (uint16_t)le(d + pos + 2, 2);
		uint32_t block_len = le(d + pos + 4, 4), header = pos + 8;
		unsigned n = d[header];

		if (ncoeffs < MAX_COEFFS && header + 1 + n <= len) {
			struct coeff *c = &coeffs[ncoeffs++];

			memcpy(c->name, d + header + 1, n < sizeof(c->name) ? n : sizeof(c->name) - 1);
			c->name[n < sizeof(c->name) ? n : sizeof(c->name) - 1] = 0;
			c->algo = id;
			c->type = type;
			c->offset = offset;
		}
		pos = header + block_len;
	}
}

/* the .wmfw: its memory blocks written, its algorithms' controls kept (cl_dsp_firmware_parse) */
static bool
load_wmfw(const char *file)
{
	uint64_t size = 0;
	const uint8_t *f = hakios_image_file("fw", file, &size, NULL);
	uint32_t pos = 40, blocks = 0, bytes = 0;

	if (f == NULL || size < 40 || memcmp(f, "WMFW", 4) != 0 || le(f + 4, 4) != 40 || f[10] != 0x04 || (size & 3)) {
		printf("akita haptic: fw/%s %s\n", file, f == NULL ? "not in the image (fw-haptic/ of the build)" :
		    "not a HALO .wmfw");
		return false;
	}
	while (pos + 8 <= size) {
		uint32_t offset = le(f + pos, 3), type = f[pos + 3], len = le(f + pos + 4, 4), reg = 0;
		const uint8_t *d = f + pos + 8;

		if (pos + 8 + len > size) {
			printf("akita haptic: fw/%s: a block past the end\n", file);
			return false;
		}
		switch (type) {
		case T_PM_PACKED: reg = HALO_PM + offset * 5; break;
		case T_XM_PACKED: reg = HALO_XM_PACKED + offset * 3; break;
		case T_XM_UNPACKED: reg = HALO_XM_UNPACKED24 + offset * 4; break;
		case T_YM_PACKED: reg = HALO_YM_PACKED + offset * 3; break;
		case T_YM_UNPACKED: reg = HALO_YM_UNPACKED24 + offset * 4; break;
		case T_ALGO_INFO: algo_info(d, len); break;
		case T_INFO: break;
		default:
			printf("akita haptic: fw/%s: a block of type %#x\n", file, type);
			return false;
		}
		if (reg != 0) {
			if (!raw_write(reg, d, len)) {
				return false;
			}
			blocks++;
			bytes += len;
		}
		pos += 8 + ((len + 3) & ~3u);
	}
	printf("akita haptic: fw/%s: %u blocks, %u bytes written; %u controls\n", file, blocks, bytes, ncoeffs);
	return true;
}

/* the algorithm list the firmware put in XM, and the controls' registers from it (cl_dsp_coeff_init) */
static bool
algorithms(void)
{
	uint32_t n = 0, at = HALO_FW_ID, term = 0;

	if (!reg_read(HALO_NUM_ALGOS, &n) || n + 1 > MAX_ALGOS) {
		printf("akita haptic: the number of algorithms %u\n", n);
		return false;
	}
	nalgos = n + 1;
	for (unsigned i = 0; i < nalgos; i++) {
		if (!reg_read(at, &algos[i].id) || !reg_read(at + 4, &algos[i].rev) || !reg_read(at + 8, &algos[i].xm_base) ||
		    !reg_read(at + 12, &algos[i].xm_size) || !reg_read(at + 16, &algos[i].ym_base) ||
		    !reg_read(at + 20, &algos[i].ym_size)) {
			return false;
		}
		for (unsigned k = 0; k < ncoeffs; k++) {
			if (coeffs[k].algo != algos[i].id) {
				continue;
			}
			if (coeffs[k].type == T_XM_UNPACKED) {
				coeffs[k].reg = HALO_XM_UNPACKED24 + algos[i].xm_base * 4 + coeffs[k].offset * 4;
			} else if (coeffs[k].type == T_YM_UNPACKED) {
				coeffs[k].reg = HALO_YM_UNPACKED24 + algos[i].ym_base * 4 + coeffs[k].offset * 4;
			}
		}
		at += i == 0 ? 24 + 4 : 24;     /* (the system's entry has one more word: the number of algorithms) */
	}
	(void)reg_read(at, &term);
	printf("akita haptic: %u algorithms (firmware %#x rev %u.%u.%u)%s\n", nalgos, algos[0].id, algos[0].rev >> 16 & 0xff,
	    algos[0].rev >> 8 & 0xff, algos[0].rev & 0xff, term == ALGO_LIST_TERM ? "" : " -- NO LIST TERMINATOR");
	return term == ALGO_LIST_TERM;
}

static bool
algo_present(uint32_t id)
{
	for (unsigned i = 0; i < nalgos; i++) {
		if ((algos[i].id & 0xffff) == (id & 0xffff)) {
			return true;
		}
	}
	return false;
}

/* a .bin (WMDR): its blocks written relative to their algorithms (cl_dsp_coeff_file_parse) */
static bool
load_bin(const char *file)
{
	uint64_t size = 0;
	const uint8_t *f = hakios_image_file("fw", file, &size, NULL);
	uint32_t pos = 16, blocks = 0, bytes = 0;

	if (f == NULL || size < 16 || memcmp(f, "WMDR", 4) != 0 || le(f + 4, 4) != 16 || (size & 3)) {
		printf("akita haptic: fw/%s %s\n", file, f == NULL ? "not in the image" : "not a .bin (WMDR)");
		return false;
	}
	while (pos + 20 <= size) {
		uint32_t offset = le(f + pos, 2), type = le(f + pos + 2, 2), algo = le(f + pos + 4, 4) & 0xffff;
		uint32_t len = le(f + pos + 16, 4), reg = 0;
		unsigned i;

		if (pos + 20 + len > size) {
			printf("akita haptic: fw/%s: a block past the end\n", file);
			return false;
		}
		if (type != 0xfe00 && type != 0xff00) {
			for (i = 0; i < nalgos && (algos[i].id & 0xffff) != algo; i++) {
			}
			if (i == nalgos) {
				printf("akita haptic: fw/%s: algorithm %#x not in the firmware\n", file, algo);
				return false;
			}
			switch (type) {
			case T_XM_UNPACKED: reg = HALO_XM_UNPACKED24 + offset + algos[i].xm_base * 4; break;
			case T_XM_PACKED: reg = (HALO_XM_PACKED + offset + algos[i].xm_base * 3) & ~3u; break;
			case T_YM_UNPACKED: reg = HALO_YM_UNPACKED24 + offset + algos[i].ym_base * 4; break;
			case T_YM_PACKED: reg = (HALO_YM_PACKED + offset + algos[i].ym_base * 3) & ~3u; break;
			default:
				printf("akita haptic: fw/%s: a block of type %#x\n", file, type);
				return false;
			}
			if (!raw_write(reg, f + pos + 20, len)) {
				return false;
			}
			blocks++;
			bytes += len;
		}
		pos += 20 + ((len + 3) & ~3u);
	}
	printf("akita haptic: fw/%s: %u blocks, %u bytes written\n", file, blocks, bytes);
	return true;
}

static uint32_t
pm_state(void)
{
	uint32_t reg = control("PM_CUR_STATE", ALGO_PM), v = ~0u;

	if (reg == 0 || !reg_read(reg, &v)) {
		return ~0u;
	}
	return v & 0xff;
}

/* (bits of a register changed, read back said) */
static bool
update(uint32_t reg, uint32_t mask, uint32_t value)
{
	uint32_t v = 0;

	return reg_read(reg, &v) && reg_write(reg, (v & ~mask) | (value & mask));
}

/*
 * While the core is stopped (Google's cs40l26_lbst_short_test, cs40l26_handle_errata and
 * cs40l26_update_reg_defaults_via_pseq, the device tree saying bst-expl-mode-disable): the boost inductor tried for a
 * short -- the boost fixed at VP, GLOBAL_EN on 10 ms, BST_SHORT_ERR looked at -- then the errata and the new defaults
 */
static bool
amp_before_start(void)
{
	uint32_t err = 0;

	if (!update(REG_VBST_CTL_2, 0x3, 0x0) || !update(REG_VBST_CTL_1, 0xff, 0x00) ||
	    !update(REG_GLOBAL_ENABLES, 0x1, 0x1)) {
		return false;
	}
	delay(10000);
	if (!reg_read(REG_ERROR_RELEASE, &err)) {
		return false;
	}
	(void)update(REG_GLOBAL_ENABLES, 0x1, 0x0);
	if (err & (1u << 2)) {
		printf("akita haptic: THE BOOST INDUCTOR LOOKS SHORTED (ERROR_RELEASE %#x): nothing played\n", err);
		return false;
	}
	if (!update(REG_VBST_CTL_2, 0x3, 0x1) || !update(REG_VBST_CTL_1, 0xff, 0x00)) {
		return false;
	}
	/* the A1 errata (cs40l26_a1_errata, without exploration mode) and the defaults Google's driver changes */
	if (!reg_write(REG_PLL_REFCLK_DETECT_0, 0) || !reg_write(REG_TEST_KEY, 0x55) || !reg_write(REG_TEST_KEY, 0xaa) ||
	    !reg_write(REG_TEST_LBST, 0x014dc080) || !update(REG_NGATE1_INPUT, 0xffff, 0x35) ||
	    !reg_write(REG_MIXER_NGATE_CH1_CFG, 0x00010003) || !update(REG_TST_DAC_MSM_CONFIG, 0xffff0000u, 0x11330000u)) {
		return false;
	}
	printf("akita haptic: the boost inductor is not shorted (ERROR_RELEASE %#x); the errata and new defaults written\n",
	    err);
	return true;
}

/* after the start (cs40l26_bst_dcm_config, cs40l26_bst_ipk_config, cs40l26_zero_cross_config,
 * calib_device_tree_config, the compensation and TIMEOUT_MS of cs40l26_dsp_config) */
static bool
amp_after_start(void)
{
	uint32_t reg;
	bool ok = update(REG_BST_DCM_CTL, 0x1, 0x0) && reg_write(REG_BST_IPK_CTL, BST_IPK_2500MA) &&
	    update(REG_VBST_CTL_1, 0xff, BST_CTL_10V) && update(REG_VBST_CTL_2, 1u << 2, 1u << 2);

	if (ok && (reg = control("F0_OTP_STORED", ALGO_VIBEGEN)) != 0) ok = reg_write(reg, F0_STORED);
	if (ok && (reg = control("REDC_OTP_STORED", ALGO_VIBEGEN)) != 0) ok = reg_write(reg, REDC_STORED);
	if (ok && (reg = control("PWLE_EXTEND_ZERO_CROSS", ALGO_VIBEGEN)) != 0) ok = reg_write(reg, 1);
	if (ok && (reg = control("COMPENSATION_ENABLE", ALGO_VIBEGEN)) != 0) ok = reg_write(reg, 1u << 0);   /* f0 on, ReDC off */
	if (ok && (reg = control("TIMEOUT_MS", ALGO_VIBEGEN)) != 0) ok = reg_write(reg, 0);
	printf("akita haptic: the amplifier %s: boost 10 V, 2.5 A peak, DCM off; f0 %#x, ReDC %#x\n", ok ? "set up" : "NOT set up",
	    F0_STORED, REDC_STORED);
	return ok;
}

/* the gain: SOURCE_ATTENUATION, Q21.2 dB down (Google's table: 100 % 0, 50 % 24, 1 % 160, mute 400) */
static void
gain(uint32_t percent)
{
	uint32_t reg = control("SOURCE_ATTENUATION", ALGO_EXT), q = 0;

	/* 20 log10(100 / percent) dB, in quarters: from a small table, no floating point */
	static const struct { uint32_t pct, q; } t[] = { { 100, 0 }, { 80, 8 }, { 63, 16 }, { 50, 24 }, { 40, 32 },
		{ 32, 40 }, { 25, 48 }, { 20, 56 }, { 10, 80 }, { 5, 104 }, { 1, 160 } };
	for (unsigned i = 0; i < sizeof(t) / sizeof(t[0]); i++) {
		if (percent >= t[i].pct) {
			q = t[i].q;
			break;
		}
		q = 160;
	}
	if (reg != 0) {
		(void)reg_write(reg, q);
	}
}

/* one effect of the wavetable (RAM index n), the long one for DURATION ms (0: the waveform's own length) */
static bool
play(uint32_t n, uint32_t duration_ms)
{
	uint32_t reg = control("TIMEOUT_MS", ALGO_VIBEGEN);

	if (reg != 0) {
		(void)reg_write(reg, duration_ms);
	}
	return mailbox(0x01000000u + n);
}

static void serve(void);

static bool
ram_firmware(void)
{
	uint32_t v = 0, halo = 0, id = 0, rev = 0, waves = 0, reg;

	if (coeffs == NULL) {
		coeffs = kalloc_data(MAX_COEFFS * sizeof(*coeffs), Z_WAITOK | Z_ZERO);
		chunk = kalloc_data(4 + 4096, Z_WAITOK | Z_ZERO);
		if (coeffs == NULL || chunk == NULL) {
			return false;
		}
	}
	ncoeffs = nalgos = 0;
	/* as Google's probe: the PLL in open loop, no GPI mappings (the DSP may lock up otherwise while loading) */
	if (reg_read(REG_REFCLK_INPUT, &v)) {
		(void)reg_write(REG_REFCLK_INPUT, v | (1u << 11));
	}
	(void)reg_write(REG_EVENT_MAP_1, 0x1ff);
	(void)reg_write(REG_EVENT_MAP_2, 0x1ff);
	/* cs40l26_dsp_pre_config: awake, the ROM running, then the core safe to stop (shutdown or standby), stopped */
	if (!mailbox(MBOX_PREVENT_HIBER) || !reg_read(REG_HALO_STATE, &halo) || halo != 2) {
		printf("akita haptic: the ROM not running (halo state %#x)\n", halo);
		return false;
	}
	for (int i = 0; i < 10; i++) {
		if (reg_read(REG_PM_CUR_STATE, &v) && ((v & 0xff) == 1 || (v & 0xff) == 2)) {
			break;
		}
		delay(10000);
	}
	if (!reg_write(CCM_CORE_CONTROL, CCM_CORE_KILL)) {
		return false;
	}
	printf("akita haptic: the ROM's core stopped (pm state %#x); the RAM firmware\n", v);
	/* cl_dsp_firmware_parse, cl_dsp_coeff_init, cs40l26_get_fw_params */
	if (!load_wmfw("cs40l26.wmfw") || !algorithms()) {
		return false;
	}
	(void)reg_read(HALO_FW_ID, &id);
	(void)reg_read(HALO_FW_ID + 4, &rev);
	if (id != FW_ID || rev < FW_MIN_REV) {
		printf("akita haptic: firmware %#x rev %#x: not what Google's driver takes\n", id, rev);
		return false;
	}
	/* cs40l26_coeff_load: the wavetable, the SVC tuning, the DVL's if its algorithm is there */
	(void)load_bin("cs40l26.bin");
	(void)load_bin("cs40l26-svc.bin");
	if (algo_present(ALGO_DVL)) {
		(void)load_bin("cs40l26-dvl.bin");
	}
	/* cs40l26_dsp_config: the memory ready, CALL_RAM_INIT */
	if (!reg_read(REG_PWRMGT_CTL, &v) || !reg_write(REG_PWRMGT_CTL, v | (1u << 1)) ||
	    (reg = control("CALL_RAM_INIT", ALGO_FW)) == 0 || !reg_write(reg, 1)) {
		return false;
	}
	if (mode >= 4 && !amp_before_start()) {
		return false;
	}
	/* ... the core started (cs40l26_dsp_start), awake, running */
	if (!reg_write(CCM_CORE_CONTROL, CCM_CORE_RESET)) {
		return false;
	}
	for (int i = 0; i < 10 && (v = pm_state()) != 3 && v != 2; i++) {
		delay(10000);
	}
	if (!mailbox(MBOX_PREVENT_HIBER)) {
		return false;
	}
	reg = control("HALO_STATE", ALGO_FW);
	if (reg == 0 || !reg_read(reg, &halo)) {
		return false;
	}
	reg = control("NUM_OF_WAVES", ALGO_VIBEGEN);
	if (reg != 0) {
		(void)reg_read(reg, &waves);
	}
	printf("akita haptic: THE RAM FIRMWARE %s: pm state %u, halo state %#x, %u waves\n", halo == 2 ? "RUNS" : "does NOT run",
	    pm_state(), halo, waves);
	if (halo != 2) {
		return false;
	}
	if (mode >= 5) {
		if (amp_after_start()) {
			serve();
		}
	} else if (mode == 4 && amp_after_start()) {
		uint32_t percent = 50, irq = 0, err = 0, st = 0;

		PE_parse_boot_argn("haptic_gain", &percent, sizeof(percent));
		percent = percent > 100 ? 100 : percent;
		gain(percent);
		bool clicked = play(2, 0);
		delay(1000000);
		bool buzzed = play(0, 300);
		delay(1000000);
		(void)reg_read(REG_IRQ1_STATUS, &irq);
		(void)reg_read(REG_ERROR_RELEASE, &err);
		(void)reg_read(REG_PWRMGT_STS, &st);
		printf("akita haptic: PLAYED at %u %%: the click %s, the long vibration %s; IRQ1 status %#x, ERROR_RELEASE %#x, "
		    "power status %#x\n", percent, clicked ? "taken" : "NOT taken", buzzed ? "taken" : "NOT taken", irq, err, st);
	}
	return true;
}


/* ---- haptic=5: Darwin's haptics ------------------------------------------------------------------------------------ */

#define HAPTIC_MAX_MS           1000            /* a vibration at most, then as long a rest */
#define ACTIVE_US               30000           /* a write that was not silence counts for this long */
#define SILENT_BELOW            256             /* of 32768: samples no larger are silence */
#define TAP_US                  60000           /* longer than this, and it is a vibration, not a tap */
#define SETTLE_US               8000            /* Darwin's waveform listened to before a tap: how loud it is */
#define FOLLOW_US               20000           /* how often a vibration's strength follows Darwin's */
#define TOLD_EVENTS             24
#define MBOX_STOP               0x05000000
#define WAVE_CLICK              2               /* the Pixel's vibrator HAL's indices: 0 long, 2 click, 9 light tick */
#define WAVE_LONG               0

static volatile uint64_t heard_us, quiet_us, onset_us;
static volatile uint32_t heard_peak;
static uint32_t max_percent = 80;
static bool serving, broken;

static uint64_t
now_us(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / NSEC_PER_USEC;
}

/* From the audio server's I/O thread: never waits (the same as the POCO's driver). */
static int
haptic_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	int16_t samples[256];
	int error = 0;

	while (uio_resid(uio) >= 2) {
		user_ssize_t bytes = uio_resid(uio);
		uint32_t peak = 0;

		if (bytes > (user_ssize_t)sizeof(samples)) {
			bytes = sizeof(samples);
		}
		bytes &= ~(user_ssize_t)1;
		if ((error = uiomove((caddr_t)samples, (int)bytes, uio)) != 0) {
			break;
		}
		for (unsigned i = 0; i < bytes / 2; i++) {
			uint32_t v = samples[i] < 0 ? (uint32_t)-(int32_t)samples[i] : (uint32_t)samples[i];

			peak = v > peak ? v : peak;
		}
		if (peak > SILENT_BELOW) {
			uint64_t now = now_us();

			if (heard_us == 0 || quiet_us > heard_us || now - heard_us >= ACTIVE_US) {
				onset_us = now;
				heard_peak = 0;
			}
			heard_us = now;
			if (peak > heard_peak) {
				heard_peak = peak;
			}
		} else {
			quiet_us = now_us();
		}
	}
	return error;
}

static int
haptic_open(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	return 0;
}

static const struct cdevsw haptic_cdevsw = {
	.d_open = haptic_open, .d_close = haptic_open, .d_read = eno_rdwrt, .d_write = haptic_write,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

/* /dev/hakios-haptic, as early as devfs takes it (the audio plug-in opens it once, when Darwin first plays) */
static bool
make_node(void)
{
	int major = cdevsw_add(-1, &haptic_cdevsw);

	for (int i = 0; major >= 0 && i < 300; i++) {
		if (devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "hakios-haptic") != NULL) {
			return true;
		}
		delay_for_interval(100, NSEC_PER_MSEC);
	}
	printf("akita haptic: /dev/hakios-haptic cannot be made\n");
	return false;
}

/* as strong as Darwin's waveform is loud: from a third of full scale on, haptic_gain; below, down to 5/8 of it */
static uint32_t
percent_for(uint32_t peak)
{
	uint32_t top = max_percent, low = top * 5 / 8, third = 32767 / 3;

	return peak >= third ? top : low + (top - low) * peak / third;
}

static bool
healthy(void)
{
	uint32_t err = 0;

	if (!reg_read(REG_ERROR_RELEASE, &err)) {
		return false;
	}
	if (err & 0xfe) {               /* (the amplifier's, the boost's, temperature's errors) */
		printf("akita haptic: ERROR_RELEASE %#x while playing: the motor stopped for the run\n", err);
		return false;
	}
	return true;
}

static void
serve(void)
{
	enum { IDLE, TAP, LOOP, REST } state = IDLE;
	uint64_t since = 0, tap_onset = 0, followed = 0, rest_until = 0, last_look = 0;
	uint32_t taps = 0, loops = 0, capped = 0, playing = 0;

	printf("akita haptic: THE MOTOR PLAYS WHAT DARWIN PLAYS ON ITS ACTUATOR (/dev/hakios-haptic), up to %u %%\n",
	    max_percent);
	serving = true;
	while (!broken) {
		uint64_t now = now_us();
		bool active = heard_us != 0 && now - heard_us < ACTIVE_US;
		uint32_t peak = heard_peak, percent = percent_for(peak);

		switch (state) {
		case IDLE:
		case TAP:
			if (active && onset_us != tap_onset && now - onset_us >= SETTLE_US) {
				uint64_t began = onset_us;

				gain(percent);
				if (!play(WAVE_CLICK, 0)) {
					broken = true;
					break;
				}
				state = TAP;
				since = now;
				tap_onset = began;
				if (++taps + loops <= TOLD_EVENTS) {
					printf("akita haptic: tap %u: Darwin's waveform peaks at %u of 32768: the click at %u %%\n", taps,
					    peak, percent);
				}
				break;
			}
			if (state == IDLE) {
				break;
			}
			if (active && quiet_us < tap_onset && now - since > TAP_US) {
				gain(percent);
				if (!mailbox(MBOX_STOP) || !play(WAVE_LONG, HAPTIC_MAX_MS)) {
					broken = true;
					break;
				}
				state = LOOP;
				since = followed = now;
				playing = percent;
				heard_peak = 0;
				if (taps + ++loops <= TOLD_EVENTS) {
					printf("akita haptic: vibration %u: Darwin goes on playing (peak %u): the long waveform at %u %%\n",
					    loops, peak, percent);
				}
			} else if (!active && now - since > TAP_US) {
				state = IDLE;
			}
			break;
		case LOOP:
			if (active && now - followed >= FOLLOW_US) {
				followed = now;
				heard_peak = 0;
				if (peak != 0 && (percent > playing + 8 || percent + 8 < playing)) {
					gain(percent);
					playing = percent;
				}
			}
			if (!active || now - since > HAPTIC_MAX_MS * 1000ull) {
				bool cut = now - since > HAPTIC_MAX_MS * 1000ull;

				if (cut) {
					capped++;
					rest_until = now + HAPTIC_MAX_MS * 1000ull;
				}
				if (!mailbox(MBOX_STOP)) {
					broken = true;
					break;
				}
				state = rest_until > now ? REST : IDLE;
				if (taps + loops <= TOLD_EVENTS + 1) {
					printf("akita haptic: vibration %u: %s after %llu ms\n", loops, cut ? "CUT at the limit" :
					    "Darwin stopped, and so does the motor", (now - since) / 1000);
				}
			}
			break;
		case REST:
			if (now >= rest_until) {
				state = IDLE;
			}
			break;
		}
		if (state != IDLE && state != REST && now - last_look > 50000) {
			last_look = now;
			if (!healthy()) {
				(void)mailbox(MBOX_STOP);
				broken = true;
			}
		}
		delay_for_interval(4, NSEC_PER_MSEC);
	}
	printf("akita haptic: the motor given up on (%u taps and %u vibrations played, %u cut)\n", taps, loops, capped);
}

static void
akita_haptic_thread(__unused void *arg, __unused wait_result_t wr_)
{
	uint32_t devid = 0, revid = 0, v[6] = { 0 };

	if (mode >= 5) {
		PE_parse_boot_argn("haptic_gain", &max_percent, sizeof(max_percent));
		max_percent = max_percent > 100 ? 100 : max_percent < 10 ? 10 : max_percent;
		if (!make_node()) {
			return;
		}
	}

	say_bus("the battery's bus (U-Boot's)", i2c_battery);
	printf("akita haptic: CMU_PERIC1 USI0: mux %#x div %#x gates %#x/%#x/%#x qch %#x; sysreg mode %#x; gpp19 con %#x dat %#x\n",
	    rd(cmu + CMU_MUX_USI0), rd(cmu + CMU_DIV_USI0), rd(cmu + CMU_GATE_IPCLK_USI0), rd(cmu + CMU_GATE_PCLK_USI0),
	    rd(cmu + CMU_GATE_RSTNSYNC_USI0), rd(cmu + CMU_QCH_USI0), rd(sysreg + SYSREG_USI0_MODE), rd(gpio + GPP19),
	    rd(gpio + GPP19 + 4));
	if (!bus_as_found()) {
		printf("akita haptic: the bus not set up by the bootloader: nothing more\n");
		return;
	}
	if (!reg_read(REG_DEVID, &devid)) {
		bus_timing_from_battery();
	}
	/* as the bootloader left it first; if it does not answer (held in reset), out of reset as Google's probe does it:
	 * low 1.5 ms, then high, then 6 ms before the control port is ready (gph2 is HSI1's: said before it is touched) */
	if (!reg_read(REG_DEVID, &devid)) {
		printf("akita haptic: no answer as found; the reset (gph2-3 of pinctrl@12040000) pulsed\n");
		delay(100000);
		printf("akita haptic: gph2 con %#x dat %#x\n", rd(gpio_hsi1 + GPH2), rd(gpio_hsi1 + GPH2 + 4));
		wr(gpio_hsi1 + GPH2 + 4, rd(gpio_hsi1 + GPH2 + 4) & ~(1u << 3));
		wr(gpio_hsi1 + GPH2, (rd(gpio_hsi1 + GPH2) & ~(0xfu << 12)) | (1u << 12));
		reset_touched = true;
		delay(1600);
		wr(gpio_hsi1 + GPH2 + 4, rd(gpio_hsi1 + GPH2 + 4) | (1u << 3));
		delay(7000);
		if (!reg_read(REG_DEVID, &devid)) {
			printf("akita haptic: no answer from the CS40L26 at %#x\n", CS40L26);
			return;
		}
	}
	(void)reg_read(REG_REVID, &revid);
	(void)reg_read(REG_GLOBAL_ENABLES, &v[0]);
	(void)reg_read(REG_PWRMGT_STS, &v[1]);
	(void)reg_read(REG_IRQ1_STATUS, &v[2]);
	(void)reg_read(REG_PM_CUR_STATE, &v[3]);
	(void)reg_read(REG_HALO_STATE, &v[4]);
	(void)reg_read(REG_REQ_ACTIVE, &v[5]);
	printf("akita haptic: CS40L26 devid %#x%s revid %#x; global enables %#x, power status %#x, IRQ1 status %#x; the DSP: "
	    "pm state %#x, halo state %#x, req active %#x\n", devid, (devid & 0xffffff) == DEVID_CS40L26A ? " (CS40L26A)" : "",
	    revid, v[0], v[1], v[2], v[3], v[4], v[5]);
	if (mode >= 3 && (devid & 0xffffff) == DEVID_CS40L26A) {
		(void)ram_firmware();
	} else if (mode == 2 && (devid & 0xffffff) == DEVID_CS40L26A) {
		uint32_t effect = 0, state = 0, irq = 0;

		PE_parse_boot_argn("haptic_effect", &effect, sizeof(effect));
		effect = effect > 38 ? 0 : effect;
		if (mailbox(MBOX_PREVENT_HIBER)) {
			(void)reg_read(REG_PM_CUR_STATE, &state);
			printf("akita haptic: awake (pm state %#x); the ROM's effect %u\n", state, effect);
			bool played = mailbox(MBOX_ROM_EFFECT(effect));
			delay(600000);
			(void)reg_read(REG_IRQ1_STATUS, &irq);
			(void)reg_read(REG_PWRMGT_STS, &state);
			printf("akita haptic: the ROM's effect %u %s; IRQ1 status %#x, power status %#x\n", effect,
			    played ? "taken" : "NOT taken", irq, state);
			(void)mailbox(MBOX_ALLOW_HIBER);
		}
	}
}

static void
akita_haptic_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(akita_haptic_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

/* (drivers/core/hakios.stubs.c's names) */
void hakios_haptic_init(void);
void hakios_haptic_quiesce(void);
void hakios_haptic_pretend(uint32_t peak);

void hakios_haptic_init(void) {}
void hakios_haptic_pretend(__unused uint32_t peak) {}

void
hakios_haptic_quiesce(void)
{
	/* the chip back into reset, if it was let out */
	if (reset_touched) {
		wr(gpio_hsi1 + GPH2 + 4, rd(gpio_hsi1 + GPH2 + 4) & ~(1u << 3));
	}
}

__startup_func
static void
akita_haptic_init(void)
{
	static thread_call_t start;
	uint64_t deadline;

	if (!PE_parse_boot_argn("haptic", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	cmu = ml_io_map(CMU_PERIC1_PHYS, 0x4000);
	sysreg = ml_io_map(SYSREG_PERIC1_PHYS, 0x1000);
	gpio = ml_io_map(GPIO_PERIC1_PHYS, 0x1000);
	gpio_hsi1 = ml_io_map(GPIO_HSI1_PHYS, 0x1000);
	i2c = ml_io_map(HSI2C0_PHYS, 0x1000);
	i2c_battery = ml_io_map(HSI2C_BATTERY_PHYS, 0x1000);
	if (!cmu || !sysreg || !gpio || !gpio_hsi1 || !i2c) {
		printf("akita haptic: could not map its registers\n");
		return;
	}
	start = thread_call_allocate(akita_haptic_start, NULL);
	/* 2 s from now: /dev/hakios-haptic first (haptic=5), before Darwin's first haptic -- the audio plug-in opened it
	 * then, at ~40 s, and it came at 48 (11-11) --, then the bus and the firmware */
	clock_interval_to_deadline(2, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_haptic_init);
