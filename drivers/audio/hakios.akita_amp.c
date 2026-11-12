/*
 * The Pixel 8a's speaker amplifiers: two Cirrus Logic CS35L41s on the SPI controller at 0x111C0000 (USI1 of zuma's HSI0
 * block), the left one on the controller's own chip select, the right one ("R", cirrus,right-channel-amp) on a GPIO one
 * (gpn1-0). The AoC drives their audio over its TDM_0 port (DSP-A, 4 slots of 32 bits, 16-bit stereo at 48 kHz: a
 * 6.144 MHz bit clock); this driver is their control side. Only with boot-arg amp=N.
 *
 *   amp=1  a look: the SPI clock and controller, both chips reset (gpp1-1, gph2-1), their OTP boot, device and revision ids
 *   amp=2  and set up for the AoC's TDM as Linux's cs35l41 does without its DSP (no firmware: the protection algorithm is
 *          not run, so the level is kept low): test key, rev B2 errata, the OTP's trims, GPIO2 the interrupt line, the boost
 *          (1 uH, 4 uF, 3.5 A peak: the device tree), DSP-A as a clock consumer, 48 kHz, 32-bit slots of 16-bit data, the
 *          left on slot 0 and the right on slot 1, the PLL from the 6.144 MHz bit clock, ASP RX1 straight to the DAC, class
 *          H, the analog gain at its least; akita_amp_speaker(1) (the AoC's driver, once its TDM runs) powers both up --
 *          Linux's "Main AMP" widget: the power-up patch, GLOBAL_EN, PUP_DONE -- and akita_amp_speaker(0) down
 *   speaker=1  needed for any of it to be heard: without it akita_amp_speaker(1) refuses and the amplifiers stay
 *          powered down (the tester, 11-11: no sound from the phone unless he is with it)
 *   amp=4  and, whenever they are powered up (speaker=1), the protection RUN between the AoC's samples and the DAC
 *          (Linux's DSP1 widgets: VMON/IMON/VPMON/VBSTMON/TEMPMON on, DSP1 RX1-4 from ASP RX1/RX2, VMON, IMON, the DAC
 *          fed from DSP1 TX1; cs_dsp_run: the MPU opened, the core started; CSPL's mailbox status, RESUME), PAUSE before
 *          they power down. WRITTEN 11-11 NIGHT, NOT YET RUN: it needs the TDM's clock, so sound -- a test for when the
 *          tester is with the phone
 *   amp=3  and Cirrus's speaker protection loaded into each one's HALO DSP (cs_dsp_power_up, silent: the core is not
 *          started, the outputs stay off): Linux's DSP setup (the rates' errata, DSP1 RX5-8 from VPMON, VBSTMON, TEMPMON),
 *          fw/cs35l41-dsp1-spk-prot.wmfw's memory blocks written and read back, the algorithm list the firmware keeps
 *          in XM, the tuning fw/[R-]cs35l41-dsp1-spk-prot.bin written relative to its algorithm (CSPL). The loader is
 *          the CS40L26's (drivers/haptic) over SPI: the same HALO memory map and file formats
 *
 * The SPI controller is the touch screen's (drivers/input/hakios.akita_touch.c: Samsung's s3c64xx, polled, 8-bit, mode 0,
 * chip select by hand), on USI1 instead of USI2. The chips' SPI format (Linux's cs35l41_regmap_spi): a 32-bit big-endian
 * register address (bit 31 set for a read), 16 bits of padding, the 32-bit big-endian value.
 *
 * Sources: Linux sound/soc/codecs/cs35l41*.c and include/sound/cs35l41.h (v6.12, ~/akita/ref/cs35l41), Google's
 * aoc/alsa/aoc_alsa_card.c (the TDM format), the phone's device tree (spi@111C0000, cs35l41@0/@1, their pins).
 */
#include <stdint.h>
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

/* ---- the addresses ------------------------------------------------------------------------------------------------ */

#define SPI_PHYS                0x111c0000ULL   /* spi@111C0000, USI1 of HSI0 */
#define CMU_PHYS                0x11001000ULL   /* CMU_HSI0 (0x11000000) from its muxes on */
#define CMU_MUX_USI1            0x018           /* CLK_CON_MUX_MUX_CLK_HSI0_USI1 (0x1018): bit 0, 1 = the crystal */
#define CMU_DIV_USI1            0x818           /* CLK_CON_DIV_DIV_CLK_HSI0_USI1 (0x1818) */
#define CMU_GATE_RSTNSYNC_USI1  0x10b0          /* (0x20b0) */
#define CMU_GATE_IPCLK_USI1     0x10f4          /* CLK_BLK_HSI0_UID_USI1_HSI0_IPCLKPORT_IPCLK (0x20f4) */
#define CMU_GATE_PCLK_USI1      0x10f8          /* ..._PCLK (0x20f8) */
#define CMU_GATE_USI1           0x111c          /* CLK_CON_GAT_GATE_CLK_HSI0_USI1 (0x211c) */
#define CMU_GATE_ON             0x00300000      /* MANUAL and CG_VAL: held on */
#define SYSREG_PHYS             0x11021000ULL   /* sysreg_hsi0 */
#define SYSREG_USI1_MODE        0x018           /* samsung,usi-offset = <0x1018> */
#define USI_MODE_SPI            0x2

/* the pins: an Exynos bank is CON (4 bits a pin) at +0, DAT at +4 */
#define RST_L_PHYS              0x10840000ULL   /* gpp1 of pinctrl@10840000 (PERIC0) at +0x20: pin 1 */
#define RST_L_BANK              0x20
#define RST_L_PIN               1
#define RST_R_PHYS              0x12040000ULL   /* gph2 of pinctrl@12040000 (HSI1) at +0x40: pin 1 */
#define RST_R_BANK              0x40
#define RST_R_PIN               1
#define CS_R_PHYS               0x15060000ULL   /* gpn1 of pinctrl@15060000 at +0x20: pin 0 (active low) */
#define CS_R_BANK               0x20
#define CS_R_PIN                0

/* s3c64xx SPI */
#define CH_CFG                  0x00
#define MODE_CFG                0x08
#define CS_REG                  0x0c
#define INT_EN                  0x10
#define STATUS                  0x14
#define TX_DATA                 0x18
#define RX_DATA                 0x1c
#define PACKET_CNT              0x20
#define PENDING_CLR             0x24
#define SWAP_CFG                0x28
#define USI_CON                 0xc4
#define USI_OPTION              0xc8
#define CH_TXCH_ON              (1u << 0)
#define CH_RXCH_ON              (1u << 1)
#define CH_SW_RST               (1u << 5)
#define CS_INACTIVE             (1u << 0)
#define PACKET_CNT_EN           (1u << 16)
#define USI_CON_RESET           (1u << 0)
#define USI_OPTION_CLKREQ_ON    (1u << 1)
#define USI_OPTION_CLKSTOP_ON   (1u << 2)
#define RX_LVL(s)               (((s) >> 15) & 0x7f)
#define FIFO_INFLIGHT           32

/* CS35L41 (include/sound/cs35l41.h) */
#define CS35L41_DEVID           0x00000000
#define CS35L41_REVID           0x00000004
#define CS35L41_OTPID           0x00000010
#define CS35L41_IRQ1_STATUS3    0x00010018
#define CS35L41_IRQ1_STATUS4    0x0001001c
#define CS35L41_OTP_BOOT_DONE   0x02
#define CS35L41_OTP_BOOT_ERR    0x80000000
#define CS35L41_CHIP_ID         0x35a40
#define CS35L41R_CHIP_ID        0x35b40

/* include/sound/cs35l41.h, generated from it (11-11) */
#define L41_TEST_KEY_CTL             0x00000040
#define L41_PWR_CTRL1                0x00002014
#define L41_PWR_CTRL2                0x00002018
#define L41_PWR_CTRL3                0x0000201c
#define L41_GLOBAL_CLK_CTRL          0x00002c0c
#define L41_PLL_CLK_CTRL             0x00002c04
#define L41_SP_ENABLES               0x00004800
#define L41_INPUT_SRC_VMON_EARLY     0x18            /* (ASP TX sources: VMON, IMON -- L41_INPUT_SRC_* below, for the DSP) */
#define L41_INPUT_SRC_IMON_EARLY     0x19
#define L41_SP_RATE_CTRL             0x00004804
#define L41_SP_FORMAT                0x00004808
#define L41_SP_HIZ_CTRL              0x0000480c
#define L41_SP_FRAME_TX_SLOT         0x00004810
#define L41_SP_FRAME_RX_SLOT         0x00004820
#define L41_SP_TX_WL                 0x00004830
#define L41_SP_RX_WL                 0x00004840
#define L41_DAC_PCM1_SRC             0x00004c00
#define L41_AMP_DIG_VOL_CTRL         0x00006000
#define L41_AMP_GAIN_CTRL            0x00006c04
#define L41_IRQ1_STATUS1             0x00010010
#define L41_IRQ1_STATUS2             0x00010014
#define L41_IRQ1_STATUS3             0x00010018
#define L41_IRQ1_STATUS4             0x0001001c
#define L41_IRQ1_MASK1               0x00010110
#define L41_IRQ1_MASK3               0x00010118
#define L41_GPIO_PAD_CONTROL         0x0000242c
#define L41_GPIO1_CTRL1              0x00011008
#define L41_GPIO2_CTRL1              0x0001100c
#define L41_BSTCVRT_COEFF            0x00003810
#define L41_BSTCVRT_SLOPE_LBST       0x00003814
#define L41_BSTCVRT_PEAK_CUR         0x00003808
#define L41_TST_FS_MON0              0x00002d10
#define L41_DSP1_CCM_CORE_CTRL       0x02bc1000
#define L41_VIMON_SPKMON_RESYNC      0x00004100
#define L41_VPVBST_FS_SEL            0x00004400
#define L41_BSTCVRT_DCM_CTRL         0x0000381c
#define L41_DSP1_YM_ACCEL_PL0_PRI    0x02bc20e0
#define L41_DSP1_XM_ACCEL_PL0_PRI    0x02bc2020
#define L41_ASP_TX3_SRC              0x00004c28
#define L41_ASP_TX4_SRC              0x00004c2c
#define L41_OTP_MEM0                 0x00000400
#define L41_OTP_SIZE_WORDS           0x00000020
#define L41_NG_CFG                   0x00006808
#define L41_INT1_MASK_DEFAULT        0x7ffcfe3f
#define L41_GLOBAL_EN_MASK           0x00000001
#define L41_PUP_DONE_MASK            0x01000000
#define L41_PDN_DONE_MASK            0x00800000
#define L41_BST_EN_MASK              0x00000030
#define L41_BST_EN_SHIFT             0x00000004
#define L41_BST_EN_DEFAULT           0x00000002
#define L41_BST_K1_MASK              0x000000ff
#define L41_BST_K1_SHIFT             0x00000000
#define L41_BST_K2_MASK              0x0000ff00
#define L41_BST_K2_SHIFT             0x00000008
#define L41_BST_SLOPE_MASK           0x0000ff00
#define L41_BST_SLOPE_SHIFT          0x00000008
#define L41_BST_LBST_VAL_MASK        0x00000003
#define L41_BST_LBST_VAL_SHIFT       0x00000000
#define L41_BST_IPK_MASK             0x0000007f
#define L41_BST_IPK_SHIFT            0x00000000
#define L41_ASP_DOUT_HIZ_MASK        0x00000003
#define L41_GPIO2_CTRL_MASK          0x07000000
#define L41_GPIO2_CTRL_SHIFT         0x00000018
#define L41_GPIO_POL_MASK            0x00001000
#define L41_GPIO_DIR_MASK            0x80000000
#define L41_GPIO_DIR_SHIFT           0x0000001f
#define L41_GLOBAL_FS_MASK           0x0000001f
#define L41_GLOBAL_FS_SHIFT          0x00000000
#define L41_ASP_WIDTH_RX_MASK        0xff000000
#define L41_ASP_WIDTH_RX_SHIFT       0x00000018
#define L41_ASP_RX_WL_MASK           0x0000003f
#define L41_ASP_RX_WL_SHIFT          0x00000000
#define L41_ASP_FMT_MASK             0x00000700
#define L41_ASP_FMT_SHIFT            0x00000008
#define L41_SCLK_MSTR_MASK           0x00000010
#define L41_LRCLK_MSTR_MASK          0x00000001
#define L41_SCLK_INV_MASK            0x00000040
#define L41_LRCLK_INV_MASK           0x00000004
#define L41_PLL_OPENLOOP_MASK        0x00000800
#define L41_PLL_OPENLOOP_SHIFT       0x0000000b
#define L41_REFCLK_FREQ_MASK         0x000007e0
#define L41_REFCLK_FREQ_SHIFT        0x00000005
#define L41_PLL_CLK_EN_MASK          0x00000010
#define L41_PLL_CLK_EN_SHIFT         0x00000004
#define L41_PLL_CLK_SEL_MASK         0x00000007
#define L41_PLLSRC_SCLK              0x00000000
#define L41_FS2_WINDOW_MASK          0x00fff800
#define L41_FS2_WINDOW_SHIFT         0x0000000c
#define L41_ASP_SOURCE_MASK          0x0000007f
#define L41_AMP_GAIN_PCM_MASK        0x000003e0
#define L41_AMP_GAIN_PCM_SHIFT       0x00000005
#define L41_AMP_SHORT_ERR            0x80000000
#define L41_TEMP_WARN                0x00008000
#define L41_TEMP_ERR                 0x00020000
#define L41_BST_OVP_ERR              0x00000040
#define L41_BST_DCM_UVP_ERR          0x00000080
#define L41_BST_SHORT_ERR            0x00000100
#define L41_PLL_LOCK                 0x00000002
#define L41_AMP_VOL_PCM_MASK         0x00003ff8      /* AMP_DIG_VOL_CTRL bits 13:3, 1/8 dB steps, signed; 0x400 mute */
#define L41_AMP_VOL_PCM_SHIFT        3
#define L41_AMP_EN_MASK              0x00000001      /* PWR_CTRL2: "Main AMP" */
#define L41_CLASSH_EN_MASK           0x00000010      /* PWR_CTRL3 bit 4: "CLASS H" */
#define L41_ASP_RX1_EN_MASK          0x00010000      /* SP_ENABLES bit 16: "ASPRX1" */
#define L41_DACPCM_ASPRX1            0x08            /* DAC_PCM1_SRC: "PCM Source" = ASP */
#define L41_GPIO2_INT_PUSH_PULL_LOW  4               /* cirrus,gpio-src-select = <4> */
#define L41_FS_48K                   0x03
#define L41_REFCLK_6144000           0x28            /* cs35l41_pll_sysclk */
#define L41_FS_MON_6144000           (16u | (24u << 12))     /* cs35l41_fs_mon: fs1 16, fs2 24 */
#define L41_REVID_B2                 0xb2
#define L41_REVID_REG                0x00000004
#define L41_OTPID_REG                0x00000010

/* Linux cs35l41-lib.c otp_map_1 (OTP ids 0x01, 0x08; bit offset 16, word offset 2): register, shift, size */
static const struct { uint32_t reg; uint8_t shift, size; } otp_map_1[] = {
	{ 0x00002030, 0, 4 },
	{ 0x00002030, 7, 1 },
	{ 0x0000208c, 24, 6 },
	{ 0x00002090, 14, 4 },
	{ 0x00002090, 10, 4 },
	{ 0x0000300c, 11, 4 },
	{ 0x0000394c, 23, 2 },
	{ 0x00003950, 0, 7 },
	{ 0x00003950, 8, 7 },
	{ 0x00003950, 16, 8 },
	{ 0x00003950, 24, 8 },
	{ 0x00003954, 0, 7 },
	{ 0x00003954, 8, 7 },
	{ 0x00003954, 16, 8 },
	{ 0x00003954, 24, 8 },
	{ 0x00003958, 0, 7 },
	{ 0x00003958, 8, 7 },
	{ 0x00003958, 16, 8 },
	{ 0x00003958, 24, 8 },
	{ 0x0000395c, 0, 7 },
	{ 0x0000395c, 8, 7 },
	{ 0x0000395c, 16, 8 },
	{ 0x0000395c, 24, 8 },
	{ 0x0000416c, 0, 8 },
	{ 0x00004160, 0, 7 },
	{ 0x0000416c, 8, 8 },
	{ 0x00004160, 16, 10 },
	{ 0x0000416c, 16, 12 },
	{ 0x0000416c, 28, 1 },
	{ 0x00004170, 0, 6 },
	{ 0x00004170, 6, 1 },
	{ 0x00004170, 8, 6 },
	{ 0x00004170, 14, 1 },
	{ 0x00004170, 16, 9 },
	{ 0x00004360, 0, 5 },
	{ 0x00004360, 6, 9 },
	{ 0x00004448, 0, 8 },
	{ 0x00004448, 8, 8 },
	{ 0x00004448, 16, 8 },
	{ 0x00004448, 24, 8 },
	{ 0x0000444c, 0, 3 },
	{ 0x00006e30, 0, 5 },
	{ 0x00006e30, 8, 5 },
	{ 0x00006e30, 16, 5 },
	{ 0x00006e30, 24, 5 },
	{ 0x00006e34, 0, 5 },
	{ 0x00006e34, 8, 5 },
	{ 0x00006e34, 16, 5 },
	{ 0x00006e34, 24, 5 },
	{ 0x00006e38, 0, 5 },
	{ 0x00006e38, 8, 5 },
	{ 0x00006e38, 16, 5 },
	{ 0x00006e38, 24, 5 },
	{ 0x00006e3c, 0, 5 },
	{ 0x00006e3c, 8, 5 },
	{ 0x00006e3c, 16, 5 },
	{ 0x00006e3c, 24, 5 },
	{ 0x00006e40, 0, 5 },
	{ 0x00006e40, 8, 5 },
	{ 0x00006e40, 16, 5 },
	{ 0x00006e40, 24, 5 },
	{ 0x00006e44, 0, 5 },
	{ 0x00006e48, 0, 10 },
	{ 0x00006e48, 10, 10 },
	{ 0x00006e48, 20, 10 },
	{ 0x00006e4c, 0, 10 },
	{ 0x00006e4c, 10, 10 },
	{ 0x00006e4c, 20, 10 },
	{ 0x00006e50, 0, 10 },
	{ 0x00006e50, 10, 10 },
	{ 0x00006e50, 20, 10 },
	{ 0x00006e54, 0, 10 },
	{ 0x00006e54, 10, 10 },
	{ 0x00006e54, 20, 10 },
	{ 0x00006e58, 0, 10 },
	{ 0x00006e58, 10, 10 },
	{ 0x00006e58, 20, 10 },
	{ 0x00006e5c, 0, 10 },
	{ 0x00006e5c, 10, 10 },
	{ 0x00006e5c, 20, 10 },
	{ 0x00006e60, 0, 10 },
	{ 0x00006e60, 10, 10 },
	{ 0x00006e60, 20, 10 },
	{ 0x00006e64, 0, 10 },
	{ 0x00007418, 7, 5 },
	{ 0x0000741c, 0, 5 },
	{ 0x0000741c, 11, 4 },
	{ 0x0000741c, 19, 4 },
	{ 0x00007434, 17, 1 },
	{ 0x00007434, 18, 7 },
	{ 0x00007068, 0, 9 },
	{ 0x0000410c, 7, 1 },
	{ 0x0000400c, 0, 7 },
	{ 0x00000000, 0, 1 },
	{ 0x00017040, 0, 8 },
	{ 0x00017040, 8, 8 },
	{ 0x00017040, 16, 8 },
	{ 0x00017040, 24, 8 },
	{ 0x00017044, 0, 24 },
};
/* 99 entries */


static vm_offset_t spi, cmu, sysreg, rst_l, rst_r, cs_r;
static uint32_t mode;
/* the controller's chip-select register while the right amplifier (its own chip select a GPIO) is spoken to: Google's
 * spi-s3c64xx.c with samsung,spi-chip-select-mode = <2> (AUTO_CS_MODE_FORCE_QUIESCE) sets SLAVE_AUTO | NSC_CNT_2 on top of
 * what is there (SIG_INACT); in manual mode with its own select inactive the controller does not clock at all (11-11) */
#define CS_AUTO                 (1u << 1)
#define CS_NSC_CNT_2            (2u << 4)
static uint32_t cs_reg_right = CS_INACTIVE | CS_AUTO | CS_NSC_CNT_2;

static inline uint32_t
rd(vm_offset_t a)
{
	return *(volatile uint32_t *)a;
}

static inline void
wr(vm_offset_t a, uint32_t v)
{
	*(volatile uint32_t *)a = v;
}

static void
sleep_ms(uint32_t ms)
{
	delay_for_interval(ms, NSEC_PER_MSEC);
}

/* pin PIN of the bank at BANK an output at LEVEL */
static void
pin_out(vm_offset_t bank, int pin, int level)
{
	uint32_t dat = rd(bank + 4);

	wr(bank + 4, level ? (dat | (1u << pin)) : (dat & ~(1u << pin)));
	wr(bank, (rd(bank) & ~(0xfu << (pin * 4))) | (1u << (pin * 4)));
}

/* ---- the clock, the USI block, the controller (as the touch screen's) --------------------------------------------- */

static void
spi_clock_on(void)
{
	wr(cmu + CMU_GATE_PCLK_USI1, CMU_GATE_ON);
	wr(cmu + CMU_GATE_RSTNSYNC_USI1, CMU_GATE_ON);
	wr(cmu + CMU_GATE_USI1, CMU_GATE_ON);
	wr(cmu + CMU_MUX_USI1, 1);
	wr(cmu + CMU_DIV_USI1, 0);
	wr(cmu + CMU_GATE_IPCLK_USI1, CMU_GATE_ON);
}

static bool
spi_init(void)
{
	uint32_t v;

	wr(sysreg + SYSREG_USI1_MODE, USI_MODE_SPI);
	wr(spi + USI_CON, rd(spi + USI_CON) & ~USI_CON_RESET);
	delay(10);
	v = rd(spi + USI_OPTION);
	wr(spi + USI_OPTION, (v & ~USI_OPTION_CLKSTOP_ON) | USI_OPTION_CLKREQ_ON);

	wr(spi + CH_CFG, CH_SW_RST);
	delay(10);
	wr(spi + CH_CFG, 0);
	wr(spi + MODE_CFG, 0);
	wr(spi + CS_REG, CS_INACTIVE);
	wr(spi + INT_EN, 0);
	wr(spi + SWAP_CFG, 0);
	wr(spi + PENDING_CLR, 0x1f);
	wr(spi + PENDING_CLR, 0);

	/* the controller answers: a field of MODE_CFG written and read back */
	wr(spi + MODE_CFG, 0x55u << 5);
	v = rd(spi + MODE_CFG);
	wr(spi + MODE_CFG, 0);
	printf("akita amp: SPI controller at 0x%llx: MODE_CFG read back 0x%x (0x%x written), STATUS 0x%x\n", SPI_PHYS, v,
	    0x55u << 5, rd(spi + STATUS));
	return v == (0x55u << 5);
}

/* one transfer to chip CHIP (0 left, 1 right): LENGTH bytes out of TX, as many in to RX */
static bool
spi_transfer(int chip, const uint8_t *tx, uint8_t *rx, uint32_t length)
{
	uint32_t sent = 0, got = 0;
	uint64_t deadline;

	wr(spi + CH_CFG, CH_SW_RST);
	wr(spi + CH_CFG, 0);
	wr(spi + PACKET_CNT, (length & 0xffff) | PACKET_CNT_EN);
	wr(spi + CH_CFG, CH_TXCH_ON | CH_RXCH_ON);
	if (chip == 0) {
		wr(spi + CS_REG, 0);
	} else {
		pin_out(cs_r, CS_R_PIN, 0);
		wr(spi + CS_REG, cs_reg_right);
	}
	clock_interval_to_deadline(50, NSEC_PER_MSEC, &deadline);
	while (got < length && mach_absolute_time() < deadline) {
		while (sent < length && sent - got < FIFO_INFLIGHT) {
			wr(spi + TX_DATA, tx ? tx[sent] : 0);
			sent++;
		}
		for (uint32_t n = RX_LVL(rd(spi + STATUS)); n > 0 && got < length; n--) {
			uint8_t b = (uint8_t)rd(spi + RX_DATA);

			if (rx) {
				rx[got] = b;
			}
			got++;
		}
	}
	if (chip == 0) {
		wr(spi + CS_REG, CS_INACTIVE);
	} else {
		pin_out(cs_r, CS_R_PIN, 1);
		wr(spi + CS_REG, CS_INACTIVE);
	}
	wr(spi + CH_CFG, 0);
	wr(spi + PACKET_CNT, 0);
	return got == length;
}

/* ---- the CS35L41's registers -------------------------------------------------------------------------------------- */

static bool
amp_read(int chip, uint32_t reg, uint32_t *value)
{
	uint8_t tx[10] = { (uint8_t)((reg >> 24) | 0x80), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg };
	uint8_t rx[10];

	if (!spi_transfer(chip, tx, rx, sizeof(tx))) {
		return false;
	}
	*value = ((uint32_t)rx[6] << 24) | ((uint32_t)rx[7] << 16) | ((uint32_t)rx[8] << 8) | rx[9];
	return true;
}

static bool
amp_write(int chip, uint32_t reg, uint32_t value)
{
	uint8_t tx[10] = { (uint8_t)(reg >> 24), (uint8_t)(reg >> 16), (uint8_t)(reg >> 8), (uint8_t)reg, 0, 0,
		           (uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)value };

	return spi_transfer(chip, tx, NULL, sizeof(tx));
}

static bool
amp_update(int chip, uint32_t reg, uint32_t mask, uint32_t value)
{
	uint32_t v;

	if (!amp_read(chip, reg, &v)) {
		return false;
	}
	return amp_write(chip, reg, (v & ~mask) | (value & mask));
}

static void
amp_test_key(int chip, bool unlock)
{
	amp_write(chip, L41_TEST_KEY_CTL, unlock ? 0x55 : 0xcc);
	amp_write(chip, L41_TEST_KEY_CTL, unlock ? 0xaa : 0x33);
}

/* cs35l41_otp_unpack (the test key unlocked): the chip's factory trims from its OTP into their registers */
static int
amp_otp_unpack(int chip)
{
	uint32_t mem[L41_OTP_SIZE_WORDS], val;
	int bit = 16, word = 2, written = 0;

	for (uint32_t i = 0; i < L41_OTP_SIZE_WORDS; i++) {
		if (!amp_read(chip, L41_OTP_MEM0 + 4 * i, &mem[i])) {
			return -1;
		}
	}
	for (unsigned i = 0; i < sizeof(otp_map_1) / sizeof(otp_map_1[0]); i++) {
		int size = otp_map_1[i].size;

		if (bit + size - 1 >= 32) {             /* across two words */
			val = (mem[word] >> bit) & (uint32_t)((1ull << (32 - bit)) - 1);
			word++;
			val |= (mem[word] & (uint32_t)((1ull << (bit + size - 32)) - 1)) << (32 - bit);
			bit += size - 32;
		} else {
			val = (mem[word] >> bit) & (uint32_t)((1ull << size) - 1);
			bit += size;
		}
		if (bit == 32) {
			bit = 0;
			word++;
		}
		if (otp_map_1[i].reg != 0) {
			uint32_t mask = (uint32_t)(((1ull << size) - 1) << otp_map_1[i].shift);
			amp_update(chip, otp_map_1[i].reg, mask, val << otp_map_1[i].shift);
			written++;
		}
	}
	return written;
}

/* chip CHIP set up for the AoC's TDM_0, powered down still (cs35l41_probe's second half, set_pdata, set_dai_fmt,
 * hw_params, set_sysclk, the DAPM path ASPRX1 -> PCM Source -> CLASS H) */
static bool
amp_setup(int chip)
{
	const char *name = chip ? "right" : "left";
	uint32_t revid = 0, otpid = 0;

	amp_read(chip, L41_REVID_REG, &revid);
	amp_read(chip, L41_OTPID_REG, &otpid);
	if ((revid & 0xff) != L41_REVID_B2 || (otpid & 0xf) != 0x8) {
		printf("akita amp: %s: revision 0x%x, OTP id 0x%x -- only B2 with OTP map 8 is known here; not set up\n", name,
		    revid, otpid);
		return false;
	}
	amp_test_key(chip, true);
	/* cs35l41_revb2_errata_patch, then DSP1_CCM_CORE_CTRL 0 */
	amp_write(chip, L41_VIMON_SPKMON_RESYNC, 0);
	amp_write(chip, 0x4310, 0);
	amp_write(chip, L41_VPVBST_FS_SEL, 0);
	amp_write(chip, L41_BSTCVRT_DCM_CTRL, 0x51);
	amp_write(chip, L41_DSP1_YM_ACCEL_PL0_PRI, 0);
	amp_write(chip, L41_DSP1_XM_ACCEL_PL0_PRI, 0);
	amp_write(chip, L41_PWR_CTRL2, 0);
	amp_write(chip, L41_AMP_GAIN_CTRL, 0);
	amp_write(chip, L41_ASP_TX3_SRC, 0);
	amp_write(chip, L41_ASP_TX4_SRC, 0);
	amp_write(chip, L41_DSP1_CCM_CORE_CTRL, 0);
	int trims = amp_otp_unpack(chip);
	amp_test_key(chip, false);

	/* GPIO1 an input (unused), GPIO2 the interrupt line, push-pull active low; the critical interrupts unmasked */
	amp_update(chip, L41_GPIO1_CTRL1, L41_GPIO_POL_MASK | L41_GPIO_DIR_MASK, L41_GPIO_DIR_MASK);
	amp_update(chip, L41_GPIO2_CTRL1, L41_GPIO_POL_MASK | L41_GPIO_DIR_MASK, 0);
	amp_update(chip, L41_GPIO_PAD_CONTROL, L41_GPIO2_CTRL_MASK, L41_GPIO2_INT_PUSH_PULL_LOW << L41_GPIO2_CTRL_SHIFT);
	amp_write(chip, L41_IRQ1_MASK1, L41_INT1_MASK_DEFAULT);

	/* cs35l41_boost_config: 1000 nH -> LBST 0; 4 uF -> range 0; 3500 mA -> ((3500 - 1600) / 50) + 0x10 */
	amp_update(chip, L41_BSTCVRT_COEFF, L41_BST_K1_MASK | L41_BST_K2_MASK, (0x24u << L41_BST_K1_SHIFT) | (0x24u << L41_BST_K2_SHIFT));
	amp_update(chip, L41_BSTCVRT_SLOPE_LBST, L41_BST_SLOPE_MASK | L41_BST_LBST_VAL_MASK, 0x75u << L41_BST_SLOPE_SHIFT);
	amp_update(chip, L41_BSTCVRT_PEAK_CUR, L41_BST_IPK_MASK, ((3500u - 1600u) / 50u + 0x10u) << L41_BST_IPK_SHIFT);
	amp_update(chip, L41_PWR_CTRL2, L41_BST_EN_MASK, L41_BST_EN_DEFAULT << L41_BST_EN_SHIFT);
	amp_update(chip, L41_SP_HIZ_CTRL, L41_ASP_DOUT_HIZ_MASK, 3);   /* cirrus,asp-sdout-hiz = <3> */

	/* DSP-A, the AoC the clock provider, neither clock inverted; 48 kHz; 32-bit slots (the word length below) */
	amp_update(chip, L41_SP_FORMAT, L41_SCLK_MSTR_MASK | L41_LRCLK_MSTR_MASK | L41_ASP_FMT_MASK | L41_LRCLK_INV_MASK |
	    L41_SCLK_INV_MASK, 0);
	amp_update(chip, L41_GLOBAL_CLK_CTRL, L41_GLOBAL_FS_MASK, L41_FS_48K << L41_GLOBAL_FS_SHIFT);
	amp_update(chip, L41_SP_FORMAT, L41_ASP_WIDTH_RX_MASK, 32u << L41_ASP_WIDTH_RX_SHIFT);
	{
		/*
		 * The word length the amps take from each 32-bit slot: 32, as Android's mixer gives them (11-12; it was 16 --
		 * the speakers silent, the earpiece buzzing faintly: if the AoC puts its samples in the LOW bits of a slot, a
		 * 16-bit word is only their sign bits. 16-bit samples at the top read the same with 32). UNTESTED by ear.
		 * amp_wl=N: N (16, 24, 32).
		 */
		uint32_t wl = 32;

		PE_parse_boot_argn("amp_wl", &wl, sizeof(wl));
		if (wl != 16 && wl != 24 && wl != 32) {
			wl = 32;
		}
		amp_update(chip, L41_SP_RX_WL, L41_ASP_RX_WL_MASK, wl << L41_ASP_RX_WL_SHIFT);
	}
	/* the left amplifier plays slot 0, the right slot 1 (RX1 bits 5:0, RX2 bits 13:8) */
	amp_update(chip, L41_SP_FRAME_RX_SLOT, 0x3f3f, chip ? 0x0001 : 0x0100);
	{
		/*
		 * amp_vi=1: the speakers' voltage and current back to the AoC on TDM_0_TX, as Android's mixer has them (ASP TX1
		 * VMON, TX2 IMON; left TX1 slot 0 / TX2 slot 2, right 1 / 3; 32-bit) -- 11-12, the AoC sending silence: does its
		 * speaker chain wait for the feedback? (This driver never sent it: SDOUT high-impedance.)
		 */
		uint32_t vi = 0;

		PE_parse_boot_argn("amp_vi", &vi, sizeof(vi));
		if (vi) {
			amp_write(chip, 0x00004c20, L41_INPUT_SRC_VMON_EARLY);     /* ASP_TX1_SRC */
			amp_write(chip, 0x00004c24, L41_INPUT_SRC_IMON_EARLY);     /* ASP_TX2_SRC */
			amp_update(chip, L41_SP_FRAME_TX_SLOT, 0x3f3f, chip ? 0x0301 : 0x0200);
			amp_update(chip, L41_SP_FORMAT, 0x00ff0000, 32u << 16);    /* ASP_WIDTH_TX */
			amp_update(chip, L41_SP_TX_WL, 0x3f, 32);
			amp_update(chip, L41_PWR_CTRL2, 0x3000, 0x3000);           /* VMON, IMON on */
			amp_update(chip, L41_SP_ENABLES, 0x3, 0x3);                /* ASP TX1, TX2 */
			if (chip == 0) {
				printf("akita amp: amp_vi=1: VMON/IMON back to the AoC on TDM_0_TX (slots 0-3)\n");
			}
		}
	}

	/* the PLL from SCLK (6.144 MHz = 48 kHz x 4 slots x 32 bits); the frame-sync monitor for that clock */
	amp_update(chip, L41_PLL_CLK_CTRL, L41_PLL_OPENLOOP_MASK, L41_PLL_OPENLOOP_MASK);
	amp_update(chip, L41_PLL_CLK_CTRL, L41_REFCLK_FREQ_MASK, L41_REFCLK_6144000 << L41_REFCLK_FREQ_SHIFT);
	amp_update(chip, L41_PLL_CLK_CTRL, L41_PLL_CLK_EN_MASK, 0);
	amp_update(chip, L41_PLL_CLK_CTRL, L41_PLL_CLK_SEL_MASK, L41_PLLSRC_SCLK);
	amp_update(chip, L41_PLL_CLK_CTRL, L41_PLL_OPENLOOP_MASK, 0);
	amp_update(chip, L41_PLL_CLK_CTRL, L41_PLL_CLK_EN_MASK, L41_PLL_CLK_EN_MASK);
	amp_write(chip, L41_TST_FS_MON0, L41_FS_MON_6144000);

	/* the path without the DSP: ASP RX1 on, the DAC fed from it, class H; 0 dB digital, the least analog gain */
	amp_update(chip, L41_SP_ENABLES, L41_ASP_RX1_EN_MASK, L41_ASP_RX1_EN_MASK);
	amp_update(chip, L41_DAC_PCM1_SRC, L41_ASP_SOURCE_MASK, L41_DACPCM_ASPRX1);
	amp_update(chip, L41_PWR_CTRL3, L41_CLASSH_EN_MASK, L41_CLASSH_EN_MASK);
	amp_update(chip, L41_AMP_DIG_VOL_CTRL, L41_AMP_VOL_PCM_MASK, 0);
	{
		/* the analog gain: the least (0: 0.5 dB) unless amp_gain=N (0-20, 0.5 dB + N dB; Android's mixer: 17) */
		uint32_t gain = 0;

		PE_parse_boot_argn("amp_gain", &gain, sizeof(gain));
		if (gain > 20) {
			gain = 0;
		}
		amp_update(chip, L41_AMP_GAIN_CTRL, L41_AMP_GAIN_PCM_MASK, gain << L41_AMP_GAIN_PCM_SHIFT);
	}

	uint32_t fmt = 0, slots = 0, pll = 0, pwr2 = 0, src = 0;
	amp_read(chip, L41_SP_FORMAT, &fmt);
	amp_read(chip, L41_SP_FRAME_RX_SLOT, &slots);
	amp_read(chip, L41_PLL_CLK_CTRL, &pll);
	amp_read(chip, L41_PWR_CTRL2, &pwr2);
	amp_read(chip, L41_DAC_PCM1_SRC, &src);
	printf("akita amp: %s: set up: %d OTP trims, SP_FORMAT 0x%x, RX slots 0x%x, PLL_CLK_CTRL 0x%x, PWR_CTRL2 0x%x, DAC source 0x%x\n",
	    name, trims, fmt, slots, pll, pwr2, src);
	return trims > 0;
}

static bool amps_ready;
static uint32_t speaker_allowed;                /* boot-arg speaker=1: akita_amp_speaker(1) may power them up (audible) */

/* ---- amp=3: the speaker protection firmware in the HALO DSP (Linux's cs_dsp.c; the CS40L26's loader, drivers/haptic) */

extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);

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
#define MAX_COEFFS              128
#define MAX_ALGOS               16
#define L41_DSP1_RX1_RATE       0x02b80080      /* .. RX8 every 8 bytes; TX1-8 at 0x02b80280 */
#define L41_DSP1_TX1_RATE       0x02b80280
#define L41_DSP1_RX5_SRC        0x00004c50      /* RX6 +4, RX7 +8, RX8 +12 */
#define L41_INPUT_SRC_VPMON     0x28
#define L41_INPUT_SRC_VBSTMON   0x29
#define L41_INPUT_SRC_TEMPMON   0x3a
#define L41_INPUT_SRC_RSVD      0x3b

struct coeff {
	char name[32];
	uint32_t algo;
	uint16_t type, offset;
	uint32_t reg;
};
static struct coeff coeffs[MAX_COEFFS];
static unsigned ncoeffs;
static struct { uint32_t id, rev, xm_base, xm_size, ym_base, ym_size; } algos[MAX_ALGOS];
static unsigned nalgos;
static uint8_t chunk[6 + 4096];                 /* a block's address, padding and up to 4096 bytes, for one transfer */

static uint32_t
le(const uint8_t *p, unsigned n)
{
	uint32_t v = 0;

	for (unsigned i = 0; i < n; i++) {
		v |= (uint32_t)p[i] << (8 * i);
	}
	return v;
}

/* LEN bytes as they are (the files are in the bus's order) from register REG on, 4096 at a time: one SPI write each */
static bool
raw_write(int chip, uint32_t reg, const uint8_t *data, uint32_t len)
{
	for (uint32_t done = 0; done < len;) {
		uint32_t n = len - done > 4096 ? 4096 : len - done, at = reg + done;

		chunk[0] = (uint8_t)(at >> 24);
		chunk[1] = (uint8_t)(at >> 16);
		chunk[2] = (uint8_t)(at >> 8);
		chunk[3] = (uint8_t)at;
		chunk[4] = chunk[5] = 0;
		memcpy(chunk + 6, data + done, n);
		if (!spi_transfer(chip, chunk, NULL, 6 + n)) {
			printf("akita amp: %s: %u bytes at %#x NOT written\n", chip ? "right" : "left", n, at);
			return false;
		}
		done += n;
	}
	return true;
}

/* the first and the last word of a block read back as written. XM and YM packed hold four 24-bit words in three
 * registers, least significant first (w0 = R0 bits 23:0, w1 = R0 31:24 + R1 15:0, w2 = R1 31:16 + R2 7:0, w3 = R2 31:8),
 * and a register past a group's first does not read back as written (it reads 0): their last word is read through the
 * unpacked window (the 24-bit word at UNPACKED24 + 4 x word) instead -- the last group's w3, the last register >> 8 */
static bool
raw_check(int chip, uint32_t reg, const uint8_t *data, uint32_t len)
{
	uint32_t first = 0, last = 0, want_last, last_reg;

	if (len < 4) {
		return true;
	}
	uint32_t want_first = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
	if ((reg >= HALO_XM_PACKED && reg < HALO_XM_UNPACKED24) || (reg >= HALO_YM_PACKED && reg < HALO_YM_UNPACKED24)) {
		uint32_t base = reg >= HALO_YM_PACKED ? HALO_YM_PACKED : HALO_XM_PACKED;
		uint32_t unpacked = base == HALO_YM_PACKED ? HALO_YM_UNPACKED24 : HALO_XM_UNPACKED24;
		uint32_t word = (reg - base) / 3 + len / 3 - 1, at = len - 4;

		if (len % 12 != 0 || (reg - base) % 12 != 0) {
			return amp_read(chip, reg, &first) && first == want_first;  /* (not whole groups: the first only) */
		}
		last_reg = unpacked + 4 * word;
		want_last = (((uint32_t)data[at] << 24) | ((uint32_t)data[at + 1] << 16) | ((uint32_t)data[at + 2] << 8) | data[at + 3]) >> 8;
		if (amp_read(chip, last_reg, &last)) {
			last &= 0xffffff;
		}
	} else {
		uint32_t at = (len - 4) & ~3u;

		last_reg = reg + at;
		want_last = ((uint32_t)data[at] << 24) | ((uint32_t)data[at + 1] << 16) | ((uint32_t)data[at + 2] << 8) | data[at + 3];
		amp_read(chip, last_reg, &last);
	}
	bool ok = amp_read(chip, reg, &first) && first == want_first && last == want_last;
	static unsigned said;
	if (!ok && said++ < 12) {
		printf("akita amp: %s: the block at %#x (%u bytes) reads back %#010x .. %#010x (at %#x), written %#010x .. %#010x\n",
		    chip ? "right" : "left", reg, len, first, last, last_reg, want_first, want_last);
	}
	return ok;
}

/* an ALGO_INFO block of the .wmfw: an algorithm's controls, their names, types and offsets */
static void
algo_info(const uint8_t *d, uint32_t len)
{
	uint32_t pos, id, count;
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

/* the .wmfw into chip CHIP: its memory blocks written and checked; its algorithms' controls kept (the first time) */
static bool
load_wmfw(int chip, const char *file)
{
	uint64_t size = 0;
	const uint8_t *f = hakios_image_file("fw", file, &size, NULL);
	uint32_t pos = 40, blocks = 0, bytes = 0, bad = 0;
	bool keep = ncoeffs == 0;

	if (f == NULL || size < 40 || memcmp(f, "WMFW", 4) != 0 || le(f + 4, 4) != 40 || f[10] != 0x04 || (size & 3)) {
		printf("akita amp: fw/%s %s\n", file, f == NULL ? "not in the image (../fw-audio of the build)" : "not a HALO .wmfw");
		return false;
	}
	while (pos + 8 <= size) {
		uint32_t offset = le(f + pos, 3), type = f[pos + 3], len = le(f + pos + 4, 4), reg = 0;
		const uint8_t *d = f + pos + 8;

		if (pos + 8 + len > size) {
			printf("akita amp: fw/%s: a block past the end\n", file);
			return false;
		}
		switch (type) {
		case T_PM_PACKED: reg = HALO_PM + offset * 5; break;
		case T_XM_PACKED: reg = HALO_XM_PACKED + offset * 3; break;
		case T_XM_UNPACKED: reg = HALO_XM_UNPACKED24 + offset * 4; break;
		case T_YM_PACKED: reg = HALO_YM_PACKED + offset * 3; break;
		case T_YM_UNPACKED: reg = HALO_YM_UNPACKED24 + offset * 4; break;
		case T_ALGO_INFO:
			if (keep) {
				algo_info(d, len);
			}
			break;
		case T_INFO: break;
		default:
			printf("akita amp: fw/%s: a block of type %#x\n", file, type);
			return false;
		}
		if (reg != 0) {
			if (!raw_write(chip, reg, d, len)) {
				return false;
			}
			if (!raw_check(chip, reg, d, len)) {
				bad++;
			}
			blocks++;
			bytes += len;
		}
		pos += 8 + ((len + 3) & ~3u);
	}
	printf("akita amp: %s: fw/%s: %u blocks, %u bytes written, %s; %u controls\n", chip ? "right" : "left", file, blocks,
	    bytes, bad ? "SOME DID NOT READ BACK" : "all read back", ncoeffs);
	return bad == 0;
}

/* the algorithm list the firmware keeps in XM, and the controls' registers from it */
static bool
algorithms(int chip)
{
	uint32_t n = 0, at = HALO_FW_ID, term = 0;

	if (!amp_read(chip, HALO_NUM_ALGOS, &n) || n + 1 > MAX_ALGOS) {
		printf("akita amp: the number of algorithms %u\n", n);
		return false;
	}
	nalgos = n + 1;
	for (unsigned i = 0; i < nalgos; i++) {
		if (!amp_read(chip, at, &algos[i].id) || !amp_read(chip, at + 4, &algos[i].rev) ||
		    !amp_read(chip, at + 8, &algos[i].xm_base) || !amp_read(chip, at + 12, &algos[i].xm_size) ||
		    !amp_read(chip, at + 16, &algos[i].ym_base) || !amp_read(chip, at + 20, &algos[i].ym_size)) {
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
	(void)amp_read(chip, at, &term);
	printf("akita amp: %s: %u algorithms (firmware %#x rev %u.%u.%u):", chip ? "right" : "left", nalgos, algos[0].id,
	    algos[0].rev >> 16 & 0xff, algos[0].rev >> 8 & 0xff, algos[0].rev & 0xff);
	for (unsigned i = 0; i < nalgos; i++) {
		printf(" %#x (XM %#x+%u, YM %#x+%u)", algos[i].id, algos[i].xm_base, algos[i].xm_size, algos[i].ym_base,
		    algos[i].ym_size);
	}
	printf("%s\n", term == ALGO_LIST_TERM ? "" : " -- NO LIST TERMINATOR");
	return term == ALGO_LIST_TERM;
}

/* a .bin (WMDR) into chip CHIP: its blocks written relative to their algorithms (cs_dsp_load_coeff) */
static bool
load_bin(int chip, const char *file)
{
	uint64_t size = 0;
	const uint8_t *f = hakios_image_file("fw", file, &size, NULL);
	uint32_t pos = 16, blocks = 0, bytes = 0, bad = 0;

	if (f == NULL || size < 16 || memcmp(f, "WMDR", 4) != 0 || le(f + 4, 4) != 16 || (size & 3)) {
		printf("akita amp: fw/%s %s\n", file, f == NULL ? "not in the image" : "not a .bin (WMDR)");
		return false;
	}
	while (pos + 20 <= size) {
		uint32_t offset = le(f + pos, 2), type = le(f + pos + 2, 2), algo = le(f + pos + 4, 4) & 0xffff;
		uint32_t len = le(f + pos + 16, 4), reg = 0;
		unsigned i;

		if (pos + 20 + len > size) {
			printf("akita amp: fw/%s: a block past the end\n", file);
			return false;
		}
		if (type != 0xfe00 && type != 0xff00 && type != 0xfc00) {      /* (name, information, metadata: nothing to write) */
			for (i = 0; i < nalgos && (algos[i].id & 0xffff) != algo; i++) {
			}
			if (i == nalgos) {
				printf("akita amp: fw/%s: algorithm %#x not in the firmware\n", file, algo);
				return false;
			}
			switch (type) {
			case T_XM_UNPACKED: reg = HALO_XM_UNPACKED24 + algos[i].xm_base * 4 + offset; break;
			case T_XM_PACKED: reg = ((HALO_XM_PACKED + algos[i].xm_base * 3) & ~3u) + offset; break;
			case T_YM_UNPACKED: reg = HALO_YM_UNPACKED24 + algos[i].ym_base * 4 + offset; break;
			case T_YM_PACKED: reg = ((HALO_YM_PACKED + algos[i].ym_base * 3) & ~3u) + offset; break;
			default:
				printf("akita amp: fw/%s: a block of type %#x\n", file, type);
				return false;
			}
			if (!raw_write(chip, reg, f + pos + 20, len)) {
				return false;
			}
			if (!raw_check(chip, reg, f + pos + 20, len)) {
				bad++;
			}
			blocks++;
			bytes += len;
		}
		pos += 20 + ((len + 3) & ~3u);
	}
	printf("akita amp: %s: fw/%s: %u blocks, %u bytes written, %s\n", chip ? "right" : "left", file, blocks, bytes,
	    bad ? "SOME DID NOT READ BACK" : "all read back");
	return bad == 0;
}

static bool protection_loaded[2], protection_running[2];

#define L41_DSP_MBOX_2          0x00013004      /* CSPL's status: 0 running, 1 paused, 2 ready to reinit, -1 error */
#define L41_DSP_VIRT1_MBOX_1    0x00013020      /* CSPL's command: 1 pause, 2 resume */
#define L41_DSP1_RX1_SRC        0x00004c40
#define L41_INPUT_SRC_ASPRX1    0x08
#define L41_INPUT_SRC_ASPRX2    0x09
#define L41_INPUT_SRC_VMON      0x18
#define L41_INPUT_SRC_IMON      0x19
#define L41_DACPCM_DSP1TX1      0x32
#define L41_MONITORS_EN         0x3700          /* PWR_CTRL2: VMON 12, IMON 13, VPMON 8, VBSTMON 9, TEMPMON 10 */
#define L41_DSP_BASE            0x02b80000      /* CS35L41_DSP1_CTRL_BASE: the HALO core's registers */
#define HALO_CCM_CORE_CONTROL   0x41000
#define HALO_CORE_RESET         0x200
#define HALO_CORE_EN            0x1
#define HALO_CORE_SOFT_RESET    0x00010
#define HALO_MPU_LOCK_CONFIG    0x43140

/* a CSPL mailbox command and the status it should bring (cs35l41_set_cspl_mbox_cmd) */
static bool
cspl_command(int chip, uint32_t cmd, uint32_t want)
{
	uint32_t sts = ~0u;

	amp_write(chip, L41_DSP_VIRT1_MBOX_1, cmd);
	for (int i = 0; i < 5; i++) {
		sleep_ms(1);
		if (amp_read(chip, L41_DSP_MBOX_2, &sts) && sts == want) {
			return true;
		}
		if (sts == 0xffffffffu || sts == 0x00ffffffu) {
			break;
		}
	}
	printf("akita amp: %s: CSPL command %u: status %#x, not %u\n", chip ? "right" : "left", cmd, sts, want);
	return false;
}

/* the protection between the ASP and the DAC, started (the DSP1 widgets' power-up; cs_dsp_run with the MPU opened as
 * cs_dsp_halo_configure_mpu does, lock_regions 0xffffffff) -- with the TDM's clock running */
static bool
protection_start(int chip)
{
	static const uint32_t mpu[] = { 0x43000, 0x43004, 0x43008, 0x4300c, 0x43014, 0x43018, 0x4301c, 0x43020, 0x43024, 0x4302c,
		                        0x43030, 0x43034, 0x43038, 0x4303c, 0x43044, 0x43048, 0x4304c, 0x43050, 0x43054, 0x4305c };
	uint32_t sts = ~0u;

	amp_update(chip, L41_PWR_CTRL2, L41_MONITORS_EN, L41_MONITORS_EN);
	amp_write(chip, L41_DSP1_RX1_SRC, L41_INPUT_SRC_ASPRX1);
	amp_write(chip, L41_DSP1_RX1_SRC + 4, L41_INPUT_SRC_ASPRX1);   /* (Android: DSP RX1 and RX2 both ASPRX1) */
	amp_write(chip, L41_DSP1_RX1_SRC + 8, L41_INPUT_SRC_VMON);
	amp_write(chip, L41_DSP1_RX1_SRC + 12, L41_INPUT_SRC_IMON);
	if (!protection_running[chip]) {
		amp_write(chip, L41_DSP_BASE + HALO_MPU_LOCK_CONFIG, 0x5555);
		amp_write(chip, L41_DSP_BASE + HALO_MPU_LOCK_CONFIG, 0xaaaa);
		for (unsigned i = 0; i < sizeof(mpu) / sizeof(mpu[0]); i++) {
			amp_write(chip, L41_DSP_BASE + mpu[i], 0xffffffffu);
		}
		amp_write(chip, L41_DSP_BASE + HALO_MPU_LOCK_CONFIG, 0);
		amp_update(chip, L41_DSP_BASE + HALO_CCM_CORE_CONTROL, HALO_CORE_RESET | HALO_CORE_EN, HALO_CORE_RESET | HALO_CORE_EN);
		amp_update(chip, L41_DSP_BASE + HALO_CCM_CORE_CONTROL, HALO_CORE_RESET, 0);
		protection_running[chip] = true;
		sleep_ms(5);
	}
	amp_read(chip, L41_DSP_MBOX_2, &sts);
	if (sts != 0 && sts != 1) {                     /* (cs35l41_dsp_audio_ev: running or paused, else invalid) */
		printf("akita amp: %s: the protection's status %#x after its start: the DAC stays on ASP RX1 (unprotected)\n",
		    chip ? "right" : "left", sts);
		return false;
	}
	if (!cspl_command(chip, 2, 0)) {                /* RESUME -> running */
		return false;
	}
	amp_update(chip, L41_DAC_PCM1_SRC, L41_ASP_SOURCE_MASK, L41_DACPCM_DSP1TX1);
	printf("akita amp: %s: THE SPEAKER PROTECTION RUNS (status was %u), the DAC fed from it\n", chip ? "right" : "left", sts);
	return true;
}

/* before powering down: CSPL paused (the DSP1 widget's PRE_PMD), the DAC back on ASP RX1 */
static void
protection_pause(int chip)
{
	if (protection_running[chip]) {
		cspl_command(chip, 1, 1);               /* PAUSE -> paused */
	}
	amp_update(chip, L41_DAC_PCM1_SRC, L41_ASP_SOURCE_MASK, L41_DACPCM_ASPRX1);
}

/* Cirrus's speaker protection into chip CHIP's DSP, not started (cs35l41_dsp_init, then cs_dsp_power_up) */
static bool
amp_load_protection(int chip)
{
	for (uint32_t i = 0; i < 8; i++) {              /* cs35l41_write_fs_errata */
		amp_write(chip, L41_DSP1_RX1_RATE + 8 * i, 1);
		amp_write(chip, L41_DSP1_TX1_RATE + 8 * i, 1);
	}
	amp_write(chip, L41_DSP1_RX5_SRC, L41_INPUT_SRC_VPMON);         /* the internal boost's input: VP */
	amp_write(chip, L41_DSP1_RX5_SRC + 4, L41_INPUT_SRC_VBSTMON);
	amp_write(chip, L41_DSP1_RX5_SRC + 8, L41_INPUT_SRC_TEMPMON);
	amp_write(chip, L41_DSP1_RX5_SRC + 12, L41_INPUT_SRC_RSVD);
	uint64_t t0 = mach_absolute_time(), ns;
	bool ok = load_wmfw(chip, "cs35l41-dsp1-spk-prot.wmfw") && algorithms(chip) &&
	    load_bin(chip, chip ? "R-cs35l41-dsp1-spk-prot.bin" : "cs35l41-dsp1-spk-prot.bin");
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	printf("akita amp: %s: the speaker protection %s in %llu ms (the DSP not started)\n", chip ? "right" : "left",
	    ok ? "LOADED" : "NOT loaded", ns / 1000000);
	protection_loaded[chip] = ok;
	return ok;
}

static void
amp_say_status(int chip, const char *when)
{
	uint32_t st[4] = { 0 }, pwr1 = 0;

	for (int i = 0; i < 4; i++) {
		amp_read(chip, L41_IRQ1_STATUS1 + 4 * i, &st[i]);
	}
	amp_read(chip, L41_PWR_CTRL1, &pwr1);
	printf("akita amp: %s %s: PWR_CTRL1 0x%x, IRQ1_STATUS 0x%x 0x%x 0x%x 0x%x%s%s%s%s%s%s%s\n", chip ? "right" : "left", when,
	    pwr1, st[0], st[1], st[2], st[3], (st[0] & L41_AMP_SHORT_ERR) ? " AMP SHORT" : "", (st[0] & L41_TEMP_ERR) ? " TEMP ERROR" : "",
	    (st[0] & L41_TEMP_WARN) ? " temp warning" : "", (st[0] & L41_BST_OVP_ERR) ? " BOOST OVP" : "",
	    (st[0] & L41_BST_SHORT_ERR) ? " BOOST SHORT" : "", (st[0] & L41_BST_DCM_UVP_ERR) ? " boost UVP" : "",
	    (st[2] & L41_PLL_LOCK) ? " PLL locked" : "");
}

/* both amplifiers on (the AoC's TDM running) or off: Linux's "Main AMP" PRE_PMU / POST_PMD (cs35l41_global_enable) */
int akita_amp_speaker(int on);
int
akita_amp_speaker(int on)
{
	static bool said;

	if (!amps_ready) {
		return -1;
	}
	if (on && !speaker_allowed) {
		/* the tester (11-11): nothing audible unless he is there -- the speakers only with speaker=1 on the command line */
		if (!said) {
			said = true;
			printf("akita amp: asked to play, but speaker=1 is not given: the amplifiers stay off (silent)\n");
		}
		return -1;
	}
	for (int chip = 0; chip < 2; chip++) {
		uint32_t st = 0;
		int ms;

		if (on) {
			if (mode >= 4 && protection_loaded[chip]) {
				protection_start(chip);         /* (on failure the DAC stays on ASP RX1: what amp=2 plays) */
			}
			amp_test_key(chip, true);
			amp_write(chip, 0x2084, 0x002f1aa0);    /* cs35l41_pup_patch */
			amp_test_key(chip, false);
			amp_update(chip, L41_PWR_CTRL1, L41_GLOBAL_EN_MASK, 1);
			for (ms = 0; ms < 100; ms++) {
				if (amp_read(chip, L41_IRQ1_STATUS1, &st) && (st & L41_PUP_DONE_MASK)) {
					break;
				}
				sleep_ms(1);
			}
			amp_write(chip, L41_IRQ1_STATUS1, L41_PUP_DONE_MASK);
			amp_update(chip, L41_PWR_CTRL2, L41_AMP_EN_MASK, L41_AMP_EN_MASK);
			printf("akita amp: %s: powered up (%s after %d ms)\n", chip ? "right" : "left",
			    (st & L41_PUP_DONE_MASK) ? "PUP_DONE" : "NO PUP_DONE", ms);
			amp_say_status(chip, "on");
		} else {
			amp_say_status(chip, "before off");
			if (mode >= 4) {
				protection_pause(chip);
			}
			amp_update(chip, L41_PWR_CTRL2, L41_AMP_EN_MASK, 0);
			amp_update(chip, L41_PWR_CTRL1, L41_GLOBAL_EN_MASK, 0);
			for (ms = 0; ms < 100; ms++) {
				if (amp_read(chip, L41_IRQ1_STATUS1, &st) && (st & L41_PDN_DONE_MASK)) {
					break;
				}
				sleep_ms(1);
			}
			amp_write(chip, L41_IRQ1_STATUS1, L41_PDN_DONE_MASK);
			amp_test_key(chip, true);
			amp_write(chip, 0x2084, 0x002f1aa3);    /* cs35l41_pdn_patch */
			amp_test_key(chip, false);
			printf("akita amp: %s: powered down (%s after %d ms)\n", chip ? "right" : "left",
			    (st & L41_PDN_DONE_MASK) ? "PDN_DONE" : "NO PDN_DONE", ms);
		}
	}
	return 0;
}

/*
 * A listening test (11-12, the speakers silent though the AoC streams and both amplifiers run): akita_amp_variant(K)
 * powers both down, sets their audio port up the K-th way, powers them up -- the AoC's driver (aocplay_matrix=1) plays
 * K beeps with each K, so the tester's "I heard N beeps" names the setting that works. 1 as set up (DSP-A, 32-bit slots
 * and words, left slot 0 / right slot 1), 2 I2S, 3 slots 2 and 3, 4 SCLK inverted, 5 LRCLK inverted, 6 16-bit slots and
 * words. Anything else: as set up, powered down.
 */
int akita_amp_variant(int k);
int
akita_amp_variant(int k)
{
	static const char *what[] = { "as set up", "as set up (DSP-A, 32-bit, left slot 0 / right slot 1)", "I2S",
		                      "slots 2 and 3", "SCLK inverted", "LRCLK inverted", "16-bit slots and words" };

	if (!amps_ready || !speaker_allowed) {
		return -1;
	}
	akita_amp_speaker(0);
	for (int chip = 0; chip < 2; chip++) {
		uint32_t fmt = 0, inv = 0, width = 32, wl = 32, slots = chip ? 0x0001 : 0x0100;

		switch (k) {
		case 2: fmt = 2; break;
		case 3: slots = chip ? 0x0203 : 0x0302; break;
		case 4: inv = L41_SCLK_INV_MASK; break;
		case 5: inv = L41_LRCLK_INV_MASK; break;
		case 6: width = 16; wl = 16; break;
		default: break;
		}
		amp_update(chip, L41_SP_FORMAT, L41_ASP_FMT_MASK | L41_SCLK_INV_MASK | L41_LRCLK_INV_MASK | L41_ASP_WIDTH_RX_MASK,
		    (fmt << L41_ASP_FMT_SHIFT) | inv | (width << L41_ASP_WIDTH_RX_SHIFT));
		amp_update(chip, L41_SP_FRAME_RX_SLOT, 0x3f3f, slots);
		amp_update(chip, L41_SP_RX_WL, L41_ASP_RX_WL_MASK, wl << L41_ASP_RX_WL_SHIFT);
	}
	printf("akita amp: VARIANT %d: %s\n", k, (k >= 1 && k <= 6) ? what[k] : what[0]);
	return (k >= 1 && k <= 6) ? akita_amp_speaker(1) : 0;
}

/* chip CHIP out of reset and asked who it is (cs35l41_probe's first half) */
static bool
amp_identify(int chip)
{
	const char *name = chip ? "right" : "left";
	uint32_t st4 = 0, st3 = 0, devid = 0, revid = 0, otpid = 0;
	int waited;

	if (chip == 1) {
		/* which chip-select register value lets the right amplifier answer (alone): each tried for its device id */
		static const uint32_t tries[] = { CS_INACTIVE | CS_AUTO | CS_NSC_CNT_2, CS_AUTO | CS_NSC_CNT_2, CS_AUTO, 0 };
		for (unsigned t = 0; t < sizeof(tries) / sizeof(tries[0]); t++) {
			cs_reg_right = tries[t];
			devid = 0xdeadbeef;
			bool ok = amp_read(chip, CS35L41_DEVID, &devid);
			printf("akita amp: right, CS_REG 0x%x while selected: %s, device id 0x%x\n", tries[t],
			    ok ? "transfer done" : "TRANSFER TIMED OUT", devid);
			if (ok && (devid == CS35L41_CHIP_ID || devid == CS35L41R_CHIP_ID)) {
				break;
			}
		}
		printf("akita amp: pins: gpn1 (right's chip select) CON 0x%x DAT 0x%x, gpp1 (left's reset) CON 0x%x DAT 0x%x, gph2 "
		    "(right's reset) CON 0x%x DAT 0x%x\n", rd(cs_r), rd(cs_r + 4), rd(rst_l), rd(rst_l + 4), rd(rst_r), rd(rst_r + 4));
	}
	for (waited = 0; waited < 100; waited++) {
		if (amp_read(chip, CS35L41_IRQ1_STATUS4, &st4) && (st4 & CS35L41_OTP_BOOT_DONE)) {
			break;
		}
		sleep_ms(1);
	}
	amp_read(chip, CS35L41_IRQ1_STATUS3, &st3);
	amp_read(chip, CS35L41_DEVID, &devid);
	amp_read(chip, CS35L41_REVID, &revid);
	amp_read(chip, CS35L41_OTPID, &otpid);
	printf("akita amp: %s: OTP boot %s after %d ms (IRQ1_STATUS4 0x%x, STATUS3 0x%x%s), device id 0x%x (%s), revision 0x%x "
	    "(metal %u), OTP id 0x%x\n", name, (st4 & CS35L41_OTP_BOOT_DONE) ? "done" : "NOT DONE", waited, st4, st3,
	    (st3 & CS35L41_OTP_BOOT_ERR) ? ": OTP BOOT ERROR" : "", devid,
	    devid == CS35L41_CHIP_ID ? "a CS35L41" : devid == CS35L41R_CHIP_ID ? "a CS35L41R" : "not a CS35L41",
	    revid, revid & 0xf, otpid & 0xf);
	return devid == CS35L41_CHIP_ID || devid == CS35L41R_CHIP_ID;
}

static void
akita_amp_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	spi_clock_on();
	if (!spi_init()) {
		printf("akita amp: the SPI controller does not answer; stopping\n");
		return;
	}
	pin_out(cs_r, CS_R_PIN, 1);                     /* the right amplifier's chip select inactive */
	pin_out(rst_l, RST_L_PIN, 0);                   /* both in reset (cs35l41_probe: GPIOD_OUT_LOW, 2 ms) */
	pin_out(rst_r, RST_R_PIN, 0);
	sleep_ms(3);
	pin_out(rst_l, RST_L_PIN, 1);
	pin_out(rst_r, RST_R_PIN, 1);
	sleep_ms(3);
	bool l = amp_identify(0), r = amp_identify(1);
	printf("akita amp: %s\n", l && r ? "BOTH SPEAKER AMPLIFIERS ANSWER" : "an amplifier did not answer");
	if (mode >= 2 && l && r && amp_setup(0) && amp_setup(1)) {
		amps_ready = true;
		printf("akita amp: both set up for the AoC's TDM_0, powered down until akita_amp_speaker(1)\n");
		if (mode >= 3) {
			bool pl = amp_load_protection(0), pr = amp_load_protection(1);
			printf("akita amp: %s\n", pl && pr ? "THE SPEAKER PROTECTION IS LOADED IN BOTH (not running yet)" :
			    "the speaker protection did not load");
		}
	}
}

__startup_func
static void
akita_amp_init(void)
{
	static thread_call_t start;
	uint64_t deadline;

	if (!PE_parse_boot_argn("amp", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	PE_parse_boot_argn("speaker", &speaker_allowed, sizeof(speaker_allowed));
	spi = ml_io_map(SPI_PHYS, 0x1000);
	cmu = ml_io_map(CMU_PHYS, 0x2000);
	sysreg = ml_io_map(SYSREG_PHYS, 0x1000);
	rst_l = ml_io_map(RST_L_PHYS, 0x1000);
	rst_r = ml_io_map(RST_R_PHYS, 0x1000);
	cs_r = ml_io_map(CS_R_PHYS, 0x1000);
	if (!spi || !cmu || !sysreg || !rst_l || !rst_r || !cs_r) {
		printf("akita amp: could not map its registers\n");
		return;
	}
	rst_l += RST_L_BANK;
	rst_r += RST_R_BANK;
	cs_r += CS_R_BANK;
	start = thread_call_allocate(akita_amp_start, NULL);
	clock_interval_to_deadline(20, NSEC_PER_SEC, &deadline);       /* after the USB log is up */
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_amp_init);
