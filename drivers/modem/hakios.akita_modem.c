/*
 * The Pixel 8a's cellular modem: a Samsung Exynos 5300 ("s5300") beside the Tensor G3, reached over zuma's PCIe
 * channel 0 (hakios.akita_pcie0.c) and a few megabytes of this phone's RAM that both sides read and write. Android
 * starts it with a kernel driver (cpif) and a daemon (cbd); here the kernel does both, polled, a step at a time.
 *
 * Nothing here is touched unless the boot-arg asks: modem=N performs steps 1 to N and stops, each saying what it is
 * about to touch before it does and what it found after; a step that does not find what it needs ends the sequence.
 *
 *   1  reads   the pins between the two (gph0, gph1 of the HSI1 pin controller; gpa0, gpa6, gpa7 of the alive one),
 *              the shared memory's control words and the page the modem's ROM reports its boot stage in, as the
 *              bootloader left them; the firmware's table of contents from the partition modem_a (ext4:
 *              hakios.akita_modem_fs.c)
 *   2  writes  the shared memory's control words for a boot (the queues emptied, the boot magic), this phone's
 *              "handover block", the first-stage loader (BOOT of modem.bin) at the place the ROM fetches it from;
 *              the pins as Android's driver sets them and the modem powered off and on with them
 *   3  writes  PCIe channel 0 brought up at Gen1 (the ROM's speed), the modem found on it (144d:a5a5), its BAR0 and
 *              MSI capability set; the loader's address and size told, the doorbell rung; the boot stage waited for
 *              (0x3fff: the ROM took the loader and jumped to it)
 *   4  writes  the link taken down, the modem's wake pin waited for, the link up again at Gen3, the "link ack"
 *              doorbell
 *   5  writes  the firmware sent to the loader through the raw queue, stage by stage as cbd does: the table of
 *              contents, MAIN, VSS, APM from modem.bin; NV_NORM, NV_PROT, REPLAY from fw/ of the small image (this
 *              phone's calibration and identity: pulled from its Android into fw-modem-akita/, never in git)
 *   6  writes  the start handshake: INIT_START from the modem, PIF_INIT_DONE, PHONE_START, INIT_END: the modem is
 *              online
 *   7  reads   what the modem then says by itself, for three seconds: every frame's channel, length and first bytes
 *   8  writes  requests in the modem's command protocol (SIT, on channel 245 as Android's rild): radio power, the
 *              baseband's version, the SIM's state, the registration states, the operator, the signal; the answers
 *              and whatever else comes are said for modem_listen seconds (20). The device's identity is not asked for
 *
 * Where everything comes from: Google's driver for this phone (google-modules/soc/gs, drivers/soc/google/cpif:
 * modem_ctrl_s5100.c, link_device.c, link_device_memory*.c, s51xx_pcie.c; the layout of the shared memory, the pins'
 * order and delays, the frame header), the phone's device tree (node cpif: offsets, channel numbers, the reserved
 * memory), and a trace of Android's cbd starting the modem (11-19: its ioctls and every write to /dev/umts_boot0) --
 * cbd is closed, the download's commands below are what it was seen to send. That code is GPL and none of it is
 * here: the numbers are the hardware's and the firmware's facts.
 *
 * The download (step 5), each command a frame on channel 241 answered by a 4-byte frame (byte 1's high nibble 0xc):
 *   10 a1 00 00                                         start
 *   1b a1 LEN TOTAL OFFSET + the table of contents      (LEN = bytes that follow LEN, 16 bit; 0x410 bytes)
 *   1d a1 00 00                                         end of the table
 *   for stage s = 2..7 (MAIN VSS APM NV_NORM NV_PROT REPLAY, the table's order):
 *     s0 a1 00 00;  sb a1 LEN TOTAL OFFSET + up to 0xc000 bytes, until all are sent;
 *     (MAIN only) 21 a3 00 00 + the table's checksum word, answered 20 c3;  sd a1 00 00
 *   00 a4 00 00                                         run it
 *
 *   9  stays   the modem's file service answered (channel 41, Android's rfsd: the modem saves its NV data to the
 *              phone a few seconds after it starts) and everything it says said, until the phone restarts. The NV
 *              data it sends is kept in memory and counted, not written anywhere yet
 *
 * The file service, as rfsd was seen to speak it: messages { u16 command, u16 sequence, u32 length } + values, the
 * answer under the request's sequence number.
 *   the modem: 7 { id }                         open an NV file (3: protected, 1: normal)    we: 3 { 0, id }
 *   the modem: 6 { id, offset, size, 2 }        "take size bytes from me"                    we: 2 { id, offset, n } ...
 *   the modem: 2 { 0, id, n } + n bytes         ... each answered so, n <= 0x7dc, then       we: 3 { 0, id }
 *   the modem: 5 { id }                         close                                        we: 3 { 0, id }
 *   the modem: 3 { 0, id, 0 }                   its acknowledgement of our 3
 *   the modem: 4 { id, n } + a path             open a file of Android's by name (6 with direction 1 then reads it):
 *                                               not answered here -- the files are Android's vendor partition's
 *
 * More boot-args: modem_delay=SECONDS (25) after the kernel's start before step 1; modem_listen=SECONDS (20);
 * modem_link_sleep=0: step 9 keeps the link up (by default it follows the modem's wake pin: down when the modem lets
 * it go, up when either side has something); modem_ask_every=SECONDS: step 9 asks the radio's power that often (a test
 * of the wake from our side); modem_radio=1: step 8 also turns the radio on (SET_RADIO_POWER), as Android does with a SIM.
; modem_irq=0: step 9 by the clock
 * alone (200 looks a second) instead of woken by the modem's interrupts; modem_crash_after=SECONDS: a test of
 * step 9's restart -- the modem is asked to stop that long into step 9 (the dump-request pin).
 *
 * For Darwin: sysctl kern.hakios.cellular_radio, _sim, _baseband, _registered, _rat, _operator, _dbm, _bars, _up, kept
 * from the modem's answers and indications while step 8 or 9 runs.
 */
#include <kern/clock.h>
#include <kern/kalloc.h>
#include <kern/sched_prim.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_map_address_t pmap_map(vm_map_address_t virt, vm_offset_t start, vm_offset_t end, vm_prot_t prot, unsigned int flags);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
extern int hakios_ufs_read_partition(const char *name, uint64_t offset, uint8_t *dst, uint64_t len, uint64_t *bytes);
extern kern_return_t thread_terminate(thread_t thread);

/* osfmk/arm64/gic_v3_irq.c */
extern kern_return_t gic_v3_spi_attach(uint32_t intid, bool edge, void (*handler)(void *context, uint32_t intid), void *context);
extern void gic_v3_spi_mask(uint32_t intid);
extern void gic_v3_spi_unmask(uint32_t intid);

/* hakios.akita_modem_fs.c */
struct akita_modem_file;
typedef int (*akita_modem_fs_read_t)(void *context, uint64_t offset, uint8_t *dst, uint64_t len);
extern int akita_modem_fs_open(akita_modem_fs_read_t read, void *context, const char *path, struct akita_modem_file *file);
extern int akita_modem_file_read(struct akita_modem_file *file, uint64_t offset, uint8_t *dst, uint64_t len);
extern uint64_t akita_modem_file_size(const struct akita_modem_file *file);
extern size_t akita_modem_file_bytes(void);

/* hakios.akita_pcie0.c */
extern bool akita_pcie0_up(unsigned gen);               /* the PHY, the root complex, PERST#, training; the windows */
extern void akita_pcie0_down(void);
extern uint32_t akita_pcie0_ep_read(uint32_t reg);       /* the endpoint's configuration space */
extern void akita_pcie0_ep_write(uint32_t reg, uint32_t value);
extern volatile uint8_t *akita_pcie0_window(uint32_t bus_offset, uint32_t size);  /* bus 0x14e00000 + offset, mapped */
extern uint32_t akita_pcie0_msi_take(void);              /* the root complex's MSI status (group 0), read and cleared */

/* ---- where things are ---- */

/* RAM both sides use (the device tree's cp_rmem, cp_rmem_1, cp_msi_rmem: no-map in U-Boot's tree, never Darwin's) */
#define IPC_PHYS                0xea400000ULL   /* control words and the queues, 8 MB */
#define IPC_SIZE                0x00800000u
#define MSI_PHYS                0xf6200000ULL   /* a page: the address the modem's MSIs are written to; the ROM's words */
#define PKTPROC_PHYS            0xe8000000ULL   /* the packet processor's 32 MB (the modem's 0x20000000) */
#define PKTPROC_DL_INFO         0x00000000u
#define PKTPROC_UL_INFO         0x01c00000u
#define PKTPROC_MAPPED          0x01c01000u     /* as far as the two info regions */

/* the ROM's words in the MSI page */
#define MSI_ERR_REPORT          0x08
#define MSI_BOOT_STAGE          0x10            /* bits the ROM sets as it goes; 0xff: waits for the doorbell */
#define MSI_IMG_ADDR_LO         0x14
#define MSI_IMG_ADDR_HI         0x18
#define MSI_IMG_SIZE            0x1c
#define BOOT_STAGE_DONE         0x3fffu

/* the shared memory */
#define IPC_MAGIC               0x000
#define IPC_ACCESS              0x004
#define IPC_FMT_TX_HEAD         0x008
#define IPC_FMT_TX_TAIL         0x00c
#define IPC_FMT_RX_HEAD         0x010
#define IPC_FMT_RX_TAIL         0x014
#define IPC_RAW_TX_HEAD         0x018
#define IPC_RAW_TX_TAIL         0x01c
#define IPC_RAW_RX_HEAD         0x020
#define IPC_RAW_RX_TAIL         0x024
#define IPC_SRINFO_OFFSET       0x064
#define IPC_CAP_OFFSET          0x070
#define IPC_BOOT_INDICATOR      0x090
#define IPC_AP_CAP0             0x0a0
#define IPC_CP_CAP0             0x0a4
#define IPC_AP_CAP1             0x0a8
#define IPC_CP_CAP1             0x0ac
#define IPC_AP2CP_MSG           0x800
#define IPC_CP2AP_MSG           0x804
#define IPC_AP2CP_STATUS        0x808
#define IPC_CP2AP_STATUS        0x80c
#define IPC_HANDOVER            0x82c
#define IPC_HANDOVER_SIZE       161u
#define IPC_BOOT_LOG            0xc00           /* a length word, then text the modem's loader writes */
#define IPC_FMT_TX_BUF          0x1000
#define IPC_FMT_TX_SIZE         0x1000u
#define IPC_FMT_RX_BUF          0x2000
#define IPC_FMT_RX_SIZE         0x1000u
#define IPC_RAW_TX_BUF          0x3000
#define IPC_RAW_TX_SIZE         0x1fd000u
#define IPC_RAW_RX_BUF          0x200000
#define IPC_RAW_RX_SIZE         0x200000u
#define IPC_BOOT_IMG            0x10000         /* the first-stage loader, for the ROM (inside the raw TX buffer) */

#define MAGIC_IPC               0x000000aau
#define MAGIC_BOOT              0x0000bdbdu
#define STATUS_DUAL_SIM         0x00004000u     /* ap2cp status bits 15:14 = 1 */
#define AP_CAPABILITIES         0x00000003u     /* the uplink packet processor, extended channels: what Android says */

/* the interrupt words (ap2cp_msg, cp2ap_msg) */
#define MSG_INIT_START          0xc1u
#define MSG_INIT_END            0xc2u
#define MSG_PHONE_START         0xc8u
#define MSG_PIF_INIT_DONE       0xcdu
#define MSG_SEND_RAW            0x81u
#define MSG_SEND_FMT            0x82u
#define DOORBELL_OFFSET         0x60000u        /* in the modem's BAR0 window */
#define DOORBELL_MSG            0x00010000u
#define DOORBELL_LINK_ACK       0x0001000eu

/* a frame: 12 bytes, then the payload, then zeros to a multiple of 8 */
#define FRAME_SYNC              0xabcdu
#define FRAME_SINGLE            0xc000u
#define FRAME_HEADER            12u
#define CH_BOOT                 241u
#define CH_RFS                  41u             /* umts_rfs0: the modem's file service */
#define CH_RIL                  245u            /* umts_ipc0: the command protocol, SIM 1's stack */

/*
 * The command protocol ("SIT"; the numbers are the names' in Android's vendor.radio.protocol.sit.base, the layout as
 * rild was seen to use it): a request is { u16 0, u16 id, u16 length, u16 token, u32 0 } + parameters; its answer
 * { u16 1, id, length, token, u16 0, u16 error } + values; an indication { u16 2, id, length, u16 0 } + values.
 */
#define SIT_GET_SIM_STATUS              0x0200u
#define SIT_VERIFY_SIM_PIN              0x0201u
#define SIT_IND_SIM_STATUS_CHANGED      0x0210u
#define SIT_GET_CS_REG_STATE            0x0700u
#define SIT_GET_PS_REG_STATE            0x0701u
#define SIT_GET_OPERATOR                0x0702u
#define SIT_SET_RADIO_POWER             0x0800u
#define SIT_GET_RADIO_POWER             0x0801u
#define SIT_IND_RADIO_STATE_CHANGED     0x0802u
#define SIT_IND_RADIO_READY             0x0803u
#define SIT_GET_SIGNAL_STRENGTH         0x0900u
#define SIT_SET_DEVICE_INFO             0x0922u
#define SIT_SET_AP_SYSTEM_TIME          0x0949u
#define SIT_IND_NTW_STATE_CHANGED       0x070eu
#define SIT_GET_BASEBAND_VERSION        0x0901u
#define SIT_IND_SIGNAL_STRENGTH         0x0906u

/* the endpoint */
#define EP_VENDOR_SAMSUNG       0x144du
#define EP_BAR0_BUS             0x14e00000u
#define EP_MSI_CAP              0x50

/* the pins */
#define PIN_HSI1                0x12040000ULL
#define PIN_ALIVE               0x154d0000ULL
#define BANK_GPH0               0x00
#define BANK_GPH1               0x20
#define BANK_GPA0               0x00
#define BANK_GPA6               0xa0
#define BANK_GPA7               0xc0
#define PIN_CON                 0x00
#define PIN_DAT                 0x04
#define PIN_PUD                 0x08
/* the alive controller's pins as interrupts (one line of the GIC a pin): how each triggers, masked or not, what waits */
#define EINT_CON_GPA6           0x714           /* 4 bits a pin: 4 = both edges */
#define EINT_CON_GPA7           0x718
#define EINT_MASK_GPA6          0x914           /* a bit a pin: 1 = masked */
#define EINT_MASK_GPA7          0x918
#define EINT_PEND_GPA6          0xa14           /* a bit a pin: written 1 to clear */
#define EINT_PEND_GPA7          0xa18
#define INTID_CP2AP_WAKEUP      67u             /* gpa6-5: SPI 35 */
#define INTID_CP2AP_ACTIVE      68u             /* gpa7-0: SPI 36 */
#define INTID_MSI0              448u            /* the root complex's MSI group 0: SPI 416 */

struct pin { uint8_t alive, bank, bit; const char *name; };
static const struct pin AP2CP_WAKEUP = { 0, BANK_GPH0, 2, "AP2CP_WAKEUP" }, AP2CP_DUMP_NOTI = { 0, BANK_GPH0, 3, "AP2CP_DUMP_NOTI" },
    AP2CP_AP_ACTIVE = { 0, BANK_GPH1, 0, "AP2CP_AP_ACTIVE" }, AP2CP_CP_WRST_N = { 0, BANK_GPH1, 1, "AP2CP_CP_WRST_N" },
    AP2CP_CP_PWR = { 0, BANK_GPH1, 2, "AP2CP_CP_PWR" }, AP2CP_NRESET = { 0, BANK_GPH1, 7, "AP2CP_NRESET" },
    CP2AP_CP_WRST_N = { 1, BANK_GPA0, 1, "CP2AP_CP_WRST_N" }, AP2CP_PM_WRST_N = { 1, BANK_GPA0, 2, "AP2CP_PM_WRST_N" },
    CP2AP_PS_HOLD = { 1, BANK_GPA6, 4, "CP2AP_PS_HOLD" }, CP2AP_WAKEUP = { 1, BANK_GPA6, 5, "CP2AP_WAKEUP" },
    CP2AP_CP_ACTIVE = { 1, BANK_GPA7, 0, "CP2AP_CP_ACTIVE" };

/* modem.bin's table of contents: 32-byte entries */
struct toc_entry { char name[12]; uint32_t file_offset, load_address, size, crc, index; };
#define TOC_BYTES               0x410u
#define TOC_ENTRIES             8u
#define DOWNLOAD_CHUNK          0xc000u

static unsigned int steps, step, listen_s = 20, link_sleep = 1, ask_every_s, radio_on, restarts, crash_after_s, use_irq = 1;
static volatile uint8_t *ipc, *msi, *pktproc, *pin_hsi1, *pin_alive, *doorbell;
static struct akita_modem_file *firmware;
static struct toc_entry toc[TOC_ENTRIES];
static uint8_t toc_raw[TOC_BYTES];
static uint16_t frame_count;
static uint8_t channel_count[256];
static uint32_t ep_saved_msi[4], ep_pm_cap;

/* ---- small things ---- */

static inline uint32_t
rd(volatile uint8_t *base, uint32_t off)
{
	return *(volatile uint32_t *)(base + off);
}

static inline void
wr(volatile uint8_t *base, uint32_t off, uint32_t value)
{
	*(volatile uint32_t *)(base + off) = value;
	__asm__ volatile ("dsb sy" ::: "memory");
}

static void
sleep_ms(uint32_t ms)
{
	assert_wait_timeout((event_t)&steps, THREAD_UNINT, ms, NSEC_PER_MSEC);
	thread_block(THREAD_CONTINUE_NULL);
}

static uint64_t
now_ms(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / NSEC_PER_MSEC;
}

static bool
announce(unsigned int n, const char *what)
{
	if (n > steps) {
		return false;
	}
	step = n;
	printf("modem: %u: %s\n", n, what);
	return true;
}

/* RAM that is not Darwin's, mapped with kernel addresses of its own (not ml_io_map: its whole area is 8 MB) */
static volatile uint8_t *
map_ram(uint64_t phys, uint32_t size)
{
	vm_offset_t v = 0;

	size = (uint32_t)round_page(size);      /* (16 KB pages: the MSI page's mapping reaches 12 KB past it, never touched) */
	if (kmem_alloc(kernel_map, &v, size, KMA_VAONLY | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		return NULL;
	}
	pmap_map(v, (vm_offset_t)phys, (vm_offset_t)(phys + size), VM_PROT_READ | VM_PROT_WRITE, VM_WIMG_WCOMB);
	return (volatile uint8_t *)v;
}

/* bytes into and out of the shared memory: plain stores and loads, no memcpy (uncached memory the modem reads) */
static void
shared_put(volatile uint8_t *dst, const uint8_t *src, uint32_t len)
{
	while (len != 0 && ((uintptr_t)dst & 7) != 0) {
		*dst++ = *src++;
		len--;
	}
	for (; len >= 8; len -= 8, dst += 8, src += 8) {
		uint64_t v;

		memcpy(&v, src, 8);
		*(volatile uint64_t *)dst = v;
	}
	while (len != 0) {
		*dst++ = *src++;
		len--;
	}
	__asm__ volatile ("dsb sy" ::: "memory");
}

static void
shared_get(uint8_t *dst, volatile const uint8_t *src, uint32_t len)
{
	while (len != 0) {
		*dst++ = *src++;
		len--;
	}
}

static void
shared_zero(volatile uint8_t *dst, uint32_t len)
{
	while (len != 0) {
		*dst++ = 0;
		len--;
	}
	__asm__ volatile ("dsb sy" ::: "memory");
}

/* ---- pins ---- */

static volatile uint8_t *
pin_bank(const struct pin *p)
{
	return (p->alive ? pin_alive : pin_hsi1) + p->bank;
}

static int
pin_get(const struct pin *p)
{
	return (rd(pin_bank(p), PIN_DAT) >> p->bit) & 1;
}

static void
pin_input(const struct pin *p, int pud)
{
	volatile uint8_t *b = pin_bank(p);

	wr(b, PIN_PUD, (rd(b, PIN_PUD) & ~(0xfu << (4 * p->bit))) | ((uint32_t)pud << (4 * p->bit)));
	wr(b, PIN_CON, rd(b, PIN_CON) & ~(0xfu << (4 * p->bit)));
}

/* an output at that level; the wait only if the level changed (as Android's driver: mif_gpio_set_value) */
static void
pin_out(const struct pin *p, int level, uint32_t wait_ms)
{
	volatile uint8_t *b = pin_bank(p);
	bool was_output = ((rd(b, PIN_CON) >> (4 * p->bit)) & 0xf) == 1;
	int before = pin_get(p);

	wr(b, PIN_DAT, (rd(b, PIN_DAT) & ~(1u << p->bit)) | ((uint32_t)level << p->bit));
	if (!was_output) {
		wr(b, PIN_PUD, rd(b, PIN_PUD) & ~(0xfu << (4 * p->bit)));
		wr(b, PIN_CON, (rd(b, PIN_CON) & ~(0xfu << (4 * p->bit))) | (1u << (4 * p->bit)));
	}
	if ((before != level || !was_output) && wait_ms != 0) {
		sleep_ms(wait_ms);
	}
}

static void
pins_say(const char *when)
{
	printf("modem: %u: %s: gph0 CON 0x%08x DAT 0x%02x; gph1 CON 0x%08x DAT 0x%02x; gpa0 CON 0x%08x DAT 0x%02x; gpa6 CON 0x%08x "
	    "DAT 0x%02x PUD 0x%08x; gpa7 CON 0x%08x DAT 0x%02x\n", step, when, rd(pin_hsi1, BANK_GPH0 + PIN_CON),
	    rd(pin_hsi1, BANK_GPH0 + PIN_DAT), rd(pin_hsi1, BANK_GPH1 + PIN_CON), rd(pin_hsi1, BANK_GPH1 + PIN_DAT),
	    rd(pin_alive, BANK_GPA0 + PIN_CON), rd(pin_alive, BANK_GPA0 + PIN_DAT), rd(pin_alive, BANK_GPA6 + PIN_CON),
	    rd(pin_alive, BANK_GPA6 + PIN_DAT), rd(pin_alive, BANK_GPA6 + PIN_PUD), rd(pin_alive, BANK_GPA7 + PIN_CON),
	    rd(pin_alive, BANK_GPA7 + PIN_DAT));
	printf("modem: %u: %s: to the modem: wake %d, dump %d, AP active %d, CP_WRST_N %d, power %d, NRESET %d, PM_WRST_N %d; from it: "
	    "CP_WRST_N %d, PS_HOLD %d, wake %d, active %d\n", step, when, pin_get(&AP2CP_WAKEUP), pin_get(&AP2CP_DUMP_NOTI),
	    pin_get(&AP2CP_AP_ACTIVE), pin_get(&AP2CP_CP_WRST_N), pin_get(&AP2CP_CP_PWR), pin_get(&AP2CP_NRESET), pin_get(&AP2CP_PM_WRST_N),
	    pin_get(&CP2AP_CP_WRST_N), pin_get(&CP2AP_PS_HOLD), pin_get(&CP2AP_WAKEUP), pin_get(&CP2AP_CP_ACTIVE));
}

static void
shared_say(const char *when)
{
	printf("modem: %u: %s: magic 0x%x access %u; FMT tx %u/%u rx %u/%u; RAW tx %u/%u rx %u/%u; caps AP 0x%x CP 0x%x; msg AP 0x%x "
	    "CP 0x%x; status AP 0x%x CP 0x%x; boot indicator 0x%x\n", step, when, rd(ipc, IPC_MAGIC), rd(ipc, IPC_ACCESS),
	    rd(ipc, IPC_FMT_TX_HEAD), rd(ipc, IPC_FMT_TX_TAIL), rd(ipc, IPC_FMT_RX_HEAD), rd(ipc, IPC_FMT_RX_TAIL),
	    rd(ipc, IPC_RAW_TX_HEAD), rd(ipc, IPC_RAW_TX_TAIL), rd(ipc, IPC_RAW_RX_HEAD), rd(ipc, IPC_RAW_RX_TAIL),
	    rd(ipc, IPC_AP_CAP0), rd(ipc, IPC_CP_CAP0), rd(ipc, IPC_AP2CP_MSG), rd(ipc, IPC_CP2AP_MSG), rd(ipc, IPC_AP2CP_STATUS),
	    rd(ipc, IPC_CP2AP_STATUS), rd(ipc, IPC_BOOT_INDICATOR));
	printf("modem: %u: %s: the ROM's page: boot stage 0x%x, error 0x%x, image 0x%x%08x + 0x%x; word 0 0x%x, word 1 0x%x\n", step, when,
	    rd(msi, MSI_BOOT_STAGE), rd(msi, MSI_ERR_REPORT), rd(msi, MSI_IMG_ADDR_HI), rd(msi, MSI_IMG_ADDR_LO), rd(msi, MSI_IMG_SIZE),
	    rd(msi, 0), rd(msi, 4));
}

/* the text the modem's loader leaves in the shared memory (a length, then characters), a line at a time */
static void
boot_log_say(void)
{
	uint32_t length = rd(ipc, IPC_BOOT_LOG);
	char line[160];
	unsigned n = 0;

	if (length == 0 || length > 0x1fb) {
		printf("modem: %u: the modem's boot log: none (length 0x%x)\n", step, length);
		return;
	}
	for (uint32_t i = 0; i < length; i++) {
		char c = (char)ipc[IPC_BOOT_LOG + 4 + i];

		if (c == '\n' || c == '\r' || n == sizeof(line) - 1) {
			line[n] = 0;
			if (n != 0) {
				printf("modem: %u: boot log: %s\n", step, line);
			}
			n = 0;
		}
		if (c >= 0x20 && c < 0x7f) {
			line[n++] = c;
		}
	}
	line[n] = 0;
	if (n != 0) {
		printf("modem: %u: boot log: %s\n", step, line);
	}
}

/* ---- the firmware ---- */

static int
partition_read(__unused void *context, uint64_t offset, uint8_t *dst, uint64_t len)
{
	return hakios_ufs_read_partition("modem", offset, dst, len, NULL);
}

static bool
firmware_open(void)
{
	if (firmware != NULL) {
		return true;
	}
	firmware = kalloc_data(akita_modem_file_bytes(), Z_WAITOK | Z_ZERO);
	if (firmware == NULL || akita_modem_fs_open(partition_read, NULL, "images/default/modem.bin", firmware) != 0) {
		printf("modem: %u: images/default/modem.bin was not found in the partition modem (ext4): stopped\n", step);
		firmware = NULL;
		return false;
	}
	if (akita_modem_file_read(firmware, 0, toc_raw, TOC_BYTES) != 0 || memcmp(toc_raw, "TOC", 4) != 0) {
		printf("modem: %u: modem.bin (%llu bytes) does not begin with a table of contents: stopped\n", step,
		    akita_modem_file_size(firmware));
		firmware = NULL;
		return false;
	}
	memcpy(toc, toc_raw, sizeof(toc));
	printf("modem: %u: modem.bin: %llu bytes\n", step, akita_modem_file_size(firmware));
	for (unsigned i = 0; i < TOC_ENTRIES; i++) {
		toc[i].name[11] = 0;
		printf("modem: %u:   %-8s file 0x%08x  to 0x%08x  0x%08x bytes  crc 0x%08x\n", step, toc[i].name, toc[i].file_offset,
		    toc[i].load_address, toc[i].size, toc[i].crc);
	}
	if (strcmp(toc[1].name, "BOOT") != 0 || strcmp(toc[2].name, "MAIN") != 0 || toc[1].size > 0x100000 ||
	    (uint64_t)toc[2].file_offset + toc[2].size > akita_modem_file_size(firmware)) {
		printf("modem: %u: not the table expected (BOOT, MAIN, ...): stopped\n", step);
		firmware = NULL;
		return false;
	}
	return true;
}

/* ---- the queues ---- */

struct queue { uint32_t head, tail, buffer, size; const char *name; };
static const struct queue RAW_TX = { IPC_RAW_TX_HEAD, IPC_RAW_TX_TAIL, IPC_RAW_TX_BUF, IPC_RAW_TX_SIZE, "RAW tx" },
    RAW_RX = { IPC_RAW_RX_HEAD, IPC_RAW_RX_TAIL, IPC_RAW_RX_BUF, IPC_RAW_RX_SIZE, "RAW rx" },
    FMT_TX = { IPC_FMT_TX_HEAD, IPC_FMT_TX_TAIL, IPC_FMT_TX_BUF, IPC_FMT_TX_SIZE, "FMT tx" },
    FMT_RX = { IPC_FMT_RX_HEAD, IPC_FMT_RX_TAIL, IPC_FMT_RX_BUF, IPC_FMT_RX_SIZE, "FMT rx" };

/* Bytes into a queue we fill (we move the head, the modem the tail); false if they do not fit within wait_ms. */
static bool
queue_put(const struct queue *q, const uint8_t *data, uint32_t len, uint32_t wait_ms)
{
	uint64_t until = now_ms() + wait_ms;

	for (;;) {
		uint32_t head = rd(ipc, q->head), tail = rd(ipc, q->tail);
		uint32_t room = head < tail ? tail - head - 1 : q->size + tail - head - 1;

		if (head >= q->size || tail >= q->size) {
			printf("modem: %u: %s: head %u, tail %u past its %u bytes\n", step, q->name, head, tail, q->size);
			return false;
		}
		if (room >= len) {
			uint32_t first = q->size - head < len ? q->size - head : len;

			shared_put(ipc + q->buffer + head, data, first);
			shared_put(ipc + q->buffer, data + first, len - first);
			wr(ipc, q->head, (head + len) % q->size);
			return true;
		}
		if (now_ms() > until) {
			printf("modem: %u: %s: no room for %u bytes in %u ms (head %u, tail %u)\n", step, q->name, len, wait_ms, head, tail);
			return false;
		}
		delay(100);
	}
}

static uint32_t
queue_used(const struct queue *q)
{
	uint32_t head = rd(ipc, q->head), tail = rd(ipc, q->tail);

	if (head >= q->size || tail >= q->size) {
		return 0;
	}
	return head >= tail ? head - tail : q->size - tail + head;
}

static void
queue_peek(const struct queue *q, uint32_t at, uint8_t *dst, uint32_t len)
{
	uint32_t tail = (rd(ipc, q->tail) + at) % q->size, first = q->size - tail < len ? q->size - tail : len;

	shared_get(dst, ipc + q->buffer + tail, first);
	shared_get(dst + first, ipc + q->buffer, len - first);
}

/*
 * One frame out of a queue the modem fills: its channel, its payload (up to room bytes kept) and the payload's whole
 * length. 0: nothing whole is there; -1: what is there is not a frame (the queue is then emptied).
 */
static int
frame_take(const struct queue *q, uint8_t *channel, uint8_t *payload, uint32_t room, uint32_t *length, uint16_t *config)
{
	uint8_t header[FRAME_HEADER];
	uint32_t used = queue_used(q), total, len;

	if (used < FRAME_HEADER) {
		return 0;
	}
	queue_peek(q, 0, header, sizeof(header));
	len = header[6] | header[7] << 8;
	total = (len + 7) & ~7u;
	if ((header[0] | header[1] << 8) != FRAME_SYNC || len < FRAME_HEADER) {
		printf("modem: %u: %s: not a frame at its tail (%02x %02x %02x %02x %02x %02x %02x %02x, %u bytes there): emptied\n", step,
		    q->name, header[0], header[1], header[2], header[3], header[4], header[5], header[6], header[7], used);
		wr(ipc, q->tail, rd(ipc, q->head));
		return -1;
	}
	if (used < total) {
		return 0;
	}
	*channel = header[8];
	*length = len - FRAME_HEADER;
	if (config != NULL) {
		*config = (uint16_t)(header[4] | header[5] << 8);
	}
	queue_peek(q, FRAME_HEADER, payload, *length < room ? *length : room);
	wr(ipc, q->tail, (rd(ipc, q->tail) + total) % q->size);
	return 1;
}

/* A frame into a queue: the header, the payload (two pieces: a command's head and its data), the padding. */
static bool
frame_put(const struct queue *q, uint8_t channel, const uint8_t *head, uint32_t head_len, const uint8_t *data, uint32_t data_len)
{
	static uint8_t *frame;
	uint32_t len = FRAME_HEADER + head_len + data_len, total = (len + 7) & ~7u;

	if (frame == NULL && (frame = kalloc_data(FRAME_HEADER + 16 + DOWNLOAD_CHUNK + 8, Z_WAITOK | Z_ZERO)) == NULL) {
		return false;
	}
	if (len > 0xffff || head_len > 16 || data_len > DOWNLOAD_CHUNK) {
		return false;
	}
	frame_count++;
	channel_count[channel]++;
	frame[0] = FRAME_SYNC & 0xff;
	frame[1] = FRAME_SYNC >> 8;
	frame[2] = (uint8_t)frame_count;
	frame[3] = (uint8_t)(frame_count >> 8);
	frame[4] = FRAME_SINGLE & 0xff;
	frame[5] = FRAME_SINGLE >> 8;
	frame[6] = (uint8_t)len;
	frame[7] = (uint8_t)(len >> 8);
	frame[8] = channel;
	frame[9] = channel_count[channel];
	frame[10] = frame[11] = 0;
	memcpy(frame + FRAME_HEADER, head, head_len);
	if (data_len != 0) {
		memcpy(frame + FRAME_HEADER + head_len, data, data_len);
	}
	memset(frame + len, 0, total - len);
	return queue_put(q, frame, total, 3000);
}

static bool link_up, doorbell_owed;
static unsigned link_sleeps, link_wakes;

static void
ring(uint32_t value)
{
	wr(doorbell, 0, value);
	(void)rd(doorbell, 0);
}

/*
 * A word for the modem and the doorbell that makes it look. With the link asleep (step 9) the doorbell is owed: our
 * wake pin goes up, the modem answers with its own, the link comes up and the doorbell is rung then (link_follow).
 */
static void
message(uint32_t word)
{
	wr(ipc, IPC_AP2CP_MSG, word);
	if (link_up) {
		ring(DOORBELL_MSG);
	} else {
		doorbell_owed = true;
		pin_out(&AP2CP_WAKEUP, 1, 0);
	}
}

/* ---- the download ---- */

/*
 * One command of the download and its answer: 4 bytes on the boot channel, byte 1's low nibble the same and its high
 * nibble 0xc. The loader is not told with a doorbell (Android's driver sends none here): it watches the queue.
 */
static bool
download_command(const uint8_t *head, uint32_t head_len, const uint8_t *data, uint32_t data_len, uint32_t wait_ms)
{
	uint64_t until = now_ms() + wait_ms;
	uint8_t channel = 0, answer[16];
	uint32_t length = 0;

	if (!frame_put(&RAW_TX, CH_BOOT, head, head_len, data, data_len)) {
		printf("modem: %u: command %02x %02x: not sent\n", step, head[0], head[1]);
		return false;
	}
	for (unsigned spins = 0;; spins++) {
		int r = frame_take(&RAW_RX, &channel, answer, sizeof(answer), &length, NULL);

		(void)akita_pcie0_msi_take();
		if (r < 0) {
			return false;
		}
		if (r > 0) {
			if (channel == CH_BOOT && length >= 2 && (answer[1] & 0xf0) == 0xc0 && (answer[1] & 0x0f) == (head[1] & 0x0f) &&
			    (answer[0] & 0xf0) == (head[0] & 0xf0)) {
				return true;
			}
			printf("modem: %u: command %02x %02x %02x %02x: answered on channel %u with %u bytes: %02x %02x %02x %02x\n", step, head[0],
			    head[1], head[2], head[3], channel, length, answer[0], answer[1], answer[2], answer[3]);
			return false;
		}
		if (now_ms() > until) {
			printf("modem: %u: command %02x %02x %02x %02x (+%u bytes): no answer in %u ms (RAW tx %u/%u, rx %u/%u; msg 0x%x; boot "
			    "stage 0x%x)\n", step, head[0], head[1], head[2], head[3], data_len, wait_ms, rd(ipc, IPC_RAW_TX_HEAD),
			    rd(ipc, IPC_RAW_TX_TAIL), rd(ipc, IPC_RAW_RX_HEAD), rd(ipc, IPC_RAW_RX_TAIL), rd(ipc, IPC_CP2AP_MSG),
			    rd(msi, MSI_BOOT_STAGE));
			return false;
		}
		if (spins < 2000) {
			delay(20);
		} else {
			sleep_ms(1);
		}
	}
}

static bool
download_simple(uint8_t code, uint8_t kind, uint32_t wait_ms)
{
	const uint8_t head[4] = { code, kind, 0, 0 };

	return download_command(head, sizeof(head), NULL, 0, wait_ms);
}

/* a piece of a stage: sb a1, the bytes after this word, the stage's size, the piece's offset, the piece */
static bool
download_piece(unsigned stage, uint32_t total, uint32_t offset, const uint8_t *data, uint32_t len)
{
	uint8_t head[12] = { (uint8_t)(stage << 4 | 0xb), 0xa1, (uint8_t)(len + 8), (uint8_t)((len + 8) >> 8) };

	memcpy(head + 4, &total, 4);
	memcpy(head + 8, &offset, 4);
	return download_command(head, sizeof(head), data, len, 5000);
}

static bool
download(void)
{
	static const char *const nv_names[TOC_ENTRIES] = { [5] = "nv_normal.bin", [6] = "nv_protected.bin", [7] = "replay_region.bin" };
	uint8_t *chunk = kalloc_data(DOWNLOAD_CHUNK, Z_WAITOK);
	uint64_t t0 = now_ms();
	bool ok = false;

	if (chunk == NULL) {
		return false;
	}
	if (!download_simple(0x10, 0xa1, 5000) || !download_piece(1, TOC_BYTES, 0, toc_raw, TOC_BYTES) || !download_simple(0x1d, 0xa1, 5000)) {
		goto out;
	}
	printf("modem: %u: the table of contents is with the loader (%llu ms)\n", step, now_ms() - t0);
	for (unsigned stage = 2; stage < TOC_ENTRIES; stage++) {
		const struct toc_entry *e = &toc[stage];
		const uint8_t *file = NULL;
		uint64_t file_size = 0, t1 = now_ms();

		if (nv_names[stage] != NULL) {
			file = hakios_image_file("fw", nv_names[stage], &file_size, NULL);
			if (file == NULL || file_size < e->size) {
				printf("modem: %u: %s: fw/%s is not in the small image or shorter than 0x%x bytes (fw-modem-akita/ of the build): "
				    "stopped\n", step, e->name, nv_names[stage], e->size);
				goto out;
			}
		}
		if (!download_simple((uint8_t)(stage << 4), 0xa1, 5000)) {
			goto out;
		}
		for (uint32_t offset = 0; offset < e->size;) {
			uint32_t len = e->size - offset < DOWNLOAD_CHUNK ? e->size - offset : DOWNLOAD_CHUNK;

			if (file != NULL) {
				memcpy(chunk, file + offset, len);
			} else if (akita_modem_file_read(firmware, (uint64_t)e->file_offset + offset, chunk, len) != 0) {
				printf("modem: %u: %s: modem.bin could not be read at 0x%x: stopped\n", step, e->name, e->file_offset + offset);
				goto out;
			}
			if (!download_piece(stage, e->size, offset, chunk, len)) {
				printf("modem: %u: %s: stopped at 0x%x of 0x%x\n", step, e->name, offset, e->size);
				goto out;
			}
			offset += len;
		}
		if (stage == 2) {
			uint8_t head[8] = { 0x21, 0xa3, 0, 0 };

			memcpy(head + 4, &e->crc, 4);
			if (!download_command(head, sizeof(head), NULL, 0, 10000)) {
				goto out;
			}
		}
		if (!download_simple((uint8_t)(stage << 4 | 0xd), 0xa1, 10000)) {
			goto out;
		}
		printf("modem: %u: %-8s 0x%08x bytes sent in %llu ms\n", step, e->name, e->size, now_ms() - t1);
	}
	ok = download_simple(0x00, 0xa4, 5000);
	printf("modem: %u: the download %s (%llu ms)\n", step, ok ? "is done, the loader told to run it" : "was not finished", now_ms() - t0);
out:
	kfree_data(chunk, DOWNLOAD_CHUNK);
	return ok;
}

/* ---- the sequence ---- */

/* the queues emptied, the magic that says "ready": Android's init_legacy_link */
static bool
queues_reset(void)
{
	wr(ipc, IPC_MAGIC, 0);
	wr(ipc, IPC_ACCESS, 0);
	for (uint32_t o = IPC_FMT_TX_HEAD; o <= IPC_RAW_RX_TAIL; o += 4) {
		wr(ipc, o, 0);
	}
	wr(ipc, IPC_MAGIC, MAGIC_IPC);
	wr(ipc, IPC_ACCESS, 1);
	return rd(ipc, IPC_MAGIC) == MAGIC_IPC && rd(ipc, IPC_ACCESS) == 1;
}

/* the modem's configuration space after a link came up: its BAR0 where the window is, memory and mastering, its MSIs */
static bool
endpoint_setup(bool first)
{
	uint32_t id = akita_pcie0_ep_read(0x00);

	if ((id & 0xffff) != EP_VENDOR_SAMSUNG) {
		printf("modem: %u: the endpoint's identity reads 0x%08x, not Samsung's (0x144d): stopped\n", step, id);
		return false;
	}
	if (!first && ep_pm_cap != 0) {
		akita_pcie0_ep_write(ep_pm_cap + 4, akita_pcie0_ep_read(ep_pm_cap + 4) & ~0x3u);       /* D0 */
		sleep_ms(10);
	}
	akita_pcie0_ep_write(0x10, EP_BAR0_BUS);
	akita_pcie0_ep_write(0x14, 0);
	akita_pcie0_ep_write(0x04, (akita_pcie0_ep_read(0x04) & 0xffff0000u) | 0x6);
	if (first) {
		/* the MSI capability: where its writes go (the reserved page: the root complex catches them), four vectors, on */
		uint32_t control = akita_pcie0_ep_read(EP_MSI_CAP);
		bool wide = (control & (1u << 23)) != 0;

		akita_pcie0_ep_write(EP_MSI_CAP + 4, (uint32_t)MSI_PHYS);
		if (wide) {
			akita_pcie0_ep_write(EP_MSI_CAP + 8, 0);
		}
		akita_pcie0_ep_write(EP_MSI_CAP + (wide ? 12 : 8), 4);       /* data 4: its four vectors are 4 to 7 of group 0, as Android's */
		akita_pcie0_ep_write(EP_MSI_CAP, (control & ~(7u << 20)) | (2u << 20) | (1u << 16));
		for (unsigned i = 0; i < 4; i++) {
			ep_saved_msi[i] = akita_pcie0_ep_read(EP_MSI_CAP + 4 * i);
		}
		printf("modem: %u: the endpoint %04x:%04x, class 0x%06x; BAR0 0x%08x; command 0x%04x; MSI capability %08x %08x %08x %08x\n",
		    step, id & 0xffff, id >> 16, akita_pcie0_ep_read(0x08) >> 8, akita_pcie0_ep_read(0x10), akita_pcie0_ep_read(0x04) & 0xffff,
		    akita_pcie0_ep_read(EP_MSI_CAP), akita_pcie0_ep_read(EP_MSI_CAP + 4), akita_pcie0_ep_read(EP_MSI_CAP + 8),
		    akita_pcie0_ep_read(EP_MSI_CAP + 12));
	} else {
		for (unsigned i = 1; i < 4; i++) {
			akita_pcie0_ep_write(EP_MSI_CAP + 4 * i, ep_saved_msi[i]);
		}
		akita_pcie0_ep_write(EP_MSI_CAP, ep_saved_msi[0]);
		printf("modem: %u: the endpoint %04x:%04x again; BAR0 0x%08x; command 0x%04x; MSI %08x\n", step, id & 0xffff, id >> 16,
		    akita_pcie0_ep_read(0x10), akita_pcie0_ep_read(0x04) & 0xffff, akita_pcie0_ep_read(EP_MSI_CAP));
	}
	if (first) {
		/* its power-management capability, for D3hot before the link sleeps */
		for (uint32_t at = akita_pcie0_ep_read(0x34) & 0xfc, n = 0; at >= 0x40 && n < 16; n++) {
			uint32_t cap = akita_pcie0_ep_read(at);

			if ((cap & 0xff) == 0x01) {
				ep_pm_cap = at;
				break;
			}
			at = (cap >> 8) & 0xfc;
		}
	}
	if (doorbell == NULL) {
		doorbell = akita_pcie0_window(DOORBELL_OFFSET, 0x1000);
	}
	link_up = doorbell != NULL;
	return link_up;
}

/*
 * Interrupts for step 9, so that it need not look 200 times a second: the modem's MSIs (group 0 of the root complex,
 * a level that stays until the status is cleared: masked here, the thread clears it and lets it in again) and the two
 * pins the modem speaks with -- its wake pin and its "active" pin, both edges, through the alive pin controller's
 * interrupt registers. Each only wakes the thread. A line that fires without end (a wrong register here) is masked
 * for good and said, and the thread goes on by the clock as before.
 */
static volatile uint32_t irq_event, irq_msi_masked, irq_count[3], irq_storm;
static bool irq_on, irq_attached;

static void
modem_interrupt(void *context, uint32_t intid)
{
	unsigned which = (unsigned)(uintptr_t)context;

	if (which == 0) {
		gic_v3_spi_mask(intid);         /* (until the thread has cleared the status) */
		irq_msi_masked = 1;
	} else if (which == 1) {
		wr(pin_alive, EINT_PEND_GPA6, 1u << 5);
	} else {
		wr(pin_alive, EINT_PEND_GPA7, 1u << 0);
	}
	if (++irq_count[which] > 20000 && !irq_storm && which != 0) {
		gic_v3_spi_mask(intid);
		irq_storm = 1 + which;
	}
	irq_event = 1;
	thread_wakeup((event_t)&irq_event);
}

static void
interrupts_on(void)
{
	volatile uint8_t *b6 = pin_alive + BANK_GPA6, *b7 = pin_alive + BANK_GPA7;

	/* the two pins as interrupt inputs (their level still reads in DAT), both edges, nothing waiting, let in */
	wr(pin_alive, EINT_MASK_GPA6, rd(pin_alive, EINT_MASK_GPA6) | (1u << 5));
	wr(pin_alive, EINT_MASK_GPA7, rd(pin_alive, EINT_MASK_GPA7) | (1u << 0));
	wr(b6, PIN_CON, rd(b6, PIN_CON) | (0xfu << 20));
	wr(b7, PIN_CON, rd(b7, PIN_CON) | 0xfu);
	wr(pin_alive, EINT_CON_GPA6, (rd(pin_alive, EINT_CON_GPA6) & ~(0xfu << 20)) | (4u << 20));
	wr(pin_alive, EINT_CON_GPA7, (rd(pin_alive, EINT_CON_GPA7) & ~0xfu) | 4u);
	wr(pin_alive, EINT_PEND_GPA6, 1u << 5);
	wr(pin_alive, EINT_PEND_GPA7, 1u << 0);
	if (!irq_attached && (gic_v3_spi_attach(INTID_MSI0, false, modem_interrupt, (void *)0) != KERN_SUCCESS ||
	    gic_v3_spi_attach(INTID_CP2AP_WAKEUP, false, modem_interrupt, (void *)1) != KERN_SUCCESS ||
	    gic_v3_spi_attach(INTID_CP2AP_ACTIVE, false, modem_interrupt, (void *)2) != KERN_SUCCESS)) {
		printf("modem: %u: the interrupts could not be attached: by the clock\n", step);
		return;
	}
	irq_attached = true;
	wr(pin_alive, EINT_MASK_GPA6, rd(pin_alive, EINT_MASK_GPA6) & ~(1u << 5));
	wr(pin_alive, EINT_MASK_GPA7, rd(pin_alive, EINT_MASK_GPA7) & ~(1u << 0));
	irq_on = true;
	printf("modem: %u: interrupts: the MSIs (%u), the modem's wake pin (%u) and active pin (%u); gpa6 CON 0x%08x, gpa7 CON 0x%08x\n",
	    step, INTID_MSI0, INTID_CP2AP_WAKEUP, INTID_CP2AP_ACTIVE, rd(b6, PIN_CON), rd(b7, PIN_CON));
}

/* the thread's wait in step 9: until an interrupt, or ms at most (without interrupts: 5 ms as before) */
static void
stay_wait(uint32_t ms)
{
	if (!irq_on || irq_storm) {
		sleep_ms(5);
		return;
	}
	assert_wait_timeout((event_t)&irq_event, THREAD_UNINT, ms, NSEC_PER_MSEC);
	if (irq_event) {
		clear_wait(current_thread(), THREAD_AWAKENED);
	} else {
		thread_block(THREAD_CONTINUE_NULL);
	}
	irq_event = 0;
}

/*
 * The link's sleep, as Android's driver keeps it: the modem's wake pin low says the link may go; with nothing of
 * ours waiting in the queues it is taken down (the modem to D3hot, mastering off, PME_Turn_Off) and our wake pin
 * dropped. The modem's wake pin high -- by itself, or answering ours -- brings it up again at Gen3, the modem's
 * configuration put back, the "link ack" doorbell rung and then the one that was owed.
 */
static void
link_follow(void)
{
	static uint64_t idle_since;
	int cp_wake = pin_get(&CP2AP_WAKEUP);

	if (link_up) {
		bool idle = cp_wake == 0 && !doorbell_owed && rd(ipc, IPC_FMT_TX_HEAD) == rd(ipc, IPC_FMT_TX_TAIL) &&
		    rd(ipc, IPC_RAW_TX_HEAD) == rd(ipc, IPC_RAW_TX_TAIL) && queue_used(&FMT_RX) == 0 && queue_used(&RAW_RX) == 0;

		if (!idle) {
			idle_since = 0;
		} else if (idle_since == 0) {
			idle_since = now_ms();
		} else if (now_ms() - idle_since >= 30) {
			idle_since = 0;
			link_up = false;
			akita_pcie0_ep_write(0x04, akita_pcie0_ep_read(0x04) & ~0x4u);
			if (ep_pm_cap != 0) {
				akita_pcie0_ep_write(ep_pm_cap + 4, akita_pcie0_ep_read(ep_pm_cap + 4) | 0x3);  /* D3hot */
			}
			pin_out(&AP2CP_WAKEUP, 0, 5);
			akita_pcie0_down();
			link_sleeps++;
		}
	} else if (cp_wake != 0) {
		uint64_t t0 = now_ms();

		pin_out(&AP2CP_WAKEUP, 1, 5);
		if (!akita_pcie0_up(3) || !endpoint_setup(false)) {
			printf("modem: %u: the link did not come back (active %d)\n", step, pin_get(&CP2AP_CP_ACTIVE));
			return;
		}
		ring(DOORBELL_LINK_ACK);
		if (doorbell_owed) {
			doorbell_owed = false;
			ring(DOORBELL_MSG);
		}
		link_wakes++;
		printf("modem: %u: the link is up again after %llu ms (sleep %u, wake %u)\n", step, now_ms() - t0, link_sleeps, link_wakes);
	}
}

/* a word from the modem (cp2ap_msg), waited for; what else it says meanwhile is said */
static bool
message_wait(uint32_t word, uint32_t wait_ms, const char *name)
{
	uint64_t t0 = now_ms();
	uint32_t last = rd(ipc, IPC_CP2AP_MSG), stage = rd(msi, MSI_BOOT_STAGE);

	for (;;) {
		uint32_t now = rd(ipc, IPC_CP2AP_MSG), s = rd(msi, MSI_BOOT_STAGE);

		(void)akita_pcie0_msi_take();
		if (now == word) {
			printf("modem: %u: %s (0x%x) from the modem after %llu ms\n", step, name, word, now_ms() - t0);
			return true;
		}
		if (now != last || s != stage) {
			printf("modem: %u: waiting for %s: the modem's word 0x%x, boot stage 0x%x (%llu ms)\n", step, name, now, s, now_ms() - t0);
			last = now;
			stage = s;
		}
		if (now_ms() - t0 > wait_ms) {
			printf("modem: %u: %s (0x%x) did not come in %u ms: the modem's word 0x%x; active %d, wake %d; RAW rx %u/%u\n", step, name,
			    word, wait_ms, now, pin_get(&CP2AP_CP_ACTIVE), pin_get(&CP2AP_WAKEUP), rd(ipc, IPC_RAW_RX_HEAD), rd(ipc, IPC_RAW_RX_TAIL));
			return false;
		}
		sleep_ms(1);
	}
}

/* the packet processor's two headers as Android leaves them (the modem's addresses of the rings); no buffer is given */
static void
pktproc_headers(bool uplink)
{
	static const uint32_t dl[4][3] = { { 0x20001000, 0xd80, 0x20100000 }, { 0x2000e800, 0xd80, 0x207c0000 },
		{ 0x2001c000, 0xd80, 0x20e80000 }, { 0x20029800, 0xd80, 0x21540000 } };
	static const uint32_t ul[2][3] = { { 0x21c01000, 0x6e0, 0x21c90000 }, { 0x21c0ec00, 0x370, 0x21e48000 } };

	if (!uplink) {
		wr(pktproc, PKTPROC_DL_INFO, 0x00063054);       /* 4 queues, socket-buffer descriptors, own interrupts, 0x630 a packet */
		for (unsigned q = 0; q < 4; q++) {
			volatile uint8_t *info = pktproc + PKTPROC_DL_INFO + 4 + 20 * q;

			wr(info, 0, dl[q][0]);
			wr(info, 4, dl[q][1]);
			wr(info, 8, dl[q][2]);
			wr(info, 12, 0);        /* fore: no buffer of ours yet */
			wr(info, 16, 0);        /* rear */
		}
	} else {
		for (unsigned q = 0; q < 2; q++) {
			volatile uint8_t *info = pktproc + PKTPROC_UL_INFO + 8 + 20 * q;

			wr(info, 0, ul[q][0]);
			wr(info, 4, ul[q][1]);
			wr(info, 8, ul[q][2]);
			wr(info, 12, 0);
			wr(info, 16, 0);
		}
	}
}

/*
 * What the modem said last, for Darwin's side: the names userland/src/inject_sb.c reads for the status bar (the same
 * as the other phone's modem driver gives: cellular_up, _registered, _bars, _dbm, _operator, _radio) and three more.
 * Layouts: the vendor's parsers read (vendor.radio.protocol.sit.stream), checked against a start without a SIM; the
 * registration, operator and signal ones have not been seen filled yet (no SIM was in the phone).
 */
static int cellular_up, cellular_registered, cellular_bars, cellular_dbm, cellular_radio = -1, cellular_sim = -1, cellular_rat;
static char cellular_operator[72], cellular_baseband[64];

SYSCTL_DECL(_kern_hakios);
SYSCTL_INT(_kern_hakios, OID_AUTO, cellular_up, CTLFLAG_RD | CTLFLAG_LOCKED, &cellular_up, 0, "1: the modem is registered on a network");
SYSCTL_INT(_kern_hakios, OID_AUTO, cellular_registered, CTLFLAG_RD | CTLFLAG_LOCKED, &cellular_registered, 0,
    "the packet registration state: 1 home, 5 roaming");
SYSCTL_INT(_kern_hakios, OID_AUTO, cellular_bars, CTLFLAG_RD | CTLFLAG_LOCKED, &cellular_bars, 0, "signal, 0 to 4 bars");
SYSCTL_INT(_kern_hakios, OID_AUTO, cellular_dbm, CTLFLAG_RD | CTLFLAG_LOCKED, &cellular_dbm, 0, "signal, dBm; 0: none read");
SYSCTL_INT(_kern_hakios, OID_AUTO, cellular_radio, CTLFLAG_RD | CTLFLAG_LOCKED, &cellular_radio, 0,
    "the modem's radio state (0 off, 1 unavailable, 3 no SIM or locked, 4 SIM ready, 10 on; -1: not asked)");
SYSCTL_INT(_kern_hakios, OID_AUTO, cellular_sim, CTLFLAG_RD | CTLFLAG_LOCKED, &cellular_sim, 0,
    "the SIM: 0 absent, 1 present, 2 error, 5 present and ready; -1: not asked");
SYSCTL_INT(_kern_hakios, OID_AUTO, cellular_rat, CTLFLAG_RD | CTLFLAG_LOCKED, &cellular_rat, 0, "radio technology (14 LTE, 21 NR, 3 UMTS, 16 GSM)");
SYSCTL_STRING(_kern_hakios, OID_AUTO, cellular_operator, CTLFLAG_RD | CTLFLAG_LOCKED, cellular_operator, 0, "the network's name");
SYSCTL_STRING(_kern_hakios, OID_AUTO, cellular_baseband, CTLFLAG_RD | CTLFLAG_LOCKED, cellular_baseband, 0, "the baseband's version");

static bool sit_request(unsigned id, const uint8_t *parameters, uint32_t len);

/*
 * (11-19) The SIM's PIN. A card whose application waits for its PIN (state 2) registers nowhere; Darwin has no telephony
 * here to ask for it, so SpringBoard (userland/src/inject_sb.c) asks and writes the digits to kern.hakios.cellular_pin;
 * this thread sends them (SIT_VERIFY_SIM_PIN: the length, 8 characters, the application's identifier as the card's status
 * gave it -- ~/akita/ref/modem-android/sit-layouts.md, from rild's library) and the buffer is wiped. THE DIGITS ARE NEVER
 * PRINTED, here or anywhere: a request is logged by its number alone, the answer holds only the tries left.
 * cellular_pin_state: 0 no PIN is wanted (or not known), 1 wanted, 2 sent and not answered yet, 3 accepted, 4 refused
 * (wrong: cellular_pin_tries says how many tries the card has left before it wants its PUK).
 */
static int cellular_pin_state, cellular_pin_tries = -1;
static uint8_t sim_aid[16], sim_aid_len;
static char pin_waiting[9];
static volatile bool pin_to_send;
static unsigned sit_answer_error;       /* the error of the answer sit_understand() is looking at */

static int
pin_sysctl SYSCTL_HANDLER_ARGS
{
	char digits[16] = "";
	int error = sysctl_handle_string(oidp, digits, sizeof(digits), req);
	size_t n = strnlen(digits, sizeof(digits));

	(void)arg1;
	(void)arg2;
	if (error != 0 || req->newptr == USER_ADDR_NULL) {
		return error;
	}
	for (size_t i = 0; i < n; i++) {
		if (digits[i] < '0' || digits[i] > '9') {
			n = 0;
		}
	}
	if (n < 4 || n > 8) {
		bzero(digits, sizeof(digits));
		return EINVAL;
	}
	if ((cellular_pin_state != 1 && cellular_pin_state != 4) || pin_to_send) {
		bzero(digits, sizeof(digits));
		return EBUSY;                   /* (no PIN is wanted, or one is on its way) */
	}
	bzero(pin_waiting, sizeof(pin_waiting));
	memcpy(pin_waiting, digits, n);
	bzero(digits, sizeof(digits));
	cellular_pin_state = 2;
	pin_to_send = true;
	return 0;
}
SYSCTL_PROC(_kern_hakios, OID_AUTO, cellular_pin, CTLTYPE_STRING | CTLFLAG_WR | CTLFLAG_ANYBODY | CTLFLAG_LOCKED, NULL, 0, pin_sysctl, "A",
    "the SIM's PIN, to be sent to the card (write only; 4 to 8 digits)");
SYSCTL_INT(_kern_hakios, OID_AUTO, cellular_pin_state, CTLFLAG_RD | CTLFLAG_LOCKED, &cellular_pin_state, 0,
    "the SIM's PIN: 0 not wanted, 1 wanted, 2 sent, 3 accepted, 4 refused");
SYSCTL_INT(_kern_hakios, OID_AUTO, cellular_pin_tries, CTLFLAG_RD | CTLFLAG_LOCKED, &cellular_pin_tries, 0,
    "the tries the SIM's PIN has left (-1: not known)");

/* (the modem's thread: a PIN that was written goes out, and is wiped) */
static void
pin_send(void)
{
	uint8_t request[26];
	size_t n;

	if (!pin_to_send) {
		return;
	}
	n = strnlen(pin_waiting, 8);
	bzero(request, sizeof(request));
	request[0] = (uint8_t)n;
	memcpy(request + 1, pin_waiting, n);
	request[9] = sim_aid_len;
	memcpy(request + 10, sim_aid, sim_aid_len);
	bzero(pin_waiting, sizeof(pin_waiting));
	pin_to_send = false;
	if (!sit_request(SIT_VERIFY_SIM_PIN, request, sizeof(request))) {
		cellular_pin_state = 4;
	}
	bzero(request, sizeof(request));
	printf("modem: %u: the SIM's PIN sent to the card (%u digits; the card had %d tries left)\n", step, (unsigned)n, cellular_pin_tries);
}

static int32_t
s32_at(const uint8_t *d, uint32_t off)
{
	int32_t v;

	memcpy(&v, d + off, 4);
	return v;
}

/* a message of SIM 1's stack understood: d is what follows its header (12 bytes of an answer, 8 of an indication) */
static void
sit_understand(unsigned type, unsigned id, const uint8_t *d, uint32_t n)
{
	if (type == 2 && id == SIT_IND_SIM_STATUS_CHANGED) {
		(void)sit_request(SIT_GET_SIM_STATUS, NULL, 0);
	} else if (type == 2 && id == SIT_IND_NTW_STATE_CHANGED) {
		(void)sit_request(SIT_GET_PS_REG_STATE, NULL, 0);
		(void)sit_request(SIT_GET_OPERATOR, NULL, 0);
		(void)sit_request(SIT_GET_SIGNAL_STRENGTH, NULL, 0);
	} else if (type == 2 && (id == SIT_IND_RADIO_STATE_CHANGED || id == SIT_IND_RADIO_READY)) {
		(void)sit_request(SIT_GET_RADIO_POWER, NULL, 0);
	} else if (type == 1 && id == SIT_GET_RADIO_POWER && n >= 4) {
		if (cellular_radio != s32_at(d, 0)) {
			printf("modem: %u: the radio's state: %d (was %d)\n", step, s32_at(d, 0), cellular_radio);
		}
		cellular_radio = s32_at(d, 0);
	} else if (type == 1 && id == SIT_GET_BASEBAND_VERSION && n >= 2) {
		strlcpy(cellular_baseband, (const char *)d + 1, n - 1 < sizeof(cellular_baseband) ? n - 1 : sizeof(cellular_baseband));
	} else if (type == 1 && id == SIT_GET_SIM_STATUS && n >= 3) {
		/* the card's state, then 63 bytes an application: its type, its state (5: ready; 2: PIN wanted, 3: PUK) */
		int state = d[0] == 3 ? 1 : d[0];

		if (state == 1 && d[2] != 0 && n >= 3 + 63) {
			printf("modem: %u: the SIM: present, %u application%s; the first: type %u, state %u (5 ready, 2 PIN, 3 PUK), PIN1 state %u, "
			    "%u tries left\n", step, d[2], d[2] == 1 ? "" : "s", d[3], d[4], d[3 + 57], d[3 + 59]);
			sim_aid_len = d[3 + 3] <= 16 ? d[3 + 3] : 0;
			memcpy(sim_aid, d + 3 + 4, sim_aid_len);
			cellular_pin_tries = d[3 + 59];
			if (d[4] == 5) {
				state = 5;
				if (cellular_pin_state != 0) {
					cellular_pin_state = 3;
				}
			} else if (d[4] == 2 && cellular_pin_state != 2 && cellular_pin_state != 4) {
				cellular_pin_state = 1;         /* (the PIN is wanted) */
			}
		} else if (state != cellular_sim) {
			printf("modem: %u: the SIM: card state %u (0 absent, 1 present, 2 error), %u applications\n", step, d[0], d[2]);
		}
		cellular_sim = state;
	} else if (type == 1 && id == SIT_VERIFY_SIM_PIN) {
		if (n >= 4 && s32_at(d, 0) >= 0) {
			cellular_pin_tries = s32_at(d, 0);
		}
		cellular_pin_state = sit_answer_error == 0 ? 3 : 4;
		printf("modem: %u: the SIM's PIN: %s (error %u); %d tries left\n", step, sit_answer_error == 0 ? "ACCEPTED" : "refused",
		    sit_answer_error, cellular_pin_tries);
		(void)sit_request(SIT_GET_SIM_STATUS, NULL, 0);
		(void)sit_request(SIT_GET_PS_REG_STATE, NULL, 0);
	} else if (type == 1 && id == SIT_GET_PS_REG_STATE && n >= 4) {
		bool registered = d[0] == 1 || d[0] == 5;

		if (cellular_registered != d[0] || cellular_rat != d[3]) {
			printf("modem: %u: packet registration: state %u (1 home, 5 roaming, 2 searching, 3 denied), reject cause %u, technology %u\n",
			    step, d[0], d[1], d[3]);
		}
		cellular_registered = registered ? d[0] : 0;
		cellular_rat = d[3];
		cellular_up = registered;
		/* (the bars are the signal's own: a network heard without a SIM has a strength too, and the next signal
		 * indication, a second later, says it again) */
	} else if (type == 1 && id == SIT_GET_OPERATOR && n >= 102) {
		/* the network's number (6 characters, '#' for none), a short name (32), a long one (64) */
		char name[72];
		const char *from = d[38] != 0 ? (const char *)d + 38 : d[6] != 0 ? (const char *)d + 6 : (const char *)d;
		size_t room = d[38] != 0 ? 64 : d[6] != 0 ? 32 : 6, k = 0;

		for (size_t i = 0; i < room && from[i] != 0 && from[i] != '#' && k < sizeof(name) - 1; i++) {
			name[k++] = from[i] >= 0x20 && from[i] < 0x7f ? from[i] : '?';
		}
		name[k] = 0;
		if (strcmp(name, cellular_operator) != 0) {
			printf("modem: %u: the network: \"%s\"\n", step, name);
		}
		strlcpy(cellular_operator, name, sizeof(cellular_operator));
	} else if ((id == SIT_GET_SIGNAL_STRENGTH && type == 1 || id == SIT_IND_SIGNAL_STRENGTH && type == 2) && n >= 102) {
		/* which technologies have a reading (a mask), then 32-bit values: LTE's RSRP at 66, NR's at 90 (both as -dBm),
		 * WCDMA's RSCP at 22, GSM's strength at 2 (0..31: -113 + 2n dBm) */
		unsigned mask = d[0] | d[1] << 8;
		int dbm = 0, v;

		if ((mask & 0x40) != 0 && (v = s32_at(d, 90)) > 0 && v < 200) {
			dbm = -v;
		} else if ((mask & 0x84) != 0 && (v = s32_at(d, 66)) > 0 && v < 200) {
			dbm = -v;
		} else if ((mask & 0x02) != 0 && (v = s32_at(d, 22)) != 0 && v != 0x7fffffff) {
			dbm = v > 0 ? -v : v;
		} else if ((mask & 0x01) != 0 && (v = s32_at(d, 2)) >= 0 && v <= 31) {
			dbm = -113 + 2 * v;
		}
		if (dbm != cellular_dbm) {
			printf("modem: %u: the signal: mask 0x%x: %d dBm (GSM %d, WCDMA RSCP %d, LTE RSRP %d RSRQ %d, NR RSRP %d)\n", step, mask, dbm,
			    s32_at(d, 2), s32_at(d, 22), s32_at(d, 66), s32_at(d, 70), s32_at(d, 90));
		}
		cellular_dbm = dbm;
		cellular_bars = dbm == 0 ? 0 : dbm >= -95 ? 4 : dbm >= -105 ? 3 : dbm >= -115 ? 2 : 1;
	}
}

static const char *
sit_name(unsigned id)
{
	static const struct { uint16_t id; const char *name; } names[] = {
		{ SIT_GET_SIM_STATUS, "GET_SIM_STATUS" }, { SIT_VERIFY_SIM_PIN, "VERIFY_SIM_PIN" }, { SIT_IND_SIM_STATUS_CHANGED, "IND_SIM_STATUS_CHANGED" },
		{ 0x0246, "IND_SIM_PB_READY" }, { 0x024d, "GET_SLOT_STATUS" }, { 0x024e, "IND_SIM_SLOT_STATUS_CHANGED" },
		{ SIT_GET_CS_REG_STATE, "GET_CS_REG_STATE" }, { SIT_GET_PS_REG_STATE, "GET_PS_REG_STATE" },
		{ SIT_GET_OPERATOR, "GET_OPERATOR" }, { SIT_SET_RADIO_POWER, "SET_RADIO_POWER" },
		{ SIT_GET_RADIO_POWER, "GET_RADIO_POWER" }, { SIT_IND_RADIO_STATE_CHANGED, "IND_RADIO_STATE_CHANGED" },
		{ SIT_IND_RADIO_READY, "IND_RADIO_READY" }, { SIT_GET_SIGNAL_STRENGTH, "GET_SIGNAL_STRENGTH" },
		{ SIT_GET_BASEBAND_VERSION, "GET_BASEBAND_VERSION" }, { SIT_IND_SIGNAL_STRENGTH, "IND_SIGNAL_STRENGTH" },
		{ SIT_SET_DEVICE_INFO, "SET_DEVICE_INFO" }, { SIT_SET_AP_SYSTEM_TIME, "SET_AP_SYSTEM_TIME" },
		{ SIT_IND_NTW_STATE_CHANGED, "IND_NTW_STATE_CHANGED" }, { 0x0248, "IND 0x0248" }, { 0x0306, "IND_STK_SESSION_END" },
	};

	for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
		if (names[i].id == id) {
			return names[i].name;
		}
	}
	return "";
}

/* a request of the command protocol on SIM 1's stack; the modem told to look */
static bool
sit_request(unsigned id, const uint8_t *parameters, uint32_t len)
{
	static uint16_t token = 0x100;
	uint8_t head[12] = { 0, 0, (uint8_t)id, (uint8_t)(id >> 8), (uint8_t)(12 + len), (uint8_t)((12 + len) >> 8) };

	token++;
	head[6] = (uint8_t)token;
	head[7] = (uint8_t)(token >> 8);
	if (!frame_put(&FMT_TX, CH_RIL, head, sizeof(head), parameters, len)) {
		return false;
	}
	message(MSG_SEND_FMT);
	printf("modem: %u: asked 0x%04x %s (token 0x%x)\n", step, id, sit_name(id), token);
	return true;
}

/* every whole frame in a queue said: its channel, its length, its first bytes */
static unsigned
frames_say(const struct queue *q)
{
	static uint8_t payload[1025];
	unsigned n = 0;

	for (;;) {
		uint8_t channel = 0;
		uint32_t length = 0, shown;
		uint16_t config = 0;
		char text[3 * 96 + 1];
		int r = frame_take(q, &channel, payload, sizeof(payload) - 1, &length, &config);

		if (r <= 0) {
			return n;
		}
		shown = length < 96 ? length : 96;
		for (uint32_t i = 0; i < shown; i++) {
			snprintf(text + 3 * i, 4, " %02x", payload[i]);
		}
		text[3 * shown] = 0;
		if ((channel == CH_RIL || channel == CH_RIL + 1) && length >= 8) {
			unsigned type = payload[0] | payload[1] << 8, id = payload[2] | payload[3] << 8, head = type == 2 ? 8 : 12;
			const char *name = sit_name(id);

			if (type == 2 && id == SIT_IND_SIGNAL_STRENGTH) {
				/* (once a second while a cell is heard: only a change is said, by sit_understand) */
			} else if (type == 1 && length >= 12) {
				printf("modem: %u: SIM stack %u: answer 0x%04x %s (token 0x%x, error %u), %u bytes:%s\n", step, channel - CH_RIL + 1, id,
				    name, payload[6] | payload[7] << 8, payload[10] | payload[11] << 8, length - head, text + 3 * head);
			} else {
				printf("modem: %u: SIM stack %u: %s 0x%04x %s, %u bytes:%s\n", step, channel - CH_RIL + 1,
				    type == 2 ? "indication" : "message", id, name, length > head ? length - head : 0, shown > head ? text + 3 * head : "");
			}
			if (channel == CH_RIL && length >= head && length <= sizeof(payload) - 1) {
				sit_answer_error = type == 1 && length >= 12 ? payload[10] | payload[11] << 8 : 0;
				sit_understand(type, id, payload + head, length - head);
			}
			if (type == 1 && id == SIT_GET_BASEBAND_VERSION && shown > 14) {
				payload[shown < sizeof(payload) ? shown : sizeof(payload) - 1] = 0;
				printf("modem: %u: the baseband's version: %s\n", step, (const char *)payload + 13);
			}
		} else {
			printf("modem: %u: %s: channel %u, %u bytes (frame config 0x%04x):%s\n", step, q->name, channel, length, config, text);
		}
		n++;
	}
}

/* ---- the file service ---- */

#define RFS_CHUNK               0x7dcu
#define RFS_MAX_FILE            0x100000u

static struct {
	uint32_t id, size, offset;      /* the transfer under way (size 0: none) */
	uint16_t sequence;
	uint8_t *data;
	uint32_t saved[8];              /* bytes kept, by file id */
} rfs;

static void
rfs_send(unsigned command, unsigned sequence, uint32_t a, uint32_t b, uint32_t c, unsigned words)
{
	uint8_t m[20] = { (uint8_t)command, 0, (uint8_t)sequence, (uint8_t)(sequence >> 8), (uint8_t)(4 * words) };

	memcpy(m + 8, &a, 4);
	memcpy(m + 12, &b, 4);
	memcpy(m + 16, &c, 4);
	if (frame_put(&RAW_TX, CH_RFS, m, 8, m + 8, 4 * words)) {
		message(MSG_SEND_RAW);
	}
}

static void
rfs_message(const uint8_t *m, uint32_t length)
{
	uint32_t command, sequence, v[4] = { 0, 0, 0, 0 };

	if (length < 8) {
		return;
	}
	command = m[0] | m[1] << 8;
	sequence = m[2] | m[3] << 8;
	memcpy(v, m + 8, length - 8 < sizeof(v) ? length - 8 : sizeof(v));
	switch (command) {
	case 7:
		printf("modem: %u: files: the modem opens NV file %u\n", step, v[0]);
		rfs_send(3, sequence, 0, v[0], 0, 2);
		break;
	case 5:
		printf("modem: %u: files: the modem closes file %u\n", step, v[0]);
		rfs_send(3, sequence, 0, v[0], 0, 2);
		break;
	case 6:
		if (v[3] != 2 || v[2] == 0 || v[2] > RFS_MAX_FILE || v[1] != 0 || rfs.size != 0) {
			printf("modem: %u: files: a transfer not served: file %u, offset 0x%x, 0x%x bytes, direction %u\n", step, v[0], v[1], v[2],
			    v[3]);
			break;
		}
		if (rfs.data == NULL && (rfs.data = kalloc_data(RFS_MAX_FILE, Z_WAITOK)) == NULL) {
			break;
		}
		rfs.id = v[0];
		rfs.size = v[2];
		rfs.offset = 0;
		rfs.sequence = (uint16_t)sequence;
		printf("modem: %u: files: the modem saves file %u: 0x%x bytes\n", step, rfs.id, rfs.size);
		rfs_send(2, sequence, rfs.id, 0, rfs.size < RFS_CHUNK ? rfs.size : RFS_CHUNK, 3);
		break;
	case 2:
		if (rfs.size == 0 || length < 20 || v[1] != rfs.id || v[2] > length - 20 || v[2] > rfs.size - rfs.offset) {
			printf("modem: %u: files: data nobody asked for (file %u, %u bytes, %u in the message)\n", step, v[1], v[2], length);
			break;
		}
		memcpy(rfs.data + rfs.offset, m + 20, v[2]);
		rfs.offset += v[2];
		if (rfs.offset < rfs.size) {
			rfs_send(2, rfs.sequence, rfs.id, rfs.offset, rfs.size - rfs.offset < RFS_CHUNK ? rfs.size - rfs.offset : RFS_CHUNK, 3);
		} else {
			uint32_t sum = 0;

			for (uint32_t i = 0; i < rfs.size; i++) {
				sum = sum * 31 + rfs.data[i];
			}
			printf("modem: %u: files: file %u: 0x%x bytes taken (kept in memory only; sum 0x%08x)\n", step, rfs.id, rfs.size, sum);
			if (rfs.id < 8) {
				rfs.saved[rfs.id] = rfs.size;
			}
			rfs_send(3, rfs.sequence, 0, rfs.id, 0, 2);
			rfs.size = 0;
		}
		break;
	case 3:
		break;          /* its acknowledgement */
	case 4:
		printf("modem: %u: files: the modem asks for a file of Android's (not served): %.*s\n", step,
		    length > 16 ? (int)(length - 16 < 120 ? length - 16 : 120) : 0, (const char *)m + 16);
		break;
	default:
		printf("modem: %u: files: command %u (sequence %u, %u bytes): %08x %08x %08x %08x\n", step, command, sequence, length, v[0], v[1],
		    v[2], v[3]);
		break;
	}
}

/* the raw queue's frames: the file service's served, the others said */
static void
raw_serve(void)
{
	static uint8_t *payload;

	if (payload == NULL && (payload = kalloc_data(2048, Z_WAITOK)) == NULL) {
		return;
	}
	for (;;) {
		uint8_t channel = 0;
		uint32_t length = 0;
		uint16_t config = 0;

		if (frame_take(&RAW_RX, &channel, payload, 2048, &length, &config) <= 0) {
			return;
		}
		if (channel == CH_RFS && length <= 2048 && (config & 0xc000) == 0xc000) {
			rfs_message(payload, length);
		} else if (channel == CH_RFS && length <= 2048 && config == 0) {
			rfs_message(payload, length);   /* (the modem's own frames here carry 0: whole messages all the same) */
		} else {
			printf("modem: %u: RAW rx: channel %u, %u bytes (frame config 0x%04x): %02x %02x %02x %02x %02x %02x %02x %02x\n", step,
			    channel, length, config, payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7]);
		}
	}
}

static void
modem_run(void)
{
	const uint8_t *handover;
	uint64_t handover_size = 0, t0, t_stay, t_said;
	uint32_t turns = 0;
	uint8_t *boot;
	uint32_t stage = 0;

	if (!announce(1, "reads: the pins, the shared memory, the ROM's page; the firmware's table of contents from modem_a")) {
		return;
	}
	pin_hsi1 = (volatile uint8_t *)ml_io_map(PIN_HSI1, 0x1000);
	pin_alive = (volatile uint8_t *)ml_io_map(PIN_ALIVE, 0x1000);
	ipc = map_ram(IPC_PHYS, IPC_SIZE);
	msi = map_ram(MSI_PHYS, 0x1000);
	pktproc = map_ram(PKTPROC_PHYS, PKTPROC_MAPPED);
	if (ipc == NULL || msi == NULL || pktproc == NULL) {
		printf("modem: 1: the shared memory could not be mapped: stopped\n");
		return;
	}
	pins_say("as found");
	shared_say("as found");
	boot_log_say();
	if (!firmware_open()) {
		return;
	}

restart:
	if (!announce(2, "writes: the shared memory for a boot, the handover block, the first-stage loader; the modem off and on")) {
		return;
	}
	handover = hakios_image_file("fw", "handover.bin", &handover_size, NULL);
	if (handover == NULL || handover_size != IPC_HANDOVER_SIZE) {
		printf("modem: 2: fw/handover.bin (%u bytes: this phone's, from fw-modem-akita/ of the build) is not in the small image: "
		    "stopped\n", IPC_HANDOVER_SIZE);
		return;
	}
	if ((boot = kalloc_data(toc[1].size, Z_WAITOK)) == NULL ||
	    akita_modem_file_read(firmware, toc[1].file_offset, boot, toc[1].size) != 0) {
		printf("modem: 2: BOOT could not be read from modem.bin: stopped\n");
		return;
	}
	/* (Android's order: the handover block; power_on_cp; the image; start_normal_boot's control words) */
	shared_put(ipc + IPC_HANDOVER, handover, IPC_HANDOVER_SIZE);
	wr(msi, MSI_BOOT_STAGE, 0);
	pin_out(&AP2CP_AP_ACTIVE, 1, 0);
	pin_out(&AP2CP_DUMP_NOTI, 0, 0);
	pin_input(&CP2AP_CP_WRST_N, 0);
	pin_input(&CP2AP_PS_HOLD, 0);
	pin_input(&CP2AP_WAKEUP, 1);            /* pulled down */
	pin_input(&CP2AP_CP_ACTIVE, 0);
	wr(ipc, IPC_AP2CP_MSG, 0);
	wr(ipc, IPC_CP2AP_MSG, 0);
	/* off ... */
	pin_out(&AP2CP_WAKEUP, 1, 10);
	pin_out(&AP2CP_WAKEUP, 0, 0);
	pin_out(&AP2CP_NRESET, 0, 0);
	pin_out(&AP2CP_CP_WRST_N, 0, 0);
	pin_out(&AP2CP_CP_PWR, 0, 30);
	pin_out(&AP2CP_PM_WRST_N, 0, 50);
	pins_say("off");
	/* ... and on */
	pin_out(&AP2CP_PM_WRST_N, 1, 10);
	pin_out(&AP2CP_CP_PWR, 1, 10);
	pin_out(&AP2CP_NRESET, 1, 10);
	pin_out(&AP2CP_CP_WRST_N, 1, 0);
	pins_say("on");
	shared_put(ipc + IPC_BOOT_IMG, boot, toc[1].size);
	kfree_data(boot, toc[1].size);
	wr(ipc, IPC_CAP_OFFSET, IPC_AP_CAP0);
	wr(ipc, IPC_SRINFO_OFFSET, 0x400000);
	wr(ipc, IPC_AP2CP_STATUS, 0);
	wr(ipc, IPC_CP2AP_STATUS, 0);
	for (uint32_t o = IPC_AP_CAP0; o <= IPC_CP_CAP1; o += 4) {
		wr(ipc, o, 0);
	}
	wr(ipc, IPC_AP2CP_STATUS, STATUS_DUAL_SIM);
	pin_out(&AP2CP_AP_ACTIVE, 1, 0);
	if (!queues_reset()) {
		printf("modem: 2: the shared memory does not keep what is written (magic 0x%x): stopped\n", rd(ipc, IPC_MAGIC));
		return;
	}
	wr(ipc, IPC_MAGIC, MAGIC_BOOT);
	shared_say("for the boot");
	sleep_ms(200);
	pins_say("200 ms on");

	if (!announce(3, "writes: PCIe channel 0 up at Gen1, the modem on it; the loader told to the ROM, the doorbell; the boot stage")) {
		return;
	}
	if (!akita_pcie0_up(1)) {
		printf("modem: 3: no link (boot stage 0x%x, error 0x%x): stopped\n", rd(msi, MSI_BOOT_STAGE), rd(msi, MSI_ERR_REPORT));
		pins_say("no link");
		return;
	}
	if (!endpoint_setup(true)) {
		return;
	}
	/* the ROM says when it waits for the doorbell (stage bit 7; Android reads 0xff at this point) */
	t0 = now_ms();
	while ((rd(msi, MSI_BOOT_STAGE) & 0x80) == 0 && now_ms() - t0 < 1000) {
		sleep_ms(2);
	}
	printf("modem: 3: boot stage 0x%x before the doorbell (%llu ms after the link)\n", rd(msi, MSI_BOOT_STAGE), now_ms() - t0);
	wr(msi, MSI_IMG_ADDR_LO, (uint32_t)(IPC_PHYS + IPC_BOOT_IMG));
	wr(msi, MSI_IMG_ADDR_HI, 0);
	wr(msi, MSI_IMG_SIZE, toc[1].size);
	ring(DOORBELL_MSG);
	t0 = now_ms();
	for (;;) {
		uint32_t s = rd(msi, MSI_BOOT_STAGE);

		if (s != stage) {
			printf("modem: 3: boot stage 0x%x (%llu ms)\n", s, now_ms() - t0);
			stage = s;
		}
		if (s == BOOT_STAGE_DONE) {
			break;
		}
		if (now_ms() - t0 > 4000) {
			printf("modem: 3: the ROM did not finish in 4 s: boot stage 0x%x, error 0x%x, the doorbell reads 0x%x: stopped\n", s,
			    rd(msi, MSI_ERR_REPORT), rd(doorbell, 0));
			pins_say("ROM");
			return;
		}
		sleep_ms(2);
	}
	printf("modem: 3: the ROM took the loader and jumped to it\n");

	if (!announce(4, "writes: the link down, the modem's wake pin, the link up at Gen3, the link ack")) {
		return;
	}
	sleep_ms(30);
	pin_out(&AP2CP_WAKEUP, 0, 5);
	link_up = false;
	akita_pcie0_down();
	t0 = now_ms();
	while (pin_get(&CP2AP_WAKEUP) == 0) {
		if (now_ms() - t0 > 4000) {
			printf("modem: 4: the modem's wake pin stayed low for 4 s: stopped\n");
			pins_say("no wake");
			boot_log_say();
			return;
		}
		sleep_ms(5);
	}
	printf("modem: 4: the modem's wake pin went high after %llu ms\n", now_ms() - t0);
	pin_out(&AP2CP_WAKEUP, 1, 5);
	if (!akita_pcie0_up(3)) {
		printf("modem: 4: no link: stopped\n");
		pins_say("no link");
		boot_log_say();
		return;
	}
	if (!endpoint_setup(false)) {
		return;
	}
	ring(DOORBELL_LINK_ACK);
	boot_log_say();

	if (!announce(5, "writes: the firmware to the loader (the table, MAIN, VSS, APM, NV_NORM, NV_PROT, REPLAY)")) {
		return;
	}
	if (!download()) {
		shared_say("download");
		pins_say("download");
		boot_log_say();
		return;
	}

	if (!announce(6, "writes: the start handshake (INIT_START, PIF_INIT_DONE, PHONE_START, INIT_END)")) {
		return;
	}
	if (!message_wait(MSG_INIT_START, 15000, "INIT_START")) {
		shared_say("handshake");
		boot_log_say();
		return;
	}
	pktproc_headers(false);
	wr(ipc, IPC_AP_CAP0, AP_CAPABILITIES);
	wr(ipc, IPC_AP_CAP1, 0);
	message(MSG_PIF_INIT_DONE);
	if (!message_wait(MSG_PHONE_START, 15000, "PHONE_START")) {
		shared_say("handshake");
		boot_log_say();
		return;
	}
	printf("modem: 6: capabilities: ours 0x%x, the modem's 0x%x%s\n", AP_CAPABILITIES, rd(ipc, IPC_CP_CAP0),
	    (AP_CAPABILITIES & ~rd(ipc, IPC_CP_CAP0)) != 0 ? " -- IT LACKS ONE OF OURS" : "");
	printf("modem: 6: the uplink header as the modem left it: 0x%x 0x%x\n", rd(pktproc, PKTPROC_UL_INFO), rd(pktproc, PKTPROC_UL_INFO + 4));
	pktproc_headers(true);
	if (!queues_reset()) {
		printf("modem: 6: the queues could not be reset: stopped\n");
		return;
	}
	message(MSG_INIT_END);
	printf("modem: 6: INIT_END sent: THE MODEM IS ONLINE\n");
	shared_say("online");
	pins_say("online");
	boot_log_say();

	if (!announce(7, "reads: what the modem says by itself")) {
		return;
	}
	t0 = now_ms();
	for (uint32_t word = rd(ipc, IPC_CP2AP_MSG), status = rd(ipc, IPC_CP2AP_STATUS); now_ms() - t0 < 3000;) {
		uint32_t w = rd(ipc, IPC_CP2AP_MSG), s = rd(ipc, IPC_CP2AP_STATUS), taken = akita_pcie0_msi_take();

		if (w != word || s != status) {
			printf("modem: 7: the modem's word 0x%x, status 0x%x (MSI 0x%x; %llu ms)\n", w, s, taken, now_ms() - t0);
			word = w;
			status = s;
		}
		frames_say(&FMT_RX);
		frames_say(&RAW_RX);
		sleep_ms(2);
	}
	shared_say("after listening");
	pins_say("after listening");

	if (!announce(8, "writes: requests to the modem (radio power, version, SIM, registration, operator, signal)")) {
		return;
	}
	{
		static const struct { uint16_t id; uint8_t len; uint8_t parameters[4]; } asks[] = {
			{ SIT_GET_RADIO_POWER, 0, { 0 } }, { SIT_GET_BASEBAND_VERSION, 1, { 0xff } }, { SIT_GET_SIM_STATUS, 0, { 0 } },
			{ SIT_GET_CS_REG_STATE, 0, { 0 } }, { SIT_GET_PS_REG_STATE, 0, { 0 } }, { SIT_GET_OPERATOR, 0, { 0 } },
			{ SIT_GET_SIGNAL_STRENGTH, 0, { 0 } },
		};
		unsigned next = 0;
		uint64_t next_at;
		clock_sec_t seconds = 0;
		clock_usec_t micro = 0;
		uint8_t when[6], info[128];
		uint64_t days, of_day, era, doe, yoe, doy, mp, year, month, day;

		/*
		 * As rild begins: the time (year - 1900, month from 0, day, hour, minute, second: UTC) and who we are -- four
		 * 32-byte names, Android's own for this phone (the model, a build, the device, the system's version). The modem's
		 * indications (SIM, radio ready, network, signal) start only after the second one's answer.
		 */
		clock_get_calendar_microtime(&seconds, &micro);
		days = seconds / 86400;
		of_day = seconds % 86400;
		days += 719468;                         /* days to a date (the civil calendar from a day count) */
		era = days / 146097;
		doe = days - era * 146097;
		yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
		doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
		mp = (5 * doy + 2) / 153;
		day = doy - (153 * mp + 2) / 5 + 1;
		month = mp < 10 ? mp + 3 : mp - 9;
		year = yoe + era * 400 + (month <= 2 ? 1 : 0);
		when[0] = (uint8_t)(year - 1900);
		when[1] = (uint8_t)(month - 1);
		when[2] = (uint8_t)day;
		when[3] = (uint8_t)(of_day / 3600);
		when[4] = (uint8_t)(of_day / 60 % 60);
		when[5] = (uint8_t)(of_day % 60);
		printf("modem: 8: the time told: %llu-%02llu-%02llu %02u:%02u:%02u UTC\n", year, month, day, when[3], when[4], when[5]);
		(void)sit_request(SIT_SET_AP_SYSTEM_TIME, when, sizeof(when));
		memset(info, 0, sizeof(info));
		strlcpy((char *)info, "Pixel 8a", 32);
		strlcpy((char *)info + 32, "CP1A.260505.005.A1", 32);
		strlcpy((char *)info + 64, "akita", 32);
		strlcpy((char *)info + 96, "16", 32);
		(void)sit_request(SIT_SET_DEVICE_INFO, info, sizeof(info));
		if (radio_on) {
			static const uint8_t on[6] = { 2, 0, 0, 0, 0, 0 };     /* u32 2: on; not for an emergency call */

			(void)sit_request(SIT_SET_RADIO_POWER, on, sizeof(on));
		}
		next_at = now_ms() + 1500;

		t0 = now_ms();
		for (uint32_t word = rd(ipc, IPC_CP2AP_MSG), status = rd(ipc, IPC_CP2AP_STATUS); now_ms() - t0 < 1000ull * listen_s;) {
			uint32_t w = rd(ipc, IPC_CP2AP_MSG), st = rd(ipc, IPC_CP2AP_STATUS), taken = akita_pcie0_msi_take();

			if (next < sizeof(asks) / sizeof(asks[0]) && now_ms() >= next_at) {
				(void)sit_request(asks[next].id, asks[next].parameters, asks[next].len);
				next++;
				next_at = now_ms() + 400;
			}
			if (w != word || st != status) {
				printf("modem: 8: the modem's word 0x%x, status 0x%x (MSI 0x%x; %llu ms)\n", w, st, taken, now_ms() - t0);
				word = w;
				status = st;
			}
			frames_say(&FMT_RX);
			raw_serve();
			sleep_ms(2);
		}
	}
	shared_say("after the requests");
	pins_say("after the requests");

	if (!announce(9, "stays: the modem's file service answered, what it says said")) {
		return;
	}
	if (use_irq) {
		interrupts_on();        /* (again after a restart: the power sequence makes the pins plain inputs) */
	}
	t0 = t_stay = t_said = now_ms();
	for (uint32_t word = rd(ipc, IPC_CP2AP_MSG), status = rd(ipc, IPC_CP2AP_STATUS), wake = (uint32_t)pin_get(&CP2AP_WAKEUP);;) {
		uint32_t w = rd(ipc, IPC_CP2AP_MSG), st = rd(ipc, IPC_CP2AP_STATUS), k = (uint32_t)pin_get(&CP2AP_WAKEUP);

		(void)akita_pcie0_msi_take();
		if (irq_msi_masked && link_up) {
			irq_msi_masked = 0;
			gic_v3_spi_unmask(INTID_MSI0);
		}
		if (irq_storm > 1) {
			printf("modem: 9: the interrupt of the modem's %s pin fired without end: masked, by the clock from here\n",
			    irq_storm == 2 ? "wake" : "active");
			irq_storm = 1;
		}
		if (w != word || st != status || k != wake) {
			printf("modem: 9: the modem's word 0x%x, status 0x%x, its wake pin %u, active %d\n", w, st, k, pin_get(&CP2AP_CP_ACTIVE));
			word = w;
			status = st;
			wake = k;
		}
		if (crash_after_s != 0 && now_ms() - t_stay >= 1000ull * crash_after_s) {
			crash_after_s = 0;
			printf("modem: 9: a test: the modem asked to stop (the dump-request pin high, as Android's forced crash)\n");
			pin_out(&AP2CP_DUMP_NOTI, 1, 0);
		}
		if (pin_get(&CP2AP_CP_ACTIVE) == 0) {
			/*
			 * Its "active" pin low after the start: the modem's firmware stopped (a crash, as Android's driver takes
			 * it). What it left is said; then it is started again from the power sequence, three times at most.
			 */
			printf("modem: 9: THE MODEM STOPPED (its active pin is low): its word 0x%x, status 0x%x\n", w, st);
			shared_say("stopped");
			pins_say("stopped");
			boot_log_say();
			cellular_up = cellular_registered = cellular_bars = cellular_dbm = 0;
			cellular_radio = cellular_sim = -1;
			if (link_up) {
				link_up = false;
				akita_pcie0_down();
			}
			doorbell_owed = false;
			rfs.size = 0;
			if (++restarts > 3) {
				printf("modem: 9: started again three times already: left off\n");
				return;
			}
			printf("modem: 9: starting it again (%u)\n", restarts);
			goto restart;
		}
		frames_say(&FMT_RX);
		raw_serve();
		if (link_sleep) {
			link_follow();
		}
		pin_send();
		if (ask_every_s != 0 && now_ms() - t0 >= 1000ull * ask_every_s) {
			t0 = now_ms();
			(void)sit_request(SIT_GET_RADIO_POWER, NULL, 0);        /* (a request now and then: the wake from our side) */
		}
		if (now_ms() - t_said >= 60000) {
			t_said = now_ms();
			printf("modem: 9: a minute: %u turns of this loop; interrupts: MSI %u, wake pin %u, active pin %u; link sleeps %u, wakes %u\n",
			    turns, irq_count[0], irq_count[1], irq_count[2], link_sleeps, link_wakes);
			turns = 0;
		}
		turns++;
		/* with the link up the queues are looked at every 10 ms (and its idle time counted); asleep: once a second */
		stay_wait(link_up || doorbell_owed ? 10 : 1000);
	}
}

static void
modem_thread(__unused void *a, __unused wait_result_t w)
{
	modem_run();
	thread_terminate(current_thread());
}

static void
modem_start(__unused thread_call_param_t a, __unused thread_call_param_t b)
{
	thread_t thread;

	if (kernel_thread_start(modem_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
}

static void
akita_modem_init(void)
{
	static thread_call_t call;
	uint32_t delay_s = 25;
	uint64_t deadline;

	if (!PE_parse_boot_argn("modem", &steps, sizeof(steps)) || steps == 0) {
		return;
	}
	PE_parse_boot_argn("modem_delay", &delay_s, sizeof(delay_s));
	PE_parse_boot_argn("modem_listen", &listen_s, sizeof(listen_s));
	PE_parse_boot_argn("modem_link_sleep", &link_sleep, sizeof(link_sleep));
	PE_parse_boot_argn("modem_radio", &radio_on, sizeof(radio_on));
	PE_parse_boot_argn("modem_crash_after", &crash_after_s, sizeof(crash_after_s));
	PE_parse_boot_argn("modem_irq", &use_irq, sizeof(use_irq));
	PE_parse_boot_argn("modem_ask_every", &ask_every_s, sizeof(ask_every_s));
	printf("modem: steps 1 to %u in %u s\n", steps, delay_s);
	call = thread_call_allocate(modem_start, NULL);
	clock_interval_to_deadline(delay_s, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(call, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_modem_init);
