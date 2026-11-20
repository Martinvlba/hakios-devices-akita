/*
 * The Pixel 8a's AoC ("always-on compute", Google's DSP subsystem on zuma): audio (the two CS35L41 speaker amplifiers'
 * data comes from its TDM ports) and the sensors (the IMU, proximity, light: CHRE) are behind it -- TODO 33 and 34.
 * Android's way to it (google-modules/aoc, aoc.c and aoc_v1.c; soc/gs drivers/soc/google/gsa): the AoC asked on
 * (aoc_req 0x154b0000: write 1, status at +0x40), its signed firmware (aoc.bin) copied into the AoC's DRAM carveout
 * (0x94000000, 48 MB), then the GSA -- Google's security co-processor, through its non-secure mailbox (gsa-ns,
 * 0x16490000) -- authenticates the image (GSA_MB_CMD_LOAD_AOC_FW_IMG, 90: the header's and the body's physical
 * addresses) and starts the AoC (GSA_MB_CMD_AOC_CMD, 91: GSA_AOC_START); the AoC then publishes its services in an IPC
 * block in its DRAM, reached through mailboxes.
 *
 * The GSA mailbox (gsa_mbox.c): 16 shared registers SR0-15 at +0x80; a request writes the command into SR0, the
 * argument count into SR1 and the arguments after; the response interrupt (bit 0) is cleared (INTCR0 +0x24) and
 * unmasked (INTMR0 +0x28), the request raised (INTGR1 +0x40, bit 0), and the GSA answers in SR0 (the command | bit 31),
 * SR1 (an error: 0 OK), SR2 (its argument count) and the arguments after, the response bit then set in INTSR0 (+0x2c).
 *
 *   aoc=1   a look: the AoC's power request and status read, the GSA mailbox's registers, and one harmless request
 *           (GSA_MB_CMD_GET_CHIP_ID, 3) -- whether this kernel can talk to the GSA at all. Nothing switched on,
 *           nothing written but the mailbox.
 *   aoc=2   ... and one word of the AoC's DRAM carveout (0x94000000, 48 MB; U-Boot's tree keeps it no-map, so XNU never
 *           has it): 6 MB in, where a running firmware's IPC block (aoc-ipc's struct aoc_control_block, magic
 *           0xA0C00A0C) is by default -- whether Android's AoC firmware still runs when this kernel starts.
 *
 *   aoc=3   THE AoC STARTED, as Android's aoc_fw_callback does with the GSA: its state asked first (only from 0,
 *           inactive); the image (fw/aoc.bin: Google's signed one, ../fw-aoc in the build) checked -- the 4 KB auth
 *           header (magic "AOC", generation 2: its image_config says where the GSA locks it, 0x94000000 + 28 MB) and the
 *           superbin after it (0xaabbccdd; image_size = the IPC block's offset) --, the body copied to the carveout's
 *           start, the header into a page of ours, GSA_MB_CMD_LOAD_AOC_FW_IMG (90: header lo/hi, body lo/hi), the
 *           firmware's parameters written at the IPC block (aoc_pass_fw_information: AOC_PARAMETER_MAGIC, the board,
 *           the carveout, the heaps at its top, the chip), GSA_AOC_START through Trusty (devices/akita/drivers/trusty,
 *           akita_trusty_aoc_cmd), and the IPC block's magic (0xA0C00A0C) awaited.
 *
 * Starting the AoC is the GSA's: the image's load and authentication through the mailbox (90), but GSA_AOC_START,
 * GET_STATE and the unload through Trusty (gsa_tz.c: a TIPC channel to the secure world's "hwmgr" service) --
 * devices/akita/drivers/trusty (11-11: it works; the AoC said INACTIVE).
 *
 * Found 11-11 (aoc=1): the AoC is ON when this kernel starts (aoc_req 1, state 1: left so by Android's boot or the
 * bootloader); the GSA answers (GET_CHIP_ID: error 1, invalid arguments -- the ROM's form wants a buffer).
 */
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <kern/thread.h>
#include <sys/hakios.audio.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <miscfs/devfs/devfs.h>
#include <kern/kalloc.h>
#include <sys/kauth.h>
#include <sys/vnode.h>
#include <sys/vnode_internal.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_map_address_t pmap_map(vm_map_address_t virt, vm_offset_t start, vm_offset_t end, vm_prot_t prot, unsigned int flags);
extern uint64_t kvtophys(vm_offset_t va);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
extern int akita_trusty_aoc_cmd(uint32_t cmd, uint32_t *state);
extern int akita_amp_speaker(int on) __attribute__((weak));       /* drivers/audio (amp=2): the CS35L41s */
extern int akita_amp_variant(int k) __attribute__((weak));        /* (their audio port set up the K-th way: a listening test) */
extern kern_return_t kernel_thread_start_priority(thread_continue_t continuation, void *parameter, integer_t priority,
    thread_t *new_thread);

#define AOC_REQ_PHYS            0x154b0000ULL   /* aoc_req: +0 the request, +0x40 the AoC's state */
#define AOC_DRAM_PHYS           0x94000000ULL   /* the AoC's carveout (aoc@94000000), 48 MB */
#define AOC_DRAM_SIZE           (48u << 20)
#define AOC_MAGIC               0xA0C00A0Cu
#define GSA_MBOX_PHYS           0x16490000ULL   /* gsa-ns */

#define MBOX_INTCR0             0x0024
#define MBOX_INTMR0             0x0028
#define MBOX_INTSR0             0x002c
#define MBOX_INTMSR0            0x0030
#define MBOX_INTGR1             0x0040
#define MBOX_SR(n)              (0x0080 + (n) * 4)
#define MBOX_SR_NUM             16

#define GSA_MB_CMD_RSP          (1u << 31)
#define GSA_MB_CMD_GET_CHIP_ID  3

static vm_offset_t aoc_req, gsa;
static uint32_t mode;

static inline uint32_t
rd(vm_offset_t base, uint32_t off)
{
	return *(volatile uint32_t *)(base + off);
}

static inline void
wr(vm_offset_t base, uint32_t off, uint32_t v)
{
	*(volatile uint32_t *)(base + off) = v;
}

/* one GSA request, polled (no interrupt): 0 and the response's words, or -1 when the GSA did not answer */
static int gsa_request_ms(uint32_t cmd, const uint32_t *args, uint32_t argc, uint32_t *rsp, uint32_t *rsp_n, uint32_t *err,
    int timeout_ms);

static int
gsa_request(uint32_t cmd, const uint32_t *args, uint32_t argc, uint32_t *rsp, uint32_t *rsp_n, uint32_t *err)
{
	return gsa_request_ms(cmd, args, argc, rsp, rsp_n, err, 500);
}

static int
gsa_request_ms(uint32_t cmd, const uint32_t *args, uint32_t argc, uint32_t *rsp, uint32_t *rsp_n, uint32_t *err,
    int timeout_ms)
{
	if (argc + 2 > MBOX_SR_NUM) {
		return -1;
	}
	wr(gsa, MBOX_SR(0), cmd);
	wr(gsa, MBOX_SR(1), argc);
	for (uint32_t i = 0; i < argc; i++) {
		wr(gsa, MBOX_SR(i + 2), args[i]);
	}
	wr(gsa, MBOX_INTCR0, 1u);
	wr(gsa, MBOX_INTMR0, rd(gsa, MBOX_INTMR0) & ~1u);
	__builtin_arm_dsb(0xf);
	wr(gsa, MBOX_INTGR1, 1u);
	int answered = 0;
	for (int ms = 0; ms < timeout_ms; ms++) {
		if (rd(gsa, MBOX_INTSR0) & 1u) {
			answered = 1;
			break;
		}
		delay(1000);
	}
	uint32_t got = rd(gsa, MBOX_SR(0));
	*err = rd(gsa, MBOX_SR(1));
	uint32_t n = rd(gsa, MBOX_SR(2));
	*rsp_n = 0;
	if (answered && got == (cmd | GSA_MB_CMD_RSP) && n + 3 <= MBOX_SR_NUM) {
		for (uint32_t i = 0; i < n; i++) {
			rsp[i] = rd(gsa, MBOX_SR(i + 3));
		}
		*rsp_n = n;
	}
	wr(gsa, MBOX_INTCR0, 1u);
	wr(gsa, MBOX_INTMR0, rd(gsa, MBOX_INTMR0) | 1u);
	printf("akita aoc: GSA request %u: %s (SR0 0x%x, error %u, %u words)\n", cmd, answered ? "answered" : "NO ANSWER",
	    got, *err, n);
	return answered && got == (cmd | GSA_MB_CMD_RSP) ? 0 : -1;
}

/*
 * The carveout mapped once, all 48 MB (write-combined), with kernel addresses of its own and pmap_map: not ml_io_map,
 * whose whole area is 8 MB (io_map.c's IO_MAP_SIZE) -- a 48 MB ml_io_map, then the second of 1 MB megabyte pieces, each
 * killed the kernel (11-11); hakios.akita_display.c maps its buffers so for the same reason
 */
static vm_offset_t
carveout_map(void)
{
	static vm_offset_t va;

	if (va == 0) {
		vm_offset_t v = 0;

		if (kmem_alloc(kernel_map, &v, AOC_DRAM_SIZE, KMA_VAONLY | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
			return 0;
		}
		pmap_map(v, (vm_offset_t)AOC_DRAM_PHYS, (vm_offset_t)(AOC_DRAM_PHYS + AOC_DRAM_SIZE), VM_PROT_READ | VM_PROT_WRITE,
		    VM_WIMG_WCOMB);
		va = v;
	}
	return va;
}

#define GSA_MB_CMD_LOAD_AOC_FW_IMG      90
#define AOC_AUTH_HEADER_SIZE            4096u
#define AOC_PARAMETER_MAGIC             0x0a0cda7au
#define AOC_BINARY_DRAM_BASE            0x98000000u     /* the carveout as the AoC sees it */
#define CHIPID_PHYS                     0x10000000ULL

/* ---- the AoC's IPC: services, their queues, the doorbells (aoc-ipc's aoc_ipc_core.c, aoc's mailbox-wc.c) ---------- */

static vm_offset_t ipc_base;                    /* the IPC block (carveout + the image's size), mapped */
static uint32_t ipc_room;                       /* the bytes mapped from ipc_base on (the carveout's rest) */
static vm_offset_t aoc_mbox[3];                 /* the AP->AoC doorbells: mbox@15200000/15220000/15240000, 16 each */
#define AOC_UP                          0       /* the AoC to us */
#define AOC_DOWN                        1       /* us to the AoC */
#define MB_INTGR0                       0x20
#define MB_INTSR0                       0x2c

/* service NAME's header (struct aoc_ipc_service_header: name[32], flags, two regions of offset, size, slots, tx, rx,
 * wp, rp), or 0 */
static vm_offset_t
aoc_service(const char *name)
{
	uint32_t count = rd(ipc_base, 24), each = rd(ipc_base, 28), at = rd(ipc_base, 32);

	for (uint32_t k = 0; k < count && each >= 92; k++) {
		vm_offset_t h = ipc_base + at + k * each;
		char nm[33];

		for (int c = 0; c < 32; c++) {
			nm[c] = (char)*(volatile uint8_t *)(h + c);
		}
		nm[32] = 0;
		if (strcmp(nm, name) == 0) {
			return h;
		}
	}
	return 0;
}

static inline vm_offset_t
region(vm_offset_t svc, int dir)
{
	return svc + 36 + 28 * dir;     /* offset 0, size 4, slots 8, tx 12, rx 16, wp 20, rp 24 */
}

/* the AoC rung for SVC's channel (flags bits 19-23: the mailbox interrupt, 16 a mailbox) */
static void
aoc_signal(vm_offset_t svc)
{
	uint32_t irq = (rd(svc, 32) >> 19) & 0x1f;
	vm_offset_t mb = aoc_mbox[irq / 16];

	if (!mb) {
		return;
	}
	for (int i = 0; i < 100 && (rd(mb, MB_INTSR0) & (1u << (irq % 16))); i++) {
		delay(100);                     /* (the AoC not done with the last one yet) */
	}
	wr(mb, MB_INTGR0, 1u << (irq % 16));
}

/* a message onto SVC's queue towards the AoC: false when it is full or too big */
static bool
aoc_write(vm_offset_t svc, const void *msg, uint32_t len)
{
	vm_offset_t r = region(svc, AOC_DOWN);
	uint32_t off = rd(r, 0), size = rd(r, 4), slots = rd(r, 8), tx = rd(r, 12), rx = rd(r, 16);

	if (slots == 0 || len + 8 > size || tx - rx >= slots) {
		return false;
	}
	vm_offset_t slot = ipc_base + off + size * (tx % slots);
	*(volatile uint32_t *)slot = len;                       /* struct aoc_ipc_message_header: length, 3 reserved */
	*(volatile uint32_t *)(slot + 4) = 0;
	for (uint32_t i = 0; i < len; i++) {
		*(volatile uint8_t *)(slot + 8 + i) = ((const uint8_t *)msg)[i];
	}
	__builtin_arm_dsb(0xf);
	wr(r, 12, tx + 1);
	wr(r, 20, (rd(r, 20) + 1) % size);
	__builtin_arm_dsb(0xf);
	aoc_signal(svc);
	return true;
}

/* the next message from the AoC on SVC (polled, up to MS ms): its length, or -1 */
static int
aoc_read(vm_offset_t svc, void *buf, uint32_t cap, int ms)
{
	vm_offset_t r = region(svc, AOC_UP);

	for (int i = 0; i <= ms * 10; i++) {
		uint32_t off = rd(r, 0), size = rd(r, 4), slots = rd(r, 8), tx = rd(r, 12), rx = rd(r, 16);

		if (slots != 0 && tx != rx) {
			vm_offset_t slot = ipc_base + off + size * (rx % slots);
			uint32_t len = rd(slot, 0) & 0xffff;
			uint32_t n = len < cap ? len : cap;

			for (uint32_t k = 0; k < n; k++) {
				((uint8_t *)buf)[k] = *(volatile uint8_t *)(slot + 8 + k);
			}
			wr(r, 16, rx + 1);
			wr(r, 24, (rd(r, 24) + 1) % size);
			return (int)len;
		}
		delay(100);
	}
	return -1;
}

static void
aoc_say_bytes(const char *what, const uint8_t *b, int n)
{
	char text[3 * 64 + 1];
	unsigned at = 0;

	text[0] = 0;
	for (int i = 0; i < n && i < 64; i++) {
		at += (unsigned)snprintf(text + at, sizeof(text) - at, "%02x ", b[i]);
	}
	printf("akita aoc: bt: %s: %d bytes: %s\n", what, n, text);
}

/*
 * What Android's Bluetooth HAL says to the AoC before it switches the chip's power on (its file I/O traced on this
 * phone, 11-15: /dev/acd-chre_bt_offload_ctl and _data_tx/_data_rx, in this order, then rfkill on, then the UART) --
 * without it the chip stays silent under BT_REG_ON. Replayed as it was, the answers said. True when all three were
 * answered.
 */
bool hakios_aoc_bt_prepare(void);
bool
hakios_aoc_bt_prepare(void)
{
	static const uint8_t hello[48] = { 0 };
	static const uint8_t setup[48] = { 0x02, 0x11, 0x00, 0x00, 0x00, 0x01, 0x02 };
	static const uint8_t request[64] = {
		0x08, 0x58, 0x02, 0x72, 0xcc, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x01, 0x27, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x07, 0x00, 0x08,
		0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x08, 0x00, 0x00, 0x00, 0x04, 0x00, 0x04,
		0x00, 0x04, 0x00, 0x00, 0x00,
	};
	static const char *const names[] = { "chre_bt_offload_ctl", "chre_bt_offload_data_tx", "chre_bt_offload_data_rx" };
	vm_offset_t svc[3];
	uint8_t answer[128];
	int n, good = 0;

	if (!ipc_base) {
		printf("akita aoc: bt: the AoC is not up\n");
		return false;
	}
	for (int i = 0; i < 3; i++) {
		svc[i] = aoc_service(names[i]);
		if (!svc[i]) {
			printf("akita aoc: bt: no service %s\n", names[i]);
			return false;
		}
		printf("akita aoc: bt: %s: flags %#x; up: size %u slots %u tx %u rx %u; down: size %u slots %u tx %u rx %u\n", names[i],
		    rd(svc[i], 32), rd(region(svc[i], AOC_UP), 4), rd(region(svc[i], AOC_UP), 8), rd(region(svc[i], AOC_UP), 12),
		    rd(region(svc[i], AOC_UP), 16), rd(region(svc[i], AOC_DOWN), 4), rd(region(svc[i], AOC_DOWN), 8),
		    rd(region(svc[i], AOC_DOWN), 12), rd(region(svc[i], AOC_DOWN), 16));
	}
	while ((n = aoc_read(svc[0], answer, sizeof(answer), 0)) >= 0) {
		aoc_say_bytes("ctl, waiting there", answer, n);
	}
	if (!aoc_write(svc[0], hello, sizeof(hello))) {
		printf("akita aoc: bt: ctl takes no message\n");
		return false;
	}
	n = aoc_read(svc[0], answer, sizeof(answer), 500);
	aoc_say_bytes("ctl, to the first (Android: 01 00 ...)", answer, n < 0 ? 0 : n);
	good += n > 0 && answer[0] == 0x01;
	(void)aoc_write(svc[0], setup, sizeof(setup));
	n = aoc_read(svc[0], answer, sizeof(answer), 500);
	aoc_say_bytes("ctl, to the second (Android: 03 00 00 04 00 00 00 04 00 1e 00 00 00 01 05)", answer, n < 0 ? 0 : n);
	good += n > 0 && answer[0] == 0x03;
	/* the data services are rings of bytes (size 262144, one slot): the request's 64 bytes onto ours, the answer off the
	 * AoC's (tx - rx bytes from rp on) */
	{
		vm_offset_t down = region(svc[1], AOC_DOWN), up = region(svc[2], AOC_UP);
		uint32_t size = rd(down, 4), wp = rd(down, 20);

		for (uint32_t i = 0; i < sizeof(request); i++) {
			*(volatile uint8_t *)(ipc_base + rd(down, 0) + (wp + i) % size) = request[i];
		}
		__builtin_arm_dsb(0xf);
		wr(down, 12, rd(down, 12) + (uint32_t)sizeof(request));
		wr(down, 20, (wp + (uint32_t)sizeof(request)) % size);
		__builtin_arm_dsb(0xf);
		aoc_signal(svc[1]);
		n = -1;
		for (int ms = 0; ms < 500; ms++) {
			uint32_t have = rd(up, 12) - rd(up, 16), usize = rd(up, 4), rp = rd(up, 24);

			if (have != 0 && usize != 0) {
				delay(2000);                    /* (the rest of it) */
				have = rd(up, 12) - rd(up, 16);
				n = have < sizeof(answer) ? (int)have : (int)sizeof(answer);
				for (int i = 0; i < n; i++) {
					answer[i] = *(volatile uint8_t *)(ipc_base + rd(up, 0) + (rp + (uint32_t)i) % usize);
				}
				wr(up, 16, rd(up, 16) + have);
				wr(up, 24, (rp + have) % usize);
				break;
			}
			delay(1000);
		}
	}
	aoc_say_bytes("data_rx (Android: 32 bytes, 2f b8 26 21 05 ...)", answer, n < 0 ? 0 : n);
	good += n > 0;
	return good == 3;
}

/* an audio command round trip on SERVICE (aoc_alsa_hw.c's aoc_audio_control): old replies dropped, the command, its reply */
static int
aoc_audio_cmd(const char *service, void *cmd, uint32_t len)
{
	static uint8_t cntr;
	vm_offset_t svc = aoc_service(service);
	uint8_t junk[64];

	if (!svc) {
		printf("akita aoc: no service %s\n", service);
		return -1;
	}
	while (aoc_read(svc, junk, sizeof(junk), 0) >= 0) {
	}
	uint8_t *c = cmd;
	c[0] = 0;                                       /* struct CONTAINER_HDR: DATA_TYPE_CMD, a counter, the length */
	c[1] = cntr++;
	*(uint16_t *)(c + 2) = (uint16_t)len;
	if (!aoc_write(svc, cmd, len)) {
		printf("akita aoc: %s: its queue is full\n", service);
		return -1;
	}
	uint16_t id = *(uint16_t *)(c + 4);
	uint8_t reply[128];
	int got = aoc_read(svc, reply, sizeof(reply), 1000);
	if (got < 0) {
		printf("akita aoc: %s: no reply to command %u in 1 s\n", service, id);
	} else if (got == 4) {                          /* (aoc_audio_control: a 4-byte reply is an error code) */
		printf("akita aoc: %s: command %u: error 0x%x\n", service, id, *(uint32_t *)reply);
		return -1;
	} else {
		memcpy(cmd, reply, (uint32_t)got < len ? (uint32_t)got : len);
	}
	return got;
}

/* ---- a sound through the AoC (aocplay=SECONDS with aoc=3): Google's ALSA driver's playback, by hand ------------------
 * prepare: CMD_AUDIO_OUTPUT_EP_SETUP (206) and EP_SETUP2 (298) for entry point 0 (audio_playback0); the route:
 * CMD_AUDIO_OUTPUT_BIND2 (271) to SINK_SPEAKER (TDM_0_RX: the two CS35L41s); start: CMD_AUDIO_OUTPUT_SOURCE (201) on; then
 * 48 kHz 16-bit stereo into audio_playback0's ring (a ring service: bytes at wp, tx counts them; the AoC moves rx). */

static uint32_t aocplay;
static uint32_t aocplay_amp = 2000;             /* aocplay_amp=N: the tone's peak (2000: about -24 dBFS; 32767 full scale) */
static uint32_t aocplay_lr;                     /* aocplay_lr=1: the tone one second left only, one second right only, by turns */
static uint32_t aocsink = ~0u;                  /* aocsink=M: the speaker sink's processing mode before each stream (CMD_AUDIO_OUTPUT_SINK2:
                                                 * 0 idle, 1 active, 2 bypass -- Waves' chain left out); unset: as the AoC has it */
static uint32_t aocep = 1;                      /* aocep=N: the entry point played into (aocplay and Darwin's sound), audio_playbackN.
                                                 * 1: Android's low-latency playback (EP2), the AoC's "Primary" source -- what
                                                 * sounds (11-12). 0: Android's mmap/AAudio one, "UltraLL": the speaker mixer
                                                 * takes it but never gives the speaker anything ("Input 0 RB overflow") */
static uint32_t aocplay_matrix;                 /* aocplay_matrix=1: a listening test -- six seconds for each amplifier setting
                                                 * K (akita_amp_variant), K beeps twice in them; aocplay=38 for all six */
#define MATRIX_SEG      (6u * 48000u)           /* frames for each setting */
#define MATRIX_N        6u

/* the tone's on/off in the listening test: K beeps of 0.2 s (0.2 s apart) from 0.5 s and again from 3.5 s into setting K's
 * six seconds; always on otherwise */
static bool
aoc_tone_on(uint64_t frame)
{
	if (!aocplay_matrix) {
		return true;
	}
	uint32_t k = (uint32_t)(frame / MATRIX_SEG) + 1, f = (uint32_t)(frame % MATRIX_SEG);
	if (k > MATRIX_N) {
		return false;
	}
	for (uint32_t g = 0; g < 2; g++) {
		uint32_t start = 24000 + g * 144000;
		if (f >= start && f < start + k * 19200) {
			return (f - start) % 19200 < 9600;
		}
	}
	return false;
}

static bool
aoc_out_cmd(const char *what, void *cmd, uint32_t len, uint16_t id)
{
	*(uint16_t *)((uint8_t *)cmd + 4) = id;
	int got = aoc_audio_cmd("audio_output_control", cmd, len);
	printf("akita aoc: %s (%u): %s, reply field %d\n", what, id, got < 0 ? "FAILED" : "done", got < 0 ? 0 :
	    *(int16_t *)((uint8_t *)cmd + 6));
	return got >= 0;
}

/*
 * The entry point's volume at a sink (aoc_alsa_hw.c's aoc_audio_volume_set: CMD_AUDIO_OUTPUT_SET_PARAMETER 209, packed --
 * the header, block 16 + sink, component 0, key the source, value 0-1000). Google's driver sends chip->volume, 1000 from
 * its start, for every playback stream at prepare (aoc_audio_set_ctls); this driver never sent one until 11-12 -- the
 * AoC's own default may be silence, which is what the speakers gave with everything else in order (TODO 33: sink
 * processing active, DSP in playback, the amps up). aocvol=N: N instead of 1000. UNTESTED by ear (11-12 night, no sound
 * without the tester).
 */
static bool
aoc_volume(uint8_t source, uint8_t sink)
{
	uint32_t value = 1000, key = source;
	uint8_t p[18] = { 0 };

	PE_parse_boot_argn("aocvol", &value, sizeof(value));
	if (value > 1000) {
		value = 1000;
	}
	p[8] = (uint8_t)(16 + sink);                    /* AOC_AUDIO_SINK_BLOCK_ID_BASE + sink */
	p[9] = 0;                                       /* component */
	memcpy(p + 10, &key, 4);
	memcpy(p + 14, &value, 4);
	printf("akita aoc: volume %u of 1000 for entry point %u at sink %u\n", value, source, sink);
	return aoc_out_cmd("SET_PARAMETER (volume)", p, sizeof(p), 209);
}

/* what the AoC says of its output: the DSP's mode, the speaker sink's processing state, each sink's channel bitmap */
static void
aoc_out_state(void)
{
	uint8_t dsp[9] = { 0 }, sink[10] = { 0 }, maps[18] = { 0 };

	sink[8] = 0;                                    /* ASNK_SPEAKER */
	if (aoc_out_cmd("GET_DSP_STATE", dsp, sizeof(dsp), 233) && aoc_out_cmd("GET_SINK_PROCESSING_STATE", sink, sizeof(sink), 234) &&
	    aoc_out_cmd("GET_SINKS_BITMAPS", maps, sizeof(maps), 245)) {
		printf("akita aoc: output: DSP mode %u (0 idle, 1 playback, 2 voice), speaker sink processing %u (0 idle, 1 active, 2 "
		    "bypass), channel bitmaps speaker 0x%x headphone 0x%x bt 0x%x modem 0x%x usb 0x%x\n", dsp[8], sink[9],
		    *(uint16_t *)(maps + 8), *(uint16_t *)(maps + 10), *(uint16_t *)(maps + 12), *(uint16_t *)(maps + 14),
		    *(uint16_t *)(maps + 16));
	}
}

/*
 * The test tone: aocplay_hz=N (default 440) from a 257-entry quarter sine, linearly interpolated, a 32-bit phase (11-12:
 * the first one stepped through 64 phases without interpolation -- clean harmonics but spurs at 7.9, 8.8, 12 kHz, -36 dB,
 * which the speakers made louder than the tone itself), peak aocplay_amp (of 32767; 2000 about -24 dBFS); aocplay_lr=1:
 * odd seconds left only, even seconds right only; aocplay_sweep=1: 1.5 s each at -30, -24, -18, -12, -9 and -6 dBFS
 * (a distortion test against level, with aocmic=S recording it), then silence.
 */
static const int16_t sine_q[257] = {
	0, 201, 402, 603, 804, 1005, 1206, 1407, 1608, 1809, 2009, 2210, 2410, 2611, 2811, 3012,
	3212, 3412, 3612, 3811, 4011, 4210, 4410, 4609, 4808, 5007, 5205, 5404, 5602, 5800, 5998, 6195,
	6393, 6590, 6786, 6983, 7179, 7375, 7571, 7767, 7962, 8157, 8351, 8545, 8739, 8933, 9126, 9319,
	9512, 9704, 9896, 10087, 10278, 10469, 10659, 10849, 11039, 11228, 11417, 11605, 11793, 11980, 12167, 12353,
	12539, 12725, 12910, 13094, 13279, 13462, 13645, 13828, 14010, 14191, 14372, 14553, 14732, 14912, 15090, 15269,
	15446, 15623, 15800, 15976, 16151, 16325, 16499, 16673, 16846, 17018, 17189, 17360, 17530, 17700, 17869, 18037,
	18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357, 19519, 19680, 19841, 20000, 20159, 20317, 20475, 20631,
	20787, 20942, 21096, 21250, 21403, 21554, 21705, 21856, 22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027,
	23170, 23311, 23452, 23592, 23731, 23870, 24007, 24143, 24279, 24413, 24547, 24680, 24811, 24942, 25072, 25201,
	25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198, 26319, 26438, 26556, 26674, 26790, 26905, 27019, 27133,
	27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001, 28105, 28208, 28310, 28411, 28510, 28609, 28706, 28803,
	28898, 28992, 29085, 29177, 29268, 29358, 29447, 29534, 29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195,
	30273, 30349, 30424, 30498, 30571, 30643, 30714, 30783, 30852, 30919, 30985, 31050, 31113, 31176, 31237, 31297,
	31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736, 31785, 31833, 31880, 31926, 31971, 32014, 32057, 32098,
	32137, 32176, 32213, 32250, 32285, 32318, 32351, 32382, 32412, 32441, 32469, 32495, 32521, 32545, 32567, 32589,
	32609, 32628, 32646, 32663, 32678, 32692, 32705, 32717, 32728, 32737, 32745, 32752, 32757, 32761, 32765, 32766,
	32767,
};

static uint32_t aocplay_hz = 440, aocplay_sweep;
#define SWEEP_STEP      72000u                  /* frames at each level */
static const uint16_t sweep_peak[6] = { 1036, 2067, 4125, 8231, 11627, 16423 };      /* -30 -24 -18 -12 -9 -6 dBFS */

static int32_t
tone_sample(uint64_t frame)
{
	/* (aocplay_sweep=2: the six levels at aocplay_hz, then the six again at 440 Hz) */
	uint32_t hz = (aocplay_sweep == 2 && frame / SWEEP_STEP >= 6) ? 440 : aocplay_hz;
	uint32_t inc = (uint32_t)(((uint64_t)hz << 32) / 48000u);
	uint32_t phase = (uint32_t)(frame * (uint64_t)inc), u = phase & 0x3fffffffu, q = phase >> 30;

	if (q & 1) {
		u = 0x40000000u - u;
	}
	uint32_t idx = u >> 22, frac = (u >> 6) & 0xffffu;
	int32_t a = sine_q[idx], b = sine_q[idx < 256 ? idx + 1 : 256];
	int32_t v = a + (int32_t)(((int64_t)(b - a) * frac) >> 16);
	return q >= 2 ? -v : v;
}

/* the tone, frame FRAME onwards, BYTES of it at ring offset AT (wrapping): 16-bit stereo */
static void
aoc_tone(vm_offset_t ring, uint32_t size, uint32_t at, uint32_t bytes, uint64_t *frame)
{
	for (uint32_t i = 0; i + 4 <= bytes; i += 4) {
		uint32_t peak = aocplay_amp;
		if (aocplay_sweep) {
			uint64_t step = *frame / SWEEP_STEP;
			if (aocplay_sweep == 2 && step >= 6 && step < 12) {
				step -= 6;
			}
			peak = step < 6 ? sweep_peak[step] : 0;
		}
		int32_t v = aoc_tone_on(*frame) ? (int32_t)(((int64_t)tone_sample(*frame) * peak) / 32767) : 0;
		uint16_t l = (uint16_t)(int16_t)v, r = l;
		if (aocplay_lr) {
			if ((*frame / 48000) % 2 == 0) {
				r = 0;
			} else {
				l = 0;
			}
		}
		uint32_t o = (at + i) % size;
		*(volatile uint32_t *)(ring + o) = l | ((uint32_t)r << 16);
		(*frame)++;
	}
}

/*
 * aoclog=1: what the AoC wrote on service NAME towards us (its "logging" -- Android's aocd puts it in logcat -- and
 * "debug"), as text lines (11-12: why its speaker output is silent). A ring: the last 4 KB; a queue: each message waiting.
 */
static uint32_t aoclog;

static void
aoc_text(const char *name, const uint8_t *b, uint32_t n, uint32_t *lines)
{
	char line[161];
	uint32_t at = 0;

	for (uint32_t i = 0; i <= n && *lines < 400; i++) {
		uint8_t c = i < n ? b[i] : '\n';
		if (c == '\n' || c == 0 || at == 160) {
			if (at > 0) {
				line[at] = 0;
				printf("akita aoc: [%s] %s\n", name, line);
				(*lines)++;
			}
			at = 0;
			if (c == '\n' || c == 0) {
				continue;
			}
		}
		line[at++] = (c >= 32 && c < 127) ? (char)c : '.';
	}
}

static void
aoc_dump_service(const char *name)
{
	vm_offset_t svc = aoc_service(name);
	static uint8_t buf[4096];
	uint32_t lines = 0;

	if (!svc) {
		printf("akita aoc: [%s] no such service\n", name);
		return;
	}
	vm_offset_t r = region(svc, AOC_UP);
	uint32_t type = (rd(svc, 32) >> 16) & 7, off = rd(r, 0), size = rd(r, 4), slots = rd(r, 8), tx = rd(r, 12),
	    rx = rd(r, 16);
	printf("akita aoc: [%s] type %u, towards us: %u bytes at +0x%x, slots %u, tx %u rx %u wp %u rp %u\n", name, type, size,
	    off, slots, tx, rx, rd(r, 20), rd(r, 24));
	if (type == 1 && size != 0) {                   /* a ring: bytes; tx counts them */
		uint32_t n = tx < sizeof(buf) ? tx : sizeof(buf);
		n = n < size ? n : size;
		for (uint32_t i = 0; i < n; i++) {
			buf[i] = *(volatile uint8_t *)(ipc_base + off + (tx - n + i) % size);
		}
		aoc_text(name, buf, n, &lines);
	} else if (slots != 0 && size > 8) {            /* a queue: messages with an 8-byte header */
		for (uint32_t k = (tx - rx > slots ? tx - slots : rx); k != tx && lines < 400; k++) {
			vm_offset_t slot = ipc_base + off + size * (k % slots);
			uint32_t len = rd(slot, 0) & 0xffff;
			len = len < size - 8 ? len : size - 8;
			len = len < sizeof(buf) ? len : sizeof(buf);
			for (uint32_t i = 0; i < len; i++) {
				buf[i] = *(volatile uint8_t *)(slot + 8 + i);
			}
			aoc_text(name, buf, len, &lines);
		}
	}
	printf("akita aoc: [%s] %u lines\n", name, lines);
}

/* for the Bluetooth probe: what the AoC wrote on service NAME, into the log */
void hakios_aoc_dump(const char *name);
void
hakios_aoc_dump(const char *name)
{
	if (ipc_base) {
		aoc_dump_service(name);
	}
}

/* aoclog=1: the AoC's "logging" queue drained every 20 ms into the kernel log from its start on (what Android's aocd puts
 * in logcat: plain text, one message a slot; a core's "F1: " prefix comes as a message of its own) -- left alone it fills
 * at 256 messages and the AoC says nothing more */
static void
aoc_log_thread(__unused void *arg, __unused wait_result_t wr_)
{
	vm_offset_t svc = aoc_service("logging");
	static uint8_t buf[512];
	char prefix[8] = "";
	uint32_t lines = 0;

	if (!svc) {
		return;
	}
	vm_offset_t r = region(svc, AOC_UP);
	for (;;) {
		uint32_t off = rd(r, 0), size = rd(r, 4), slots = rd(r, 8), tx = rd(r, 12), rx = rd(r, 16);

		if (slots == 0 || size <= 8 || tx == rx) {
			delay(20000);
			continue;
		}
		vm_offset_t slot = ipc_base + off + size * (rx % slots);
		uint32_t len = rd(slot, 0) & 0xffff;
		len = len < size - 8 ? len : size - 8;
		len = len < sizeof(buf) - 1 ? len : sizeof(buf) - 1;
		for (uint32_t i = 0; i < len; i++) {
			uint8_t c = *(volatile uint8_t *)(slot + 8 + i);
			buf[i] = (c >= 32 && c < 127) ? c : (c == 0 ? 0 : ' ');
		}
		buf[len] = 0;
		wr(r, 16, rx + 1);
		wr(r, 24, (rd(r, 24) + 1) % size);
		uint32_t n = (uint32_t)strlen((const char *)buf);
		while (n > 0 && buf[n - 1] == ' ') {
			buf[--n] = 0;
		}
		if (n == 0) {
			continue;
		}
		if (n <= 3 && buf[n - 1] == ':') {      /* "F1:", "H0:", "A3:" -- the core of the next message */
			strlcpy(prefix, (const char *)buf, sizeof(prefix));
			continue;
		}
		printf("akita aoc: [log %u] %s%s%s\n", ++lines, prefix, prefix[0] ? " " : "", buf);
		prefix[0] = 0;
	}
}

/*
 * aocasp=M: the speaker mixer's post-processing (Google's "AoC Speaker Mixer ASP Mode": SET_PARAMETER block 16, component
 * 2, key 0 -- 0 off, 1 on, 2 bypass, 3 ground). 11-12, the AoC's own log: the speaker mixer AMixSPKR (core H0, "PostProcessing
 * Config: WAVES") takes our samples but gives the speaker's TDM nothing ("HF overrun. RP not 384 bytes behind WP", then
 * "Input 0 RB overflow"): Waves, which Android's HAL sets up with its tunings, never set up here.
 */
static uint32_t aocasp = ~0u;

static void
aoc_speaker_asp(void)
{
	uint8_t g[18] = { 0 }, p[18] = { 0 };
	uint32_t key = 0, val = 0;

	if (aocasp > 3) {
		return;
	}
	g[8] = 16;
	g[9] = 2;
	memcpy(g + 10, &key, 4);
	if (aoc_out_cmd("GET_PARAMETER (speaker mixer ASP mode)", g, sizeof(g), 211)) {
		memcpy(&val, g + 14, 4);
		printf("akita aoc: the speaker mixer's ASP mode: %u (0 off, 1 on, 2 bypass, 3 ground), as found\n", val);
	}
	p[8] = 16;
	p[9] = 2;
	memcpy(p + 10, &key, 4);
	memcpy(p + 14, &aocasp, 4);
	aoc_out_cmd(aocasp == 2 ? "SET_PARAMETER (speaker mixer ASP BYPASS)" : "SET_PARAMETER (speaker mixer ASP mode)", p, sizeof(p),
	    209);
	memset(g + 14, 0, 4);
	if (aoc_out_cmd("GET_PARAMETER (speaker mixer ASP mode)", g, sizeof(g), 211)) {
		memcpy(&val, g + 14, 4);
		printf("akita aoc: the speaker mixer's ASP mode: %u, after ours (%u)\n", val, aocasp);
	}
}

/* the speaker sink's processing mode (aocsink=M), before a stream's EP_SETUP: Google's "Audio Sink 0 Processing State" */
static void
aoc_sink_mode(void)
{
	aoc_speaker_asp();
	if (aocsink <= 2) {
		uint8_t sk[11] = { 0 };                 /* struct CMD_AUDIO_OUTPUT_SINK2: sink, mode, timestamping */
		sk[8] = 0;                              /* SINK_SPEAKER */
		sk[9] = (uint8_t)aocsink;
		aoc_out_cmd(aocsink == 2 ? "SINK2 speaker BYPASS" : aocsink == 1 ? "SINK2 speaker active" : "SINK2 speaker idle", sk,
		    sizeof(sk), 270);
	}
}

static void
aoc_play(void)
{
	char svcname[24];

	snprintf(svcname, sizeof(svcname), "audio_playback%u", aocep);
	vm_offset_t svc = aoc_service(svcname);

	if (!svc) {
		printf("akita aoc: no %s\n", svcname);
		return;
	}
	vm_offset_t r = region(svc, AOC_DOWN);
	uint32_t size = rd(r, 4), off = rd(r, 0);
	printf("akita aoc: %s: flags 0x%x (type %u, mailbox %u), down: %u bytes at +0x%x, slots %u, tx %u rx %u wp %u rp %u\n",
	    svcname, rd(svc, 32), (rd(svc, 32) >> 16) & 7, (rd(svc, 32) >> 19) & 0x1f, size, off, rd(r, 8), rd(r, 12), rd(r, 16),
	    rd(r, 20), rd(r, 24));
	if (size < 7680 || ((rd(svc, 32) >> 16) & 7) != 1 || (uint64_t)off + size > ipc_room) {   /* (15360 here: four 20 ms periods) */
		printf("akita aoc: %s is not the ring expected; no sound\n", svcname);
		return;
	}
	uint32_t period = 3840, buffer = (size / period) * period;  /* 20 ms periods */

	/* the ring's write pointer to its start (aoc_ring_reset_write_pointer) */
	if (rd(r, 20) != 0) {
		uint32_t n = size - rd(r, 20);
		wr(r, 12, rd(r, 12) + n);
		wr(r, 20, 0);
	}

	uint8_t ep[28] = { 0 };                         /* struct CMD_AUDIO_OUTPUT_EP_SETUP (akita's Android 16 header: the
	                                                 * metadata is 5 bytes -- chan, bits, sr, format, offset; an older
	                                                 * header's 4 set up entry point 1 in float, 11-11) */
	*(uint32_t *)(ep + 8) = 0;                      /* address */
	*(uint32_t *)(ep + 12) = 48000;                 /* watermark: PLAYBACK_WATERMARK_DEFAULT */
	*(uint32_t *)(ep + 16) = 0;                     /* length */
	ep[20] = 2;                                     /* CH_STEREO */
	ep[21] = 1;                                     /* WIDTH_16_BIT */
	ep[22] = 5;                                     /* SR_48KHZ */
	ep[23] = 1;                                     /* FRMT_FIXED_POINT */
	ep[24] = 0;                                     /* offset */
	ep[25] = (uint8_t)aocep;                        /* channel: the entry point (aocep=N; 0 Android's mmap/ULL one) */
	ep[26] = 1;                                     /* wraparound */
	ep[27] = 0;                                     /* ENTRYPOINT_MODE_PLAYBACK */
	uint8_t ep2[18] = { 0 };                        /* struct CMD_AUDIO_OUTPUT_EP_SETUP2 */
	ep2[8] = (uint8_t)aocep;
	*(uint32_t *)(ep2 + 9) = buffer;
	*(uint32_t *)(ep2 + 13) = period;
	ep2[17] = 0;
	uint8_t bind[11] = { 0 };                       /* struct CMD_AUDIO_OUTPUT_BIND: src, dst, bind */
	bind[8] = (uint8_t)aocep;
	bind[9] = 0;                                    /* SINK_SPEAKER */
	bind[10] = 1;
	uint8_t src[10] = { 0 };                        /* struct CMD_AUDIO_OUTPUT_SOURCE: source, on */
	src[8] = (uint8_t)aocep;
	src[9] = 1;

	aoc_sink_mode();
	if (!aoc_out_cmd("EP_SETUP", ep, sizeof(ep), 206) || !aoc_out_cmd("EP_SETUP2", ep2, sizeof(ep2), 298) ||
	    !aoc_out_cmd("BIND2 to the speaker", bind, sizeof(bind), 271)) {
		return;
	}
	(void)aoc_volume((uint8_t)aocep, 0);            /* (the entry point at SINK_SPEAKER, as Google's driver at prepare) */

	/* the ring filled half before the source goes on, then kept half full for AOCPLAY seconds */
	vm_offset_t ring = ipc_base + off;
	uint64_t frame = 0;
	uint32_t ahead = buffer - period;               /* (three periods of four: Android's HAL starts on a fuller ring) */
	aoc_tone(ring, size, rd(r, 20), ahead, &frame);
	__builtin_arm_dsb(0xf);
	wr(r, 12, rd(r, 12) + ahead);
	wr(r, 20, (rd(r, 20) + ahead) % size);
	__builtin_arm_dsb(0xf);
	aoc_signal(svc);
	if (!aoc_out_cmd("SOURCE on", src, sizeof(src), 201)) {
		return;
	}
	aoc_sink_mode();                                /* (again once bound and on: the bind set the sink active, 11-12) */
	aoc_out_state();

	uint32_t rx0 = rd(r, 16);
	uint64_t t0 = mach_absolute_time(), ns;
	bool amps = false;
	uint32_t seg = ~0u;                             /* (the listening test's setting now, from 0) */
	for (uint32_t ms = 0; ms < aocplay * 1000; ms += 10) {
		delay(10000);
		uint32_t tx = rd(r, 12), rx = rd(r, 16);
		uint32_t queued = tx - rx;
		if (queued < ahead) {
			uint32_t n = ((ahead - queued) / 4) * 4;
			aoc_tone(ring, size, rd(r, 20), n, &frame);
			__builtin_arm_dsb(0xf);
			wr(r, 12, tx + n);
			wr(r, 20, (rd(r, 20) + n) % size);
			__builtin_arm_dsb(0xf);
			aoc_signal(svc);
		}
		if (ms == 490) {
			aoc_out_state();
		}
		if (ms == 200 && rd(r, 16) != rx0 && akita_amp_speaker) {     /* the AoC reading: its TDM runs, the amps' PLL can lock */
			amps = (aocplay_matrix && akita_amp_variant ? akita_amp_variant(1) : akita_amp_speaker(1)) == 0;
			seg = 0;
		}
		if (aocplay_matrix && akita_amp_variant && seg != ~0u) {
			/* the frame the AoC is at: what was made less what waits in the ring */
			uint64_t now = frame - (uint64_t)((rd(r, 12) - rd(r, 16)) / 4);
			uint32_t s = (uint32_t)(now / MATRIX_SEG);
			if (s != seg && s < MATRIX_N) {
				seg = s;
				amps = akita_amp_variant((int)s + 1) == 0;
			}
		}
		if (ms % 1000 == 990) {
			absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
			printf("akita aoc: playing: the AoC has read %u bytes in %llu ms (%llu bytes/s; 192000 is 48 kHz stereo 16-bit), "
			    "queued %u\n", rx - rx0, ns / 1000000, ns ? (uint64_t)(rx - rx0) * 1000000000ULL / ns : 0, queued);
		}
	}
	if (aocplay_matrix && akita_amp_variant && seg != ~0u) {
		akita_amp_variant(0);                   /* (the amplifiers as set up again, down) */
		amps = false;
	}
	if (amps) {
		akita_amp_speaker(0);
	}
	src[9] = 0;
	aoc_out_cmd("SOURCE off", src, sizeof(src), 201);
	bind[10] = 0;
	aoc_out_cmd("BIND2 off", bind, sizeof(bind), 271);
	if (aoclog) {
		aoc_dump_service("debug");
	}
}

/* the first conversation: CMD_AUDIO_OUTPUT_GET_DSP_STATE (233) on audio_output_control */
static void
aoc_talk(void)
{
	vm_offset_t svc = aoc_service("audio_output_control");

	for (int i = 0; i < 3; i++) {
		aoc_mbox[i] = ml_io_map(0x15200000ULL + 0x20000ULL * (uint64_t)i, 0x1000);
	}
	if (svc) {
		vm_offset_t up = region(svc, AOC_UP), down = region(svc, AOC_DOWN);
		printf("akita aoc: audio_output_control: flags 0x%x (type %u, mailbox %u); up %u slots of %u at +0x%x (tx %u rx %u), "
		    "down %u slots of %u at +0x%x (tx %u rx %u)\n", rd(svc, 32), (rd(svc, 32) >> 16) & 7, (rd(svc, 32) >> 19) & 0x1f,
		    rd(up, 8), rd(up, 4), rd(up, 0), rd(up, 12), rd(up, 16), rd(down, 8), rd(down, 4), rd(down, 0), rd(down, 12),
		    rd(down, 16));
	}
	uint8_t cmd[9] = { 0 };
	*(uint16_t *)(cmd + 4) = 233;                   /* CMD_AUDIO_OUTPUT_GET_DSP_STATE_ID */
	int got = aoc_audio_cmd("audio_output_control", cmd, sizeof(cmd));
	if (got >= 0) {
		printf("akita aoc: THE AoC ANSWERS: %d bytes, type %u counter %u length %u, id %u, reply %d, DSP mode %u (0 idle, 1 "
		    "playback, 2 voice)\n", got, cmd[0], cmd[1], *(uint16_t *)(cmd + 2), *(uint16_t *)(cmd + 4), *(int16_t *)(cmd + 6),
		    cmd[8]);
	}
}

/* ---- Darwin's sound through the AoC (aocsound=1, with aoc=3 and amp=2) -------------------------------------------------
 * What the audio server mixes (userland/src/audio_plugin.c -> /dev/hakios-audio, drivers/audio/hakios.audio.c) to the
 * speakers, as redwood's sound=4 does it (drivers/audio/hakios.sound.c): the writer's frames into a ring of ours (never
 * waiting); a feeder thread keeps audio_playback0's ring (80 ms) three 20 ms blocks full, each block from our ring (the
 * rest silence) through the speakers' guard (drivers/audio/hakios.guard.c: the CS35L41s run without their protection
 * DSP), so the AoC's stream never stops; the amplifiers are powered up when a write is not silence and down after three
 * seconds of silence, and off while Darwin's volume is 0. The guard's limits are against the amplifiers at Android's
 * level (AMP PCM Gain 17, what CONFIG_DEFAULTS gives with amp_gain=17): with a lower amp_gain it is told the difference
 * (hakios_guard_attenuation). Before its limits it adds aocsound_gain eighths of a dB (default 0, at most 96): 11-12,
 * measured with the phone's own microphones (aocmic, aocplay_sweep), the speakers distort from about -15 dBFS at
 * Android's level -- 2.4 % at -18, 5.6 % at -12, 9.5 % at -6 (1 kHz), the same at a lower amp gain for the same
 * loudness: the speakers' own limit -- and Darwin at its top volume gives a full-scale tone at about -13 dBFS; the old
 * default, +12 dB, put Darwin's loud sounds at the peak limiter, 9-10 % distortion. */

#define DRING_FRAMES            32768           /* 0.68 s; a power of two, the indices run on and wrap */
#define SILENCE                 8
#define BLOCK_FRAMES            960             /* 20 ms */
#define BLOCK_BYTES             (BLOCK_FRAMES * 4)
#define QUIET_BEFORE_OFF_US     3000000ull

static uint32_t aocsound, aocsound_gain = 0;
static int16_t dring[DRING_FRAMES * 2];
static volatile uint32_t dring_head, dring_tail;
static volatile uint64_t last_sound_us;
static volatile uint32_t darwin_peak;
static uint64_t darwin_frames;

static uint64_t
now_us(void)
{
	uint64_t ns;

	absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
	return ns / 1000;
}

/* from the writer's thread, under hakios.audio.c's lock: never waits */
static uint32_t
aoc_darwin_sink(const int16_t *frames, uint32_t count)
{
	uint32_t head = dring_head, room = DRING_FRAMES - (head - dring_tail), taken = count < room ? count : room;
	bool sound = false;

	for (uint32_t i = 0; i < taken; i++) {
		int16_t left = frames[2 * i], right = frames[2 * i + 1];
		uint32_t l = left < 0 ? (uint32_t)-(int32_t)left : (uint32_t)left, r = right < 0 ? (uint32_t)-(int32_t)right : (uint32_t)right;

		dring[2 * ((head + i) % DRING_FRAMES)] = left;
		dring[2 * ((head + i) % DRING_FRAMES) + 1] = right;
		sound |= l > SILENCE || r > SILENCE;
		if (l > darwin_peak || r > darwin_peak) {
			darwin_peak = l > r ? l : r;
		}
	}
	__builtin_arm_dmb(0xb);
	dring_head = head + taken;
	darwin_frames += taken;
	if (sound) {
		last_sound_us = now_us();
	}
	return taken;
}

/* BYTES of SRC into the AoC's ring service region R (ring RING of SIZE bytes) at its write pointer, wrapping */
static void
aoc_ring_put(vm_offset_t svc, vm_offset_t r, vm_offset_t ring, uint32_t size, const void *src, uint32_t bytes)
{
	uint32_t wp = rd(r, 20);
	const uint32_t *w = src;

	for (uint32_t i = 0; i < bytes; i += 4) {
		*(volatile uint32_t *)(ring + (wp + i) % size) = w[i / 4];
	}
	__builtin_arm_dsb(0xf);
	wr(r, 12, rd(r, 12) + bytes);
	wr(r, 20, (wp + bytes) % size);
	__builtin_arm_dsb(0xf);
	aoc_signal(svc);
}

/* the AoC's entry point 0 set up for 48 kHz 16-bit stereo and bound to the speaker, its source on (aoc_play's commands),
 * a silent block ahead; or its source off and unbound */
static bool
aoc_stream(bool on, vm_offset_t svc, vm_offset_t r, vm_offset_t ring, uint32_t size, uint32_t buffer)
{
	static int16_t silence[BLOCK_FRAMES * 2];
	uint8_t ep[28] = { 0 }, ep2[18] = { 0 }, bind[11] = { 0 }, src[10] = { 0 };

	bind[8] = (uint8_t)aocep;
	src[8] = (uint8_t)aocep;
	bind[9] = 0;                                    /* SINK_SPEAKER */
	if (!on) {
		src[9] = 0;
		bind[10] = 0;
		return aoc_out_cmd("SOURCE off", src, sizeof(src), 201) & aoc_out_cmd("BIND2 off", bind, sizeof(bind), 271);
	}
	if (rd(r, 20) != 0) {                           /* aoc_ring_reset_write_pointer */
		wr(r, 12, rd(r, 12) + size - rd(r, 20));
		wr(r, 20, 0);
	}
	*(uint32_t *)(ep + 12) = 48000;
	ep[20] = 2; ep[21] = 1; ep[22] = 5; ep[23] = 1; ep[25] = (uint8_t)aocep; ep[26] = 1; ep[27] = 0;     /* as aoc_play's */
	ep2[8] = (uint8_t)aocep;
	*(uint32_t *)(ep2 + 9) = buffer;
	*(uint32_t *)(ep2 + 13) = BLOCK_BYTES;
	bind[10] = 1;
	src[9] = 1;
	aoc_sink_mode();
	if (!aoc_out_cmd("EP_SETUP", ep, sizeof(ep), 206) || !aoc_out_cmd("EP_SETUP2", ep2, sizeof(ep2), 298) ||
	    !aoc_out_cmd("BIND2 to the speaker", bind, sizeof(bind), 271)) {
		return false;
	}
	(void)aoc_volume((uint8_t)aocep, 0);
	aoc_ring_put(svc, r, ring, size, silence, BLOCK_BYTES);
	if (!aoc_out_cmd("SOURCE on", src, sizeof(src), 201)) {
		return false;
	}
	aoc_sink_mode();                                /* (again once bound and on: the bind set the sink active, 11-12) */
	return true;
}

/*
 * The AoC's stream runs only while there is sound: started when a write is not silence (what came before it in our ring
 * is dropped, so it starts at once), stopped -- the amplifiers first -- after three seconds of silence. While it is
 * stopped Darwin's frames (silence, or nothing) are dropped as they come. (The guard's averages stand still meanwhile
 * instead of falling: on a restart it holds back as if the speakers had not cooled -- the safe side.)
 */
static void
aoc_sound_thread(__unused void *parameter, __unused wait_result_t result)
{
	static int16_t block[BLOCK_FRAMES * 2];
	char svcname[24];

	snprintf(svcname, sizeof(svcname), "audio_playback%u", aocep);
	vm_offset_t svc = aoc_service(svcname);
	vm_offset_t r = svc ? region(svc, AOC_DOWN) : 0;
	uint32_t size = r ? rd(r, 4) : 0, period = BLOCK_BYTES, buffer = (size / period) * period;
	vm_offset_t ring = r ? ipc_base + rd(r, 0) : 0;

	if (!svc || buffer < 3 * period) {
		printf("akita aoc: sound: no %s ring to play into (%u bytes)\n", svcname, size);
		return;
	}
	{
		/* the amplifiers' level under Android's (AMP PCM Gain 17): what the guard's limits are measured against */
		uint32_t g = 0;

		PE_parse_boot_argn("amp_gain", &g, sizeof(g));
		if (g > 20) {
			g = 0;
		}
		hakios_guard_configure(true, aocsound_gain);
		hakios_guard_attenuation(g < 17 ? (17 - g) * 8 : 0);
		if (g > 17) {
			printf("akita aoc: sound: amp_gain=%u is %u dB over Android's level: the guard's limits hold %u dB too loud\n", g,
			    g - 17, g - 17);
		}
	}
	hakios_audio_set_sink("the speakers (the AoC and two CS35L41s, hakios.akita_aoc.c)", aoc_darwin_sink);
	printf("akita aoc: sound: WHAT DARWIN PLAYS GOES TO THE SPEAKERS from here on (the AoC's entry point %u, %s, started "
	    "while there is sound: 48 kHz 16-bit stereo, three 20 ms blocks ahead; the guard's gain %u.%03u dB; the amplifiers %s)\n",
	    aocep, svcname, aocsound_gain / 8, aocsound_gain % 8 * 125, akita_amp_speaker ? "on while there is sound (with speaker=1)" :
	    "NOT BUILT: unheard");

	bool streaming = false, on = false, muted = false, tried = false;
	uint32_t generation = 0, starts = 0, streams = 0, underruns = 0;
	uint64_t fed = 0, silent = 0, dropped = 0, last_report = now_us(), reported_frames = 0, stream_t0 = 0;
	for (;;) {
		uint64_t now = now_us(), sounded = last_sound_us;
		uint32_t below_now, generation_now;
		bool muted_now;

		if (hakios_audio_volume(&below_now, &muted_now, &generation_now) && generation_now != generation) {
			generation = generation_now;
			muted = muted_now;
			printf("akita aoc: sound: Darwin sets the volume: %s%u.%03u dB under the top\n", muted ? "muted (0), " : "",
			    below_now / 8, below_now % 8 * 125);
			hakios_guard_attenuation(muted ? HAKIOS_GUARD_SILENT : 0);
		}
		bool sound_now = sounded != 0 && now - sounded < 100000;

		if (!streaming) {
			if (sound_now && !muted) {
				uint64_t t0 = now_us();

				streaming = aoc_stream(true, svc, r, ring, size, buffer);
				streams++;
				tried = false;
				stream_t0 = now_us();
				printf("akita aoc: sound: Darwin plays: the AoC's stream %s in %llu ms\n", streaming ? "is on" : "DID NOT START",
				    (stream_t0 - t0) / 1000);
				if (!streaming) {
					aoc_stream(false, svc, r, ring, size, buffer);
					last_sound_us = 0;      /* (not again until the next sound) */
				}
			} else {
				uint32_t head = dring_head;     /* nothing plays: what Darwin writes is dropped as it comes */

				dropped += head - dring_tail;
				dring_tail = head;
			}
		}
		if (streaming) {
			/* the AoC's ring kept three blocks full */
			uint32_t queued = rd(r, 12) - rd(r, 16);
			if (queued < period && now - stream_t0 > 100000) {
				underruns++;
			}
			while (queued + period <= 3 * period) {
				uint32_t tail = dring_tail, available = dring_head - tail, n = available < BLOCK_FRAMES ? available : BLOCK_FRAMES;

				__builtin_arm_dmb(0xb);
				for (uint32_t i = 0; i < n; i++) {
					block[2 * i] = dring[2 * ((tail + i) % DRING_FRAMES)];
					block[2 * i + 1] = dring[2 * ((tail + i) % DRING_FRAMES) + 1];
				}
				bzero(block + 2 * n, (BLOCK_FRAMES - n) * 4);
				__builtin_arm_dmb(0xb);
				dring_tail = tail + n;
				fed += n;
				silent += BLOCK_FRAMES - n;
				hakios_guard_process(block, BLOCK_FRAMES);      /* (silence too: its filter and averages go on) */
				if (n != 0 && hakios_audio_recording_fed()) {
					hakios_audio_record_fed(block, n);
				}
				aoc_ring_put(svc, r, ring, size, block, period);
				queued += period;
			}

			/* the amplifiers: on while there is sound, off after three seconds of silence and while Darwin's volume is 0 */
			if (akita_amp_speaker && !on && !tried && !muted && sound_now) {
				tried = true;           /* (once a stream: they may refuse -- speaker=1 not given) */
				on = akita_amp_speaker(1) == 0;
				starts += on;
				if (on) {
					printf("akita aoc: sound: the amplifiers are up (%u ms after the write that was not silence)\n",
					    (uint32_t)((now_us() - sounded) / 1000));
				}
			}
			if (muted || now - sounded > QUIET_BEFORE_OFF_US) {
				if (on) {
					akita_amp_speaker(0);
				}
				aoc_stream(false, svc, r, ring, size, buffer);
				streaming = false;
				printf("akita aoc: sound: %s: %sthe AoC's stream is off; what Darwin wrote peaked at %u%% of full scale\n",
				    muted ? "muted" : "three seconds of silence", on ? "the amplifiers down, " : "", darwin_peak * 100 / 32768);
				on = false;
				darwin_peak = 0;
			}
		}
		if (now - last_report >= 10000000 && darwin_frames != reported_frames) {
			printf("akita aoc: sound: from Darwin %llu frames so far; %llu fed to the AoC (%llu of silence filled in), %llu "
			    "dropped while nothing played, %u underruns; the stream started %u times, the amplifiers %u times\n",
			    darwin_frames, fed, silent, dropped, underruns, streams, starts);
			reported_frames = darwin_frames;
			last_report = now;
		}
		delay(5000);
	}
}

/*
 * aocmic=S: the phone's own two microphones (the built-in PDM mics 0 and 1, as Android's "handset-mic"/"speaker-mic"),
 * S seconds, through the AoC as Google's driver records (aoc_alsa_hw.c, UC_AUDIO_RECORD on audio_capture0):
 * MIC_RECORD_AP_SET_PARAMS (224: 2 channels, 16-bit, 48 kHz, mics 0/1 interleaved, 10 ms periods, raw), SET_PARAMS2
 * (359), the mic mask (SET_PARAMETER 233 block 139 ABLOCK_INPUT_PDM_MIC), AP_INPUT_START (208: the built-in mics),
 * SET_AP_MIC_INDEX (319: raw) and MIC_RECORD_AP_START_DATA (293) on audio_input_control; the samples come up
 * audio_capture0's ring. Each second's level of each mic goes in the log; with rec_sound=N rec_sound_mic=1 they go into
 * the recording too (mic 0 left, mic 1 right), fetched over USB (usbmux port 8: devices/akita/tools/usbmux-sound.py).
 * With aocplay=S it records while the tone plays: the speakers heard by the phone itself (11-12, the tester: "test for
 * distortion").
 */
static uint32_t aocmic;
static volatile bool mic_done, mic_self_stop;   /* (without aocplay the thread stops the capture itself: Darwin's sound
                                                 * goes on meanwhile -- the mics hear iOS's own sounds) */
static void aoc_mic_stop(void);

static bool
aoc_in_cmd(const char *what, void *cmd, uint32_t len, uint16_t id)
{
	*(uint16_t *)((uint8_t *)cmd + 4) = id;
	int got = aoc_audio_cmd("audio_input_control", cmd, len);
	printf("akita aoc: mic: %s (%u): %s, reply field %d\n", what, id, got < 0 ? "FAILED" : "done", got < 0 ? 0 :
	    *(int16_t *)((uint8_t *)cmd + 6));
	return got >= 0;
}

static uint32_t
isqrt64(uint64_t v)
{
	uint64_t r = 0, b = 1ULL << 62;

	while (b > v) {
		b >>= 2;
	}
	while (b != 0) {
		if (v >= r + b) {
			v -= r + b;
			r = (r >> 1) + b;
		} else {
			r >>= 1;
		}
		b >>= 2;
	}
	return (uint32_t)r;
}

static void
aoc_mic_thread(__unused void *parameter, __unused wait_result_t result)
{
	static int16_t buf[4800 * 2];                   /* 100 ms of two channels */
	vm_offset_t svc = aoc_service("audio_capture0");
	vm_offset_t r = svc ? region(svc, AOC_UP) : 0;
	uint32_t size = r ? rd(r, 4) : 0, off = r ? rd(r, 0) : 0;
	uint64_t t0 = now_us(), frames = 0, sq[2] = { 0 }, n_sec = 0;
	uint32_t peak[2] = { 0 }, second = 0;
	bool rec = hakios_audio_recording_mic();

	if (!svc || size == 0 || (uint64_t)off + size > ipc_room) {
		printf("akita aoc: mic: no audio_capture0 ring (%u bytes at +0x%x)\n", size, off);
		mic_done = true;
		return;
	}
	printf("akita aoc: mic: reading audio_capture0 (%u bytes at +0x%x, tx %u rx %u) for %u s%s\n", size, off, rd(r, 12),
	    rd(r, 16), aocmic, rec ? ", into the recording (rec_sound_mic=1)" : "");
	while (now_us() - t0 < (uint64_t)aocmic * 1000000ULL) {
		uint32_t tx = rd(r, 12), rx = rd(r, 16), avail = tx - rx;

		if (avail > size) {                     /* (overrun: the oldest is gone) */
			wr(r, 16, tx - size / 2);
			wr(r, 24, (rd(r, 20) + size - size / 2) % size);
			printf("akita aoc: mic: the ring overran (%u bytes waiting)\n", avail);
			continue;
		}
		if (avail < 4) {
			delay(5000);
			continue;
		}
		uint32_t n = avail & ~3u, rp = rd(r, 24);
		__builtin_arm_dmb(0xf);                 /* (the samples read after the count that says they are there) */
		n = n < sizeof(buf) ? n : (uint32_t)sizeof(buf);
		for (uint32_t i = 0; i < n; i += 4) {
			*(uint32_t *)((uint8_t *)buf + i) = *(volatile uint32_t *)(ipc_base + off + (rp + i) % size);
		}
		__builtin_arm_dsb(0xf);
		wr(r, 16, rx + n);
		wr(r, 24, (rp + n) % size);
		for (uint32_t f = 0; f < n / 4; f++) {
			for (int c = 0; c < 2; c++) {
				int32_t v = buf[2 * f + c];
				uint32_t a = (uint32_t)(v < 0 ? -v : v);
				sq[c] += (uint64_t)(v * v);
				peak[c] = a > peak[c] ? a : peak[c];
			}
		}
		n_sec += n / 4;
		frames += n / 4;
		if (rec) {
			hakios_audio_record_fed(buf, n / 4);
		}
		if (now_us() - t0 >= (uint64_t)(second + 1) * 1000000ULL) {
			printf("akita aoc: mic: second %u: %llu frames; mic 0 rms %u peak %u, mic 1 rms %u peak %u (of 32768)\n", second,
			    n_sec, n_sec ? isqrt64(sq[0] / n_sec) : 0, peak[0], n_sec ? isqrt64(sq[1] / n_sec) : 0, peak[1]);
			second++;
			n_sec = 0;
			sq[0] = sq[1] = 0;
			peak[0] = peak[1] = 0;
		}
	}
	printf("akita aoc: mic: %llu frames in %u s (48000 a second expected)\n", frames, aocmic);
	if (mic_self_stop) {
		aoc_mic_stop();
	}
	mic_done = true;
}

static bool
aoc_mic_start(void)
{
	uint8_t p[22] = { 0 }, p2[17] = { 0 }, m[18] = { 0 }, in[9] = { 0 }, idx[9] = { 0 }, start[8] = { 0 };
	vm_offset_t svc = aoc_service("audio_capture0");
	vm_offset_t r = svc ? region(svc, AOC_UP) : 0;
	uint32_t key = 0, val = 0xffff0100u;            /* the mic mask: mic ids 0, 1, then none */
	uint32_t period = 1920, buffer = 4 * 1920;      /* 10 ms of two 16-bit channels; four of them */

	p[8] = 2;                                       /* requested_format: CH_STEREO */
	p[9] = 1;                                       /* WIDTH_16_BIT */
	p[10] = 5;                                      /* SR_48KHZ */
	p[11] = 1;                                      /* FRMT_FIXED_POINT */
	p[12] = 0;                                      /* offset */
	p[13] = 0x3;                                    /* pdm_mask: mics 0 and 1 */
	p[14] = 10;                                     /* period_ms */
	p[15] = 4;                                      /* num_periods */
	p[16] = 0; p[17] = 1; p[18] = 0xff; p[19] = 0xff;       /* interleaving */
	p[20] = 5;                                      /* sample_rate: SR_48KHZ */
	p[21] = 0;                                      /* mic_process_index: AP_MIC_PROCESS_RAW */
	p2[8] = 8;                                      /* channel: UC_AUDIO_RECORD (audio_capture0) */
	memcpy(p2 + 9, &buffer, 4);
	memcpy(p2 + 13, &period, 4);
	m[8] = 139;                                     /* ABLOCK_INPUT_PDM_MIC */
	m[9] = 0;                                       /* ASP_ID_NONE */
	memcpy(m + 10, &key, 4);                        /* not voice */
	memcpy(m + 14, &val, 4);
	in[8] = 1;                                      /* AP_INPUT_PROCESSOR_MIC_INPUT_INDEX */
	idx[8] = 0;                                     /* AP_MIC_PROCESS_RAW */

	if (!aoc_in_cmd("MIC_RECORD_AP_SET_PARAMS", p, sizeof(p), 224) ||
	    !aoc_in_cmd("MIC_RECORD_AP_SET_PARAMS2", p2, sizeof(p2), 359)) {
		return false;
	}
	(void)aoc_in_cmd("SET_PARAMETER (mic mask)", m, sizeof(m), 233);
	if (!aoc_in_cmd("AP_INPUT_START (built-in mics)", in, sizeof(in), 208)) {
		return false;
	}
	if (r) {                                        /* the ring flushed (aoc_ring_flush_read_data) */
		wr(r, 16, rd(r, 12));
		wr(r, 24, rd(r, 20));
	}
	(void)aoc_in_cmd("SET_AP_MIC_INDEX (raw)", idx, sizeof(idx), 319);
	return aoc_in_cmd("MIC_RECORD_AP_START_DATA", start, sizeof(start), 293);
}

static void
aoc_mic_stop(void)
{
	uint8_t stop[8] = { 0 }, in[8] = { 0 };

	(void)aoc_in_cmd("MIC_RECORD_AP_STOP", stop, sizeof(stop), 226);
	(void)aoc_in_cmd("AP_INPUT_STOP", in, sizeof(in), 209);
}

/*
 * aocrec=1: the microphone for Darwin -- /dev/hakios-mic, as redwood's hakios.mic.c (mic=4) gives it to
 * HakiosAudio.driver's device "Codec" (userland/src/audio_plugin.c), of which the virtual audio device makes the
 * built-in microphone: 16-bit samples, one channel (mic 0), 48 kHz; a read never waits, keeping the reader about 40 ms
 * behind; nothing until that much is there. The AoC records (aoc_mic_start) only while the device is open -- an
 * application records, iOS shows its orange dot -- and stops three seconds after the last close.
 */
#define MRING_FRAMES            16384           /* 341 ms; a power of two */
#define MRING_KEEP              1920            /* 40 ms */
#define MRING_MOST              4800            /* 100 ms: more waiting, and the oldest is skipped */

static uint32_t aocrec;
static int16_t mring[MRING_FRAMES];
static volatile uint64_t mring_written;
static uint64_t mring_read, mring_skipped, mring_dry;
static bool mring_primed;
static volatile uint32_t mic_wanted;

static int
aoc_mic_dev_read(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	uint64_t written = __atomic_load_n(&mring_written, __ATOMIC_ACQUIRE), avail, want;
	int error = 0;

	if (uio_resid(uio) < 2) {
		return 0;
	}
	want = (uint64_t)uio_resid(uio) / 2;
	avail = written - mring_read;
	if (avail > MRING_MOST) {
		mring_skipped += avail - MRING_KEEP;
		mring_read = written - MRING_KEEP;
		avail = MRING_KEEP;
	}
	if (!mring_primed) {
		if (avail < MRING_KEEP) {
			return 0;
		}
		mring_primed = true;
	}
	if (avail < want) {
		mring_dry++;
		mring_primed = false;
	}
	while (want > 0 && avail > 0 && error == 0) {
		uint64_t at = mring_read % MRING_FRAMES, n = want < avail ? want : avail;

		n = n < MRING_FRAMES - at ? n : MRING_FRAMES - at;
		error = uiomove((caddr_t)&mring[at], (int)(n * 2), uio);
		mring_read += n;
		want -= n;
		avail -= n;
	}
	return error;
}

static int
aoc_mic_dev_open(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	mic_wanted = 1;
	return 0;
}

static int
aoc_mic_dev_close(__unused dev_t dev, __unused int flags, __unused int type, __unused struct proc *p)
{
	mic_wanted = 0;
	return 0;
}

static const struct cdevsw aoc_mic_cdevsw = {
	.d_open = aoc_mic_dev_open, .d_close = aoc_mic_dev_close, .d_read = aoc_mic_dev_read, .d_write = eno_rdwrt,
	.d_ioctl = eno_ioctl, .d_stop = eno_stop, .d_reset = eno_reset, .d_select = eno_select,
	.d_mmap = eno_mmap, .d_strategy = eno_strat, .d_type = 0
};

/* the AoC's capture while Darwin has /dev/hakios-mic open */
static void
aoc_darwin_mic_thread(__unused void *parameter, __unused wait_result_t result)
{
	static int16_t buf[4800 * 2];
	vm_offset_t svc = aoc_service("audio_capture0");
	vm_offset_t r = svc ? region(svc, AOC_UP) : 0;
	uint32_t size = r ? rd(r, 4) : 0, off = r ? rd(r, 0) : 0;

	if (!svc || size == 0 || (uint64_t)off + size > ipc_room) {
		printf("akita aoc: mic for Darwin: no audio_capture0 ring\n");
		return;
	}
	for (;;) {
		while (!mic_wanted) {
			delay(50000);
		}
		uint64_t t0 = now_us(), frames = 0, idle_since = 0;
		uint32_t peak = 0;
		bool rec = hakios_audio_recording_mic();

		if (!aoc_mic_start()) {
			printf("akita aoc: mic for Darwin: the AoC would not record; again in a second\n");
			delay(1000000);
			continue;
		}
		mring_primed = false;
		mring_read = __atomic_load_n(&mring_written, __ATOMIC_ACQUIRE);
		printf("akita aoc: mic for Darwin: LISTENING (/dev/hakios-mic opened)\n");
		for (;;) {
			if (!mic_wanted) {
				if (idle_since == 0) {
					idle_since = now_us();
				} else if (now_us() - idle_since > 3000000ULL) {
					break;
				}
			} else {
				idle_since = 0;
			}
			uint32_t tx = rd(r, 12), rx = rd(r, 16), avail = tx - rx;
			if (avail > size) {
				wr(r, 16, tx - size / 2);
				wr(r, 24, (rd(r, 20) + size - size / 2) % size);
				continue;
			}
			if (avail < 4) {
				delay(5000);
				continue;
			}
			uint32_t n = avail & ~3u, rp = rd(r, 24);
			__builtin_arm_dmb(0xf);
			n = n < sizeof(buf) ? n : (uint32_t)sizeof(buf);
			for (uint32_t i = 0; i < n; i += 4) {
				*(uint32_t *)((uint8_t *)buf + i) = *(volatile uint32_t *)(ipc_base + off + (rp + i) % size);
			}
			__builtin_arm_dsb(0xf);
			wr(r, 16, rx + n);
			wr(r, 24, (rp + n) % size);
			uint64_t at = mring_written;
			for (uint32_t f = 0; f < n / 4; f++) {
				int16_t v = buf[2 * f];                 /* mic 0 */
				uint32_t a = (uint32_t)(v < 0 ? -(int32_t)v : v);
				mring[(at + f) % MRING_FRAMES] = v;
				peak = a > peak ? a : peak;
			}
			__atomic_store_n(&mring_written, at + n / 4, __ATOMIC_RELEASE);
			frames += n / 4;
			if (rec) {
				hakios_audio_record_fed(buf, n / 4);
			}
		}
		aoc_mic_stop();
		printf("akita aoc: mic for Darwin: stopped after %llu ms, %llu frames, peak %u of 32768; %llu skipped, %llu reads "
		    "found too little\n", (now_us() - t0) / 1000, frames, peak, mring_skipped, mring_dry);
	}
}

static void
aoc_darwin_mic_init(void)
{
	thread_t t;
	int major = cdevsw_add(-1, &aoc_mic_cdevsw);

	if (major < 0 || devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0666, "hakios-mic") == NULL) {
		printf("akita aoc: mic for Darwin: /dev/hakios-mic cannot be made\n");
		return;
	}
	if (kernel_thread_start_priority(aoc_darwin_mic_thread, NULL, 90, &t) == KERN_SUCCESS) {
		thread_deallocate(t);
	}
	printf("akita aoc: mic for Darwin: /dev/hakios-mic is the built-in microphone (the AoC listens only while it is open)\n");
}

/*
 * THE FIRMWARE ON THE DATA VOLUME (11-15): aoc.bin is 28 MB, half the RAM disk, and the flashed boot image was 58 of
 * its partition's 64 MB. The AoC starts 25 s in, long after /private/var is mounted, so the image may leave the file
 * out: it is then read from the data volume (boot-arg aoc_fw=PATH, default below), where init puts the RAM disk's copy
 * whenever an image carries one (userland/src/init.c). The RAM disk's own copy still wins when there is one.
 */
#define AOC_FW_DISK_PATH        "/private/var/hakios/fw/aoc.bin"

static uint8_t *
aoc_disk_file(uint64_t *size)
{
	char path[128] = AOC_FW_DISK_PATH;
	vfs_context_t context = vfs_context_kernel();
	vnode_t vnode = NULLVP;
	struct vnode_attr attributes;
	uint8_t *data;
	int left = 0, error;

	PE_parse_boot_argn("aoc_fw", path, sizeof(path));
	if (rootvnode == NULLVP || vnode_lookup(path, 0, &vnode, context) != 0) {
		printf("akita aoc: no %s on the disk either\n", path);
		return NULL;
	}
	VATTR_INIT(&attributes);
	VATTR_WANTED(&attributes, va_data_size);
	if (vnode_getattr(vnode, &attributes, context) != 0 || attributes.va_data_size < AOC_AUTH_HEADER_SIZE + 0x1000 ||
	    attributes.va_data_size > AOC_DRAM_SIZE) {
		vnode_put(vnode);
		printf("akita aoc: %s: not a size an AoC image has\n", path);
		return NULL;
	}
	data = kalloc_data(attributes.va_data_size, Z_WAITOK);
	if (data == NULL) {
		vnode_put(vnode);
		return NULL;
	}
	error = vn_rdwr(UIO_READ, vnode, (caddr_t)data, (int)attributes.va_data_size, 0, UIO_SYSSPACE, 0,
	    vfs_context_ucred(context), &left, vfs_context_proc(context));
	vnode_put(vnode);
	if (error != 0 || left != 0) {
		printf("akita aoc: %s: read error %d, %d bytes left\n", path, error, left);
		kfree_data(data, attributes.va_data_size);
		return NULL;
	}
	printf("akita aoc: the firmware from the disk: %s (%llu bytes)\n", path, (uint64_t)attributes.va_data_size);
	*size = attributes.va_data_size;
	return data;
}


/*
 * SENSORS THROUGH THE AoC (boot-arg usf=1, 11-16 night, A PROBE: it says what it finds and drives nothing). Google's
 * sensor stack ("USF", com.google.usf) as decoded from the Android HAL's start-up traffic (~/akita/ref/usf-android,
 * decoded/README.md): the service "com.google.usf", every message a u32 channel header and a flatbuffer; channel 2
 * opened by a control message on channel 0 ({0, OPEN 0, 2}); requests are the HAL's own bytes with the transaction id,
 * the destination object and the payload's fields patched: type 3 (an object by UUID, to object 1), 5 (the
 * registry's sensors), 4 (a sensor's info: name, USF type -- accel 1, gyro 2, prox 3, ALS 9), 8 (subscribe: cookie,
 * period in ns). Answers: kind 2 with the same transaction id; events kind 9, a raw struct (count, cookie,
 * subscription, u16 type, u8, u8 data length, u64 ns, data).
 */
static void sensor_power(const char *when, bool cycle);
static void aoc_dump_service(const char *name);
static const uint8_t usf_t_lookup[] = { 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x14, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x08, 0x00, 0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x6b, 0x31, 0xcb, 0xf0, 0x74, 0x4a, 0x14, 0xa1, 0xd4, 0xe8, 0xae, 0x8a, 0x3e, 0x09, 0xfc, 0x4d };
static const uint8_t usf_t_info[] = { 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x10, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x39, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00 };
static const uint8_t usf_t_timesync[] = { 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x48, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x14, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x14, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x9a, 0xbc, 0xa8, 0x82, 0xf0, 0x00, 0x00, 0x00, 0x9f, 0xf4, 0x70, 0x3c, 0xec, 0x00, 0x00, 0x00 };   /* type 11 to object 4: two u64 ns at 84, 92 */
static const uint8_t usf_t_32[] = { 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x14, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x08, 0x00, 0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00 };         /* type 32 {1} to object 8 */
static const uint8_t usf_t_subscribe[] = { 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x14, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x35, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x20, 0x00, 0x1c, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x07, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0e, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa0, 0x25, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00 };
#define USF_AT_DEST     48
#define USF_AT_TYPE     52
#define USF_AT_TX       56
#define USF_AT_UUID     92      /* (the lookup's last 16 bytes) */
#define USF_AT_COOKIE   108     /* the subscribe's */
#define USF_AT_PERIOD   116
#define USF_AT_F0       128
#define USF_CHANNEL     2

static vm_offset_t usf_svc;
static uint32_t usf_tx = 1, usf_events, usf_said;

static inline uint32_t usf_u32(const uint8_t *b, uint32_t o) { return (uint32_t)b[o] | (uint32_t)b[o + 1] << 8 | (uint32_t)b[o + 2] << 16 | (uint32_t)b[o + 3] << 24; }
static inline uint32_t usf_u16(const uint8_t *b, uint32_t o) { return (uint32_t)b[o] | (uint32_t)b[o + 1] << 8; }

/* field IDX of the table at T in B (LEN bytes): its absolute offset, or 0 */
static uint32_t
usf_field(const uint8_t *b, uint32_t len, uint32_t t, unsigned idx)
{
	if (t + 4 > len) {
		return 0;
	}
	uint32_t vt = t - (uint32_t)(int32_t)usf_u32(b, t);
	if (vt + 4 > len || vt > t) {
		return 0;
	}
	uint32_t vlen = usf_u16(b, vt);
	if (4 + 2 * idx + 2 > vlen || vt + 4 + 2 * idx + 2 > len) {
		return 0;
	}
	uint32_t fo = usf_u16(b, vt + 4 + 2 * idx);
	return fo != 0 && t + fo + 4 <= len ? t + fo : 0;
}

/* the vector (or string) a field at AT points to: its data's offset and count; false if out of bounds */
static bool
usf_vector(const uint8_t *b, uint32_t len, uint32_t at, uint32_t *data, uint32_t *n)
{
	uint32_t v = at + usf_u32(b, at);
	if (at == 0 || v + 4 > len) {
		return false;
	}
	*n = usf_u32(b, v);
	*data = v + 4;
	return *data + *n <= len || *data + *n * 4 <= len;
}

/* what the sensors last said, for iOS: kern.hakios.motion (read by backboardd's injected HID services, inject.c) */
static struct {
	uint32_t version;               /* 1 */
	uint32_t accel_seq, light_seq, proximity_seq;
	uint64_t accel_ns;              /* the AoC's time stamp */
	float accel[3];                 /* m/s2, Android's axes (x right, y up, z out of the screen; +9.8 on z lying face up) */
	float lux, proximity;
} usf_motion = { .version = 1 };
SYSCTL_DECL(_kern_hakios);
SYSCTL_OPAQUE(_kern_hakios, OID_AUTO, motion, CTLFLAG_RD | CTLFLAG_LOCKED, &usf_motion, sizeof(usf_motion), "S",
    "the AoC's sensors, the latest: accelerometer (m/s2), light (lux), proximity");

static void
usf_event(const uint8_t *e, uint32_t n)
{
	usf_events++;
	if (n >= 28) {
		/* the latest of each for iOS (kern.hakios.motion): USF type 1 accelerometer (m/s2, Android's axes), 9 light
		 * (lux first), 3 proximity (its first value: the distance, 5 = far) */
		uint32_t type = usf_u16(e, 12), dlen = e[15];
		uint64_t ts = (uint64_t)usf_u32(e, 16) | (uint64_t)usf_u32(e, 20) << 32;

		if (type == 1 && dlen >= 12 && n >= 36) {
			memcpy(usf_motion.accel, e + 24, 12);
			usf_motion.accel_ns = ts;
			usf_motion.accel_seq++;
		} else if (type == 9 && dlen >= 4) {
			memcpy(&usf_motion.lux, e + 24, 4);
			usf_motion.light_seq++;
		} else if (type == 3 && dlen >= 4) {
			memcpy(&usf_motion.proximity, e + 24, 4);
			usf_motion.proximity_seq++;
		}
	}
	if (n < 24 || usf_said >= 12) {
		return;
	}
	uint32_t cookie = usf_u32(e, 4), sub = usf_u32(e, 8), type = usf_u16(e, 12), dlen = e[15];
	uint64_t ts = (uint64_t)usf_u32(e, 16) | (uint64_t)usf_u32(e, 20) << 32;
	char vals[96] = "";
	unsigned at = 0;

	for (uint32_t k = 0; k + 4 <= dlen && 24 + k + 4 <= n && k < 16; k += 4) {
		uint32_t w = usf_u32(e, 24 + k);
		float f;
		memcpy(&f, &w, 4);
		int milli = (int)(f * 1000.0f);
		at += (unsigned)snprintf(vals + at, sizeof(vals) - at, " %s%d.%03d", milli < 0 ? "-" : "", (milli < 0 ? -milli : milli) / 1000,
		    (milli < 0 ? -milli : milli) % 1000);
	}
	printf("akita aoc: usf: event: cookie 0x%x subscription %u type %u, %u bytes, at %llu ns:%s\n", cookie, sub, type, dlen, ts, vals);
	usf_said++;
}

/* REQ (a template, LEN bytes, patched by the caller) sent; the answer with its transaction id into ANS (its inner
 * message: offset and length in BUF), events met meanwhile counted. true if answered in MS ms */
static bool
usf_request(uint8_t *req, uint32_t len, uint8_t *buf, uint32_t cap, uint32_t *inner, uint32_t *ilen, int ms)
{
	uint8_t out[160];
	uint32_t tx = usf_tx++;

	req[USF_AT_TX] = (uint8_t)tx; req[USF_AT_TX + 1] = (uint8_t)(tx >> 8); req[USF_AT_TX + 2] = 0; req[USF_AT_TX + 3] = 0;
	if (len + 4 > sizeof(out)) {
		return false;
	}
	out[0] = USF_CHANNEL; out[1] = out[2] = out[3] = 0;
	memcpy(out + 4, req, len);
	if (!aoc_write(usf_svc, out, len + 4)) {
		printf("akita aoc: usf: the queue to the AoC is full\n");
		return false;
	}
	for (int waited = 0; waited < ms;) {
		int n = aoc_read(usf_svc, buf, cap, 10);
		if (n < 0) {
			waited += 10;
			continue;
		}
		if (n < 16 || (uint32_t)n > cap) {
			continue;
		}
		const uint8_t *m = buf + 4;     /* (after the channel's header) */
		uint32_t ml = (uint32_t)n - 4, root = usf_u32(m, 0), kind_at = usf_field(m, ml, root, 0), vec_at = usf_field(m, ml, root, 1);
		uint32_t kind = kind_at ? usf_u32(m, kind_at) : 0, d = 0, dn = 0;

		if (!usf_vector(m, ml, vec_at, &d, &dn) || d + dn > ml) {
			continue;
		}
		if (kind == 9) {
			usf_event(m + d, dn);
			continue;
		}
		if (kind != 2) {
			continue;
		}
		uint32_t iroot = usf_u32(m + d, 0), txa = usf_field(m + d, dn, iroot, 1);
		if (txa != 0 && usf_u32(m + d, txa) == tx) {
			*inner = 4 + d;
			*ilen = dn;
			return true;
		}
	}
	return false;
}

/* the answer's payload (inner field 3, itself a flatbuffer): its root table's offset in P and P's length; false if none */
static bool
usf_payload(const uint8_t *in, uint32_t n, const uint8_t **p, uint32_t *plen, uint32_t *status)
{
	uint32_t root = usf_u32(in, 0), st = usf_field(in, n, root, 2), pa = usf_field(in, n, root, 3), d, dn;

	*status = st ? usf_u32(in, st) : 0;
	if (!usf_vector(in, n, pa, &d, &dn) || d + dn > n || dn < 8) {
		return false;
	}
	*p = in + d;
	*plen = dn;
	return true;
}

/*
 * THE SENSOR REGISTRY (11-20): the AoC's sensor core knows no sensor until the AP gives it the board's registry
 * scripts -- Android's sensors HAL does after every AoC start (traced across an AoC reset on Android,
 * fdtrace of its writes): to the AoC's file-system object (UUID d76a14cc..., object 5) type 24 requests { the script's text, the
 * board's CDT 0x46100 } -- /vendor/etc/sensors/registry/akita_common, _pvt, _proto, _dev, _dvt .reg (the AoC keeps
 * what the CDT filters select), append/motion_state.reg, power.reg, the factory calibration (/fac_cal/...) and
 * /persist from the phone's /mnt/vendor/persist -- and type 22 (/dev/display_info/0: the panel). Big ones in
 * fragments (kind 3: { 1, total, offset, up to 940 bytes of the whole message }). Those writes, as Android sent them,
 * are fw/usf-registry.bin ("HKUR0001", then { u16 length, u16 1 if an answer comes after it } + the message; this
 * phone's calibration: never in git, fw-sensors/); sent here unchanged, before the list is asked for.
 */
static uint32_t
usf_registry_push(uint8_t *buf, uint32_t cap)
{
	static const uint8_t fs_uuid[16] = { 0xd7, 0x6a, 0x14, 0xcc, 0x26, 0x4d, 0xee, 0xa1, 0xc6, 0x40, 0x6f, 0x8b, 0x46, 0x16, 0x82, 0xc0 };
	static uint8_t out[1100];
	uint8_t req[sizeof(usf_t_lookup)];
	uint32_t in, il, st, plen, fs = 0, sent = 0, answered = 0;
	const uint8_t *p, *file;
	uint64_t size = 0;

	file = hakios_image_file("fw", "usf-registry.bin", &size, NULL);
	if (file == NULL || size < 8 || memcmp(file, "HKUR0001", 8) != 0) {
		printf("akita aoc: usf: no fw/usf-registry.bin: the sensor core gets no registry (and knows no sensor)\n");
		return 0;
	}
	memcpy(req, usf_t_lookup, sizeof(usf_t_lookup));
	memcpy(req + USF_AT_UUID, fs_uuid, 16);
	if (usf_request(req, sizeof(usf_t_lookup), buf, cap, &in, &il, 1000) && usf_payload(buf + in, il, &p, &plen, &st)) {
		uint32_t f = usf_field(p, plen, usf_u32(p, 0), 0);
		fs = f ? usf_u32(p, f) : 0;
	}
	if (fs != 5) {
		printf("akita aoc: usf: the file system is object %u, not Android's 5: the registry (addressed to 5) not sent\n", fs);
		return 0;
	}
	for (uint64_t o = 8; o + 4 <= size;) {
		uint16_t len, expect;

		memcpy(&len, file + o, 2);
		memcpy(&expect, file + o + 2, 2);
		if (o + 4 + len > size || len + 4u > sizeof(out)) {
			break;
		}
		out[0] = USF_CHANNEL; out[1] = out[2] = out[3] = 0;
		memcpy(out + 4, file + o + 4, len);
		if (!aoc_write(usf_svc, out, len + 4u)) {
			printf("akita aoc: usf: the queue to the AoC is full at the registry's message %u\n", sent);
			break;
		}
		sent++;
		o += 4u + len;
		if (!expect) {
			continue;
		}
		/* its answer (any response: a kind 2; events said as they come) */
		for (int waited = 0; waited < 1000;) {
			int n = aoc_read(usf_svc, buf, cap, 10);
			if (n < 0) {
				waited += 10;
				continue;
			}
			if (n < 16 || (uint32_t)n > cap) {
				continue;
			}
			const uint8_t *m = buf + 4;
			uint32_t ml = (uint32_t)n - 4, root = usf_u32(m, 0), ka = usf_field(m, ml, root, 0);
			if (ka && usf_u32(m, ka) == 2) {
				answered++;
				break;
			}
		}
	}
	printf("akita aoc: usf: the registry given to the sensor core: %u messages sent, %u answered\n", sent, answered);
	return answered;
}

static void
usf_probe(void)
{
	static uint8_t buf[1100];
	uint8_t req[sizeof(usf_t_subscribe)];
	uint32_t in, il, st, plen, reg = 0;
	const uint8_t *p;
	static const uint8_t reg_uuid[16] = { 0x6b, 0x31, 0xcb, 0xf0, 0x74, 0x4a, 0x14, 0xa1, 0xd4, 0xe8, 0xae, 0x8a, 0x3e, 0x09, 0xfc, 0x4d };
	uint32_t ids[64], nids = 0, accel = 0, light = 0, prox = 0;

	sensor_power("as the probe begins", false);
	{
		/* every service the AoC offers, with how much each one's two queues hold (what the AP may be expected to serve) */
		uint32_t count = rd(ipc_base, 24), each = rd(ipc_base, 28), at = rd(ipc_base, 32);
		for (uint32_t k = 0; k < count && each >= 92 && k < 96; k++) {
			vm_offset_t h = ipc_base + at + k * each;
			char nm[33];
			for (int c = 0; c < 32; c++) {
				nm[c] = (char)*(volatile uint8_t *)(h + c);
			}
			nm[32] = 0;
			vm_offset_t up = region(h, AOC_UP), down = region(h, AOC_DOWN);
			printf("akita aoc: service %u: \"%s\" up tx %u rx %u, down tx %u rx %u\n", k, nm, rd(up, 12), rd(up, 16), rd(down, 12), rd(down, 16));
		}
	}
	if ((usf_svc = aoc_service("com.google.usf")) == 0) {
		printf("akita aoc: usf: no service com.google.usf\n");
		return;
	}
	{
		const uint32_t open[3] = { 0, 0, USF_CHANNEL };         /* channel 0: OPEN_CHANNEL 2 */
		printf("akita aoc: usf: the service found; channel %u opened (%s)\n", USF_CHANNEL,
		    aoc_write(usf_svc, open, sizeof(open)) ? "sent" : "NOT sent");
		for (int k = 0; k < 5; k++) {
			int n = aoc_read(usf_svc, buf, sizeof(buf), 50);
			if (n < 0) {
				break;
			}
			aoc_say_bytes("usf after the open", buf, n);
		}
	}
	memcpy(req, usf_t_lookup, sizeof(usf_t_lookup));
	memcpy(req + USF_AT_UUID, reg_uuid, 16);
	if (!usf_request(req, sizeof(usf_t_lookup), buf, sizeof(buf), &in, &il, 1000)) {
		printf("akita aoc: usf: the registry's lookup NOT answered\n");
		return;
	}
	if (usf_payload(buf + in, il, &p, &plen, &st)) {
		uint32_t f = usf_field(p, plen, usf_u32(p, 0), 0);
		reg = f ? usf_u32(p, f) : 0;
	}
	printf("akita aoc: usf: the sensor registry is object %u (status %u)\n", reg, st);
	if (reg == 0) {
		return;
	}
	/* the HAL's steps before its list (Android: objects 4 and 8 by UUID, a time sync to 4, type 32 {1} to 8) */
	{
		static const uint8_t uuid4[16] = { 0x8b, 0xb8, 0x1b, 0x18, 0x25, 0x4c, 0x29, 0x9f, 0x0a, 0x45, 0xde, 0xb4, 0xd8, 0xcb, 0xc5, 0x41 };
		static const uint8_t uuid8[16] = { 0xae, 0x75, 0x6f, 0x06, 0x4c, 0x4f, 0x5a, 0x3b, 0xbc, 0xc3, 0x9f, 0x8d, 0xa1, 0x06, 0xef, 0xc3 };
		uint32_t o4 = 0, o8 = 0;

		memcpy(req, usf_t_lookup, sizeof(usf_t_lookup));
		memcpy(req + USF_AT_UUID, uuid4, 16);
		if (usf_request(req, sizeof(usf_t_lookup), buf, sizeof(buf), &in, &il, 1000) && usf_payload(buf + in, il, &p, &plen, &st)) {
			uint32_t f = usf_field(p, plen, usf_u32(p, 0), 0);
			o4 = f ? usf_u32(p, f) : 0;
		}
		memcpy(req, usf_t_lookup, sizeof(usf_t_lookup));
		memcpy(req + USF_AT_UUID, uuid8, 16);
		if (usf_request(req, sizeof(usf_t_lookup), buf, sizeof(buf), &in, &il, 1000) && usf_payload(buf + in, il, &p, &plen, &st)) {
			uint32_t f = usf_field(p, plen, usf_u32(p, 0), 0);
			o8 = f ? usf_u32(p, f) : 0;
		}
		printf("akita aoc: usf: objects %u and %u (Android's 4 and 8)\n", o4, o8);
		if (o4 != 0) {
			uint64_t ns;
			absolutetime_to_nanoseconds(mach_absolute_time(), &ns);
			memcpy(req, usf_t_timesync, sizeof(usf_t_timesync));
			memcpy(req + USF_AT_DEST, &o4, 4);
			memcpy(req + 84, &ns, 8);
			memcpy(req + 92, &ns, 8);
			printf("akita aoc: usf: the time sync %s\n", usf_request(req, sizeof(usf_t_timesync), buf, sizeof(buf), &in, &il, 1000) ? "answered" : "NOT answered");
		}
		if (o8 != 0) {
			memcpy(req, usf_t_32, sizeof(usf_t_32));
			memcpy(req + USF_AT_DEST, &o8, 4);
			printf("akita aoc: usf: type 32 {1} %s\n", usf_request(req, sizeof(usf_t_32), buf, sizeof(buf), &in, &il, 1000) ? "answered" : "NOT answered");
		}
	}
	(void)usf_registry_push(buf, sizeof(buf));
	/* type 5: the registry's sensors (the info template's shape: no payload); asked again for 30 s while empty */
	for (int again = 0; again < 60 && nids == 0; again++) {
		if (again != 0) {
			delay(2000000);
		}
	memcpy(req, usf_t_info, sizeof(usf_t_info));
	req[USF_AT_TYPE] = 5;
	memcpy(req + USF_AT_DEST, &reg, 4);
	if (!usf_request(req, sizeof(usf_t_info), buf, sizeof(buf), &in, &il, 1000) || !usf_payload(buf + in, il, &p, &plen, &st)) {
		printf("akita aoc: usf: the sensor list NOT answered\n");
		continue;
	}
	{
		uint32_t f = usf_field(p, plen, usf_u32(p, 0), 0), d, n;
		if (usf_vector(p, plen, f, &d, &n)) {
			for (uint32_t k = 0; k < n && k < 64 && d + 4 * k + 4 <= plen; k++) {
				ids[nids++] = usf_u32(p, d + 4 * k);
			}
		}
		if (nids == 0 && again < 2) {
			aoc_say_bytes("usf: the list's answer", buf + in, (int)il);
		}
	}
	}
	printf("akita aoc: usf: %u sensors\n", nids);
	if (nids == 0) {
		/* what the sensor core says of itself: its trace and the debug queue */
		aoc_dump_service("debug");
		aoc_dump_service("trace_a32");
		aoc_dump_service("com.google.umfw_stat");
	}
	for (uint32_t k = 0; k < nids; k++) {
		memcpy(req, usf_t_info, sizeof(usf_t_info));
		memcpy(req + USF_AT_DEST, &ids[k], 4);
		if (!usf_request(req, sizeof(usf_t_info), buf, sizeof(buf), &in, &il, 500) || !usf_payload(buf + in, il, &p, &plen, &st)) {
			printf("akita aoc: usf: sensor %u: no info\n", ids[k]);
			continue;
		}
		uint32_t t = usf_u32(p, 0), nf = usf_field(p, plen, t, 0), tf = usf_field(p, plen, t, 1), pf = usf_field(p, plen, t, 3);
		uint32_t type = tf ? usf_u32(p, tf) : 0, d = 0, n = 0;
		char name[48] = "?";
		if (usf_vector(p, plen, nf, &d, &n) && n < sizeof(name) && d + n <= plen) {
			memcpy(name, p + d, n);
			name[n] = 0;
		}
		uint64_t period = pf ? ((uint64_t)usf_u32(p, pf) | (uint64_t)usf_u32(p, pf + 4) << 32) : 0;
		printf("akita aoc: usf: sensor %u: \"%s\", type %u, fastest %llu us\n", ids[k], name, type, period / 1000);
		if (type == 1 && accel == 0) {
			accel = ids[k];
		} else if (type == 9 && light == 0) {
			light = ids[k];
		} else if (type == 3 && prox == 0) {
			prox = ids[k];
		}
	}
	/* subscribe: the accelerometer at 50 a second, then light and proximity (f0 2 as the brightness sensor's; 3 if
	   refused) */
	const struct { uint32_t id; uint64_t period_ns; const char *what; } subs[3] = {
		{ accel, 20000000ull, "the accelerometer" }, { light, 200000000ull, "the ambient light" }, { prox, 100000000ull, "proximity" } };
	for (int k = 0; k < 3; k++) {
		if (subs[k].id == 0) {
			printf("akita aoc: usf: %s: not in the list\n", subs[k].what);
			continue;
		}
		for (uint32_t f0 = 2; f0 <= 3; f0++) {
			uint32_t cookie = 0x00020000u | subs[k].id, sub = 0;
			memcpy(req, usf_t_subscribe, sizeof(usf_t_subscribe));
			memcpy(req + USF_AT_DEST, &subs[k].id, 4);
			memcpy(req + USF_AT_COOKIE, &cookie, 4);
			memcpy(req + USF_AT_PERIOD, &subs[k].period_ns, 8);
			memcpy(req + USF_AT_F0, &f0, 4);
			bool answered = usf_request(req, sizeof(usf_t_subscribe), buf, sizeof(buf), &in, &il, 1000);
			st = 0;
			if (answered && usf_payload(buf + in, il, &p, &plen, &st)) {
				uint32_t f = usf_field(p, plen, usf_u32(p, 0), 0);
				sub = f ? usf_u32(p, f) : 0;
			} else if (answered) {
				uint32_t root = usf_u32(buf + in, 0), sa = usf_field(buf + in, il, root, 2);
				st = sa ? usf_u32(buf + in, sa) : 0;
			}
			printf("akita aoc: usf: %s (sensor %u) subscribed with f0 %u: %s, subscription %u, status %u\n", subs[k].what, subs[k].id, f0,
			    answered ? "answered" : "NOT answered", sub, st);
			if (sub != 0) {
				break;
			}
		}
	}
	/* the events, for 20 s: the first dozen said, all counted */
	uint32_t before = usf_events;
	for (int t = 0; t < 2000; t++) {
		int n = aoc_read(usf_svc, buf, sizeof(buf), 10);
		if (n < 16 || n > (int)sizeof(buf)) {
			continue;
		}
		const uint8_t *m = buf + 4;
		uint32_t ml = (uint32_t)n - 4, root = usf_u32(m, 0), ka = usf_field(m, ml, root, 0), va = usf_field(m, ml, root, 1), d, dn;
		if (ka && usf_u32(m, ka) == 9 && usf_vector(m, ml, va, &d, &dn) && d + dn <= ml) {
			usf_event(m + d, dn);
		}
	}
	printf("akita aoc: usf: %u events in 20 s (%u before)\n", usf_events - before, before);
	/* ... and from then on for ever: the latest values kept for iOS (kern.hakios.motion) */
	for (uint64_t said = mach_absolute_time();;) {
		int n = aoc_read(usf_svc, buf, sizeof(buf), 20);
		if (n >= 16 && n <= (int)sizeof(buf)) {
			const uint8_t *m = buf + 4;
			uint32_t ml = (uint32_t)n - 4, root = usf_u32(m, 0), ka = usf_field(m, ml, root, 0), va = usf_field(m, ml, root, 1), d, dn;
			if (ka && usf_u32(m, ka) == 9 && usf_vector(m, ml, va, &d, &dn) && d + dn <= ml) {
				usf_event(m + d, dn);
			}
		}
		uint64_t ns;
		absolutetime_to_nanoseconds(mach_absolute_time() - said, &ns);
		if (ns > 300ull * NSEC_PER_SEC) {
			said = mach_absolute_time();
			printf("akita aoc: usf: %u events so far (accelerometer %u, light %u, proximity %u)\n", usf_events,
			    usf_motion.accel_seq, usf_motion.light_seq, usf_motion.proximity_seq);
		}
	}
}

/*
 * THE AoC's SysMMU (11-20, read only): Android's aoc_configure_sysmmu maps the firmware's own table (image_config's
 * sysmmu entries, 7 in this aoc.bin: the carveout 0x98000000 -> 0x94000000 48 MB, 0x9b000000 -> 0x97000000 4 MB, and
 * peripheral windows 0x9e000000 -> 0x15100000, 0x9e200000 -> 0x16400000, 0x9e500000 -> 0x11200000, 0x9e300000 ->
 * 0x40000000, 0x9e380000 -> 0x40400000) into the SysMMU at 0x19090000 (samsung,sysmmu-v9) after every AoC start; this
 * driver does not. Said: whether it is on and where its page table is.
 */
static void
aoc_sysmmu_say(const char *when)
{
	static vm_offset_t mmu;

	if (mmu == 0) {
		mmu = ml_io_map(0x19090000ULL, 0x10000);
	}
	if (mmu == 0) {
		return;
	}
	printf("akita aoc: sysmmu (%s): CTRL 0x%x STATUS 0x%x VERSION 0x%x; VM0 CTRL 0x%x FLPT_BASE 0x%x ATTRIBUTE 0x%x; contexts 0x%x\n",
	    when, *(volatile uint32_t *)(mmu + 0x0), *(volatile uint32_t *)(mmu + 0x8), *(volatile uint32_t *)(mmu + 0x34),
	    *(volatile uint32_t *)(mmu + 0x8000), *(volatile uint32_t *)(mmu + 0x8404), *(volatile uint32_t *)(mmu + 0x8408),
	    *(volatile uint32_t *)(mmu + 0x100));
}

/*
 * ... and set up as Android does (boot-arg aocmmu=1, 11-20): a page table of the firmware's own entries (copied out of
 * its image_config when it is loaded: aoc_sysmmu_table), 1 MB sections where an entry allows them and 4 KB pages
 * elsewhere, then the registers Android's samsung_iommu_v9 was seen to write after an AoC restart (rwmmio trace):
 * FLPT_BASE (+0x8404, the table >> 12), ATTRIBUTE 0x20f84 (+0x8408: the walk non-cacheable), PMMU 0 (+0x2ffc), the
 * default stream 0x10b (+0x2000), 29 streams { 0x10b (the first 0x100010b), 0x301, SID, 0x3f } from +0x2010, CTRL_VM
 * 0x25 (+0x8000: on), invalidate all (+0x8010). Entries: a section (pa >> 4) | 2 | read 1<<4 | write 1<<5; a second
 * level (its table >> 4) | 1, small pages (pa >> 4) | 2 | read 1<<2 | write 1<<3.
 */
#define AOC_MMU_ENTRIES_MAX     16
static uint64_t aoc_sysmmu_table[AOC_MMU_ENTRIES_MAX];
static uint32_t aoc_sysmmu_count;

static bool
aoc_sysmmu_setup(void)
{
	static const uint8_t sids[] = { 0x2, 0x3, 0x4, 0x5, 0x6, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
	    0x26, 0x27, 0x28, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38 };
	static vm_offset_t mmu, l1, l2pool;
	static uint32_t l2used;
	const uint32_t L1_BYTES = 65536 * 4, L2_BYTES = 1024, L2_POOL = 64 * 1024;

	if (aoc_sysmmu_count == 0) {
		printf("akita aoc: sysmmu: the firmware gave no table\n");
		return false;
	}
	if (mmu == 0) {
		mmu = ml_io_map(0x19090000ULL, 0x10000);
	}
	if (mmu == 0 || (*(volatile uint32_t *)(mmu + 0x34) >> 28) != 0x9) {
		printf("akita aoc: sysmmu: not answering (version 0x%x): left alone\n", mmu ? *(volatile uint32_t *)(mmu + 0x34) : 0);
		return false;
	}
	if (l1 == 0 && (kmem_alloc_contig(kernel_map, &l1, L1_BYTES, L1_BYTES - 1, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_PERMANENT,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS || kmem_alloc_contig(kernel_map, &l2pool, L2_POOL, PAGE_SIZE - 1, 0, 0,
	    KMA_KOBJECT | KMA_ZERO | KMA_PERMANENT, VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS)) {
		printf("akita aoc: sysmmu: no memory for its table\n");
		l1 = 0;
		return false;
	}
	volatile uint32_t *t1 = (volatile uint32_t *)l1;
	for (uint32_t i = 0; i < aoc_sysmmu_count; i++) {
		uint64_t v = aoc_sysmmu_table[i];
		uint64_t va = (v & 0xfffff) << 12, pa = ((v >> 20) & 0xffffff) << 12, size = ((v >> 44) & 0xfffff) << 12;

		printf("akita aoc: sysmmu: 0x%llx -> 0x%llx, 0x%llx bytes\n", va, pa, size);
		for (uint64_t off = 0; off < size;) {
			uint64_t a = va + off, b = pa + off;
			uint32_t i1 = (uint32_t)(a >> 20);

			if (((a | b) & 0xfffff) == 0 && size - off >= (1u << 20)) {
				t1[i1] = (uint32_t)(b >> 4) | 2 | (1u << 4) | (1u << 5);
				off += 1u << 20;
				continue;
			}
			if ((t1[i1] & 7) != 1) {        /* a second-level table for this megabyte */
				if (l2used + L2_BYTES > L2_POOL) {
					printf("akita aoc: sysmmu: out of second-level tables\n");
					return false;
				}
				t1[i1] = (uint32_t)(kvtophys(l2pool + l2used) >> 4) | 1;
				l2used += L2_BYTES;
			}
			volatile uint32_t *t2 = (volatile uint32_t *)(l2pool + ((((uint64_t)(t1[i1] & ~0x3fu)) << 4) - kvtophys(l2pool)));
			t2[(a >> 12) & 0xff] = (uint32_t)(b >> 4) | 2 | (1u << 2) | (1u << 3);
			off += 1u << 12;
		}
	}
	for (vm_offset_t line = l1; line < l1 + L1_BYTES; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	for (vm_offset_t line = l2pool; line < l2pool + L2_POOL; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__builtin_arm_dsb(0xf);
	wr(mmu, 0x8404, (uint32_t)(kvtophys(l1) >> 12));
	wr(mmu, 0x8408, 0x20f84);
	wr(mmu, 0x2ffc, 0);
	wr(mmu, 0x2000, 0x10b);
	for (uint32_t i = 0; i < sizeof(sids); i++) {
		wr(mmu, 0x2010 + 16 * i, i == 0 ? 0x100010b : 0x10b);
		wr(mmu, 0x2014 + 16 * i, 0x301);
		wr(mmu, 0x2018 + 16 * i, sids[i]);
		wr(mmu, 0x201c + 16 * i, 0x3f);
	}
	wr(mmu, 0x8000, rd(mmu, 0x8000) | 1);
	wr(mmu, 0x8010, 1);
	__builtin_arm_dsb(0xf);
	aoc_sysmmu_say("set up");
	return true;
}

static void
aoc_usf_thread(__unused void *arg, __unused wait_result_t wr_)
{
	aoc_sysmmu_say("before the sensors' probe");
	usf_probe();
}


/*
 * THE SENSORS' SUPPLIES (11-16): the device tree's aoc node names them -- sensor_1v8-supply = the S2MPG15's LDO7S
 * ("L7S_SENSORS", 1.8 V), sensor_3v3-supply = LDO5S ("L5S_PROX") -- and Google's aoc driver turns them on at its probe
 * and power-cycles them (off, 150 ms, on) before every start of the AoC (reset_sensor_power). Their control registers
 * (s2mpg15-register.h: L5S_CTRL 0x33, L7S_CTRL 0x35, enable bits 7:6) through ACPM's PMIC protocol, as the camera
 * driver reads its L12S (bus 1, type 1 the PM block). usf=1 only reads them; usf=2 also power-cycles them before the
 * firmware is loaded.
 */
extern int akita_acpm_xfer(unsigned channel, uint32_t cmd[4]);
#define SENSOR_L5S_CTRL 0x33
#define SENSOR_L7S_CTRL 0x35

static bool
sensor_pmic(bool write, unsigned reg, uint8_t *value)
{
	uint32_t cmd[4] = { 1u << 12 | 1u << 8 | (reg & 0xff), (write ? 1u : 0u) | (uint32_t)*value << 8 | 0xffu << 16, 0, 0 };

	if (akita_acpm_xfer(2, cmd) != 0 || (cmd[1] >> 24) != 0) {
		return false;
	}
	if (!write) {
		*value = (uint8_t)(cmd[1] >> 8);
	}
	return true;
}

static void
sensor_power(const char *when, bool cycle)
{
	uint8_t l5 = 0, l7 = 0;
	bool ok = sensor_pmic(false, SENSOR_L5S_CTRL, &l5) && sensor_pmic(false, SENSOR_L7S_CTRL, &l7);

	printf("akita aoc: sensors' supplies %s: %s L5S_CTRL %#x (enable %u), L7S_CTRL %#x (enable %u)\n", when,
	    ok ? "read:" : "NOT READ:", l5, l5 >> 6, l7, l7 >> 6);
	if (!ok || !cycle) {
		return;
	}
	uint8_t off5 = l5 & 0x3f, off7 = l7 & 0x3f, on5 = (l5 >> 6) ? l5 : (uint8_t)(l5 | 0xc0), on7 = (l7 >> 6) ? l7 : (uint8_t)(l7 | 0xc0);

	ok = sensor_pmic(true, SENSOR_L7S_CTRL, &off7) && sensor_pmic(true, SENSOR_L5S_CTRL, &off5);
	delay(150000);
	ok = sensor_pmic(true, SENSOR_L5S_CTRL, &on5) && sensor_pmic(true, SENSOR_L7S_CTRL, &on7) && ok;
	delay(10000);
	l5 = l7 = 0;
	sensor_pmic(false, SENSOR_L5S_CTRL, &l5);
	sensor_pmic(false, SENSOR_L7S_CTRL, &l7);
	printf("akita aoc: sensors' supplies power-cycled (%s): now L5S_CTRL %#x, L7S_CTRL %#x\n", ok ? "written" : "SOME WRITES FAILED",
	    l5, l7);
}

static void aoc_boot_image(const uint8_t *f, uint64_t size);

/* the AoC's firmware loaded by the GSA and started through Trusty (aoc=3): the RAM disk's, else the data volume's */
static void
aoc_boot(void)
{
	uint64_t size = 0;
	const uint8_t *f = hakios_image_file("fw", "aoc.bin", &size, NULL);
	uint8_t *disk = NULL;

	if (f == NULL || size < AOC_AUTH_HEADER_SIZE + 0x1000) {
		printf("akita aoc: no fw/aoc.bin in the image; looking on the disk\n");
		if ((disk = aoc_disk_file(&size)) == NULL) {
			return;
		}
		f = disk;
	}
	{
		uint32_t usf = 0;

		if (PE_parse_boot_argn("usf", &usf, sizeof(usf)) && usf != 0) {
			sensor_power("before the AoC's start", usf >= 2);
		}
	}
	aoc_boot_image(f, size);
	if (disk != NULL) {
		kfree_data(disk, size);         /* (the GSA works from the carveout and the header page: the file is done with) */
	}
}

static void
aoc_boot_image(const uint8_t *f, uint64_t size)
{
	uint32_t state = 0xff;

	uint32_t auth_magic = *(const uint32_t *)(f + 1024), generation = *(const uint32_t *)(f + 1028);
	const uint8_t *cfg = f + 1024 + (generation == 2 ? 384 : 352);         /* header_v2: a 64-byte hash */
	{
		/* the SysMMU's entries (image_config: +20 u16 sysmmu offset, +22 its size, 8 bytes an entry; aoc_sysmmu_setup) */
		uint16_t so = *(const uint16_t *)(cfg + 20), ss = *(const uint16_t *)(cfg + 22);

		aoc_sysmmu_count = 0;
		if (so < 256 && so + ss <= 256 && ss % 8 == 0) {
			for (uint32_t i = 0; i < ss / 8 && i < AOC_MMU_ENTRIES_MAX; i++) {
				memcpy(&aoc_sysmmu_table[aoc_sysmmu_count++], cfg + so + 8 * i, 8);
			}
		}
	}
	uint32_t fw_start = *(const uint32_t *)(cfg + 4), fw_size = *(const uint32_t *)(cfg + 8);
	const uint8_t *sb = f + AOC_AUTH_HEADER_SIZE;
	uint32_t sb_magic = *(const uint32_t *)sb, image_size = *(const uint32_t *)(sb + 12);
	char version[65];

	memcpy(version, sb + 48, 64);
	version[64] = 0;
	printf("akita aoc: fw/aoc.bin: %llu bytes, auth magic 0x%x generation %u, locked at 0x%x + 0x%x; superbin 0x%x, image 0x%x, "
	    "\"%s\"\n", size, auth_magic, generation, fw_start, fw_size, sb_magic, image_size, version);
	if (auth_magic != 0x00434f41 || sb_magic != 0xaabbccddu || fw_start != AOC_DRAM_PHYS ||
	    image_size + 0x10000 > AOC_DRAM_SIZE || size - AOC_AUTH_HEADER_SIZE > image_size) {
		printf("akita aoc: not an image this driver knows how to load\n");
		return;
	}
	if (akita_trusty_aoc_cmd(0, &state) != 0) {
		printf("akita aoc: the GSA's AoC service not reached (Trusty): not loaded\n");
		return;
	}
	printf("akita aoc: the AoC's state %u; copying the image ...\n", state);
	delay(300000);
	if (state != 0) {
		printf("akita aoc: the AoC is not inactive (%u): not loaded again\n", state);
		return;
	}
	/* the body at the carveout's start (aligned 64-bit stores, no memcpy/bzero into it), the IPC block's area cleared */
	vm_offset_t dram = carveout_map();
	if (!dram) {
		printf("akita aoc: the carveout could not be mapped\n");
		return;
	}
	uint64_t body = size - AOC_AUTH_HEADER_SIZE;
	const uint8_t *src = f + AOC_AUTH_HEADER_SIZE;
	uint64_t i = 0;
	for (; i + 8 <= body; i += 8) {
		uint64_t w;
		__builtin_memcpy(&w, src + i, 8);
		*(volatile uint64_t *)(dram + i) = w;
		if ((i & ((8u << 20) - 1)) == 0 && i != 0) {
			printf("akita aoc: ... %llu MB\n", i >> 20);
		}
	}
	for (; i < body; i++) {
		*(volatile uint8_t *)(dram + i) = src[i];
	}
	for (uint32_t k = 0; k < 0x10000; k += 8) {
		*(volatile uint64_t *)(dram + image_size + k) = 0;
	}
	__builtin_arm_dsb(0xf);
	/* the header in a page of ours, cleaned to memory for the GSA to read */
	vm_offset_t hdr = 0;
	if (kmem_alloc_contig(kernel_map, &hdr, PAGE_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		printf("akita aoc: no memory for the header\n");
		return;
	}
	memcpy((void *)hdr, f, AOC_AUTH_HEADER_SIZE);
	for (vm_offset_t line = hdr; line < hdr + PAGE_SIZE; line += 64) {
		__asm__ volatile ("dc civac, %0" : : "r" (line));
	}
	__builtin_arm_dsb(0xf);
	uint64_t hdr_pa = kvtophys(hdr);
	uint32_t args[4] = { (uint32_t)hdr_pa, (uint32_t)(hdr_pa >> 32), (uint32_t)AOC_DRAM_PHYS, (uint32_t)(AOC_DRAM_PHYS >> 32) };
	uint32_t rsp[MBOX_SR_NUM], n = 0, err = 0;
	uint64_t t0 = mach_absolute_time(), ns;

	printf("akita aoc: the image copied (%llu MB); the GSA asked to authenticate and load it (header at 0x%llx)\n",
	    (size - AOC_AUTH_HEADER_SIZE) >> 20, hdr_pa);
	delay(300000);
	if (gsa_request_ms(GSA_MB_CMD_LOAD_AOC_FW_IMG, args, 4, rsp, &n, &err, 20000) != 0 || err != 0) {
		printf("akita aoc: THE GSA REFUSED THE IMAGE: error %u (1 invalid args, 2 auth failed, 3 busy, 4 running, 5 resources)\n",
		    err);
		return;
	}
	absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
	printf("akita aoc: THE GSA LOADED THE IMAGE in %llu ms\n", ns / 1000000);
	/* the firmware's parameters at the IPC block (aoc_pass_fw_information), as Android passes them */
	uint32_t aoc_mm_on = 0;
	PE_parse_boot_argn("aocmm", &aoc_mm_on, sizeof(aoc_mm_on));
	vm_offset_t chip = ml_io_map(CHIPID_PHYS, 0x1000);
	uint32_t chip_type = chip ? rd(chip, 0) & 0xf : 0, r10 = chip ? rd(chip, 0x10) : 0;
	uint32_t chip_rev = (((r10 >> 20) & 0xf) << 4) | ((r10 >> 16) & 0xf);
	uint32_t top = AOC_DRAM_SIZE, sensor = top - (4u << 20), playback = sensor - 0x4000, capture = playback - 0x10000;
	const uint32_t kv[][2] = {
		{ 0x1001, 0x40402 }, { 0x1002, 0x10000 },               /* the board (aoc-board-id, -rev: "AK3") */
		{ 0x1003, 0 },                                          /* SRAM repaired */
		{ 0x1005, (uint32_t)AOC_DRAM_PHYS }, { 0x1006, AOC_DRAM_SIZE },
		{ 0x1007, AOC_BINARY_DRAM_BASE + sensor }, { 0x1008, 4u << 20 },
		{ 0x100C, AOC_BINARY_DRAM_BASE + playback }, { 0x100D, 0x4000 },
		{ 0x100E, AOC_BINARY_DRAM_BASE + capture }, { 0x100F, 0x10000 },
		/*
		 * (11-20) AS ANDROID PASSES THEM (its aoc_pass_fw_information traced at an AoC restart, rwmmio): 0x100A
		 * kAOCDisableMM = 1 -- the AoC's monitor mode OFF ("AoC Monitor Mode disabled"); it was 0 here, monitor mode
		 * on, and the sensor core registered no sensor at all (USF's list empty, TODO 34) --, 0x1015
		 * kAOCVolteReleaseMif = 1, 0x1016 kAOCChipProductId (0x9865000), 0x1017 kAOCWifiChip (0). aocmm=1: as before.
		 */
		{ 0x1009, 0 }, { 0x100A, aoc_mm_on ? 0u : 1u }, { 0x100B, 0 }, { 0x1010, 0 },     /* VNOM, monitor mode off, UART, ultrasonic */
		{ 0x1011, (uint32_t)mach_absolute_time() },             /* a seed */
		/* (the chip's revision as Android passes it on this phone, 0x11: read from CHIPID here it came out 0x1) */
		{ 0x1012, chip_rev == 0x1 ? 0x11u : chip_rev }, { 0x1013, chip_type }, { 0x1014, 1 }, { 0x1015, aoc_mm_on ? 0u : 1u },   /* the chip, gnss-type 1, VoLTE MIF */
		{ 0x1016, 0x9865000 }, { 0x1017, 0 },                   /* the SoC's product id, the Wi-Fi chip (as Android) */
	};
	volatile uint32_t *p = (volatile uint32_t *)(dram + image_size);
	uint32_t num = sizeof(kv) / sizeof(kv[0]);
	p[0] = AOC_PARAMETER_MAGIC;
	p[1] = num;
	p[2] = 12 + num * 12;
	for (uint32_t i = 0; i < num; i++) {
		p[3 + 3 * i] = kv[i][0];
		p[4 + 3 * i] = 4;
		p[5 + 3 * i] = kv[i][1];
	}
	__builtin_arm_dsb(0xf);
	printf("akita aoc: parameters written (chip type %u revision 0x%x); the AoC asked on (aoc_req), started through the GSA\n",
	    chip_type, chip_rev);
	delay(300000);
	wr(aoc_req, 0, 1);
	{
		uint32_t mmu_on = 0;

		PE_parse_boot_argn("aocmmu", &mmu_on, sizeof(mmu_on));
		if (mmu_on) {
			delay(1000);
			(void)aoc_sysmmu_setup();
		}
	}
	if (akita_trusty_aoc_cmd(1, &state) != 0) {         /* GSA_AOC_START */
		printf("akita aoc: GSA_AOC_START refused\n");
		return;
	}
	for (int ms = 0; ms < 10000; ms += 100) {
		if (rd(dram, image_size) == AOC_MAGIC) {
			char fw[33] = "";
			for (int i = 0; i < 32; i++) {
				fw[i] = (char)*(volatile uint8_t *)(dram + image_size + 36 + i);
				if (fw[i] && (fw[i] < 32 || fw[i] > 126)) {
					fw[i] = 0;
					break;
				}
			}
			printf("akita aoc: THE AoC IS UP after %d ms: its IPC block, fw 0x%x hw 0x%x status 0x%x, %u services of %u bytes at "
			    "+0x%x, \"%s\"\n", ms, rd(dram, image_size + 4), rd(dram, image_size + 8), rd(dram, image_size + 12),
			    rd(dram, image_size + 24), rd(dram, image_size + 28), rd(dram, image_size + 32), fw);
			/* its services by name (aoc-ipc's struct aoc_ipc_service_header: name[32], flags, two rings), a line of a few */
			uint32_t count = rd(dram, image_size + 24), each = rd(dram, image_size + 28), at = rd(dram, image_size + 32);
			char line[200];
			int len = 0;
			for (uint32_t k = 0; k < count && k < 128 && each >= 32; k++) {
				char nm[33];
				for (int c = 0; c < 32; c++) {
					nm[c] = (char)*(volatile uint8_t *)(dram + image_size + at + k * each + c);
				}
				nm[32] = 0;
				if (len + (int)strlen(nm) + 3 > (int)sizeof(line) - 1) {
					printf("akita aoc: services:%s\n", line);
					len = 0;
				}
				len += snprintf(line + len, sizeof(line) - len, " %s", nm);
			}
			if (len) {
				printf("akita aoc: services:%s\n", line);
			}
			ipc_base = dram + image_size;
			ipc_room = AOC_DRAM_SIZE - image_size;
			aoc_talk();
			{
				/* (aocvol_probe=1: the volume command once, no stream -- does the AoC take it? silent) */
				uint32_t probe = 0;

				if (PE_parse_boot_argn("aocvol_probe", &probe, sizeof(probe)) && probe != 0) {
					/* the AoC's volume for entry point 0 at the speaker before and after (GET_PARAMETER 211) */
					for (int pass = 0; pass < 2; pass++) {
						uint8_t g[18] = { 0 };
						uint32_t key = 0, val = ~0u;

						g[8] = 16;
						memcpy(g + 10, &key, 4);
						if (aoc_out_cmd(pass == 0 ? "GET_PARAMETER (volume, as found)" : "GET_PARAMETER (volume, set)",
						    g, sizeof(g), 211)) {
							memcpy(&val, g + 14, 4);
							printf("akita aoc: the AoC's volume for entry point 0 at the speaker: %u (of 1000)%s\n",
							    val, pass == 0 ? ", as found" : ", after ours");
						}
						if (pass == 0) {
							(void)aoc_volume(0, 0);
						}
					}
				}
			}
			PE_parse_boot_argn("aocsink", &aocsink, sizeof(aocsink));
			PE_parse_boot_argn("aocasp", &aocasp, sizeof(aocasp));
			PE_parse_boot_argn("aocep", &aocep, sizeof(aocep));
			if (aocep > 6) {
				aocep = 0;
			}
			PE_parse_boot_argn("aoclog", &aoclog, sizeof(aoclog));
			if (aoclog) {
				thread_t lt;
				char nm[24];

				/* where each playback ring is, both ways (11-12: aocep=1 froze the phone) -- said before anything
				 * touches them, the log given time to get out */
				for (uint32_t k = 0; k <= 6; k++) {
					snprintf(nm, sizeof(nm), "audio_playback%u", k);
					vm_offset_t v = aoc_service(nm);
					if (v) {
						vm_offset_t dn = region(v, AOC_DOWN), up = region(v, AOC_UP);
						printf("akita aoc: %s: flags 0x%x type %u; down %u bytes at +0x%x slots %u tx %u rx %u; up %u at "
						    "+0x%x; room 0x%x\n", nm, rd(v, 32), (rd(v, 32) >> 16) & 7, rd(dn, 4), rd(dn, 0), rd(dn, 8),
						    rd(dn, 12), rd(dn, 16), rd(up, 4), rd(up, 0), ipc_room);
					}
				}
				delay(500000);

				if (kernel_thread_start_priority(aoc_log_thread, NULL, 80, &lt) == KERN_SUCCESS) {
					thread_deallocate(lt);
				}
				delay(300000);                  /* (the boot's 256 messages out before ours start) */
			}
			bool mic = false;
			uint32_t play = 0;

			PE_parse_boot_argn("aocplay", &play, sizeof(play));
			if (PE_parse_boot_argn("aocmic", &aocmic, sizeof(aocmic)) && aocmic != 0 && aoc_mic_start()) {
				thread_t mt;

				mic_done = false;
				mic_self_stop = play == 0;
				mic = kernel_thread_start_priority(aoc_mic_thread, NULL, 90, &mt) == KERN_SUCCESS;
				if (mic) {
					thread_deallocate(mt);
					delay(500000);          /* (half a second of the room before the tone) */
				}
			}
			if (PE_parse_boot_argn("aocplay", &aocplay, sizeof(aocplay)) && aocplay != 0) {
				PE_parse_boot_argn("aocplay_amp", &aocplay_amp, sizeof(aocplay_amp));
				if (aocplay_amp > 32767) {
					aocplay_amp = 32767;
				}
				PE_parse_boot_argn("aocplay_lr", &aocplay_lr, sizeof(aocplay_lr));
				PE_parse_boot_argn("aocplay_hz", &aocplay_hz, sizeof(aocplay_hz));
				if (aocplay_hz < 20 || aocplay_hz > 20000) {
					aocplay_hz = 440;
				}
				PE_parse_boot_argn("aocplay_sweep", &aocplay_sweep, sizeof(aocplay_sweep));
				PE_parse_boot_argn("aocplay_matrix", &aocplay_matrix, sizeof(aocplay_matrix));
				if (aocplay_matrix) {
					printf("akita aoc: aocplay_matrix: THE LISTENING TEST -- for each amplifier setting K (1-%u) K beeps, "
					    "twice, in %u s\n", MATRIX_N, MATRIX_SEG / 48000);
				}
				printf("akita aoc: aocplay: a %u Hz tone for %u s, %s%s\n", aocplay_hz, aocplay, aocplay_sweep == 2 ?
				    "1.5 s each at -30 -24 -18 -12 -9 -6 dBFS, then the same at 440 Hz" : aocplay_sweep ?
				    "1.5 s each at -30 -24 -18 -12 -9 -6 dBFS" : "peak as aocplay_amp", aocplay_lr ?
				    ", left only and right only by turns (left first)" : ", both sides");
				aoc_play();
			}
			if (mic && !mic_self_stop) {
				while (!mic_done) {
					delay(100000);
				}
				aoc_mic_stop();
			}
			if (PE_parse_boot_argn("aocrec", &aocrec, sizeof(aocrec)) && aocrec != 0) {
				aoc_darwin_mic_init();
			}
			if (!PE_parse_boot_argn("aocsound", &aocsound, sizeof(aocsound))) {
				uint32_t speaker = 0;

				/* (speaker=1 -- the speakers wanted, drivers/audio -- means Darwin's sound there: aocsound=1 implied,
				 * 11 characters of a command line that has few to spare) */
				if (PE_parse_boot_argn("speaker", &speaker, sizeof(speaker)) && speaker != 0) {
					aocsound = 1;
				}
			}
			{
				uint32_t usf = 0;

				if (PE_parse_boot_argn("usf", &usf, sizeof(usf)) && usf != 0) {
					thread_t ut;

					if (kernel_thread_start_priority(aoc_usf_thread, NULL, 80, &ut) == KERN_SUCCESS) {
						thread_deallocate(ut);
					}
				}
			}
			if (aocsound != 0) {
				thread_t t;

				PE_parse_boot_argn("aocsound_gain", &aocsound_gain, sizeof(aocsound_gain));
				if (aocsound_gain > HAKIOS_GUARD_MAX_GAIN) {
					aocsound_gain = HAKIOS_GUARD_MAX_GAIN;
				}
				/* (the touch screen's priority: BASEPRI_PREEMPT_HIGH, so a busy system does not starve the AoC's ring) */
				if (kernel_thread_start_priority(aoc_sound_thread, NULL, 93, &t) == KERN_SUCCESS) {
					thread_deallocate(t);
				}
			}
			return;
		}
		delay(100000);
	}
	printf("akita aoc: no IPC block in 10 s (word 0x%x, state %u)\n", rd(dram, image_size), state);
}

static void
akita_aoc_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	uint32_t rsp[MBOX_SR_NUM], n = 0, err = 0;

	printf("akita aoc: aoc_req 0x%x, the AoC's state 0x%x\n", rd(aoc_req, 0), rd(aoc_req, 0x40));
	printf("akita aoc: GSA mailbox: INTMR0 0x%x INTSR0 0x%x INTMSR0 0x%x; SR0-3 0x%x 0x%x 0x%x 0x%x\n", rd(gsa, MBOX_INTMR0),
	    rd(gsa, MBOX_INTSR0), rd(gsa, MBOX_INTMSR0), rd(gsa, MBOX_SR(0)), rd(gsa, MBOX_SR(1)), rd(gsa, MBOX_SR(2)),
	    rd(gsa, MBOX_SR(3)));
	if (gsa_request(GSA_MB_CMD_GET_CHIP_ID, NULL, 0, rsp, &n, &err) == 0) {
		printf("akita aoc: THE GSA ANSWERS: chip id (%u words) %08x %08x %08x %08x\n", n, n > 0 ? rsp[0] : 0, n > 1 ? rsp[1] : 0,
		    n > 2 ? rsp[2] : 0, n > 3 ? rsp[3] : 0);
	}
	/*
	 * The carveout: the IPC block a running firmware keeps, 6 MB in (aoc.c's default when it loads no firmware). ONE
	 * word, and only there: the firmware body the GSA authenticated (from the carveout's start) is LOCKED against every
	 * other master -- the first version of this probe read the whole carveout from 0 and the kernel died at once (the
	 * phone came back in its flashed slot, 11-11). aoc=2: the look at 6 MB.
	 */
	if (mode == 5) {
		/* aoc=5 (a test, 11-11): may the CPU WRITE the carveout? aoc=3 died at its first copy; reads are fine. One word
		 * at the start, at the IPC block (28 MB) and in the heaps (47 MB), said before, read back after */
		const uint32_t at[3] = { 0, 28, 47 };
		for (int i = 0; i < 3; i++) {
			vm_offset_t pg = ml_io_map(AOC_DRAM_PHYS + ((uint64_t)at[i] << 20), 0x1000);
			printf("akita aoc: writing a word at +%u MB (was 0x%08x) ...\n", at[i], pg ? rd(pg, 0x100) : 0);
			delay(300000);
			if (pg) {
				wr(pg, 0x100, 0x5a5a1234u + (uint32_t)i);
				__builtin_arm_dsb(0xf);
			}
			printf("akita aoc: +%u MB written: reads back 0x%08x\n", at[i], pg ? rd(pg, 0x100) : 0);
			delay(300000);
		}
		return;
	}
	if (mode == 4) {
		/*
		 * aoc=4 (a test, 11-11): which of the carveout the CPU may touch -- aoc=3's copy and the first probe's read both
		 * killed the kernel, one word 6 MB in did not. One word a megabyte, said BEFORE it is read, slowly (the log
		 * must get out before an abort does)
		 */
		for (uint32_t mb = 0; mb < (AOC_DRAM_SIZE >> 20); mb++) {
			printf("akita aoc: reading +%u MB ...\n", mb);
			delay(150000);
			vm_offset_t pg = ml_io_map(AOC_DRAM_PHYS + ((uint64_t)mb << 20), 0x1000);
			uint32_t w = pg ? rd(pg, 0) : 0xdeadbeef;
			printf("akita aoc: +%u MB: 0x%08x\n", mb, w);
		}
		printf("akita aoc: the whole carveout read, one word a megabyte\n");
		return;
	}
	if (mode >= 3) {
		aoc_boot();
		return;
	}
	if (mode < 2) {
		return;
	}
	vm_offset_t ipc = ml_io_map(AOC_DRAM_PHYS + (6u << 20), 0x1000);

	if (!ipc) {
		printf("akita aoc: 6 MB into the carveout could not be mapped\n");
		return;
	}
	uint32_t magic = rd(ipc, 0);

	printf("akita aoc: 6 MB into the carveout: 0x%08x%s\n", magic, magic == AOC_MAGIC ? " -- AN IPC BLOCK" : "");
	if (magic == AOC_MAGIC) {
		char fw[33] = "";

		for (int i = 0; i < 32; i++) {
			fw[i] = (char)*(volatile uint8_t *)(ipc + 36 + i);
			if (fw[i] && (fw[i] < 32 || fw[i] > 126)) {
				fw[i] = 0;
				break;
			}
		}
		printf("akita aoc: fw_version 0x%x hw_version 0x%x status 0x%x, %u services of %u bytes at +0x%x; \"%s\"\n",
		    rd(ipc, 4), rd(ipc, 8), rd(ipc, 12), rd(ipc, 24), rd(ipc, 28), rd(ipc, 32), fw);
	}
}

__startup_func
static void
akita_aoc_init(void)
{
	static thread_call_t start;
	uint64_t deadline;

	if (!PE_parse_boot_argn("aoc", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	aoc_req = ml_io_map(AOC_REQ_PHYS, 0x1000);
	gsa = ml_io_map(GSA_MBOX_PHYS, 0x1000);
	if (!aoc_req || !gsa) {
		printf("akita aoc: could not map its registers\n");
		return;
	}
	start = thread_call_allocate(akita_aoc_start, NULL);
	/* (aoc=3 at 25 s: after the USB log is up -- a load that killed the kernel at 3 s left no line of it, 11-11) */
	clock_interval_to_deadline(mode >= 3 ? 25 : 3, NSEC_PER_SEC, &deadline);     /* (aoc=4 and 5 too) */
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_aoc_init);
