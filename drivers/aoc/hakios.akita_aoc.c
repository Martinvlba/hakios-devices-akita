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

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_map_address_t pmap_map(vm_map_address_t virt, vm_offset_t start, vm_offset_t end, vm_prot_t prot, unsigned int flags);
extern uint64_t kvtophys(vm_offset_t va);
extern const uint8_t *hakios_image_file(const char *dir, const char *name, uint64_t *size, int *disk_out);
extern int akita_trusty_aoc_cmd(uint32_t cmd, uint32_t *state);
extern int akita_amp_speaker(int on) __attribute__((weak));       /* drivers/audio (amp=2): the CS35L41s */
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

static bool
aoc_out_cmd(const char *what, void *cmd, uint32_t len, uint16_t id)
{
	*(uint16_t *)((uint8_t *)cmd + 4) = id;
	int got = aoc_audio_cmd("audio_output_control", cmd, len);
	printf("akita aoc: %s (%u): %s, reply field %d\n", what, id, got < 0 ? "FAILED" : "done", got < 0 ? 0 :
	    *(int16_t *)((uint8_t *)cmd + 6));
	return got >= 0;
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

/* a 440 Hz tone at about -24 dBFS (amplitude 2000), frame FRAME onwards, BYTES of it at ring offset AT (wrapping) */
static void
aoc_tone(vm_offset_t ring, uint32_t size, uint32_t at, uint32_t bytes, uint64_t *frame)
{
	/* one period of 440 Hz at 48 kHz is 109.09 frames: a 16-entry quarter table interpolated is plenty for a test tone */
	static const int16_t q[17] = { 0, 196, 390, 581, 765, 942, 1111, 1269, 1414, 1546, 1663, 1764, 1848, 1914, 1962, 1990,
		                       2000 };
	for (uint32_t i = 0; i + 4 <= bytes; i += 4) {
		uint32_t ph = (uint32_t)((*frame * 440ULL * 64ULL / 48000ULL) % 64);    /* 64 steps a period */
		int16_t v = ph < 16 ? q[ph] : ph < 32 ? q[32 - ph] : ph < 48 ? (int16_t)-q[ph - 32] : (int16_t)-q[64 - ph];
		uint32_t o = (at + i) % size;
		*(volatile uint32_t *)(ring + o) = (uint16_t)v | ((uint32_t)(uint16_t)v << 16);
		(*frame)++;
	}
}

static void
aoc_play(void)
{
	vm_offset_t svc = aoc_service("audio_playback0");

	if (!svc) {
		printf("akita aoc: no audio_playback0\n");
		return;
	}
	vm_offset_t r = region(svc, AOC_DOWN);
	uint32_t size = rd(r, 4), off = rd(r, 0);
	printf("akita aoc: audio_playback0: flags 0x%x (type %u, mailbox %u), down: %u bytes at +0x%x, slots %u, tx %u rx %u wp %u rp %u\n",
	    rd(svc, 32), (rd(svc, 32) >> 16) & 7, (rd(svc, 32) >> 19) & 0x1f, size, off, rd(r, 8), rd(r, 12), rd(r, 16), rd(r, 20),
	    rd(r, 24));
	if (size < 7680 || ((rd(svc, 32) >> 16) & 7) != 1) {        /* (15360 here: four 20 ms periods) */
		printf("akita aoc: audio_playback0 is not the ring expected; no sound\n");
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
	ep[25] = 0;                                     /* channel: entry point 0 */
	ep[26] = 1;                                     /* wraparound */
	ep[27] = 0;                                     /* ENTRYPOINT_MODE_PLAYBACK */
	uint8_t ep2[18] = { 0 };                        /* struct CMD_AUDIO_OUTPUT_EP_SETUP2 */
	ep2[8] = 0;
	*(uint32_t *)(ep2 + 9) = buffer;
	*(uint32_t *)(ep2 + 13) = period;
	ep2[17] = 0;
	uint8_t bind[11] = { 0 };                       /* struct CMD_AUDIO_OUTPUT_BIND: src, dst, bind */
	bind[8] = 0;
	bind[9] = 0;                                    /* SINK_SPEAKER */
	bind[10] = 1;
	uint8_t src[10] = { 0 };                        /* struct CMD_AUDIO_OUTPUT_SOURCE: source, on */
	src[8] = 0;
	src[9] = 1;

	if (!aoc_out_cmd("EP_SETUP", ep, sizeof(ep), 206) || !aoc_out_cmd("EP_SETUP2", ep2, sizeof(ep2), 298) ||
	    !aoc_out_cmd("BIND2 to the speaker", bind, sizeof(bind), 271)) {
		return;
	}

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
	aoc_out_state();

	uint32_t rx0 = rd(r, 16);
	uint64_t t0 = mach_absolute_time(), ns;
	bool amps = false;
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
			amps = akita_amp_speaker(1) == 0;
		}
		if (ms % 1000 == 990) {
			absolutetime_to_nanoseconds(mach_absolute_time() - t0, &ns);
			printf("akita aoc: playing: the AoC has read %u bytes in %llu ms (%llu bytes/s; 192000 is 48 kHz stereo 16-bit), "
			    "queued %u\n", rx - rx0, ns / 1000000, ns ? (uint64_t)(rx - rx0) * 1000000000ULL / ns : 0, queued);
		}
	}
	if (amps) {
		akita_amp_speaker(0);
	}
	src[9] = 0;
	aoc_out_cmd("SOURCE off", src, sizeof(src), 201);
	bind[10] = 0;
	aoc_out_cmd("BIND2 off", bind, sizeof(bind), 271);
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
 * seconds of silence, and off while Darwin's volume is 0. Their level is fixed at the CS35L41's least analog gain (0.5
 * dB): a full-scale sample is about a volt at the speaker, which no phone speaker minds -- "the loudest" for the guard,
 * which then adds aocsound_gain eighths of a dB (default 96, at most 96) before its limits. */

#define DRING_FRAMES            32768           /* 0.68 s; a power of two, the indices run on and wrap */
#define SILENCE                 8
#define BLOCK_FRAMES            960             /* 20 ms */
#define BLOCK_BYTES             (BLOCK_FRAMES * 4)
#define QUIET_BEFORE_OFF_US     3000000ull

static uint32_t aocsound, aocsound_gain = HAKIOS_GUARD_MAX_GAIN;
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
	ep[20] = 2; ep[21] = 1; ep[22] = 5; ep[23] = 1; ep[25] = 0; ep[26] = 1; ep[27] = 0;     /* as aoc_play's */
	*(uint32_t *)(ep2 + 9) = buffer;
	*(uint32_t *)(ep2 + 13) = BLOCK_BYTES;
	bind[10] = 1;
	src[9] = 1;
	if (!aoc_out_cmd("EP_SETUP", ep, sizeof(ep), 206) || !aoc_out_cmd("EP_SETUP2", ep2, sizeof(ep2), 298) ||
	    !aoc_out_cmd("BIND2 to the speaker", bind, sizeof(bind), 271)) {
		return false;
	}
	aoc_ring_put(svc, r, ring, size, silence, BLOCK_BYTES);
	return aoc_out_cmd("SOURCE on", src, sizeof(src), 201);
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
	vm_offset_t svc = aoc_service("audio_playback0");
	vm_offset_t r = svc ? region(svc, AOC_DOWN) : 0;
	uint32_t size = r ? rd(r, 4) : 0, period = BLOCK_BYTES, buffer = (size / period) * period;
	vm_offset_t ring = r ? ipc_base + rd(r, 0) : 0;

	if (!svc || buffer < 3 * period) {
		printf("akita aoc: sound: no audio_playback0 ring to play into (%u bytes)\n", size);
		return;
	}
	hakios_guard_configure(true, aocsound_gain);
	hakios_guard_attenuation(0);
	hakios_audio_set_sink("the speakers (the AoC and two CS35L41s, hakios.akita_aoc.c)", aoc_darwin_sink);
	printf("akita aoc: sound: WHAT DARWIN PLAYS GOES TO THE SPEAKERS from here on (the AoC's entry point 0, started while "
	    "there is sound: 48 kHz 16-bit stereo, three 20 ms blocks ahead; the guard's gain %u.%03u dB; the amplifiers %s)\n",
	    aocsound_gain / 8, aocsound_gain % 8 * 125, akita_amp_speaker ? "on while there is sound (with speaker=1)" :
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

/* the AoC's firmware loaded by the GSA and started through Trusty (aoc=3) */
static void
aoc_boot(void)
{
	uint64_t size = 0;
	const uint8_t *f = hakios_image_file("fw", "aoc.bin", &size, NULL);
	uint32_t state = 0xff;

	if (f == NULL || size < AOC_AUTH_HEADER_SIZE + 0x1000) {
		printf("akita aoc: no fw/aoc.bin in the image (the build's ../fw-aoc)\n");
		return;
	}
	uint32_t auth_magic = *(const uint32_t *)(f + 1024), generation = *(const uint32_t *)(f + 1028);
	const uint8_t *cfg = f + 1024 + (generation == 2 ? 384 : 352);         /* header_v2: a 64-byte hash */
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
		{ 0x1009, 0 }, { 0x100A, 0 }, { 0x100B, 0 }, { 0x1010, 0 },     /* VNOM, monitor mode, UART, ultrasonic */
		{ 0x1011, (uint32_t)mach_absolute_time() },             /* a seed */
		{ 0x1012, chip_rev }, { 0x1013, chip_type }, { 0x1014, 1 }, { 0x1015, 0 },   /* the chip, gnss-type 1 */
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
			aoc_talk();
			if (PE_parse_boot_argn("aocplay", &aocplay, sizeof(aocplay)) && aocplay != 0) {
				aoc_play();
			}
			if (PE_parse_boot_argn("aocsound", &aocsound, sizeof(aocsound)) && aocsound != 0) {
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
