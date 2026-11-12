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
			if (!PE_parse_boot_argn("aocsound", &aocsound, sizeof(aocsound))) {
				uint32_t speaker = 0;

				/* (speaker=1 -- the speakers wanted, drivers/audio -- means Darwin's sound there: aocsound=1 implied,
				 * 11 characters of a command line that has few to spare) */
				if (PE_parse_boot_argn("speaker", &speaker, sizeof(speaker)) && speaker != 0) {
					aocsound = 1;
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
