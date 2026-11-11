/*
 * A client of Trusty, the Pixel 8a's secure-world OS (TODO 33: the AoC is started by the GSA's "hwmgr" service, which
 * the non-secure world reaches only through Trusty IPC -- soc/gs drivers/soc/google/gsa/gsa_tz.c). Android's driver:
 * google-modules/trusty (drivers/trusty: trusty.c the calls, trusty-virtio.c the rings, trusty-ipc.c the protocol).
 *
 * Trusty is entered with SMC calls from EL1 (as XNU's PSCI calls are, "psci { method = smc }"): fast calls (bit 31) run
 * to completion; standard calls may come back "interrupted" and are resumed with SMC_SC_NOP / SMC_SC_RESTART_LAST.
 * Memory is shared with it through Arm FF-A (FFA_RXTX_MAP once, then FFA_MEM_SHARE descriptors). Every boot loads the
 * secure firmware afresh (BL31, Trusty), so this kernel is a new client to it.
 *
 *   trusty=1   a look, read-only fast calls: Trusty's version string (SMC_FC_GET_VERSION_STR), its API version
 *              (SMC_FC_API_VERSION, asking for 5: memory objects through FF-A), FF-A's version (FFA_VERSION) and this
 *              world's FF-A endpoint id (FFA_ID_GET)
 *   trusty=2   ... and the first standard call: our FF-A message buffers registered (FFA_RXTX_MAP), a page shared with
 *              Trusty (FFA_MEM_SHARE, endpoint 0x8000 as Android's driver hardcodes it), Trusty's virtio devices'
 *              description fetched into it (SMC_SC_VIRTIO_GET_DESCR: a remoteproc resource table) and said. Virtio is
 *              not started.
 *   trusty=3   ... and Trusty IPC brought up, polled (no interrupt: SMC_SC_NOP gives Trusty the processor): the vdev's
 *              two rings (0 rx, 1 tx: trusty-ipc.c's { "rx", "tx" }) shared and their handles written into the
 *              description (da the low half, the reserved word the high), the status DRIVER_OK, SMC_SC_VIRTIO_START,
 *              8 receive buffers posted (a descriptor's address is a buffer's FF-A handle, trusty-virtio.c's map_page);
 *              Trusty's GO_ONLINE awaited, a connection to the GSA's AoC service ("com.android.trusty.gsa.hwmgr.aoc",
 *              hwmgr-ipc.h) asked for, and HWMGR_CMD_STATE_CMD GSA_AOC_GET_STATE sent: the AoC's state as the GSA has
 *              it (0 inactive, 1 loaded, 2 running).
 *
 * Found 11-11 (trusty=1): "Project: ripcurrent, Build: 15649251, Built: 22:23:58 Jun 15 2026"; API version 5; FF-A 1.0;
 * our endpoint id 0.
 */
#include <kern/clock.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>
#include <kern/kalloc.h>
#include <kern/locks.h>

extern uint64_t kvtophys(vm_offset_t va);
extern boolean_t ml_set_interrupts_enabled(boolean_t enable);

/* SMC numbers (google-modules/trusty include/linux/trusty/smcall.h, arm_ffa.h) */
#define SMC_NR(entity, fn, fast, smc64) ((((fast) & 1u) << 31) | (((smc64) & 1u) << 30) | (((entity) & 0x3fu) << 24) | \
	                                 ((fn) & 0xffffu))
#define SMC_ENTITY_STD                  4
#define SMC_ENTITY_SECURE_MONITOR       60
#define SMC_FC_GET_VERSION_STR          SMC_NR(SMC_ENTITY_SECURE_MONITOR, 10, 1, 0)
#define SMC_FC_API_VERSION              SMC_NR(SMC_ENTITY_SECURE_MONITOR, 11, 1, 0)
#define SMC_FC_FFA_ERROR                SMC_NR(SMC_ENTITY_STD, 0x60, 1, 0)
#define SMC_FC_FFA_SUCCESS              SMC_NR(SMC_ENTITY_STD, 0x61, 1, 0)
#define SMC_FC_FFA_VERSION              SMC_NR(SMC_ENTITY_STD, 0x63, 1, 0)
#define SMC_FC_FFA_ID_GET               SMC_NR(SMC_ENTITY_STD, 0x69, 1, 0)
#define SMC_FC_FFA_FEATURES             SMC_NR(SMC_ENTITY_STD, 0x64, 1, 0)
#define SMC_FC64_FFA_RXTX_MAP           SMC_NR(SMC_ENTITY_STD, 0x66, 1, 1)
#define SMC_FC_FFA_MEM_SHARE            SMC_NR(SMC_ENTITY_STD, 0x73, 1, 0)
#define SMC_ENTITY_TRUSTED_OS           50
#define SMC_SC_RESTART_LAST             SMC_NR(SMC_ENTITY_SECURE_MONITOR, 0, 0, 0)
#define SMC_SC_RESTART_FIQ              SMC_NR(SMC_ENTITY_SECURE_MONITOR, 2, 0, 0)
#define SMC_SC_NOP                      SMC_NR(SMC_ENTITY_SECURE_MONITOR, 3, 0, 0)
#define SMC_SC_VIRTIO_GET_DESCR         SMC_NR(SMC_ENTITY_TRUSTED_OS, 20, 0, 0)
#define SMC_SC_VIRTIO_START             SMC_NR(SMC_ENTITY_TRUSTED_OS, 21, 0, 0)
#define SMC_SC_VDEV_KICK_VQ             SMC_NR(SMC_ENTITY_TRUSTED_OS, 24, 0, 0)
#define SMC_NC_VDEV_KICK_VQ             SMC_NR(SMC_ENTITY_TRUSTED_OS, 25, 0, 0)
#define SM_ERR_NOP_INTERRUPTED          (-14)
#define SM_ERR_NOP_DONE                 (-15)
#define SM_ERR_INTERRUPTED              (-3)
#define SM_ERR_BUSY                     (-5)
#define SM_ERR_PANIC                    (-11)
#define SM_ERR_FIQ_INTERRUPTED          (-12)
#define SM_ERR_CPU_IDLE                 (-13)
#define FFA_PAGE_SIZE                   4096u
#define FFA_MEM_ATTR_NORMAL_WB_INNER    ((2u << 4) | (3u << 2) | 3u)
#define FFA_MEM_PERM_RW                 (1u << 1)
#define TRUSTY_FFA_ID                   0x8000u
#define TRUSTY_API_VERSION_CURRENT      5
#define SM_ERR_UNDEFINED_SMC            0xffffffffu     /* (sm_err.h: -1) */

struct smc_ret8 {
	uint64_t r0, r1, r2, r3, r4, r5, r6, r7;
};

static struct smc_ret8
smc8(uint64_t a0, uint64_t a1, uint64_t a2, uint64_t a3, uint64_t a4, uint64_t a5, uint64_t a6, uint64_t a7)
{
	register uint64_t x0 __asm__("x0") = a0;
	register uint64_t x1 __asm__("x1") = a1;
	register uint64_t x2 __asm__("x2") = a2;
	register uint64_t x3 __asm__("x3") = a3;
	register uint64_t x4 __asm__("x4") = a4;
	register uint64_t x5 __asm__("x5") = a5;
	register uint64_t x6 __asm__("x6") = a6;
	register uint64_t x7 __asm__("x7") = a7;

	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3), "+r" (x4), "+r" (x5), "+r" (x6), "+r" (x7) : :
	    "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return (struct smc_ret8){ x0, x1, x2, x3, x4, x5, x6, x7 };
}

static uint32_t
fast_call32(uint32_t nr, uint32_t a1, uint32_t a2, uint32_t a3)
{
	return (uint32_t)smc8(nr, a1, a2, a3, 0, 0, 0, 0).r0;
}

static uint32_t mode;
static uint16_t ffa_local_id;
static vm_offset_t ffa_tx, ffa_rx;              /* our FF-A message buffers (one page each) */

/* a page of ours, physically contiguous and cached, and its physical address */
static vm_offset_t
shared_page(uint64_t *pa)
{
	vm_offset_t va = 0;

	if (kmem_alloc_contig(kernel_map, &va, PAGE_SIZE, 0, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_DATA, VM_KERN_MEMORY_IOKIT) !=
	    KERN_SUCCESS) {
		return 0;
	}
	bzero((void *)va, PAGE_SIZE);
	*pa = kvtophys(va);
	return va;
}

/*
 * A standard call, as Android's trusty_std_call32 makes it with nobody else calling: interrupts off for each SMC; an
 * FIQ's interruption resumed with SMC_SC_RESTART_FIQ, "busy" retried, an interruption (or another CPU's idling) resumed
 * with SMC_SC_RESTART_LAST
 */
static int32_t
std_call32(uint32_t nr, uint32_t a0, uint32_t a1, uint32_t a2)
{
	int32_t ret;
	int busy = 0, restarts = 0;
	uint32_t first = nr;

	for (;;) {
		boolean_t was = ml_set_interrupts_enabled(FALSE);

		ret = (int32_t)smc8(nr, a0, a1, a2, 0, 0, 0, 0).r0;
		while (ret == SM_ERR_FIQ_INTERRUPTED) {
			ret = (int32_t)smc8(SMC_SC_RESTART_FIQ, 0, 0, 0, 0, 0, 0, 0).r0;
		}
		ml_set_interrupts_enabled(was);
		if (ret == SM_ERR_INTERRUPTED || ret == SM_ERR_CPU_IDLE) {
			/*
			 * resumed with SMC_SC_RESTART_LAST; while it waits (CPU_IDLE: for work another CPU's NOP would do --
			 * Android's driver waits for a NOP to complete), Trusty's other threads are given a NOP here first.
			 * Never for good: a call Trusty cannot finish (VIRTIO_START with no receive buffer for its GO_ONLINE)
			 * hung this thread, 11-11
			 */
			if (++restarts > 20000) {
				printf("akita trusty: call 0x%x never finished (last %d): given up\n", first, ret);
				return ret;
			}
			if (ret == SM_ERR_CPU_IDLE) {
				was = ml_set_interrupts_enabled(FALSE);
				(void)smc8(SMC_SC_NOP, 0, 0, 0, 0, 0, 0, 0);
				ml_set_interrupts_enabled(was);
				delay(100);
			}
			nr = SMC_SC_RESTART_LAST;
			a0 = a1 = a2 = 0;
			continue;
		}
		if (ret == SM_ERR_BUSY && busy++ < 50) {
			delay(1000 * (busy < 10 ? busy : 10));
			continue;
		}
		break;
	}
	if (ret == SM_ERR_PANIC) {
		printf("akita trusty: TRUSTY PANICKED\n");
	}
	return ret;
}

/*
 * BYTES at PA shared with Trusty for reading and writing: its FF-A handle, or 0. (Shared at the size Android's driver
 * shares on its 4 KB pages -- a message buffer one FF-A page, a vring two --, not this kernel's 16 KB page: with every
 * object 16 KB, Trusty's kick of the rx ring said -7, SM_ERR_INTERNAL_FAILURE)
 */
static uint64_t
share_mem(uint64_t pa, uint32_t bytes)
{
	uint8_t *tx = (uint8_t *)ffa_tx;
	uint32_t pages = (bytes + FFA_PAGE_SIZE - 1) / FFA_PAGE_SIZE;

	bzero(tx, 128);
	/* struct ffa_mtd (32 bytes): sender, attributes, flags, handle, tag, emad count */
	*(uint16_t *)(tx + 0) = ffa_local_id;
	tx[2] = FFA_MEM_ATTR_NORMAL_WB_INNER;
	*(uint32_t *)(tx + 28) = 1;
	/* struct ffa_emad (16): Trusty's endpoint, read-write, the composite descriptor's offset */
	*(uint16_t *)(tx + 32) = TRUSTY_FFA_ID;
	tx[34] = FFA_MEM_PERM_RW;
	*(uint32_t *)(tx + 36) = 48;
	/* struct ffa_comp_mrd (16) at 48: total pages, one range; struct ffa_cons_mrd (16) at 64: the range */
	*(uint32_t *)(tx + 48) = pages;
	*(uint32_t *)(tx + 52) = 1;
	*(uint64_t *)(tx + 64) = pa;
	*(uint32_t *)(tx + 72) = pages;
	struct smc_ret8 r = smc8(SMC_FC_FFA_MEM_SHARE, 80, 80, 0, 0, 0, 0, 0);

	if (r.r0 != SMC_FC_FFA_SUCCESS) {
		printf("akita trusty: FFA_MEM_SHARE failed: r0 0x%llx r1 0x%llx r2 0x%llx\n", r.r0, r.r1, r.r2);
		return 0;
	}
	return (r.r2 & 0xffffffffull) | (r.r3 << 32);
}

/* a whole page of ours (PAGE_SIZE) shared */
static uint64_t
share_page(uint64_t pa)
{
	return share_mem(pa, PAGE_SIZE);
}

/* our FF-A message buffers registered; false when Trusty's FF-A refuses */
static bool
ffa_setup(void)
{
	uint64_t tx_pa, rx_pa;
	/* FFA_VERSION first, 1.0: the descriptors below are 1.0's (without it, from the AoC's driver, FFA_MEM_SHARE said
	 * INVALID_PARAMETERS -- the firmware took this world for a later version, 11-11; Android's trusty_init_msg_buf) */
	struct smc_ret8 r = smc8(SMC_FC_FFA_VERSION, 0x10000u, 0, 0, 0, 0, 0, 0);

	if (((r.r0 >> 16) & 0x7fff) != 1) {
		printf("akita trusty: FF-A version 0x%llx, not 1.x\n", r.r0);
		return false;
	}
	r = smc8(SMC_FC_FFA_FEATURES, SMC_FC_FFA_MEM_SHARE, 0, 0, 0, 0, 0, 0);

	if (r.r0 != SMC_FC_FFA_SUCCESS) {
		printf("akita trusty: FFA_FEATURES(MEM_SHARE): r0 0x%llx r2 0x%llx -- no memory sharing\n", r.r0, r.r2);
		return false;
	}
	r = smc8(SMC_FC_FFA_ID_GET, 0, 0, 0, 0, 0, 0, 0);
	ffa_local_id = (uint16_t)r.r2;
	ffa_tx = shared_page(&tx_pa);
	ffa_rx = shared_page(&rx_pa);
	if (!ffa_tx || !ffa_rx) {
		printf("akita trusty: no memory for the FF-A buffers\n");
		return false;
	}
	r = smc8(SMC_FC64_FFA_RXTX_MAP, tx_pa, rx_pa, PAGE_SIZE / FFA_PAGE_SIZE, 0, 0, 0, 0);
	if (r.r0 != SMC_FC_FFA_SUCCESS) {
		printf("akita trusty: FFA_RXTX_MAP (tx 0x%llx rx 0x%llx): r0 0x%llx r2 0x%llx\n", tx_pa, rx_pa, r.r0, r.r2);
		return false;
	}
	printf("akita trusty: FF-A buffers registered (tx 0x%llx, rx 0x%llx, %u pages each)\n", tx_pa, rx_pa,
	    PAGE_SIZE / FFA_PAGE_SIZE);
	return true;
}

/* the description (a shared page), its handle and length, and where its Trusty IPC vdev is */
static vm_offset_t descr;
static uint64_t descr_handle;
static int32_t descr_len;
static uint32_t vdev_at;                        /* the vdev entry's offset in the description (0: none) */

/* Trusty's virtio devices: their description fetched into a shared page, and said */
static void
virtio_describe(void)
{
	uint64_t pa;
	vm_offset_t d = shared_page(&pa);
	uint64_t handle;

	if (!d || (handle = share_page(pa)) == 0) {
		return;
	}
	descr = d;
	descr_handle = handle;
	printf("akita trusty: a page shared (0x%llx): handle 0x%llx\n", pa, handle);
	int32_t len = std_call32(SMC_SC_VIRTIO_GET_DESCR, (uint32_t)handle, (uint32_t)(handle >> 32), PAGE_SIZE);

	if (len <= 0) {
		printf("akita trusty: SMC_SC_VIRTIO_GET_DESCR: %d\n", len);
		return;
	}
	descr_len = len;
	const uint32_t *t = (const uint32_t *)d;
	printf("akita trusty: THE VIRTIO DESCRIPTION, %d bytes: version %u, %u entries\n", len, t[0], t[1]);
	for (uint32_t i = 0; i < t[1] && i < 8; i++) {
		uint32_t off = t[4 + i];

		if (off + 8 > (uint32_t)len) {
			break;
		}
		const uint8_t *e = (const uint8_t *)d + off;
		uint32_t type = *(const uint32_t *)e;

		if (type != 3) {                /* RSC_VDEV */
			printf("akita trusty:   entry %u: type %u\n", i, type);
			continue;
		}
		const uint32_t *vd = (const uint32_t *)(e + 4);
		uint8_t nvr = e[4 + 21];
		if (vd[0] == 13 && nvr == 2 && vdev_at == 0) {
			vdev_at = off + 4;      /* (VIRTIO_ID_TRUSTY_IPC) */
		}
		printf("akita trusty:   entry %u: vdev id %u notifyid %u features 0x%x/0x%x, config %u bytes, status %u, %u vrings\n",
		    i, vd[0], vd[1], vd[2], vd[3], vd[4], e[4 + 20], nvr);
		const uint32_t *vr = (const uint32_t *)(e + 4 + 24);
		for (uint32_t k = 0; k < nvr && k < 4; k++) {
			printf("akita trusty:     vring %u: da 0x%x align %u num %u notifyid %u\n", k, vr[5 * k], vr[5 * k + 1],
			    vr[5 * k + 2], vr[5 * k + 3]);
		}
		const uint8_t *cfg = (const uint8_t *)(vr + 5 * nvr);
		if (vd[4] >= 40) {              /* tipc's config: msg_buf_max_size, msg_buf_alignment, dev_name[32] */
			char name[33];
			memcpy(name, cfg + 8, 32);
			name[32] = 0;
			printf("akita trusty:     tipc config: message buffers up to %u bytes, aligned %u, \"%s\"\n",
			    *(const uint32_t *)cfg, *(const uint32_t *)(cfg + 4), name);
		}
	}
}

/* ---- Trusty IPC over the vdev, polled ------------------------------------------------------------------------- */

#define VQ_RX                           0
#define VQ_TX                           1
#define RX_BUFS                         8
#define TX_BUFS                         4
#define MSG_MAX                         4096u
#define TIPC_CTRL_ADDR                  53
#define TIPC_LOCAL_ADDR                 1024
#define VRING_DESC_F_WRITE              2

struct vq {
	vm_offset_t va;                 /* the ring: descriptors, avail at 16 * num, used at the next `align` */
	uint64_t handle;
	uint32_t num, align, notifyid;
	uint16_t avail_idx, used_seen;
};
static struct vq vqs[2];
static vm_offset_t rxbuf[RX_BUFS], txbuf[TX_BUFS];
static uint64_t rxbuf_handle[RX_BUFS], txbuf_handle[TX_BUFS];
static uint32_t vdev_notifyid, tx_next;
static bool tipc_online;
static uint32_t conn_remote, conn_status;
static bool conn_answered, reply_got;
static uint8_t reply[64];
static uint32_t reply_len;

static inline volatile uint16_t *
vq_avail(struct vq *q)
{
	return (volatile uint16_t *)(q->va + 16 * q->num);    /* flags, idx, ring[num], used_event */
}

static inline volatile uint8_t *
vq_used(struct vq *q)
{
	uint64_t at = 16 * q->num + 2 * (3 + q->num);
	at = (at + q->align - 1) & ~(uint64_t)(q->align - 1);
	return (volatile uint8_t *)(q->va + at);             /* flags, idx, {id, len}[num], avail_event */
}

/*
 * Trusty given the processor: SMC_SC_NOP, with a "nop handler" call in its arguments or none, again while it says
 * interrupted (SM_ERR_NOP_INTERRUPTED), until SM_ERR_NOP_DONE (trusty.c's nop work)
 */
static int32_t
trusty_nop(uint32_t a0, uint32_t a1, uint32_t a2)
{
	int32_t r;
	int n = 0;

	do {
		r = std_call32(SMC_SC_NOP, a0, a1, a2);
		a0 = a1 = a2 = 0;
	} while (r == SM_ERR_NOP_INTERRUPTED && ++n < 100);
	return r;
}

/*
 * a vring's kick: as a nop handler call (trusty-virtio.c's kick_nop: SMC_NC_VDEV_KICK_VQ). -7 is LK's ERR_NOT_VALID
 * passed through: the vdev not active yet -- SMC_SC_VIRTIO_START only schedules its activation in Trusty's threads,
 * which run when Trusty is given the processor (11-11: an immediate kick said -7). So: nops, and the kick again.
 */
static int32_t
vq_kick(struct vq *q)
{
	int32_t r = 0;

	for (int tries = 0; tries < 50; tries++) {
		r = trusty_nop(SMC_NC_VDEV_KICK_VQ, vdev_notifyid, q->notifyid);
		if (r == SM_ERR_NOP_DONE || r == 0) {
			if (tries != 0) {
				printf("akita trusty: kick of vring %u taken after %d tries\n", q->notifyid, tries + 1);
			}
			return 0;
		}
		trusty_nop(0, 0, 0);
		delay(2000);
	}
	printf("akita trusty: kick of vring %u: %d\n", q->notifyid, r);
	return r;
}

/* buffer HANDLE of LEN bytes offered on Q (WRITE: the device writes it), under descriptor DESC */
static void
vq_add(struct vq *q, uint32_t desc, uint64_t handle, uint32_t len, bool write)
{
	volatile uint8_t *d = (volatile uint8_t *)(q->va + 16 * desc);

	*(volatile uint64_t *)(d + 0) = handle;
	*(volatile uint32_t *)(d + 8) = len;
	*(volatile uint16_t *)(d + 12) = write ? VRING_DESC_F_WRITE : 0;
	*(volatile uint16_t *)(d + 14) = 0;
	volatile uint16_t *av = vq_avail(q);
	av[2 + (q->avail_idx % q->num)] = (uint16_t)desc;
	__builtin_arm_dmb(0xb);
	q->avail_idx++;
	av[1] = q->avail_idx;
	__builtin_arm_dsb(0xf);
}

/* a received message: control messages acted on, the connected service's reply kept */
static void
tipc_received(const uint8_t *m, uint32_t len)
{
	if (len < 16) {
		return;
	}
	uint32_t src = *(const uint32_t *)m, dst = *(const uint32_t *)(m + 4);
	uint16_t mlen = *(const uint16_t *)(m + 12);
	const uint8_t *data = m + 16;

	if (dst == TIPC_CTRL_ADDR && src == TIPC_CTRL_ADDR && mlen >= 8) {
		uint32_t type = *(const uint32_t *)data;

		if (type == 1) {                /* GO_ONLINE */
			tipc_online = true;
			printf("akita trusty: TIPC ONLINE\n");
		} else if (type == 4 && mlen >= 8 + 20) {     /* CONN_RSP: target, status, remote, max size, max count */
			const uint32_t *b = (const uint32_t *)(data + 8);
			conn_status = b[1];
			conn_remote = b[2];
			conn_answered = true;
			printf("akita trusty: connection answered: target %u status %d remote %u, messages up to %u bytes, %u at once\n",
			    b[0], (int32_t)b[1], b[2], b[3], b[4]);
		} else {
			printf("akita trusty: control message type %u (%u bytes)\n", type, mlen);
		}
		return;
	}
	printf("akita trusty: a message from %u to %u, %u bytes\n", src, dst, mlen);
	if (dst == TIPC_LOCAL_ADDR) {           /* (the service's replies came "from 0", not its remote 1000: 11-11) */
		reply_len = mlen < sizeof(reply) ? mlen : sizeof(reply);
		memcpy(reply, data, reply_len);
		reply_got = true;
	}
}

/* Trusty given the processor until something is received or COND holds (polled: SMC_SC_NOP), up to ~MS ms */
static void
tipc_pump(bool *cond, int ms)
{
	struct vq *rx = &vqs[VQ_RX];

	for (int i = 0; i < ms && !*cond; i++) {
		int32_t r = trusty_nop(0, 0, 0);

		if (i == 0 && r != SM_ERR_NOP_DONE) {
			printf("akita trusty: SMC_SC_NOP: %d\n", r);
		}
		volatile uint8_t *u = vq_used(rx);
		__builtin_arm_dmb(0xb);
		uint16_t idx = *(volatile uint16_t *)(u + 2);
		while (rx->used_seen != idx) {
			uint32_t slot = rx->used_seen % rx->num;
			uint32_t id = *(volatile uint32_t *)(u + 4 + 8 * slot), len = *(volatile uint32_t *)(u + 8 + 8 * slot);

			rx->used_seen++;
			if (id < RX_BUFS) {
				tipc_received((const uint8_t *)rxbuf[id], len);
				vq_add(rx, id, rxbuf_handle[id], MSG_MAX, true);        /* back to Trusty */
				vq_kick(rx);
			}
		}
		if (!*cond) {
			delay(1000);
		}
	}
}

/* a message from our endpoint to DST */
static void
tipc_send(uint32_t dst, const void *data, uint32_t len)
{
	uint32_t b = tx_next++ % TX_BUFS;
	uint8_t *m = (uint8_t *)txbuf[b];

	bzero(m, 16);
	*(uint32_t *)m = TIPC_LOCAL_ADDR;
	*(uint32_t *)(m + 4) = dst;
	*(uint16_t *)(m + 12) = (uint16_t)len;
	memcpy(m + 16, data, len);
	vq_add(&vqs[VQ_TX], b, txbuf_handle[b], 16 + len, false);
	vq_kick(&vqs[VQ_TX]);
}

/* the vdev's rings shared, virtio started, receive buffers posted; Trusty's GO_ONLINE awaited */
static bool
tipc_bringup(void)
{
	uint8_t *vd = (uint8_t *)descr + vdev_at;
	uint64_t pa;

	if (vdev_at == 0) {
		printf("akita trusty: no Trusty IPC vdev in the description\n");
		return false;
	}
	vdev_notifyid = *(uint32_t *)(vd + 4);
	for (int k = 0; k < 2; k++) {
		uint8_t *vr = vd + 24 + 20 * k;
		struct vq *q = &vqs[k];

		q->align = *(uint32_t *)(vr + 4);
		q->num = *(uint32_t *)(vr + 8);
		q->notifyid = *(uint32_t *)(vr + 12);
		if (16 * q->num + 2 * (3 + q->num) + q->align + 8 * q->num + 8 > PAGE_SIZE || q->align == 0 ||
		    (q->align & (q->align - 1))) {
			printf("akita trusty: vring %d of %u, align %u: not in a page\n", k, q->num, q->align);
			return false;
		}
		q->va = shared_page(&pa);
		uint64_t ring_bytes = (uint64_t)((uint8_t *)vq_used(q) - (uint8_t *)q->va) + 4 + 8 * q->num + 2;
		if (!q->va || (q->handle = share_mem(pa, (uint32_t)((ring_bytes + FFA_PAGE_SIZE - 1) & ~(uint64_t)(FFA_PAGE_SIZE - 1)))) == 0) {
			return false;
		}
		*(uint32_t *)(vr + 0) = (uint32_t)q->handle;          /* da: the handle's low half */
		*(uint32_t *)(vr + 16) = (uint32_t)(q->handle >> 32); /* the reserved word: its high half */
	}
	/* the receive buffers in Trusty's TX ring (vring 0) BEFORE the start: its probe sends GO_ONLINE at once and waits for
	 * one (Android's tipc probe posts them before trusty_virtio_start) */
	for (int i = 0; i < RX_BUFS; i++) {
		rxbuf[i] = shared_page(&pa);
		if (!rxbuf[i] || (rxbuf_handle[i] = share_mem(pa, MSG_MAX)) == 0) {
			return false;
		}
		vq_add(&vqs[VQ_RX], (uint32_t)i, rxbuf_handle[i], MSG_MAX, true);
	}
	*(uint32_t *)(vd + 12) = 0;                     /* gfeatures */
	vd[20] = 1 | 2 | 4;     /* ACKNOWLEDGE, DRIVER, DRIVER_OK -- exactly: Trusty's validate_descr refuses anything else
				 * (with FEATURES_OK too the probe failed silently and every kick said -7, 11-11) */
	__builtin_arm_dsb(0xf);
	int32_t r = std_call32(SMC_SC_VIRTIO_START, (uint32_t)descr_handle, (uint32_t)(descr_handle >> 32), (uint32_t)descr_len);
	if (r != 0) {
		printf("akita trusty: SMC_SC_VIRTIO_START: %d\n", r);
		return false;
	}
	printf("akita trusty: virtio started (rings: rx handle 0x%llx, tx handle 0x%llx)\n", vqs[VQ_RX].handle,
	    vqs[VQ_TX].handle);
	for (int i = 0; i < 20; i++) {                  /* (its activation: Trusty's threads given the processor) */
		trusty_nop(0, 0, 0);
		delay(1000);
	}
	for (int i = 0; i < TX_BUFS; i++) {
		txbuf[i] = shared_page(&pa);
		if (!txbuf[i] || (txbuf_handle[i] = share_mem(pa, MSG_MAX)) == 0) {
			return false;
		}
	}
	vq_kick(&vqs[VQ_RX]);
	tipc_pump(&tipc_online, 2000);
	if (!tipc_online) {
		printf("akita trusty: no GO_ONLINE in 2 s (rx used %u)\n", vqs[VQ_RX].used_seen);
	}
	return tipc_online;
}

/* the GSA's AoC service connected (once): false when Trusty refused or did not answer */
static bool aoc_connected;

static bool
aoc_connect(void)
{
	struct {
		uint32_t type, body_len;
		char name[256];
	} __attribute__((packed)) req;

	if (aoc_connected) {
		return true;
	}
	bzero(&req, sizeof(req));
	req.type = 3;                                   /* CONN_REQ */
	req.body_len = sizeof(req.name);
	strlcpy(req.name, "com.android.trusty.gsa.hwmgr.aoc", sizeof(req.name));
	conn_answered = false;
	tipc_send(TIPC_CTRL_ADDR, &req, sizeof(req));
	tipc_pump(&conn_answered, 2000);
	if (!conn_answered || conn_status != 0) {
		printf("akita trusty: the AoC service: %s\n", conn_answered ? "refused" : "no answer in 2 s");
		return false;
	}
	aoc_connected = true;
	return true;
}

/* HWMGR_CMD_STATE_CMD CMD to the GSA's AoC service: its error (0 OK, < 0 no answer), *STATE the AoC's state after */
static int
aoc_state_cmd(uint32_t cmd, uint32_t *state, int wait_ms)
{
	uint32_t req[2] = { 1, cmd };                   /* HWMGR_CMD_STATE_CMD, the gsa_aoc_cmd */

	reply_got = false;
	tipc_send(conn_remote, req, sizeof(req));
	tipc_pump(&reply_got, wait_ms);
	if (!reply_got || reply_len < 12) {
		printf("akita trusty: the AoC service did not answer state command %u\n", cmd);
		return -1;
	}
	const uint32_t *rw = (const uint32_t *)reply;
	*state = rw[2];
	printf("akita trusty: THE GSA SAYS (state command %u): 0x%x, error %d, THE AoC'S STATE %u (0 inactive, 1 loaded, 2 running)\n",
	    cmd, rw[0], (int32_t)rw[1], rw[2]);
	return (int)rw[1];
}

static bool trusty_up;
static LCK_GRP_DECLARE(trusty_lck_grp, "akita_trusty");
static LCK_MTX_DECLARE(trusty_lock, &trusty_lck_grp);

/* Trusty brought up (once): API 5, FF-A buffers, the virtio description, Trusty IPC online */
static bool
trusty_bringup(void)
{
	if (trusty_up) {
		return true;
	}
	if (fast_call32(SMC_FC_API_VERSION, TRUSTY_API_VERSION_CURRENT, 0, 0) != TRUSTY_API_VERSION_CURRENT || !ffa_setup()) {
		return false;
	}
	virtio_describe();
	if (descr_len <= 0 || !tipc_bringup()) {
		return false;
	}
	trusty_up = true;
	return true;
}

/*
 * For the AoC's driver (devices/akita/drivers/aoc): GSA_AOC_* command CMD (0 GET_STATE, 1 START, 4 SHUTDOWN, 5
 * RELEASE_RESET, 6 RESET) through the GSA's hwmgr service. 0 and *STATE the AoC's state after; < 0 when Trusty is not
 * reachable, > 0 the GSA's error.
 */
int akita_trusty_aoc_cmd(uint32_t cmd, uint32_t *state);
int
akita_trusty_aoc_cmd(uint32_t cmd, uint32_t *state)
{
	int r = -1;

	lck_mtx_lock(&trusty_lock);
	if (trusty_bringup() && aoc_connect()) {
		r = aoc_state_cmd(cmd, state, cmd == 1 ? 10000 : 2000);
	}
	lck_mtx_unlock(&trusty_lock);
	return r;
}

static void
akita_trusty_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	char version[160] = "";
	int32_t len = (int32_t)fast_call32(SMC_FC_GET_VERSION_STR, 0xffffffffu, 0, 0);

	if (len > 0) {
		int n = len < (int32_t)sizeof(version) - 1 ? len : (int)sizeof(version) - 1;

		for (int i = 0; i < n; i++) {
			int32_t c = (int32_t)fast_call32(SMC_FC_GET_VERSION_STR, (uint32_t)i, 0, 0);

			version[i] = c > 0 && c < 127 ? (char)c : '?';
		}
		version[n] = 0;
		printf("akita trusty: TRUSTY ANSWERS: version (%d characters) \"%s\"\n", len, version);
	} else {
		printf("akita trusty: SMC_FC_GET_VERSION_STR: %d (no Trusty answering?)\n", len);
	}
	uint32_t api = fast_call32(SMC_FC_API_VERSION, TRUSTY_API_VERSION_CURRENT, 0, 0);

	printf("akita trusty: API version asked %d, given %d%s\n", TRUSTY_API_VERSION_CURRENT, (int32_t)api,
	    api == SM_ERR_UNDEFINED_SMC ? " (undefined: an old Trusty)" : "");
	struct smc_ret8 r = smc8(SMC_FC_FFA_VERSION, 0x10000u, 0, 0, 0, 0, 0, 0);

	printf("akita trusty: FF-A version 0x%llx (%llu.%llu)\n", r.r0, (r.r0 >> 16) & 0x7fff, r.r0 & 0xffff);
	r = smc8(SMC_FC_FFA_ID_GET, 0, 0, 0, 0, 0, 0, 0);
	printf("akita trusty: FFA_ID_GET: %s, id 0x%llx (r0 0x%llx r2 0x%llx)\n", r.r0 == SMC_FC_FFA_SUCCESS ? "success" :
	    r.r0 == SMC_FC_FFA_ERROR ? "ERROR" : "?", r.r2, r.r0, r.r2);
	if (mode == 2 && api == TRUSTY_API_VERSION_CURRENT && ffa_setup()) {
		virtio_describe();
	} else if (mode >= 3) {
		uint32_t state = 0;

		(void)akita_trusty_aoc_cmd(0, &state);  /* GSA_AOC_GET_STATE */
	}
}

__startup_func
static void
akita_trusty_init(void)
{
	static thread_call_t start;
	uint64_t deadline;

	if (!PE_parse_boot_argn("trusty", &mode, sizeof(mode)) || mode == 0) {
		return;
	}
	start = thread_call_allocate(akita_trusty_start, NULL);
	clock_interval_to_deadline(4, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_trusty_init);
