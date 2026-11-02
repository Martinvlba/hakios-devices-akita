/*
 * ACPM, Google's power-management firmware on zuma's APM core, as a client from the kernel: its IPC -- a request of
 * four words into the channel's queue in the APM's SRAM, the mailbox's doorbell, and the answer of the same sequence
 * number polled for in the other queue. The same as U-Boot's cmd/hakios_acpm.c (uefi), which works on this phone, and
 * Linux's drivers/firmware/samsung/exynos-acpm.c (gs101) before it. One request at a time (a mutex), polled.
 *
 * The Pixel 8a's (Android's tree: acpm_ipc, reg 0x15110000 the mailbox, 0x15700000 the SRAM, initdata-base 0xa000).
 * Its users here: the thermal sensors (channel 9, hakios.akita_thermal.c); the PMIC (channel 2) and the processors'
 * clocks (DVFS) will be next.
 *
 *   int akita_acpm_xfer(unsigned channel, uint32_t cmd[4])   0, and the answer in cmd; or an errno
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <sys/errno.h>
#include <sys/systm.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

#define ACPM_MBOX_PHYS          0x15110000UL
#define ACPM_SRAM_PHYS          0x15700000UL
#define ACPM_SRAM_SIZE          0x58000UL
#define ACPM_INITDATA           0xa000
#define MBOX_INTGR1             0x40

/* the shared memory's head (struct acpm_shmem) and a channel (struct acpm_chan_shmem): the APM's rx is our tx */
#define SHMEM_CHANS             0x08
#define SHMEM_NUM_CHANS         0x18
#define CHAN_RX_REAR            0x10
#define CHAN_RX_FRONT           0x14
#define CHAN_RX_BASE            0x18
#define CHAN_TX_REAR            0x28
#define CHAN_TX_FRONT           0x2c
#define CHAN_TX_BASE            0x30
#define CHAN_QLEN               0x34
#define CHAN_MLEN               0x38
#define CHAN_SIZE               0x48
#define SEQNUM_SHIFT            16
#define SEQNUM_MASK             (0x3fu << SEQNUM_SHIFT)
#define POLL_US                 100000

static LCK_GRP_DECLARE(acpm_grp, "akita acpm");
static LCK_MTX_DECLARE(acpm_lock, &acpm_grp);
static vm_offset_t sram, mbox;
static uint32_t seqnum;
static bool said_bad;

static inline uint32_t rd(vm_offset_t a) { return *(volatile uint32_t *)a; }
static inline void wr(vm_offset_t a, uint32_t v) { *(volatile uint32_t *)a = v; }

static bool
acpm_map(void)
{
	if (sram && mbox) {
		return true;
	}
	sram = ml_io_map(ACPM_SRAM_PHYS, ACPM_SRAM_SIZE);
	mbox = ml_io_map(ACPM_MBOX_PHYS, 0x1000);
	return sram && mbox;
}

/* the offsets the shared memory holds are offsets into the SRAM: checked before they are used */
static bool
in_sram(uint32_t offset, uint32_t size)
{
	return offset < ACPM_SRAM_SIZE && size <= ACPM_SRAM_SIZE - offset;
}

int akita_acpm_xfer(unsigned channel, uint32_t cmd[4]);

int
akita_acpm_xfer(unsigned channel, uint32_t cmd[4])
{
	vm_offset_t head, ch, tx_base, rx_base;
	uint32_t n, qlen, mlen, front, next, rear, rfront, mine;
	int error = 0;

	lck_mtx_lock(&acpm_lock);
	if (!acpm_map()) {
		error = ENXIO;
		goto out;
	}
	head = sram + ACPM_INITDATA;
	n = rd(head + SHMEM_NUM_CHANS);
	if (channel >= n || n > 64 || !in_sram(rd(head + SHMEM_CHANS) + channel * CHAN_SIZE, CHAN_SIZE)) {
		error = EINVAL;
		goto bad;
	}
	ch = sram + rd(head + SHMEM_CHANS) + channel * CHAN_SIZE;
	qlen = rd(ch + CHAN_QLEN);
	mlen = rd(ch + CHAN_MLEN);
	if (qlen == 0 || qlen > 1024 || mlen < 16 || mlen > 256 || !in_sram(rd(ch + CHAN_RX_BASE), qlen * mlen) ||
	    !in_sram(rd(ch + CHAN_TX_BASE), qlen * mlen)) {
		if (!said_bad) {
			printf("akita acpm: channel %u: qlen %u mlen %u, queues at %#x and %#x?\n", channel, qlen, mlen,
			    rd(ch + CHAN_RX_BASE), rd(ch + CHAN_TX_BASE));
		}
		error = EDOM;
		goto bad;
	}
	tx_base = sram + rd(ch + CHAN_RX_BASE);
	rx_base = sram + rd(ch + CHAN_TX_BASE);

	/* answers nobody read (an earlier request that timed out): marked read */
	wr(sram + rd(ch + CHAN_TX_REAR), rd(sram + rd(ch + CHAN_TX_FRONT)));

	seqnum = seqnum % 63 + 1;
	mine = seqnum;
	cmd[0] = (cmd[0] & ~SEQNUM_MASK) | (mine << SEQNUM_SHIFT);
	front = rd(sram + rd(ch + CHAN_RX_FRONT)) % qlen;
	next = (front + 1) % qlen;
	for (unsigned waited = 0; rd(sram + rd(ch + CHAN_RX_REAR)) == next; waited += 20) {
		if (waited > POLL_US) {
			error = EBUSY;
			goto out;
		}
		delay(20);
	}
	for (unsigned i = 0; i < 4; i++) {
		wr(tx_base + mlen * front + 4 * i, cmd[i]);
	}
	__builtin_arm_dsb(0xf);
	wr(sram + rd(ch + CHAN_RX_FRONT), next);
	__builtin_arm_dsb(0xf);
	wr(mbox + MBOX_INTGR1, 1u << channel);

	for (unsigned waited = 0;; waited += 20) {
		rear = rd(sram + rd(ch + CHAN_TX_REAR)) % qlen;
		rfront = rd(sram + rd(ch + CHAN_TX_FRONT)) % qlen;
		while (rear != rfront) {
			vm_offset_t m = rx_base + mlen * rear;

			rear = (rear + 1) % qlen;
			if (((rd(m) & SEQNUM_MASK) >> SEQNUM_SHIFT) == mine) {
				for (unsigned i = 0; i < 4; i++) {
					cmd[i] = rd(m + 4 * i);
				}
				wr(sram + rd(ch + CHAN_TX_REAR), rear);
				goto out;
			}
		}
		wr(sram + rd(ch + CHAN_TX_REAR), rear);
		if (waited > POLL_US) {
			error = ETIMEDOUT;
			goto out;
		}
		delay(20);
	}
bad:
	if (!said_bad) {
		said_bad = true;
		printf("akita acpm: channel %u: the shared memory does not look like ACPM's (%u channels, table at %#x)\n",
		    channel, n, rd(head + SHMEM_CHANS));
	}
	error = error ? error : EINVAL;
out:
	lck_mtx_unlock(&acpm_lock);
	return error;
}
