/*
 * /dev/hakios-ptp: the Pixel 8a's PTP interface (interface 0 of its USB device, as an iPhone's: what a computer shows
 * as the phone's "Internal Storage", its DCIM) to a program of the user space that answers it
 * (userland/src/ptp_responder.c, job com.hakios.ptp) -- what ptpcamerad does on an iPhone through Apple's USB stack.
 *
 *   read()   what the computer sent on the bulk OUT endpoint (PTP containers, as they come), blocking until there is
 *            some; ENXIO while no computer has the interface configured
 *   write()  bytes for the bulk IN endpoint, sent in transfers of at most 16 KiB, each waited for; a write of 0 bytes
 *            sends a zero-length packet (the end of a data phase whose length is a multiple of 512); EIO if the
 *            computer goes meanwhile
 *
 * The transfers themselves are started by the USB driver's poll thread (hakios.akita_usb.c: ptp_service), as all of
 * its endpoints' are: here only the buffers and the waiting.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <miscfs/devfs/devfs.h>
#include <sys/conf.h>
#include <sys/errno.h>
#include <sys/proc.h>
#include <sys/systm.h>
#include <sys/uio.h>

#define RING            (64 * 1024)
#define CHUNK           16384

static LCK_GRP_DECLARE(ptp_grp, "akita ptp");
static LCK_MTX_DECLARE(ptp_lock, &ptp_grp);
static uint8_t out_ring[RING];
static uint32_t out_head, out_fill;             /* what came in and was not read yet */
static uint8_t in_buf[CHUNK];
static uint32_t in_len;
static int in_state;                            /* 0 free, 1 left for the poll thread, 2 out on the bus */
static bool link_up;
static uint32_t link_generation;

/* from the USB driver's poll thread ---------------------------------------------------------------------------- */

void akita_ptp_link(int up);
void akita_ptp_out(const uint8_t *data, uint32_t len);
bool akita_ptp_out_room(void);
bool akita_ptp_in_take(uint8_t *buf, uint32_t max, uint32_t *len);
void akita_ptp_in_done(void);

void
akita_ptp_link(int up)
{
	lck_mtx_lock(&ptp_lock);
	link_up = up != 0;
	link_generation++;
	out_head = out_fill = 0;
	in_state = 0;
	lck_mtx_unlock(&ptp_lock);
	wakeup(&out_fill);
	wakeup(&in_state);
}

void
akita_ptp_out(const uint8_t *data, uint32_t len)
{
	lck_mtx_lock(&ptp_lock);
	for (uint32_t i = 0; i < len && out_fill < RING; i++) {
		out_ring[(out_head + out_fill) % RING] = data[i];
		out_fill++;
	}
	lck_mtx_unlock(&ptp_lock);
	wakeup(&out_fill);
}

bool
akita_ptp_out_room(void)
{
	return RING - out_fill >= CHUNK;        /* (a read without the lock: at worst a transfer later) */
}

bool
akita_ptp_in_take(uint8_t *buf, uint32_t max, uint32_t *len)
{
	bool took = false;

	lck_mtx_lock(&ptp_lock);
	if (in_state == 1 && in_len <= max) {
		memcpy(buf, in_buf, in_len);
		*len = in_len;
		in_state = 2;
		took = true;
	}
	lck_mtx_unlock(&ptp_lock);
	return took;
}

void
akita_ptp_in_done(void)
{
	lck_mtx_lock(&ptp_lock);
	if (in_state == 2) {
		in_state = 0;
	}
	lck_mtx_unlock(&ptp_lock);
	wakeup(&in_state);
}

/* the device node --------------------------------------------------------------------------------------------- */

static int
ptp_open(__unused dev_t dev, __unused int flags, __unused int devtype, __unused proc_t p)
{
	return 0;
}

static int
ptp_close(__unused dev_t dev, __unused int flags, __unused int devtype, __unused proc_t p)
{
	return 0;
}

/*
 * Every change of the link is told to the reader, however short: a bus reset and the host's new configuration can both
 * happen while it waits, and it must still see that the computer went -- the computer's trust goes with it (a reset
 * with the pairing removed was still shown the photos). The first read after a change is ENXIO.
 */
static uint32_t read_generation;

static int
ptp_read(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	int error = 0;

	lck_mtx_lock(&ptp_lock);
	while (out_fill == 0 || read_generation != link_generation) {
		if (!link_up || read_generation != link_generation) {
			read_generation = link_generation;
			lck_mtx_unlock(&ptp_lock);
			return ENXIO;
		}
		error = msleep(&out_fill, &ptp_lock, PCATCH | PRIBIO, "ptp_read", NULL);
		if (error) {
			lck_mtx_unlock(&ptp_lock);
			return error;
		}
	}
	while (out_fill > 0 && uio_resid(uio) > 0 && !error) {
		uint32_t n = RING - out_head;

		if (n > out_fill) {
			n = out_fill;
		}
		if ((user_ssize_t)n > uio_resid(uio)) {
			n = (uint32_t)uio_resid(uio);
		}
		error = uiomove((char *)out_ring + out_head, (int)n, uio);
		if (!error) {
			out_head = (out_head + n) % RING;
			out_fill -= n;
		}
	}
	lck_mtx_unlock(&ptp_lock);
	return error;
}

/* one transfer of len bytes (0: a zero-length packet), waited for */
static int
ptp_send(const uint8_t *data, uint32_t len)
{
	uint32_t generation;
	int error = 0;

	lck_mtx_lock(&ptp_lock);
	while (in_state != 0 && link_up && !error) {
		error = msleep(&in_state, &ptp_lock, PCATCH | PRIBIO, "ptp_wait", NULL);
	}
	if (!link_up || error) {
		lck_mtx_unlock(&ptp_lock);
		return error ? error : EIO;
	}
	generation = link_generation;
	memcpy(in_buf, data, len);
	in_len = len;
	in_state = 1;
	while (in_state != 0 && link_up && generation == link_generation && !error) {
		error = msleep(&in_state, &ptp_lock, PCATCH | PRIBIO, "ptp_write", NULL);
	}
	if (!error && (!link_up || generation != link_generation)) {
		error = EIO;
	}
	lck_mtx_unlock(&ptp_lock);
	return error;
}

static int
ptp_write(__unused dev_t dev, struct uio *uio, __unused int ioflag)
{
	static uint8_t chunk[CHUNK];            /* one writer: the responder */
	int error = 0;

	if (uio_resid(uio) == 0) {
		return ptp_send(chunk, 0);
	}
	while (uio_resid(uio) > 0 && !error) {
		uint32_t n = uio_resid(uio) > CHUNK ? CHUNK : (uint32_t)uio_resid(uio);

		error = uiomove((char *)chunk, (int)n, uio);
		if (!error) {
			error = ptp_send(chunk, n);
		}
	}
	return error;
}

static const struct cdevsw ptp_cdevsw = {
	.d_open = ptp_open,
	.d_close = ptp_close,
	.d_read = ptp_read,
	.d_write = ptp_write,
	.d_ioctl = eno_ioctl,
	.d_stop = eno_stop,
	.d_reset = eno_reset,
	.d_ttys = NULL,
	.d_select = eno_select,
	.d_mmap = eno_mmap,
	.d_strategy = eno_strat,
	.d_reserved_1 = eno_getc,
	.d_reserved_2 = eno_putc,
	.d_type = 0,
};

static void
ptp_node(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	int major = cdevsw_add(-1, &ptp_cdevsw);

	if (major < 0 || devfs_make_node(makedev(major, 0), DEVFS_CHAR, UID_ROOT, GID_WHEEL, 0600, "hakios-ptp") == NULL) {
		printf("akita ptp: /dev/hakios-ptp could not be made\n");
		return;
	}
	printf("akita ptp: /dev/hakios-ptp (the PTP interface's endpoints)\n");
}

/* from the USB driver's start (usbmux=1): the node, once BSD's devfs is there */
void akita_ptp_init(void);

void
akita_ptp_init(void)
{
	static thread_call_t call;
	uint64_t deadline;

	call = thread_call_allocate(ptp_node, NULL);
	clock_interval_to_deadline(3, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(call, deadline);
}

