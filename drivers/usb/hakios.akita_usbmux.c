/*
 * The phone's side of usbmux (usbmux=1), on the Pixel 8a's USB port (hakios.akita_usb.c): what Apple's AppleUSBDeviceMux
 * does on an iPhone. A computer's usbmuxd (or Finder, iTunes) carries TCP-like connections over the "Apple Mobile Device"
 * interface's two bulk endpoints; each one is joined here to a TCP port on the phone itself (127.0.0.1: lockdownd's
 * 62078, and the ports of the services it starts).
 *
 * The protocol, as libimobiledevice's usbmuxd (src/device.c) speaks it to a device; numbers big-endian:
 *   a packet: u32 protocol, u32 length (the whole packet's) -- version 1 of the mux header, which this side answers with
 *   (version 2 adds a magic and sequence numbers; usbmuxd takes either)
 *   protocol 0, version: u32 major, u32 minor, u32 padding. The host's comes first; the answer makes the device usbmuxd's
 *   protocol 6, TCP: a 20-byte TCP header (ports, seq, ack, offset, flags, window in units of 256 bytes), then data.
 *     the host's SYN (seq 0) -> our SYN|ACK (seq 0, ack 1) once 127.0.0.1:port answers, else RST;
 *     data: flags ACK only, acknowledged by us (usbmuxd sends only what our window and acks allow);
 *     RST from either side ends the connection (usbmuxd sends RST when its client goes, we when the port closes).
 *
 * Port 1 is the kernel's own: no socket behind it, the console log instead (osfmk/arm64/loader_return.c's, what the
 * loader gets back from a run: printf, IOLog, and what processes write to /dev/console, a job's StandardErrorPath
 * among them), from its first character still kept on and then as it grows, for as long as the host keeps it. The log of a run while it runs, with nothing but the USB cable; on the host:
 * devices/akita/tools/usbmux-log.py (through usbmuxd's socket).
 *
 * Ports 2 and 3 restart the phone, as `adb reboot bootloader` and `adb reboot` would: the data volume synced first,
 * then the reboot command Google's bootloader reads at its start (PMU + 0x810, Android's pixel-reboot: 0xfc the
 * bootloader, 0 a normal start; written through the PMU's secure-register SMC, as upstream Linux writes Tensor's PMU),
 * then a warm PSCI SYSTEM_RESET2 (a cold SYSTEM_RESET, which port 4 asks for, came back up normally: the command was
 * not kept), the loader leaving EL2 out of the way: the SMC goes to the firmware. From the host:
 * devices/akita/tools/usbmux-reboot.py [bootloader].
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <libkern/OSByteOrder.h>
#include <mach/mach_types.h>
#include <netinet/in.h>
#include <sys/kpi_socket.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <kern/kalloc.h>

/* hakios.akita_usb.c: a packet (its header, then its data) queued for bulk IN; false: no room, or not configured */
bool akita_usb_send(const void *header, uint32_t header_len, const void *data, uint32_t data_len, bool wait);

#define PROTO_VERSION           0
#define PROTO_TCP               6
#define TH_FIN                  0x01
#define TH_SYN                  0x02
#define TH_RST                  0x04
#define TH_ACK                  0x10
#define HDR                     8
#define TCP_HDR                 20
#define MAX_PACKET              16384
#define MAX_DATA                (MAX_PACKET - HDR - TCP_HDR)
#define WINDOW                  (256 * 1024)
#define CONNS                   16
#define RX_MAX                  (256 * 1024)

struct mux_conn {
	bool used, open, klog;
	uint32_t sending;               /* the poll thread in sock_send on it: the reader waits before it closes */
	uint16_t host_port, phone_port;
	uint32_t tx_seq;                /* ours */
	uint32_t rx_next;               /* the host's next sequence number: our ack */
	socket_t so;
};

static struct mux_conn conns[CONNS];
static lck_grp_t *mux_grp;
static lck_mtx_t *mux_lock;
static uint8_t *rx;                     /* bulk OUT, put together into whole packets */
static uint32_t rx_len;
static uint32_t said;

static inline void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static inline void put16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; }
static inline uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; }
static inline uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] << 8 | p[1]); }

/* a TCP packet of the connection to the host; data may be NULL. Holds mux_lock or not: the seq is the caller's */
static bool
send_tcp(struct mux_conn *c, uint8_t flags, uint32_t seq, uint32_t ack, const void *data, uint32_t len, bool wait)
{
	uint8_t p[HDR + TCP_HDR];

	if (len > MAX_DATA) {
		return false;
	}
	bzero(p, sizeof(p));
	put32(p, PROTO_TCP);
	put32(p + 4, HDR + TCP_HDR + len);
	put16(p + HDR, c->phone_port);          /* our source port is the one the host connected to */
	put16(p + HDR + 2, c->host_port);
	put32(p + HDR + 4, seq);
	put32(p + HDR + 8, ack);
	p[HDR + 12] = (TCP_HDR / 4) << 4;
	p[HDR + 13] = flags;
	put16(p + HDR + 14, WINDOW >> 8);
	return akita_usb_send(p, sizeof(p), data, len, wait);
}

static void
conn_free(struct mux_conn *c)
{
	c->used = false;
	c->open = false;
	c->so = NULL;
}

/* one connection's port, read until it closes: what it says goes to the host */
static void
conn_thread(void *parameter, __unused wait_result_t result)
{
	struct mux_conn *c = parameter;
	static uint8_t bufs[CONNS][MAX_DATA];
	uint8_t *buf = bufs[c - conns];
	socket_t so = c->so;

	for (;;) {
		struct iovec iov = { .iov_base = buf, .iov_len = MAX_DATA };
		struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1 };
		size_t got = 0;
		uint32_t seq;

		if (sock_receive(so, &msg, 0, &got) != 0 || got == 0) {
			break;
		}
		lck_mtx_lock(mux_lock);
		if (!c->open) {
			lck_mtx_unlock(mux_lock);
			break;
		}
		seq = c->tx_seq;
		c->tx_seq += (uint32_t)got;
		lck_mtx_unlock(mux_lock);
		if (!send_tcp(c, TH_ACK, seq, c->rx_next, buf, (uint32_t)got, true)) {
			break;
		}
	}
	lck_mtx_lock(mux_lock);
	if (c->open) {
		send_tcp(c, TH_RST, c->tx_seq, c->rx_next, NULL, 0, false);
		if (said++ < 40) {
			printf("akita usbmux: port %u closed (host port %u)\n", c->phone_port, c->host_port);
		}
	}
	c->open = false;
	while (c->sending) {
		lck_mtx_unlock(mux_lock);
		delay_for_interval(1, NSEC_PER_MSEC);
		lck_mtx_lock(mux_lock);
	}
	lck_mtx_unlock(mux_lock);
	sock_close(so);
	lck_mtx_lock(mux_lock);
	conn_free(c);
	lck_mtx_unlock(mux_lock);
}

/* osfmk/arm64/loader_return.c: the console log (printf, IOLog, and what processes write to /dev/console) */
extern size_t loader_log_copy(uint64_t *cursor, char *out, size_t max);

/* port 1: the console log, from its first character still kept on, then what is added, until the host goes */
static void
klog_thread(void *parameter, __unused wait_result_t result)
{
	struct mux_conn *c = parameter;
	static char chunk[MAX_DATA];
	uint64_t cursor = 0;

	while (c->open) {
		size_t n = loader_log_copy(&cursor, chunk, sizeof(chunk));
		uint32_t seq;

		if (n == 0) {
			delay_for_interval(200, NSEC_PER_MSEC);
			continue;
		}
		lck_mtx_lock(mux_lock);
		seq = c->tx_seq;
		c->tx_seq += (uint32_t)n;
		lck_mtx_unlock(mux_lock);
		if (!send_tcp(c, TH_ACK, seq, c->rx_next, chunk, (uint32_t)n, true)) {
			break;
		}
	}
	lck_mtx_lock(mux_lock);
	if (c->open) {
		send_tcp(c, TH_RST, c->tx_seq, c->rx_next, NULL, 0, false);
	}
	conn_free(c);
	lck_mtx_unlock(mux_lock);
}

/*
 * Port 6: a picture of what the panel shows -- what the display's DMA reads, as hakios.akita_display.c finds it -- as a
 * binary PPM (P6, RGB), a comment line first with the DMA's base, stride and size; then the connection is closed.
 * devices/akita/tools/usbmux-shot.py on the host.
 */
extern int akita_display_snapshot(const uint8_t **pixels, uint64_t *phys, uint32_t *width, uint32_t *height, uint32_t *row_bytes);

static bool
screen_send(struct mux_conn *c, const void *data, uint32_t n)
{
	uint32_t seq;

	lck_mtx_lock(mux_lock);
	seq = c->tx_seq;
	c->tx_seq += n;
	lck_mtx_unlock(mux_lock);
	return c->open && send_tcp(c, TH_ACK, seq, c->rx_next, data, n, true);
}

static void
screen_thread(void *parameter, __unused wait_result_t result)
{
	struct mux_conn *c = parameter;
	static char chunk[MAX_DATA];
	const uint8_t *pixels;
	uint64_t phys;
	uint32_t w, h, rb, n = 0;

	if (akita_display_snapshot(&pixels, &phys, &w, &h, &rb) != 0) {
		n = (uint32_t)snprintf(chunk, sizeof(chunk), "no picture: the display reads %#llx (%u x %u, rows of %u bytes), "
		    "outside the scan-out area\n", phys, w, h, rb);
		screen_send(c, chunk, n);
	} else {
		n = (uint32_t)snprintf(chunk, sizeof(chunk), "P6\n# base %#llx stride %u size %ux%u\n%u %u\n255\n", phys, rb, w, h,
		    w, h);
		for (uint32_t y = 0; y < h && c->open; y++) {
			const uint8_t *row = pixels + (uint64_t)y * rb;
			for (uint32_t x = 0; x < w; x++) {
				if (n + 3 > sizeof(chunk)) {
					if (!screen_send(c, chunk, n)) goto done;
					n = 0;
				}
				chunk[n++] = (char)row[4 * x + 2];      /* BGRA in memory */
				chunk[n++] = (char)row[4 * x + 1];
				chunk[n++] = (char)row[4 * x + 0];
			}
		}
		if (n != 0) {
			screen_send(c, chunk, n);
		}
	}
done:
	delay_for_interval(200, NSEC_PER_MSEC);
	lck_mtx_lock(mux_lock);
	if (c->open) {
		send_tcp(c, TH_FIN | TH_ACK, c->tx_seq, c->rx_next, NULL, 0, false);
		send_tcp(c, TH_RST, c->tx_seq + 1, c->rx_next, NULL, 0, false);
	}
	conn_free(c);
	lck_mtx_unlock(mux_lock);
}

#define PMU_REBOOT_CMD          0x15460810UL    /* Android's tree: pixel-reboot, syscon PMU 0x15460000, reboot-cmd-offset 0x810 */
#define REBOOT_BOOTLOADER       0xfc
#define TENSOR_SMC_PMU_SEC_REG  0x82000504
#define PSCI_SYSTEM_RESET       0x84000009
#define PSCI_SYSTEM_RESET2      0xc4000012      /* reset type 0: a warm reset, as Pixel kernels reboot */

extern int sync_internal(void);

static uint64_t
smc(uint64_t function, uint64_t a1, uint64_t a2, uint64_t a3)
{
	register uint64_t x0 __asm__("x0") = function, x1 __asm__("x1") = a1, x2 __asm__("x2") = a2, x3 __asm__("x3") = a3;

	__asm__ volatile ("smc #0" : "+r" (x0), "+r" (x1), "+r" (x2), "+r" (x3) : :
	    "x4", "x5", "x6", "x7", "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17", "memory");
	return x0;
}

static void
reboot_now(bool bootloader, bool cold)
{
	uint64_t was, now;

	printf("akita usbmux: the host asks for a restart%s: the data volume synced first\n",
	    bootloader ? " into the bootloader" : "");
	sync_internal();
	was = smc(TENSOR_SMC_PMU_SEC_REG, PMU_REBOOT_CMD, 0, 0);
	smc(TENSOR_SMC_PMU_SEC_REG, PMU_REBOOT_CMD, 1, bootloader ? REBOOT_BOOTLOADER : 0);
	now = smc(TENSOR_SMC_PMU_SEC_REG, PMU_REBOOT_CMD, 0, 0);
	printf("akita usbmux: reboot command (PMU + 0x810) %#llx -> %#llx; PSCI %s\n", was, now,
	    cold ? "SYSTEM_RESET (cold)" : "SYSTEM_RESET2 (warm)");
	delay_for_interval(500, NSEC_PER_MSEC);         /* for the log to reach the host */
	if (!cold) {
		uint64_t r = smc(PSCI_SYSTEM_RESET2, 0, 0, 0);

		printf("akita usbmux: PSCI SYSTEM_RESET2 came back (%lld): a cold SYSTEM_RESET\n", (long long)r);
	}
	smc(PSCI_SYSTEM_RESET, 0, 0, 0);
	printf("akita usbmux: PSCI SYSTEM_RESET came back\n");
}

/* bsd/sys/msgbuf.h's (as osfmk/arm64/loader_return.c has it): the kernel message buffer, dmesg's */
struct msgbuf {
	int msg_magic, msg_size, msg_bufx, msg_bufr;
	char *msg_bufc;
};
extern struct msgbuf *msgbufp;

/* bytes [from, to) of the message buffer (wrapping) to the host; false when they cannot go */
static bool
dmesg_send(struct mux_conn *c, struct msgbuf *mb, int from, int to, char *chunk)
{
	while (from != to && c->open) {
		uint32_t n = 0, seq;

		while (from != to && n < MAX_DATA) {
			if (mb->msg_bufc[from]) {
				chunk[n++] = mb->msg_bufc[from];
			}
			from = (from + 1) % mb->msg_size;
		}
		if (n == 0) {
			continue;
		}
		lck_mtx_lock(mux_lock);
		seq = c->tx_seq;
		c->tx_seq += n;
		lck_mtx_unlock(mux_lock);
		if (!send_tcp(c, TH_ACK, seq, c->rx_next, chunk, n, true)) {
			return false;
		}
	}
	return true;
}

/*
 * Port 5: the kernel message buffer (what dmesg shows: every printf and IOLog, which without -v go there and not to the
 * console), what it holds and then what is added, until the host goes. Port 1 is the console (processes' output).
 */
static void
dmesg_thread(void *parameter, __unused wait_result_t result)
{
	struct mux_conn *c = parameter;
	static char chunk[MAX_DATA];
	struct msgbuf *mb = msgbufp;
	int pos;

	if (mb == NULL || mb->msg_bufc == NULL || mb->msg_size <= 0 || mb->msg_bufx < 0 || mb->msg_bufx >= mb->msg_size) {
		goto out;
	}
	pos = mb->msg_bufx;
	if ((mb->msg_bufc[mb->msg_size - 1] != 0 && !dmesg_send(c, mb, pos, 0, chunk)) || !dmesg_send(c, mb, 0, pos, chunk)) {
		goto out;
	}
	while (c->open) {
		int end = mb->msg_bufx;

		if (end >= 0 && end < mb->msg_size && end != pos) {
			if (!dmesg_send(c, mb, pos, end, chunk)) {
				break;
			}
			pos = end;
		}
		delay_for_interval(200, NSEC_PER_MSEC);
	}
out:
	lck_mtx_lock(mux_lock);
	if (c->open) {
		send_tcp(c, TH_RST, c->tx_seq, c->rx_next, NULL, 0, false);
	}
	conn_free(c);
	lck_mtx_unlock(mux_lock);
}

static struct mux_conn *
find(uint16_t host_port, uint16_t phone_port)
{
	for (unsigned i = 0; i < CONNS; i++) {
		if (conns[i].used && conns[i].host_port == host_port && conns[i].phone_port == phone_port) {
			return &conns[i];
		}
	}
	return NULL;
}

static void
tcp_syn(uint16_t host_port, uint16_t phone_port, uint32_t seq)
{
	struct sockaddr_in to = { .sin_len = sizeof(to), .sin_family = AF_INET, .sin_port = htons(phone_port) };
	struct mux_conn *c = NULL, refused = { .host_port = host_port, .phone_port = phone_port };
	socket_t so = NULL;
	thread_t thread;
	errno_t err;

	to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	if (phone_port == 2 || phone_port == 3 || phone_port == 4) {
		reboot_now(phone_port != 3, phone_port == 4);
		send_tcp(&refused, TH_RST, 0, seq + 1, NULL, 0, false);
		return;
	}
	if (phone_port == 1 || phone_port == 5 || phone_port == 6) {
		lck_mtx_lock(mux_lock);
		for (unsigned i = 0; i < CONNS && !c; i++) {
			if (!conns[i].used) {
				c = &conns[i];
				bzero(c, sizeof(*c));
				c->used = c->open = c->klog = true;
				c->host_port = host_port;
				c->phone_port = phone_port;
				c->rx_next = seq + 1;
				c->tx_seq = 1;
			}
		}
		lck_mtx_unlock(mux_lock);
		if (!c) {
			send_tcp(&refused, TH_RST, 0, seq + 1, NULL, 0, false);
			return;
		}
		printf("akita usbmux: host port %u: the %s\n", host_port,
		    phone_port == 1 ? "console log" : phone_port == 5 ? "kernel message buffer" : "screen");
		send_tcp(c, TH_SYN | TH_ACK, 0, seq + 1, NULL, 0, false);
		if (kernel_thread_start(phone_port == 1 ? klog_thread : phone_port == 5 ? dmesg_thread : screen_thread, c,
		    &thread) == KERN_SUCCESS) {
			thread_deallocate(thread);
		}
		return;
	}
	lck_mtx_lock(mux_lock);
	for (unsigned i = 0; i < CONNS && !c; i++) {
		if (!conns[i].used) {
			c = &conns[i];
			bzero(c, sizeof(*c));
			c->used = true;
			c->host_port = host_port;
			c->phone_port = phone_port;
		}
	}
	lck_mtx_unlock(mux_lock);
	if (!c) {
		printf("akita usbmux: no room for a connection to port %u\n", phone_port);
		send_tcp(&refused, TH_RST, 0, seq + 1, NULL, 0, false);
		return;
	}
	err = sock_socket(PF_INET, SOCK_STREAM, IPPROTO_TCP, NULL, NULL, &so);
	if (!err) {
		err = sock_connect(so, (struct sockaddr *)&to, 0);
	}
	if (err) {
		if (said++ < 40) {
			printf("akita usbmux: host port %u to port %u: refused (%d)\n", host_port, phone_port, err);
		}
		if (so) {
			sock_close(so);
		}
		send_tcp(&refused, TH_RST, 0, seq + 1, NULL, 0, false);
		lck_mtx_lock(mux_lock);
		conn_free(c);
		lck_mtx_unlock(mux_lock);
		return;
	}
	lck_mtx_lock(mux_lock);
	c->so = so;
	c->open = true;
	c->rx_next = seq + 1;
	c->tx_seq = 1;
	lck_mtx_unlock(mux_lock);
	if (said++ < 40) {
		printf("akita usbmux: host port %u to port %u: connected\n", host_port, phone_port);
	}
	send_tcp(c, TH_SYN | TH_ACK, 0, seq + 1, NULL, 0, false);
	if (kernel_thread_start(conn_thread, c, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		sock_shutdown(so, SHUT_RDWR);
	}
}

static void
tcp_input(const uint8_t *t, uint32_t len)
{
	uint16_t host_port = get16(t), phone_port = get16(t + 2);
	uint32_t seq = get32(t + 4), off = (t[12] >> 4) * 4;
	uint8_t flags = t[13];
	struct mux_conn *c;

	if (len < TCP_HDR || off < TCP_HDR || off > len) {
		return;
	}
	if (flags & TH_SYN) {
		tcp_syn(host_port, phone_port, seq);
		return;
	}
	lck_mtx_lock(mux_lock);
	c = find(host_port, phone_port);
	if (!c || !c->open) {
		lck_mtx_unlock(mux_lock);
		return;
	}
	if (flags & (TH_RST | TH_FIN)) {
		/* the reader closes it; shut down under the lock, before the reader can see it closed and free the socket */
		c->open = false;
		if (c->so) {
			sock_shutdown(c->so, SHUT_RDWR);
		}
		lck_mtx_unlock(mux_lock);
		return;
	}
	lck_mtx_unlock(mux_lock);
	if (len > off && c->klog) {
		lck_mtx_lock(mux_lock);
		c->rx_next = seq + (len - off);         /* whatever the host says on the log's port: taken, unread */
		lck_mtx_unlock(mux_lock);
		send_tcp(c, TH_ACK, c->tx_seq, c->rx_next, NULL, 0, false);
	} else if (len > off) {
		struct iovec iov = { .iov_base = (void *)(uintptr_t)(t + off), .iov_len = len - off };
		struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1 };
		size_t sent = 0;
		errno_t err;

		lck_mtx_lock(mux_lock);
		if (!c->open) {
			lck_mtx_unlock(mux_lock);
			return;
		}
		c->sending++;
		lck_mtx_unlock(mux_lock);
		err = sock_send(c->so, &msg, 0, &sent);
		lck_mtx_lock(mux_lock);
		c->sending--;
		lck_mtx_unlock(mux_lock);
		if (err != 0) {
			lck_mtx_lock(mux_lock);
			if (c->open) {
				c->open = false;
				sock_shutdown(c->so, SHUT_RDWR);
			}
			lck_mtx_unlock(mux_lock);
			return;
		}
		lck_mtx_lock(mux_lock);
		c->rx_next = seq + (len - off);
		lck_mtx_unlock(mux_lock);
		send_tcp(c, TH_ACK, c->tx_seq, c->rx_next, NULL, 0, false);
	}
}

static void
packet(const uint8_t *p, uint32_t len)
{
	uint32_t proto = get32(p);

	if (proto == PROTO_VERSION && len >= HDR + 12) {
		uint8_t v[HDR + 12];

		printf("akita usbmux: the host's version %u.%u; answered with 1.0\n", get32(p + HDR), get32(p + HDR + 4));
		bzero(v, sizeof(v));
		put32(v, PROTO_VERSION);
		put32(v + 4, sizeof(v));
		put32(v + HDR, 1);
		akita_usb_send(v, sizeof(v), NULL, 0, false);
	} else if (proto == PROTO_TCP) {
		tcp_input(p + HDR, len - HDR);
	} else if (said++ < 40) {
		printf("akita usbmux: a packet of protocol %u, %u bytes: ignored\n", proto, len);
	}
}

/* from bulk OUT, as it comes (the poll thread): put together into packets */
void
akita_usbmux_input(const uint8_t *data, uint32_t len)
{
	if (!rx) {
		return;
	}
	if (rx_len + len > RX_MAX) {
		printf("akita usbmux: %u bytes waiting for a whole packet: dropped\n", rx_len + len);
		rx_len = 0;
		return;
	}
	memcpy(rx + rx_len, data, len);
	rx_len += len;
	while (rx_len >= HDR) {
		uint32_t plen = get32(rx + 4);

		if (plen < HDR || plen > RX_MAX) {
			printf("akita usbmux: a packet of %u bytes cannot be: the rest dropped\n", plen);
			rx_len = 0;
			return;
		}
		if (rx_len < plen) {
			return;
		}
		packet(rx, plen);
		memmove(rx, rx + plen, rx_len - plen);
		rx_len -= plen;
	}
}

/* the host went (reset, disconnect, deconfigured): every connection closed */
void
akita_usbmux_reset(void)
{
	rx_len = 0;
	if (!mux_lock) {
		return;
	}
	lck_mtx_lock(mux_lock);
	for (unsigned i = 0; i < CONNS; i++) {
		if (conns[i].used && conns[i].open) {
			conns[i].open = false;
			if (conns[i].so) {
				sock_shutdown(conns[i].so, SHUT_RDWR);
			}
		}
	}
	lck_mtx_unlock(mux_lock);
}

void
akita_usbmux_init(void)
{
	mux_grp = lck_grp_alloc_init("akita usbmux", LCK_GRP_ATTR_NULL);
	mux_lock = lck_mtx_alloc_init(mux_grp, LCK_ATTR_NULL);
	rx = kalloc_data(RX_MAX, Z_WAITOK | Z_ZERO);
}
