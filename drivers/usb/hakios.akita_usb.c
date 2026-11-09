/*
 * The Pixel 8a's USB port as an iPhone's (usbmux=1): zuma's DWC3 (usb@11210000) in device mode, polled, enumerating as
 * Apple's iPhone (05ac:12a8) with the "Apple Mobile Device" interface a computer's usbmuxd looks for (vendor class
 * 0xff, subclass 0xfe, protocol 2: two bulk endpoints, the usbmux protocol on them).
 *
 * Enumeration and the bulk endpoints; what comes in on bulk OUT goes to the mux (hakios.akita_usbmux.c), and what it
 * sends goes out on bulk IN through a queue the poll thread empties, a packet at a time. Interface 0 is PTP, as on an
 * iPhone (the photos, read with a camera's protocol): its endpoints go to /dev/hakios-ptp (hakios.akita_ptp.c), where
 * a program of the user space answers (userland/src/ptp_responder.c).
 *
 * The controller: Google's bootloader drives it for its own fastboot, and U-Boot for its own (xl), so its clocks, the
 * eUSB2 PHY and the repeater are up when the kernel starts (U-Boot's `eusb init`). Here: the controller stopped and
 * soft-reset (whatever U-Boot's gadget left), the event buffer, endpoint 0, run. Registers as Linux's drivers/usb/dwc3
 * (core.h, gadget.h, ep0.c) name them. Polled (the event buffer's interrupt masked) every millisecond from a thread.
 *
 * Memory the controller reads and writes (the event buffer, TRBs, buffers): one physically contiguous piece, used through
 * a non-cacheable mapping of it (ml_io_map_wcomb); its cached mapping is cleaned once first, so that no dirty line of it
 * is written back over what the controller wrote.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/startup.h>
#include <kern/thread.h>
#include <kern/thread_call.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <pexpert/pexpert.h>
#include <sys/systm.h>
#include <vm/vm_kern.h>

extern vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern vm_offset_t ml_io_map_wcomb(vm_offset_t phys_addr, vm_size_t size);
extern void FlushPoC_DcacheRegion(vm_offset_t va, size_t length);
extern vm_offset_t kvtophys(vm_offset_t va);

#define DWC3_PHYS               0x11210000UL
#define DWC3_SIZE               0x10000

/* global and device registers */
#define GCTL                    0xc110
#define GSNPSID                 0xc120
#define GUSB2PHYCFG             0xc200
#define GEVNTADRLO              0xc400
#define GEVNTADRHI              0xc404
#define GEVNTSIZ                0xc408
#define GEVNTCOUNT              0xc40c
#define DCFG                    0xc700
#define DCTL                    0xc704
#define DEVTEN                  0xc708
#define DSTS                    0xc70c
#define DALEPENA                0xc720
#define DEPCMDPAR2(n)           (0xc800 + (n) * 0x10)
#define DEPCMDPAR1(n)           (0xc804 + (n) * 0x10)
#define DEPCMDPAR0(n)           (0xc808 + (n) * 0x10)
#define DEPCMD(n)               (0xc80c + (n) * 0x10)

#define GCTL_PRTCAP_MASK        (3u << 12)
#define GCTL_PRTCAP_DEVICE      (2u << 12)
#define GEVNTSIZ_INTMASK        (1u << 31)
#define DCTL_RUN_STOP           (1u << 31)
#define DCTL_CSFTRST            (1u << 30)
#define DSTS_DEVCTRLHLT         (1u << 22)
#define DSTS_SPEED(s)           ((s) & 7)
#define DEVTEN_DISCONN          (1u << 0)
#define DEVTEN_USBRST           (1u << 1)
#define DEVTEN_CONNDONE         (1u << 2)

#define CMD_SETEPCONFIG         1
#define CMD_SETTRANSFRESOURCE   2
#define CMD_SETSTALL            4
#define CMD_CLEARSTALL          5
#define CMD_STARTTRANSFER       6
#define CMD_ENDTRANSFER         8
#define CMD_DEPSTARTCFG         9
#define CMD_IOC                 (1u << 8)
#define CMD_ACT                 (1u << 10)
#define CMD_FORCERM             (1u << 11)
#define CMD_PARAM(x)            ((uint32_t)(x) << 16)
#define CMD_STATUS(c)           (((c) >> 12) & 0xf)
#define CMD_RESOURCE(c)         (((c) >> 16) & 0x7f)

#define CFG0_TYPE(t)            ((uint32_t)(t) << 1)
#define CFG0_MPS(m)             ((uint32_t)(m) << 3)
#define CFG0_FIFO(f)            ((uint32_t)(f) << 17)
#define CFG0_MODIFY             (2u << 30)
#define CFG1_XFER_COMPLETE      (1u << 8)
#define CFG1_XFER_NOT_READY     (1u << 10)
#define CFG1_EP(n)              ((uint32_t)(n) << 25)

#define TRB_HWO                 (1u << 0)
#define TRB_LST                 (1u << 1)
#define TRB_ISP_IMI             (1u << 10)
#define TRB_IOC                 (1u << 11)
#define TRB_NORMAL              (1u << 4)
#define TRB_SETUP               (2u << 4)
#define TRB_STATUS2             (3u << 4)
#define TRB_STATUS3             (4u << 4)
#define TRB_DATA                (5u << 4)

/* physical endpoints: 0 ep0 OUT, 1 ep0 IN, 2 ep1 OUT, 3 ep1 IN */
#define EP0_OUT                 0
#define EP0_IN                  1
#define BULK_OUT                2
#define BULK_IN                 3
#define BULK_MPS                512
#define BULK_BUF                16384
/* PTP (interface 0, as on an iPhone): bulk OUT ep3 (physical 6), bulk IN ep3 (7), interrupt IN ep4 (9) */
#define PTP_OUT                 6
#define PTP_IN                  7
#define PTP_INT                 9

/* the piece of memory the controller uses */
#define MEM_SIZE                (128 * 1024)
#define EVT_OFF                 0x0000          /* 4 KiB event buffer */
#define EVT_SIZE                4096
#define TRB_OFF                 0x1000          /* a TRB per physical endpoint, 16 bytes each, 64 apart */
#define SETUP_OFF               0x1400          /* the 8 bytes of a SETUP */
#define EP0_BUF_OFF             0x1800          /* ep0's data stage, up to 512 bytes */
#define OUT_BUF_OFF             0x4000
#define IN_BUF_OFF              0x8000
#define PTP_OUT_BUF_OFF         0x10000
#define PTP_IN_BUF_OFF          0x14000

struct trb {
	uint32_t bpl, bph, size, ctrl;
};

static vm_offset_t regs, mem;           /* mem: the non-cacheable mapping */
static uint64_t mem_pa;
static uint32_t evt_pos;
/* each physical endpoint's transfer resource (DEPCMD's), by physical number: up to 32 on DWC3. It was 4, and PTP's
 * endpoints 6, 7 (and the interrupt one, 9) wrote past it: over in_busy/ptp_*_running/in_zlp, and over the eUSB2 link's
 * register mappings -- the replug's re-init then wrote through 0x600000000 and panicked (11-09) */
static uint32_t resource[32];
static bool bulk_on;

/* for the battery's charger details (hakios.akita_battery.c): a computer has configured the mux interface */
bool akita_usb_host_configured(void);

bool
akita_usb_host_configured(void)
{
	return bulk_on;
}

/* bulk IN: packets queued by anyone (the mux's connections, the poll thread), sent one at a time by the poll thread */
#define TXQ                     16
static uint8_t txq[TXQ][BULK_BUF];
static uint32_t txq_len[TXQ], txq_head, txq_count;
static bool in_busy, in_zlp;
static lck_grp_t *usb_grp;
static lck_mtx_t *txq_lock;

void akita_usbmux_input(const uint8_t *data, uint32_t len);
void akita_usbmux_reset(void);
void akita_usbmux_init(void);
void akita_ptp_init(void);              /* hakios.akita_ptp.c: /dev/hakios-ptp */
void akita_usbmux_iokit(int connected);  /* hakios.akita_usbmux_iokit.cpp: AppleUSBDeviceMux, for lockdownd */

/* ep0's state: what the next event of it means */
enum { EP0_SETUP, EP0_DATA, EP0_WAIT_STATUS, EP0_STATUS } static ep0_state;
static unsigned ep0_status_ep;          /* the physical endpoint the status stage goes on */
static uint32_t ep0_status_type;        /* its TRB: two stages (no data) or three */
static uint8_t pending_address = 0xff;

static uint32_t seen_setups, seen_bulk_out, bulk_out_bytes, seen_events, start_result = 0xdead;
static char serial[48] = "0000000000000000000000000000000000000000";

static inline uint32_t rd(uint32_t off) { return *(volatile uint32_t *)(regs + off); }
static inline void wr(uint32_t off, uint32_t v) { *(volatile uint32_t *)(regs + off) = v; }

static inline struct trb *
trb_of(unsigned ep)
{
	return (struct trb *)(mem + TRB_OFF + ep * 64);
}

static uint32_t
ep_cmd(unsigned ep, uint32_t cmd, uint32_t p0, uint32_t p1, uint32_t p2)
{
	__builtin_arm_dsb(0xf);
	wr(DEPCMDPAR2(ep), p2);
	wr(DEPCMDPAR1(ep), p1);
	wr(DEPCMDPAR0(ep), p0);
	wr(DEPCMD(ep), cmd | CMD_ACT);
	for (unsigned i = 0; i < 100000; i++) {
		uint32_t c = rd(DEPCMD(ep));

		if (!(c & CMD_ACT)) {
			static unsigned failures;

			if (CMD_STATUS(c) && failures++ < 20) {
				printf("akita usb: endpoint %u command %#x: status %u\n", ep, cmd & 0xf, CMD_STATUS(c));
			}
			return c;
		}
		delay(1);
	}
	printf("akita usb: endpoint %u command %#x did not finish\n", ep, cmd & 0xf);
	return ~0u;
}

static bool
start_transfer(unsigned ep, uint64_t buf_pa, uint32_t len, uint32_t type)
{
	struct trb *t = trb_of(ep);
	uint64_t trb_pa = mem_pa + TRB_OFF + ep * 64;
	uint32_t c;

	t->bpl = (uint32_t)buf_pa;
	t->bph = (uint32_t)(buf_pa >> 32);
	t->size = len;
	t->ctrl = type | TRB_LST | TRB_IOC | TRB_ISP_IMI | TRB_HWO;
	c = ep_cmd(ep, CMD_STARTTRANSFER, (uint32_t)(trb_pa >> 32), (uint32_t)trb_pa, 0);
	resource[ep] = CMD_RESOURCE(c);
	if (c == ~0u || CMD_STATUS(c)) {
		t->ctrl &= ~TRB_HWO;            /* not the controller's after all */
		return false;
	}
	return true;
}

static void
ep0_setup_next(void)
{
	ep0_state = EP0_SETUP;
	start_transfer(EP0_OUT, mem_pa + SETUP_OFF, 8, TRB_SETUP);
}

static void
ep0_stall(void)
{
	ep_cmd(EP0_OUT, CMD_SETSTALL, 0, 0, 0);
	ep0_setup_next();
}

static void
config_ep(unsigned ep, uint32_t type, uint32_t mps, bool modify)
{
	uint32_t p0 = CFG0_TYPE(type) | CFG0_MPS(mps) | ((ep & 1) ? CFG0_FIFO(ep >> 1) : 0) | (modify ? CFG0_MODIFY : 0);

	ep_cmd(ep, CMD_SETEPCONFIG, p0, CFG1_XFER_COMPLETE | CFG1_XFER_NOT_READY | CFG1_EP(ep), 0);
	if (!modify) {
		ep_cmd(ep, CMD_SETTRANSFRESOURCE, 1, 0, 0);
	}
}

/* an endpoint's configuration (action: initialize) when its transfer resource was given already */
static void
config_ep_only(unsigned ep, uint32_t type, uint32_t mps)
{
	uint32_t p0 = CFG0_TYPE(type) | CFG0_MPS(mps) | ((ep & 1) ? CFG0_FIFO(ep >> 1) : 0);

	ep_cmd(ep, CMD_SETEPCONFIG, p0, CFG1_XFER_COMPLETE | CFG1_XFER_NOT_READY | CFG1_EP(ep), 0);
}

/* descriptors */
static const uint8_t dev_desc[18] = {
	18, 1, 0x00, 0x02, 0, 0, 0, 64,
	0xac, 0x05, 0xa8, 0x12,                 /* Apple, iPhone (usbmuxd takes 0x1290..0x12af) */
	0x01, 0x10,                             /* bcdDevice 0x1001 */
	1, 2, 3, 1
};
static const uint8_t qualifier[10] = { 10, 6, 0x00, 0x02, 0, 0, 0, 64, 1, 0 };
/*
 * The BOS: a header and a USB 2.0 Extension with nothing in it (no link power management). Windows' Apple driver asks for
 * it after the charging request (GET_DESCRIPTOR 0x0f00) and, refused, sent nothing more: the device was given up.
 */
static const uint8_t bos_desc[12] = { 5, 0x0f, 12, 0, 1, 7, 0x10, 0x02, 0, 0, 0, 0 };
static const uint8_t cfg_desc[62] = {
	9, 2, 62, 0, 2, 1, 0, 0xc0, 250,        /* two interfaces, self-powered, 500 mA */
	9, 4, 0, 0, 3, 0x06, 0x01, 0x01, 5,     /* PTP (Still Image Capture), as an iPhone's interface 0: the photos */
	7, 5, 0x83, 2, BULK_MPS & 0xff, BULK_MPS >> 8, 0,
	7, 5, 0x03, 2, BULK_MPS & 0xff, BULK_MPS >> 8, 0,
	7, 5, 0x84, 3, 64, 0, 8,                /* its events: never sent */
	9, 4, 1, 0, 2, 0xff, 0xfe, 2, 4,        /* Apple Mobile Device: usbmux */
	7, 5, 0x81, 2, BULK_MPS & 0xff, BULK_MPS >> 8, 0,
	7, 5, 0x01, 2, BULK_MPS & 0xff, BULK_MPS >> 8, 0,
};

static unsigned
string_desc(unsigned index, uint8_t *out)
{
	static const char *const strings[] = { NULL, "Apple Inc.", "iPhone", serial, "Apple Mobile Device", "PTP" };
	const char *s;
	unsigned n = 2;

	if (index == 0) {
		out[0] = 4; out[1] = 3; out[2] = 0x09; out[3] = 0x04;
		return 4;
	}
	if (index >= sizeof(strings) / sizeof(strings[0])) {
		return 0;
	}
	for (s = strings[index]; *s && n < 254; s++) {
		out[n++] = (uint8_t)*s;
		out[n++] = 0;
	}
	out[0] = (uint8_t)n;
	out[1] = 3;
	return n;
}

/*
 * PTP's endpoints (hakios.akita_ptp.c has the device node and its buffers): the poll thread starts every transfer, as
 * for the mux. OUT is started again whenever the node has room for a whole transfer; IN takes what a writer left.
 */
void akita_ptp_link(int up);                                    /* the host configured (1) or went (0) */
void akita_ptp_out(const uint8_t *data, uint32_t len);          /* what came in */
bool akita_ptp_out_room(void);                                  /* room for BULK_BUF more */
bool akita_ptp_in_take(uint8_t *buf, uint32_t max, uint32_t *len); /* the next IN transfer, if a writer left one */
void akita_ptp_in_done(void);                                   /* ... and it is out */
static bool ptp_out_running, ptp_in_busy, ptp_failed;     /* failed: a start was refused; not again until configured */

static bool
ptp_service(void)
{
	uint32_t len;

	if (!bulk_on || ptp_failed) {
		return false;
	}
	if (!ptp_out_running && akita_ptp_out_room()) {
		ptp_out_running = true;
		if (!start_transfer(PTP_OUT, mem_pa + PTP_OUT_BUF_OFF, BULK_BUF, TRB_NORMAL)) {
			printf("akita usb: PTP's OUT endpoint refused a transfer: PTP off until the host configures again\n");
			ptp_failed = true;
			return false;
		}
	}
	if (!ptp_in_busy && akita_ptp_in_take((uint8_t *)(mem + PTP_IN_BUF_OFF), BULK_BUF, &len)) {
		ptp_in_busy = true;
		if (!start_transfer(PTP_IN, mem_pa + PTP_IN_BUF_OFF, len, TRB_NORMAL)) {
			printf("akita usb: PTP's IN endpoint refused a transfer: PTP off until the host configures again\n");
			ptp_failed = true;
			ptp_in_busy = false;
			akita_ptp_in_done();
			return false;
		}
		return true;
	}
	return ptp_in_busy;
}

static void
bulk_start_out(void)
{
	start_transfer(BULK_OUT, mem_pa + OUT_BUF_OFF, BULK_BUF, TRB_NORMAL);
}

static void
bulk_enable(void)
{
	if (bulk_on) {
		return;
	}
	/*
	 * Transfer resources after DEPSTARTCFG from resource 2: every physical endpoint's in order, used or not; then each
	 * used one configured. (Physical endpoint 4, EP2 OUT, refused every Start Transfer on this phone -- "no resource",
	 * status 1 -- however it was given its resource and wherever its buffer was; PTP is on EP3, which works.)
	 */
	ep_cmd(EP0_OUT, CMD_DEPSTARTCFG | CMD_PARAM(2), 0, 0, 0);
	for (unsigned ep = 2; ep < 10; ep++) {
		ep_cmd(ep, CMD_SETTRANSFRESOURCE, 1, 0, 0);
	}
	config_ep_only(BULK_OUT, 2, BULK_MPS);
	config_ep_only(BULK_IN, 2, BULK_MPS);
	config_ep_only(PTP_OUT, 2, BULK_MPS);
	config_ep_only(PTP_IN, 2, BULK_MPS);
	wr(DALEPENA, rd(DALEPENA) | (1u << BULK_OUT) | (1u << BULK_IN) | (1u << PTP_OUT) | (1u << PTP_IN));
	bulk_on = true;
	bulk_start_out();
	ptp_out_running = false;
	ptp_in_busy = false;
	ptp_failed = false;
	ptp_service();                          /* PTP's OUT started too */
	akita_ptp_link(1);
	printf("akita usb: configured: the mux interface's bulk endpoints are on\n");
	akita_usbmux_iokit(1);
}

static void
bulk_disable(void)
{
	if (!bulk_on) {
		return;
	}
	ep_cmd(BULK_OUT, CMD_ENDTRANSFER | CMD_FORCERM | CMD_PARAM(resource[BULK_OUT]), 0, 0, 0);
	if (in_busy) {
		ep_cmd(BULK_IN, CMD_ENDTRANSFER | CMD_FORCERM | CMD_PARAM(resource[BULK_IN]), 0, 0, 0);
	}
	if (ptp_out_running) {
		ep_cmd(PTP_OUT, CMD_ENDTRANSFER | CMD_FORCERM | CMD_PARAM(resource[PTP_OUT]), 0, 0, 0);
	}
	if (ptp_in_busy) {
		ep_cmd(PTP_IN, CMD_ENDTRANSFER | CMD_FORCERM | CMD_PARAM(resource[PTP_IN]), 0, 0, 0);
	}
	ptp_out_running = ptp_in_busy = false;
	akita_ptp_link(0);
	wr(DALEPENA, rd(DALEPENA) & ~((1u << BULK_OUT) | (1u << BULK_IN) | (1u << PTP_OUT) | (1u << PTP_IN)));
	bulk_on = false;
	lck_mtx_lock(txq_lock);
	in_busy = in_zlp = false;
	txq_count = 0;
	lck_mtx_unlock(txq_lock);
	akita_usbmux_reset();
	akita_usbmux_iokit(0);
}

/* a packet for the host: its header and its data, put together in a slot of the queue (wait: for room, up to 5 s) */
bool
akita_usb_send(const void *header, uint32_t header_len, const void *data, uint32_t data_len, bool wait)
{
	if (header_len + data_len > BULK_BUF) {
		return false;
	}
	for (unsigned tries = 0; tries < 5000; tries++) {
		if (!bulk_on) {
			return false;
		}
		lck_mtx_lock(txq_lock);
		if (txq_count < TXQ) {
			unsigned slot = (txq_head + txq_count) % TXQ;

			memcpy(txq[slot], header, header_len);
			if (data_len) {
				memcpy(txq[slot] + header_len, data, data_len);
			}
			txq_len[slot] = header_len + data_len;
			txq_count++;
			lck_mtx_unlock(txq_lock);
			return true;
		}
		lck_mtx_unlock(txq_lock);
		if (!wait) {
			break;
		}
		delay_for_interval(1, NSEC_PER_MSEC);
	}
	printf("akita usb: bulk IN queue full: a packet dropped\n");
	return false;
}

/* the poll thread: the next queued packet onto bulk IN, if it is free; true while there is something to send */
static bool
tx_service(void)
{
	uint32_t len;

	if (!bulk_on || in_busy) {
		return in_busy;
	}
	lck_mtx_lock(txq_lock);
	if (txq_count == 0) {
		lck_mtx_unlock(txq_lock);
		return false;
	}
	len = txq_len[txq_head];
	memcpy((void *)(mem + IN_BUF_OFF), txq[txq_head], len);
	lck_mtx_unlock(txq_lock);
	in_busy = true;
	in_zlp = len % BULK_MPS == 0;           /* a multiple of the packet size ends with an empty packet */
	start_transfer(BULK_IN, mem_pa + IN_BUF_OFF, len, TRB_NORMAL);
	return true;
}

/* a SETUP has come: answer it (data stage started, or the status stage expected) or stall */
static void
ep0_request(void)
{
	const uint8_t *s = (const uint8_t *)(mem + SETUP_OFF);
	uint8_t *buf = (uint8_t *)(mem + EP0_BUF_OFF);
	unsigned type = s[0], req = s[1], value = s[2] | s[3] << 8, index = s[4] | s[5] << 8, length = s[6] | s[7] << 8;
	unsigned n = 0;
	bool in = (type & 0x80) != 0, ok = true;

	if (seen_setups++ < 300) {
		printf("akita usb: setup %02x %02x value %#x index %#x length %u\n", type, req, value, index, length);
	}
	if (type == 0x40 && req == 0x40 && length == 0) {
		/*
		 * Apple's vendor request for the charging current (Windows' Apple driver and Linux's apple-mfi-fastcharge send
		 * it; wValue the current asked for). An iPhone takes it; refused, Windows' driver asked it again twice a second
		 * and went no further: no phone in Explorer, no Trust prompt. Taken, and the charger left as it is.
		 */
		static unsigned said;

		if (said++ < 3) {
			printf("akita usb: the host's charging request (vendor 0x40, value %#x): taken\n", value);
		}
	} else if ((type & 0x60) != 0) {
		ok = false;                     /* class or other vendor requests: none yet */
	} else if (req == 6 && in) {            /* GET_DESCRIPTOR */
		switch (value >> 8) {
		case 1: memcpy(buf, dev_desc, n = sizeof(dev_desc)); break;
		case 2: memcpy(buf, cfg_desc, n = sizeof(cfg_desc)); break;
		case 3: n = string_desc(value & 0xff, buf); ok = n != 0; break;
		case 6: memcpy(buf, qualifier, n = sizeof(qualifier)); break;
		case 15: memcpy(buf, bos_desc, n = sizeof(bos_desc)); break;
		default: ok = false;
		}
	} else if (req == 5 && !in) {           /* SET_ADDRESS */
		pending_address = value & 0x7f;
		wr(DCFG, (rd(DCFG) & ~(0x7fu << 3)) | (pending_address << 3));
	} else if (req == 9 && !in) {           /* SET_CONFIGURATION */
		if (value == 1) {
			bulk_enable();
		} else {
			bulk_disable();
		}
	} else if (req == 8 && in) {            /* GET_CONFIGURATION */
		buf[0] = bulk_on ? 1 : 0;
		n = 1;
	} else if (req == 0 && in) {            /* GET_STATUS */
		buf[0] = (type & 0x1f) == 0 ? 1 : 0;    /* the device: self-powered */
		buf[1] = 0;
		n = 2;
	} else if (req == 10 && in) {           /* GET_INTERFACE */
		buf[0] = 0;
		n = 1;
	} else if (req == 11 || req == 1 || req == 3) {   /* SET_INTERFACE 0, CLEAR_FEATURE, SET_FEATURE */
		;
	} else {
		ok = false;
	}
	if (!ok) {
		if (seen_setups < 300) {
			printf("akita usb: setup %02x %02x: stalled\n", type, req);
		}
		ep0_stall();
		return;
	}
	if (length == 0) {
		ep0_state = EP0_WAIT_STATUS;
		ep0_status_ep = EP0_IN;         /* two stages: the status goes IN */
		ep0_status_type = TRB_STATUS2;
		return;
	}
	ep0_status_type = TRB_STATUS3;
	if (in) {
		if (n > length) {
			n = length;
		}
		ep0_state = EP0_DATA;
		start_transfer(EP0_IN, mem_pa + EP0_BUF_OFF, n, TRB_DATA);
		ep0_status_ep = EP0_OUT;
	} else {
		/* OUT data we do not use: taken and dropped */
		ep0_state = EP0_DATA;
		start_transfer(EP0_OUT, mem_pa + EP0_BUF_OFF, (length + 63) & ~63u, TRB_DATA);
		ep0_status_ep = EP0_IN;
	}
}

static void
ep0_event(unsigned ep, unsigned event, unsigned status)
{
	if (event == 1) {                       /* XferComplete */
		switch (ep0_state) {
		case EP0_SETUP:
			ep0_request();
			break;
		case EP0_DATA:
			ep0_state = EP0_WAIT_STATUS;
			break;
		case EP0_STATUS:
			ep0_setup_next();
			break;
		default:
			break;
		}
	} else if (event == 3 && (status & 3) == 2 && ep0_state == EP0_WAIT_STATUS && ep == ep0_status_ep) {
		/* XferNotReady, the status stage: the host asks for it now */
		ep0_state = EP0_STATUS;
		start_transfer(ep, mem_pa + EP0_BUF_OFF, 0, ep0_status_type);
	}
}

static void
bulk_event(unsigned ep, unsigned event)
{
	if (ep == PTP_OUT && event == 1) {
		struct trb *t = trb_of(PTP_OUT);
		ptp_out_running = false;
		akita_ptp_out((const uint8_t *)(mem + PTP_OUT_BUF_OFF), BULK_BUF - (t->size & 0xffffff));
		ptp_service();
		return;
	}
	if (ep == PTP_IN && event == 1) {
		ptp_in_busy = false;
		akita_ptp_in_done();
		ptp_service();
		return;
	}
	if (ep == BULK_OUT && event == 1) {
		struct trb *t = trb_of(BULK_OUT);
		uint32_t got = BULK_BUF - (t->size & 0xffffff);
		const uint8_t *d = (const uint8_t *)(mem + OUT_BUF_OFF);

		bulk_out_bytes += got;
		if (seen_bulk_out++ < 8) {
			printf("akita usb: bulk OUT %u bytes: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x "
			    "%02x %02x %02x %02x\n", got, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9], d[10],
			    d[11], d[12], d[13], d[14], d[15]);
		}
		akita_usbmux_input(d, got);
		bulk_start_out();
	} else if (ep == BULK_IN && event == 1) {
		if (in_zlp) {
			in_zlp = false;
			start_transfer(BULK_IN, mem_pa + IN_BUF_OFF, 0, TRB_NORMAL);
			return;
		}
		lck_mtx_lock(txq_lock);
		if (txq_count) {
			txq_head = (txq_head + 1) % TXQ;
			txq_count--;
		}
		in_busy = false;
		lck_mtx_unlock(txq_lock);
	}
}

static void
device_event(unsigned type)
{
	switch (type) {
	case 0:                                 /* disconnect */
		printf("akita usb: disconnected\n");
		bulk_disable();
		break;
	case 1:                                 /* USB reset */
		printf("akita usb: bus reset\n");
		bulk_disable();
		wr(DCFG, rd(DCFG) & ~(0x7fu << 3));
		break;
	case 2: {                               /* connection done: the speed is known */
		uint32_t speed = DSTS_SPEED(rd(DSTS));

		printf("akita usb: connected at %s speed\n", speed == 0 ? "high" : speed == 1 || speed == 3 ? "full" :
		    speed == 4 ? "super" : "some");
		config_ep(EP0_OUT, 0, 64, true);
		config_ep(EP0_IN, 0, 64, true);
		break;
	}
	default:
		break;
	}
}

/* the controller's events, handled; true if there were any */
static bool
poll_events(void)
{
	uint32_t count = rd(GEVNTCOUNT) & 0xffff;
	bool any = count >= 4;

	while (count >= 4) {
		uint32_t e = *(volatile uint32_t *)(mem + EVT_OFF + evt_pos);

		evt_pos = (evt_pos + 4) % EVT_SIZE;
		seen_events++;
		count -= 4;
		wr(GEVNTCOUNT, 4);
		if (e & 1) {
			if (((e >> 1) & 0x7f) == 0) {
				device_event((e >> 8) & 0xf);
			}
		} else {
			unsigned ep = (e >> 1) & 0x1f, event = (e >> 6) & 0xf, status = (e >> 12) & 0xf;

			if (ep <= EP0_IN) {
				ep0_event(ep, event, status);
			} else {
				bulk_event(ep, event);
			}
		}
	}
	return any;
}

static bool
controller_start(void)
{
	uint32_t id = rd(GSNPSID);

	printf("akita usb: DWC3 %#x (GSNPSID); DCTL %#x DSTS %#x GCTL %#x GUSB2PHYCFG %#x as found\n", id, rd(DCTL),
	    rd(DSTS), rd(GCTL), rd(GUSB2PHYCFG));
	printf("akita usb: TX FIFOs (GTXFIFOSIZ0-3): %#x %#x %#x %#x; RX %#x; GHWPARAMS3 %#x\n", rd(0xc300), rd(0xc304),
	    rd(0xc308), rd(0xc30c), rd(0xc380), rd(0xc14c));
	if (id == 0 || id == ~0u) {
		printf("akita usb: no DWC3 answers (clocks off?): off\n");
		return false;
	}
	/* stop what U-Boot's gadget left running, then the device controller's soft reset */
	wr(DCTL, rd(DCTL) & ~DCTL_RUN_STOP);
	for (unsigned i = 0; i < 100000 && !(rd(DSTS) & DSTS_DEVCTRLHLT); i++) {
		delay(1);
	}
	wr(DCTL, DCTL_CSFTRST);
	for (unsigned i = 0; i < 100000 && (rd(DCTL) & DCTL_CSFTRST); i++) {
		delay(1);
	}
	if (rd(DCTL) & DCTL_CSFTRST) {
		printf("akita usb: the soft reset did not finish\n");
		return false;
	}
	wr(GCTL, (rd(GCTL) & ~GCTL_PRTCAP_MASK) | GCTL_PRTCAP_DEVICE);

	evt_pos = 0;
	wr(GEVNTADRLO, (uint32_t)(mem_pa + EVT_OFF));
	wr(GEVNTADRHI, (uint32_t)((mem_pa + EVT_OFF) >> 32));
	wr(GEVNTSIZ, GEVNTSIZ_INTMASK | EVT_SIZE);
	wr(GEVNTCOUNT, rd(GEVNTCOUNT) & 0xffff);

	wr(DCFG, rd(DCFG) & ~(0x3ffu));         /* high speed, address 0 */
	ep_cmd(EP0_OUT, CMD_DEPSTARTCFG, 0, 0, 0);
	config_ep(EP0_OUT, 0, 64, false);
	config_ep(EP0_IN, 0, 64, false);
	wr(DALEPENA, (1u << EP0_OUT) | (1u << EP0_IN));
	wr(DEVTEN, DEVTEN_DISCONN | DEVTEN_USBRST | DEVTEN_CONNDONE);
	ep0_setup_next();
	wr(DCTL, rd(DCTL) | DCTL_RUN_STOP);
	printf("akita usb: running as Apple iPhone 05ac:12a8, serial %s\n", serial);
	return true;
}

/* a fresh start of the controller, asked from elsewhere (the battery driver, when the cable comes back): done by the poll
 * thread, which owns the controller */
static volatile int restart_asked;

/*
 * THE eUSB2 LINK AGAIN (U-Boot's `eusb init`, cmd/hakios_eusb.c, as Google's Linux driver brings it up at every plug):
 * when the cable goes the repeater loses power, and the link between the SoC's eUSB2 PHY and it must be made again --
 * a restart of the controller alone left the host seeing nothing (11-09). The link control (ESS_CTL, 0x11100000) and
 * the PHY (0x11110000); the PMU's isolation bit stays as the bootloader left it (the SoC kept its power).
 */
#define USBCON_PHYS             0x11100000UL
#define EUSB_PHYS               0x11110000UL
#define USBCON_LINKCTRL         0x04
#define USBCON_LINK_CLKRST      0x0c
#define USBCON_UTMI_CTRL        0x10
#define EUSB_RST_CTRL           0x00
#define EUSB_CMN_CTRL           0x04
#define EUSB_PLLCFG0            0x08
#define EUSB_PLLCFG1            0x0c
#define EUSB_TXTUNE             0x14
#define EUSB_TESTSE             0x20
static vm_offset_t usbcon_regs, eusb_regs;

static void
clrset(vm_offset_t base, uint32_t off, uint32_t clr, uint32_t set)
{
	volatile uint32_t *r = (volatile uint32_t *)(base + off);

	*r = (*r & ~clr) | set;
}

static void
eusb_reinit(void)
{
	if (usbcon_regs == 0) {
		usbcon_regs = ml_io_map(USBCON_PHYS, 0x1000);
		eusb_regs = ml_io_map(EUSB_PHYS, 0x1000);
	}
	/* the link: Q-channel off, filters, a link reset, VBUS and B-session forced valid */
	clrset(usbcon_regs, USBCON_LINKCTRL, 1u << 8, 1u << 9 | 1u << 10 | 1u << 11 | 1u << 12);
	delay(500);
	clrset(usbcon_regs, USBCON_LINKCTRL, 0xf0, 1u << 8 | 0xf0);
	clrset(usbcon_regs, USBCON_LINK_CLKRST, 0, 1);
	delay(10);
	clrset(usbcon_regs, USBCON_LINK_CLKRST, 1, 0);
	clrset(usbcon_regs, USBCON_UTMI_CTRL, 0, 3);
	/* the eUSB2 PHY: held in reset, repeater mode at 19.2 MHz (PLL feedback 368), out of IDDQ, enabled, out of reset */
	clrset(eusb_regs, EUSB_RST_CTRL, 0, 1u << 0 | 1u << 1 | 1u << 4 | 1u << 5);
	clrset(eusb_regs, EUSB_CMN_CTRL, 7u << 4, 1u << 10);
	clrset(eusb_regs, EUSB_PLLCFG0, 0xfffu << 8, 368u << 8);
	clrset(eusb_regs, EUSB_PLLCFG1, 0xfu << 8, 0);
	clrset(eusb_regs, EUSB_TXTUNE, 3u << 1, 0);
	clrset(eusb_regs, EUSB_TESTSE, 1u << 6, 0);
	clrset(eusb_regs, EUSB_CMN_CTRL, 1u << 0, 0);
	delay(10);
	clrset(eusb_regs, EUSB_RST_CTRL, 1u << 0, 0);
	delay(10);
	clrset(eusb_regs, EUSB_CMN_CTRL, 0, 1u << 0);
	delay(3600);
	clrset(eusb_regs, EUSB_RST_CTRL, 1u << 4 | 1u << 5, 0);
	printf("akita usb: the eUSB2 link made again: RST_CTRL %#x CMN_CTRL %#x LINKCTRL %#x UTMI_CTRL %#x\n",
	    *(volatile uint32_t *)(eusb_regs + EUSB_RST_CTRL), *(volatile uint32_t *)(eusb_regs + EUSB_CMN_CTRL),
	    *(volatile uint32_t *)(usbcon_regs + USBCON_LINKCTRL), *(volatile uint32_t *)(usbcon_regs + USBCON_UTMI_CTRL));
}

void akita_usb_reconnect(void);
void
akita_usb_reconnect(void)
{
	restart_asked = 1;
}

static void
akita_usb_thread(__unused void *parameter, __unused wait_result_t result)
{
	usb_grp = lck_grp_alloc_init("akita usb", LCK_GRP_ATTR_NULL);
	txq_lock = lck_mtx_alloc_init(usb_grp, LCK_ATTR_NULL);
	akita_usbmux_init();
	akita_ptp_init();
	if (!controller_start()) {
		start_result = 0;
		for (unsigned i = 0; i < 24; i++) {     /* on the screen (-v) for two minutes: why there is no USB */
			printf("akita usb: NOT STARTED (GSNPSID %#x DCTL %#x DSTS %#x GCTL %#x)\n", rd(GSNPSID), rd(DCTL),
			    rd(DSTS), rd(GCTL));
			delay_for_interval(5000, NSEC_PER_MSEC);
		}
		return;
	}
	start_result = 1;
	/*
	 * Polled: while transfers come and go, the controller is looked at again after 20 us (a 16 KB transfer takes about
	 * 300 us at high speed: the bus stays busy); after 20 ms with nothing, once a millisecond. A millisecond between
	 * every look held AFC to 6-8 MB/s, one transfer a millisecond.
	 */
	uint64_t now, last_busy = 0, next_status = 0, next_summary, busy_window, five_s, ten_min;
	unsigned statuses = 0;

	nanoseconds_to_absolutetime(20 * NSEC_PER_MSEC, &busy_window);
	nanoseconds_to_absolutetime(5 * NSEC_PER_SEC, &five_s);
	nanoseconds_to_absolutetime(600 * NSEC_PER_SEC, &ten_min);
	next_summary = mach_absolute_time() + ten_min;
	for (;;) {
		/* (boot-arg usb_reinit_at=SECONDS: the same once, with the cable in -- a test of the way back the host can see) */
		{
			static uint32_t reinit_s = ~0u;
			static uint64_t reinit_at;

			if (reinit_s == ~0u) {
				reinit_s = 0;
				PE_parse_boot_argn("usb_reinit_at", &reinit_s, sizeof(reinit_s));
				nanoseconds_to_absolutetime((uint64_t)reinit_s * NSEC_PER_SEC, &reinit_at);
			}
			if (reinit_s && mach_absolute_time() >= reinit_at) {
				reinit_s = 0;
				printf("akita usb: usb_reinit_at: the way back after a replug, with the cable in\n");
				restart_asked = 1;
			}
		}
		if (restart_asked) {
			restart_asked = 0;
			printf("akita usb: started again (the cable came back)\n");
			bulk_disable();
			wr(DCTL, rd(DCTL) & ~DCTL_RUN_STOP);            /* (off the bus while the link is made again) */
			eusb_reinit();
			(void)controller_start();
			statuses = 0;           /* (the status lines again, to see the new link come up) */
		}
		bool busy = poll_events();

		busy = tx_service() || busy;
		busy = ptp_service() || busy;
		now = mach_absolute_time();
		if (busy) {
			last_busy = now;
		}
		if (now >= next_status && statuses < 24) {      /* every 5 s for two minutes: on the screen with -v */
			statuses++;
			next_status = now + five_s;
			printf("akita usb: DCTL %#x DSTS %#x DCFG %#x: %u events, %u setups, %u bulk OUT, %s%s\n", rd(DCTL),
			    rd(DSTS), rd(DCFG), seen_events, seen_setups, seen_bulk_out, bulk_on ? "configured" :
			    "not configured", ptp_failed ? "; PTP off" : "");
		}
		if (now >= next_summary) {                      /* and every ten minutes after */
			next_summary = now + ten_min;
			printf("akita usb: %u setups, %u bulk OUT transfers (%u bytes)\n", seen_setups, seen_bulk_out,
			    bulk_out_bytes);
		}
		/* a trace of what a host does, for a host the USB link does not survive: usb_trace_panic=SECONDS after the start,
		 * a panic on purpose, and the whole log in pstore (the loader's way out of a panic) */
		{
			static uint32_t trace_s = ~0u;
			static uint64_t trace_at;

			if (trace_s == ~0u) {
				trace_s = 0;
				PE_parse_boot_argn("usb_trace_panic", &trace_s, sizeof(trace_s));
				nanoseconds_to_absolutetime((uint64_t)trace_s * NSEC_PER_SEC, &trace_at);
			}
			if (trace_s && now >= trace_at) {
				panic("akita usb: usb_trace_panic=%u: the trace's time is up (%u setups seen)", trace_s, seen_setups);
			}
		}
		if (now - last_busy < busy_window) {
			delay(20);
		} else {
			delay_for_interval(1, NSEC_PER_MSEC);
		}
	}
}

static void
akita_usb_start(__unused thread_call_param_t p0, __unused thread_call_param_t p1)
{
	thread_t thread;

	if (kernel_thread_start(akita_usb_thread, NULL, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	} else {
		printf("akita usb: no thread\n");
	}
}

__startup_func
static void
akita_usb_init(void)
{
	uint32_t enabled = 0;
	uint64_t deadline;
	vm_offset_t cached = 0;
	static thread_call_t start;

	if (!PE_parse_boot_argn("usbmux", &enabled, sizeof(enabled)) || !enabled) {
		return;
	}
	PE_parse_boot_argn("usb_serial", serial, sizeof(serial));
	regs = ml_io_map(DWC3_PHYS, DWC3_SIZE);
	if (kmem_alloc_contig(kernel_map, &cached, MEM_SIZE, MEM_SIZE - 1, 0, 0, KMA_KOBJECT | KMA_ZERO | KMA_PERMANENT,
	    VM_KERN_MEMORY_IOKIT) != KERN_SUCCESS) {
		printf("akita usb: no memory for the controller; off\n");
		return;
	}
	FlushPoC_DcacheRegion(cached, MEM_SIZE);
	mem_pa = kvtophys(cached);
	mem = ml_io_map_wcomb((vm_offset_t)mem_pa, MEM_SIZE);
	if (!regs || !mem) {
		printf("akita usb: could not map the controller or its memory; off\n");
		return;
	}
	bzero((void *)mem, MEM_SIZE);
	/* its own thread once the kernel is up, two seconds from now */
	start = thread_call_allocate(akita_usb_start, NULL);
	clock_interval_to_deadline(2, NSEC_PER_SEC, &deadline);
	thread_call_enter_delayed(start, deadline);
}
STARTUP(EARLY_BOOT, STARTUP_RANK_LAST, akita_usb_init);
