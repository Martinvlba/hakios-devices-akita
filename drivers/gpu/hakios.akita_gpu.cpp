/*
 * HakiosGPU on the Pixel 8a: zuma's Mali-G715 for programs to draw and compute with.
 *
 * hakios.akita_mali.c brings the GPU up and, with boot-arg mali_keep=1, keeps it on and starts this service. A program
 * that opens it speaks the interface of Linux's panthor DRM driver, the one Mesa's panfrost and panvk know: every
 * request is a call of the user client with the request's number as the selector (DRM's: 0x40 + panthor's own number,
 * 0x09 GEM_CLOSE, 0xbf.. the sync objects) and the request's structure passed in and back -- as redwood's HakiosGPU does
 * with msm's for freedreno (drivers/gpu/hakios.gpu.cpp; the userland side, tools/mesa/hakios-drm, stands in for libdrm).
 * A buffer's memory is the client memory of the type that is its handle (BO_MMAP_OFFSET answers the handle << 12).
 *
 * What there is: the device's description; VMs (each its own address space of the GPU, with its own tables); buffers
 * (zeroed, write-combined for the processors, mapped into a VM page by page); groups of queues fed by the kernel; the
 * submission of a program's command streams; DRM sync objects, binary and timeline, signalled by submissions and
 * waited for (a submission's wait operations are waited for here, before it goes to the GPU). Tiler heaps: their initial
 * chunks (growing one is not here yet). Not here yet: sync files, interrupts (waiting is asking again every millisecond), recovery of a group that faulted.
 *
 * The structures are those of Linux's include/uapi/drm/panthor_drm.h and drm.h (MIT, Collabora and others), as
 * Mesa 26.2.5 carries them, written out here.
 *
 * Timeouts of the waits are RELATIVE nanoseconds here (DRM's are absolute, CLOCK_MONOTONIC): the userland side
 * converts, since it shares the clock with the caller.
 */
#include <IOKit/IOLib.h>
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <libkern/c++/OSObject.h>
#include <pexpert/pexpert.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOPlatformExpert.h>

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

extern "C" {
bool akita_mali_ready(void);
int akita_mali_vm_new(void);
void akita_mali_vm_free(int vm);
bool akita_mali_vm_map(int vm, uint64_t va, uint64_t phys, uint32_t flags);
void akita_mali_vm_unmap(int vm, uint64_t va, uint64_t size);
void akita_mali_vm_flush(int vm);
int akita_mali_group_new(int vm, unsigned int nqueues, const uint32_t *ring_sizes, uint8_t priority);
void akita_mali_group_free(int g);
uint64_t akita_mali_submit(int g, unsigned int q, uint64_t stream, uint32_t size);
uint64_t akita_mali_reached(int g, unsigned int q);
void akita_mali_flush(void);
int akita_mali_heap_new(int vm, uint32_t chunk_size, uint32_t initial, uint32_t max_chunks, uint32_t in_flight,
    uint64_t *ctx_va, uint64_t *first_chunk_va);
void akita_mali_heap_free(int h);
void akita_mali_debug(void);
void akita_mali_info(uint32_t *regs, uint32_t *csif);
void akita_gpu_start(void);
}

/* ---- panthor_drm.h and drm.h, as much as is answered here ---- */

struct drm_panthor_obj_array { uint32_t stride, count; uint64_t array; };
struct drm_panthor_sync_op { uint32_t flags, handle; uint64_t timeline_value; };
#define SYNC_OP_TIMELINE        1u              /* DRM_PANTHOR_SYNC_OP_HANDLE_TYPE_TIMELINE_SYNCOBJ */
#define SYNC_OP_SIGNAL          (1u << 31)
struct drm_panthor_dev_query { uint32_t type, size; uint64_t pointer; };
struct drm_panthor_gpu_info {
	uint32_t gpu_id, gpu_rev, csf_id, l2_features, tiler_features, mem_features, mmu_features, thread_features,
	    max_threads, thread_max_workgroup_size, thread_max_barrier_size, coherency_features, texture_features[4],
	    as_present, selected_coherency;
	uint64_t shader_present, l2_present, tiler_present;
	uint32_t core_features, pad;
	uint64_t gpu_features;
};
struct drm_panthor_csif_info {
	uint32_t csg_slot_count, cs_slot_count, cs_reg_count, scoreboard_slot_count, unpreserved_cs_reg_count, pad;
};
struct drm_panthor_timestamp_info {
	uint64_t timestamp_frequency, current_timestamp, timestamp_offset;
	uint32_t flags, duration_nsec;
	uint64_t cycle_count, cpu_timestamp_sec, cpu_timestamp_nsec;
};
struct drm_panthor_group_priorities_info { uint8_t allowed_mask, pad[3]; };
struct drm_panthor_vm_create { uint32_t flags, id; uint64_t user_va_range; };
struct drm_panthor_vm_destroy { uint32_t id, pad; };
struct drm_panthor_vm_bind_op {
	uint32_t flags, bo_handle;
	uint64_t bo_offset, va, size;
	struct drm_panthor_obj_array syncs;
};
#define BIND_OP_TYPE_MASK       (0xfu << 28)
#define BIND_OP_TYPE_MAP        (0u << 28)
#define BIND_OP_TYPE_UNMAP      (1u << 28)
struct drm_panthor_vm_bind { uint32_t vm_id, flags; struct drm_panthor_obj_array ops; };
struct drm_panthor_vm_get_state { uint32_t vm_id, state; };
struct drm_panthor_bo_create { uint64_t size; uint32_t flags, exclusive_vm_id, handle, pad; };
struct drm_panthor_bo_mmap_offset { uint32_t handle, pad; uint64_t offset; };
struct drm_panthor_queue_create { uint8_t priority, pad[3]; uint32_t ringbuf_size; };
struct drm_panthor_group_create {
	struct drm_panthor_obj_array queues;
	uint8_t max_compute_cores, max_fragment_cores, max_tiler_cores, priority;
	uint32_t pad;
	uint64_t compute_core_mask, fragment_core_mask, tiler_core_mask;
	uint32_t vm_id, group_handle;
};
struct drm_panthor_group_destroy { uint32_t group_handle, pad; };
struct drm_panthor_queue_submit {
	uint32_t queue_index, stream_size;
	uint64_t stream_addr;
	uint32_t latest_flush, pad;
	struct drm_panthor_obj_array syncs;
};
struct drm_panthor_group_submit { uint32_t group_handle, pad; struct drm_panthor_obj_array queue_submits; };
struct drm_panthor_group_get_state { uint32_t group_handle, state, fatal_queues, pad; };
struct drm_panthor_tiler_heap_create { uint32_t vm_id, initial_chunk_count, chunk_size, max_chunks, target_in_flight, handle;
	uint64_t tiler_heap_ctx_gpu_va, first_heap_chunk_gpu_va; };
struct drm_panthor_tiler_heap_destroy { uint32_t handle, pad; };
struct drm_gem_close { uint32_t handle, pad; };
struct drm_syncobj_create { uint32_t handle, flags; };
#define SYNCOBJ_CREATE_SIGNALED 1u
struct drm_syncobj_destroy { uint32_t handle, pad; };
struct drm_syncobj_wait {
	uint64_t handles; int64_t timeout_nsec; uint32_t count_handles, flags, first_signaled, pad; uint64_t deadline_nsec;
};
struct drm_syncobj_timeline_wait {
	uint64_t handles, points; int64_t timeout_nsec; uint32_t count_handles, flags, first_signaled, pad;
	uint64_t deadline_nsec;
};
#define WAIT_ALL                1u
#define WAIT_FOR_SUBMIT         2u
#define WAIT_AVAILABLE          4u
struct drm_syncobj_array { uint64_t handles; uint32_t count_handles, pad; };
struct drm_syncobj_timeline_array { uint64_t handles, points; uint32_t count_handles, flags; };
struct drm_syncobj_transfer { uint32_t src_handle, dst_handle; uint64_t src_point, dst_point; uint32_t flags, pad; };

enum {
	kGemClose = 0x09,
	kDevQuery = 0x40, kVmCreate, kVmDestroy, kVmBind, kVmGetState, kBoCreate, kBoMmapOffset, kGroupCreate,
	kGroupDestroy, kGroupSubmit, kGroupGetState, kTilerHeapCreate, kTilerHeapDestroy, kBoSetLabel, kSetUserMmioOffset,
	kBoSync, kBoQueryInfo,
	kSyncobjCreate = 0xbf, kSyncobjDestroy = 0xc0, kSyncobjWait = 0xc3, kSyncobjReset = 0xc4, kSyncobjSignal = 0xc5,
	kSyncobjTimelineWait = 0xca, kSyncobjQuery = 0xcb, kSyncobjTransfer = 0xcc, kSyncobjTimelineSignal = 0xcd,
	kHakiosDebug = 0x7f,            /* not Linux's: the queues' state into the kernel's log */
};

/* ---- the service ---- */

static IOLock *gLock;

class HakiosGPU : public IOService
{
	OSDeclareDefaultStructors(HakiosGPU);
public:
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(HakiosGPU, IOService);

bool
HakiosGPU::start(IOService *provider)
{
	if (!IOService::start(provider)) {
		return false;
	}
	setProperty(kIOUserClientClassKey, "HakiosGPUUserClient");
	setProperty("DRMDriver", "panthor");
	setName("HakiosGPU");
	registerService();
	return true;
}

void
akita_gpu_start(void)
{
	IOService *provider = (IOService *)IOService::getPlatform();
	HakiosGPU *gpu;

	gLock = IOLockAlloc();
	gpu = OSTypeAlloc(HakiosGPU);
	if (gpu == NULL || provider == NULL || !gpu->init() || !gpu->attach(provider)) {
		OSSafeReleaseNULL(gpu);
		IOLog("HakiosGPU: cannot create the service\n");
		return;
	}
	if (!gpu->start(provider)) {
		gpu->detach(provider);
		IOLog("HakiosGPU: the service did not start\n");
	} else {
		IOLog("HakiosGPU: the Mali-G715, with panthor's interface\n");
	}
	gpu->release();
}

/* ---- a client ---- */

#define MAX_BOS                 1024
#define MAX_SYNCS               1024
#define MAX_POINTS              32
#define MAX_CLIENT_VMS          4
#define MAX_CLIENT_GROUPS       8
#define MAX_CLIENT_HEAPS        8

/* what a sync object waits for: nothing yet, done, or a queue reaching a number */
struct fence {
	int16_t group;                  /* -1: done already (signalled) */
	uint8_t queue;
	uint64_t seqno;
};

struct syncobj {
	bool used, binary_set;
	struct fence binary;            /* point 0, a binary one's */
	uint32_t points;                /* a timeline's: (point, fence), oldest first */
	uint64_t point[MAX_POINTS];
	struct fence pf[MAX_POINTS];
};

class HakiosGPUUserClient : public IOUserClient
{
	OSDeclareDefaultStructors(HakiosGPUUserClient);

	struct bo { IOBufferMemoryDescriptor *memory; uint64_t size; } *fBos;
	struct syncobj *fSyncs;
	int fVms[MAX_CLIENT_VMS];
	int fGroups[MAX_CLIENT_GROUPS];
	int fHeaps[MAX_CLIENT_HEAPS];              /* the kernel's heap slots (handle: index + 1) */

	bool done(const struct fence *f);
	bool syncPointDone(struct syncobj *s, uint64_t point, bool *submitted);
	void syncSignal(uint32_t handle, uint64_t point, const struct fence *f);
	bool waitPoints(const uint32_t *handles, const uint64_t *points, uint32_t count, uint32_t flags, int64_t timeout,
	    uint32_t *first);
	IOReturn devQuery(struct drm_panthor_dev_query *q);
	IOReturn vmBind(struct drm_panthor_vm_bind *b);
	IOReturn groupCreate(struct drm_panthor_group_create *c);
	IOReturn groupSubmit(struct drm_panthor_group_submit *s);
	bool ownsVm(uint32_t id);
	int group(uint32_t handle);

public:
	virtual bool initWithTask(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;
	virtual IOReturn clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory) APPLE_KEXT_OVERRIDE;
	virtual IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args, IOExternalMethodDispatch *dispatch,
	    OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(HakiosGPUUserClient, IOUserClient);

bool
HakiosGPUUserClient::initWithTask(task_t owningTask, void *securityID, UInt32 type, OSDictionary *properties)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type, properties)) {
		return false;
	}
	fBos = (struct bo *)IOMallocZero(sizeof(*fBos) * MAX_BOS);
	fSyncs = (struct syncobj *)IOMallocZero(sizeof(*fSyncs) * MAX_SYNCS);
	for (int i = 0; i < MAX_CLIENT_VMS; i++) {
		fVms[i] = -1;
	}
	for (int i = 0; i < MAX_CLIENT_GROUPS; i++) {
		fGroups[i] = -1;
	}
	for (int i = 0; i < MAX_CLIENT_HEAPS; i++) {
		fHeaps[i] = -1;
	}
	return fBos != NULL && fSyncs != NULL;
}

IOReturn
HakiosGPUUserClient::clientClose(void)
{
	IOLockLock(gLock);
	for (int i = 0; i < MAX_CLIENT_GROUPS; i++) {
		if (fGroups[i] >= 0) {
			akita_mali_group_free(fGroups[i]);
			fGroups[i] = -1;
		}
	}
	for (int i = 0; i < MAX_CLIENT_HEAPS; i++) {
		if (fHeaps[i] >= 0) {
			akita_mali_heap_free(fHeaps[i]);
			fHeaps[i] = -1;
		}
	}
	for (int i = 0; i < MAX_CLIENT_VMS; i++) {
		if (fVms[i] >= 0) {
			akita_mali_vm_free(fVms[i]);
			fVms[i] = -1;
		}
	}
	IOLockUnlock(gLock);
	terminate();
	return kIOReturnSuccess;
}

void
HakiosGPUUserClient::free(void)
{
	if (fBos != NULL) {
		for (int i = 0; i < MAX_BOS; i++) {
			OSSafeReleaseNULL(fBos[i].memory);
		}
		IOFree(fBos, sizeof(*fBos) * MAX_BOS);
	}
	if (fSyncs != NULL) {
		IOFree(fSyncs, sizeof(*fSyncs) * MAX_SYNCS);
	}
	IOUserClient::free();
}

IOReturn
HakiosGPUUserClient::clientMemoryForType(UInt32 type, IOOptionBits *options, IOMemoryDescriptor **memory)
{
	IOReturn ret = kIOReturnBadArgument;

	IOLockLock(gLock);
	if (type >= 1 && type <= MAX_BOS && fBos[type - 1].memory != NULL) {
		fBos[type - 1].memory->retain();
		*memory = fBos[type - 1].memory;
		*options = 0;
		ret = kIOReturnSuccess;
	}
	IOLockUnlock(gLock);
	return ret;
}

bool
HakiosGPUUserClient::ownsVm(uint32_t id)
{
	for (int i = 0; i < MAX_CLIENT_VMS; i++) {
		if (fVms[i] >= 0 && (uint32_t)fVms[i] == id) {
			return true;
		}
	}
	return false;
}

int
HakiosGPUUserClient::group(uint32_t handle)
{
	for (int i = 0; i < MAX_CLIENT_GROUPS; i++) {
		if (fGroups[i] >= 0 && (uint32_t)fGroups[i] + 1 == handle) {
			return fGroups[i];
		}
	}
	return -1;
}

bool
HakiosGPUUserClient::done(const struct fence *f)
{
	return f->group < 0 || akita_mali_reached(f->group, f->queue) >= f->seqno;
}

/* is the point reached; *submitted: is there a fence for it at all */
bool
HakiosGPUUserClient::syncPointDone(struct syncobj *s, uint64_t point, bool *submitted)
{
	*submitted = false;
	if (point == 0) {
		*submitted = s->binary_set;
		return s->binary_set && done(&s->binary);
	}
	for (uint32_t i = 0; i < s->points; i++) {
		if (s->point[i] >= point) {
			*submitted = true;
			return done(&s->pf[i]);
		}
	}
	return false;
}

void
HakiosGPUUserClient::syncSignal(uint32_t handle, uint64_t point, const struct fence *f)
{
	struct syncobj *s;

	if (handle == 0 || handle > MAX_SYNCS || !fSyncs[handle - 1].used) {
		return;
	}
	s = &fSyncs[handle - 1];
	if (point == 0) {
		s->binary = *f;
		s->binary_set = true;
		return;
	}
	/* the oldest points that are done make way */
	while (s->points == MAX_POINTS || (s->points > 1 && done(&s->pf[0]) && done(&s->pf[1]))) {
		memmove(&s->point[0], &s->point[1], (s->points - 1) * sizeof(s->point[0]));
		memmove(&s->pf[0], &s->pf[1], (s->points - 1) * sizeof(s->pf[0]));
		s->points--;
	}
	s->point[s->points] = point;
	s->pf[s->points] = *f;
	s->points++;
}

/* waits, the lock dropped while it sleeps; true if they are done in time */
bool
HakiosGPUUserClient::waitPoints(const uint32_t *handles, const uint64_t *points, uint32_t count, uint32_t flags,
    int64_t timeout, uint32_t *first)
{
	uint64_t deadline;

	clock_interval_to_deadline((uint32_t)MIN(timeout > 0 ? timeout / 1000 : 0, 10 * 1000 * 1000), kMicrosecondScale,
	    &deadline);
	for (;;) {
		uint32_t reached = 0;
		bool all = true;

		for (uint32_t i = 0; i < count; i++) {
			bool submitted;
			struct syncobj *s = handles[i] && handles[i] <= MAX_SYNCS ? &fSyncs[handles[i] - 1] : NULL;

			if (s != NULL && s->used && syncPointDone(s, points ? points[i] : 0, &submitted)) {
				if (reached++ == 0 && first != NULL) {
					*first = i;
				}
			} else {
				all = false;
			}
		}
		if ((flags & WAIT_ALL) ? all : reached > 0) {
			akita_mali_flush();
			return true;
		}
		if (mach_absolute_time() >= deadline) {
			if (timeout > 0) {
				akita_mali_debug();     /* where the queues stand, for the log */
			}
			return false;
		}
		IOLockUnlock(gLock);
		IOSleep(1);
		IOLockLock(gLock);
	}
}

IOReturn
HakiosGPUUserClient::devQuery(struct drm_panthor_dev_query *q)
{
	union {
		struct drm_panthor_gpu_info gpu;
		struct drm_panthor_csif_info csif;
		struct drm_panthor_timestamp_info ts;
		struct drm_panthor_group_priorities_info prio;
	} u;
	static uint32_t regs[256], csif[5];
	uint32_t size;

	bzero(&u, sizeof(u));
	akita_mali_info(regs, csif);
	switch (q->type) {
	case 0:
		size = sizeof(u.gpu);
		u.gpu.gpu_id = regs[0];
		u.gpu.csf_id = regs[0x1c / 4];
		u.gpu.l2_features = regs[0x04 / 4];
		u.gpu.tiler_features = regs[0x0c / 4];
		u.gpu.mem_features = regs[0x10 / 4];
		u.gpu.mmu_features = regs[0x14 / 4];
		u.gpu.thread_features = regs[0xac / 4];
		u.gpu.max_threads = regs[0xa0 / 4];
		u.gpu.thread_max_workgroup_size = regs[0xa4 / 4];
		u.gpu.thread_max_barrier_size = regs[0xa8 / 4];
		u.gpu.coherency_features = regs[0x300 / 4];
		for (int i = 0; i < 4; i++) {
			u.gpu.texture_features[i] = regs[0xb0 / 4 + i];
		}
		u.gpu.as_present = regs[0x18 / 4];
		u.gpu.selected_coherency = 31;          /* none: the GPU does not snoop the processors' caches */
		u.gpu.shader_present = regs[0x100 / 4] | (uint64_t)regs[0x104 / 4] << 32;
		u.gpu.tiler_present = regs[0x110 / 4] | (uint64_t)regs[0x114 / 4] << 32;
		u.gpu.l2_present = regs[0x120 / 4] | (uint64_t)regs[0x124 / 4] << 32;
		u.gpu.core_features = regs[0x08 / 4];
		u.gpu.gpu_features = regs[0x60 / 4] | (uint64_t)regs[0x64 / 4] << 32;
		break;
	case 1:
		size = sizeof(u.csif);
		u.csif.csg_slot_count = csif[0];
		u.csif.cs_slot_count = csif[1];
		u.csif.cs_reg_count = csif[2];
		u.csif.scoreboard_slot_count = csif[3];
		u.csif.unpreserved_cs_reg_count = csif[4];
		break;
	case 2:
		size = sizeof(u.ts);
		u.ts.timestamp_frequency = 24576000;    /* the system counter's, which the GPU's timestamp counts */
		u.ts.current_timestamp = regs[0x98 / 4] | (uint64_t)regs[0x9c / 4] << 32;
		break;
	case 3:
		size = sizeof(u.prio);
		u.prio.allowed_mask = 0xf;
		break;
	default:
		return kIOReturnBadArgument;
	}
	if (q->pointer == 0) {
		q->size = size;
		return kIOReturnSuccess;
	}
	if (copyout(&u, (user_addr_t)q->pointer, MIN(q->size, size)) != 0) {
		return kIOReturnBadArgument;
	}
	return kIOReturnSuccess;
}

IOReturn
HakiosGPUUserClient::vmBind(struct drm_panthor_vm_bind *b)
{
	struct drm_panthor_vm_bind_op op;

	if (!ownsVm(b->vm_id) || b->ops.stride < sizeof(op) - sizeof(op.syncs)) {
		return kIOReturnBadArgument;
	}
	for (uint32_t i = 0; i < b->ops.count; i++) {
		bzero(&op, sizeof(op));
		if (copyin((user_addr_t)(b->ops.array + (uint64_t)i * b->ops.stride), &op, MIN(b->ops.stride, sizeof(op))) != 0) {
			return kIOReturnBadArgument;
		}
		if ((op.va | op.size) & 0xfff) {
			return kIOReturnBadArgument;
		}
		switch (op.flags & BIND_OP_TYPE_MASK) {
		case BIND_OP_TYPE_MAP: {
			struct bo *bo = op.bo_handle && op.bo_handle <= MAX_BOS ? &fBos[op.bo_handle - 1] : NULL;

			if (bo == NULL || bo->memory == NULL || op.bo_offset + op.size > bo->size || op.bo_offset & 0xfff) {
				return kIOReturnBadArgument;
			}
			for (uint64_t off = 0; off < op.size; off += 0x1000) {
				IOByteCount len;
				addr64_t phys = bo->memory->getPhysicalSegment(op.bo_offset + off, &len, kIOMemoryMapperNone);

				if (off == 0) {
					IOLog("HakiosGPU: bind of buffer %u at 0x%llx: its first page at 0x%llx\n", op.bo_handle, op.va,
					    (unsigned long long)phys);
				}
				if (phys == 0 || !akita_mali_vm_map((int)b->vm_id, op.va + off, phys, op.flags & 7)) {
					return kIOReturnNoMemory;
				}
			}
			break;
		}
		case BIND_OP_TYPE_UNMAP:
			akita_mali_vm_unmap((int)b->vm_id, op.va, op.size);
			break;
		default:
			break;                          /* SYNC_ONLY: nothing to do here, the binds are done at once */
		}
		/* binds happen at once: their signal operations are done ones */
		for (uint32_t k = 0; k < op.syncs.count; k++) {
			struct drm_panthor_sync_op so;
			struct fence f = { -1, 0, 0 };

			if (copyin((user_addr_t)(op.syncs.array + (uint64_t)k * op.syncs.stride), &so, sizeof(so)) == 0 &&
			    (so.flags & SYNC_OP_SIGNAL)) {
				syncSignal(so.handle, (so.flags & 0xff) == SYNC_OP_TIMELINE ? so.timeline_value : 0, &f);
			}
		}
	}
	akita_mali_vm_flush((int)b->vm_id);
	return kIOReturnSuccess;
}

IOReturn
HakiosGPUUserClient::groupCreate(struct drm_panthor_group_create *c)
{
	uint32_t sizes[4];
	struct drm_panthor_queue_create qc;
	int slot = -1, g;

	if (!ownsVm(c->vm_id) || c->queues.count == 0 || c->queues.count > 4) {
		return kIOReturnBadArgument;
	}
	for (uint32_t i = 0; i < c->queues.count; i++) {
		bzero(&qc, sizeof(qc));
		if (copyin((user_addr_t)(c->queues.array + (uint64_t)i * c->queues.stride), &qc, MIN(c->queues.stride, sizeof(qc))) != 0) {
			return kIOReturnBadArgument;
		}
		sizes[i] = qc.ringbuf_size;
	}
	for (int i = 0; i < MAX_CLIENT_GROUPS; i++) {
		if (fGroups[i] < 0) {
			slot = i;
			break;
		}
	}
	if (slot < 0) {
		return kIOReturnNoResources;
	}
	g = akita_mali_group_new((int)c->vm_id, c->queues.count, sizes, (uint8_t)(c->priority * 4 + 2));
	if (g < 0) {
		return kIOReturnNoResources;
	}
	fGroups[slot] = g;
	c->group_handle = (uint32_t)g + 1;
	return kIOReturnSuccess;
}

IOReturn
HakiosGPUUserClient::groupSubmit(struct drm_panthor_group_submit *s)
{
	int g = group(s->group_handle);
	struct drm_panthor_queue_submit qs;
	struct drm_panthor_sync_op so;

	if (g < 0) {
		return kIOReturnBadArgument;
	}
	for (uint32_t i = 0; i < s->queue_submits.count; i++) {
		uint64_t seqno;
		struct fence f;

		bzero(&qs, sizeof(qs));
		if (copyin((user_addr_t)(s->queue_submits.array + (uint64_t)i * s->queue_submits.stride), &qs,
		    MIN(s->queue_submits.stride, sizeof(qs))) != 0) {
			return kIOReturnBadArgument;
		}
		/* its wait operations: waited for here, before the stream goes to the GPU */
		for (uint32_t k = 0; k < qs.syncs.count; k++) {
			if (copyin((user_addr_t)(qs.syncs.array + (uint64_t)k * qs.syncs.stride), &so, sizeof(so)) != 0) {
				return kIOReturnBadArgument;
			}
			if (!(so.flags & SYNC_OP_SIGNAL)) {
				uint64_t point = (so.flags & 0xff) == SYNC_OP_TIMELINE ? so.timeline_value : 0;

				if (!waitPoints(&so.handle, &point, 1, WAIT_ALL | WAIT_FOR_SUBMIT, 10000000000LL, NULL)) {
					return kIOReturnTimeout;
				}
			}
		}
		if (qs.stream_size != 0) {
			seqno = akita_mali_submit(g, qs.queue_index, qs.stream_addr, qs.stream_size);
			if (seqno == 0) {
				return kIOReturnNoSpace;
			}
		} else {
			seqno = akita_mali_reached(g, qs.queue_index);  /* only syncs: what the queue has reached */
		}
		f.group = (int16_t)g;
		f.queue = (uint8_t)qs.queue_index;
		f.seqno = seqno;
		for (uint32_t k = 0; k < qs.syncs.count; k++) {
			if (copyin((user_addr_t)(qs.syncs.array + (uint64_t)k * qs.syncs.stride), &so, sizeof(so)) == 0 &&
			    (so.flags & SYNC_OP_SIGNAL)) {
				syncSignal(so.handle, (so.flags & 0xff) == SYNC_OP_TIMELINE ? so.timeline_value : 0, &f);
			}
		}
	}
	return kIOReturnSuccess;
}

IOReturn
HakiosGPUUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args, IOExternalMethodDispatch *dispatch,
    OSObject *target, void *reference)
{
	union {
		uint8_t raw[128];
		struct drm_panthor_dev_query devQuery;
		struct drm_panthor_vm_create vmCreate;
		struct drm_panthor_vm_destroy vmDestroy;
		struct drm_panthor_vm_bind vmBind;
		struct drm_panthor_vm_get_state vmState;
		struct drm_panthor_bo_create boCreate;
		struct drm_panthor_bo_mmap_offset boMmap;
		struct drm_panthor_group_create groupCreate;
		struct drm_panthor_group_destroy groupDestroy;
		struct drm_panthor_group_submit groupSubmit;
		struct drm_panthor_group_get_state groupState;
		struct drm_panthor_tiler_heap_create heapCreate;
		struct drm_panthor_tiler_heap_destroy heapDestroy;
		struct drm_gem_close gemClose;
		struct drm_syncobj_create syncCreate;
		struct drm_syncobj_destroy syncDestroy;
		struct drm_syncobj_wait syncWait;
		struct drm_syncobj_timeline_wait syncTlWait;
		struct drm_syncobj_array syncArray;
		struct drm_syncobj_timeline_array syncTlArray;
		struct drm_syncobj_transfer syncTransfer;
	} r;
	size_t size = args->structureInputSize;
	IOReturn ret = kIOReturnSuccess;

	if (args->structureInput == NULL || size > sizeof(r)) {
		return kIOReturnBadArgument;
	}
	bzero(&r, sizeof(r));
	memcpy(&r, args->structureInput, size);
	IOLockLock(gLock);
	if (!akita_mali_ready()) {
		IOLockUnlock(gLock);
		return kIOReturnNotReady;
	}
	switch (selector) {
	case kDevQuery:
		ret = devQuery(&r.devQuery);
		break;
	case kVmCreate: {
		int vm = -1, slot = -1;

		for (int i = 0; i < MAX_CLIENT_VMS; i++) {
			if (fVms[i] < 0) {
				slot = i;
				break;
			}
		}
		if (slot >= 0) {
			vm = akita_mali_vm_new();
		}
		if (vm < 0) {
			ret = kIOReturnNoResources;
		} else {
			fVms[slot] = vm;
			r.vmCreate.id = (uint32_t)vm;
		}
		break;
	}
	case kVmDestroy:
		for (int i = 0; i < MAX_CLIENT_VMS; i++) {
			if (fVms[i] >= 0 && (uint32_t)fVms[i] == r.vmDestroy.id) {
				akita_mali_vm_free(fVms[i]);
				fVms[i] = -1;
			}
		}
		break;
	case kVmBind:
		ret = vmBind(&r.vmBind);
		break;
	case kVmGetState:
		r.vmState.state = 0;                    /* usable */
		break;
	case kBoCreate: {
		uint64_t bytes = round_page_64(r.boCreate.size);
		IOBufferMemoryDescriptor *memory;
		int h;

		for (h = 0; h < MAX_BOS && fBos[h].memory != NULL; h++) {
		}
		if (bytes == 0 || bytes > (512ULL << 20) || h == MAX_BOS) {
			ret = kIOReturnBadArgument;
			break;
		}
		IOLockUnlock(gLock);
		memory = IOBufferMemoryDescriptor::inTaskWithOptions(kernel_task, kIODirectionInOut | kIOMemoryKernelUserShared |
		    kIOMapWriteCombineCache, (vm_size_t)bytes, PAGE_SIZE);
		IOLockLock(gLock);
		if (memory == NULL || fBos[h].memory != NULL) {
			OSSafeReleaseNULL(memory);
			ret = kIOReturnNoMemory;
			break;
		}
		bzero(memory->getBytesNoCopy(), (size_t)bytes);
		fBos[h].memory = memory;
		fBos[h].size = bytes;
		r.boCreate.size = bytes;
		r.boCreate.handle = (uint32_t)h + 1;
		break;
	}
	case kBoMmapOffset:
		if (r.boMmap.handle == 0 || r.boMmap.handle > MAX_BOS || fBos[r.boMmap.handle - 1].memory == NULL) {
			ret = kIOReturnBadArgument;
		} else {
			r.boMmap.offset = (uint64_t)r.boMmap.handle << 12;
		}
		break;
	case kGemClose:
		if (r.gemClose.handle >= 1 && r.gemClose.handle <= MAX_BOS) {
			/* (unmapped from VMs by the program's own UNMAP binds first, as panthor's users do) */
			OSSafeReleaseNULL(fBos[r.gemClose.handle - 1].memory);
		}
		break;
	case kGroupCreate:
		ret = groupCreate(&r.groupCreate);
		break;
	case kGroupDestroy:
		for (int i = 0; i < MAX_CLIENT_GROUPS; i++) {
			if (fGroups[i] >= 0 && (uint32_t)fGroups[i] + 1 == r.groupDestroy.group_handle) {
				akita_mali_group_free(fGroups[i]);
				fGroups[i] = -1;
			}
		}
		break;
	case kGroupSubmit:
		ret = groupSubmit(&r.groupSubmit);
		break;
	case kGroupGetState:
		r.groupState.state = 0;
		r.groupState.fatal_queues = 0;
		break;
	case kTilerHeapCreate: {
		int slot = -1, h = -1;

		for (int i = 0; i < MAX_CLIENT_HEAPS; i++) {
			if (fHeaps[i] < 0) {
				slot = i;
				break;
			}
		}
		if (slot < 0 || !ownsVm(r.heapCreate.vm_id)) {
			ret = slot < 0 ? kIOReturnNoResources : kIOReturnBadArgument;
			break;
		}
		h = akita_mali_heap_new((int)r.heapCreate.vm_id, r.heapCreate.chunk_size, r.heapCreate.initial_chunk_count,
		    r.heapCreate.max_chunks, r.heapCreate.target_in_flight, &r.heapCreate.tiler_heap_ctx_gpu_va,
		    &r.heapCreate.first_heap_chunk_gpu_va);
		if (h < 0) {
			ret = kIOReturnBadArgument;
			break;
		}
		fHeaps[slot] = h;
		r.heapCreate.handle = (uint32_t)slot + 1;
		break;
	}
	case kTilerHeapDestroy:
		if (r.heapDestroy.handle >= 1 && r.heapDestroy.handle <= MAX_CLIENT_HEAPS && fHeaps[r.heapDestroy.handle - 1] >= 0) {
			akita_mali_heap_free(fHeaps[r.heapDestroy.handle - 1]);
			fHeaps[r.heapDestroy.handle - 1] = -1;
		} else {
			ret = kIOReturnBadArgument;
		}
		break;
	case kBoSetLabel:
	case kSetUserMmioOffset:
	case kBoSync:
	case kBoQueryInfo:
		break;                                  /* nothing to do here */
	case kSyncobjCreate: {
		int h;

		for (h = 0; h < MAX_SYNCS && fSyncs[h].used; h++) {
		}
		if (h == MAX_SYNCS) {
			ret = kIOReturnNoResources;
			break;
		}
		bzero(&fSyncs[h], sizeof(fSyncs[h]));
		fSyncs[h].used = true;
		if (r.syncCreate.flags & SYNCOBJ_CREATE_SIGNALED) {
			fSyncs[h].binary.group = -1;
			fSyncs[h].binary_set = true;
		}
		r.syncCreate.handle = (uint32_t)h + 1;
		break;
	}
	case kSyncobjDestroy:
		if (r.syncDestroy.handle >= 1 && r.syncDestroy.handle <= MAX_SYNCS) {
			fSyncs[r.syncDestroy.handle - 1].used = false;
		}
		break;
	case kSyncobjWait:
	case kSyncobjTimelineWait: {
		bool timeline = selector == kSyncobjTimelineWait;
		uint32_t count = timeline ? r.syncTlWait.count_handles : r.syncWait.count_handles;
		uint32_t handles[64], first = 0;
		uint64_t points[64];

		if (count == 0 || count > 64 ||
		    copyin((user_addr_t)(timeline ? r.syncTlWait.handles : r.syncWait.handles), handles, count * 4) != 0 ||
		    (timeline && copyin((user_addr_t)r.syncTlWait.points, points, count * 8) != 0)) {
			ret = kIOReturnBadArgument;
			break;
		}
		if (!waitPoints(handles, timeline ? points : NULL, count, timeline ? r.syncTlWait.flags : r.syncWait.flags,
		    timeline ? r.syncTlWait.timeout_nsec : r.syncWait.timeout_nsec, &first)) {
			ret = kIOReturnTimeout;
		}
		if (timeline) {
			r.syncTlWait.first_signaled = first;
		} else {
			r.syncWait.first_signaled = first;
		}
		break;
	}
	case kSyncobjReset:
	case kSyncobjSignal: {
		uint32_t handles[64];
		uint32_t count = r.syncArray.count_handles;
		struct fence f = { -1, 0, 0 };

		if (count > 64 || copyin((user_addr_t)r.syncArray.handles, handles, count * 4) != 0) {
			ret = kIOReturnBadArgument;
			break;
		}
		for (uint32_t i = 0; i < count; i++) {
			if (handles[i] >= 1 && handles[i] <= MAX_SYNCS && fSyncs[handles[i] - 1].used) {
				if (selector == kSyncobjReset) {
					fSyncs[handles[i] - 1].binary_set = false;
				} else {
					syncSignal(handles[i], 0, &f);
				}
			}
		}
		break;
	}
	case kSyncobjQuery:
	case kSyncobjTimelineSignal: {
		uint32_t handles[64];
		uint64_t points[64];
		uint32_t count = r.syncTlArray.count_handles;
		struct fence f = { -1, 0, 0 };

		if (count > 64 || copyin((user_addr_t)r.syncTlArray.handles, handles, count * 4) != 0 ||
		    copyin((user_addr_t)r.syncTlArray.points, points, count * 8) != 0) {
			ret = kIOReturnBadArgument;
			break;
		}
		for (uint32_t i = 0; i < count; i++) {
			struct syncobj *s = handles[i] >= 1 && handles[i] <= MAX_SYNCS ? &fSyncs[handles[i] - 1] : NULL;

			if (s == NULL || !s->used) {
				continue;
			}
			if (selector == kSyncobjTimelineSignal) {
				syncSignal(handles[i], points[i], &f);
			} else {
				uint64_t value = 0;

				for (uint32_t k = 0; k < s->points; k++) {
					if ((r.syncTlArray.flags & 1) || done(&s->pf[k])) {
						value = MAX(value, s->point[k]);
					}
				}
				points[i] = value;
			}
		}
		if (selector == kSyncobjQuery && copyout(points, (user_addr_t)r.syncTlArray.points, count * 8) != 0) {
			ret = kIOReturnBadArgument;
		}
		break;
	}
	case kSyncobjTransfer: {
		struct syncobj *src = r.syncTransfer.src_handle >= 1 && r.syncTransfer.src_handle <= MAX_SYNCS ?
		    &fSyncs[r.syncTransfer.src_handle - 1] : NULL;
		struct fence f = { -1, 0, 0 };

		if (src == NULL || !src->used) {
			ret = kIOReturnBadArgument;
			break;
		}
		if (r.syncTransfer.src_point == 0) {
			if (src->binary_set) {
				f = src->binary;
			}
		} else {
			for (uint32_t k = 0; k < src->points; k++) {
				if (src->point[k] >= r.syncTransfer.src_point) {
					f = src->pf[k];
					break;
				}
			}
		}
		syncSignal(r.syncTransfer.dst_handle, r.syncTransfer.dst_point, &f);
		break;
	}
	case kHakiosDebug:
		akita_mali_debug();
		break;
	default:
		IOLog("HakiosGPU: request %#x (%zu bytes) is not answered here yet\n", selector, size);
		ret = kIOReturnUnsupported;
		break;
	}
	IOLockUnlock(gLock);
	if (ret == kIOReturnSuccess && args->structureOutput != NULL) {
		memcpy(args->structureOutput, &r, MIN(args->structureOutputSize, sizeof(r)));
	}
	return ret;
}
