#ifndef _PHIDGETOBJECT_H_
#define _PHIDGETOBJECT_H_

typedef struct _Phidget Phidget;
typedef struct _Phidget *PhidgetHandle;

#ifndef EXTERNALPROTO

#include "mos/mos_tlock.h"
#include "mos/bsdqueue.h"

typedef struct _PhidgetChannel *PhidgetChannelHandle;
typedef struct _PhidgetDevice *PhidgetDeviceHandle;

typedef enum {
	PHIDGET_BASE = 0xB00D3EE6,
	PHIDGET_CHANNEL,
	PHIDGET_DEVICE,
	PHIDGET_MANAGER,
	PHIDGET_HIDUSB_CONNECTION,
	PHIDGET_SPI_CONNECTION,
	PHIDGET_LIGHTNING_CONNECTION,
	PHIDGET_VINT_CONNECTION,
	PHIDGET_MESH_CONNECTION,
	PHIDGET_NET_CONNECTION,
	PHIDGET_NETCONN,		/* actual network connection */
	PHIDGET_VIRTUAL_CONNECTION,
	PHIDGET_PHIDUSB_CONNECTION
} PhidgetStructType;

#ifdef PLOCK
#undef PLOCK
#endif
#define PLOCK(p)		((p)->__ops->_lock((PhidgetHandle)(p)))

#ifdef PUNLOCK
#undef PUNLOCK
#endif
#define PUNLOCK(p)		((p)->__ops->_unlock((PhidgetHandle)(p)))

#define PRUNLOCK(p)		((p)->__ops->_runlock((PhidgetHandle)(p)))
#define PRUNUNLOCK(p)	((p)->__ops->_rununlock((PhidgetHandle)(p)))

typedef void (*PhidgetDelete_t)(void *);
typedef void (*PhidgetDestruct_t)(void *);
typedef void (*PhidgetRelease_t)(PhidgetHandle);
typedef void (*PhidgetRetain_t)(PhidgetHandle);
typedef uint32_t (*PhidgetRefCnt_t)(PhidgetHandle);
typedef void (*PhidgetLock_t)(PhidgetHandle);
typedef void (*PhidgetUnLock_t)(PhidgetHandle);

typedef struct PhidgetOps {
	PhidgetDestruct_t _destruct;
	PhidgetRetain_t _retain;
	PhidgetRelease_t _release;
	PhidgetRefCnt_t _refcnt;
	PhidgetLock_t _lock;
	PhidgetUnLock_t _unlock;
	PhidgetLock_t _runlock;
	PhidgetUnLock_t _rununlock;
} PhidgetOps_t;

#define PHIDGET_STRUCT_START											\
	PhidgetStructType type;												\
	PhidgetDelete_t _delete;											\
	const PhidgetOps_t *__ops;											\
	uint32_t __refcnt;													\
	uint32_t __flags;													\
	mos_fasttlock_t __lock;												\
	mos_tlock_t *__runlock;												\
	mos_cond_t cond;													\
	mos_task_t dispatchThread;											\
	void *dispatchOutHandle;											\
	void *dispatchInHandle;												\
	PhidgetDeviceHandle parent;											\
	MTAILQ_ENTRY(_Phidget) dispatchOutLink;	/* dispatcher list out (to user) linkage */ \
	MTAILQ_ENTRY(_Phidget) dispatchInLink;	/* dispatcher list in (from use) linkage */

struct _Phidget {
	PHIDGET_STRUCT_START
};

PhidgetHandle PhidgetCast(void *);

void PhidgetLock(void *);
void PhidgetUnlock(void *);
void PhidgetRunLock(void *);
void PhidgetRunUnlock(void *);
void PhidgetDeviceMemberLock(PhidgetDeviceHandle);
void PhidgetDeviceMemberUnlock(PhidgetDeviceHandle);

void PhidgetRetain(void *_phid);
void PhidgetRelease(void *_phidp);
uint32_t PhidgetGetRefCnt(void *_phid);

PhidgetReturnCode PhidgetTimedWait(void *, uint32_t ms);
void PhidgetWait(void *);
void PhidgetSignal(void *);
void PhidgetBroadcast(void *);

uint32_t PhidgetSetFlags(void *, uint32_t);
PhidgetReturnCode PhidgetCKandSetFlags(void *, uint32_t);
PhidgetReturnCode PhidgetCKandCLRFlags(void *, uint32_t);
uint32_t PhidgetCKFlags(void *, uint32_t);
uint32_t PhidgetCKFlagsNoLock(void *, uint32_t);
uint32_t PhidgetCLRFlags(void *, uint32_t);

#endif /* EXTERNALPROTO */
#endif /* _PHIDGETOBJECT_H_ */
