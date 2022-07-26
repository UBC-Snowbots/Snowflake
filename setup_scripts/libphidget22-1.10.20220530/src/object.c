#include "phidgetbase.h"
#include "object.h"
#include "locks.h"

#include "mos/mos_atomic.h"

static uint32_t phidcount;

void PhidgetObjectInit(void);

void
PhidgetObjectInit(void) {
}

void PhidgetObjectFini(void);

void
PhidgetObjectFini(void) {

	logdebug("phidcount=%u", phidcount);
}

static void
_phidget_lock(PhidgetHandle phid) {

	MOS_ASSERT(phid != NULL);
	mos_fasttlock_lock(&phid->__lock);
}

static void
_phidget_unlock(PhidgetHandle phid) {

	MOS_ASSERT(phid != NULL);
	mos_fasttlock_unlock(&phid->__lock);
}

static void
_phidget_runlock(PhidgetHandle phid) {

	MOS_ASSERT(phid != NULL);
	mos_tlock_lock(phid->__runlock);
}

static void
_phidget_rununlock(PhidgetHandle phid) {

	MOS_ASSERT(phid != NULL);
	mos_tlock_unlock(phid->__runlock);
}

static void
_phidget_retain(PhidgetHandle phid) {

	MOS_ASSERT(phid != NULL);

	PLOCK(phid);
	phid->__refcnt++;
	PUNLOCK(phid);
}

static void
_phidget_release(PhidgetHandle phid) {

	MOS_ASSERT(phid != NULL);

	PLOCK(phid);
	phid->__refcnt--;
	if (phid->__refcnt == 0) {
		phid->__ops->_destruct(phid);
		return;	/* object is freed, and the lock is closed and destroyed */
	}
	PUNLOCK(phid);
}

static uint32_t
_phidget_refcnt(PhidgetHandle phid) {
	uint32_t res;

	MOS_ASSERT(phid != NULL);

	PLOCK(phid);
	res = phid->__refcnt;
	PUNLOCK(phid);

	return (res);
}

static void
_phidgetdevice_fini(PhidgetDeviceHandle device) {

	mos_tlock_destroy(&device->__memberlock);
	mos_mutex_destroy(&device->bridgeInputLock);

	if (device->lastOpenErrstr) {
		mos_free(device->lastOpenErrstr, PHIDGET_DEVICE_LAST_ERROR_STR_LEN);
		device->lastOpenErrstr = NULL;
	}

	//XXX why is this here?
	//assert(PhidgetGetRefCnt(device->conn) == 1);
	PhidgetRelease(&device->conn);

	if (device->packetTracking)
		freePhidgetPacketTrackers(device->packetTracking);
}

static void
_phidgetchannel_fini(PhidgetChannelHandle channel) {

	removeChannelNetworkConnections(channel);

	mos_mutex_destroy(&channel->netconnslk);

	if (channel->openInfo)
		freePhidgetOpenInfo(channel->openInfo);
}

static void
_phidget_fini(PhidgetHandle phid) {

	mos_atomic_add_32(&phidcount, -1);

	PUNLOCK(phid);

	clearPhidgetDispatch(phid);
	freeDispatchHandle(phid);

	if (phid->parent)
		PhidgetRelease(&phid->parent);

	if (phid->type == PHIDGET_DEVICE)
		_phidgetdevice_fini((PhidgetDeviceHandle)phid);

	if (phid->type == PHIDGET_CHANNEL)
		_phidgetchannel_fini((PhidgetChannelHandle)phid);

	phid->__ops = (void *)(uintptr_t)0xdeadbeef;

	mos_fasttlock_destroy(&phid->__lock);
	mos_tlock_destroy(&phid->__runlock);
	mos_cond_destroy(&phid->cond);

	MOS_ASSERT(phid->_delete != NULL);
	phid->_delete(&phid);
	/* the phid is now free */
}

static const PhidgetOps_t core_phidget_ops = {
	(PhidgetDestruct_t)_phidget_fini,
	_phidget_retain,
	_phidget_release,
	_phidget_refcnt,
	_phidget_lock,
	_phidget_unlock,
	_phidget_runlock,
	_phidget_rununlock
};

void
phidget_init(PhidgetHandle phid, PhidgetStructType type, PhidgetDelete_t _delete) {

	mos_atomic_add_32(&phidcount, 1);

	MOS_ASSERT(_delete != NULL);
	phid->_delete = _delete;

	switch (type) {
	case PHIDGET_CHANNEL:
		mos_fasttlock_init(&phid->__lock, P22LOCK_CHANNELLOCK, P22LOCK_FLAGS);
		mos_tlock_init(phid->__runlock, P22LOCK_CHANNELRUNLOCK, P22LOCK_FLAGS);
		break;
	case PHIDGET_DEVICE:
		mos_fasttlock_init(&phid->__lock, P22LOCK_DEVICELOCK, P22LOCK_FLAGS);
		mos_tlock_init(phid->__runlock, P22LOCK_DEVICERUNLOCK, P22LOCK_FLAGS);
		mos_tlock_init(((PhidgetDeviceHandle)phid)->__memberlock, P22LOCK_DEVICEMEMBERLOCK, P22LOCK_FLAGS);
		mos_mutex_init(&((PhidgetDeviceHandle)phid)->bridgeInputLock);
		break;
	case PHIDGET_MANAGER:
		mos_fasttlock_init(&phid->__lock, P22LOCK_MANAGERLOCK, P22LOCK_FLAGS);
		mos_tlock_init(phid->__runlock, P22LOCK_MANAGERRUNLOCK, P22LOCK_FLAGS);
		break;
	case PHIDGET_HIDUSB_CONNECTION:
	case PHIDGET_PHIDUSB_CONNECTION:
	case PHIDGET_SPI_CONNECTION:
	case PHIDGET_LIGHTNING_CONNECTION:
	case PHIDGET_VINT_CONNECTION:
	case PHIDGET_MESH_CONNECTION:
	case PHIDGET_NET_CONNECTION:
	case PHIDGET_VIRTUAL_CONNECTION:
		mos_fasttlock_init(&phid->__lock, P22LOCK_CONNECTIONLOCK, P22LOCK_FLAGS);
		mos_tlock_init(phid->__runlock, P22LOCK_CONNECTIONRUNLOCK, P22LOCK_FLAGS);
		break;
	case PHIDGET_NETCONN:
		mos_fasttlock_init(&phid->__lock, P22LOCK_NETCONNLOCK, P22LOCK_FLAGS);
		mos_tlock_init(phid->__runlock, P22LOCK_NETCONNWRLOCK, P22LOCK_FLAGS);
		break;
	default:
		MOS_PANIC("Invalid Phidget Type");
	}

	mos_cond_init(&phid->cond);

	phid->type = type;
	phid->__flags = 0;
	phid->__refcnt = 1;

	phid->__ops = &core_phidget_ops;
}

void
PhidgetLock(void *_phid) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	PLOCK(phid);
}

void
PhidgetUnlock(void *_phid) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	PUNLOCK(phid);
}

void
PhidgetRunLock(void *_phid) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	PRUNLOCK(phid);
}

void
PhidgetRunUnlock(void *_phid) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	PRUNUNLOCK(phid);
}

void
PhidgetDeviceMemberLock(PhidgetDeviceHandle phid) {

	MOS_ASSERT(phid != NULL);
	mos_tlock_lock(phid->__memberlock);
}

void
PhidgetDeviceMemberUnlock(PhidgetDeviceHandle phid) {

	MOS_ASSERT(phid != NULL);
	mos_tlock_unlock(phid->__memberlock);
}
void
PhidgetWait(void *_phid) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	mos_fasttlock_wait(&phid->cond, &phid->__lock);
}

PhidgetReturnCode
PhidgetTimedWait(void *_phid, uint32_t ms) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	return (mos_fasttlock_timedwait(&phid->cond, &phid->__lock, (uint64_t)(ms * 1000000)));
}

void
PhidgetSignal(void *_phid) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	mos_cond_signal(&phid->cond);
}

void
PhidgetBroadcast(void *_phid) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	mos_cond_broadcast(&phid->cond);
}

PhidgetChannelHandle
PhidgetChannelCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_CHANNEL:
		return ((PhidgetChannelHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetDeviceHandle
PhidgetDeviceCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_DEVICE:
		return ((PhidgetDeviceHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetHandle
PhidgetCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_CHANNEL:
	case PHIDGET_DEVICE:
	case PHIDGET_MANAGER:
	case PHIDGET_HIDUSB_CONNECTION:
	case PHIDGET_PHIDUSB_CONNECTION:
	case PHIDGET_SPI_CONNECTION:
	case PHIDGET_LIGHTNING_CONNECTION:
	case PHIDGET_VINT_CONNECTION:
	case PHIDGET_MESH_CONNECTION:
	case PHIDGET_NET_CONNECTION:
	case PHIDGET_NETCONN:
	case PHIDGET_VIRTUAL_CONNECTION:
		return ((PhidgetHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetUSBConnectionHandle
PhidgetUSBConnectionCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_HIDUSB_CONNECTION:
	case PHIDGET_PHIDUSB_CONNECTION:
		return ((PhidgetUSBConnectionHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetHIDUSBConnectionHandle
PhidgetHIDUSBConnectionCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_HIDUSB_CONNECTION:
		return ((PhidgetHIDUSBConnectionHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetPHIDUSBConnectionHandle
PhidgetPHIDUSBConnectionCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_PHIDUSB_CONNECTION:
		return ((PhidgetPHIDUSBConnectionHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetNetworkConnectionHandle
PhidgetNetworkConnectionCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_NET_CONNECTION:
		return ((PhidgetNetworkConnectionHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetNetConnHandle
PhidgetNetConnCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_NETCONN:
		return ((PhidgetNetConnHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetSPIConnectionHandle
PhidgetSPIConnectionCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_SPI_CONNECTION:
		return ((PhidgetSPIConnectionHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetLightningConnectionHandle
PhidgetLightningConnectionCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_LIGHTNING_CONNECTION:
		return ((PhidgetLightningConnectionHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetVINTConnectionHandle
PhidgetVINTConnectionCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_VINT_CONNECTION:
		return ((PhidgetVINTConnectionHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetMeshConnectionHandle
PhidgetMeshConnectionCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_MESH_CONNECTION:
		return ((PhidgetMeshConnectionHandle)_phid);
	default:
		return (NULL);
	}
}

PhidgetVirtualConnectionHandle
PhidgetVirtualConnectionCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_VIRTUAL_CONNECTION:
		return ((PhidgetVirtualConnectionHandle)_phid);
	default:
		return (NULL);
	}
}

/*
 * If flags is already set, we return an error indicated that the phidget is already in the flagged state;
 * otherwise, the flags are set and EPHIDGET_OK is returned.
 *
 * This provides an atomic way to check and set the state of a phidget.
 */
PhidgetReturnCode
PhidgetCKandSetFlags(void *_phid, uint32_t flags) {
	PhidgetHandle phid;
	uint32_t res;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	res = EPHIDGET_OK;

	PLOCK(phid);
	if ((phid->__flags & flags) == flags) {
		res = EPHIDGET_DUPLICATE;
	} else {
		phid->__flags |= flags;
		mos_cond_broadcast(&phid->cond);
	}
	PUNLOCK(phid);

	return (res);
}

/*
* If flags are not set, we return an error indicated that the phidget is already not in the flagged state;
* otherwise, the flags are cleared and EPHIDGET_OK is returned.
*
* This provides an atomic way to check and clear the state of a phidget.
*/
PhidgetReturnCode
PhidgetCKandCLRFlags(void *_phid, uint32_t flags) {
	PhidgetHandle phid;
	uint32_t res;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	res = EPHIDGET_OK;

	PLOCK(phid);
	if ((phid->__flags & flags) == flags) {
		phid->__flags &= ~flags;
		mos_cond_broadcast(&phid->cond);
	} else {
		res = EPHIDGET_NOENT;
	}
	PUNLOCK(phid);

	return (res);
}

uint32_t
PhidgetCKFlags(void *_phid, uint32_t flags) {
	PhidgetHandle phid;
	uint32_t res;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	PLOCK(phid);
	res = phid->__flags & flags;
	PUNLOCK(phid);

	return (res);
}
uint32_t
PhidgetCKFlagsNoLock(void *_phid, uint32_t flags) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	return (phid->__flags & flags);
}

uint32_t
PhidgetSetFlags(void *_phid, uint32_t flags) {
	PhidgetHandle phid;
	uint32_t res;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	PLOCK(phid);
	phid->__flags |= flags;
	res = phid->__flags;
	mos_cond_broadcast(&phid->cond);
	PUNLOCK(phid);

	return (res);
}

uint32_t
PhidgetCLRFlags(void *_phid, uint32_t flags) {
	PhidgetHandle phid;
	uint32_t res;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	PLOCK(phid);
	phid->__flags &= ~flags;
	res = phid->__flags;
	mos_cond_broadcast(&phid->cond);
	PUNLOCK(phid);

	return (res);
}

void
PhidgetRetain(void *_phid) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	phid->__ops->_retain(phid);
}

void
PhidgetRelease(void *_phid) {
	PhidgetHandle phid;

	if (_phid == NULL)
		return;

	phid = PhidgetCast(*(void **)_phid);
	if (phid == NULL)
		return;

	phid->__ops->_release(phid);
	*(void **)_phid = NULL;
}

uint32_t
PhidgetGetRefCnt(void *_phid) {
	PhidgetHandle phid;

	phid = PhidgetCast(_phid);
	MOS_ASSERT(phid != NULL);

	return (phid->__ops->_refcnt(phid));
}

