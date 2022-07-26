/*
 * This file is part of libphidget22
 *
 * Copyright 2015 Phidgets Inc <patrick@phidgets.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/>
 */


#include "phidgetbase.h"
#include "manager.h"
#include "device/hubdevice.h"
#include "network/network.h"

#include "stats.h"

PhidgetReturnCode dispatchManagerAttachChannel(PhidgetManagerHandle, PhidgetChannelHandle);
PhidgetReturnCode dispatchManagerDetachChannel(PhidgetManagerHandle, PhidgetChannelHandle);

/* A list of local Phidget Managers */
struct phidgetmanager_list phidgetManagerList = MTAILQ_HEAD_INITIALIZER(phidgetManagerList);

/* Protects phidgetManagerList */
static mos_tlock_t *managersLock;

void
PhidgetLockManagers() {

	mos_tlock_lock(managersLock);
}

void
PhidgetUnlockManagers() {

	mos_tlock_unlock(managersLock);
}

PhidgetManagerHandle
PhidgetManagerCast(void *_phid) {

	if (_phid == NULL)
		return (NULL);

	switch (((PhidgetHandle)_phid)->type) {
	case PHIDGET_MANAGER:
		return ((PhidgetManagerHandle)_phid);
	default:
		return (NULL);
	}
}

/*
 * Needs to be called during startup.
 *
 * Currently on Windows, in DllMain()
 */
void
PhidgetManagerInit() {

	mos_tlock_init(managersLock, P22LOCK_MANAGERLISTLOCK, P22LOCK_FLAGS);
}

void
PhidgetManagerFini() {

	if (managersLock)
		mos_tlock_destroy(&managersLock);
}

API_PRETURN
PhidgetManager_setOnErrorHandler(PhidgetManagerHandle manager, PhidgetManager_OnErrorCallback on, void *ctx) {

	TESTPTR_PR(manager);
	manager->onError = on;
	manager->onErrorCtx = ctx;
	return (EPHIDGET_OK);
}

PhidgetReturnCode
sendAttachVisitor(PhidgetDeviceHandle device, const PhidgetUniqueChannelDef *ucd, int index,
  int uniqueIndex, void *ctx) {
	PhidgetChannelHandle channel;
	PhidgetManagerHandle manager;
	PhidgetReturnCode res;
	const char *srvname;

	res = createTypedPhidgetChannelHandle(&channel, ucd->class);
	if (res != EPHIDGET_OK)
		return (res);

	channel->UCD = ucd;
	channel->index = index;
	channel->uniqueIndex = uniqueIndex;
	setParent(channel, device);
	PhidgetSetFlags(channel, PHIDGET_ATTACHED_FLAG);

	/*
	 * Restrict the open parameters for the new channel.
	 * XXX Do we want to flag the channel so that the filters cannot be changed?
	 */
	if (isNetworkPhidget(device)) {
		PhidgetSetFlags(channel, PHIDGET_NETWORK_FLAG);
		srvname = getPhidgetServerName(device);
		if (mos_strlen(srvname) > 0)
			Phidget_setServerName((PhidgetHandle)channel, srvname);
		Phidget_setIsRemote((PhidgetHandle)channel, 1);
	} else {
		Phidget_setIsLocal((PhidgetHandle)channel, 1);
	}

	Phidget_setDeviceSerialNumber((PhidgetHandle)channel, device->deviceInfo.serialNumber);
	if (mos_strlen(device->deviceInfo.label) > 0)
		Phidget_setDeviceLabel((PhidgetHandle)channel, device->deviceInfo.label);
	Phidget_setChannel((PhidgetHandle)channel, index);
	Phidget_setHubPort((PhidgetHandle)channel, device->deviceInfo.hubPort);
	Phidget_setIsHubPortDevice((PhidgetHandle)channel, device->deviceInfo.isHubPort);

	if (ctx) {	// send initial events
		res = dispatchManagerAttachChannel((PhidgetManagerHandle)ctx, channel);
	} else {	// new device
		PhidgetLockManagers();
		FOREACH_MANAGER(manager) {
			/*
			 * Dispatch the attach to the manager so that all of the attach events are serialized.
			 */
			dispatchManagerAttachChannel(manager, channel);
		}
		PhidgetUnlockManagers();
	}

	PhidgetRelease(&channel);
	return (res);
}

PhidgetReturnCode
sendDetachVisitor(PhidgetDeviceHandle device, const PhidgetUniqueChannelDef *ucd, int index,
  int uniqueIndex, void *ctx) {
	PhidgetChannelHandle channel;
	PhidgetManager *manager;
	PhidgetReturnCode res;

	res = createTypedPhidgetChannelHandle(&channel, ucd->class);
	if (res != EPHIDGET_OK)
		return (res);

	channel->UCD = ucd;
	channel->index = index;
	channel->uniqueIndex = uniqueIndex;
	setParent(channel, device);
	PhidgetSetFlags(channel, PHIDGET_DETACHING_FLAG);

	PhidgetLockManagers();
	FOREACH_MANAGER(manager) {
		/*
		 * Dispatch the attach to the manager so that all of the attach events are serialized.
		 */
		dispatchManagerDetachChannel(manager, channel);
	}

	PhidgetUnlockManagers();
	PhidgetRelease(&channel);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
deviceAttach(PhidgetDeviceHandle device, int needDevicesLock) {
	PhidgetReturnCode res;

	MOS_ASSERT(device != NULL);

	if (PhidgetCKandSetFlags(device, PHIDGET_ATTACHED_FLAG) != EPHIDGET_OK)
		MOS_PANIC("Already attached");

	if (device->initAfterCreate != NULL) {
		res = device->initAfterCreate(device);
		if (res != EPHIDGET_OK) {
			logerr("Device Initialization after create failed: "PRC_FMT, PRC_ARGS(res));
			return (res);
		}
	}

	if (needDevicesLock)
		addDevice(device);
	else
		_addDevice(device);

	startDispatch((PhidgetHandle)device);
	sendNetDeviceAttached(device, NULL);

	incPhidgetStat("device.attached");

	if (!device->deviceInfo.isHubPort) {
		if (isNetworkPhidget(device))
			loginfo("(Attach) %"PRIphid" -> %s", device, getPhidgetServerName(device));
		else
			loginfo("(Attach) %"PRIphid, device);
	}

	return (EPHIDGET_OK);
}

void
channelClose(PhidgetChannelHandle channel) {

	if (channel == NULL)
		return;

	logdebug("%"PRIphid" closing", channel);

	// If it's detaching, the dispatch has already been stopped
	if (!_ISDETACHING(channel)) {
		PhidgetCLRFlags(channel, PHIDGET_ATTACHED_FLAG);
		stopDispatch((PhidgetHandle)channel, 0);
	}

	removeChannelNetworkConnections(channel);

	if (PhidgetCKFlags(channel, PHIDGET_OPENBYNETCLIENT_FLAG))
		PhidgetRelease(&channel);	/* release the network reference */
}

void
channelDetach(PhidgetChannelHandle channel) {

	if (channel == NULL)
		return;

	chlog("%"PRIphid"", channel);

	/*
	 * If attaching is happening on another thread, wait
	 */
	PhidgetLock(channel);

	if (_ISATTACHING(channel) && channel->dispatchThread == mos_self()) {
		logdebug("Not waiting for attaching to clear, as we are in the context of the attaching thread");
		PhidgetUnlock(channel);
		return;
	}

	while (_ISATTACHING(channel))
		PhidgetWait(channel);

	PhidgetUnlock(channel);

	/*
	 * If channel was never attached, don't run detach event
	 */
	if (PhidgetCKFlags(channel, PHIDGET_ATTACHED_FLAG) == 0) {
		stopDispatch((PhidgetHandle)channel, 0);
	} else {
		PhidgetSetFlags(channel, PHIDGET_DETACHING_FLAG);
		stopDispatch((PhidgetHandle)channel, 1);
	}

	removeChannelNetworkConnections(channel);

	/*
	 * If the channel was opened over the network, close it.
	 */
	if (PhidgetCKFlags(channel, PHIDGET_OPENBYNETCLIENT_FLAG))
		(void)Phidget_close((PhidgetHandle)channel);
}

void
deviceDetach(PhidgetDeviceHandle device) {
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle child;
	int i;

	chlog("%"PRIphid"", device);
	if (device == NULL)
		return;

	MOS_ASSERT(mos_tlock_islocked(devicesLock));

	if (PhidgetCKFlags(device, PHIDGET_ATTACHED_FLAG) == 0)
		return;

	// This prevents simultaneous detaches of the same device from separate threads
	if (PhidgetCKandSetFlags(device, PHIDGET_DETACHING_FLAG) != EPHIDGET_OK)
		return;

	if (!device->deviceInfo.isHubPort)
		logdebug("%"PRIphid"", device);

	for (i = 0; i < PHIDGET_MAXCHILDREN; i++) {
		child = getChild(device, i);
		if (child) {
			chlog("child %"PRIphid"", device);
			deviceDetach(child);
			setChild(device, i, NULL);
			PhidgetRelease(&child);
		}
	}

	for (i = 0; i < PHIDGET_MAXCHANNELS; i++) {
		channel = getChannel(device, i);
		if (channel) {
			chlog("channel %"PRIphid"", channel);
			channelDetach(channel);
			PhidgetRelease(&channel);
			setChannel(device, i, NULL);
		}
	}

	PhidgetCLRFlags(device, PHIDGET_ATTACHED_FLAG);

	sendNetDeviceDetached(device);

	closeDevice(device, PTRUE);

	stopDispatch((PhidgetHandle)device, 1);

	// XXX testing - need to make sure the deviceHandle is closed on detach, rather then sticking around until the connection object is finalized
	//  But, this is a bit of a hackly place to work with the conn object directly. Maybe add a flag to the closeDevice call?
#if defined(_LINUX) || defined (_FREEBSD)
	if (device->connType == PHIDCONN_HIDUSB) {
		PhidgetHIDUSBConnectionHandle conn;
		PhidgetRunLock(device);
		conn = PhidgetHIDUSBConnectionCast(device->conn);
		assert(conn);
		if (conn->deviceHandle)
			libusb_close((libusb_device_handle *)conn->deviceHandle); //close handle
		conn->deviceHandle = NULL;
		if (conn->dev)
			libusb_unref_device(conn->dev); //decrease reference cnt.
		conn->dev = NULL;
		PhidgetRunUnlock(device);
	}
	if (device->connType == PHIDCONN_PHIDUSB) {
		PhidgetPHIDUSBConnectionHandle conn;
		PhidgetRunLock(device);
		conn = PhidgetPHIDUSBConnectionCast(device->conn);
		assert(conn);
		if (conn->deviceHandle)
			libusb_close((libusb_device_handle *)conn->deviceHandle); //close handle
		conn->deviceHandle = NULL;
		if (conn->dev)
			libusb_unref_device(conn->dev); //decrease reference cnt.
		conn->dev = NULL;
		PhidgetRunUnlock(device);
	}
#endif

	if (!device->deviceInfo.isHubPort) {
		if (isNetworkPhidget(device))
			loginfo("(Detach) %"PRIphid" -> %s", device, getPhidgetServerName(device));
		else
			loginfo("(Detach) %"PRIphid, device);
	}

	_removeDevice(device);
	/* Device may not exist anymore.. */

	decPhidgetStat("device.attached");
}

PhidgetReturnCode
walkDeviceChannels(PhidgetDeviceHandle device, deviceChannelVisitor_t visit, void *ctx) {
	const PhidgetUniqueChannelDef *ucd;
	PhidgetReturnCode res;
	int channelIndex;
	int channelCnt;
	int i;

	channelCnt = 0;

	ucd = device->deviceInfo.UDD->channels;
	while (((int)ucd->uid) != END_OF_LIST) {
		for (i = 0; i < ucd->count; i++) {
			channelIndex = i + ucd->index;
			res = visit(device, ucd, channelIndex, channelCnt, ctx);
			if (res != EPHIDGET_OK)
				return (res);
			channelCnt++;
		}
		ucd++;
	}
	return (EPHIDGET_OK);
}


/*
 * _poll() is called from the CentralThread in phidget22.c
 *  _poll() calls PhidgetUSBScanDevices()
 *    PhidgetUSBScanDevices() walks all USB devices, adding (deviceAttach()) and flagging existing
 *  _poll() detaches any devices without PHIDGET_SCANNED_FLAG
 */
PhidgetReturnCode
PhidgetManager_poll() {
	PhidgetDeviceHandle device;

again:

	PhidgetUSBScanDevices();
	PhidgetWriteLockDevices();

	/*
	 * detachDevice() may detach more then once device (VINT hub), so
	 * if we detach, we must start this whole process over again.
	 */
	FOREACH_DEVICE(device) {
		if (isNetworkPhidget(device))
			continue;
		if (device->connType == PHIDCONN_SPI)
			continue;
		if (device->connType == PHIDCONN_VINT)
			continue;
		if (device->connType == PHIDCONN_VIRTUAL)
			continue;

		if (PhidgetCKandCLRFlags(device, PHIDGET_SCANNED_FLAG) != EPHIDGET_OK) {
			chlog("noscan %"PRIphid"", device);
			deviceDetach(device);
			PhidgetUnlockDevices();
			goto again;
		}
	}

	FOREACH_DEVICE(device) {
		switch (device->deviceInfo.class) {
		case PHIDCLASS_MESHDONGLE:
			addMeshHubs(device);
			break;
		case PHIDCLASS_HUB:
			scanVintDevices(device);
			break;
		default:
			break;
		}
	}

	PhidgetUnlockDevices();

	return (EPHIDGET_OK);
}

static void
PhidgetManager_free(void **arg) {

	if (arg == NULL || *arg == NULL)
		return;

	mos_free(*arg, sizeof(PhidgetManager));
}

API_PRETURN
PhidgetManager_create(PhidgetManagerHandle *manager) {

	TESTPTR_PR(manager);
	*manager = mos_zalloc(sizeof(PhidgetManager));
	phidget_init((PhidgetHandle)*manager, PHIDGET_MANAGER, (PhidgetDelete_t)PhidgetManager_free);

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetManager_open(PhidgetManagerHandle manager) {
	PhidgetReturnCode res;

	TESTPTR_PR(manager);

	res = PhidgetCKandSetFlags(manager, PHIDGET_OPEN_FLAG | PHIDGET_ATTACHED_FLAG);
	if (res != EPHIDGET_OK) {
		logwarn("Open was called on an already opened Manager handle.");
		return (EPHIDGET_OK);
	}

	PhidgetLockManagers();
	MTAILQ_INSERT_HEAD(&phidgetManagerList, manager, link);
	PhidgetUnlockManagers();

	startDispatch((PhidgetHandle)manager);

	res = StartCentralThread();

	return (PHID_RETURN(res));
}

API_PRETURN
PhidgetManager_close(PhidgetManagerHandle manager) {
	PhidgetReturnCode res;

	TESTPTR_PR(manager);

	res = PhidgetCKandCLRFlags(manager, PHIDGET_OPEN_FLAG | PHIDGET_ATTACHED_FLAG);
	if (res != EPHIDGET_OK) {
		logverbose("Close was called on an already closed Manager handle.");
		return (EPHIDGET_OK);
	}

	PhidgetLockManagers();
	MTAILQ_REMOVE(&phidgetManagerList, manager, link);
	PhidgetUnlockManagers();

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetManager_delete(PhidgetManagerHandle *manager) {

	TESTPTR_PR(manager);

	if (PhidgetCKFlags(*manager, PHIDGET_OPEN_FLAG))
		PhidgetManager_close(*manager);

	clearPhidgetDispatch((PhidgetHandle)*manager);
	PhidgetRelease(manager);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetManager_setOnAttachHandler(PhidgetManagerHandle manager, PhidgetManager_OnAttachCallback on, void *ctx) {

	TESTPTR_PR(manager);
	PhidgetLock(manager);
	manager->onAttach = on;
	manager->onAttachCtx = ctx;
	PhidgetUnlock(manager);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetManager_setOnDetachHandler(PhidgetManagerHandle manager, PhidgetManager_OnDetachCallback on, void *ctx) {

	TESTPTR_PR(manager);
	PhidgetLock(manager);
	manager->onDetach = on;
	manager->onDetachCtx = ctx;
	PhidgetUnlock(manager);
	return (EPHIDGET_OK);
}
