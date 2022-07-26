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
#include "device/hubdevice.h"
#include "stats.h"

//PhidgetDeviceListHandle ActiveSPIDevices = 0;
//PhidgetDeviceListHandle AttachedSPIDevices = 0;

static void
PhidgetSPIConnectionDelete(PhidgetSPIConnectionHandle *conn) {

	mos_mutex_destroy(&(*conn)->rlock);
	mos_cond_destroy(&(*conn)->rcond);

	mos_free(*conn, sizeof(PhidgetSPIConnection));
}

PhidgetReturnCode
PhidgetSPIConnectionCreate(PhidgetSPIConnectionHandle *conn) {

	assert(conn);

	*conn = mos_zalloc(sizeof(PhidgetSPIConnection));
	phidget_init((PhidgetHandle)*conn, PHIDGET_SPI_CONNECTION, (PhidgetDelete_t)PhidgetSPIConnectionDelete);

	mos_mutex_init(&(*conn)->rlock);
	mos_cond_init(&(*conn)->rcond);
	(*conn)->dev = -1;

	return (EPHIDGET_OK);
}

MOS_TASK_RESULT
PhidgetSPIReadThreadFunction(void *param) {
	PhidgetSPIConnectionHandle conn;
	PhidgetDeviceHandle device;
	PhidgetReturnCode res;
	int attached;

	res = EPHIDGET_OK;
	conn = NULL;

	device = PhidgetDeviceCast(param);
	if (device == NULL) {
		logerr("SPI ReadThread exiting - Invalid device handle");
		goto exit;
	}

	loginfo("SPI ReadThread running for %"PRIphid"", device);

	conn = PhidgetSPIConnectionCast(device->conn);
	assert(conn != NULL);

	for (;;) {
		mos_mutex_lock(&conn->rlock);
		if (conn->run != 1 || !ISATTACHED(device))
			break;
		mos_mutex_unlock(&conn->rlock);

		res = PhidgetDevice_read(device);
		switch (res) {
		case EPHIDGET_TIMEOUT: // timeout is ok - spi hub only sends data when there is data to send.
		case EPHIDGET_OK:
			break;
		default:
			//logerr("SPI ReadThread exiting - PhidgetDevice_read returned: "PRC_FMT, PRC_ARGS(res));
			//goto exit;

			// XXX - for now, do NOT exit the read thread even in error condition,
			// because this leaves the device open but useless. Eventually, may want to
			// close the device handle in this case? Or critical error and crash?
			logerr("SPI ReadThread continuing - PhidgetDevice_read returned: "PRC_FMT, PRC_ARGS(res));
			break;
		}
	}

	attached = ISATTACHED(device);
	loginfo("SPI ReadThread exiting normally (Phidget %"PRIphid" is %s)", device, attached ? "attached" : "detached");

exit:

	if (conn) {
		conn->run = 0;
		mos_cond_broadcast(&conn->rcond);
		mos_mutex_unlock(&conn->rlock);
	}

	decPhidgetStat("spi.readthreads");
	MOS_TASK_EXIT(res);
}

void
joinSPIReadThread(PhidgetSPIConnectionHandle conn) {

	mos_mutex_lock(&conn->rlock);
	while (conn->run != 0) {
		conn->run = 2;
		mos_cond_broadcast(&conn->rcond);
		mos_cond_wait(&conn->rcond, &conn->rlock);
	}
	mos_mutex_unlock(&conn->rlock);
}

void
stopSPIReadThread(PhidgetSPIConnectionHandle conn) {

	mos_mutex_lock(&conn->rlock);
	if (conn->run != 0)
		conn->run = 2;
	mos_cond_broadcast(&conn->rcond);
	mos_mutex_unlock(&conn->rlock);
}

PhidgetReturnCode
openAttachedSPIDevice(PhidgetDeviceHandle device) {
	PhidgetSPIConnectionHandle conn;
	PhidgetReturnCode res;

	res = PhidgetSPIOpenHandle(device);
	if (res != EPHIDGET_OK)
		return (res);

	conn = PhidgetSPIConnectionCast(device->conn);
	assert(conn);

	mos_mutex_lock(&conn->rlock);
	conn->run = 1;
	res = mos_task_create(&conn->task, PhidgetSPIReadThreadFunction, device);
	if (res != 0) {
		conn->run = 0;
		mos_mutex_unlock(&conn->rlock);
		logwarn("unable to create read thread");
		if (device->_closing)
			device->_closing(device);
		PhidgetSPICloseHandle(conn);
		return (EPHIDGET_UNEXPECTED);
	}
	mos_mutex_unlock(&conn->rlock);
	incPhidgetStat("spi.readthreads_ever");
	incPhidgetStat("spi.readthreads");

	PhidgetSetFlags(device, PHIDGET_ATTACHING_FLAG);
	res = device->initAfterOpen((PhidgetDeviceHandle)device);
	if (res != EPHIDGET_OK) {
		logerr("Device Initialization functions failed: "PRC_FMT, PRC_ARGS(res));
		if (res == EPHIDGET_BADVERSION)
			logwarn("This Phidget requires a newer library - please upgrade.");
		PhidgetCLRFlags(device, PHIDGET_ATTACHING_FLAG);
		if (device->_closing)
			device->_closing(device);
		PhidgetSPICloseHandle(conn);
		return (res);
	}
	PhidgetCLRFlags(device, PHIDGET_ATTACHING_FLAG);

	return (EPHIDGET_OK);
}

#ifndef SPI_SUPPORT

PhidgetReturnCode
PhidgetSPIGetVINTDevicesString(char *str, size_t len) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPIOpenHandle(PhidgetDeviceHandle device) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPICloseHandle(PhidgetSPIConnectionHandle conn) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPIReadPacket(PhidgetSPIConnectionHandle conn, unsigned char *buffer, size_t *len) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPISendPacket(mosiop_t iop, PhidgetSPIConnectionHandle conn, const unsigned char *buffer, size_t len) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPISetLabel(PhidgetDeviceHandle device, char *buffer) {
	return (EPHIDGET_UNSUPPORTED);
}
PhidgetReturnCode
PhidgetSPIRefreshLabelString(PhidgetDeviceHandle device) {
	return (EPHIDGET_UNSUPPORTED);
}

#endif

