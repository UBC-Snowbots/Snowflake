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

#ifndef LIGHTNING_SUPPORT

PhidgetReturnCode
PhidgetLightningSendPacket(mosiop_t iop, PhidgetLightningConnectionHandle conn, const unsigned char *buffer, size_t len) {

	return (EPHIDGET_UNSUPPORTED);
}

#endif

PhidgetLightningConnectionHandle
mallocPhidgetLightningConnection() {

	return (mos_zalloc(sizeof (PhidgetLightningConnection)));
}

void
freePhidgetLightningConnection(PhidgetLightningConnectionHandle item) {

	mos_free(item, sizeof (*item));
}

PhidgetReturnCode
openAttachedLightningDevice(PhidgetDeviceHandle device) {
	PhidgetReturnCode res;

	PhidgetSetFlags(device, PHIDGET_ATTACHING_FLAG);
	res = device->initAfterOpen(device);
	PhidgetCLRFlags(device, PHIDGET_ATTACHING_FLAG);
	if (res == EPHIDGET_OK)
		return (EPHIDGET_OK);

	logerr("Device Initialization functions failed: "PRC_FMT, PRC_ARGS(res));
	if (res == EPHIDGET_BADVERSION)
		logwarn("This Phidget requires a newer library - please upgrade.");
	// Call device specific close function if it exists
	if (device->_closing)
		device->_closing(device);
	return (res);
}
