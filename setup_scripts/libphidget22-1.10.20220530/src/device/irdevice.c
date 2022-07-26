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
#include "device/irdevice.h"
#include "usb.h"

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetIRDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetIRDeviceHandle phid = (PhidgetIRDeviceHandle)device;
	assert(phid);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1055:
	case PHIDUID_1055_1_USB:
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetIRDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetIRDeviceHandle phid = (PhidgetIRDeviceHandle)device;
	PhidgetChannelHandle channel;
	PhidgetReturnCode res;

	assert(phid);
	assert(buffer);

	res = EPHIDGET_OK;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1055:
	case PHIDUID_1055_1_USB:
		if ((channel = getChannel(device, 0)) != NULL) {
			res = PhidgetIRSupport_dataInput(channel, buffer, length);
			PhidgetRelease(&channel);
		}
		return (res);

	default:
		MOS_PANIC("Unexpected device");
	}
}

static PhidgetReturnCode CCONV
PhidgetIRDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetIRDeviceHandle phid;
	phid = (PhidgetIRDeviceHandle)ch->parent;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_IR);
	assert(ch->class == PHIDCHCLASS_IR);
	assert(ch->index == 0);

	switch (bp->vpkt) {
	case BP_TRANSMIT:
	case BP_TRANSMITRAW:
	case BP_TRANSMITREPEAT:
		return (PhidgetIRSupport_bridgeInput(ch, bp));

	case BP_OPENRESET:
	case BP_CLOSERESET:
		switch (phid->phid.deviceInfo.UDD->uid) {
		case PHIDUID_1055:
		case PHIDUID_1055_1_USB:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected device");
		}
	case BP_ENABLE:
		return (EPHIDGET_OK);

	default:
		MOS_PANIC("Unexpected packet type");
	}
}

static void CCONV
PhidgetIRDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetIRDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetIRDevice_create(PhidgetIRDeviceHandle *phidp) {
	DEVICECREATE_BODY(IRDevice, PHIDCLASS_IR);
	return (EPHIDGET_OK);
}
