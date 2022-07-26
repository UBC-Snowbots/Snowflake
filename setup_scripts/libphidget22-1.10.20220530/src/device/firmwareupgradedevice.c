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
#include "gpp.h"
#include "device/firmwareupgradedevice.h"

// === Internal Functions === //

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetFirmwareUpgradeDevice_initAfterOpen(PhidgetDeviceHandle device) {

	PhidgetFirmwareUpgradeDeviceHandle phid = (PhidgetFirmwareUpgradeDeviceHandle)device;
	assert(phid);

	phid->progress[0] = PUNK_DBL;

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetFirmwareUpgradeDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetFirmwareUpgradeDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetFirmwareUpgradeDeviceHandle phid = (PhidgetFirmwareUpgradeDeviceHandle)ch->parent;
	PhidgetReturnCode ret;
	size_t base64Len;
	void *base64;
	int off;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_FIRMWAREUPGRADE);
	assert(ch->class == PHIDCHCLASS_FIRMWAREUPGRADE);

	switch (bp->vpkt) {
	case BP_SENDFIRMWARE:
		off = 0;
		ret = bridgePacketBase64Decode(bp, NULL, &base64Len, &off);
		if (ret != EPHIDGET_OK)
			return (ret);

		base64 = mos_malloc(base64Len);
		ret = bridgePacketBase64Decode(bp, base64, &base64Len, &off);
		if (ret != EPHIDGET_OK)
			return (ret);

		ret = GPP_eraseFirmware(bp->iop, (PhidgetDeviceHandle)phid);
		if (ret != EPHIDGET_OK) {
			mos_free(base64, base64Len);
			return (ret);
		}

		ret = GPP_upgradeFirmware(bp->iop, (PhidgetDeviceHandle)phid, base64, base64Len, ch);
		mos_free(base64, base64Len);
		return (ret);

	case BP_OPENRESET:
	case BP_CLOSERESET:
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

static void CCONV
PhidgetFirmwareUpgradeDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetFirmwareUpgradeDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetFirmwareUpgradeDevice_create(PhidgetFirmwareUpgradeDeviceHandle *phidp) {
	DEVICECREATE_BODY(FirmwareUpgradeDevice, PHIDCLASS_FIRMWAREUPGRADE);
	return (EPHIDGET_OK);
}
