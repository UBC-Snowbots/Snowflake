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

PhidgetReturnCode
addMeshHubs(PhidgetDeviceHandle device) {
	int serial, id, version, portCnt, portModeCnt;
	PhidgetUSBConnectionHandle conn;
	const PhidgetUniqueDeviceDef *pdd;
	PhidgetHubDeviceHandle hub;
	char vintHubString[256];
	PhidgetReturnCode res;
	Phidget_MeshMode mode;
	int hubIndex, found;
	size_t len;
	int index;

	vintHubString[0] = '\0';
	index = 0;

	if (isNetworkPhidget(device))
		return (EPHIDGET_OK);

	conn = PhidgetUSBConnectionCast(device->conn);
	assert(conn);

	for (;;) {
		pdd = Phidget_Unique_Device_Def;

		if (PhidgetUSBGetString(conn, index++ + 5, vintHubString) != EPHIDGET_OK)
			break;

		len = strlen(vintHubString);
		if (len == 1 && vintHubString[0] == '0')	/* XXX This isn't possible.. */
			break;

		if (len < 19) {
			logerr("Wrong string length of VINT string from Mesh Dongle (too short): %zu", len);
			continue;
		}

		//read hub properties
		id = ((unsigned int)(vintHubString[9] - '0') << 8)
			+ ((unsigned int)(vintHubString[10] - '0') << 4)
			+ (vintHubString[11] - '0');

		version = ((unsigned int)(vintHubString[12] - '0') * 100)
			+ ((unsigned int)(vintHubString[13] - '0') << 4)
			+ (vintHubString[14] - '0');

		mode = (Phidget_MeshMode)(vintHubString[16] - '0');

		vintHubString[9] = '\0';
		hubIndex = (((unsigned int)(vintHubString[0] - '0') << 4) + (vintHubString[1] - '0'));
		serial = (int)strtol(vintHubString + 2, NULL, 10);
		portCnt = vintHubString[17] - '0';
		portModeCnt = vintHubString[18] - '0';

		if (len < (size_t)(19 + portCnt * 6 + portModeCnt * 6)) {
			logerr("Wrong string length of VINT string from Mesh Dongle: %zu/%d", len,
			  (19 + portCnt * 6 + portModeCnt * 6));
			continue;
		}

		found = PFALSE;

		//fill in the properties
		while (((int)pdd->type) != END_OF_LIST) {
			if (pdd->type == PHIDTYPE_MESH && id == pdd->productID && version >= pdd->versionLow &&
			  version < pdd->versionHigh) {
				hub = (PhidgetHubDeviceHandle)getChild(device, hubIndex);

				if (hub && hub->phid.deviceInfo.UDD == pdd && hub->phid.deviceInfo.serialNumber == serial &&
				  hub->phid.deviceInfo.version == version) {
					PhidgetSetFlags(hub, PHIDGET_SCANNED_FLAG);
				} else {
					res = createPhidgetMeshDevice(pdd, version, device->deviceInfo.label, serial, (PhidgetDeviceHandle *)&hub);
					if (res != EPHIDGET_OK)
						return (res);

					PhidgetSetFlags(hub, PHIDGET_SCANNED_FLAG);

					hub->phid.deviceInfo.uniqueIndex = hubIndex;
					hub->phid.deviceInfo.meshMode = mode;

					setParent(hub, device);
					setChild(device, hubIndex, hub);

					deviceAttach((PhidgetDeviceHandle)hub, 0);
				}

				//Update the vint device string and add vint devices
				memcpy(hub->portDescString, vintHubString + 19, portCnt * 6 + portModeCnt * 6);
				hub->portDescString[portCnt * 6 + portModeCnt * 6] = '\0';

				scanVintDevices((PhidgetDeviceHandle)hub);
				PhidgetRelease(&hub);

				found = PTRUE;
				break;
			}
			pdd++;
		}

		if (!found)
			logwarn("A Mesh Phidget (ID: 0x%04x Version: %d Serial: %d) was found which is not supported "
			  "by the library. A library upgrade is probably required to work with this Phidget",
			  id, version, serial);
	}

	return (EPHIDGET_OK);
}

static void
PhidgetMeshConnectionDelete(PhidgetMeshConnectionHandle *phid) {

	mos_free(*phid, sizeof(PhidgetMeshConnection));
}

PhidgetReturnCode
PhidgetMeshConnectionCreate(PhidgetMeshConnectionHandle *phid) {

	assert(phid);

	*phid = mos_zalloc(sizeof(PhidgetMeshConnection));
	phidget_init((PhidgetHandle)*phid, PHIDGET_MESH_CONNECTION, (PhidgetDelete_t)PhidgetMeshConnectionDelete);

	return (EPHIDGET_OK);
}

API_PRETURN
Phidget_setMeshMode(PhidgetHandle phid, Phidget_MeshMode mode) {
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle parent;
	PhidgetReturnCode ret;
#if PHIDUID_MESHHUB_SUPPORTED
	unsigned char buffer[1];
#endif

	TESTPTR_PR(phid);
	TESTATTACHED_PR(phid);

	CHANNELNOTDEVICE_PR(channel, phid);

	parent = getParent(channel);
	MOS_ASSERT(parent != NULL);	/* attached channel should have a parent */

	if (parent->deviceInfo.UDD->type != PHIDTYPE_MESH) {
		PhidgetRelease(&parent);
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	}

	switch (channel->UCD->uid) {

#if PHIDUID_MESHHUB_SUPPORTED
	case PHIDCHUID_MESHHUB_HUB_100:
		buffer[0] = mode;
		ret = sendHubPacket(NULL, (PhidgetHubDeviceHandle)parent, VINTHUB_HUBPACKET_MESHMODE, buffer, 1);
		break;
#endif /* PHIDUID_MESHHUB_SUPPORTED */

	default:
		ret = EPHIDGET_UNSUPPORTED;
		break;
	}

	if (ret == EPHIDGET_OK)
		parent->deviceInfo.meshMode = mode;

	PhidgetRelease(&parent);

	return (PHID_RETURN(ret));
}

API_PRETURN
Phidget_getMeshMode(PhidgetHandle phid, Phidget_MeshMode *mode) {
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle parent;

	CHANNELNOTDEVICE_PR(channel, phid);
	TESTATTACHED_PR(phid);

	parent = getParent(channel);
	MOS_ASSERT(parent != NULL);	/* attached channel should have a parent */

	if (parent->deviceInfo.UDD->type != PHIDTYPE_MESH) {
		PhidgetRelease(&parent);
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	}

	*mode = parent->deviceInfo.meshMode;
	PhidgetRelease(&parent);

	return (EPHIDGET_OK);
}
