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
#include "device/analogdevice.h"

// === Internal Functions === //
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetAnalogDeviceHandle phid);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetAnalogDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetAnalogDeviceHandle phid = (PhidgetAnalogDeviceHandle)device;
	int i;

	assert(phid);

	// set data arrays to unknown
	for (i = 0; i < phid->devChannelCnts.numAnalogOutputs; i++) {
		phid->voltage[i] = PUNK_DBL;
		phid->enabled[i] = PUNK_BOOL;
	}

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetAnalogDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetAnalogDeviceHandle phid = (PhidgetAnalogDeviceHandle)device;
	uint8_t overcurrent[ANALOG_MAXOUTPUTS];
	PhidgetChannelHandle channel;
	uint8_t tsd;
	int i;

	assert(phid);
	assert(buffer);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1002:
		// Bit-->	|   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
		//Byte 0	| oc[3] | oc[2] | oc[1] | oc[0] | en[3] | en[2] | en[1] | en[0] |
		//Byte 1	|   0   |   0   |   0   |   0   |   0   |   0   |   0   |  TSD  |
		//Byte 2	|       voltage[0] bits 3-0     |   0   |   0   |   0   |   0   |
		//Byte 3	|                   voltage[0] bits 11-4                        |
		//Byte 4	|       voltage[1] bits 3-0     |   0   |   0   |   0   |   0   |
		//Byte 5	|                   voltage[1] bits 11-4                        |
		//Byte 6	|       voltage[2] bits 3-0     |   0   |   0   |   0   |   0   |
		//Byte 7	|                   voltage[2] bits 11-4                        |
		//Byte 8	|       voltage[3] bits 3-0     |   0   |   0   |   0   |   0   |
		//Byte 9	|                   voltage[3] bits 11-4                        |
		for (i = 0; i < phid->devChannelCnts.numAnalogOutputs; i++)
			overcurrent[i] = (buffer[0] & (0x10 << i)) ? PTRUE : PFALSE;
		tsd = buffer[1];
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	for (i = 0; i < phid->devChannelCnts.numAnalogOutputs; i++) {
		if (!overcurrent[i])
			continue;
		if ((channel = getChannel(phid, i)) != NULL) {
			SEND_ERROR_EVENT(channel, EEPHIDGET_OVERCURRENT, "Output is trying to draw > 20mA - possible short circuit.");
			PhidgetRelease(&channel);
		}
	}

	if (tsd) {
		for (i = 0; i < phid->devChannelCnts.numAnalogOutputs; i++) {
			if ((channel = getChannel(phid, i)) != NULL) {
				SEND_ERROR_EVENT(channel, EEPHIDGET_OVERTEMP, "Thermal shutdown detected. All outputs have been disabled.");
				PhidgetRelease(&channel);
			}
		}
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetAnalogDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetAnalogDeviceHandle phid = (PhidgetAnalogDeviceHandle)ch->parent;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_ANALOG);
	assert(ch->class == PHIDCHCLASS_VOLTAGEOUTPUT);
	assert(ch->index < phid->devChannelCnts.numAnalogOutputs);

	switch (bp->vpkt) {
	case BP_SETENABLED:
		phid->enabled[ch->index] = (uint8_t)getBridgePacketInt32(bp, 0);
		return (_sendpacket(bp->iop, phid));
	case BP_SETVOLTAGE:
		phid->voltage[ch->index] = getBridgePacketDouble(bp, 0);
		return (_sendpacket(bp->iop, phid));
	case BP_OPENRESET:
	case BP_CLOSERESET:
		phid->enabled[ch->index] = PFALSE;
		phid->voltage[ch->index] = PUNK_DBL;
		return (_sendpacket(bp->iop, phid));
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}


//makePacket - constructs a packet using current device state
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetAnalogDeviceHandle phid) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	int iVoltage;
	int i;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1002:
		// Bit-->	|   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
		//Byte 0	|       voltage[0] bits 3-0     | en[3] | en[2] | en[1] | en[0] |
		//Byte 1	|                   voltage[0] bits 11-4                        |
		//Byte 2	|       voltage[1] bits 3-0     |   0   |   0   |   0   |   0   |
		//Byte 3	|                   voltage[1] bits 11-4                        |
		//Byte 4	|       voltage[2] bits 3-0     |   0   |   0   |   0   |   0   |
		//Byte 5	|                   voltage[2] bits 11-4                        |
		//Byte 6	|       voltage[3] bits 3-0     |   0   |   0   |   0   |   0   |
		//Byte 7	|                   voltage[3] bits 11-4                        |
		for (i = 0; i < phid->devChannelCnts.numAnalogOutputs; i++) {
			//fill in buffer
			if (phid->voltage[i] == PUNK_DBL)
				iVoltage = 0;
			else
				iVoltage = round(phid->voltage[i] * 2047 / 10.0);
			buffer[i * 2] = (unsigned char)(iVoltage << 4);
			buffer[i * 2 + 1] = (unsigned char)(iVoltage >> 4);
			if (phid->enabled[i])
				buffer[0] |= 1 << i;
		}
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
}

static void CCONV
PhidgetAnalogDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetAnalogDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetAnalogDevice_create(PhidgetAnalogDeviceHandle *phidp) {
	DEVICECREATE_BODY(AnalogDevice, PHIDCLASS_ANALOG);
	return (EPHIDGET_OK);
}
