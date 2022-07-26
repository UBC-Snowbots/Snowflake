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
#include "device/servodevice.h"

// === Internal Functions === //=
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetServoDeviceHandle phid, int Index);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetServoDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetServoDeviceHandle phid = (PhidgetServoDeviceHandle)device;
	int i = 0;

	assert(phid);

	//Setup max/min values
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1000_OLD1:
	case PHIDUID_1000_OLD2:
	case PHIDUID_1001_OLD1:
	case PHIDUID_1001_OLD2:
		phid->motorPositionMaxLimit = 2550;
		phid->motorPositionMinLimit = 1;
		phid->fullStateEcho = PFALSE;
		break;

	case PHIDUID_1000_NO_ECHO:
	case PHIDUID_1001_NO_ECHO:
		phid->motorPositionMaxLimit = 4095;
		phid->motorPositionMinLimit = 1;
		phid->fullStateEcho = PFALSE;
		break;

	case PHIDUID_1000:
	case PHIDUID_1001:
		phid->motorPositionMaxLimit = 4095;
		phid->motorPositionMinLimit = 1;
		phid->fullStateEcho = PTRUE;
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		//set data arrays to unknown
		phid->position[i] = PUNK_DBL;
		phid->motorEngagedState[i] = PUNK_BOOL;
		phid->targetPosition[i] = PUNK_DBL;
	}

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetServoDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetServoDeviceHandle phid = (PhidgetServoDeviceHandle)device;
	double lastPosition[SERVO_MAXSERVOS] = { 0 };
	double position[SERVO_MAXSERVOS] = { 0 };
	int engaged[SERVO_MAXSERVOS];
	PhidgetChannelHandle channel;
	int i;

	assert(phid);
	assert(buffer);

	//Parse device packets - store data locally
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1000:
		position[0] = ((((uint16_t)buffer[0]) << 5) + buffer[1]);
		break;

	case PHIDUID_1001:
		for (i = 0; i < phid->devChannelCnts.numMotors; i++)
			position[i] = ((((uint16_t)buffer[i * 2]) << 5) + buffer[(i * 2) + 1]);
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	//Make sure values are within defined range, and store to structure
	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		lastPosition[i] = phid->position[i];
		if (position[i] > phid->motorPositionMaxLimit || position[i] < phid->motorPositionMinLimit) {
			phid->position[i] = PUNK_DBL;
			engaged[i] = PFALSE;
		} else {
			phid->position[i] = position[i];
			engaged[i] = PTRUE;
		}
	}

	//TODO: implement STOPPED event

	//send out any events for changed data
	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		if ((channel = getChannel(phid, i)) != NULL) {
			if (phid->position[i] != PUNK_DBL && phid->position[i] != lastPosition[i] && engaged[i] == PTRUE) {
				bridgeSendToChannel(channel, BP_POSITIONCHANGE, "%g", phid->position[i]);
			}
			PhidgetRelease(&channel);
		}
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetServoDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetServoDeviceHandle phid = (PhidgetServoDeviceHandle)ch->parent;
	PhidgetReturnCode res;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_SERVO);
	assert(ch->class == PHIDCHCLASS_RCSERVO);
	assert(ch->index < phid->devChannelCnts.numMotors);

	switch (bp->vpkt) {
	case BP_SETENGAGED:
		if (phid->targetPosition[ch->index] == PUNK_DBL)
			return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Target position must be set before engaging servo."));
		phid->motorEngagedState[ch->index] = getBridgePacketInt32(bp, 0);
		return (_sendpacket(bp->iop, phid, ch->index));

	case BP_SETTARGETPOSITION:
		phid->targetPosition[ch->index] = getBridgePacketDouble(bp, 0);

		if (phid->motorEngagedState[ch->index] == PFALSE)
			return (EPHIDGET_OK);

		res = _sendpacket(bp->iop, phid, ch->index);
		if (res != EPHIDGET_OK)
			return (res);
		// For devices which don't return position, fire position change event manually.
		if (phid->fullStateEcho == PFALSE) {
			if (phid->position[ch->index] == PUNK_BOOL || phid->position[ch->index] != getBridgePacketDouble(bp, 0)) {
				phid->position[ch->index] = getBridgePacketDouble(bp, 0);
				bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%g", phid->position[ch->index]);
			}
		}
		return (EPHIDGET_OK);

	case BP_SETMINPULSEWIDTH:
		return (EPHIDGET_OK);
	case BP_SETMAXPULSEWIDTH:
		return (EPHIDGET_OK);

	case BP_OPENRESET:
	case BP_CLOSERESET:
		phid->motorEngagedState[ch->index] = PFALSE;
		phid->targetPosition[ch->index] = PUNK_DBL;
		return (_sendpacket(bp->iop, phid, ch->index));
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

//makePacket - constructs a packet using current device state
static PhidgetReturnCode
_sendpacket(mosiop_t iop, PhidgetServoDeviceHandle phid, int Index) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	int i = 0;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1000_OLD1:
	case PHIDUID_1000_OLD2:
		buffer[0] = 0;
		if (phid->targetPosition[0] == PUNK_DBL || phid->motorEngagedState[0] == PFALSE)
			buffer[1] = 0;
		else
			buffer[1] = (uint8_t)round(phid->targetPosition[Index] / 10.0);
		break;

	case PHIDUID_1000_NO_ECHO:
	case PHIDUID_1000:
	{
		int microsecondPulse = 0;

		if (phid->targetPosition[0] == PUNK_DBL || phid->motorEngagedState[0] == PFALSE)
			microsecondPulse = 0;
		else
			microsecondPulse = round(phid->targetPosition[0]);

		buffer[0] = (uint8_t)(microsecondPulse & 0xFF);
		buffer[1] = (uint8_t)(microsecondPulse >> 8);
	}
	break;

	case PHIDUID_1001_OLD1:
	case PHIDUID_1001_OLD2:
		if (Index == 0) buffer[0] = 2;
		if (Index == 1) buffer[0] = 3;
		if (Index == 2) buffer[0] = 0;
		if (Index == 3) buffer[0] = 1;
		if (phid->targetPosition[Index] == PUNK_DBL || phid->motorEngagedState[Index] == PFALSE)
			buffer[1] = 0;
		else
			buffer[1] = (uint8_t)round(phid->targetPosition[Index] / 10.0);
		break;

	case PHIDUID_1001_NO_ECHO:
	case PHIDUID_1001:
	{
		int microsecondPulse[4] = { 0 };

		for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
			if (phid->targetPosition[i] == PUNK_DBL || phid->motorEngagedState[i] == PFALSE) {
				microsecondPulse[i] = 0;
			} else {
				microsecondPulse[i] = round(phid->targetPosition[i]);
			}
		}

		buffer[0] = (uint8_t)(microsecondPulse[0] & 0xFF);
		buffer[1] = (uint8_t)((microsecondPulse[0] >> 8) & 0x0F);

		buffer[2] = (uint8_t)(microsecondPulse[1] & 0xFF);
		buffer[1] |= (uint8_t)((microsecondPulse[1] >> 4) & 0xF0);

		buffer[3] = (uint8_t)(microsecondPulse[2] & 0xFF);
		buffer[4] = (uint8_t)((microsecondPulse[2] >> 8) & 0x0F);

		buffer[5] = (uint8_t)(microsecondPulse[3] & 0xFF);
		buffer[4] |= (uint8_t)((microsecondPulse[3] >> 4) & 0xF0);
	}
	break;

	default:
		MOS_PANIC("Unexpected device");
	}

	return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
}

static void CCONV
PhidgetServoDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetServoDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetServoDevice_create(PhidgetServoDeviceHandle *phidp) {
	DEVICECREATE_BODY(ServoDevice, PHIDCLASS_SERVO);
	return (EPHIDGET_OK);
}
