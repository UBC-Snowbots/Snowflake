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
#include "device/accelerometerdevice.h"

// === Internal Functions === //

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetAccelerometerDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetAccelerometerDeviceHandle phid = (PhidgetAccelerometerDeviceHandle)device;
	int i;

	assert(phid);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1008:
		phid->_maxAcceleration = 2.0;
		phid->_minAcceleration = -2.0;
		phid->_interruptRate = 39; //NOTE: Interrupt rate is 16ms but ~41% are NAKed
		break;
	case PHIDUID_1053:
		phid->_maxAcceleration = 5.0;
		phid->_minAcceleration = -5.0;
		phid->_interruptRate = 16;
		break;
	case PHIDUID_1059:
		phid->_maxAcceleration = 3.0;
		phid->_minAcceleration = -3.0;
		phid->_interruptRate = 16;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	//initialize triggers, set data arrays to unknown
	for (i = 0; i < phid->devChannelCnts.numAxis; i++) {
		phid->_accelerationLastTrigger[i] = PUNK_DBL;
		phid->acceleration[0][i] = PUNK_DBL;
	}
	phid->accelerationChangeTrigger[0] = 0.001;
	phid->timestamp[0] = 0;

	//issue one read
	waitForReads((PhidgetDeviceHandle)phid, 1, 100);

	return (EPHIDGET_OK);
}

#define CALLTM(phid)	((phid)->_interruptRate * (phid)->_callcnt)

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetAccelerometerDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetAccelerometerDeviceHandle phid = (PhidgetAccelerometerDeviceHandle)device;
	double axis[ACCEL_MAXAXES] = { 0 };
	PhidgetChannelHandle channel;
	int fireSaturation;
	int fireEvent;
	mostime_t tm;
	int data;
	int i;

	assert(phid);
	assert(buffer);

	//Parse device packets - store data locally
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1008:
		data = (int16_t)((uint16_t)buffer[0] + ((uint16_t)buffer[1] << 8));
		axis[0] = round_double((((double)data - 16384) / 2000), 4);
		data = (int16_t)((uint16_t)buffer[2] + ((uint16_t)buffer[3] << 8));
		axis[1] = round_double((((double)data - 16384) / 2000), 4);
		break;
	case PHIDUID_1053:
		data = ((uint16_t)buffer[0] + ((uint16_t)buffer[1] << 8));
		axis[0] = round_double((((double)(data - 32768)) / 4000), 5);
		data = ((uint16_t)buffer[2] + ((uint16_t)buffer[3] << 8));
		axis[1] = round_double((((double)(data - 32768)) / 4000), 5);
		break;
	case PHIDUID_1059:
		data = ((uint16_t)buffer[0] + ((uint16_t)buffer[1] << 8));
		axis[0] = round_double((((double)(data - 32768)) / 6553.6), 5);
		data = ((uint16_t)buffer[2] + ((uint16_t)buffer[3] << 8));
		axis[1] = round_double((((double)(data - 32768)) / 6553.6), 5);
		data = ((uint16_t)buffer[4] + ((uint16_t)buffer[5] << 8));
		axis[2] = round_double((((double)(data - 32768)) / 6553.6), 5);
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	phid->timestamp[0] += phid->_interruptRate;
	phid->_callcnt++;

	//Make sure values are within defined range, and store to structure
	for (i = 0; i < phid->devChannelCnts.numAxis; i++) {
		if (axis[i] > phid->_maxAcceleration)
			axis[i] = PUNK_DBL;
		if (axis[i] < phid->_minAcceleration)
			axis[i] = PUNK_DBL;

		phid->acceleration[0][i] = axis[i];
	}

	tm = CALLTM(phid);

	if (tm < phid->_accelDeadline[0])
		return (EPHIDGET_OK);

	//Now, notify any channels
	channel = getChannel(phid, 0);
	if (channel == NULL)
		return (EPHIDGET_OK);

	fireSaturation = PFALSE;
	fireEvent = PFALSE;
	for (i = 0; i < phid->devChannelCnts.numAxis; i++) {
		phid->acceleration[0][i] = axis[i];
		if (phid->acceleration[0][i] == PUNK_DBL) {
			fireSaturation = PTRUE;
		} else if (fabs(phid->acceleration[0][i] - phid->_accelerationLastTrigger[i]) >= phid->accelerationChangeTrigger[0]
		  || phid->_accelerationLastTrigger[i] == PUNK_DBL) {
			fireEvent = PTRUE;
		}
	}

	if (fireSaturation) {
		SEND_ERROR_EVENT(channel, EEPHIDGET_SATURATION, "Saturation Detected.");
	} else if (fireEvent) {
		bridgeSendToChannel(channel, BP_ACCELERATIONCHANGE, "%3G%g", phid->acceleration[0], phid->timestamp[0]);
		for (i = 0; i < phid->devChannelCnts.numAxis; i++)
			phid->_accelerationLastTrigger[i] = phid->acceleration[0][i];
		phid->_accelDeadline[0] = tm + phid->_accelDataInterval[0];
	}

	PhidgetRelease(&channel);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetAccelerometerDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetAccelerometerDeviceHandle phid = (PhidgetAccelerometerDeviceHandle)ch->parent;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_ACCELEROMETER);
	assert(ch->class == PHIDCHCLASS_ACCELEROMETER);
	assert(ch->index == 0);

	switch (bp->vpkt) {
	case BP_SETDATAINTERVAL:
		phid->_accelDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
		phid->_accelDeadline[ch->index] = 0;
		return (EPHIDGET_OK);
	case BP_SETCHANGETRIGGER:
		phid->accelerationChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
		return (EPHIDGET_OK);
	case BP_OPENRESET:
	case BP_CLOSERESET:
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

static void CCONV
PhidgetAccelerometerDevice_free(PhidgetDeviceHandle *phid) {
	mos_free(*phid, sizeof(struct _PhidgetAccelerometerDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetAccelerometerDevice_create(PhidgetAccelerometerDeviceHandle *phidp) {
	DEVICECREATE_BODY(AccelerometerDevice, PHIDCLASS_ACCELEROMETER);
	return (EPHIDGET_OK);
}
