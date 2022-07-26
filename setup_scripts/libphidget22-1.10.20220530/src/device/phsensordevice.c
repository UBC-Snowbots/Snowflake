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
#include "device/phsensordevice.h"

static double calculate_ph(double Vad, double temperature);

static PhidgetReturnCode CCONV
PhidgetPHSensorDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetPHSensorDeviceHandle phid = (PhidgetPHSensorDeviceHandle)device;
	int i;

	assert(phid);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1058:
		phid->potentialMax = round_double(((2.5) / 4.745), 6);
		phid->potentialMin = round_double(((2.5 - (65535.0 / 13104.0)) / 4.745), 6);
		phid->phMax = 14;
		phid->phMin = 0;
		phid->_interruptRate = 80;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	for (i = 0; i < phid->devChannelCnts.numVoltageInputs; i++) {
		phid->voltage[0] = PUNK_DBL;
		phid->lastVoltageTrigger[0] = PUNK_DBL;
		phid->voltageChangeTrigger[0] = 0.0001;
	}

	for (i = 0; i < phid->devChannelCnts.numPHInputs; i++) {
		phid->PH[0] = PUNK_DBL;
		phid->lastPHTrigger[0] = PUNK_DBL;
		phid->PHChangeTrigger[0] = 0.0001;
		phid->correctionTemperature[0] = 25;
	}

	// issue one read
	waitForReads((PhidgetDeviceHandle)phid, 1, 100);

	return (EPHIDGET_OK);
}

#define CALLTM(phid) ((phid)->_interruptRate * (phid)->_callcnt)

static PhidgetReturnCode CCONV
PhidgetPHSensorDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetPHSensorDeviceHandle phid;
	PhidgetChannelHandle ch;
	double voltage, Vad, E, PH;
	mostime_t tm;
	int i;

	assert(device);
	assert(buffer);

	phid = (PhidgetPHSensorDeviceHandle)device;
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1058:
		Vad = ((double)((uint16_t)buffer[0] + ((uint16_t)buffer[1] << 8))) / 13104.0;
		PH = round_double(calculate_ph(Vad, phid->correctionTemperature[0]), 4);
		E = (2.5 - Vad) / 4.745;
		voltage = round_double(E, 6);
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	phid->_callcnt++;

	if (PH < phid->phMin || PH > phid->phMax)
		phid->PH[0] = PUNK_DBL;
	else
		phid->PH[0] = PH;

	if (voltage < phid->potentialMin || voltage > phid->potentialMax)
		phid->voltage[0] = PUNK_DBL;
	else
		phid->voltage[0] = voltage;

	tm = CALLTM(phid);

	/* Voltage */
	for (i = 0; i < phid->devChannelCnts.numVoltageInputs; i++) {
		if (phid->voltage[i] == PUNK_DBL)
			continue;

		if (tm < phid->voltageDeadline[i])
			continue;

		if ((ch = getChannel(phid, i)) == NULL)
			continue;

		if (phid->lastVoltageTrigger[i] == PUNK_DBL ||
			fabs(phid->voltage[i] - phid->lastVoltageTrigger[i]) >= phid->voltageChangeTrigger[i]) {
			bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", phid->voltage[i]);
			phid->lastVoltageTrigger[i] = phid->voltage[i];
			phid->voltageDeadline[i] = tm + (phid->voltageDataInterval[i]);
		}
		PhidgetRelease(&ch);
	}

	/* PH */
	for (i = 0; i < phid->devChannelCnts.numPHInputs; i++) {
		if (phid->PH[i] == PUNK_DBL)
			continue;

		if (tm < phid->phDeadline[i])
			continue;

		if ((ch = getChannel(phid, i + phid->devChannelCnts.numVoltageInputs)) == NULL)
			continue;

		if (phid->lastPHTrigger[i] == PUNK_DBL ||
			fabs(phid->PH[i] - phid->lastPHTrigger[i]) >= phid->PHChangeTrigger[i]) {
			bridgeSendToChannel(ch, BP_PHCHANGE, "%g", phid->PH[i]);
			phid->lastPHTrigger[i] = phid->PH[i];
			phid->phDeadline[i] = tm + phid->phDataInterval[i];
		}
		PhidgetRelease(&ch);
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetPHSensorDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetPHSensorDeviceHandle phid;

	phid = (PhidgetPHSensorDeviceHandle)ch->parent;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_PHSENSOR);

	switch (ch->class) {
	case PHIDCHCLASS_PHSENSOR:
		assert(ch->index < phid->devChannelCnts.numPHInputs);

		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			phid->phDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
			phid->phDeadline[ch->index] = 0;
			return (EPHIDGET_OK);
		case BP_SETCHANGETRIGGER:
			phid->PHChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_SETCORRECTIONTEMPERATURE:
			phid->correctionTemperature[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_OPENRESET:
		case BP_CLOSERESET:
			phid->correctionTemperature[ch->index] = 25;
			return (EPHIDGET_OK);
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_VOLTAGEINPUT:
		assert(ch->index < phid->devChannelCnts.numVoltageInputs);

		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			phid->voltageDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
			phid->voltageDeadline[ch->index] = 0;
			return (EPHIDGET_OK);
			break;
		case BP_SETCHANGETRIGGER:
			phid->voltageChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Unexpected channel class");
	}
}

static void CCONV
PhidgetPHSensorDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetPHSensorDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetPHSensorDevice_create(PhidgetPHSensorDeviceHandle *phidp) {
	DEVICECREATE_BODY(PHSensorDevice, PHIDCLASS_PHSENSOR);
	return (EPHIDGET_OK);
}


static double calculate_ph(double Vad, double temperature) {
	double E, E0, C, T;
	const double R=8.31441, N=1, F=96484.6;
	T=(273.15) + temperature;
	C = 2.3 * ((R*T) / (N*F));
	E0 = 7 * C;
	E = (2.5 - Vad) / 4.745;
	return ((E0 - E) / C);
}