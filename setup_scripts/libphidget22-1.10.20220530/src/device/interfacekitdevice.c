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
#include "device/interfacekitdevice.h"

// === Internal Functions === //
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetInterfaceKitDeviceHandle phid, int Index);
static PhidgetReturnCode adjustIfkitDataIntervalsAsNeeded(PhidgetInterfaceKitDeviceHandle phid, PhidgetChannelHandle channelIn, int closing);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetInterfaceKitDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetInterfaceKitDeviceHandle phid = (PhidgetInterfaceKitDeviceHandle)device;
	int j = 0;
	uint8_t buffer[8] = { 0 };
	PhidgetReturnCode result;
#if (PHIDUID_1014_3_USB_SUPPORTED || PHIDUID_1017_2_USB_SUPPORTED)
	size_t len;
#endif

	assert(phid);

	//init device specific attrs
	phid->awdc_enabled = PFALSE;
	phid->maxDataPerPacket = phid->devChannelCnts.numSensors;
	phid->ratiometricSwitching = 0;
	phid->lastPacketCount = -1;
	phid->dataSinceAttach = 0;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1013:
	case PHIDUID_1202_IFKIT:
		phid->awdc_enabled = PTRUE;
		phid->referenceVoltage = 5.0;
	case PHIDUID_IFKIT488:
	case PHIDUID_1013_NO_ECHO:
	case PHIDUID_1202_IFKIT_NO_ECHO:
		phid->interruptRate = 16;
		phid->dataRateMin = IFKIT_MIN_DATA_RATE / phid->interruptRate*phid->interruptRate;
		phid->dataRateMax = 16; //actual data rate
		phid->referenceVoltage = 5.0;
		break;

	case PHIDUID_1018:
	case PHIDUID_1202_IFKIT_FAST:
		phid->interruptRate = 8;
		phid->dataRateMin = IFKIT_MIN_DATA_RATE / phid->interruptRate*phid->interruptRate;
		phid->dataRateMax = 1; //actual data rate
		phid->awdc_enabled = PTRUE; //To help with pre v902 bug (see firmware)
		phid->maxDataPerPacket = 36;
		phid->referenceVoltage = 5.0;
		break;
	case PHIDUID_1018_3:
		phid->interruptRate = 8;
		phid->dataRateMin = IFKIT_MIN_DATA_RATE / phid->interruptRate*phid->interruptRate;
		phid->dataRateMax = 1; //actual data rate
		phid->awdc_enabled = PTRUE; //To help with pre v902 bug (see firmware)
		phid->maxDataPerPacket = 36;
		phid->referenceVoltage = 5.456;
		break;
	case PHIDUID_1011:
		phid->interruptRate = 8;
		phid->dataRateMin = IFKIT_MIN_DATA_RATE / phid->interruptRate*phid->interruptRate;
		phid->dataRateMax = 1; //actual data rate
		phid->maxDataPerPacket = 16;
		phid->referenceVoltage = 5.0;
		break;

	case PHIDUID_1015:
		//NOTE: real rate is 32ms, but the firmware NAKs at a certain rate...
		phid->referenceVoltage = 5.0;
		switch (phid->phid.deviceInfo.version) {
		case 500:
			phid->interruptRate = 51;
			break;
		case 101:
			phid->interruptRate = 59;
			break;
		case 102:
			phid->interruptRate = 52;
			break;
		}
		phid->dataRateMin = IFKIT_MIN_DATA_RATE / phid->interruptRate*phid->interruptRate;
		phid->dataRateMax = phid->interruptRate; //actual data rate
		break;

	case PHIDUID_1016:
		//NOTE: real rate is 32ms, but the firmware NAKs at a certain rate...
		phid->referenceVoltage = 5.0;
		switch (phid->phid.deviceInfo.version) {
		case 510:
			phid->interruptRate = 59;
			break;
		case 101:
			phid->interruptRate = 50;
			break;
		case 102:
			phid->interruptRate = 45;
			break;
		}
		phid->dataRateMin = IFKIT_MIN_DATA_RATE / phid->interruptRate*phid->interruptRate;
		phid->dataRateMax = phid->interruptRate; //actual data rate
		break;

	case PHIDUID_1012_BITBUG:
	case PHIDUID_1012:
	case PHIDUID_1014:
#if PHIDUID_1014_3_USB_SUPPORTED
	case PHIDUID_1014_3_USB:
#endif
	case PHIDUID_1017:
#if PHIDUID_1017_2_USB_SUPPORTED
	case PHIDUID_1017_2_USB:
#endif
	case PHIDUID_1012_NO_ECHO:
	case PHIDUID_1014_NO_ECHO:
#if PHIDUID_USBSWITCH_SUPPORTED
	case PHIDUID_USBSWITCH:
#endif
		phid->referenceVoltage = 5.0;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	phid->dataRateMaxUsing = phid->dataRateMax;

	//initialize triggers, set data arrays to unknown
	phid->ratiometric = PUNK_BOOL;
	phid->ratiometricEcho = PUNK_BOOL;
	for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
		phid->inputState[j] = PUNK_BOOL;
	}
	for (j = 0; j < phid->devChannelCnts.numSensors; j++) {
		phid->eventMode[j] = EVENTMODE_CHANGETRIGGER;
		phid->dataInterval[j] = phid->interruptRate;
		phid->gotSensorData[j] = PFALSE;

		phid->voltageRatio[j] = PUNK_DBL;
		phid->sensorRatioLastValue[j] = PUNK_DBL;
		phid->sensorRatioAccumulator[j] = PUNK_DBL;
		phid->sensorRatioAccumulatorCount[j] = 0;
		phid->sensorRatioInterruptCount[j] = 0;
		phid->voltageRatioChangeTrigger[j] = 0.001;

		switch (phid->phid.deviceInfo.UDD->uid) {
			//Only the new devices go into dataRate event mode by default
		case PHIDUID_1018:
		case PHIDUID_1018_3:
		case PHIDUID_1202_IFKIT_FAST:
		case PHIDUID_1011:
			phid->eventMode[j] = EVENTMODE_DATARATE;
			break;

		default:
			break;
		}
	}
	for (j = 0; j < phid->devChannelCnts.numVoltageInputs; j++) {
		phid->voltage[j] = PUNK_DBL;
		phid->sensorVoltageLastValue[j] = PUNK_DBL;
		phid->sensorVoltageAccumulator[j] = PUNK_DBL;
		phid->sensorVoltageAccumulatorCount[j] = 0;
		phid->sensorVoltageInterruptCount[j] = 0;
		phid->voltageChangeTrigger[j] = 0.005;
	}
	for (j = 0; j < phid->devChannelCnts.numOutputs; j++) {
		phid->outputState[j] = PUNK_BOOL;
		phid->failsafeState[j] = PFALSE;
	}
	for (j = 0; j < phid->devChannelCnts.numCapTouches; j++) {
		phid->touchValue[j] = PUNK_DBL;
		phid->isTouched[j] = PUNK_BOOL;
		phid->touchValueChangeTrigger[j] = 0.005;
		phid->touchInputLastValue[j] = PUNK_DBL;
	}

	//send out any initial pre-read packets
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1016:
		//performs initial calibration
		buffer[0] = 0x01;

		result = PhidgetDevice_sendpacket(NULL, (PhidgetDeviceHandle)phid, buffer, 1);
		if (result)
			return result;
		break;

	case PHIDUID_1015:
		//performs initial calibration
		buffer[0] = 0x01;

		result = PhidgetDevice_sendpacket(NULL, (PhidgetDeviceHandle)phid, buffer, 1);
		if (result)
			return result;
		mos_usleep(100000);
		buffer[0] = 0x02;

		result = PhidgetDevice_sendpacket(NULL, (PhidgetDeviceHandle)phid, buffer, 1);
		if (result)
			return result;
		break;

	case PHIDUID_IFKIT488:
		phid->ratiometric = PTRUE;
		loginfo("Sending workaround startup packet");

		result = PhidgetDevice_sendpacket(NULL, (PhidgetDeviceHandle)phid, buffer, 1);
		if (result)
			return result;
		break;

	default:
		break;
	}

	//read in device state - don't issue reads on devices that block
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1012_BITBUG:
	case PHIDUID_1012:
	case PHIDUID_1014:
	case PHIDUID_1017:
#if PHIDUID_USBSWITCH_SUPPORTED
	case PHIDUID_USBSWITCH:
#endif
		waitForReads((PhidgetDeviceHandle)phid, 1, 100);
		break;

#if (PHIDUID_1014_3_USB_SUPPORTED || PHIDUID_1017_2_USB_SUPPORTED)
#if PHIDUID_1014_3_USB_SUPPORTED
	case PHIDUID_1014_3_USB:
#endif
#if PHIDUID_1017_2_USB_SUPPORTED
	case PHIDUID_1017_2_USB:
#endif
		len = 1;
		result = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_READ, 0, 0, buffer, &len, 100);
		if (result != EPHIDGET_OK)
			return result;
		for (j = 0; j < phid->devChannelCnts.numOutputs; j++) {
			phid->outputState[j] = (buffer[0] & (1<<j)) != 0;
		}
		break;
#endif /* (PHIDUID_1014_3_USB_SUPPORTED || PHIDUID_1017_2_USB_SUPPORTED) */

	case PHIDUID_1013_NO_ECHO:
	case PHIDUID_1013:
	case PHIDUID_1018:
	case PHIDUID_1018_3:
	case PHIDUID_1202_IFKIT_NO_ECHO:
	case PHIDUID_1202_IFKIT:
	case PHIDUID_1202_IFKIT_FAST:
	case PHIDUID_1011:
	case PHIDUID_IFKIT488:
		waitForReads((PhidgetDeviceHandle)phid, 4, 500);
		break;

	case PHIDUID_1015:
	case PHIDUID_1016:
		waitForReads((PhidgetDeviceHandle)phid, 2, 100);
		break;

	//these only send data on a change - so we have no way of knowing initial state, or even state at all, until an input changes state
	case PHIDUID_1012_NO_ECHO:
	case PHIDUID_1014_NO_ECHO:
	default:
		break;
	}

	//initialize outputs
	for (j = 0; j < phid->devChannelCnts.numOutputs; j++) {
		phid->_outputStateSet[j] = phid->outputState[j];
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
doRatiometricSwitching(PhidgetInterfaceKitDeviceHandle phid) {
	PhidgetReturnCode result = EPHIDGET_OK;
	int i;
	int voltageInputCnt = 0, voltageRatioCnt = 0;
	assert(phid);

	for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
		//Don't set until we have recieved data on all channels
		if (!phid->gotSensorData[i]) {
			return (EPHIDGET_OK);
		}

		if (phid->phid.channel[i + phid->devChannelCnts.numVoltageInputs])
			voltageRatioCnt++;
	}

	for (i = 0; i < phid->devChannelCnts.numVoltageInputs; i++)
		if (phid->phid.channel[i])
			voltageInputCnt++;

	if (!phid->ratiometricSwitching) {
		int ratiometricWanted = PTRUE; //default to true
		if (voltageInputCnt > 0 && voltageRatioCnt > 0) {
			ratiometricWanted = phid->ratiometric ^ 0x01;
		} else {
			if (voltageInputCnt > 0)
				ratiometricWanted = PFALSE;
			if (voltageRatioCnt > 0)
				ratiometricWanted = PTRUE;
		}

		if (phid->ratiometric != ratiometricWanted) {
			switch (phid->phid.deviceInfo.UDD->uid) {
			case PHIDUID_1013_NO_ECHO:
			case PHIDUID_1013:
			case PHIDUID_1202_IFKIT_NO_ECHO:
			case PHIDUID_1202_IFKIT:
				phid->ratiometricSwitching = 4;
				break;

			case PHIDUID_1011:
			case PHIDUID_1018:
			case PHIDUID_1018_3:
			case PHIDUID_1202_IFKIT_FAST:
				phid->ratiometricSwitching = 2;
				break;
			default:
				break;
			}
			phid->ratiometric = ratiometricWanted;

			result = _sendpacket(NULL, phid, 0);
			if (result != EPHIDGET_OK)
				return result;

			for (i = 0; i < phid->devChannelCnts.numSensors; i++)
				phid->gotSensorData[i] = PFALSE;
		}
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode
adjustIfkitDataIntervalsAsNeeded(PhidgetInterfaceKitDeviceHandle phid, PhidgetChannelHandle channelIn, int closing) {
	PhidgetReturnCode result = EPHIDGET_OK;
	PhidgetChannelHandle channel;
	int openVoltageInputCnt = 0, openVoltageRatioCnt = 0;
	uint32_t dataRateMax;
	int rateChanged;
	int i;

	assert(phid);

	for (i = 0; i < phid->devChannelCnts.numVoltageInputs; i++) {
		if (phid->phid.channel[i]) {
			// Don't count closing channel
			if (phid->phid.channel[i] == channelIn && closing)
				continue;
			openVoltageInputCnt++;
		}
	}

	for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
		if (phid->phid.channel[i + phid->devChannelCnts.numVoltageInputs]) {
			// Don't count closing channel
			if (phid->phid.channel[i + phid->devChannelCnts.numVoltageInputs] == channelIn && closing)
				continue;
			openVoltageRatioCnt++;
		}
	}

	//When we are switching ratiometric, the best we can do is 32ms.
	if (openVoltageRatioCnt > 0 && openVoltageInputCnt > 0)
		dataRateMax = 32;
	else
		dataRateMax = phid->dataRateMax;

	for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
		rateChanged = PFALSE;

		if (phid->dataInterval[i] < dataRateMax) {
			rateChanged = PTRUE;
			phid->dataInterval[i] = dataRateMax;
		}

		if ((channel = getChannel(phid, i)) != NULL) {
			bridgeSendToChannel((PhidgetChannelHandle)channel, BP_MINDATAINTERVALCHANGE, "%u", dataRateMax);
			if (rateChanged)
				bridgeSendToChannel((PhidgetChannelHandle)channel, BP_DATAINTERVALCHANGE, "%u", phid->dataInterval[i]);
			PhidgetRelease(&channel);
		}

		if ((channel = getChannel(phid, i + phid->devChannelCnts.numVoltageInputs)) != NULL) {
			bridgeSendToChannel((PhidgetChannelHandle)channel, BP_MINDATAINTERVALCHANGE, "%u", dataRateMax);
			if (rateChanged)
				bridgeSendToChannel((PhidgetChannelHandle)channel, BP_DATAINTERVALCHANGE, "%u", phid->dataInterval[i]);
			PhidgetRelease(&channel);
		}

		if (rateChanged)
			phid->dataInterval[i] = dataRateMax;
	}

	phid->dataRateMaxUsing = dataRateMax;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1011:
	case PHIDUID_1018:
	case PHIDUID_1018_3:
	case PHIDUID_1202_IFKIT_FAST:
		result = _sendpacket(NULL, phid, 0);
		if (result != EPHIDGET_OK)
			return result;
		break;
	default:
		break;
	}

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetInterfaceKitDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetInterfaceKitDeviceHandle phid = (PhidgetInterfaceKitDeviceHandle)device;
	PhidgetChannelHandle channel;
	int j = 0, i = 0, k = 0;
#if (PHIDUID_1014_3_USB_SUPPORTED || PHIDUID_1017_2_USB_SUPPORTED)
	int pkt;
#endif

	uint8_t outputState[IFKIT_MAXOUTPUTS];
	uint8_t inputState[IFKIT_MAXINPUTS];
	uint8_t lastInputState[IFKIT_MAXINPUTS];
	uint8_t lastTouchState[IFKIT_MAXCAPTOUCHES] = { 0 };
	int sensorRawValue[IFKIT_MAXSENSORS][IFKIT_MAX_DATA_PER_PACKET];
	int sensorDataCount[IFKIT_MAXSENSORS] = { 0 };
	uint8_t ratiometricEcho = PUNK_BOOL;

	assert(phid);
	assert(buffer);

	if (phid->dataSinceAttach < 100)
		phid->dataSinceAttach++;

	for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
		inputState[j] = PUNK_BOOL;
		lastInputState[j] = phid->inputState[j];
	}
	for (j = 0; j < phid->devChannelCnts.numSensors; j++) {
		for (i = 0; i < IFKIT_MAX_DATA_PER_PACKET; i++) {
			sensorRawValue[j][i] = PUNK_INT32;
		}
		sensorDataCount[j] = 0;
	}
	for (j = 0; j < phid->devChannelCnts.numOutputs; j++) {
		outputState[j] = PUNK_BOOL;
	}
	for (j = 0; j < phid->devChannelCnts.numCapTouches; j++) {
		lastTouchState[j] = phid->isTouched[j];
	}

	//Parse device packets - store data locally
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_IFKIT488:
		//Sensors
		sensorRawValue[0][0] = ((uint8_t)buffer[3] + ((uint8_t)buffer[2] & 0x0f) * 256);
		sensorDataCount[0]++;
		sensorRawValue[1][0] = ((uint8_t)buffer[1] + ((uint8_t)buffer[2] & 0xf0) * 16);
		sensorDataCount[1]++;
		sensorRawValue[2][0] = ((uint8_t)buffer[6] + ((uint8_t)buffer[5] & 0x0f) * 256);
		sensorDataCount[2]++;
		sensorRawValue[3][0] = ((uint8_t)buffer[4] + ((uint8_t)buffer[5] & 0xf0) * 16);
		sensorDataCount[3]++;

		for (j = 0; j < phid->devChannelCnts.numSensors; j++)
			phid->sensorRatioInterruptCount[j]++;

		//Inputs
		for (i = 0, j = 0x80; i < 4; i++, j >>= 1) {
			if ((buffer[0] & j) != 0)
				inputState[i] = PFALSE;
			else
				inputState[i] = PTRUE;
		}
		for (i = 4, j = 0x01; i < 8; i++, j <<= 1) {
			if ((buffer[0] & j) != 0)
				inputState[i] = PFALSE;
			else
				inputState[i] = PTRUE;
		}

		break;

	case PHIDUID_1012:
		//Outputs
		for (k = 0; k < 2; k++) {
			for (i = 0, j = 0x01; i < 8; i++, j <<= 1) {
				if ((buffer[k + 2] & j) == 0)
					outputState[i + k * 8] = PFALSE;
				else
					outputState[i + k * 8] = PTRUE;
			}
		}
		//NOTE: fall through
	case PHIDUID_1012_NO_ECHO:
		//Inputs
		for (k = 0; k < 2; k++) {
			for (i = 0, j = 0x01; i < 8; i++, j <<= 1) {
				if ((buffer[k] & j) == 0)
					inputState[i + k * 8] = PFALSE;
				else
					inputState[i + k * 8] = PTRUE;
			}
		}
		break;

	case PHIDUID_1012_BITBUG:
		//Inputs
		for (k = 0; k < 2; k++) {
			for (i = 0, j = 0x01; i < 8; i++, j <<= 1) {
				if ((buffer[k] & j) == 0)
					inputState[i + k * 8] = PFALSE;
				else
					inputState[i + k * 8] = PTRUE;
			}
		}

		//Outputs
		for (i = 0, j = 0x01; i < 8; i++, j <<= 1) {
			if ((buffer[2] & j) == 0)
				outputState[i] = PFALSE;
			else
				outputState[i] = PTRUE;
		}
		for (i = 7, j = 0x01; i >= 0; i--, j <<= 1) {
			if ((buffer[3] & j) == 0)
				outputState[i + 8] = PFALSE;
			else
				outputState[i + 8] = PTRUE;
		}

		break;

	case PHIDUID_1013:
	case PHIDUID_1202_IFKIT:
		//Outputs
		if (!(buffer[0] & 0x01)) {
			for (i = 0, j = 0x10; i < 4; i++, j <<= 1) {
				if ((buffer[0] & j) == 0)
					outputState[i] = PFALSE;
				else
					outputState[i] = PTRUE;
			}
		} else {
			for (i = 4, j = 0x10; i < 8; i++, j <<= 1) {
				if ((buffer[0] & j) == 0)
					outputState[i] = PFALSE;
				else
					outputState[i] = PTRUE;
			}
		}
		//NOTE: fall through
	case PHIDUID_1013_NO_ECHO:
	case PHIDUID_1202_IFKIT_NO_ECHO:
		//Inputs
		for (i = 0, j = 0x01; i < phid->devChannelCnts.numInputs; i++, j <<= 1) {
			if (buffer[1] & j)
				inputState[i] = PFALSE;
			else
				inputState[i] = PTRUE;
		}

		//there are two types of packets
		if (!(buffer[0] & 0x01)) {
			//Sensors
			sensorRawValue[0][0] = ((uint8_t)buffer[2] + ((uint8_t)buffer[3] & 0x0f) * 256);
			sensorDataCount[0]++;
			sensorRawValue[1][0] = ((uint8_t)buffer[4] + ((uint8_t)buffer[3] & 0xf0) * 16);
			sensorDataCount[1]++;
			sensorRawValue[2][0] = ((uint8_t)buffer[5] + ((uint8_t)buffer[6] & 0x0f) * 256);
			sensorDataCount[2]++;
			sensorRawValue[3][0] = ((uint8_t)buffer[7] + ((uint8_t)buffer[6] & 0xf0) * 16);
			sensorDataCount[3]++;

			for (j = 0; j < 4; j++) {
				phid->sensorRatioInterruptCount[j]++;
				phid->sensorVoltageInterruptCount[j]++;
			}
		} else {
			//Sensors
			sensorRawValue[4][0] = ((uint8_t)buffer[2] + ((uint8_t)buffer[3] & 0x0f) * 256);
			sensorDataCount[4]++;
			sensorRawValue[5][0] = ((uint8_t)buffer[4] + ((uint8_t)buffer[3] & 0xf0) * 16);
			sensorDataCount[5]++;
			sensorRawValue[6][0] = ((uint8_t)buffer[5] + ((uint8_t)buffer[6] & 0x0f) * 256);
			sensorDataCount[6]++;
			sensorRawValue[7][0] = ((uint8_t)buffer[7] + ((uint8_t)buffer[6] & 0xf0) * 16);
			sensorDataCount[7]++;

			for (j = 4; j < 8; j++) {
				phid->sensorRatioInterruptCount[j]++;
				phid->sensorVoltageInterruptCount[j]++;
			}
		}
		break;

	case PHIDUID_1011:
	case PHIDUID_1018:
	case PHIDUID_1018_3:
	case PHIDUID_1202_IFKIT_FAST:
	{
		int overrunBits, overrunPtr, countPtr, packetCount, channelCount[IFKIT_MAXSENSORS], overrunCount[IFKIT_MAXSENSORS];
		uint8_t overcurrentFlag = 0;
		int datacount = 0;
		int flip, bufindx;

		//counters, etc.
		packetCount = (buffer[0] >> 6) & 0x03;
		overcurrentFlag = (buffer[0] >> 5) & 0x01;
		ratiometricEcho = (buffer[0] >> 4) & 0x01;
		overrunBits = buffer[0] & 0x0f;

		//Inputs
		for (i = 0, j = 0x01; i < phid->devChannelCnts.numInputs; i++, j <<= 1) {
			if (buffer[1] & j)
				inputState[i] = PFALSE;
			else
				inputState[i] = PTRUE;
		}

		//Outputs
		for (i = 0, j = 0x01; i < phid->devChannelCnts.numOutputs; i++, j <<= 1) {
			if ((buffer[2] & j) == 0)
				outputState[i] = PFALSE;
			else
				outputState[i] = PTRUE;
		}

		//Sensors
		//Overruns
		overrunPtr = 3;
		for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
			overrunCount[i] = 0;
		}
		if (overrunBits & 0x01) {
			overrunCount[0] = buffer[overrunPtr] >> 4;
			overrunCount[1] = buffer[overrunPtr] & 0x0f;
			overrunPtr++;
		}
		if (overrunBits & 0x02) {
			overrunCount[2] = buffer[overrunPtr] >> 4;
			overrunCount[3] = buffer[overrunPtr] & 0x0f;
			overrunPtr++;
		}
		if (overrunBits & 0x04) {
			overrunCount[4] = buffer[overrunPtr] >> 4;
			overrunCount[5] = buffer[overrunPtr] & 0x0f;
			overrunPtr++;
		}
		if (overrunBits & 0x08) {
			overrunCount[6] = buffer[overrunPtr] >> 4;
			overrunCount[7] = buffer[overrunPtr] & 0x0f;
			overrunPtr++;
		}

		//Counts
		countPtr = overrunPtr;
		for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
			if (i % 2) {
				channelCount[i] = buffer[countPtr] & 0x0F;
				countPtr++;
			} else {
				channelCount[i] = buffer[countPtr] >> 4;
			}
			datacount += channelCount[i];

			phid->sensorRatioInterruptCount[i]++;
			phid->sensorVoltageInterruptCount[i]++;
		}

		//Data
		j = 0;
		flip = 0;
		bufindx = countPtr;
		while (datacount > 0) {
			for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
				if (channelCount[i] > j) {
					if (!flip) {
						sensorRawValue[i][j] = ((uint8_t)buffer[bufindx] + (((uint8_t)buffer[bufindx + 1] & 0xf0) << 4));
						bufindx += 2;
					} else {
						sensorRawValue[i][j] = ((uint8_t)buffer[bufindx] + (((uint8_t)buffer[bufindx - 1] & 0x0f) << 8));
						bufindx++;
					}
					//compensating for resistors, etc. - on earlier versions, this was done in Firmware.
					sensorRawValue[i][j] = round(sensorRawValue[i][j] * 1.001);
					if (sensorRawValue[i][j] > 0xfff)
						sensorRawValue[i][j] = 0xfff;
					sensorDataCount[i]++;
					flip ^= 0x01;
					datacount--;
				}
			}
			j++;
		}
		if (datacount < 0)
			logdebug("Datacount error");

		//Send out some errors - overruns/lost packets
		if (phid->dataSinceAttach >= 10) {
			for (i = 0; i < phid->devChannelCnts.numVoltageInputs; i++) {
				if (overrunCount[i]) {
					if ((channel = getChannel(phid, i)) != NULL) {
						SEND_ERROR_EVENT(channel, EEPHIDGET_OVERRUN, "%d sample overrun detected.", overrunCount[i]);
						PhidgetRelease(&channel);
					}
				}
			}
			for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
				if (overrunCount[i]) {
					if ((channel = getChannel(phid, i + phid->devChannelCnts.numVoltageInputs)) != NULL) {
						SEND_ERROR_EVENT(channel, EEPHIDGET_OVERRUN, "%d sample overrun detected.", overrunCount[i]);
						PhidgetRelease(&channel);
					}
				}
			}
		}
		if ((phid->lastPacketCount >= 0) && ((phid->lastPacketCount + 1) & 0x03) != packetCount) {
			//Send this error out on all channels
			for (i = 0; i < phid->devChannelCnts.numVoltageInputs + phid->devChannelCnts.numSensors + phid->devChannelCnts.numInputs + phid->devChannelCnts.numOutputs; i++) {
				if ((channel = getChannel(phid, i)) != NULL) {
					SEND_ERROR_EVENT(channel, EEPHIDGET_PACKETLOST, "One or more data packets were lost");
					PhidgetRelease(&channel);
				}
			}
		}
		if (overcurrentFlag) {
			//send this one to all voltage inputs
			for (i = 0; i < phid->devChannelCnts.numVoltageInputs + phid->devChannelCnts.numSensors; i++) {
				if ((channel = getChannel(phid, i)) != NULL) {
					SEND_ERROR_EVENT(channel, EEPHIDGET_OVERCURRENT, "overcurrent detected.");
					PhidgetRelease(&channel);
				}
			}
		}

		phid->lastPacketCount = packetCount;
	}
	break;

	case PHIDUID_1014:
	case PHIDUID_1017:
		//Outputs
		for (i = 0, j = 0x01; i < phid->devChannelCnts.numOutputs; i++, j <<= 1) {
			if ((buffer[0] & j) == 0)
				outputState[i] = PFALSE;
			else
				outputState[i] = PTRUE;
		}
		break;

#if (PHIDUID_1014_3_USB_SUPPORTED || PHIDUID_1017_2_USB_SUPPORTED)
#if PHIDUID_1014_3_USB_SUPPORTED
	case PHIDUID_1014_3_USB:
#endif
#if PHIDUID_1017_2_USB_SUPPORTED
	case PHIDUID_1017_2_USB:
#endif
		pkt = buffer[0];
		buffer++;
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			i = buffer[0];
			channel = getChannel(phid, i);
			phid->failsafeState[i] = PTRUE;
			if (channel == NULL)
				break;
			return (bridgeSendToChannel(channel, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		}
		break;
#endif /* (PHIDUID_1014_3_USB_SUPPORTED || PHIDUID_1017_2_USB_SUPPORTED) */

	case PHIDUID_1016:
	case PHIDUID_1015:
		//Top bit of buffer[0] indicates a touch
		if (buffer[0] & 0x80) //touch detected
		{
			phid->isTouched[0] = PTRUE;
			phid->proximity[0] = PTRUE;
			phid->touchValue[0] = (buffer[0] & 0x7f) / 127.0;
		} else { //no touch detected
			phid->isTouched[0] = PFALSE;
			phid->proximity[0] = (buffer[0] & 0x01);
			phid->touchValue[0] = PUNK_DBL;
		}
		break;

#if PHIDUID_USBSWITCH_SUPPORTED
	case PHIDUID_USBSWITCH:
		//Outputs
		for (k = 0; k < 3; k++) {
			for (i = 0, j = 0x01; i < 8; i++, j <<= 1) {
				if ((buffer[k] & j) == 0)
					outputState[i + k * 8] = PFALSE;
				else
					outputState[i + k * 8] = PTRUE;
			}
		}
		break;
#endif /* PHIDUID_USBSWITCH_SUPPORTED */

	default:
		MOS_PANIC("Unexpected device");
	}

	//this is set after the sensor data, so users can poll ratiometric after changing it, to know when to to read sensors.
	if (ratiometricEcho != PUNK_BOOL) {
		if (phid->ratiometric == PUNK_BOOL)
			phid->ratiometric = ratiometricEcho;

		if (phid->ratiometricSwitching > 0 && ratiometricEcho == phid->ratiometric)
			phid->ratiometricSwitching--;
		if (!phid->ratiometricSwitching)
			phid->ratiometricEcho = ratiometricEcho;
	}
	//Don't do any skipping unless ratiometric state is echoed back
	else {
		if (phid->ratiometricSwitching > 0)
			phid->ratiometricSwitching = 0;
	}

	//Make sure values are within defined range, and store to structure
	if (!phid->ratiometricSwitching) {
		for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
			int rawSensorAccumulate = 0, sensorAccumulateCount = 0;

			for (j = 0; j < sensorDataCount[i]; j++) {
				if (sensorRawValue[i][j] != PUNK_INT32) {
					rawSensorAccumulate += sensorRawValue[i][j];
					sensorAccumulateCount++;
				}
			}
			if (sensorAccumulateCount > 0) {
				phid->voltage[i] = round_double((rawSensorAccumulate / (double)sensorAccumulateCount) * phid->referenceVoltage / 4095.0, 4);
				phid->voltageRatio[i] = round_double((rawSensorAccumulate / (double)sensorAccumulateCount) / 4095.0, 4);
				phid->gotSensorData[i] = PTRUE;
			}
		}
	}
	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		if (inputState[i] != PUNK_BOOL)
			phid->inputState[i] = inputState[i];
	}
	for (i = 0; i < phid->devChannelCnts.numOutputs; i++) {
		if (outputState[i] != PUNK_BOOL) {
			phid->outputState[i] = outputState[i];
			if (phid->_outputStateSet[i] == PUNK_BOOL)
				phid->_outputStateSet[i] = outputState[i];
		}
	}

	/* Voltage Input */
	//send out any events for changed data
	//only if not switching ratiometric
	if (!phid->ratiometricSwitching && phid->ratiometric == PFALSE) {
		for (i = 0; i < phid->devChannelCnts.numVoltageInputs; i++) {
			if ((channel = getChannel(phid, i)) != NULL) {
				if (sensorDataCount[i] > 0) {
					switch (phid->eventMode[i]) {
					case EVENTMODE_CHANGETRIGGER:
						if (phid->voltage[i] != PUNK_DBL) {
							if (fabs(phid->sensorVoltageLastValue[i] - phid->voltage[i]) >= phid->voltageChangeTrigger[i]
								|| (phid->sensorVoltageLastValue[i] == PUNK_DBL)) {
								bridgeSendToChannel(channel, BP_VOLTAGECHANGE, "%g", phid->voltage[i]);
								phid->sensorVoltageLastValue[i] = phid->voltage[i];
							}
						}
						break;

					case EVENTMODE_DATARATE:
					{
						//Case 1: datarate > interrupt rate, just send out all data that came in
						if (phid->dataInterval[i] <= phid->interruptRate) {
							for (j = 0; j < sensorDataCount[i]; j++) {
								double voltage = round_double(sensorRawValue[i][j] * phid->referenceVoltage / 4095.0, 4);
								//we still follow the changeTrigger rules
								if (fabs(phid->sensorVoltageLastValue[i] - voltage) >= phid->voltageChangeTrigger[i]
									|| (phid->sensorVoltageLastValue[i] == PUNK_DBL)) {
									bridgeSendToChannel(channel, BP_VOLTAGECHANGE, "%g", voltage);
									phid->sensorVoltageLastValue[i] = voltage;
								}
							}
						}
						//Case 2: data is sent out slower then interrupt rate, so we need to accumulate the data and send it out only sometimes.
						else {
							int dataPerEvent = phid->dataInterval[i] / phid->interruptRate;
							phid->sensorVoltageAccumulator[i] += phid->voltage[i];
							phid->sensorVoltageAccumulatorCount[i]++;
							if (phid->sensorVoltageInterruptCount[i] >= dataPerEvent && phid->sensorVoltageAccumulatorCount[i] > 0) {
								double eventVal = round_double(phid->sensorVoltageAccumulator[i] / (double)phid->sensorVoltageAccumulatorCount[i], 4);
								//we still follow the changeTrigger rules
								if (fabs(phid->sensorVoltageLastValue[i] - eventVal) >= phid->voltageChangeTrigger[i]
									|| (phid->sensorVoltageLastValue[i] == PUNK_DBL)) {
									bridgeSendToChannel(channel, BP_VOLTAGECHANGE, "%g", eventVal);
									phid->sensorVoltageLastValue[i] = eventVal;
								}
								phid->sensorVoltageAccumulator[i] = 0;
								phid->sensorVoltageAccumulatorCount[i] = 0;
								phid->sensorVoltageInterruptCount[i] -= dataPerEvent;
							}
						}
					}
					break;
					default:
						break;
					}
				}
				PhidgetRelease(&channel);
			}
		}
	}

	/* Voltage Ratio Input */
	if (!phid->ratiometricSwitching && phid->ratiometric == PTRUE) {
		for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
			int voltageRatioInputChannelIndex = i + phid->devChannelCnts.numVoltageInputs;
			if ((channel = getChannel(phid, voltageRatioInputChannelIndex)) != NULL) {
				if (sensorDataCount[i] > 0) {
					switch (phid->eventMode[i]) {
					case EVENTMODE_CHANGETRIGGER:
						if (phid->voltageRatio[i] != PUNK_DBL) {
							if (fabs(phid->sensorRatioLastValue[i] - phid->voltageRatio[i]) >= phid->voltageRatioChangeTrigger[i]
								|| (phid->sensorRatioLastValue[i] == PUNK_DBL)) {
								bridgeSendToChannel((PhidgetChannelHandle)channel, BP_VOLTAGERATIOCHANGE, "%g", phid->voltageRatio[i]);
								phid->sensorRatioLastValue[i] = phid->voltageRatio[i];
							}
						}
						break;

					case EVENTMODE_DATARATE:
					{
						//Case 1: datarate > interrupt rate, just send out all data that came in
						if (phid->dataInterval[i] <= phid->interruptRate) {
							for (j = 0; j < sensorDataCount[i]; j++) {
								double voltageRatio = round_double(sensorRawValue[i][j] / 4095.0, 4);
								//we still follow the changeTrigger rules
								if (fabs(phid->sensorRatioLastValue[i] - voltageRatio) >= phid->voltageRatioChangeTrigger[i]
									|| (phid->sensorRatioLastValue[i] == PUNK_DBL)) {
									bridgeSendToChannel((PhidgetChannelHandle)channel, BP_VOLTAGERATIOCHANGE, "%g", voltageRatio);
									phid->sensorRatioLastValue[i] = voltageRatio;
								}
							}
						}
						//Case 2: data is sent out slower then interrupt rate, so we need to accumulate the data and send it out only sometimes.
						else {
							int dataPerEvent = phid->dataInterval[i] / phid->interruptRate;
							phid->sensorRatioAccumulator[i] += phid->voltageRatio[i];
							phid->sensorRatioAccumulatorCount[i]++;
							if (phid->sensorRatioInterruptCount[i] >= dataPerEvent && phid->sensorRatioAccumulatorCount[i] > 0) {
								double eventVal = round_double(phid->sensorRatioAccumulator[i] / (double)phid->sensorRatioAccumulatorCount[i], 4);
								//we still follow the changeTrigger rules
								if (fabs(phid->sensorRatioLastValue[i] - eventVal) >= phid->voltageRatioChangeTrigger[i]
									|| (phid->sensorRatioLastValue[i] == PUNK_DBL)) {
									bridgeSendToChannel((PhidgetChannelHandle)channel, BP_VOLTAGERATIOCHANGE, "%g", eventVal);
									phid->sensorRatioLastValue[i] = eventVal;
								}
								phid->sensorRatioAccumulator[i] = 0;
								phid->sensorRatioAccumulatorCount[i] = 0;
								phid->sensorRatioInterruptCount[i] -= dataPerEvent;
							}
						}
					}
					break;
					default:
						break;
					}
				}

				PhidgetRelease(&channel);
			}
		}
	}

	/* Digital Input */
	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		if (phid->inputState[i] != PUNK_BOOL && phid->inputState[i] != lastInputState[i]) {
			int chIndex = i + phid->devChannelCnts.numVoltageInputs + phid->devChannelCnts.numSensors;
			if ((channel = getChannel(phid, chIndex)) != NULL) {
				bridgeSendToChannel(channel, BP_STATECHANGE, "%d", (int)(phid->inputState[i]));
				PhidgetRelease(&channel);
			}
		}
	}

	/* Capacitive Touch */
	for (i = 0; i < phid->devChannelCnts.numCapTouches; i++) {
		int chIndex = i + phid->devChannelCnts.numVoltageInputs + phid->devChannelCnts.numSensors + phid->devChannelCnts.numInputs + phid->devChannelCnts.numOutputs;
		if ((channel = getChannel(phid, chIndex)) == NULL)
			continue;

		if (phid->isTouched[i] != PUNK_BOOL && phid->isTouched[i] != lastTouchState[i]) {
			if (phid->isTouched[i]) {
				/* BUGFIX: sometimes the firmware will send an old value as the first value after being re-touched */
				if(phid->touchValue[i] != phid->touchInputLastValue[i])
					bridgeSendToChannel((PhidgetChannelHandle)channel, BP_TOUCHINPUTVALUECHANGE, "%g", phid->touchValue[i]);
				phid->touchInputLastValue[i] = phid->touchValue[i];
			} else {
				bridgeSendToChannel((PhidgetChannelHandle)channel, BP_TOUCHINPUTEND, "");
			}
		} else if (phid->isTouched[i] == PTRUE) {
			if (phid->touchInputLastValue[i] == PUNK_DBL ||
			  fabs(phid->touchValue[i] - phid->touchInputLastValue[i]) >= phid->touchValueChangeTrigger[i]) {
				bridgeSendToChannel((PhidgetChannelHandle)channel, BP_TOUCHINPUTVALUECHANGE, "%g", phid->touchValue[i]);
				phid->touchInputLastValue[i] = phid->touchValue[i];
			}
		}
		PhidgetRelease(&channel);
	}

	//Decide if we need to set ratiometric
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1011:
	case PHIDUID_1013_NO_ECHO:
	case PHIDUID_1013:
	case PHIDUID_1018:
	case PHIDUID_1018_3:
	case PHIDUID_1202_IFKIT_NO_ECHO:
	case PHIDUID_1202_IFKIT:
	case PHIDUID_1202_IFKIT_FAST:
		doRatiometricSwitching(phid);
		break;
	default:
		break;
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
_setDataRate(mosiop_t iop, PhidgetInterfaceKitDeviceHandle phid, int Index, uint32_t newVal) {

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_IFKIT488:
	case PHIDUID_1011:
	case PHIDUID_1013_NO_ECHO:
	case PHIDUID_1013:
	case PHIDUID_1015:
	case PHIDUID_1016:
	case PHIDUID_1018:
	case PHIDUID_1018_3:
	case PHIDUID_1202_IFKIT_NO_ECHO:
	case PHIDUID_1202_IFKIT:
	case PHIDUID_1202_IFKIT_FAST:
	{
		int i, dataPerPacket = 0;

		//make sure it's a power of 2, or 1
		if (newVal < phid->interruptRate) {
			newVal = (uint32_t)upper_power_of_two(newVal);
		}
		//make sure it's divisible by interruptRate
		else {
			//round to nearest
			newVal = (int)round(newVal / (double)phid->interruptRate) * phid->interruptRate;
		}

		//make sure we're not asking for too much data per packet
		for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
			if (i == Index)
				dataPerPacket += phid->interruptRate / newVal;
			else
				dataPerPacket += phid->interruptRate / phid->dataInterval[i];
		}
		if (dataPerPacket > phid->maxDataPerPacket) {
			return (MOS_ERROR(iop, EPHIDGET_UNSUPPORTED, "Specified data interval is unavailable, because of other channels on this device also specifying fast data intervals."));
		}

		//this just signals the write thread that a write is available
		phid->eventMode[Index] = EVENTMODE_DATARATE;
		phid->dataInterval[Index] = newVal;
		phid->sensorVoltageAccumulator[Index] = 0;
		phid->sensorVoltageAccumulatorCount[Index] = 0;
		phid->sensorVoltageInterruptCount[Index] = 0;
		phid->sensorRatioAccumulator[Index] = 0;
		phid->sensorRatioAccumulatorCount[Index] = 0;
		phid->sensorRatioInterruptCount[Index] = 0;
	}
	break;

	default:
		MOS_PANIC("Unexpected device");
	}

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1011:
	case PHIDUID_1018:
	case PHIDUID_1018_3:
	case PHIDUID_1202_IFKIT_FAST:
		phid->dataInterval[Index] = newVal;
		return (_sendpacket(iop, phid, 0));
	default:
		break;
	}

	return (EPHIDGET_OK);
}


static PhidgetReturnCode CCONV
PhidgetInterfaceKitDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetInterfaceKitDeviceHandle phid = (PhidgetInterfaceKitDeviceHandle)ch->parent;
	PhidgetReturnCode ret;
	double dutyCycle;
#if (PHIDUID_1014_3_USB_SUPPORTED || PHIDUID_1017_2_USB_SUPPORTED)
	uint8_t buffer[2];
	size_t len;
#endif

	assert(phid->phid.deviceInfo.class == PHIDCLASS_INTERFACEKIT);

	switch (ch->class) {
	case PHIDCHCLASS_DIGITALINPUT:
		assert(ch->index < phid->devChannelCnts.numInputs);
		switch (bp->vpkt) {
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_DIGITALOUTPUT:
		assert(ch->index < phid->devChannelCnts.numOutputs);
		switch (phid->phid.deviceInfo.UDD->uid) {

#if (PHIDUID_1014_3_USB_SUPPORTED || PHIDUID_1017_2_USB_SUPPORTED)
#if PHIDUID_1014_3_USB_SUPPORTED
		case PHIDUID_1014_3_USB:
#endif
#if PHIDUID_1017_2_USB_SUPPORTED
		case PHIDUID_1017_2_USB:
#endif
			if (phid->failsafeState[ch->index] && (bp->vpkt != BP_OPENRESET) && (bp->vpkt != BP_CLOSERESET))
				return (EPHIDGET_FAILSAFE);
			switch (bp->vpkt) {
			case BP_SETSTATE:
				phid->_outputStateSet[ch->index] = getBridgePacketInt32(bp, 0);
				buffer[0] = phid->_outputStateSet[ch->index] ? 0xFF : 0x00;
				len = 1;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->index, buffer, &len, 100);
			case BP_SETDUTYCYCLE:
				dutyCycle = getBridgePacketDouble(bp, 0);
				if (dutyCycle != 0.0 && dutyCycle != 1.0)
					return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Duty cycle must be 0 or 1"));
				phid->_outputStateSet[ch->index] = (int)dutyCycle;
				buffer[0] = phid->_outputStateSet[ch->index] ? 0xFF : 0x00;
				len = 1;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->index, buffer, &len, 100);
			case BP_OPENRESET:
			case BP_CLOSERESET:
				len = 0;
				phid->failsafeState[ch->index] = PFALSE;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, ch->index, buffer, &len, 100);
			case BP_ENABLE:
				return (EPHIDGET_OK);
			case BP_SETFAILSAFETIME:
				pack16(buffer, getBridgePacketUInt32(bp, 0));
				len = 2;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_FAILSAFE_TIME, ch->index, buffer, &len, 100);
			case BP_FAILSAFERESET:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_FAILSAFE_RESET, ch->index, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
#endif /* (PHIDUID_1014_3_USB_SUPPORTED || PHIDUID_1017_2_USB_SUPPORTED) */

		default:
			switch (bp->vpkt) {
			case BP_SETSTATE:
				phid->_outputStateSet[ch->index] = getBridgePacketInt32(bp, 0);
				return (_sendpacket(bp->iop, phid, ch->index));
			case BP_SETDUTYCYCLE:
				dutyCycle = getBridgePacketDouble(bp, 0);
				if (dutyCycle != 0.0 && dutyCycle != 1.0)
					return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Duty cycle must be 0 or 1"));
				phid->_outputStateSet[ch->index] = (int)dutyCycle;
				return (_sendpacket(bp->iop, phid, ch->index));
			case BP_OPENRESET:
			case BP_CLOSERESET:
				phid->_outputStateSet[ch->index] = PFALSE;
				return (_sendpacket(bp->iop, phid, ch->index));
			case BP_ENABLE:
				return (EPHIDGET_OK);
			default:
				MOS_PANIC("Unexpected packet type");
			}
	}
	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		assert(ch->index < phid->devChannelCnts.numSensors);
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			ret = _setDataRate(bp->iop, phid, ch->index, getBridgePacketUInt32(bp, 0));
			if (ret == EPHIDGET_OK) {
				setBridgePacketUInt32(bp, phid->dataInterval[ch->index], 0);
				if (bp->entrycnt > 1)
					setBridgePacketDouble(bp, (double)phid->dataInterval[ch->index], 1);
			}
			return ret;
		case BP_SETCHANGETRIGGER:
			phid->voltageRatioChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_SETSENSORTYPE:
			return (supportedVoltageRatioSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		case BP_OPENRESET:
			return (adjustIfkitDataIntervalsAsNeeded(phid, ch, PFALSE));
		case BP_CLOSERESET:
			return (adjustIfkitDataIntervalsAsNeeded(phid, ch, PTRUE));
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_VOLTAGEINPUT:
		assert(ch->index < phid->devChannelCnts.numVoltageInputs);
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			ret = _setDataRate(bp->iop, phid, ch->index, getBridgePacketUInt32(bp, 0));
			if (ret == EPHIDGET_OK) {
				setBridgePacketUInt32(bp, phid->dataInterval[ch->index], 0);
				if (bp->entrycnt > 1)
					setBridgePacketDouble(bp, (double)phid->dataInterval[ch->index], 1);
			}
			return ret;
		case BP_SETCHANGETRIGGER:
			phid->voltageChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_SETSENSORTYPE:
			return (supportedVoltageSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		case BP_OPENRESET:
			return (adjustIfkitDataIntervalsAsNeeded(phid, ch, PFALSE));
		case BP_CLOSERESET:
			return (adjustIfkitDataIntervalsAsNeeded(phid, ch, PTRUE));
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_CAPACITIVETOUCH:
		assert(ch->index == 0);
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			ret = _setDataRate(bp->iop, phid, ch->index, getBridgePacketUInt32(bp, 0));
			if (ret == EPHIDGET_OK) {
				setBridgePacketUInt32(bp, phid->dataInterval[ch->index], 0);
				if (bp->entrycnt > 1)
					setBridgePacketDouble(bp, (double)phid->dataInterval[ch->index], 1);
			}
			return ret;
		case BP_SETCHANGETRIGGER:
			phid->touchValueChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
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

//getPacket - used by write thread to get the next packet to send to device
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetInterfaceKitDeviceHandle phid, int Index) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	int i, j, k;
	size_t len;

	for (i = 0; i < phid->devChannelCnts.numOutputs; i++) {
		//set unknown outputs to false
		if (phid->_outputStateSet[i] == PUNK_BOOL)
			phid->_outputStateSet[i] = PFALSE;
	}

	//fill in the buffer and length
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_IFKIT488:
	{
		unsigned const char phid488_lookup[8] = { 0x00, 0x01, 0x02, 0x03, 0x07, 0x06, 0x05, 0x04 };
		buffer[0] = phid488_lookup[Index];
		if (phid->_outputStateSet[Index])
			buffer[0] |= 0x08;
	}
	break;

	case PHIDUID_1012_NO_ECHO:
	case PHIDUID_1012_BITBUG:
	case PHIDUID_1012:
		for (k = 0; k < 2; k++) {
			for (i = 0, j = 1; i < 8; i++, j <<= 1) {
				if (phid->_outputStateSet[k * 8 + i])
					buffer[k] |= j;
			}
		}
		break;

#if PHIDUID_USBSWITCH_SUPPORTED
	case PHIDUID_USBSWITCH:
		for (k = 0; k < 3; k++) {
			for (i = 0, j = 1; i < 8; i++, j <<= 1) {
				if (phid->_outputStateSet[k * 8 + i])
					buffer[k] |= j;
			}
		}
		break;
#endif /* PHIDUID_USBSWITCH_SUPPORTED */

	case PHIDUID_1013_NO_ECHO:
	case PHIDUID_1013:
	case PHIDUID_1202_IFKIT_NO_ECHO:
	case PHIDUID_1202_IFKIT:
		if (phid->ratiometric == PUNK_BOOL)
			phid->ratiometric = PTRUE;
		for (k = 0, j = 1; k < 8; k++, j <<= 1) {
			if (phid->_outputStateSet[k])
				buffer[0] |= j;
		}
		buffer[3] = (char)phid->ratiometric;
		break;

	case PHIDUID_1011:
	case PHIDUID_1018:
	case PHIDUID_1202_IFKIT_FAST:
		if (phid->ratiometric == PUNK_BOOL)
			phid->ratiometric = PTRUE;
		for (k = 0, j = 1; k < phid->devChannelCnts.numOutputs; k++, j <<= 1) {
			//outputs
			if (phid->_outputStateSet[k])
				buffer[0] |= j;
		}
		for (k = 0, j = 1; k < phid->devChannelCnts.numSensors; k++, j <<= 1) {
			//datarate
			int datarate = (phid->dataInterval[k] > phid->interruptRate ? phid->interruptRate : phid->dataInterval[k]);
			datarate--; //so that 8ms fits in 3 bits
			//odd ones are shifted
			if (k % 2) datarate <<= 4;
			buffer[k / 2 + 1] |= datarate;
		}
		buffer[k / 2 + 1] = (char)phid->ratiometric;
		break;
	case PHIDUID_1018_3:
		if (phid->ratiometric == PUNK_BOOL)
			phid->ratiometric = PTRUE;
		for (k = 0, j = 1; k < phid->devChannelCnts.numOutputs; k++, j <<= 1) {
			//outputs
			if (phid->_outputStateSet[k])
				buffer[0] |= j;
		}
		for (k = 0, j = 1; k < phid->devChannelCnts.numSensors; k++, j <<= 1) {
			//datarate
			int datarate = (phid->dataInterval[k] > phid->interruptRate ? phid->interruptRate : phid->dataInterval[k]);
			datarate--; //so that 8ms fits in 3 bits
						//odd ones are shifted
			if (k % 2) datarate <<= 4;
			buffer[k / 2 + 1] |= datarate;
		}
		buffer[k / 2 + 1] = (char)phid->ratiometric;
		len = getMaxOutPacketSize((PhidgetDeviceHandle)phid);
		return PhidgetDevice_transferpacket(iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, 0, 0, buffer, &len, 100);
	case PHIDUID_1014_NO_ECHO:
	case PHIDUID_1014:
	case PHIDUID_1017:
		for (k = 0, j = 1; k < phid->devChannelCnts.numOutputs; k++, j <<= 1) {
			if (phid->_outputStateSet[k])
				buffer[0] |= j;
		}
		break;

	case PHIDUID_1016:
	case PHIDUID_1015:
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
}

static void CCONV
PhidgetInterfaceKitDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetInterfaceKitDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetInterfaceKitDevice_create(PhidgetInterfaceKitDeviceHandle *phidp) {
	DEVICECREATE_BODY(InterfaceKitDevice, PHIDCLASS_INTERFACEKIT);
	return (EPHIDGET_OK);
}
