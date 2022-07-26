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
#include "device/motorcontroldevice.h"

static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetMotorControlDeviceHandle, int);

static PhidgetReturnCode CCONV
PhidgetMotorControlDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetMotorControlDeviceHandle phid;
	int i;

	assert(device);
	phid = (PhidgetMotorControlDeviceHandle)device;

	//Setup max/min values
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1060:
		phid->_interruptRate = 64;
		phid->accelerationMax = 1245; //NOTE: measured empirically
		phid->accelerationMin = round_double((phid->accelerationMax / 1023.0), 2);
		break;

	case PHIDUID_1064:
		phid->_interruptRate = 32 * phid->devChannelCnts.numMotors; //NOTE: actually 32, but only 1 motor per packet
		phid->accelerationMax = 1940; //NOTE: measured empirically
		phid->accelerationMin = round_double((phid->accelerationMax / 1023.0), 2);
		phid->currentMax = 37.9;
		break;

	case PHIDUID_1065:
		phid->_interruptRate = 8;
		phid->accelerationMax = 100 / 0.016; //0-100 in 16ms
		phid->accelerationMin = round_double(phid->accelerationMax / 255.0, 2); //0-100 in 4.08 seconds
		phid->currentMax = 13.9;
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	// initialize triggers, set data arrays to unknown
	for (i = 0; i < phid->devChannelCnts.numInputs; i++)
		phid->inputState[i] = PUNK_BOOL;

	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		phid->dutyCycle[i] = PUNK_DBL;
		phid->backEMF[i] = PUNK_DBL;
		phid->motorSetSpeedEcho[i] = PUNK_DBL;
		phid->acceleration[i] = PUNK_DBL;

		// set some defaults for devices that don't support these
		switch (phid->phid.deviceInfo.UDD->uid) {
		case PHIDUID_1060:
		case PHIDUID_1064:
			phid->brakingDutyCycle[i] = 0;
			phid->backEMFSensingStateEcho[i] = PFALSE;
			break;
		case PHIDUID_1065:
			phid->brakingDutyCycle[i] = PUNK_DBL;
			phid->backEMFSensingStateEcho[i] = PUNK_BOOL;
			break;
		default:
			break;
		}
	}

	for (i = 0; i < phid->devChannelCnts.numCurrentInputs; i++) {
		phid->current[i] = PUNK_DBL;
		phid->currentLastTrigger[i] = PUNK_DBL;
		phid->currentChangeTrigger[i] = 0.001;
	}

	for (i = 0; i < phid->devChannelCnts.numEncoders; i++) {
		phid->encoderPositionEcho[i] = 0;
		phid->encoderTimeStamp[i] = 0;
		phid->encoderPositionDelta[i] = 0;

		phid->positionChangeAccumulator[i] = 0;
		phid->timeChangeAccumulator[i] = 0;
		phid->encoderChangeTrigger[i] = 1;
	}

	for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
		phid->voltageRatio[i] = PUNK_DBL;
		phid->sensorRatioLastValue[i] = PUNK_DBL;
	}

	for (i = 0; i < phid->devChannelCnts.numVoltageInputs; i++) {
		phid->voltage[i] = PUNK_DBL;
		phid->sensorVoltageLastValue[i] = PUNK_DBL;
	}

	phid->ratiometricEcho = PUNK_BOOL;
	phid->ratiometricSwitching = 0;

	// read in initial state
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1060:
	case PHIDUID_1065:
		waitForReads((PhidgetDeviceHandle)phid, 1, 100);
		break;
	case PHIDUID_1064:
		waitForReads((PhidgetDeviceHandle)phid, 4, 250);
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	// recover what we can, set others to unknown
	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		if (phid->motorSetSpeedEcho[i] != PUNK_DBL)
			phid->motorSpeed[i] = phid->motorSetSpeedEcho[i];
		else
			phid->motorSpeed[i] = phid->dutyCycle[i] * 100.0;
		phid->motorAcceleration[i] = phid->acceleration[i];
		phid->motorBraking[i] = phid->brakingDutyCycle[i];
		phid->backEMFSensingState[i] = phid->backEMFSensingStateEcho[i];
	}

	for (i = 0; i < phid->devChannelCnts.numEncoders; i++)
		phid->encoderPositionDelta[i] = phid->encoderPositionEcho[i];

	phid->ratiometric = phid->ratiometricEcho;

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
doRatiometricSwitching(PhidgetMotorControlDeviceHandle phid) {
	int ratiometricWanted;
	int voltageInputCnt;
	int voltageRatioCnt;
	int i;

	assert(phid);

	if (!phid->ratiometricSwitching) {
		ratiometricWanted = PTRUE; //default to true
		voltageInputCnt = 0;
		voltageRatioCnt = 0;

		for (i = 0; i < phid->devChannelCnts.numVoltageInputs; i++)
			if (phid->phid.channel[i +
			  phid->devChannelCnts.numMotors +
			  phid->devChannelCnts.numInputs +
			  phid->devChannelCnts.numEncoders])
				voltageInputCnt++;

		for (i = 0; i < phid->devChannelCnts.numSensors; i++)
			if (phid->phid.channel[i +
			  phid->devChannelCnts.numMotors +
			  phid->devChannelCnts.numInputs +
			  phid->devChannelCnts.numEncoders +
			  phid->devChannelCnts.numVoltageInputs])
				voltageRatioCnt++;

		if (voltageInputCnt > 0 && voltageRatioCnt > 0) {
			ratiometricWanted = phid->ratiometric ^ 0x01;
		} else {
			if (voltageInputCnt > 0)
				ratiometricWanted = PFALSE;
			if (voltageRatioCnt > 0)
				ratiometricWanted = PTRUE;
		}

		if (phid->ratiometric != ratiometricWanted) {
			phid->ratiometricSwitching = 4;
			phid->ratiometric = ratiometricWanted;

			return (_sendpacket(NULL, phid, 0));
		}
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
dataInput_1060(PhidgetMotorControlDeviceHandle phid, uint8_t *buffer, size_t length, mostime_t tm) {
	uint8_t lastInputState[MOTORCONTROL_MAXINPUTS] = { 0 };
	uint8_t input[MOTORCONTROL_MAXINPUTS] = { 0 };
	uint8_t error[MOTORCONTROL_MAXINPUTS] = { 0 };
	double speed[MOTORCONTROL_MAXMOTORS] = { 0 };
	PhidgetChannelHandle channel;
	int chIndex;
	int i, j;

	// Parse device packet - store data locally
	for (i = 0, j = 1; i < phid->devChannelCnts.numInputs; i++, j <<= 1) {
		if (buffer[0] & j)
			input[i] = PTRUE;
		else
			input[i] = PFALSE;
	}

	for (i = 0, j = 1; i < phid->devChannelCnts.numMotors; i++, j <<= 1) {
		speed[i] = (char)buffer[4 + i];
		speed[i] = round_double(((speed[i] * 100) / 127.0), 2);

		if (buffer[1] & j)
			error[i] = PTRUE;
	}

	// Make sure values are within defined range, and store to structure
	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		lastInputState[i] = phid->inputState[i];
		phid->inputState[i] = input[i];
	}

	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		phid->dutyCycle[i] = speed[i] / 100.0;

		if (phid->dutyCycle[i] == PUNK_DBL)
			continue;

		if (tm < phid->_dcmotorDeadline[i])
			continue;

		channel = getChannel(phid, i);
		if (channel == NULL)
			continue;

		bridgeSendToChannel(channel, BP_DUTYCYCLECHANGE, "%g", phid->dutyCycle[i]);
		if (error[i])
			SEND_ERROR_EVENT(channel, EEPHIDGET_OVERCURRENT, "Motor exceeded 1.5 Amp current limit.");

		phid->_dcmotorDeadline[i] = tm + phid->_dcmotorDataInterval[i];
		PhidgetRelease(&channel);
	}

	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		if (phid->inputState[i] == PUNK_BOOL || phid->inputState[i] == lastInputState[i])
			continue;

		chIndex = i + phid->devChannelCnts.numMotors;

		channel = getChannel(phid, chIndex);
		if (channel == NULL)
			continue;

		bridgeSendToChannel(channel, BP_STATECHANGE, "%d", (int)(phid->inputState[i]));
		PhidgetRelease(&channel);
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
dataInput_1064(PhidgetMotorControlDeviceHandle phid, uint8_t *buffer, size_t length, mostime_t tm) {
	int error[MOTORCONTROL_MAXINPUTS] = { 0 };
	PhidgetChannelHandle channel;
	double speed, current;
	int curIndex;
	int index;
	int i;

	//Parse device packet - store data locally
	index = buffer[3];
	curIndex = phid->devChannelCnts.numMotors + index;

	speed = (int8_t)buffer[4];
	speed = round_double(((speed * 100) / 127.0), 2);

	// NOTE: current sense is only accurate at +100 and -100 velocity
	current = (uint32_t)(((uint8_t)buffer[6] << 8) | (uint8_t)buffer[7]);
	current -= 5;
	if (current < 0) current = 0;
	current /= 51.2; //volts
	current = (current * 11370) / 1500; //amps

	if (!(buffer[1] & 0x10)) error[0] |= 0x01;
	if (!(buffer[1] & 0x20)) error[1] |= 0x01;
	if (!(buffer[1] & 0x40)) error[0] |= 0x02;
	if (!(buffer[1] & 0x80)) error[1] |= 0x02;

	// Make sure values are within defined range, and store to structure
	phid->dutyCycle[index] = speed / 100.0;
	phid->current[index] = current;

	// send out any events for changed data

	if (tm >= phid->_dcmotorDeadline[index]) {
		if ((channel = getChannel(phid, index)) != NULL) {
			bridgeSendToChannel(channel, BP_DUTYCYCLECHANGE, "%g", phid->dutyCycle[index]);
			phid->_dcmotorDeadline[index] = tm + phid->_dcmotorDataInterval[index];
			PhidgetRelease(&channel);
		}
	}

	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		// There are two error conditions - but as far as I can tell they both mean the same thing
		if (error[i]) {
			if ((channel = getChannel(phid, i)) != NULL) {
				SEND_ERROR_EVENT(channel, EEPHIDGET_OVERTEMP, "Motor overtemperature or short detected.");
				PhidgetRelease(&channel);
			}
		}
	}

	if (phid->current[index] == PUNK_DBL)
		return (EPHIDGET_OK);

	if (phid->currentLastTrigger[index] != PUNK_DBL &&
	  (fabs(phid->current[index] - phid->currentLastTrigger[index]) < phid->currentChangeTrigger[index]))
		return (EPHIDGET_OK);

	if (tm < phid->_currentinputDeadline[index])
		return (EPHIDGET_OK);

	channel = getChannel(phid, curIndex);
	if (channel == NULL)
		return (EPHIDGET_OK);

	bridgeSendToChannel(channel, BP_CURRENTCHANGE, "%g", phid->current[index]);
	phid->currentLastTrigger[index] = phid->current[index];
	phid->_currentinputDeadline[index] = tm + phid->_currentinputDataInterval[index];
	PhidgetRelease(&channel);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
dataInput_1065(PhidgetMotorControlDeviceHandle phid, uint8_t *buffer, size_t length, mostime_t tm) {
	double speed, current, backEmf, backEmfPos, backEmfNeg, supplyVoltage, setSpeed, setAccel, setBraking;
	uint8_t ratioEn, dir, bEMFValid, bEMFenabled, lowVoltageErr, overTempErr, setDir;
	uint8_t lastInputState[MOTORCONTROL_MAXINPUTS] = { 0 };
	int sensorRaw[MOTORCONTROL_MAXSENSORS] = { 0 };
	uint8_t input[MOTORCONTROL_MAXINPUTS] = { 0 };
	uint16_t encoderTimeChange, encoderTime;
	int encoderPositionChange;
	int32_t timeChangeInt;
	int encoderPos;
	int chIndex;
	int i, j;

	PhidgetChannelHandle channel;

	/* Extract Data */

	for (i = 0, j = 1; i < phid->devChannelCnts.numInputs; i++, j <<= 1) {
		if (buffer[0] & j)
			input[i] = PTRUE;
		else
			input[i] = PFALSE;
	}

	overTempErr = ((buffer[1] & 0x01) ? PTRUE : PFALSE);
	lowVoltageErr = ((buffer[1] & 0x02) ? PTRUE : PFALSE);
	bEMFValid = ((buffer[1] & 0x04) ? PTRUE : PFALSE);
	dir = ((buffer[1] & 0x08) ? PTRUE : PFALSE);
	ratioEn = ((buffer[1] & 0x10) ? PTRUE : PFALSE);
	bEMFenabled = ((buffer[1] & 0x20) ? PTRUE : PFALSE);
	setDir = ((buffer[1] & 0x40) ? PTRUE : PFALSE);

	for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
		sensorRaw[i] = ((uint8_t)buffer[i * 2 + 4] << 8) + (uint8_t)buffer[i * 2 + 5];
		sensorRaw[i] = round(sensorRaw[i] * 1.001);
		if (sensorRaw[i] > 0xfff)
			sensorRaw[i] = 0xfff;
	}

	// TODO: check this calculation
	current = (uint32_t)(((uint8_t)buffer[2] << 8) + (uint8_t)buffer[3]);
	current = ((current * 5) / 4096.0);	// voltage on adc (12-bit, 0-5 V)
	current = (current / 150.0);		// Current through resistor (0.24% of current through motor)
	current = round_double(current / 0.0024, 3); // Normalized Current

	backEmfPos = (uint32_t)(((uint8_t)buffer[10] << 8) + (uint8_t)buffer[11]);
	backEmfNeg = (uint32_t)(((uint8_t)buffer[12] << 8) + (uint8_t)buffer[13]);
	backEmf = backEmfPos - backEmfNeg;
	backEmf = round_double(((backEmf * 80) / 4096.0), 3);

	supplyVoltage = (uint32_t)(((uint8_t)buffer[8] << 8) + (uint8_t)buffer[9]);
	supplyVoltage = round_double(((supplyVoltage * 80) / 2048.0), 3); // 80v full scale on ADC

	//logdebug("Supply Voltage: %0.3lf", supplyVoltage);

	speed = ((uint8_t)buffer[20] / 255.0) * 100.0;
	if (!dir)
		speed = -speed;

	setSpeed = ((uint8_t)buffer[21] / 255.0) * 100.0;
	if (!setDir)
		setSpeed = -setSpeed;

	setAccel = ((uint8_t)buffer[22] / 255.0) * phid->accelerationMax;
	setBraking = ((uint8_t)buffer[23] / 255.0) * 100.0;

	encoderPos = (signed int)(((uint8_t)buffer[14] << 24)
	  + ((uint8_t)buffer[15] << 16)
	  + ((uint8_t)buffer[16] << 8)
	  + (uint8_t)buffer[17]);

	encoderTime = (uint32_t)(((uint8_t)buffer[18] << 8) + (uint8_t)buffer[19]);

	/* Process Data */

	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		lastInputState[i] = phid->inputState[i];
		phid->inputState[i] = input[i];
	}

	phid->dutyCycle[0] = speed / 100.0;
	phid->current[0] = current;
	phid->motorSetSpeedEcho[0] = setSpeed;
	phid->acceleration[0] = setAccel;
	phid->brakingDutyCycle[0] = setBraking;

	if (bEMFValid)
		phid->backEMF[0] = backEmf;
	if (!bEMFenabled)
		phid->backEMF[0] = PUNK_DBL;
	phid->backEMFSensingStateEcho[0] = bEMFenabled;

	phid->voltage[2] = supplyVoltage;

	encoderPositionChange = encoderPos - phid->encoderPositionEcho[0];
	phid->encoderPositionEcho[0] = encoderPos;

	//this handles wraparounds because we're using uint16_ts
	encoderTimeChange = (encoderTime - phid->encoderTimeStamp[0]);

	//timeout is 20 seconds
	if (encoderTimeChange > 60000 || phid->encoderTimeStamp[0] == PUNK_INT32)
		timeChangeInt = PUNK_INT32;
	else
		timeChangeInt = encoderTimeChange;

	phid->encoderTimeStamp[0] = encoderTime;

	/*
	 * This is set after the sensor data so users can poll ratiometric after changing it
	 * to know when to to read sensors.
	 */
	if (ratioEn != PUNK_BOOL) {
		if (phid->ratiometric == PUNK_BOOL)
			phid->ratiometric = ratioEn;

		if (phid->ratiometricSwitching > 0)
			phid->ratiometricSwitching--;
		if (!phid->ratiometricSwitching)
			phid->ratiometricEcho = ratioEn;
	} else { // Don't do any skipping unless ratiometric state is echoed back
		if (phid->ratiometricSwitching > 0)
			phid->ratiometricSwitching = 0;
	}

	if (!phid->ratiometricSwitching) {
		for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
			phid->voltage[i] = round_double(sensorRaw[i] * 5.0 / 4095.0, 4);
			phid->voltageRatio[i] = round_double(sensorRaw[i] / 4095.0, 4);
		}
	}

	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		channel = getChannel(phid, i);
		if (channel == NULL)
			continue;

		if (tm >= phid->_dcmotorDeadline[i]) {
			bridgeSendToChannel(channel, BP_DUTYCYCLECHANGE, "%g", phid->dutyCycle[i]);
			if (phid->backEMF[0] != PUNK_DBL)
				bridgeSendToChannel(channel, BP_BACKEMFCHANGE, "%g", phid->backEMF[i]);
			phid->_dcmotorDeadline[i] = tm + phid->_dcmotorDataInterval[i];
		}

		/*
		 * This is true when MC33931 status pin is LOW (active)
		 * could be under-voltage lockout, over-temperature, or short-circuit
		 * over-temperature and short-circuit conditions need to be cleared by toggling D1 or D2
		 */
		if (overTempErr) {
			SEND_ERROR_EVENT(channel, EEPHIDGET_OVERTEMP,
				"Over-temperature / Short-circuit condition detected.");
		}

		/*
		 * If supply voltage is > 5v, then we have likely already recovered from this error.
		 * This happens when the power is first plugged in.
		 * This error happens when the power supply voltage falls, not when it starts low.
		 */
		if (lowVoltageErr && supplyVoltage < 5) {
			SEND_ERROR_EVENT(channel, EEPHIDGET_BADPOWER,
			  "Under-voltage Lockout condition detected. Supply voltage is %0.2lf V", supplyVoltage);
		}

		// Errors I detect - not error flags from the Motor Control IC:
		if (supplyVoltage < 1) {
			SEND_ERROR_EVENT(channel, EEPHIDGET_BADPOWER,
			  "Power supply is unplugged or non-functional. Supply voltage is %0.2lf V", supplyVoltage);
		} else if (supplyVoltage <= 7) {
			SEND_ERROR_EVENT(channel, EEPHIDGET_BADPOWER,
			  "Power supply voltage is too low. Supply voltage is %0.2lf V", supplyVoltage);
		}

		if (supplyVoltage >= 40) {
			SEND_ERROR_EVENT(channel, EEPHIDGET_BADPOWER,
			  "DANGER: Over-voltage condition, braking motor. Supply voltage is %0.2lf V", supplyVoltage);
		} else if (supplyVoltage >= 34) {
			SEND_ERROR_EVENT(channel, EEPHIDGET_BADPOWER,
			  "Power supply voltage is too low. Supply voltage is %0.2lf V", supplyVoltage);
		}

		PhidgetRelease(&channel);
	}

	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		if (phid->inputState[i] == PUNK_BOOL || phid->inputState[i] == lastInputState[i])
			continue;

		chIndex = i + phid->devChannelCnts.numMotors;
		channel = getChannel(phid, chIndex);
		if (channel == NULL)
			continue;

		bridgeSendToChannel(channel, BP_STATECHANGE, "%d", (int)(phid->inputState[i]));
		PhidgetRelease(&channel);
	}


	for (i = 0; i < phid->devChannelCnts.numEncoders; i++) {
		chIndex = i + phid->devChannelCnts.numMotors + phid->devChannelCnts.numInputs;
		channel = getChannel(phid, chIndex);
		if (channel == NULL)
			continue;

		// sync accumulators
		PhidgetRunLock(phid);
		phid->positionChangeAccumulator[i] += encoderPositionChange;
		phid->timeChangeAccumulator[i] += timeChangeInt;

		if (tm < phid->_encoderDeadline[i])
			goto skip1;

		if ((unsigned)abs(phid->positionChangeAccumulator[i]) >= phid->encoderChangeTrigger[i]) {
			uint64_t timechange = (phid->timeChangeAccumulator[i] * 1000000) / 3; // 1/3ms -> ns
			int positionChange = phid->positionChangeAccumulator[i];
			PhidgetRunUnlock(phid);
			bridgeSendToChannel(channel, BP_POSITIONCHANGE, "%d%g%c%d", positionChange, timechange / 1000000.0, 0, 0);
			PhidgetRunLock(phid);
			phid->positionChangeAccumulator[i] = 0;
			phid->timeChangeAccumulator[i] = 0;
			phid->_encoderDeadline[i] = tm + phid->_encoderDataInterval[i];
		}
skip1:
		PhidgetRelease(&channel);
		PhidgetRunUnlock(phid);
	}

	if (!phid->ratiometricSwitching && phid->ratiometric == PFALSE) {
		for (i = 0; i < phid->devChannelCnts.numVoltageInputs; i++) {
			if (phid->voltage[i] == PUNK_DBL)
				continue;

			if (phid->sensorVoltageLastValue[i] != PUNK_DBL &&
			  fabs(phid->sensorVoltageLastValue[i] - phid->voltage[i]) < phid->voltageChangeTrigger[i])
				continue;

			chIndex = i + phid->devChannelCnts.numMotors + phid->devChannelCnts.numInputs + phid->devChannelCnts.numEncoders;

			if (tm < phid->_voltageinputDeadline[i])
				continue;

			channel = getChannel(phid, chIndex);
			if (channel == NULL)
				continue;

			bridgeSendToChannel(channel, BP_VOLTAGECHANGE, "%g", phid->voltage[i]);
			phid->sensorVoltageLastValue[i] = phid->voltage[i];
			phid->_voltageinputDeadline[i] = tm + phid->_voltageinputDataInterval[i];
			PhidgetRelease(&channel);
		}
	}

	if (!phid->ratiometricSwitching && phid->ratiometric == PTRUE) {
		for (i = 0; i < phid->devChannelCnts.numSensors; i++) {
			if (phid->voltageRatio[i] == PUNK_DBL)
				continue;

			if (phid->sensorRatioLastValue[i] != PUNK_DBL &&
			  (fabs(phid->sensorRatioLastValue[i] - phid->voltageRatio[i]) < phid->voltageRatioChangeTrigger[i]))
				continue;

			chIndex = i +
			  phid->devChannelCnts.numMotors +
			  phid->devChannelCnts.numInputs +
			  phid->devChannelCnts.numEncoders +
			  phid->devChannelCnts.numVoltageInputs;

			if (tm < phid->_voltageratioinputDeadline[i])
				continue;

			channel = getChannel(phid, chIndex);
			if (channel == NULL)
				continue;

			bridgeSendToChannel(channel, BP_VOLTAGERATIOCHANGE, "%g", phid->voltageRatio[i]);
			phid->sensorRatioLastValue[i] = phid->voltageRatio[i];
			phid->_voltageratioinputDeadline[i] = tm + phid->_voltageratioinputDataInterval[i];
			PhidgetRelease(&channel);
		}
	}

	for (i = 0; i < phid->devChannelCnts.numCurrentInputs; i++) {
		if (phid->current[i] == PUNK_DBL)
			continue;

		if (phid->currentLastTrigger[i] != PUNK_DBL &&
		  (fabs(phid->current[i] - phid->currentLastTrigger[i]) < phid->currentChangeTrigger[i]))
			continue;

		chIndex = i +
		  phid->devChannelCnts.numMotors +
		  phid->devChannelCnts.numInputs +
		  phid->devChannelCnts.numEncoders +
		  phid->devChannelCnts.numVoltageInputs +
		  phid->devChannelCnts.numSensors;

		if (tm < phid->_currentinputDeadline[i])
			continue;

		channel = getChannel(phid, chIndex);
		if (channel == NULL)
			continue;

		bridgeSendToChannel(channel, BP_CURRENTCHANGE, "%g", phid->current[i]);
		phid->currentLastTrigger[i] = phid->current[i];
		phid->_currentinputDeadline[i] = tm + phid->_currentinputDataInterval[i];
		PhidgetRelease(&channel);
	}

	return (doRatiometricSwitching(phid));
}

static PhidgetReturnCode CCONV
PhidgetMotorControlDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetMotorControlDeviceHandle phid;
	mostime_t tm;

	assert(device);
	assert(buffer);

	phid = (PhidgetMotorControlDeviceHandle)device;

	phid->_callcnt++;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1060: /* Original Motor controller */
		tm = phid->_callcnt * phid->_interruptRate;
		return (dataInput_1060(phid, buffer, length, tm));
	case PHIDUID_1064: /* HC motor controller - packets are indexed */
		tm = phid->_callcnt * phid->_interruptRate / 2; // each call is for one of the motors
		return (dataInput_1064(phid, buffer, length, tm));
	case PHIDUID_1065: /* 1-Motor controller with encoder and sensor inputs */
		tm = phid->_callcnt * phid->_interruptRate;
		return (dataInput_1065(phid, buffer, length, tm));
	default:
		MOS_PANIC("Unexpected device");

	}
}

static PhidgetReturnCode CCONV
PhidgetMotorControlDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetMotorControlDeviceHandle phid;

	assert(ch);
	assert(ch->parent);

	phid = (PhidgetMotorControlDeviceHandle)ch->parent;
	assert(phid->phid.deviceInfo.class == PHIDCLASS_MOTORCONTROL);

	switch (ch->class) {
	case PHIDCHCLASS_DCMOTOR:
		assert(ch->index < phid->devChannelCnts.numMotors);
		switch (bp->vpkt) {
		case BP_SETDUTYCYCLE:
			if (phid->motorAcceleration[ch->index] == PUNK_DBL)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Acceleration must be configured before setting duty cycle."));
			phid->motorSpeed[ch->index] = getBridgePacketDouble(bp, 0) * 100;
			return (_sendpacket(bp->iop, phid, ch->index));
		case BP_SETACCELERATION:
			phid->motorAcceleration[ch->index] = getBridgePacketDouble(bp, 0) * 100;
			return (_sendpacket(bp->iop, phid, ch->index));
		case BP_SETBRAKINGDUTYCYCLE:
			phid->motorBraking[ch->index] = getBridgePacketDouble(bp, 0) * 100;
			return (_sendpacket(bp->iop, phid, ch->index));
		case BP_SETBACKEMFSENSINGSTATE:
			phid->backEMFSensingState[ch->index] = getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index));
		case BP_SETDATAINTERVAL:
			phid->_dcmotorDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
			phid->_dcmotorDeadline[ch->index] = 0;
			return (EPHIDGET_OK);
		case BP_OPENRESET:
		case BP_CLOSERESET:
			phid->motorSpeed[ch->index] = 0;
			phid->backEMFSensingState[ch->index] = PFALSE;
			phid->motorBraking[ch->index] = PUNK_DBL;
			phid->motorAcceleration[ch->index] = PUNK_DBL;
			return (_sendpacket(bp->iop, phid, ch->index));
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

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

	case PHIDCHCLASS_ENCODER:
		assert(ch->index < phid->devChannelCnts.numEncoders);
		switch (bp->vpkt) {
		case BP_SETCHANGETRIGGER:
			phid->encoderChangeTrigger[ch->index] = getBridgePacketUInt32(bp, 0);
			return (EPHIDGET_OK);
		case BP_SETDATAINTERVAL:
			phid->_encoderDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
			phid->_encoderDeadline[ch->index] = 0;
			return (EPHIDGET_OK);
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_VOLTAGEINPUT:
		assert(ch->index < phid->devChannelCnts.numVoltageInputs);
		switch (bp->vpkt) {
		case BP_SETCHANGETRIGGER:
			phid->voltageChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_SETSENSORTYPE:
			return (supportedVoltageSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		case BP_SETDATAINTERVAL:
			phid->_voltageinputDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
			phid->_voltageinputDeadline[ch->index] = 0;
			return (EPHIDGET_OK);
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_VOLTAGERATIOINPUT:
		assert(ch->index < phid->devChannelCnts.numSensors);
		switch (bp->vpkt) {
		case BP_SETCHANGETRIGGER:
			phid->voltageRatioChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_SETSENSORTYPE:
			return (supportedVoltageRatioSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		case BP_SETDATAINTERVAL:
			phid->_voltageratioinputDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
			phid->_voltageratioinputDeadline[ch->index] = 0;
			return (EPHIDGET_OK);
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_CURRENTINPUT:
		assert(ch->index < phid->devChannelCnts.numCurrentInputs);
		switch (bp->vpkt) {
		case BP_SETCHANGETRIGGER:
			phid->currentChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_SETDATAINTERVAL:
			phid->_currentinputDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
			phid->_currentinputDeadline[ch->index] = 0;
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

static PhidgetReturnCode
_sendpacket(mosiop_t iop, PhidgetMotorControlDeviceHandle phid, int idx) {
	uint8_t buffer[MAX_OUT_PACKET_SIZE];
	int velocity;
	int braking;
	uint8_t dir;
	int accel;;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1060:
	case PHIDUID_1064:
		if (phid->motorSpeed[idx] == PUNK_DBL)
			phid->motorSpeed[idx] = 0; //not moving

		velocity = (int)round((phid->motorSpeed[idx] * 127.0) / 100.0);
		if (phid->motorAcceleration[idx] == PUNK_DBL)
			accel = (int)round(0.5 * 1023);
		else
			accel = (int)round((phid->motorAcceleration[idx] / phid->accelerationMax) * 1023);

		buffer[0] = (uint8_t)idx;
		buffer[1] = (uint8_t)(velocity & 0xff);
		buffer[2] = (uint8_t)((accel >> 8) & 0x0f);
		buffer[3] = (uint8_t)(accel & 0xff);
		break;

	case PHIDUID_1065:
		if (phid->motorSpeed[idx] == PUNK_DBL)
			phid->motorSpeed[idx] = 0; //not moving
		if (phid->motorBraking[idx] == PUNK_DBL)
			phid->motorBraking[idx] = 0; //not braking
		if (phid->motorSpeed[idx] >= 0) {
			velocity = (int)round((phid->motorSpeed[idx] * 255.0) / 100.0);
			dir = 1;
		} else {
			velocity = (int)round((-phid->motorSpeed[idx] * 255.0) / 100.0);
			dir = 0;
		}
		if (phid->motorAcceleration[idx] == PUNK_DBL)
			accel = (int)round(0.5 * 255);
		else
			accel = (int)round((phid->motorAcceleration[idx] / phid->accelerationMax) * 255);
		braking = (int)round((phid->motorBraking[idx] * 255.0) / 100.0);
		if (phid->ratiometric == PUNK_BOOL)
			phid->ratiometric = PTRUE;
		if (phid->backEMFSensingState[0] == PUNK_BOOL)
			phid->backEMFSensingState[0] = PFALSE;

		buffer[0] = (uint8_t)(dir | (phid->ratiometric == PTRUE ? 0x04 : 0x00) |
		  (phid->backEMFSensingState[0] == PTRUE ? 0x02 : 0x00));
		buffer[1] = (uint8_t)(velocity & 0xff);
		buffer[2] = (uint8_t)(accel & 0xff);
		buffer[3] = (uint8_t)(braking & 0xff);
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
}

static void CCONV
PhidgetMotorControlDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetMotorControlDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetMotorControlDevice_create(PhidgetMotorControlDeviceHandle *phidp) {
	DEVICECREATE_BODY(MotorControlDevice, PHIDCLASS_MOTORCONTROL);
	return (EPHIDGET_OK);
}