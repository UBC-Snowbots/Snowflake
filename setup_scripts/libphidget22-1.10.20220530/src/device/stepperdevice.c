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
#include "device/stepperdevice.h"

// === Internal Functions === //
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetStepperDeviceHandle phid, int Index);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetStepperDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetStepperDeviceHandle phid = (PhidgetStepperDeviceHandle)device;
	int i = 0;

	assert(phid);

	//Setup max/min values
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1062:
		phid->microSteps = 2;
		phid->motorSpeedMax = ((phid->microSteps * 0x100) - 1) * 0.75;
		phid->motorSpeedMin = 0;
		phid->accelerationMax = (250 * (0.75 * 0.75)) * 0x3f;
		phid->accelerationMin = (250 * (0.75 * 0.75));
		phid->motorPositionMax = 0x7FFFFFFFFFLL;
		phid->motorPositionMin = -0x7FFFFFFFFFLL;
		phid->currentMax = PUNK_DBL;
		phid->currentMin = PUNK_DBL;
		phid->interruptRate = 16;
		break;
	case PHIDUID_1063:
		phid->microSteps = 16;
		phid->motorSpeedMax = 8 * phid->microSteps * 0x100; //0x8000
		phid->motorSpeedMin = 0;
		phid->accelerationMax = 4000 * 0xff;
		phid->accelerationMin = 4000;
		phid->motorPositionMax = 0x7FFFFFFFFFLL;
		phid->motorPositionMin = -0x7FFFFFFFFFLL;
		phid->currentMax = 2.492;
		phid->currentMin = 0.0542;
		phid->interruptRate = 8;
		break;
	case PHIDUID_1067:
	case PHIDUID_1067_1:
		phid->microSteps = 16;
		phid->motorSpeedMax = 250000; //nice round number
		phid->motorSpeedMin = 0;
		phid->accelerationMax = 10000000;
		phid->accelerationMin = 2;
		phid->motorPositionMax = 1000000000000000LL; // +-1 Quadrillion - enough for 211 years at max speed
		phid->motorPositionMin = -1000000000000000LL;
		phid->currentMax = 4;
		phid->currentMin = 0;
		phid->interruptRate = 8;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	//set data arrays to unknown
	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		phid->inputState[i] = PUNK_BOOL;
	}
	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		phid->velocity[i] = PUNK_DBL;
		phid->position[i] = PUNK_INT64;
		phid->current[i] = PUNK_DBL;
		phid->motorSensedCurrentLastTrigger[i] = PUNK_DBL;
		phid->engaged[i] = PUNK_BOOL;
		phid->isMoving[i] = PUNK_BOOL;
		phid->packetCounterEcho[i] = PUNK_INT32;
		phid->packetCounter[i] = PUNK_INT32;
		phid->currentChangeTrigger[i] = 0.001;
		phid->controlMode[i] = CONTROL_MODE_STEP;
		phid->stepper1stepBugActive[i] = PFALSE;
		phid->lastVelocity[i] = PUNK_DBL;
		phid->lastPosition[i] = PUNK_INT64;
	}

	//read in initial state - only one packet needed
	waitForReads((PhidgetDeviceHandle)phid, 1, 100);

	//At this point, we can only recover (maybe) the position, engaged state, set others to unknown
	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		phid->targetPosition[i] = phid->position[i];
		phid->motorEngagedState[i] = phid->engaged[i];
		phid->acceleration[i] = PUNK_DBL;
		phid->velocityLimit[i] = PUNK_DBL;
		phid->currentLimit[i] = PUNK_DBL;
		phid->packetCounter[i] = phid->packetCounterEcho[i];

		if (phid->isMoving[i] == PUNK_BOOL) {
			if (phid->velocity[i] == 0 || phid->engaged[i] == PFALSE)
				phid->isMoving[i] = PFALSE;
			else
				phid->isMoving[i] = PTRUE;
		}
	}

	return (EPHIDGET_OK);
}

#define CALLTM(phid) ((phid)->interruptRate * (phid)->_callcnt)

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetStepperDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetStepperDeviceHandle phid = (PhidgetStepperDeviceHandle)device;
	PhidgetChannelHandle channel;
	int i = 0, j = 0;
	mostime_t tm;
	int chIndex;
	int fired;

	double speed[STEPPER_MAXSTEPPERS], current[STEPPER_MAXSTEPPERS];
	int64_t position[STEPPER_MAXSTEPPERS];
	uint8_t input[STEPPER_MAXINPUTS] = { 0 }, lastInput[STEPPER_MAXINPUTS] = { 0 };
	uint8_t motorEngaged[STEPPER_MAXSTEPPERS], motorDone[STEPPER_MAXSTEPPERS], justStopped[STEPPER_MAXSTEPPERS];

	assert(phid);
	assert(buffer);

	//Parse device packets - store data locally
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1062:
		for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
			phid->packetCounterEcho[i] = buffer[0 + (i * 9)] & 0x0F;
			motorEngaged[i] = (buffer[0 + (i * 9)] & MOTOR_DISABLED_STEPPER) ? PFALSE : PTRUE;
			motorDone[i] = (buffer[0 + (i * 9)] & MOTOR_DONE_STEPPER) ? PTRUE : PFALSE;

			speed[i] = (double)((int16_t)((buffer[1 + (i * 9)] << 8) | buffer[2 + (i * 9)]));
			speed[i] = (double)((speed[i] / 511.0) * phid->motorSpeedMax);

			position[i] = ((((int64_t)(int8_t)buffer[3 + (i * 9)]) << 40) +
				(((uint64_t)buffer[4 + (i * 9)]) << 32) +
				(((uint64_t)buffer[5 + (i * 9)]) << 24) +
				(((uint64_t)buffer[6 + (i * 9)]) << 16) +
				(((uint64_t)buffer[7 + (i * 9)]) << 8) +
				((uint64_t)buffer[8 + (i * 9)]));

			position[i] -= 0x20; //round
			position[i] >>= 6;

			//current is not returned
			current[i] = PUNK_DBL;
		}
		break;
	case PHIDUID_1063:
	{
		double Vad;
		phid->packetCounterEcho[0] = buffer[0] & 0x0F;
		motorEngaged[0] = (buffer[0] & MOTOR_DISABLED_STEPPER) ? PFALSE : PTRUE;
		motorDone[0] = (buffer[0] & MOTOR_DONE_STEPPER) ? PTRUE : PFALSE;

		speed[0] = (double)((int16_t)((buffer[1] << 8) | buffer[2]));
		speed[0] = (double)((speed[i] / 4096.0) * phid->motorSpeedMax);

		position[0] = ((((int64_t)(int8_t)buffer[3]) << 40) +
			(((uint64_t)buffer[4]) << 32) +
			(((uint64_t)buffer[5]) << 24) +
			(((uint64_t)buffer[6]) << 16) +
			(((uint64_t)buffer[7]) << 8) +
			((uint64_t)buffer[8]));

		position[0] -= 0x04; //round
		position[0] >>= 3;

		for (i = 0, j = 0x01; i < phid->devChannelCnts.numInputs; i++, j <<= 1) {
			if ((buffer[9] & j))
				input[i] = PFALSE;
			else
				input[i] = PTRUE;
		}

		//value is 0-4.16 v in 8 bits
		//(4.16 = (Vbandgap + 1.6) * 2, min 4.096v, max 4.224v according to bandgap voltage min, max)
		//ie +-1.5% error
		Vad = (((uint8_t)buffer[10] * 4.16) / 255.0); // The voltage sensed, to a max of 4.16v
		current[0] = round_double((Vad / (BIPOLAR_STEPPER_CURRENT_SENSE_GAIN * BIPOLAR_STEPPER_CURRENT_LIMIT_Rs)), 3);
	}
	break;
	case PHIDUID_1067:
	case PHIDUID_1067_1:
	{
		phid->packetCounterEcho[0] = buffer[0] & 0x0F;
		motorEngaged[0] = (buffer[0] & MOTOR_DISABLED_STEPPER) ? PFALSE : PTRUE;
		motorDone[0] = (buffer[0] & MOTOR_DONE_STEPPER) ? PTRUE : PFALSE;

		//24.8 floating point format
		speed[0] = (double)(((signed int)((((int8_t)buffer[1]) << 24) | (buffer[2] << 16) | (buffer[3] << 8) | buffer[4])) / 256.0);

		position[0] = ((((int64_t)(int8_t)buffer[5]) << 56) +
			(((uint64_t)buffer[6]) << 48) +
			(((uint64_t)buffer[7]) << 40) +
			(((uint64_t)buffer[8]) << 32) +
			(((uint64_t)buffer[9]) << 24) +
			(((uint64_t)buffer[10]) << 16) +
			(((uint64_t)buffer[11]) << 8) +
			((uint64_t)buffer[12]));
	}
	break;
	default:
		MOS_PANIC("Unexpected device");
	}

	phid->_callcnt++;

	//Make sure values are within defined range, and store to structure
	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {

		if (position[i] > phid->motorPositionMax || position[i] < phid->motorPositionMin)
			phid->position[i] = PUNK_INT64;
		else
			phid->position[i] = position[i];

		if (speed[i] > phid->motorSpeedMax || speed[i] < -phid->motorSpeedMax)
			logwarn("Phidget stepper received out of range speed data: %lE", speed[i]);
		else
			phid->velocity[i] = speed[i];

		phid->current[i] = current[i];

		phid->engaged[i] = motorEngaged[i];
	}

	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		lastInput[i] = phid->inputState[i];
		phid->inputState[i] = input[i];
	}

	//make sure phid->motorStoppedState isn't updated until the other data is filled in

	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {

		// Deal with firmware bug - if we step 1, it takes a LONG time for motorDone to become true
		if (phid->stepper1stepBugActive[i]) {
			if (phid->packetCounter[i] == phid->packetCounterEcho[i] && phid->position[i] == phid->targetPosition[i] && phid->velocity[i] == 0) {
				if (motorDone[i] == PFALSE)
					motorDone[i] = PTRUE;
				else
					phid->stepper1stepBugActive[i] = PFALSE;
			}
		}

		//if we are up to date, and the motor is done, set moving to false - this is the only place that this gets set false;
		justStopped[i] = PFALSE;
		if (phid->packetCounter[i] == phid->packetCounterEcho[i] && motorDone[i] == PTRUE
			&& ((phid->position[i] == phid->targetPosition[i] && phid->velocity[i] == 0) || phid->engaged[i] == PFALSE)) {
			if (phid->isMoving[i] == PTRUE)
				justStopped[i] = PTRUE;
			phid->isMoving[i] = PFALSE;
		} else if (motorDone[i] == PFALSE)
			phid->isMoving[i] = PTRUE;
	}

	//send out any events for changed data

	tm = CALLTM(phid);

	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		channel = getChannel(phid, i);
		if (channel == NULL)
			continue;

		if (tm >= phid->_stepperDeadline[i] || justStopped[i]) {
			fired = 0;
			if (phid->position[i] != PUNK_INT64 && phid->engaged[i] == PTRUE
			  && (phid->position[i] != phid->lastPosition[i] || justStopped[i] == PTRUE)) {
				bridgeSendToChannel(channel, BP_POSITIONCHANGE, "%l", phid->position[i]);
				phid->lastPosition[i] = phid->position[i];
				fired = 1;
			}

			if (phid->velocity[i] != PUNK_DBL && phid->velocity[i] != phid->lastVelocity[i]) {
				bridgeSendToChannel(channel, BP_VELOCITYCHANGE, "%g", phid->velocity[i]);
				phid->lastVelocity[i] = phid->velocity[i];
				fired = 1;
			}

			if (fired)
				phid->_stepperDeadline[i] = tm + phid->_stepperDataInterval[i];
		}

		// Run STOPPED event last, so that position and velocity are updated first
		if (justStopped[i])
			bridgeSendToChannel(channel, BP_STOPPED, 0);

		PhidgetRelease(&channel);
	}

	/* Not under data interval */
	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		chIndex = i + phid->devChannelCnts.numMotors;
		if ((channel = getChannel(phid, chIndex)) != NULL) {
			if (phid->inputState[i] != PUNK_BOOL && phid->inputState[i] != lastInput[i])
				bridgeSendToChannel(channel, BP_STATECHANGE, "%d", phid->inputState[i]);
			PhidgetRelease(&channel);
		}
	}

	for (i = 0; i < phid->devChannelCnts.numCurrentInputs; i++) {
		if (phid->current[i] == PUNK_DBL)
			continue;

		chIndex = i + phid->devChannelCnts.numMotors + phid->devChannelCnts.numInputs;
		if (tm < phid->_currentinputDeadline[i])
			continue;

		channel = getChannel(phid, chIndex);
		if (channel == NULL)
			continue;

		if (fabs(phid->current[i] - phid->motorSensedCurrentLastTrigger[i]) >= phid->currentChangeTrigger[i]
		  || phid->motorSensedCurrentLastTrigger[i] == PUNK_DBL) {
			bridgeSendToChannel(channel, BP_CURRENTCHANGE, "%g", phid->current[i]);
			phid->_currentinputDeadline[i] = tm + phid->_currentinputDataInterval[i];
			phid->motorSensedCurrentLastTrigger[i] = phid->current[i];
		}
		PhidgetRelease(&channel);
	}

	return (EPHIDGET_OK);
}

static int
configured(PhidgetStepperDeviceHandle phid, int index) {
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1062:
		if (phid->acceleration[index] == PUNK_DBL
			|| phid->velocityLimit[index] == PUNK_DBL)
			return (PFALSE);
		return (PTRUE);
	case PHIDUID_1063:
	case PHIDUID_1067:
	case PHIDUID_1067_1:
		if (phid->acceleration[index] == PUNK_DBL
			|| phid->velocityLimit[index] == PUNK_DBL
			|| phid->currentLimit[index] == PUNK_DBL)
			return (PFALSE);
		return (PTRUE);
	}
	return (PFALSE);
}

static PhidgetReturnCode CCONV
PhidgetStepperDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetStepperDeviceHandle phid = (PhidgetStepperDeviceHandle)ch->parent;
	PhidgetReturnCode ret;
	int64_t posnDiff;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_STEPPER);

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

	case PHIDCHCLASS_STEPPER:
		assert(ch->index < phid->devChannelCnts.numMotors);
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			posnDiff = phid->targetPosition[ch->index] - getBridgePacketInt64(bp, 0);
			if(posnDiff == 1 || posnDiff == -1)
				phid->stepper1stepBugActive[ch->index] = PTRUE;
			else
				phid->stepper1stepBugActive[ch->index] = PFALSE;
			phid->targetPosition[ch->index] = getBridgePacketInt64(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | STEPPER_POSITION_PACKET));

		case BP_SETVELOCITYLIMIT:
			switch (phid->controlMode[ch->index]) {
			case CONTROL_MODE_RUN:
				if (getBridgePacketDouble(bp, 0) >= 0) {
					phid->velocityLimit[ch->index] = getBridgePacketDouble(bp, 0);
					phid->targetPosition[ch->index] = phid->motorPositionMax;
				} else {
					phid->velocityLimit[ch->index] = -getBridgePacketDouble(bp, 0);
					phid->targetPosition[ch->index] = phid->motorPositionMin;
				}
				ret = _sendpacket(bp->iop, phid, ch->index | STEPPER_POSITION_PACKET);
				if (ret != EPHIDGET_OK)
					return (ret);
				break;
			case CONTROL_MODE_STEP:
				phid->velocityLimit[ch->index] = getBridgePacketDouble(bp, 0);
				break;
			default:
				MOS_PANIC("Unexpected control mode");
			}

			return (_sendpacket(bp->iop, phid, ch->index | STEPPER_VEL_ACCEL_PACKET));

		case BP_SETACCELERATION:
			phid->acceleration[ch->index] = getBridgePacketDouble(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | STEPPER_VEL_ACCEL_PACKET));

		case BP_SETCURRENTLIMIT:
			phid->currentLimit[ch->index] = getBridgePacketDouble(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | STEPPER_VEL_ACCEL_PACKET));

		case BP_SETENGAGED:
			if (!configured(phid, ch->index))
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Make sure acceleration, velocity, and current limits are set before engaging motor."));
			//hang off the STEPPER_VEL_ACCEL_PACKET because this will have the side effect of also setting these two states
			phid->motorEngagedState[ch->index] = getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | STEPPER_VEL_ACCEL_PACKET));

		case BP_SETCONTROLMODE:
			// If we're switching control modes, set the velocity to 0
			//phid->velocityLimit[ch->index] = 0;

			phid->controlMode[ch->index] = (PhidgetStepper_ControlMode)getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | STEPPER_VEL_ACCEL_PACKET));

		case BP_SETDATAINTERVAL:
			phid->_stepperDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->interruptRate);
			phid->_stepperDeadline[ch->index] = 0;
			return (EPHIDGET_OK);

		case BP_OPENRESET:
		case BP_CLOSERESET:
			phid->motorPositionReset[ch->index] = 0;
			phid->targetPosition[ch->index] = 0;
			phid->motorEngagedState[ch->index] = PFALSE;
			phid->velocityLimit[ch->index] = PUNK_DBL;
			phid->acceleration[ch->index] = PUNK_DBL;
			phid->currentLimit[ch->index] = PUNK_DBL;

			// stop the motor from moving
			ret = _sendpacket(bp->iop, phid, ch->index | STEPPER_VEL_ACCEL_PACKET);
			if (ret != EPHIDGET_OK)
				return ret;
			// reset the position and target in firmware
			ret = _sendpacket(bp->iop, phid, ch->index | STEPPER_RESET_PACKET);
			if (ret != EPHIDGET_OK)
				return ret;
			ret = _sendpacket(bp->iop, phid, ch->index | STEPPER_POSITION_PACKET);
			if (ret != EPHIDGET_OK)
				return ret;

			return (EPHIDGET_OK);

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
			phid->_currentinputDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->interruptRate);
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

//makePacket - constructs a packet using current device state
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetStepperDeviceHandle phid, int Index) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	uint8_t accel = 0, flags = 0;
	uint16_t speed = 0;
	int64_t position = 0;
	double vel;
	double acc;
	size_t len;

	int packet_type = Index & 0x30;
	Index = Index & 0x0F;

	assert(Index < STEPPER_MAXSTEPPERS);

	if (phid->packetCounter[Index] == PUNK_INT32)
		phid->packetCounter[Index] = 0;

	phid->packetCounter[Index]++;
	phid->packetCounter[Index] &= 0x0F;
	phid->isMoving[Index] = PTRUE;

	if (phid->motorEngagedState[Index] != PTRUE)
		flags |= MOTOR_DISABLED_STEPPER;

	if (phid->velocityLimit[Index] == PUNK_DBL)
		vel = phid->motorSpeedMax / 2;
	else
		vel = phid->velocityLimit[Index];

	if (phid->acceleration[Index] == PUNK_DBL)
		acc = phid->accelerationMax / 2; //mid-level acceleration
	else
		acc = phid->acceleration[Index];

	switch (packet_type) {
	case STEPPER_RESET_PACKET:
		phid->targetPosition[Index] = phid->motorPositionReset[Index];
		break;
	}

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1062:
		//2-bit index, 2-bit packet type, 4-bit counter
		buffer[0] = (Index << 6) | packet_type | phid->packetCounter[Index];
		//flags
		buffer[1] = flags;

		switch (packet_type) {
		case STEPPER_POSITION_PACKET:
			position = phid->targetPosition[Index] << 6;
			position += 0x20;

			//48-bit position
			buffer[2] = (uint8_t)(position >> 40);
			buffer[3] = (uint8_t)(position >> 32);
			buffer[4] = (uint8_t)(position >> 24);
			buffer[5] = (uint8_t)(position >> 16);
			buffer[6] = (uint8_t)(position >> 8);
			buffer[7] = (uint8_t)(position);
			break;
		case STEPPER_VEL_ACCEL_PACKET:
			accel = (uint8_t)round((acc / phid->accelerationMax) * 63.0);
			speed = (uint16_t)round((vel / phid->motorSpeedMax) * 511.0);

			//6-bit acceleration
			buffer[2] = accel;
			//9-bit speed
			buffer[3] = speed >> 8;
			buffer[4] = speed & 0xFF;
			//not used
			buffer[5] = 0;
			buffer[6] = 0;
			buffer[7] = 0;
			break;
		case STEPPER_RESET_PACKET:
			position = phid->motorPositionReset[Index] << 6;
			position += 0x20;

			//48-bit position
			buffer[2] = (uint8_t)(position >> 40);
			buffer[3] = (uint8_t)(position >> 32);
			buffer[4] = (uint8_t)(position >> 24);
			buffer[5] = (uint8_t)(position >> 16);
			buffer[6] = (uint8_t)(position >> 8);
			buffer[7] = (uint8_t)(position);
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}
		break;
	case PHIDUID_1063:
	{
		uint8_t currentLimit = 0;
		double Vref;

		//2-bit index, 2-bit packet type, 4-bit counter
		buffer[0] = (Index << 6) | packet_type | phid->packetCounter[Index];
		//flags
		buffer[1] = flags;

		switch (packet_type) {
		case STEPPER_POSITION_PACKET:
			position = phid->targetPosition[Index] << 3;
			position += 0x04;

			//48-bit position
			buffer[2] = (uint8_t)(position >> 40);
			buffer[3] = (uint8_t)(position >> 32);
			buffer[4] = (uint8_t)(position >> 24);
			buffer[5] = (uint8_t)(position >> 16);
			buffer[6] = (uint8_t)(position >> 8);
			buffer[7] = (uint8_t)(position);
			break;
		case STEPPER_VEL_ACCEL_PACKET:
			accel = (uint8_t)round((acc / phid->accelerationMax) * 255.0);
			speed = (uint16_t)round((vel / phid->motorSpeedMax) * 4095.0);

			// highest Vref is 3v (2.5A limit as defined by stepping chip)
			// The 8 is defined by the stepping chip - (ItripMAX = Vref/8Rs)
			if (phid->currentLimit[Index] == PUNK_DBL)
				Vref = 0.5 * 8 * BIPOLAR_STEPPER_CURRENT_LIMIT_Rs; //choose 500mA - should at least work for the most part
			else
				Vref = phid->currentLimit[Index] * 8 * BIPOLAR_STEPPER_CURRENT_LIMIT_Rs;
			// DAC output is 0-63 = 0-4.16v, highest value is 45 (3v)
			// 2.08 = Vbandgap * 1.6
			currentLimit = (uint8_t)round((((Vref - 2.08) / 2.08) * 32) + 31);

			//8-bit acceleration
			buffer[2] = accel;
			//12-bit speed
			buffer[3] = speed >> 8;
			buffer[4] = speed & 0xFF;
			//6-bit current limit
			buffer[5] = currentLimit;
			//not used
			buffer[6] = 0;
			buffer[7] = 0;
			break;
		case STEPPER_RESET_PACKET:
			position = phid->motorPositionReset[Index] << 3;
			position += 0x04;

			//48-bit position
			buffer[2] = (uint8_t)(position >> 40);
			buffer[3] = (uint8_t)(position >> 32);
			buffer[4] = (uint8_t)(position >> 24);
			buffer[5] = (uint8_t)(position >> 16);
			buffer[6] = (uint8_t)(position >> 8);
			buffer[7] = (uint8_t)(position);
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}
	}
	break;
	case PHIDUID_1067:
	case PHIDUID_1067_1:
	{
		uint8_t currentLimit = 0;
		int accelInt, velInt;

		//2-bit index, 2-bit packet type, 4-bit counter
		buffer[0] = (Index << 6) | packet_type | phid->packetCounter[Index];
		//flags
		buffer[1] = flags;

		switch (packet_type) {
		case STEPPER_POSITION_PACKET:
			//64-bit position
			buffer[2] = (uint8_t)(phid->targetPosition[Index] >> 56);
			buffer[3] = (uint8_t)(phid->targetPosition[Index] >> 48);
			buffer[4] = (uint8_t)(phid->targetPosition[Index] >> 40);
			buffer[5] = (uint8_t)(phid->targetPosition[Index] >> 32);
			buffer[6] = (uint8_t)(phid->targetPosition[Index] >> 24);
			buffer[7] = (uint8_t)(phid->targetPosition[Index] >> 16);
			buffer[8] = (uint8_t)(phid->targetPosition[Index] >> 8);
			buffer[9] = (uint8_t)(phid->targetPosition[Index]);
			break;
		case STEPPER_VEL_ACCEL_PACKET:
			accelInt = (int)acc;
			velInt = (int)vel;
			if (phid->currentLimit[Index] == PUNK_DBL)
				currentLimit = (uint8_t)round((0.5 / phid->currentMax) * 255.0);
			else
				currentLimit = (uint8_t)round((phid->currentLimit[Index] / phid->currentMax) * 255.0);

				//3-byte acceleration
			buffer[2] = accelInt >> 16;
			buffer[3] = accelInt >> 8;
			buffer[4] = accelInt;

			//3-byte velocity
			buffer[5] = velInt >> 16;
			buffer[6] = velInt >> 8;
			buffer[7] = velInt;

			//8-bit current limit
			buffer[8] = currentLimit;

			//not used
			buffer[9] = 0;
			break;
		case STEPPER_RESET_PACKET:
			//64-bit position
			buffer[2] = (uint8_t)(phid->motorPositionReset[Index] >> 56);
			buffer[3] = (uint8_t)(phid->motorPositionReset[Index] >> 48);
			buffer[4] = (uint8_t)(phid->motorPositionReset[Index] >> 40);
			buffer[5] = (uint8_t)(phid->motorPositionReset[Index] >> 32);
			buffer[6] = (uint8_t)(phid->motorPositionReset[Index] >> 24);
			buffer[7] = (uint8_t)(phid->motorPositionReset[Index] >> 16);
			buffer[8] = (uint8_t)(phid->motorPositionReset[Index] >> 8);
			buffer[9] = (uint8_t)(phid->motorPositionReset[Index]);
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}
	}
	break;
	default:
		MOS_PANIC("Unexpected device");
	}
	
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1062:
	case PHIDUID_1063:
	case PHIDUID_1067:
		return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
	case PHIDUID_1067_1:
		len = getMaxOutPacketSize((PhidgetDeviceHandle)phid);
		return PhidgetDevice_transferpacket(iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, 0, 0, buffer, &len, 100);
	default:
		MOS_PANIC("Unexpected device");
	}
}

static void CCONV
PhidgetStepperDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetStepperDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetStepperDevice_create(PhidgetStepperDeviceHandle *phidp) {
	DEVICECREATE_BODY(StepperDevice, PHIDCLASS_STEPPER);
	return (EPHIDGET_OK);
}
