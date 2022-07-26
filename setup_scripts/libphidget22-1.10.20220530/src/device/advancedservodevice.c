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
#include "device/advancedservodevice.h"

// === Internal Functions === //
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetAdvancedServoDeviceHandle phid, int Index);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetAdvancedServoDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetAdvancedServoDeviceHandle phid = (PhidgetAdvancedServoDeviceHandle)device;
	int i;

	assert(phid);

	//Setup max/min values
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1061:
	case PHIDUID_1061_PGOOD_FLAG:
	case PHIDUID_1061_CURSENSE_FIX:
	case PHIDUID_RCC0004:
	case PHIDUID_1066:
		phid->motorPositionMaxLimit = 0x8000 / 12.0;
		phid->motorPositionMinLimit = 0x0001 / 12.0;
		phid->velocityMax = (50 / 12.0) * 0x4000; //68266.67 us/s
		phid->accelerationMax = ((50 / 12.0) / 0.02) * 0x4000; //3413333.33 us/s^2
		phid->_interruptRate = 32;
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	//initialize triggers, set data arrays to unknown
	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		phid->velocity[i] = PUNK_DBL;
		phid->position[i] = PUNK_DBL;
		phid->isMoving[i] = PUNK_BOOL;
		phid->packetCounterEcho[i] = PUNK_INT32;
		phid->packetCounter[i] = PUNK_INT32;
		phid->targetPosition[i] = PUNK_DBL;
		phid->acceleration[i] = PUNK_DBL;
		phid->velocityLimit[i] = PUNK_DBL;
		phid->minPulseWidth[i] = PUNK_DBL;
		phid->maxPulseWidth[i] = PUNK_DBL;
		phid->motorSpeedRampingState[i] = PTRUE;
		phid->motorEngagedState[i] = PFALSE;
		phid->velocityLastTrigger[i] = PUNK_DBL;
		phid->positionLastTrigger[i] = PUNK_DBL;
	}
	for (i = 0; i < phid->devChannelCnts.numCurrentInputs; i++) {
		phid->current[i] = PUNK_DBL;
		phid->currentLastTrigger[i] = PUNK_DBL;
		phid->currentChangeTrigger[i] = 0.001;
	}
	phid->voltage = PUNK_ENUM;

	//read in initial state - only one packet needed
	waitForReads((PhidgetDeviceHandle)phid, 1, 100);

	for (i = 0; i < phid->devChannelCnts.numMotors; i++)
		phid->packetCounter[i] = phid->packetCounterEcho[i];
	phid->voltage = phid->voltageEcho;

	return (EPHIDGET_OK);
}

#define CALLTM(phid)	((phid)->_interruptRate * (phid)->_callcnt)

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetAdvancedServoDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetAdvancedServoDeviceHandle phid = (PhidgetAdvancedServoDeviceHandle)device;
	//uint8_t speedRamping[ADVSERVO_MAXSERVOS];
	int motorEngaged[ADVSERVO_MAXSERVOS];
	int justStopped[ADVSERVO_MAXSERVOS];
	int motorDone[ADVSERVO_MAXSERVOS];
	double current[ADVSERVO_MAXSERVOS];
	double position[ADVSERVO_MAXSERVOS];
	double velocity[ADVSERVO_MAXSERVOS];
	int pwmEcho[ADVSERVO_MAXSERVOS];
	PhidgetChannelHandle channel;
	PhidgetReturnCode res;
	mostime_t tm;
	int sentEvent;
	int chIndex;
	int pwm;
	int i;

	assert(phid);
	assert(buffer);

	//Parse device packets - store data locally
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1061:
		for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
			phid->packetCounterEcho[i] = buffer[0 + (i * 7)] & 0x0F;
			motorEngaged[i] = (buffer[0 + (i * 7)] & MOTOR_DISABLED_ADVSERVO) ? PFALSE : PTRUE;
			//speedRamping[i] = (buffer[0 + (i * 7)] & NO_RAMPING_FLAG_ADVSERVO) ? PFALSE : PTRUE;
			motorDone[i] = (buffer[0 + (i * 7)] & MOTOR_DONE_ADVSERVO) ? PTRUE : PFALSE;

			pwmEcho[i] = (uint16_t)(buffer[1 + (i * 7)] << 8) + (uint8_t)(buffer[2 + (i * 7)]);
			position[i] = pwmEcho[i] / 12.0;

			velocity[i] = (int16_t)(buffer[3 + (i * 7)] << 8) + (uint8_t)(buffer[4 + (i * 7)]);
			velocity[i] = round_double(((velocity[i] / (double)0x4000)*phid->velocityMax), 2);

			current[i] = (uint16_t)(buffer[5 + (i * 7)] << 8) + (uint8_t)(buffer[6 + (i * 7)]);
			current[i] = round_double(((50.0 / 11.0) * (current[i] / (double)0x4000)), 4);
		}
		break;

		//overcurrent detect
	case PHIDUID_1061_PGOOD_FLAG:
		for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
			phid->packetCounterEcho[i] = buffer[0 + (i * 7)] & 0x0F;
			motorEngaged[i] = (buffer[0 + (i * 7)] & MOTOR_DISABLED_ADVSERVO) ? PFALSE : PTRUE;
			motorDone[i] = (buffer[0 + (i * 7)] & MOTOR_DONE_ADVSERVO) ? PTRUE : PFALSE;

			pwmEcho[i] = (uint16_t)(buffer[1 + (i * 7)] << 8) + (uint8_t)(buffer[2 + (i * 7)]);
			position[i] = pwmEcho[i] / 12.0;

			velocity[i] = (int16_t)(buffer[3 + (i * 7)] << 8) + (uint8_t)(buffer[4 + (i * 7)]);
			velocity[i] = round_double(((velocity[i] / (double)0x4000)*phid->velocityMax), 2);

			current[i] = (uint16_t)(buffer[5 + (i * 7)] << 8) + (uint8_t)(buffer[6 + (i * 7)]);
			current[i] = round_double(((50.0 / 11.0) * (current[i] / (double)0x4000)), 4);
		}

		//PowerGood
		if (!(buffer[56] & ADVSERVO_PGOOD_FLAG)) {
			for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
				if ((channel = getChannel(phid, i)) != NULL) {
					SEND_ERROR_EVENT(channel, EEPHIDGET_BADPOWER, "Bad power supply detected - undervoltage or overcurrent.");
					PhidgetRelease(&channel);
				}
			}
		}
		break;

	case PHIDUID_1061_CURSENSE_FIX:
		//different current sense formula
		for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
			phid->packetCounterEcho[i] = buffer[0 + (i * 7)] & 0x0F;
			motorEngaged[i] = (buffer[0 + (i * 7)] & MOTOR_DISABLED_ADVSERVO) ? PFALSE : PTRUE;
			motorDone[i] = (buffer[0 + (i * 7)] & MOTOR_DONE_ADVSERVO) ? PTRUE : PFALSE;

			pwmEcho[i] = (uint16_t)(buffer[1 + (i * 7)] << 8) + (uint8_t)(buffer[2 + (i * 7)]);
			position[i] = pwmEcho[i] / 12.0;

			velocity[i] = (int16_t)(buffer[3 + (i * 7)] << 8) + (uint8_t)(buffer[4 + (i * 7)]);
			velocity[i] = round_double(((velocity[i] / (double)0x4000)*phid->velocityMax), 2);

			current[i] = (uint16_t)(buffer[5 + (i * 7)] << 8) + (uint8_t)(buffer[6 + (i * 7)]);
			current[i] = round_double((((5.0 / 11.0) / 0.022) * (current[i] / (double)0x4000)), 4);
		}

		//PowerGood
		if (!(buffer[56] & ADVSERVO_PGOOD_FLAG)) {
			for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
				if ((channel = getChannel(phid, i)) != NULL) {
					SEND_ERROR_EVENT(channel, EEPHIDGET_BADPOWER, "Bad power supply detected - undervoltage or overcurrent.");
					PhidgetRelease(&channel);
				}
			}
		}
		break;

	case PHIDUID_RCC0004:
		for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
			phid->packetCounterEcho[i] = buffer[0 + (i * 5)] & 0x0F;
			motorEngaged[i] = (buffer[0 + (i * 5)] & MOTOR_DISABLED_ADVSERVO) ? PFALSE : PTRUE;
			motorDone[i] = (buffer[0 + (i * 5)] & MOTOR_DONE_ADVSERVO) ? PTRUE : PFALSE;

			pwmEcho[i] = (uint16_t)(buffer[1 + (i * 5)] << 8) + (uint8_t)(buffer[2 + (i * 5)]);
			position[i] = pwmEcho[i] / 12.0;

			velocity[i] = (int16_t)(buffer[3 + (i * 5)] << 8) + (uint8_t)(buffer[4 + (i * 5)]);
			velocity[i] = round_double(((velocity[i] / (double)0x4000)*phid->velocityMax), 2);
		}

		switch (buffer[40] & (ADVSERVO_PRWSELA_FLAG | ADVSERVO_PRWSELB_FLAG)) {
		case 0:
			phid->voltageEcho = RCSERVO_VOLTAGE_5V;
			break;
		case ADVSERVO_PRWSELA_FLAG:
			phid->voltageEcho = RCSERVO_VOLTAGE_6V;
			break;
		case (ADVSERVO_PRWSELA_FLAG | ADVSERVO_PRWSELB_FLAG):
			phid->voltageEcho = RCSERVO_VOLTAGE_7_4V;
			break;
		case ADVSERVO_PRWSELB_FLAG:
		default:
			// Error..
			phid->voltageEcho = PUNK_ENUM;
			break;
		}

		//PowerGood
		if (!(buffer[40] & ADVSERVO_PGOOD_FLAG)) {
			for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
				if ((channel = getChannel(phid, i)) != NULL) {
					SEND_ERROR_EVENT(channel, EEPHIDGET_BADPOWER, "Bad power supply detected - undervoltage or overcurrent.");
					PhidgetRelease(&channel);
				}
			}
		}
		break;

	case PHIDUID_1066:
		phid->packetCounterEcho[0] = buffer[0] & 0x0F;
		motorEngaged[0] = (buffer[0] & MOTOR_DISABLED_ADVSERVO) ? PFALSE : PTRUE;
		//speedRamping[0] = (buffer[0] & NO_RAMPING_FLAG_ADVSERVO) ? PFALSE : PTRUE;
		motorDone[0] = (buffer[0] & MOTOR_DONE_ADVSERVO) ? PTRUE : PFALSE;

		pwmEcho[0] = (uint16_t)(buffer[1] << 8) + (uint8_t)(buffer[2]);
		position[0] = pwmEcho[0] / 12.0;

		velocity[0] = (int16_t)(buffer[3] << 8) + (uint8_t)(buffer[4]);
		velocity[0] = round_double(((velocity[0] / (double)0x4000)*phid->velocityMax), 2);

		current[0] = (uint16_t)(buffer[5] << 8) + (uint8_t)(buffer[6]);
		current[0] = round_double((current[0] / 2068.0), 4);
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	//Make sure values are within defined range, and store to structure
	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {

		if (position[i] > phid->motorPositionMaxLimit || position[i] < phid->motorPositionMinLimit)
			phid->position[i] = PUNK_DBL;
		else
			phid->position[i] = position[i];
		if (velocity[i] > phid->velocityMax || velocity[i] < -phid->velocityMax)
			logwarn("Phidget advanced servo received out of range velocity data: %lE", velocity[i]);
		else
			phid->velocity[i] = velocity[i];

		phid->current[i] = current[i];

		if (phid->isMoving[i] == PUNK_BOOL) {
			if (phid->velocity[i] == 0 || motorEngaged[i] == PFALSE)
				phid->isMoving[i] = PFALSE;
			else
				phid->isMoving[i] = PTRUE;
		}
	}
	if ((int32_t)phid->voltageEcho == PUNK_ENUM)
		phid->voltageEcho = RCSERVO_VOLTAGE_5V;

	phid->_callcnt++;

	//make sure phid->motorStoppedState isn't updated until the other data is filled in

	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		pwm = round(phid->targetPosition[i] * 12.0);
		//if we are up to date, and the motor is done, set stopped to true - this is the only place that this gets set true;
		justStopped[i] = PFALSE;
		if (phid->packetCounter[i] == phid->packetCounterEcho[i] && motorDone[i] == PTRUE
			&& ((pwmEcho[i] == pwm && phid->velocity[i] == 0) || motorEngaged[i] == PFALSE)) {
			if (phid->isMoving[i] == PTRUE)
				justStopped[i] = PTRUE;
			phid->isMoving[i] = PFALSE;
		} else if (motorDone[i] == PFALSE)
			phid->isMoving[i] = PTRUE;
	}

	tm = CALLTM(phid);

	//send out any events for changed data
	//only send a position event if the motor is engaged
	for (i = 0; i < phid->devChannelCnts.numMotors; i++) {
		channel = getChannel(phid, i);
		if (channel == NULL)
			continue;

		if (justStopped[i] == PTRUE) {
			res = bridgeSendToChannel(channel, BP_TARGETPOSITIONREACHED, "%g", phid->position[i]);
			if (res != EPHIDGET_OK) {
				PhidgetRelease(&channel);
				return (res);
			}
		}

		if (tm < phid->_rcservoDeadline[i]) {
			PhidgetRelease(&channel);
			continue;
		}

		sentEvent = 0;

		if (phid->position[i] != PUNK_DBL && motorEngaged[i] == PTRUE &&
			(phid->position[i] != phid->positionLastTrigger[i] || phid->positionLastTrigger[i] == PUNK_DBL || justStopped[i] == PTRUE)) {
			res = bridgeSendToChannel(channel, BP_POSITIONCHANGE, "%g", phid->position[i]);
			if (res != EPHIDGET_OK) {
				PhidgetRelease(&channel);
				return (res);
			}
			phid->positionLastTrigger[i] = phid->position[i];
			sentEvent = 1;
		}

		if (phid->velocity[i] != PUNK_DBL && (phid->velocity[i] != phid->velocityLastTrigger[i] || phid->velocityLastTrigger[i] == PUNK_DBL)) {
			res = bridgeSendToChannel(channel, BP_VELOCITYCHANGE, "%g",
				phid->velocity[i]);
			if (res != EPHIDGET_OK) {
				PhidgetRelease(&channel);
				return (res);
			}
			phid->velocityLastTrigger[i] = phid->velocity[i];
			sentEvent = 1;
		}

		if (sentEvent)
			phid->_rcservoDeadline[i] = tm + phid->_rcservoDataInterval[i];

		PhidgetRelease(&channel);
	}

	for (i = 0; i < phid->devChannelCnts.numCurrentInputs; i++) {
		chIndex = i + phid->devChannelCnts.numMotors;

		if (tm < phid->_currentinputDeadline[i])
			continue;

		channel = getChannel(phid, chIndex);
		if (channel == NULL)
			continue;

		if (phid->current[i] != PUNK_DBL
			&& (fabs(phid->current[i] - phid->currentLastTrigger[i]) >= phid->currentChangeTrigger[i]
				|| phid->currentLastTrigger[i] == PUNK_DBL)) {
			bridgeSendToChannel(channel, BP_CURRENTCHANGE, "%g", phid->current[i]);
			phid->currentLastTrigger[i] = phid->current[i];
			phid->_currentinputDeadline[i] = tm + phid->_currentinputDataInterval[i];
		}
		PhidgetRelease(&channel);
	}

	return (EPHIDGET_OK);
}

static int
configured(PhidgetAdvancedServoDeviceHandle phid, int index) {
	if (phid->targetPosition[index] == PUNK_DBL
		|| phid->velocityLimit[index] == PUNK_DBL
		|| phid->acceleration[index] == PUNK_DBL
		|| phid->minPulseWidth[index] == PUNK_DBL
		|| phid->maxPulseWidth[index] == PUNK_DBL)
		return PFALSE;
	return PTRUE;
}

static PhidgetReturnCode CCONV
PhidgetAdvancedServoDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetAdvancedServoDeviceHandle phid = (PhidgetAdvancedServoDeviceHandle)ch->parent;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_ADVANCEDSERVO);

	switch (ch->class) {
	case PHIDCHCLASS_CURRENTINPUT:
		assert(ch->index < phid->devChannelCnts.numCurrentInputs);
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			phid->_currentinputDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
			phid->_currentinputDeadline[ch->index] = 0;
			return (EPHIDGET_OK);
		case BP_SETCHANGETRIGGER:
			phid->currentChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_RCSERVO:
		assert(ch->index < phid->devChannelCnts.numMotors);
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			phid->_rcservoDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
			phid->_rcservoDeadline[ch->index] = 0;
			return (EPHIDGET_OK);
		case BP_SETSPEEDRAMPINGSTATE:
			phid->motorSpeedRampingState[ch->index] = getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | ADVSERVO_NORMAL_PACKET));
		case BP_SETENGAGED:
			// Don't allow engaging of the motor until it is configured
			if (getBridgePacketInt32(bp, 0) == PTRUE && !configured(phid, ch->index)) {
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Target position must be set before engaging the servo."));
			}
			phid->motorEngagedState[ch->index] = getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | ADVSERVO_NORMAL_PACKET));
		case BP_SETACCELERATION:
			phid->acceleration[ch->index] = getBridgePacketDouble(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | ADVSERVO_NORMAL_PACKET));
		case BP_SETMINPULSEWIDTH:
			phid->minPulseWidth[ch->index] = getBridgePacketDouble(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | ADVSERVO_MINMAX_PACKET));
		case BP_SETMAXPULSEWIDTH:
			phid->maxPulseWidth[ch->index] = getBridgePacketDouble(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | ADVSERVO_MINMAX_PACKET));
		case BP_SETVELOCITYLIMIT:
			phid->velocityLimit[ch->index] = getBridgePacketDouble(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | ADVSERVO_NORMAL_PACKET));
		case BP_SETTARGETPOSITION:
			phid->targetPosition[ch->index] = getBridgePacketDouble(bp, 0);
			return (_sendpacket(bp->iop, phid, ch->index | ADVSERVO_NORMAL_PACKET));
		case BP_SETVOLTAGE:
			phid->voltage = getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid, 0 | ADVSERVO_MINMAX_PACKET));
		case BP_OPENRESET:
		case BP_CLOSERESET:
			phid->motorEngagedState[ch->index] = PFALSE;
			phid->motorSpeedRampingState[ch->index] = PTRUE;
			phid->targetPosition[ch->index] = PUNK_DBL;
			phid->velocityLimit[ch->index] = PUNK_DBL;
			phid->acceleration[ch->index] = PUNK_DBL;
			phid->minPulseWidth[ch->index] = PUNK_DBL;
			phid->maxPulseWidth[ch->index] = PUNK_DBL;
			phid->voltage = RCSERVO_VOLTAGE_5V;
			return (_sendpacket(bp->iop, phid, ch->index | ADVSERVO_MINMAX_PACKET));
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
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetAdvancedServoDeviceHandle phid, int Index) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	int packet_type;
	uint8_t flags;
	int velocity;
	int minpwm;
	int maxpwm;
	int accel;
	int pwm;

	packet_type = Index & 0x10;
	Index = Index & 0x07;

	if (phid->packetCounter[Index] == PUNK_INT32)
		phid->packetCounter[Index] = 0;

	phid->packetCounter[Index]++;
	phid->packetCounter[Index] &= 0x0F;

	if (phid->motorEngagedState[Index] == PTRUE)
		phid->isMoving[Index] = PTRUE;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1061:
	case PHIDUID_1061_PGOOD_FLAG:
	case PHIDUID_1061_CURSENSE_FIX:
	case PHIDUID_RCC0004:
	case PHIDUID_1066:
		flags = 0;

		if (phid->motorSpeedRampingState[Index] == PFALSE)
			flags |= NO_RAMPING_FLAG_ADVSERVO;

		if (phid->motorEngagedState[Index] == PFALSE || phid->motorEngagedState[Index] == PUNK_BOOL)
			flags |= MOTOR_DISABLED_ADVSERVO;

		if (phid->phid.deviceInfo.UDD->uid == PHIDUID_RCC0004) {
			switch (phid->voltage) {
			case RCSERVO_VOLTAGE_6V:
				flags |= ADVSERVO_PRWSELA_FLAG;
				break;
			case RCSERVO_VOLTAGE_7_4V:
				flags |= (ADVSERVO_PRWSELA_FLAG | ADVSERVO_PRWSELB_FLAG);
				break;
			case RCSERVO_VOLTAGE_5V:
			default:
				break;
			}
		}

		//2-bit index, 2-bit packet type, 4-bit counter
		buffer[0] = (unsigned char)(Index << 5) | packet_type | phid->packetCounter[Index];
		buffer[1] = flags;

		switch (packet_type) {
		case ADVSERVO_NORMAL_PACKET:
			//have to make sure that everything to be sent has some sort of default value if the user hasn't set a value
			if (phid->targetPosition[Index] == PUNK_DBL)
				pwm = 900 * 12; //mid-range valid posn (much better then 0)
			else
				pwm = round(phid->targetPosition[Index] * 12.0);

			if (phid->velocityLimit[Index] == PUNK_DBL)
				velocity = (int)(0.05 * 0x4000);
			else
				velocity = round((phid->velocityLimit[Index] / (double)phid->velocityMax) * 0x4000);

			if (phid->acceleration[Index] == PUNK_DBL)
				accel = (int)(0.5 * 0x4000); //mid-level acceleration
			else
				accel = round((phid->acceleration[Index] / (double)phid->accelerationMax) * 0x4000);

			buffer[2] = (uint8_t)((pwm >> 8) & 0xff);
			buffer[3] = (uint8_t)(pwm & 0xff);
			buffer[4] = (uint8_t)((velocity >> 8) & 0xff);
			buffer[5] = (uint8_t)(velocity & 0xff);
			buffer[6] = (uint8_t)((accel >> 8) & 0xff);
			buffer[7] = (uint8_t)(accel & 0xff);
			break;
		case ADVSERVO_MINMAX_PACKET:
			if (phid->minPulseWidth[Index] == PUNK_DBL)
				minpwm = round(phid->motorPositionMinLimit * 12.0);
			else
				minpwm = round(phid->minPulseWidth[Index] * 12.0);

			if (phid->maxPulseWidth[Index] == PUNK_DBL)
				maxpwm = round(phid->motorPositionMaxLimit * 12.0);
			else
				maxpwm = round(phid->maxPulseWidth[Index] * 12.0);

			buffer[2] = (uint8_t)((minpwm >> 8) & 0xff);
			buffer[3] = (uint8_t)(minpwm & 0xff);
			buffer[4] = (uint8_t)((maxpwm >> 8) & 0xff);
			buffer[5] = (uint8_t)(maxpwm & 0xff);
			buffer[6] = 0;
			buffer[7] = 0;
			break;
		default:
			MOS_PANIC("Unexpected packet");
		}
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
}

static void CCONV
PhidgetAdvancedServoDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetAdvancedServoDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetAdvancedServoDevice_create(PhidgetAdvancedServoDeviceHandle *phidp) {
	DEVICECREATE_BODY(AdvancedServoDevice, PHIDCLASS_ADVANCEDSERVO);
	return (EPHIDGET_OK);
}
