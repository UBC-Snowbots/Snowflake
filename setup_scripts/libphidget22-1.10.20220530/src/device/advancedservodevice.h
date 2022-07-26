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

#ifndef __CPHIDGETADVANCEDSERVODEVICE
#define __CPHIDGETADVANCEDSERVODEVICE

typedef struct _PhidgetAdvancedServoDevice *PhidgetAdvancedServoDeviceHandle;
PhidgetReturnCode PhidgetAdvancedServoDevice_create(PhidgetAdvancedServoDeviceHandle *phid);

#define ADVSERVO_MAXSERVOS 8

#define ADVSERVO_NORMAL_PACKET	0x00
#define ADVSERVO_MINMAX_PACKET	0x10

//Flags - need to match those defined in firmware
#define MOTOR_DONE_ADVSERVO	 		0x20
#define MOTOR_DISABLED_ADVSERVO		0x40
#define NO_RAMPING_FLAG_ADVSERVO	0x80

#define ADVSERVO_PGOOD_FLAG		0x01
#define ADVSERVO_PRWSELA_FLAG	0x02
#define ADVSERVO_PRWSELB_FLAG	0x04

struct _PhidgetAdvancedServoDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.advancedservo
	PhidgetDevice phid;

	/* Public Members */

	double position[ADVSERVO_MAXSERVOS];
	double velocity[ADVSERVO_MAXSERVOS];
	int isMoving[ADVSERVO_MAXSERVOS];
	double current[ADVSERVO_MAXSERVOS];

	/* Private Members */

	/* Sets */
	double currentChangeTrigger[ADVSERVO_MAXSERVOS];
	double maxPulseWidth[ADVSERVO_MAXSERVOS];
	double minPulseWidth[ADVSERVO_MAXSERVOS];
	double targetPosition[ADVSERVO_MAXSERVOS];
	double acceleration[ADVSERVO_MAXSERVOS];
	double velocityLimit[ADVSERVO_MAXSERVOS];
	int motorSpeedRampingState[ADVSERVO_MAXSERVOS];
	int motorEngagedState[ADVSERVO_MAXSERVOS];

	/* Internal use */
	int32_t packetCounter[ADVSERVO_MAXSERVOS];
	int32_t packetCounterEcho[ADVSERVO_MAXSERVOS];
	double currentLastTrigger[ADVSERVO_MAXSERVOS];
	double velocityLastTrigger[ADVSERVO_MAXSERVOS];
	double positionLastTrigger[ADVSERVO_MAXSERVOS];
	double velocityMax;
	double accelerationMax;
	double motorPositionMaxLimit;
	double motorPositionMinLimit;
	PhidgetRCServo_Voltage voltage;
	PhidgetRCServo_Voltage voltageEcho;

	uint32_t _interruptRate;
	uint32_t _callcnt;
	uint32_t _rcservoDataInterval[ADVSERVO_MAXSERVOS];
	mostime_t _rcservoDeadline[ADVSERVO_MAXSERVOS];
	uint32_t _currentinputDataInterval[ADVSERVO_MAXSERVOS];
	mostime_t _currentinputDeadline[ADVSERVO_MAXSERVOS];
} typedef PhidgetAdvancedServoDeviceInfo;

#endif
