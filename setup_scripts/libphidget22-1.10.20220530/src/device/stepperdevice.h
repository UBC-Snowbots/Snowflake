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

#ifndef __CPHIDGETSTEPPERDEVICE
#define __CPHIDGETSTEPPERDEVICE

typedef struct _PhidgetStepperDevice *PhidgetStepperDeviceHandle;
PhidgetReturnCode PhidgetStepperDevice_create(PhidgetStepperDeviceHandle *phid);

#define STEPPER_MAXSTEPPERS 8
#define STEPPER_MAXINPUTS 8
#define STEPPER_CHANNELS ((STEPPER_MAXSTEPPERS * 2) + STEPPER_MAXINPUTS)

#define BIPOLAR_STEPPER_CURRENT_SENSE_GAIN 8.5
#define BIPOLAR_STEPPER_CURRENT_LIMIT_Rs 0.150

//flags - make sure these are in the upper 4 bits
#define MOTOR_DONE_STEPPER	 	0x10
#define MOTOR_DISABLED_STEPPER	0x20

//packet types - room for one more
#define STEPPER_POSITION_PACKET		0x00
#define STEPPER_VEL_ACCEL_PACKET	0x10
#define STEPPER_RESET_PACKET		0x20

struct _PhidgetStepperDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.stepper
	PhidgetDevice phid;

	/* Public Members */

	double current[STEPPER_MAXSTEPPERS];
	uint8_t inputState[STEPPER_MAXINPUTS];
	int64_t position[STEPPER_MAXSTEPPERS];
	double velocity[STEPPER_MAXSTEPPERS];
	uint8_t isMoving[STEPPER_MAXSTEPPERS];

	/* Private Members */

	double currentChangeTrigger[STEPPER_MAXSTEPPERS];
	uint8_t engaged[STEPPER_MAXSTEPPERS];
	int64_t targetPosition[STEPPER_MAXSTEPPERS];
	double velocityLimit[STEPPER_MAXSTEPPERS];
	double acceleration[STEPPER_MAXSTEPPERS];
	double currentLimit[STEPPER_MAXSTEPPERS];

	//data from the device
	int32_t packetCounterEcho[STEPPER_MAXSTEPPERS];

	//data from the user
	int64_t motorPositionReset[STEPPER_MAXSTEPPERS];
	uint8_t motorEngagedState[STEPPER_MAXSTEPPERS];
	int32_t packetCounter[STEPPER_MAXSTEPPERS];
	double motorSensedCurrentLastTrigger[STEPPER_MAXSTEPPERS];
	PhidgetStepper_ControlMode controlMode[STEPPER_MAXSTEPPERS];
	int64_t lastPosition[STEPPER_MAXSTEPPERS];
	double lastVelocity[STEPPER_MAXSTEPPERS];

	int stepper1stepBugActive[STEPPER_MAXSTEPPERS];

	double motorSpeedMax, motorSpeedMin;
	double accelerationMax, accelerationMin;
	int64_t motorPositionMax, motorPositionMin;
	double currentMax, currentMin;
	int microSteps;

	int interruptRate;
	uint32_t _stepperDataInterval[STEPPER_MAXSTEPPERS];
	mostime_t _stepperDeadline[STEPPER_MAXSTEPPERS];
	uint32_t _currentinputDataInterval[STEPPER_MAXSTEPPERS];
	mostime_t _currentinputDeadline[STEPPER_MAXSTEPPERS];
	uint32_t _callcnt;
} typedef PhidgetStepperDeviceInfo;

#endif
