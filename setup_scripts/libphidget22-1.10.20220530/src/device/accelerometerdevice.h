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

#ifndef __CPHIDGETACCELEROMETERDEVICE
#define __CPHIDGETACCELEROMETERDEVICE

typedef struct _PhidgetAccelerometerDevice *PhidgetAccelerometerDeviceHandle;
PhidgetReturnCode PhidgetAccelerometerDevice_create(PhidgetAccelerometerDeviceHandle *phid);

#define ACCEL_MAXACCELEROMETERS 1
#define ACCEL_MAXAXES 3
struct _PhidgetAccelerometerDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.accelerometer
	PhidgetDevice phid;

	/* Public Members */
	double acceleration[ACCEL_MAXACCELEROMETERS][ACCEL_MAXAXES];
	double accelerationChangeTrigger[ACCEL_MAXACCELEROMETERS];
	double timestamp[ACCEL_MAXACCELEROMETERS];

	/* Private Members */
	double _maxAcceleration;
	double _minAcceleration;
	uint32_t _interruptRate;
	uint32_t _accelDataInterval[ACCEL_MAXACCELEROMETERS];
	mostime_t _accelDeadline[ACCEL_MAXACCELEROMETERS];
	uint32_t _callcnt;

	double _accelerationLastTrigger[ACCEL_MAXAXES];
} typedef PhidgetAccelerometerDeviceInfo;

#endif
