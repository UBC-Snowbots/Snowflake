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

#ifndef __CPHIDGETPHSENSORDEVICE
#define __CPHIDGETPHSENSORDEVICE

typedef struct _PhidgetPHSensorDevice *PhidgetPHSensorDeviceHandle;
PhidgetReturnCode PhidgetPHSensorDevice_create(PhidgetPHSensorDeviceHandle *phid);

#define PHSENSOR_MAXSENSORS 1

struct _PhidgetPHSensorDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.phsensor
	PhidgetDevice phid;

	/* Public Members */

	double voltage[PHSENSOR_MAXSENSORS];
	double voltageChangeTrigger[PHSENSOR_MAXSENSORS];

	double PH[PHSENSOR_MAXSENSORS];
	double PHChangeTrigger[PHSENSOR_MAXSENSORS];
	double correctionTemperature[PHSENSOR_MAXSENSORS];

	/* Private Members */

	double potentialMax, potentialMin;
	double phMin, phMax;

	double lastVoltageTrigger[PHSENSOR_MAXSENSORS];
	double lastPHTrigger[PHSENSOR_MAXSENSORS];

	uint32_t _interruptRate;
	uint32_t voltageDataInterval[PHSENSOR_MAXSENSORS];
	mostime_t voltageDeadline[PHSENSOR_MAXSENSORS];
	uint32_t phDataInterval[PHSENSOR_MAXSENSORS];
	mostime_t phDeadline[PHSENSOR_MAXSENSORS];
	uint32_t _callcnt;

} typedef PhidgetPHSensorDeviceInfo;

#endif
