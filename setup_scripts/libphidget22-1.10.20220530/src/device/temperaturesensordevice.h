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

#ifndef __CPHIDGETTEMPERATURESENSORDEVICE
#define __CPHIDGETTEMPERATURESENSORDEVICE

typedef struct _PhidgetTemperatureSensorDevice *PhidgetTemperatureSensorDeviceHandle;
PhidgetReturnCode PhidgetTemperatureSensorDevice_create(PhidgetTemperatureSensorDeviceHandle *phid);

#define TEMPSENSOR_MAXSENSORS 5

#define TEMPSENSOR_AMBIENT_FILTER_COUNT 20

#define GAIN 85.0
#define OFFSET_200 -0.0065
#define OFFSET_300 ((200.0/237.0)*5.0)

#define PHIDID_1048_GAIN		((80 / 2.2) + 5)
// using 53.6K + 10K offset resistors: VOffset = (4.096Vref * 10K) / (10K + 53.6K)
#define PHIDID_1048_OFFSET	(4.096 / 6.36)

struct _PhidgetTemperatureSensorDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.temperaturesensor
	PhidgetDevice phid;

	double temperature[TEMPSENSOR_MAXSENSORS];
	double temperatureChangeTrigger[TEMPSENSOR_MAXSENSORS];
	uint32_t temperatureDataInterval[TEMPSENSOR_MAXSENSORS];
	mostime_t temperatureDeadline[TEMPSENSOR_MAXSENSORS];
	PhidgetTemperatureSensor_ThermocoupleType thermocoupleType[TEMPSENSOR_MAXSENSORS];
	double maxTemperature[TEMPSENSOR_MAXSENSORS];
	double minTemperature[TEMPSENSOR_MAXSENSORS];
	double voltage[TEMPSENSOR_MAXSENSORS];
	double lastTrigger[TEMPSENSOR_MAXSENSORS];
	double lastVoltageTrigger[TEMPSENSOR_MAXSENSORS];
	uint32_t voltageDataInterval[TEMPSENSOR_MAXSENSORS];
	mostime_t voltageDeadline[TEMPSENSOR_MAXSENSORS];
	double voltageChangeTrigger[TEMPSENSOR_MAXSENSORS];
	double lastAmbientTrigger;
	double potentialMax, potentialMin;
	uint32_t di_callcnt;
	int interruptRate;
	int ambientSensorIndex;

	double ambientTemperatureBuffer[TEMPSENSOR_AMBIENT_FILTER_COUNT];
	int32_t ambientTemperatureIndex;
	uint32_t ambientBufferFull;
} typedef PhidgetTemperatureSensorDeviceInfo;

#endif
