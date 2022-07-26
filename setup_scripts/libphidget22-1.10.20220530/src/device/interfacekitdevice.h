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

#ifndef __CPHIDGETINTERFACEKITDEVICE
#define __CPHIDGETINTERFACEKITDEVICE

typedef struct _PhidgetInterfaceKitDevice *PhidgetInterfaceKitDeviceHandle;
PhidgetReturnCode PhidgetInterfaceKitDevice_create(PhidgetInterfaceKitDeviceHandle *phid);

#define PACKET_OUTPUT_STATE_ECHO 1
#define PACKET_FAILSAFE_TRIGGERED 2

#define IFKIT_MAXINPUTS 32
#define IFKIT_MAXOUTPUTS 32
#define IFKIT_MAXSENSORS 8
#define IFKIT_MAXCAPTOUCHES 1

#define IFKIT_MAXSENSORCHANGE 1000 //BL: Had to check for this, might as well use a define

//usually it is <=8, but could be bigger if a packet gets missed.
#define IFKIT_MAX_DATA_PER_PACKET	16
//in milliseconds - this is the fastest hardware rate of any device
#define IFKIT_MAX_DATA_RATE 1
//1 second is the longest between events that we support
#define IFKIT_MIN_DATA_RATE 1000

struct _PhidgetInterfaceKitDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.ifkit
	PhidgetDevice phid;

	/* Public Members */

	uint8_t outputState[IFKIT_MAXOUTPUTS];		// Values returned from the device
	uint8_t inputState[IFKIT_MAXINPUTS];
	uint32_t dataInterval[IFKIT_MAXSENSORS];
	double voltageChangeTrigger[IFKIT_MAXSENSORS];
	double voltageRatioChangeTrigger[IFKIT_MAXSENSORS];
	double voltage[IFKIT_MAXSENSORS];
	double voltageRatio[IFKIT_MAXSENSORS];
	double touchValueChangeTrigger[IFKIT_MAXCAPTOUCHES];
	double touchValue[IFKIT_MAXCAPTOUCHES];
	int isTouched[IFKIT_MAXCAPTOUCHES];
	int proximity[IFKIT_MAXCAPTOUCHES];

	/* Private Members */

	double referenceVoltage;

	uint8_t ratiometric;
	uint8_t ratiometricEcho;
	uint8_t ratiometricSwitching;

	uint8_t _outputStateSet[IFKIT_MAXOUTPUTS];

	uint8_t failsafeState[IFKIT_MAXOUTPUTS];

	double sensorVoltageLastValue[IFKIT_MAXSENSORS];
	double sensorVoltageAccumulator[IFKIT_MAXSENSORS];
	int sensorVoltageAccumulatorCount[IFKIT_MAXSENSORS];
	int sensorVoltageInterruptCount[IFKIT_MAXSENSORS];

	double sensorRatioLastValue[IFKIT_MAXSENSORS];
	double sensorRatioAccumulator[IFKIT_MAXSENSORS];
	int sensorRatioAccumulatorCount[IFKIT_MAXSENSORS];
	int sensorRatioInterruptCount[IFKIT_MAXSENSORS];

	double touchInputLastValue[IFKIT_MAXSENSORS];

	int gotSensorData[IFKIT_MAXSENSORS];

	Phidget_EventMode eventMode[IFKIT_MAXSENSORS];

	uint32_t interruptRate;
	uint32_t dataRateMax, dataRateMin;
	uint32_t dataRateMaxUsing;

	int32_t lastPacketCount;
	int maxDataPerPacket;

	int dataSinceAttach;

	uint8_t awdc_enabled;
} typedef PhidgetInterfaceKitDeviceInfo;

#endif
