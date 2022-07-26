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

#ifndef __CPHIDGETBRIDGEDEVICE
#define __CPHIDGETBRIDGEDEVICE

typedef struct _PhidgetBridgeDevice *PhidgetBridgeDeviceHandle;
PhidgetReturnCode PhidgetBridgeDevice_create(PhidgetBridgeDeviceHandle *phid);

#define BRIDGE_MAXINPUTS 4
struct _PhidgetBridgeDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.bridge
	PhidgetDevice phid;

	/* Public Members */

	double voltageRatio[BRIDGE_MAXINPUTS];

	/* Private Members */

	double dataInterval[BRIDGE_MAXINPUTS];
	double voltageRatioChangeTrigger[BRIDGE_MAXINPUTS];

	uint8_t enabled[BRIDGE_MAXINPUTS];
	PhidgetVoltageRatioInput_BridgeGain gain[BRIDGE_MAXINPUTS];
	uint32_t dataRate;

	uint8_t enabledEcho[BRIDGE_MAXINPUTS];
	PhidgetVoltageRatioInput_BridgeGain gainEcho[BRIDGE_MAXINPUTS];
	double bridgeLastTrigger[BRIDGE_MAXINPUTS];

	uint32_t dataRateMin, dataRateMax;
	double bridgeMin[BRIDGE_MAXINPUTS], bridgeMax[BRIDGE_MAXINPUTS];

	uint8_t outOfRange[BRIDGE_MAXINPUTS];

	//Firmware bug handling
	uint8_t chEnabledBugNotValid[BRIDGE_MAXINPUTS];
	uint8_t ch0EnableOverride;
} typedef PhidgetBridgeDeviceInfo;

#endif
