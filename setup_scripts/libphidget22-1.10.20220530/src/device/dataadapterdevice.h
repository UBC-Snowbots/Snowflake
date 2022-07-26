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

#ifndef __CPHIDGETDATAADAPTERDEVICE
#define __CPHIDGETDATAADAPTERDEVICE



//DIGITAL OUTPUT IN
#define STATE_CHANGE 0x0C
#define STATE_INVALID 0x07

//Constants
#define DATAADAPTER_MAXINPUTS 8

typedef struct _PhidgetDataAdapterDevice *PhidgetDataAdapterDeviceHandle;
PhidgetReturnCode PhidgetDataAdapterDevice_create(PhidgetDataAdapterDeviceHandle *phid);

#include "util/dataadaptersupport.h"

PhidgetReturnCode checkIOValid(PhidgetChannelHandle ch);

struct _PhidgetDataAdapterDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.dataadapter
	
	PhidgetDevice phid;

	/* Public Members */
	double baudRate;

	uint8_t inputState[DATAADAPTER_MAXINPUTS];

	/* Private Members */
	uint8_t inputsEnabled;
	uint8_t outputsEnabled;
	uint8_t stateInvalidSent;
	uint8_t lockedPins;
	uint32_t address;

	PhidgetDataAdapter_Protocol protocol;
	PhidgetDataAdapter_HandshakeMode handshakeMode;

} typedef PhidgetDataAdapterDeviceInfo;

#endif
