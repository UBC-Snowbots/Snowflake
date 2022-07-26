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

#ifndef __CPHIDGETGENERICDEVICE
#define __CPHIDGETGENERICDEVICE

typedef struct _PhidgetGenericDevice *PhidgetGenericDeviceHandle;
PhidgetReturnCode PhidgetGenericDevice_create(PhidgetGenericDeviceHandle *phid);

struct _PhidgetGenericDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.generic
	PhidgetDevice phid;

	/* Public Members */

	uint32_t INPacketLength[1];
	uint32_t OUTPacketLength[1];
	uint32_t CTRPacketLength[1];

	/* Private Members */

} typedef PhidgetGenericDeviceInfo;

#endif
