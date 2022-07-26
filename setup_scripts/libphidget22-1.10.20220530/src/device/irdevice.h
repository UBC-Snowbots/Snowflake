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

#ifndef __CPHIDGETIRDEVICE
#define __CPHIDGETIRDEVICE

typedef struct _PhidgetIRDevice *PhidgetIRDeviceHandle;
PhidgetReturnCode PhidgetIRDevice_create(PhidgetIRDeviceHandle *phid);

#include "util/irsupport.h"

struct _PhidgetIRDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.ir
	PhidgetDevice phid;

} typedef PhidgetIRDeviceInfo;

#endif
