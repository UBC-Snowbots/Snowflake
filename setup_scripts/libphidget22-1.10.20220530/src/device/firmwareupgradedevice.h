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

#ifndef __CPHIDGETFIRMWAREUPGRADEDEVICE
#define __CPHIDGETFIRMWAREUPGRADEDEVICE

typedef struct _PhidgetFirmwareUpgradeDevice *PhidgetFirmwareUpgradeDeviceHandle;
PhidgetReturnCode PhidgetFirmwareUpgradeDevice_create(PhidgetFirmwareUpgradeDeviceHandle *phid);

struct _PhidgetFirmwareUpgradeDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.firmwareupgrade
	PhidgetDevice phid;

	double progress[1];
} typedef PhidgetFirmwareUpgradeDeviceInfo;

#endif
