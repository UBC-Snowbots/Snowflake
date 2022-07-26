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

#ifndef __CPHIDGETVINTDEVICE
#define __CPHIDGETVINTDEVICE

typedef struct _PhidgetVINTDevice *PhidgetVINTDeviceHandle;
PhidgetReturnCode PhidgetVINTDevice_create(PhidgetVINTDeviceHandle *phid);

PhidgetReturnCode PhidgetVINTDevice_makePacket(
	PhidgetVINTDeviceHandle		vintDevice,
	PhidgetChannelHandle		vintChannel,
	VINTDeviceCommand			deviceCommand,
	VINTPacketType				devicePacketType,
	const uint8_t				*bufferIn,
	size_t						bufferInLen,
	uint8_t						*buffer,
	size_t						*bufferLen);

struct _PhidgetVINTDevice {
	PhidgetDevice phid;

	// VINT Protocol version
	uint8_t deviceProtocolVersion;
	uint8_t deviceSupportsSetSpeed;
	uint32_t deviceMaxSpeed;

	// XXX - these are redundant as they could be read out of the hub device structure
	uint8_t portProtocolVersion;
	uint8_t portSupportsSetSpeed;
	uint32_t portMaxSpeed;

	// The communication speed in Hz
	uint32_t vintCommSpeed;
} typedef PhidgetVINTDeviceInfo;

#endif
