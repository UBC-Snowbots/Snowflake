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

#ifndef __CPHIDGETVINT
#define __CPHIDGETVINT

#include "phidget.h"
#include "macros.h"
#include "vintpackets.h"

//Max packet sizes
#define VINT_MAX_OUT_PACKETSIZE	48
#define VINT_MAX_IN_PACKETSIZE	48

//Messages
#define VINT_CMD 0x80
#define VINT_DATA 0x00

#define VINT_DATA_wCHANNEL 0x40

typedef PhidgetReturnCode (*vintsend_t)(PhidgetChannelHandle, BridgePacket *);
typedef PhidgetReturnCode (*vintrecv_t)(PhidgetChannelHandle, const uint8_t *, size_t);

typedef struct {
	const vintsend_t send;
	const vintrecv_t recv;
} VINTIO_t;

const VINTIO_t * const getVINTIO(unsigned int uid);

//PC -> VINT device Commands / Data
typedef enum {
	// VINT1
	VINT_CMD_DATA					= VINT_DATA,
	VINT_CMD_RESET 					= (VINT_CMD | 0x03),
	VINT_CMD_UPGRADE_FIRMWARE		= (VINT_CMD | 0x0B),
	VINT_CMD_FIRMWARE_UPGRADE_DONE	= (VINT_CMD | 0x0C),
	// VINT2
	VINT_CMD_SETSPEED2				= (VINT_CMD | 0x0F)
} VINTDeviceCommand;

typedef struct _PhidgetVINTConnection {
	PHIDGET_STRUCT_START
} PhidgetVINTConnection, *PhidgetVINTConnectionHandle;

//Messages going to PC in the data stream
#define VINT_MSG_ATTACH	0x80
#define VINT_MSG_DETACH	0x81

//VINT Properties
// [7-5: length],[4-0: ID]
// Property IDs are 0x00 - 0x1F
// Top 3 bits are number of data bytes following property

// 1 data byte - 0 means vint powered, 1 means self powered
#define VINT_PROP_POWERSOURCE			0x00
// 1 data byte - bit0 is means isolation, bit1 is means high-speed isolation
#define VINT_PROP_ISOLATION				0x01
// 0 data bytes - means that the communication speed can be set
#define VINT_PROP_SETSPEEDSUPPORT		0x02
// 4 data bytes (uint32_t) - max supported VINT speed in Hz
#define VINT_PROP_SETSPEEDLIMIT			0x03


PhidgetReturnCode sendVINTPacket(mosiop_t iop, PhidgetChannelHandle channel, VINTDeviceCommand command,
  VINTPacketType devicePacketType, const uint8_t *buffer, size_t bufferLen);
PhidgetReturnCode sendVINTDataPacket(mosiop_t iop, PhidgetChannelHandle channel,
  VINTPacketType devicePacketType, const uint8_t *buffer, size_t bufferLen);
PhidgetReturnCode sendVINTDataPacketTransaction(mosiop_t iop, PhidgetChannelHandle channel,
  VINTPacketType devicePacketType, const uint8_t *buffer, size_t bufferLen, PhidgetTransactionHandle trans);

PhidgetReturnCode scanVintDevices(PhidgetDeviceHandle);
PhidgetReturnCode scanVintDevice(PhidgetDeviceHandle, int, int, int, int);

PhidgetReturnCode PhidgetVINTConnectionCreate(PhidgetVINTConnectionHandle *phid);
PhidgetVINTConnectionHandle PhidgetVINTConnectionCast(void *);

const char * Phidget_strVINTPacketStatusCode(VINTPacketStatusCode code);
const char * Phidget_strPhidgetHub_PortMode(PhidgetHub_PortMode mode);

PhidgetReturnCode Phidget_setHubPortSpeed_internal(mosiop_t iop, PhidgetChannelHandle channel, uint32_t speed);

#endif
