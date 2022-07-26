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

#include "phidgetbase.h"
#include "device/vintdevice.h"
#include "device/hubdevice.h"

/*
 * initAfterOpen
 * sets up the initial state of an object, reading in packets from the device if needed
 * used during attach initialization - on every attach
 */
static PhidgetReturnCode CCONV
PhidgetVINTDevice_initAfterOpen(PhidgetDeviceHandle device) {

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetVINTDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetChannelHandle vintChannel;
	PhidgetReturnCode res;
	int channelIndex;
	int dataCount;
	int readPtr;

	TESTPTR(device);

	// Data Length and Channel
	if (buffer[0] & VINT_DATA_wCHANNEL) {
		dataCount = (buffer[0] & 0x3F) - 1;
		channelIndex = buffer[1];
		readPtr = 2;
	} else {
		dataCount = buffer[0] & 0x3F;
		channelIndex = 0;
		readPtr = 1;
	}

	// Length needs to be at least 1 (packet type)
	if (dataCount < 1) {
		logerr("Got an invalid data length in a vint message");
		return (EPHIDGET_UNEXPECTED);
	}

	vintChannel = getAttachedChannel(device, channelIndex);
	if (vintChannel == NULL)
		return (EPHIDGET_OK);


	res = device->vintIO->recv(vintChannel, buffer + readPtr, dataCount);
	PhidgetRelease(&vintChannel);
	return (res);
}

static PhidgetReturnCode CCONV
PhidgetVINTDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetReturnCode res;

	assert(ch);
	assert(ch->parent);
	assert(ch->parent->parent);

	switch (bp->vpkt) {
	case BP_CLOSERESET:
		// RESET the device channel
		res = sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_RESET, NULL, 0);
		if (res != EPHIDGET_OK) {
			logerr("Failed to send VINT_PACKET_TYPE_PHIDGET_RESET message: "PRC_FMT, PRC_ARGS(res));
			return (res);
		}

		// Reset the port mode
		res = PhidgetHubDevice_setPortMode(bp->iop, (PhidgetHubDeviceHandle)ch->parent->parent,
		  ch->parent->deviceInfo.hubPort, PORT_MODE_VINT_PORT);
		if (res != EPHIDGET_OK) {
			logerr("Setting Hub Port mode failed: "PRC_FMT, PRC_ARGS(res));
			return (res);
		}

		// Wait for any pending packets.
		waitForPendingPackets(ch->parent->parent, ch->parent->deviceInfo.hubPort);

		return (EPHIDGET_OK);

	case BP_OPENRESET:
		res = PhidgetHubDevice_setPortMode(bp->iop, (PhidgetHubDeviceHandle)ch->parent->parent,
		  ch->parent->deviceInfo.hubPort, ch->openInfo->hubPortMode);
		if (res != EPHIDGET_OK) {
			logerr("Setting Hub Port mode failed: "PRC_FMT, PRC_ARGS(res));
			return (res);
		}

		res = sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_RESET, NULL, 0);
		if (res != EPHIDGET_OK) {
			logerr("Failed to send VINT_PACKET_TYPE_PHIDGET_RESET message: "PRC_FMT, PRC_ARGS(res));
			return (res);
		}

		return (EPHIDGET_OK);

	case BP_ENABLE:
		res = sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_ENABLE, NULL, 0);
		if (res != EPHIDGET_OK) {
			logerr("Failed to send VINT_PACKET_TYPE_PHIDGET_ENABLE message: "PRC_FMT, PRC_ARGS(res));
			return (res);
		}
		return (EPHIDGET_OK);

	default:
		MOS_ASSERT(ch->parent->vintIO != NULL);
		return (ch->parent->vintIO->send(ch, bp));
	}
}

PhidgetReturnCode
PhidgetVINTDevice_makePacket(PhidgetVINTDeviceHandle vintDevice, PhidgetChannelHandle vintChannel,
  VINTDeviceCommand deviceCommand, VINTPacketType devicePacketType, const uint8_t *bufferIn,
  size_t bufferInLen, uint8_t *buffer, size_t *bufferLen) {
	size_t bufIndex;

	assert(vintDevice);
	assert(vintChannel);
	assert(!bufferInLen || bufferIn);
	assert(buffer);
	assert(bufferLen);
	assert(*bufferLen >= getMaxOutPacketSize((PhidgetDeviceHandle)vintDevice));

	bufIndex = 0;

	switch (deviceCommand) {
	case VINT_CMD_DATA:
		assert(bufferInLen <= VINT_MAX_OUT_PACKETSIZE);
		if (vintChannel->uniqueIndex) {
			buffer[bufIndex++] = VINT_DATA_wCHANNEL | ((uint8_t)bufferInLen + 2);
			buffer[bufIndex++] = vintChannel->uniqueIndex;
			buffer[bufIndex++] = devicePacketType;
		} else {
			buffer[bufIndex++] = (uint8_t)bufferInLen + 1;
			buffer[bufIndex++] = devicePacketType;
		}
		if (bufferIn) {
			memcpy(buffer + bufIndex, bufferIn, bufferInLen);
			bufIndex += bufferInLen;
		}
		break;

	// Supported data-less commands
	case VINT_CMD_RESET:
	case VINT_CMD_UPGRADE_FIRMWARE:
	case VINT_CMD_FIRMWARE_UPGRADE_DONE:
		buffer[bufIndex++] = deviceCommand;
		break;

	// Supported data-containing commands
	case VINT_CMD_SETSPEED2:
		assert(bufferInLen == 4);
		buffer[bufIndex++] = deviceCommand;
		memcpy(buffer + bufIndex, bufferIn, bufferInLen);
		bufIndex += bufferInLen;
		break;

	default:
		MOS_PANIC("Unexpected packet type");
	}

	*bufferLen = bufIndex;
	assert(*bufferLen <= getMaxOutPacketSize((PhidgetDeviceHandle)vintDevice));

	return (EPHIDGET_OK);
}

static void CCONV
PhidgetVINTDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetVINTDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetVINTDevice_create(PhidgetVINTDeviceHandle *phidp) {

	DEVICECREATE_BODY(VINTDevice, PHIDCLASS_VINT);

	phid->portProtocolVersion = 1;
	phid->deviceProtocolVersion = PUNK_UINT8;

	// NOTE: Default to assuming false
	phid->portSupportsSetSpeed = PFALSE;
	phid->portMaxSpeed = PUNK_UINT32;

	// NOTE: Default to assuming false
	phid->deviceSupportsSetSpeed = PFALSE;
	phid->deviceMaxSpeed = PUNK_UINT32;

	phid->vintCommSpeed = PUNK_UINT32;

	return (EPHIDGET_OK);
}
