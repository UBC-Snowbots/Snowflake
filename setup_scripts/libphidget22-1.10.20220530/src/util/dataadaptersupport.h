/*
* This file is part of libphidget22
*
* Copyright 2020 Phidgets Inc <patrick@phidgets.com>
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

#ifndef __DATAADAPTERSUPPORT
#define __DATAADAPTERSUPPORT



#define DATAADAPTER_MAX_PACKET_LENGTH 8192
#define USB_OUT_PACKET_LENGTH 64

#define USB_OUT_PACKET_OVERHEAD 5
#define USB_IN_PACKET_OVERHEAD 5
#define NEW_PACKET_FLAG 0x8000
#define WAIT_RESP_FLAG 0x4000

#define NEW_RX_READY 0xFFFF
#define NO_ACTIVE_PACKET 0xFFFF
#define ANONYMOUS_PACKET_ID 0x0000

#define DATAADAPTER_MAX_EOL_LENGTH 8

PhidgetReturnCode sendData(PhidgetChannelHandle ch, BridgePacket* bp, int waitResponse);
PhidgetReturnCode sendI2CData(PhidgetChannelHandle ch, BridgePacket* bp, int waitResponse);
PhidgetReturnCode sendDataBuffer(PhidgetChannelHandle ch, size_t len, const uint8_t *buffer, BridgePacket* bp, int waitResponse);


PhidgetReturnCode parseI2CFormat(PhidgetChannelHandle ch, const char *string);

typedef enum DataAdapter_TXDroppedReason {
	TX_DROPPED_UNKNOWN = 0,
	TX_DROPPED_TIMEOUT = 1,
	TX_DROPPED_CORRUPT = 2,
	TX_DROPPED_BUSY = 3,
	TX_DROPPED_NOT_CONFIGURED = 4
} DataAdapter_TXDroppedReason;

typedef struct {

	/* Public Members */
	double baudRate;
	uint8_t lastData[DATAADAPTER_MAX_PACKET_LENGTH];
	size_t lastDataLength;


	/* Private Members */
	uint16_t usbInPacketCount;
	uint32_t packetID;
	uint16_t rxPacketID;

	uint16_t ackID;

	uint16_t rxPacketError;

	mos_mutex_t sendLock;
	mos_mutex_t receiveLock;

	uint16_t droppedPacketID;
	DataAdapter_TXDroppedReason droppedPacketReason;

	uint8_t nakFlag;

	uint8_t storedPacket[8192];
	size_t storedPacketLength;

	PhidgetDataAdapter_Protocol protocol;
	PhidgetDataAdapter_HandshakeMode handshakeMode;
	uint8_t i2cFormatList[128];
	char i2cFormatCount;
	uint32_t address;
	uint16_t txTimeout;

} PhidgetDataAdapterSupport, *PhidgetDataAdapterSupportHandle;

void PhidgetDataAdapterSupport_free(PhidgetDataAdapterSupportHandle *arg);
PhidgetReturnCode PhidgetDataAdapterSupport_create(PhidgetDataAdapterSupportHandle *ir);
void PhidgetDataAdapterSupport_init(PhidgetDataAdapterSupportHandle ir);
PhidgetReturnCode PhidgetDataAdapterSupport_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp);
PhidgetReturnCode PhidgetDataAdapterSupport_dataInput(PhidgetChannelHandle ch, const uint8_t *buf, size_t len);

#endif
