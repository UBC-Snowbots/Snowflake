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

#include "phidgetbase.h"
#include "util/dataadaptersupport.h"

// Access the PhidgetIRSupport struct via the channel private pointer
#define DATAADAPTER_SUPPORT(ch) ((PhidgetDataAdapterSupportHandle)(((PhidgetChannelHandle)(ch))->private))

static PhidgetReturnCode
SetNAK(PhidgetChannelHandle ch) {

	PhidgetLock(ch);
	DATAADAPTER_SUPPORT(ch)->nakFlag = 1;
	PhidgetBroadcast(ch);
	PhidgetUnlock(ch);
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
ClearNAK(PhidgetChannelHandle ch) {

	PhidgetLock(ch);
	DATAADAPTER_SUPPORT(ch)->nakFlag = 0;
	PhidgetBroadcast(ch);
	PhidgetUnlock(ch);
	return (EPHIDGET_OK);
}

// === Internal Functions === //
static PhidgetReturnCode
processDataPackets(PhidgetChannelHandle ch, const uint8_t *buffer, size_t length) {
	PhidgetDataAdapterSupportHandle dataAdapterSupport = DATAADAPTER_SUPPORT(ch);
	uint16_t receivedPacketCount;
	PhidgetReturnCode ret;
	size_t remPacketLength = 0;
	size_t usedPacketLength = 0;
	int endPacket = 0;
	int packetOverrun = 0;
	int error = 0;
	int packetOverhead = USB_IN_PACKET_OVERHEAD;
	PhidgetDataAdapter_PacketErrorCode errorCode = PACKET_ERROR_OK;
	int newResponsePacket = 0;
	uint16_t packetInfo;
	uint16_t packetID;

	switch (buffer[0]) {
	case VINT_PACKET_TYPE_DATAADAPTER_PACKET_SYNC:
		dataAdapterSupport->usbInPacketCount = unpack16(&buffer[1]);
		//This clears all packets received before the reset packet was able to make it to the device
		// - Needed for when the device wasn't properly closed
		if (dataAdapterSupport->storedPacketLength) {
			ret = bridgeSendToChannel(ch, BP_DATAIN, "%*R%u%u%uh", dataAdapterSupport->storedPacketLength, dataAdapterSupport->storedPacket, 1, 0, dataAdapterSupport->rxPacketID);
			dataAdapterSupport->storedPacketLength = 0;
		}
		dataAdapterSupport->rxPacketError = 0;
		dataAdapterSupport->droppedPacketID = NO_ACTIVE_PACKET;
		dataAdapterSupport->rxPacketID = NEW_RX_READY;

		return (EPHIDGET_OK);
	case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA_ERROR:
		error = 1;  //we intend to fall through to the next here
		errorCode = buffer[5];
		packetOverhead++;
	case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA_END:
		endPacket = 1;
	case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DATA:
		//Verify the validity of the packet
		receivedPacketCount = unpack16(&buffer[1]);
		packetInfo = unpack16(&buffer[3]);
		packetID = packetInfo & 0x3FFF;
		newResponsePacket = ((packetInfo & NEW_PACKET_FLAG) != 0);

		if (dataAdapterSupport->usbInPacketCount != receivedPacketCount || (newResponsePacket && (dataAdapterSupport->storedPacketLength != 0))) {
			//Send old data to channel
			if (!newResponsePacket) {// we missed the start of this packet, it is in error
				error = 1;
				errorCode = PACKET_ERROR_CORRUPT;
			}
			ret = bridgeSendToChannel(ch, BP_DATAIN, "%*R%u%u%uh", dataAdapterSupport->storedPacketLength, dataAdapterSupport->storedPacket, errorCode, (dataAdapterSupport->rxPacketID != ANONYMOUS_PACKET_ID), dataAdapterSupport->rxPacketID);
			dataAdapterSupport->storedPacketLength = 0;
		}

		if (dataAdapterSupport->rxPacketID == NEW_RX_READY) {
			dataAdapterSupport->rxPacketID = packetID;
			dataAdapterSupport->rxPacketError = 0;
		}

		dataAdapterSupport->rxPacketError |= error;

		if (packetID != dataAdapterSupport->rxPacketID && !newResponsePacket) {
			//MOS_PANIC("TODO: Something got out of sequence");
			//Send old data to channel with an error flag
			error = 1;
			errorCode = PACKET_ERROR_CORRUPT;
			ret = bridgeSendToChannel(ch, BP_DATAIN, "%*R%u%u%uh", dataAdapterSupport->storedPacketLength, dataAdapterSupport->storedPacket, errorCode, (dataAdapterSupport->rxPacketID != ANONYMOUS_PACKET_ID), dataAdapterSupport->rxPacketID);
			dataAdapterSupport->storedPacketLength = 0;
		}

		//handle the packet
		dataAdapterSupport->usbInPacketCount = receivedPacketCount + 1;
		dataAdapterSupport->lastDataLength = length - packetOverhead;
		memcpy(dataAdapterSupport->lastData, buffer + packetOverhead, dataAdapterSupport->lastDataLength);

		if (dataAdapterSupport->storedPacketLength + dataAdapterSupport->lastDataLength <= DATAADAPTER_MAX_PACKET_LENGTH) {
			memcpy(&dataAdapterSupport->storedPacket[dataAdapterSupport->storedPacketLength], dataAdapterSupport->lastData, dataAdapterSupport->lastDataLength);
			dataAdapterSupport->storedPacketLength += (uint16_t)dataAdapterSupport->lastDataLength;
			if ((endPacket == 0))// && (dataAdapterSupport->protocol != PROTOCOL_RS422 && dataAdapterSupport->protocol != PROTOCOL_RS485)) //send async packets immediately, store up syncronous ones
				return EPHIDGET_OK;
		} else {
			usedPacketLength = (DATAADAPTER_MAX_PACKET_LENGTH - dataAdapterSupport->storedPacketLength);
			memcpy(&dataAdapterSupport->storedPacket[dataAdapterSupport->storedPacketLength], dataAdapterSupport->lastData, usedPacketLength);
			dataAdapterSupport->storedPacketLength = DATAADAPTER_MAX_PACKET_LENGTH;

			remPacketLength = dataAdapterSupport->lastDataLength - usedPacketLength;
			packetOverrun = 1;
		}
		//Send data to channel
		ret = bridgeSendToChannel(ch, BP_DATAIN, "%*R%u%u%uh", dataAdapterSupport->storedPacketLength, dataAdapterSupport->storedPacket, errorCode, 0, packetID);

		if (ret != EPHIDGET_NOSPC) {
			dataAdapterSupport->storedPacketLength = 0;
		}

		dataAdapterSupport->rxPacketID = NEW_RX_READY;

		if (packetOverrun && ret == EPHIDGET_OK) {
			if (endPacket == 0) {
				memcpy(&dataAdapterSupport->storedPacket, &dataAdapterSupport->lastData[dataAdapterSupport->lastDataLength - remPacketLength], remPacketLength);
				dataAdapterSupport->storedPacketLength += remPacketLength;
			}
			else
				ret = bridgeSendToChannel(ch, BP_DATAIN, "%*R%u%u%uh", dataAdapterSupport->lastDataLength, dataAdapterSupport->lastData, errorCode, 0, packetID);
		}

		return (EPHIDGET_OK);
	case VINT_PACKET_TYPE_DATAADAPTER_PACKET_TIMEOUT:
		packetInfo = unpack16(&buffer[1]);
		packetID = packetInfo & 0x3FFF;
		dataAdapterSupport->rxPacketID = NEW_RX_READY;
		ret = bridgeSendToChannel(ch, BP_DATAIN, "%*R%u%u%uh", 0, dataAdapterSupport->storedPacket, 1, 0, packetID);
		return (EPHIDGET_OK);
	case VINT_PACKET_TYPE_DATAADAPTER_PACKET_DROPPED:
		PhidgetLock(ch);
		dataAdapterSupport->droppedPacketID = unpack16(&buffer[1]);
		dataAdapterSupport->droppedPacketReason = buffer[3];
		PhidgetBroadcast(ch);
		PhidgetUnlock(ch);
		//Could add packet ID here for faster acknowledgement of rejection, but for now the corresponding send will time out.
		//SEND_ERROR_EVENT(ch, EEPHIDGET_PACKETLOST, "One or more of the transmitted packets were lost.");
		return (EPHIDGET_OK);
	case VINT_PACKET_TYPE_DATAADAPTER_PACKET_ACK:
		PhidgetLock(ch);
		dataAdapterSupport->ackID = (unpack16(&buffer[1])) & 0x3FFF;
		PhidgetBroadcast(ch);
		PhidgetUnlock(ch);
		ClearNAK(ch);
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

//dataInput - parses device packets
PhidgetReturnCode CCONV
PhidgetDataAdapterSupport_dataInput(PhidgetChannelHandle ch, const uint8_t *buffer, size_t length) {

	assert(ch);
	assert(buffer);

	//Parse device packets - store data locally
	switch (ch->UCD->uid) {
#if (PHIDUID_ADP1001_USB_SUPPORTED || PHIDUID_ADP1001_VINT_SUPPORTED || PHIDUID_ADP_RS485_422_USB_SUPPORTED \
	|| PHIDUID_ADP_RS485_422_VINT_SUPPORTED || PHIDUID_ADP_SERIAL_USB_SUPPORTED || PHIDUID_ADP_SERIAL_VINT_SUPPORTED)
#if PHIDUID_ADP1001_USB_SUPPORTED
	case PHIDCHUID_ADP1001_DATAADAPTER_100_USB:
#endif
#if PHIDUID_ADP1001_VINT_SUPPORTED
	case PHIDCHUID_ADP1001_DATAADAPTER_100_VINT:
#endif
#if PHIDUID_ADP_RS485_422_USB_SUPPORTED
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100_USB:
#endif
#if PHIDUID_ADP_RS485_422_VINT_SUPPORTED
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100_VINT:
#endif
#if PHIDUID_ADP_SERIAL_USB_SUPPORTED
	case PHIDCHUID_ADP_SERIAL_DATAADAPTER_100_USB:
#endif
#if PHIDUID_ADP_SERIAL_VINT_SUPPORTED
	case PHIDCHUID_ADP_SERIAL_DATAADAPTER_100_VINT:
#endif
		if (length > 0) {
			switch (buffer[0]) {
			default:
				return processDataPackets(ch, buffer, length);
			}
		}
		MOS_PANIC("Unexpected packet type");
#endif /* None supported */
	default:
		MOS_PANIC("Unexpected device");
	}

}

PhidgetReturnCode CCONV
PhidgetDataAdapterSupport_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetDeviceHandle device = (PhidgetDeviceHandle)ch->parent;
#if (PHIDUID_ADP1001_USB_SUPPORTED || PHIDUID_ADP1001_VINT_SUPPORTED || PHIDUID_ADP_RS485_422_USB_SUPPORTED \
	|| PHIDUID_ADP_RS485_422_VINT_SUPPORTED || PHIDUID_ADP_SERIAL_USB_SUPPORTED || PHIDUID_ADP_SERIAL_VINT_SUPPORTED)
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	PhidgetReturnCode ret;
	size_t len;
#endif

	PhidgetDataAdapterSupportHandle dataAdapterSupport = DATAADAPTER_SUPPORT(ch);

	switch (ch->UCD->uid) {
#if PHIDUID_ADP1001_USB_SUPPORTED
	case PHIDCHUID_ADP1001_DATAADAPTER_100_USB:
		switch (ch->class) {
		case PHIDCHCLASS_DATAADAPTER:
			switch (bp->vpkt) {
			case BP_DATAOUT:
				ret = sendData(ch, bp, 0);
				return ret;
			case BP_DATAEXCHANGE:
				ret = sendData(ch, bp, 1);
				return ret;
			case BP_SETBAUDRATE:
				len = 4;
				pack32(buffer, getBridgePacketUInt32(bp, 0));
				ret = PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_BAUD_RATE, 0, buffer, &len, 100);
				if (ret == EPHIDGET_OK)
					dataAdapterSupport->baudRate = getBridgePacketUInt32(bp, 0);
				return ret;
			case BP_SETPARITY:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_PARITY_MODE, 0, buffer, &len, 100);
			case BP_SETSTOPBITS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_STOP_BITS, 0, buffer, &len, 100);
			case BP_SETDATABITS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_DATA_BITS, 0, buffer, &len, 100);
			case BP_SETHANDSHAKEMODE:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_HANDSHAKE_MODE, 0, buffer, &len, 100);
			case BP_SETTIMEOUT:
				len = 2;
				pack16(&buffer[0], (uint16_t)getBridgePacketUInt32(bp, 0));
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_TIMEOUT, 0, buffer, &len, 100);
			case BP_SETTXTIMEOUT:
				len = 2;
				dataAdapterSupport->txTimeout = (uint16_t)getBridgePacketUInt32(bp, 0);
				pack16(&buffer[0], dataAdapterSupport->txTimeout);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_TX_TIMEOUT, 0, buffer, &len, 100);
			case BP_OPENRESET:
				len = 0;
				dataAdapterSupport->protocol = PROTOCOL_RS232;
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, 0, buffer, &len, 100);
			case BP_CLOSERESET:
				len = 0;
				dataAdapterSupport->protocol = PUNK_ENUM;
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, 0, buffer, &len, 100);
			case BP_ENABLE:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, 0, buffer, &len, 100);
			case BP_SETENDIANNESS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_ENDIANNESS, 0, buffer, &len, 100);
			case BP_SETENDOFLINE:
				return EPHIDGET_OK;
			default:
				MOS_PANIC("Unexpected packet type");
			}
		default:
			MOS_PANIC("Unexpected Channel Class");
		}
#endif /* PHIDUID_ADP1001_USB_SUPPORTED */
#if PHIDUID_ADP1001_VINT_SUPPORTED
	case PHIDCHUID_ADP1001_DATAADAPTER_100_VINT:
		switch (ch->class) {
		case PHIDCHCLASS_DATAADAPTER:
			switch (bp->vpkt) {
			case BP_DATAOUT:
				ret = sendData(ch, bp, 0);
				return ret;
			case BP_DATAEXCHANGE:
				ret = sendData(ch, bp, 1);
				return ret;
			case BP_SETBAUDRATE:
				len = 4;
				pack32(buffer, getBridgePacketUInt32(bp, 0));
				ret = sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_BAUD_RATE, buffer, len);
				if (ret == EPHIDGET_OK)
					dataAdapterSupport->baudRate = getBridgePacketUInt32(bp, 0);
				return ret;
			case BP_OPENRESET:
				len = 0;
				dataAdapterSupport->protocol = PROTOCOL_RS232;
				return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_RESET, buffer, len);
			case BP_SETPARITY:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_PARITY_MODE, buffer, len);
			case BP_SETSTOPBITS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_STOP_BITS, buffer, len);
			case BP_SETDATABITS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
				return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_DATA_BITS, buffer, len);
			case BP_SETHANDSHAKEMODE:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_HANDSHAKE_MODE, buffer, len);
			case BP_SETTIMEOUT:
				len = 2;
				pack16(&buffer[0], (uint16_t)getBridgePacketUInt32(bp, 0));
				return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_TIMEOUT, buffer, len);
			case BP_SETTXTIMEOUT:
				len = 2;
				dataAdapterSupport->txTimeout = (uint16_t)getBridgePacketUInt32(bp, 0);
				pack16(&buffer[0], dataAdapterSupport->txTimeout);
				return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_TX_TIMEOUT, buffer, len);
			case BP_CLOSERESET:
				len = 0;
				dataAdapterSupport->protocol = PUNK_ENUM;
				return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_RESET, buffer, len);
			case BP_ENABLE:
				len = 0;
				return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_ENABLE, buffer, len);
			case BP_SETENDIANNESS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_ENDIANNESS, buffer, len);
			case BP_SETENDOFLINE:
				return EPHIDGET_OK;
			default:
				MOS_PANIC("Unexpected packet type");
			}
		default:
			MOS_PANIC("Unexpected Channel Class");
		}
#endif /* PHIDUID_ADP1001_VINT_SUPPORTED */
#if PHIDUID_ADP_RS485_422_USB_SUPPORTED
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100_USB:
		switch (bp->vpkt) {
		case BP_DATAOUT:
			ret = sendData(ch, bp, 0);
			return ret;
		case BP_DATAEXCHANGE:
			ret = sendData(ch, bp, 1);
			return ret;
		case BP_SETBAUDRATE:
			len = 4;
			pack32(buffer, getBridgePacketUInt32(bp, 0));
			ret = PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_BAUD_RATE, 0, buffer, &len, 100);
			if (ret == EPHIDGET_OK)
				dataAdapterSupport->baudRate = getBridgePacketUInt32(bp, 0);
			return ret;
		case BP_SETPARITY:
			len = 1;
			if (dataAdapterSupport->protocol == PROTOCOL_DMX512)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "DMX512 Protocol does not support Parity bits"));
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_PARITY_MODE, 0, buffer, &len, 100);
		case BP_SETSTOPBITS:
			len = 1;
			if (dataAdapterSupport->protocol == PROTOCOL_MODBUS_RTU || dataAdapterSupport->protocol == PROTOCOL_DMX512)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "MODBUS RTU and DMX512 Protocols have a fixed number of Stop Bits"));
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_STOP_BITS, 0, buffer, &len, 100);
		case BP_SETDATABITS:
			len = 1;
			if (dataAdapterSupport->protocol == PROTOCOL_MODBUS_RTU || dataAdapterSupport->protocol == PROTOCOL_DMX512)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "MODBUS RTU and DMX512 Protocols have a fixed number of Data Bits"));
			buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
			return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_DATA_BITS, 0, buffer, &len, 100);
		case BP_OPENRESET:
		case BP_CLOSERESET:
			len = 0;
			return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, 0, buffer, &len, 100);
		case BP_ENABLE:
			len = 0;
			return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, 0, buffer, &len, 100);

		case BP_SETPROTOCOL:
			dataAdapterSupport->protocol = (uint8_t)getBridgePacketInt32(bp, 0);
			len = 1;
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_PROTOCOL, 0, buffer, &len, 100);
		case BP_SETTIMEOUT:
			len = 2;
			pack16(&buffer[0], (uint16_t)getBridgePacketUInt32(bp, 0));
			return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_TIMEOUT, 0, buffer, &len, 100);
		case BP_SETTXTIMEOUT:
			len = 2;
			dataAdapterSupport->txTimeout = (uint16_t)getBridgePacketUInt32(bp, 0);
			pack16(&buffer[0], dataAdapterSupport->txTimeout);
			return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_TX_TIMEOUT, 0, buffer, &len, 100);
		case BP_SETENDIANNESS:
			len = 1;
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_ENDIANNESS, 0, buffer, &len, 100);
		default:
			MOS_PANIC("Unexpected packet type");
		}
#endif /* PHIDUID_ADP_RS485_422_USB_SUPPORTED */
#if PHIDUID_ADP_RS485_422_VINT_SUPPORTED
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100_VINT:
		switch (bp->vpkt) {
		case BP_DATAOUT:
			ret = sendData(ch, bp, 0);
			return ret;
		case BP_DATAEXCHANGE:
			ret = sendData(ch, bp, 1);
			return ret;
		case BP_SETBAUDRATE:
			len = 4;
			pack32(buffer, getBridgePacketUInt32(bp, 0));
			ret = sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_BAUD_RATE, buffer, len);
			if (ret == EPHIDGET_OK)
				dataAdapterSupport->baudRate = getBridgePacketUInt32(bp, 0);
			return ret;
		case BP_SETPARITY:
			len = 1;
			if (dataAdapterSupport->protocol == PROTOCOL_DMX512)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "DMX512 Protocol does not support Parity Bits"));
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_PARITY_MODE, buffer, len);
		case BP_SETSTOPBITS:
			len = 1;
			if (dataAdapterSupport->protocol == PROTOCOL_MODBUS_RTU || dataAdapterSupport->protocol == PROTOCOL_DMX512)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "MODBUS RTU and DMX512 Protocols have a fixed number of Stop Bits"));
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_STOP_BITS, buffer, len);
		case BP_SETDATABITS:
			len = 1;
			if (dataAdapterSupport->protocol == PROTOCOL_MODBUS_RTU || dataAdapterSupport->protocol == PROTOCOL_DMX512)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "MODBUS RTU and DMX512 Protocols have a fixed number of Data Bits"));
			buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_DATA_BITS, buffer, len);
		case BP_OPENRESET:
		case BP_CLOSERESET:
			len = 0;
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_RESET, buffer, len);
		case BP_ENABLE:
			len = 0;
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_ENABLE, buffer, len);

		case BP_SETPROTOCOL:
			dataAdapterSupport->protocol = (uint8_t)getBridgePacketInt32(bp, 0);
			len = 1;
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_PROTOCOL, buffer, len);
		case BP_SETTIMEOUT:
			len = 2;
			pack16(&buffer[0], (uint16_t)getBridgePacketUInt32(bp, 0));
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_TIMEOUT, buffer, len);
		case BP_SETTXTIMEOUT:
			len = 2;
			dataAdapterSupport->txTimeout = (uint16_t)getBridgePacketUInt32(bp, 0);
			pack16(&buffer[0], dataAdapterSupport->txTimeout);
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_TX_TIMEOUT, buffer, len);
		case BP_SETENDIANNESS:
			len = 1;
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_ENDIANNESS, buffer, len);
		default:
			MOS_PANIC("Unexpected packet type");
		}
#endif /* PHIDUID_ADP_RS485_422_VINT_SUPPORTED */
#if PHIDUID_ADP_SERIAL_USB_SUPPORTED
	case PHIDCHUID_ADP_SERIAL_DATAADAPTER_100_USB:
		switch (ch->class) {
		case PHIDCHCLASS_DATAADAPTER:
			switch (bp->vpkt) {
			case BP_DATAOUT:
				if (dataAdapterSupport->protocol == PROTOCOL_I2C)
					ret = sendI2CData(ch, bp, 1);
				else if (dataAdapterSupport->protocol == PROTOCOL_UART)
					ret = sendData(ch, bp, 0);
				else
					ret = sendData(ch, bp, 1);

				return ret;
			case BP_DATAEXCHANGE:
				if (dataAdapterSupport->protocol == PROTOCOL_I2C)
					ret = sendI2CData(ch, bp, 1);
				else
					ret = sendData(ch, bp, 1);
				return ret;
			case BP_SETBAUDRATE:
				len = 4;
				pack32(buffer, getBridgePacketUInt32(bp, 0));
				ret = PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_BAUD_RATE, 0, buffer, &len, 100);
				if (ret == EPHIDGET_OK)
					dataAdapterSupport->baudRate = getBridgePacketUInt32(bp, 0);
				return ret;
			case BP_OPENRESET:
				len = 0;
				dataAdapterSupport->protocol = PUNK_ENUM;
				dataAdapterSupport->handshakeMode = HANDSHAKE_MODE_NONE;
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, 0, buffer, &len, 100);
			case BP_CLOSERESET:
				len = 0;
				dataAdapterSupport->protocol = PUNK_ENUM;
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, 0, buffer, &len, 100);
			case BP_ENABLE:
				len = 0;
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_PHIDGET_ENABLE, 0, buffer, &len, 100);

			case BP_SETDATABITS:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_DATA_BITS, 0, buffer, &len, 100);
			case BP_SETPROTOCOL:
				dataAdapterSupport->protocol = (uint8_t)getBridgePacketInt32(bp, 0);
				/*switch (dataAdapterSupport->protocol) {
				case PROTOCOL_SPI:
					if ((phid->outputsEnabled | phid->inputsEnabled) & ((1 << 0) | (1 << 1) | (1 << 2)))
						return EPHIDGET_NOTCONFIGURED;
					break;
				case PROTOCOL_I2C:
					if ((phid->outputsEnabled | phid->inputsEnabled) & ((1 << 3) | (1 << 4)))
						return EPHIDGET_NOTCONFIGURED;
					break;
				case PROTOCOL_UART:
					if ((phid->outputsEnabled | phid->inputsEnabled) & ((1 << 1) | (1 << 2)))
						return EPHIDGET_NOTCONFIGURED;
					if (phid->handshakeMode == HANDSHAKE_MODE_READY_TO_RECEIVE || phid->handshakeMode == HANDSHAKE_MODE_REQUEST_TO_SEND) {
						if (ch->uniqueIndex == 3 && ch->uniqueIndex == 4)
							return EPHIDGET_NOTCONFIGURED;
					}
					break;
				}*/

				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_PROTOCOL, 0, buffer, &len, 100);
			case BP_SETHANDSHAKEMODE:
				len = 1;
				if (dataAdapterSupport->protocol != PROTOCOL_UART)
					return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Handshake Mode only supported by UART Protocol"));
				dataAdapterSupport->handshakeMode = (uint8_t)getBridgePacketInt32(bp, 0);
				buffer[0] = dataAdapterSupport->handshakeMode;
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_HANDSHAKE_MODE, 0, buffer, &len, 100);
			case BP_SETSPIMODE:
				len = 1;
				if (dataAdapterSupport->protocol != PROTOCOL_SPI)
					return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "SPI Mode only supported by SPI Protocol"));
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_SPIMODE, 0, buffer, &len, 100);
			case BP_SETADDRESS:
				if (dataAdapterSupport->protocol != PROTOCOL_SPI && dataAdapterSupport->protocol != PROTOCOL_I2C)
					return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Address only supported by SPI and I2C Protocols"));
				if (dataAdapterSupport->protocol != PROTOCOL_I2C) {
					len = 1;
					buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
					return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_ADDRESS, 0, buffer, &len, 100);
				} else {
					dataAdapterSupport->address = getBridgePacketUInt32(bp, 0);
					return EPHIDGET_OK;
				}
			case BP_SETENDIANNESS:
				if (dataAdapterSupport->protocol == PROTOCOL_I2C)
					return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Endianness is fixed for I2C Protocol"));
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_ENDIANNESS, 0, buffer, &len, 100);
			case BP_SETIOVOLTAGE:
				len = 1;
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_IOVOLTAGE, 0, buffer, &len, 100);
			case BP_SETTIMEOUT:
				len = 2;
				pack16(&buffer[0], (uint16_t)getBridgePacketUInt32(bp, 0));
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_TIMEOUT, 0, buffer, &len, 100);
			case BP_SETTXTIMEOUT:
				len = 2;
				dataAdapterSupport->txTimeout = (uint16_t)getBridgePacketUInt32(bp, 0);
				pack16(&buffer[0], dataAdapterSupport->txTimeout);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_TX_TIMEOUT, 0, buffer, &len, 100);
			case BP_SETPARITY:
				len = 1;
				if (dataAdapterSupport->protocol != PROTOCOL_UART)
					return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Parity Bits only supported by UART Protocol"));
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_PARITY_MODE, 0, buffer, &len, 100);
			case BP_SETSTOPBITS:
				len = 1;
				if (dataAdapterSupport->protocol != PROTOCOL_UART)
					return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Stop Bits only supported by UART Protocol"));
				buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
				return PhidgetDevice_transferpacket(bp->iop, device, PHIDGETUSB_REQ_DEVICE_WRITE, VINT_PACKET_TYPE_DATAADAPTER_STOP_BITS, 0, buffer, &len, 100);
			case BP_SETI2CFORMAT:
				ret = parseI2CFormat(ch, getBridgePacketString(bp, 0));
				if (dataAdapterSupport->protocol != PROTOCOL_I2C)
					return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "I2C Format only supported by I2C Protocol"));
				return ret;
			default:
				MOS_PANIC("Unexpected packet type");
			}
		default:
			MOS_PANIC("Unexpected Channel Class");
		}
#endif /* PHIDUID_ADP_SERIAL_USB_SUPPORTED */
#if PHIDUID_ADP_SERIAL_VINT_SUPPORTED
		case PHIDCHUID_ADP_SERIAL_DATAADAPTER_100_VINT:
			switch (ch->class) {
			case PHIDCHCLASS_DATAADAPTER:
				switch (bp->vpkt) {
				case BP_DATAOUT:
					if (dataAdapterSupport->protocol == PROTOCOL_I2C)
						ret = sendI2CData(ch, bp, 1);
					else if (dataAdapterSupport->protocol == PROTOCOL_UART)
						ret = sendData(ch, bp, 0);
					else
						ret = sendData(ch, bp, 1);

					return ret;
				case BP_DATAEXCHANGE:
					if (dataAdapterSupport->protocol == PROTOCOL_I2C)
						ret = sendI2CData(ch, bp, 1);
					else
						ret = sendData(ch, bp, 1);
					return ret;
				case BP_SETBAUDRATE:
					len = 4;
					pack32(buffer, getBridgePacketUInt32(bp, 0));
					ret = sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_BAUD_RATE, buffer, len);
					if (ret == EPHIDGET_OK)
						dataAdapterSupport->baudRate = getBridgePacketUInt32(bp, 0);
					return ret;
				case BP_OPENRESET:
					len = 0;
					dataAdapterSupport->protocol = PUNK_ENUM;
					dataAdapterSupport->handshakeMode = HANDSHAKE_MODE_NONE;
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_RESET, buffer, len);
				case BP_CLOSERESET:
					len = 0;
					dataAdapterSupport->protocol = PUNK_ENUM;
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_RESET, buffer, len);
				case BP_ENABLE:
					len = 0;
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHIDGET_ENABLE, buffer, len);

				case BP_SETDATABITS:
					len = 1;
					buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_DATA_BITS, buffer, len);
				case BP_SETPROTOCOL:
					dataAdapterSupport->protocol = (uint8_t)getBridgePacketInt32(bp, 0);
					/*switch (dataAdapterSupport->protocol) {
					case PROTOCOL_SPI:
					if ((phid->outputsEnabled | phid->inputsEnabled) & ((1 << 0) | (1 << 1) | (1 << 2)))
					return EPHIDGET_NOTCONFIGURED;
					break;
					case PROTOCOL_I2C:
					if ((phid->outputsEnabled | phid->inputsEnabled) & ((1 << 3) | (1 << 4)))
					return EPHIDGET_NOTCONFIGURED;
					break;
					case PROTOCOL_UART:
					if ((phid->outputsEnabled | phid->inputsEnabled) & ((1 << 1) | (1 << 2)))
					return EPHIDGET_NOTCONFIGURED;
					if (phid->handshakeMode == HANDSHAKE_MODE_READY_TO_RECEIVE || phid->handshakeMode == HANDSHAKE_MODE_REQUEST_TO_SEND) {
					if (ch->uniqueIndex == 3 && ch->uniqueIndex == 4)
					return EPHIDGET_NOTCONFIGURED;
					}
					break;
					}*/

					len = 1;
					buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_PROTOCOL, buffer, len);
				case BP_SETHANDSHAKEMODE:
					len = 1;
					if (dataAdapterSupport->protocol != PROTOCOL_UART)
						return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Handshake Mode only supported by UART Protocol"));
					dataAdapterSupport->handshakeMode = (uint8_t)getBridgePacketInt32(bp, 0);
					buffer[0] = dataAdapterSupport->handshakeMode;
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_HANDSHAKE_MODE, buffer, len);
				case BP_SETSPIMODE:
					len = 1;
					if (dataAdapterSupport->protocol != PROTOCOL_SPI)
						return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "SPI Mode only supported by SPI Protocol"));
					buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_SPIMODE, buffer, len);
				case BP_SETADDRESS:
					if (dataAdapterSupport->protocol != PROTOCOL_SPI && dataAdapterSupport->protocol != PROTOCOL_I2C)
						return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Address only supported by SPI and I2C Protocols"));
					dataAdapterSupport->address = getBridgePacketUInt32(bp, 0);
					if (dataAdapterSupport->protocol != PROTOCOL_I2C) {
						len = 1;
						buffer[0] = (uint8_t)getBridgePacketUInt32(bp, 0);
						return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_ADDRESS, buffer, len);
					} else {
						return EPHIDGET_OK;
					}
				case BP_SETENDIANNESS:
					if (dataAdapterSupport->protocol == PROTOCOL_I2C)
						return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Endianness is fixed for I2C Protocol"));
					len = 1;
					buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_ENDIANNESS, buffer, len);
				case BP_SETIOVOLTAGE:
					len = 1;
					buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_IOVOLTAGE, buffer, len);
				case BP_SETTIMEOUT:
					len = 2;
					pack16(&buffer[0], (uint16_t)getBridgePacketUInt32(bp, 0));
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_TIMEOUT, buffer, len);
				case BP_SETTXTIMEOUT:
					len = 2;
					dataAdapterSupport->txTimeout = (uint16_t)getBridgePacketUInt32(bp, 0);
					pack16(&buffer[0], dataAdapterSupport->txTimeout);
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_TX_TIMEOUT, buffer, len);
				case BP_SETPARITY:
					len = 1;
					if (dataAdapterSupport->protocol != PROTOCOL_UART)
						return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Parity Bits only supported by UART Protocol"));
					buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_PARITY_MODE, buffer, len);
				case BP_SETSTOPBITS:
					len = 1;
					if (dataAdapterSupport->protocol != PROTOCOL_UART)
						return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Stop Bits only supported by UART Protocol"));
					buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0);
					return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DATAADAPTER_STOP_BITS, buffer, len);
				case BP_SETI2CFORMAT:
					ret = parseI2CFormat(ch, getBridgePacketString(bp, 0));
					if (dataAdapterSupport->protocol != PROTOCOL_I2C)
						return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "I2C Format only supported by I2C Protocol"));
					return ret;
				default:
					MOS_PANIC("Unexpected packet type");
				}
			default:
				MOS_PANIC("Unexpected Channel Class");
			}
#endif /* PHIDUID_ADP_SERIAL_VINT_SUPPORTED */
	default:
		MOS_PANIC("Unexpected device");
	}
}


PhidgetReturnCode sendData(PhidgetChannelHandle ch, BridgePacket* bp, int waitResponse) {

	return sendDataBuffer(ch, bp->entry[0].len, (const uint8_t *)getBridgePacketUInt8Array(bp, 0), bp, waitResponse);

}

PhidgetReturnCode parseI2CFormat(PhidgetChannelHandle ch, const char *string) {
	int index = 0;
	int count = 0;
	int stopped = 0;
	int mode = 0; //0 transmit, 1 receive
	int totalFormatCount = 0;
	int transmitCount = 0;
	size_t i;

	uint8_t tmpFormatList[128];

	for (i = 0; i < mos_strlen(string); i++) {
		if (stopped)
			return EPHIDGET_INVALIDARG;
		switch (string[i]) {
		case 's':
			if (i == 0)
				continue;
			if (count != 0) {
				tmpFormatList[index] = count;
				if (mode)
					tmpFormatList[index] |= 0x80;

				index++;
				count = 0;
			} else
				return EPHIDGET_INVALIDARG;
			break;
		case 'T':
			if (i == 0)
				return EPHIDGET_INVALIDARG;
			if (count == 0) {
				mode = 0;
			}
			if (mode != 0) {
				return EPHIDGET_INVALIDARG;
			}
			count++;
			totalFormatCount++;
			transmitCount++;
			if (count > 127 || totalFormatCount > 256)
				return EPHIDGET_INVALIDARG;
			break;
		case 'R':
			if (i == 0)
				return EPHIDGET_INVALIDARG;
			if (count == 0) {
				mode = 1;
			}
			if (mode != 1) {
				return EPHIDGET_INVALIDARG;
			}
			count++;
			totalFormatCount++;
			if (count > 127 || totalFormatCount > 256)
				return EPHIDGET_INVALIDARG;
			break;
		case 'p':
			if (i == 0)
				return EPHIDGET_INVALIDARG;
			if (count != 0) {
				tmpFormatList[index] = count;
				if (mode)
					tmpFormatList[index] |= 0x80;



			} else
				return EPHIDGET_INVALIDARG;
			stopped = 1;
			break;
		default:
			return EPHIDGET_INVALIDARG;
		}
	}
	if (!stopped)
		return EPHIDGET_INVALIDARG;

	if((index + 1 + transmitCount + 2) > 512) //Total Transmitted bytes given the proposed format
		return EPHIDGET_INVALIDARG;

	DATAADAPTER_SUPPORT(ch)->i2cFormatCount = index + 1;
	memcpy(DATAADAPTER_SUPPORT(ch)->i2cFormatList, tmpFormatList, DATAADAPTER_SUPPORT(ch)->i2cFormatCount);

	return EPHIDGET_OK;
}

PhidgetReturnCode sendI2CData(PhidgetChannelHandle ch, BridgePacket* bp, int waitResposne) {
	uint8_t buffer[1024];
	int transmitCount = 0;
	int i;
	uint8_t dataSize = bp->entry[0].len;

	if (DATAADAPTER_SUPPORT(ch)->i2cFormatCount == 0)
		return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "I2C Format must be set brfore data can be exchanged"));

	uint8_t totalCount = dataSize + DATAADAPTER_SUPPORT(ch)->i2cFormatCount + 2;

	if (totalCount > 512)
		return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Packet is too large"));

	for (i = 0; i < DATAADAPTER_SUPPORT(ch)->i2cFormatCount; i++) {
		if (!(DATAADAPTER_SUPPORT(ch)->i2cFormatList[i] & 0x80)) //if transmit segment
			transmitCount += DATAADAPTER_SUPPORT(ch)->i2cFormatList[i] & 0x7F;
	}

	if (transmitCount != dataSize)
		return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Transmit array length does not match the length specified in I2CFormat"));

	buffer[0] = DATAADAPTER_SUPPORT(ch)->address;
	buffer[1] = DATAADAPTER_SUPPORT(ch)->i2cFormatCount;
	memcpy(&buffer[2], DATAADAPTER_SUPPORT(ch)->i2cFormatList, DATAADAPTER_SUPPORT(ch)->i2cFormatCount);
	memcpy(&buffer[DATAADAPTER_SUPPORT(ch)->i2cFormatCount + 2], (const uint8_t *)getBridgePacketUInt8Array(bp, 0), dataSize);

	return sendDataBuffer(ch, totalCount, (const uint8_t *)buffer, bp, waitResposne);
}

static PhidgetReturnCode sendTXDataVINT(mosiop_t iop, PhidgetChannelHandle ch, uint8_t *buf, size_t packetLen, PhidgetTransaction *trans){
	PhidgetReturnCode ret;
	SetNAK(ch);
	ret = sendVINTDataPacketTransaction(iop, ch, VINT_PACKET_TYPE_DATAADAPTER_TX_DATA, buf, packetLen, trans);
	//if (ret == EPHIDGET_BUSY) {

	//	ret = waitForNAKClear(iop, ch, DATAADAPTER_SUPPORT(ch)->txTimeout);
	//	if (ret != EPHIDGET_OK)
	//		return ret;

	//	/* Try again (once) */
	//	ret = sendVINTDataPacketTransaction(iop, ch, VINT_PACKET_TYPE_DATAADAPTER_TX_DATA, buf, packetLen, trans);
	//}
	if (ret != EPHIDGET_OK)
		return ret;

	return EPHIDGET_OK;
}

PhidgetReturnCode sendDataBuffer(PhidgetChannelHandle ch, size_t len, const uint8_t *buffer, BridgePacket* bp, int waitResposne) {
	PhidgetReturnCode ret;
	size_t packetLen;
	uint32_t packetCount = 0;
	PhidgetTransaction trans;
	uint16_t packetID;
	uint16_t packetInfo;
	PhidgetReturnCode res1;
	size_t i = 0;

	uint8_t buf[USB_OUT_PACKET_LENGTH];

	mostime_t duration;
	mostime_t start;
	mostime_t maxDuration;
	mostime_t remainingTime;

	PhidgetDataAdapterSupportHandle dataAdapterSupport = DATAADAPTER_SUPPORT(ch);

	if (dataAdapterSupport->txTimeout == 0)
		maxDuration = ((double)len / DATAADAPTER_SUPPORT(ch)->baudRate) * 2000000 * 10 + 100000; //2 * transmission time in us + 100ms
	else
		maxDuration = (dataAdapterSupport->txTimeout + 50) * 1000; //we want the FW to indicate a timeout before the sendData call times out

	if (dataAdapterSupport->protocol == PROTOCOL_I2C && dataAdapterSupport->address == PUNK_INT32)
		return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "I2C Requires a Device Address to be set before sending data"));
	if (dataAdapterSupport->protocol == PROTOCOL_SPI && dataAdapterSupport->address == PUNK_INT32)
		return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "SPI Requires a Device Address to be set before sending data"));

	if (DATAADAPTER_SUPPORT(ch)->baudRate == 0)
		return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Baud Rate Not Configured"));

	start = mos_gettime_usec();

	//PhidgetRunLock(ch);
	//Assign unique packet ID
	PhidgetLock(ch);
	DATAADAPTER_SUPPORT(ch)->packetID++;
	DATAADAPTER_SUPPORT(ch)->packetID &= 0x3FFF; //14-bit packet ID
	if (DATAADAPTER_SUPPORT(ch)->packetID == ANONYMOUS_PACKET_ID)
		DATAADAPTER_SUPPORT(ch)->packetID = 0x0001;
	packetID = DATAADAPTER_SUPPORT(ch)->packetID;
	PhidgetUnlock(ch);
	//PhidgetRunUnlock(ch);

	//Bridge packet reply is packet ID
	bp->reply_bpe = mos_malloc(sizeof(BridgePacketEntry));
	memset(bp->reply_bpe, 0, sizeof(BridgePacketEntry));

	bp->reply_bpe->type = BPE_UI8ARRAY;
	bp->reply_bpe->bpe_len = (uint16_t)2;
	bp->reply_bpe->bpe_ptr = mos_malloc(2);
	bp->reply_bpe->bpe_ui8array = bp->reply_bpe->bpe_ptr;
	bp->reply_bpe->bpe_cnt = (uint16_t)1;

	bp->reply_bpe->bpe_ui8array[0] = (DATAADAPTER_SUPPORT(ch)->packetID & 0xFF00) >> 8;
	int tmp = (DATAADAPTER_SUPPORT(ch)->packetID & 0xFF00) >> 8;
	tmp = tmp;
	bp->reply_bpe->bpe_ui8array[1] = DATAADAPTER_SUPPORT(ch)->packetID & 0xFF;


	//Wait until previous packets are dealt with, ensures the device will accept the packet

	ret = EPHIDGET_OK;

	//Transfer the packet
	if (ch->parent->deviceInfo.class != PHIDCLASS_VINT) {
		if (len > 0) {
			if (len <= (USB_OUT_PACKET_LENGTH - USB_OUT_PACKET_OVERHEAD)) {
				packetLen = len + USB_OUT_PACKET_OVERHEAD;
				packetInfo = NEW_PACKET_FLAG | packetID;
				if (waitResposne)
					packetInfo |= WAIT_RESP_FLAG;
				pack16(&buf[0], packetInfo);
				buf[2] = (uint8_t)(len >> 16); //pack 24
				buf[3] = (uint8_t)(len >> 8);
				buf[4] = (uint8_t)(len & 0xFF);
				memcpy(&buf[USB_OUT_PACKET_OVERHEAD], buffer, len);
				remainingTime = maxDuration - (mos_gettime_usec() - start);
				ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)ch->parent, PHIDGETUSB_REQ_BULK_WRITE, 0, 0, buf, &packetLen, remainingTime/1000);
				if (ret != EPHIDGET_OK) {
					return ret;
				}

			} else {
				for (i = 0; i + (USB_OUT_PACKET_LENGTH - USB_OUT_PACKET_OVERHEAD) < len; i += (USB_OUT_PACKET_LENGTH - USB_OUT_PACKET_OVERHEAD)) {
					packetLen = USB_OUT_PACKET_LENGTH;
					if (i == 0) {
						packetInfo = NEW_PACKET_FLAG | packetID;
						if (waitResposne)
							packetInfo |= WAIT_RESP_FLAG;
						pack16(&buf[0], packetInfo);
						buf[2] = (uint8_t)(len >> 16); //pack 24
						buf[3] = (uint8_t)(len >> 8);
						buf[4] = (uint8_t)(len & 0xFF);
					} else {
						pack16(&buf[0], packetID);
						buf[2] = (uint8_t)(packetCount >> 16); //pack 24
						buf[3] = (uint8_t)(packetCount >> 8);
						buf[4] = (uint8_t)(packetCount & 0xFF);
					}

					if (DATAADAPTER_SUPPORT(ch)->droppedPacketID == packetID) {
						DATAADAPTER_SUPPORT(ch)->droppedPacketID = NO_ACTIVE_PACKET;
						return EPHIDGET_INTERRUPTED;
					}
					memcpy(&buf[USB_OUT_PACKET_OVERHEAD], buffer + i, (packetLen - USB_OUT_PACKET_OVERHEAD));
					remainingTime = maxDuration - (mos_gettime_usec() - start);
					ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)ch->parent, PHIDGETUSB_REQ_BULK_WRITE, 0, 0, buf, &packetLen, remainingTime/1000);
					if (ret != EPHIDGET_OK) {
						return ret;
					}
					packetCount++;
				}
				if (i != len) {
					packetLen = len - i + USB_OUT_PACKET_OVERHEAD;
					pack16(&buf[0], packetID);
					buf[2] = (uint8_t)(packetCount >> 16); //pack 24
					buf[3] = (uint8_t)(packetCount >> 8);
					buf[4] = (uint8_t)(packetCount & 0xFF);

					if (DATAADAPTER_SUPPORT(ch)->droppedPacketID == packetID) {
						DATAADAPTER_SUPPORT(ch)->droppedPacketID = NO_ACTIVE_PACKET;
						return EPHIDGET_INTERRUPTED;
					}
					memcpy(&buf[USB_OUT_PACKET_OVERHEAD], buffer + i, (packetLen - USB_OUT_PACKET_OVERHEAD));
					remainingTime = maxDuration - (mos_gettime_usec() - start);
					ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)ch->parent, PHIDGETUSB_REQ_BULK_WRITE, 0, 0, buf, &packetLen, remainingTime /1000);
					if (ret != EPHIDGET_OK) {
						return ret;
					}
				}
			}
		}
	}
	else {
		ret = PhidgetChannel_beginTransaction(ch, &trans);
		if (ret != EPHIDGET_OK)
			goto done;

		if (len > 0) {
			if (len <= (VINT_MAX_OUT_PACKETSIZE - USB_OUT_PACKET_OVERHEAD)) {
				packetLen = len + USB_OUT_PACKET_OVERHEAD;
				packetInfo = NEW_PACKET_FLAG | packetID;
				if (waitResposne)
					packetInfo |= WAIT_RESP_FLAG;
				pack16(&buf[0], packetInfo);
				buf[2] = (uint8_t)(len >> 16); //pack 24
				buf[3] = (uint8_t)(len >> 8);
				buf[4] = (uint8_t)(len & 0xFF);
				memcpy(&buf[USB_OUT_PACKET_OVERHEAD], buffer, len);
				//remainingTime = maxDuration - (mos_gettime_usec() - start);
				ret = sendTXDataVINT(bp->iop, ch, buf, packetLen, &trans);
				if (ret != EPHIDGET_OK) {
					goto done;
				}

			} else {
				for (i = 0; i + (VINT_MAX_OUT_PACKETSIZE - USB_OUT_PACKET_OVERHEAD) < len; i += (VINT_MAX_OUT_PACKETSIZE - USB_OUT_PACKET_OVERHEAD)) {
					packetLen = VINT_MAX_OUT_PACKETSIZE;
					if (i == 0) {
						packetInfo = NEW_PACKET_FLAG | packetID;
						if (waitResposne)
							packetInfo |= WAIT_RESP_FLAG;
						pack16(&buf[0], packetInfo);
						buf[2] = (uint8_t)(len >> 16); //pack 24
						buf[3] = (uint8_t)(len >> 8);
						buf[4] = (uint8_t)(len & 0xFF);
					} else {
						pack16(&buf[0], packetID);
						buf[2] = (uint8_t)(packetCount >> 16); //pack 24
						buf[3] = (uint8_t)(packetCount >> 8);
						buf[4] = (uint8_t)(packetCount & 0xFF);
					}

					if (DATAADAPTER_SUPPORT(ch)->droppedPacketID == packetID) {
						DATAADAPTER_SUPPORT(ch)->droppedPacketID = NO_ACTIVE_PACKET;
						ret = EPHIDGET_INTERRUPTED;
						goto done;
					}
					memcpy(&buf[USB_OUT_PACKET_OVERHEAD], buffer + i, (packetLen - USB_OUT_PACKET_OVERHEAD));
					//remainingTime = maxDuration - (mos_gettime_usec() - start);
					ret = sendTXDataVINT(bp->iop, ch, buf, packetLen, &trans);
					if (ret != EPHIDGET_OK) {
						goto done;
					}
					packetCount++;
				}
				if (i != len) {
					packetLen = len - i + USB_OUT_PACKET_OVERHEAD;
					pack16(&buf[0], packetID);
					buf[2] = (uint8_t)(packetCount >> 16); //pack 24
					buf[3] = (uint8_t)(packetCount >> 8);
					buf[4] = (uint8_t)(packetCount & 0xFF);

					if (DATAADAPTER_SUPPORT(ch)->droppedPacketID == packetID) {
						DATAADAPTER_SUPPORT(ch)->droppedPacketID = NO_ACTIVE_PACKET;
						ret = EPHIDGET_INTERRUPTED;
						goto done;
					}
					memcpy(&buf[USB_OUT_PACKET_OVERHEAD], buffer + i, (packetLen - USB_OUT_PACKET_OVERHEAD));
					//remainingTime = maxDuration - (mos_gettime_usec() - start);
					ret = sendTXDataVINT(bp->iop, ch, buf, packetLen, &trans);
					if (ret != EPHIDGET_OK) {
						goto done;
					}
				}
			}
		}

	done:
		res1 = PhidgetChannel_endTransaction(ch, &trans);
		if (res1 != EPHIDGET_OK)
			return (res1);
	//	if(ret)
	//		return (ret);
	}


	////Wait to see if the device accepted the packet, this makes the return code from sendPacket mean something
	//start = mos_gettime_usec();

	for (;;) {
		if (DATAADAPTER_SUPPORT(ch)->ackID == packetID) { // a direct comparison should work here, as there is only ever one active packet at a time
			return (EPHIDGET_OK);
		}

		if (DATAADAPTER_SUPPORT(ch)->droppedPacketID == packetID) {
			DATAADAPTER_SUPPORT(ch)->droppedPacketID = NO_ACTIVE_PACKET;
			if (DATAADAPTER_SUPPORT(ch)->droppedPacketReason == TX_DROPPED_TIMEOUT) {
				return (MOS_ERROR(bp->iop, EPHIDGET_TIMEOUT, "The packet timed out while waiting to be transmitted. Check that your system can keep up with the data being sent."));
			}
			return EPHIDGET_INTERRUPTED;
		}

		if (!(ISATTACHED(ch))) {
			return (EPHIDGET_CLOSED);
		}

		duration = (mos_gettime_usec() - start);
		if (duration >= maxDuration) {
			//	phid->activePacket = NO_ACTIVE_PACKET; // transmission failed, the packet is no longer active
			return (EPHIDGET_TIMEOUT);
		}
		PhidgetLock(ch);
		if (DATAADAPTER_SUPPORT(ch)->baudRate != 0)
			PhidgetTimedWait(ch, ((800000 / DATAADAPTER_SUPPORT(ch)->baudRate) + 50) - ((uint32_t)(duration/1000)));
		else {
			PhidgetUnlock(ch);
			return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Baud Rate Not Configured"));
		}
		PhidgetUnlock(ch);
	}

}


/*
* Public API
*/

void
PhidgetDataAdapterSupport_free(PhidgetDataAdapterSupportHandle *arg) {

	if (arg == NULL || *arg == NULL)
		return;

	assert(arg);
	mos_mutex_destroy(&((*arg)->sendLock));
	mos_mutex_destroy(&((*arg)->receiveLock));

	mos_free(*arg, sizeof(PhidgetDataAdapterSupport));
	*arg = NULL;
}

PhidgetReturnCode
PhidgetDataAdapterSupport_create(PhidgetDataAdapterSupportHandle *arg) {

	TESTPTR_PR(arg);
	*arg = mos_zalloc(sizeof(PhidgetDataAdapterSupport));

	assert(arg);
	mos_mutex_init(&((*arg)->sendLock));
	mos_mutex_init(&((*arg)->receiveLock));

	return (EPHIDGET_OK);
}

void
PhidgetDataAdapterSupport_init(PhidgetDataAdapterSupportHandle dataAdapter) {

	assert(dataAdapter);

	dataAdapter->usbInPacketCount = 0;
	dataAdapter->packetID = 0;
	dataAdapter->rxPacketID = NEW_RX_READY;
	dataAdapter->baudRate = 9600;
	dataAdapter->protocol = PUNK_ENUM;
	dataAdapter->handshakeMode = PUNK_ENUM;

	dataAdapter->lastDataLength = 0;

	dataAdapter->ackID = NO_ACTIVE_PACKET;

	dataAdapter->nakFlag = 0;

	dataAdapter->rxPacketError = 0;

	dataAdapter->droppedPacketID = NO_ACTIVE_PACKET;

	dataAdapter->storedPacketLength = 0;
	dataAdapter->i2cFormatCount = 0;
	dataAdapter->address = PUNK_INT32;

	dataAdapter->txTimeout = 0;

}

