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
#include "gpp.h"
#include "manager.h"

/**
 * General Packet Protocol
 *
 * These are devices on the new M3. They all support this protocol.
 *  MSB is set in buf[0] for incoming and outgoing packets.
 *
 * GPP is also supported on PHIDUSB devices, but in that case, we don't reserve the top bit, we just use different transfer types
 */
BOOL
deviceSupportsGeneralPacketProtocol(PhidgetDeviceHandle device) {

	// All PhidUSB devices
	if (device->connType == PHIDCONN_PHIDUSB)
		return PTRUE;

	switch (device->deviceInfo.UDD->uid) {
	case PHIDUID_1041:
	case PHIDUID_1043:
	case PHIDUID_1042:
	case PHIDUID_1044:
	case PHIDUID_1044_1:
	case PHIDUID_1044_1_510:
	case PHIDUID_1032:
	case PHIDUID_1024:
	case PHIDUID_1067:
	case PHIDUID_HUB0000:
#if PHIDUID_MESHDONGLE_SUPPORTED
	case PHIDUID_MESHDONGLE:
#endif
	case PHIDUID_HUB0004:
	case PHIDUID_HUB0004_BADPORT:
#if PHIDUID_LIGHTNINGHUB_SUPPORTED
	case PHIDUID_LIGHTNINGHUB:
#endif
	case PHIDUID_HUB5000:
#if PHIDUID_MOT0108_SUPPORTED
	case PHIDUID_MOT0108:
#endif
#if PHIDUID_MOT0110_SUPPORTED
	case PHIDUID_MOT0110:
#endif
	case PHIDUID_FIRMWARE_UPGRADE_M3_USB:
	case PHIDUID_FIRMWARE_UPGRADE_STM32_USB:
	case PHIDUID_FIRMWARE_UPGRADE_M3_SPI:
#if PHIDUID_GENERIC_HIDUSB_SUPPORTED
	case PHIDUID_GENERIC_HIDUSB:
#endif
		return PTRUE;

	default:
		return PFALSE;
	}
}

BOOL
deviceSupportsGeneralPacketProtocolDataInput(PhidgetDeviceHandle device) {

	// PHIDUSB devices don't use datainput
	if (device->connType == PHIDCONN_PHIDUSB)
		return PFALSE;

	return (deviceSupportsGeneralPacketProtocol(device));
}

PhidgetReturnCode
PhidgetGPP_dataInput(PhidgetDeviceHandle device, unsigned char *buffer, size_t length) {
	PhidgetReturnCode result = EPHIDGET_OK;

	//if response bits are set (0x00 is ignore), then store response
	if (buffer[0] & 0x3f)
		device->GPPResponse = buffer[0];

	return result;
}

static PhidgetReturnCode
GPP_getResponse(mosiop_t iop, PhidgetDeviceHandle device, int packetType, int timeout) {
	while ((device->GPPResponse & 0x3f) != packetType && timeout > 0) {
		mos_usleep(20000);
		timeout -= 20;
	}

	//Didn't get the response!
	if ((device->GPPResponse & 0x3f) != packetType) {
		return (MOS_ERROR(iop, EPHIDGET_TIMEOUT, "Timed out waiting for GPP response."));
	}

	if (device->GPPResponse & PHID_GENERAL_PACKET_FAIL)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "GPP response reported general failure."));

	return (EPHIDGET_OK);
}


static PhidgetReturnCode
GPP_setConfigTable(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *data, size_t length, int index, int packetType) {
	PhidgetReturnCode res;
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	int i, j;
	size_t buflen;

	assert(device);
	if (!ISATTACHED(device))
		return (EPHIDGET_NOTATTACHED);


	if (!deviceSupportsGeneralPacketProtocol(device))
		return (EPHIDGET_UNSUPPORTED);

	switch (device->connType) {
	case PHIDCONN_HIDUSB:
	case PHIDCONN_SPI:
		buflen = getMaxOutPacketSize(device);
		buffer[0] = PHID_GENERAL_PACKET_FLAG | packetType;
		buffer[1] = index;
		for (i = 2, j = 0; i < (int)buflen && j < (int)length; i++, j++)
			buffer[i] = data[j];

		if ((res = PhidgetDevice_sendpacket(iop, device, buffer, i)) != EPHIDGET_OK)
			return (res);

		while (j < (int)length && res == EPHIDGET_OK) {
			memset(buffer, 0, sizeof(buffer));
			buffer[0] = PHID_GENERAL_PACKET_FLAG | PHID_GENERAL_PACKET_CONTINUATION;
			for (i = 1; i < (int)buflen && j < (int)length; i++, j++)
				buffer[i] = data[j];
			if ((res = PhidgetDevice_sendpacket(iop, device, buffer, i)) != EPHIDGET_OK)
				return (res);
		}
		break;

	case PHIDCONN_PHIDUSB:
		buffer[0] = index;
		for (i = 1, j = 0; i < ((PhidgetPHIDUSBConnectionHandle)device->conn)->pusbParams.maxPacketEP0 && j < (int)length; i++, j++)
			buffer[i] = data[j];

		buflen = i;
		res = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
			packetType, 0, buffer, &buflen, DEFAULT_TRANSFER_TIMEOUT);
		if (res  != EPHIDGET_OK)
			return (res);

		while (j < (int)length && res == EPHIDGET_OK) {

			memset(buffer, 0, sizeof(buffer));
			for (i = 0; i < ((PhidgetPHIDUSBConnectionHandle)device->conn)->pusbParams.maxPacketEP0 && j < (int)length; i++, j++)
				buffer[i] = data[j];

			buflen = i;
			res = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
				PHID_GENERAL_PACKET_CONTINUATION, 0, buffer, &buflen, DEFAULT_TRANSFER_TIMEOUT);
			if (res != EPHIDGET_OK)
				return (res);
		}
		break;

	default:
		return (EPHIDGET_UNSUPPORTED);
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode
GPP_upgradeFirmware(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *data, size_t length, PhidgetChannelHandle channel) {
	int i, j, index, indexEnd;
	PhidgetReturnCode res;
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	const uint8_t *bufPtr;
	size_t offset;
	size_t buflen;
	double progress, lastProgress;

	TESTPTR(device);
	TESTATTACHED(device);

	lastProgress = 0;

	if (!deviceSupportsGeneralPacketProtocol(device))
		return (EPHIDGET_UNSUPPORTED);

	if (channel) {
		bridgeSendToChannel(channel, BP_PROGRESSCHANGE, "%g", 0.0);
		lastProgress = 0;
	}

	switch (device->connType) {

	case PHIDCONN_HIDUSB:
	case PHIDCONN_SPI:
		buflen = getMaxOutPacketSize(device);
		device->GPPResponse = 0;

		index = ((length & 0xf000) >> 12) + 1;
		indexEnd = length & 0xfff;
		j = 0;

		while (index) {
			int secLength = ((int)length) - ((index - 1) * 0x1000);
			if (secLength > 0x1000)
				secLength = 0x1000;

			buffer[0] = PHID_GENERAL_PACKET_FLAG | PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_WRITE_SECTOR;
			buffer[1] = index;
			buffer[2] = secLength;
			buffer[3] = secLength >> 8;

			for (i = 4; i < (int)buflen && j < indexEnd; i++, j++)
				buffer[i] = data[j];

			if ((res = PhidgetDevice_sendpacket(iop, device, buffer, i)) != EPHIDGET_OK)
				goto done;

			while (j < indexEnd && res == EPHIDGET_OK) {
				memset(buffer, 0, sizeof(buffer));
				buffer[0] = PHID_GENERAL_PACKET_FLAG | PHID_GENERAL_PACKET_CONTINUATION;
				for (i = 1; i < (int)buflen && j < indexEnd; i++, j++)
					buffer[i] = data[j];

				if (channel) {
					progress = (double)j / (double)length;
					if (progress - lastProgress >= 0.01) {
						bridgeSendToChannel(channel, BP_PROGRESSCHANGE, "%g", progress);
						lastProgress = progress;
					}
				}

				if ((res = PhidgetDevice_sendpacket(iop, device, buffer, i)) != EPHIDGET_OK)
					goto done;
			}
			index--;
			indexEnd += 0x1000;
		}

	done:

		res = GPP_getResponse(iop, device, PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_WRITE_SECTOR, 200);

		if (channel) {
			if (lastProgress != 1)
				bridgeSendToChannel(channel, BP_PROGRESSCHANGE, "%g", 1.0);
		}
		break;

	case PHIDCONN_PHIDUSB:

		bufPtr = data;
		offset = 0;
		buflen = ((PhidgetPHIDUSBConnectionHandle)device->conn)->pusbParams.maxPacketEP0;

		while (offset < length) {

			buffer[0] = (unsigned char)offset;
			buffer[1] = (unsigned char)(offset >> 8);

			if ((length - offset) < (buflen - 2))
				buflen = (length - offset) + 2;

			memcpy(&(buffer[2]), bufPtr, buflen - 2);
			offset += (buflen - 2);
			bufPtr += (buflen - 2);

			res = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
				PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_WRITE, 0, buffer, &buflen, DEFAULT_TRANSFER_TIMEOUT);

			if (res != EPHIDGET_OK)
				goto err;

			if (channel) {
				progress = (double)offset / (double)length;
				if (progress - lastProgress >= 0.01) {
					bridgeSendToChannel(channel, BP_PROGRESSCHANGE, "%g", progress);
					lastProgress = progress;
				}
			}
		}

		buflen = 0;
		res = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
			PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_DONE, 0, buffer, &buflen, DEFAULT_TRANSFER_TIMEOUT);

	err:
		if (channel) {
			if (lastProgress != 1)
				bridgeSendToChannel(channel, BP_PROGRESSCHANGE, "%g", 1.0);
		}

		break;

	default:
		return (EPHIDGET_UNSUPPORTED);
	}

	return (res);
}

PhidgetReturnCode
GPP_eraseFirmware(mosiop_t iop, PhidgetDeviceHandle device) {
	PhidgetReturnCode res;
	unsigned char buffer[1];
	size_t bufferlen;

	assert(device);
	TESTATTACHED(device);

	if (!deviceSupportsGeneralPacketProtocol(device))
		return (EPHIDGET_UNSUPPORTED);

	switch (device->connType) {

	case PHIDCONN_HIDUSB:
	case PHIDCONN_SPI:
		buffer[0] = PHID_GENERAL_PACKET_FLAG | PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_ERASE;
		bufferlen = 1;
		device->GPPResponse = 0;
		if ((res = PhidgetDevice_sendpacket(iop, device, buffer, bufferlen)) == EPHIDGET_OK)
			res = GPP_getResponse(iop, device, PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_ERASE, 200);

		break;

	case PHIDCONN_PHIDUSB:
		bufferlen = 0;
		res = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
			PHID_GENERAL_PACKET_FIRMWARE_UPGRADE_ERASE, 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);
		break;

	default:
		return (EPHIDGET_UNSUPPORTED);
	}

	return (res);
}

PhidgetReturnCode
GPP_eraseConfig(mosiop_t iop, PhidgetDeviceHandle device) {
	PhidgetReturnCode res;
	unsigned char buffer[1];
	size_t bufferlen;

	assert(device);
	TESTATTACHED(device);

	if (!deviceSupportsGeneralPacketProtocol(device))
		return (EPHIDGET_UNSUPPORTED);

	switch (device->connType) {

	case PHIDCONN_HIDUSB:
	case PHIDCONN_SPI:
		buffer[0] = PHID_GENERAL_PACKET_FLAG | PHID_GENERAL_PACKET_ERASE_CONFIG;
		bufferlen = 1;
		device->GPPResponse = 0;
		if ((res = PhidgetDevice_sendpacket(iop, device, buffer, bufferlen)) == EPHIDGET_OK)
			res = GPP_getResponse(iop, device, PHID_GENERAL_PACKET_ERASE_CONFIG, 200);
		break;

	case PHIDCONN_PHIDUSB:
		bufferlen = 0;
		res = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
			PHID_GENERAL_PACKET_ERASE_CONFIG, 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);
		break;

	default:
		return (EPHIDGET_UNSUPPORTED);
	}

	return (res);
}

PhidgetReturnCode
GPP_reboot_firmwareUpgrade(mosiop_t iop, PhidgetDeviceHandle device) {
	PhidgetUSBConnectionHandle usbConn;
#ifdef SPI_SUPPORT
	PhidgetSPIConnectionHandle spiconn;
#endif
	PhidgetReturnCode res;
	unsigned char buffer[1];
	size_t bufferlen;

	assert(device);
	TESTATTACHED(device);

	if (!deviceSupportsGeneralPacketProtocol(device))
		return (EPHIDGET_UNSUPPORTED);

	//Stop the read/write threads first
	switch (device->connType) {

#ifdef SPI_SUPPORT
	case PHIDCONN_SPI:
		spiconn = PhidgetSPIConnectionCast(device->conn);
		assert(spiconn);
		joinSPIReadThread(spiconn);

		//Then send the command
		buffer[0] = PHID_GENERAL_PACKET_FLAG | PHID_GENERAL_PACKET_REBOOT_FIRMWARE_UPGRADE;
		res = PhidgetDevice_sendpacket(iop, device, buffer, 1);
		break;
#endif

	case PHIDCONN_HIDUSB:
		usbConn = PhidgetUSBConnectionCast(device->conn);
		assert(usbConn);
		joinUSBReadThread(usbConn);

		//Then send the command
		buffer[0] = PHID_GENERAL_PACKET_FLAG | PHID_GENERAL_PACKET_REBOOT_FIRMWARE_UPGRADE;
		res = PhidgetDevice_sendpacket(iop, device, buffer, 1);

		// Detach this device explicitely so the library doesn't try to keep writing to it
		PhidgetWriteLockDevices();
		deviceDetach(device);
		PhidgetUnlockDevices();
		break;

	case PHIDCONN_PHIDUSB:
		usbConn = PhidgetUSBConnectionCast(device->conn);
		assert(usbConn);
		joinUSBReadThread(usbConn);

		//Then send the command
		bufferlen = 0;
		res = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
			PHID_GENERAL_PACKET_REBOOT_FIRMWARE_UPGRADE, 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);

		// Detach this device explicitely so the library doesn't try to keep writing to it
		PhidgetWriteLockDevices();
		deviceDetach(device);
		PhidgetUnlockDevices();
		break;

	default:
		return (EPHIDGET_UNSUPPORTED);
	}

#ifdef SPI_SUPPORT
	if (device->connType == PHIDCONN_SPI) {
		clearAttachedSPIDevices();
		mos_usleep(500000);
		populateAttachedSPIDevices();

		if (res == EPHIDGET_TIMEOUT)
			return (EPHIDGET_OK); //expected for spi
	}
#endif

	return (res);
}

PhidgetReturnCode
GPP_open_reset(mosiop_t iop, PhidgetDeviceHandle device) {
	PhidgetReturnCode res;
	unsigned char buffer[1];
	size_t bufferlen;

	assert(device);
	TESTATTACHED(device);

	if (!deviceSupportsGeneralPacketProtocol(device))
		return (EPHIDGET_UNSUPPORTED);

	if (device->connType != PHIDCONN_PHIDUSB)
		return (EPHIDGET_UNSUPPORTED);

	bufferlen = 0;
	res = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
		PHID_GENERAL_PACKET_OPEN_RESET, 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);

	return (res);
}

PhidgetReturnCode
GPP_close_reset(mosiop_t iop, PhidgetDeviceHandle device) {
	PhidgetReturnCode res;
	unsigned char buffer[1];
	size_t bufferlen;

	assert(device);
	TESTATTACHED(device);

	if (!deviceSupportsGeneralPacketProtocol(device))
		return (EPHIDGET_UNSUPPORTED);

	if (device->connType != PHIDCONN_PHIDUSB)
		return (EPHIDGET_UNSUPPORTED);

	bufferlen = 0;
	res = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
		PHID_GENERAL_PACKET_CLOSE_RESET, 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);

	return (res);
}

PhidgetReturnCode
GPP_reboot_ISP(mosiop_t iop, PhidgetDeviceHandle device) {
	PhidgetReturnCode res;
	unsigned char buffer[1];

	assert(device);
	TESTATTACHED(device);

	if (!deviceSupportsGeneralPacketProtocol(device))
		return (EPHIDGET_UNSUPPORTED);

	if (device->connType != PHIDCONN_HIDUSB)
		return (EPHIDGET_UNSUPPORTED);

	buffer[0] = PHID_GENERAL_PACKET_FLAG | PHID_GENERAL_PACKET_REBOOT_ISP;
	res = PhidgetDevice_sendpacket(iop, device, buffer, 1);

	return (res);
}

PhidgetReturnCode
GPP_writeFlash(mosiop_t iop, PhidgetDeviceHandle device) {
	PhidgetReturnCode res;
	unsigned char buffer[1];
	size_t bufferlen;

	assert(device);
	TESTATTACHED(device);

	if (!deviceSupportsGeneralPacketProtocol(device))
		return (EPHIDGET_UNSUPPORTED);

	switch (device->connType) {

	case PHIDCONN_HIDUSB:
	case PHIDCONN_SPI:
		buffer[0] = PHID_GENERAL_PACKET_FLAG | PHID_GENERAL_PACKET_WRITE_FLASH;
		bufferlen = 1;
		res = PhidgetDevice_sendpacket(iop, device, buffer, bufferlen);
		break;

	case PHIDCONN_PHIDUSB:
		bufferlen = 0;
		res = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
			PHID_GENERAL_PACKET_WRITE_FLASH, 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);
		break;

	default:
		return (EPHIDGET_UNSUPPORTED);
	}

	return (res);
}


PhidgetReturnCode
GPP_setLabel(mosiop_t iop, PhidgetDeviceHandle device, const char *label) {
	unsigned char buffer[26] = { 0 };
	PhidgetReturnCode result;
	size_t bufferlen;

	assert(device);
	TESTATTACHED(device);

	if (!deviceSupportsGeneralPacketProtocol(device))
		return (EPHIDGET_UNSUPPORTED);

	switch (device->connType) {

	case PHIDCONN_HIDUSB:
	case PHIDCONN_SPI:

		//Label Table Header is: 0x0010001A
		buffer[3] = 0x00; //header high byte
		buffer[2] = 0x10;
		buffer[1] = 0x00;
		buffer[0] = 0x1a; //header low byte

		memcpy(buffer + 4, label, label[0]);

		//Label Table index is: 0
		if ((result=GPP_setDeviceWideConfigTable(iop, device, buffer, 26, 0)) == EPHIDGET_OK)
			return GPP_writeFlash(iop, device);

		break;

	case PHIDCONN_PHIDUSB:
		memcpy(buffer, label, label[0]);
		bufferlen = label[0];
		result = PhidgetDevice_transferpacket(iop, device, PHIDGETUSB_REQ_GPP_WRITE,
			PHID_GENERAL_PACKET_WRITE_LABEL, 0, buffer, &bufferlen, DEFAULT_TRANSFER_TIMEOUT);
		break;

	default:
		return (EPHIDGET_UNSUPPORTED);
	}

	return result;
}

PhidgetReturnCode
GPP_setDeviceSpecificConfigTable(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *data, size_t length, int index) {
	assert(device);
	return GPP_setConfigTable(iop, device, data, length, index, PHID_GENERAL_PACKET_SET_DS_TABLE);
}

PhidgetReturnCode
GPP_setDeviceWideConfigTable(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *data, size_t length, int index) {
	assert(device);
	return GPP_setConfigTable(iop, device, data, length, index, PHID_GENERAL_PACKET_SET_DW_TABLE);
}

PhidgetReturnCode
PhidgetGPP_upgradeFirmware(mosiop_t iop, PhidgetChannelHandle channel, const unsigned char *data, size_t length) {
	assert(channel);
	assert(channel->parent);
	return GPP_upgradeFirmware(iop, channel->parent, data, length, channel);
}

PhidgetReturnCode
PhidgetGPP_eraseFirmware(mosiop_t iop, PhidgetChannelHandle channel) {
	assert(channel);
	assert(channel->parent);
	return GPP_eraseFirmware(iop, channel->parent);
}

PhidgetReturnCode
PhidgetGPP_reboot_firmwareUpgrade(mosiop_t iop, PhidgetChannelHandle channel) {
	assert(channel);
	assert(channel->parent);
	return GPP_reboot_firmwareUpgrade(iop, channel->parent);
}

PhidgetReturnCode
PhidgetGPP_setLabel(mosiop_t iop, PhidgetChannelHandle channel, const char *label) {
	assert(channel);
	assert(channel->parent);
	return GPP_setLabel(iop, channel->parent, label);
}

/****************
 * Exported API *
 ****************/



API_PRETURN
Phidget_writeFlash(PhidgetHandle phid) {
	PhidgetReturnCode res;
	mosiop_t iop;

	TESTPTR_PR(phid);
	TESTPTR_PR(phid->parent);
	TESTCHANNEL_PR(phid);

	iop = mos_iop_alloc();
	res = GPP_writeFlash(iop, phid->parent);
	return (PHID_RETURN_IOP(res, iop));
}
