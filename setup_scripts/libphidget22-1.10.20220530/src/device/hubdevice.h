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

#ifndef __CPHIDGETHUBDEVICE
#define __CPHIDGETHUBDEVICE

typedef struct _PhidgetHubDevice *PhidgetHubDeviceHandle;
PhidgetReturnCode PhidgetHubDevice_create(PhidgetHubDeviceHandle *phid);

//USB Out packet types
// 3-bit (Mask: 0x70)
typedef enum {
	VINTHUB_PACKET_HUB = 0x40,
	VINTHUB_PACKET_DEVICE = 0x20
} PhidgetHubDevice_PacketType;

//Hub configuration messages
// 4-bit (Mask: 0x0F)
typedef enum {
	VINTHUB_HUBPACKET_SETPORTMODE = 0x00,
	VINTHUB_HUBPACKET_UPGRADE_FIRMWARE = 0x01,
	VINTHUB_HUBPACKET_GETTXBUFFERSTATUS = 0x02,
	VINTHUB_HUBPACKET_PORTPOWER = 0x03,
	VINTHUB_HUBPACKET_MESHMODE = 0x04,
	VINTHUB_HUBPACKET_CALIBRATION_MODE	= 0x05,
	VINTHUB_HUBPACKET_CALIBRATION_WRITE	= 0x06,
	VINTHUB_HUBPACKET_CALIBRATION_EXIT	= 0x07,
} PhidgetHubDevice_HubPacketType;

PhidgetReturnCode PhidgetHubDevice_setPortMode(mosiop_t iop, PhidgetHubDeviceHandle phid, int index, PhidgetHub_PortMode mode);
PhidgetReturnCode PhidgetHubDevice_sendpacket(mosiop_t iop, PhidgetHubDeviceHandle phid, const uint8_t *buf);
PhidgetReturnCode sendHubPacket(
	mosiop_t iop,
	PhidgetHubDeviceHandle hub,
	PhidgetHubDevice_HubPacketType hubPacketType,
	uint8_t *bufferIn,
	size_t bufferInLen);
PhidgetReturnCode sendHubPortPacket(
	mosiop_t iop,
	PhidgetHubDeviceHandle hub,
	int hubPort,
	PhidgetHubDevice_HubPacketType hubPacketType,
	uint8_t *bufferIn,
	size_t bufferInLen);
PhidgetReturnCode PhidgetHubDevice_makePacket(
	PhidgetHubDeviceHandle		phid,
	PhidgetDeviceHandle			vintDevice,
	int							packetID,
	const uint8_t			*bufferIn,
	size_t						bufferInLen,
	uint8_t				*buffer,
	size_t						*bufferLen);
PhidgetReturnCode PhidgetHubDevice_claimPacketSpace(PhidgetHubDeviceHandle hub, int hubPort, size_t packetSize);
void PhidgetHubDevice_releasePacketSpace(PhidgetHubDeviceHandle phid, int hubPort, size_t packetSize);
PhidgetReturnCode PhidgetHubDevice_updatePortProperties(PhidgetHubDeviceHandle hub, int port);

#define HUB_PORT_ID_MAX				0x0F

#define VINTHUB_MAXPORTS 6
#define VINTHUB_MAXCHANNELS 32

#define VINTHUB_INPACKET_HUBMSG_FLAG 0x80

typedef enum {
	VINTHUB_HUBINPACKET_TXBUFFERSTATUS = 0x00,
	VINTHUB_HUBINPACKET_OVERCURRENT = 0x01,
	VINTHUB_HUBINPACKET_DETACH = 0x02,
	VINTHUB_HUBINPACKET_DISABLE = 0x03,
} PhidgetHubDevice_HubInPacketType;

#define VINTHUB_PACKETRETURN_notACK	0x80

#define HUB0001_CALIBRATION_UNLOCK_KEY	0x8ef90234

//Flag for identifying the start of a VINT packet in the USB packet stream
#define VINTHUB_IN_VINTPACKET_START		0x08

#define VINTHUB_PACKETID_MAX		126
#define VINTHUB_PACKETIDS_PER_PORT	(VINTHUB_PACKETID_MAX/6)

//Calibration table defines
#define VINTHUB_ADC_CALIB_TABLE_INDEX	0
#define VINTHUB_ADCCalibTable_ID 		1004
#define VINTHUB_ADCCalibTable_LENGTH 	28

struct _PhidgetHubDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.hub
	PhidgetDevice phid;

	size_t outstandingPacketCnt[VINTHUB_MAXPORTS];
	BOOL outstandingPacketCntValid;

	size_t internalPacketInBufferLen;

	int packetCounter;
	int packetOutCounter[VINTHUB_MAXPORTS];

	uint8_t splitPacketStorage[54];
	int splitPacketStoragePtr;

	//This stuff is for the Mesh Hub
	//6 chars per port, up to 4 portModes, plus ending NULL
	char portDescString[VINTHUB_MAXPORTS * 6 + 4 * 6 + 1];

	// Props that are available at attach - before open, read via descriptors
	uint32_t portProtocolVersion[VINTHUB_MAXPORTS];
	uint32_t portSupportsSetSpeed[VINTHUB_MAXPORTS];
	uint32_t portMaxSpeed[VINTHUB_MAXPORTS];
	uint32_t portSpeed[VINTHUB_MAXPORTS];
	PhidgetHub_PortMode portMode[VINTHUB_MAXPORTS];
	uint8_t portPowered[VINTHUB_MAXPORTS];

} typedef PhidgetHubDeviceInfo;

#endif
