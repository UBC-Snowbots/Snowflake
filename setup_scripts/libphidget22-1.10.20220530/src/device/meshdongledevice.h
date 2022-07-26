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

#ifndef __CPHIDGETMESHDONGLEDEVICE
#define __CPHIDGETMESHDONGLEDEVICE

typedef struct _PhidgetMeshDongleDevice *PhidgetMeshDongleDeviceHandle;
PhidgetReturnCode PhidgetMeshDongleDevice_create(PhidgetMeshDongleDeviceHandle *phid);

PhidgetReturnCode openMeshDongleDeviceDongle(PhidgetMeshDongleDeviceHandle managerMeshDongleDevice, PhidgetMeshDongleDeviceHandle *openedMeshDongleDevice);
PhidgetReturnCode PhidgetMeshDongleDevice_makePacket(
	PhidgetMeshDongleDeviceHandle		phid,
	PhidgetDeviceHandle			meshDevice,
	int							packetID,
	const uint8_t			*bufferIn,
	size_t						bufferInLen,
	uint8_t				*buffer,
	size_t						*bufferLen);

void PhidgetMeshDongleDevice_releasePacketSpace(PhidgetMeshDongleDeviceHandle phid, int hubIndex, size_t packetSize);
PhidgetReturnCode PhidgetMeshDongleDevice_claimPacketSpace(PhidgetMeshDongleDeviceHandle phid, PhidgetDeviceHandle meshDevice, size_t packetSize);

typedef enum {
	MESHDONGLE_PACKET_PACKETSTATUS = 0x00,
	MESHDONGLE_PACKET_GETTXBUFFERSTATUS = 0x01,
	MESHDONGLE_PACKET_TXBUFFERSTATUS = 0x02,
} PhidgetMeshDongle_PacketType;

#define VINTMESH_INPACKET_DONGLE_INDEX 0x7F
#define VINTMESH_OUTPACKET_DONGLE_INDEX 0x7FFFFFFF

#define MESH_PACKETRETURN_notACK		0x80

#define VINTHUB_MAX_IN_PACKET_SIZE 128

#define MESHDONGLE_MAXHUBS	50

struct _PhidgetMeshDongleDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.mesh
	PhidgetDevice phid;

	mos_mutex_t outstandingPacketCntLock[MESHDONGLE_MAXHUBS]; /* protects outstandingPacketCnt */
	mos_cond_t outstandingPacketCntCond[MESHDONGLE_MAXHUBS];
	size_t outstandingPacketCnt[MESHDONGLE_MAXHUBS];
	BOOL outstandingPacketCntValid;

	size_t internalPacketInBufferLen;

	size_t packetCnt;
	uint8_t packetBuf[VINTHUB_MAX_IN_PACKET_SIZE];
	int packetBufWritePtr;
} typedef PhidgetMeshDongleDeviceInfo;

#endif
