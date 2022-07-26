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

#ifndef __PACKETTRACKER
#define __PACKETTRACKER

#include "phidget.h"

#define MAX_PACKET_IDS	128

#define PACKETTRACKER_INUSE		0x01
#define PACKETTRACKER_SIGNALLED	0x02
#define PACKETTRACKER_SENT		0x04
#define PACKETTRACKER_ABANDONED	0x08

typedef struct _PhidgetPacketTracker {
	int					_state;
	PhidgetReturnCode	_returnCode;
	int					childIndex;
	size_t				len;
	mos_cond_t			_cond;
	mos_mutex_t			_lock;
	MTAILQ_ENTRY(_PhidgetPacketTracker) link;
} PhidgetPacketTracker, *PhidgetPacketTrackerHandle;

typedef struct {
	PhidgetPacketTracker packetTracker[MAX_PACKET_IDS];
	int counter[PHIDGET_MAXCHILDREN];
} PhidgetPacketTrackers, *PhidgetPacketTrackersHandle;

typedef MTAILQ_HEAD(PhidgetPacketTrackerlist, _PhidgetPacketTracker) PhidgetPacketTrackerlist_t;

typedef struct {
	PhidgetReturnCode			error;
	int							errorCnt;
	PhidgetPacketTrackerlist_t	list;
} PhidgetTransaction, *PhidgetTransactionHandle;

PhidgetPacketTrackersHandle mallocPhidgetPacketTrackers(void);
void freePhidgetPacketTrackers(PhidgetPacketTrackersHandle);
void setPacketLength(PhidgetPacketTrackerHandle, size_t len);
PhidgetReturnCode setPacketReturnCode(PhidgetPacketTrackerHandle, PhidgetReturnCode);
void setPacketsReturnCode(PhidgetDeviceHandle, int child, PhidgetReturnCode status);
void _setPacketsReturnCode(PhidgetDeviceHandle, int child, PhidgetReturnCode status);
PhidgetReturnCode getPacketReturnCode(PhidgetPacketTrackerHandle, PhidgetReturnCode *);
PhidgetReturnCode waitForPendingPacket(PhidgetPacketTrackerHandle, uint32_t ms);
void waitForPendingPackets(PhidgetDeviceHandle, int child);
void waitForAllPendingPackets(PhidgetDeviceHandle);
PhidgetReturnCode getPacketTracker(PhidgetDeviceHandle, int *packetID, PhidgetPacketTrackerHandle *,
   int min, int max, int childIndex);
PhidgetReturnCode getPacketTrackerWait(PhidgetDeviceHandle device, int *packetID, PhidgetPacketTrackerHandle *packetTracker,
	int min, int max, int childIndex, uint32_t timeout);
void releasePacketTracker(PhidgetDeviceHandle, PhidgetPacketTrackerHandle, int force);

#endif
