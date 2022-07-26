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

#ifndef __PHIDGET_LIGHTNING_H
#define __PHIDGET_LIGHTNING_H

#define MAX_LIGHTNING_IN_PACKET_SIZE	1023
#define MAX_LIGHTNING_OUT_PACKET_SIZE	50

//Lightning Packet Types (top 2 bits)
#define VINT_LIGHTING_IN_PACKET_DATA             0x00
#define VINT_LIGHTING_IN_PACKET_DEVICESINFO   0x40
#define VINT_LIGHTING_OUT_PACKET_DATA            0x00
#define VINT_LIGHTING_OUT_PACKET_GETDEVICESINFO     0x40

typedef struct {
	void *accessory;
} PhidgetLightningConnection, *PhidgetLightningConnectionHandle;

PhidgetLightningConnectionHandle PhidgetLightningConnectionCast(void *);

PhidgetReturnCode PhidgetLightningSendPacket(mosiop_t iop, PhidgetLightningConnectionHandle conn, const unsigned char *buffer, size_t len);

PhidgetReturnCode openAttachedLightningDevice(PhidgetDeviceHandle device);
void freePhidgetLightningConnection(PhidgetLightningConnectionHandle item);
PhidgetLightningConnectionHandle mallocPhidgetLightningConnection(void);

#ifdef LIGHTNING_SUPPORT
PhidgetReturnCode PhidgetLightning_setup(void);
PhidgetReturnCode PhidgetLightning_teardown(void);
#endif

#endif
