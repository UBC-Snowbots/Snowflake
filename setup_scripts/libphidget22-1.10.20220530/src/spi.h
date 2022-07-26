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

#ifndef __PHIDGET_SPI_H
#define __PHIDGET_SPI_H

// NOTE: SPI transfers are 128 bytes with 3 byte overhead
#define MAX_SPI_PACKET_SIZE	125

typedef struct {
	PHIDGET_STRUCT_START
	mos_task_t task;
	mos_mutex_t rlock;
	mos_cond_t rcond;
	int run;
	int dev;
} PhidgetSPIConnection, *PhidgetSPIConnectionHandle;

PhidgetReturnCode PhidgetSPIConnectionCreate(PhidgetSPIConnectionHandle *);
PhidgetSPIConnectionHandle PhidgetSPIConnectionCast(void *);

MOS_TASK_RESULT PhidgetSPIReadThreadFunction(void *);
void stopSPIReadThread(PhidgetSPIConnectionHandle);
void joinSPIReadThread(PhidgetSPIConnectionHandle);

PhidgetReturnCode PhidgetSPIGetVINTDevicesString(char *, size_t);
PhidgetReturnCode PhidgetSPIOpenHandle(PhidgetDeviceHandle);
PhidgetReturnCode PhidgetSPICloseHandle(PhidgetSPIConnectionHandle);
PhidgetReturnCode PhidgetSPIReadPacket(PhidgetSPIConnectionHandle, unsigned char *, size_t *);
PhidgetReturnCode PhidgetSPISendPacket(mosiop_t iop, PhidgetSPIConnectionHandle, const unsigned char *, size_t);
PhidgetReturnCode PhidgetSPISetLabel(PhidgetDeviceHandle, char *);
PhidgetReturnCode PhidgetSPIRefreshLabelString(PhidgetDeviceHandle);

PhidgetReturnCode openAttachedSPIDevice(PhidgetDeviceHandle);

#ifdef SPI_SUPPORT
PhidgetReturnCode populateAttachedSPIDevices(void);
PhidgetReturnCode clearAttachedSPIDevices(void);
#endif /* SPI_SUPPORT */

#endif /* __PHIDGET_SPI_H */
