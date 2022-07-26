#ifndef EXTERNALPROTO
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
#endif

#ifndef _PHIDGET_MANAGER_H_
#define _PHIDGET_MANAGER_H_

#include "phidget.h"

typedef struct _PhidgetManager *PhidgetManagerHandle;

/* Methods */
API_PRETURN_HDR PhidgetManager_create				(PhidgetManagerHandle *phidm);
API_PRETURN_HDR PhidgetManager_delete				(PhidgetManagerHandle *phidm);

API_PRETURN_HDR PhidgetManager_open					(PhidgetManagerHandle phidm);
API_PRETURN_HDR PhidgetManager_close				(PhidgetManagerHandle phidm);

/* Events */
typedef void (CCONV *PhidgetManager_OnAttachCallback)	(PhidgetManagerHandle phidm, void *ctx, PhidgetHandle phid);
typedef void (CCONV *PhidgetManager_OnDetachCallback)	(PhidgetManagerHandle phidm, void *ctx, PhidgetHandle phid);
typedef void (CCONV *PhidgetManager_OnErrorCallback)	(PhidgetManagerHandle phidm, void *ctx, Phidget_ErrorEventCode errorCode, const char *errorString);

API_PRETURN_HDR PhidgetManager_setOnAttachHandler	(PhidgetManagerHandle phidm, PhidgetManager_OnAttachCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetManager_setOnDetachHandler	(PhidgetManagerHandle phidm, PhidgetManager_OnDetachCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetManager_setOnErrorHandler	(PhidgetManagerHandle phidm, PhidgetManager_OnErrorCallback fptr, void *ctx);

#ifndef EXTERNALPROTO

typedef struct _PhidgetManager {
	PHIDGET_STRUCT_START
	PhidgetManager_OnAttachCallback onAttach;
	void *onAttachCtx;
	PhidgetManager_OnDetachCallback onDetach;
	void *onDetachCtx;
	PhidgetManager_OnErrorCallback onError;
	void *onErrorCtx;

	MTAILQ_ENTRY(_PhidgetManager) link;
} PhidgetManager;

MTAILQ_HEAD(phidgetmanager_list, _PhidgetManager);
extern struct phidgetmanager_list phidgetManagerList;

PhidgetReturnCode sendAttachVisitor(PhidgetDeviceHandle, const PhidgetUniqueChannelDef *, int index,
  int uniqueIndex, void *ctx);
PhidgetReturnCode sendDetachVisitor(PhidgetDeviceHandle, const PhidgetUniqueChannelDef *, int index,
  int uniqueIndex, void *ctx);

#define FOREACH_MANAGER(phidm)				MTAILQ_FOREACH((phidm), &phidgetManagerList, link)
#define FOREACH_MANAGER_SAFE(phidm, tmp)	\
	MTAILQ_FOREACH_SAFE((phidm), &phidgetManagerList, link, (PhidgetManager *)(tmp))

void PhidgetLockManagers(void);
void PhidgetUnlockManagers(void);
PhidgetManagerHandle PhidgetManagerCast(void *);

void handleDetaches(void);
PhidgetReturnCode PhidgetManager_poll(void);
void channelAttach(PhidgetChannelHandle channel);
void channelDetach(PhidgetChannelHandle channel);
void channelClose(PhidgetChannelHandle channel);
PhidgetReturnCode deviceAttach(PhidgetDeviceHandle device, int needDevicesLock);
void deviceDetach(PhidgetDeviceHandle device);
PhidgetReturnCode walkDeviceChannels(PhidgetDeviceHandle, deviceChannelVisitor_t, void *ctx);

void PhidgetManagerInit(void);
void PhidgetManagerFini(void);

#endif //#ifndef EXTERNALPROTO

#endif /* _PHIDGET_MANAGER_H_ */
