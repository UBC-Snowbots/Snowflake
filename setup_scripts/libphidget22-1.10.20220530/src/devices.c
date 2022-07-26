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
#include "constantsinternal.h"
#include "constants.h"
#include "devices.h"

#include "device/accelerometerdevice.h"
#include "device/advancedservodevice.h"
#include "device/analogdevice.h"
#include "device/bridgedevice.h"
#include "device/dataadapterdevice.h"
#include "device/dictionarydevice.h"
#include "device/encoderdevice.h"
#include "device/firmwareupgradedevice.h"
#include "device/frequencycounterdevice.h"
#include "device/genericdevice.h"
#include "device/gpsdevice.h"
#include "device/hubdevice.h"
#include "device/interfacekitdevice.h"
#include "device/irdevice.h"
#include "device/leddevice.h"
#include "device/meshdongledevice.h"
#include "device/motorcontroldevice.h"
#include "device/phsensordevice.h"
#include "device/rfiddevice.h"
#include "device/servodevice.h"
#include "device/spatialdevice.h"
#include "device/stepperdevice.h"
#include "device/temperaturesensordevice.h"
#include "device/textlcddevice.h"
#include "device/vintdevice.h"

#define INDEX(x) x
#define COUNT(x) x
#define EXCLUSIVE(x) x
#define VERSION_RANGE(x,y) x,y

#ifndef _WINDOWS
#define ATTR(name, ...) { .name = {__VA_ARGS__} }
#define VINT_PHIDGET(uid, did, id) PHIDTYPE_VINT, did ,uid, PHIDCLASS_VINT, {.dummy = { 0 }}, 0, 0, 0, 0, id
#define VIRTUAL_PHIDGET(uid, did, cls, id) PHIDTYPE_VIRTUAL, did, uid, cls, {.dummy = { 0 }}, id, 0, 0, 0, 0
#else
#define ATTR(name, ...) { __VA_ARGS__ }
#define VINT_PHIDGET(uid, did, id) PHIDTYPE_VINT, did, uid, PHIDCLASS_VINT, {0}, 0, 0, 0, 0, id
#define VIRTUAL_PHIDGET(uid, did, cls, id) PHIDTYPE_VIRTUAL, did, uid, cls, {0}, id, 0, 0, 0, 0
#endif

#define USB_PHIDGET(uid, did, usbclass, vid, pid, interface, canNak, attr, ...) PHIDTYPE_USB, did, uid, usbclass, ATTR(attr, __VA_ARGS__), pid, vid, interface, canNak, 0
#define MESH_PHIDGET(uid, did, meshclass, id, attr, ...) PHIDTYPE_MESH, did, uid, meshclass, ATTR(attr, __VA_ARGS__), id, 0, 0, 0, 0
#define SPI_PHIDGET(uid, did, spiclass, id, attr, ...) PHIDTYPE_SPI, did, uid, spiclass, ATTR(attr, __VA_ARGS__), id, 0, 0, 0, 0
#define LIGHTNING_PHIDGET(uid, did, usbclass, vid, pid, interface, attr, ...) PHIDTYPE_LIGHTNING, did, uid, usbclass, ATTR(attr, __VA_ARGS__), pid, vid, interface, 0, 0

#define CHANNEL_REMOTE_SHARE		0x01
#define CHANNEL_REMOTE_EXCLUSIVE	0x02

#include "devices.gen.c"

const PhidgetChannelAttributeDef *
getPhidgetChannelAttributes(PhidgetChannelHandle channel) {

	return (getPhidgetChannelAttributesByClass(channel->class));
}

const PhidgetChannelAttributeDef *
getPhidgetChannelAttributesByClass(Phidget_ChannelClass cclass) {
	int i;

	for (i = 0; (int)(Phidget_Channel_Attribute_Def[i].chclass) != END_OF_LIST; i++)
		if (Phidget_Channel_Attribute_Def[i].chclass == cclass)
			return (&(Phidget_Channel_Attribute_Def[i]));

	return (NULL);
}

/*
 * Determines if the level of network access is allowed by the channel.
 *
 * First, the channel must support being shared over the network at all.
 * Second, if 'multiple' is set, the channel must not be flagged exclusive.
 */
int
allowNetworkAccess(PhidgetChannelHandle channel, int multiple) {
	const PhidgetChannelAttributeDef *def;

	def = getPhidgetChannelAttributes(channel);
	if (def == NULL) {
		logerr("No channel attribute definition found for channel class:0x%x", channel->class);
		return (0);
	}

	if ((def->remoteFlags & CHANNEL_REMOTE_SHARE) == 0)
		return (0);

	if (multiple) {
		if (def->remoteFlags & CHANNEL_REMOTE_EXCLUSIVE)
			return (0);
	}

	return (1);
}


PhidgetReturnCode
getUniqueChannelDef(const PhidgetUniqueDeviceDef *udd, Phidget_ChannelClass cclass, int uniqueIndex,
  int *cindex, const PhidgetUniqueChannelDef **_ucd) {
	const PhidgetUniqueChannelDef *ucd;
	int channelCnt;
	int index;

	TESTPTR(udd);
	TESTPTR(cindex);
	TESTPTR(_ucd);

	channelCnt = 0;
	index = 0;

	for (ucd = udd->channels; ((int)ucd->uid) != END_OF_LIST; ucd++) {
		for (index = 0; index < ucd->count; index++, channelCnt++) {
			if (channelCnt == uniqueIndex) {
				*cindex = index + ucd->index;
				*_ucd = ucd;
				return (EPHIDGET_OK);
			}
		}
	}
	return (EPHIDGET_NOENT);
}

PhidgetReturnCode
createTypedPhidgetDeviceHandle(PhidgetDeviceHandle *device, Phidget_DeviceClass class) {

	assert(device);

	switch (class) {
	case PHIDCLASS_ACCELEROMETER:
		return PhidgetAccelerometerDevice_create((PhidgetAccelerometerDeviceHandle *)device);
	case PHIDCLASS_ADVANCEDSERVO:
		return PhidgetAdvancedServoDevice_create((PhidgetAdvancedServoDeviceHandle *)device);
	case PHIDCLASS_ANALOG:
		return PhidgetAnalogDevice_create((PhidgetAnalogDeviceHandle *)device);
	case PHIDCLASS_BRIDGE:
		return PhidgetBridgeDevice_create((PhidgetBridgeDeviceHandle *)device);
	case PHIDCLASS_DATAADAPTER:
		return PhidgetDataAdapterDevice_create((PhidgetDataAdapterDeviceHandle *)device);
	case PHIDCLASS_DICTIONARY:
		return PhidgetDictionaryDevice_create((PhidgetDictionaryDeviceHandle *)device);
	case PHIDCLASS_ENCODER:
		return PhidgetEncoderDevice_create((PhidgetEncoderDeviceHandle *)device);
	case PHIDCLASS_FIRMWAREUPGRADE:
		return PhidgetFirmwareUpgradeDevice_create((PhidgetFirmwareUpgradeDeviceHandle *)device);
	case PHIDCLASS_FREQUENCYCOUNTER:
		return PhidgetFrequencyCounterDevice_create((PhidgetFrequencyCounterDeviceHandle *)device);
	case PHIDCLASS_GPS:
		return PhidgetGPSDevice_create((PhidgetGPSDeviceHandle *)device);
	case PHIDCLASS_HUB:
		return PhidgetHubDevice_create((PhidgetHubDeviceHandle *)device);
	case PHIDCLASS_INTERFACEKIT:
		return PhidgetInterfaceKitDevice_create((PhidgetInterfaceKitDeviceHandle *)device);
	case PHIDCLASS_IR:
		return PhidgetIRDevice_create((PhidgetIRDeviceHandle *)device);
	case PHIDCLASS_LED:
		return PhidgetLEDDevice_create((PhidgetLEDDeviceHandle *)device);
	case PHIDCLASS_MESHDONGLE:
		return PhidgetMeshDongleDevice_create((PhidgetMeshDongleDeviceHandle *)device);
	case PHIDCLASS_MOTORCONTROL:
		return PhidgetMotorControlDevice_create((PhidgetMotorControlDeviceHandle *)device);
	case PHIDCLASS_PHSENSOR:
		return PhidgetPHSensorDevice_create((PhidgetPHSensorDeviceHandle *)device);
	case PHIDCLASS_RFID:
		return PhidgetRFIDDevice_create((PhidgetRFIDDeviceHandle *)device);
	case PHIDCLASS_SERVO:
		return PhidgetServoDevice_create((PhidgetServoDeviceHandle *)device);
	case PHIDCLASS_SPATIAL:
		return PhidgetSpatialDevice_create((PhidgetSpatialDeviceHandle *)device);
	case PHIDCLASS_STEPPER:
		return PhidgetStepperDevice_create((PhidgetStepperDeviceHandle *)device);
	case PHIDCLASS_TEMPERATURESENSOR:
		return PhidgetTemperatureSensorDevice_create((PhidgetTemperatureSensorDeviceHandle *)device);
	case PHIDCLASS_TEXTLCD:
		return PhidgetTextLCDDevice_create((PhidgetTextLCDDeviceHandle *)device);
	case PHIDCLASS_GENERIC:
		return PhidgetGenericDevice_create((PhidgetGenericDeviceHandle *)device);
	case PHIDCLASS_VINT:
		return PhidgetVINTDevice_create((PhidgetVINTDeviceHandle *)device);
	default:
		return (EPHIDGET_UNEXPECTED);
	}
}

PhidgetReturnCode
deviceBridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetReturnCode res;

	if (isNetworkPhidget(ch))
		return (EPHIDGET_OK);

	// Ensure that device bridgeInput is called atomically
	//  Requests can come in from channel dispatchers on different threads, unlike with the channel bridgeInput
	mos_mutex_lock(&ch->parent->bridgeInputLock);
	res = ch->parent->bridgeInput(ch, bp);
	mos_mutex_unlock(&ch->parent->bridgeInputLock);

	return (res);
}
