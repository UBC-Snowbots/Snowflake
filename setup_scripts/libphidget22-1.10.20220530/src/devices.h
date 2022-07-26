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

#ifndef __PHIDGETDEVICESH
#define __PHIDGETDEVICESH

#define USBVID_PHIDGETS			0x06C2
#define USBVID_OLD				0x0925
#define USBPID_PHIDGETS_MIN		0x0030
#define USBPID_PHIDGETS_MAX		0x00AF

#include "constantsinternal.h"
#include "devices.gen.h"
#include "bridge.h"

typedef enum {
	PHIDTYPE_USB=1,
	PHIDTYPE_VINT=2,
	PHIDTYPE_MESH=3,
	PHIDTYPE_SPI=4,
	PHIDTYPE_LIGHTNING=5,
	PHIDTYPE_VIRTUAL=6
} PhidgetUniqueDeviceType;

typedef struct {
	int dummy1;				int dummy2;				int dummy3;				int dummy4;				int dummy5; int dummy6;
}	PhidgetDummyAttr;
typedef struct {
	int numAxis;
}	PhidgetAccelerometerAttr;
typedef struct {
	int numMotors;			int numCurrentInputs;
}	PhidgetAdvancedServoAttr;
typedef struct {
	int numAnalogOutputs;
}	PhidgetAnalogAttr;
typedef struct {
	int numBridgeInputs;
}	PhidgetBridgeAttr;
typedef struct {
	int numDataAdapters;		int numInputs;		int numOutputs;
}	PhidgetDataAdapterAttr;
typedef struct {
	int numEncoders;		int numInputs;
}	PhidgetEncoderAttr;
typedef struct {
	int numFreqInputs;
}	PhidgetFrequencyCounterAttr;
typedef struct {
	int numGPSes;
}	PhidgetGPSAttr;
typedef struct {
	int numVintPorts;		int numVintPortModes;
}	PhidgetHubAttr;
typedef struct {
	int numVoltageInputs;	int numSensors;			int numInputs;			int numOutputs;			int numCapTouches;
}	PhidgetInterfaceKitAttr;
typedef struct {
	int numIRs;
}	PhidgetIRAttr;
typedef struct {
	int numLEDs;
}	PhidgetLEDAttr;
typedef struct {
	int numHubs;
}	PhidgetMeshDongleAttr;
typedef struct {
	int numMotors;			int numInputs;			int numEncoders;		int numVoltageInputs;	int numSensors; int numCurrentInputs;
}	PhidgetMotorControlAttr;
typedef struct {
	int numVoltageInputs;	int numPHInputs;
}	PhidgetPHSensorAttr;
typedef struct {
	int numOutputs;
}	PhidgetRFIDAttr;
typedef struct {
	int numMotors;
}	PhidgetServoAttr;
typedef struct {
	int numAccelAxes;		int numGyroAxes;		int numCompassAxes;		int spatial;	int temperature;
}	PhidgetSpatialAttr;
typedef struct {
	int numMotors;			int numInputs;			int numCurrentInputs;
}	PhidgetStepperAttr;
typedef struct {
	int numTempInputs;		int numVoltageInputs;
}	PhidgetTemperatureSensorAttr;
typedef struct {
	int numScreens;			int numInputs;			int numOutputs;
}	PhidgetTextLCDAttr;
typedef union {
	PhidgetDummyAttr				dummy; /* field for unnamed initializers */
	PhidgetAccelerometerAttr		accelerometer;
	PhidgetAdvancedServoAttr		advancedservo;
	PhidgetAnalogAttr				analog;
	PhidgetBridgeAttr				bridge;
	PhidgetDataAdapterAttr			dataadapter;
	PhidgetEncoderAttr				encoder;
	PhidgetFrequencyCounterAttr		frequencycounter;
	PhidgetGPSAttr					gps;
	PhidgetHubAttr					hub;
	PhidgetInterfaceKitAttr			ifkit;
	PhidgetIRAttr					ir;
	PhidgetLEDAttr					led;
	PhidgetMeshDongleAttr			mesh;
	PhidgetMotorControlAttr			motorcontrol;
	PhidgetPHSensorAttr				phsensor;
	PhidgetRFIDAttr					rfid;
	PhidgetServoAttr				servo;
	PhidgetSpatialAttr				spatial;
	PhidgetStepperAttr				stepper;
	PhidgetTemperatureSensorAttr	temperaturesensor;
	PhidgetTextLCDAttr				textlcd;
} Phidget_DeviceAttr;

typedef struct {
	Phidget_DeviceChannelUID uid;
	Phidget_ChannelClass class;
	Phidget_ChannelSubclass subclass;
	int index;
	int count;
	int exclusive;
	const char *name;
} PhidgetUniqueChannelDef;

typedef struct {
	PhidgetUniqueDeviceType type;
	Phidget_DeviceID id;

	Phidget_DeviceUID uid;
	Phidget_DeviceClass class;
	Phidget_DeviceAttr channelCnts;
	int productID;
	int vendorID;
	int interfaceNum;
	int canNak;
	unsigned short vintID;

	int versionLow;
	int versionHigh;
	const char *SKU;
	const char *name;
	PhidgetUniqueChannelDef channels[9];
} PhidgetUniqueDeviceDef;

typedef struct {
	Phidget_ChannelClass chclass;
	int version;
	int remoteFlags;
} PhidgetChannelAttributeDef;

PhidgetReturnCode getUniqueChannelDef(const PhidgetUniqueDeviceDef *, Phidget_ChannelClass, int uniqueIndex,
  int *cindex, const PhidgetUniqueChannelDef **);
int allowNetworkAccess(PhidgetChannelHandle channel, int multiple);

PhidgetReturnCode createTypedPhidgetDeviceHandle(PhidgetDeviceHandle *phid, Phidget_DeviceClass deviceClass);
PhidgetReturnCode deviceBridgeInput(PhidgetChannelHandle, BridgePacket *);

#endif /* __PHIDGETDEVICESH */
