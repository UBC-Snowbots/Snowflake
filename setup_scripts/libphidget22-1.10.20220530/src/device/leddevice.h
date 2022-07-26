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

#ifndef __CPHIDGETLEDDEVICE
#define __CPHIDGETLEDDEVICE

typedef struct _PhidgetLEDDevice *PhidgetLEDDeviceHandle;
PhidgetReturnCode PhidgetLEDDevice_create(PhidgetLEDDeviceHandle *phid);

#define LED_MAXLEDS 64

//OUT Packet Types
#define LED64_NORMAL_PACKET 0x00
#define LED64_CONTROL_PACKET 0x40
#define LED64_OUTLOW_PACKET 0x80
#define LED64_OUTHIGH_PACKET 0xc0

#define LED64_M3_OUT_LOW_PACKET 0x00
#define LED64_M3_OUT_HIGH_PACKET 0x20
#define LED64_M3_CONTROL_PACKET 0x40

//IN Packet Types
#define LED64_IN_LOW_PACKET 0x00
#define LED64_IN_HIGH_PACKET 0x80

#define LED64_M3_IN_LOW_PACKET 0x00
#define LED64_M3_IN_HIGH_PACKET 0x20
#define LED64_M3_IN_MISC_PACKET 0x40

//Flags
#define LED64_PGOOD_FLAG 0x01
#define LED64_CURSELA_FLAG 0x02
#define LED64_CURSELB_FLAG 0x04
#define LED64_PWRSELA_FLAG 0x08
#define LED64_PWRSELB_FLAG 0x10
#define LED64_FAULT_FLAG 0x20
#define LED64_OE_FLAG 0x40

#define LED64_CURRENTLIMIT	0.08 //80 mA max

struct _PhidgetLEDDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.led
	PhidgetDevice phid;

	/* Public Members */

	/* Private Members */

	double currentLimit[LED_MAXLEDS];
	double currentLimitBoard;
	PhidgetDigitalOutput_LEDForwardVoltage voltage[LED_MAXLEDS];
	PhidgetDigitalOutput_LEDForwardVoltage voltageBoard;

	double LED_Power[LED_MAXLEDS];
	double nextLED_Power[LED_MAXLEDS];
	double lastLED_Power[LED_MAXLEDS];
	uint8_t changedLED_Power[LED_MAXLEDS];

	uint8_t TSDCount[4], TSDClearCount[4], TWarnCount[4], TWarnClearCount[4];

	uint8_t lastOutputPacket;

	unsigned char lastControlPacket[MAX_OUT_PACKET_SIZE];
	int lastControlPacketValid;

} typedef PhidgetLEDDeviceInfo;

#endif
