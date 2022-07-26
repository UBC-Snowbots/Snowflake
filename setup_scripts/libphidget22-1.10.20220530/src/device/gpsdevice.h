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

#ifndef __CPHIDGETGPSDEVICE
#define __CPHIDGETGPSDEVICE

typedef struct _PhidgetGPSDevice *PhidgetGPSDeviceHandle;
PhidgetReturnCode PhidgetGPSDevice_create(PhidgetGPSDeviceHandle *phid);

#define GPS_MAXGPSES 1

// SkyTraq 6 binary messages
// Input system messages (commands)
#define GPS_SKYTRAQ_IN_SYSTEM_RESTART				0x01
#define GPS_SKYTRAQ_IN_QUERY_SOFTWARE_VERSION		0x02
#define GPS_SKYTRAQ_IN_QUERY_POSITION_UPDATE_RATE	0x10
// Input GPS messages (commands)
#define GPS_SKYTRAQ_IN_CONFIGURE_WAAS				0x37
#define GPS_SKYTRAQ_IN_QUERY_WAAS					0x38
// Output system messages (responses)
#define GPS_SKYTRAQ_OUT_SOFTWARE_VERSION			0x80
#define GPS_SKYTRAQ_OUT_SOFTWARE_CRC				0x81
#define GPS_SKYTRAQ_OUT_ACK							0x83
#define GPS_SKYTRAQ_OUT_NACK						0x84
#define GPS_SKYTRAQ_OUT_POSITION_UPDATE_RATE		0x86
// Output GPS messages

struct _PhidgetGPSDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.gps
	PhidgetDevice phid;

	/* Public Members */

	PhidgetGPS_NMEAData NMEAData[GPS_MAXGPSES];
	uint8_t NMEADataValid[GPS_MAXGPSES];
	double heading[GPS_MAXGPSES];
	double velocity[GPS_MAXGPSES];
	double altitude[GPS_MAXGPSES];
	double latitude[GPS_MAXGPSES];
	double longitude[GPS_MAXGPSES];
	uint8_t positionFixState[GPS_MAXGPSES];
	uint8_t timeValid[GPS_MAXGPSES];
	PhidgetGPS_Time time[GPS_MAXGPSES];
	uint8_t dateValid[GPS_MAXGPSES];
	PhidgetGPS_Date date[GPS_MAXGPSES];

	/* Private Members */

	double lastLongitude, lastLatitude, lastAltitude;
	uint8_t lastFix, lastDateValid, lastTimeValid;

	uint8_t sckbuf[256];
	uint8_t sckbuf_write, sckbuf_read;
} typedef PhidgetGPSDeviceInfo;
#endif
