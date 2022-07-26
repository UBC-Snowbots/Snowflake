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

#ifndef __CPHIDGETRFIDDEVICE
#define __CPHIDGETRFIDDEVICE
//#include "class/rfid.h"

typedef struct _PhidgetRFIDDevice *PhidgetRFIDDeviceHandle;
PhidgetReturnCode PhidgetRFIDDevice_create(PhidgetRFIDDeviceHandle *phid);

#include "util/rfidsupport.h"

#define RFIDDevice_PACKET_TAG 0
#define RFIDDevice_PACKET_OUTPUT_ECHO 1

#define RFIDDevice_LED_FLAG 0x04
#define RFIDDevice_ANTENNA_FLAG 0x08
#define RFIDDevice_LISTEN_DURING_EOF_FLAG 0x10

#define RFIDDevice_WRITE_DATA_OUT_PACKET	0x00
#define RFIDDevice_CONTROL_OUT_PACKET		0x40

#define RFIDDevice_READ_DATA_IN_PACKET	0x00
#define RFIDDevice_ECHO_IN_PACKET			0x40

#define RFIDDevice_MAX_DATA_PER_PACKET	63

#define RFIDDevice_DATA_ARRAY_SIZE		1024
#define RFIDDevice_DATA_ARRAY_MASK		0x3ff

#define RFIDDevice_MAXOUTPUTS 3
#define RFIDDevice_MAXRFIDS 1


/* 4097 constants */
#define RFIDDevice_4097_AmpDemod		0x00	//Amplitude demodulation
#define RFIDDevice_4097_PhaseDemod	0x01	//Phase demodulation

#define RFIDDevice_4097_PowerDown		0x00
#define RFIDDevice_4097_Active		0x02

#define RFIDDevice_4097_DataOut		0x00	//DATA_OUT is data from the rfid card
#define RFIDDevice_4097_ClkOut		0x04	//DATA_OUT is the internal clock/32

#define	RFIDDevice_4097_IntPLL		0x00
#define RFIDDevice_4097_ExtClk		0x08

#define RFIDDevice_4097_FastStart		0x10

#define RFIDDevice_4097_Gain960		0x40
#define RFIDDevice_4097_Gain480		0x00
#define RFIDDevice_4097_Gain240		0x60
#define RFIDDevice_4097_Gain120		0x20

#define RFIDDevice_4097_TestMode		0x80

#define RFIDDevice_4097_DefaultON		(RFIDDevice_4097_AmpDemod | RFIDDevice_4097_Active | RFIDDevice_4097_DataOut | RFIDDevice_4097_IntPLL | RFIDDevice_4097_FastStart | RFIDDevice_4097_Gain960)


/* T5577 Write Timing Constants */
#define RFIDDevice_T5577_StartGap 30
#define RFIDDevice_T5577_WriteGap 15
#define RFIDDevice_T5577_EndGap 15
#define RFIDDevice_T5577_Zero 24
#define RFIDDevice_T5577_One 56
#define RFIDDevice_T5577_EOF 100
#define RFIDDevice_T5577_PrePulse (136 + RFIDDevice_T5577_Zero)

#define RFIDDevice_MAX_TAG_STRING_LEN 25
typedef struct _PhidgetRFIDDevice_Tag {
	PhidgetRFID_Protocol protocol;
	char tagString[RFIDDevice_MAX_TAG_STRING_LEN];
#ifdef RFIDDevice_HITAGS_SUPPORT
	PhidgetRFIDDevice_TagType tagType;
#endif
} PhidgetRFIDDevice_Tag, *PhidgetRFIDDevice_TagHandle;

struct _PhidgetRFIDDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.rfid
	PhidgetDevice phid;

	/* Public Members */

	uint8_t outputState[RFIDDevice_MAXOUTPUTS];
	uint8_t antennaEnabled[RFIDDevice_MAXRFIDS];
	uint8_t tagPresent[RFIDDevice_MAXRFIDS];

	/* Private Members */

	/* State */

	int32_t spaceClocks, pregapClocks, postgapClocks, oneClocks, zeroClocks, prepulseClocks, eofpulseClocks;
	uint8_t listenDuringEOF;
	int32_t _4097Conf;

	uint8_t outputStateSet[RFIDDevice_MAXOUTPUTS];
	uint8_t antennaState;
	int32_t spaceClocksEcho, pregapClocksEcho, postgapClocksEcho, oneClocksEcho, zeroClocksEcho, prepulseClocksEcho, eofpulseClocksEcho;
	uint8_t listenDuringEOFEcho;
	int32_t _4097ConfEcho;

	uint8_t fullStateEcho;

	mos_task_t tagTimerThread;
	mos_mutex_t tagLock; /* protects tag thread access to things */
	mos_cond_t tagCond;
	int tagThreadRun;	/* 0 stopped; 1 running; 2 stop pending */

	/* Tag event */
	PhidgetRFIDDevice_Tag lastTag;
	uint8_t lastTagValid;
	mostime_t lastTagTime;
	PhidgetRFIDDevice_Tag pendingTag;
	uint8_t tagEventPending;

	/* Raw data buffer */
	int32_t dataBuffer[RFIDDevice_DATA_ARRAY_SIZE];
	uint32_t dataReadPtr, dataWritePtr;

	int shortClocks, longClocks;

	/* Manchester decoder */
	uint8_t manBuffer[RFIDDevice_DATA_ARRAY_SIZE];
	uint32_t manReadPtr, manWritePtr;
	uint8_t manLockedIn;
	uint8_t manShortChange;

	/* BiPhase Decoder */
	uint8_t biphaseBuffer[RFIDDevice_DATA_ARRAY_SIZE];
	uint32_t biphaseReadPtr, biphaseWritePtr;
	uint8_t biphaseLockedIn;
	uint8_t biphaseShortChange;

#ifdef RFIDDevice_RAWDATA_API_SUPPORT
	uint32_t userReadPtr;
	uint32_t manEventReadPtr;
	uint8_t lastManEventLong;
#endif

} typedef PhidgetRFIDDeviceInfo;

#endif
