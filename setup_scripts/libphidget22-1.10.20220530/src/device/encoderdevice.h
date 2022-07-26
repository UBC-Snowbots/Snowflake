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

#ifndef __CPHIDGETENCODERDEVICE
#define __CPHIDGETENCODERDEVICE

 //packet types
 //IN
 //OUT
#define ENCODER_GENERIC_PACKET			0x01
#define ENCODER_IO_MODE_PACKET			0x02

typedef struct _PhidgetEncoderDevice *PhidgetEncoderDeviceHandle;
PhidgetReturnCode PhidgetEncoderDevice_create(PhidgetEncoderDeviceHandle *phid);

#define ENCODER_MAXENCODERS 4
#define ENCODER_MAXINPUTS 4
struct _PhidgetEncoderDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.encoder
	PhidgetDevice phid;

	/* Public Members */

	uint8_t enabled[ENCODER_MAXENCODERS];
	uint8_t inputState[ENCODER_MAXINPUTS];

	/* Private Members */

	uint8_t enableState[ENCODER_MAXENCODERS];

	int32_t encoderTimeStamp[ENCODER_MAXENCODERS];

	uint32_t encoderChangeTrigger[ENCODER_MAXENCODERS];
	int positionChangeAccumulator[ENCODER_MAXENCODERS];
	uint64_t timeChangeAccumulator[ENCODER_MAXENCODERS]; //ns
	int indexTrueAccumulator[ENCODER_MAXENCODERS];
	int indexOffset[ENCODER_MAXENCODERS];

	/* we only need enough for encoders as we do not limit inputs */
	uint32_t _encoderDataInterval[ENCODER_MAXENCODERS];
	mostime_t _encoderDeadline[ENCODER_MAXENCODERS];
	uint32_t _callcnt;
	uint32_t _interruptRate;

} typedef PhidgetEncoderDeviceInfo;

#endif
