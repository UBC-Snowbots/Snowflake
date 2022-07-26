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

#ifndef __CPHIDGETFREQUENCYCOUNTERDEVICE
#define __CPHIDGETFREQUENCYCOUNTERDEVICE

typedef struct _PhidgetFrequencyCounterDevice *PhidgetFrequencyCounterDeviceHandle;
PhidgetReturnCode PhidgetFrequencyCounterDevice_create(PhidgetFrequencyCounterDeviceHandle *phid);

#define FREQCOUNTER_MAXINPUTS 2

#define FREQCOUNTER_TICKS_PER_SEC	100000
#define FREQCOUNTER_MS_PER_TICK	(1000 / (double)FREQCOUNTER_TICKS_PER_SEC)

//OUT packet flags
#define FREQCOUNTER_FLAG_CH1_LOGIC 0x01
#define FREQCOUNTER_FLAG_CH0_LOGIC 0x02
#define FREQCOUNTER_FLAG_CH1_ENABLE 0x04
#define FREQCOUNTER_FLAG_CH0_ENABLE 0x08

struct _PhidgetFrequencyCounterDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.frequencycounter
	PhidgetDevice phid;

	/* Public Members */

	/* Private Members */

	PhidgetFrequencyCounter_FilterType filter[FREQCOUNTER_MAXINPUTS];
	uint8_t enabled[FREQCOUNTER_MAXINPUTS];

	int flip[FREQCOUNTER_MAXINPUTS];
	int lastPacketCount;

	uint32_t ticksAcc[FREQCOUNTER_MAXINPUTS];
	uint32_t countsAcc[FREQCOUNTER_MAXINPUTS];
	uint32_t ticksAtLastCountAcc[FREQCOUNTER_MAXINPUTS];

	uint32_t _interruptRate;
	uint32_t _frequencycounterDataInterval[FREQCOUNTER_MAXINPUTS];
	mostime_t _frequencycounterDeadline[FREQCOUNTER_MAXINPUTS];
	uint32_t _callcnt;
} typedef PhidgetFrequencyCounterDeviceInfo;

#endif
