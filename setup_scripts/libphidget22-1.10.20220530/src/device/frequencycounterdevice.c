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
#include "device/frequencycounterdevice.h"

// === Internal Functions === //
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetFrequencyCounterDeviceHandle phid);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetFrequencyCounterDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetFrequencyCounterDeviceHandle phid = (PhidgetFrequencyCounterDeviceHandle)device;
	int i;

	assert(phid);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1054:
		phid->_interruptRate = 32;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	//initialize triggers, set data arrays to unknown
	for (i = 0; i < phid->devChannelCnts.numFreqInputs; i++) {
		phid->filter[i] = PUNK_ENUM;
		phid->enabled[i] = PUNK_BOOL;

		phid->countsAcc[i] = 0;
		phid->ticksAtLastCountAcc[i] = 0;
		phid->ticksAcc[i] = 0;
	}
	phid->lastPacketCount = PUNK_INT32;

	//issue one read
	waitForReads((PhidgetDeviceHandle)phid, 1, 100);

	return (EPHIDGET_OK);
}

#define CALLTM(phid)	((phid)->_callcnt * (phid)->_interruptRate)

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetFrequencyCounterDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetFrequencyCounterDeviceHandle phid = (PhidgetFrequencyCounterDeviceHandle)device;
	uint32_t ticksAtLastCount[FREQCOUNTER_MAXINPUTS];
	uint32_t counts[FREQCOUNTER_MAXINPUTS];
	uint8_t enabled[FREQCOUNTER_MAXINPUTS];
	PhidgetChannelHandle channel;
	int packetCount;
	mostime_t tm;
	uint32_t ticks;
	int i;

	//Setup max/min values
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1054:
		ticks = buffer[0] + (buffer[1] << 8);

		counts[0] = buffer[2] + (buffer[3] << 8) + (buffer[4] << 16);
		ticksAtLastCount[0] = buffer[5] + (buffer[6] << 8);

		counts[1] = buffer[7] + (buffer[8] << 8) + (buffer[9] << 16);
		ticksAtLastCount[1] = buffer[10] + (buffer[11] << 8);

		//Enabled state echo
		if (buffer[12] & FREQCOUNTER_FLAG_CH0_ENABLE)
			enabled[0] = PTRUE;
		else
			enabled[0] = PFALSE;
		if (buffer[12] & FREQCOUNTER_FLAG_CH1_ENABLE)
			enabled[1] = PTRUE;
		else
			enabled[1] = PFALSE;

		packetCount = (buffer[12] & 0xF0) >> 4;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	phid->_callcnt++;

	if ((phid->lastPacketCount != PUNK_INT32) && ((phid->lastPacketCount + 1) & 0x0F) != packetCount) {
		for (i = 0; i < phid->devChannelCnts.numFreqInputs; i++) {
			if ((channel = getChannel(phid, i)) != NULL) {
				SEND_ERROR_EVENT(channel, EEPHIDGET_PACKETLOST, "One or more data packets were lost");
				PhidgetRelease(&channel);
			}
		}
	}
	phid->lastPacketCount = packetCount;

	tm = CALLTM(phid);

	for (i = 0; i < phid->devChannelCnts.numFreqInputs; i++) {
		if (enabled[i] != PTRUE || phid->enabled[i] != PTRUE)
			continue;

		phid->countsAcc[i] += counts[i];
		phid->ticksAtLastCountAcc[i] = phid->ticksAcc[i] + ticksAtLastCount[i]; /* previously accumulated ticks plus current ticksAtLastCount */
		phid->ticksAcc[i] += ticks;

		if (tm < phid->_frequencycounterDeadline[i])
			continue;

		channel = getChannel(phid, i);
		if (channel == NULL)
			continue;

		bridgeSendToChannel(channel, BP_FREQUENCYDATA, "%g%u%g", phid->ticksAcc[i] * FREQCOUNTER_MS_PER_TICK, phid->countsAcc[i], phid->ticksAtLastCountAcc[i] * FREQCOUNTER_MS_PER_TICK);
		PhidgetRelease(&channel);

		phid->countsAcc[i] = 0;
		phid->ticksAtLastCountAcc[i] = 0;
		phid->ticksAcc[i] = 0;

		phid->_frequencycounterDeadline[i] = tm + phid->_frequencycounterDataInterval[i];
	}
	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetFrequencyCounterDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetFrequencyCounterDeviceHandle phid = (PhidgetFrequencyCounterDeviceHandle)ch->parent;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_FREQUENCYCOUNTER);
	assert(ch->class == PHIDCHCLASS_FREQUENCYCOUNTER);
	assert(ch->index < phid->devChannelCnts.numFreqInputs);

	switch (bp->vpkt) {
	case BP_SETENABLED:
		phid->enabled[ch->index] = getBridgePacketInt32(bp, 0);
		return (_sendpacket(bp->iop, phid));
	case BP_SETDATAINTERVAL:
		phid->_frequencycounterDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
		phid->_frequencycounterDeadline[ch->index] = 0;
		return (EPHIDGET_OK);
	case BP_SETFILTERTYPE:
		phid->filter[ch->index] = (PhidgetFrequencyCounter_FilterType)getBridgePacketInt32(bp, 0);
		return (_sendpacket(bp->iop, phid));
	case BP_OPENRESET:
	case BP_CLOSERESET:
		phid->enabled[ch->index] = PFALSE;
		phid->filter[ch->index] = PUNK_ENUM;
		return (_sendpacket(bp->iop, phid));
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

//makePacket - constructs a packet using current device state
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetFrequencyCounterDeviceHandle phid) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };

	//Setup max/min values
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1054:
		buffer[0] = 0;
		if (phid->enabled[0] == PTRUE)
			buffer[0] |= FREQCOUNTER_FLAG_CH0_ENABLE;
		if (phid->filter[0] == FILTER_TYPE_LOGIC_LEVEL)
			buffer[0] |= FREQCOUNTER_FLAG_CH0_LOGIC;

		if (phid->enabled[1] == PTRUE)
			buffer[0] |= FREQCOUNTER_FLAG_CH1_ENABLE;
		if (phid->filter[1] == FILTER_TYPE_LOGIC_LEVEL)
			buffer[0] |= FREQCOUNTER_FLAG_CH1_LOGIC;
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
}

static void CCONV
PhidgetFrequencyCounterDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetFrequencyCounterDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetFrequencyCounterDevice_create(PhidgetFrequencyCounterDeviceHandle *phidp) {
	DEVICECREATE_BODY(FrequencyCounterDevice, PHIDCLASS_FREQUENCYCOUNTER);
	return (EPHIDGET_OK);
}
