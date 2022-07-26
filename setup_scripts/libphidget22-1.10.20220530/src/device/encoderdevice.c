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
#include "device/encoderdevice.h"

// === Internal Functions === //
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetEncoderDeviceHandle phid);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetEncoderDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetEncoderDeviceHandle phid = (PhidgetEncoderDeviceHandle)device;
	PhidgetReturnCode result;
	uint8_t buffer[1];
	int i;

	assert(phid);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1047:
	case PHIDUID_1047_2:
		phid->_interruptRate = 8;
		break;
	case PHIDUID_1052_OLD:
		phid->_interruptRate = 16;
		break;
	case PHIDUID_1052_v1:
		phid->_interruptRate = 16;
		break;
	case PHIDUID_1052_v2:
		phid->_interruptRate = 16;
		break;
	case PHIDUID_1057:
	case PHIDUID_1057_3:
		phid->_interruptRate = 8;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	//set data arrays to unknown, initial states
	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		phid->inputState[i] = PUNK_BOOL;
	}
	for (i = 0; i < phid->devChannelCnts.numEncoders; i++) {
		switch (phid->phid.deviceInfo.UDD->uid) {
		case PHIDUID_1047:
		case PHIDUID_1047_2:
			phid->enableState[i] = PUNK_BOOL;
			phid->enabled[i] = PUNK_BOOL;
			break;
		case PHIDUID_1052_OLD:
		case PHIDUID_1052_v1:
		case PHIDUID_1052_v2:
		case PHIDUID_1057:
		case PHIDUID_1057_3:
			phid->enableState[i] = PTRUE;
			phid->enabled[i] = PTRUE;
			break;
		default:
			MOS_PANIC("Unexpected device");
		}
		phid->encoderTimeStamp[i] = PUNK_INT32;
		phid->positionChangeAccumulator[i] = 0;
		phid->timeChangeAccumulator[i] = 0;
		phid->indexTrueAccumulator[i] = 0;
		phid->indexOffset[i] = 0;
		phid->encoderChangeTrigger[i] = 1;
	}

	//send out any initial pre-read packets
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1052_OLD:
		loginfo("Sending workaround startup packet");

		buffer[0] = 0;
		result = PhidgetDevice_sendpacket(NULL, (PhidgetDeviceHandle)phid, buffer, 1);

		if (result)
			return result;
		break;
	default:
		break;
	}

	//issue a read on the 4-input HS to get enable states
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1047:
	case PHIDUID_1047_2:
		waitForReads((PhidgetDeviceHandle)phid, 1, 100);
		break;
	default:
		break;
	}

	//fill in enabled states
	for (i = 0; i < phid->devChannelCnts.numEncoders; i++) {
		phid->enableState[i] = phid->enabled[i];
	}

	return (EPHIDGET_OK);
}

#define CALLTM(phid)	((phid)->_interruptRate * (phid)->_callcnt)

static PhidgetReturnCode CCONV
PhidgetEncoderDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {

	PhidgetEncoderDeviceHandle phid = (PhidgetEncoderDeviceHandle)device;
	uint64_t timeChangeInt[ENCODER_MAXENCODERS];
	int16_t positionChange[ENCODER_MAXENCODERS];
	int16_t indexChange[ENCODER_MAXENCODERS];
	int32_t packetTime[ENCODER_MAXENCODERS];
	int32_t curTime[ENCODER_MAXENCODERS];
	uint8_t indexTrue[ENCODER_MAXINPUTS];
	uint8_t input[ENCODER_MAXINPUTS];
	PhidgetChannelHandle channel;
	uint64_t timeChangeA;
	int positionChangeA;
	uint8_t indexTrueA;
	int indexOffsetA;
	mostime_t tm;

	uint8_t throwaway[ENCODER_MAXENCODERS] = {0};
	uint8_t lastInputState;
	uint16_t timeChange;
	uint8_t enabledEcho;
	uint16_t timeStamp;
	int i;
	int j;

	assert(phid);
	assert(buffer);

	timeStamp = 0;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1052_OLD:
	case PHIDUID_1052_v1:
		if (buffer[1] & 0x04)
			input[0] = PFALSE;
		else
			input[0] = PTRUE;

		positionChange[0] = (int8_t)buffer[2];
		indexTrue[0] = PFALSE;

		timeStamp = (((uint16_t)buffer[4]) << 8) + buffer[3];
		break;

	case PHIDUID_1052_v2:
		if (buffer[1] & 0x01)
			input[0] = PFALSE;
		else
			input[0] = PTRUE;

		positionChange[0] = (int8_t)buffer[4];
		indexTrue[0] = PFALSE;

		timeStamp = (((uint16_t)buffer[3]) << 8) + buffer[2];
		break;

	case PHIDUID_1057:
		/* high speed encoder */

		//this will work for 8 inputs before we need to change the protocol
		//currently no high-speed encoder has any inputs
		for (i = 0, j = 1; i < (phid->devChannelCnts.numInputs); i++, j <<= 1) {
			if (buffer[1] & j)
				input[i] = PFALSE;
			else
				input[i] = PTRUE;
		}

		//this will work for two encoders before we need to change the protocol
		for (i = 0; i < phid->devChannelCnts.numEncoders; i++) {
			positionChange[i] = (((uint16_t)buffer[2 * i + 5]) << 8) + buffer[2 * i + 4];
			indexTrue[i] = PFALSE;
		}

		timeStamp = (((uint16_t)buffer[3]) << 8) + buffer[2];
		break;
	case PHIDUID_1057_3:
		/* high speed encoder */

		positionChange[0] = ((int32_t)buffer[4] << 24) | ((int32_t)buffer[5] << 16) | ((int32_t)buffer[6] << 8) | ((int32_t)buffer[7]);
		indexTrue[0] = buffer[12];
		indexChange[0] = positionChange[0] - (((int32_t)buffer[8] << 24) | ((int32_t)buffer[9] << 16) | ((int32_t)buffer[10] << 8) | ((int32_t)buffer[11]));

		packetTime[0] = ((int32_t)buffer[0] << 24) | ((int32_t)buffer[1] << 16) | ((int32_t)buffer[2] << 8) | ((int32_t)buffer[0]);
		break;
	case PHIDUID_1047:
	case PHIDUID_1047_2:
		/* high speed encoder 4 input, with enable and index */
		//this will work for two encoders before we need to change the protocol
		for (i = 0; i < phid->devChannelCnts.numEncoders; i++) {
			positionChange[i] = (((uint16_t)buffer[10 * i + 0]) << 8) + buffer[10 * i + 1] - 0x7fff;
			indexChange[i] = (((uint16_t)buffer[10 * i + 2]) << 8) + buffer[10 * i + 3] - 0x7fff;
			packetTime[i] = (((int)buffer[10 * i + 4]) << 16) + (((uint16_t)buffer[10 * i + 5]) << 8) + buffer[10 * i + 6];
			curTime[i] = (((uint16_t)buffer[10 * i + 7]) << 8) + buffer[10 * i + 8];
			indexTrue[i] = (buffer[10 * i + 9] & 0x01) ? PTRUE : PFALSE;
			enabledEcho = (buffer[10 * i + 9] & 0x02) ? PTRUE : PFALSE;
			if (buffer[10 * i + 9] & 0x04)
				input[i] = PFALSE;
			else
				input[i] = PTRUE;

			//enabled echo
			phid->enabled[i] = enabledEcho;
		}
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	phid->_callcnt++;

	for (i = 0; i < phid->devChannelCnts.numEncoders; i++) {

		//Different timing stuff for different devices
		switch (phid->phid.deviceInfo.UDD->uid) {
		case PHIDUID_1052_OLD:
		case PHIDUID_1052_v1:
		case PHIDUID_1052_v2:
		case PHIDUID_1057:
			//this handles wraparounds because we're using uint16_t
			timeChange = (timeStamp - (uint16_t)phid->encoderTimeStamp[i]);

			//TODO: this misses the case where timeChange > 65 seconds - we would need to add timing to the library
			if (timeChange > 30000 || phid->encoderTimeStamp[i] == PUNK_INT32)
				timeChangeInt[i] = 0; // TODO: This time is actually unknown.. used to be PUNK_INT, but this isn't going to work anymore...
			else
				timeChangeInt[i] = (uint64_t)timeChange * 1000000; //ms -> ns

			phid->encoderTimeStamp[i] = timeStamp;
			break;
		case PHIDUID_1057_3:
			//timing
			if (phid->encoderTimeStamp[i] == PUNK_INT32) {
				phid->encoderTimeStamp[i] = packetTime[i];
				timeChangeInt[i] = 0;
				throwaway[i] = 1;
				break;
			}

			timeChangeInt[i] = packetTime[i] - phid->encoderTimeStamp[i];
			// convert 1/12,000,000 to ns
			timeChangeInt[i] = round(timeChangeInt[i] * 1000 / 12.0);

			if (timeChangeInt[i] > 16000000)
				throwaway[i] = 1;

			phid->encoderTimeStamp[i] = packetTime[i];
			break;
		case PHIDUID_1047:
			//timing
			if (phid->encoderTimeStamp[i] == PUNK_INT32)
				phid->encoderTimeStamp[i] = 0;

			phid->encoderTimeStamp[i] += packetTime[i];

			timeChangeInt[i] = phid->encoderTimeStamp[i] - curTime[i];
			// convert 1/12,000,000 to ns
			timeChangeInt[i] = round(timeChangeInt[i] * 1000 / 12.0);

			phid->encoderTimeStamp[i] = curTime[i];
			break;
		case PHIDUID_1047_2:
			//timing
			if (phid->encoderTimeStamp[i] == PUNK_INT32)
				phid->encoderTimeStamp[i] = 0;

			phid->encoderTimeStamp[i] += packetTime[i];

			timeChangeInt[i] = phid->encoderTimeStamp[i] - curTime[i];
			// convert 1/6,000,000 to ns
			timeChangeInt[i] = round(timeChangeInt[i] * 1000 / 6.0);

			phid->encoderTimeStamp[i] = curTime[i];
			break;
		default:
			MOS_PANIC("Unexpected device");
		}

		if (throwaway[i] == 0) {
			phid->positionChangeAccumulator[i] += positionChange[i];
			phid->timeChangeAccumulator[i] += timeChangeInt[i];
			if (indexTrue[i]) {
				phid->indexTrueAccumulator[i] = PTRUE;
				phid->indexOffset[i] = phid->positionChangeAccumulator[i] - indexChange[i];
			}
		}
		else {
			phid->positionChangeAccumulator[i] = 0;
			phid->timeChangeAccumulator[i] = 0;
			phid->indexTrueAccumulator[i] = 0;
			phid->indexOffset[i] = 0;
		}
	}

	tm = CALLTM(phid);

	for (i = 0; i < phid->devChannelCnts.numEncoders; i++) {

		if (phid->enabled[i] != PTRUE)
			continue;

		if (tm < phid->_encoderDeadline[i])
			continue;

		if ((unsigned)abs(phid->positionChangeAccumulator[i]) < phid->encoderChangeTrigger[i])
			continue;

		channel = getChannel(phid, i);
		if (channel == NULL)
			continue;

		if (throwaway[i] == 0) {
			timeChangeA = phid->timeChangeAccumulator[i];
			positionChangeA = phid->positionChangeAccumulator[i];
			indexTrueA = phid->indexTrueAccumulator[i];
			indexOffsetA = phid->indexOffset[i];
			bridgeSendToChannel(channel, BP_POSITIONCHANGE, "%d%g%c%d", positionChangeA, timeChangeA / 1000000.0, indexTrueA, indexOffsetA);
			PhidgetRelease(&channel);
		}
		phid->_encoderDeadline[i] = tm + phid->_encoderDataInterval[i];
		if(throwaway[i])
			phid->_encoderDeadline[i] += phid->_interruptRate;

		phid->positionChangeAccumulator[i] = 0;
		phid->timeChangeAccumulator[i] = 0;
		phid->indexTrueAccumulator[i] = PFALSE;
	}

	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		lastInputState = phid->inputState[i];
		phid->inputState[i] = input[i];

		if ((channel = getChannel(phid, i + phid->devChannelCnts.numEncoders)) != NULL) {
			if (phid->inputState[i] != lastInputState)
				bridgeSendToChannel(channel, BP_STATECHANGE, "%d", (int)(phid->inputState[i]));
			PhidgetRelease(&channel);
		}
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetEncoderDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetEncoderDeviceHandle phid = (PhidgetEncoderDeviceHandle)ch->parent;
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	size_t len;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_ENCODER);

	switch (ch->class) {
	case PHIDCHCLASS_DIGITALINPUT:
		assert(ch->index < phid->devChannelCnts.numInputs);
		switch (bp->vpkt) {
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_ENCODER:
		assert(ch->index < phid->devChannelCnts.numEncoders);
		switch (bp->vpkt) {
		case BP_SETENABLED:
			phid->enableState[ch->index] = getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid));
		case BP_SETDATAINTERVAL:
			if (phid->phid.deviceInfo.UDD->canNak) {
				// In this case, we ignore the message, as we cannot support data interval on a device that NAKs
				logwarn("Ignoring Set Data Interval request for this device, as it cannot be supported.");
				return (EPHIDGET_OK);
			}
			phid->_encoderDataInterval[ch->index] = HANDLE_DATAINTERVAL_PKT(bp, phid->_interruptRate);
			phid->_encoderDeadline[ch->index] = 0;
			return (EPHIDGET_OK);
		case BP_SETCHANGETRIGGER:
			phid->encoderChangeTrigger[ch->index] = getBridgePacketUInt32(bp, 0);
			return (EPHIDGET_OK);
		case BP_OPENRESET:
		case BP_CLOSERESET:
			phid->enableState[ch->index] = PFALSE;
			return (_sendpacket(bp->iop, phid));
		case BP_ENABLE:
			return (EPHIDGET_OK);
		case BP_SETIOMODE:
			switch (phid->phid.deviceInfo.UDD->uid) {
			case PHIDUID_1057_3:
				len = 1;
				buffer[0] = bp->entry[0].val.ui64;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, ENCODER_IO_MODE_PACKET, 0, buffer, &len, 100);
			default:
				MOS_PANIC("Unexpected packet type");
			}
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Unexpected channel class");
	}
}

//makePacket - constructs a packet using current device state
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetEncoderDeviceHandle phid) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	size_t len;
	int i;
	int j;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1047:
	case PHIDUID_1047_2:
	case PHIDUID_1057_3:
		//have to make sure that everything to be sent has some sort of default value if the user hasn't set a value
		for (i = 0; i < phid->devChannelCnts.numEncoders; i++) {
			if (phid->enableState[i] == PUNK_BOOL)
				phid->enableState[i] = PFALSE;
		}

		//construct the packet
		for (i = 0, j = 1; i < phid->devChannelCnts.numEncoders; i++, j <<= 1) {
			if (phid->enableState[i])
				buffer[0] |= j;
		}
		break;

	default:
		// No packets for these
		return (EPHIDGET_OK);
	}

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1047:
		return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
	case PHIDUID_1047_2:
		len = 1;
		return PhidgetDevice_transferpacket(iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, ENCODER_GENERIC_PACKET, 0, buffer, &len, 100);
	case PHIDUID_1057_3:
		len = 1;
		return PhidgetDevice_transferpacket(iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_WRITE, ENCODER_GENERIC_PACKET, 0, buffer, &len, 100);
	default:
		// No packets for these
		return (EPHIDGET_OK);
	}
}

static void CCONV
PhidgetEncoderDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetEncoderDevice));
	*phid = NULL;
}

// === Exported Functions === //

//create and initialize a device structure
PhidgetReturnCode
PhidgetEncoderDevice_create(PhidgetEncoderDeviceHandle *phidp) {
	DEVICECREATE_BODY(EncoderDevice, PHIDCLASS_ENCODER);
	return (EPHIDGET_OK);
}
