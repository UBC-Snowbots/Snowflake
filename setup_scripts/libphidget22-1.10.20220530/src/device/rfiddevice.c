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
#include "device/rfiddevice.h"

// === Internal Functions === //

MOS_TASK_RESULT tagTimerThreadFunction(void *ctx);
static PhidgetReturnCode analyze_data(PhidgetRFIDDeviceHandle phid);
static PhidgetReturnCode tagEvent_fromEM4100Data(PhidgetRFIDDeviceHandle phid, uint8_t *data);
static PhidgetReturnCode startTagTimerThread(PhidgetRFIDDeviceHandle phid);
static PhidgetReturnCode rfid_write(mosiop_t iop, PhidgetRFIDDeviceHandle phid, const char *tagString, PhidgetRFID_Protocol protocol, int lock);
static PhidgetReturnCode rfid_writeRaw(mosiop_t iop, PhidgetRFIDDeviceHandle phid, uint8_t *data, int bitlength, int pregap, int space, int postgap, int zero, int one, int prepulse, int eof, int listenDuringEOF);
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetRFIDDeviceHandle phid);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetRFIDDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetRFIDDeviceHandle phid = (PhidgetRFIDDeviceHandle)device;
	int i;
	PhidgetReturnCode result;
	assert(phid);

	//setup anything device specific
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1023_OLD:
	case PHIDUID_1023:
		phid->fullStateEcho = PFALSE;
		phid->antennaEnabled[0] = PTRUE;
		break;
	case PHIDUID_1023_2OUTPUT_NO_ECHO:
		phid->fullStateEcho = PFALSE;
		phid->antennaEnabled[0] = PUNK_BOOL;
		break;
	case PHIDUID_1023_2OUTPUT:
	case PHIDUID_1024:
		phid->fullStateEcho = PTRUE;
		phid->antennaEnabled[0] = PUNK_BOOL;
		break;
#if PHIDUID_1024_V2_USB_SUPPORTED
	case PHIDUID_1024_V2_USB:
		phid->fullStateEcho = PFALSE;
		phid->antennaEnabled[0] = PUNK_BOOL;
		break;
#endif /* PHIDUID_1024_V2_USB_SUPPORTED */
	default:
		MOS_PANIC("Unexpected device");
	}

	//set data arrays to unknown
	for (i = 0; i < phid->devChannelCnts.numOutputs; i++)
		phid->outputState[i] = PUNK_BOOL;

	//phid->ledEchoState = PUNK_BOOL;
	phid->tagPresent[0] = PUNK_BOOL;
	memset(&phid->lastTag, 0, sizeof(PhidgetRFIDDevice_Tag));
	memset(&phid->pendingTag, 0, sizeof(PhidgetRFIDDevice_Tag));
	phid->tagEventPending = PFALSE;
	phid->lastTagValid = PFALSE;

	phid->pregapClocksEcho = PUNK_INT32;
	phid->prepulseClocksEcho = PUNK_INT32;
	phid->zeroClocksEcho = PUNK_INT32;
	phid->oneClocksEcho = PUNK_INT32;
	phid->spaceClocksEcho = PUNK_INT32;
	phid->postgapClocksEcho = PUNK_INT32;
	phid->eofpulseClocksEcho = PUNK_INT32;
	phid->listenDuringEOFEcho = PUNK_BOOL;

	phid->_4097ConfEcho = PUNK_INT32;

	phid->dataReadPtr = 0;
	phid->dataWritePtr = 0;
	phid->manReadPtr = 0;
	phid->manWritePtr = 0;
	phid->biphaseReadPtr = 0;
	phid->biphaseWritePtr = 0;

	phid->shortClocks = phid->longClocks = 0;

	phid->manLockedIn = PFALSE;
	phid->biphaseLockedIn = PFALSE;
	phid->manShortChange = PFALSE;
	phid->biphaseShortChange = PFALSE;

	phid->tagThreadRun = 0;

#ifdef RFIDDevice_RAWDATA_API_SUPPORT
	phid->userReadPtr = 0;
	phid->manEventReadPtr = 0;
	phid->lastManEventLong = PFALSE;
#endif

	//send out any initial pre-read packets
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1023_OLD:
	{
		uint8_t buffer[1] = { 0 };
		loginfo("Sending workaround startup packet");

		result = PhidgetDevice_sendpacket(NULL, (PhidgetDeviceHandle)phid, buffer, 1);
		if (result)
			return result;
		break;
	}
	default:
		break;
	}

	//issue a read for devices that return output data
	if (phid->fullStateEcho) {
		int readtries = 16; //should guarentee a packet with output data - even if a tag is present
		while (readtries-- > 0) {
			waitForReads((PhidgetDeviceHandle)phid, 1, 100);
			if (phid->outputState[0] != PUNK_BOOL)
				break;
		}
		//one more read guarantees that if there is a tag present, we will see it - output packets only happen every 255ms
		waitForReads((PhidgetDeviceHandle)phid, 1, 100);
	}

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1024:
		//Did we get some data? Wait for a tag
		if (phid->dataWritePtr != 0) {
			int readtries = 30; //This should be enough data to guarantee detection of a tag.
			while (readtries-- > 0) {
				waitForReads((PhidgetDeviceHandle)phid, 1, 100);
				if (phid->tagPresent[0] != PUNK_BOOL)
					break;
			}
		}
		break;
#if PHIDUID_1024_V2_USB_SUPPORTED
	case PHIDUID_1024_V2_USB:
		{
			uint8_t buffer[MAX_IN_PACKET_SIZE];
			size_t len;
			len = 5;
			result = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_READ, 5, 0, buffer, &len, 100);
			if (result != EPHIDGET_OK)
				return result;

			phid->antennaEnabled[0] = (buffer[0] & 0x01) != 0;
			phid->outputState[0] = (buffer[0] & 0x02) != 0;
			phid->outputState[1] = (buffer[0] & 0x04) != 0;
			phid->outputState[2] = (buffer[0] & 0x08) != 0;

			break;
		}
#endif /* PHIDUID_1024_V2_USB_SUPPORTED */
	default:
		break;
	}

	//if the antenna is on, and tagPresent is unknown, then it is false
	if (phid->antennaEnabled[0] == PTRUE && phid->tagPresent[0] == PUNK_BOOL)
		phid->tagPresent[0] = PFALSE;

	//So that we can get the tag in the attach handler
	if (phid->tagPresent[0] == PTRUE) {
		phid->lastTag = phid->pendingTag;
		phid->lastTagValid = PTRUE;
	}

	//recover what we can - if anything isn't filled out, it's PUNK anyways
	for (i = 0; i < phid->devChannelCnts.numOutputs; i++) {
		phid->outputStateSet[i] = phid->outputState[i];
	}
	phid->antennaState = phid->antennaEnabled[0];
	//phid->ledState = phid->ledEchoState;

	phid->pregapClocks = phid->pregapClocksEcho;
	phid->postgapClocks = phid->postgapClocksEcho;
	phid->zeroClocks = phid->zeroClocksEcho;
	phid->oneClocks = phid->oneClocksEcho;
	phid->spaceClocks = phid->spaceClocksEcho;
	phid->eofpulseClocks = phid->eofpulseClocksEcho;
	phid->prepulseClocks = phid->prepulseClocksEcho;
	phid->listenDuringEOF = phid->listenDuringEOFEcho;
	phid->_4097Conf = phid->_4097ConfEcho;

	//NETWORKSYNC(RFIDDevice);

	//TODO: We may want to start this after the attach event..?
	return startTagTimerThread(phid);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetRFIDDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetRFIDDeviceHandle phid = (PhidgetRFIDDeviceHandle)device;
	uint8_t outputs[RFIDDevice_MAXOUTPUTS];
	uint8_t newStateData = PFALSE;
	uint8_t antennaState = PFALSE;
	int i = 0, j = 0;
#if PHIDUID_1024_V2_USB_SUPPORTED
	PhidgetChannelHandle channel;
	int pkt;
#endif

	assert(phid);
	assert(buffer);

	//Parse device packets - store data locally
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1023_OLD:
	case PHIDUID_1023:
		// Enything other then all 0's means a tag is detected
		if (memcmp("\0\0\0\0\0", buffer + 1, 5))
			tagEvent_fromEM4100Data(phid, buffer + 1);
		break;

	case PHIDUID_1023_2OUTPUT_NO_ECHO:
	case PHIDUID_1023_2OUTPUT:
		switch (buffer[0]) {
		case RFIDDevice_PACKET_TAG:
			// Enything other then all 0's means a tag is detected
			if (memcmp("\0\0\0\0\0", buffer + 1, 5))
				tagEvent_fromEM4100Data(phid, buffer + 1);
			break;
		case RFIDDevice_PACKET_OUTPUT_ECHO:
			if (phid->fullStateEcho) {
				newStateData = PTRUE;

				for (i = 0, j = 0x01; i < phid->devChannelCnts.numOutputs; i++, j <<= 1) {
					if (buffer[1] & j)
						outputs[i] = PTRUE;
					else
						outputs[i] = PFALSE;
				}

				if (buffer[1] & RFIDDevice_ANTENNA_FLAG)
					antennaState = PTRUE;
				else
					antennaState = PFALSE;
			}
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}
		break;

	// RFIDDevice with decoding in software and write support
	case PHIDUID_1024:
	{
		int dataLength = 0;
		int dataOffset = 1;
		int data[RFIDDevice_MAX_DATA_PER_PACKET];
		switch (buffer[0] & 0x40) {
		case RFIDDevice_ECHO_IN_PACKET:
			dataOffset = 11;
			newStateData = PTRUE;

			//Don't bother - we don't use it.
			//phid->frequencyEcho = buffer[1] * 1000;

			phid->pregapClocksEcho = buffer[2];
			phid->prepulseClocksEcho = buffer[7];
			phid->zeroClocksEcho = buffer[4];
			phid->oneClocksEcho = buffer[5];
			phid->spaceClocksEcho = buffer[6];
			phid->postgapClocksEcho = buffer[3];
			phid->eofpulseClocksEcho = buffer[8];

			for (i = 0, j = 0x01; i < phid->devChannelCnts.numOutputs; i++, j <<= 1) {
				if (buffer[9] & j)
					outputs[i] = PTRUE;
				else
					outputs[i] = PFALSE;
			}

			if (buffer[9] & RFIDDevice_ANTENNA_FLAG)
				antennaState = PTRUE;
			else
				antennaState = PFALSE;

			if (buffer[9] & RFIDDevice_LISTEN_DURING_EOF_FLAG)
				phid->listenDuringEOFEcho = PTRUE;
			else
				phid->listenDuringEOFEcho = PFALSE;

			phid->_4097ConfEcho = buffer[10];

			//NOTE: Fall Through
		case RFIDDevice_READ_DATA_IN_PACKET:

			//move RFIDDevice data into local storage
			dataLength = buffer[0] & 0x3F;
			for (i = 0; i < dataLength; i++) {
				data[i] = buffer[i + dataOffset] << 1;
				if ((data[i] & 0xFE) == 0xFE) {
					data[i] = PUNK_INT32;
					phid->dataBuffer[phid->dataWritePtr] = PUNK_INT32;
				} else {
					// convert to data lengths that we expect to deal with internally
					int polarity = data[i] & 0x100;
					int clocks = data[i] & 0xff;

					if (clocks >= 10 && clocks <= 22)
						phid->dataBuffer[phid->dataWritePtr] = polarity | 16;
					else if (clocks >= 26 && clocks <= 40)
						phid->dataBuffer[phid->dataWritePtr] = polarity | 32;
					else if (clocks >= 42 && clocks <= 54)
						phid->dataBuffer[phid->dataWritePtr] = polarity | 48;
					else if (clocks >= 56 && clocks <= 72)
						phid->dataBuffer[phid->dataWritePtr] = polarity | 64;
					else if (clocks >= 120 && clocks <= 136)
						phid->dataBuffer[phid->dataWritePtr] = polarity | 128;
					else
						phid->dataBuffer[phid->dataWritePtr] = PUNK_INT32;
				}

				phid->dataWritePtr++;
				phid->dataWritePtr &= RFIDDevice_DATA_ARRAY_MASK;

				//if we run into data that hasn't been read... too bad, we overwrite it and adjust the read pointer
				if (phid->dataWritePtr == phid->dataReadPtr) {
					phid->dataReadPtr++;
					phid->dataReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
				}
			}

			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

		if (dataLength) {

#ifdef RFIDDevice_RAWDATA_API_SUPPORT
			if ((channel = getChannel(phid, 0))) {
				bridgeSendToChannel(channel, BP_RAWDATA, "%*U", dataLength, data);
				PhidgetRelease(&channel);
			}
#endif

			//analyze data
			analyze_data(phid);
		}
		break;
	}
#if PHIDUID_1024_V2_USB_SUPPORTED
	case PHIDUID_1024_V2_USB:
		pkt = buffer[0];
		buffer++;

		switch (pkt) {
		case VINT_PACKET_TYPE_RFID_TAG:
			if ((channel = getChannel(phid, 0)) != NULL) {
				mos_mutex_lock(&phid->tagLock);
				RFIDSupport_setLatestTagString(NULL, channel, (const char*)&buffer[1]);
				mos_mutex_unlock(&phid->tagLock);
				bridgeSendToChannel(channel, BP_TAG, "%s%d", &buffer[1], buffer[0]);
				PhidgetRelease(&channel);
			}
			return EPHIDGET_OK;
		case VINT_PACKET_TYPE_RFID_TAG_LOST:
			if ((channel = getChannel(phid, 0)) != NULL) {

				bridgeSendToChannel(channel, BP_TAGLOST, "%s%d", &buffer[1], buffer[0]);
				PhidgetRelease(&channel);
			}
			return EPHIDGET_OK;
		default:
			MOS_PANIC("Unexpected packet type");
		}
#endif /* PHIDUID_1024_V2_USB_SUPPORTED */
	default:
		MOS_PANIC("Unexpected device");
	}

	//Make sure values are within defined range, and store to structure
	if (newStateData) {
		for (i = 0; i < phid->devChannelCnts.numOutputs; i++)
			phid->outputState[i] = outputs[i];

		//phid->ledEchoState = ledState;
		phid->antennaEnabled[0] = antennaState;
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetRFIDDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetRFIDDeviceHandle phid = (PhidgetRFIDDeviceHandle)ch->parent;
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	double dutyCycle;
#if PHIDUID_1024_V2_USB_SUPPORTED
	PhidgetRFID_Protocol protocol;
	PhidgetChannelHandle channel;
	PhidgetReturnCode ret;
	const char* tagString;
	size_t stringLength;
	size_t len;
	int i;
#endif

	assert(phid->phid.deviceInfo.class == PHIDCLASS_RFID);

	switch (phid->phid.deviceInfo.UDD->uid) {
#if PHIDUID_1024_V2_USB_SUPPORTED
	case PHIDUID_1024_V2_USB:
		switch (ch->class) {
		case PHIDCHCLASS_DIGITALOUTPUT:
			switch (bp->vpkt) {
			case BP_SETSTATE:
				len = 1;
				phid->outputState[ch->index] = getBridgePacketInt32(bp, 0) != 0;
				buffer[0] = phid->outputState[ch->index];
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_SETDUTYCYCLE:
				dutyCycle = getBridgePacketDouble(bp, 0);
				if (dutyCycle != 0.0 && dutyCycle != 1.0)
					return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Duty cycle must be 0 or 1."));
				len = 1;
				phid->outputState[ch->index] = dutyCycle != 0;
				buffer[0] = phid->outputState[ch->index];
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_OPENRESET:
			case BP_CLOSERESET:
				len = 1;
				buffer[0] = 0;
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, ch->uniqueIndex, buffer, &len, 100);
			case BP_ENABLE:
				return EPHIDGET_OK;
			default:
				MOS_PANIC("Unexpected packet type");
			}
		case PHIDCHCLASS_RFID:
			assert(ch->index == 0);
			switch (bp->vpkt) {
			case BP_SETANTENNAON:
				len = 1;
				buffer[0] = getBridgePacketInt32(bp, 0) != 0;
				phid->antennaState = buffer[0];
				return PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_RFID_ANTENNA_ON, ch->uniqueIndex, buffer, &len, 100);

			case BP_WRITE:
				protocol = getBridgePacketInt32(bp, 1);
				TESTRANGE_IOP(bp->iop, "%d", protocol, PROTOCOL_EM4100, PROTOCOL_PHIDGETS);
				if (phid->antennaState == 0)
					return (MOS_ERROR(bp->iop, EPHIDGET_INVALID, "Cannot write tag while antenna is off"));
				buffer[0] = getBridgePacketInt32(bp, 1);
				buffer[1] = getBridgePacketInt32(bp, 2) != 0;

				tagString = getBridgePacketString(bp, 0);
				if (protocol == PROTOCOL_EM4100 && tagString[1] == 'x')
					tagString += 2;
				stringLength = strlen(tagString);
				if (stringLength > (MAX_OUT_PACKET_SIZE - 3))
					return EPHIDGET_INVALIDARG;
				memcpy((char*)&buffer[2], tagString, stringLength);
				buffer[2 + stringLength] = '\0';
				if (protocol == PROTOCOL_EM4100) {
					for (i = 0; i < 10; i++) {
						buffer[2 + i] = mos_tolower(buffer[2 + i]);
					}
				}

				len = 2 + stringLength;
				mos_mutex_lock(&phid->tagLock);
				RFIDSupport_setLatestTagString(NULL, ch, ""); //clear latest tag
				ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_RFID_TAG_WRITE, ch->uniqueIndex, buffer, &len, 100);
				if (ret != EPHIDGET_OK)
					return ret;
				mos_mutex_unlock(&phid->tagLock);

				if ((channel = getChannel(phid, 0)) != NULL) {
					ret = RFIDSupport_waitForTag(bp->iop, channel, (const char*)&buffer[2], 500, &phid->tagLock);
					PhidgetRelease(&channel);
					return ret;
				}
				return EPHIDGET_OK;
			case BP_OPENRESET:
			case BP_CLOSERESET:
			case BP_ENABLE:
				return EPHIDGET_OK;
			default:
				MOS_PANIC("Unexpected packet type");
			}
		default:
			MOS_PANIC("Unexpected channel class");
		}
#endif /* PHIDUID_1024_V2_USB_SUPPORTED */

	default:
		switch (ch->class) {
		case PHIDCHCLASS_DIGITALOUTPUT:
			assert(ch->index < phid->devChannelCnts.numOutputs);
			switch (bp->vpkt) {
			case BP_SETSTATE:
				phid->outputStateSet[ch->index] = getBridgePacketInt32(bp, 0);
				//echo back output state if the device doesn't
				//do it here because this is after the packet has been sent off - so blocking in this event will not delay the output
				if (phid->fullStateEcho == PFALSE)
					phid->outputState[ch->index] = getBridgePacketInt32(bp, 0);
				return (_sendpacket(bp->iop, phid));
			case BP_SETDUTYCYCLE:
				dutyCycle = getBridgePacketDouble(bp, 0);
				if (dutyCycle != 0.0 && dutyCycle != 1.0)
					return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Duty cycle must be 0 or 1."));
				phid->outputStateSet[ch->index] = (int)dutyCycle;
				//echo back output state if the device doesn't
				//do it here because this is after the packet has been sent off - so blocking in this event will not delay the output
				if (phid->fullStateEcho == PFALSE)
					phid->outputState[ch->index] = (int)dutyCycle;
				return (_sendpacket(bp->iop, phid));
			case BP_OPENRESET:
			case BP_CLOSERESET:
				phid->outputStateSet[ch->index] = PFALSE;
				return (_sendpacket(bp->iop, phid));
			case BP_ENABLE:
				return (EPHIDGET_OK);
			default:
				MOS_PANIC("Unexpected packet type");
			}

		case PHIDCHCLASS_RFID:
			assert(ch->index == 0);
			switch (bp->vpkt) {
			case BP_SETANTENNAON:
				phid->antennaState = getBridgePacketInt32(bp, 0);
				//echo back state if the device doesn't
				if (phid->fullStateEcho == PFALSE)
					phid->antennaEnabled[0] = getBridgePacketInt32(bp, 0);
				return (_sendpacket(bp->iop, phid));

			case BP_WRITE:
				TESTRANGE_IOP(bp->iop, "%d", getBridgePacketInt32(bp, 1), PROTOCOL_EM4100, PROTOCOL_PHIDGETS);
				return (rfid_write(bp->iop, phid, getBridgePacketString(bp, 0), (PhidgetRFID_Protocol)getBridgePacketInt32(bp, 1), getBridgePacketInt32(bp, 2)));

			case BP_OPENRESET:
			case BP_CLOSERESET:
				phid->antennaState = PFALSE;
				return (_sendpacket(bp->iop, phid));
			case BP_ENABLE:
				return (EPHIDGET_OK);
			default:
				MOS_PANIC("Unexpected packet type");
			}

		default:
			MOS_PANIC("Unexpected channel class");
		}
	}
}

static PhidgetReturnCode
startTagTimerThread(PhidgetRFIDDeviceHandle phid) {
	PhidgetReturnCode res;

	if (!ISATTACHED((PhidgetHandle)phid))
		return (EPHIDGET_NOTATTACHED);

	/*
	 * Start the tagTimerThread -
	 * do it here because we are about to start the read thread, and that will keep it active
	 */
	mos_mutex_lock(&phid->tagLock);

	while (phid->tagThreadRun == 2) {
		mos_cond_broadcast(&phid->tagCond);
		mos_cond_wait(&phid->tagCond, &phid->tagLock);
	}
	if (phid->tagThreadRun == 1) {
		mos_mutex_unlock(&phid->tagLock);
		return (EPHIDGET_OK);
	}

	phid->tagThreadRun = 1;
	res = mos_task_create(&phid->tagTimerThread, tagTimerThreadFunction, phid);
	if (res != EPHIDGET_OK) {
		phid->tagThreadRun = 0;
		mos_mutex_unlock(&phid->tagLock);
		logerr("Failed to start tag timer thread.");
		return (res);
	}

	mos_mutex_unlock(&phid->tagLock);
	return (EPHIDGET_OK);
}

//Extra things to do during a close
//This is run before the other things that close does
static PhidgetReturnCode CCONV
PhidgetRFIDDevice_close(PhidgetDeviceHandle phidG) {
	PhidgetRFIDDeviceHandle phid = (PhidgetRFIDDeviceHandle)phidG;

	//make sure the tagTimerThread isn't running
	mos_mutex_lock(&phid->tagLock);
	if (phid->tagThreadRun != 0)
		phid->tagThreadRun = 2;
	mos_cond_broadcast(&phid->tagCond);
	mos_mutex_unlock(&phid->tagLock);

	return (EPHIDGET_OK);
}

//Extra things to do during a free
//This is run before the other things that free does
static void CCONV
PhidgetRFIDDevice_free(PhidgetDeviceHandle *phidG) {

	PhidgetRFIDDeviceHandle phid = (PhidgetRFIDDeviceHandle)*phidG;
	mos_mutex_destroy(&phid->tagLock);
	mos_cond_destroy(&phid->tagCond);
	mos_free(phid, sizeof(struct _PhidgetRFIDDevice));
	*phidG = NULL;
}

//makePacket - constructs a packet using current device state
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetRFIDDeviceHandle phid) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	int i = 0, j = 0;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1023_2OUTPUT_NO_ECHO:
	case PHIDUID_1023_2OUTPUT:
		//have to make sure that everything to be sent has some sort of default value if the user hasn't set a value
		for (i = 0; i < phid->devChannelCnts.numOutputs; i++) {
			if (phid->outputStateSet[i] == PUNK_BOOL)
				phid->outputStateSet[i] = PFALSE;
		}
		if (phid->antennaState == PUNK_BOOL)
			phid->antennaState = PFALSE;

		//construct the packet
		for (i = 0, j = 1; i < phid->devChannelCnts.numOutputs; i++, j <<= 1) {
			if (phid->outputStateSet[i])
				buffer[0] |= j;
		}
		if (phid->antennaState == PTRUE)
			buffer[0] |= RFIDDevice_ANTENNA_FLAG;
		break;
	case PHIDUID_1024:
		//have to make sure that everything to be sent has some sort of default value if the user hasn't set a value
		for (i = 0; i < phid->devChannelCnts.numOutputs; i++) {
			if (phid->outputStateSet[i] == PUNK_BOOL)
				phid->outputStateSet[i] = PFALSE;
		}
		if (phid->antennaState == PUNK_BOOL)
			phid->antennaState = PFALSE;

		// Default write timing to T5577
		if (phid->pregapClocks == PUNK_INT32)
			phid->pregapClocks = RFIDDevice_T5577_StartGap;
		if (phid->prepulseClocks == PUNK_INT32)
			phid->prepulseClocks = RFIDDevice_T5577_PrePulse;
		if (phid->zeroClocks == PUNK_INT32)
			phid->zeroClocks = RFIDDevice_T5577_Zero;
		if (phid->oneClocks == PUNK_INT32)
			phid->oneClocks = RFIDDevice_T5577_One;
		if (phid->spaceClocks == PUNK_INT32)
			phid->spaceClocks = RFIDDevice_T5577_WriteGap;
		if (phid->postgapClocks == PUNK_INT32)
			phid->postgapClocks = RFIDDevice_T5577_EndGap;
		if (phid->eofpulseClocks == PUNK_INT32)
			phid->eofpulseClocks = RFIDDevice_T5577_EOF;
		if (phid->listenDuringEOF == PUNK_BOOL)
			phid->listenDuringEOF = PFALSE;

		//construct the packet
		for (i = 0, j = 1; i < phid->devChannelCnts.numOutputs; i++, j <<= 1) {
			if (phid->outputStateSet[i])
				buffer[0] |= j;
		}
		if (phid->antennaState == PTRUE)
			buffer[0] |= RFIDDevice_ANTENNA_FLAG;

		if (phid->antennaState == PTRUE)
			phid->_4097Conf = RFIDDevice_4097_DefaultON;
		else
			phid->_4097Conf = RFIDDevice_4097_PowerDown;

		buffer[0] |= RFIDDevice_CONTROL_OUT_PACKET;
		buffer[1] = (phid->pregapClocks - 1) & 0x3F;
		buffer[1] |= ((phid->postgapClocks - 1) << 2) & 0xC0;
		buffer[2] = (phid->postgapClocks - 1) & 0x0F;
		buffer[2] |= ((phid->spaceClocks - 1) << 2) & 0xF0;
		buffer[3] = (phid->spaceClocks - 1) & 0x03;
		buffer[3] |= ((phid->zeroClocks - 1) << 1) & 0xFC;
		buffer[4] = (phid->zeroClocks - 1) & 0x01;
		buffer[4] |= ((phid->oneClocks - 1) << 1) & 0xFE;
		buffer[5] = phid->prepulseClocks;
		buffer[6] = phid->eofpulseClocks;
		buffer[7] = phid->_4097Conf;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
}

/*
 * If the time since last tag read > 200ms, fire tagLost event
 * NOTE: blocking in data events for too long will cause tagLost events
 */
MOS_TASK_RESULT
tagTimerThreadFunction(void *ctx) {
	PhidgetRFIDDeviceHandle phid;
	PhidgetChannelHandle channel;

	mos_task_setname("Phidget22 RFID Tag Timer Thread");
	loginfo("RFID tag timer thread started: 0x%08x", mos_self());

	phid = (PhidgetRFIDDeviceHandle)ctx;
	if (phid == NULL)
		MOS_TASK_EXIT(EPHIDGET_INVALIDARG);

	mos_mutex_lock(&phid->tagLock);

	for (;;) {
		if (!ISATTACHED((PhidgetHandle)phid) || phid->tagThreadRun != 1)
			break;

		//sleeps for up to 50ms, but can be signalled externally to return immediately
		mos_cond_timedwait(&phid->tagCond, &phid->tagLock, 50 * MOS_MSEC);

		//Tag events
		if (phid->tagEventPending) {
			if ((channel = getChannel(phid, 0)) != NULL) {
				mos_mutex_unlock(&phid->tagLock);
				bridgeSendToChannel(channel, BP_TAG, "%s%d", phid->pendingTag.tagString, phid->pendingTag.protocol);
				mos_mutex_lock(&phid->tagLock);
				PhidgetRelease(&channel);
			}
			phid->lastTag = phid->pendingTag;
			phid->lastTagValid = PTRUE;
			phid->tagEventPending = PFALSE;
		}

		//TAG Lost events
		if (phid->tagPresent[0] != PFALSE) {
			/* check for tag lost */
			if (mos_gettime_usec() - phid->lastTagTime > 200000) {
				if (phid->tagPresent[0] == PTRUE) {
					phid->tagPresent[0] = PFALSE;
					if ((channel = getChannel(phid, 0)) != NULL) {
						mos_mutex_unlock(&phid->tagLock);
						bridgeSendToChannel(channel, BP_TAGLOST, "%s%d", phid->lastTag.tagString, phid->lastTag.protocol);
						mos_mutex_lock(&phid->tagLock);
						PhidgetRelease(&channel);
					}
				//could be PUNK_BOOL - don't send event, just set to PFALSE (but only if the antenna is on)
				} else if (phid->antennaEnabled[0] == PTRUE) {
					phid->tagPresent[0] = PFALSE;
				}
			}
		}
	}

	phid->tagThreadRun = 0;
	mos_cond_broadcast(&phid->tagCond);	// notify anybody waiting for us to exit
	mos_mutex_unlock(&phid->tagLock);

	loginfo("tagTimerThread exiting normally");
	MOS_TASK_EXIT(EPHIDGET_OK);
}

static PhidgetReturnCode
tagEvent(PhidgetRFIDDeviceHandle phid, PhidgetRFIDDevice_TagHandle tagPtr) {
	PhidgetChannelHandle channel;

again:
	mos_mutex_lock(&phid->tagLock);

	//update time
//	setTimeNow(&phid->lastTagTime);
	phid->lastTagTime = mos_gettime_usec();

	//See if there is a current tag, and if it matches this tag
	if (phid->tagPresent[0] != PTRUE ||
	  (phid->lastTagValid && strcmp(phid->lastTag.tagString, tagPtr->tagString) &&
		  phid->lastTag.protocol == tagPtr->protocol)) {
		   //Wait for tagEventPending to be false if it's true
		if (phid->tagEventPending == PTRUE) {
			mos_mutex_unlock(&phid->tagLock);
			mos_usleep(10000);
			goto again;
		}

		phid->pendingTag = *tagPtr;
		phid->tagEventPending = PTRUE;

		if ((channel = getChannel(phid, 0)) != NULL) {
			RFIDSupport_setLatestTagString(NULL, channel, phid->pendingTag.tagString);
			PhidgetRelease(&channel);
		}
		mos_cond_broadcast(&phid->tagCond);
	}

	phid->tagPresent[0] = PTRUE;

	mos_mutex_unlock(&phid->tagLock);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
tagEvent_fromEM4100Data(PhidgetRFIDDeviceHandle phid, uint8_t *data) {
	PhidgetRFIDDevice_Tag tag = { 0 };
	snprintf(tag.tagString, RFIDDevice_MAX_TAG_STRING_LEN, "%02x%02x%02x%02x%02x", data[0], data[1], data[2], data[3], data[4]);
	tag.protocol = PROTOCOL_EM4100;
	return tagEvent(phid, &tag);
}

static PhidgetReturnCode
sendRAWData(mosiop_t iop, PhidgetRFIDDeviceHandle phid, uint8_t *data, int bitlength) {
	uint8_t buffer[8] = { 0 };
	int i, j, bits;
	PhidgetReturnCode result;

	int length = bitlength / 8 + ((bitlength % 8) ? 1 : 0);
	if (length > 0xff)
		return (EPHIDGET_INVALIDARG);

	bits = 0;
	for (i = 0, j = 1; i < length; i++, j++) {
		buffer[j] = data[i];
		if (bitlength < 8) {
			bits += bitlength;
			bitlength = 0;
		} else {
			bits += 8;
			bitlength -= 8;
		}
		if (j == 7 || i == (length - 1)) {
			buffer[0] = RFIDDevice_WRITE_DATA_OUT_PACKET | bits;
			if ((result = PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, j + 1)) != EPHIDGET_OK)
				return result;
			j = 0;
			bits = 0;
		}
	}

	return (EPHIDGET_OK);
}

static void
resetValuesFromSpace(PhidgetRFIDDeviceHandle phid) {
	phid->shortClocks = phid->longClocks = 0;
	phid->manLockedIn = 0;
	phid->manReadPtr = 0;
	phid->manWritePtr = 0;
	phid->biphaseReadPtr = 0;
	phid->biphaseWritePtr = 0;
	phid->manShortChange = 0;
	phid->biphaseShortChange = 0;
	phid->biphaseLockedIn = 0;

#ifdef RFIDDevice_RAWDATA_API_SUPPORT
	phid->manEventReadPtr = 0;

	//Manchester event with space
	if (phid->lastManEventLong == PFALSE) {
		uint8_t manEventData[1];
		phid->lastManEventLong = PTRUE;
		manEventData[0] = PUNK_BOOL;
		if ((channel = getChannel(phid, 0))) {
			bridgeSendToChannel(channel, BP_MANCHESTER, "%1R%d", manEventData, 1);
			PhidgetRelease(&channel);
		}
	}
#endif
}

//ISO11785 CRC
static void
CRC_16_CCITT_update(uint16_t *crc, uint8_t x) {
	uint16_t crc_new = (uint8_t)((*crc) >> 8) | ((*crc) << 8);
	crc_new ^= x;
	crc_new ^= (uint8_t)(crc_new & 0xff) >> 4;
	crc_new ^= crc_new << 12;
	crc_new ^= (crc_new & 0xff) << 5;
	(*crc) = crc_new;
}

//Reverse all bits
static uint64_t
reverse(uint64_t x) {
	x = (((x & 0xaaaaaaaaaaaaaaaaLL) >> 1) | ((x & 0x5555555555555555LL) << 1));
	x = (((x & 0xccccccccccccccccLL) >> 2) | ((x & 0x3333333333333333LL) << 2));
	x = (((x & 0xf0f0f0f0f0f0f0f0LL) >> 4) | ((x & 0x0f0f0f0f0f0f0f0fLL) << 4));
	x = (((x & 0xff00ff00ff00ff00LL) >> 8) | ((x & 0x00ff00ff00ff00ffLL) << 8));
	x = (((x & 0xffff0000ffff0000LL) >> 16) | ((x & 0x0000ffff0000ffffLL) << 16));
	return((x >> 32) | (x << 32));
}

/* Takes the tagString in 10-digit hex, and produces data for programming.
 *  blockData are 32-bit blocks.
 *  blockDataLen will be 2 when we return, as EM4100 takes 64-bits.
 */
static PhidgetReturnCode
encodeEM4100(mosiop_t iop, const char *tagString, uint32_t *blockData, int *blockDataLen) {
	int64_t tagData, mask;
	uint64_t encodedTagData = 0;
	int i, j, row, col;
	assert(tagString);
	assert(blockData);
	assert(blockDataLen);
	if (*blockDataLen < 2)
		return (EPHIDGET_INVALIDARG);
	if (strlen(tagString) != 10)
		if (!(strlen(tagString) == 12 && tagString[0] == '0' && tagString[1] == 'x'))
			return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "EM4100 tag string must be specified as 'XXXXXXXXXX' or '0xXXXXXXXXXX' where 'X' is a hex digit."));

	tagData = strtoll(tagString, NULL, 16);

	//9 leading 1's
	encodedTagData = 0xFF80000000000000ULL;
	//Data
	for (i = 0, mask = 0x0078000000000000LL, j = 15; i < 10; i++, j--, mask >>= 5) {
		encodedTagData |= ((tagData << j) & mask);
	}
	//Parity
	for (i = 0, row = 1, col = 1; i < 40; i++, col++) {
		if (col > 4) {
			col = 1; row++;
		}

//column parity
		encodedTagData ^= ((tagData & 0x1) << col);
		//row parity
		encodedTagData ^= ((tagData & 0x1) << (row * 5));

		tagData >>= 1;
	}

	blockData[0] = (int)(encodedTagData >> 32);
	blockData[1] = (int)encodedTagData;
	*blockDataLen = 2;

	return (EPHIDGET_OK);
}

/* Takes the tagString in 15-digit decimal, and produces data for programming.
 *  blockData are 32-bit blocks.
 *  blockDataLen will be 4 when we return, as FDX-B takes 128-bits.
 */
static PhidgetReturnCode
encodeISO11785_FDX_B(mosiop_t iop, const char *tagString, uint32_t *blockData, int *blockDataLen) {
	uint64_t tagData, encodedTagData[2] = { 0,0 }, mask, uniqueID;
	uint16_t crc = 0x0000;
	uint32_t countryCode;
	char countryCodeStr[4];
	int i, j;
	assert(tagString);
	assert(blockData);
	assert(blockDataLen);
	if (*blockDataLen < 4)
		return (EPHIDGET_INVALIDARG);
	if (strlen(tagString) != 15)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "ISO11785_FDX_B tag must be specified as 15 decimal digits."));

	//Get uniqueID
	uniqueID = (uint64_t)strtoll(tagString + 3, NULL, 10);
	//must be 38-bit or less
	if (uniqueID > 0x3FFFFFFFFFLL)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "ISO11785_FDX_B Unique ID (last 12 digits) must be <= 274877906943."));

	//Get Country Code
	memcpy(countryCodeStr, tagString, 3);
	countryCodeStr[3] = '\0';
	countryCode = (uint32_t)strtoul(countryCodeStr, NULL, 10);

	//Create ISO11784 64-bit data
	tagData = (((uint64_t)countryCode) << 38) | uniqueID;
	//Add the animal bit
	tagData |= 0x8000000000000000LL;
	//Reverse because order is LSB 1st.
	tagData = reverse(tagData);
	//Calculate CRC
	for (i = 0, j = 7 * 8; i < 8; i++, j -= 8)
		CRC_16_CCITT_update(&crc, (uint8_t)((tagData >> j) & 0xFF));

	//Put it into the FDX-B Format

	//Header and control bits
	encodedTagData[0] = 0x0020100804020100ULL;
	encodedTagData[1] = 0x8040201008040201ULL;
	//Data
	for (i = 0, mask = 0x001FE00000000000LL, j = 11; i < 6; i++, mask >>= 9, j++)
		encodedTagData[0] |= ((tagData >> j) & mask);
	for (i = 0, mask = 0x7F80000000000000LL, j = 47; i < 2; i++, mask >>= 9, j--)
		encodedTagData[1] |= ((tagData << j) & mask);
	//CRC
	encodedTagData[1] |= ((((uint64_t)crc) << 29) & 0x00001FE000000000LL);
	encodedTagData[1] |= ((((uint64_t)crc) << 28) & 0x0000000FF0000000LL);

	blockData[0] = (int)(encodedTagData[0] >> 32);
	blockData[1] = (int)encodedTagData[0];
	blockData[2] = (int)(encodedTagData[1] >> 32);
	blockData[3] = (int)encodedTagData[1];
	*blockDataLen = 4;

	return (EPHIDGET_OK);
}

/* Takes the tagString in ASCII up to 24 characters, and produces data for programming.
 *  blockData are 32-bit blocks.
 *  blockDataLen will be 7 when we return, as PHIDGETS_TAG takes 224-bits.
 */
static PhidgetReturnCode
encodePHIDGETS_TAG(mosiop_t iop, const char *tagString, uint32_t *blockData, int *blockDataLen) {
	char tagData[24];
	int len, i, j;
	uint16_t crc;
	assert(tagString);
	assert(blockData);
	assert(blockDataLen);
	if (*blockDataLen < 7)
		return (EPHIDGET_INVALIDARG);
	if (strlen(tagString) > 24)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "PHIDGETS_TAG must be <= 24 characters."));

	//copy of string.
	len = (int)strlen(tagString);
	memset(tagData, 0, 24);
	memcpy(tagData, tagString, len);

	//Calculate CRC
	crc = 0;
	for (i = 0; i < 24; i++)
		CRC_16_CCITT_update(&crc, (uint8_t)tagData[i]);

	//Header, control bits and CRC
	blockData[0] = 0x00040201;
	blockData[0] |= (((uint32_t)crc) << 2 & 0x0003FC00);
	blockData[0] |= (((uint32_t)crc) << 1 & 0x000001FE);
	//Control bits
	for (i = 1; i < 7; i++)
		blockData[i] = 0x01010101;
	//Data
	for (i = 0, j = 184; i < 24; i++, j -= 8) {
		int block = i / 4 + 1;
		int shift = (j % 32) + 1;
		blockData[block] |= (tagData[i] << shift);
	}

	*blockDataLen = 7;

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
decodeEM4100(PhidgetRFIDDeviceHandle phid, uint8_t *data, uint32_t *startPtr, uint32_t *endPtr, PhidgetRFIDDevice_TagHandle tag) {
	int i, foundStart, k, j;
	uint32_t myReadPtr = *startPtr;
	int em4103data[64];
	uint8_t decodedData[5];
	int bytesInQueue;
	//Look for the starting pattern of 9 Ones
start:
	bytesInQueue = *endPtr - myReadPtr;
	if (myReadPtr > *endPtr)
		bytesInQueue += RFIDDevice_DATA_ARRAY_SIZE;

	while (myReadPtr != *endPtr) {
		if (bytesInQueue < 64)
			return (EPHIDGET_NOENT);
		foundStart = 1;

		for (i = 0; i < 9; i++) {
			if (data[(myReadPtr + i) & RFIDDevice_DATA_ARRAY_MASK] != 1) {
				foundStart = 0;
				break;
			}
		}
		if (foundStart)
			break;

		myReadPtr++;
		myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;

		bytesInQueue--;
	}

	//Got here? - We found the start pattern
	//Now decode the EM4100 data
	for (i = 0; i < 64; i++) {
		em4103data[i] = data[(myReadPtr + i) & RFIDDevice_DATA_ARRAY_MASK];
	}

	//last bit should be zero (stop bit)
	if (em4103data[63] != 0)
		goto tryagain;

	//row Parity
	for (i = 0; i < 10; i++) {
		int rowParity = 0;
		for (k = 0; k < 4; k++)
			rowParity ^= em4103data[9 + i * 5 + k];
		if (rowParity != em4103data[9 + i * 5 + k])
			goto tryagain;
	}
	//column parity
	for (i = 0; i < 4; i++) {
		int colParity = 0;
		for (k = 0; k < 10; k++)
			colParity ^= em4103data[9 + i + k * 5];
		if (colParity != em4103data[9 + i + k * 5])
			goto tryagain;
	}

	//We're good! Strip out data
	memset(decodedData, 0, 5);
	for (i = 0, j = 9; i < 5; i++) {
		for (k = 7; k >= 4; k--, j++)
			decodedData[i] |= em4103data[j] << k;
		j++; //skip row parity bit
		for (k = 3; k >= 0; k--, j++)
			decodedData[i] |= em4103data[j] << k;
		j++; //skip row parity bit
	}


	//Update the tag struct for the tag event
	snprintf(tag->tagString, RFIDDevice_MAX_TAG_STRING_LEN, "%02x%02x%02x%02x%02x", decodedData[0], decodedData[1], decodedData[2], decodedData[3], decodedData[4]);
	tag->protocol = PROTOCOL_EM4100;

	//update master read pointer
	(*startPtr) += 64;
	(*startPtr) &= RFIDDevice_DATA_ARRAY_MASK;
	return (EPHIDGET_OK);

tryagain:
	myReadPtr++;
	myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
	goto start;
}

static PhidgetReturnCode
decodeISO11785_FDX_B(PhidgetRFIDDeviceHandle phid, uint8_t *data, uint32_t *startPtr, uint32_t *endPtr, PhidgetRFIDDevice_TagHandle tag) {
	int i, foundStart, k;
	uint32_t myReadPtr = *startPtr;
	uint8_t iso11785data[8];
	uint8_t iso11785dataReversed[8];
	uint16_t iso11785checksum;
	int bytesInQueue;
	uint16_t crc = 0x0000;
	//Look for the starting pattern of 10 zeroes and 1 one
start:
	bytesInQueue = *endPtr - myReadPtr;
	if (myReadPtr > *endPtr)
		bytesInQueue += RFIDDevice_DATA_ARRAY_SIZE;

	while (myReadPtr != *endPtr) {
		//full sequence is 128 bits
		if (bytesInQueue < 128)
			return (EPHIDGET_NOENT);
		foundStart = 1;

		for (i = 0; i < 10; i++) {
			if (data[(myReadPtr + i) & RFIDDevice_DATA_ARRAY_MASK] != 0) {
				foundStart = 0;
				break;
			}
		}
		if (data[(myReadPtr + 10) & RFIDDevice_DATA_ARRAY_MASK] != 1) {
			foundStart = 0;
		}
		if (foundStart)
			break;

		myReadPtr++;
		myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;

		bytesInQueue--;
	}

	//advance past header
	myReadPtr += 11;
	myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;

	//Got here? - We found the start pattern
	//Now decode the ISO11785 data
	//every block of 8 is followed by a '1'
	memset(iso11785data, 0, 8);
	memset(iso11785dataReversed, 0, 8);
	for (i = 0, k = 0; i < 64; i++, k++) {
		if (i > 0 && i % 8 == 0) {
			if (data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] != 1) goto tryagain;
			k++;
		}
		iso11785data[i / 8] |= data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] << (7 - (i % 8));
		iso11785dataReversed[7 - (i / 8)] |= data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] << (i % 8);
	}
	if (data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] != 1) goto tryagain;
	k++;

	//Now the checksum
	iso11785checksum = 0;
	for (i = 0; i < 16; i++, k++) {
		if (i > 0 && i % 8 == 0) {
			if (data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] != 1) goto tryagain;
			k++;
		}
		iso11785checksum |= data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] << (15 - i);
	}
	if (data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] != 1) goto tryagain;
	k++;

	//TODO: there is also a 24 bit trailer which can contain extra info

	//Checksum
	crc = 0x0000;
	for (i = 0; i < 8; i++) {
		CRC_16_CCITT_update(&crc, iso11785data[i]);
	}

	if (crc != iso11785checksum) {
		//FAIL
		goto tryagain;
	}

	//We're good!
	//Parse out the different sections
	{
		//These are not used for now
		//uint8_t animal = iso11785dataReversed[0] & 0x80 ? PTRUE : PFALSE; //1 bit - bit 0
		//uint8_t extraData = iso11785dataReversed[1] & 0x01 ? PTRUE : PFALSE; //1 bit - bit 15
		uint16_t countryCode = (iso11785dataReversed[2] << 2 | iso11785dataReversed[3] >> 6) & 0x3ff; //10 bit - bits 16-26
		uint64_t UID = (
			(((uint64_t)iso11785dataReversed[3]) << 32) +
			(((uint64_t)iso11785dataReversed[4]) << 24) +
			(((uint64_t)iso11785dataReversed[5]) << 16) +
			(((uint64_t)iso11785dataReversed[6]) << 8) +
			((uint64_t)iso11785dataReversed[7])) & 0x3FFFFFFFFFll;// 38 bit - bits 27-63

		snprintf(tag->tagString, RFIDDevice_MAX_TAG_STRING_LEN, "%03u%012"PRIu64, countryCode, UID);
		tag->protocol = PROTOCOL_ISO11785_FDX_B;
	}

	//update master read pointer
	(*startPtr) += 128;
	(*startPtr) &= RFIDDevice_DATA_ARRAY_MASK;
	return (EPHIDGET_OK);

tryagain:
	myReadPtr++;
	myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
	goto start;
}

static PhidgetReturnCode
decodePHIDGETS_TAG(PhidgetRFIDDeviceHandle phid, uint8_t *data, uint32_t *startPtr, uint32_t *endPtr, PhidgetRFIDDevice_TagHandle tag) {
	int i, foundStart, k;
	uint32_t myReadPtr = *startPtr;
	char tagData[24];
	int bytesInQueue;
	uint16_t crcCalc = 0x0000, crcRead = 0x0000;
	//Look for the starting pattern of 13 zeroes and 1 one
start:
	bytesInQueue = *endPtr - myReadPtr;
	if (myReadPtr > *endPtr)
		bytesInQueue += RFIDDevice_DATA_ARRAY_SIZE;

	while (myReadPtr != *endPtr) {
		//full sequence is 224 bits
		if (bytesInQueue < 224)
			return (EPHIDGET_NOENT);
		foundStart = 1;

		for (i = 0; i < 13; i++) {
			if (data[(myReadPtr + i) & RFIDDevice_DATA_ARRAY_MASK] != 0) {
				foundStart = 0;
				break;
			}
		}
		if (data[(myReadPtr + 13) & RFIDDevice_DATA_ARRAY_MASK] != 1) {
			foundStart = 0;
		}
		if (foundStart)
			break;

		myReadPtr++;
		myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;

		bytesInQueue--;
	}

	//advance past header
	myReadPtr += 14;
	myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;

	//Got here? - We found the start pattern

	//Pull out the CRC
	for (k = 0, i = 0; i < 16; i++, k++) {
		if (i > 0 && i % 8 == 0) {
			if (data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] != 1)
				goto tryagain;
			k++;
		}
		crcRead |= data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] << (15 - i);
	}
	if (data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] != 1)
		goto tryagain;
	k++;

	//Now decode the PHIDGETS_TAG data
	//every block of 7 is followed by a '1'
	memset(tagData, 0, 24);
	for (i = 0; i < 192; i++, k++) {
		//Check for control bit
		if ((i + 1) % 8 == 0) {
			if (data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] != 1)
				goto tryagain;
		}
		//pull out data
		else {
			tagData[i / 8] |= data[(myReadPtr + k) & RFIDDevice_DATA_ARRAY_MASK] << (6 - (i % 8));
		}
	}

	//Calculate CRC
	crcCalc = 0;
	for (i = 0; i < 24; i++)
		CRC_16_CCITT_update(&crcCalc, (uint8_t)tagData[i]);

	if (crcCalc != crcRead) {
		//FAIL
		goto tryagain;
	}

	//We're good!
	memcpy(tag->tagString, tagData, 24);
	tag->tagString[24] = '\0';
	tag->protocol = PROTOCOL_PHIDGETS;

	//update master read pointer
	(*startPtr) += 224;
	(*startPtr) &= RFIDDevice_DATA_ARRAY_MASK;
	return (EPHIDGET_OK);

tryagain:
	myReadPtr++;
	myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
	goto start;
}

static PhidgetReturnCode
add_biphase_data(PhidgetRFIDDeviceHandle phid, int readToPtr) {
	int myReadPtr = phid->dataReadPtr;
	while (myReadPtr != readToPtr) {
		int clocks = phid->dataBuffer[myReadPtr] & 0xFF;

		if (clocks == phid->longClocks) {

			phid->biphaseBuffer[phid->biphaseWritePtr] = 1;

			phid->biphaseWritePtr++;
			phid->biphaseWritePtr &= RFIDDevice_DATA_ARRAY_MASK;

			if (phid->biphaseWritePtr == phid->biphaseReadPtr) {
				phid->biphaseReadPtr++;
				phid->biphaseReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
			}

			phid->biphaseLockedIn = 1;
			phid->biphaseShortChange = 0;
		} else if (clocks == phid->shortClocks) {
			if (phid->biphaseLockedIn && phid->biphaseShortChange) {
				phid->biphaseBuffer[phid->biphaseWritePtr] = 0;

				phid->biphaseWritePtr++;
				phid->biphaseWritePtr &= RFIDDevice_DATA_ARRAY_MASK;

				if (phid->biphaseWritePtr == phid->biphaseReadPtr) {
					phid->biphaseReadPtr++;
					phid->biphaseReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
				}

				phid->biphaseShortChange = 0;
			} else
				phid->biphaseShortChange = 1;
		} else {
			phid->biphaseLockedIn = 0;
			//invalid
			phid->biphaseReadPtr = phid->biphaseWritePtr;
			//This is not BiPhase encoded data
			return (EPHIDGET_NOENT);
		}

		myReadPtr++;
		myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
	}
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
add_manchester_data(PhidgetRFIDDeviceHandle phid, int readToPtr) {
	int myReadPtr = phid->dataReadPtr;
	while (myReadPtr != readToPtr) {
		int clocks = phid->dataBuffer[myReadPtr] & 0xFF;
		int polarity = (phid->dataBuffer[myReadPtr] & 0x100) ? 1 : 0;

		if (clocks == phid->longClocks) {
			//We're out of sync - re-sync
			if (phid->manShortChange) {
				phid->manReadPtr = phid->manWritePtr;
				phid->manReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
#ifdef RFIDDevice_RAWDATA_API_SUPPORT
				phid->manEventReadPtr = phid->manWritePtr;
				phid->manEventReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
#endif
			}

			if (polarity)
				phid->manBuffer[phid->manWritePtr] = 1;
			else
				phid->manBuffer[phid->manWritePtr] = 0;

			phid->manWritePtr++;
			phid->manWritePtr &= RFIDDevice_DATA_ARRAY_MASK;

			//TODO: is there a danger of these actually happening??
			if (phid->manWritePtr == phid->manReadPtr) {
				phid->manReadPtr++;
				phid->manReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
			}

#ifdef RFIDDevice_RAWDATA_API_SUPPORT
			if (phid->manWritePtr == phid->manEventReadPtr) {
				phid->manEventReadPtr++;
				phid->manEventReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
			}
#endif

			phid->manLockedIn = 1;
			phid->manShortChange = 0;
		}
		// Short clocks
		else {
			if (!phid->manLockedIn) {
				if (polarity) {
					phid->manShortChange = 1;
					phid->manLockedIn = 1;
				}
			} else {
				if (phid->manShortChange) {
					if (polarity)
						phid->manBuffer[phid->manWritePtr] = 1;
					else
						phid->manBuffer[phid->manWritePtr] = 0;

					phid->manWritePtr++;
					phid->manWritePtr &= RFIDDevice_DATA_ARRAY_MASK;

					if (phid->manWritePtr == phid->manReadPtr) {
						phid->manReadPtr++;
						phid->manReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
					}

#ifdef RFIDDevice_RAWDATA_API_SUPPORT
					if (phid->manWritePtr == phid->manEventReadPtr) {
						phid->manEventReadPtr++;
						phid->manEventReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
					}
#endif

					phid->manShortChange = 0;
				} else
					phid->manShortChange = 1;
			}
		}

		myReadPtr++;
		myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
	}
	return (EPHIDGET_OK);
}

//Analyses streaming data in Manchester or Biphase coding
static PhidgetReturnCode
analyze_data(PhidgetRFIDDeviceHandle phid) {
	int bytesToRead = 0, bytesRead = 0;
	int temp, clocks;
	uint32_t myReadPtr;
	PhidgetRFIDDevice_Tag tag = { 0 };

	//read till we have real data
start:
	while (phid->dataReadPtr != phid->dataWritePtr) {
		if (phid->dataBuffer[phid->dataReadPtr] == PUNK_INT32) {
			phid->dataReadPtr++;
			phid->dataReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
			resetValuesFromSpace(phid);
		} else
			break;
	}

	//Make sure we have enough data to do something useful with..
	bytesToRead = phid->dataWritePtr - phid->dataReadPtr;
	if (phid->dataReadPtr > phid->dataWritePtr)
		bytesToRead += RFIDDevice_DATA_ARRAY_SIZE;

	//then read till we have a space or run out of data - figure out data rate
	bytesRead = 0;
	myReadPtr = phid->dataReadPtr;
	while (myReadPtr != phid->dataWritePtr) {
		if (phid->dataBuffer[myReadPtr] == PUNK_INT32)
			break;

		clocks = (phid->dataBuffer[myReadPtr] & 0xFF);

		if (!phid->shortClocks) {
			phid->shortClocks = clocks;
		} else if (clocks != phid->shortClocks) {
			if (!phid->longClocks) {
				if (phid->shortClocks * 2 == clocks || phid->shortClocks / 2 == clocks)
					phid->longClocks = clocks;
				else
					//found a values that doesn't work - error
					goto update_readPtr_restart;
			} else if (clocks != phid->longClocks)
				//found a values that doesn't work - error
				goto update_readPtr_restart;
		}

		myReadPtr++;
		myReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
		bytesRead++;
	}

	//We haven't seen a one and a two!!
	if (phid->shortClocks == 0 || phid->longClocks == 0) {
		//got a gap? move on to more data
		if (bytesRead != bytesToRead)
			goto check_done;

		//otherwise, wait for more data
		return (EPHIDGET_OK);
	}

	//Order them by size
	if (phid->longClocks < phid->shortClocks) {
		temp = phid->longClocks;
		phid->longClocks = phid->shortClocks;
		phid->shortClocks = temp;
	}

	//Shift data into Manchester and Biphase decoders, update read ptr
	if (!add_manchester_data(phid, myReadPtr)) {
#ifdef RFIDDevice_RAWDATA_API_SUPPORT
		uint8_t manEventData[RFIDDevice_DATA_ARRAY_SIZE];
		int manEventDataWritePtr = 0;
#endif

		if (!decodeEM4100(phid, phid->manBuffer, &phid->manReadPtr, &phid->manWritePtr, &tag))
			tagEvent(phid, &tag);

#ifdef RFIDDevice_RAWDATA_API_SUPPORT
		//Manchester data event
		while (phid->manEventReadPtr != phid->manWritePtr) {
			manEventData[manEventDataWritePtr++] = phid->manBuffer[phid->manEventReadPtr];
			phid->manEventReadPtr++;
			phid->manEventReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
		}
		if (manEventDataWritePtr) {
			phid->lastManEventLong = PFALSE;
			if ((channel = getChannel(phid, 0))) {
				bridgeSendToChannel(channel, BP_MANCHESTER, "%*R%d", (int)manEventDataWritePtr, manEventData, manEventDataWritePtr);
				PhidgetRelease(&channel);
			}
		}
	}
	//Manchester event with space
	else if (phid->lastManEventLong == PFALSE) {
		uint8_t manEventData[1];
		phid->lastManEventLong = PTRUE;
		manEventData[0] = PUNK_BOOL;
		if ((channel = getChannel(phid, 0))) {
			bridgeSendToChannel(channel, BP_MANCHESTER, "%1R%d", manEventData, 1);
			PhidgetRelease(&channel);
	}
#endif
}

	if (!add_biphase_data(phid, myReadPtr)) {
		if (!decodePHIDGETS_TAG(phid, phid->biphaseBuffer, &phid->biphaseReadPtr, &phid->biphaseWritePtr, &tag))
			tagEvent(phid, &tag);
		if (!decodeISO11785_FDX_B(phid, phid->biphaseBuffer, &phid->biphaseReadPtr, &phid->biphaseWritePtr, &tag))
			tagEvent(phid, &tag);
	}

check_done:
	//update read pointer
	phid->dataReadPtr = myReadPtr;

	//If there is more data, loop around
	if (phid->dataReadPtr != phid->dataWritePtr)
		goto start;

	return (EPHIDGET_OK);

	//ran into a bad pulse length or a gap - reset stuff
update_readPtr_restart:
	phid->dataReadPtr++;
	phid->dataReadPtr &= RFIDDevice_DATA_ARRAY_MASK;
	resetValuesFromSpace(phid);

	goto start;
}

static PhidgetReturnCode
T5577_WriteBlock(mosiop_t iop, PhidgetRFIDDeviceHandle phid, int page, int block, uint32_t data, int lockpage) {
	PhidgetReturnCode res = EPHIDGET_OK;
	uint8_t byteData[5];

	byteData[0] = ((0x02 | (page & 0x01)) << 6); //OpCode
	byteData[0] |= ((lockpage ? 0x01 : 0x00) << 5); //Lock
	byteData[0] |= ((data >> 27) & 0x1F); //Data 31:27
	byteData[1] = ((data >> 19) & 0xFF); //Data 26:19
	byteData[2] = ((data >> 11) & 0xFF); //Data 18:11
	byteData[3] = ((data >> 3) & 0xFF); //Data 10:3
	byteData[4] = ((data << 5) & 0xE0); //Data 2:0
	byteData[4] |= ((block & 0x07) << 2); //Block (0-7)

	res = rfid_writeRaw(iop, phid, byteData, 38,
		RFIDDevice_T5577_StartGap,
		RFIDDevice_T5577_WriteGap,
		RFIDDevice_T5577_EndGap,
		RFIDDevice_T5577_Zero,
		RFIDDevice_T5577_One,
		RFIDDevice_T5577_PrePulse,
		RFIDDevice_T5577_EOF,
		PFALSE);

	return res;
}

static PhidgetReturnCode
T5577_Reset(mosiop_t iop, PhidgetRFIDDeviceHandle phid) {
	PhidgetReturnCode res = EPHIDGET_OK;
	uint8_t byteData[1];

	byteData[0] = 0x00; //OpCode

	res = rfid_writeRaw(iop, phid, byteData, 2,
		RFIDDevice_T5577_StartGap,
		RFIDDevice_T5577_WriteGap,
		RFIDDevice_T5577_EndGap,
		RFIDDevice_T5577_Zero,
		RFIDDevice_T5577_One,
		RFIDDevice_T5577_PrePulse,
		RFIDDevice_T5577_EOF,
		PFALSE);

	return res;
}

/* Programs a T5577 using the specified protocol and data
 *  data: Tag string.
 *  lock: lock the T5577 so it cannot be reprogrammed.
 */
static PhidgetReturnCode
T5577_WriteTag(mosiop_t iop, PhidgetRFIDDeviceHandle phid, PhidgetRFID_Protocol protocol, const char *tagString, int lock) {
	PhidgetReturnCode ret = EPHIDGET_OK;
	int i;
	uint32_t data[7];
	int dataLen = 7;
	uint32_t configBlock;

	switch (protocol) {
	case PROTOCOL_EM4100:
		if ((ret = encodeEM4100(iop, tagString, data, &dataLen)) != EPHIDGET_OK)
			return ret;
		configBlock = 0x00148040; // RF/63, manchester, Maxblock = 2
		break;
	case PROTOCOL_ISO11785_FDX_B:
		if ((ret = encodeISO11785_FDX_B(iop, tagString, data, &dataLen)) != EPHIDGET_OK)
			return ret;
		configBlock = 0x603F8080; // RF/32, differential bi-phase, Maxblock = 4
		break;
	case PROTOCOL_PHIDGETS:
		if ((ret = encodePHIDGETS_TAG(iop, tagString, data, &dataLen)) != EPHIDGET_OK)
			return ret;
		configBlock = 0x603F80E0; // RF/32, differential bi-phase, Maxblock = 7
		break;
	default:
		return (EPHIDGET_INVALIDARG);
	}

	//Write Data
	for (i = 0; i < dataLen; i++) {
		if ((ret = T5577_WriteBlock(iop, phid, 0, i + 1, data[i], lock)) != EPHIDGET_OK)
			return ret;
		mos_usleep(50000); //some time beetween writes!
	}
	//Write config
	if ((ret = T5577_WriteBlock(iop, phid, 0, 0, configBlock, lock)) != EPHIDGET_OK)
		return ret;

	mos_usleep(50000); //some time beetween writes!
	//Reset Chip
	if ((ret = T5577_Reset(iop, phid)) != EPHIDGET_OK)
		return ret;

	return ret;
}

static PhidgetReturnCode
rfid_writeRaw(mosiop_t iop, PhidgetRFIDDeviceHandle phid, uint8_t *data, int bitlength,
	int pregap, int space, int postgap, int zero, int one, int prepulse, int eof, int listenDuringEOF) {
	PhidgetReturnCode res;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1024:
		//limit spaces to 64 clocks
		if (pregap < 2 || pregap > 64)
			return (EPHIDGET_INVALIDARG);
		if (space < 2 || space > 64)
			return (EPHIDGET_INVALIDARG);
		if (postgap < 2 || postgap > 64)
			return (EPHIDGET_INVALIDARG);

		//pulses can be up to 256 clocks
		if (zero < 4 || zero > 128)
			return (EPHIDGET_INVALIDARG);
		if (one < 4 || one > 128)
			return (EPHIDGET_INVALIDARG);
		if (prepulse < 0 || prepulse > 255)
			return (EPHIDGET_INVALIDARG);
		if (eof < 0 || eof > 255)
			return (EPHIDGET_INVALIDARG);
		if (listenDuringEOF < PFALSE || listenDuringEOF > PTRUE)
			return (EPHIDGET_INVALIDARG);

		phid->pregapClocks = pregap;
		phid->postgapClocks = postgap;
		phid->spaceClocks = space;
		phid->zeroClocks = zero;
		phid->oneClocks = one;
		phid->prepulseClocks = prepulse;
		phid->eofpulseClocks = eof;
		phid->listenDuringEOF = listenDuringEOF;

		//Send timing
		res = _sendpacket(iop, phid);
		if (res != 0)
			return (res);

		res = sendRAWData(iop, phid, data, bitlength);
		if (res != 0)
			return (res);

		return (EPHIDGET_OK);

	default:
		return (EPHIDGET_UNSUPPORTED);
	}
}

static PhidgetReturnCode
rfid_write(mosiop_t iop, PhidgetRFIDDeviceHandle phid, const char *tagString, PhidgetRFID_Protocol protocol, int lock) {
	PhidgetReturnCode ret = EPHIDGET_OK;
	PhidgetChannelHandle channel;
	assert(tagString);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1024:
		if (protocol < (PROTOCOL_EM4100) || protocol >(PROTOCOL_PHIDGETS))
			return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Invalid protocol."));
		if (lock < (PFALSE) || lock >(PTRUE))
			return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Lock must be 0 or 1."));

		mos_mutex_lock(&phid->tagLock);
		ret = T5577_WriteTag(iop, phid, protocol, tagString, lock);
		mos_mutex_unlock(&phid->tagLock);
		if (ret != EPHIDGET_OK)
			return (ret);

		if ((channel = getChannel(phid, 0)) != NULL) {
			ret = RFIDSupport_waitForTag(iop, channel, tagString, 500, &phid->tagLock);
			PhidgetRelease(&channel);
			return ret;
		}
	default:
		return (EPHIDGET_UNSUPPORTED);
	}
}

PhidgetReturnCode
PhidgetRFIDDevice_create(PhidgetRFIDDeviceHandle *phidp) {
	DEVICECREATE_BODY(RFIDDevice, PHIDCLASS_RFID);
	mos_mutex_init(&phid->tagLock);
	mos_cond_init(&phid->tagCond);
	phid->tagTimerThread = MOS_TASK_NONE;
	phid->phid._closing = PhidgetRFIDDevice_close;
	return (EPHIDGET_OK);
}
