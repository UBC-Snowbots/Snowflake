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
#include "device/bridgedevice.h"

// === Internal Functions === //
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetBridgeDeviceHandle phid);
static void PhidgetBridgeDevice_updateBridgeIntervals(PhidgetChannelHandle ch, PhidgetBridgeDeviceHandle phid, BridgePacket *bp);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetBridgeDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetBridgeDeviceHandle phid = (PhidgetBridgeDeviceHandle)device;
	int i;
#if PHIDUID_1046_1_SUPPORTED
	uint8_t buffer[MAX_IN_PACKET_SIZE];
	PhidgetReturnCode ret;
	double interval;
	size_t len;
#endif /* PHIDUID_1046_1_SUPPORTED */

	assert(phid);

	//Setup max/min values
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1046_GAINBUG:
	case PHIDUID_1046:
		phid->dataRateMin = 1000;
		phid->dataRateMax = 8;
		for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {
			phid->bridgeMax[i] = 1.000;
			phid->bridgeMin[i] = -1.000;
			phid->enabled[i] = PUNK_BOOL;
		}
		break;
#if PHIDUID_1046_1_SUPPORTED
	case PHIDUID_1046_1:
		phid->dataRateMin = 1000;
		phid->dataRateMax = 1;
		for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {
			phid->bridgeMax[i] = 1.000;
			phid->bridgeMin[i] = -1.000;
		}
		len = (4 * phid->devChannelCnts.numBridgeInputs) + 1;
		ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_READ, 0, 0, buffer, &len, 100);
		if (ret != EPHIDGET_OK)
			return ret;
		for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {
			phid->enabled[i] = ((buffer[0] & (1 << i)) != 0);
			interval = unpackfloat(&buffer[(4 * i) + 1]);
			if (interval <= 60000)
				phid->dataInterval[i] = interval;
			else
				phid->dataInterval[i] = PUNK_DBL;
		}
		break;
#endif /* PHIDUID_1046_1_SUPPORTED */
	default:
		MOS_PANIC("Unexpected device");
	}

	for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {
		phid->voltageRatio[i] = PUNK_DBL;
		phid->enabledEcho[i] = PUNK_BOOL;
		phid->gainEcho[i] = PUNK_ENUM;
		phid->outOfRange[i] = PFALSE;
		phid->chEnabledBugNotValid[i] = PFALSE;
		phid->bridgeLastTrigger[i] = PUNK_DBL;
		phid->voltageRatioChangeTrigger[i] = 0.001;
		phid->dataInterval[i] = PUNK_UINT32;
		phid->gain[i] = PUNK_ENUM;
	}
	phid->ch0EnableOverride = 0;
	phid->dataRate = PUNK_UINT32;

	//issue one read
	waitForReads((PhidgetDeviceHandle)phid, 1, 100);

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetBridgeDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetBridgeDeviceHandle phid = (PhidgetBridgeDeviceHandle)device;
	double bridgeval[BRIDGE_MAXINPUTS];
	PhidgetChannelHandle channel;
	int roundingFactor;
	int bridgeRawVal;
	double bridgeMax;
	double bridgeMin;
	uint8_t err;
	double gain;
	int i;
#if PHIDUID_1046_1_SUPPORTED
	PhidgetReturnCode res;
	int chIndex;
#endif /* PHIDUID_1046_1_SUPPORTED */

	assert(phid);
	assert(buffer);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1046_GAINBUG:
	case PHIDUID_1046:
		for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {

			//gain
			switch (buffer[(i / 2) + 2] >> ((i % 2) ? 0 : 4) & 0x0F) {
			default:
			case 0x00:
				phid->gainEcho[i] = BRIDGE_GAIN_1;
				gain = 1;
				roundingFactor = 9;
				break;
			case 0x03:
				phid->gainEcho[i] = BRIDGE_GAIN_8;
				gain = 8;
				roundingFactor = 10;
				break;
			case 0x04:
				phid->gainEcho[i] = BRIDGE_GAIN_16;
				gain = 16;
				roundingFactor = 11;
				break;
			case 0x05:
				phid->gainEcho[i] = BRIDGE_GAIN_32;
				gain = 32;
				roundingFactor = 11;
				break;
			case 0x06:
				phid->gainEcho[i] = BRIDGE_GAIN_64;
				gain = 64;
				roundingFactor = 11;
				break;
			case 0x07:
				phid->gainEcho[i] = BRIDGE_GAIN_128;
				gain = 128;
				roundingFactor = 12;
				break;
			}

			if (gain == 1) {
				bridgeMax = 1.000;
				bridgeMin = -1.000;
			} else {
				//with PGA, allow +-99.5%, because if doesn't clamp properly
				bridgeMax = 0.995 / gain;
				bridgeMin = -0.995 / gain;
			}

			phid->enabledEcho[i] = (buffer[0] & (0x01 << i)) ? PTRUE : PFALSE;
			//enabled and have data
			if (phid->enabled[i] == PTRUE && phid->enabledEcho[i] && (buffer[0] & (0x10 << i))) {
				//Don't count data this time - wait for next event
				if (phid->chEnabledBugNotValid[i] == PTRUE) {
					bridgeval[i] = PUNK_DBL;
					phid->chEnabledBugNotValid[i] = PFALSE;
				} else {
					bridgeRawVal = (buffer[i * 3 + 6] << 16) + (buffer[i * 3 + 7] << 8) + buffer[i * 3 + 8];
					bridgeval[i] = round_double(((bridgeRawVal - 0x800000) / (double)0x800000) / (double)gain, roundingFactor);

					err = (buffer[1] & (0x01 << i)) ? PTRUE : PFALSE;

					if (err || bridgeval[i] > bridgeMax || bridgeval[i] < bridgeMin) {
						if ((channel = getChannel(phid, i)) != NULL) {
							if (bridgeval[i] > bridgeMax)
								SEND_ERROR_EVENT(channel, EEPHIDGET_OUTOFRANGE, "Overrange condition detected on input, try lowering the gain.");
							else if (bridgeval[i] < bridgeMin)
								SEND_ERROR_EVENT(channel, EEPHIDGET_OUTOFRANGE, "Underrange condition detected on input, try lowering the gain.");
							PhidgetRelease(&channel);
						}

						bridgeval[i] = PUNK_DBL;
					}
				}
			} else if (!phid->enabledEcho[i] || phid->enabled[i] != PTRUE) {
				bridgeval[i] = PUNK_DBL;
			} else
				bridgeval[i] = PUNI_DBL;

			//decrement until 0 - this is related to a firmware bug
			if (phid->ch0EnableOverride) {
				phid->ch0EnableOverride--;
				if (phid->ch0EnableOverride == 0)
					_sendpacket(NULL, phid);
			}
		}
		break;
#if PHIDUID_1046_1_SUPPORTED
	case PHIDUID_1046_1:
		chIndex = buffer[1];
		switch (buffer[0]) {
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_VOLTAGERATIOCHANGE:
			res = EPHIDGET_OK;
			phid->voltageRatio[chIndex] = unpackfloat(&buffer[2]);
			if ((channel = getChannel(phid, chIndex)) != NULL) {
				//Send out events
				if (length > 6) {
					switch ((int8_t)buffer[6]) {
					case 1:
						res = bridgeSendToChannel(channel, BP_ERROREVENT, "%d%s", EEPHIDGET_OUTOFRANGEHIGH, "The sensor reading is too high for the selected gain mode.");
						break;
					case -1:
						res = bridgeSendToChannel(channel, BP_ERROREVENT, "%d%s", EEPHIDGET_OUTOFRANGELOW, "The sensor reading is too low for the selected gain mode.");
						break;
					default:
						res = bridgeSendToChannel(channel, BP_ERROREVENT, "%d%s", EEPHIDGET_OUTOFRANGE, "Value is unknown");
						break;
					}
				}

				if (res != EPHIDGET_OK)
					return res;

				if (isnan(phid->voltageRatio[chIndex]))
					phid->voltageRatio[chIndex] = PUNK_DBL;

				res = bridgeSendToChannel(channel, BP_VOLTAGERATIOCHANGE, "%g", phid->voltageRatio[chIndex]);
				PhidgetRelease(&channel);
				return res;
			}
			return EPHIDGET_OK;
		default:
			MOS_PANIC("Unexpected packet type");
		}
#endif /* PHIDUID_1046_1_SUPPORTED */
	default:
		MOS_PANIC("Unexpected device");
	}

	for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {
		if (bridgeval[i] != PUNI_DBL) {
			phid->voltageRatio[i] = bridgeval[i];
		}
	}

	for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {
		if ((channel = getChannel(phid, i)) != NULL) {
			//Send out events
			if (bridgeval[i] != PUNI_DBL && phid->voltageRatio[i] != PUNK_DBL
				&& (fabs(phid->voltageRatio[i] - phid->bridgeLastTrigger[i]) >= phid->voltageRatioChangeTrigger[i]
					|| phid->bridgeLastTrigger[i] == PUNK_DBL)) {
				bridgeSendToChannel(channel, BP_VOLTAGERATIOCHANGE, "%g", phid->voltageRatio[i]);
				phid->bridgeLastTrigger[i] = phid->voltageRatio[i];
			}
			PhidgetRelease(&channel);
		}
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetBridgeDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetBridgeDeviceHandle phid = (PhidgetBridgeDeviceHandle)ch->parent;
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	PhidgetChannelHandle channel;
	PhidgetReturnCode ret;
	int i;
#if PHIDUID_1046_1_SUPPORTED
	size_t len;
#endif /* PHIDUID_1046_1_SUPPORTED */

	assert(phid->phid.deviceInfo.class == PHIDCLASS_BRIDGE);
	assert(ch->class == PHIDCHCLASS_VOLTAGERATIOINPUT);
	assert(ch->index < phid->devChannelCnts.numBridgeInputs);
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1046_GAINBUG:
	case PHIDUID_1046:
		switch (bp->vpkt) {
		case BP_SETCHANGETRIGGER:
			phid->voltageRatioChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);

		case BP_SETDATAINTERVAL:
			// XXX - figure out what firmware is doing and reflect the true rate back to the channel layer, and here.
			phid->dataInterval[ch->index] = getBridgePacketUInt32(bp, 0);
			phid->dataRate = phid->dataInterval[ch->index];
			ret = _sendpacket(bp->iop, phid);
			//Need to update the other channels because the data interval is board-wide
			for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {
				if ((channel = getChannel(phid, i)) != NULL && channel != ch) {
					bridgeSendToChannel(channel, BP_DATAINTERVALCHANGE, "%u", getBridgePacketUInt32(bp, 0));
					PhidgetRelease(&channel);
				}
			}
			return (ret);

		case BP_SETBRIDGEGAIN:
			switch ((PhidgetVoltageRatioInput_BridgeGain)getBridgePacketInt32(bp, 0)) {
			case BRIDGE_GAIN_1:
			case BRIDGE_GAIN_8:
			case BRIDGE_GAIN_16:
			case BRIDGE_GAIN_32:
			case BRIDGE_GAIN_64:
			case BRIDGE_GAIN_128:
				phid->gain[ch->index] = (PhidgetVoltageRatioInput_BridgeGain)getBridgePacketInt32(bp, 0);
				return (_sendpacket(bp->iop, phid));
			default:
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Specified gain is not supported by this device."));
			}

		case BP_SETENABLED:
			phid->enabled[ch->index] = (uint8_t)getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid));

		case BP_OPENRESET:
		case BP_CLOSERESET:
			phid->gain[ch->index] = BRIDGE_GAIN_1;
			phid->enabled[ch->index] = PFALSE;
			//NOTE: not resetting data interval because it affects multiple channels.
			return (_sendpacket(bp->iop, phid));

		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}
		break;
#if PHIDUID_1046_1_SUPPORTED
	case PHIDUID_1046_1:
		switch (bp->vpkt) {
		case BP_SETENABLED:
			buffer[0] = (uint8_t)getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			len = 1;
			ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_BRIDGEENABLED, ch->uniqueIndex, buffer, &len, 100);
			if (ret != EPHIDGET_OK)
				return ret;

			phid->enabled[ch->index] = (buffer[0] != 0);
			PhidgetBridgeDevice_updateBridgeIntervals(ch, phid, bp);
			return EPHIDGET_OK;
		case BP_SETDATAINTERVAL:
			if (bp->entrycnt > 1)
				packfloat(buffer, getBridgePacketDouble(bp, 1));
			else
				packfloat(buffer, (float)getBridgePacketUInt32(bp, 0));
			len = 4;
			ret = (PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, ch->uniqueIndex, buffer, &len, 100));
			if (ret != EPHIDGET_OK)
				return ret;

			len = 4;
			ret = (PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_READ, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, ch->uniqueIndex, buffer, &len, 100));
			if (ret != EPHIDGET_OK)
				return ret;

			phid->dataInterval[ch->index] = unpackfloat(buffer);

			PhidgetBridgeDevice_updateBridgeIntervals(ch, phid, bp);
			return EPHIDGET_OK;
		case BP_SETCHANGETRIGGER:
			pack32(buffer, (uint32_t)(getBridgePacketDouble(bp, 0) * 0x80000000u));
			len = 4;
			return (PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SETVOLTAGERATIOCHANGETRIGGER, ch->uniqueIndex, buffer, &len, 100));
		case BP_SETBRIDGEGAIN:
			buffer[0] = getBridgePacketInt32(bp, 0);
			len = 1;
			return (PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_BRIDGEGAIN, ch->uniqueIndex, buffer, &len, 100));
		case BP_OPENRESET:
		case BP_CLOSERESET:
			len = 0;
			phid->enabled[ch->index] = 0;
			ret = PhidgetDevice_transferpacket(bp->iop, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_CHANNEL_WRITE, VINT_PACKET_TYPE_PHIDGET_RESET, ch->uniqueIndex, buffer, &len, 100);
			if (ret != EPHIDGET_OK)
				return ret;
			PhidgetBridgeDevice_updateBridgeIntervals(ch, phid, bp);
			return (EPHIDGET_OK);
		case BP_ENABLE:
			PhidgetBridgeDevice_updateBridgeIntervals(ch, phid, bp);
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}
		break;
#endif /* PHIDUID_1046_1_SUPPORTED */
	default:
		MOS_PANIC("Unexpected device");
	}
}

static void PhidgetBridgeDevice_updateBridgeIntervals(PhidgetChannelHandle ch, PhidgetBridgeDeviceHandle phid, BridgePacket *bp)
{
	uint8_t buffer[MAX_IN_PACKET_SIZE];
	PhidgetChannelHandle channel;
	PhidgetReturnCode ret;
	int integerInterval;
	double interval;
	size_t len;
	int i;

	len = (4 * phid->devChannelCnts.numBridgeInputs) + 1;
	ret = PhidgetDevice_transferpacket(NULL, (PhidgetDeviceHandle)phid, PHIDGETUSB_REQ_DEVICE_READ, 0, 0, buffer, &len, 100);
	if (ret != EPHIDGET_OK)
		return;
	for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {
		phid->enabled[i] = ((buffer[0] & (1 << i)) != 0);
		interval = unpackfloat(&buffer[(4 * i) + 1]);
		if (interval <= 60000)
			phid->dataInterval[i] = interval;
		else
			phid->dataInterval[i] = PUNK_DBL;
	}

	for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {
		interval = phid->dataInterval[i];

		if (interval == PUNK_DBL)
			continue;

		integerInterval = round((interval > 0) ? interval : 1);

		if (i == ch->index) {
			if (bp->vpkt == BP_CLOSERESET)
				continue;
			if (bp->vpkt != BP_SETDATAINTERVAL) {
				addBridgePacketDouble(bp, interval, "DataIntervalResp");
			} else {
				setBridgePacketUInt32(bp, integerInterval, 0);
				if (bp->entrycnt > 1)
					setBridgePacketDouble(bp, interval, 1);
				else
					addBridgePacketDouble(bp, interval, "");
			}
		}

		if ((channel = getChannel(phid, i)) != NULL) {
			ret = bridgeSendToChannel(channel, BP_DATAINTERVALCHANGE, "%u%g", integerInterval, interval);
			PhidgetRelease(&channel);
		}
	}
}

//makePacket - constructs a packet using current device state
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetBridgeDeviceHandle phid) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	int dataRateRaw;
	int gainRaw;
	int i;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1046_GAINBUG:
	case PHIDUID_1046:
		//ms->raw
		if (phid->dataRate == PUNK_UINT32)
			dataRateRaw = 1;
		else
			dataRateRaw = phid->dataRate / 8;

		buffer[0] = 0;
		buffer[1] = 0;
		buffer[2] = 0;
		for (i = 0; i < phid->devChannelCnts.numBridgeInputs; i++) {
			//Deal with some firmware bugs
			if (phid->phid.deviceInfo.UDD->uid == PHIDUID_1046_GAINBUG) {
				if (phid->gain[i] != phid->gainEcho[i] && (uint32_t)phid->gain[i] != PUNK_ENUM) {
					//anytime we change gain on any channel, we have to enable channel 0, and leave it enabled for at least 250ms
					phid->ch0EnableOverride = 256 / 8; //USB interval is 8ms, this is decremented by one each DATA packet until it gets back to 0.
				}

				if (phid->enabledEcho[i] == PFALSE && phid->enabled[i] == PTRUE) {
					//1st data after a channel is enabled can be bad, so we just ignore this.
					phid->chEnabledBugNotValid[i] = PTRUE;
				}
			}

			if (phid->enabled[i] == PTRUE)
				buffer[0] |= (0x01 << i);

			if ((uint32_t)phid->gain[i] == PUNK_ENUM)
				phid->gain[i] = BRIDGE_GAIN_1;

			switch (phid->gain[i]) {
			case BRIDGE_GAIN_1:
				gainRaw = 0x00;
				break;
			case BRIDGE_GAIN_8:
				gainRaw = 0x03;
				break;
			case BRIDGE_GAIN_16:
				gainRaw = 0x04;
				break;
			case BRIDGE_GAIN_32:
				gainRaw = 0x05;
				break;
			case BRIDGE_GAIN_64:
				gainRaw = 0x06;
				break;
			case BRIDGE_GAIN_128:
				gainRaw = 0x07;
				break;
			default:
				MOS_PANIC("Unexpected Gain");
			}
			buffer[(i / 2) + 1] |= gainRaw << ((i % 2) ? 0 : 4);
		}

		//Deal with some firmware bugs
		if (phid->phid.deviceInfo.UDD->uid == PHIDUID_1046_GAINBUG && phid->ch0EnableOverride) {
			//override and enable ch0
			buffer[0] |= 0x01;
		}

		buffer[3] = dataRateRaw & 0xFF;
		buffer[4] = (dataRateRaw >> 8) & 0x03;
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
}

static void CCONV
PhidgetBridgeDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetBridgeDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetBridgeDevice_create(PhidgetBridgeDeviceHandle *phidp) {
	DEVICECREATE_BODY(BridgeDevice, PHIDCLASS_BRIDGE);
	return (EPHIDGET_OK);
}
