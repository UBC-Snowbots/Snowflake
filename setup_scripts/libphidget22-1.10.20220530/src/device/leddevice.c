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
#include "device/leddevice.h"

// === Internal Functions === //
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetLEDDeviceHandle phid, int Index);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetLEDDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetLEDDeviceHandle phid = (PhidgetLEDDeviceHandle)device;
	int i = 0;

	assert(phid);

	//set data arrays to unknown
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1030:
		for (i = 0; i < phid->devChannelCnts.numLEDs; i++) {
			phid->changedLED_Power[i] = PFALSE;
			phid->nextLED_Power[i] = PUNK_DBL;
			phid->LED_Power[i] = PUNK_DBL;
		}
		break;

	case PHIDUID_1031:
	case PHIDUID_1032:
		for (i = 0; i < phid->devChannelCnts.numLEDs; i++) {
			phid->changedLED_Power[i] = PFALSE;
			phid->nextLED_Power[i] = PUNK_DBL;
			phid->lastLED_Power[i] = PUNK_DBL;
			phid->LED_Power[i] = PUNK_DBL;
			phid->currentLimit[i] = PUNK_DBL;
			phid->voltage[i] = PUNK_ENUM;
		}
		phid->voltageBoard = PUNK_ENUM;
		phid->currentLimitBoard = PUNK_DBL;

		for (i = 0; i < 4; i++) {
			phid->TSDCount[i] = 0;
			phid->TSDClearCount[i] = 0;
			phid->TWarnCount[i] = 0;
			phid->TWarnClearCount[i] = 0;
		}
		phid->lastOutputPacket = 0;
		phid->lastControlPacketValid = PFALSE;
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetLEDDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetLEDDeviceHandle phid = (PhidgetLEDDeviceHandle)device;
	PhidgetChannelHandle channel;
	int ledOpenDetectCnt;
	int outputEnableEcho;
	int powerGoodEcho;
	int faultEcho;
	int i;
	int j;

	assert(phid);
	assert(buffer);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1031:
		switch (buffer[0] & 0x80) {
		case LED64_IN_LOW_PACKET:
			//PowerGood
			if (buffer[0] & LED64_PGOOD_FLAG)
				powerGoodEcho = PTRUE;
			else
				powerGoodEcho = PFALSE;

			//all outputs enabled (power on/off)
			if (buffer[0] & LED64_OE_FLAG)
				outputEnableEcho = PTRUE;
			else
				outputEnableEcho = PFALSE;

			//fault
			if (buffer[0] & LED64_FAULT_FLAG)
				faultEcho = PTRUE;
			else
				faultEcho = PFALSE;

			ledOpenDetectCnt = 0;
			for (i = 0; i < phid->devChannelCnts.numLEDs; i++)
				if (buffer[(i / 8) + 9] & (1 << (i % 8)))
					ledOpenDetectCnt++;

			//We can guess that the fault is a TSD if there is no LOD
			if (faultEcho) {
				phid->TSDCount[0]++;
				phid->TSDClearCount[0] = 30; //500ms of no faults before we clear it

				if (ledOpenDetectCnt)
					phid->TSDCount[0] = 0;

				//send out some error events on faults
				//TODO: we could also send LED Open Detect?

				//we have counted three fault flags with no LODs - TSD
				//less then 3 counts, and it could be a false positive
				//if outputs are not enabled then the fault should be guaranteed as a TSD
				if (phid->TSDCount[0] >= 3 || (phid->TSDCount[0] < 3 && outputEnableEcho == PFALSE)) {
					phid->TSDCount[0] = 3;
					for (i = 0; i < phid->devChannelCnts.numLEDs; i++) {
						if ((channel = getChannel(phid, i)) != NULL) {
							SEND_ERROR_EVENT(channel, EEPHIDGET_OVERTEMP, "Thermal Shutdown detected.");
							PhidgetRelease(&channel);
						}
					}
				}
			} else {
				if (phid->TSDClearCount[0] > 0)
					phid->TSDClearCount[0]--;
				else
					phid->TSDCount[0] = 0;
			}

			if (!powerGoodEcho) {
				for (i = 0; i < phid->devChannelCnts.numLEDs; i++) {
					if ((channel = getChannel(phid, i)) != NULL) {
						SEND_ERROR_EVENT(channel, EEPHIDGET_BADPOWER, "Bad power supply detected.");
						PhidgetRelease(&channel);
					}
				}
			}
			break;
		case LED64_IN_HIGH_PACKET:
			break;
		}
		break;

	case PHIDUID_1032:
		switch (buffer[0] & 0x60) {
		case LED64_M3_IN_MISC_PACKET:
			//PowerGood
			if (buffer[0] & LED64_PGOOD_FLAG)
				powerGoodEcho = PTRUE;
			else
				powerGoodEcho = PFALSE;

			if (!powerGoodEcho) {
				for (i = 0; i < phid->devChannelCnts.numLEDs; i++) {
					if ((channel = getChannel(phid, i)) != NULL) {
						SEND_ERROR_EVENT(channel, EEPHIDGET_BADPOWER, "Bad power supply detected.");
						PhidgetRelease(&channel);
					}
				}
			}

			//Temperature Warnings
			for (i = 0; i < 4; i++) {
				if (buffer[1] & (0x01 << i)) {
					phid->TWarnCount[i]++;
					phid->TWarnClearCount[i] = 20; //480ms of no faults before we clear it

					if (phid->TWarnCount[i] >= 10) { //240 ms of fault before we call it
						for (j = 0; j < phid->devChannelCnts.numLEDs; j++) {
							if ((channel = getChannel(phid, j)) != NULL) {
								SEND_ERROR_EVENT(channel, EEPHIDGET_OVERTEMP, "Temperature Warning detected on chip %d.", i);
								PhidgetRelease(&channel);
							}
						}
						phid->TWarnCount[i]--; //so we don't overflow the char
					}
				} else {
					if (phid->TWarnClearCount[i] > 0)
						phid->TWarnClearCount[i]--;
					else
						phid->TWarnCount[i] = 0;
				}
			}

			//Temperature Errors
			for (i = 0; i < 4; i++) {
				if (buffer[1] & (0x10 << i)) {
					phid->TSDCount[i]++;
					phid->TSDClearCount[i] = 20; //480ms of no faults before we clear it

					if (phid->TSDCount[i] >= 10) //240 ms of fault before we call it
					{
						for (j = 0; j < phid->devChannelCnts.numLEDs; j++) {
							if ((channel = getChannel(phid, j)) != NULL) {
								SEND_ERROR_EVENT(channel, EEPHIDGET_OVERTEMP, "Temperature Error detected on chip %d.", i);
								PhidgetRelease(&channel);
							}
						}
						phid->TSDCount[i]--; //so we don't overflow the char
					}
				} else {
					if (phid->TSDClearCount[i] > 0)
						phid->TSDClearCount[i]--;
					else
						phid->TSDCount[i] = 0;
				}
			}

			break;
		case LED64_M3_IN_LOW_PACKET:
		case LED64_M3_IN_HIGH_PACKET:
			break;
		}
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	return (EPHIDGET_OK);
}

static int
configured(PhidgetLEDDeviceHandle phid, int index) {
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1030:
		return (PTRUE);
	case PHIDUID_1032:
	case PHIDUID_1031:
		if (phid->voltage[index] == (PhidgetDigitalOutput_LEDForwardVoltage)PUNK_ENUM || phid->currentLimit[index] == PUNK_DBL)
			return (PFALSE);
		return (PTRUE);
	default:
		MOS_PANIC("Unexpected device");
	}
}

static PhidgetReturnCode CCONV
PhidgetLEDDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetLEDDeviceHandle phid = (PhidgetLEDDeviceHandle)ch->parent;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_LED);
	assert(ch->class == PHIDCHCLASS_DIGITALOUTPUT);
	assert(ch->index < phid->devChannelCnts.numLEDs);

	switch (bp->vpkt) {
	case BP_SETSTATE:
		if (!configured(phid, ch->index))
			return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Voltage and Current Limit must be configured before outputs can be enabled."));
		if ((double)getBridgePacketInt32(bp, 0) != phid->LED_Power[ch->index]) {
			phid->changedLED_Power[ch->index] = PTRUE;
			phid->nextLED_Power[ch->index] = (double)getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid, LED64_NORMAL_PACKET));
		}
		return (EPHIDGET_OK);

	case BP_SETDUTYCYCLE:
		if (!configured(phid, ch->index))
			return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Voltage and Current Limit must be configured before outputs can be enabled."));
		if (getBridgePacketDouble(bp, 0) != phid->LED_Power[ch->index]) {
			phid->changedLED_Power[ch->index] = PTRUE;
			phid->nextLED_Power[ch->index] = getBridgePacketDouble(bp, 0);
			return (_sendpacket(bp->iop, phid, LED64_NORMAL_PACKET));
		}
		return (EPHIDGET_OK);

	case BP_SETLEDFORWARDVOLTAGE:
		//TODO: Verify enum value
		phid->voltage[ch->index] = (PhidgetDigitalOutput_LEDForwardVoltage)getBridgePacketInt32(bp, 0);
		// Only send a packet if we changed the whole-board voltage
		if (phid->voltageBoard != phid->voltage[ch->index]) {
			phid->voltageBoard = phid->voltage[ch->index];
			return (_sendpacket(bp->iop, phid, LED64_CONTROL_PACKET));
		}
		return EPHIDGET_OK;

	case BP_SETLEDCURRENTLIMIT:
		phid->currentLimit[ch->index] = getBridgePacketDouble(bp, 0);
		switch (phid->phid.deviceInfo.UDD->uid) {
		case PHIDUID_1032:
			return (_sendpacket(bp->iop, phid, LED64_CONTROL_PACKET));
		case PHIDUID_1031:
			if (phid->currentLimitBoard != phid->currentLimit[ch->index]) {
				phid->currentLimitBoard = getBridgePacketDouble(bp, 0);
				return (_sendpacket(bp->iop, phid, LED64_CONTROL_PACKET));
			}
			return EPHIDGET_OK;
		default:
			return (EPHIDGET_UNSUPPORTED);
		}

	case BP_OPENRESET:
	case BP_CLOSERESET:
		if (phid->LED_Power[ch->index] != 0) {
			phid->changedLED_Power[ch->index] = PTRUE;
			phid->nextLED_Power[ch->index] = 0;
			return (_sendpacket(bp->iop, phid, LED64_NORMAL_PACKET));
		}
		switch (phid->phid.deviceInfo.UDD->uid) {
		case PHIDUID_1030:
			return (EPHIDGET_OK);
		case PHIDUID_1032:
		case PHIDUID_1031:
			phid->currentLimit[ch->index] = PUNK_DBL;
			phid->voltage[ch->index] = PUNK_ENUM;
			return (_sendpacket(bp->iop, phid, LED64_CONTROL_PACKET));
		}
	case BP_ENABLE:
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

//getPacket - used by write thread to get the next packet to send to device
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetLEDDeviceHandle phid, int Index) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	int numLeds = 0;
	int i = 0;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1030:
		//construct the packet, with up to 4 LED sets
		for (i = 0; i < phid->devChannelCnts.numLEDs; i++) {
			if (phid->changedLED_Power[i] && numLeds < 4) {
				phid->LED_Power[i] = phid->nextLED_Power[i];
				phid->changedLED_Power[i] = PFALSE;
				phid->nextLED_Power[i] = PUNK_DBL;
				buffer[numLeds * 2] = i;
				//0-1 -> 0-63
				buffer[numLeds * 2 + 1] = (uint8_t)round((phid->LED_Power[i]) * 63.0);
				numLeds++;
			}
		}

		assert(numLeds >= 1);

		//fill up any remaining bufferfer space with valid data - sending 0's will mess things up
		for (; numLeds < 4; numLeds++) {
			buffer[numLeds * 2] = buffer[(numLeds - 1) * 2];
			buffer[numLeds * 2 + 1] = buffer[(numLeds - 1) * 2 + 1];
		}
		break;
	case PHIDUID_1031:
		//control packet
		if (Index == LED64_CONTROL_PACKET) {

			buffer[0] = LED64_CONTROL_PACKET;

			buffer[1] = 0;
			if (phid->currentLimitBoard >= 0.080)
				buffer[1] |= (LED64_CURSELA_FLAG | LED64_CURSELB_FLAG);
			else if (phid->currentLimitBoard >= 0.060)
				buffer[1] |= LED64_CURSELB_FLAG;
			else if (phid->currentLimitBoard >= 0.040)
				buffer[1] |= LED64_CURSELA_FLAG;

			switch (phid->voltageBoard) {
			case PUNK_ENUM:
			case LED_FORWARD_VOLTAGE_1_7V:
				break;
			case LED_FORWARD_VOLTAGE_2_75V:
				buffer[1] |= LED64_PWRSELA_FLAG;
				break;
			case LED_FORWARD_VOLTAGE_3_9V:
				buffer[1] |= LED64_PWRSELB_FLAG;
				break;
			case LED_FORWARD_VOLTAGE_5_0V:
				buffer[1] |= (LED64_PWRSELA_FLAG | LED64_PWRSELB_FLAG);
				break;
			default:
				MOS_PANIC("Unexpected voltage");
			}

			// Don't need to resend an identical packet
			if (phid->lastControlPacketValid) {
				if (!memcmp(phid->lastControlPacket, buffer, MAX_OUT_PACKET_SIZE))
					return (EPHIDGET_OK);
			}

			memcpy(phid->lastControlPacket, buffer, MAX_OUT_PACKET_SIZE);
			phid->lastControlPacketValid = PTRUE;
		}
		//LED packet
		else {
			int bright_packet = PFALSE;
			int output_upper = PFALSE;
			int output_lower = PFALSE;
			//decide if we need to use a normal brightness packet, or if we can use a high efficiency output packet
			for (i = 0; i < phid->devChannelCnts.numLEDs; i++) {
				if (phid->changedLED_Power[i]) {
					if ((phid->nextLED_Power[i] != phid->lastLED_Power[i]) && phid->nextLED_Power[i] != 0)
						bright_packet = PTRUE;
					else {
						if (i < 32)
							output_lower = PTRUE;
						else
							output_upper = PTRUE;
					}
				}
			}

			//only sends brightness changes - not changes between 0 and a brightness
			if (bright_packet) {
				//construct the packet, with up to 4 LED sets
				for (i = 0; i < phid->devChannelCnts.numLEDs; i++) {
					if (phid->changedLED_Power[i] && numLeds < 4 && phid->nextLED_Power[i] != 0) {
						phid->LED_Power[i] = phid->nextLED_Power[i];
						phid->lastLED_Power[i] = phid->nextLED_Power[i];
						phid->changedLED_Power[i] = PFALSE;
						phid->nextLED_Power[i] = PUNK_DBL;
						buffer[numLeds * 2] = i;
						//0-1 -> 0-127
						buffer[numLeds * 2 + 1] = (uint8_t)round((phid->LED_Power[i]) * 127.0);
						if (buffer[numLeds * 2 + 1])
							buffer[numLeds * 2 + 1] |= 0x80; //this turns the LED on when set brightness > 0;
						numLeds++;
					}
				}

				assert(numLeds >= 1);

				//fill up any remaining bufferfer space with valid data - sending 0's will mess things up
				//this just replicates data - doesn't send anything
				for (; numLeds < 4; numLeds++) {
					buffer[numLeds * 2] = buffer[(numLeds - 1) * 2];
					buffer[numLeds * 2 + 1] = buffer[(numLeds - 1) * 2 + 1];
				}
			} else {
				//send lower packet
				if ((phid->lastOutputPacket == 0 && output_lower) || (phid->lastOutputPacket != 0 && !output_upper)) {
					buffer[0] = LED64_OUTLOW_PACKET;
					for (i = 0; i < 32; i++) {
						if (phid->changedLED_Power[i]) {
							phid->LED_Power[i] = phid->nextLED_Power[i];
							phid->changedLED_Power[i] = PFALSE;
							phid->nextLED_Power[i] = PUNK_DBL;
						}
						if (phid->LED_Power[i] > 0)
							buffer[i / 8 + 1] |= (1 << (i % 8));
					}
					phid->lastOutputPacket = 1;
				}
				//send upper packet
				else {
					buffer[0] = LED64_OUTHIGH_PACKET;
					for (i = 32; i < 64; i++) {
						if (phid->changedLED_Power[i]) {
							phid->LED_Power[i] = phid->nextLED_Power[i];
							phid->changedLED_Power[i] = PFALSE;
							phid->nextLED_Power[i] = PUNK_DBL;
						}
						if (phid->LED_Power[i] > 0)
							buffer[i / 8 - 3] |= (1 << (i % 8));
					}
					phid->lastOutputPacket = 0;
				}
			}
		}
		break;
	case PHIDUID_1032:
		//control packet
		if (Index == LED64_CONTROL_PACKET) {
			buffer[0] = LED64_M3_CONTROL_PACKET;

			switch (phid->voltageBoard) {
			case LED_FORWARD_VOLTAGE_1_7V:
				break;
			case PUNK_ENUM:
			case LED_FORWARD_VOLTAGE_2_75V:
				buffer[0] |= LED64_PWRSELA_FLAG;
				break;
			case LED_FORWARD_VOLTAGE_3_9V:
				buffer[0] |= LED64_PWRSELB_FLAG;
				break;
			case LED_FORWARD_VOLTAGE_5_0V:
				buffer[0] |= (LED64_PWRSELA_FLAG | LED64_PWRSELB_FLAG);
				break;
			default:
				MOS_PANIC("Unexpected voltage");
			}

			for (i = 0; i < phid->devChannelCnts.numLEDs; i++) {
				int value;
				int bufferIndex = (i * 6) / 8 + 1;

				//Default is 20 mA
				if (phid->currentLimit[i] == PUNK_DBL)
					phid->currentLimit[i] = 0.020;
				value = round((phid->currentLimit[i] / LED64_CURRENTLIMIT) * 63.0);

				switch (i % 4) {
				case 0:
					buffer[bufferIndex] |= (value & 0x3F);
					break;
				case 1:
					buffer[bufferIndex] |= ((value << 6) & 0xC0);
					buffer[bufferIndex + 1] |= ((value >> 2) & 0x0F);
					break;
				case 2:
					buffer[bufferIndex] |= ((value << 4) & 0xF0);
					buffer[bufferIndex + 1] |= ((value >> 4) & 0x03);
					break;
				case 3:
					buffer[bufferIndex] |= ((value << 2) & 0xFC);
					break;
				}
			}

			// Don't need to resend an identical packet
			if (phid->lastControlPacketValid) {
				if (!memcmp(phid->lastControlPacket, buffer, MAX_OUT_PACKET_SIZE))
					return (EPHIDGET_OK);
			}

			memcpy(phid->lastControlPacket, buffer, MAX_OUT_PACKET_SIZE);
			phid->lastControlPacketValid = PTRUE;
		}
		//LED packet
		else {
			int output_upper = PFALSE;
			int output_lower = PFALSE;
			int startIndex;

			for (i = 0; i < phid->devChannelCnts.numLEDs; i++) {
				if (phid->changedLED_Power[i]) {
					if (i < 32)
						output_lower = PTRUE;
					else
						output_upper = PTRUE;
				}
			}

			//send lower packet
			if ((phid->lastOutputPacket == 0 && output_lower) || (phid->lastOutputPacket != 0 && !output_upper)) {
				buffer[0] = LED64_M3_OUT_LOW_PACKET;
				startIndex = 0;
				phid->lastOutputPacket = 1;
			}
			//send upper packet
			else {
				buffer[0] = LED64_M3_OUT_HIGH_PACKET;
				startIndex = 32;
				phid->lastOutputPacket = 0;
			}

			for (i = startIndex; i < startIndex + 32; i++) {
				int value;
				int bufferIndex = ((i - startIndex) * 12) / 8 + 1;

				if (phid->changedLED_Power[i]) {
					phid->LED_Power[i] = phid->nextLED_Power[i];
					phid->changedLED_Power[i] = PFALSE;
					phid->nextLED_Power[i] = PUNK_DBL;
				}

				//Default is 0 %
				if (phid->LED_Power[i] == PUNK_DBL)
					phid->LED_Power[i] = 0;
				value = round((phid->LED_Power[i]) * 4095.0);

				if (i % 2 == 0) {
					buffer[bufferIndex] |= (value & 0xFF);
					buffer[bufferIndex + 1] |= ((value >> 8) & 0x0F);
				} else {
					buffer[bufferIndex] |= ((value << 4) & 0xF0);
					buffer[bufferIndex + 1] |= ((value >> 4) & 0xFF);
				}
			}
		}
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
}

static void CCONV
PhidgetLEDDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetLEDDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetLEDDevice_create(PhidgetLEDDeviceHandle *phidp) {
	DEVICECREATE_BODY(LEDDevice, PHIDCLASS_LED);
	return (EPHIDGET_OK);
}