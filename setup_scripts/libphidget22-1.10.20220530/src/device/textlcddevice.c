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
#include "device/textlcddevice.h"

/*
	Protocol Documentation

	buffer[7] = packet type and screen number
		Top 3 bits are screen number. [SSSCCCCC]
		0x01 - 0x07 = LCD command (data count)
		0x08 - 0x1F = Space for extra packet types:
		0x11 = Backlight, Brightness
			buffer[0] = 0x00 for off, 0x01 for on
			buffer[1] = 0x00 - 0xff (variable brightness where supported)
		0x12 = Contrast
			buffer[0] = 0x00 - 0xff
		0x13 = Init
			re-initializes a display - this can fix a display that's gone wonky, or bring up a display plugged in after power-up
		anything else is ignored

	HD44780-based Character-LCD
	Documentation -		http://home.iae.nl/users/pouweha/lcd/lcd0.shtml
						http://www.doc.ic.ac.uk/~ih/doc/lcd/index.html

	buffer[0-6] = LCD command / data
	So sending a packet with any arbitrary set of LCD commands in 0-6 and the command length in 7 is how it works.
	You can combine any number of commands / data in one packet - up to 7 bytes.
		LCD commands - special characters:
			0x00 = escape the next character (can escape 0x00, 0x01, 0x02)
			0x01 = following is commands (clears RS)
			0x02 = following is data (sets RS)

	Always leave in command mode (0x01) when you finish

	So, we can send any command / data, but we can not read back anything (busy flag, CGRAM, DDRAM)

	On our 2x20 display:
		Display Data (DDRAM): Row 0 address 0x00-0x13, Row 1 address 0x40-0x53
		Custom characters (CGRAM): 0x08-0x15 - don't use 0x00-0x07 because 0x00 will terminate displaystring early

*/


// === Internal Functions === //
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetTextLCDDeviceHandle phid, int Index);
static void escapeLcdString(char *string, size_t len, char *buffer, int *buf_ptr);
static PhidgetReturnCode _writeString(mosiop_t iop, PhidgetTextLCDDeviceHandle phid, int screen, int Row, int Column, const char *chars);
static PhidgetReturnCode writeScreen(mosiop_t iop, PhidgetTextLCDDeviceHandle phid, int Index);
static PhidgetReturnCode _setCustomCharacter(mosiop_t iop, PhidgetTextLCDDeviceHandle phid, int screen, const char *charStr, const uint8_t *bitmap);
static PhidgetReturnCode writeBuffer(mosiop_t iop, PhidgetTextLCDDeviceHandle phid, int packet_type, uint8_t *buffer, int buflen, int screen);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetTextLCDDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetTextLCDDeviceHandle phid = (PhidgetTextLCDDeviceHandle)device;
	PhidgetReturnCode ret = EPHIDGET_OK;
	uint8_t buffer[8] = { 0 };
	int i;
	int j;

	assert(phid);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1215:
	case PHIDUID_1219:
	case PHIDUID_1202_TEXTLCD:
	case PHIDUID_1202_TEXTLCD_BRIGHTNESS:
		for (i = 0; i < phid->devChannelCnts.numScreens; i++) {
			//set data arrays to unknown
			phid->contrast[i] = PUNK_DBL;
			phid->backlight[i] = PUNK_DBL;
			phid->screenSize[i] = SCREEN_SIZE_2x20;
			phid->height[i] = 2;
			phid->width[i] = 20;
			phid->cursorLocation[i] = 0;
			phid->cursorColumn[i] = 0;
			phid->cursorScreen[i] = i;
			phid->cursorLastScreen[i] = i;
			phid->lastScreenWritten = 0;
		}
		break;
	case PHIDUID_1204:
		for (i = 0; i < phid->devChannelCnts.numScreens; i++) {
			//set data arrays to unknown
			phid->contrast[i] = PUNK_DBL;
			phid->backlight[i] = PUNK_DBL;
			phid->screenSize[i] = PUNK_ENUM;
			phid->height[i] = 0;
			phid->width[i] = 0;
			phid->cursorLocation[i] = 0;
			phid->cursorColumn[i] = 0;
			phid->cursorScreen[i] = i;
			phid->cursorLastScreen[i] = i;
			phid->lastScreenWritten = PUNK_INT32;
		}
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	for (i = 0; i < phid->devChannelCnts.numInputs; i++)
		phid->inputState[i] = PUNK_BOOL;

	for (i = 0; i < phid->devChannelCnts.numOutputs; i++)
		phid->outputState[i] = PUNK_BOOL;

	phid->fullStateEcho = PFALSE;

	//Device specific stuff
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1204:
		phid->fullStateEcho = PTRUE;
		break;
	default:
		break;
	}

	// Initialize internal framebuffer with spaces
	for (i = 0; i < phid->devChannelCnts.numScreens; i++) {
		for (j = 0; j < TEXTLCD_MAXROWS; j++) {
			phid->displayFramebufferValid[i][j] = PFALSE;
			memset(phid->internalFramebuffer[i][j], ' ', TEXTLCD_MAXCOLS);
		}
	}

	//Make sure the TextLCDDevice is in command mode! Also turn off cursor and cursor blink explicitly
	for (i = 0; i < phid->devChannelCnts.numScreens; i++) {
		buffer[0] = TEXTLCD_COMMAND_MODE;
		buffer[1] = HD44780_DISPLAY_CNTRL | HD44780_DISPLAY_ON;
		buffer[7] = 2 | TEXTLCD_SCREEN(i);

		ret = writeBuffer(NULL, phid, 0, buffer, 2, i);
		if (ret != EPHIDGET_OK)
			return ret;

		phid->cursorBlink[i] = 0;
		phid->cursorOn[i] = 0;
	}

	//Issue a read if device supports state echo
	if (phid->fullStateEcho)
		waitForReads((PhidgetDeviceHandle)phid, 1, 100);

	//set everything to it's echo
	for (i = 0; i < phid->devChannelCnts.numScreens; i++) {
		phid->_contrastSet[i] = phid->contrast[i];
		phid->_backlightSet[i] = phid->backlight[i];
	}

	return (EPHIDGET_OK);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetTextLCDDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetTextLCDDeviceHandle phid = (PhidgetTextLCDDeviceHandle)device;
	uint8_t lastInputState[TEXTLCD_MAXINPUTS];
	uint8_t inputState[TEXTLCD_MAXINPUTS];
	PhidgetChannelHandle channel;
	int i;
	int j;

	assert(phid);
	assert(buffer);

	for (j = 0; j < phid->devChannelCnts.numInputs; j++) {
		inputState[j] = PUNK_BOOL;
		lastInputState[j] = phid->inputState[j];
	}

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1204:
		for (i = 0; i < phid->devChannelCnts.numScreens; i++) {
			phid->backlight[i] = (buffer[i + 2] / 255.0);
			phid->contrast[i] = (buffer[i + 4] / 255.0);
		}
		break;
	case PHIDUID_1219:
		//Inputs
		for (i = 0, j = 0x01; i < 8; i++, j <<= 1) {
			if ((buffer[0] & j) == 0)
				inputState[i] = PFALSE;
			else
				inputState[i] = PTRUE;
		}
		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		if (inputState[i] != PUNK_BOOL)
			phid->inputState[i] = inputState[i];
	}

	/* Digital Input */
	for (i = 0; i < phid->devChannelCnts.numInputs; i++) {
		int chIndex = i + phid->devChannelCnts.numScreens;
		if ((channel = getChannel(phid, chIndex)) != NULL) {
			if (phid->inputState[i] != PUNK_BOOL && phid->inputState[i] != lastInputState[i]) {
				bridgeSendToChannel(channel, BP_STATECHANGE, "%d", (int)(phid->inputState[i]));
			}
			PhidgetRelease(&channel);
		}
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetTextLCDDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetTextLCDDeviceHandle phid = (PhidgetTextLCDDeviceHandle)ch->parent;
	PhidgetLCD_ScreenSize screenSize;
	PhidgetReturnCode res;
	double dutyCycle;
	int i;
	int j;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_TEXTLCD);

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

	case PHIDCHCLASS_DIGITALOUTPUT:
		assert(ch->index < phid->devChannelCnts.numOutputs);
		switch (bp->vpkt) {
		case BP_SETSTATE:
			phid->outputState[ch->index] = getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_DIGITALOUTPUT_PACKET));
		case BP_SETDUTYCYCLE:
			dutyCycle = getBridgePacketDouble(bp, 0);
			if (dutyCycle != 0.0 && dutyCycle != 1.0)
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Duty cycle must be 0 or 1."));
			phid->outputState[ch->index] = dutyCycle;
			return (_sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_DIGITALOUTPUT_PACKET));
		case BP_OPENRESET:
		case BP_CLOSERESET:
			phid->outputState[ch->index] = PFALSE;
			return (_sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_DIGITALOUTPUT_PACKET));
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_LCD:
		assert(ch->index < phid->devChannelCnts.numScreens);
		switch (bp->vpkt) {
		case BP_SETBACKLIGHT:
			switch (phid->phid.deviceInfo.UDD->uid) {
			case PHIDUID_1202_TEXTLCD:
			case PHIDUID_1215:
			case PHIDUID_1219:
				//Only support true or false, so we round it.
				phid->_backlightSet[ch->index] = getBridgePacketDouble(bp, 0) ? 1 : 0;
				break;
			default:
				phid->_backlightSet[ch->index] = getBridgePacketDouble(bp, 0);
				break;
			}
			if (phid->fullStateEcho == PFALSE)
				phid->backlight[ch->index] = phid->_backlightSet[ch->index];
			return (_sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_BACKLIGHT_PACKET));

		case BP_SETCURSORON:
			phid->cursorOn[ch->index] = getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_CURSOR_PACKET));

		case BP_SETCURSORBLINK:
			phid->cursorBlink[ch->index] = getBridgePacketInt32(bp, 0);
			return (_sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_CURSOR_PACKET));

		case BP_SETSCREENSIZE:
			screenSize = (PhidgetLCD_ScreenSize)getBridgePacketInt32(bp, 0);

			TESTRANGE_IOP(bp->iop, "%d", screenSize, SCREEN_SIZE_NONE, SCREEN_SIZE_4x40);

			// 4x40 Only supported on screen 0
			if (screenSize == SCREEN_SIZE_4x40 && ch->index != 0)
				return (MOS_ERROR(bp->iop, EPHIDGET_UNSUPPORTED, "4x40 screen size can only be set on screen 0."));

			// 4x40 makes the other screen unavailable
			// XXX - this information should be communicated up to the channel
			if (screenSize == SCREEN_SIZE_4x40) {
				phid->height[1] = 0;
				phid->width[1] = 0;
				phid->screenSize[1] = SCREEN_SIZE_NONE;
			}

			// Trying to set a size of screen 1 when screen 0 is 4x40 - set screen 0 to NONE
			if (ch->index == 1 && phid->screenSize[0] == SCREEN_SIZE_4x40 && screenSize != SCREEN_SIZE_NONE) {
				phid->height[0] = 0;
				phid->width[0] = 0;
				phid->screenSize[0] = SCREEN_SIZE_NONE;
			}

			//clear text
			for (i = 0; i < phid->devChannelCnts.numScreens; i++) {
				for (j = 0; j < TEXTLCD_MAXROWS; j++)
					memset(phid->internalFramebuffer[i][j], ' ', TEXTLCD_MAXCOLS);
			}

			// reset cursor position
			phid->cursorLocation[ch->index] = 0;
			phid->cursorColumn[ch->index] = 0;
			phid->cursorScreen[ch->index] = ch->index;

			// set screen size
			PhidgetTextLCDDevice_setWidthHeightFromScreenSize(screenSize, &phid->width[ch->index], &phid->height[ch->index]);
			phid->screenSize[ch->index] = screenSize;

			// flush
			return (writeScreen(bp->iop, phid, ch->index));

		case BP_SETCONTRAST:
			phid->_contrastSet[ch->index] = getBridgePacketDouble(bp, 0);
			if (!phid->fullStateEcho)
				phid->contrast[ch->index] = phid->_contrastSet[ch->index];
			return (_sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_CONTRAST_PACKET));

		case BP_WRITETEXT:
			if (phid->screenSize[ch->index] == (PhidgetLCD_ScreenSize)PUNK_ENUM)
				return (EPHIDGET_NOTCONFIGURED);
			return (_writeString(bp->iop, phid, ch->index, getBridgePacketInt32(bp, 2), getBridgePacketInt32(bp, 1), getBridgePacketString(bp, 3)));

		case BP_SETCHARACTERBITMAP:
			if (phid->screenSize[ch->index] == (PhidgetLCD_ScreenSize)PUNK_ENUM)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Screen size must be configured."));
			if (getBridgePacketInt32(bp, 0) != FONT_5x8)
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Font must be 5x8."));
			return (_setCustomCharacter(bp->iop, phid, ch->index, getBridgePacketString(bp, 1), getBridgePacketUInt8Array(bp, 2)));

		case BP_FLUSH:
			if (phid->screenSize[ch->index] == (PhidgetLCD_ScreenSize)PUNK_ENUM)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Screen size must be configured."));
			return (writeScreen(bp->iop, phid, ch->index));

		case BP_CLEAR:
			for (i = 0; i < phid->devChannelCnts.numScreens; i++) {
				for (j = 0; j < TEXTLCD_MAXROWS; j++)
					memset(phid->internalFramebuffer[i][j], ' ', TEXTLCD_MAXCOLS);
			}
			phid->cursorLocation[ch->index] = 0;
			phid->cursorColumn[ch->index] = 0;
			phid->cursorScreen[ch->index] = ch->index;
			return (EPHIDGET_OK);

		case BP_INITIALIZE:
			if (phid->screenSize[ch->index] == (PhidgetLCD_ScreenSize)PUNK_ENUM)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOTCONFIGURED, "Screen size must be configured."));

			//clear text
			for (i = 0; i < phid->devChannelCnts.numScreens; i++) {
				for (j = 0; j < TEXTLCD_MAXROWS; j++)
					memset(phid->internalFramebuffer[i][j], ' ', TEXTLCD_MAXCOLS);
			}

			phid->init[ch->index] = PTRUE;
			return (_sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_INIT_PACKET));

		case BP_OPENRESET:
		case BP_CLOSERESET:

			//Cursors off
			phid->cursorOn[ch->index] = PFALSE;
			phid->cursorBlink[ch->index] = PFALSE;
			res = _sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_CURSOR_PACKET);
			if (res != EPHIDGET_OK)
				return (res);

			//Clear screen
			for (i = 0; i < phid->devChannelCnts.numScreens; i++) {
				for (j = 0; j < TEXTLCD_MAXROWS; j++)
					memset(phid->internalFramebuffer[i][j], ' ', TEXTLCD_MAXCOLS);
			}
			phid->cursorLocation[ch->index] = 0;
			phid->cursorColumn[ch->index] = 0;
			phid->cursorScreen[ch->index] = ch->index;

			res = writeScreen(bp->iop, phid, ch->index);
			if (res != EPHIDGET_OK)
				return (res);

			// Backlight off
			phid->_backlightSet[ch->index] = 0;
			phid->backlight[ch->index] = 0;
			res = _sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_BACKLIGHT_PACKET);
			if (res != EPHIDGET_OK)
				return (res);

			// Reset contrast
			switch (phid->phid.deviceInfo.UDD->uid) {
			case PHIDUID_1215:
				phid->_contrastSet[ch->index] = 0.5;
				break;
			case PHIDUID_1219:
				phid->_contrastSet[ch->index] = 0.5;
				break;
			case PHIDUID_1202_TEXTLCD:
				phid->_contrastSet[ch->index] = 0.502;
				break;
			case PHIDUID_1202_TEXTLCD_BRIGHTNESS:
				phid->_contrastSet[ch->index] = 0.491;
				break;
			case PHIDUID_1204:
				phid->_contrastSet[ch->index] = 0.785;

				// Reset screen size
				phid->width[ch->index] = 0;
				phid->height[ch->index] = 0;
				phid->screenSize[ch->index] = PUNK_ENUM;
				break;
			default:
				MOS_PANIC("Unexpected device");
			}
			phid->contrast[ch->index] = phid->_contrastSet[ch->index];
			res = _sendpacket(bp->iop, phid, (ch->index << 8) | TEXTLCD_CONTRAST_PACKET);
			if (res != EPHIDGET_OK)
				return (res);

			return (EPHIDGET_OK);
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Unexpected channel class");
	}
}

//makePacket - constructs a packet using current device state
static PhidgetReturnCode _sendpacket(mosiop_t iop, PhidgetTextLCDDeviceHandle phid, int Index) {
	unsigned char buffer[MAX_OUT_PACKET_SIZE] = { 0 };
	int packetType = Index & 0xFF;
	int screen = Index >> 8;
	int i, j;

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1202_TEXTLCD:
	case PHIDUID_1202_TEXTLCD_BRIGHTNESS:
	case PHIDUID_1204:
	case PHIDUID_1215:
	case PHIDUID_1219:
		switch (packetType) {
		case TEXTLCD_BACKLIGHT_PACKET: //backlight
			switch (phid->phid.deviceInfo.UDD->uid) {
			case PHIDUID_1202_TEXTLCD:
			case PHIDUID_1215:
			case PHIDUID_1219:
				buffer[0] = phid->_backlightSet[screen] ? PTRUE : PFALSE;
				break;

			case PHIDUID_1202_TEXTLCD_BRIGHTNESS:
			case PHIDUID_1204:
				buffer[0] = phid->_backlightSet[screen] ? PTRUE : PFALSE;
				buffer[1] = (uint8_t)(phid->_backlightSet[screen] * 255);
				break;

			default:
				break;
			}

			return (writeBuffer(iop, phid, TEXTLCD_BACKLIGHT_PACKET, buffer, 2, screen));

		case TEXTLCD_CONTRAST_PACKET: //contrast
			buffer[0] = (uint8_t)(phid->_contrastSet[screen] * 255);

			return (writeBuffer(iop, phid, TEXTLCD_CONTRAST_PACKET, buffer, 1, screen));

		case TEXTLCD_INIT_PACKET: //re-init a screen

			phid->cursorOn[screen] = PFALSE;
			phid->cursorBlink[screen] = PFALSE;
			phid->cursorColumn[screen] = 0;
			phid->cursorLocation[screen] = 0;
			phid->cursorScreen[screen] = screen;

			if (phid->screenSize[screen] == SCREEN_SIZE_4x40 && screen == 0) {
				screen = 2;
				phid->cursorOn[1] = PFALSE;
				phid->cursorBlink[1] = PFALSE;
				phid->cursorColumn[1] = 0;
				phid->cursorLocation[1] = 0;
				phid->cursorScreen[1] = 1;
			}

			return (writeBuffer(iop, phid, TEXTLCD_INIT_PACKET, buffer, 0, screen));

		case TEXTLCD_CURSOR_PACKET: //LCD commands - Cursor

			if (phid->cursorOn[screen] == PUNK_BOOL) {
				phid->cursorOn[screen] = PFALSE;
			}

			if (phid->cursorBlink[screen] == PUNK_BOOL) {
				phid->cursorBlink[screen] = PFALSE;
			}

			buffer[0] = HD44780_DISPLAY_CNTRL | HD44780_DISPLAY_ON;
			//only actually turn on if the cursor is on-display
			if (phid->cursorColumn[screen] < phid->width[screen]) {
				if (phid->cursorOn[screen])
					buffer[0] |= HD44780_CURSOR_ON;
				if (phid->cursorBlink[screen])
					buffer[0] |= HD44780_CURSOR_BLINK_ON;
			}

			phid->cursorLastScreen[screen] = phid->cursorScreen[screen];
			return (writeBuffer(iop, phid, 0, buffer, 1, phid->cursorScreen[screen]));

		case TEXTLCD_DIGITALOUTPUT_PACKET:
			assert(phid->phid.deviceInfo.UDD->uid == PHIDUID_1219);
			for (i = 0, j = 1; i < 8; i++, j <<= 1) {

				//set unknown outputs to false
				if (phid->outputState[i] == PUNK_BOOL)
					phid->outputState[i] = PFALSE;

				if (phid->outputState[i])
					buffer[0] |= j;
			}
			buffer[7] = 0x10;  /* Signal an output */

			return (PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, buffer, getMaxOutPacketSize((PhidgetDeviceHandle)phid)));
		default:
			MOS_PANIC("Unexpected packet type");
		}
		break;
	default:
		MOS_PANIC("Unexpected device");
	}
}

/*
 * Returns DDRAM position for the requested row and col
 * May change screen number for the special 4x40 case
 */
static int
getScreenAndPos(PhidgetTextLCDDeviceHandle phid, int *screen, int row, int col) {
	int cols = phid->width[*screen];
	int pos = 0;

	switch (row) {
	case 0:
		pos = 0x00;
		break;
	case 1:
		pos = 0x40;
		break;
	case 2:
		if (cols == 16) {
			pos = 0x10;
		} else if (cols == 40) {
			pos = 0x00;
			*screen = 1;
		} else {
			pos = 0x14;
		}
		break;
	case 3:
		if (cols == 16) {
			pos = 0x50;
		} else if (cols == 40) {
			pos = 0x40;
			*screen = 1;
		} else {
			pos = 0x54;
		}
		break;
	}

	return pos + col;
}

//0x00 is the escape character
//escape 0x01 and 0x02 because these are interpreted as special commands by the firmware
//we don't have to worry about 0x00 in the input string because that would end the string.
static void
escapeLcdString(char *string, size_t len, char *buffer, int *buf_ptr) {
	uint32_t ui;

	//Escape 0x01, 0x02
	for (ui = 0; ui < len; ui++) {
		if ((string[ui] == 0x01) || (string[ui] == 0x02))
			buffer[(*buf_ptr)++] = TEXTLCD_ESCAPE_CHAR;
		buffer[(*buf_ptr)++] = string[ui];
	}
}

static PhidgetReturnCode
writeBuffer(mosiop_t iop, PhidgetTextLCDDeviceHandle phid, int packetType, uint8_t *buffer, int buflen, int screen) {
	PhidgetReturnCode ret;
	uint8_t outbuf[8];
	int len;
	int i;

	// NOTE: This is required for 1204 when switching screens to avoid corruption of LCD screen.
	if (phid->lastScreenWritten != screen)
		mos_usleep(10000);

	// NOTE: This is required for 1204, because sending init right after another packet can lock up the device
	if (packetType == TEXTLCD_INIT_PACKET)
		mos_usleep(10000);

	if (packetType != 0) {
		assert(buflen < 8);

		memset(outbuf, 0, 8);
		memcpy(outbuf, buffer, buflen);

		outbuf[7] = packetType | TEXTLCD_SCREEN(screen);

		ret = PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, outbuf, 8);
		if (ret != EPHIDGET_OK)
			return (ret);
	} else {
		/* we have to send the whole form_buffer, 7 bytes at a time */
		for (i = 0; i < buflen; i += 7) {

			/* data length for this packet */
			len = buflen - i;
			if (len > 7)
				len = 7;

			memset(outbuf, 0, 8);
			memcpy(outbuf, buffer + i, len);
			//memset(outbuf + len, 0, 7 - len);

			/* choose screen and data length */
			outbuf[7] = (uint8_t)len | TEXTLCD_SCREEN(screen);

			/* send to device */
			ret = PhidgetDevice_sendpacket(iop, (PhidgetDeviceHandle)phid, outbuf, 8);
			if (ret != EPHIDGET_OK)
				return (ret);
		}
	}

	phid->lastScreenWritten = screen;
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
writeScreen(mosiop_t iop, PhidgetTextLCDDeviceHandle phid, int Index) {
	uint8_t buffer[250] = { 0 };
	PhidgetReturnCode ret;
	int screenEnable;
	int changedRows;
	int buf_ptr;
	int pos;
	int i;

	ret = EPHIDGET_OK;

	// If the framebuffer has not changed, we don't need to write anything
	changedRows = 0;
	for (i = 0; i < phid->height[Index]; i++) {
		if (!phid->displayFramebufferValid[Index][i]
		  || memcmp(phid->internalFramebuffer[Index][i], phid->displayFramebuffer[Index][i], phid->width[Index])) {
			changedRows |= (0x01 << i);
		}
	}

	if (changedRows || phid->cursorOn[Index] || phid->cursorBlink[Index]) {

		//turn off cursor if it's on
		if (phid->cursorOn[Index] == PTRUE || phid->cursorBlink[Index] == PTRUE) {
			buffer[0] = TEXTLCD_COMMAND_MODE; /* Command Mode */
			buffer[1] = HD44780_DISPLAY_CNTRL | HD44780_DISPLAY_ON;
			ret = writeBuffer(iop, phid, 0, buffer, 2, phid->cursorLastScreen[Index]);
			if (ret != EPHIDGET_OK)
				return (ret);
		}

		buf_ptr = 0;
		buffer[buf_ptr++] = TEXTLCD_COMMAND_MODE; /* Command Mode */

		for (i = 0; i < phid->height[Index]; i++) {
			if (changedRows & (0x01 << i)) {
				//logdebug("Writing row %d: \"%.*s\"", i, phid->width[Index], phid->internalFramebuffer[Index][i]);

				screenEnable = Index;
				pos = getScreenAndPos(phid, &screenEnable, i, 0);

				buffer[buf_ptr++] = pos | HD44780_SET_DDRAM_ADDR; /* Set DDRAM Address */
				buffer[buf_ptr++] = TEXTLCD_DATA_MODE; /* Data Mode */

				/* get escaped string */
				escapeLcdString((char *)phid->internalFramebuffer[Index][i], phid->width[Index], (char *)buffer, &buf_ptr);

				buffer[buf_ptr++] = TEXTLCD_COMMAND_MODE; /* Command Mode */

				ret = writeBuffer(iop, phid, 0, buffer, buf_ptr, screenEnable);
				if (ret != EPHIDGET_OK)
					return (ret);

				buf_ptr = 0;
				buffer[buf_ptr++] = TEXTLCD_COMMAND_MODE; /* Command Mode */

				memcpy(phid->displayFramebuffer[Index][i], phid->internalFramebuffer[Index][i], phid->width[Index]);
				phid->displayFramebufferValid[Index][i] = PTRUE;
			}
		}

		/* Set DDRAM Address so cursor appears in the right place */
		buffer[buf_ptr++] = phid->cursorLocation[Index] | HD44780_SET_DDRAM_ADDR;

		/* re-enable the cursors if we need to */
		if ((phid->cursorOn[Index] == PTRUE || phid->cursorBlink[Index] == PTRUE) && phid->cursorColumn[Index] < phid->width[Index]) {
			uint8_t cursorData = HD44780_DISPLAY_CNTRL | HD44780_DISPLAY_ON;
			if (phid->cursorOn[Index])
				cursorData |= HD44780_CURSOR_ON;
			if (phid->cursorBlink[Index])
				cursorData |= HD44780_CURSOR_BLINK_ON;
			buffer[buf_ptr++] = cursorData;
		}

		ret = writeBuffer(iop, phid, 0, buffer, buf_ptr, phid->cursorScreen[Index]);
		if (ret != EPHIDGET_OK)
			return (ret);
		phid->cursorLastScreen[Index] = phid->cursorScreen[Index];
	}
	return ret;
}

static void CCONV
PhidgetTextLCDDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetTextLCDDevice));
	*phid = NULL;
}

void
PhidgetTextLCDDevice_setWidthHeightFromScreenSize(PhidgetLCD_ScreenSize size, int *width, int *height) {
	switch (size) {
	case SCREEN_SIZE_NONE:
		(*height) = 0;
		(*width) = 0;
		break;
	case SCREEN_SIZE_1x8:
		(*height) = 1;
		(*width) = 8;
		break;
	case SCREEN_SIZE_2x8:
		(*height) = 2;
		(*width) = 8;
		break;
	case SCREEN_SIZE_1x16:
		(*height) = 1;
		(*width) = 16;
		break;
	case SCREEN_SIZE_2x16:
		(*height) = 2;
		(*width) = 16;
		break;
	case SCREEN_SIZE_4x16:
		(*height) = 4;
		(*width) = 16;
		break;
	case SCREEN_SIZE_2x20:
		(*height) = 2;
		(*width) = 20;
		break;
	case SCREEN_SIZE_4x20:
		(*height) = 4;
		(*width) = 20;
		break;
	case SCREEN_SIZE_2x24:
		(*height) = 2;
		(*width) = 24;
		break;
	case SCREEN_SIZE_1x40:
		(*height) = 1;
		(*width) = 40;
		break;
	case SCREEN_SIZE_2x40:
		(*height) = 2;
		(*width) = 40;
		break;
	case SCREEN_SIZE_4x40:
		(*height) = 4;
		(*width) = 40;
		break;
	default:
		break;
	}
}

// === Exported Functions === //

//create and initialize a device structure
PhidgetReturnCode
PhidgetTextLCDDevice_create(PhidgetTextLCDDeviceHandle *phidp) {
	DEVICECREATE_BODY(TextLCDDevice, PHIDCLASS_TEXTLCD);
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
_writeString(mosiop_t iop, PhidgetTextLCDDeviceHandle phid, int screen, int Row, int Column,
  const char *chars) {
	PhidgetReturnCode ret = EPHIDGET_OK;
	int screenEnable;
	int pos;
	int len;

	assert(phid);
	assert(phid->phid.deviceInfo.class == PHIDCLASS_TEXTLCD);
	TESTATTACHED(phid);

	if (Row >= phid->height[screen] || Row < 0)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Invalid row."));
	if (Column >= phid->width[screen] || Column < 0)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Invalid column."));

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1204:
	case PHIDUID_1202_TEXTLCD:
	case PHIDUID_1215:
	case PHIDUID_1219:
	case PHIDUID_1202_TEXTLCD_BRIGHTNESS:
		len = (int)strlen(chars);
		if (len > (phid->width[screen] - Column))
			len = phid->width[screen] - Column;

		screenEnable = screen;
		pos = getScreenAndPos(phid, &screenEnable, Row, Column);
		phid->cursorLocation[screen] = pos + len;
		phid->cursorColumn[screen] = Column + len;
		phid->cursorScreen[screen] = screenEnable;

		memcpy(phid->internalFramebuffer[screen][Row] + Column, chars, len);

		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	return ret;
}

static PhidgetReturnCode
_setCustomCharacter(mosiop_t iop, PhidgetTextLCDDeviceHandle phid, int screen, const char *character, const uint8_t *bitmap) {
	uint8_t buffer[8] = { 0 };
	PhidgetReturnCode ret = EPHIDGET_OK;
	int Val1 = 0, Val2 = 0;
	int i, j, k;
	int Index;
	int screenEnable;

	assert(phid);
	assert(phid->phid.deviceInfo.class == PHIDCLASS_TEXTLCD);
	TESTATTACHED(phid);

	if (strlen(character) != 1)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Multi-byte characters are not supported."));
	Index = character[0];

	if ((Index < 0) || (Index > 15))
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Charater code must be between 0 and 15."));
	k = 4;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 5; j++, k--) {
			if (bitmap[i * 5 + j])
				Val1 |= 0x01 << k;
		}
		k += 10;
	}
	k = 4;
	for (i = 4; i < 8; i++) {
		for (j = 0; j < 5; j++, k--) {
			if (bitmap[i * 5 + j])
				Val2 |= 0x01 << k;
		}
		k += 10;
	}

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1204:
	case PHIDUID_1202_TEXTLCD:
	case PHIDUID_1215:
	case PHIDUID_1219:
	case PHIDUID_1202_TEXTLCD_BRIGHTNESS:

		//actual index is 0-7
		if (Index >= 8)
			Index -= 8;

		screenEnable = screen;

	again:
		buffer[0] = HD44780_DISPLAY_CNTRL | HD44780_DISPLAY_ON; /* disable cursors */
		buffer[1] = HD44780_SET_CGRAM_ADDR | TEXTLCD_CGRAM_ADDR(Index); /* set CGRAM address */
		buffer[2] = TEXTLCD_DATA_MODE;
		buffer[3] = (Val1 & 0x1F) | 0x80;
		buffer[4] = ((Val1 >> 5) & 0x1F) | 0x80;
		buffer[5] = ((Val1 >> 10) & 0x1F) | 0x80;
		buffer[6] = ((Val1 >> 15) & 0x1F) | 0x80;
		buffer[7] = 7 | TEXTLCD_SCREEN(screenEnable);

		ret = writeBuffer(iop, phid, 0, buffer, 7, screenEnable);
		if (ret != EPHIDGET_OK)
			goto done;

		buffer[0] = (Val2 & 0x1F) | 0x80;
		buffer[1] = ((Val2 >> 5) & 0x1F) | 0x80;
		buffer[2] = ((Val2 >> 10) & 0x1F) | 0x80;
		buffer[3] = ((Val2 >> 15) & 0x1F) | 0x80;
		buffer[4] = TEXTLCD_COMMAND_MODE;
		buffer[5] = phid->cursorLocation[screen] | HD44780_SET_DDRAM_ADDR; /* reset DDRAM address for cursors */
		buffer[6] = HD44780_DISPLAY_CNTRL | HD44780_DISPLAY_ON; /* enable cursors */

		if (phid->cursorColumn[screen] < phid->width[screen] && screenEnable == phid->cursorScreen[screen]) {
			if (phid->cursorOn[screen])
				buffer[6] |= HD44780_CURSOR_ON;
			if (phid->cursorBlink[screen])
				buffer[6] |= HD44780_CURSOR_BLINK_ON;
		}
		buffer[7] = 7 | TEXTLCD_SCREEN(screenEnable);

		ret = writeBuffer(iop, phid, 0, buffer, 7, screenEnable);
		if (ret != EPHIDGET_OK)
			goto done;

		/* for 4x40, we need to set these characters on screen 1 as well */
		if (phid->screenSize[0] == SCREEN_SIZE_4x40 && screenEnable == 0) {
			screenEnable = 1;
			goto again;
		}
	done:

		break;
	default:
		MOS_PANIC("Unexpected device");
	}

	return ret;
}
