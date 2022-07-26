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

#ifndef __CPHIDGETTEXTLCDDEVICE
#define __CPHIDGETTEXTLCDDEVICE

typedef struct _PhidgetTextLCDDevice *PhidgetTextLCDDeviceHandle;
PhidgetReturnCode PhidgetTextLCDDevice_create(PhidgetTextLCDDeviceHandle *phid);

void PhidgetTextLCDDevice_setWidthHeightFromScreenSize(PhidgetLCD_ScreenSize size, int *width, int *height);

#define TEXTLCD_MAXROWS 4
#define TEXTLCD_MAXCOLS 40
#define TEXTLCD_MAXSCREENS 2
#define TEXTLCD_MAXINPUTS 8
#define TEXTLCD_MAXOUTPUTS 8

#define TEXTLCD_CURSOR_PACKET			0x00
#define TEXTLCD_DIGITALOUTPUT_PACKET	0x02
#define TEXTLCD_BACKLIGHT_PACKET		0x11
#define TEXTLCD_CONTRAST_PACKET			0x12
#define TEXTLCD_INIT_PACKET				0x13

#define TEXTLCD_SCREEN(x)			(x << 5)
#define TEXTLCD_CGRAM_ADDR(x)		(x << 3)	//each custom character takes 8 bytes of CGRAM storage

#define TEXTLCD_ESCAPE_CHAR			0x00
#define TEXTLCD_COMMAND_MODE		0x01
#define TEXTLCD_DATA_MODE			0x02

//HD44780 commands
#define HD44780_CLEAR_DISPLAY	0x01
#define HD44780_CURSOR_HOME		0x02

//These are ORed together
#define HD44780_DISPLAY_CNTRL	0x08
#define HD44780_DISPLAY_ON		0x04
#define HD44780_CURSOR_ON		0x02
#define HD44780_CURSOR_BLINK_ON	0x01

#define HD44780_SET_CGRAM_ADDR	0x40
#define HD44780_SET_DDRAM_ADDR	0x80

struct _PhidgetTextLCDDevice {
#undef devChannelCnts
#define devChannelCnts	phid.deviceInfo.UDD->channelCnts.textlcd
	PhidgetDevice phid;

	/* Public Members */

	uint8_t cursorOn[TEXTLCD_MAXSCREENS];
	uint8_t cursorBlink[TEXTLCD_MAXSCREENS];
	double contrast[TEXTLCD_MAXSCREENS];
	double backlight[TEXTLCD_MAXSCREENS];
	int height[TEXTLCD_MAXSCREENS];
	int width[TEXTLCD_MAXSCREENS];
	PhidgetLCD_ScreenSize screenSize[TEXTLCD_MAXSCREENS];

	uint8_t inputState[TEXTLCD_MAXINPUTS];
	uint8_t outputState[TEXTLCD_MAXOUTPUTS];

	/* Private Members */

	int cursorLocation[TEXTLCD_MAXSCREENS], cursorColumn[TEXTLCD_MAXSCREENS];
	int cursorScreen[TEXTLCD_MAXSCREENS], cursorLastScreen[TEXTLCD_MAXSCREENS];
	int lastScreenWritten;

	double _contrastSet[TEXTLCD_MAXSCREENS], _backlightSet[TEXTLCD_MAXSCREENS];

	uint8_t fullStateEcho;

	uint8_t internalFramebuffer[TEXTLCD_MAXSCREENS][TEXTLCD_MAXROWS][TEXTLCD_MAXCOLS];
	uint8_t displayFramebuffer[TEXTLCD_MAXSCREENS][TEXTLCD_MAXROWS][TEXTLCD_MAXCOLS];
	int displayFramebufferValid[TEXTLCD_MAXSCREENS][TEXTLCD_MAXROWS];

	uint8_t init[TEXTLCD_MAXSCREENS];
} typedef PhidgetTextLCDDeviceInfo;

#endif
