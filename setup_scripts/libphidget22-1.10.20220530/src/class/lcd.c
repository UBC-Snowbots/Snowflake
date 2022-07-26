/* Generated: Thu Feb 04 2016 11:46:27 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/lcd.gen.h"
#include "class/lcd.gen.c"

static void CCONV
PhidgetLCD_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {}

static void CCONV
PhidgetLCD_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetLCD_create(PhidgetLCDHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetLCD_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetLCD_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetLCD_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetLCD_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetLCD_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;
	PhidgetLCDHandle ch;
	int height;
	int width;
	int font;

	ch = (PhidgetLCDHandle)phid;

	switch (bp->vpkt) {
	case BP_WRITEBITMAP:
		TESTRANGE_IOP(bp->iop, "%d", getBridgePacketInt32(bp, 0), 0, ch->width - 1);
		TESTRANGE_IOP(bp->iop, "%d", getBridgePacketInt32(bp, 1), 0, ch->height - 1);
		TESTRANGE_IOP(bp->iop, "%d", getBridgePacketInt32(bp, 2), 1, ch->width);
		TESTRANGE_IOP(bp->iop, "%d", getBridgePacketInt32(bp, 3), 1, ch->height);
		res = _bridgeInput(phid, bp);
		break;

	case BP_SETFONTSIZE:
		font = getBridgePacketInt32(bp, 0);
		width = getBridgePacketInt32(bp, 1);
		height = getBridgePacketInt32(bp, 2);
		//only writable frame buffers can be used for user-defined fonts (0,1,2)
		TESTRANGE_IOP(bp->iop, "%d", font, 1, 2);
		//verify dimensions fit on screen
		TESTRANGE_IOP(bp->iop, "%d", height, 0, ch->height);
		TESTRANGE_IOP(bp->iop, "%d", width, 0, ch->width);
		ch->fontHeight[font] = (uint8_t)height;
		ch->fontWidth[font] = (uint8_t)width;
		res = EPHIDGET_OK;
		break;

	case BP_SETSCREENSIZE:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK)
			PhidgetTextLCDDevice_setWidthHeightFromScreenSize((PhidgetLCD_ScreenSize)getBridgePacketInt32(bp, 0), &(ch->width), &(ch->height));
		break;

	// These packets wake up an LCD1100
	case BP_SETBACKLIGHT:
	case BP_SETCONTRAST:
	case BP_FLUSH:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK) {
			switch (phid->UCD->uid) {
			case PHIDCHUID_LCD1100_LCD_100:
				ch->sleeping = PFALSE;
				break;
			}
		}
		break;

	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	if (res)
		return (res);

	switch (bp->vpkt) {
	case BP_CLEAR:
	case BP_COPY:
	case BP_DRAWLINE:
	case BP_DRAWPIXEL:
	case BP_DRAWRECT:
	case BP_INITIALIZE:
	case BP_WRITEBITMAP:
	case BP_WRITETEXT:
	case BP_SETFRAMEBUFFER:
		if (ch->autoFlush) {
			res = createBridgePacket(&bp, BP_FLUSH, NULL);
			return DEVBRIDGEINPUT(phid, bp);
		}
		break;
	default:
		break;
	}

	return (res);
}

static void
PhidgetLCD_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetLCD_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetLCD_setCharacterBitmap(PhidgetLCDHandle ch, PhidgetLCD_Font font, const char * character,
  const uint8_t *bitmap) {
	PhidgetReturnCode ret;
	int height;
	int width;

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LCD);
	TESTATTACHED_PR(ch);

	ret = PhidgetLCD_getFontSize(ch, font, &width, &height);
	if (ret != EPHIDGET_OK)
		return (PHID_RETURN(ret));
	if (width <= 0)
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Font width is <= 0."));
	if (height <= 0)
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Font height is <= 0."));

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHARACTERBITMAP, NULL, NULL,
		"%d%s%*R", font, character, (int)(width * height), bitmap);
}

API_VRETURN
PhidgetLCD_setCharacterBitmap_async(PhidgetLCDHandle ch, PhidgetLCD_Font font, const char * character,
  const uint8_t *bitmap, Phidget_AsyncCallback fptr, void *ctx) {
	PhidgetReturnCode res;
	int height;
	int width;

	if (ch == NULL) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (ch->phid.class != PHIDCHCLASS_LCD) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
		return;
	}
	if (!ISATTACHED(ch)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
		return;
	}

	res = PhidgetLCD_getFontSize(ch, font, &width, &height);
	if (res != EPHIDGET_OK) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, res);
		return;
	}
	if (width <= 0) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (height <= 0) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}

	res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHARACTERBITMAP, fptr, ctx,
		"%d%s%*R", font, character, (int)(width * height), bitmap);

	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}

API_PRETURN
PhidgetLCD_getFontSize(PhidgetLCDHandle ch, PhidgetLCD_Font font, int *width, int *height) {
	TESTPTR_PR(ch);
	TESTPTR_PR(width);
	TESTPTR_PR(height);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LCD);
	TESTATTACHED_PR(ch);

	switch (font) {
	case FONT_6x10:
		*width = 6;
		*height = 10;
		break;
	case FONT_5x8:
		*width = 5;
		*height = 8;
		break;
	case FONT_6x12:
		*width = 6;
		*height = 12;
		break;
	case FONT_User1:
	case FONT_User2:
		*width = ch->fontWidth[(int)font];
		*height = ch->fontHeight[(int)font];
		break;
	default:
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Invalid font."));
	}

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLCD_getMaxCharacters(PhidgetLCDHandle ch, PhidgetLCD_Font font, int *maxCharacters) {
	PhidgetReturnCode ret;
	int height;
	int width;

	TESTPTR_PR(ch);
	TESTPTR_PR(maxCharacters);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LCD);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_LCD1100_LCD_100:
		TESTRANGE_PR(font, "%d", FONT_User1, FONT_6x12);

		ret = PhidgetLCD_getFontSize(ch, font, &width, &height);
		if (ret != EPHIDGET_OK)
			return (PHID_RETURN(ret));
		if (width <= 0)
			return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Font width is <= 0."));
		if (height <= 0)
			return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Font height is <= 0."));

		*maxCharacters = MOS_MIN(255, (ch->width / width) * (ch->height / height));
		break;

	case PHIDCHUID_1202_TEXTLCD_000:
	case PHIDCHUID_1202_TEXTLCD_200:
	case PHIDCHUID_1204_TEXTLCD_000:
	case PHIDCHUID_1215_TEXTLCD_000:
	case PHIDCHUID_1219_TEXTLCD_000:
		*maxCharacters = 0xff;
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLCD_writeBitmap(PhidgetLCDHandle ch, int xpos, int ypos, int xsize, int ysize,
	const uint8_t *bitmap) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LCD);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_WRITEBITMAP, NULL, NULL,
		"%d%d%d%d%*R", xpos, ypos, xsize, ysize, (int)(xsize * ysize), bitmap));
}

API_VRETURN
PhidgetLCD_writeBitmap_async(PhidgetLCDHandle ch, int xpos, int ypos, int xsize, int ysize,
	const uint8_t *bitmap, Phidget_AsyncCallback fptr, void *ctx) {
	PhidgetReturnCode res;

	if (ch == NULL) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (ch->phid.class != PHIDCHCLASS_LCD) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
		return;
	}
	if (!ISATTACHED(ch)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
		return;
	}

	res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_WRITEBITMAP, fptr, ctx,
		"%d%d%d%d%*R", xpos, ypos, xsize, ysize, (int)(xsize * ysize), bitmap);

	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}

API_PRETURN
PhidgetLCD_setAutoFlush(PhidgetLCDHandle ch, int autoFlush) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LCD);
	TESTATTACHED_PR(ch);

	ch->autoFlush = autoFlush ? 1 : 0;

	return EPHIDGET_OK;
}
