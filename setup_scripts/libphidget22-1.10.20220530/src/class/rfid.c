/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/rfid.gen.h"
#include "class/rfid.gen.c"

// Access the PhidgetRFIDSupport struct via the channel private pointer
#define RFID_SUPPORT(ch) ((PhidgetRFIDSupportHandle)(((PhidgetChannelHandle)(ch))->private))

static void
PhidgetRFID_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {}

static void CCONV
PhidgetRFID_free(PhidgetChannelHandle *ch) {
	if (ch && *ch)
		PhidgetRFIDSupport_free((PhidgetRFIDSupportHandle *)&(*ch)->private);
	_free(ch);
}

API_PRETURN
PhidgetRFID_create(PhidgetRFIDHandle *phidp) {
	PhidgetReturnCode res;

	res = _create(phidp);
	if (res == EPHIDGET_OK)
		res = PhidgetRFIDSupport_create((PhidgetRFIDSupportHandle *)&(*phidp)->phid.private);

	return (res);
}

static PhidgetReturnCode CCONV
PhidgetRFID_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetRFID_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetRFID_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetRFIDSupport_init(RFID_SUPPORT(phid));
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetRFID_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetRFID_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetRFIDHandle ch;
	PhidgetReturnCode res;
	const char* tagString;
	uint64_t uniqueID;
	int i;

	ch = (PhidgetRFIDHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_WRITE:
		if (!supportedRFIDProtocol(phid, (PhidgetRFID_Protocol)getBridgePacketInt32(bp, 1)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Specified RFID protocol is not supported by this device."));		
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 2)); //lockTag

		//Verify the tag string is valid for the chosen protocol
		tagString = getBridgePacketString(bp, 0);
		switch ((PhidgetRFID_Protocol)getBridgePacketInt32(bp, 1)) {
		case PROTOCOL_EM4100:
			if (strlen(tagString) != 10)
				if (!(strlen(tagString) == 12 && tagString[0] == '0' && tagString[1] == 'x'))
					return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "EM4100 tag string must be specified as 'XXXXXXXXXX' or '0xXXXXXXXXXX' where 'X' is a hex digit."));
			for (i = 0; i < 10; i++) {
				char hexDigit = tagString[(strlen(tagString) - 1) - i];
				if((hexDigit < '0' || hexDigit > '9') && (hexDigit < 'A' || hexDigit > 'F') && (hexDigit < 'a' || hexDigit > 'f'))
					return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "EM4100 tag string must be specified as 'XXXXXXXXXX' or '0xXXXXXXXXXX' where 'X' is a hex digit."));
			}
			break;
		case PROTOCOL_ISO11785_FDX_B:
			if (strlen(tagString) != 15)
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "ISO11785_FDX_B tag must be specified as 15 decimal digits."));

			for (i = 0; i < 15; i++) {
				char digit = tagString[(strlen(tagString) - 1) - i];
				if (digit < '0' || digit > '9')
					return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "ISO11785_FDX_B tag must be a string of 15 decimal digits"));
			}

			//Get uniqueID
			uniqueID = (uint64_t)strtoll(tagString + 3, NULL, 10);
			//must be 38-bit or less
			if (uniqueID > 0x3FFFFFFFFFLL)
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "ISO11785_FDX_B Unique ID (last 12 digits) must be <= 274877906943."));
			break;
		case PROTOCOL_PHIDGETS:
			if (strlen(tagString) > 24)
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "PHIDGETS_TAG must be <= 24 characters."));
			break;
		}
		res = _bridgeInput(phid, bp);
		break;
	case BP_TAG:
		ch->tagPresent = PTRUE;
		FIRECH(ch, Tag, getBridgePacketString(bp, 0), (PhidgetRFID_Protocol)getBridgePacketInt32(bp, 1));
		PhidgetRunLock(ch);
		mos_strlcpy(ch->lastTagString, getBridgePacketString(bp, 0), RFIDDevice_MAX_TAG_STRING_LEN);
		ch->lastTagProtocol = (PhidgetRFID_Protocol)getBridgePacketInt32(bp, 1);
		PhidgetRunUnlock(ch);
		break;
	case BP_TAGLOST:
		ch->tagPresent = PFALSE;
		FIRECH(ch, TagLost, getBridgePacketString(bp, 0), (PhidgetRFID_Protocol)getBridgePacketInt32(bp, 1));
		break;
	case BP_SETANTENNAON:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK) {
			if (ch->tagPresent == PUNK_BOOL)
				ch->tagPresent = PFALSE;
		}
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetRFID_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetRFID_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetRFID_getLastTag(PhidgetRFIDHandle ch, char *tagString, size_t tagStringLen, PhidgetRFID_Protocol *protocol) {
	size_t len;

	TESTPTR_PR(ch);
	TESTPTR_PR(tagString);
	TESTPTR_PR(protocol);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RFID);
	TESTATTACHED_PR(ch);

	PhidgetRunLock(ch);
	len = mos_strlen(ch->lastTagString);
	if (len > 0) {
		if (tagStringLen <= len)
			return (PHID_RETURN_ERRSTR(EPHIDGET_NOSPC, "tagString too short (%d / %d).", tagStringLen, len));
		mos_strlcpy(tagString, ch->lastTagString, tagStringLen);
		*protocol = ch->lastTagProtocol;
		PhidgetRunUnlock(ch);
		return (EPHIDGET_OK);
	}
	PhidgetRunUnlock(ch);
	return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
}
