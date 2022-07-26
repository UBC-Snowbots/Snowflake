/* Generated: Thu Jan 28 2016 10:46:06 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "util/irsupport.h"
#include "class/ir.gen.h"
#include "class/ir.gen.c"

// Access the PhidgetIRSupport struct via the channel private pointer
#define IR_SUPPORT(ch) ((PhidgetIRSupportHandle)(((PhidgetChannelHandle)(ch))->private))

static void
PhidgetIR_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {

}

static void CCONV
PhidgetIR_free(PhidgetChannelHandle *ch) {
	if (ch && *ch)
		PhidgetIRSupport_free((PhidgetIRSupportHandle *)&(*ch)->private);
	_free(ch);
}

API_PRETURN
PhidgetIR_create(PhidgetIRHandle *phidp) {
	PhidgetReturnCode res;

	res = _create(phidp);
	if (res == EPHIDGET_OK)
		res = PhidgetIRSupport_create((PhidgetIRSupportHandle *)&(*phidp)->phid.private);

	return (res);
}

static PhidgetReturnCode CCONV
PhidgetIR_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetIR_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetIR_initAfterOpen(PhidgetChannelHandle phid) {

	PhidgetIRSupport_init(IR_SUPPORT(phid));
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetIR_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetIR_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetIR_CodeInfo codeInfo;
	PhidgetReturnCode res;
	PhidgetIRHandle ch;
	const char *codeStr;
	int32_t lastRepeat;

	ch = (PhidgetIRHandle)phid;

	switch (bp->vpkt) {
	case BP_CODE:
		mos_strlcpy(ch->lastCodeStr, getBridgePacketString(bp, 0), sizeof (ch->lastCodeStr));
		ch->lastCodeInfo.bitCount = getBridgePacketInt32(bp, 1);
		lastRepeat = getBridgePacketInt32(bp, 2);
		ch->lastCodeInfo.encoding = (PhidgetIR_Encoding)getBridgePacketInt32(bp, 3);
		ch->lastCodeKnown = PTRUE;
		FIRECH(ch, Code, ch->lastCodeStr, ch->lastCodeInfo.bitCount, lastRepeat);
		res = EPHIDGET_OK;
		break;

	case BP_REPEAT:
		FIRECH(ch, Code, ch->lastCodeStr, ch->lastCodeInfo.bitCount, PTRUE);
		res = EPHIDGET_OK;
		break;

	case BP_LEARN:
		codeStr = getBridgePacketString(bp, 0);
		res = readCodeInfo(bp, &codeInfo);
		if (res != EPHIDGET_OK)
			break;

		FIRECH(ch, Learn, codeStr, &codeInfo);
		strcpy(ch->lastLearnedCodeStr, codeStr);
		ch->lastLearnedCodeInfo = codeInfo;
		ch->lastLearnedCodeKnown = PTRUE;

		res = EPHIDGET_OK;
		break;

	case BP_RAWDATA:
		FIRECH(ch, RawData, getBridgePacketUInt32Array(bp, 0), getBridgePacketArrayCnt(bp, 0));
		res = EPHIDGET_OK;
		break;

	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetIR_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetIR_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetIR_getLastCode(PhidgetIRHandle ch, char *code, size_t codeLength, uint32_t *bitCount) {
	int len;

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);
	TESTATTACHED_PR(ch);

	if (!ch->lastCodeKnown)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));

	*bitCount = ch->lastCodeInfo.bitCount;

	len = IR_DATASIZE(ch->lastCodeInfo.bitCount) * 2 + 1;
	if ((int)codeLength < len)
		return (PHID_RETURN_ERRSTR(EPHIDGET_NOSPC, "Code length is too small (%d / %d).", codeLength, len));

	mos_strlcpy(code, ch->lastCodeStr, codeLength);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetIR_getLastLearnedCode(PhidgetIRHandle ch, char *code, size_t codeLength, PhidgetIR_CodeInfoHandle codeInfo) {
	int len;

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);
	TESTATTACHED_PR(ch);

	TESTPTR_PR(code);
	TESTPTR_PR(codeInfo);

	if (!ch->lastLearnedCodeKnown)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	len = IR_DATASIZE(ch->lastCodeInfo.bitCount) * 2 + 1;
	if ((int)codeLength < len) {
		return (PHID_RETURN_ERRSTR(EPHIDGET_NOSPC, "Code length is too small (%d / %d).", codeLength, len));
	}

	mos_strlcpy(code, ch->lastLearnedCodeStr, codeLength);
	*codeInfo = ch->lastLearnedCodeInfo;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetIR_transmit(PhidgetIRHandle ch, const char *code, PhidgetIR_CodeInfoHandle codeInfo) {
	PhidgetReturnCode res;
	BridgePacket *bp;

	TESTPTR_PR(ch);
	TESTPTR_PR(code);
	TESTPTR_PR(codeInfo);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);
	TESTATTACHED_PR(ch);

	res = createBridgePacket(&bp, BP_TRANSMIT, "%s", code);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	res = writeCodeInfo(codeInfo, bp);
	if (res != EPHIDGET_OK) {
		destroyBridgePacket(&bp);
		return (PHID_RETURN(res));
	}
	return (bridgeSendBPToDevice((PhidgetChannelHandle)ch, NULL, NULL, bp));
}

API_PRETURN
PhidgetIR_transmitRaw(PhidgetIRHandle ch, const uint32_t *data, size_t dataLength,
	uint32_t carrierFrequency, double dutyCycle, uint32_t gap) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_TRANSMITRAW, NULL, NULL, "%*U%u%g%u", (int)dataLength, data, carrierFrequency, dutyCycle, gap));
}
