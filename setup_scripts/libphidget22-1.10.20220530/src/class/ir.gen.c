/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

#include "device/irdevice.h"
static void CCONV PhidgetIR_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetIR_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetIR_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetIR_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetIR_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetIR_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetIR_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetIR_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetIR_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetIR {
	struct _PhidgetChannel phid;
	PhidgetIR_CodeInfo lastCodeInfo;
	PhidgetIR_CodeInfo lastLearnedCodeInfo;
	char lastCodeStr[33];
	char lastLearnedCodeStr[33];
	int lastCodeKnown;
	int lastLearnedCodeKnown;
	PhidgetIR_OnCodeCallback Code;
	void *CodeCtx;
	PhidgetIR_OnLearnCallback Learn;
	void *LearnCtx;
	PhidgetIR_OnRawDataCallback RawData;
	void *RawDataCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetIRHandle ch;
	int version;

	ch = (PhidgetIRHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 1) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 1 - functionality may be limited.", phid, version);
	}

	if(version >= 0)
		memcpy(&ch->lastCodeStr, getBridgePacketUInt8ArrayByName(bp, "lastCodeStr"), sizeof (char) * 33);
	if(version >= 0)
		memcpy(&ch->lastLearnedCodeStr, getBridgePacketUInt8ArrayByName(bp, "lastLearnedCodeStr"),
	  sizeof (char) * 33);
	if(version >= 0)
		ch->lastCodeKnown = getBridgePacketInt32ByName(bp, "lastCodeKnown");
	if(version >= 0)
		ch->lastLearnedCodeKnown = getBridgePacketInt32ByName(bp, "lastLearnedCodeKnown");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetIRHandle ch;

	ch = (PhidgetIRHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",lastCodeStr=%33R"
	  ",lastLearnedCodeStr=%33R"
	  ",lastCodeKnown=%d"
	  ",lastLearnedCodeKnown=%d"
	  ,1 /* class version */
	  ,ch->lastCodeStr
	  ,ch->lastLearnedCodeStr
	  ,ch->lastCodeKnown
	  ,ch->lastLearnedCodeKnown
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_TRANSMIT:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_TRANSMITRAW:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_TRANSMITREPEAT:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetIRHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetIRHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_1055_IR_100:
		break;
	case PHIDCHUID_1055_IR_200_USB:
		break;
	case PHIDCHUID_1055_IR_200_VINT:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	memset(&ch->lastCodeInfo, 0, sizeof (ch->lastCodeInfo));
	memset(&ch->lastLearnedCodeInfo, 0, sizeof (ch->lastLearnedCodeInfo));
	memset(ch->lastCodeStr, 0, sizeof (char) * 33);
	memset(ch->lastLearnedCodeStr, 0, sizeof (char) * 33);
	ch->lastCodeKnown = 0;
	ch->lastLearnedCodeKnown = 0;

	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1055_IR_100:
		break;
	case PHIDCHUID_1055_IR_200_USB:
		break;
	case PHIDCHUID_1055_IR_200_VINT:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetIR));
}

static PhidgetReturnCode CCONV
_create(PhidgetIRHandle *phidp) {

	CHANNELCREATE_BODY(IR, PHIDCHCLASS_IR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetIR_delete(PhidgetIRHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetIR_transmitRepeat(PhidgetIRHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_TRANSMITREPEAT, NULL, NULL, NULL);
}

API_PRETURN
PhidgetIR_setOnCodeHandler(PhidgetIRHandle ch, PhidgetIR_OnCodeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);

	ch->Code = fptr;
	ch->CodeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetIR_setOnLearnHandler(PhidgetIRHandle ch, PhidgetIR_OnLearnCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);

	ch->Learn = fptr;
	ch->LearnCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetIR_setOnRawDataHandler(PhidgetIRHandle ch, PhidgetIR_OnRawDataCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);

	ch->RawData = fptr;
	ch->RawDataCtx = ctx;

	return (EPHIDGET_OK);
}
