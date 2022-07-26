/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

#include "device/dictionarydevice.h"
static void CCONV PhidgetDictionary_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetDictionary_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetDictionary_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDictionary_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDictionary_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetDictionary_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetDictionary_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetDictionary_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetDictionary_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetDictionary {
	struct _PhidgetChannel phid;
	PhidgetDictionary_OnAddCallback Add;
	void *AddCtx;
	PhidgetDictionary_OnRemoveCallback Remove;
	void *RemoveCtx;
	PhidgetDictionary_OnUpdateCallback Update;
	void *UpdateCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	int version;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 0 - functionality may be limited.", phid, version);
	}


	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ,0 /* class version */
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDictionaryHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDictionaryHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_DICTIONARYADD:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DICTIONARYREMOVEALL:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DICTIONARYGET:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DICTIONARYREMOVE:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DICTIONARYSCAN:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DICTIONARYSET:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DICTIONARYUPDATE:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DICTIONARYADDED:
		FIRECH(ch, Add, getBridgePacketString(bp, 0), getBridgePacketString(bp, 1));
		break;
	case BP_DICTIONARYREMOVED:
		FIRECH(ch, Remove, getBridgePacketString(bp, 0));
		break;
	case BP_DICTIONARYUPDATED:
		FIRECH(ch, Update, getBridgePacketString(bp, 0), getBridgePacketString(bp, 1));
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_DICTIONARY:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_DICTIONARY:
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

	mos_free(*ch, sizeof (struct _PhidgetDictionary));
}

static PhidgetReturnCode CCONV
_create(PhidgetDictionaryHandle *phidp) {

	CHANNELCREATE_BODY(Dictionary, PHIDCHCLASS_DICTIONARY);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDictionary_delete(PhidgetDictionaryHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetDictionary_add(PhidgetDictionaryHandle ch, const char *key, const char *value) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DICTIONARY);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DICTIONARYADD, NULL, NULL, "%s%s", key,
	  value);
}

API_PRETURN
PhidgetDictionary_removeAll(PhidgetDictionaryHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DICTIONARY);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DICTIONARYREMOVEALL, NULL, NULL, NULL);
}

API_PRETURN
PhidgetDictionary_remove(PhidgetDictionaryHandle ch, const char *key) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DICTIONARY);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DICTIONARYREMOVE, NULL, NULL, "%s", key);
}

API_PRETURN
PhidgetDictionary_set(PhidgetDictionaryHandle ch, const char *key, const char *value) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DICTIONARY);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DICTIONARYSET, NULL, NULL, "%s%s", key,
	  value);
}

API_PRETURN
PhidgetDictionary_update(PhidgetDictionaryHandle ch, const char *key, const char *value) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DICTIONARY);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DICTIONARYUPDATE, NULL, NULL, "%s%s", key,
	  value);
}

API_PRETURN
PhidgetDictionary_setOnAddHandler(PhidgetDictionaryHandle ch, PhidgetDictionary_OnAddCallback fptr,
  void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DICTIONARY);

	ch->Add = fptr;
	ch->AddCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDictionary_setOnRemoveHandler(PhidgetDictionaryHandle ch, PhidgetDictionary_OnRemoveCallback fptr,
  void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DICTIONARY);

	ch->Remove = fptr;
	ch->RemoveCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDictionary_setOnUpdateHandler(PhidgetDictionaryHandle ch, PhidgetDictionary_OnUpdateCallback fptr,
  void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DICTIONARY);

	ch->Update = fptr;
	ch->UpdateCtx = ctx;

	return (EPHIDGET_OK);
}
