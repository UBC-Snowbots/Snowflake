/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

#include "device/genericdevice.h"
static void CCONV PhidgetGeneric_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetGeneric_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetGeneric_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetGeneric_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetGeneric_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetGeneric_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetGeneric_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetGeneric_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetGeneric_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetGeneric {
	struct _PhidgetChannel phid;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	int version;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 1) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 1 - functionality may be limited.", phid, version);
	}


	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ,1 /* class version */
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = EPHIDGET_OK;

	switch (bp->vpkt) {
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
	case PHIDCHUID_USB_UNKNOWN:
		break;
	case PHIDCHUID_VINT_UNKNOWN:
		break;
	case PHIDCHUID_SPI_UNKNOWN:
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
	case PHIDCHUID_USB_UNKNOWN:
		break;
	case PHIDCHUID_VINT_UNKNOWN:
		break;
	case PHIDCHUID_SPI_UNKNOWN:
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

	mos_free(*ch, sizeof (struct _PhidgetGeneric));
}

static PhidgetReturnCode CCONV
_create(PhidgetGenericHandle *phidp) {

	CHANNELCREATE_BODY(Generic, PHIDCHCLASS_GENERIC);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGeneric_delete(PhidgetGenericHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}
