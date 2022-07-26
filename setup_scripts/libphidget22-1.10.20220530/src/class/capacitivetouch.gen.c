/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

#include "device/interfacekitdevice.h"
static void CCONV PhidgetCapacitiveTouch_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static void CCONV PhidgetCapacitiveTouch_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetCapacitiveTouch_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetCapacitiveTouch_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetCapacitiveTouch_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetCapacitiveTouch_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetCapacitiveTouch_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetCapacitiveTouch_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetCapacitiveTouch_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetCapacitiveTouch {
	struct _PhidgetChannel phid;
	double dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double minDataRate;
	double maxDataRate;
	double sensitivity;
	double minSensitivity;
	double maxSensitivity;
	int isTouched;
	double touchValue;
	double minTouchValue;
	double maxTouchValue;
	double touchValueChangeTrigger;
	double minTouchValueChangeTrigger;
	double maxTouchValueChangeTrigger;
	PhidgetCapacitiveTouch_OnTouchCallback Touch;
	void *TouchCtx;
	PhidgetCapacitiveTouch_OnTouchEndCallback TouchEnd;
	void *TouchEndCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetCapacitiveTouchHandle ch;
	int version;

	ch = (PhidgetCapacitiveTouchHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 3) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 3 - functionality may be limited.", phid, version);
	}

	if (version >= 1)
		ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	if (version >= 1)
		ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	if (version >= 3)
		ch->dataInterval = getBridgePacketDoubleByName(bp, "dataIntervalDbl");
	else if(version >= 1)
		ch->dataInterval = (double)getBridgePacketUInt32ByName(bp, "dataInterval");
	if (version >= 3)
		ch->minDataRate = getBridgePacketDoubleByName(bp, "minDataRate");
	else if(version >= 1)
		ch->minDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "maxDataInterval"));
	if (version >= 3)
		ch->maxDataRate = getBridgePacketDoubleByName(bp, "maxDataRate");
	else if(version >= 1)
		ch->maxDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "minDataInterval"));
	if (version >= 1)
		ch->sensitivity = getBridgePacketDoubleByName(bp, "sensitivity");
	if (version >= 1)
		ch->minSensitivity = getBridgePacketDoubleByName(bp, "minSensitivity");
	if (version >= 1)
		ch->maxSensitivity = getBridgePacketDoubleByName(bp, "maxSensitivity");
	if (version >= 2)
		ch->isTouched = getBridgePacketInt32ByName(bp, "isTouched");
	if (version >= 2)
		ch->touchValue = getBridgePacketDoubleByName(bp, "touchValue");
	if (version >= 2)
		ch->minTouchValue = getBridgePacketDoubleByName(bp, "minTouchValue");
	if (version >= 2)
		ch->maxTouchValue = getBridgePacketDoubleByName(bp, "maxTouchValue");
	if (version >= 2)
		ch->touchValueChangeTrigger = getBridgePacketDoubleByName(bp, "touchValueChangeTrigger");
	if (version >= 2)
		ch->minTouchValueChangeTrigger = getBridgePacketDoubleByName(bp, "minTouchValueChangeTrigger");
	if (version >= 2)
		ch->maxTouchValueChangeTrigger = getBridgePacketDoubleByName(bp, "maxTouchValueChangeTrigger");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetCapacitiveTouchHandle ch;

	ch = (PhidgetCapacitiveTouchHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",dataIntervalDbl=%g"
	  ",minDataRate=%g"
	  ",maxDataRate=%g"
	  ",sensitivity=%g"
	  ",minSensitivity=%g"
	  ",maxSensitivity=%g"
	  ",isTouched=%d"
	  ",touchValue=%g"
	  ",minTouchValue=%g"
	  ",maxTouchValue=%g"
	  ",touchValueChangeTrigger=%g"
	  ",minTouchValueChangeTrigger=%g"
	  ",maxTouchValueChangeTrigger=%g"
	  ,3 /* class version */
	  ,(uint32_t)round(ch->dataInterval)
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->dataInterval
	  ,ch->minDataRate
	  ,ch->maxDataRate
	  ,ch->sensitivity
	  ,ch->minSensitivity
	  ,ch->maxSensitivity
	  ,ch->isTouched
	  ,ch->touchValue
	  ,ch->minTouchValue
	  ,ch->maxTouchValue
	  ,ch->touchValueChangeTrigger
	  ,ch->minTouchValueChangeTrigger
	  ,ch->maxTouchValueChangeTrigger
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetCapacitiveTouchHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetCapacitiveTouchHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETDATAINTERVAL:
		if (bp->entrycnt > 1)
			TESTRANGE_IOP(bp->iop, "%lf", round_double((1000.0 / getBridgePacketDouble(bp, 1)), 4),
			  ch->minDataRate, ch->maxDataRate);
		else
			TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minDataInterval,
			  ch->maxDataInterval);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		if (bp->entrycnt > 1)
			ch->dataInterval = getBridgePacketDouble(bp, 1);
		else
			ch->dataInterval = (double)getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "DataInterval");
			FIRE_PROPERTYCHANGE(ch, "DataRate");
		}
		break;
	case BP_SETSENSITIVITY:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minSensitivity,
		  ch->maxSensitivity);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->sensitivity = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Sensitivity");
		}
		break;
	case BP_SETCHANGETRIGGER:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minTouchValueChangeTrigger,
		  ch->maxTouchValueChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->touchValueChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "TouchValueChangeTrigger");
		}
		break;
	case BP_TOUCHINPUTVALUECHANGE:
		ch->touchValue = getBridgePacketDouble(bp, 0);
		FIRECH(ch, Touch, ch->touchValue);
		break;
	case BP_TOUCHINPUTEND:
		FIRECH0(ch, TouchEnd);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetInterfaceKitDeviceHandle parentInterfaceKit;
	PhidgetCapacitiveTouchHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetCapacitiveTouchHandle)phid;

	ret = EPHIDGET_OK;

	parentInterfaceKit = (PhidgetInterfaceKitDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1015_CAPACITIVETOUCH_000:
		ch->touchValueChangeTrigger = 0.005;
		ch->minTouchValueChangeTrigger = 0;
		ch->maxTouchValueChangeTrigger = 1;
		ch->dataInterval = 60;
		ch->minDataInterval = 60;
		ch->maxDataInterval = 1000;
		ch->minDataRate = 1;
		ch->maxDataRate = 16.666666666666668;
		ch->maxTouchValue = 1;
		ch->minTouchValue = 0;
		ch->touchValue = parentInterfaceKit->touchValue[ch->phid.index];
		ch->isTouched = parentInterfaceKit->isTouched[ch->phid.index];
		break;
	case PHIDCHUID_1016_CAPACITIVETOUCH_000:
		ch->touchValueChangeTrigger = 0.005;
		ch->minTouchValueChangeTrigger = 0;
		ch->maxTouchValueChangeTrigger = 1;
		ch->dataInterval = 60;
		ch->minDataInterval = 60;
		ch->maxDataInterval = 1000;
		ch->minDataRate = 1;
		ch->maxDataRate = 16.666666666666668;
		ch->maxTouchValue = 1;
		ch->minTouchValue = 0;
		ch->touchValue = parentInterfaceKit->touchValue[ch->phid.index];
		ch->isTouched = parentInterfaceKit->isTouched[ch->phid.index];
		break;
	case PHIDCHUID_HIN1000_CAPACITIVETOUCH_100:
		ch->touchValueChangeTrigger = 0;
		ch->minTouchValueChangeTrigger = 0;
		ch->maxTouchValueChangeTrigger = 1;
		ch->dataInterval = 25;
		ch->minDataInterval = 25;
		ch->maxDataInterval = 1000;
		ch->minDataRate = 1;
		ch->maxDataRate = 40;
		ch->sensitivity = 0.2;
		ch->minSensitivity = 0;
		ch->maxSensitivity = 1;
		ch->maxTouchValue = 1;
		ch->minTouchValue = 0;
		ch->touchValue = PUNK_DBL;
		ch->isTouched = PUNK_BOOL;
		break;
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_BUTTONS_100:
		ch->touchValueChangeTrigger = 0;
		ch->minTouchValueChangeTrigger = 0;
		ch->maxTouchValueChangeTrigger = 0.5;
		ch->dataInterval = 20;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 250;
		ch->minDataRate = 4;
		ch->maxDataRate = 50;
		ch->sensitivity = 0.5;
		ch->minSensitivity = 0;
		ch->maxSensitivity = 1;
		ch->maxTouchValue = 1;
		ch->minTouchValue = 0;
		ch->touchValue = PUNK_DBL;
		ch->isTouched = PUNK_BOOL;
		break;
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_WHEEL_100:
		ch->touchValueChangeTrigger = 0;
		ch->minTouchValueChangeTrigger = 0;
		ch->maxTouchValueChangeTrigger = 0.5;
		ch->dataInterval = 20;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 250;
		ch->minDataRate = 4;
		ch->maxDataRate = 50;
		ch->sensitivity = 0.7;
		ch->minSensitivity = 0;
		ch->maxSensitivity = 1;
		ch->maxTouchValue = 1;
		ch->minTouchValue = 0;
		ch->touchValue = PUNK_DBL;
		ch->isTouched = PUNK_BOOL;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetCapacitiveTouchHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetCapacitiveTouchHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1015_CAPACITIVETOUCH_000:
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->touchValueChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1016_CAPACITIVETOUCH_000:
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->touchValueChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_HIN1000_CAPACITIVETOUCH_100:
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->touchValueChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSENSITIVITY, NULL, NULL, "%g", ch->sensitivity);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_BUTTONS_100:
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->touchValueChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSENSITIVITY, NULL, NULL, "%g", ch->sensitivity);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_WHEEL_100:
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->touchValueChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSENSITIVITY, NULL, NULL, "%g", ch->sensitivity);
		if (ret != EPHIDGET_OK)
			break;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {
	PhidgetCapacitiveTouchHandle ch;

	ch = (PhidgetCapacitiveTouchHandle)phid;

	if(ch->touchValue != PUNK_DBL)
		FIRECH(ch, Touch, ch->touchValue);

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetCapacitiveTouchHandle ch;

	ch = (PhidgetCapacitiveTouchHandle)phid;

	if(ch->touchValue == PUNK_DBL)
		return (PFALSE);

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetCapacitiveTouch));
}

static PhidgetReturnCode CCONV
_create(PhidgetCapacitiveTouchHandle *phidp) {

	CHANNELCREATE_BODY(CapacitiveTouch, PHIDCHCLASS_CAPACITIVETOUCH);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_delete(PhidgetCapacitiveTouchHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetCapacitiveTouch_setDataInterval(PhidgetCapacitiveTouchHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetCapacitiveTouch_getDataInterval(PhidgetCapacitiveTouchHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*dataInterval = (uint32_t)round(ch->dataInterval);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getMinDataInterval(PhidgetCapacitiveTouchHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getMaxDataInterval(PhidgetCapacitiveTouchHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_setDataRate(PhidgetCapacitiveTouchHandle ch, double dataRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u%g",
	  (uint32_t)round(1000.0 / dataRate), (double)(1000.0 / dataRate));
}

API_PRETURN
PhidgetCapacitiveTouch_getDataRate(PhidgetCapacitiveTouchHandle ch, double *dataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*dataRate = round_double((1000.0 / ch->dataInterval), 4);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getMinDataRate(PhidgetCapacitiveTouchHandle ch, double *minDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*minDataRate = ch->minDataRate;
	if (ch->minDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getMaxDataRate(PhidgetCapacitiveTouchHandle ch, double *maxDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*maxDataRate = ch->maxDataRate;
	if (ch->maxDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_setSensitivity(PhidgetCapacitiveTouchHandle ch, double sensitivity) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSENSITIVITY, NULL, NULL, "%g",
	  sensitivity));
}

API_PRETURN
PhidgetCapacitiveTouch_getSensitivity(PhidgetCapacitiveTouchHandle ch, double *sensitivity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(sensitivity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1015_CAPACITIVETOUCH_000:
	case PHIDCHUID_1016_CAPACITIVETOUCH_000:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*sensitivity = ch->sensitivity;
	if (ch->sensitivity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getMinSensitivity(PhidgetCapacitiveTouchHandle ch, double *minSensitivity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minSensitivity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1015_CAPACITIVETOUCH_000:
	case PHIDCHUID_1016_CAPACITIVETOUCH_000:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minSensitivity = ch->minSensitivity;
	if (ch->minSensitivity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getMaxSensitivity(PhidgetCapacitiveTouchHandle ch, double *maxSensitivity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxSensitivity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1015_CAPACITIVETOUCH_000:
	case PHIDCHUID_1016_CAPACITIVETOUCH_000:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxSensitivity = ch->maxSensitivity;
	if (ch->maxSensitivity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getIsTouched(PhidgetCapacitiveTouchHandle ch, int *isTouched) {

	TESTPTR_PR(ch);
	TESTPTR_PR(isTouched);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*isTouched = ch->isTouched;
	if (ch->isTouched == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getTouchValue(PhidgetCapacitiveTouchHandle ch, double *touchValue) {

	TESTPTR_PR(ch);
	TESTPTR_PR(touchValue);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*touchValue = ch->touchValue;
	if (ch->touchValue == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getMinTouchValue(PhidgetCapacitiveTouchHandle ch, double *minTouchValue) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minTouchValue);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*minTouchValue = ch->minTouchValue;
	if (ch->minTouchValue == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getMaxTouchValue(PhidgetCapacitiveTouchHandle ch, double *maxTouchValue) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxTouchValue);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*maxTouchValue = ch->maxTouchValue;
	if (ch->maxTouchValue == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_setTouchValueChangeTrigger(PhidgetCapacitiveTouchHandle ch,
  double touchValueChangeTrigger) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  touchValueChangeTrigger));
}

API_PRETURN
PhidgetCapacitiveTouch_getTouchValueChangeTrigger(PhidgetCapacitiveTouchHandle ch,
  double *touchValueChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(touchValueChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*touchValueChangeTrigger = ch->touchValueChangeTrigger;
	if (ch->touchValueChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getMinTouchValueChangeTrigger(PhidgetCapacitiveTouchHandle ch,
  double *minTouchValueChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minTouchValueChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*minTouchValueChangeTrigger = ch->minTouchValueChangeTrigger;
	if (ch->minTouchValueChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_getMaxTouchValueChangeTrigger(PhidgetCapacitiveTouchHandle ch,
  double *maxTouchValueChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxTouchValueChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);
	TESTATTACHED_PR(ch);

	*maxTouchValueChangeTrigger = ch->maxTouchValueChangeTrigger;
	if (ch->maxTouchValueChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_setOnTouchHandler(PhidgetCapacitiveTouchHandle ch,
  PhidgetCapacitiveTouch_OnTouchCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);

	ch->Touch = fptr;
	ch->TouchCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetCapacitiveTouch_setOnTouchEndHandler(PhidgetCapacitiveTouchHandle ch,
  PhidgetCapacitiveTouch_OnTouchEndCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_CAPACITIVETOUCH);

	ch->TouchEnd = fptr;
	ch->TouchEndCtx = ctx;

	return (EPHIDGET_OK);
}
