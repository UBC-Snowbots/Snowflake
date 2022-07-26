/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

static void CCONV PhidgetHumiditySensor_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static void CCONV PhidgetHumiditySensor_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetHumiditySensor_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetHumiditySensor_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetHumiditySensor_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetHumiditySensor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetHumiditySensor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetHumiditySensor_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetHumiditySensor_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetHumiditySensor {
	struct _PhidgetChannel phid;
	double dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double minDataRate;
	double maxDataRate;
	double humidity;
	double minHumidity;
	double maxHumidity;
	double humidityChangeTrigger;
	double minHumidityChangeTrigger;
	double maxHumidityChangeTrigger;
	PhidgetHumiditySensor_OnHumidityChangeCallback HumidityChange;
	void *HumidityChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetHumiditySensorHandle ch;
	int version;

	ch = (PhidgetHumiditySensorHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 1) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 1 - functionality may be limited.", phid, version);
	}

	if (version >= 0)
		ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	if (version >= 0)
		ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	if (version >= 1)
		ch->dataInterval = getBridgePacketDoubleByName(bp, "dataIntervalDbl");
	else if(version >= 0)
		ch->dataInterval = (double)getBridgePacketUInt32ByName(bp, "dataInterval");
	if (version >= 1)
		ch->minDataRate = getBridgePacketDoubleByName(bp, "minDataRate");
	else if(version >= 0)
		ch->minDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "maxDataInterval"));
	if (version >= 1)
		ch->maxDataRate = getBridgePacketDoubleByName(bp, "maxDataRate");
	else if(version >= 0)
		ch->maxDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "minDataInterval"));
	if (version >= 0)
		ch->humidity = getBridgePacketDoubleByName(bp, "humidity");
	if (version >= 0)
		ch->minHumidity = getBridgePacketDoubleByName(bp, "minHumidity");
	if (version >= 0)
		ch->maxHumidity = getBridgePacketDoubleByName(bp, "maxHumidity");
	if (version >= 0)
		ch->humidityChangeTrigger = getBridgePacketDoubleByName(bp, "humidityChangeTrigger");
	if (version >= 0)
		ch->minHumidityChangeTrigger = getBridgePacketDoubleByName(bp, "minHumidityChangeTrigger");
	if (version >= 0)
		ch->maxHumidityChangeTrigger = getBridgePacketDoubleByName(bp, "maxHumidityChangeTrigger");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetHumiditySensorHandle ch;

	ch = (PhidgetHumiditySensorHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",dataIntervalDbl=%g"
	  ",minDataRate=%g"
	  ",maxDataRate=%g"
	  ",humidity=%g"
	  ",minHumidity=%g"
	  ",maxHumidity=%g"
	  ",humidityChangeTrigger=%g"
	  ",minHumidityChangeTrigger=%g"
	  ",maxHumidityChangeTrigger=%g"
	  ,1 /* class version */
	  ,(uint32_t)round(ch->dataInterval)
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->dataInterval
	  ,ch->minDataRate
	  ,ch->maxDataRate
	  ,ch->humidity
	  ,ch->minHumidity
	  ,ch->maxHumidity
	  ,ch->humidityChangeTrigger
	  ,ch->minHumidityChangeTrigger
	  ,ch->maxHumidityChangeTrigger
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetHumiditySensorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetHumiditySensorHandle)phid;
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
	case BP_SETCHANGETRIGGER:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minHumidityChangeTrigger,
		  ch->maxHumidityChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->humidityChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "HumidityChangeTrigger");
		}
		break;
	case BP_HUMIDITYCHANGE:
		ch->humidity = getBridgePacketDouble(bp, 0);
		FIRECH(ch, HumidityChange, ch->humidity);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetHumiditySensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetHumiditySensorHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		ch->dataInterval = 500;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 2;
		ch->maxHumidity = 100;
		ch->maxHumidityChangeTrigger = 100;
		ch->minDataInterval = 500;
		ch->minHumidity = 0;
		ch->minHumidityChangeTrigger = 0;
		ch->humidity = PUNK_DBL;
		ch->humidityChangeTrigger = 0;
		break;
	case PHIDCHUID_HUM1001_HUMIDITYSENSOR_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 20;
		ch->maxHumidity = 100;
		ch->maxHumidityChangeTrigger = 100;
		ch->minDataInterval = 50;
		ch->minHumidity = 0;
		ch->minHumidityChangeTrigger = 0;
		ch->humidity = PUNK_DBL;
		ch->humidityChangeTrigger = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetHumiditySensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetHumiditySensorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->humidityChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_HUM1001_HUMIDITYSENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->humidityChangeTrigger);
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
	PhidgetHumiditySensorHandle ch;

	ch = (PhidgetHumiditySensorHandle)phid;

	if(ch->humidity != PUNK_DBL)
		FIRECH(ch, HumidityChange, ch->humidity);

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetHumiditySensorHandle ch;

	ch = (PhidgetHumiditySensorHandle)phid;

	if(ch->humidity == PUNK_DBL)
		return (PFALSE);

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetHumiditySensor));
}

static PhidgetReturnCode CCONV
_create(PhidgetHumiditySensorHandle *phidp) {

	CHANNELCREATE_BODY(HumiditySensor, PHIDCHCLASS_HUMIDITYSENSOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_delete(PhidgetHumiditySensorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetHumiditySensor_setDataInterval(PhidgetHumiditySensorHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetHumiditySensor_getDataInterval(PhidgetHumiditySensorHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*dataInterval = (uint32_t)round(ch->dataInterval);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMinDataInterval(PhidgetHumiditySensorHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMaxDataInterval(PhidgetHumiditySensorHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_setDataRate(PhidgetHumiditySensorHandle ch, double dataRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u%g",
	  (uint32_t)round(1000.0 / dataRate), (double)(1000.0 / dataRate));
}

API_PRETURN
PhidgetHumiditySensor_getDataRate(PhidgetHumiditySensorHandle ch, double *dataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*dataRate = round_double((1000.0 / ch->dataInterval), 4);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMinDataRate(PhidgetHumiditySensorHandle ch, double *minDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*minDataRate = ch->minDataRate;
	if (ch->minDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMaxDataRate(PhidgetHumiditySensorHandle ch, double *maxDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*maxDataRate = ch->maxDataRate;
	if (ch->maxDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getHumidity(PhidgetHumiditySensorHandle ch, double *humidity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(humidity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*humidity = ch->humidity;
	if (ch->humidity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMinHumidity(PhidgetHumiditySensorHandle ch, double *minHumidity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minHumidity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*minHumidity = ch->minHumidity;
	if (ch->minHumidity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMaxHumidity(PhidgetHumiditySensorHandle ch, double *maxHumidity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxHumidity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*maxHumidity = ch->maxHumidity;
	if (ch->maxHumidity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_setHumidityChangeTrigger(PhidgetHumiditySensorHandle ch,
  double humidityChangeTrigger) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  humidityChangeTrigger));
}

API_PRETURN
PhidgetHumiditySensor_getHumidityChangeTrigger(PhidgetHumiditySensorHandle ch,
  double *humidityChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(humidityChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*humidityChangeTrigger = ch->humidityChangeTrigger;
	if (ch->humidityChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMinHumidityChangeTrigger(PhidgetHumiditySensorHandle ch,
  double *minHumidityChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minHumidityChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*minHumidityChangeTrigger = ch->minHumidityChangeTrigger;
	if (ch->minHumidityChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMaxHumidityChangeTrigger(PhidgetHumiditySensorHandle ch,
  double *maxHumidityChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxHumidityChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHED_PR(ch);

	*maxHumidityChangeTrigger = ch->maxHumidityChangeTrigger;
	if (ch->maxHumidityChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_setOnHumidityChangeHandler(PhidgetHumiditySensorHandle ch,
  PhidgetHumiditySensor_OnHumidityChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUMIDITYSENSOR);

	ch->HumidityChange = fptr;
	ch->HumidityChangeCtx = ctx;

	return (EPHIDGET_OK);
}
