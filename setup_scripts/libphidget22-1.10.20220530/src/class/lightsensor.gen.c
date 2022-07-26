/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

static void CCONV PhidgetLightSensor_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetLightSensor_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetLightSensor_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetLightSensor_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetLightSensor_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetLightSensor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetLightSensor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetLightSensor_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetLightSensor_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetLightSensor {
	struct _PhidgetChannel phid;
	double dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double minDataRate;
	double maxDataRate;
	double illuminance;
	double minIlluminance;
	double maxIlluminance;
	double illuminanceChangeTrigger;
	double minIlluminanceChangeTrigger;
	double maxIlluminanceChangeTrigger;
	PhidgetLightSensor_OnIlluminanceChangeCallback IlluminanceChange;
	void *IlluminanceChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetLightSensorHandle ch;
	int version;

	ch = (PhidgetLightSensorHandle)phid;

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
		ch->illuminance = getBridgePacketDoubleByName(bp, "illuminance");
	if (version >= 0)
		ch->minIlluminance = getBridgePacketDoubleByName(bp, "minIlluminance");
	if (version >= 0)
		ch->maxIlluminance = getBridgePacketDoubleByName(bp, "maxIlluminance");
	if (version >= 0)
		ch->illuminanceChangeTrigger = getBridgePacketDoubleByName(bp, "illuminanceChangeTrigger");
	if (version >= 0)
		ch->minIlluminanceChangeTrigger = getBridgePacketDoubleByName(bp, "minIlluminanceChangeTrigger");
	if (version >= 0)
		ch->maxIlluminanceChangeTrigger = getBridgePacketDoubleByName(bp, "maxIlluminanceChangeTrigger");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetLightSensorHandle ch;

	ch = (PhidgetLightSensorHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",dataIntervalDbl=%g"
	  ",minDataRate=%g"
	  ",maxDataRate=%g"
	  ",illuminance=%g"
	  ",minIlluminance=%g"
	  ",maxIlluminance=%g"
	  ",illuminanceChangeTrigger=%g"
	  ",minIlluminanceChangeTrigger=%g"
	  ",maxIlluminanceChangeTrigger=%g"
	  ,1 /* class version */
	  ,(uint32_t)round(ch->dataInterval)
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->dataInterval
	  ,ch->minDataRate
	  ,ch->maxDataRate
	  ,ch->illuminance
	  ,ch->minIlluminance
	  ,ch->maxIlluminance
	  ,ch->illuminanceChangeTrigger
	  ,ch->minIlluminanceChangeTrigger
	  ,ch->maxIlluminanceChangeTrigger
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetLightSensorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetLightSensorHandle)phid;
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
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minIlluminanceChangeTrigger,
		  ch->maxIlluminanceChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->illuminanceChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "IlluminanceChangeTrigger");
		}
		break;
	case BP_ILLUMINANCECHANGE:
		ch->illuminance = getBridgePacketDouble(bp, 0);
		FIRECH(ch, IlluminanceChange, ch->illuminance);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetLightSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetLightSensorHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 8;
		ch->maxIlluminance = 131072;
		ch->maxIlluminanceChangeTrigger = 131072;
		ch->minDataInterval = 125;
		ch->minIlluminance = 0;
		ch->minIlluminanceChangeTrigger = 0;
		ch->illuminance = PUNK_DBL;
		ch->illuminanceChangeTrigger = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetLightSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetLightSensorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->illuminanceChangeTrigger);
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
	PhidgetLightSensorHandle ch;

	ch = (PhidgetLightSensorHandle)phid;

	if(ch->illuminance != PUNK_DBL)
		FIRECH(ch, IlluminanceChange, ch->illuminance);

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetLightSensorHandle ch;

	ch = (PhidgetLightSensorHandle)phid;

	if(ch->illuminance == PUNK_DBL)
		return (PFALSE);

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetLightSensor));
}

static PhidgetReturnCode CCONV
_create(PhidgetLightSensorHandle *phidp) {

	CHANNELCREATE_BODY(LightSensor, PHIDCHCLASS_LIGHTSENSOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_delete(PhidgetLightSensorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetLightSensor_setDataInterval(PhidgetLightSensorHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetLightSensor_getDataInterval(PhidgetLightSensorHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*dataInterval = (uint32_t)round(ch->dataInterval);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMinDataInterval(PhidgetLightSensorHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMaxDataInterval(PhidgetLightSensorHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_setDataRate(PhidgetLightSensorHandle ch, double dataRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u%g",
	  (uint32_t)round(1000.0 / dataRate), (double)(1000.0 / dataRate));
}

API_PRETURN
PhidgetLightSensor_getDataRate(PhidgetLightSensorHandle ch, double *dataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*dataRate = round_double((1000.0 / ch->dataInterval), 4);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMinDataRate(PhidgetLightSensorHandle ch, double *minDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*minDataRate = ch->minDataRate;
	if (ch->minDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMaxDataRate(PhidgetLightSensorHandle ch, double *maxDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*maxDataRate = ch->maxDataRate;
	if (ch->maxDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getIlluminance(PhidgetLightSensorHandle ch, double *illuminance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(illuminance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*illuminance = ch->illuminance;
	if (ch->illuminance == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMinIlluminance(PhidgetLightSensorHandle ch, double *minIlluminance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minIlluminance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*minIlluminance = ch->minIlluminance;
	if (ch->minIlluminance == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMaxIlluminance(PhidgetLightSensorHandle ch, double *maxIlluminance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxIlluminance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*maxIlluminance = ch->maxIlluminance;
	if (ch->maxIlluminance == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_setIlluminanceChangeTrigger(PhidgetLightSensorHandle ch,
  double illuminanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  illuminanceChangeTrigger));
}

API_PRETURN
PhidgetLightSensor_getIlluminanceChangeTrigger(PhidgetLightSensorHandle ch,
  double *illuminanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(illuminanceChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*illuminanceChangeTrigger = ch->illuminanceChangeTrigger;
	if (ch->illuminanceChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMinIlluminanceChangeTrigger(PhidgetLightSensorHandle ch,
  double *minIlluminanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minIlluminanceChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*minIlluminanceChangeTrigger = ch->minIlluminanceChangeTrigger;
	if (ch->minIlluminanceChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMaxIlluminanceChangeTrigger(PhidgetLightSensorHandle ch,
  double *maxIlluminanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxIlluminanceChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHED_PR(ch);

	*maxIlluminanceChangeTrigger = ch->maxIlluminanceChangeTrigger;
	if (ch->maxIlluminanceChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_setOnIlluminanceChangeHandler(PhidgetLightSensorHandle ch,
  PhidgetLightSensor_OnIlluminanceChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_LIGHTSENSOR);

	ch->IlluminanceChange = fptr;
	ch->IlluminanceChangeCtx = ctx;

	return (EPHIDGET_OK);
}
