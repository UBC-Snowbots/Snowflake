/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

static void CCONV PhidgetResistanceInput_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static void CCONV PhidgetResistanceInput_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetResistanceInput_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetResistanceInput_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetResistanceInput_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetResistanceInput_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetResistanceInput_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetResistanceInput_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetResistanceInput_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetResistanceInput {
	struct _PhidgetChannel phid;
	double dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double minDataRate;
	double maxDataRate;
	double resistance;
	double minResistance;
	double maxResistance;
	double resistanceChangeTrigger;
	double minResistanceChangeTrigger;
	double maxResistanceChangeTrigger;
	Phidget_RTDWireSetup RTDWireSetup;
	PhidgetResistanceInput_OnResistanceChangeCallback ResistanceChange;
	void *ResistanceChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetResistanceInputHandle ch;
	int version;

	ch = (PhidgetResistanceInputHandle)phid;

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
		ch->resistance = getBridgePacketDoubleByName(bp, "resistance");
	if (version >= 0)
		ch->minResistance = getBridgePacketDoubleByName(bp, "minResistance");
	if (version >= 0)
		ch->maxResistance = getBridgePacketDoubleByName(bp, "maxResistance");
	if (version >= 0)
		ch->resistanceChangeTrigger = getBridgePacketDoubleByName(bp, "resistanceChangeTrigger");
	if (version >= 0)
		ch->minResistanceChangeTrigger = getBridgePacketDoubleByName(bp, "minResistanceChangeTrigger");
	if (version >= 0)
		ch->maxResistanceChangeTrigger = getBridgePacketDoubleByName(bp, "maxResistanceChangeTrigger");
	if (version >= 0)
		ch->RTDWireSetup = getBridgePacketInt32ByName(bp, "RTDWireSetup");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetResistanceInputHandle ch;

	ch = (PhidgetResistanceInputHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",dataIntervalDbl=%g"
	  ",minDataRate=%g"
	  ",maxDataRate=%g"
	  ",resistance=%g"
	  ",minResistance=%g"
	  ",maxResistance=%g"
	  ",resistanceChangeTrigger=%g"
	  ",minResistanceChangeTrigger=%g"
	  ",maxResistanceChangeTrigger=%g"
	  ",RTDWireSetup=%d"
	  ,1 /* class version */
	  ,(uint32_t)round(ch->dataInterval)
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->dataInterval
	  ,ch->minDataRate
	  ,ch->maxDataRate
	  ,ch->resistance
	  ,ch->minResistance
	  ,ch->maxResistance
	  ,ch->resistanceChangeTrigger
	  ,ch->minResistanceChangeTrigger
	  ,ch->maxResistanceChangeTrigger
	  ,ch->RTDWireSetup
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetResistanceInputHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetResistanceInputHandle)phid;
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
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minResistanceChangeTrigger,
		  ch->maxResistanceChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->resistanceChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "ResistanceChangeTrigger");
		}
		break;
	case BP_SETRTDWIRESETUP:
		if (!supportedRTDWireSetup(phid, (Phidget_RTDWireSetup)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified RTDWireSetup is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->RTDWireSetup = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "RTDWireSetup");
		}
		break;
	case BP_RESISTANCECHANGE:
		ch->resistance = getBridgePacketDouble(bp, 0);
		FIRECH(ch, ResistanceChange, ch->resistance);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetResistanceInputHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetResistanceInputHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 4;
		ch->maxResistance = 50000;
		ch->maxResistanceChangeTrigger = 19000;
		ch->minDataInterval = 250;
		ch->minResistance = 0;
		ch->minResistanceChangeTrigger = 0;
		ch->resistance = PUNK_DBL;
		ch->resistanceChangeTrigger = 0;
		ch->RTDWireSetup = RTD_WIRE_SETUP_4WIRE;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetResistanceInputHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetResistanceInputHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->resistanceChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETRTDWIRESETUP, NULL, NULL, "%d", ch->RTDWireSetup);
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
	PhidgetResistanceInputHandle ch;

	ch = (PhidgetResistanceInputHandle)phid;

	if(ch->resistance != PUNK_DBL)
		FIRECH(ch, ResistanceChange, ch->resistance);

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetResistanceInputHandle ch;

	ch = (PhidgetResistanceInputHandle)phid;

	if(ch->resistance == PUNK_DBL)
		return (PFALSE);

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetResistanceInput));
}

static PhidgetReturnCode CCONV
_create(PhidgetResistanceInputHandle *phidp) {

	CHANNELCREATE_BODY(ResistanceInput, PHIDCHCLASS_RESISTANCEINPUT);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_delete(PhidgetResistanceInputHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetResistanceInput_setDataInterval(PhidgetResistanceInputHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetResistanceInput_getDataInterval(PhidgetResistanceInputHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*dataInterval = (uint32_t)round(ch->dataInterval);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMinDataInterval(PhidgetResistanceInputHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMaxDataInterval(PhidgetResistanceInputHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_setDataRate(PhidgetResistanceInputHandle ch, double dataRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u%g",
	  (uint32_t)round(1000.0 / dataRate), (double)(1000.0 / dataRate));
}

API_PRETURN
PhidgetResistanceInput_getDataRate(PhidgetResistanceInputHandle ch, double *dataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*dataRate = round_double((1000.0 / ch->dataInterval), 4);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMinDataRate(PhidgetResistanceInputHandle ch, double *minDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*minDataRate = ch->minDataRate;
	if (ch->minDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMaxDataRate(PhidgetResistanceInputHandle ch, double *maxDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*maxDataRate = ch->maxDataRate;
	if (ch->maxDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getResistance(PhidgetResistanceInputHandle ch, double *resistance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(resistance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*resistance = ch->resistance;
	if (ch->resistance == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMinResistance(PhidgetResistanceInputHandle ch, double *minResistance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minResistance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*minResistance = ch->minResistance;
	if (ch->minResistance == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMaxResistance(PhidgetResistanceInputHandle ch, double *maxResistance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxResistance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*maxResistance = ch->maxResistance;
	if (ch->maxResistance == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_setResistanceChangeTrigger(PhidgetResistanceInputHandle ch,
  double resistanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  resistanceChangeTrigger));
}

API_PRETURN
PhidgetResistanceInput_getResistanceChangeTrigger(PhidgetResistanceInputHandle ch,
  double *resistanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(resistanceChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*resistanceChangeTrigger = ch->resistanceChangeTrigger;
	if (ch->resistanceChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMinResistanceChangeTrigger(PhidgetResistanceInputHandle ch,
  double *minResistanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minResistanceChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*minResistanceChangeTrigger = ch->minResistanceChangeTrigger;
	if (ch->minResistanceChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMaxResistanceChangeTrigger(PhidgetResistanceInputHandle ch,
  double *maxResistanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxResistanceChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*maxResistanceChangeTrigger = ch->maxResistanceChangeTrigger;
	if (ch->maxResistanceChangeTrigger == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_setRTDWireSetup(PhidgetResistanceInputHandle ch,
  Phidget_RTDWireSetup RTDWireSetup) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETRTDWIRESETUP, NULL, NULL, "%d",
	  RTDWireSetup));
}

API_PRETURN
PhidgetResistanceInput_getRTDWireSetup(PhidgetResistanceInputHandle ch,
  Phidget_RTDWireSetup *RTDWireSetup) {

	TESTPTR_PR(ch);
	TESTPTR_PR(RTDWireSetup);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHED_PR(ch);

	*RTDWireSetup = ch->RTDWireSetup;
	if (ch->RTDWireSetup == (Phidget_RTDWireSetup)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_setOnResistanceChangeHandler(PhidgetResistanceInputHandle ch,
  PhidgetResistanceInput_OnResistanceChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_RESISTANCEINPUT);

	ch->ResistanceChange = fptr;
	ch->ResistanceChangeCtx = ctx;

	return (EPHIDGET_OK);
}
