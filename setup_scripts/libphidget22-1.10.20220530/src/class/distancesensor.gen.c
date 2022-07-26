/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

static void CCONV PhidgetDistanceSensor_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static void CCONV PhidgetDistanceSensor_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetDistanceSensor_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDistanceSensor_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDistanceSensor_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetDistanceSensor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetDistanceSensor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetDistanceSensor_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetDistanceSensor_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetDistanceSensor {
	struct _PhidgetChannel phid;
	uint32_t amplitudes[8];
	uint32_t distances[8];
	uint32_t count;
	double dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double minDataRate;
	double maxDataRate;
	uint32_t distance;
	uint32_t minDistance;
	uint32_t maxDistance;
	uint32_t distanceChangeTrigger;
	uint32_t minDistanceChangeTrigger;
	uint32_t maxDistanceChangeTrigger;
	int sonarQuietMode;
	PhidgetDistanceSensor_OnDistanceChangeCallback DistanceChange;
	void *DistanceChangeCtx;
	PhidgetDistanceSensor_OnSonarReflectionsUpdateCallback SonarReflectionsUpdate;
	void *SonarReflectionsUpdateCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDistanceSensorHandle ch;
	int version;

	ch = (PhidgetDistanceSensorHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 2) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 2 - functionality may be limited.", phid, version);
	}

	if(version >= 1)
		memcpy(&ch->amplitudes, getBridgePacketUInt32ArrayByName(bp, "amplitudes"),
	  sizeof (uint32_t) * 8);
	if(version >= 1)
		memcpy(&ch->distances, getBridgePacketUInt32ArrayByName(bp, "distances"), sizeof (uint32_t) * 8);
	if(version >= 0)
		ch->count = getBridgePacketUInt32ByName(bp, "count");
	if (version >= 0)
		ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	if (version >= 0)
		ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	if (version >= 2)
		ch->dataInterval = getBridgePacketDoubleByName(bp, "dataIntervalDbl");
	else if(version >= 0)
		ch->dataInterval = (double)getBridgePacketUInt32ByName(bp, "dataInterval");
	if (version >= 2)
		ch->minDataRate = getBridgePacketDoubleByName(bp, "minDataRate");
	else if(version >= 0)
		ch->minDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "maxDataInterval"));
	if (version >= 2)
		ch->maxDataRate = getBridgePacketDoubleByName(bp, "maxDataRate");
	else if(version >= 0)
		ch->maxDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "minDataInterval"));
	if (version >= 0)
		ch->distance = getBridgePacketUInt32ByName(bp, "distance");
	if (version >= 0)
		ch->minDistance = getBridgePacketUInt32ByName(bp, "minDistance");
	if (version >= 0)
		ch->maxDistance = getBridgePacketUInt32ByName(bp, "maxDistance");
	if (version >= 0)
		ch->distanceChangeTrigger = getBridgePacketUInt32ByName(bp, "distanceChangeTrigger");
	if (version >= 0)
		ch->minDistanceChangeTrigger = getBridgePacketUInt32ByName(bp, "minDistanceChangeTrigger");
	if (version >= 0)
		ch->maxDistanceChangeTrigger = getBridgePacketUInt32ByName(bp, "maxDistanceChangeTrigger");
	if (version >= 0)
		ch->sonarQuietMode = getBridgePacketInt32ByName(bp, "sonarQuietMode");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetDistanceSensorHandle ch;

	ch = (PhidgetDistanceSensorHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",amplitudes=%8U"
	  ",distances=%8U"
	  ",count=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",dataIntervalDbl=%g"
	  ",minDataRate=%g"
	  ",maxDataRate=%g"
	  ",distance=%u"
	  ",minDistance=%u"
	  ",maxDistance=%u"
	  ",distanceChangeTrigger=%u"
	  ",minDistanceChangeTrigger=%u"
	  ",maxDistanceChangeTrigger=%u"
	  ",sonarQuietMode=%d"
	  ,2 /* class version */
	  ,ch->amplitudes
	  ,ch->distances
	  ,ch->count
	  ,(uint32_t)round(ch->dataInterval)
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->dataInterval
	  ,ch->minDataRate
	  ,ch->maxDataRate
	  ,ch->distance
	  ,ch->minDistance
	  ,ch->maxDistance
	  ,ch->distanceChangeTrigger
	  ,ch->minDistanceChangeTrigger
	  ,ch->maxDistanceChangeTrigger
	  ,ch->sonarQuietMode
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDistanceSensorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDistanceSensorHandle)phid;
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
		TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minDistanceChangeTrigger,
		  ch->maxDistanceChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->distanceChangeTrigger = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "DistanceChangeTrigger");
		}
		break;
	case BP_SETSONARQUIETMODE:
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->sonarQuietMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "SonarQuietMode");
		}
		break;
	case BP_DISTANCECHANGE:
		ch->distance = getBridgePacketUInt32(bp, 0);
		FIRECH(ch, DistanceChange, ch->distance);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetDistanceSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetDistanceSensorHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_DST1000_DISTANCESENSOR_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->maxDistance = 200;
		ch->maxDistanceChangeTrigger = 200;
		ch->minDataInterval = 100;
		ch->minDistance = 0;
		ch->minDistanceChangeTrigger = 0;
		ch->distance = PUNK_UINT32;
		ch->distanceChangeTrigger = 0;
		break;
	case PHIDCHUID_DST1001_DISTANCESENSOR_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 33.333333333333336;
		ch->maxDistance = 650;
		ch->maxDistanceChangeTrigger = 650;
		ch->minDataInterval = 30;
		ch->minDistance = 0;
		ch->minDistanceChangeTrigger = 0;
		ch->distance = PUNK_UINT32;
		ch->distanceChangeTrigger = 0;
		break;
	case PHIDCHUID_DST1002_DISTANCESENSOR_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 33.333333333333336;
		ch->maxDistance = 1300;
		ch->maxDistanceChangeTrigger = 1300;
		ch->minDataInterval = 30;
		ch->minDistance = 0;
		ch->minDistanceChangeTrigger = 0;
		ch->distance = PUNK_UINT32;
		ch->distanceChangeTrigger = 0;
		break;
	case PHIDCHUID_DST1200_DISTANCESENSOR_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->maxDistance = 10000;
		ch->maxDistanceChangeTrigger = 10000;
		ch->minDataInterval = 100;
		ch->minDistance = 40;
		ch->minDistanceChangeTrigger = 0;
		ch->distance = PUNK_UINT32;
		ch->distanceChangeTrigger = 0;
		ch->sonarQuietMode = 1;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	memset(ch->amplitudes, 0, sizeof (uint32_t) * 8);
	memset(ch->distances, 0, sizeof (uint32_t) * 8);
	ch->count = 0;

	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetDistanceSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetDistanceSensorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_DST1000_DISTANCESENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->distanceChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DST1001_DISTANCESENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->distanceChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DST1002_DISTANCESENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->distanceChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DST1200_DISTANCESENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
		  ch->distanceChangeTrigger);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSONARQUIETMODE, NULL, NULL, "%d", ch->sonarQuietMode);
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
	PhidgetDistanceSensorHandle ch;

	ch = (PhidgetDistanceSensorHandle)phid;

	if(ch->distance != PUNK_UINT32)
		FIRECH(ch, DistanceChange, ch->distance);

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetDistanceSensorHandle ch;

	ch = (PhidgetDistanceSensorHandle)phid;

	if(ch->distance == PUNK_UINT32)
		return (PFALSE);

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetDistanceSensor));
}

static PhidgetReturnCode CCONV
_create(PhidgetDistanceSensorHandle *phidp) {

	CHANNELCREATE_BODY(DistanceSensor, PHIDCHCLASS_DISTANCESENSOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_delete(PhidgetDistanceSensorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetDistanceSensor_setDataInterval(PhidgetDistanceSensorHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetDistanceSensor_getDataInterval(PhidgetDistanceSensorHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*dataInterval = (uint32_t)round(ch->dataInterval);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_getMinDataInterval(PhidgetDistanceSensorHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_getMaxDataInterval(PhidgetDistanceSensorHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_setDataRate(PhidgetDistanceSensorHandle ch, double dataRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u%g",
	  (uint32_t)round(1000.0 / dataRate), (double)(1000.0 / dataRate));
}

API_PRETURN
PhidgetDistanceSensor_getDataRate(PhidgetDistanceSensorHandle ch, double *dataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*dataRate = round_double((1000.0 / ch->dataInterval), 4);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_getMinDataRate(PhidgetDistanceSensorHandle ch, double *minDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*minDataRate = ch->minDataRate;
	if (ch->minDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_getMaxDataRate(PhidgetDistanceSensorHandle ch, double *maxDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*maxDataRate = ch->maxDataRate;
	if (ch->maxDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_getDistance(PhidgetDistanceSensorHandle ch, uint32_t *distance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(distance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*distance = ch->distance;
	if (ch->distance == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_getMinDistance(PhidgetDistanceSensorHandle ch, uint32_t *minDistance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDistance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*minDistance = ch->minDistance;
	if (ch->minDistance == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_getMaxDistance(PhidgetDistanceSensorHandle ch, uint32_t *maxDistance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDistance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*maxDistance = ch->maxDistance;
	if (ch->maxDistance == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_setDistanceChangeTrigger(PhidgetDistanceSensorHandle ch,
  uint32_t distanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%u",
	  distanceChangeTrigger));
}

API_PRETURN
PhidgetDistanceSensor_getDistanceChangeTrigger(PhidgetDistanceSensorHandle ch,
  uint32_t *distanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(distanceChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*distanceChangeTrigger = ch->distanceChangeTrigger;
	if (ch->distanceChangeTrigger == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_getMinDistanceChangeTrigger(PhidgetDistanceSensorHandle ch,
  uint32_t *minDistanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDistanceChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*minDistanceChangeTrigger = ch->minDistanceChangeTrigger;
	if (ch->minDistanceChangeTrigger == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_getMaxDistanceChangeTrigger(PhidgetDistanceSensorHandle ch,
  uint32_t *maxDistanceChangeTrigger) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDistanceChangeTrigger);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	*maxDistanceChangeTrigger = ch->maxDistanceChangeTrigger;
	if (ch->maxDistanceChangeTrigger == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_setSonarQuietMode(PhidgetDistanceSensorHandle ch, int sonarQuietMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSONARQUIETMODE, NULL, NULL, "%d",
	  sonarQuietMode));
}

API_PRETURN
PhidgetDistanceSensor_getSonarQuietMode(PhidgetDistanceSensorHandle ch, int *sonarQuietMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(sonarQuietMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DST1000_DISTANCESENSOR_100:
	case PHIDCHUID_DST1001_DISTANCESENSOR_100:
	case PHIDCHUID_DST1002_DISTANCESENSOR_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*sonarQuietMode = ch->sonarQuietMode;
	if (ch->sonarQuietMode == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_setOnDistanceChangeHandler(PhidgetDistanceSensorHandle ch,
  PhidgetDistanceSensor_OnDistanceChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);

	ch->DistanceChange = fptr;
	ch->DistanceChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDistanceSensor_setOnSonarReflectionsUpdateHandler(PhidgetDistanceSensorHandle ch,
  PhidgetDistanceSensor_OnSonarReflectionsUpdateCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);

	ch->SonarReflectionsUpdate = fptr;
	ch->SonarReflectionsUpdateCtx = ctx;

	return (EPHIDGET_OK);
}
