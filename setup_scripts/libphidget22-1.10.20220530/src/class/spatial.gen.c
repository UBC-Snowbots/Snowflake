/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

#include "device/spatialdevice.h"
static void CCONV PhidgetSpatial_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetSpatial_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetSpatial_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetSpatial_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetSpatial_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetSpatial_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetSpatial_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetSpatial_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetSpatial_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetSpatial {
	struct _PhidgetChannel phid;
	Phidget_SpatialAlgorithm algorithm;
	double algorithmMagnetometerGain;
	double dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double minDataRate;
	double maxDataRate;
	PhidgetSpatial_SpatialEulerAngles eulerAngles;
	uint8_t eulerAnglesValid;
	int heatingEnabled;
	Phidget_SpatialPrecision precision;
	PhidgetSpatial_SpatialQuaternion quaternion;
	uint8_t quaternionValid;
	PhidgetSpatial_OnAlgorithmDataCallback AlgorithmData;
	void *AlgorithmDataCtx;
	PhidgetSpatial_OnSpatialDataCallback SpatialData;
	void *SpatialDataCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetSpatialHandle ch;
	int version;

	ch = (PhidgetSpatialHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 6) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 6 - functionality may be limited.", phid, version);
	}

	if (version >= 3)
		ch->algorithm = getBridgePacketInt32ByName(bp, "algorithm");
	if (version >= 3)
		ch->algorithmMagnetometerGain = getBridgePacketDoubleByName(bp, "algorithmMagnetometerGain");
	if (version >= 0)
		ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	if (version >= 0)
		ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	if (version >= 6)
		ch->dataInterval = getBridgePacketDoubleByName(bp, "dataIntervalDbl");
	else if(version >= 0)
		ch->dataInterval = (double)getBridgePacketUInt32ByName(bp, "dataInterval");
	if (version >= 6)
		ch->minDataRate = getBridgePacketDoubleByName(bp, "minDataRate");
	else if(version >= 0)
		ch->minDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "maxDataInterval"));
	if (version >= 6)
		ch->maxDataRate = getBridgePacketDoubleByName(bp, "maxDataRate");
	else if(version >= 0)
		ch->maxDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "minDataInterval"));
	if (version >= 4)
		ch->heatingEnabled = getBridgePacketInt32ByName(bp, "heatingEnabled");
	if (version >= 2)
		ch->precision = getBridgePacketInt32ByName(bp, "precision");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetSpatialHandle ch;

	ch = (PhidgetSpatialHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",algorithm=%d"
	  ",algorithmMagnetometerGain=%g"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",dataIntervalDbl=%g"
	  ",minDataRate=%g"
	  ",maxDataRate=%g"
	  ",heatingEnabled=%d"
	  ",precision=%d"
	  ,6 /* class version */
	  ,ch->algorithm
	  ,ch->algorithmMagnetometerGain
	  ,(uint32_t)round(ch->dataInterval)
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->dataInterval
	  ,ch->minDataRate
	  ,ch->maxDataRate
	  ,ch->heatingEnabled
	  ,ch->precision
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetSpatialHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetSpatialHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETAHRSPARAMETERS:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETCORRECTIONPARAMETERS:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_RESETCORRECTIONPARAMETERS:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SAVECORRECTIONPARAMETERS:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_ZEROSPATIALALGORITHM:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_ZERO:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETSPATIALALGORITHM:
		if (!supportedSpatialAlgorithm(phid, (Phidget_SpatialAlgorithm)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified SpatialAlgorithm is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->algorithm = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Algorithm");
		}
		break;
	case BP_SETSPATIALALGORITHMMAGGAIN:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->algorithmMagnetometerGain = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "AlgorithmMagnetometerGain");
		}
		break;
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
	case BP_SETHEATINGENABLED:
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->heatingEnabled = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "HeatingEnabled");
		}
		break;
	case BP_SETSPATIALPRECISION:
		if (!supportedSpatialPrecision(phid, (Phidget_SpatialPrecision)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified SpatialPrecision is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->precision = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Precision");
		}
		break;
	case BP_SPATIALDATA:
		FIRECH(ch, SpatialData, getBridgePacketDoubleArray(bp, 0), getBridgePacketDoubleArray(bp, 1),
		  getBridgePacketDoubleArray(bp, 2), getBridgePacketDouble(bp, 3));
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetSpatialHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetSpatialHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_1042_SPATIAL_300:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		ch->minDataRate = 1;
		ch->maxDataRate = 250;
		ch->precision = SPATIAL_PRECISION_LOW;
		ch->algorithm = SPATIAL_ALGORITHM_NONE;
		break;
	case PHIDCHUID_1044_SPATIAL_400:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		ch->minDataRate = 1;
		ch->maxDataRate = 250;
		ch->precision = SPATIAL_PRECISION_HYBRID;
		ch->algorithm = SPATIAL_ALGORITHM_NONE;
		break;
	case PHIDCHUID_1044_SPATIAL_500:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		ch->minDataRate = 1;
		ch->maxDataRate = 250;
		ch->precision = SPATIAL_PRECISION_HYBRID;
		ch->algorithm = SPATIAL_ALGORITHM_AHRS;
		ch->algorithmMagnetometerGain = 0.005;
		break;
	case PHIDCHUID_1044_SPATIAL_510:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		ch->minDataRate = 1;
		ch->maxDataRate = 250;
		ch->precision = SPATIAL_PRECISION_HYBRID;
		ch->algorithm = SPATIAL_ALGORITHM_AHRS;
		ch->algorithmMagnetometerGain = 0.005;
		break;
	case PHIDCHUID_1056_SPATIAL_000:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		ch->minDataRate = 1;
		ch->maxDataRate = 250;
		ch->precision = SPATIAL_PRECISION_HIGH;
		ch->algorithm = SPATIAL_ALGORITHM_NONE;
		break;
	case PHIDCHUID_1056_SPATIAL_200:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		ch->minDataRate = 1;
		ch->maxDataRate = 250;
		ch->precision = SPATIAL_PRECISION_HIGH;
		ch->algorithm = SPATIAL_ALGORITHM_NONE;
		break;
	case PHIDCHUID_MOT1101_SPATIAL_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 20;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 50;
		ch->precision = SPATIAL_PRECISION_LOW;
		ch->algorithm = SPATIAL_ALGORITHM_NONE;
		break;
	case PHIDCHUID_MOT1102_SPATIAL_200:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 20;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 50;
		ch->precision = SPATIAL_PRECISION_LOW;
		ch->algorithm = SPATIAL_ALGORITHM_AHRS;
		ch->algorithmMagnetometerGain = 0.005;
		break;
	case PHIDCHUID_MOT0109_SPATIAL_100:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		ch->minDataRate = 1;
		ch->maxDataRate = 250;
		ch->precision = SPATIAL_PRECISION_HYBRID;
		ch->algorithm = SPATIAL_ALGORITHM_AHRS;
		ch->algorithmMagnetometerGain = 0.005;
		ch->heatingEnabled = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetSpatialHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetSpatialHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1042_SPATIAL_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1044_SPATIAL_400:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1044_SPATIAL_500:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSPATIALALGORITHMMAGGAIN, NULL, NULL, "%g",
		  ch->algorithmMagnetometerGain);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1044_SPATIAL_510:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSPATIALALGORITHMMAGGAIN, NULL, NULL, "%g",
		  ch->algorithmMagnetometerGain);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1056_SPATIAL_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_1056_SPATIAL_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_MOT1101_SPATIAL_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_MOT1102_SPATIAL_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSPATIALALGORITHMMAGGAIN, NULL, NULL, "%g",
		  ch->algorithmMagnetometerGain);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_MOT0109_SPATIAL_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSPATIALALGORITHMMAGGAIN, NULL, NULL, "%g",
		  ch->algorithmMagnetometerGain);
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

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetSpatial));
}

static PhidgetReturnCode CCONV
_create(PhidgetSpatialHandle *phidp) {

	CHANNELCREATE_BODY(Spatial, PHIDCHCLASS_SPATIAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_delete(PhidgetSpatialHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetSpatial_setAHRSParameters(PhidgetSpatialHandle ch, double angularVelocityThreshold,
  double angularVelocityDeltaThreshold, double accelerationThreshold, double magTime, double accelTime, double biasTime) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETAHRSPARAMETERS, NULL, NULL, "%g%g%g%g%g%g",
	  angularVelocityThreshold, angularVelocityDeltaThreshold, accelerationThreshold, magTime, accelTime, biasTime);
}

API_PRETURN
PhidgetSpatial_setMagnetometerCorrectionParameters(PhidgetSpatialHandle ch, double magneticField,
  double offset0, double offset1, double offset2, double gain0, double gain1, double gain2, double T0, double T1, double T2, double T3, double T4, double T5) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCORRECTIONPARAMETERS, NULL, NULL,
	  "%g%g%g%g%g%g%g%g%g%g%g%g%g", magneticField, offset0, offset1, offset2, gain0, gain1, gain2, T0, T1, T2, T3, T4, T5);
}

API_PRETURN
PhidgetSpatial_resetMagnetometerCorrectionParameters(PhidgetSpatialHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_RESETCORRECTIONPARAMETERS, NULL, NULL,
	  NULL);
}

API_PRETURN
PhidgetSpatial_saveMagnetometerCorrectionParameters(PhidgetSpatialHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SAVECORRECTIONPARAMETERS, NULL, NULL, NULL);
}

API_PRETURN
PhidgetSpatial_zeroAlgorithm(PhidgetSpatialHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_ZEROSPATIALALGORITHM, NULL, NULL, NULL);
}

API_PRETURN
PhidgetSpatial_zeroGyro(PhidgetSpatialHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_ZERO, NULL, NULL, NULL);
}

API_PRETURN
PhidgetSpatial_setAlgorithm(PhidgetSpatialHandle ch, Phidget_SpatialAlgorithm algorithm) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSPATIALALGORITHM, NULL, NULL, "%d",
	  algorithm));
}

API_PRETURN
PhidgetSpatial_getAlgorithm(PhidgetSpatialHandle ch, Phidget_SpatialAlgorithm *algorithm) {

	TESTPTR_PR(ch);
	TESTPTR_PR(algorithm);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	*algorithm = ch->algorithm;
	if (ch->algorithm == (Phidget_SpatialAlgorithm)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_setAlgorithmMagnetometerGain(PhidgetSpatialHandle ch, double algorithmMagnetometerGain) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSPATIALALGORITHMMAGGAIN, NULL, NULL, "%g",
	  algorithmMagnetometerGain));
}

API_PRETURN
PhidgetSpatial_getAlgorithmMagnetometerGain(PhidgetSpatialHandle ch,
  double *algorithmMagnetometerGain) {

	TESTPTR_PR(ch);
	TESTPTR_PR(algorithmMagnetometerGain);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1042_SPATIAL_300:
	case PHIDCHUID_1044_SPATIAL_400:
	case PHIDCHUID_1056_SPATIAL_000:
	case PHIDCHUID_1056_SPATIAL_200:
	case PHIDCHUID_MOT1101_SPATIAL_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*algorithmMagnetometerGain = ch->algorithmMagnetometerGain;
	if (ch->algorithmMagnetometerGain == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_setDataInterval(PhidgetSpatialHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetSpatial_getDataInterval(PhidgetSpatialHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	*dataInterval = (uint32_t)round(ch->dataInterval);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_getMinDataInterval(PhidgetSpatialHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_getMaxDataInterval(PhidgetSpatialHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_setDataRate(PhidgetSpatialHandle ch, double dataRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u%g",
	  (uint32_t)round(1000.0 / dataRate), (double)(1000.0 / dataRate));
}

API_PRETURN
PhidgetSpatial_getDataRate(PhidgetSpatialHandle ch, double *dataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	*dataRate = round_double((1000.0 / ch->dataInterval), 4);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_getMinDataRate(PhidgetSpatialHandle ch, double *minDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	*minDataRate = ch->minDataRate;
	if (ch->minDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_getMaxDataRate(PhidgetSpatialHandle ch, double *maxDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	*maxDataRate = ch->maxDataRate;
	if (ch->maxDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_setHeatingEnabled(PhidgetSpatialHandle ch, int heatingEnabled) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETHEATINGENABLED, NULL, NULL, "%d",
	  heatingEnabled));
}

API_PRETURN
PhidgetSpatial_getHeatingEnabled(PhidgetSpatialHandle ch, int *heatingEnabled) {

	TESTPTR_PR(ch);
	TESTPTR_PR(heatingEnabled);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1042_SPATIAL_300:
	case PHIDCHUID_1044_SPATIAL_400:
	case PHIDCHUID_1044_SPATIAL_500:
	case PHIDCHUID_1044_SPATIAL_510:
	case PHIDCHUID_1056_SPATIAL_000:
	case PHIDCHUID_1056_SPATIAL_200:
	case PHIDCHUID_MOT1101_SPATIAL_100:
	case PHIDCHUID_MOT1102_SPATIAL_200:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*heatingEnabled = ch->heatingEnabled;
	if (ch->heatingEnabled == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_setPrecision(PhidgetSpatialHandle ch, Phidget_SpatialPrecision precision) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSPATIALPRECISION, NULL, NULL, "%d",
	  precision));
}

API_PRETURN
PhidgetSpatial_getPrecision(PhidgetSpatialHandle ch, Phidget_SpatialPrecision *precision) {

	TESTPTR_PR(ch);
	TESTPTR_PR(precision);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	*precision = ch->precision;
	if (ch->precision == (Phidget_SpatialPrecision)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_getQuaternion(PhidgetSpatialHandle ch, PhidgetSpatial_SpatialQuaternion *quaternion) {

	TESTPTR_PR(ch);
	TESTPTR_PR(quaternion);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1042_SPATIAL_300:
	case PHIDCHUID_1044_SPATIAL_400:
	case PHIDCHUID_1056_SPATIAL_000:
	case PHIDCHUID_1056_SPATIAL_200:
	case PHIDCHUID_MOT1101_SPATIAL_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*quaternion = ch->quaternion;
	if (ch->quaternionValid == PFALSE)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_setOnAlgorithmDataHandler(PhidgetSpatialHandle ch,
  PhidgetSpatial_OnAlgorithmDataCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);

	ch->AlgorithmData = fptr;
	ch->AlgorithmDataCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_setOnSpatialDataHandler(PhidgetSpatialHandle ch, PhidgetSpatial_OnSpatialDataCallback fptr,
  void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);

	ch->SpatialData = fptr;
	ch->SpatialDataCtx = ctx;

	return (EPHIDGET_OK);
}
