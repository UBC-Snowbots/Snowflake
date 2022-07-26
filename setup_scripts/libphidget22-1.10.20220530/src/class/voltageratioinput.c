/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "analogsensor.h"
#include "class/voltageratioinput.gen.h"
#include "class/voltageratioinput.gen.c"

static void
PhidgetVoltageRatioInput_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetVoltageRatioInputHandle ch = (PhidgetVoltageRatioInputHandle)phid;
	switch (code) {
	case EEPHIDGET_SATURATION:
		ch->voltageRatio = PUNK_DBL;
		ch->sensorValue = PUNK_DBL;
		break;
	}
}

static void CCONV
PhidgetVoltageRatioInput_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetVoltageRatioInput_create(PhidgetVoltageRatioInputHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetVoltageRatioInput_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetVoltageRatioInput_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetVoltageRatioInput_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetVoltageRatioInputHandle ch;
	PhidgetReturnCode res;

	TESTPTR(phid);
	ch = (PhidgetVoltageRatioInputHandle)phid;

	res = _initAfterOpen(phid);
	if (res != EPHIDGET_OK)
		return (res);

	ch->sensorUnit = PhidgetAnalogSensor_getVoltageRatioSensorUnit(ch->sensorType);
	ch->sensorUnitValid = PTRUE;
	ch->sensorValue = PhidgetAnalogSensor_getVoltageRatioSensorValue(ch->voltageRatio, ch->sensorType);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetVoltageRatioInput_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
bangSensorVoltage(PhidgetChannelHandle phid, int *sentSensorEvent, int includeVoltage) {
	PhidgetVoltageRatioInputHandle ch;
	Phidget_UnitInfo unitInfo;
	PhidgetReturnCode res;
	double sensorValue;
	BridgePacket *bp;
	double ratio;
	
	ch = (PhidgetVoltageRatioInputHandle)phid;
	res = EPHIDGET_OK;

	ratio = ch->voltageRatio;
	if (ratio == PUNK_DBL)
		ratio = NAN;

	if (supportedBridgePacket(phid, BP_SENSORCHANGE) && !isNetworkPhidget(ch) && ch->sensorType != SENSOR_TYPE_VOLTAGERATIO) {
		sensorValue = PhidgetAnalogSensor_getVoltageRatioSensorValue(ratio, ch->sensorType);
		if (!PhidgetAnalogSensor_getVoltageRatioSensorValueInRange(sensorValue, ch->sensorType)) {
			ch->sensorValue = PUNK_DBL;
			if (ISATTACHEDDONE(ch))
				SEND_ERROR_EVENT(ch, EEPHIDGET_OUTOFRANGE, "Sensor value is ouside the valid range for this sensor.");
		} else if (ch->sensorValue == PUNK_DBL || fabs(sensorValue - ch->sensorValue) >= ch->sensorValueChangeTrigger) {
			ch->sensorValue = sensorValue;
			if (ISATTACHEDDONE(ch)) {
				res = createBridgePacket(&bp, BP_SENSORCHANGE, "%g", sensorValue);
				if (res == EPHIDGET_OK) {
					unitInfo = PhidgetAnalogSensor_getVoltageRatioSensorUnit(ch->sensorType);
					res = writeUnitInfo(&unitInfo, bp);
					if (res == EPHIDGET_OK)
						res = dispatchChannelBridgePacket(phid, bp);
				}
			}
		}
		if (res == EPHIDGET_OK)
			*sentSensorEvent = PTRUE;
	} else if (includeVoltage) {
		ch->sensorUnit = PhidgetAnalogSensor_getVoltageRatioSensorUnit(ch->sensorType);
		ch->sensorUnitValid = PTRUE;
		ch->sensorValue = PhidgetAnalogSensor_getVoltageRatioSensorValue(ratio, ch->sensorType);

		if (ISATTACHEDDONE(ch))
			FIRECH(ch, VoltageRatioChange, ratio);
	}

	return (res);
}

static int
getBridgePacketEntryOffsetByName(BridgePacket *bp, const char *name) {
	int off;

	for (off = 0; off < bp->entrycnt; off++) {
		if (bp->entry[off].name && mos_strcmp(bp->entry[off].name, name) == 0)
			return (off);
	}
	return (-1);
}


static PhidgetReturnCode
PhidgetVoltageRatioInput_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetVoltageRatioInputHandle ch;
	Phidget_UnitInfo unitInfo;
	int sentSensorEvent = 0;
	PhidgetReturnCode res;

	TESTPTR(phid);
	ch = (PhidgetVoltageRatioInputHandle)phid;

	switch (bp->vpkt) {
	case BP_MINDATAINTERVALCHANGE:
		ch->minDataInterval = getBridgePacketUInt32(bp, 0);
		FIRE_PROPERTYCHANGE(ch, "MinDataInterval");
		res = EPHIDGET_OK;
		break;
	case BP_DATAINTERVALCHANGE:
		if (bp->entrycnt > 1)
			ch->dataInterval = getBridgePacketDouble(bp, 1);
		else
			ch->dataInterval = (double)getBridgePacketUInt32(bp, 0);
		FIRE_PROPERTYCHANGE(ch, "DataInterval");
		FIRE_PROPERTYCHANGE(ch, "DataRate");
		res = EPHIDGET_OK;
		break;
	case BP_SETSENSORTYPE:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK)
			res = bangSensorVoltage(phid, &sentSensorEvent, 0);
		break;
	case BP_SETSENSORVALUECHANGETRIGGER:
		if (getBridgePacketDouble(bp, 0) < 0)
			return (EPHIDGET_INVALIDARG);
		res = _bridgeInput(phid, bp);
		break;
	case BP_SENSORCHANGE:
		res = readUnitInfo(bp, &unitInfo);
		if (res != EPHIDGET_OK)
			break;
		ch->sensorValue = getBridgePacketDouble(bp, 0);
		ch->sensorUnit = Phidget_Units[unitInfo.unit];
		ch->sensorUnitValid = PTRUE;
		FIRECH(ch, SensorChange, ch->sensorValue, &unitInfo);
		res = EPHIDGET_OK;
		break;
	case BP_VOLTAGERATIOCHANGE:
		ch->voltageRatio = getBridgePacketDouble(bp, 0);
		res = bangSensorVoltage(phid, &sentSensorEvent, 1);
		// If we are opened by a network client, we want to stop the dispatcher from forwarding the VoltageChange event to network clients
		if (sentSensorEvent && res == EPHIDGET_OK && PhidgetCKFlags(ch, PHIDGET_OPENBYNETCLIENT_FLAG))
			res = EPHIDGET_UNSUPPORTED;
		break;
	case BP_SETENABLED:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK) {
			if (getBridgePacketEntryOffsetByName(bp, "DataIntervalResp") != -1)
				ch->dataInterval = getBridgePacketDoubleByName(bp, "DataIntervalResp");
		}
		break;
#if PHIDUID_1046_1_SUPPORTED
	case BP_SETBRIDGEGAIN:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK) {
			switch (ch->phid.parent->deviceInfo.UDD->uid) {
			case PHIDUID_1046_1:
				switch (ch->bridgeGain) {
				case BRIDGE_GAIN_1:
					ch->maxVoltageRatio = 0.85;
					ch->minVoltageRatio = -0.85;
					break;
				case BRIDGE_GAIN_8:
					ch->maxVoltageRatio = 1.0 / 8;
					ch->minVoltageRatio = -1.0 / 8;
					break;
				case BRIDGE_GAIN_16:
					ch->maxVoltageRatio = 1.0 / 16;
					ch->minVoltageRatio = -1.0 / 16;
					break;
				case BRIDGE_GAIN_32:
					ch->maxVoltageRatio = 1.0 / 32;
					ch->minVoltageRatio = -1.0 / 32;
					break;
				case BRIDGE_GAIN_64:
					ch->maxVoltageRatio = 1.0 / 64;
					ch->minVoltageRatio = -1.0 / 64;
					break;
				case BRIDGE_GAIN_128:
					ch->maxVoltageRatio = 1.0 / 128;
					ch->minVoltageRatio = -1.0 / 128;
					break;
				}
				bridgeSendToChannel((PhidgetChannelHandle)ch, BP_PROPERTYCHANGE, "%s", "MinVoltageRatio");
				bridgeSendToChannel((PhidgetChannelHandle)ch, BP_PROPERTYCHANGE, "%s", "MaxVoltageRatio");
				break;
			}
		}
		break;
	case BP_PROPERTYCHANGE:
		FIRE_PROPERTYCHANGE(ch, getBridgePacketString(bp, 0));
		res = EPHIDGET_OK;
		break;
#endif //PHIDUID_1046_1_SUPPORTED
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetVoltageRatioInput_fireInitialEvents(PhidgetChannelHandle phid) {
	PhidgetVoltageRatioInputHandle ch;
	Phidget_UnitInfo unitInfo;

	ch = (PhidgetVoltageRatioInputHandle)phid;

	if (supportedBridgePacket(phid, BP_SENSORCHANGE) && !isNetworkPhidget(ch) && ch->sensorType != SENSOR_TYPE_VOLTAGERATIO) {
		if (ch->sensorValue != PUNK_DBL && ch->sensorUnitValid) {
			unitInfo = ch->sensorUnit;
			FIRECH(ch, SensorChange, ch->sensorValue, &unitInfo);
		}
	} else {
		if (ch->voltageRatio != PUNK_DBL)
			FIRECH(ch, VoltageRatioChange, ch->voltageRatio);
	}

	_fireInitialEvents(phid);
}

static int
PhidgetVoltageRatioInput_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetVoltageRatioInputHandle ch;
	ch = (PhidgetVoltageRatioInputHandle)phid;

	if (ch->voltageRatio == PUNK_DBL)
		return (PFALSE);

	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetVoltageRatioInput_getVoltageRatio(PhidgetVoltageRatioInputHandle ch, double *voltageRatio) {

	TESTPTR_PR(ch);
	TESTPTR_PR(voltageRatio);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHED_PR(ch);

	*voltageRatio = ch->voltageRatio;
	if (ch->voltageRatio == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	if (ch->voltageRatio > ch->maxVoltageRatio)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVALHIGH));
	if (ch->voltageRatio < ch->minVoltageRatio)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVALLOW));

	return (EPHIDGET_OK);
}
