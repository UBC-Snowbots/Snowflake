/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "analogsensor.h"
#include "class/voltageinput.gen.h"
#include "class/voltageinput.gen.c"
#include "util/voltageinputsupport.h"


// Access the PhidgetIRSupport struct via the channel private pointer
#define VOLTAGEINPUT_SUPPORT(ch) ((PhidgetVoltageInputSupportHandle)(((PhidgetChannelHandle)(ch))->private))

static void
PhidgetVoltageInput_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetVoltageInputHandle ch = (PhidgetVoltageInputHandle)phid;
	switch (code) {
	case EEPHIDGET_SATURATION:
		ch->voltage = PUNK_DBL;
		ch->sensorValue = PUNK_DBL;
		break;
	}
}

static void CCONV
PhidgetVoltageInput_free(PhidgetChannelHandle *ch) {
	if (ch && *ch)
		PhidgetVoltageInputSupport_free((PhidgetVoltageInputSupportHandle *)&(*ch)->private);
	_free(ch);
}

API_PRETURN
PhidgetVoltageInput_create(PhidgetVoltageInputHandle *phidp) {
	PhidgetReturnCode res;

	res = _create(phidp);
	if (res == EPHIDGET_OK)
		res = PhidgetVoltageInputSupport_create((PhidgetVoltageInputSupportHandle *)&(*phidp)->phid.private);

	return (res);
}

static PhidgetReturnCode CCONV
PhidgetVoltageInput_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetVoltageInput_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetVoltageInput_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetVoltageInputHandle ch;
	PhidgetReturnCode res;

	TESTPTR(phid);
	ch = (PhidgetVoltageInputHandle)phid;
	PhidgetVoltageInputSupport_init(VOLTAGEINPUT_SUPPORT(phid));
	res = _initAfterOpen(phid);
	if (res != EPHIDGET_OK)
		return (res);

	ch->sensorUnit = PhidgetAnalogSensor_getVoltageSensorUnit(ch->sensorType);
	ch->sensorUnitValid = PTRUE;
	ch->sensorValue = PhidgetAnalogSensor_getVoltageSensorValue(ch->voltage, ch->sensorType, VOLTAGEINPUT_SUPPORT(ch));

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetVoltageInput_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
bangSensorVoltage(PhidgetChannelHandle phid, int *sentSensorEvent, int includeVoltage) {
	PhidgetVoltageInputHandle ch;
	Phidget_UnitInfo unitInfo;
	PhidgetReturnCode res;
	double sensorValue;
	BridgePacket *bp;

	ch = (PhidgetVoltageInputHandle)phid;
	res = EPHIDGET_OK;

	if (supportedBridgePacket(phid, BP_SENSORCHANGE) && !isNetworkPhidget(ch) && ch->sensorType != SENSOR_TYPE_VOLTAGE) {
		sensorValue = PhidgetAnalogSensor_getVoltageSensorValue(ch->voltage, ch->sensorType, VOLTAGEINPUT_SUPPORT(ch));
		if (!PhidgetAnalogSensor_getVoltageSensorValueInRange(sensorValue, ch->sensorType, VOLTAGEINPUT_SUPPORT(ch))) {
			ch->sensorValue = PUNK_DBL;
			if (ISATTACHEDDONE(ch))
				SEND_ERROR_EVENT(ch, EEPHIDGET_OUTOFRANGE, "Sensor value is ouside the valid range for this sensor.");
		} else if (ch->sensorValue == PUNK_DBL || fabs(sensorValue - ch->sensorValue) >= ch->sensorValueChangeTrigger) {
			ch->sensorValue = sensorValue;
			if (ISATTACHEDDONE(ch)) {
				res = createBridgePacket(&bp, BP_SENSORCHANGE, "%g", sensorValue);
				if (res == EPHIDGET_OK) {
					unitInfo = PhidgetAnalogSensor_getVoltageSensorUnit(ch->sensorType);
					res = writeUnitInfo(&unitInfo, bp);
					if (res == EPHIDGET_OK)
						res = dispatchChannelBridgePacket(phid, bp);
				}
			}
		}
		if (res == EPHIDGET_OK)
			*sentSensorEvent = PTRUE;
	} else if (includeVoltage) {
		ch->sensorUnit = PhidgetAnalogSensor_getVoltageSensorUnit(ch->sensorType);
		ch->sensorUnitValid = PTRUE;
		ch->sensorValue = PhidgetAnalogSensor_getVoltageSensorValue(ch->voltage, ch->sensorType, VOLTAGEINPUT_SUPPORT(ch));

		if (ISATTACHEDDONE(ch))
			FIRECH(ch, VoltageChange, ch->voltage);
	}

	return (res);
}

static PhidgetReturnCode
PhidgetVoltageInput_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetVoltageInputHandle ch;
	Phidget_UnitInfo unitInfo;
	int sentSensorEvent = 0;
	PhidgetReturnCode res;
	BridgePacket *subbp;

	TESTPTR(phid);
	ch = (PhidgetVoltageInputHandle)phid;

	switch (bp->vpkt) {
	case BP_SETDATAINTERVAL:
		res = EPHIDGET_OK;
		switch (ch->sensorType) {
		case SENSOR_TYPE_MOT2002_LOW:
		case SENSOR_TYPE_MOT2002_MED:
		case SENSOR_TYPE_MOT2002_HIGH:
			res = EPHIDGET_INVALID;
			break;
		default:
			break;
		}
		if(res == EPHIDGET_OK)
			res = _bridgeInput(phid, bp);
		break;
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
		if (res == EPHIDGET_OK) {
			res = bangSensorVoltage(phid, &sentSensorEvent, 0);
			if (res != EPHIDGET_OK)
				break;
			switch (ch->sensorType) {
			case SENSOR_TYPE_MOT2002_LOW:
			case SENSOR_TYPE_MOT2002_MED:
			case SENSOR_TYPE_MOT2002_HIGH:
				createBridgePacket(&subbp, BP_SETDATAINTERVAL, "%u", 200);
				_bridgeInput(phid, subbp);
				destroyBridgePacket(&subbp);
				break;
			default:
				break;
			}
		}
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
	case BP_VOLTAGECHANGE:
		ch->voltage = getBridgePacketDouble(bp, 0);

		PhidgetVoltageInputSupport_updateVoltageBuffer(VOLTAGEINPUT_SUPPORT(ch), ch->voltage);

		res = bangSensorVoltage(phid, &sentSensorEvent, 1);
		// If we are opened by a network client, we want to stop the dispatcher from forwarding the VoltageChange event to network clients
		if (sentSensorEvent && res == EPHIDGET_OK && PhidgetCKFlags(ch, PHIDGET_OPENBYNETCLIENT_FLAG))
			res = EPHIDGET_UNSUPPORTED;
		break;
	case BP_SETVOLTAGERANGE:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK) {
			switch (ch->voltageRange) {
			case VOLTAGE_RANGE_10mV:	/* Range ±10mV DC */
				ch->minVoltage = -0.01;
				ch->maxVoltage = 0.01;
				break;
			case VOLTAGE_RANGE_40mV:	/* Range ±40mV DC */
				ch->minVoltage = -0.04;
				ch->maxVoltage = 0.04;
				break;
			case VOLTAGE_RANGE_200mV:	/* Range ±200mV DC */
				ch->minVoltage = -0.2;
				ch->maxVoltage = 0.2;
				break;
			case VOLTAGE_RANGE_312_5mV:	/* Range ±312.5mV DC */
				ch->minVoltage = -0.3125;
				ch->maxVoltage = 0.3125;
				break;
			case VOLTAGE_RANGE_400mV:	/* Range ±400mV DC */
				ch->minVoltage = -0.4;
				ch->maxVoltage = 0.4;
				break;
			case VOLTAGE_RANGE_1000mV:	/* Range ±1000mV DC */
				ch->minVoltage = -1.0;
				ch->maxVoltage = 1.0;
				break;
			case VOLTAGE_RANGE_2V:	/* Range ±2V DC */
				ch->minVoltage = -2.0;
				ch->maxVoltage = 2.0;
				break;
			case VOLTAGE_RANGE_5V:	/* Range ±5V DC */
				ch->minVoltage = -5.0;
				ch->maxVoltage = 5.0;
				break;
			case VOLTAGE_RANGE_15V:	/* Range ±15V DC */
				ch->minVoltage = -15.0;
				ch->maxVoltage = 15.0;
				break;
			case VOLTAGE_RANGE_40V:	/* Range ±40V DC */
				ch->minVoltage = -40.0;
				ch->maxVoltage = 40.0;
				break;
			case VOLTAGE_RANGE_AUTO:	/* Auto-range mode changes based on the present voltage measurements. */
				switch (phid->parent->deviceInfo.UDD->id) {
				case PHIDID_VCP1001:
					ch->minVoltage = -40.0;
					ch->maxVoltage = 40.0;
					break;
				case PHIDID_VCP1002:
					ch->minVoltage = -1.0;
					ch->maxVoltage = 1.0;
					break;
				}
			}
		}
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetVoltageInput_fireInitialEvents(PhidgetChannelHandle phid) {
	PhidgetVoltageInputHandle ch;
	Phidget_UnitInfo unitInfo;

	ch = (PhidgetVoltageInputHandle)phid;

	if (supportedBridgePacket(phid, BP_SENSORCHANGE) && !isNetworkPhidget(ch) && ch->sensorType != SENSOR_TYPE_VOLTAGE) {
		if (ch->sensorValue != PUNK_DBL && ch->sensorUnitValid) {
			unitInfo = ch->sensorUnit;
			FIRECH(ch, SensorChange, ch->sensorValue, &unitInfo);
		}
	} else {
		if (ch->voltage != PUNK_DBL)
			FIRECH(ch, VoltageChange, ch->voltage);
	}

	_fireInitialEvents(phid);
}

static int
PhidgetVoltageInput_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetVoltageInputHandle ch;
	ch = (PhidgetVoltageInputHandle)phid;

	if (ch->voltage == PUNK_DBL)
		return (PFALSE);

	return (_hasInitialState(phid));
}
