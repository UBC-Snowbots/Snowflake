/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/temperaturesensor.gen.h"
#include "class/temperaturesensor.gen.c"

static void
PhidgetTemperatureSensor_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetTemperatureSensorHandle ch = (PhidgetTemperatureSensorHandle)phid;
	switch (code) {
	case EEPHIDGET_SATURATION:
		ch->temperature = PUNK_DBL;
		break;
	case EEPHIDGET_OUTOFRANGE:
		ch->temperature = PUNK_DBL;
		break;
	}
}

static void CCONV
PhidgetTemperatureSensor_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetTemperatureSensor_create(PhidgetTemperatureSensorHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetTemperatureSensor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetTemperatureSensor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetTemperatureSensor_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetTemperatureSensor_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

extern const int thermocouple_useful_range[5][2];

#include "types.gen.h"

static PhidgetReturnCode
PhidgetTemperatureSensor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetTemperatureSensorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetTemperatureSensorHandle)phid;

	/*
	 * Set the temperature to unknown if the RTD or thermocouple type is changed.
	 * For thermocouples, reset the max/min range based on thermocouple type.
	 */
	switch (bp->vpkt) {
	case BP_SETRTDTYPE:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK) {
			PhidgetRunLock(ch);
			ch->temperature = PUNK_DBL;
			PhidgetRunUnlock(ch);
		}
		break;
	case BP_SETTHERMOCOUPLETYPE:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK) {
			PhidgetRunLock(ch);
			ch->temperature = PUNK_DBL;
			ch->minTemperature = thermocouple_useful_range[(PhidgetTemperatureSensor_ThermocoupleType)getBridgePacketInt32(bp, 0)][0];
			ch->maxTemperature = thermocouple_useful_range[(PhidgetTemperatureSensor_ThermocoupleType)getBridgePacketInt32(bp, 0)][1];
			PhidgetRunUnlock(ch);
		}
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
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetTemperatureSensor_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetTemperatureSensor_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}