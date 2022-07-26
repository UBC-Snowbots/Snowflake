/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/lightsensor.gen.h"
#include "class/lightsensor.gen.c"

static void
PhidgetLightSensor_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetLightSensorHandle ch = (PhidgetLightSensorHandle)phid;
	switch (code) {
	case EEPHIDGET_SATURATION:
		ch->illuminance = PUNK_DBL;
		break;
	}
}

static void CCONV
PhidgetLightSensor_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetLightSensor_create(PhidgetLightSensorHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetLightSensor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetLightSensor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetLightSensor_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetLightSensor_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetLightSensor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_bridgeInput(phid, bp));
}

static void
PhidgetLightSensor_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetLightSensor_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
