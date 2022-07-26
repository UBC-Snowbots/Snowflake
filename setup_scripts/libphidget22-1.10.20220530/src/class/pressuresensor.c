/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/pressuresensor.gen.h"
#include "class/pressuresensor.gen.c"

static void
PhidgetPressureSensor_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetPressureSensorHandle ch = (PhidgetPressureSensorHandle)phid;
	switch (code) {
	case EEPHIDGET_SATURATION:
		ch->pressure = PUNK_DBL;
		break;
	}
}

static void CCONV
PhidgetPressureSensor_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetPressureSensor_create(PhidgetPressureSensorHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetPressureSensor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetPressureSensor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetPressureSensor_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetPressureSensor_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetPressureSensor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_bridgeInput(phid, bp));
}

static void
PhidgetPressureSensor_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetPressureSensor_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
