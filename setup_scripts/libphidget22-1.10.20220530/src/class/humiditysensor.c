/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/humiditysensor.gen.h"
#include "class/humiditysensor.gen.c"

static void
PhidgetHumiditySensor_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetHumiditySensorHandle ch = (PhidgetHumiditySensorHandle)phid;
	switch (code) {
	case EEPHIDGET_SATURATION:
		ch->humidity = PUNK_DBL;
		break;
	}
}

static void CCONV
PhidgetHumiditySensor_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetHumiditySensor_create(PhidgetHumiditySensorHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetHumiditySensor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetHumiditySensor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetHumiditySensor_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetHumiditySensor_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetHumiditySensor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_bridgeInput(phid, bp));
}

static void
PhidgetHumiditySensor_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetHumiditySensor_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
