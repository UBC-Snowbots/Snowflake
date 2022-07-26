/* Generated: Thu Oct 06 2016 09:55:35 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/capacitivetouch.gen.h"
#include "class/capacitivetouch.gen.c"

static void CCONV
PhidgetCapacitiveTouch_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetCapacitiveTouch_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetCapacitiveTouch_create(PhidgetCapacitiveTouchHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetCapacitiveTouch_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetCapacitiveTouch_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetCapacitiveTouch_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetCapacitiveTouch_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetCapacitiveTouch_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {

	switch (bp->vpkt) {
	case BP_TOUCHINPUTVALUECHANGE:
		((PhidgetCapacitiveTouchHandle)phid)->isTouched = PTRUE;
		break;
	case BP_TOUCHINPUTEND:
		((PhidgetCapacitiveTouchHandle)phid)->isTouched = PFALSE;
		break;
	}
	return (_bridgeInput(phid, bp));
}

static void CCONV
PhidgetCapacitiveTouch_fireInitialEvents(PhidgetChannelHandle phid) {

	_fireInitialEvents(phid);
}

static int
PhidgetCapacitiveTouch_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
