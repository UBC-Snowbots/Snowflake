/* Generated: Wed Apr 25 2018 09:34:04 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/currentoutput.gen.h"
#include "class/currentoutput.gen.c"

static void CCONV
PhidgetCurrentOutput_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetCurrentOutput_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetCurrentOutput_create(PhidgetCurrentOutputHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetCurrentOutput_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetCurrentOutput_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetCurrentOutput_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetCurrentOutput_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetCurrentOutput_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;

	switch (bp->vpkt) {
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void CCONV
PhidgetCurrentOutput_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetCurrentOutput_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
