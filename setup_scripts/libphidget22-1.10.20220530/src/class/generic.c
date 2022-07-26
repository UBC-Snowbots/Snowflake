/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/generic.gen.h"
#include "class/generic.gen.c"

static void
PhidgetGeneric_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetGeneric_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetGeneric_create(PhidgetGenericHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetGeneric_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetGeneric_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetGeneric_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetGeneric_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetGeneric_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetGenericHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetGenericHandle)phid;

	switch (bp->vpkt) {

	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetGeneric_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetGeneric_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

