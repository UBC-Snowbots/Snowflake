/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/currentinput.gen.h"
#include "class/currentinput.gen.c"

static void
PhidgetCurrentInput_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetCurrentInputHandle ch = (PhidgetCurrentInputHandle)phid;
	switch (code) {
	case EEPHIDGET_SATURATION:
		ch->current = PUNK_DBL;
		break;
	}
}

static void CCONV
PhidgetCurrentInput_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetCurrentInput_create(PhidgetCurrentInputHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetCurrentInput_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetCurrentInput_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetCurrentInput_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetCurrentInput_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetCurrentInput_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_bridgeInput(phid, bp));
}

static void
PhidgetCurrentInput_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetCurrentInput_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
