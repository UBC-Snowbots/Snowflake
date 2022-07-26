/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/digitalinput.gen.h"
#include "class/digitalinput.gen.c"

static void
PhidgetDigitalInput_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetDigitalInputHandle ch = (PhidgetDigitalInputHandle)phid;

	ch->state = PUNK_BOOL;
}

static void CCONV
PhidgetDigitalInput_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetDigitalInput_create(PhidgetDigitalInputHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetDigitalInput_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDigitalInput_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDigitalInput_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetDigitalInput_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetDigitalInput_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {

	return (_bridgeInput(phid, bp));
}

static void
PhidgetDigitalInput_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetDigitalInput_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
