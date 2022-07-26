/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/resistanceinput.gen.h"
#include "class/resistanceinput.gen.c"

static void
PhidgetResistanceInput_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetResistanceInputHandle ch = (PhidgetResistanceInputHandle)phid;
	switch (code) {
	case EEPHIDGET_SATURATION:
		ch->resistance = PUNK_DBL;
		break;
	}
}

static void CCONV
PhidgetResistanceInput_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetResistanceInput_create(PhidgetResistanceInputHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetResistanceInput_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetResistanceInput_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetResistanceInput_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetResistanceInput_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetResistanceInput_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_bridgeInput(phid, bp));
}

static void
PhidgetResistanceInput_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetResistanceInput_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

