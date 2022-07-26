/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/digitaloutput.gen.h"
#include "class/digitaloutput.gen.c"

static void
PhidgetDigitalOutput_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetDigitalOutputHandle ch = (PhidgetDigitalOutputHandle)phid;
	switch (code) {
	case EEPHIDGET_FAILSAFE:
		ch->state = 0;
		ch->dutyCycle = 0;
		break;
	case EEPHIDGET_INVALIDSTATE:
		ch->state = PUNK_BOOL;
		ch->dutyCycle = PUNK_DBL;
		break;
	}
}

static void CCONV
PhidgetDigitalOutput_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetDigitalOutput_create(PhidgetDigitalOutputHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetDigitalOutput_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDigitalOutput_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDigitalOutput_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetDigitalOutput_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetDigitalOutput_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDigitalOutputHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDigitalOutputHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETFAILSAFETIME:
		TESTRANGE_IOP(bp->iop, "%u", getBridgePacketUInt32(bp, 0), ch->minFailsafeTime, ch->maxFailsafeTime);
		res = _bridgeInput(phid, bp);
		break;
	case BP_SETDUTYCYCLE:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK)
			ch->state = (ch->dutyCycle ? 1 : 0);
		break;
	case BP_SETSTATE:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK)
			ch->dutyCycle = (ch->state ? 1 : 0);
		break;
	default:
		res = _bridgeInput(phid, bp);
	}

	return (res);
}

static void
PhidgetDigitalOutput_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetDigitalOutput_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
