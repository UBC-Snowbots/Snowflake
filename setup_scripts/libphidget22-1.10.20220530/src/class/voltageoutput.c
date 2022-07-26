/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/voltageoutput.gen.h"
#include "class/voltageoutput.gen.c"

static void
PhidgetVoltageOutput_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetVoltageOutput_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetVoltageOutput_create(PhidgetVoltageOutputHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetVoltageOutput_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetVoltageOutput_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetVoltageOutput_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetVoltageOutput_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetVoltageOutput_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetVoltageOutputHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetVoltageOutputHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETVOLTAGERANGE:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK) {
			switch (ch->voltageOutputRange) {
			case VOLTAGE_OUTPUT_RANGE_10V:
				ch->minVoltage = -10;
				ch->maxVoltage = 10;
				break;
			case VOLTAGE_OUTPUT_RANGE_5V:
				ch->minVoltage = 0;
				ch->maxVoltage = 5;
				break;
			}
		}
		break;
	case BP_SETFAILSAFETIME:
		TESTRANGE_IOP(bp->iop, "%u", getBridgePacketUInt32(bp, 0), ch->minFailsafeTime, ch->maxFailsafeTime);
		res = _bridgeInput(phid, bp);
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetVoltageOutput_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetVoltageOutput_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
