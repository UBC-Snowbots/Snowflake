/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/gps.gen.h"
#include "class/gps.gen.c"

static void
PhidgetGPS_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetGPS_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetGPS_create(PhidgetGPSHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetGPS_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetGPS_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetGPS_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetGPS_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetGPS_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetGPSHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetGPSHandle)phid;

	switch (bp->vpkt) {
	case BP_DATA:
		res = readNMEAData(bp, &ch->NMEAData);
		if(res == EPHIDGET_OK)
			ch->NMEADataValid = PTRUE;
		break;
	case BP_TIME:
		if (getBridgePacketInt32(bp, 0))
			res = readGPSTime(bp, &ch->time);
		else
			res = EPHIDGET_OK;
		ch->timeValid = (uint8_t)getBridgePacketInt32(bp, 0);
		break;
	case BP_DATE:
		if (getBridgePacketInt32(bp, 0))
			res = readGPSDate(bp, &ch->date);
		else
			res = EPHIDGET_OK;
		ch->dateValid = (uint8_t)getBridgePacketInt32(bp, 0);
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetGPS_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetGPS_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
