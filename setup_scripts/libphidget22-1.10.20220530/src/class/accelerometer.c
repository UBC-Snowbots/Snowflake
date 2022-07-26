/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/accelerometer.gen.h"
#include "class/accelerometer.gen.c"

static void
PhidgetAccelerometer_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetAccelerometerHandle ch = (PhidgetAccelerometerHandle)phid;
	switch (code) {
	case EEPHIDGET_SATURATION:
		ch->acceleration[0] = PUNK_DBL;
		ch->acceleration[1] = PUNK_DBL;
		ch->acceleration[2] = PUNK_DBL;
		ch->timestamp = PUNK_DBL;
		break;
	}
}

static void CCONV
PhidgetAccelerometer_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetAccelerometer_create(PhidgetAccelerometerHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetAccelerometer_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetAccelerometer_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetAccelerometer_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetAccelerometer_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetAccelerometer_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetAccelerometerHandle ch;

	TESTPTR(phid);
	ch = (PhidgetAccelerometerHandle)phid;

	switch (bp->vpkt) {
	case BP_DATAINTERVALCHANGE:
		if (bp->entrycnt > 1)
			ch->dataInterval = getBridgePacketDouble(bp, 1);
		else
			ch->dataInterval = (double)getBridgePacketUInt32(bp, 0);
		FIRE_PROPERTYCHANGE(ch, "DataInterval");
		FIRE_PROPERTYCHANGE(ch, "DataRate");
		return (EPHIDGET_OK);
	}

	return (_bridgeInput(phid, bp));
}

static void
PhidgetAccelerometer_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetAccelerometer_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
