/* Generated: Mon Apr 11 2016 14:15:00 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/distancesensor.gen.h"
#include "class/distancesensor.gen.c"

static void CCONV
PhidgetDistanceSensor_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetDistanceSensorHandle ch = (PhidgetDistanceSensorHandle)phid;
	switch (code) {
	case EEPHIDGET_SATURATION:
	case EEPHIDGET_OUTOFRANGE:
		ch->distance = PUNK_UINT32;
		break;
	}
}

static void CCONV
PhidgetDistanceSensor_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetDistanceSensor_create(PhidgetDistanceSensorHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetDistanceSensor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDistanceSensor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDistanceSensor_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetDistanceSensorHandle ch;

	TESTPTR(phid);
	ch = (PhidgetDistanceSensorHandle)phid;

	ch->distances[0] = PUNK_UINT32;
	ch->distances[1] = PUNK_UINT32;
	ch->distances[2] = PUNK_UINT32;
	ch->distances[3] = PUNK_UINT32;
	ch->distances[4] = PUNK_UINT32;
	ch->distances[5] = PUNK_UINT32;
	ch->distances[6] = PUNK_UINT32;
	ch->distances[7] = PUNK_UINT32;
	ch->amplitudes[0] = PUNK_UINT32;
	ch->amplitudes[1] = PUNK_UINT32;
	ch->amplitudes[2] = PUNK_UINT32;
	ch->amplitudes[3] = PUNK_UINT32;
	ch->amplitudes[4] = PUNK_UINT32;
	ch->amplitudes[5] = PUNK_UINT32;
	ch->amplitudes[6] = PUNK_UINT32;
	ch->amplitudes[7] = PUNK_UINT32;
	ch->count = 0;
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetDistanceSensor_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetDistanceSensor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDistanceSensorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDistanceSensorHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SONARUPDATE:
		memcpy(&ch->distances, getBridgePacketUInt32Array(bp, 0), sizeof (uint32_t) * 8);
		memcpy(&ch->amplitudes, getBridgePacketUInt32Array(bp, 1), sizeof (uint32_t) * 8);
		ch->count = getBridgePacketUInt32(bp, 2);
		FIRECH(ch, SonarReflectionsUpdate, ch->distances, ch->amplitudes, ch->count);
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetDistanceSensor_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetDistanceSensor_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetDistanceSensor_getSonarReflections(PhidgetDistanceSensorHandle ch, uint32_t (*distances)[8],
  uint32_t (*amplitudes)[8], uint32_t *count) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DISTANCESENSOR);
	TESTATTACHED_PR(ch);
	TESTPTR_PR(distances);
	TESTPTR_PR(amplitudes);
	TESTPTR_PR(count);

	memcpy(*distances, &ch->distances, sizeof(uint32_t) * 8);
	memcpy(*amplitudes, &ch->amplitudes, sizeof(uint32_t) * 8);
	*count = ch->count;
	memset(&(*distances)[ch->count], PUNK_UINT32, sizeof(uint32_t) * (8-ch->count));
	memset(&(*amplitudes)[ch->count], PUNK_UINT32, sizeof(uint32_t) * (8-ch->count));

	return (EPHIDGET_OK);
}
