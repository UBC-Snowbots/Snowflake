/* Generated: Thu Feb 04 2016 10:58:25 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/spatial.gen.h"
#include "class/spatial.gen.c"

static void CCONV
PhidgetSpatial_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetSpatial_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetSpatial_create(PhidgetSpatialHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetSpatial_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetSpatial_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetSpatial_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetReturnCode res;
	res = _initAfterOpen(phid);
	if (res)
		return res;

	PhidgetSpatialHandle ch = (PhidgetSpatialHandle)phid;
	ch->quaternion.x = PUNK_DBL;
	ch->quaternion.y = PUNK_DBL;
	ch->quaternion.z = PUNK_DBL;
	ch->quaternion.w = PUNK_DBL;

	ch->eulerAngles.pitch = PUNK_DBL;
	ch->eulerAngles.roll = PUNK_DBL;
	ch->eulerAngles.heading = PUNK_DBL;

	ch->quaternionValid = 0;
	ch->eulerAnglesValid = 0;
	return EPHIDGET_OK;
}

static PhidgetReturnCode CCONV
PhidgetSpatial_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetSpatial_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetSpatialHandle ch;

	TESTPTR(phid);
	ch = (PhidgetSpatialHandle)phid;

	switch (bp->vpkt) {
	case BP_DATAINTERVALCHANGE:
		if (bp->entrycnt > 1)
			ch->dataInterval = getBridgePacketDouble(bp, 1);
		else
			ch->dataInterval = (double)getBridgePacketUInt32(bp, 0);
		FIRE_PROPERTYCHANGE(ch, "DataInterval");
		FIRE_PROPERTYCHANGE(ch, "DataRate");
		return (EPHIDGET_OK);
	case BP_SPATIALALGDATA:
		ch->quaternion.x = getBridgePacketDoubleArray(bp, 0)[0];
		ch->quaternion.y = getBridgePacketDoubleArray(bp, 0)[1];
		ch->quaternion.z = getBridgePacketDoubleArray(bp, 0)[2];
		ch->quaternion.w = getBridgePacketDoubleArray(bp, 0)[3];

		ch->quaternionValid = 1;
		ch->eulerAnglesValid = 1;

		FIRECH(ch, AlgorithmData, getBridgePacketDoubleArray(bp, 0), getBridgePacketDouble(bp, 1));
		return (EPHIDGET_OK);
	}

	return (_bridgeInput(phid, bp));
}

API_PRETURN
PhidgetSpatial_getEulerAngles(PhidgetSpatialHandle ch, PhidgetSpatial_SpatialEulerAngles *eulerAngles) {
	double heading_rad;
	double pitch_rad;
	double roll_rad;

	TESTPTR_PR(ch);
	TESTPTR_PR(eulerAngles);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1042_SPATIAL_300:
	case PHIDCHUID_1044_SPATIAL_400:
	case PHIDCHUID_1056_SPATIAL_000:
	case PHIDCHUID_1056_SPATIAL_200:
	case PHIDCHUID_MOT1101_SPATIAL_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	if (ch->eulerAnglesValid == PFALSE)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));

	roll_rad = atan2(2.0 * (ch->quaternion.x * ch->quaternion.w + ch->quaternion.y * ch->quaternion.z), 1 - 2.0 * (ch->quaternion.x * ch->quaternion.x + ch->quaternion.y * ch->quaternion.y));
	pitch_rad = asin(2.0 * (ch->quaternion.w * ch->quaternion.y - ch->quaternion.z * ch->quaternion.x));
	heading_rad = atan2(2.0 * (ch->quaternion.w * ch->quaternion.z + ch->quaternion.x * ch->quaternion.y), 1 - 2.0 * (ch->quaternion.y * ch->quaternion.y + ch->quaternion.z * ch->quaternion.z));

	// Convert radians to degrees
	ch->eulerAngles.heading = heading_rad * 180.0 / 3.14159265358979;
	ch->eulerAngles.pitch = pitch_rad * 180.0 / 3.14159265358979;
	ch->eulerAngles.roll = roll_rad * 180.0 / 3.14159265358979;

	*eulerAngles = ch->eulerAngles;

	return (EPHIDGET_OK);
}

static void
PhidgetSpatial_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetSpatial_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
