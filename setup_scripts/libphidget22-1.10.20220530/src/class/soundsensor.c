/* Generated: Wed Apr 20 2016 13:42:56 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/soundsensor.gen.h"
#include "class/soundsensor.gen.c"

void PhidgetSoundSensor_getLastDB(PhidgetChannelHandle, double *);
void PhidgetSoundSensor_setLastDB(PhidgetChannelHandle, double);

static void CCONV
PhidgetSoundSensor_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetSoundSensor_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetSoundSensor_create(PhidgetSoundSensorHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetSoundSensor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetSoundSensor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetSoundSensor_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetSoundSensor_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetSoundSensor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_bridgeInput(phid, bp));
}

static void
PhidgetSoundSensor_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetSoundSensor_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetSoundSensorHandle ch;
	ch = (PhidgetSoundSensorHandle)phid;

	if (ch->dB == PUNK_DBL ||
	  ch->dBA == PUNK_DBL ||
	  ch->dBC == PUNK_DBL)
		return (PFALSE);

	return (_hasInitialState(phid));
}

// NOTE: these are for use by vintpackets.c
void
PhidgetSoundSensor_getLastDB(PhidgetChannelHandle phid, double *lastDB) {
	PhidgetSoundSensorHandle ch;
	ch = (PhidgetSoundSensorHandle)phid;
	*lastDB = ch->lastdB;
}

void
PhidgetSoundSensor_setLastDB(PhidgetChannelHandle phid, double lastDB) {
	PhidgetSoundSensorHandle ch;
	ch = (PhidgetSoundSensorHandle)phid;
	ch->lastdB = lastDB;
}