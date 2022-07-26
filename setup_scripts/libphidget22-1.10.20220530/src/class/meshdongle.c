/* Generated: Tue Sep 20 2016 17:20:03 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/meshdongle.gen.h"
#include "class/meshdongle.gen.c"

static void CCONV
PhidgetMeshDongle_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetMeshDongle_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetMeshDongle_create(PhidgetMeshDongleHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetMeshDongle_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetMeshDongle_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetMeshDongle_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetMeshDongle_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetMeshDongle_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_bridgeInput(phid, bp));
}

static void CCONV
PhidgetMeshDongle_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetMeshDongle_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}
