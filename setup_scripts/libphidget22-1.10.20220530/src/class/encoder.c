/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/encoder.gen.h"
#include "class/encoder.gen.c"

static void
PhidgetEncoder_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetEncoder_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetEncoder_create(PhidgetEncoderHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetEncoder_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetEncoder_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetEncoder_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetEncoder_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetEncoder_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetEncoderHandle ch;
	PhidgetReturnCode res;
	int64_t indexPosition;
	double timeChange;
	int indexTriggered;
	int positionChange;

	ch = (PhidgetEncoderHandle)phid;

	switch (bp->vpkt) {
	case BP_POSITIONCHANGE:
		positionChange = getBridgePacketInt32(bp, 0);
		timeChange = getBridgePacketDouble(bp, 1);
		indexTriggered = getBridgePacketUInt8(bp, 2);
		PhidgetRunLock(phid);
		if (indexTriggered) {
			indexPosition = getBridgePacketInt32(bp, 3);
			ch->indexPosition = ch->position + indexPosition;
		}
		ch->position += positionChange;
		PhidgetRunUnlock(phid);
		FIRECH(ch, PositionChange, positionChange, timeChange, indexTriggered);
		res = EPHIDGET_OK;
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetEncoder_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetEncoder_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}


API_PRETURN
PhidgetEncoder_setPosition(PhidgetEncoderHandle ch, int64_t position) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_ENCODER);
	TESTATTACHED_PR(ch);

	PhidgetRunLock(ch);
	if (ch->indexPosition != PUNK_INT64)
		ch->indexPosition += (position - ch->position);
	ch->position = position;
	PhidgetRunUnlock(ch);

	return (EPHIDGET_OK);
}