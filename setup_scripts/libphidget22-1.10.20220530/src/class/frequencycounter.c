/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/frequencycounter.gen.h"
#include "class/frequencycounter.gen.c"

static void
PhidgetFrequencyCounter_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {}

static void CCONV
PhidgetFrequencyCounter_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetFrequencyCounter_create(PhidgetFrequencyCounterHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetFrequencyCounter_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetFrequencyCounter_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetFrequencyCounter_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetFrequencyCounterHandle ch;
	PhidgetReturnCode res;

	TESTPTR(phid);
	ch = (PhidgetFrequencyCounterHandle)phid;

	res = _initAfterOpen(phid);

	ch->totalTicksSinceLastCount = PUNK_DBL;
	ch->frequencyPrecision = 2;

	return (res);
}

static PhidgetReturnCode CCONV
PhidgetFrequencyCounter_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static void
handleFrequencyData(PhidgetFrequencyCounterHandle phid, double ticks, int counts, double ticksAtLastCount) {
	double countTimeSpan;
	double cutoffTime;
	double frequency;

	PhidgetRunLock(phid);
	phid->timeElapsed += ticks;
	phid->count += counts;
	PhidgetRunUnlock(phid);

	if (counts == 0) {
		if (phid->totalTicksSinceLastCount == PUNK_DBL)
			return;
		phid->totalTicksSinceLastCount += ticks;

		/* Frequency has already been cut off */
		if (phid->frequency == 0)
			return;

		/* Check for frequency cutoff */
		cutoffTime = round(1000 / phid->frequencyCutoff); //in ms
		if (phid->totalTicksSinceLastCount > cutoffTime) {
			phid->frequency = 0;
			FIRECH(phid, FrequencyChange, 0);
		}

		return;
	}

	if (phid->totalTicksSinceLastCount == PUNK_DBL) {
		phid->totalTicksSinceLastCount = ticks - ticksAtLastCount;
		/* Initial count event */
		FIRECH(phid, CountChange, counts, ticksAtLastCount);
		return;
	}

	countTimeSpan = phid->totalTicksSinceLastCount + ticksAtLastCount; //in ms
	phid->totalTicksSinceLastCount = ticks - ticksAtLastCount;

	frequency = round_double((double)((double)counts / ((double)countTimeSpan / 1000.0)), phid->frequencyPrecision);

	if (frequency < phid->frequencyCutoff) {
		if (phid->frequency != 0) {
			phid->frequency = 0;
			FIRECH(phid, FrequencyChange, 0);
		}
	} else {
		phid->frequency = frequency;
		FIRECH(phid, FrequencyChange, phid->frequency);
	}

	FIRECH(phid, CountChange, counts, (double)countTimeSpan);
}

static PhidgetReturnCode
PhidgetFrequencyCounter_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetFrequencyCounterHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetFrequencyCounterHandle)phid;

	switch (bp->vpkt) {
	case BP_FREQUENCYDATA:
		handleFrequencyData(ch, getBridgePacketDouble(bp, 0), getBridgePacketUInt32(bp, 1), getBridgePacketDouble(bp, 2));
		res = EPHIDGET_OK;
		break;
	case BP_SETENABLED:
		res = _bridgeInput(phid, bp);
		if (res == EPHIDGET_OK && getBridgePacketInt32(bp, 0) == 0)
			ch->frequency = PUNK_DBL;
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetFrequencyCounter_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetFrequencyCounter_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetFrequencyCounter_setFrequencyCutoff(PhidgetFrequencyCounterHandle ch, double frequencyCutoff) {
	int precision;
	double d;

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHED_PR(ch);
	TESTRANGE_PR(frequencyCutoff, "%lf", ch->minFrequencyCutoff, ch->maxFrequencyCutoff);

	d = frequencyCutoff;
	precision = 2;
	while (d < 1) {
		precision++;
		d *= 10;
	}
	ch->frequencyPrecision = precision;
	ch->frequencyCutoff = frequencyCutoff;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_reset(PhidgetFrequencyCounterHandle phid) {

	TESTPTR_PR(phid);
	TESTCHANNELCLASS_PR(phid, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHED_PR(phid);

	PhidgetRunLock(phid);
	phid->count = 0;
	phid->timeElapsed = 0;
	PhidgetRunUnlock(phid);

	phid->frequency = PUNK_DBL;

	return (EPHIDGET_OK);
}
