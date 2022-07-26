/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/hub.gen.h"
#include "class/hub.gen.c"

static void
PhidgetHub_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {}

static void CCONV
PhidgetHub_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetHub_create(PhidgetHubHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetHub_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetHub_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetHub_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetHub_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetHub_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;

	switch (bp->vpkt) {
	case BP_SETFIRMWAREUPGRADEFLAG:
		TESTRANGE_IOP(bp->iop, "%d", getBridgePacketInt32(bp, 0), 0, phid->parent->dev_hub.numVintPorts);
		if (getBridgePacketUInt32(bp, 1) > 0xFFFF)
			return (EPHIDGET_INVALIDARG);
		res = _bridgeInput(phid, bp);
		break;
	case BP_SETPORTMODE:
		TESTRANGE_IOP(bp->iop, "%d", getBridgePacketInt32(bp, 0), 0, phid->parent->dev_hub.numVintPorts);
		if (!supportedPortMode(phid, (PhidgetHub_PortMode)getBridgePacketInt32(bp, 1)))
			return (EPHIDGET_INVALIDARG);
		res = _bridgeInput(phid, bp);
		break;
	case BP_SETPORTPOWER:
		TESTRANGE_IOP(bp->iop, "%d", getBridgePacketInt32(bp, 0), 0, phid->parent->dev_hub.numVintPorts);
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 1));
		res = _bridgeInput(phid, bp);
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetHub_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetHub_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetHub_getPortMode(PhidgetHubHandle ch, int port, PhidgetHub_PortMode *mode) {
	PhidgetHubDeviceHandle parentHub;
	PhidgetReturnCode res;

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUB);
	TESTATTACHEDORDETACHING_PR(ch);
	TESTRANGE_PR(port, "%d", 0, ((PhidgetChannelHandle)ch)->parent->dev_hub.numVintPorts);

	res = Phidget_getHub((PhidgetHandle)ch, (PhidgetHandle*)&parentHub);
	if (res != EPHIDGET_OK)
		return (res);

	PhidgetHubDevice_updatePortProperties(parentHub, port);

	*mode = parentHub->portMode[port];

	PhidgetRelease(&parentHub);

	if (*mode == (PhidgetHub_PortMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHub_getPortPower(PhidgetHubHandle ch, int port, int *state) {
	PhidgetHubDeviceHandle parentHub;
	PhidgetReturnCode res;

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_HUB);
	TESTATTACHEDORDETACHING_PR(ch);
	TESTRANGE_PR(port, "%d", 0, ((PhidgetChannelHandle)ch)->parent->dev_hub.numVintPorts);

	res = Phidget_getHub((PhidgetHandle)ch, (PhidgetHandle*)&parentHub);
	if (res != EPHIDGET_OK)
		return (res);

	PhidgetHubDevice_updatePortProperties(parentHub, port);

	*state = parentHub->portPowered[port];

	PhidgetRelease(&parentHub);

	if (*state == PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}