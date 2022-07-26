/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "gpp.h"
#include "class/firmwareupgrade.gen.h"
#include "class/firmwareupgrade.gen.c"
#include "mos/mos_base64.h"

static const char* emptyStr = "";

static void
PhidgetFirmwareUpgrade_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetFirmwareUpgrade_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetFirmwareUpgrade_create(PhidgetFirmwareUpgradeHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetFirmwareUpgrade_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	const PhidgetUniqueDeviceDef *pdd;
	PhidgetFirmwareUpgradeHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetFirmwareUpgradeHandle)phid;

	res = _setStatus(phid, bp);
	if (res != EPHIDGET_OK)
		return (res);

	// The strings will not be valid as they are pointing at the bridge packet
	ch->actualDeviceSKU = emptyStr;
	ch->actualDeviceName = emptyStr;
	// Try to recover
	if (isVintChannel(phid)) {
		//Look up the unique device structure
		pdd = Phidget_Unique_Device_Def;
		while (((int)pdd->type) != END_OF_LIST) {
			if (pdd->type == PHIDTYPE_VINT
			  && ch->actualDeviceVINTID == pdd->vintID
			  && ch->actualDeviceID == pdd->id
			  && ((ch->actualDeviceVersion >= pdd->versionLow && ch->actualDeviceVersion < pdd->versionHigh) || ch->actualDeviceVersion == 0)) {
				ch->actualDeviceSKU = pdd->SKU;
				ch->actualDeviceName = pdd->name;
				break;
			}
			pdd++;
		}
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetFirmwareUpgrade_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetFirmwareUpgrade_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetFirmwareUpgrade_setDefaults(PhidgetChannelHandle phid) {
	PhidgetFirmwareUpgradeHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetFirmwareUpgradeHandle)phid;

	res = _setDefaults(phid);

	// Strings should not be NULL, it messes up getStatus
	ch->actualDeviceSKU = emptyStr;
	ch->actualDeviceName = emptyStr;

	return (res);
}

static PhidgetReturnCode
PhidgetFirmwareUpgrade_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	const PhidgetUniqueDeviceDef *pdd;
	PhidgetFirmwareUpgradeHandle ch;
	int vintId;

	ch = (PhidgetFirmwareUpgradeHandle)phid;

	switch (bp->vpkt) {
	case BP_DEVICEINFO:
		vintId = getBridgePacketInt32(bp, 0);
		ch->actualDeviceVersion = getBridgePacketInt32(bp, 1);

		//Look up the unique device structure
		pdd = Phidget_Unique_Device_Def;
		while (((int)pdd->type) != END_OF_LIST) {
			if (pdd->type == PHIDTYPE_VINT
			  && vintId == pdd->vintID
			  && ((ch->actualDeviceVersion >= pdd->versionLow && ch->actualDeviceVersion < pdd->versionHigh) || ch->actualDeviceVersion == 0)) {
				ch->actualDeviceSKU = pdd->SKU;
				ch->actualDeviceName = pdd->name;
				ch->actualDeviceVINTID = pdd->vintID;
				ch->actualDeviceID = pdd->id;
				break;
			}
			pdd++;
		}
		return (EPHIDGET_OK);
	default:
		return (_bridgeInput(phid, bp));
	}
}

static void
PhidgetFirmwareUpgrade_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetFirmwareUpgrade_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetFirmwareUpgradeHandle ch;

	ch = (PhidgetFirmwareUpgradeHandle)phid;

	if (isVintChannel(phid) && ch->actualDeviceVINTID == PUNK_UINT32)
		return (PFALSE);

	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetFirmwareUpgrade_sendFirmware(PhidgetFirmwareUpgradeHandle ch, const uint8_t *data, size_t dataLen) {
	PhidgetReturnCode res;
	BridgePacket *bp;

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_FIRMWAREUPGRADE);
	TESTATTACHED_PR(ch);

	res = createBridgePacket(&bp, BP_SENDFIRMWARE, NULL);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	res = bridgePacketBase64Encode(bp, data, dataLen);
	if (res != EPHIDGET_OK) {
		destroyBridgePacket(&bp);
		return (PHID_RETURN(res));
	}

	return (bridgeSendBPToDevice((PhidgetChannelHandle)ch, NULL, NULL, bp));
}
