/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

#include "device/firmwareupgradedevice.h"
static void CCONV PhidgetFirmwareUpgrade_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static void CCONV PhidgetFirmwareUpgrade_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetFirmwareUpgrade_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetFirmwareUpgrade_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetFirmwareUpgrade_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetFirmwareUpgrade_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetFirmwareUpgrade_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetFirmwareUpgrade_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetFirmwareUpgrade_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetFirmwareUpgrade {
	struct _PhidgetChannel phid;
	Phidget_DeviceID actualDeviceID;
	const char * actualDeviceName;
	const char * actualDeviceSKU;
	int actualDeviceVersion;
	uint32_t actualDeviceVINTID;
	double progress;
	PhidgetFirmwareUpgrade_OnProgressChangeCallback ProgressChange;
	void *ProgressChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetFirmwareUpgradeHandle ch;
	int version;

	ch = (PhidgetFirmwareUpgradeHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 1) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 1 - functionality may be limited.", phid, version);
	}

	if (version >= 0)
		ch->actualDeviceID = getBridgePacketInt32ByName(bp, "actualDeviceID");
	if (version >= 0)
		ch->actualDeviceName = getBridgePacketStringByName(bp, "actualDeviceName");
	if (version >= 1)
		ch->actualDeviceSKU = getBridgePacketStringByName(bp, "actualDeviceSKU");
	if (version >= 0)
		ch->actualDeviceVersion = getBridgePacketInt32ByName(bp, "actualDeviceVersion");
	if (version >= 1)
		ch->actualDeviceVINTID = getBridgePacketUInt32ByName(bp, "actualDeviceVINTID");
	if (version >= 0)
		ch->progress = getBridgePacketDoubleByName(bp, "progress");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetFirmwareUpgradeHandle ch;

	ch = (PhidgetFirmwareUpgradeHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",actualDeviceID=%d"
	  ",actualDeviceName=%s"
	  ",actualDeviceSKU=%s"
	  ",actualDeviceVersion=%d"
	  ",actualDeviceVINTID=%u"
	  ",progress=%g"
	  ,1 /* class version */
	  ,ch->actualDeviceID
	  ,ch->actualDeviceName
	  ,ch->actualDeviceSKU
	  ,ch->actualDeviceVersion
	  ,ch->actualDeviceVINTID
	  ,ch->progress
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetFirmwareUpgradeHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetFirmwareUpgradeHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SENDFIRMWARE:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_PROGRESSCHANGE:
		ch->progress = getBridgePacketDouble(bp, 0);
		FIRECH(ch, ProgressChange, ch->progress);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetFirmwareUpgradeHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetFirmwareUpgradeHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
		ch->progress = 0;
		break;
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
		ch->progress = 0;
		break;
	case PHIDCHUID_STM32F0_FIRMWARE_UPGRADE_100:
		ch->actualDeviceID = PUNK_ENUM;
		ch->actualDeviceVINTID = PUNK_UINT32;
		ch->actualDeviceSKU = NULL;
		ch->actualDeviceVersion = PUNK_INT32;
		ch->actualDeviceName = NULL;
		ch->progress = 0;
		break;
	case PHIDCHUID_STM32G0_FIRMWARE_UPGRADE_110:
		ch->actualDeviceID = PUNK_ENUM;
		ch->actualDeviceVINTID = PUNK_UINT32;
		ch->actualDeviceSKU = NULL;
		ch->actualDeviceVersion = PUNK_INT32;
		ch->actualDeviceName = NULL;
		ch->progress = 0;
		break;
	case PHIDCHUID_STM8S_FIRMWARE_UPGRADE_100:
		ch->actualDeviceID = PUNK_ENUM;
		ch->actualDeviceVINTID = PUNK_UINT32;
		ch->actualDeviceSKU = NULL;
		ch->actualDeviceVersion = PUNK_INT32;
		ch->actualDeviceName = NULL;
		ch->progress = 0;
		break;
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		ch->progress = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
		break;
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
		break;
	case PHIDCHUID_STM32F0_FIRMWARE_UPGRADE_100:
		break;
	case PHIDCHUID_STM32G0_FIRMWARE_UPGRADE_110:
		break;
	case PHIDCHUID_STM8S_FIRMWARE_UPGRADE_100:
		break;
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {
	PhidgetFirmwareUpgradeHandle ch;

	ch = (PhidgetFirmwareUpgradeHandle)phid;

	if(ch->progress != PUNK_DBL)
		FIRECH(ch, ProgressChange, ch->progress);

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetFirmwareUpgradeHandle ch;

	ch = (PhidgetFirmwareUpgradeHandle)phid;

	if(ch->progress == PUNK_DBL)
		return (PFALSE);

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetFirmwareUpgrade));
}

static PhidgetReturnCode CCONV
_create(PhidgetFirmwareUpgradeHandle *phidp) {

	CHANNELCREATE_BODY(FirmwareUpgrade, PHIDCHCLASS_FIRMWAREUPGRADE);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFirmwareUpgrade_delete(PhidgetFirmwareUpgradeHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetFirmwareUpgrade_getActualDeviceID(PhidgetFirmwareUpgradeHandle ch,
  Phidget_DeviceID *actualDeviceID) {

	TESTPTR_PR(ch);
	TESTPTR_PR(actualDeviceID);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_FIRMWAREUPGRADE);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*actualDeviceID = ch->actualDeviceID;
	if (ch->actualDeviceID == (Phidget_DeviceID)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFirmwareUpgrade_getActualDeviceName(PhidgetFirmwareUpgradeHandle ch,
  const char **actualDeviceName) {

	TESTPTR_PR(ch);
	TESTPTR_PR(actualDeviceName);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_FIRMWAREUPGRADE);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*actualDeviceName = ch->actualDeviceName;
	if (ch->actualDeviceName == (char *)NULL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFirmwareUpgrade_getActualDeviceSKU(PhidgetFirmwareUpgradeHandle ch,
  const char **actualDeviceSKU) {

	TESTPTR_PR(ch);
	TESTPTR_PR(actualDeviceSKU);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_FIRMWAREUPGRADE);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*actualDeviceSKU = ch->actualDeviceSKU;
	if (ch->actualDeviceSKU == (char *)NULL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFirmwareUpgrade_getActualDeviceVersion(PhidgetFirmwareUpgradeHandle ch,
  int *actualDeviceVersion) {

	TESTPTR_PR(ch);
	TESTPTR_PR(actualDeviceVersion);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_FIRMWAREUPGRADE);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*actualDeviceVersion = ch->actualDeviceVersion;
	if (ch->actualDeviceVersion == (int)PUNK_INT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFirmwareUpgrade_getActualDeviceVINTID(PhidgetFirmwareUpgradeHandle ch,
  uint32_t *actualDeviceVINTID) {

	TESTPTR_PR(ch);
	TESTPTR_PR(actualDeviceVINTID);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_FIRMWAREUPGRADE);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_M3_USB_FIRMWARE_UPGRADE_000:
	case PHIDCHUID_STM32_USB_FIRMWARE_UPGRADE_100:
	case PHIDCHUID_M3_SPI_FIRMWARE_UPGRADE_000:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*actualDeviceVINTID = ch->actualDeviceVINTID;
	if (ch->actualDeviceVINTID == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFirmwareUpgrade_getProgress(PhidgetFirmwareUpgradeHandle ch, double *progress) {

	TESTPTR_PR(ch);
	TESTPTR_PR(progress);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_FIRMWAREUPGRADE);
	TESTATTACHED_PR(ch);

	*progress = ch->progress;
	if (ch->progress == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFirmwareUpgrade_setOnProgressChangeHandler(PhidgetFirmwareUpgradeHandle ch,
  PhidgetFirmwareUpgrade_OnProgressChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_FIRMWAREUPGRADE);

	ch->ProgressChange = fptr;
	ch->ProgressChangeCtx = ctx;

	return (EPHIDGET_OK);
}
