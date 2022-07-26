/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

static void CCONV PhidgetPowerGuard_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetPowerGuard_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetPowerGuard_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetPowerGuard_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetPowerGuard_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetPowerGuard_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetPowerGuard_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetPowerGuard_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetPowerGuard_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetPowerGuard {
	struct _PhidgetChannel phid;
	uint32_t minFailsafeTime;
	uint32_t maxFailsafeTime;
	Phidget_FanMode fanMode;
	double overVoltage;
	double minOverVoltage;
	double maxOverVoltage;
	int powerEnabled;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetPowerGuardHandle ch;
	int version;

	ch = (PhidgetPowerGuardHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 1) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 1 - functionality may be limited.", phid, version);
	}

	if (version >= 1)
		ch->minFailsafeTime = getBridgePacketUInt32ByName(bp, "minFailsafeTime");
	if (version >= 1)
		ch->maxFailsafeTime = getBridgePacketUInt32ByName(bp, "maxFailsafeTime");
	if (version >= 0)
		ch->fanMode = getBridgePacketInt32ByName(bp, "fanMode");
	if (version >= 0)
		ch->overVoltage = getBridgePacketDoubleByName(bp, "overVoltage");
	if (version >= 0)
		ch->minOverVoltage = getBridgePacketDoubleByName(bp, "minOverVoltage");
	if (version >= 0)
		ch->maxOverVoltage = getBridgePacketDoubleByName(bp, "maxOverVoltage");
	if (version >= 0)
		ch->powerEnabled = getBridgePacketInt32ByName(bp, "powerEnabled");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetPowerGuardHandle ch;

	ch = (PhidgetPowerGuardHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",minFailsafeTime=%u"
	  ",maxFailsafeTime=%u"
	  ",fanMode=%d"
	  ",overVoltage=%g"
	  ",minOverVoltage=%g"
	  ",maxOverVoltage=%g"
	  ",powerEnabled=%d"
	  ,1 /* class version */
	  ,ch->minFailsafeTime
	  ,ch->maxFailsafeTime
	  ,ch->fanMode
	  ,ch->overVoltage
	  ,ch->minOverVoltage
	  ,ch->maxOverVoltage
	  ,ch->powerEnabled
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetPowerGuardHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetPowerGuardHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETFAILSAFETIME:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_FAILSAFERESET:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETFANMODE:
		if (!supportedFanMode(phid, (Phidget_FanMode)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified FanMode is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->fanMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "FanMode");
		}
		break;
	case BP_SETOVERVOLTAGE:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minOverVoltage,
		  ch->maxOverVoltage);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->overVoltage = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "OverVoltage");
		}
		break;
	case BP_SETENABLED:
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->powerEnabled = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "PowerEnabled");
		}
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetPowerGuardHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetPowerGuardHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		ch->powerEnabled = 0;
		ch->overVoltage = PUNK_DBL;
		ch->maxOverVoltage = 33;
		ch->minOverVoltage = 8;
		ch->fanMode = FAN_MODE_AUTO;
		break;
	case PHIDCHUID_SAF1000_POWERGUARD_110:
		ch->powerEnabled = 0;
		ch->overVoltage = PUNK_DBL;
		ch->maxOverVoltage = 33;
		ch->minOverVoltage = 8;
		ch->fanMode = FAN_MODE_AUTO;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetPowerGuardHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetPowerGuardHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->powerEnabled);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETFANMODE, NULL, NULL, "%d", ch->fanMode);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_SAF1000_POWERGUARD_110:
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->powerEnabled);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETFANMODE, NULL, NULL, "%d", ch->fanMode);
		if (ret != EPHIDGET_OK)
			break;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetPowerGuard));
}

static PhidgetReturnCode CCONV
_create(PhidgetPowerGuardHandle *phidp) {

	CHANNELCREATE_BODY(PowerGuard, PHIDCHCLASS_POWERGUARD);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_delete(PhidgetPowerGuardHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetPowerGuard_enableFailsafe(PhidgetPowerGuardHandle ch, uint32_t failsafeTime) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFAILSAFETIME, NULL, NULL, "%u",
	  failsafeTime);
}

API_PRETURN
PhidgetPowerGuard_resetFailsafe(PhidgetPowerGuardHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_FAILSAFERESET, NULL, NULL, NULL);
}

API_PRETURN
PhidgetPowerGuard_getMinFailsafeTime(PhidgetPowerGuardHandle ch, uint32_t *minFailsafeTime) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minFailsafeTime);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minFailsafeTime = ch->minFailsafeTime;
	if (ch->minFailsafeTime == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_getMaxFailsafeTime(PhidgetPowerGuardHandle ch, uint32_t *maxFailsafeTime) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxFailsafeTime);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxFailsafeTime = ch->maxFailsafeTime;
	if (ch->maxFailsafeTime == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_setFanMode(PhidgetPowerGuardHandle ch, Phidget_FanMode fanMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFANMODE, NULL, NULL, "%d", fanMode));
}

API_PRETURN
PhidgetPowerGuard_getFanMode(PhidgetPowerGuardHandle ch, Phidget_FanMode *fanMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(fanMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	*fanMode = ch->fanMode;
	if (ch->fanMode == (Phidget_FanMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_setOverVoltage(PhidgetPowerGuardHandle ch, double overVoltage) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETOVERVOLTAGE, NULL, NULL, "%g",
	  overVoltage));
}

API_PRETURN
PhidgetPowerGuard_getOverVoltage(PhidgetPowerGuardHandle ch, double *overVoltage) {

	TESTPTR_PR(ch);
	TESTPTR_PR(overVoltage);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	*overVoltage = ch->overVoltage;
	if (ch->overVoltage == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_getMinOverVoltage(PhidgetPowerGuardHandle ch, double *minOverVoltage) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minOverVoltage);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	*minOverVoltage = ch->minOverVoltage;
	if (ch->minOverVoltage == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_getMaxOverVoltage(PhidgetPowerGuardHandle ch, double *maxOverVoltage) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxOverVoltage);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	*maxOverVoltage = ch->maxOverVoltage;
	if (ch->maxOverVoltage == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_setPowerEnabled(PhidgetPowerGuardHandle ch, int powerEnabled) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENABLED, NULL, NULL, "%d",
	  powerEnabled));
}

API_PRETURN
PhidgetPowerGuard_getPowerEnabled(PhidgetPowerGuardHandle ch, int *powerEnabled) {

	TESTPTR_PR(ch);
	TESTPTR_PR(powerEnabled);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHED_PR(ch);

	*powerEnabled = ch->powerEnabled;
	if (ch->powerEnabled == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}
