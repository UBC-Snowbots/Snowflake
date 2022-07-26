/* Generated: Mon May 30 2022 10:16:30 GMT-0600 (Mountain Daylight Time) */

static void CCONV PhidgetDataAdapter_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetDataAdapter_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetDataAdapter_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDataAdapter_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDataAdapter_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetDataAdapter_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetDataAdapter_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetDataAdapter_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetDataAdapter_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetDataAdapter {
	struct _PhidgetChannel phid;
	uint16_t responseID;
	uint8_t lastData[8192];
	uint32_t lastDataIndex;
	uint8_t eventData[8193];
	uint32_t eventDataLen;
	uint32_t eventDataError;
	uint32_t lastDataLen;
	PhidgetDataAdapter_PacketErrorCode lastDataError;
	char I2CFormat[2049];
	int lastDataRead;
	char endOfLine[8];
	uint32_t baudRate;
	uint32_t minBaudRate;
	uint32_t maxBaudRate;
	uint32_t dataBits;
	uint32_t minDataBits;
	uint32_t maxDataBits;
	uint32_t deviceAddress;
	PhidgetDataAdapter_HandshakeMode handshakeMode;
	PhidgetDataAdapter_Endianness endianness;
	PhidgetDataAdapter_IOVoltage IOVoltage;
	int newDataAvailable;
	PhidgetDataAdapter_Parity parity;
	PhidgetDataAdapter_Protocol protocol;
	uint32_t maxReceivePacketLength;
	uint32_t responseTimeout;
	uint32_t minResponseTimeout;
	uint32_t maxResponseTimeout;
	uint32_t maxSendPacketLength;
	uint32_t maxSendWaitPacketLength;
	PhidgetDataAdapter_SPIMode SPIMode;
	PhidgetDataAdapter_StopBits stopBits;
	uint32_t transmitTimeout;
	uint32_t minTransmitTimeout;
	uint32_t maxTransmitTimeout;
	PhidgetDataAdapter_OnPacketCallback Packet;
	void *PacketCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDataAdapterHandle ch;
	int version;

	ch = (PhidgetDataAdapterHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 3) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 3 - functionality may be limited.", phid, version);
	}

	if(version >= 3)
		ch->responseID = getBridgePacketUInt16ByName(bp, "responseID");
	if(version >= 3)
		ch->lastDataIndex = getBridgePacketUInt32ByName(bp, "lastDataIndex");
	if(version >= 3)
		ch->eventDataLen = getBridgePacketUInt32ByName(bp, "eventDataLen");
	if(version >= 3)
		ch->eventDataError = getBridgePacketUInt32ByName(bp, "eventDataError");
	if(version >= 3)
		ch->lastDataLen = getBridgePacketUInt32ByName(bp, "lastDataLen");
	if(version >= 3)
		ch->lastDataRead = getBridgePacketInt32ByName(bp, "lastDataRead");
	if (version >= 3)
		ch->baudRate = getBridgePacketUInt32ByName(bp, "baudRate");
	if (version >= 3)
		ch->minBaudRate = getBridgePacketUInt32ByName(bp, "minBaudRate");
	if (version >= 3)
		ch->maxBaudRate = getBridgePacketUInt32ByName(bp, "maxBaudRate");
	if (version >= 3)
		ch->dataBits = getBridgePacketUInt32ByName(bp, "dataBits");
	if (version >= 3)
		ch->minDataBits = getBridgePacketUInt32ByName(bp, "minDataBits");
	if (version >= 3)
		ch->maxDataBits = getBridgePacketUInt32ByName(bp, "maxDataBits");
	if (version >= 3)
		ch->deviceAddress = getBridgePacketUInt32ByName(bp, "deviceAddress");
	if (version >= 3)
		ch->handshakeMode = getBridgePacketInt32ByName(bp, "handshakeMode");
	if (version >= 3)
		ch->endianness = getBridgePacketInt32ByName(bp, "endianness");
	if (version >= 3)
		ch->IOVoltage = getBridgePacketInt32ByName(bp, "IOVoltage");
	if (version >= 3)
		ch->newDataAvailable = getBridgePacketInt32ByName(bp, "newDataAvailable");
	if (version >= 3)
		ch->parity = getBridgePacketInt32ByName(bp, "parity");
	if (version >= 3)
		ch->protocol = getBridgePacketInt32ByName(bp, "protocol");
	if (version >= 3)
		ch->maxReceivePacketLength = getBridgePacketUInt32ByName(bp, "maxReceivePacketLength");
	if (version >= 3)
		ch->responseTimeout = getBridgePacketUInt32ByName(bp, "responseTimeout");
	if (version >= 3)
		ch->minResponseTimeout = getBridgePacketUInt32ByName(bp, "minResponseTimeout");
	if (version >= 3)
		ch->maxResponseTimeout = getBridgePacketUInt32ByName(bp, "maxResponseTimeout");
	if (version >= 3)
		ch->maxSendPacketLength = getBridgePacketUInt32ByName(bp, "maxSendPacketLength");
	if (version >= 3)
		ch->maxSendWaitPacketLength = getBridgePacketUInt32ByName(bp, "maxSendWaitPacketLength");
	if (version >= 3)
		ch->SPIMode = getBridgePacketInt32ByName(bp, "SPIMode");
	if (version >= 3)
		ch->stopBits = getBridgePacketInt32ByName(bp, "stopBits");
	if (version >= 3)
		ch->transmitTimeout = getBridgePacketUInt32ByName(bp, "transmitTimeout");
	if (version >= 3)
		ch->minTransmitTimeout = getBridgePacketUInt32ByName(bp, "minTransmitTimeout");
	if (version >= 3)
		ch->maxTransmitTimeout = getBridgePacketUInt32ByName(bp, "maxTransmitTimeout");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetDataAdapterHandle ch;

	ch = (PhidgetDataAdapterHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",responseID=uh"
	  ",lastDataIndex=%u"
	  ",eventDataLen=%u"
	  ",eventDataError=%u"
	  ",lastDataLen=%u"
	  ",lastDataRead=%d"
	  ",baudRate=%u"
	  ",minBaudRate=%u"
	  ",maxBaudRate=%u"
	  ",dataBits=%u"
	  ",minDataBits=%u"
	  ",maxDataBits=%u"
	  ",deviceAddress=%u"
	  ",handshakeMode=%d"
	  ",endianness=%d"
	  ",IOVoltage=%d"
	  ",newDataAvailable=%d"
	  ",parity=%d"
	  ",protocol=%d"
	  ",maxReceivePacketLength=%u"
	  ",responseTimeout=%u"
	  ",minResponseTimeout=%u"
	  ",maxResponseTimeout=%u"
	  ",maxSendPacketLength=%u"
	  ",maxSendWaitPacketLength=%u"
	  ",SPIMode=%d"
	  ",stopBits=%d"
	  ",transmitTimeout=%u"
	  ",minTransmitTimeout=%u"
	  ",maxTransmitTimeout=%u"
	  ,3 /* class version */
	  ,ch->responseID
	  ,ch->lastDataIndex
	  ,ch->eventDataLen
	  ,ch->eventDataError
	  ,ch->lastDataLen
	  ,ch->lastDataRead
	  ,ch->baudRate
	  ,ch->minBaudRate
	  ,ch->maxBaudRate
	  ,ch->dataBits
	  ,ch->minDataBits
	  ,ch->maxDataBits
	  ,ch->deviceAddress
	  ,ch->handshakeMode
	  ,ch->endianness
	  ,ch->IOVoltage
	  ,ch->newDataAvailable
	  ,ch->parity
	  ,ch->protocol
	  ,ch->maxReceivePacketLength
	  ,ch->responseTimeout
	  ,ch->minResponseTimeout
	  ,ch->maxResponseTimeout
	  ,ch->maxSendPacketLength
	  ,ch->maxSendWaitPacketLength
	  ,ch->SPIMode
	  ,ch->stopBits
	  ,ch->transmitTimeout
	  ,ch->minTransmitTimeout
	  ,ch->maxTransmitTimeout
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDataAdapterHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETI2CFORMAT:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETENDOFLINE:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DATAOUT:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_DATAEXCHANGE:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETBAUDRATE:
		TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minBaudRate,
		  ch->maxBaudRate);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->baudRate = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "BaudRate");
		}
		break;
	case BP_SETDATABITS:
		TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minDataBits,
		  ch->maxDataBits);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->dataBits = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "DataBits");
		}
		break;
	case BP_SETADDRESS:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->deviceAddress = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "DeviceAddress");
		}
		break;
	case BP_SETHANDSHAKEMODE:
		if (!supportedDataAdapterHandshakeMode(phid,
		  (PhidgetDataAdapter_HandshakeMode)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified DataAdapterHandshakeMode is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->handshakeMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "HandshakeMode");
		}
		break;
	case BP_SETENDIANNESS:
		if (!supportedDataAdapterEndianness(phid,
		  (PhidgetDataAdapter_Endianness)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified DataAdapterEndianness is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->endianness = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Endianness");
		}
		break;
	case BP_SETIOVOLTAGE:
		if (!supportedDataAdapterIOVoltage(phid, (PhidgetDataAdapter_IOVoltage)getBridgePacketInt32(bp,
		  0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified DataAdapterIOVoltage is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->IOVoltage = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "IOVoltage");
		}
		break;
	case BP_SETPARITY:
		if (!supportedDataAdapterParity(phid, (PhidgetDataAdapter_Parity)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified DataAdapterParity is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->parity = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Parity");
		}
		break;
	case BP_SETPROTOCOL:
		if (!supportedDataAdapterProtocol(phid, (PhidgetDataAdapter_Protocol)getBridgePacketInt32(bp,
		  0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified DataAdapterProtocol is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->protocol = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Protocol");
		}
		break;
	case BP_SETTIMEOUT:
		TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minResponseTimeout,
		  ch->maxResponseTimeout);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->responseTimeout = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "ResponseTimeout");
		}
		break;
	case BP_SETSPIMODE:
		if (!supportedDataAdapterSPIMode(phid, (PhidgetDataAdapter_SPIMode)getBridgePacketInt32(bp,
		  0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified DataAdapterSPIMode is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->SPIMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "SPIMode");
		}
		break;
	case BP_SETSTOPBITS:
		if (!supportedDataAdapterStopBits(phid, (PhidgetDataAdapter_StopBits)getBridgePacketInt32(bp,
		  0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified DataAdapterStopBits is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->stopBits = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "StopBits");
		}
		break;
	case BP_SETTXTIMEOUT:
		TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minTransmitTimeout,
		  ch->maxTransmitTimeout);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->transmitTimeout = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "TransmitTimeout");
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
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetDataAdapterHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	default:
		MOS_PANIC("Unsupported Channel");
	}

	ch->responseID = 0;
	memset(ch->lastData, 0, sizeof (uint8_t) * 8192);
	ch->lastDataIndex = 0;
	memset(ch->eventData, 0, sizeof (uint8_t) * 8193);
	ch->eventDataLen = 0;
	ch->eventDataError = 0;
	ch->lastDataLen = 0;
	ch->lastDataError = 0;
	memset(ch->I2CFormat, 0, sizeof (char) * 2049);
	ch->lastDataRead = 0;
	memset(ch->endOfLine, 0, sizeof (char) * 8);

	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
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

	mos_free(*ch, sizeof (struct _PhidgetDataAdapter));
}

static PhidgetReturnCode CCONV
_create(PhidgetDataAdapterHandle *phidp) {

	CHANNELCREATE_BODY(DataAdapter, PHIDCHCLASS_DATAADAPTER);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_delete(PhidgetDataAdapterHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetDataAdapter_setI2CFormat(PhidgetDataAdapterHandle ch, const char *format) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETI2CFORMAT, NULL, NULL, "%s", format);
}

API_PRETURN
PhidgetDataAdapter_setEndOfLine(PhidgetDataAdapterHandle ch, const char *endOfLine) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENDOFLINE, NULL, NULL, "%s", endOfLine);
}

API_PRETURN
PhidgetDataAdapter_setBaudRate(PhidgetDataAdapterHandle ch, uint32_t baudRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETBAUDRATE, NULL, NULL, "%u", baudRate));
}

API_PRETURN
PhidgetDataAdapter_getBaudRate(PhidgetDataAdapterHandle ch, uint32_t *baudRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(baudRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*baudRate = ch->baudRate;
	if (ch->baudRate == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMinBaudRate(PhidgetDataAdapterHandle ch, uint32_t *minBaudRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minBaudRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*minBaudRate = ch->minBaudRate;
	if (ch->minBaudRate == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxBaudRate(PhidgetDataAdapterHandle ch, uint32_t *maxBaudRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxBaudRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*maxBaudRate = ch->maxBaudRate;
	if (ch->maxBaudRate == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setDataBits(PhidgetDataAdapterHandle ch, uint32_t dataBits) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATABITS, NULL, NULL, "%u", dataBits));
}

API_PRETURN
PhidgetDataAdapter_getDataBits(PhidgetDataAdapterHandle ch, uint32_t *dataBits) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataBits);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*dataBits = ch->dataBits;
	if (ch->dataBits == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMinDataBits(PhidgetDataAdapterHandle ch, uint32_t *minDataBits) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataBits);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*minDataBits = ch->minDataBits;
	if (ch->minDataBits == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxDataBits(PhidgetDataAdapterHandle ch, uint32_t *maxDataBits) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataBits);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*maxDataBits = ch->maxDataBits;
	if (ch->maxDataBits == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setDeviceAddress(PhidgetDataAdapterHandle ch, uint32_t deviceAddress) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETADDRESS, NULL, NULL, "%u",
	  deviceAddress));
}

API_PRETURN
PhidgetDataAdapter_getDeviceAddress(PhidgetDataAdapterHandle ch, uint32_t *deviceAddress) {

	TESTPTR_PR(ch);
	TESTPTR_PR(deviceAddress);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*deviceAddress = ch->deviceAddress;
	if (ch->deviceAddress == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setHandshakeMode(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_HandshakeMode handshakeMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETHANDSHAKEMODE, NULL, NULL, "%d",
	  handshakeMode));
}

API_PRETURN
PhidgetDataAdapter_getHandshakeMode(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_HandshakeMode *handshakeMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(handshakeMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*handshakeMode = ch->handshakeMode;
	if (ch->handshakeMode == (PhidgetDataAdapter_HandshakeMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setEndianness(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_Endianness endianness) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENDIANNESS, NULL, NULL, "%d",
	  endianness));
}

API_PRETURN
PhidgetDataAdapter_getEndianness(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_Endianness *endianness) {

	TESTPTR_PR(ch);
	TESTPTR_PR(endianness);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*endianness = ch->endianness;
	if (ch->endianness == (PhidgetDataAdapter_Endianness)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setIOVoltage(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_IOVoltage IOVoltage) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETIOVOLTAGE, NULL, NULL, "%d",
	  IOVoltage));
}

API_PRETURN
PhidgetDataAdapter_getIOVoltage(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_IOVoltage *IOVoltage) {

	TESTPTR_PR(ch);
	TESTPTR_PR(IOVoltage);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*IOVoltage = ch->IOVoltage;
	if (ch->IOVoltage == (PhidgetDataAdapter_IOVoltage)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getNewDataAvailable(PhidgetDataAdapterHandle ch, int *newDataAvailable) {

	TESTPTR_PR(ch);
	TESTPTR_PR(newDataAvailable);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*newDataAvailable = ch->newDataAvailable;
	if (ch->newDataAvailable == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setParity(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_Parity parity) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETPARITY, NULL, NULL, "%d", parity));
}

API_PRETURN
PhidgetDataAdapter_getParity(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_Parity *parity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(parity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*parity = ch->parity;
	if (ch->parity == (PhidgetDataAdapter_Parity)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setProtocol(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_Protocol protocol) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETPROTOCOL, NULL, NULL, "%d", protocol));
}

API_PRETURN
PhidgetDataAdapter_getProtocol(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_Protocol *protocol) {

	TESTPTR_PR(ch);
	TESTPTR_PR(protocol);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*protocol = ch->protocol;
	if (ch->protocol == (PhidgetDataAdapter_Protocol)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxReceivePacketLength(PhidgetDataAdapterHandle ch,
  uint32_t *maxReceivePacketLength) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxReceivePacketLength);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*maxReceivePacketLength = ch->maxReceivePacketLength;
	if (ch->maxReceivePacketLength == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setResponseTimeout(PhidgetDataAdapterHandle ch, uint32_t responseTimeout) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETTIMEOUT, NULL, NULL, "%u",
	  responseTimeout));
}

API_PRETURN
PhidgetDataAdapter_getResponseTimeout(PhidgetDataAdapterHandle ch, uint32_t *responseTimeout) {

	TESTPTR_PR(ch);
	TESTPTR_PR(responseTimeout);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*responseTimeout = ch->responseTimeout;
	if (ch->responseTimeout == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMinResponseTimeout(PhidgetDataAdapterHandle ch, uint32_t *minResponseTimeout) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minResponseTimeout);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*minResponseTimeout = ch->minResponseTimeout;
	if (ch->minResponseTimeout == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxResponseTimeout(PhidgetDataAdapterHandle ch, uint32_t *maxResponseTimeout) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxResponseTimeout);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*maxResponseTimeout = ch->maxResponseTimeout;
	if (ch->maxResponseTimeout == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxSendPacketLength(PhidgetDataAdapterHandle ch, uint32_t *maxSendPacketLength) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxSendPacketLength);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*maxSendPacketLength = ch->maxSendPacketLength;
	if (ch->maxSendPacketLength == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxSendWaitPacketLength(PhidgetDataAdapterHandle ch,
  uint32_t *maxSendWaitPacketLength) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxSendWaitPacketLength);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*maxSendWaitPacketLength = ch->maxSendWaitPacketLength;
	if (ch->maxSendWaitPacketLength == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setSPIMode(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_SPIMode SPIMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSPIMODE, NULL, NULL, "%d", SPIMode));
}

API_PRETURN
PhidgetDataAdapter_getSPIMode(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_SPIMode *SPIMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(SPIMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*SPIMode = ch->SPIMode;
	if (ch->SPIMode == (PhidgetDataAdapter_SPIMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setStopBits(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_StopBits stopBits) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSTOPBITS, NULL, NULL, "%d", stopBits));
}

API_PRETURN
PhidgetDataAdapter_getStopBits(PhidgetDataAdapterHandle ch, PhidgetDataAdapter_StopBits *stopBits) {

	TESTPTR_PR(ch);
	TESTPTR_PR(stopBits);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*stopBits = ch->stopBits;
	if (ch->stopBits == (PhidgetDataAdapter_StopBits)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setTransmitTimeout(PhidgetDataAdapterHandle ch, uint32_t transmitTimeout) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETTXTIMEOUT, NULL, NULL, "%u",
	  transmitTimeout));
}

API_PRETURN
PhidgetDataAdapter_getTransmitTimeout(PhidgetDataAdapterHandle ch, uint32_t *transmitTimeout) {

	TESTPTR_PR(ch);
	TESTPTR_PR(transmitTimeout);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*transmitTimeout = ch->transmitTimeout;
	if (ch->transmitTimeout == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMinTransmitTimeout(PhidgetDataAdapterHandle ch, uint32_t *minTransmitTimeout) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minTransmitTimeout);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*minTransmitTimeout = ch->minTransmitTimeout;
	if (ch->minTransmitTimeout == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_getMaxTransmitTimeout(PhidgetDataAdapterHandle ch, uint32_t *maxTransmitTimeout) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxTransmitTimeout);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	*maxTransmitTimeout = ch->maxTransmitTimeout;
	if (ch->maxTransmitTimeout == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setOnPacketHandler(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_OnPacketCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);

	ch->Packet = fptr;
	ch->PacketCtx = ctx;

	return (EPHIDGET_OK);
}
