/* Generated: Wed Jun 22 2016 14:15:21 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "util/dataadaptersupport.h"
#include "class/dataadapter.gen.h"
#include "class/dataadapter.gen.c"

// Access the PhidgetIRSupport struct via the channel private pointer
#define DATAADAPTER_SUPPORT(ch) ((PhidgetDataAdapterSupportHandle)(((PhidgetChannelHandle)(ch))->private))

//Define the generalized sendPacketWaitResponse for use with and without EOL termination
API_PRETURN sendPacketWaitResponse(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length, uint8_t *recvData, size_t *recvDataLen,
	PhidgetDataAdapter_PacketErrorCode* error, bridgepacket_t packetType);
API_PRETURN sendPacket(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length, bridgepacket_t packetType);


static int _PhidgetDataAdapter_isEOLDefault(PhidgetDataAdapterHandle ch);
static const char *_PhidgetDataAdapter_getEOLString(PhidgetDataAdapterHandle ch);

static void CCONV
PhidgetDataAdapter_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static void CCONV
PhidgetDataAdapter_free(PhidgetChannelHandle *ch) {

	if (ch && *ch)
		PhidgetDataAdapterSupport_free((PhidgetDataAdapterSupportHandle *)&(*ch)->private);
	_free(ch);
}

API_PRETURN
PhidgetDataAdapter_create(PhidgetDataAdapterHandle *phidp) {
	PhidgetReturnCode res;

	res = _create(phidp);
	if (res == EPHIDGET_OK)
		res = PhidgetDataAdapterSupport_create((PhidgetDataAdapterSupportHandle *)&(*phidp)->phid.private);

	return (res);
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode res;
	const char* tmpString;
	int version;

	res = _setStatus(phid, bp);
	if (res != EPHIDGET_OK)
		return (res);

	ch = (PhidgetDataAdapterHandle)phid;
	version = getBridgePacketUInt32ByName(bp, "_class_version_");

	if (version >= 3) {
		tmpString = getBridgePacketStringByName(bp, "I2CFormat");
		mos_strncpy(ch->I2CFormat, tmpString, sizeof(ch->I2CFormat) - 1);
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode res;

	res = _getStatus(phid, bp);
	if (res != EPHIDGET_OK)
		return (res);

	ch = (PhidgetDataAdapterHandle)phid;
	res = addBridgePacketString(*bp, ch->I2CFormat, "I2CFormat");
	if (res != EPHIDGET_OK)
		return (res);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetDataAdapterSupport_init(DATAADAPTER_SUPPORT(phid));
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetDataAdapter_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDataAdapter_Protocol protocol;
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode res;
	uint32_t dataLen;
	Phidget_DeviceID deviceID;
	Phidget_DeviceClass deviceClass;
	PhidgetDataAdapter_PacketErrorCode err = PACKET_ERROR_OK;
	const char* eolString;

	ch = (PhidgetDataAdapterHandle)phid;

	switch (bp->vpkt) {
	case BP_DATAIN:
		dataLen = getBridgePacketArrayCnt(bp, 0);


		mos_mutex_lock(&(DATAADAPTER_SUPPORT(phid)->receiveLock));
		if (ch->lastDataLen == PUNK_SIZE) {
			ch->lastDataLen = 0;
		}

		if (dataLen > 8192) {
			dataLen = 8192;
			err = PACKET_ERROR_OVERRUN;
		}

		memcpy(ch->eventData, getBridgePacketUInt8Array(bp, 0), dataLen);
		ch->eventData[dataLen] = 0; //null terminate
		ch->eventDataLen = dataLen;

		if (dataLen > 0) {
			if ((ch->lastDataIndex + dataLen) < 8192) {
				memcpy(&ch->lastData[ch->lastDataIndex], getBridgePacketUInt8Array(bp, 0), dataLen);
				ch->lastDataIndex += dataLen;
			}
			else {
				int overhang = (ch->lastDataIndex + dataLen) % 8192;
				memcpy(&ch->lastData[ch->lastDataIndex], getBridgePacketUInt8Array(bp, 0), dataLen - overhang);
				memcpy(ch->lastData, &(getBridgePacketUInt8Array(bp, 0)[(dataLen - overhang)]), overhang);
				ch->lastDataIndex = overhang;
			}

			ch->lastDataLen += dataLen;
			if (ch->lastDataLen > 8192) {
				ch->lastDataLen = 8192;
				ch->lastDataError = PACKET_ERROR_OVERRUN;
			}
		}

		ch->newDataAvailable = 1;
		if (err == PACKET_ERROR_OK) {
			err = getBridgePacketUInt32(bp, 1);
		}
		if (err == PACKET_ERROR_OK) {
			if(getBridgePacketUInt32(bp, 2))
				err = PACKET_ERROR_CORRUPT;
		}

		ch->eventDataError = getBridgePacketUInt32(bp, 1);

		if (err) {
			ch->lastDataError = err;
		}

		PhidgetLock(ch);
		ch->responseID = getBridgePacketUInt16(bp, 3);
		PhidgetBroadcast(ch);
		PhidgetUnlock(ch);

		if(dataLen != 0 || err)
			FIRECH(ch, Packet, ch->eventData, dataLen, err);
		mos_mutex_unlock(&DATAADAPTER_SUPPORT(phid)->receiveLock);
		res = EPHIDGET_OK;
		break;
	case BP_SETPROTOCOL:
		protocol = (PhidgetDataAdapter_Protocol)getBridgePacketInt32(bp, 0);
		if (!supportedDataAdapterProtocol(phid, protocol))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Specified Protocol is unsupported by this device."));
		switch (protocol) {
		case PROTOCOL_I2C:
			ch->maxSendPacketLength = 512;
			ch->maxReceivePacketLength = 512;
			ch->maxSendWaitPacketLength = 512;
			ch->maxBaudRate = 400000;
			ch->minBaudRate = 10000;
			ch->minDataBits = 8;
			ch->maxDataBits = 8;
			ch->dataBits = 8;
			break;
		case PROTOCOL_SPI:
			ch->maxSendPacketLength = 512;
			ch->maxReceivePacketLength = 512;
			ch->maxSendWaitPacketLength = 512;
			ch->maxBaudRate = 1500000;
			ch->minBaudRate = 187500;
			ch->minDataBits = 4;
			ch->maxDataBits = 8;
			ch->dataBits = 8;
			break;
		case PROTOCOL_RS422:
		case PROTOCOL_RS485:
		case PROTOCOL_UART:
			Phidget_getDeviceClass((PhidgetHandle)ch, &deviceClass);
			Phidget_getDeviceID((PhidgetHandle)ch, &deviceID);
			if (deviceClass == PHIDCLASS_VINT) {
				ch->maxSendPacketLength = 512;
			}
			else { //USB
				ch->maxSendPacketLength = 10000000;
			}
			ch->maxReceivePacketLength = 8192;
			ch->maxSendWaitPacketLength = 1024;
			ch->maxBaudRate = 2500000;
			ch->minBaudRate = 800;
			ch->minDataBits = 7;
			ch->maxDataBits = 8;
			ch->dataBits = 8;
			break;
		case PROTOCOL_DMX512:
			ch->maxSendPacketLength = 513;
			ch->maxReceivePacketLength = 0;
			ch->maxSendWaitPacketLength = 513;
			ch->maxBaudRate = 250000;
			ch->baudRate = 250000;
			ch->minBaudRate = 250000;
			ch->minDataBits = 8;
			ch->maxDataBits = 8;
			ch->dataBits = 8;
			ch->stopBits = STOP_BITS_TWO;
			break;
		case PROTOCOL_MODBUS_RTU:
			ch->maxSendPacketLength = 256;
			ch->maxReceivePacketLength = 256;
			ch->maxSendWaitPacketLength = 256;
			ch->maxBaudRate = 2500000;
			ch->minBaudRate = 800;
			ch->minDataBits = 8;
			ch->maxDataBits = 8;
			ch->dataBits = 8;
			ch->stopBits = STOP_BITS_ONE;
			break;
		}
		res = _bridgeInput(phid, bp);
		break;
	case BP_SETTXTIMEOUT: //Transmit Timeout has a special case for -1
		if(getBridgePacketUInt32(bp, 0) != 0)
			TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minTransmitTimeout, ch->maxTransmitTimeout);

		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			break;
		}
		ch->transmitTimeout = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "TransmitTimeout");
		break;
	case BP_SETENDOFLINE:
		eolString = getBridgePacketString(bp, 0);
		if (strlen(eolString) > 8)
			return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "EOL Length must be <= 8"));
		strcpy(ch->endOfLine, eolString);
		FIRE_PROPERTYCHANGE(ch, "EndOfLine");
		res = _bridgeInput(phid, bp);
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	if (bp->vpkt == BP_SETBAUDRATE && res == EPHIDGET_OK) {
		uint32_t baudRate = ch->baudRate;
		switch (ch->protocol) {
		case PROTOCOL_I2C:
			if (baudRate >= 400000)
				ch->baudRate = 400000;
			else if (baudRate >= 100000)
				ch->baudRate = 100000;
			else
				ch->baudRate = 10000;
			break;
		case PROTOCOL_SPI:
			if (baudRate >= 1500000)
				ch->baudRate = 1500000;
			else if (baudRate >= 750000)
				ch->baudRate = 750000;
			else if (baudRate >= 375000)
				ch->baudRate = 375000;
			else
				ch->baudRate = 187500;
			break;
		}
	}

	return (res);
}

static void
PhidgetDataAdapter_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetDataAdapter_hasInitialState(PhidgetChannelHandle phid) {
	return (_hasInitialState(phid));
}

API_PRETURN
PhidgetDataAdapter_sendPacket(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length) {
	PhidgetReturnCode res = EPHIDGET_OK;
	uint32_t  maxBridgeLength;
	uint32_t i;

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);

	if (length == 0)
		return EPHIDGET_OK;

	if (ch->protocol == PUNK_ENUM)
		return (PHID_RETURN_ERRSTR(EPHIDGET_NOTCONFIGURED, "Protocol needs to be set before packets can be sent."));

	mos_mutex_lock(&DATAADAPTER_SUPPORT(ch)->sendLock);
	if (length > ch->maxSendPacketLength) {
		mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->sendLock);
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Packet length too long."));
	}
	//Break the packet into smaller chunks if the baud rate is low enough to avoid timing out the bridge packet
	if (ch->baudRate > 6400)
		maxBridgeLength = 8192;
	else if (ch->baudRate > 3200)
		maxBridgeLength = 4096;
	else if (ch->baudRate > 1600)
		maxBridgeLength = 2048;
	else
		maxBridgeLength = 1024;

	for (i = 0; i < (uint32_t)length; i += maxBridgeLength) {
		uint32_t tmpLength = ((maxBridgeLength <= (length - i)) ? maxBridgeLength : (length % maxBridgeLength));
		res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DATAOUT, NULL, NULL, "%*R", tmpLength, &data[i]); //break long packets into bridge-packet sized chunks
		if (res != EPHIDGET_OK) {
			mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->sendLock);
			if (res == EPHIDGET_TIMEOUT)
				return (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "The packet timed out while waiting to be transmitted. Check that your system can keep up with the data being sent."));
			return res;
		}
	}
	mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->sendLock);
	if (res == EPHIDGET_TIMEOUT)
		return (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "The packet timed out while waiting to be transmitted. Check that your system can keep up with the data being sent."));
	return res;
}

API_VRETURN
PhidgetDataAdapter_sendPacket_async(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length,
  Phidget_AsyncCallback fptr, void *ctx) {
	PhidgetReturnCode res = EPHIDGET_OK;
//	uint32_t maxBridgeLength;
//	uint32_t i;

	if (ch == NULL) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (ch->phid.class != PHIDCHCLASS_DATAADAPTER) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
		return;
	}
	if (!ISATTACHED(ch)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
		return;
	}
	if (ch->protocol == PUNK_ENUM) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTCONFIGURED);
		return;
	}
	if (length > ch->maxSendPacketLength) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}

	//Could error out, but technically we did send 0 bytes
	if (length == 0)
		return;

	//A packet must be able to fit fully in the memory buffer of the device to be accepted to be sent async
	if ((length > ch->maxSendWaitPacketLength) || (length > BPE_MAXARRAY_LEN)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}

	res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DATAOUT, fptr, ctx, "%*R", length, data); //break long packets into bridge-packet sized chunks

	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}

API_PRETURN
PhidgetDataAdapter_getLastData(PhidgetDataAdapterHandle ch, uint8_t *data, size_t *length, PhidgetDataAdapter_PacketErrorCode *error) {
	TESTPTR_PR(ch);
	TESTPTR_PR(data);
	TESTPTR_PR(length);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);
	PhidgetDataAdapter_PacketErrorCode err = 0;

	mos_mutex_lock(&DATAADAPTER_SUPPORT(ch)->receiveLock);

	if (ch->lastDataLen == PUNK_SIZE) {
		mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
		return (EPHIDGET_UNKNOWNVAL);
	}

	size_t dataLen = ch->lastDataLen;
	if (*length < ch->lastDataLen) {
		dataLen = *length;
		err = PACKET_ERROR_OVERRUN;
	}

	size_t lastDataStartIndex = ch->lastDataIndex - dataLen;
	lastDataStartIndex %= 8192;

	if ((lastDataStartIndex + dataLen) < 8192) {
		memcpy(data, &ch->lastData[lastDataStartIndex], dataLen);
	} else {
		int overhang = (lastDataStartIndex + dataLen) % 8192;
		memcpy(data, &ch->lastData[lastDataStartIndex], dataLen - overhang);
		memcpy(&data[dataLen - overhang], ch->lastData, overhang);
	}

	*length = dataLen;
	if (ch->lastDataError)
		*error = ch->lastDataError;
	else
		*error = err;

	ch->lastDataLen -= (uint32_t)dataLen;
	if(ch->lastDataLen == 0)
		ch->newDataAvailable = 0;
	ch->lastDataError = 0;
	mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
	return (EPHIDGET_OK);
}


API_PRETURN
PhidgetDataAdapter_sendPacketWaitResponse(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length, uint8_t *recvData, size_t *recvDataLen, PhidgetDataAdapter_PacketErrorCode* error) {

	PhidgetReturnCode res;
	TESTPTR_PR(ch);
	TESTPTR_PR(data);
	TESTPTR_PR(recvData);
	TESTPTR_PR(recvDataLen);
	TESTPTR_PR(error);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);
	uint8_t response[2];
	size_t responseLen = 2;
	uint16_t packetID;
	mostime_t duration;
	mostime_t start;

	if (length == 0)
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "The packet being sent must be longer than 0 bytes."));

	if (ch->protocol == PUNK_ENUM)
		return (PHID_RETURN_ERRSTR(EPHIDGET_NOTCONFIGURED, "Protocol needs to be set before packets can be sent."));

	if ((uint32_t)length > ch->maxSendWaitPacketLength) {
		*recvDataLen = 0;
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Packet length too long."));
	}

	/*if (milliseconds < ch->responseTimeout) {
	*recvDataLen = 0;
	return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Timeout cannot be less than the response timeout set for the device."));
	}*/

	//if (milliseconds)
	start = mos_gettime_usec();
	mos_mutex_lock(&DATAADAPTER_SUPPORT(ch)->sendLock);

	duration = (mos_gettime_usec() - start) / 1000;
	if (duration >= (ch->responseTimeout)) {
		mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->sendLock);
		*recvDataLen = 0;
		return (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "Timed out before data could send. Other Send Data functions could be holding this one up."));
	}

	do { //retry if the packet is rejected until this function times out
		res = bridgeSendToDeviceWithReply((PhidgetChannelHandle)ch, BP_DATAEXCHANGE, NULL, NULL, (uint8_t *)response, (uint32_t *)&responseLen, "%*R", length, data);
		if (res != EPHIDGET_OK) {
			duration = (mos_gettime_usec() - start) / 1000;
			if (duration >= ch->responseTimeout) {
				mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->sendLock);
				*recvDataLen = 0;
				return (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "The packet timed out while waiting to be transmitted. Check that your system can keep up with the data being sent."));
			}
		}
		if (responseLen == 0)
			MOS_PANIC("The bridge packet was lost");
	} while (res == EPHIDGET_INTERRUPTED);

	if (res) {
		mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->sendLock);
		*recvDataLen = 0;
		if (res == EPHIDGET_TIMEOUT)
			return (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "The packet timed out while waiting to be transmitted. Check that your system can keep up with the data being sent."));
		return res;
	}

	packetID = (((uint16_t)response[0]) << 8);
	packetID |= response[1];
	PhidgetLock(ch);
	for (;;) {
		if (ch->responseID == packetID) {
			break;
		}

		if (ch->responseTimeout) {
			duration = (mos_gettime_usec() - start) / 1000;
			if (duration >= ch->responseTimeout) {

				*recvDataLen = 0;
				mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->sendLock);
				PhidgetUnlock(ch);
				return (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "Timed out before a response was received. Consider increasing the Milliseconds parameter."));
			}


			PhidgetTimedWait(ch, ch->responseTimeout - (uint32_t)duration);

		}
	}
	PhidgetUnlock(ch);

	mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->sendLock);
	mos_mutex_lock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
	if (*recvDataLen < ch->eventDataLen) {
		mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
		*recvDataLen = 0;
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Receive array length too short."));
	}

	//Check if we can null terminate
	if (*recvDataLen >= (ch->eventDataLen + 1))
		recvData[ch->eventDataLen] = 0;

	memcpy(recvData, ch->eventData, ch->eventDataLen);
	*recvDataLen = ch->eventDataLen;
	*error = ch->eventDataError;

	mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);

	return (res);
}

API_PRETURN
PhidgetDataAdapter_writeLineWaitResponse(PhidgetDataAdapterHandle ch, const char* data, char* recvData, size_t* recvDataLen, PhidgetDataAdapter_PacketErrorCode* error) {
	const char *endOfLineString = _PhidgetDataAdapter_getEOLString(ch);
	size_t eollen = strlen(endOfLineString);
	char responseStr[8193] = { 0 };
	size_t datalen = strlen(data);
	size_t responseLength;
	PhidgetReturnCode res;
	char outstr[1024];
	char* eolPtr;

	if ((strlen(data) + eollen) >= ch->maxSendWaitPacketLength) {
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Packet length + EOL length too long."));
	}

	strcpy(outstr, data);
	strcat(outstr, endOfLineString);

	res = PhidgetDataAdapter_sendPacketWaitResponse(ch, (const uint8_t*)outstr, (datalen + eollen), (uint8_t*)recvData, recvDataLen, error);
	if (res)
		return res;

	mos_mutex_lock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
	//For safety, truncate recvDataLen to 8192, even though it should already be <= 8192
	if (*recvDataLen > 8192)
		*recvDataLen = 8192;

	//Make a copy into a chunk of memory we can guarantee will result in the response string being null-terminated
	memcpy(responseStr, recvData, *recvDataLen);

	if (_PhidgetDataAdapter_isEOLDefault(ch) == 0) {
		eolPtr = strstr(responseStr, endOfLineString);
	}
	else {
		eolPtr = strstr(responseStr, "\r\n");
		if (eolPtr == NULL)
			eolPtr = strstr(responseStr, "\n\r");
		if (eolPtr == NULL)
			eolPtr = strstr(responseStr, "\r");
		if (eolPtr == NULL)
			eolPtr = strstr(responseStr, "\n");
	}

	//If the EOL terminator is not found
	if (eolPtr == NULL) {
		if (*recvDataLen != 0)
			recvData[0] = '\0';
		*recvDataLen = 0;
		mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
		return EPHIDGET_OK;
	}

	responseLength = eolPtr - responseStr;

	if (*recvDataLen <= responseLength) {
		if (*recvDataLen != 0)
			recvData[0] = '\0';
		mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
		*recvDataLen = 0;
		return (PHID_RETURN_ERRSTR(EEPHIDGET_OUTOFRANGE, "Response length plus a null terminator is longer than the provided buffer"));
	}

	*recvDataLen = responseLength;
	recvData[responseLength] = '\0';

	mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
	return res;
}


API_PRETURN
PhidgetDataAdapter_write(PhidgetDataAdapterHandle ch, const char* data) {

	return PhidgetDataAdapter_sendPacket(ch, (const uint8_t *)data, strlen(data));
}


API_PRETURN
PhidgetDataAdapter_writeLine(PhidgetDataAdapterHandle ch, const char* data) {
	const char *endOfLineString = _PhidgetDataAdapter_getEOLString(ch);
	size_t datalen = strlen(data);
	size_t eollen = strlen(endOfLineString);
	char outstr[1024];

	if ((strlen(data) + strlen(endOfLineString)) > ch->maxSendWaitPacketLength) {
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Packet length + EOL length too long."));
	}

	strcpy(outstr, data);
	strcat(outstr, endOfLineString);

	return PhidgetDataAdapter_sendPacket(ch, (const uint8_t *)outstr, datalen + eollen);
}


API_VRETURN
PhidgetDataAdapter_write_async(PhidgetDataAdapterHandle ch, const char *data, Phidget_AsyncCallback fptr, void *ctx) {

	if (strlen(data) > ch->maxSendWaitPacketLength) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
	}

	PhidgetDataAdapter_sendPacket_async(ch, (const uint8_t *)data, strlen(data), fptr, ctx);
}

API_VRETURN
PhidgetDataAdapter_writeLine_async(PhidgetDataAdapterHandle ch, const char *data, Phidget_AsyncCallback fptr, void *ctx) {
	const char *endOfLineString = _PhidgetDataAdapter_getEOLString(ch);
	size_t dataLen = strlen(data);
	size_t eolLen = strlen(endOfLineString);
	char outstr[1024];
	if ((dataLen + eolLen) > ch->maxSendWaitPacketLength) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
	}

	strcpy(outstr, data);
	strcat(outstr, endOfLineString);

	PhidgetDataAdapter_sendPacket_async(ch, (const uint8_t*)outstr, (dataLen + eolLen), fptr, ctx);
}

API_PRETURN
PhidgetDataAdapter_readLine(PhidgetDataAdapterHandle ch, char *data, size_t *length) {
	TESTPTR_PR(ch);
	TESTPTR_PR(data);
	TESTPTR_PR(length);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED_PR(ch);
	PhidgetDataAdapter_PacketErrorCode err = 0;
	PhidgetReturnCode res = EPHIDGET_OK;

	const char *endOfLineString = _PhidgetDataAdapter_getEOLString(ch);
	char receivedData[8193] = {0};
	size_t lastDataStartIndex;
	size_t receivedDataLen;
	size_t dataLen;
	size_t eolLen;
	char* eolPtr = NULL;
	mostime_t duration;
	mostime_t start;


	start = mos_gettime_usec();

	mos_mutex_lock(&(DATAADAPTER_SUPPORT(ch))->receiveLock);

	if (ch->lastDataLen == PUNK_SIZE) {
		mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
		return (EPHIDGET_UNKNOWNVAL);
	}


	do {
		dataLen = ch->lastDataLen;

		lastDataStartIndex = ch->lastDataIndex - dataLen;
		lastDataStartIndex %= 8192;

		if ((lastDataStartIndex + dataLen) < 8192) {
			memcpy(receivedData, &ch->lastData[lastDataStartIndex], dataLen);
		} else {
			int overhang = (lastDataStartIndex + dataLen) % 8192;
			memcpy(receivedData, &ch->lastData[lastDataStartIndex], dataLen - overhang);
			memcpy(&receivedData[dataLen - overhang], ch->lastData, overhang);
		}

		if (_PhidgetDataAdapter_isEOLDefault(ch) == 0) {
			eolPtr = strstr(receivedData, endOfLineString);
		}
		else {
			eolPtr = strstr(receivedData, "\r\n");
			if (eolPtr == NULL)
				eolPtr = strstr(receivedData, "\n\r");
			if (eolPtr == NULL)
				eolPtr = strstr(receivedData, "\r");
			if (eolPtr == NULL)
				eolPtr = strstr(receivedData, "\n");
		}

		//If an error is flagged, but it doesn't affect the actual data coming it, don't flag it as an issue
		if (ch->lastDataError && (ch->lastDataLen == 0)) {
			ch->lastDataError = 0;
		}

		if (eolPtr == NULL) {
			if (strlen(receivedData) < dataLen) {
				//truncate past null terminators that would stop output until the input buffer overflowed
				ch->lastDataLen -= (uint32_t)strlen(receivedData) + 1;
				if (ch->lastDataError == PACKET_ERROR_OK) {
					ch->lastDataError = PACKET_ERROR_CORRUPT;
					res = (PHID_RETURN_ERRSTR(EPHIDGET_UNKNOWNVAL, "One or more bytes in the receive buffer is corrupt. Please clear the buffer by calling getLastData."));
					break;
				}
			}

			duration = (mos_gettime_usec() - start) / 1000;
			if (duration >= (ch->responseTimeout)) {
				res = (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "Timed out before an end of line sequence was detected."));
				break;
			}
			PhidgetLock(ch);
			PhidgetTimedWait(ch, ch->responseTimeout - (uint32_t)duration);
			PhidgetUnlock(ch);
		} else {
			break;
		}
		if (ch->lastDataError) {
			//If an error is flagged, but it doesn't affect the actual data coming in, don't flag it as an issue
			if (ch->lastDataLen == 0) {

			}
			else {
				err = ch->lastDataError;
			}
		}
	}while (ch->lastDataError == PACKET_ERROR_OK && eolPtr == NULL);

	if (res == EPHIDGET_OK && eolPtr != NULL) {

		receivedDataLen = eolPtr - receivedData;

		if (*length <= receivedDataLen) {
			if (*length != 0)
				data[0] = '\0';
			*length = 0;
			mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
			return (PHID_RETURN_ERRSTR(EPHIDGET_UNKNOWNVAL, "Your receive string was not long enough to fit the entire response."));
		}

		memcpy(data, receivedData, receivedDataLen);
		data[receivedDataLen] = '\0';
		*length = receivedDataLen;


		*length = dataLen;

		//We don't have a marker for where the error occurred, so flag everything with an error
		//until the buffer is cleared
		if (ch->lastDataError) {
			//If an error is flagged, but it doesn't affect the actual data coming it, don't flag it as an issue
			if (ch->lastDataLen == 0) {

			}
			else {
				err = ch->lastDataError;
			}
		}
		eolLen = strlen(endOfLineString);

		if ((receivedDataLen + eolLen) >= ch->lastDataLen) {
			ch->newDataAvailable = 0;
			ch->lastDataLen = 0;
			ch->lastDataError = 0;
		}
		else {
			ch->lastDataLen -= (uint32_t)(receivedDataLen + eolLen);
		}
	}
	else {
		//mark the string as having no length
		if (*length != 0)
			data[0] = '\0';
		*length = 0;
		if (res) {
			mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
			return res;
		}
	}

	switch (err) {
	case PACKET_ERROR_OK:
		res = (EPHIDGET_OK);
		break;
	case PACKET_ERROR_CORRUPT:
		res = (PHID_RETURN_ERRSTR(EPHIDGET_UNKNOWNVAL, "One or more bytes in the receive buffer is corrupt. Please clear the buffer by calling getLastData."));
		break;
	case PACKET_ERROR_FORMAT:
		res = (PHID_RETURN_ERRSTR(EPHIDGET_UNKNOWNVAL, "One or more bytes in the receive buffer is corrupt. Please clear the buffer by calling getLastData."));
		break;
	case PACKET_ERROR_INVALID:
		res = (PHID_RETURN_ERRSTR(EPHIDGET_UNKNOWNVAL, "One or more bytes in the receive buffer is corrupt. Please clear the buffer by calling getLastData."));
		break;
	case PACKET_ERROR_OVERRUN:
		res = (PHID_RETURN_ERRSTR(EPHIDGET_UNKNOWNVAL, "Some received data has been overwritten. Prevent this error by polling more often."));
		ch->lastDataError = 0;
		break;
	case PACKET_ERROR_TIMEOUT:
		res = (PHID_RETURN_ERRSTR(EPHIDGET_TIMEOUT, "The response data has timed out."));
		ch->lastDataError = 0;
		break;
	case PACKET_ERROR_UNKNOWN:
	default:
		res = (PHID_RETURN_ERRSTR(EPHIDGET_UNKNOWNVAL, "One or more bytes in the receive buffer is corrupt. Please clear the buffer by calling getLastData."));
	}
	mos_mutex_unlock(&DATAADAPTER_SUPPORT(ch)->receiveLock);
	return res;

}

//Returns TRUE is EOL is default value
static int _PhidgetDataAdapter_isEOLDefault(PhidgetDataAdapterHandle ch) {

	if (ch->endOfLine[0] == '\0')
		return 1;
	return 0;
}

//Returns "\r\n" if EOL is default, returns ch->endOfLine otherwise
static const char *_PhidgetDataAdapter_getEOLString(PhidgetDataAdapterHandle ch) {

	if(_PhidgetDataAdapter_isEOLDefault(ch))
		return "\r\n";
	return ch->endOfLine;
}