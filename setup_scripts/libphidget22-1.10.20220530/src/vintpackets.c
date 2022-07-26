#include "phidgetbase.h"
#include "devices.gen.h"
#include "vintpackets.h"

#include "class/lcd.gen.h"
#include "class/stepper.gen.h"
#include "class/soundsensor.gen.h"
#include "class/dcmotor.gen.h"
#include "class/bldcmotor.gen.h"
#include "class/motorpositioncontroller.gen.h"

#include "util/irsupport.h"
#include "util/dataadaptersupport.h"
#include "util/rfidsupport.h"

#include "mos/mos_base64.h"

double PhidgetBLDCMotor_getLastBrakingStrength(PhidgetBLDCMotorHandle);
int64_t PhidgetBLDCMotor_getLastPosition(PhidgetBLDCMotorHandle);
double PhidgetDCMotor_getLastBrakingStrength(PhidgetDCMotorHandle);
int64_t PhidgetMotorPositionController_getLastPosition(PhidgetMotorPositionControllerHandle);
void PhidgetSoundSensor_getLastDB(PhidgetChannelHandle, double *);
void PhidgetSoundSensor_setLastDB(PhidgetChannelHandle, double);

static PhidgetReturnCode
_sendDEFAULT(PhidgetChannelHandle ch, BridgePacket *bp) {

	MOS_PANIC("Unsupported Device (not vint?)");
}

static PhidgetReturnCode
_recvDEFAULT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	MOS_PANIC("Unsupported Device (not vint?)");
}

#if PHIDUID_GENERICVINT_SUPPORTED
static PhidgetReturnCode
sendGENERICVINT(PhidgetChannelHandle ch, BridgePacket *bp) {
	const uint8_t *buf;

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_VINT_GENERIC:
		switch (bp->vpkt) {
		case BP_SENDPACKET:
			buf = getBridgePacketUInt8Array(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, buf[0], buf + 1, getBridgePacketArrayLen(bp, 0) - 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvGENERICVINT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	assert(buf);

	switch (ch->UCD->uid) {
	case PHIDCHUID_VINT_GENERIC:
		return (bridgeSendToChannel(ch, BP_PACKET, "%*R", (int)len, buf));

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_GENERICVINT_SUPPORTED */

static PhidgetReturnCode
sendUNKNOWNVINT(PhidgetChannelHandle ch, BridgePacket *bp) {
	const uint8_t *buf;

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_VINT_UNKNOWN:
		switch (bp->vpkt) {
		case BP_SENDPACKET:
			buf = getBridgePacketUInt8Array(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, buf[0], buf + 1, getBridgePacketArrayLen(bp, 0) - 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvUNKNOWNVINT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	assert(buf);

	switch (ch->UCD->uid) {
	case PHIDCHUID_VINT_UNKNOWN:
		return (bridgeSendToChannel(ch, BP_PACKET, "%*R", (int)len, buf));

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendFirmware(mosiop_t iop, PhidgetChannelHandle ch, const uint8_t *data, size_t dataLen) {
	unsigned char buffer[34];
	PhidgetReturnCode res;
	size_t offset;
	double progress, lastProgress;

	// Send the data
	offset = 0;
	bridgeSendToChannel(ch, BP_PROGRESSCHANGE, "%g", 0.0);
	lastProgress = 0;
	do {
		buffer[0] = offset & 0xFF;
		buffer[1] = (offset >> 8) & 0xFF;

		memcpy(buffer + 2, data + offset, 32);

		res = sendVINTDataPacket(iop, ch, VINT_BOOTLOADER_PACKET_TYPE_FIRMWAREUPGRADEDATA, buffer, 34);

		if (res != EPHIDGET_OK) {
			logerr("Error during upgrade (sending data): "PRC_FMT, PRC_ARGS(res));
			return (MOS_ERROR(iop, res, "Error sending firmware upgrade data"));
		} else
			logdebug("Sent some firmware data: %d bytes at 0x%04zx", 32, offset);

		offset += 32;

		progress = (double)offset / (double)dataLen;
		if (progress - lastProgress >= 0.01) {
			bridgeSendToChannel(ch, BP_PROGRESSCHANGE, "%g", progress);
			lastProgress = progress;
		}

	} while (offset < dataLen);

	// Finish the upgrade - this commits the change
	res = sendVINTPacket(iop, ch, VINT_CMD_FIRMWARE_UPGRADE_DONE, (VINTPacketType)0, NULL, 0);

	if (lastProgress != 1)
		bridgeSendToChannel(ch, BP_PROGRESSCHANGE, "%g", 1.0);

	if (res != EPHIDGET_OK) {
		logerr("Error during upgrade (sending VINT_CMD_FIRMWARE_UPGRADE_DONE): "PRC_FMT, PRC_ARGS(res));
		return (MOS_ERROR(iop, res, "Error commiting firmware upgrade."));
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
sendFIRMWARE_UPGRADE_STM8S(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetReturnCode res;
	size_t base64Len;
	void *base64;
	int off;

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_STM8S_FIRMWARE_UPGRADE_100:
		switch (bp->vpkt) {
		case BP_SENDFIRMWARE:
			off = 0;
			res = bridgePacketBase64Decode(bp, NULL, &base64Len, &off);
			if (res != EPHIDGET_OK)
				return (res);

			base64 = mos_malloc(base64Len);
			res = bridgePacketBase64Decode(bp, base64, &base64Len, &off);
			if (res != EPHIDGET_OK)
				return (res);

			if (base64Len & 0x3F) {
				logerr("Firmware length must be a multiple of 64 bytes.");
				mos_free(base64, base64Len);
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Firmware length must be a multiple of 64 bytes."));
			}
			res = sendFirmware(bp->iop, ch, base64, base64Len);
			mos_free(base64, base64Len);
			return (MOS_ERROR(bp->iop, res, "Error sending firmware to device."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvFIRMWARE_UPGRADE_STM8S(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	Phidget_DeviceID deviceID;
	int version;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_STM8S_FIRMWARE_UPGRADE_100:
		switch (pkt) {
		case VINT_BOOTLOADER_PACKET_TYPE_DEVICEINFO:
			deviceID = (Phidget_DeviceID)(((unsigned short)buf[0] << 4) | ((buf[1] & 0xF0) >> 4));
			version = ((unsigned short)(buf[1] & 0x0F) * 100) | buf[2];
			return (bridgeSendToChannel(ch, BP_DEVICEINFO, "%d%d", deviceID, version));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendFIRMWARE_UPGRADE_STM32F0(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetReturnCode res;
	size_t base64Len;
	void *base64;
	int off;

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_STM32F0_FIRMWARE_UPGRADE_100:
		switch (bp->vpkt) {
		case BP_SENDFIRMWARE:
			off = 0;
			res = bridgePacketBase64Decode(bp, NULL, &base64Len, &off);
			if (res != EPHIDGET_OK)
				return (res);

			base64 = mos_malloc(base64Len);
			res = bridgePacketBase64Decode(bp, base64, &base64Len, &off);
			if (res != EPHIDGET_OK)
				return (res);

			if (base64Len & 0x1F) {
				logerr("Firmware length must be a multiple of 32 bytes.");
				mos_free(base64, base64Len);
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Firmware length must be a multiple of 32 bytes."));
			}
			res = sendFirmware(bp->iop, ch, base64, base64Len);
			mos_free(base64, base64Len);
			return (MOS_ERROR(bp->iop, res, "Error sending firmware to device."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvFIRMWARE_UPGRADE_STM32F0(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	Phidget_DeviceID deviceID;
	int version;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_STM32F0_FIRMWARE_UPGRADE_100:
		switch (pkt) {
		case VINT_BOOTLOADER_PACKET_TYPE_DEVICEINFO:
			deviceID = (Phidget_DeviceID)(((unsigned short)buf[0] << 4) | ((buf[1] & 0xF0) >> 4));
			version = ((unsigned short)(buf[1] & 0x0F) * 100) | buf[2];
			return (bridgeSendToChannel(ch, BP_DEVICEINFO, "%d%d", deviceID, version));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendFIRMWARE_UPGRADE_STM32G0(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetReturnCode res;
	size_t base64Len;
	void *base64;
	int off;

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_STM32G0_FIRMWARE_UPGRADE_110:
		switch (bp->vpkt) {
		case BP_SENDFIRMWARE:
			off = 0;
			res = bridgePacketBase64Decode(bp, NULL, &base64Len, &off);
			if (res != EPHIDGET_OK)
				return (res);

			base64 = mos_malloc(base64Len);
			res = bridgePacketBase64Decode(bp, base64, &base64Len, &off);
			if (res != EPHIDGET_OK)
				return (res);

			if (base64Len & 0x1F) {
				logerr("Firmware length must be a multiple of 32 bytes.");
				mos_free(base64, base64Len);
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Firmware length must be a multiple of 32 bytes."));
			}
			res = sendFirmware(bp->iop, ch, base64, base64Len);
			mos_free(base64, base64Len);
			return (MOS_ERROR(bp->iop, res, "Error sending firmware to device."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvFIRMWARE_UPGRADE_STM32G0(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	Phidget_DeviceID deviceID;
	int version;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_STM32G0_FIRMWARE_UPGRADE_110:
		switch (pkt) {
		case VINT_BOOTLOADER_PACKET_TYPE_DEVICEINFO:
			deviceID = (Phidget_DeviceID)(((unsigned short)buf[0] << 4) | ((buf[1] & 0xF0) >> 4));
			version = ((unsigned short)(buf[1] & 0x0F) * 100) | buf[2];
			return (bridgeSendToChannel(ch, BP_DEVICEINFO, "%d%d", deviceID, version));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_1014_3_VINT_SUPPORTED
static PhidgetReturnCode
send1014_3_VINT(PhidgetChannelHandle ch, BridgePacket *bp) {
	double dutyCycle;
	uint8_t buf[2];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_1014_DIGITALOUTPUT_800_VINT:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			buf[0] = (uint8_t)(getBridgePacketInt32(bp, 0) ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETDUTYCYCLE:
			dutyCycle = getBridgePacketDouble(bp, 0);
			if ((dutyCycle != 0) && (dutyCycle != 1)) {
				return EPHIDGET_INVALIDARG;
			}
			buf[0] = (uint8_t)(dutyCycle ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recv1014_3_VINT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_1014_DIGITALOUTPUT_800_VINT:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_1014_3_VINT_SUPPORTED */

#if PHIDUID_1017_2_VINT_SUPPORTED
static PhidgetReturnCode
send1017_2_VINT(PhidgetChannelHandle ch, BridgePacket *bp) {
	double dutyCycle;
	uint8_t buf[2];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_1017_DIGITALOUTPUT_200_VINT:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			buf[0] = (uint8_t)(getBridgePacketInt32(bp, 0) ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETDUTYCYCLE:
			dutyCycle = getBridgePacketDouble(bp, 0);
			if ((dutyCycle != 0) && (dutyCycle != 1)) {
				return EPHIDGET_INVALIDARG;
			}
			buf[0] = (uint8_t)(dutyCycle ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recv1017_2_VINT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_1017_DIGITALOUTPUT_200_VINT:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_1017_2_VINT_SUPPORTED */

#if PHIDUID_1024_V2_VINT_SUPPORTED
static PhidgetReturnCode
send1024_V2_VINT(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetReturnCode ret;
	uint8_t buffer[64];
	size_t stringLength;
	const char* tagString;
	PhidgetRFID_Protocol protocol;
	const char* expectedString;
	double dutyCycle;
	size_t len;
	int i;

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_1024_RFID_200_VINT:
		switch (bp->vpkt) {
		case BP_SETANTENNAON:
			buffer[0] = getBridgePacketInt32(bp, 0) != 0;
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RFID_ANTENNA_ON, buffer, 1);
		case BP_WRITE:
			protocol = getBridgePacketInt32(bp, 1);
			TESTRANGE_IOP(bp->iop, "%d", protocol, PROTOCOL_EM4100, PROTOCOL_PHIDGETS);
			buffer[0] = getBridgePacketInt32(bp, 1);
			buffer[1] = getBridgePacketInt32(bp, 2) != 0;

			tagString = getBridgePacketString(bp, 0);
			if (protocol == PROTOCOL_EM4100 && tagString[1] == 'x')
				tagString += 2;
			stringLength = strlen(tagString);
			if (stringLength > (MAX_OUT_PACKET_SIZE - 3))
				return EPHIDGET_INVALIDARG;
			memcpy((char*)&buffer[2], tagString, stringLength);
			buffer[2 + stringLength] = '\0';
			if (protocol == PROTOCOL_EM4100) {
				for (i = 0; i < 10; i++) {
					buffer[2+i] = mos_tolower(buffer[2 + i]);
				}
			}

			len = 2 + stringLength;

			PhidgetLock(ch);
			RFIDSupport_setLatestTagString(bp->iop, ch, ""); //clear latest tag
			ret = sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RFID_TAG_WRITE, buffer, len);
			if (ret != EPHIDGET_OK)
				return ret;
			PhidgetUnlock(ch);

			expectedString = (const char*)&buffer[2];
			return RFIDSupport_waitForTag(bp->iop, ch, expectedString, 600, NULL);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_1024_DIGITALOUTPUT_5V_200_VINT:
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_200_VINT:
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_200_VINT:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			len = 1;
			buffer[0] = getBridgePacketInt32(bp, 0) != 0;
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buffer, len);
		case BP_SETDUTYCYCLE:
			dutyCycle = getBridgePacketDouble(bp, 0);
			if (dutyCycle != 0.0 && dutyCycle != 1.0)
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Duty cycle must be 0 or 1."));
			len = 1;
			buffer[0] = dutyCycle != 0;
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buffer, len);
		case BP_OPENRESET:
		case BP_CLOSERESET:
			len = 1;
			buffer[0] = 0;
			return sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buffer, len);
		case BP_ENABLE:
			return EPHIDGET_OK;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recv1024_V2_VINT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_1024_RFID_200_VINT:
		switch (pkt) {
		case VINT_PACKET_TYPE_RFID_TAG:
			PhidgetLock(ch);
			RFIDSupport_setLatestTagString(NULL, ch, (const char*)&buf[1]);
			PhidgetUnlock(ch);
			return bridgeSendToChannel(ch, BP_TAG, "%s%d", &buf[1], buf[0]);
		case VINT_PACKET_TYPE_RFID_TAG_LOST:
			return bridgeSendToChannel(ch, BP_TAGLOST, "%s%d", &buf[1], buf[0]);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_1024_DIGITALOUTPUT_5V_200_VINT:
		switch (pkt) {

		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_1024_DIGITALOUTPUT_LED_200_VINT:
		switch (pkt) {

		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_200_VINT:
		switch (pkt) {

		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_1024_V2_VINT_SUPPORTED */

static PhidgetReturnCode
send1055_1_VINT(PhidgetChannelHandle ch, BridgePacket *bp) {

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_1055_IR_200_VINT:
		return (PhidgetIRSupport_bridgeInput(ch, bp));

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recv1055_1_VINT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	assert(buf);

	switch (ch->UCD->uid) {
	case PHIDCHUID_1055_IR_200_VINT:
		return (PhidgetIRSupport_dataInput(ch, buf, len));

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_ADP1001_VINT_SUPPORTED
static PhidgetReturnCode
sendADP1001_VINT(PhidgetChannelHandle ch, BridgePacket *bp) {
	double dutyCycle;
	uint8_t buf[1];
	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100_VINT:
		return (PhidgetDataAdapterSupport_bridgeInput(ch, bp));
	case PHIDCHUID_ADP1001_DIGITALOUTPUT_DTR_100_VINT:
		switch (bp->vpkt) {
		case BP_SETDUTYCYCLE:
			dutyCycle = getBridgePacketDouble(bp, 0);
			if (dutyCycle != 0 && dutyCycle != 1)
				return EPHIDGET_INVALIDARG;
			buf[0] = (uint8_t)(dutyCycle ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETSTATE:
			buf[0] = (uint8_t)(getBridgePacketInt32(bp, 0) ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvADP1001_VINT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int state;
	int pkt;

	assert(buf);

	switch (ch->UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100_VINT:
		switch (buf[0
		]) {
		case VINT_PACKET_TYPE_DATAADAPTER_VOLTAGE_ERROR:
			SEND_ERROR_EVENT(ch, EEPHIDGET_VOLTAGEERROR, "Voltage Error Detected");
			return (EPHIDGET_OK);
		default:
			return (PhidgetDataAdapterSupport_dataInput(ch, buf, len));
		}
	case PHIDCHUID_ADP1001_DIGITALINPUT_DSR_100_VINT:
	case PHIDCHUID_ADP1001_DIGITALINPUT_DCD_100_VINT:
	case PHIDCHUID_ADP1001_DIGITALINPUT_RI_100_VINT:
		pkt = buf[0];
		buf++;
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE:
			state = buf[0] ? 1 : 0;
			return (bridgeSendToChannel(ch, BP_STATECHANGE, "%d", state));
		case VINT_PACKET_TYPE_DIGITALINPUT_INVALID:
			SEND_ERROR_EVENT(ch, EEPHIDGET_INVALIDSTATE, "Invalid State Detected");
			return EPHIDGET_OK;
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_ADP1001_VINT_SUPPORTED */

#if PHIDUID_ADP_RS485_422_VINT_SUPPORTED
static PhidgetReturnCode
sendADP_RS485_422_VINT(PhidgetChannelHandle ch, BridgePacket *bp) {

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100_VINT:
		return (PhidgetDataAdapterSupport_bridgeInput(ch, bp));

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvADP_RS485_422_VINT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	assert(buf);

	switch (ch->UCD->uid) {
	case PHIDCHUID_ADP_RS485_422_DATAADAPTER_100_VINT:
		return (PhidgetDataAdapterSupport_dataInput(ch, buf, len));
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_ADP_RS485_422_VINT_SUPPORTED */

#if PHIDUID_ADP_SERIAL_VINT_SUPPORTED
static PhidgetReturnCode
sendADP_SERIAL_VINT(PhidgetChannelHandle ch, BridgePacket *bp) {
	double dutyCycle;
	uint8_t buf[1];
	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_ADP_SERIAL_DATAADAPTER_100_VINT:
		return (PhidgetDataAdapterSupport_bridgeInput(ch, bp));
	case PHIDCHUID_ADP_SERIAL_DIGITALOUTPUT_100_VINT:
		switch (bp->vpkt) {
		case BP_SETDUTYCYCLE:
			dutyCycle = getBridgePacketDouble(bp, 0);
			if (dutyCycle != 0 && dutyCycle != 1)
				return EPHIDGET_INVALIDARG;
			buf[0] = (uint8_t)(dutyCycle ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETSTATE:
			buf[0] = (uint8_t)(getBridgePacketInt32(bp, 0) ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvADP_SERIAL_VINT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int state;
	int pkt;
	assert(buf);

	switch (ch->UCD->uid) {
	case PHIDCHUID_ADP_SERIAL_DATAADAPTER_100_VINT:
		switch (buf[0]) {
		case VINT_PACKET_TYPE_DATAADAPTER_VOLTAGE_ERROR:
			SEND_ERROR_EVENT(ch, EEPHIDGET_VOLTAGEERROR, "Voltage Error Detected");
			return (EPHIDGET_OK);
		default:
			return (PhidgetDataAdapterSupport_dataInput(ch, buf, len));
		}
	case PHIDCHUID_ADP_SERIAL_DIGITALINPUT_100_VINT:
		pkt = buf[0];
		buf++;
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE:
			state = buf[0] ? 1 : 0;
			return (bridgeSendToChannel(ch, BP_STATECHANGE, "%d", state));
		case VINT_PACKET_TYPE_DIGITALINPUT_INVALID:
			SEND_ERROR_EVENT(ch, EEPHIDGET_INVALIDSTATE, "Channel Invalidated. This means some other aspect of the device is making use of the channel.");
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Invalid Channel UID");
		}
	case PHIDCHUID_ADP_SERIAL_DIGITALOUTPUT_100_VINT:
		switch (buf[0]) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_INVALID:
			SEND_ERROR_EVENT(ch, EEPHIDGET_INVALIDSTATE, "Channel Invalidated. This means some other aspect of the device is making use of the channel.");
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Invalid Channel UID");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_ADP_SERIAL_VINT_SUPPORTED */

static PhidgetReturnCode
sendVCP1002(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
#if PHIDUID_VCP1002_110_SUPPORTED
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_110:
#endif
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 24)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		case BP_SETVOLTAGERANGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			switch (buf[0]) {
			case VOLTAGE_RANGE_10mV:
			case VOLTAGE_RANGE_40mV:
			case VOLTAGE_RANGE_200mV:
			case VOLTAGE_RANGE_1000mV:
			case VOLTAGE_RANGE_AUTO:
				break;
			default:
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Invalid or unsupported voltage range for this device."));
			}
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_VCP1002RANGE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvVCP1002(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltage;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
#if PHIDUID_VCP1002_110_SUPPORTED
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_110:
#endif
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpack32xS(buf, 24), 7);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_VCP1002_110_SUPPORTED
static PhidgetReturnCode
sendVCP1002_110(PhidgetChannelHandle ch, BridgePacket *bp) {
	return sendVCP1002(ch, bp);
}

static PhidgetReturnCode
recvVCP1002_110(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	return recvVCP1002(ch, buf, len);
}
#endif /* PHIDUID_VCP1002_110_SUPPORTED */

static PhidgetReturnCode
sendVCP1001(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
#if PHIDUID_VCP1001_110_SUPPORTED
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_110:
#endif
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 24)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		case BP_SETVOLTAGERANGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			switch (buf[0]) {
			case VOLTAGE_RANGE_5V:
			case VOLTAGE_RANGE_15V:
			case VOLTAGE_RANGE_40V:
			case VOLTAGE_RANGE_AUTO:
				break;
			default:
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Invalid or unsupported voltage range for this device."));
			}
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_VCP1001RANGE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvVCP1001(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltage;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
#if PHIDUID_VCP1001_110_SUPPORTED
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_110:
#endif
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpack32xS(buf, 24), 4);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_VCP1001_110_SUPPORTED
static PhidgetReturnCode
sendVCP1001_110(PhidgetChannelHandle ch, BridgePacket *bp) {
	return sendVCP1001(ch, bp);
}

static PhidgetReturnCode
recvVCP1001_110(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	return recvVCP1001(ch, buf, len);
}
#endif /* PHIDUID_VCP1001_110_SUPPORTED */

static PhidgetReturnCode
sendVCP1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 24)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		case BP_SETVOLTAGERANGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			switch (buf[0]) {
			case VOLTAGE_RANGE_312_5mV:
			case VOLTAGE_RANGE_40V:
				break;
			default:
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Invalid or unsupported voltage range for this device."));
			}
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_VCP1000RANGE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvVCP1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltage;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpack32xS(buf, 24), 7);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_TMP1300_SUPPORTED
static PhidgetReturnCode
sendTMP1300(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_TMP1300_VOLTAGEINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvTMP1300(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature, voltage;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = unpackfloat(buf);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_TMP1300_VOLTAGEINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = unpackfloat(buf);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_TMP1300_SUPPORTED */

static PhidgetReturnCode
sendTMP1200(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RESISTANCEINPUT_SETRESISTANCECHANGETRIGGER, buf, 4));
		case BP_SETRTDWIRESETUP:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RTD_WIRESETUP, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		case BP_SETRTDWIRESETUP:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RTD_WIRESETUP, buf, 1));
		case BP_SETRTDTYPE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_RTDTYPE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvTMP1200(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature;
	double resistance;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_RESISTANCEINPUT_RESISTANCECHANGE:
			resistance = round_double(unpackfloat(buf), 3);
			return (bridgeSendToChannel(ch, BP_RESISTANCECHANGE, "%g", resistance));
		case VINT_PACKET_TYPE_RESISTANCEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpackfloat(buf), 3);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_TMP1200_1_SUPPORTED
static PhidgetReturnCode
sendTMP1200_1(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RESISTANCEINPUT_SETRESISTANCECHANGETRIGGER, buf, 4));
		case BP_SETRTDWIRESETUP:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RTD_WIRESETUP, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		case BP_SETRTDWIRESETUP:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RTD_WIRESETUP, buf, 1));
		case BP_SETRTDTYPE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_RTDTYPE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvTMP1200_1(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature;
	double resistance;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_RESISTANCEINPUT_RESISTANCECHANGE:
			resistance = round_double(unpackfloat(buf), 3);
			return (bridgeSendToChannel(ch, BP_RESISTANCECHANGE, "%g", resistance));
		case VINT_PACKET_TYPE_RESISTANCEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpackfloat(buf), 3);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_TMP1200_1_SUPPORTED */

static PhidgetReturnCode
sendTMP1101(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		case BP_SETTHERMOCOUPLETYPE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_THERMOCOUPLETYPE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvTMP1101(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature;
	double voltage;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpackfloat(buf), 6);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double((double)unpackfloat(buf), 3);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendTMP1101_1(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		case BP_SETTHERMOCOUPLETYPE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_THERMOCOUPLETYPE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvTMP1101_1(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature;
	double voltage;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpackfloat(buf), 6);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_BADCONNECTION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADCONNECTION, "Bad Connection"));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double((double)unpackfloat(buf), 3);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double((double)unpackfloat(buf), 3);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_BADCONNECTION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADCONNECTION, "Bad Connection"));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}


static PhidgetReturnCode
sendTMP1100(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		case BP_SETTHERMOCOUPLETYPE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_THERMOCOUPLETYPE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvTMP1100(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature;
	double voltage;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpackfloat(buf), 6);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double((double)unpackfloat(buf), 3);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendTMP1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, ((int)((getBridgePacketDouble(bp, 0)) * (1 << 16))));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvTMP1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendSTC1001(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1001_STEPPER_100:
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) /* (1 << 7)*/));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 7)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT, buf, 2));
		case BP_SETHOLDINGCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETENGAGED, buf, 1));
		case BP_SETCONTROLMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCONTROLMODE, buf, 1));
#if 0 // XXX - is this removed from the Firmware?
		case BP_SETFAILSAFEMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETFAILSAFEMODE, buf, 1));
		case BP_SETFAILURETIMEOUT:
			doubleToSignedFixedPoint(getBridgePacketInt32(bp, 0), buf, 0, 4, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETFAILURETIMEOUT, buf, 4));
#endif
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_STC1001_STEPPER_110:
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) /* (1 << 7)*/));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 7)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT, buf, 2));
		case BP_SETHOLDINGCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETENGAGED, buf, 1));
		case BP_SETCONTROLMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCONTROLMODE, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendSTC1001_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendSTC1001(ch, bp));
}

static PhidgetReturnCode
sendSTC1003(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1003_STEPPER_100:
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) /* (1 << 7)*/));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 7)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT, buf, 2));
		case BP_SETHOLDINGCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETENGAGED, buf, 1));
		case BP_SETCONTROLMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCONTROLMODE, buf, 1));
#if 0 // XXX - is this removed from the Firmware?
		case BP_SETFAILSAFEMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETFAILSAFEMODE, buf, 1));
		case BP_SETFAILURETIMEOUT:
			doubleToSignedFixedPoint(getBridgePacketInt32(bp, 0), buf, 0, 4, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETFAILURETIMEOUT, buf, 4));
#endif
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_STC1003_STEPPER_110:
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) /* (1 << 7)*/));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 7)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT, buf, 2));
		case BP_SETHOLDINGCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETENGAGED, buf, 1));
		case BP_SETCONTROLMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCONTROLMODE, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendSTC1003_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendSTC1003(ch, bp));
}

static PhidgetReturnCode
sendSTC1003_1(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1003_STEPPER_200:
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) /* (1 << 7)*/));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 7)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT, buf, 2));
		case BP_SETHOLDINGCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETENGAGED, buf, 1));
		case BP_SETCONTROLMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCONTROLMODE, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_STC1003_VOLTAGEINPUT_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvSTC1003_1(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int64_t motorPosition;
	double velocity;
	double voltage;
	int stopped;
	int moving;
	int pkt;
	PhidgetReturnCode returnCode;
	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1003_STEPPER_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_STEPPER_STATUSUPDATE:
			//Position in Q32.0 format (units are steps)
			motorPosition = unpack64(buf);
			//Velocity in Q32.0 format (units are steps/sec)
			velocity = (double)((int32_t)unpack32(buf + 8));
			velocity /= 256.0;
			//Stopped
			stopped = buf[12] ? 1 : 0;
			returnCode = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", motorPosition);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = bridgeSendToChannel(ch, BP_VELOCITYCHANGE, "%g", velocity);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = PhidgetStepper_getIsMoving((PhidgetStepperHandle)ch, &moving);
			if (returnCode == EPHIDGET_OK && stopped && moving)
				returnCode = bridgeSendToChannel(ch, BP_STOPPED, "");
			return returnCode;
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		case VINT_PACKET_TYPE_STEPPER_POWER_ERROR:
			if(buf[0])
				return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER,	"Supply Voltage above expected range"));
			else
				return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER, "Supply Voltage below expected range. Possible overcurrent condition."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_STC1003_VOLTAGEINPUT_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpackfloat(buf), 7);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_STC1004_0_SUPPORTED
static PhidgetReturnCode
sendSTC1004_0(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1004_STEPPER_100:
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETVELOCITYLIMIT:
			packfloat(buf, (getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETACCELERATION:
			packfloat(buf, (getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT, buf, 4));
		case BP_SETHOLDINGCURRENTLIMIT:
			packfloat(buf, (getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT, buf, 4));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETENGAGED, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		case BP_SETCONTROLMODE:
			return EPHIDGET_OK;
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvSTC1004_0(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int64_t motorPosition;
	double velocity;
	int stopped;
	int moving;
	int pkt;
	PhidgetReturnCode returnCode;
	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1004_STEPPER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_STEPPER_STATUSUPDATE:
			//Position in Q32.0 format (units are steps)
			motorPosition = unpack64(buf);
			//Velocity in Q32.0 format (units are steps/sec)
			velocity = unpackfloat(buf + 8);
			//Stopped
			stopped = buf[12] ? 1 : 0;
			returnCode = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", motorPosition);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = bridgeSendToChannel(ch, BP_VELOCITYCHANGE, "%g", velocity);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = PhidgetStepper_getIsMoving((PhidgetStepperHandle)ch, &moving);
			if (returnCode == EPHIDGET_OK && stopped && moving)
				returnCode = bridgeSendToChannel(ch, BP_STOPPED, "");
			return returnCode;
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		case VINT_PACKET_TYPE_STEPPER_POWER_ERROR:
			if (buf[0])
				return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER, "Supply Voltage above expected range"));
			else
				return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER, "Supply Voltage below expected range. Possible overcurrent condition."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#endif //PHIDUID_STC1004_0_SUPPORTED

static PhidgetReturnCode
sendSTC1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1000_STEPPER_100:
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) /* (1 << 7)*/));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 7)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT, buf, 2));
		case BP_SETHOLDINGCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETENGAGED, buf, 1));
		case BP_SETCONTROLMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCONTROLMODE, buf, 1));
#if 0 // XXX - is this removed from the Firmware?
		case BP_SETFAILSAFEMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETFAILSAFEMODE, buf, 1));
		case BP_SETFAILURETIMEOUT:
			doubleToSignedFixedPoint(getBridgePacketInt32(bp, 0), buf, 0, 4, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETFAILURETIMEOUT, buf, 4));
#endif
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_STC1000_STEPPER_110:
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) /* (1 << 7)*/));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 7)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT, buf, 2));
		case BP_SETHOLDINGCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETENGAGED, buf, 1));
		case BP_SETCONTROLMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCONTROLMODE, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendSTC1000_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendSTC1000(ch, bp));
}

static PhidgetReturnCode
sendSTC1002(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1002_STEPPER_100:
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) /* (1 << 7)*/));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 7)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT, buf, 2));
		case BP_SETHOLDINGCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETENGAGED, buf, 1));
		case BP_SETCONTROLMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCONTROLMODE, buf, 1));
#if 0 // XXX - is this removed from the Firmware?
		case BP_SETFAILSAFEMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETFAILSAFEMODE, buf, 1));
		case BP_SETFAILURETIMEOUT:
			doubleToSignedFixedPoint(getBridgePacketInt32(bp, 0), buf, 0, 4, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETFAILURETIMEOUT, buf, 4));
#endif
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_STC1002_STEPPER_110:
		switch (bp->vpkt) {
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) /* (1 << 7)*/));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 7)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCURRENTLIMIT, buf, 2));
		case BP_SETHOLDINGCURRENTLIMIT:
			pack16(buf, (int)(getBridgePacketDouble(bp, 0) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETHOLDINGCURRENTLIMIT, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETENGAGED, buf, 1));
		case BP_SETCONTROLMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_STEPPER_SETCONTROLMODE, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendSTC1002_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendSTC1002(ch, bp));
}

static PhidgetReturnCode
recvSTC1001(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int64_t motorPosition;
	double velocity;
	int stopped;
	int moving;
	int pkt;
	PhidgetReturnCode returnCode;
	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1001_STEPPER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_STEPPER_STATUSUPDATE:
			//Position in Q32.0 format (units are steps)
			motorPosition = unpack64(buf);
			//Velocity in Q32.0 format (units are steps/sec)
			velocity = (double)((int32_t)unpack32(buf + 8));
			velocity /= 256.0;
			//Stopped
			stopped = buf[12] ? 1 : 0;
			returnCode = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", motorPosition);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = bridgeSendToChannel(ch, BP_VELOCITYCHANGE, "%g", velocity);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			PhidgetStepper_getIsMoving((PhidgetStepperHandle)ch, &moving);
			if (stopped && (stopped == moving)) {
				return(bridgeSendToChannel(ch, BP_STOPPED, ""));
			}
			return returnCode;
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		case VINT_PACKET_TYPE_STEPPER_POWER_ERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER,
				"Over current error detected. Device has been reset."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_STC1001_STEPPER_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_STEPPER_STATUSUPDATE:
			//Position in Q32.0 format (units are steps)
			motorPosition = unpack64(buf);
			//Velocity in Q32.0 format (units are steps/sec)
			velocity = (double)((int32_t)unpack32(buf + 8));
			velocity /= 256.0;
			//Stopped
			stopped = buf[12] ? 1 : 0;
			returnCode = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", motorPosition);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = bridgeSendToChannel(ch, BP_VELOCITYCHANGE, "%g", velocity);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			PhidgetStepper_getIsMoving((PhidgetStepperHandle)ch, &moving);
			if (stopped && (stopped == moving)) {
				return(bridgeSendToChannel(ch, BP_STOPPED, ""));
			}
			return returnCode;
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		case VINT_PACKET_TYPE_STEPPER_POWER_ERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER,
				"Over current error detected. Device has been reset."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvSTC1001_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvSTC1001(ch, buf, len));
}

static PhidgetReturnCode
recvSTC1003(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int64_t motorPosition;
	double velocity;
	int stopped;
	int moving;
	int pkt;
	PhidgetReturnCode returnCode;
	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1003_STEPPER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_STEPPER_STATUSUPDATE:
			//Position in Q32.0 format (units are steps)
			motorPosition = unpack64(buf);
			//Velocity in Q32.0 format (units are steps/sec)
			velocity = (double)((int32_t)unpack32(buf + 8));
			velocity /= 256.0;
			//Stopped
			stopped = buf[12] ? 1 : 0;
			returnCode = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", motorPosition);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = bridgeSendToChannel(ch, BP_VELOCITYCHANGE, "%g", velocity);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			PhidgetStepper_getIsMoving((PhidgetStepperHandle)ch, &moving);
			if (stopped && (stopped == moving)) {
				return(bridgeSendToChannel(ch, BP_STOPPED, ""));
			}
			return returnCode;
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		case VINT_PACKET_TYPE_STEPPER_POWER_ERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER,
				"Over current error detected. Device has been reset."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_STC1003_STEPPER_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_STEPPER_STATUSUPDATE:
			//Position in Q32.0 format (units are steps)
			motorPosition = unpack64(buf);
			//Velocity in Q32.0 format (units are steps/sec)
			velocity = (double)((int32_t)unpack32(buf + 8));
			velocity /= 256.0;
			//Stopped
			stopped = buf[12] ? 1 : 0;
			returnCode = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", motorPosition);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = bridgeSendToChannel(ch, BP_VELOCITYCHANGE, "%g", velocity);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			PhidgetStepper_getIsMoving((PhidgetStepperHandle)ch, &moving);
			if (stopped && (stopped == moving)) {
				return(bridgeSendToChannel(ch, BP_STOPPED, ""));
			}
			return returnCode;
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		case VINT_PACKET_TYPE_STEPPER_POWER_ERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER,
				"Over current error detected. Device has been reset."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvSTC1003_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvSTC1003(ch, buf, len));
}

static PhidgetReturnCode
recvSTC1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int64_t motorPosition;
	double velocity;
	int stopped;
	int moving;
	int pkt;
	PhidgetReturnCode returnCode;
	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1000_STEPPER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_STEPPER_STATUSUPDATE:
			//Position in Q32.0 format (units are steps)
			motorPosition = unpack64(buf);
			//Velocity in Q32.0 format (units are steps/sec)
			velocity = (double)((int32_t)unpack32(buf + 8));
			velocity /= 256.0;
			//Stopped
			stopped = buf[12] ? 1 : 0;
			returnCode = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", motorPosition);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = bridgeSendToChannel(ch, BP_VELOCITYCHANGE, "%g", velocity);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			PhidgetStepper_getIsMoving((PhidgetStepperHandle)ch, &moving);
			if (stopped && (stopped == moving)) {
				return(bridgeSendToChannel(ch, BP_STOPPED, ""));
			}
			return returnCode;
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_STC1000_STEPPER_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_STEPPER_STATUSUPDATE:
			//Position in Q32.0 format (units are steps)
			motorPosition = unpack64(buf);
			//Velocity in Q32.0 format (units are steps/sec)
			velocity = (double)((int32_t)unpack32(buf + 8));
			velocity /= 256.0;
			//Stopped
			stopped = buf[12] ? 1 : 0;
			returnCode = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", motorPosition);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = bridgeSendToChannel(ch, BP_VELOCITYCHANGE, "%g", velocity);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			PhidgetStepper_getIsMoving((PhidgetStepperHandle)ch, &moving);
			if (stopped && (stopped == moving)) {
				return(bridgeSendToChannel(ch, BP_STOPPED, ""));
			}
			return returnCode;
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvSTC1000_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvSTC1000(ch, buf, len));
}

static PhidgetReturnCode
recvSTC1002(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int64_t motorPosition;
	double velocity;
	int stopped;
	int moving;
	int pkt;
	PhidgetReturnCode returnCode;
	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_STC1002_STEPPER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_STEPPER_STATUSUPDATE:
			//Position in Q32.0 format (units are steps)
			motorPosition = unpack64(buf);
			//Velocity in Q32.0 format (units are steps/sec)
			velocity = (double)((int32_t)unpack32(buf + 8));
			velocity /= 256.0;
			//Stopped
			stopped = buf[12] ? 1 : 0;
			returnCode = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", motorPosition);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = bridgeSendToChannel(ch, BP_VELOCITYCHANGE, "%g", velocity);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			PhidgetStepper_getIsMoving((PhidgetStepperHandle)ch, &moving);
			if (stopped && (stopped == moving)) {
				return(bridgeSendToChannel(ch, BP_STOPPED, ""));
			}
			return returnCode;
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_STC1002_STEPPER_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_STEPPER_STATUSUPDATE:
			//Position in Q32.0 format (units are steps)
			motorPosition = unpack64(buf);
			//Velocity in Q32.0 format (units are steps/sec)
			velocity = (double)((int32_t)unpack32(buf + 8));
			velocity /= 256.0;
			//Stopped
			stopped = buf[12] ? 1 : 0;
			returnCode = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", motorPosition);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			returnCode = bridgeSendToChannel(ch, BP_VELOCITYCHANGE, "%g", velocity);
			if (returnCode != EPHIDGET_OK)
				return returnCode;
			PhidgetStepper_getIsMoving((PhidgetStepperHandle)ch, &moving);
			if (stopped && (stopped == moving)) {
				return(bridgeSendToChannel(ch, BP_STOPPED, ""));
			}
			return returnCode;
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvSTC1002_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvSTC1002(ch, buf, len));
}

static PhidgetReturnCode
sendSND1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_SND1000_SOUNDSENSOR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			return (EPHIDGET_OK); // NOTE: Not sent to firmware, but used directly in recvSND1000
		case BP_SETSPLRANGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SOUNDSENSOR_SPLRANGE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvSND1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double octaves[10], finaloctaves[10];
	double tempDouble, gain, dB = 0.0, dBA  = 0.0, dBC = 0.0, changeTrigger, lastdB;
	int pkt, i, saturatedFlag = PFALSE;

	//16kHz = element 0
	static const double dBA_weight[] = { -6.700912224792721,-1.144507424157968,0.9642291552357972,1.2016993444284976,0.0,-3.098968656056327,-8.456545621324212,-16.189851062139507,-26.22302364022129,-39.52906401331724 };
	static const double dBC_weight[] = { -8.62944227232129,-3.0448821481317507,-0.8252420358959885,-0.16942156787436816,0.0,0.03251328929742176,0.0019618319836562453,-0.1718184155510175,-0.8206982482269319,3.030794594641769 };

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_SND1000_SOUNDSENSOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_SOUNDSENSOR_DBCHANGE:
			gain = buf[40] == 1 ? 39.912703891950998 : 59.912703891950998; //99 and 990
			saturatedFlag = buf[41] == 1 ? PTRUE : PFALSE;
			for (i = 0; i < 10; i++) {
				tempDouble =sqrt((double)unpackfloat(buf + (i * 4))); //adc sample^2/cnt
				tempDouble *= (3300.0 / 4095.0);//convert to mV

				tempDouble /= (6.309573444801932); //convert from mV to Pa

				octaves[i] = 20.0*log10(tempDouble) + 94.0 - gain;	//Calculate SPL

				dB += pow(10, (octaves[i] / 10.0));
				dBA += pow(10, ((octaves[i] + dBA_weight[i]) / 10.0));
				dBC += pow(10, ((octaves[i] + dBC_weight[i]) / 10.0));
			}
			if (saturatedFlag) {
				return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
			} else {
				dB = round_double(10 * log10(dB), 3);
				dBA = round_double(10 * log10(dBA), 3);
				dBC = round_double(10 * log10(dBC), 3);
				for (i = 0; i < 10; i++)
					finaloctaves[i] = round_double(octaves[9 - i], 3); //flip so element 0 is lowest freq band

				PhidgetSoundSensor_getSPLChangeTrigger((PhidgetSoundSensorHandle)ch, &changeTrigger);
				PhidgetSoundSensor_getLastDB(ch, &lastdB);

				if (fabs(lastdB - dB) > changeTrigger) {
					PhidgetSoundSensor_setLastDB(ch, dB);
					return (bridgeSendToChannel(ch, BP_DBCHANGE, "%g%g%g%10G", dB, dBA, dBC, finaloctaves));
				} else
					return EPHIDGET_OK;
			}
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendSAF1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	int overVoltage_fixed;
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_110:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 16)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_110:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 24)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_SAF1000_POWERGUARD_100:
		switch (bp->vpkt) {
		case BP_SETENABLED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_POWERGUARD_SETPOWERENABLED, buf, 1));
		case BP_SETOVERVOLTAGE:
			overVoltage_fixed = ((int)((getBridgePacketDouble(bp, 0))* (1 << 24)));
			pack32(buf, overVoltage_fixed);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_POWERGUARD_SETOVERVOLTAGE, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_POWERGUARD_SETFANMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_SAF1000_POWERGUARD_110:
		switch (bp->vpkt) {
		case BP_SETENABLED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_POWERGUARD_SETPOWERENABLED, buf, 1));
		case BP_SETOVERVOLTAGE:
			overVoltage_fixed = ((int)((getBridgePacketDouble(bp, 0))* (1 << 24)));
			pack32(buf, overVoltage_fixed);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_POWERGUARD_SETOVERVOLTAGE, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_POWERGUARD_SETFANMODE, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvSAF1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature;
	double voltage;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpacku32xS(buf, 24), 7);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_SAF1000_POWERGUARD_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_POWERGUARD_ENERGYDUMP:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_ENERGYDUMP, "Energy Dump in Progress."));
			break;
		case VINT_PACKET_TYPE_POWERGUARD_OVERTEMPERATURE:
			if (buf[0])
				return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_OVERTEMP, "Overtemperature Condition Detected."));
		case VINT_PACKET_TYPE_POWERGUARD_OVERVOLTAGE:
			if (buf[0])
				return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_OVERVOLTAGE, "Overvoltage Condition Detected."));
		case VINT_PACKET_TYPE_POWERGUARD_POWERON:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_OK, "All error conditions have been resolved."));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_SAF1000_POWERGUARD_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_POWERGUARD_ENERGYDUMP:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_ENERGYDUMP, "Energy Dump in Progress."));
			break;
		case VINT_PACKET_TYPE_POWERGUARD_OVERTEMPERATURE:
			if (buf[0])
				return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_OVERTEMP, "Overtemperature Condition Detected."));
		case VINT_PACKET_TYPE_POWERGUARD_OVERVOLTAGE:
			if (buf[0])
				return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_OVERVOLTAGE, "Overvoltage Condition Detected."));
		case VINT_PACKET_TYPE_POWERGUARD_POWERON:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_OK, "All error conditions have been resolved."));
			break;
		case VINT_PACKET_TYPE_POWERGUARD_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendSAF1000_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendSAF1000(ch, bp));
}

static PhidgetReturnCode
recvSAF1000_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvSAF1000(ch, buf, len));
}

static PhidgetReturnCode
sendREL1101(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[2];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			buf[0] = (uint8_t)(getBridgePacketInt32(bp, 0) ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETDUTYCYCLE:
			buf[0] = (uint8_t)(getBridgePacketDouble(bp, 0) * 255);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_REL1101_DIGITALOUTPUT_110:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			buf[0] = (uint8_t)(getBridgePacketInt32(bp, 0) ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETDUTYCYCLE:
			buf[0] = (uint8_t)(getBridgePacketDouble(bp, 0) * 255);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvREL1101(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
		MOS_PANIC("Unexpected packet recieved.");
		break;
	case PHIDCHUID_REL1101_DIGITALOUTPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendREL1101_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendREL1101(ch, bp));
}

static PhidgetReturnCode
recvREL1101_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvREL1101(ch, buf, len));
}

static PhidgetReturnCode
sendREL1101_1(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			buf[0] = (uint8_t)(getBridgePacketInt32(bp, 0) ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETDUTYCYCLE:
			buf[0] = (uint8_t)(getBridgePacketDouble(bp, 0) * 255);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_FREQ_100:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			buf[0] = (uint8_t)(getBridgePacketInt32(bp, 0) ? 0xFF : 0x00);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETDUTYCYCLE:
			buf[0] = (uint8_t)(getBridgePacketDouble(bp, 0) * 255);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		case BP_SETFREQUENCY:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETFREQUENCY, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvREL1101_1(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1101_1_DIGITALOUTPUT_FREQ_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendREL1100(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_REL1100_DIGITALOUTPUT_110:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_120:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		case BP_SETFREQUENCY:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETFREQUENCY, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvREL1100(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
		MOS_PANIC("Unexpected packet recieved.");
		break;
	case PHIDCHUID_REL1100_DIGITALOUTPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_REL1100_DIGITALOUTPUT_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendREL1100_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendREL1100(ch, bp));
}

static PhidgetReturnCode
recvREL1100_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvREL1100(ch, buf, len));
}

static PhidgetReturnCode
sendREL1100_Failsafe_Frequency(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendREL1100(ch, bp));
}

static PhidgetReturnCode
recvREL1100_Failsafe_Frequency(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvREL1100(ch, buf, len));
}

static PhidgetReturnCode
sendREL1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[2];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_REL1000_DIGITALOUTPUT_110:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvREL1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
		MOS_PANIC("Unexpected packet recieved.");
		break;
	case PHIDCHUID_REL1000_DIGITALOUTPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_TP_DIAGNOSER_0_SUPPORTED
static PhidgetReturnCode
sendTP_DIAGNOSER_0(PhidgetChannelHandle ch, BridgePacket *bp) {

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_TP_DIAGNOSER_RESISTANCEINPUT_100:
		switch (bp->vpkt) {
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvTP_DIAGNOSER_0(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	int pkt;
	double value;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_TP_DIAGNOSER_RESISTANCEINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_RESISTANCEINPUT_RESISTANCECHANGE:
			value = round_double(unpackfloat(buf), 3);
			return (bridgeSendToChannel(ch, BP_RESISTANCECHANGE, "%g", value));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_TP_DIAGNOSER_0_SUPPORTED */

static PhidgetReturnCode
sendREL1000_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendREL1000(ch, bp));
}

static PhidgetReturnCode
recvREL1000_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvREL1000(ch, buf, len));
}

static PhidgetReturnCode
sendRCC1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_RCC1000_RCSERVO_100:
		switch (bp->vpkt) {
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETENABLED, buf, 1));
		case BP_SETVOLTAGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_VOLTAGE, buf, 1));
		case BP_SETMINPULSEWIDTH:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 1000));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETMINPULSEWIDTH, buf, 4));
		case BP_SETMAXPULSEWIDTH:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 1000));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETMAXPULSEWIDTH, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 16.0 / 2500.0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETACCELERATION, buf, 4));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 16.0 / 50.0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETVELOCITYLIMIT, buf, 4));
		case BP_SETTARGETPOSITION:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 1000));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETTARGETPOSITION, buf, 4));
		case BP_SETSPEEDRAMPINGSTATE:
			pack32(buf, getBridgePacketInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETSPEEDRAMPINGSTATE, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_RCC1000_RCSERVO_110:
		switch (bp->vpkt) {
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETENABLED, buf, 1));
		case BP_SETVOLTAGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_VOLTAGE, buf, 1));
		case BP_SETMINPULSEWIDTH:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 1000));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETMINPULSEWIDTH, buf, 4));
		case BP_SETMAXPULSEWIDTH:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 1000));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETMAXPULSEWIDTH, buf, 4));
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 16.0 / 2500.0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETACCELERATION, buf, 4));
		case BP_SETVELOCITYLIMIT:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 16.0 / 50.0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETVELOCITYLIMIT, buf, 4));
		case BP_SETTARGETPOSITION:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 1000));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETTARGETPOSITION, buf, 4));
		case BP_SETSPEEDRAMPINGSTATE:
			pack32(buf, getBridgePacketInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_RCSERVO_SETSPEEDRAMPINGSTATE, buf, 4));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvRCC1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	unsigned int motorPosition;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_RCC1000_RCSERVO_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_RCSERVO_BAD_POWER_SUPPLY:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER,
				"Servo motors are drawing too much power or the power supply is not providing enough voltage or current"));
		case VINT_PACKET_TYPE_RCSERVO_ARRIVED_AT_TARGET_POSITION:
			motorPosition = (unsigned int)unpack16(buf);
			return (bridgeSendToChannel(ch, BP_TARGETPOSITIONREACHED, "%g", (motorPosition / 16.0)));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_RCC1000_RCSERVO_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_RCSERVO_BAD_POWER_SUPPLY:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER,
				"Servo motors are drawing too much power or the power supply is not providing enough voltage or current"));
		case VINT_PACKET_TYPE_RCSERVO_ARRIVED_AT_TARGET_POSITION:
			motorPosition = (unsigned int)unpack16(buf);
			return (bridgeSendToChannel(ch, BP_TARGETPOSITIONREACHED, "%g", (motorPosition / 16.0)));
		case VINT_PACKET_TYPE_RCSERVO_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendRCC1000_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendRCC1000(ch, bp));
}

static PhidgetReturnCode
recvRCC1000_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvRCC1000(ch, buf, len));
}


static PhidgetReturnCode
sendPRE1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_PRE1000_PRESSURESENSOR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 16)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PRESSURESENSOR_SETPRESSURECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvPRE1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double pressure;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_PRE1000_PRESSURESENSOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_PRESSURESENSOR_PRESSURECHANGE:
			pressure = round_double(unpack32xS(buf, 16), 3);
			return (bridgeSendToChannel(ch, BP_PRESSURECHANGE, "%g", pressure));
		case VINT_PACKET_TYPE_PRESSURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendHUM1100(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUM1100_VOLTAGERATIOINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, (unsigned int)(getBridgePacketDouble(bp, 0) * 0xFFFF));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SETVOLTAGERATIOCHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvHUM1100(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double value;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUM1100_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_VOLTAGERATIOCHANGE:
			value = round_double(((uint16_t)unpack16(buf + 0)) / (65535.0), 3);
			return (bridgeSendToChannel(ch, BP_VOLTAGERATIOCHANGE, "%g", value));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendMOT1101_SETCORRECTIONPARAMETERS(PhidgetChannelHandle ch, BridgePacket *bp) {
	float firmware_userMagnetometerTransform[6];
	float firmware_userMagnetometerOffset[3];
	float firmware_userMagnetometerGain[3];
	float firmware_userMagField;
	PhidgetTransaction trans;
	PhidgetReturnCode res1;
	PhidgetReturnCode res;
	uint8_t buf[28];
	int i;

	res = PhidgetChannel_beginTransaction(ch, &trans);
	if (res != EPHIDGET_OK)
		goto setcorrectionparams_done;

	/*Convert double values to 32 bit int to send to firmware*/
	firmware_userMagField = (float)getBridgePacketDouble(bp, 0);
	for (i = 0; i < 3; i++) {
		firmware_userMagnetometerOffset[i] = (float)getBridgePacketDouble(bp, i + 1);
		firmware_userMagnetometerGain[i] = (float)getBridgePacketDouble(bp, i + 4);
	}
	for (i = 0; i < 6; i++)
		firmware_userMagnetometerTransform[i] = (float)getBridgePacketDouble(bp, i + 7);

	/* Pack and send 1st packet */
	packfloat(buf, firmware_userMagField);
	for (i = 0; i < 3; i++) {
		packfloat(&buf[4 + i * 4], firmware_userMagnetometerOffset[i]);
		packfloat(&buf[16 + i * 4], firmware_userMagnetometerGain[i]);
	}

	res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_SET_CORRECTION_PARAMETERS, buf, 28, &trans);
	if (res != EPHIDGET_OK)
		goto setcorrectionparams_done;

	/* Pack and send 2nd packet */
	for (i = 0; i < 6; i++)
		packfloat(&buf[i * 4], firmware_userMagnetometerTransform[i]);

	res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_SET_CORRECTION_PARAMETERS, buf, 24, &trans);

setcorrectionparams_done:
	res1 = PhidgetChannel_endTransaction(ch, &trans);
	if (res1 != EPHIDGET_OK)
		return (res1);
	return (res);
}

static PhidgetReturnCode
sendMOT1101(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_MOT1101_SPATIAL_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_ZERO:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GYROSCOPE_ZERO, NULL, 0));
		case BP_RESETCORRECTIONPARAMETERS:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_RESET_CORRECTION_PARAMETERS, NULL, 0));
		case BP_SAVECORRECTIONPARAMETERS:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_SAVECORRECTIONPARAMETERS, NULL, 0));
		case BP_SETCORRECTIONPARAMETERS:
			return (sendMOT1101_SETCORRECTIONPARAMETERS(ch, bp));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1101_MAGNETOMETER_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_AXES_SETAXISCHANGETRIGGER, buf, 4));
		case BP_RESETCORRECTIONPARAMETERS:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_RESET_CORRECTION_PARAMETERS, NULL, 0));
		case BP_SAVECORRECTIONPARAMETERS:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_SAVECORRECTIONPARAMETERS, NULL, 0));
		case BP_SETCORRECTIONPARAMETERS:
			return (sendMOT1101_SETCORRECTIONPARAMETERS(ch, bp));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1101_GYROSCOPE_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_ZERO:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GYROSCOPE_ZERO, NULL, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1101_ACCELEROMETER_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_AXES_SETAXISCHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvMOT1101(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int fireSaturation = PFALSE;
	PhidgetReturnCode ret;
	double timestamp;
	double compass[3];
	double accel[3];
	double gyro[3];
	int pkt;
	int i;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_MOT1101_SPATIAL_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXISVALUECHANGE:

			for (i = 0; i < 3; i++) {
				// Positive or negative fixed point saturation
				if ((buf[0 + (i * 2)] == 0x7F && buf[1 + (i * 2)] == 0xFF) || (buf[0 + (i * 2)] == 0x80 && buf[1 + (i * 2)] == 0x00))
					fireSaturation |= 0x04;
				else
					compass[i] = round_double(unpack16xS(buf + (i * 2), 12), 5);
			}

			for (i = 0; i < 3; i++) {
				// Positive or negative fixed point saturation
				if ((buf[6 + (i * 2)] == 0x7F && buf[7 + (i * 2)] == 0xFF) || (buf[6 + (i * 2)] == 0x80 && buf[7 + (i * 2)] == 0x00))
					fireSaturation |= 0x01;
				else
					accel[i] = round_double(unpack16xS(buf + 6 + (i * 2), 12), 5);
			}

			for (i = 0; i < 3; i++) {
				float tmp = unpackfloat(buf + 12 + (i * 4));
				// Positive or negative float saturation
				if (tmp == (float)PUNK_FLT || tmp == (float)-PUNK_FLT)
					fireSaturation |= 0x02;
				else
					gyro[i] = round_double(tmp, 5);
			}

			if (fireSaturation == 0) {
				timestamp = (double)unpack32(buf + 24);
				ret = bridgeSendToChannel(ch, BP_SPATIALDATA, "%3G%3G%3G%g", accel, gyro, compass, timestamp);
			} else {
				if (fireSaturation & 0x01)
					ret = bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Accelerometer Saturation Detected.");
				if (fireSaturation & 0x02)
					ret = bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Gyroscope Saturation Detected.");
				if (fireSaturation & 0x04)
					ret = bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Magnetometer Saturation Detected.");
				else
					ret = EPHIDGET_UNEXPECTED;
			}

			return (ret);
			break;
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1101_MAGNETOMETER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXISVALUECHANGE:
			compass[0] = round_double(unpack16xS(buf, 12), 4);
			compass[1] = round_double(unpack16xS(buf + 2, 12), 4);
			compass[2] = round_double(unpack16xS(buf + 4, 12), 4);
			timestamp = (double)unpack32(buf + 6);
			return (bridgeSendToChannel(ch, BP_FIELDSTRENGTHCHANGE, "%3G%g", compass, timestamp));
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1101_GYROSCOPE_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXESUPDATE:
			gyro[0] = round_double(unpackfloat(buf), 5);
			gyro[1] = round_double(unpackfloat(buf + 4), 5);
			gyro[2] = round_double(unpackfloat(buf + 8), 5);
			timestamp = (double)unpack32(buf + 12);
			return (bridgeSendToChannel(ch, BP_ANGULARRATEUPDATE, "%3G%g", gyro, timestamp));
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1101_ACCELEROMETER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXISVALUECHANGE:
			accel[0] = round_double(unpack16xS(buf, 12), 4);
			accel[1] = round_double(unpack16xS(buf + 2, 12), 4);
			accel[2] = round_double(unpack16xS(buf + 4, 12), 4);
			timestamp = (double)unpack32(buf + 6);
			return (bridgeSendToChannel(ch, BP_ACCELERATIONCHANGE, "%3G%g", accel, timestamp));
			break;
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Acceleration Saturation"));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}


static PhidgetReturnCode
sendMOT1102_SETCORRECTIONPARAMETERS(PhidgetChannelHandle ch, BridgePacket *bp) {
	float firmware_userMagnetometerTransform[6];
	float firmware_userMagnetometerOffset[3];
	float firmware_userMagnetometerGain[3];
	float firmware_userMagField;
	PhidgetTransaction trans;
	PhidgetReturnCode res1;
	PhidgetReturnCode res;
	uint8_t buf[28];
	int i;

	res = PhidgetChannel_beginTransaction(ch, &trans);
	if (res != EPHIDGET_OK)
		goto setcorrectionparams_done;

	/*Convert double values to 32 bit int to send to firmware*/
	firmware_userMagField = (float)getBridgePacketDouble(bp, 0);
	for (i = 0; i < 3; i++) {
		firmware_userMagnetometerOffset[i] = (float)getBridgePacketDouble(bp, i + 1);
		firmware_userMagnetometerGain[i] = (float)getBridgePacketDouble(bp, i + 4);
	}
	for (i = 0; i < 6; i++)
		firmware_userMagnetometerTransform[i] = (float)getBridgePacketDouble(bp, i + 7);

	/* Pack and send 1st packet */
	packfloat(buf, firmware_userMagField);
	for (i = 0; i < 3; i++) {
		packfloat(&buf[4 + i * 4], firmware_userMagnetometerOffset[i]);
		packfloat(&buf[16 + i * 4], firmware_userMagnetometerGain[i]);
	}

	res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_SET_CORRECTION_PARAMETERS, buf, 28, &trans);
	if (res != EPHIDGET_OK)
		goto setcorrectionparams_done;

	/* Pack and send 2nd packet */
	for (i = 0; i < 6; i++)
		packfloat(&buf[i * 4], firmware_userMagnetometerTransform[i]);

	res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_SET_CORRECTION_PARAMETERS, buf, 24, &trans);

setcorrectionparams_done:
	res1 = PhidgetChannel_endTransaction(ch, &trans);
	if (res1 != EPHIDGET_OK)
		return (res1);
	return (res);
}

static PhidgetReturnCode
sendMOT1102(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[24];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_MOT1102_SPATIAL_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_ZERO:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GYROSCOPE_ZERO, NULL, 0));
		case BP_RESETCORRECTIONPARAMETERS:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_RESET_CORRECTION_PARAMETERS, NULL, 0));
		case BP_SAVECORRECTIONPARAMETERS:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_SAVECORRECTIONPARAMETERS, NULL, 0));
		case BP_SETCORRECTIONPARAMETERS:
			return (sendMOT1102_SETCORRECTIONPARAMETERS(ch, bp));
		case BP_ZEROSPATIALALGORITHM:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SPATIAL_ZERO_AHRS, NULL, 0));
		case BP_SETSPATIALALGORITHM:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SPATIAL_AHRS_ALGORITHM, buf, 1));
		case BP_SETSPATIALALGORITHMMAGGAIN:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SPATIAL_AHRS_MAG_GAIN, buf, 4));
		case BP_SETAHRSPARAMETERS:
			packfloat(&buf[0], getBridgePacketDouble(bp, 0));
			packfloat(&buf[4], getBridgePacketDouble(bp, 1));
			packfloat(&buf[8], getBridgePacketDouble(bp, 2));
			packfloat(&buf[12], getBridgePacketDouble(bp, 3));
			packfloat(&buf[16], getBridgePacketDouble(bp, 4));
			packfloat(&buf[20], getBridgePacketDouble(bp, 5));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SPATIAL_SET_AHRS_PARAMS, buf, 24));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1102_MAGNETOMETER_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_AXES_SETAXISCHANGETRIGGER, buf, 4));
		case BP_RESETCORRECTIONPARAMETERS:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_RESET_CORRECTION_PARAMETERS, NULL, 0));
		case BP_SAVECORRECTIONPARAMETERS:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MAGNETOMETER_SAVECORRECTIONPARAMETERS, NULL, 0));
		case BP_SETCORRECTIONPARAMETERS:
			return (sendMOT1102_SETCORRECTIONPARAMETERS(ch, bp));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1102_GYROSCOPE_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_ZERO:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GYROSCOPE_ZERO, NULL, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1102_ACCELEROMETER_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_AXES_SETAXISCHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvMOT1102(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int fireSaturation = PFALSE;
	PhidgetReturnCode ret;
	double timestamp;
	double quaternion[4];
	double compass[3];
	double accel[3];
	double gyro[3];
	int pkt;
	int i;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_MOT1102_SPATIAL_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXISVALUECHANGE:

			for (i = 0; i < 3; i++) {
				// Positive or negative fixed point saturation
				if ((buf[0 + (i * 2)] == 0x7F && buf[1 + (i * 2)] == 0xFF) || (buf[0 + (i * 2)] == 0x80 && buf[1 + (i * 2)] == 0x00))
					fireSaturation |= 0x04;
				else
					compass[i] = round_double(unpack16xS(buf + (i * 2), 12), 5);
			}

			for (i = 0; i < 3; i++) {
				// Positive or negative fixed point saturation
				if ((buf[6 + (i * 2)] == 0x7F && buf[7 + (i * 2)] == 0xFF) || (buf[6 + (i * 2)] == 0x80 && buf[7 + (i * 2)] == 0x00))
					fireSaturation |= 0x01;
				else
					accel[i] = round_double(unpack16xS(buf + 6 + (i * 2), 12), 5);
			}

			for (i = 0; i < 3; i++) {
				float tmp = unpackfloat(buf + 12 + (i * 4));
				// Positive or negative float saturation
				if (tmp == (float)PUNK_FLT || tmp == (float)-PUNK_FLT)
					fireSaturation |= 0x02;
				else
					gyro[i] = round_double(tmp, 5);
			}

			quaternion[0] = (double)unpackfloat(buf + 28);
			quaternion[1] = (double)unpackfloat(buf + 32);
			quaternion[2] = (double)unpackfloat(buf + 36);
			quaternion[3] = (double)unpackfloat(buf + 40);

			if (fireSaturation == 0) {
				timestamp = (double)unpack32(buf + 24);
				ret = bridgeSendToChannel(ch, BP_SPATIALDATA, "%3G%3G%3G%g", accel, gyro, compass, timestamp);
				if(ret == EPHIDGET_OK)
					ret = bridgeSendToChannel(ch, BP_SPATIALALGDATA, "%4G%g", quaternion, timestamp);
			} else {
				if (fireSaturation & 0x01)
					ret = bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Accelerometer Saturation Detected.");
				if (fireSaturation & 0x02)
					ret = bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Gyroscope Saturation Detected.");
				if (fireSaturation & 0x04)
					ret = bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Magnetometer Saturation Detected.");
				else
					ret = EPHIDGET_UNEXPECTED;
			}

			return (ret);
			break;
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		case VINT_PACKET_TYPE_AXES_INVALID:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_INVALIDSTATE, "The gyroscope has received invalid data and will recover shortly."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1102_MAGNETOMETER_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXISVALUECHANGE:
			compass[0] = round_double(unpack16xS(buf, 12), 4);
			compass[1] = round_double(unpack16xS(buf + 2, 12), 4);
			compass[2] = round_double(unpack16xS(buf + 4, 12), 4);
			timestamp = (double)unpack32(buf + 6);
			return (bridgeSendToChannel(ch, BP_FIELDSTRENGTHCHANGE, "%3G%g", compass, timestamp));
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1102_GYROSCOPE_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXESUPDATE:
			gyro[0] = round_double(unpackfloat(buf), 5);
			gyro[1] = round_double(unpackfloat(buf + 4), 5);
			gyro[2] = round_double(unpackfloat(buf + 8), 5);
			timestamp = (double)unpack32(buf + 12);
			return (bridgeSendToChannel(ch, BP_ANGULARRATEUPDATE, "%3G%g", gyro, timestamp));
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		case VINT_PACKET_TYPE_AXES_INVALID:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_INVALIDSTATE, "The gyroscope has received invalid data and will recover shortly."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_MOT1102_ACCELEROMETER_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXISVALUECHANGE:
			accel[0] = round_double(unpack16xS(buf, 12), 4);
			accel[1] = round_double(unpack16xS(buf + 2, 12), 4);
			accel[2] = round_double(unpack16xS(buf + 4, 12), 4);
			timestamp = (double)unpack32(buf + 6);
			return (bridgeSendToChannel(ch, BP_ACCELERATIONCHANGE, "%3G%g", accel, timestamp));
			break;
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Acceleration Saturation"));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_MOT1100_OLD_SUPPORTED
static PhidgetReturnCode
sendMOT1100_OLD(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[2];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_MOT1100_ACCELEROMETER_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfltto16xS(buf, (float)getBridgePacketDouble(bp, 0), 12);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_AXES_SETAXISCHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvMOT1100_OLD(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double timestamp;
	double acc[3];
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_MOT1100_ACCELEROMETER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXISVALUECHANGE:
			acc[0] = round_double(unpack16xS(buf, 12), 4);
			acc[1] = round_double(unpack16xS(buf + 2, 12), 4);
			acc[2] = round_double(unpack16xS(buf + 4, 12), 4);
			timestamp = (double)unpack32(buf + 6);
			return (bridgeSendToChannel(ch, BP_ACCELERATIONCHANGE, "%3G%g", acc, timestamp));
			break;
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Acceleration Saturation"));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_MOT1100_OLD_SUPPORTED */

static PhidgetReturnCode
sendMOT1100(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_MOT1100_ACCELEROMETER_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_AXES_SETAXISCHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvMOT1100(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double timestamp;
	double acc[3];
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_MOT1100_ACCELEROMETER_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXISVALUECHANGE:
			acc[0] = round_double(unpack16xS(buf, 12), 4);
			acc[1] = round_double(unpack16xS(buf + 2, 12), 4);
			acc[2] = round_double(unpack16xS(buf + 4, 12), 4);
			timestamp = (double)unpack32(buf + 6);
			return (bridgeSendToChannel(ch, BP_ACCELERATIONCHANGE, "%3G%g", acc, timestamp));
			break;
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Acceleration Saturation"));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_VINTACCEL_SUPPORTED
static PhidgetReturnCode
sendVINTACCEL(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_VINTACCEL_ACCELEROMETER_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_AXES_SETAXISCHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvVINTACCEL(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double timestamp;
	double acc[3];
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_VINTACCEL_ACCELEROMETER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_AXES_AXISVALUECHANGE:
			acc[0] = round_double(unpack16xS(buf, 12), 4);
			acc[1] = round_double(unpack16xS(buf + 2, 12), 4);
			acc[2] = round_double(unpack16xS(buf + 4, 12), 4);
			timestamp = (double)unpack32(buf + 6);
			return (bridgeSendToChannel(ch, BP_ACCELERATIONCHANGE, "%3G%g", acc, timestamp));
			break;
		case VINT_PACKET_TYPE_AXES_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Acceleration Saturation"));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_VINTACCEL_SUPPORTED */

static PhidgetReturnCode
sendLUX1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 14)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_LIGHTSENSOR_SETILLUMINANCECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvLUX1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double illuminance;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_LIGHTSENSOR_ILLUMINANCECHANGE:
			illuminance = unpack32xS(buf, 14);
			if (illuminance < 100)
				illuminance = round_double(illuminance, 4);
			else
				illuminance = round_double(illuminance, 2);
			return (bridgeSendToChannel(ch, BP_ILLUMINANCECHANGE, "%g", illuminance));
		case VINT_PACKET_TYPE_LIGHTSENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendLED1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_LED1000_DIGITALOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETLEDFORWARDVOLTAGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			switch (buf[0]) {
			case LED_FORWARD_VOLTAGE_3_2V:
			case LED_FORWARD_VOLTAGE_4_0V:
			case LED_FORWARD_VOLTAGE_4_8V:
			case LED_FORWARD_VOLTAGE_5_6V:
				return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_LEDFORWARDVOLTAGE, buf, 1));
			default:
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Invalid or unsupported voltage for this device."));
			}
		case BP_SETLEDCURRENTLIMIT:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 16)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_LEDCURRENTLIMIT, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvLED1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	MOS_PANIC("Unexpected packet recieved.");
}

static PhidgetReturnCode
sendLCD1100_WRITETEXT(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetLCDHandle phid = (PhidgetLCDHandle)ch;
	PhidgetTransaction trans;
	PhidgetReturnCode res1;
	PhidgetReturnCode res;
	PhidgetLCD_Font font;
	size_t numsent;
	const char *chars;
	uint8_t buf[48];
	size_t numchars;
	int frameBuffer;
	int height;
	int width;
	int count;
	int xsize;
	int ysize;
	int xpos;
	int ypos;
	int i;

	font = (PhidgetLCD_Font)getBridgePacketInt32(bp, 0);
	xpos = getBridgePacketInt32(bp, 1);
	ypos = getBridgePacketInt32(bp, 2);
	chars = getBridgePacketString(bp, 3);

	res = PhidgetLCD_getFontSize(phid, font, &xsize, &ysize);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(bp->iop, res, "Error geting font size."));

	res = PhidgetLCD_getFrameBuffer(phid, &frameBuffer);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(bp->iop, res, "Error geting framebuffer."));

	res = PhidgetLCD_getWidth(phid, &width);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(bp->iop, res, "Error geting width."));

	res = PhidgetLCD_getHeight(phid, &height);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(bp->iop, res, "Error geting height."));

	numchars = strlen(chars);

	//Because FONT_User1 is stored in framebuffer 1
	if ((int)font == frameBuffer)
		return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Invalid font/framebuffer combination."));

	//truncate text to end of line
	numchars = MOS_MIN(width - xpos, xsize * (int)numchars) / xsize;

	//truncate text off bottom of screen
	if (ypos + ysize > height)
		numchars = 0;

	//numchars is 0 - so we don't actually send anything, but still need to return OK for async calls
	if (numchars == 0)
		return (EPHIDGET_OK);

	//Send the packets
	res = PhidgetChannel_beginTransaction(ch, &trans);
	if (res != EPHIDGET_OK)
		goto writetext_done;

	numsent = 0;
	while (numsent < numchars) {
		//number of letters to send
		count = MOS_MIN(VINT_MAX_OUT_PACKETSIZE - 6, (int)(numchars - numsent));

		//font(1), xsize(1), ysize(1), xpos(1), ypos(1), num_chars(1), text(num_chars <= 8)
		buf[0] = font;
		buf[1] = xsize;
		buf[2] = ysize;
		buf[3] = xpos;
		buf[4] = ypos;
		buf[5] = count;

		//fill the rest of the packet with chars
		for (i = 0; i < count; i++)
			buf[i + 6] = chars[numsent++];

		//increment horizontal position according to the number of chars written multiplied by the font symbol width
		xpos += count*xsize;
		res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_WRITETEXT, buf, count + 6, &trans);
		if (res != EPHIDGET_OK)
			break;
	}
writetext_done:
	res1 = PhidgetChannel_endTransaction(ch, &trans);
	if (res1 != EPHIDGET_OK)
		return res1;
	return res;
}

static PhidgetReturnCode
sendLCD1100_WRITEBITMAP(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetTransaction trans;
	PhidgetReturnCode res1;
	PhidgetReturnCode res;
	unsigned char bit;
	uint8_t buf[48];
	const uint8_t *bitmap;
	int count;
	int xsize;
	int ysize;
	int x;
	int y;
	int i;

	count = 0;
	bitmap = getBridgePacketUInt8Array(bp, 4);
	xsize = getBridgePacketInt32(bp, 2);
	ysize = getBridgePacketInt32(bp, 3);

	res = PhidgetChannel_beginTransaction(ch, &trans);
	if (res != EPHIDGET_OK)
		goto writebitmap_done;

	buf[0] = getBridgePacketInt32(bp, 0);
	buf[1] = getBridgePacketInt32(bp, 1);
	buf[2] = xsize;
	buf[3] = ysize;

	res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_WRITEBITMAP, buf, 4, &trans);
	if (res != EPHIDGET_OK)
		goto writebitmap_done;

	x = 0;
	y = 0;
	while (res == EPHIDGET_OK && x < xsize) {
		memset(buf, 0, VINT_MAX_OUT_PACKETSIZE);

		//create bit stream of pixels
		for (i = 0; i < VINT_MAX_OUT_PACKETSIZE * 8 && x < xsize; i++) {
			count = (i / 8) + 1;
			bit = 1 << (7 - (i % 8)) & (bitmap[y*xsize + x] ? 0xFF : 0x00);
			buf[(i / 8)] |= bit;

			y++;
			if (y >= ysize) {
				x++;
				y = 0;
			}
		}
		res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_BITMAPDATA, buf, count, &trans);
		if (res != EPHIDGET_OK)
			goto writebitmap_done;
	}
writebitmap_done:
	res1 = PhidgetChannel_endTransaction(ch, &trans);
	if (res1 != EPHIDGET_OK)
		return res1;
	return res;
}

static PhidgetReturnCode
sendLCD1100_SETCHARACTERBITMAP(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetLCDHandle phid = (PhidgetLCDHandle)ch;
	PhidgetTransaction trans;
	const char *characterStr;
	PhidgetReturnCode res1;
	PhidgetReturnCode res;
	const uint8_t* bitmap;
	PhidgetLCD_Font font;
	unsigned char bit;
	uint8_t buf[48];
	int frameBuffer;
	char character;
	int maxchars;
	int count;
	int lcdw;
	int w;
	int h;
	int x;
	int y;
	int i;

	font = (PhidgetLCD_Font)getBridgePacketInt32(bp, 0);
	characterStr = getBridgePacketString(bp, 1);
	bitmap = getBridgePacketUInt8Array(bp, 2);
	count = 0;

	if (font != FONT_User1 && font != FONT_User2)
		return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Font must be user font 1 or 2."));

	res = PhidgetLCD_getFrameBuffer(phid, &frameBuffer);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(bp->iop, res, "Error geting framebuffer."));

	res = PhidgetLCD_getMaxCharacters(phid, font, &maxchars);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(bp->iop, res, "Error geting max characters."));

	res = PhidgetLCD_getFontSize(phid, font, &w, &h);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(bp->iop, res, "Error geting font size."));

	res = PhidgetLCD_getWidth(phid, &lcdw);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(bp->iop, res, "Error geting width."));

	// These should have been checked in class code
	assert(maxchars > 0);
	assert(w > 0);
	assert(h > 0);

	// XXX - support a wider range of characters? Full UTF-8?
	if (strlen(characterStr) != 1)
		return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Multibyte characters are unsupported."));

	character = (char)characterStr[0];

	//Characters start at 1
	character -= 1;

	if (character < 0 || character > maxchars)
		return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Character code must be between 0x00 and 0x%02x ('%c'). Number of supported characters depends on the font size.", maxchars, maxchars));

	res = PhidgetChannel_beginTransaction(ch, &trans);
	if (res != EPHIDGET_OK)
		goto setcharacterbitmap_done;

	if (frameBuffer != (int)font) {
		//FrameBuffer in Q8.0 format
		buf[0] = font;
		res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_SETFRAMEBUFFER, buf, 1, &trans);
		if (res != EPHIDGET_OK)
			goto setcharacterbitmap_done;
	}

	//build and send initial packet
	buf[0] = w*(character % (lcdw / w)); //x position
	buf[1] = h*(character / (lcdw / w)); //y position
	buf[2] = w; //width
	buf[3] = h; //height
	res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_WRITEBITMAP, buf, 4, &trans);
	if (res != EPHIDGET_OK)
		goto setcharacterbitmap_done;

	x = 0;
	y = 0;
	while (res == EPHIDGET_OK && x < w) {
		memset(buf, 0, VINT_MAX_OUT_PACKETSIZE);

		//create bit stream of pixels
		for (i = 0; i < VINT_MAX_OUT_PACKETSIZE * 8 && x < w; i++) {
			count = (i / 8) + 1;
			bit = 1 << (7 - (i % 8)) & (bitmap[y*w + x] ? 0xFF : 0x00);
			buf[(i / 8)] |= bit;
			y++;
			if (y >= h) {
				x++;
				y = 0;
			}
		}
		res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_BITMAPDATA, buf, count, &trans);
		if (res != EPHIDGET_OK)
			goto setcharacterbitmap_done;
	}

	if (frameBuffer != (int)font) {
		//FrameBuffer in Q8.0 format
		buf[0] = frameBuffer;
		res = sendVINTDataPacketTransaction(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_SETFRAMEBUFFER, buf, 1, &trans);
		if (res != EPHIDGET_OK)
			goto setcharacterbitmap_done;
	}
setcharacterbitmap_done:
	res1 = PhidgetChannel_endTransaction(ch, &trans);
	if (res1 != EPHIDGET_OK)
		return res1;
	return res;
}

static PhidgetReturnCode
sendLCD1100(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[48];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_LCD1100_LCD_100:
		switch (bp->vpkt) {
		case BP_SAVEFRAMEBUFFER:
			buf[0] = getBridgePacketInt32(bp, 0);
			if (buf[0] < 0 || buf[0] > 2)
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Framebuffer must be 0, 1 or 2."));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_SAVEFRAMEBUFFER, buf, 1));
		case BP_SETFRAMEBUFFER:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_SETFRAMEBUFFER, buf, 1));
		case BP_SETBACKLIGHT:
			//Backlight in Q0.8 format
			// TODO: This should probably not have a special case for backlight == 1.0
			if (getBridgePacketDouble(bp, 0) >= 1)
				buf[0] = 0xFF;
			else
				buf[0] = ((int)((getBridgePacketDouble(bp, 0)) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_SETBACKLIGHT, buf, 1));
		case BP_SETCONTRAST:
			if (getBridgePacketDouble(bp, 0) >= 1)
				buf[0] = 0xFF;
			else
				buf[0] = ((int)((getBridgePacketDouble(bp, 0)) * (1 << 8)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_SETCONTRAST, buf, 1));
		case BP_DRAWPIXEL:
			buf[0] = getBridgePacketInt32(bp, 0);
			buf[1] = getBridgePacketInt32(bp, 1);
			buf[2] = getBridgePacketInt32(bp, 2);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_DRAWPIXEL, buf, 3));
		case BP_DRAWLINE:
			buf[0] = getBridgePacketInt32(bp, 0);

			buf[1] = getBridgePacketInt32(bp, 1);
			buf[2] = getBridgePacketInt32(bp, 2);
			buf[3] = getBridgePacketInt32(bp, 3);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_DRAWLINE, buf, 4));
		case BP_DRAWRECT:
			buf[0] = getBridgePacketInt32(bp, 0);
			buf[1] = getBridgePacketInt32(bp, 1);
			buf[2] = getBridgePacketInt32(bp, 2);
			buf[3] = getBridgePacketInt32(bp, 3);
			buf[4] = getBridgePacketInt32(bp, 4) ? 0xFF : 0x00;
			buf[5] = getBridgePacketInt32(bp, 5) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_DRAWRECT, buf, 6));
		case BP_WRITETEXT:
			return (sendLCD1100_WRITETEXT(ch, bp));
		case BP_WRITEBITMAP:
			return (sendLCD1100_WRITEBITMAP(ch, bp));
		case BP_FLUSH:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_FLUSH, NULL, 0));
		case BP_CLEAR:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_CLEAR, NULL, 0));
		case BP_SETSLEEP:
			buf[0] = getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_SETSLEEP, buf, 1));
		case BP_SETCHARACTERBITMAP:
			return (sendLCD1100_SETCHARACTERBITMAP(ch, bp));
		case BP_COPY:
			buf[0] = getBridgePacketInt32(bp, 0);
			buf[1] = getBridgePacketInt32(bp, 1);
			buf[2] = getBridgePacketInt32(bp, 2);
			buf[3] = getBridgePacketInt32(bp, 3);
			buf[4] = getBridgePacketInt32(bp, 4);
			buf[5] = getBridgePacketInt32(bp, 5);
			buf[6] = getBridgePacketInt32(bp, 6);
			buf[7] = getBridgePacketInt32(bp, 7);
			buf[8] = getBridgePacketInt32(bp, 8) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_COPY, buf, 9));
		case BP_INITIALIZE:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_GRAPHICLCD_INIT, NULL, 0));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvLCD1100(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	MOS_PANIC("Unexpected packet recieved.");
}

static PhidgetReturnCode
sendHUM1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, ((int)((getBridgePacketDouble(bp, 0)) * (1 << 16))));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 16)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_HUMIDITYSENSOR_SETHUMIDITYCHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvHUM1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature;
	double humidity;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_HUMIDITYSENSOR_HUMIDITYCHANGE:
			humidity = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_HUMIDITYCHANGE, "%g", humidity));
		case VINT_PACKET_TYPE_HUMIDITYSENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendHUM1001(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUM1001_TEMPERATURESENSOR_IC_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, ((int)((getBridgePacketDouble(bp, 0)) * (1 << 16))));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_HUM1001_HUMIDITYSENSOR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 16)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_HUMIDITYSENSOR_SETHUMIDITYCHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvHUM1001(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature;
	double humidity;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUM1001_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_HUM1001_HUMIDITYSENSOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_HUMIDITYSENSOR_HUMIDITYCHANGE:
			humidity = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_HUMIDITYCHANGE, "%g", humidity));
		case VINT_PACKET_TYPE_HUMIDITYSENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendHIN1100(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, (unsigned int)(getBridgePacketDouble(bp, 0) * 16384));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SETVOLTAGERATIOCHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvHIN1100(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double axis;
	int value;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_VOLTAGERATIOCHANGE:
			axis = round_double(((int16_t)unpack16(buf + 0)) / 16384.0, 4);
			return (bridgeSendToChannel(ch, BP_VOLTAGERATIOCHANGE, "%g", axis));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_HIN1100_DIGITALINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE:
			value = buf[0] ? 1 : 0;
			return (bridgeSendToChannel(ch, BP_STATECHANGE, "%d", value));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendHIN1001(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_BUTTONS_100:
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_WHEEL_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, (unsigned int)(getBridgePacketDouble(bp, 0) * (1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CAPACITIVETOUCH_TOUCHVALUECHANGETRIGGER, buf, 2));
		case BP_SETSENSITIVITY:
			if (ch->index == 4)
				buf[0] = (uint32_t)((1 - getBridgePacketDouble(bp, 0)) * 32) + 1;
			else
				buf[0] = (uint32_t)((1 - getBridgePacketDouble(bp, 0)) * 64) + 1;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CAPACITIVETOUCH_SENSITIVITY, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvHIN1001(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double value;
	int touched;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_BUTTONS_100:
	case PHIDCHUID_HIN1001_CAPACITIVETOUCH_WHEEL_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_CAPACITIVETOUCH_TOUCHVALUECHANGE:
			touched = buf[0] ? 1 : 0;
			if (ch->index < 4)
				value = (double)touched;
			else
				value = unpack16(buf + 1) / 65536.0;
			if (touched)
				return (bridgeSendToChannel(ch, BP_TOUCHINPUTVALUECHANGE, "%g", value));
			return (bridgeSendToChannel(ch, BP_TOUCHINPUTEND, ""));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendHIN1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_HIN1000_CAPACITIVETOUCH_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
	//		pack16(buf, (unsigned int)(getBridgePacketDouble(bp, 0) * (1 << 15)));
			return EPHIDGET_OK;// (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CAPACITIVETOUCH_TOUCHVALUECHANGETRIGGER, buf, 2));
		case BP_SETSENSITIVITY:
			buf[0] = (int32_t)(((1 - getBridgePacketDouble(bp, 0)) * 80) + 20);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CAPACITIVETOUCH_SENSITIVITY, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvHIN1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int touched;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HIN1000_CAPACITIVETOUCH_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_CAPACITIVETOUCH_TOUCHVALUECHANGE:
			touched = buf[0] ? 1 : 0;
			if (touched)
				return (bridgeSendToChannel(ch, BP_TOUCHINPUTVALUECHANGE, "%g", (double)touched));
			return (bridgeSendToChannel(ch, BP_TOUCHINPUTEND, ""));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendENC1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_ENC1000_ENCODER_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_SETENCODERCHANGETRIGGER, buf, 4));
		case BP_SETIOMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		case BP_SETENABLED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_ENABLE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvENC1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int indexPosition;
	uint8_t indexTriggered;
	int positionChange;
	uint64_t timeChange;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_ENC1000_ENCODER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_ENCODER_ENCODERCHANGE:
			positionChange = unpack32(buf);
			timeChange = ((uint64_t)(unpack32(buf + 4)) * 1000) / 16;//Time comes back as counts at 16MHz, convert to ns
			indexTriggered = buf[8];
			if (indexTriggered)
				indexPosition = unpack32(buf + 9);
			else
				indexPosition = 0;
			return (bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%d%g%c%d", positionChange, timeChange / 1000000.0, indexTriggered, indexPosition));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendHIN1101(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_HIN1101_ENCODER_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_SETENCODERCHANGETRIGGER, buf, 4));
		case BP_SETIOMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		case BP_SETENABLED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_ENABLE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvHIN1101(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int positionChange;
	uint64_t timeChange;
	int pkt;
	int state;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HIN1101_ENCODER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_ENCODER_ENCODERCHANGE:
			positionChange = unpack32(buf);
			timeChange = ((uint64_t)(unpack32(buf + 4)) * 1000) / 16;//Time comes back as counts at 16MHz, convert to ns
			return (bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%d%g%c%d", positionChange, timeChange / 1000000.0, 0, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_HIN1101_DIGITALINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE:
			state = buf[0] & 0x01;
			return (bridgeSendToChannel(ch, BP_STATECHANGE, "%d", state));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDST1200(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DST1200_DISTANCESENSOR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DISTANCESENSOR_SETDISTANCECHANGETRIGGER, buf, 2));
		case BP_SETSONARQUIETMODE:
			buf[0] = (getBridgePacketInt32(bp, 0) ? 1 : 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SONAR_QUIETMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#define DISTANCE_MAX_REFLECTIONS 8
static PhidgetReturnCode
recvDST1200(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	uint32_t reflections_distance[DISTANCE_MAX_REFLECTIONS];
	uint32_t reflections_strength[DISTANCE_MAX_REFLECTIONS];
	uint32_t reflections_count;
	PhidgetReturnCode ret;
	uint32_t distance;
	uint32_t i;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DST1200_DISTANCESENSOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DISTANCESENSOR_SONARREFLECTIONSUPDATE:
			reflections_count = buf[0];
			for (i = 0; i < DISTANCE_MAX_REFLECTIONS; i++) {
				if (i < reflections_count)
					reflections_distance[i] = (unsigned short)(unpack16(&buf[2 * i + 1]));
				else
					reflections_distance[i] = PUNK_UINT32;
			}
			for (i = 0; i < DISTANCE_MAX_REFLECTIONS; i++) {
				if (i < reflections_count) {
					reflections_strength[i] = (unsigned short)(unpack16(&buf[(2 * i) + (2 * reflections_count) + 1]));
					if (reflections_strength[i] == 0x7FFF)
						reflections_strength[i] = PUNK_UINT32;
				} else
					reflections_strength[i] = PUNK_UINT32;
			}
			if (reflections_count == 0)
				return (bridgeSendToChannel(ch, BP_SONARUPDATE, "%*U%*U%u", DISTANCE_MAX_REFLECTIONS, reflections_distance, DISTANCE_MAX_REFLECTIONS, reflections_strength, reflections_count));

			ret = (bridgeSendToChannel(ch, BP_SONARUPDATE, "%*U%*U%u", DISTANCE_MAX_REFLECTIONS, reflections_distance, DISTANCE_MAX_REFLECTIONS, reflections_strength, reflections_count));
			if (ret == EPHIDGET_OK)
				return (bridgeSendToChannel(ch, BP_DISTANCECHANGE, "%u", reflections_distance[0]));
			else
				return ret;
		case VINT_PACKET_TYPE_DISTANCESENSOR_DISTANCECHANGE:
			distance = unpack32(&buf[0]);
			return (bridgeSendToChannel(ch, BP_DISTANCECHANGE, "%u", distance));
		case VINT_PACKET_TYPE_DISTANCESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDST1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DST1000_DISTANCESENSOR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			buf[0] = getBridgePacketUInt32(bp, 0) & 0xFF;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DISTANCESENSOR_SETDISTANCECHANGETRIGGER, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDST1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int distance;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DST1000_DISTANCESENSOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DISTANCESENSOR_DISTANCECHANGE:
			distance = buf[0];
			return (bridgeSendToChannel(ch, BP_DISTANCECHANGE, "%u", distance));
		case VINT_PACKET_TYPE_DISTANCESENSOR_SATURATION:
			//SATURATION in the DST1000 is synonymous with OUTOFRANGE, which makes more sense in this context
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_OUTOFRANGE, "Sensor value is ouside the valid range for this sensor."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDST1001(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DST1001_DISTANCESENSOR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DISTANCESENSOR_SETDISTANCECHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDST1001(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int distance;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DST1001_DISTANCESENSOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DISTANCESENSOR_DISTANCECHANGE:
			distance = (buf[1] << 8) + buf[0];
			return (bridgeSendToChannel(ch, BP_DISTANCECHANGE, "%u", distance));
		case VINT_PACKET_TYPE_DISTANCESENSOR_SATURATION:
			//SATURATION in the DST1001 is synonymous with OUTOFRANGE, which makes more sense in this context
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_OUTOFRANGE, "Sensor value is ouside the valid range for this sensor."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDST1002(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DST1002_DISTANCESENSOR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DISTANCESENSOR_SETDISTANCECHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDST1002(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int distance;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DST1002_DISTANCESENSOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DISTANCESENSOR_DISTANCECHANGE:
			distance = (buf[1] << 8) + buf[0];
			return (bridgeSendToChannel(ch, BP_DISTANCECHANGE, "%u", distance));
		case VINT_PACKET_TYPE_DISTANCESENSOR_SATURATION:
			//SATURATION in the DST1002 is synonymous with OUTOFRANGE, which makes more sense in this context
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_OUTOFRANGE, "Sensor value is ouside the valid range for this sensor."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDCC1100(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
		switch (bp->vpkt) {
		case BP_SETDEADBAND:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND, buf, 4));
		case BP_SETKP:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP, buf, 4));
		case BP_SETKD:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD, buf, 4));
		case BP_SETKI:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI, buf, 4));
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETACCELERATION:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_BLDCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED, buf, 1));
		case BP_SETSTALLVELOCITY:
			pack32(buf, (getBridgePacketDouble(bp, 0))); //convert to commutations per second
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETSTALLVELOCITY, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
		switch (bp->vpkt) {
		case BP_SETDEADBAND:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND, buf, 4));
		case BP_SETKP:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP, buf, 4));
		case BP_SETKD:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD, buf, 4));
		case BP_SETKI:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI, buf, 4));
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETACCELERATION:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETVELOCITYLIMIT, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_BLDCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED, buf, 1));
		case BP_SETSTALLVELOCITY:
			pack32(buf, (getBridgePacketDouble(bp, 0))); //convert to commutations per second
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETSTALLVELOCITY, buf, 4));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_120:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			//TemperatureChangeTrigger in Q8.8 format
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0)*256.0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_BLDCMOTOR_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_BLDCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_BLDCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.045 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETSTALLVELOCITY:
			pack32(buf, (getBridgePacketDouble(bp, 0))); //convert to commutations per second
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_BLDCMOTOR_SETSTALLVELOCITY, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_BLDCMOTOR_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_BLDCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_BLDCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.045 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETSTALLVELOCITY:
			pack32(buf, (getBridgePacketDouble(bp, 0))); //convert to commutations per second
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_BLDCMOTOR_SETSTALLVELOCITY, buf, 4));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDCC1100_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendDCC1100(ch, bp));
}

static PhidgetReturnCode
sendDCC1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1000_CURRENTINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, (uint16_t)(((4095 / 3.3)*0.045) * getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CURRENTINPUT_SETCURRENTCHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			//TemperatureChangeTrigger in Q8.8 format
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0)*256.0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0)*(1 << 16)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SETVOLTAGERATIOCHANGETRIGGER, buf, 4));
		case BP_SETSENSORTYPE:
			return (supportedVoltageRatioSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_ENCODER_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_SETENCODERCHANGETRIGGER, buf, 4));
		case BP_SETIOMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		case BP_SETENABLED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_DCMOTOR_100:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.045 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_FAN, buf, 1));
		case BP_SETCURRENTREGULATORGAIN:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0) - 1.0) * 1804.070707 + 897.0)); //map 1-100 to 897-179500 (0.1%/A - 20%/A)
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTREGULATORGAIN, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDCC1000_POSITIONCONTROL(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1000_CURRENTINPUT_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, (uint16_t)(((4095 / 3.3)*0.045) * getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CURRENTINPUT_SETCURRENTCHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			//TemperatureChangeTrigger in Q8.8 format
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0)*256.0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0)*(1 << 16)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SETVOLTAGERATIOCHANGETRIGGER, buf, 4));
		case BP_SETSENSORTYPE:
			return (supportedVoltageRatioSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_ENCODER_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_SETENCODERCHANGETRIGGER, buf, 4));
		case BP_SETIOMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		case BP_SETENABLED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_DCMOTOR_200:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.045 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_FAN, buf, 1));
		case BP_SETCURRENTREGULATORGAIN:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0) - 1.0) * 1804.070707 + 897.0)); //map 1-100 to 897-179500 (0.1%/A - 20%/A)
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTREGULATORGAIN, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		switch (bp->vpkt) {
		case BP_SETDEADBAND:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND, buf, 4));
		case BP_SETKP:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP, buf, 4));
		case BP_SETKD:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD, buf, 4));
		case BP_SETKI:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI, buf, 4));
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETACCELERATION:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.045 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETCURRENTREGULATORGAIN:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0) - 1.0) * 1804.070707 + 897.0)); //map 1-100 to 897-179500 (0.1%/A - 20%/A)
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTREGULATORGAIN, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_FAN, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETIOMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENCODERIOMODE, buf, 1));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDCC1000_POSITIONCONTROL_FAILSAFE(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1000_CURRENTINPUT_210:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, (uint16_t)(((4095 / 3.3)*0.045) * getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CURRENTINPUT_SETCURRENTCHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_210:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			//TemperatureChangeTrigger in Q8.8 format
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0)*256.0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_210:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0)*(1 << 16)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SETVOLTAGERATIOCHANGETRIGGER, buf, 4));
		case BP_SETSENSORTYPE:
			return (supportedVoltageRatioSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_ENCODER_210:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_SETENCODERCHANGETRIGGER, buf, 4));
		case BP_SETIOMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		case BP_SETENABLED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_DCMOTOR_210:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.045 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_FAN, buf, 1));
		case BP_SETCURRENTREGULATORGAIN:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0) - 1.0) * 1804.070707 + 897.0)); //map 1-100 to 897-179500 (0.1%/A - 20%/A)
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTREGULATORGAIN, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		switch (bp->vpkt) {
		case BP_SETDEADBAND:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND, buf, 4));
		case BP_SETKP:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP, buf, 4));
		case BP_SETKD:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD, buf, 4));
		case BP_SETKI:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI, buf, 4));
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETACCELERATION:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.045 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETCURRENTREGULATORGAIN:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0) - 1.0) * 1804.070707 + 897.0)); //map 1-100 to 897-179500 (0.1%/A - 20%/A)
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTREGULATORGAIN, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_FAN, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETIOMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENCODERIOMODE, buf, 1));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_DCC1004_SUPPORTED
static PhidgetReturnCode
sendDCC1004(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1004_CURRENTINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, (uint16_t)(((4095 / 3.3)*0.09) * getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CURRENTINPUT_SETCURRENTCHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1004_TEMPERATURESENSOR_IC_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			//TemperatureChangeTrigger in Q8.8 format
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0)*256.0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1004_ENCODER_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_SETENCODERCHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1004_DCMOTOR_100:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.09 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_FAN, buf, 1));
		case BP_SETCURRENTREGULATORGAIN:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0) - 1.0) * 1804.070707 + 897.0)); //map 1-100 to 897-179500 (0.1%/A - 20%/A)
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTREGULATORGAIN, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1004_MOTORPOSITIONCONTROLLER_100:
		switch (bp->vpkt) {
		case BP_SETDEADBAND:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND, buf, 4));
		case BP_SETKP:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP, buf, 4));
		case BP_SETKD:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD, buf, 4));
		case BP_SETKI:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI, buf, 4));
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETACCELERATION:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.09 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETCURRENTREGULATORGAIN:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0) - 1.0) * 1804.070707 + 897.0)); //map 1-100 to 897-179500 (0.1%/A - 20%/A)
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTREGULATORGAIN, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_FAN, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_DCC1004_SUPPORTED */

#if PHIDUID_DCC1005_SUPPORTED
static PhidgetReturnCode
sendDCC1005(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1005_CURRENTINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, (uint16_t)(((4095 / 3.3)*0.0088) * getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CURRENTINPUT_SETCURRENTCHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1005_TEMPERATURESENSOR_IC_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			//TemperatureChangeTrigger in Q8.8 format
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0)*256.0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_TEMPERATURESENSOR_SETTEMPERATURECHANGETRIGGER, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1005_ENCODER_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_SETENCODERCHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1005_DCMOTOR_100:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.0088 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_FAN, buf, 1));
		case BP_SETCURRENTREGULATORGAIN:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0) - 1.0) * 1804.070707 + 897.0)); //map 1-100 to 897-179500 (0.1%/A - 20%/A)
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTREGULATORGAIN, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1005_MOTORPOSITIONCONTROLLER_100:
		switch (bp->vpkt) {
		case BP_SETDEADBAND:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND, buf, 4));
		case BP_SETKP:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP, buf, 4));
		case BP_SETKD:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD, buf, 4));
		case BP_SETKI:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI, buf, 4));
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETACCELERATION:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)*0.0088 * 4095 / 3.3) + 2048));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETCURRENTREGULATORGAIN:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0) - 1.0) * 1804.070707 + 897.0)); //map 1-100 to 897-179500 (0.1%/A - 20%/A)
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTREGULATORGAIN, buf, 4));
		case BP_SETFANMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_FAN, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_DCC1005_SUPPORTED */

static PhidgetReturnCode
recvDCC1100(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double  temperature, dutyCycle, brakingDutyCycle;
	int64_t position;
	int pkt;
	PhidgetReturnCode result;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_BLDCMOTOR_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 940.0, 3);
			position = ((int64_t)unpack64(buf + 8));
			if (PhidgetBLDCMotor_getLastBrakingStrength((PhidgetBLDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			if (PhidgetBLDCMotor_getLastPosition((PhidgetBLDCMotorHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_BLDCMOTOR_STALLDETECTED:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_MOTORSTALL, "Dangerous motor stall detected. Velocity has been reduced in order to prevent damage."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_BLDCMOTOR_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 940.0, 3);
			position = ((int64_t)unpack64(buf + 8));
			if (PhidgetBLDCMotor_getLastBrakingStrength((PhidgetBLDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			if (PhidgetBLDCMotor_getLastPosition((PhidgetBLDCMotorHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_BLDCMOTOR_STALLDETECTED:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_MOTORSTALL, "Dangerous motor stall detected. Velocity has been reduced in order to prevent damage."));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			position = ((int64_t)unpack64(buf + 4));
			if (PhidgetMotorPositionController_getLastPosition((PhidgetMotorPositionControllerHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_BLDCMOTOR_STALLDETECTED:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_MOTORSTALL, "Dangerous motor stall detected. Motor has been disengaged in order to prevent damage."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			position = ((int64_t)unpack64(buf + 4));
			if (PhidgetMotorPositionController_getLastPosition((PhidgetMotorPositionControllerHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_BLDCMOTOR_STALLDETECTED:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_MOTORSTALL, "Dangerous motor stall detected. Motor has been disengaged in order to prevent damage."));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDCC1100_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvDCC1100(ch, buf, len));
}

static PhidgetReturnCode
recvDCC1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltageRatio, temperature, current, dutyCycle, brakingDutyCycle, sign;
	int indexPosition;
	uint64_t timeChange;
	uint8_t indexTriggered;
	int positionChange;
	PhidgetReturnCode result;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1000_CURRENTINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_CURRENTINPUT_CURRENTCHANGE:
			current = (double)(unpack16(buf) - 2048);
			current = round_double(current*0.017908017, 4);
			return (bridgeSendToChannel(ch, BP_CURRENTCHANGE, "%g", current));
		case VINT_PACKET_TYPE_CURRENTINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_VOLTAGERATIOCHANGE:
			voltageRatio = round_double(unpack32xS(buf, 16), 5);
			return (bridgeSendToChannel(ch, BP_VOLTAGERATIOCHANGE, "%g", voltageRatio));
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_ENCODER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_ENCODER_ENCODERCHANGE:
			positionChange = unpack32(buf);
			timeChange = ((uint64_t)(unpack32(buf + 4)) * 1000) / 48;//Time comes back as counts at 48MHz, convert to ns
			indexTriggered = buf[8];
			if (indexTriggered)
				indexPosition = unpack32(buf + 9);
			else
				indexPosition = 0;
			return (bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%d%g%c%d", positionChange, timeChange / 1000000.0, indexTriggered, indexPosition));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_DCMOTOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:
			brakingDutyCycle = (double)unpack16(buf + 2);
			brakingDutyCycle = round_double(brakingDutyCycle / 65535.0, 3);
			sign = buf[5] ? -1 : 1;
			dutyCycle = (double)unpack16(buf);
			dutyCycle = round_double(dutyCycle*sign / 65535.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDCC1000_POSITIONCONTROL(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltageRatio, temperature, current, dutyCycle, brakingDutyCycle;
	int indexPosition;
	int64_t position;
	uint64_t timeChange;
	uint8_t indexTriggered;
	int positionChange;
	PhidgetReturnCode result;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1000_CURRENTINPUT_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_CURRENTINPUT_CURRENTCHANGE:
			current = (double)(unpack16(buf) - 2048);
			current = round_double(current*0.017908017, 4);
			return (bridgeSendToChannel(ch, BP_CURRENTCHANGE, "%g", current));
		case VINT_PACKET_TYPE_CURRENTINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_VOLTAGERATIOCHANGE:
			voltageRatio = round_double(unpack32xS(buf, 16), 5);
			return (bridgeSendToChannel(ch, BP_VOLTAGERATIOCHANGE, "%g", voltageRatio));
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_ENCODER_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_ENCODER_ENCODERCHANGE:
			positionChange = unpack32(buf);
			timeChange = ((uint64_t)(unpack32(buf + 4)) * 1000) / 48;//Time comes back as counts at 48MHz, convert to ns
			indexTriggered = buf[8];
			if (indexTriggered)
				indexPosition = unpack32(buf + 9);
			else
				indexPosition = 0;
			return (bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%d%g%c%d", positionChange, timeChange / 1000000.0, indexTriggered, indexPosition));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_DCMOTOR_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:
			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 940.0, 3);
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			position = ((int64_t)unpack64(buf + 4));
			if (PhidgetMotorPositionController_getLastPosition((PhidgetMotorPositionControllerHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return (bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDCC1000_POSITIONCONTROL_FAILSAFE(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltageRatio, temperature, current, dutyCycle, brakingDutyCycle;
	int indexPosition;
	int64_t position;
	uint64_t timeChange;
	uint8_t indexTriggered;
	int positionChange;
	PhidgetReturnCode result;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1000_CURRENTINPUT_210:
		switch (pkt) {
		case VINT_PACKET_TYPE_CURRENTINPUT_CURRENTCHANGE:
			current = (double)(unpack16(buf) - 2048);
			current = round_double(current*0.017908017, 4);
			return (bridgeSendToChannel(ch, BP_CURRENTCHANGE, "%g", current));
		case VINT_PACKET_TYPE_CURRENTINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_210:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_210:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_VOLTAGERATIOCHANGE:
			voltageRatio = round_double(unpack32xS(buf, 16), 5);
			return (bridgeSendToChannel(ch, BP_VOLTAGERATIOCHANGE, "%g", voltageRatio));
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_ENCODER_210:
		switch (pkt) {
		case VINT_PACKET_TYPE_ENCODER_ENCODERCHANGE:
			positionChange = unpack32(buf);
			timeChange = ((uint64_t)(unpack32(buf + 4)) * 1000) / 48;//Time comes back as counts at 48MHz, convert to ns
			indexTriggered = buf[8];
			if (indexTriggered)
				indexPosition = unpack32(buf + 9);
			else
				indexPosition = 0;
			return (bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%d%g%c%d", positionChange, timeChange / 1000000.0, indexTriggered, indexPosition));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_DCMOTOR_210:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:
			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 940.0, 3);
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		switch (pkt) {
		case VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			position = ((int64_t)unpack64(buf + 4));
			if (PhidgetMotorPositionController_getLastPosition((PhidgetMotorPositionControllerHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return (bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#if PHIDUID_DCC1004_SUPPORTED
static PhidgetReturnCode
recvDCC1004(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature, current, dutyCycle, brakingDutyCycle;
	int indexPosition;
	int64_t position;
	uint64_t timeChange;
	uint8_t indexTriggered;
	int positionChange;
	PhidgetReturnCode result;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1004_CURRENTINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_CURRENTINPUT_CURRENTCHANGE:
			current = (double)(unpack16(buf) - 2048);
			current = round_double((current*(3.3 / 4095)) / 0.09, 4);
			return (bridgeSendToChannel(ch, BP_CURRENTCHANGE, "%g", current));
		case VINT_PACKET_TYPE_CURRENTINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1004_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1004_ENCODER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_ENCODER_ENCODERCHANGE:
			positionChange = unpack32(buf);
			timeChange = ((uint64_t)(unpack32(buf + 4)) * 1000) / 48;//Time comes back as counts at 48MHz, convert to ns
			indexTriggered = buf[8];
			if (indexTriggered)
				indexPosition = unpack32(buf + 9);
			else
				indexPosition = 0;
			return (bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%d%g%c%d", positionChange, timeChange / 1000000.0, indexTriggered, indexPosition));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1004_DCMOTOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:
			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 940.0, 3);
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1004_MOTORPOSITIONCONTROLLER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			position = ((int64_t)unpack64(buf + 4));
			if (PhidgetMotorPositionController_getLastPosition((PhidgetMotorPositionControllerHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return (bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_DCC1004_SUPPORTED */

#if PHIDUID_DCC1005_SUPPORTED
static PhidgetReturnCode
recvDCC1005(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double temperature, current, dutyCycle, brakingDutyCycle;
	int indexPosition;
	int64_t position;
	uint64_t timeChange;
	uint8_t indexTriggered;
	int positionChange;
	PhidgetReturnCode result;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1005_CURRENTINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_CURRENTINPUT_CURRENTCHANGE:
			current = (double)(unpack16(buf) - 2048);
			current = round_double((current*(3.3 / 4095)) / 0.0088, 4);
			return (bridgeSendToChannel(ch, BP_CURRENTCHANGE, "%g", current));
		case VINT_PACKET_TYPE_CURRENTINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
			break;
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1005_TEMPERATURESENSOR_IC_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_TEMPERATURECHANGE:
			temperature = round_double(unpack32xS(buf, 16), 2);
			return (bridgeSendToChannel(ch, BP_TEMPERATURECHANGE, "%g", temperature));
		case VINT_PACKET_TYPE_TEMPERATURESENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1005_ENCODER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_ENCODER_ENCODERCHANGE:
			positionChange = unpack32(buf);
			timeChange = ((uint64_t)(unpack32(buf + 4)) * 1000) / 48;//Time comes back as counts at 48MHz, convert to ns
			indexTriggered = buf[8];
			if (indexTriggered)
				indexPosition = unpack32(buf + 9);
			else
				indexPosition = 0;
			return (bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%d%g%c%d", positionChange, timeChange / 1000000.0, indexTriggered, indexPosition));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1005_DCMOTOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:
			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 940.0, 3);
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1005_MOTORPOSITIONCONTROLLER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			position = ((int64_t)unpack64(buf + 4));
			if (PhidgetMotorPositionController_getLastPosition((PhidgetMotorPositionControllerHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return (bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_DCC1005_SUPPORTED */

static PhidgetReturnCode
sendDCC1001(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1001_ENCODER_100:
	case PHIDCHUID_DCC1001_ENCODER_120:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_SETENCODERCHANGETRIGGER, buf, 4));
		case BP_SETIOMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		case BP_SETENABLED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1001_DCMOTOR_100:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1001_DCMOTOR_120:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
		switch (bp->vpkt) {
		case BP_SETDEADBAND:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND, buf, 4));
		case BP_SETKP:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP, buf, 4));
		case BP_SETKD:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD, buf, 4));
		case BP_SETKI:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI, buf, 4));
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETACCELERATION:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			packfloat(buf, ((float)getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
		switch (bp->vpkt) {
		case BP_SETDEADBAND:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND, buf, 4));
		case BP_SETKP:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP, buf, 4));
		case BP_SETKD:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD, buf, 4));
		case BP_SETKI:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI, buf, 4));
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETACCELERATION:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			packfloat(buf, ((float)getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDCC1001(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double dutyCycle, brakingDutyCycle;
	int indexPosition;
	uint64_t timeChange;
	int64_t position;
	uint8_t indexTriggered;
	int positionChange;
	PhidgetReturnCode result;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1001_ENCODER_100:
	case PHIDCHUID_DCC1001_ENCODER_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_ENCODER_ENCODERCHANGE:
			positionChange = unpack32(buf);
			timeChange = ((uint64_t)(unpack32(buf + 4)) * 1000) / 48;//Time comes back as counts at 48MHz, convert to ns
			indexTriggered = buf[8];
			if (indexTriggered)
				indexPosition = unpack32(buf + 9);
			else
				indexPosition = 0;
			return (bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%d%g%c%d", positionChange, timeChange / 1000000.0, indexTriggered, indexPosition));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1001_DCMOTOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:

			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 940.0, 3);
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1001_DCMOTOR_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:

			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 940.0, 3);
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			position = ((int64_t)unpack64(buf + 4));
			if (PhidgetMotorPositionController_getLastPosition((PhidgetMotorPositionControllerHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return (bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 940.0, 3);
			position = ((int64_t)unpack64(buf + 4));
			if (PhidgetMotorPositionController_getLastPosition((PhidgetMotorPositionControllerHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return (bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDCC1001_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendDCC1001(ch, bp));
}

static PhidgetReturnCode
recvDCC1001_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvDCC1001(ch, buf, len));
}

static PhidgetReturnCode
sendDCC1002(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1002_ENCODER_100:
	case PHIDCHUID_DCC1002_ENCODER_110:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_SETENCODERCHANGETRIGGER, buf, 4));
		case BP_SETIOMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		case BP_SETENABLED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ENCODER_IOMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1002_DCMOTOR_100:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1002_DCMOTOR_110:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
		switch (bp->vpkt) {
		case BP_SETDEADBAND:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND, buf, 4));
		case BP_SETKP:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP, buf, 4));
		case BP_SETKD:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD, buf, 4));
		case BP_SETKI:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI, buf, 4));
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETACCELERATION:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			packfloat(buf, ((float)getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
		switch (bp->vpkt) {
		case BP_SETDEADBAND:
			pack32(buf, getBridgePacketUInt32(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETDEADBAND, buf, 4));
		case BP_SETKP:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKP, buf, 4));
		case BP_SETKD:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKD, buf, 4));
		case BP_SETKI:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETKI, buf, 4));
		case BP_SETTARGETPOSITION:
			pack64(buf, (uint64_t)getBridgePacketInt64(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETTARGETPOSITION, buf, 8));
		case BP_SETACCELERATION:
			packfloat(buf, (float)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			packfloat(buf, ((float)getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETENGAGED:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_SETENGAGED, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDCC1002_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendDCC1002(ch, bp));
}

static PhidgetReturnCode
sendDCC1003(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[8];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1003_DCMOTOR_100:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1003_DCMOTOR_110:
		switch (bp->vpkt) {
		case BP_SETACCELERATION:
			pack32(buf, (uint32_t)((getBridgePacketDouble(bp, 0)) * 8192));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_MOTORCONTROLLER_SETACCELERATION, buf, 4));
		case BP_SETDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETVELOCITY, buf, 4));
		case BP_SETBRAKINGDUTYCYCLE:
			pack32(buf, (int32_t)((getBridgePacketDouble(bp, 0)) * 2097151)); //have to multiply by 959 so save 10 bits for multiplication in firmware
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETBRAKINGSTRENGTH, buf, 4));
		case BP_SETCURRENTLIMIT:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DCMOTOR_SETCURRENTLIMIT, buf, 4));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDCC1003_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendDCC1003(ch, bp));
}

static PhidgetReturnCode
recvDCC1002(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double dutyCycle, brakingDutyCycle;
	int64_t indexPosition;
	uint64_t timeChange;
	int64_t position;
	uint8_t indexTriggered;
	int positionChange;
	PhidgetReturnCode result;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1002_ENCODER_100:
	case PHIDCHUID_DCC1002_ENCODER_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_ENCODER_ENCODERCHANGE:
			positionChange = unpack32(buf);
			timeChange = ((uint64_t)(unpack32(buf + 4)) * 1000) / 48;//Time comes back as counts at 48MHz, convert to ns
			indexTriggered = buf[8];
			if (indexTriggered)
				indexPosition = unpack32(buf + 9);
			else
				indexPosition = 0;
			return (bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%d%g%c%d", positionChange, timeChange / 1000000.0, indexTriggered, indexPosition));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DCC1002_DCMOTOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:

			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 959.0, 3);
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 959.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1002_DCMOTOR_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:

			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 959.0, 3);
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 959.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 959.0, 3);
			position = ((int64_t)unpack64(buf + 4));
			if (PhidgetMotorPositionController_getLastPosition((PhidgetMotorPositionControllerHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return (bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_MOTORPOSITIONCONTROLLER_MOTORSTATUSUPDATE:
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 959.0, 3);
			position = ((int64_t)unpack64(buf + 4));
			if (PhidgetMotorPositionController_getLastPosition((PhidgetMotorPositionControllerHandle)ch) != position) {
				result = bridgeSendToChannel(ch, BP_POSITIONCHANGE, "%l", position);
				if (result != EPHIDGET_OK)
					return result;
			}
			return (bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDCC1002_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvDCC1002(ch, buf, len));
}

static PhidgetReturnCode
recvDCC1003(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double dutyCycle, brakingDutyCycle;
	PhidgetReturnCode result;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DCC1003_DCMOTOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:
			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 959.0, 3);
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 959.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_DCC1003_DCMOTOR_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_DCMOTOR_MOTORSTATUSUPDATE:
			brakingDutyCycle = (double)((int32_t)unpack32(buf + 4));
			brakingDutyCycle = round_double(brakingDutyCycle / 959.0, 3);
			dutyCycle = (double)((int32_t)unpack32(buf));
			dutyCycle = round_double(dutyCycle / 959.0, 3);
			if (PhidgetDCMotor_getLastBrakingStrength((PhidgetDCMotorHandle)ch) != brakingDutyCycle) {
				result = bridgeSendToChannel(ch, BP_BRAKINGSTRENGTHCHANGE, "%g", brakingDutyCycle);
				if (result != EPHIDGET_OK)
					return result;
			}
			return(bridgeSendToChannel(ch, BP_DUTYCYCLECHANGE, "%g", dutyCycle));
		case VINT_PACKET_TYPE_MOTORCONTROLLER_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDCC1003_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvDCC1003(ch, buf, len));
}

static PhidgetReturnCode
sendDAQ1500(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];
	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		switch (bp->vpkt) {
		case BP_SETENABLED:
			buf[0] = (uint8_t)getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_BRIDGEENABLED, buf, 1));
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 0x80000000u));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SETVOLTAGERATIOCHANGETRIGGER, buf, 4));
		case BP_SETBRIDGEGAIN:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_BRIDGEGAIN, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDAQ1500(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltageRatio;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_VOLTAGERATIOCHANGE:
			voltageRatio = /*round_double(*/(int32_t)unpack32(buf) / (0x80000000u - 1.0);// , 10);
			return (bridgeSendToChannel(ch, BP_VOLTAGERATIOCHANGE, "%g", voltageRatio));
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendVCP1100(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];
	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_VCP1100_CURRENTINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 0x00010000u));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CURRENTINPUT_SETCURRENTCHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvVCP1100(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double current;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_VCP1100_CURRENTINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_CURRENTINPUT_CURRENTCHANGE:
			current = ((int32_t)unpack32(buf) / 65536.0);
			return (bridgeSendToChannel(ch, BP_CURRENTCHANGE, "%g", current));
		case VINT_PACKET_TYPE_CURRENTINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendVCP1100_1(PhidgetChannelHandle ch, BridgePacket* bp) {
	uint8_t buf[4];
	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_VCP1100_CURRENTINPUT_200:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * 0x00010000u));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CURRENTINPUT_SETCURRENTCHANGETRIGGER, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvVCP1100_1(PhidgetChannelHandle ch, const uint8_t* buf, size_t len) {
	double current;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_VCP1100_CURRENTINPUT_200:
		switch (pkt) {
		case VINT_PACKET_TYPE_CURRENTINPUT_CURRENTCHANGE:
			current = ((int32_t)unpack32(buf) / 65536.0);
			return (bridgeSendToChannel(ch, BP_CURRENTCHANGE, "%g", current));
		case VINT_PACKET_TYPE_CURRENTINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDAQ1400(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_120:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return(sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETPOWERSUPPLY:
			buf[0] = getBridgePacketInt32(bp, 0);
			return(sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ISA_SETPOWERSUPPLY, buf, 1));
		case BP_SETINPUTMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return(sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ISA_SETINPUTMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
	case PHIDCHUID_DAQ1400_DIGITALINPUT_120:
		switch (bp->vpkt) {
		case BP_SETPOWERSUPPLY:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ISA_SETPOWERSUPPLY, buf, 1));
		case BP_SETINPUTMODE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return(sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ISA_SETINPUTMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DAQ1400_CURRENTINPUT_100:
	case PHIDCHUID_DAQ1400_CURRENTINPUT_120:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (int)((getBridgePacketDouble(bp, 0)) * (1 << 24)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CURRENTINPUT_SETCURRENTCHANGETRIGGER, buf, 4));
		case BP_SETPOWERSUPPLY:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ISA_SETPOWERSUPPLY, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_120:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 24)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		case BP_SETPOWERSUPPLY:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_ISA_SETPOWERSUPPLY, buf, 1));
		case BP_SETSENSORTYPE:
			return (supportedVoltageSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDAQ1400(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double ticksAtLastCount;
	unsigned int counts;
	double ticks;
	double current;
	double voltage;
	int state;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_FREQUENCYCOUNTER_COUNTCHANGE:
			ticks = unpack32(buf) / 1000.0; //convert us to ms
			counts = unpack32(buf + 4);
			ticksAtLastCount = unpack32(buf + 8) / 1000.0; //convert us to ms
			return (bridgeSendToChannel(ch, BP_FREQUENCYDATA, "%g%u%g", ticks, counts, ticksAtLastCount));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
	case PHIDCHUID_DAQ1400_DIGITALINPUT_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE:
			state = buf[0] & 0x01;
			return (bridgeSendToChannel(ch, BP_STATECHANGE, "%d", state));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DAQ1400_CURRENTINPUT_100:
	case PHIDCHUID_DAQ1400_CURRENTINPUT_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_CURRENTINPUT_CURRENTCHANGE:
			current = round_double(unpack32xS(buf, 24), 5);
			// Don't return anything below 500 uA (ch->minCurrent) - we can't measure down to 0
			if (current < 0.0005)
				return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_OUTOFRANGE, "Sensor value is ouside the valid range for this sensor."));
			return (bridgeSendToChannel(ch, BP_CURRENTCHANGE, "%g", current));
		case VINT_PACKET_TYPE_CURRENTINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpack32xS(buf, 24), 8);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDAQ1400_120(PhidgetChannelHandle ch, BridgePacket* bp) {
	return sendDAQ1400(ch, bp);
}

static PhidgetReturnCode
recvDAQ1400_120(PhidgetChannelHandle ch, const uint8_t* buf, size_t len) {
	return recvDAQ1400(ch, buf, len);
}

static PhidgetReturnCode
sendDAQ1301(PhidgetChannelHandle ch, BridgePacket *bp) {
	MOS_PANIC("Unexpected command recieved.");
}

static PhidgetReturnCode
recvDAQ1301(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1301_DIGITALINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE:
			return (bridgeSendToChannel(ch, BP_STATECHANGE, "%d", (buf[0] & 0x1)));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDAQ1300(PhidgetChannelHandle ch, BridgePacket *bp) {
	MOS_PANIC("Unexpected command recieved.");
}

static PhidgetReturnCode
recvDAQ1300(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1300_DIGITALINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE:
			return (bridgeSendToChannel(ch, BP_STATECHANGE, "%d", (buf[0] & 0x1)));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendOUT1100(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_110:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_120:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		case BP_SETFREQUENCY:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETFREQUENCY, buf, 4));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvOUT1100(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
		MOS_PANIC("Unexpected packet recieved.");
		break;
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_120:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendOUT1100_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendOUT1100(ch, bp));
}

static PhidgetReturnCode
recvOUT1100_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvOUT1100(ch, buf, len));
}

static PhidgetReturnCode
sendOUT1100_Failsafe_Frequency(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendOUT1100(ch, bp));
}

static PhidgetReturnCode
recvOUT1100_Failsafe_Frequency(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvOUT1100(ch, buf, len));
}

static PhidgetReturnCode
sendDAQ1200(PhidgetChannelHandle ch, BridgePacket *bp) {
	MOS_PANIC("Unexpected command recieved.");
}

static PhidgetReturnCode
recvDAQ1200(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1200_DIGITALINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE:
			return (bridgeSendToChannel(ch, BP_STATECHANGE, "%d", (buf[0] & 0x1)));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendOUT1002(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETVOLTAGE:
			pack32(buf, ((int)((getBridgePacketDouble(bp, 0)) * (1 << 24))));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETVOLTAGE, buf, 4));
		case BP_SETVOLTAGERANGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_110:
		switch (bp->vpkt) {
		case BP_SETVOLTAGE:
			pack32(buf, ((int)((getBridgePacketDouble(bp, 0)) * (1 << 24))));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETVOLTAGE, buf, 4));
		case BP_SETVOLTAGERANGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETMODE, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvOUT1002(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEOUTPUT_VOLTAGEERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_VOLTAGEERROR, "Voltage Error Detected"));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_OUT1002_VOLTAGEOUTPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEOUTPUT_VOLTAGEERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_VOLTAGEERROR, "Voltage Error Detected"));
		case VINT_PACKET_TYPE_VOLTAGEOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendOUT1002_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendOUT1002(ch, bp));
}

static PhidgetReturnCode
recvOUT1002_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvOUT1002(ch, buf, len));
}

static PhidgetReturnCode
sendOUT1001(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETVOLTAGE:
			pack32(buf, ((int)((getBridgePacketDouble(bp, 0)) * (1 << 24))));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETVOLTAGE, buf, 4));
		case BP_SETVOLTAGERANGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETMODE, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_110:
		switch (bp->vpkt) {
		case BP_SETVOLTAGE:
			pack32(buf, ((int)((getBridgePacketDouble(bp, 0)) * (1 << 24))));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETVOLTAGE, buf, 4));
		case BP_SETVOLTAGERANGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETMODE, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvOUT1001(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEOUTPUT_VOLTAGEERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_VOLTAGEERROR, "Voltage Error Detected"));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_OUT1001_VOLTAGEOUTPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEOUTPUT_VOLTAGEERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_VOLTAGEERROR, "Voltage Error Detected"));
		case VINT_PACKET_TYPE_VOLTAGEOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendOUT1001_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendOUT1001(ch, bp));
}

static PhidgetReturnCode
recvOUT1001_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvOUT1001(ch, buf, len));
}

static PhidgetReturnCode
sendOUT1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETVOLTAGE:
			pack32(buf, ((int)((getBridgePacketDouble(bp, 0)) * (1 << 24))));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETVOLTAGE, buf, 4));
		case BP_SETENABLED:
			buf[0] = (uint8_t)getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETENABLED, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_110:
		switch (bp->vpkt) {
		case BP_SETVOLTAGE:
			pack32(buf, ((int)((getBridgePacketDouble(bp, 0)) * (1 << 24))));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETVOLTAGE, buf, 4));
		case BP_SETENABLED:
			buf[0] = (uint8_t)getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEOUTPUT_SETENABLED, buf, 1));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvOUT1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEOUTPUT_VOLTAGEERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_VOLTAGEERROR, "Voltage Error Detected"));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_OUT1000_VOLTAGEOUTPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEOUTPUT_VOLTAGEERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_VOLTAGEERROR, "Voltage Error Detected"));
		case VINT_PACKET_TYPE_VOLTAGEOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendOUT1000_Failsafe(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendOUT1000(ch, bp));
}

static PhidgetReturnCode
recvOUT1000_Failsafe(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvOUT1000(ch, buf, len));
}

#if PHIDUID_CURLOOP_SUPPORTED
static PhidgetReturnCode
sendCURLOOP(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_CURLOOP_CURRENTOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETCURRENT:
			pack32(buf, ((int)((getBridgePacketDouble(bp, 0)) * (1 << 24))));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CURRENTOUTPUT_SETCURRENT, buf, 4));
		case BP_SETENABLED:
			buf[0] = (uint8_t)getBridgePacketInt32(bp, 0) ? 0xFF : 0x00;
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_CURRENTOUTPUT_SETENABLED, buf, 1));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvCURLOOP(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_CURLOOP_CURRENTOUTPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_CURRENTOUTPUT_CURRENTERROR:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_BADPOWER, "Current Error Detected"));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}
#endif /* PHIDUID_CURLOOP_SUPPORTED */

static PhidgetReturnCode
sendDAQ1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_110:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		case BP_SETSENSORTYPE:
			return (supportedVoltageSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_110:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			packfloat(buf, (float)getBridgePacketDouble(bp, 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SETVOLTAGERATIOCHANGETRIGGER, buf, 4));
		case BP_SETSENSORTYPE:
			return (supportedVoltageRatioSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDAQ1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltage;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpackfloat(buf), 3);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_VOLTAGERATIOCHANGE:
			voltage = round_double(unpackfloat(buf), 4);
			return (bridgeSendToChannel(ch, BP_VOLTAGERATIOCHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDAQ1000_5V25(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendDAQ1000(ch, bp));
}

static PhidgetReturnCode
recvDAQ1000_5V25(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvDAQ1000(ch, buf, len));
}

static PhidgetReturnCode
sendADP1000(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 24)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 4));
		case BP_SETVOLTAGERANGE:
			buf[0] = getBridgePacketInt32(bp, 0);
			switch (buf[0]) {
			case VOLTAGE_RANGE_400mV:
			case VOLTAGE_RANGE_2V:
				break;
			default:
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Invalid or unsupported voltage range for this device."));
			}
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_PHRANGE, buf, 1));
		case BP_SETSENSORTYPE:
			return (supportedVoltageSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack32(buf, (uint32_t)(getBridgePacketDouble(bp, 0) * (1 << 24)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHSENSOR_SETPHCHANGETRIGGER, buf, 4));
		case BP_SETCORRECTIONTEMPERATURE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (1 << 3)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_PHSENSOR_SETCORRECTIONTEMPERATURE, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvADP1000(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltage, pH;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpack32xS(buf, 24), 5);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_PHSENSOR_PHCHANGE:
			pH = round_double(unpack32xS(buf, 24), 5);
			return (bridgeSendToChannel(ch, BP_PHCHANGE, "%g", pH));
		case VINT_PACKET_TYPE_PHSENSOR_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendVOLTAGERATIOINPUT_PORT(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * 32768));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SETVOLTAGERATIOCHANGETRIGGER, buf, 2));
		case BP_SETSENSORTYPE:
			return (supportedVoltageRatioSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvVOLTAGERATIOINPUT_PORT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltageRatio;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_VOLTAGERATIOCHANGE:
			voltageRatio = round_double(unpacku16xS(buf, 15), 5);
			return (bridgeSendToChannel(ch, BP_VOLTAGERATIOCHANGE, "%g", voltageRatio));
		case VINT_PACKET_TYPE_VOLTAGERATIOINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendVOLTAGEINPUT_PORT(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[4];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
	case PHIDCHUID_HUB_VOLTAGEINPUT_110:
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			pack16(buf, HANDLE_DATAINTERVAL_PKT(bp, 1));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_SAMPLED_SETDATAINTERVAL, buf, 2));
		case BP_SETCHANGETRIGGER:
			pack16(buf, (uint16_t)((getBridgePacketDouble(bp, 0) / 5) * (1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_VOLTAGEINPUT_SETVOLTAGECHANGETRIGGER, buf, 2));
		case BP_SETSENSORTYPE:
			return (supportedVoltageSensorType(ch, getBridgePacketInt32(bp, 0)) ? EPHIDGET_OK : EPHIDGET_INVALIDARG);
		case BP_SETSENSORVALUECHANGETRIGGER:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvVOLTAGEINPUT_PORT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	double voltage;
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
	case PHIDCHUID_HUB_VOLTAGEINPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_VOLTAGEINPUT_VOLTAGECHANGE:
			voltage = round_double(unpacku16xS(buf, 15) * 5.0, 5);
			return (bridgeSendToChannel(ch, BP_VOLTAGECHANGE, "%g", voltage));
		case VINT_PACKET_TYPE_VOLTAGEINPUT_SATURATION:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_SATURATION, "Saturation Detected."));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendVOLTAGEINPUT_PORT_5V25(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendVOLTAGEINPUT_PORT(ch, bp));
}

static PhidgetReturnCode
recvVOLTAGEINPUT_PORT_5V25(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvVOLTAGEINPUT_PORT(ch, buf, len));
}

static PhidgetReturnCode
sendDIGITALOUTPUT_PORT(PhidgetChannelHandle ch, BridgePacket *bp) {
	uint8_t buf[2];

	assert(ch);
	assert(bp);

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	case PHIDCHUID_HUB_DIGITALOUTPUT_110:
		switch (bp->vpkt) {
		case BP_SETSTATE:
			pack16(buf, (uint16_t)(getBridgePacketInt32(bp, 0) ? (unsigned int)(1 << 15) : 0));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETDUTYCYCLE:
			pack16(buf, (uint16_t)(getBridgePacketDouble(bp, 0) * (unsigned int)(1 << 15)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_DIGITALOUTPUT_SETDUTYCYCLE, buf, 2));
		case BP_SETFAILSAFETIME:
			pack16(buf, (uint16_t)(getBridgePacketUInt32(bp, 0)));
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_TIME, buf, 2));
		case BP_FAILSAFERESET:
			return (sendVINTDataPacket(bp->iop, ch, VINT_PACKET_TYPE_FAILSAFE_RESET, buf, 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
recvDIGITALOUTPUT_PORT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
		MOS_PANIC("Unexpected packet recieved.");
		break;
	case PHIDCHUID_HUB_DIGITALOUTPUT_110:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALOUTPUT_FAILSAFE:
			return (bridgeSendToChannel(ch, BP_ERROREVENT, "%d%s", EEPHIDGET_FAILSAFE, "Failsafe procedure initiated."));
		default:
			MOS_PANIC("Unexpected packet type");
		}
	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

static PhidgetReturnCode
sendDIGITALOUTPUT_PORT_FAILSAFE(PhidgetChannelHandle ch, BridgePacket *bp) {

	return (sendDIGITALOUTPUT_PORT(ch, bp));
}

static PhidgetReturnCode
recvDIGITALOUTPUT_PORT_FAILSAFE(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {

	return (recvDIGITALOUTPUT_PORT(ch, buf, len));
}

static PhidgetReturnCode
sendDIGITALINPUT_PORT(PhidgetChannelHandle ch, BridgePacket *bp) {
	MOS_PANIC("Unexpected command recieved.");
}

static PhidgetReturnCode
recvDIGITALINPUT_PORT(PhidgetChannelHandle ch, const uint8_t *buf, size_t len) {
	int pkt;

	assert(buf);

	pkt = buf[0];
	buf++;

	switch (ch->UCD->uid) {
	case PHIDCHUID_HUB_DIGITALINPUT_100:
		switch (pkt) {
		case VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE:
			return (bridgeSendToChannel(ch, BP_STATECHANGE, "%d", 1));
		case VINT_PACKET_TYPE_DIGITALINPUT_STATECHANGE2:
			return (bridgeSendToChannel(ch, BP_STATECHANGE, "%d", 0));
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Invalid Channel UID");
	}
}

#include "vintpackets.gen.c"

const VINTIO_t * const
getVINTIO(unsigned int id) {

	MOS_ASSERT(id < PHIDUID_MAXUID);
	return (&VINTIO[id]);
}
