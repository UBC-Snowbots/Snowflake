/*
 * This file is part of libphidget22
 *
 * Copyright 2015 Phidgets Inc <patrick@phidgets.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/>
 */

#include "phidgetbase.h"

#include "phidget.h"
#include "gpp.h"
#include "usb.h"
#include "manager.h"
#include "util/utils.h"
#include "util/phidgetlog.h"
#include "mos/mos_byteorder.h"

#include <sys/stat.h>
#include <sys/ioctl.h>

static libusb_context *libusb_ctx;

#ifdef NDEBUG
#define usblogerr(...) PhidgetLog_loge(NULL, 0, __func__, "phidget22usb", PHIDGET_LOG_ERROR, __VA_ARGS__)
#define usbloginfo(...) PhidgetLog_loge(NULL, 0, __func__, "phidget22usb", PHIDGET_LOG_INFO, __VA_ARGS__)
#define usblogwarn(...) PhidgetLog_loge(NULL, 0, __func__, "phidget22usb", PHIDGET_LOG_WARNING, __VA_ARGS__)
#define usblogdebug(...)
#define usblogverbose(...)
#else
#define usblogerr(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, "phidget22usb", PHIDGET_LOG_ERROR, __VA_ARGS__)
#define usbloginfo(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, "phidget22usb", PHIDGET_LOG_INFO, __VA_ARGS__)
#define usblogwarn(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, "phidget22usb", PHIDGET_LOG_WARNING, __VA_ARGS__)
#define usblogdebug(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, "phidget22usb", PHIDGET_LOG_DEBUG, __VA_ARGS__)
#define usblogverbose(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, "phidget22usb", PHIDGET_LOG_VERBOSE, __VA_ARGS__)
#endif

#if defined(LIBUSB_API_VERSION) && (LIBUSB_API_VERSION >= 0x01000102)
	#define LIBUSB_ERR_FMT "%s - %s."
	#define LIBUSB_ERR_ARGS(ret) libusb_error_name(ret), libusb_strerror(ret)
	#define LIBUSB_XFERERR_FMT "%s."
	#define LIBUSB_XFERERR_ARGS(ret) libusb_error_name(ret)
#else
	#define LIBUSB_ERR_FMT "0x%02x"
	#define LIBUSB_ERR_ARGS(ret) ret
	#define LIBUSB_XFERERR_FMT "0x%02x"
	#define LIBUSB_XFERERR_ARGS(ret) ret
#endif

#if USB_ASYNC_READS
static void joinHandleEventsThread(void);
#endif

static void
logBuffer(unsigned char *data, int dataLen, const char *message) {
	Phidget_LogLevel ll;
	char str[2000];
	int i, j;

	PhidgetLog_getSourceLevel("phidget22usb", &ll);
	if (ll != PHIDGET_LOG_VERBOSE)
		return;

	str[0]='\0';
	if (dataLen > 0) {
		for (i = 0, j = 0; i < dataLen; i++, j += 6) {
			if (!(i % 8)) {
				str[j] = '\n';
				str[j + 1] = '\t';
				j += 2;
			}
			mos_snprintf(str + j, sizeof (str) - j, "0x%02x, ", data[i]);
		}
		str[j - 2] = '\0'; //delete last ','
	}

	usblogdebug("%s%s", message, str);
}

PhidgetReturnCode
PhidgetUSBCloseHandle(PhidgetUSBConnectionHandle conn) {
	int ret;

	assert(conn);
	assert(conn->deviceHandle);

	usblogdebug("");

#if USB_ASYNC_READS
	PhidgetUSBStopAsyncReads(conn);
#endif

	stopUSBReadThread(conn);

	/* Lock so we do not close the handle while the read thread is using it */
	PhidgetRunLock(conn);
	ret = libusb_release_interface((libusb_device_handle *)conn->deviceHandle, conn->interfaceNum);
	if (ret != 0) {
		switch (ret) {
		case LIBUSB_ERROR_NO_DEVICE:
			//usb_release_interface called after the device was unplugged
			usblogdebug("libusb_release_interface() called on unplugged device.");
			break;
		default:
			usblogerr("libusb_release_interface() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
		}
	}

#if 0
	// XXX: reimplement this elsewere

	//if we notice that PHIDGET_USB_ERROR_FLAG is set, then reset this device before closing
	//this gives us a better chance of getting it back if something has gone wrong.
	if (CPhidget_statusFlagIsSet(phid->status, PHIDGET_USB_ERROR_FLAG)) {
		usblogwarn("PHIDGET_USB_ERROR_FLAG is set - resetting device.");
		if ((ret = libusb_reset_device((libusb_device_handle *)conn->deviceHandle) != 0) {
			usblogerr("libusb_reset_device failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			result = EPHIDGET_UNEXPECTED;
		}
	}
#endif

	PhidgetRunUnlock(conn);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetUSBSendPacket(mosiop_t iop, PhidgetHIDUSBConnectionHandle conn, const unsigned char *buffer, size_t bufferLen) {
	uint8_t buf[MAX_OUT_PACKET_SIZE];
	int BytesWritten = 0, ret;
	int tryagaincnt;

	assert(conn);
	assert(buffer);
	assert(bufferLen <= conn->outputReportByteLength);
	assert(bufferLen < sizeof(buf));
	assert(conn->deviceHandle);

	memcpy(buf, buffer, bufferLen);
	memset(buf + bufferLen, 0, (sizeof(buf)) - bufferLen);

	logBuffer(buf, conn->outputReportByteLength, "Sending USB Packet: ");

	tryagaincnt = 5;
tryagain:
	if (conn->interruptOutEndpoint) {
		ret = libusb_interrupt_transfer((libusb_device_handle *)conn->deviceHandle,
		  LIBUSB_ENDPOINT_OUT | (conn->interfaceNum + 1),
		  buf,
		  conn->outputReportByteLength, /* size */
		  &BytesWritten,
		  500); /* FIXME? timeout */
	} else {
		BytesWritten = libusb_control_transfer((libusb_device_handle *)conn->deviceHandle,
		  LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
		  LIBUSB_REQUEST_SET_CONFIGURATION,
		  0x0200, /* value */
		  conn->interfaceNum, /* index*/
		  buf,
		  conn->outputReportByteLength, /* size */
		  500); /* FIXME? timeout */
		ret = BytesWritten;
	}

	if (ret < 0) {
		switch (ret) {
		case LIBUSB_ERROR_TIMEOUT: //important case?
			if (conn->interruptOutEndpoint && BytesWritten != 0)
				goto sentdata;
			return (EPHIDGET_TIMEOUT);
		case LIBUSB_ERROR_NO_DEVICE:
			//device is gone - unplugged.
			usbloginfo("Device was unplugged - detach.");
			return (MOS_ERROR(iop, EPHIDGET_NOTATTACHED, "USB Device is not attached."));
		case LIBUSB_ERROR_IO:
			if (conn->interruptOutEndpoint)
				usbloginfo("libusb_interrupt_transfer() failed: " LIBUSB_ERR_FMT " Maybe detaching?", LIBUSB_ERR_ARGS(ret));
			else
				usbloginfo("libusb_control_msg() failed: " LIBUSB_ERR_FMT " Maybe detaching?", LIBUSB_ERR_ARGS(ret));
			return (MOS_ERROR(iop, EPHIDGET_IO, "USB Send failed with I/O error. Maybe detaching?"));
		case LIBUSB_ERROR_PIPE:
			if (!conn->interruptOutEndpoint) {
				if(tryagaincnt--) {
					usblogdebug("libusb_control_msg() stalled. Trying again.");
					goto tryagain;
				} else {
					usblogerr("libusb_control_msg() stalled too many times.");
					return (MOS_ERROR(iop, EPHIDGET_IO, "USB Send failed with STALL."));
				}
			}
		default:
			if (conn->interruptOutEndpoint)
				usblogerr("libusb_interrupt_transfer() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			else
				usblogerr("libusb_control_msg() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "USB Send failed with error: %d", ret));
		}
	}

sentdata:
	if (BytesWritten != conn->outputReportByteLength) {
		usblogwarn("Report Length: %d, bytes written: %d",
			(int)conn->outputReportByteLength, (int)BytesWritten);
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "USB Send wrote wrong number of bytes."));
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode PhidgetUSBTransferPhidgetPacket(mosiop_t iop, PhidgetPHIDUSBConnectionHandle conn, PhidgetUSBRequest PhidgetUSBRequest,
  PhidgetUSBPacketType packetType, uint8_t index, uint8_t *buffer, size_t *bufferLen, int timeout) {
	int BytesTransferred = 0, ret;
	int direction;

	assert(conn);
	assert(bufferLen);
	assert(conn->deviceHandle);
	assert(conn->type == PHIDGET_PHIDUSB_CONNECTION);

	if (buffer == NULL && (*bufferLen) != 0)
		return (EPHIDGET_INVALIDARG);

	switch (PhidgetUSBRequest) {
	case PHIDGETUSB_REQ_CHANNEL_WRITE:
	case PHIDGETUSB_REQ_DEVICE_WRITE:
	case PHIDGETUSB_REQ_GPP_WRITE:
	case PHIDGETUSB_REQ_BULK_WRITE:
		direction = LIBUSB_ENDPOINT_OUT;
		break;

	case PHIDGETUSB_REQ_CHANNEL_READ:
	case PHIDGETUSB_REQ_DEVICE_READ:
	case PHIDGETUSB_REQ_GPP_READ:
		direction = LIBUSB_ENDPOINT_IN;
		break;

	default:
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "Invalid PhidgetUSBRequest."));
	}

	if (PhidgetUSBRequest == PHIDGETUSB_REQ_BULK_WRITE) {
		if ((*bufferLen) > conn->pusbParams.maxPacketEP2)
			return (EPHIDGET_INVALIDARG);
		if (conn->pusbParams.ep2type != PHID_EP_BULK)
			return (EPHIDGET_UNSUPPORTED);
	} else {
		// We can't write more than the max packet size. Don't try to read more than the max packet size
		if ((*bufferLen) > conn->pusbParams.maxPacketEP0) {
			if (direction == LIBUSB_ENDPOINT_OUT)
				return (EPHIDGET_INVALIDARG);
			else
				*bufferLen = conn->pusbParams.maxPacketEP0;
		}
	}

	if (PhidgetUSBRequest == PHIDGETUSB_REQ_BULK_WRITE) {
		ret = libusb_bulk_transfer((libusb_device_handle *)conn->deviceHandle,
		  0x02,
		  buffer,
		  *bufferLen, /* size */
		  &BytesTransferred,
		  timeout); /* timeout */
	} else {
		BytesTransferred = libusb_control_transfer((libusb_device_handle *)conn->deviceHandle,
		  direction | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
		  PhidgetUSBRequest,
		  USB_PHIDGET_CH_REQ_MAKE_TYPE_AND_INDEX(packetType, index), /* value */
		  conn->interfaceNum, /* index*/
		  buffer,
		  *bufferLen, /* size */
		  timeout); /* timeout */
		ret = BytesTransferred;
	}

	if (ret < 0) {
		switch (ret) {
		case LIBUSB_ERROR_TIMEOUT:
			usbloginfo("Transfer timed out.");
			return (MOS_ERROR(iop, EPHIDGET_TIMEOUT, "Transfer timed out."));
		case LIBUSB_ERROR_NO_DEVICE:
			//device is gone - unplugged.
			usbloginfo("Device was unplugged - detach.");
			return (MOS_ERROR(iop, EPHIDGET_NOTATTACHED, "USB Device is not attached."));
		case LIBUSB_ERROR_IO:
			if (PhidgetUSBRequest == PHIDGETUSB_REQ_BULK_WRITE)
				usbloginfo("libusb_bulk_transfer() failed: " LIBUSB_ERR_FMT " Maybe detaching?", LIBUSB_ERR_ARGS(ret));
			else
				usbloginfo("libusb_control_msg() failed: " LIBUSB_ERR_FMT " Maybe detaching?", LIBUSB_ERR_ARGS(ret));
			return (MOS_ERROR(iop, EPHIDGET_IO, "USB Send failed with I/O error. Maybe detaching?"));
		case LIBUSB_ERROR_PIPE:
			if (PhidgetUSBRequest == PHIDGETUSB_REQ_BULK_WRITE)
				usblogerr("libusb_bulk_transfer() stalled");
			else
				usblogerr("libusb_control_msg() stalled");
			return (MOS_ERROR(iop, EPHIDGET_IO, "USB Send failed with STALL."));
		default:
			if (PhidgetUSBRequest == PHIDGETUSB_REQ_BULK_WRITE)
				usblogerr("libusb_bulk_transfer() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			else
				usblogerr("libusb_control_msg() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "USB Send failed with error: %d", ret));
		}
	}

	if (direction == LIBUSB_ENDPOINT_OUT) {
		if (BytesTransferred != (int)(*bufferLen)) {
			usblogerr("Failure in PhidgetUSBSendPacket - Packet Length %d, Bytes Written: %d", (*bufferLen), (int)BytesTransferred);
			return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "USB send failed to write expected number of bytes."));
		}
	} else {
		(*bufferLen) = BytesTransferred;
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetUSBGetDeviceDescriptor(PhidgetUSBConnectionHandle conn, int type, int index, uint8_t *buf, size_t *bufLen) {
	int ret;

	TESTPTR(buf);
	TESTPTR(bufLen);

	assert(conn);
	assert(conn->deviceHandle);

	ret = libusb_control_transfer((libusb_device_handle *)conn->deviceHandle,
	  LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_DEVICE,
	  LIBUSB_REQUEST_GET_DESCRIPTOR,
	  (type << 8) | index,
	  0,
	  buf,
	  *bufLen,
	  500 /* ms timeout */);

	if (ret < 0) {
		usblogerr("libusb_control_transfer() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
		return (EPHIDGET_UNEXPECTED);
	}

	(*bufLen) = ret;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetUSBSetLabel(PhidgetDeviceHandle device, char *buffer) {
	PhidgetUSBConnectionHandle conn;
	int BytesWritten;
	int size;
	int ret;

	assert(device != NULL);

	if (deviceSupportsGeneralPacketProtocol(device))
		return (GPP_setLabel(NULL, device, buffer));

	conn = PhidgetUSBConnectionCast(device->conn);
	assert(conn);
	assert(conn->deviceHandle);

	size = buffer[0];
	if (size > 22)
		return (EPHIDGET_INVALID);

	ret = libusb_control_transfer((libusb_device_handle *)conn->deviceHandle,
	  LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_DEVICE,
	  LIBUSB_REQUEST_SET_DESCRIPTOR,
	  0x0304, /* value */
	  0x0409, /* index*/
	  (unsigned char *)buffer,
	  size, /* size */
	  500); /* timeout */

	if (ret < 0) {
		switch (ret) {
		case LIBUSB_ERROR_TIMEOUT: //important case?
			usbloginfo("libusb_control_transfer() timeout (500ms)");
			return (EPHIDGET_UNSUPPORTED);
		default:
			usbloginfo("libusb_control_transfer() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			return (EPHIDGET_UNSUPPORTED);
		}
	}

	BytesWritten = ret;

	if (BytesWritten != size) {
		usblogwarn("Report Length: %d, bytes written: %d", size, (int)BytesWritten);
		return (EPHIDGET_UNEXPECTED);
	}

	return (EPHIDGET_OK);
}


// Testing for HUB0000 stall bug
//#define DEBUG_HUB0000_BUG

#ifdef DEBUG_HUB0000_BUG

typedef struct {
	PhidgetUSBConnectionHandle conn;
	int index;
} t_func_param;

static MOS_TASK_RESULT
t_func(void *param) {
	char str[256];
	int ret;

	t_func_param *t = (t_func_param *)param;

	ret = libusb_get_string_descriptor_ascii((libusb_device_handle *)t->conn->deviceHandle, t->index, (uint8_t *)str, sizeof(str));

	if (ret < 0)
		usblogdebug("Read string from t_func thread failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));

	free(t);
}

#endif

PhidgetReturnCode
PhidgetUSBGetString(PhidgetUSBConnectionHandle conn, int index, char *str) {
	int tryagaincnt;
	int ret;

// XXX - trying to cause a crash in HUB0000 firmare
#ifdef DEBUG_HUB0000_BUG
	t_func_param *t_param = malloc(sizeof(t_func_param));
	t_param->conn = conn;
	t_param->index = index;
	mos_task_t t;
	mos_task_create(&t, t_func, t_param);
#endif

	tryagaincnt = 5;
tryagain:
	ret = libusb_get_string_descriptor_ascii((libusb_device_handle *)conn->deviceHandle, index, (uint8_t *)str, 256);
	if (ret < 0) {
		switch (ret) {
		case LIBUSB_ERROR_NO_DEVICE:
			usbloginfo("Device was unplugged - detach.");
			return (EPHIDGET_NOTATTACHED);
		case LIBUSB_ERROR_IO:
			usbloginfo("libusb_get_string_descriptor_ascii() failed: " LIBUSB_ERR_FMT " Maybe detaching?", LIBUSB_ERR_ARGS(ret));
			return (EPHIDGET_IO);
		case LIBUSB_ERROR_PIPE:
			if (tryagaincnt--) {
				usblogdebug("libusb_get_string_descriptor_ascii() stalled. Trying again.");
				goto tryagain;
			} else {
				usblogerr("libusb_get_string_descriptor_ascii() stalled too many times.");
				return (EPHIDGET_IO);
			}
		default:
			usblogerr("libusb_get_string_descriptor_ascii() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			return (EPHIDGET_UNEXPECTED);
		}
	}

	//usblogdebug("Read USB String #%d: %s", index, str);

	return EPHIDGET_OK;
}

/* Buffer should be at least 8 bytes long */
#define USB_READ_TIMEOUT 500 // 500ms
PhidgetReturnCode
PhidgetUSBReadPacket(PhidgetUSBConnectionHandle conn, unsigned char *buffer, size_t *length) {
	int BytesRead = 0, ret;

#if USB_ASYNC_READS
	PhidgetUSBTransferHandle usbXfer;
	mostime_t tm;
#endif

	assert(conn);
	assert(conn->deviceHandle);

	PhidgetRunLock(conn);

#if USB_ASYNC_READS

	tm = mos_gettime_usec() + (USB_READ_TIMEOUT * 1000);
	mos_mutex_lock(&conn->xferQueueLock);

	while (MSMTAILQ_EMPTY(&conn->xferQueue)) {

		// Wait for up to USB_READ_TIMEOUT ms for incoming data
		mos_cond_timedwait(&conn->xferQueueCond, &conn->xferQueueLock, USB_READ_TIMEOUT * MOS_MSEC);

		// This means we are shutting down, don't bother waiting for more packets
		if (!conn->usingAsyncReads) {
			mos_mutex_unlock(&conn->xferQueueLock);
			PhidgetRunUnlock(conn);
			usblogdebug("Returning early because usingAsyncReads is false");
			return (EPHIDGET_INTERRUPTED);
		}

		if (mos_gettime_usec() > tm) {
			mos_mutex_unlock(&conn->xferQueueLock);
			PhidgetRunUnlock(conn);
			return (EPHIDGET_TIMEOUT);
		}
	}

	usbXfer = MSMTAILQ_FIRST(&conn->xferQueue);
	MSMTAILQ_REMOVE_HEAD(&conn->xferQueue, link);
	conn->queueCnt--;

	mos_mutex_unlock(&conn->xferQueueLock);

	if (usbXfer->result == LIBUSB_SUCCESS) {
		memcpy(buffer, usbXfer->buffer, usbXfer->actual_length);
		BytesRead = usbXfer->actual_length;
	}

	ret = usbXfer->result;
	mos_free(usbXfer, sizeof(PhidgetUSBTransfer));

	// This means the transfer was asynchronously cancelled
	if (ret == LIBUSB_ERROR_INTERRUPTED) {
		PhidgetRunUnlock(conn);
		usblogdebug("Returning early because of cancelled transfer");
		return (EPHIDGET_INTERRUPTED);
	}

#else
	ret = libusb_interrupt_transfer((libusb_device_handle *)conn->deviceHandle,
	  LIBUSB_ENDPOINT_IN | (conn->interfaceNum + 1),
	  buffer,
	  *length,
	  &BytesRead,
	  USB_READ_TIMEOUT);
#endif

	PhidgetRunUnlock(conn);

	if (ret != 0) {
		switch (ret) {
			// A timeout occured, but we'll just try again
		case LIBUSB_ERROR_TIMEOUT:
			if (BytesRead != 0)
				goto gotdata;
			return (EPHIDGET_TIMEOUT);
		case LIBUSB_ERROR_BUSY:
			/*
			 * This happens when someone else calls claim_interface on this interface
			 * (a manager for ex.) - basically just wait until they release it.
			 *
			 * This will happen if an open occurs in another app which (for some reason)
			 * can steal the interface from this one.
			 */
			usbloginfo("Device is busy on Read - try again.");
			return (EPHIDGET_AGAIN);
		case LIBUSB_ERROR_NO_DEVICE:
			//device is gone - unplugged.
			usbloginfo("Device was unplugged - detach.");
			return (EPHIDGET_NOTATTACHED);
		case LIBUSB_ERROR_IO:
			usbloginfo("libusb_interrupt_transfer() failed: " LIBUSB_ERR_FMT " Maybe detaching?", LIBUSB_ERR_ARGS(ret));
			goto tryagain;
		case LIBUSB_ERROR_PIPE:
		case LIBUSB_ERROR_OVERFLOW:
		default:
			usblogerr("libusb_interrupt_transfer() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			goto tryagain;
		}
	}

gotdata:
	switch (conn->type) {
	case PHIDGET_HIDUSB_CONNECTION:
		if (BytesRead != ((PhidgetHIDUSBConnectionHandle)conn)->inputReportByteLength) {
			/*
			 * Generally means the device was unplugged, but can mean that there is not enough
			 * Interrupt bandwidth. We keep trying and we'll get data, just not all data.
			 */
			usblogwarn("Report Length: %d, bytes read: %d. "
					   "Probably trying to use too many Phidgets at once, and some data is being lost.",
					   (int)((PhidgetHIDUSBConnectionHandle)conn)->inputReportByteLength, (int)BytesRead);
			goto tryagain;
		}
		break;

	case PHIDGET_PHIDUSB_CONNECTION:
		break;
	}

	logBuffer(buffer, BytesRead, "Received USB Packet: ");

	conn->tryAgainCounter = 0;
	*length = BytesRead;

	return (EPHIDGET_OK);

	/*
	 * If we see too many tryagains in a row, then we assume something has actually gone wrong
	 * and reset the device
	 */
tryagain:

	conn->tryAgainCounter++;
	if (conn->tryAgainCounter > 25) {
		usblogerr("EPHIDGET_AGAIN returned too many times in a row - reset device.");
		conn->tryAgainCounter = 0;
		return (EPHIDGET_UNEXPECTED);
	}
	usleep(50000);
	return (EPHIDGET_AGAIN);
}

static int
getLabel(HANDLE handle, char *label, int serialNumber, int index) {
	struct libusb_device_descriptor	desc;
	libusb_device *device;
	uint8_t labelBuf[22];
	int ret;

	// This means label is not supported by this Phidget
	if (index == 0) {
		memset(label, 0, MAX_LABEL_STORAGE);
		return (EPHIDGET_OK);
	}

	memset(labelBuf, 0, sizeof(labelBuf));
	device = libusb_get_device(handle);

	if ((ret = libusb_get_device_descriptor(device, &desc)) != 0) {
		usblogerr("libusb_get_device_descriptor() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
		return (EPHIDGET_UNEXPECTED);
	}

	// This is presumably to detect NXP M3 phidgets in bootloader mode??
	if (desc.iSerialNumber == 3) {
		//Note that this returns the whole descriptor, including the length and type bytes
		ret = libusb_get_string_descriptor(handle, 4, 0, labelBuf, sizeof(labelBuf));
		if (ret < 0) {
			switch (ret) {
			case LIBUSB_ERROR_TIMEOUT: //important case?
			default:
				usbloginfo("libusb_get_string_descriptor() failed reading label: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
				usbloginfo("device may not support labels");
			}
		} else {
			return (decodeLabelString((char *)labelBuf, label, serialNumber));
		}
	}

	memset(label, 0, MAX_LABEL_STORAGE);
	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetUSBRefreshLabelString(PhidgetDeviceHandle device) {
	PhidgetPHIDUSBConnectionHandle phidusbConn;
	PhidgetUSBConnectionHandle conn;
	int index;

	assert(device);
	conn = PhidgetUSBConnectionCast(device->conn);
	assert(conn);

	switch (device->connType) {
	case PHIDCONN_HIDUSB:
		index = 4;
		break;
	case PHIDCONN_PHIDUSB:
		phidusbConn = PhidgetPHIDUSBConnectionCast(device->conn);
		assert(phidusbConn);
		index = phidusbConn->pusbParams.labelIndex;
		break;
	default:
		return (EPHIDGET_UNEXPECTED);
	}

	return getLabel((libusb_device_handle *)conn->deviceHandle, device->deviceInfo.label, device->deviceInfo.serialNumber, index);
}

void
PhidgetUSBUninit() {

	if (libusb_ctx) {
		usbloginfo("Deinitializing libusb");
#if USB_ASYNC_READS
		joinHandleEventsThread();
#endif
		libusb_exit(libusb_ctx);
		libusb_ctx = NULL;
	}
}

static PhidgetReturnCode
readphidgetDeviceParams(libusb_device *device, PhidgetConnectionType *type, PhidgetUSBDeviceParamsHandle params) {
	struct libusb_config_descriptor *configDesc;
	const uint8_t *descEnd, *desc;
	const USBD_PhidgetDeviceDescStruct *phidgetDeviceDesc = NULL;
	const USBD_PhidgetEndpointDescStruct *phidgetEndpointDesc = NULL;
	int ret;
	int i;

	// Read out the full configuration descriptor
	ret = libusb_get_active_config_descriptor(device, &configDesc);
	if (ret < 0) {
		usblogerr("libusb_get_active_config_descriptor() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
		return (EPHIDGET_UNEXPECTED);
	}

	// 1 or 2
	if (configDesc->bNumInterfaces < 1 || configDesc->bNumInterfaces > 2) {
		usblogerr("Wrong number of interfaces!");
		libusb_free_config_descriptor(configDesc);
		return (EPHIDGET_UNEXPECTED);
	}

	// 1
	if (configDesc->interface[0].num_altsetting != 1) {
		usblogerr("Wrong number of alt settings!");
		libusb_free_config_descriptor(configDesc);
		return (EPHIDGET_UNEXPECTED);
	}

	if (configDesc->interface[0].altsetting[0].bInterfaceClass == 0xFF) { // Vendor-defined
		*type = PHIDCONN_PHIDUSB;
	} else if (configDesc->interface[0].altsetting[0].bInterfaceClass == 0x03) { // HID
		*type = PHIDCONN_HIDUSB;
	} else {
		usblogerr("Invalid bInterfaceClass");
		libusb_free_config_descriptor(configDesc);
		return (EPHIDGET_UNEXPECTED);
	}

	// For HID Phidgets, we're done!
	if (*type != PHIDCONN_PHIDUSB) {
		libusb_free_config_descriptor(configDesc);
		return (EPHIDGET_OK);
	}

	params->maxPacketEP0 = 0;
	params->ep1type = PHID_EP_UNAVAILABLE;
	params->maxPacketEP1 = 0;
	params->ep2type = PHID_EP_UNAVAILABLE;
	params->maxPacketEP2 = 0;

	// up to 2
	if (configDesc->interface[0].altsetting[0].bNumEndpoints > 2) {
		usblogerr("Wrong number of endpoints!");
		libusb_free_config_descriptor(configDesc);
		return (EPHIDGET_UNEXPECTED);
	}

	for (i = 0; i < configDesc->interface[0].altsetting[0].bNumEndpoints; i++) {

		if (configDesc->interface[0].altsetting[0].endpoint[i].bEndpointAddress == 0x81) { // bEndpointAddress
			// EP1 IN
			//params->maxPacketEP1 = endpointDesc->wMaxPacketSize;
			if (configDesc->interface[0].altsetting[0].endpoint[i].bmAttributes == LIBUSB_TRANSFER_TYPE_BULK) { // bmAttributes
				params->ep1type = PHID_EP_BULK;
			} else if (configDesc->interface[0].altsetting[0].endpoint[i].bmAttributes == LIBUSB_TRANSFER_TYPE_INTERRUPT) { // bmAttributes
				params->ep1type = PHID_EP_INTERRUPT;
			} else {
				usblogerr("Wrong endpoint type!");
				libusb_free_config_descriptor(configDesc);
				return (EPHIDGET_UNEXPECTED);
			}
		} else if (configDesc->interface[0].altsetting[0].endpoint[i].bEndpointAddress == 0x02) { // bEndpointAddress
			// EP2 OUT
			params->maxPacketEP2 = configDesc->interface[0].altsetting[0].endpoint[i].wMaxPacketSize; // wMaxPacketSize
			params->ep2type = PHID_EP_BULK;
		} else {
			usblogerr("Wrong endpoint address!");
			libusb_free_config_descriptor(configDesc);
			return (EPHIDGET_UNEXPECTED);
		}

		// Iterate over the extra descriptors
		desc = configDesc->interface[0].altsetting[0].endpoint[i].extra;
		descEnd = desc + configDesc->interface[0].altsetting[0].endpoint[i].extra_length;
		while (desc < descEnd) {
			// Switch on bDescriptorType
			switch (desc[1]) {
				case USB_DESC_TYPE_PHIDGET_ENDPOINT:
					if (desc[0] != sizeof(USBD_PhidgetEndpointDescStruct)) {
						usblogerr("Error parsing extra descriptors!");
						libusb_free_config_descriptor(configDesc);
						return (EPHIDGET_UNEXPECTED);
					}

					phidgetEndpointDesc = (const USBD_PhidgetEndpointDescStruct *)desc;

					// This phidget endpoint descriptor enhances the preceding endpoint descriptor
					if ((configDesc->interface[0].altsetting[0].endpoint[i].bEndpointAddress & 0x7F) == 0x01) // bEndpointAddress
						params->maxPacketEP1 = mos_le16toh(phidgetEndpointDesc->wMaxPacketSize);
					if ((configDesc->interface[0].altsetting[0].endpoint[i].bEndpointAddress & 0x7F) == 0x02) // bEndpointAddress
						params->maxPacketEP2 = mos_le16toh(phidgetEndpointDesc->wMaxPacketSize);

					break;
				default:
					usblogerr("Error parsing extra descriptors!");
					libusb_free_config_descriptor(configDesc);
					return (EPHIDGET_UNEXPECTED);
			}
			// Advance to the next descriptor (bLength)
			desc += desc[0];
		}
	}

	// Iterate over the extra descriptors
	desc = configDesc->extra;
	descEnd = desc + configDesc->extra_length;
	while (desc < descEnd) {
		// Switch on bDescriptorType
		switch (desc[1]) {
			case USB_DESC_TYPE_PHIDGET_DEVICE:
				if (desc[0] != sizeof(USBD_PhidgetDeviceDescStruct)) {
					usblogerr("Error parsing extra descriptors!");
					libusb_free_config_descriptor(configDesc);
					return (EPHIDGET_UNEXPECTED);
				}

				phidgetDeviceDesc = (const USBD_PhidgetDeviceDescStruct *)desc;

				if (mos_le16toh(phidgetDeviceDesc->bcdVersion) != USBD_PHIDGET_PROTO_VERSION) {
					usblogwarn("Unknown Phidget descriptor version: 0x%04x - Library upgrade may be required.",
					  mos_le16toh(phidgetDeviceDesc->bcdVersion));
					libusb_free_config_descriptor(configDesc);
					return (EPHIDGET_UNSUPPORTED);
				}

				params->labelIndex = phidgetDeviceDesc->iLabel;
				params->skuIndex = phidgetDeviceDesc->iSKU;
				params->maxPacketEP0 = mos_le16toh(phidgetDeviceDesc->wMaxPacketSize);

				break;

			default:
				usblogerr("Error parsing extra descriptors!");
				libusb_free_config_descriptor(configDesc);
				return (EPHIDGET_UNEXPECTED);
		}
		// Advance to the next descriptor (bLength)
		desc += desc[0];
	}

	libusb_free_config_descriptor(configDesc);

	if (phidgetDeviceDesc == NULL) {
		usblogerr("Couldn't find Phidget device descriptor!");
		return (EPHIDGET_UNEXPECTED);
	}

	assert(params->maxPacketEP0 <= MAX_USB_OUT_PACKET_SIZE);
	assert(params->maxPacketEP1 <= MAX_USB_BULK_INTERRUPT_IN_PACKET_SIZE);

	// For now, limit to max bridge packet array size
	assert(params->maxPacketEP2 <= 8192);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetUSBScanDevices(void) {
	struct libusb_device_descriptor	desc;
	const PhidgetUniqueDeviceDef *pdd;
	PhidgetUSBConnectionHandle conn;
	libusb_device_handle *handle;
	PhidgetDeviceHandle phid;
	libusb_device *device;
	libusb_device **list;
	char unique_name[20];
	int serialNumber;
	ssize_t cnt;
	int version;
	int found;
	int ret;
	int pid;
	int vid;
	int j;

	PhidgetPHIDUSBConnectionHandle phidusbConn;
	PhidgetUSBDeviceParams phidDevParams;
	PhidgetConnectionType connType;
	PhidgetReturnCode res;

	char label[MAX_LABEL_STORAGE];
	uint8_t productString[64];
	uint8_t skuString[64];
	uint8_t string[256];

	static int initFailures;

	if (initFailures > 10)
		return (EPHIDGET_UNEXPECTED);

	list = NULL;

	if (!libusb_ctx) {
		usbloginfo("Initializing libusb");
		if ((ret = libusb_init(&libusb_ctx)) != 0) {
			usblogerr("libusb_init failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			libusb_ctx = NULL;
			initFailures++;
			return (EPHIDGET_UNEXPECTED);
		}
		initFailures = 0;
	}


	ret = libusb_get_device_list(libusb_ctx, &list);
	if (ret < 0) {
		usblogerr("libusb_get_device_list failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
		goto done;
	}

	cnt = ret;

	//search through all USB devices
	for (j = 0; j < cnt; j++) {
		device = list[j];
		found = 0;

		snprintf(unique_name, 20, "%d/%d", libusb_get_bus_number(device), libusb_get_device_address(device));

		/*
		 * If the device is already in the attached list, flag it and move onto the next.
		 */
		PhidgetReadLockDevices();
		FOREACH_DEVICE(phid) {
			if (phid->connType != PHIDCONN_PHIDUSB && phid->connType != PHIDCONN_HIDUSB)
				continue;

			conn = PhidgetUSBConnectionCast(phid->conn);
			assert(conn);

			if (!strcmp(conn->uniqueName, unique_name)) {
				PhidgetSetFlags(phid, PHIDGET_SCANNED_FLAG);
				found = 1;
			}
		}
		PhidgetUnlockDevices();

		if (found)
			continue;

		if ((ret = libusb_get_device_descriptor(device, &desc)) != 0) {
			usblogerr("libusb_get_device_descriptor() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			continue;
		}

		pid = desc.idProduct;
		vid = desc.idVendor;

		// pre-filter on vid/pid
		if (vid == USBVID_PHIDGETS) {
			if (pid < USBPID_PHIDGETS_MIN || pid >USBPID_PHIDGETS_MAX)
				continue;
		} else if (vid == USBVID_OLD) {
			; //pass
		} else {
			continue;
		}

		res = readphidgetDeviceParams(device, &connType, &phidDevParams);
		if (res != EPHIDGET_OK) {
			usblogerr("Failed to read device params");
			continue;
		}

		switch (connType) {
			case PHIDCONN_HIDUSB:
				//decode Phidgets version from release number
				if (desc.bcdDevice < 0x100)
					version = desc.bcdDevice * 100;
				else
					version = ((desc.bcdDevice >> 8) * 100) + ((desc.bcdDevice & 0xff));
				break;

			case PHIDCONN_PHIDUSB:
				// BCD -> Decimal (up to 4 digits)
				version = (((desc.bcdDevice >> 12) & 0x0F) * 1000)
					+ (((desc.bcdDevice >> 8) & 0x0F) * 100)
					+ (((desc.bcdDevice >> 4) & 0x0F) * 10)
					+ (desc.bcdDevice & 0x0F);
				break;
		}

		//logdebug("Device %d: %04x %04x", j, desc.idVendor, desc.idProduct);


	again:
		/*
		 * NOTE: we don't stop when we find a device, because there can be multiple matches
		 * for a device with multiple interfaces.
		 */
		for (pdd = Phidget_Unique_Device_Def; ((int)pdd->type) != END_OF_LIST; pdd++) {
			if (!(pdd->type == PHIDTYPE_USB
				&& vid == pdd->vendorID && pid == pdd->productID
				&& version >= pdd->versionLow && version < pdd->versionHigh))
				continue;

			usbloginfo("New Phidget found in PhidgetUSBBuildList: %s", unique_name);

			found = 1;

			ret = libusb_open(device, &handle);
			if (ret != 0) {
				usblogwarn("libusb_open() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
				usbloginfo("This usually means you need to run as root, or install the udev rules.");
				continue;
			}

			ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, string, sizeof(string));
			if (ret < 0) {
				usblogerr("libusb_get_string_descriptor_ascii() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
				libusb_close(handle);
				continue;
			}
			serialNumber = atol((const char *)string);

			ret = libusb_get_string_descriptor_ascii(handle, desc.iProduct, productString, sizeof(productString));
			if (ret < 0) {
				// Special case for an old 1012 with broken String Descriptor
				if (vid == 0x06C2 && pid == 0x44 && version == 601) {
					productString[0] = '\0';
				} else {
					usblogerr("libusb_get_string_descriptor_ascii() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
					libusb_close(handle);
					continue;
				}
			}

			switch (connType) {
			case PHIDCONN_HIDUSB:
				getLabel(handle, label, serialNumber, 4);
				res = createPhidgetHIDUSBDevice(pdd, version, label, serialNumber, unique_name, (char *)productString, &phid);
				break;

			case PHIDCONN_PHIDUSB:
				getLabel(handle, label, serialNumber, phidDevParams.labelIndex);

				ret = libusb_get_string_descriptor_ascii(handle, phidDevParams.skuIndex, skuString, sizeof(skuString));
				if (ret < 0) {
					usblogerr("libusb_get_string_descriptor_ascii() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
					libusb_close(handle);
					continue;
				}

				res = createPhidgetPHIDUSBDevice(pdd, version, label, serialNumber, unique_name, (char *)skuString, &phid);
				if (res == EPHIDGET_OK) {
					phidusbConn = PhidgetPHIDUSBConnectionCast(phid->conn);
					assert(phidusbConn);
					phidusbConn->pusbParams = phidDevParams;
				}
				break;
			}

			if (res != EPHIDGET_OK) {
				usblogerr("failed to create phidget device handle: "PRC_FMT, PRC_ARGS(ret));
				libusb_close(handle);
				continue;
			}

			libusb_ref_device(device); // we increase the reference count so the device isn't freed

			conn = PhidgetUSBConnectionCast(phid->conn);
			conn->dev = device;
			conn->deviceHandle = (HANDLE)handle;

			PhidgetSetFlags(phid, PHIDGET_SCANNED_FLAG);
			ret = deviceAttach(phid, 1);

			PhidgetRelease((void **)&phid); /* release our reference */
		}

		if (!found) {
			/*
			 * Might be a Phidget, but not one known by this version of the library: log if that is the case.
			 */
			if (desc.idVendor == USBVID_PHIDGETS && desc.idProduct >= USBPID_PHIDGETS_MIN &&
			  desc.idProduct <= USBPID_PHIDGETS_MAX) {
				usblogwarn("A USB Phidget (PID: 0x%04x Version: %d) was found that is not supported by "
				  "the library. A library upgrade is required to work with this Phidget",
				  desc.idProduct, version);

				if (pid != USBID_0xaf) {
					pid = USBID_0xaf;
					goto again;
				}
			}
		}
	} /* iterate over USB devices */

done:
	if (list)
		libusb_free_device_list(list, 1);
	return (ret);
}

/*
 * Got this from libusb-0.1 because 1.0 doesn't expose driver name!
 */
#define USB_MAXDRIVERNAME 255

struct usb_getdriver {
	unsigned int interface;
	char driver[USB_MAXDRIVERNAME + 1];
};

struct linux_device_handle_priv {
	int fd;
};

struct list_head {
	struct list_head *prev, *next;
};

struct libusb_device_handle_internal {
	pthread_mutex_t lock;
	unsigned long claimed_interfaces;
	struct list_head list;
	void *dev;
	unsigned char os_priv[];
};

static PhidgetReturnCode
getReportLengths(PhidgetHIDUSBConnectionHandle conn) {
	const struct libusb_interface_descriptor *interfaceDesc;
	struct libusb_config_descriptor *configDesc;
	unsigned char buf[255];
	int i, j;
	int len;
	int ret;

	memset(buf, 0, sizeof(buf));

	ret = libusb_get_active_config_descriptor(libusb_get_device((libusb_device_handle *)conn->deviceHandle), &configDesc);
	if (ret != 0) {
		usblogerr("libusb_get_active_config_descriptor() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
		return (EPHIDGET_UNEXPECTED);
	}

	interfaceDesc = NULL;
	for (i = 0; i < configDesc->bNumInterfaces; i++) {
		for (j = 0; j < configDesc->interface[i].num_altsetting; j++) {
			if (configDesc->interface[i].altsetting[j].bInterfaceNumber == conn->interfaceNum) {
				interfaceDesc = &configDesc->interface[i].altsetting[j];
				break;
			}
		}
	}

	if (interfaceDesc == NULL) {
		usblogerr("Couldn't find interface descriptor!");
		return (EPHIDGET_UNEXPECTED);
	}

	if (interfaceDesc->bNumEndpoints == 2) {
		usbloginfo("Using Interrupt OUT Endpoint for Host->Device communication.");
		conn->interruptOutEndpoint = PTRUE;
	} else {
		usbloginfo("Using Control Endpoint for Host->Device communication.");
		conn->interruptOutEndpoint = PFALSE;
	}

	libusb_free_config_descriptor(configDesc);

	ret = libusb_control_transfer((libusb_device_handle *)conn->deviceHandle, LIBUSB_ENDPOINT_IN + 1,
	  LIBUSB_REQUEST_GET_DESCRIPTOR, (LIBUSB_DT_REPORT << 8) + 0, conn->interfaceNum, buf,
	  sizeof(buf), 500 /* ms timeout */);

	if (ret < 0) {
		usblogerr("libusb_control_transfer() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
		return (EPHIDGET_UNEXPECTED);
	}

	len = ret;

	if (len < 10) {
		usblogerr("failed to get report lengths");
		return (EPHIDGET_UNEXPECTED);
	}

	for (i = 10; i < len; i++) {
		if (buf[i] == 0x81 && buf[i - 2] == 0x95)
			conn->inputReportByteLength = buf[i - 1];
		else if (buf[i] == 0x81 && buf[i - 4] == 0x95)
			conn->inputReportByteLength=buf[i - 3];

		if (buf[i] == 0x91 && buf[i - 2] == 0x95)
			conn->outputReportByteLength = buf[i - 1];
		else if (buf[i] == 0x91 && buf[i - 4] == 0x95)
			conn->outputReportByteLength = buf[i - 3];
	}

	return (EPHIDGET_OK);
}

static int
detachDriver(PhidgetDeviceHandle device, struct libusb_device_handle *handle) {
	int ret;

	ret = libusb_detach_kernel_driver(handle, device->deviceInfo.UDD->interfaceNum);
	if (ret != 0) {
		usblogwarn("libusb_detach_kernel_driver() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
		return (EPHIDGET_UNEXPECTED);
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetUSBOpenHandle(PhidgetDeviceHandle device) {
	PhidgetUSBConnectionHandle conn;
	PhidgetHIDUSBConnectionHandle hidusbConn;
	//PhidgetPHIDUSBConnectionHandle phidusbConn;
	int ret;

	assert(device);
	conn = PhidgetUSBConnectionCast(device->conn);
	assert(conn);

	ret = libusb_kernel_driver_active((libusb_device_handle *)conn->deviceHandle, device->deviceInfo.UDD->interfaceNum);
	if (ret < 0)
		usblogwarn("libusb_kernel_driver_active() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
	else if (ret == 1)
		detachDriver(device, conn->deviceHandle);

	ret = libusb_claim_interface((libusb_device_handle *)conn->deviceHandle, device->deviceInfo.UDD->interfaceNum);
	if (ret != 0) {
		if (ret == LIBUSB_ERROR_BUSY) {
			usblogwarn("libusb_claim_interface() failed with BUSY - the device may already be open");
			return (EPHIDGET_BUSY);
		} else {
			usblogwarn("libusb_claim_interface() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			return (EPHIDGET_UNEXPECTED);
		}
	}

	conn->interfaceNum = device->deviceInfo.UDD->interfaceNum;

	switch (device->connType) {

	case PHIDCONN_HIDUSB:
		hidusbConn = PhidgetHIDUSBConnectionCast(device->conn);

		ret = getReportLengths(hidusbConn);
		if (ret != 0) {
			libusb_release_interface((libusb_device_handle *)conn->deviceHandle, device->deviceInfo.UDD->interfaceNum);
			return (EPHIDGET_UNEXPECTED);
		}
		break;

	case PHIDCONN_PHIDUSB:
		//phidusbConn = PhidgetPHIDUSBConnectionCast(device->conn);
		// XXX
		break;
	}

	return (EPHIDGET_OK);
}


#if USB_ASYNC_READS

#define INCREASE_USB_EVENT_THREAD_PRIORITY

// XXX - may wish to tune this according to the interrupt rate of each device
#define XFER_CNT	32

/* libusb < 1.0.9 doesn't have libusb_handle_events_timeout_completed */
// XXX - for now I'm using LIBUSB_API_VERSION which was introduced in libusb 1.0.13
//  There doesn't seem to be a good compile-time way of checking version before 1.0.13
#if !defined(LIBUSB_API_VERSION)
#define libusb_handle_events_timeout_completed(ctx, tv, c) \
	libusb_handle_events_timeout(ctx, tv)
#endif

static MOS_TASK_RESULT HandleEventsThreadFunction(void *);
static mos_task_t handleEventsThread;
static int handleEventsThreadRun;
static mos_mutex_t handleEventsThreadLock;
static mos_cond_t handleEventsThreadCond;

static int async_cancel;

static int eventThreadRefCnt;

void PhidgetUSBInit(void) {
	mos_mutex_init(&handleEventsThreadLock);
	mos_cond_init(&handleEventsThreadCond);
}
void PhidgetUSBFini(void){
	mos_mutex_destroy(&handleEventsThreadLock);
	mos_cond_destroy(&handleEventsThreadCond);
}

static PhidgetReturnCode
StartHandleEventsThread() {
	PhidgetReturnCode res;

	mos_mutex_lock(&handleEventsThreadLock);

	eventThreadRefCnt++;

	if (handleEventsThreadRun == 1) {
		mos_mutex_unlock(&handleEventsThreadLock);
		return (EPHIDGET_OK);
	}

	handleEventsThreadRun = 1;
	async_cancel = 0;
	res = mos_task_create(&handleEventsThread, HandleEventsThreadFunction, NULL);
	if (res != 0) {
		handleEventsThreadRun = 0;
		eventThreadRefCnt = 0;
		mos_mutex_unlock(&handleEventsThreadLock);
		return (res);
	}

	mos_mutex_unlock(&handleEventsThreadLock);

	return (EPHIDGET_OK);
}

static void
StopHandleEventsThread() {

	mos_mutex_lock(&handleEventsThreadLock);
	eventThreadRefCnt--;

	if (eventThreadRefCnt > 0) {
		mos_mutex_unlock(&handleEventsThreadLock);
		return;
	}

	// Signal thread to exit
	async_cancel = 1;
	if (handleEventsThreadRun != 0)
		handleEventsThreadRun = 2;
	mos_mutex_unlock(&handleEventsThreadLock);
}

static void
joinHandleEventsThread() {

	mos_mutex_lock(&handleEventsThreadLock);

	// Signal thread to exit and wait for it
	async_cancel = 1;
	while (handleEventsThreadRun != 0) {
		handleEventsThreadRun = 2;
		mos_cond_wait(&handleEventsThreadCond, &handleEventsThreadLock);
	}
	mos_mutex_unlock(&handleEventsThreadLock);

}

static int
needHandleEventsThread() {

	if (handleEventsThreadRun != 1)
		return (0);

	if (eventThreadRefCnt == 0)
		return (0);

	return (1);
}

static MOS_TASK_RESULT
HandleEventsThreadFunction(void *_param) {
	struct timeval tv;
	int ret;

	mos_task_setname("Phidget22 USB Handle Events Thread");
	logdebug("USB Handle Events Thread started: 0x%08x", mos_self());

#ifdef INCREASE_USB_EVENT_THREAD_PRIORITY
	pthread_t this_thread = pthread_self();
	struct sched_param params = {0};
	int max_priority = sched_get_priority_max(SCHED_RR);
	logdebug("Max allowed priority: %d", max_priority);
	if (max_priority > 32)
		params.sched_priority = 32;
	else
		params.sched_priority = max_priority;
	ret = pthread_setschedparam(this_thread, SCHED_RR, &params);
	if (ret != 0)
		loginfo("Failed to increase priority of USB Handle Events thread: 0x%02x - %s", ret, strerror(ret));
	else
		loginfo("Increased priority of USB Handle Events thread to: %d", params.sched_priority);
#endif

	// 60 seconds - but should be able to specify infinite timeout..
	tv.tv_sec = 60;
	tv.tv_usec = 0;

	mos_mutex_lock(&handleEventsThreadLock);
	while (needHandleEventsThread()) {
		mos_mutex_unlock(&handleEventsThreadLock);

		ret = libusb_handle_events_timeout_completed(libusb_ctx, &tv, &async_cancel);

		// XXX - what do I do about errors here??
		if (ret != 0) {
			switch (ret) {
			default:
				usblogerr("libusb_handle_events() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
				break;
			}
		}

		mos_mutex_lock(&handleEventsThreadLock);
	}

	logdebug("USB Handle Events Thread exiting");

	handleEventsThreadRun = 0;
	mos_cond_broadcast(&handleEventsThreadCond);
	mos_mutex_unlock(&handleEventsThreadLock);

	MOS_TASK_EXIT(EPHIDGET_OK);
}


#define MAX_XFER_QUEUE_SIZE	200
static void read_cb(struct libusb_transfer *xfer) {
	PhidgetUSBTransferHandle usbXfer;
	PhidgetUSBConnectionHandle conn;
	int ret;
	int i;

	PhidgetDeviceHandle device = (PhidgetDeviceHandle)xfer->user_data;
	conn = PhidgetUSBConnectionCast(device->conn);
	assert(conn);

	//logdebug("event from: %x", conn);

	if (conn->queueCnt >= MAX_XFER_QUEUE_SIZE) {
		usblogerr("%"PRIphid": too many incoming USB packets queued: %d. Dropping a USB packet.", device, conn->queueCnt);
	} else {
		// Queue a transfer for the read thread - for ALL transfers
		usbXfer = (PhidgetUSBTransferHandle)mos_malloc(sizeof(PhidgetUSBTransfer));

		if (xfer->status == LIBUSB_TRANSFER_COMPLETED) {
			memcpy(usbXfer->buffer, xfer->buffer, xfer->actual_length);
			usbXfer->actual_length = xfer->actual_length;
		}

		switch(xfer->status) {
			case LIBUSB_TRANSFER_COMPLETED:
				usbXfer->result = LIBUSB_SUCCESS;
				break;
			case LIBUSB_TRANSFER_TIMED_OUT:
				usbXfer->result = LIBUSB_ERROR_TIMEOUT;
				break;
			case LIBUSB_TRANSFER_STALL:
				usbXfer->result = LIBUSB_ERROR_PIPE;
				break;
			case LIBUSB_TRANSFER_NO_DEVICE:
				usbXfer->result = LIBUSB_ERROR_NO_DEVICE;
				break;
			case LIBUSB_TRANSFER_OVERFLOW:
				usbXfer->result = LIBUSB_ERROR_OVERFLOW;
				break;
			case LIBUSB_TRANSFER_ERROR:
				usbXfer->result = LIBUSB_ERROR_IO;
				break;
			case LIBUSB_TRANSFER_CANCELLED:
				usbXfer->result = LIBUSB_ERROR_INTERRUPTED;
				break;
			default:
				usbXfer->result = LIBUSB_ERROR_OTHER;
				break;
		}

		mos_mutex_lock(&conn->xferQueueLock);
		MSMTAILQ_INSERT_TAIL(&conn->xferQueue, usbXfer, link);
		conn->queueCnt++;
		//if (conn->queueCnt > 1)
		//	usblogdebug("Number of USB packets queued: %d", conn->queueCnt);
		mos_cond_signal(&conn->xferQueueCond);
		mos_mutex_unlock(&conn->xferQueueLock);
	}

	switch (xfer->status) {

		// Success
		case LIBUSB_TRANSFER_COMPLETED:

			// This means we are shutting down the reads. Don't resubmit. Do free
			if (!conn->usingAsyncReads)
				goto freexfer;

			// Resubmit transfer
			//logdebug("resubmitting transfer on %x", conn);
			ret = libusb_submit_transfer(xfer);
			if (ret) {
				switch (ret) {
					case LIBUSB_ERROR_NO_DEVICE:
						// Log as info, as likely just a device detach
						usbloginfo("libusb_submit_transfer() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
						break;
					default:
						usblogerr("libusb_submit_transfer() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
						break;
				}
				// XXX - we free the transfer because of this error. Need to tell the system to close the device, etc..
				goto freexfer;
			}

			break;

		default:
			goto freexfer;
	}

	return;

freexfer:

	PhidgetRunLock(conn);

	// XXX - better way?
	for (i = 0; i < XFER_CNT; i++) {
		if (conn->xfer[i] == xfer) {
			//logdebug("Freeing xfer #%d", i);
			libusb_free_transfer(conn->xfer[i]);
			conn->xfer[i] = NULL;
			break;
		}
	}
	PhidgetRunUnlock(conn);

}

void
PhidgetUSBFreeAsyncBuffers(PhidgetUSBConnectionHandle conn) {
	int i;

	assert(conn);

	if (conn->xfer) {
		for (i = 0; i < XFER_CNT; i++) {
			if (conn->xfer[i]) {
				libusb_free_transfer(conn->xfer[i]);
			}
		}

		mos_free(conn->xfer, XFER_CNT * sizeof(struct libusb_transfer *));
		conn->xfer = NULL;
	}

	if (conn->xfer_buf) {
		for (i = 0; i < XFER_CNT; i++) {
			if (conn->xfer_buf[i]) {
				if (conn->type == PHIDGET_HIDUSB_CONNECTION)
					mos_free(conn->xfer_buf[i], ((PhidgetHIDUSBConnectionHandle)conn)->inputReportByteLength);
				else
					mos_free(conn->xfer_buf[i], ((PhidgetPHIDUSBConnectionHandle)conn)->pusbParams.maxPacketEP1);
			}
		}

		mos_free(conn->xfer_buf, XFER_CNT * sizeof(unsigned char *));
		conn->xfer_buf = NULL;
	}
}

static PhidgetReturnCode
PhidgetUSBAllocAsyncBuffers(PhidgetUSBConnectionHandle conn) {
	int i;

	assert(conn);

	if (!conn->xfer) {
		conn->xfer = mos_malloc(XFER_CNT * sizeof(struct libusb_transfer *));
		for (i = 0; i < XFER_CNT; i++)
			conn->xfer[i] = libusb_alloc_transfer(0);
	}

	if (!conn->xfer_buf) {
		conn->xfer_buf = mos_malloc(XFER_CNT * sizeof(unsigned char *));
		for (i = 0; i < XFER_CNT; i++) {
			if (conn->type == PHIDGET_HIDUSB_CONNECTION)
				conn->xfer_buf[i] = mos_malloc(((PhidgetHIDUSBConnectionHandle)conn)->inputReportByteLength);
			else
				conn->xfer_buf[i] = mos_malloc(((PhidgetPHIDUSBConnectionHandle)conn)->pusbParams.maxPacketEP1);
		}
	}

	return (EPHIDGET_OK);
}

// NOTE: conn->readLock is locked by the caller
PhidgetReturnCode PhidgetUSBStartAsyncReads(PhidgetDeviceHandle device) {
	PhidgetUSBConnectionHandle conn;
	PhidgetReturnCode res;
	int ret;
	int i;
	int length;
	PhidgetUSBEndpointType eptype;

	logdebug("Starting async reads...");

	assert(device);
	conn = PhidgetUSBConnectionCast(device->conn);
	assert(conn);

	// Create and submit requests
	res = PhidgetUSBAllocAsyncBuffers(conn);
	if (res != EPHIDGET_OK) {
		logerr("PhidgetUSBAllocAsyncBuffers failed: "PRC_FMT, PRC_ARGS(res));
		return (res);
	}

	if (conn->type == PHIDGET_HIDUSB_CONNECTION) {
		length = ((PhidgetHIDUSBConnectionHandle)conn)->inputReportByteLength;
		eptype = PHID_EP_INTERRUPT;
	} else {
		length = ((PhidgetPHIDUSBConnectionHandle)conn)->pusbParams.maxPacketEP1;
		eptype = ((PhidgetPHIDUSBConnectionHandle)conn)->pusbParams.ep1type;
	}

	// Make sure that ep1 is actually available
	assert(eptype != PHID_EP_UNAVAILABLE);

	for (i = 0; i < XFER_CNT; i++) {

		if (eptype == PHID_EP_BULK) {
			libusb_fill_bulk_transfer(conn->xfer[i],
			  (libusb_device_handle *)conn->deviceHandle,
			  LIBUSB_ENDPOINT_IN | (conn->interfaceNum + 1),
			  conn->xfer_buf[i],
			  length,
			  read_cb,
			  device,
			  0 // NOTE: unlimited timeout. We use a timeout on handle_events
			);
		} else {
			libusb_fill_interrupt_transfer(conn->xfer[i],
			  (libusb_device_handle *)conn->deviceHandle,
			  LIBUSB_ENDPOINT_IN | (conn->interfaceNum + 1),
			  conn->xfer_buf[i],
			  length,
			  read_cb,
			  device,
			  0 // NOTE: unlimited timeout. We use a timeout on handle_events
			);
		}

		ret = libusb_submit_transfer(conn->xfer[i]);
		if (ret != 0) {
			usblogwarn("libusb_submit_transfer() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
			return (EPHIDGET_UNEXPECTED);
		}
	}

	// Start central libusb event handling thread if it's not already running
	res = StartHandleEventsThread();
	if (res != EPHIDGET_OK) {
		logerr("StartHandleEventsThread failed: "PRC_FMT, PRC_ARGS(res));
		return (res);
	}

	conn->usingAsyncReads = 1;

	return (EPHIDGET_OK);
}

PhidgetReturnCode PhidgetUSBStopAsyncReads(PhidgetUSBConnectionHandle conn) {
	int cancelcnt;
	int ret;
	int i;

	assert(conn);

	logdebug("Stopping async reads... for %x", conn);

	// readLock protects usingAsyncReads
	mos_mutex_lock(&conn->readLock);
	if (!conn->usingAsyncReads) {
		mos_mutex_unlock(&conn->readLock);
		return (EPHIDGET_OK);
	}

	// Cancel all transfers
	logdebug("Cancelling all transfers...");

	// This will stop this conn from resubmitting transfers
	conn->usingAsyncReads = 0;
	mos_mutex_unlock(&conn->readLock);

	libusb_lock_event_waiters(libusb_ctx);

	PhidgetRunLock(conn);
	while (1) {
		cancelcnt = 0;
		for (i = 0; i < XFER_CNT; i++) {
			if (!conn->xfer[i])
				continue;

			ret = libusb_cancel_transfer(conn->xfer[i]);
			if (ret != LIBUSB_SUCCESS && ret != LIBUSB_ERROR_NOT_FOUND) {
				usblogerr("libusb_cancel_transfer() failed: "LIBUSB_ERR_FMT, LIBUSB_ERR_ARGS(ret));
				continue;
			}

			cancelcnt++;
		}

		// All transfers have already been freed
		if (cancelcnt == 0)
			break;

		// Unlock runlock so the callback can free some transfers
		PhidgetRunUnlock(conn);
		logdebug("Waiting for event to cancel...");
		libusb_wait_for_event(libusb_ctx, NULL);
		PhidgetRunLock(conn);
	}

	// make sure xferQueue is empty
	PhidgetUSBTransferHandle n;
	mos_mutex_lock(&conn->xferQueueLock);
	while (!MSMTAILQ_EMPTY(&conn->xferQueue)) {
		n = MSMTAILQ_FIRST(&conn->xferQueue);
		MSMTAILQ_REMOVE_HEAD(&conn->xferQueue, link);
		conn->queueCnt--;
		mos_free(n, sizeof(PhidgetUSBTransfer));
	}
	MOS_ASSERT(conn->queueCnt == 0);

	// Notify read thread
	mos_cond_signal(&conn->xferQueueCond);
	mos_mutex_unlock(&conn->xferQueueLock);

	PhidgetRunUnlock(conn);

	libusb_unlock_event_waiters(libusb_ctx);
	logdebug("All transfers successfully cancelled");

	// Free all (remaining) buffers
	PhidgetUSBFreeAsyncBuffers(conn);

	// Stop libusb event handling thread if our count gets to 0
	// Don't wait for it to exit
	StopHandleEventsThread();

	return (EPHIDGET_OK);
}

#endif
