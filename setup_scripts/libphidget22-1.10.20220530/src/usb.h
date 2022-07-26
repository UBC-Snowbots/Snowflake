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

#ifndef __PhidgetUSB
#define __PhidgetUSB

#if defined(_MACOSX) && !defined(_IPHONE)
#include <IOKit/usb/IOUSBLib.h>
#endif

#ifdef _FREEBSD
#include <libusb.h>
#elif defined(_LINUX)
#include <libusb-1.0/libusb.h>
#endif

#ifdef _WINDOWS
#include "plat/windows/usbwindows.h"
#include "ext/include/winusb/winusb.h"
#endif

// These apply to control transfers
#define MAX_USB_IN_PACKET_SIZE					128
#define MAX_USB_OUT_PACKET_SIZE					128

// EP1 (0x81)
#define MAX_USB_BULK_INTERRUPT_IN_PACKET_SIZE	64
// EP2 (0x02)
// For now, limit to max bridge packet array size
#define MAX_USB_BULK_OUT_PACKET_SIZE			8192

/*
 * General USB device support
 */

#if USB_ASYNC_READS

#ifdef _WINDOWS

typedef struct _PhidgetUSBTransfer {
	unsigned char buffer[MAX_USB_BULK_INTERRUPT_IN_PACKET_SIZE];
	DWORD result;
	OVERLAPPED asyncRead;
	BOOL readPending;
	BOOL readCompleted;
} PhidgetUSBTransfer, *PhidgetUSBTransferHandle;

#elif defined(_LINUX) || defined (_FREEBSD)

typedef struct _PhidgetUSBTransfer{
	unsigned char buffer[MAX_USB_BULK_INTERRUPT_IN_PACKET_SIZE];
	int actual_length;
	int result;
	MSMTAILQ_ENTRY(_PhidgetUSBTransfer) link;
} PhidgetUSBTransfer, *PhidgetUSBTransferHandle;

#endif

#endif //USB_ASYNC_READS

// OS Specific common USB
#ifdef _WINDOWS
#define PHIDGET_USB_CONN_STRUCT_OS										\
	WCHAR  DevicePath[MAX_PATH];										\
	WCHAR  DevicePDOName[MAX_PATH];										\
	OVERLAPPED asyncRead;												\
	unsigned char readBuf[MAX_USB_BULK_INTERRUPT_IN_PACKET_SIZE + 1];	\
	BOOL readPending;													\
	HANDLE closeReadEvent;												\
	OVERLAPPED asyncWrite;												\
	int interfaceNum;
#elif defined(_MACOSX) && !defined(_IPHONE)
#define PHIDGET_USB_CONN_STRUCT_OS	\
	io_object_t usbDevice;								\
	IOUSBInterfaceInterface300 **intf;					\
	int interfaceNum;
#elif defined(_LINUX) || defined (_FREEBSD)
#define PHIDGET_USB_CONN_STRUCT_OS						\
	void *dev;											\
	char uniqueName[20];								\
	int tryAgainCounter;								\
	int interfaceNum;
#elif defined(_ANDROID)
#define PHIDGET_USB_CONN_STRUCT_OS						\
	char dev[256];
#else
#define PHIDGET_USB_CONN_STRUCT_OS
#endif

// Async read specific
#if USB_ASYNC_READS
#ifdef _WINDOWS
#define PHIDGET_USB_CONN_STRUCT_ASYNC								\
	int usingAsyncReads;											\
	int nextXfer;													\
	PhidgetUSBTransfer **xfer;
#elif defined(_LINUX) || defined (_FREEBSD)
MSMTAILQ_HEAD(PhidgetUSBTransfers, _PhidgetUSBTransfer);
#define PHIDGET_USB_CONN_STRUCT_ASYNC								\
	int usingAsyncReads;											\
	struct libusb_transfer **xfer;									\
	unsigned char **xfer_buf;										\
	struct PhidgetUSBTransfers xferQueue;							\
	mos_mutex_t xferQueueLock;										\
	mos_cond_t xferQueueCond;										\
	int queueCnt;
#endif
#else
#define PHIDGET_USB_CONN_STRUCT_ASYNC
#endif //USB_ASYNC_READS

#define PHIDGET_USB_CONN_STRUCT_START								\
	PHIDGET_STRUCT_START											\
	PHIDGET_USB_CONN_STRUCT_OS										\
	mos_mutex_t usbwritelock; /* protects write - exclusive */		\
	mos_mutex_t readLock;											\
	mos_task_t readThread;											\
	mos_cond_t readCond;											\
	int readRun;													\
	HANDLE deviceHandle;

typedef struct {
	PHIDGET_USB_CONN_STRUCT_START
#ifndef _WINDOWS
	// NOTE: async is only part of windows for the PHIDUSB connection
	PHIDGET_USB_CONN_STRUCT_ASYNC
#endif
} PhidgetUSBConnection, *PhidgetUSBConnectionHandle;

typedef struct {
	PHIDGET_USB_CONN_STRUCT_START
#ifndef _WINDOWS
	// NOTE: async is only part of windows for the PHIDUSB connection
	PHIDGET_USB_CONN_STRUCT_ASYNC
#endif

	unsigned short outputReportByteLength;
	unsigned short inputReportByteLength;
	unsigned char interruptOutEndpoint;

} PhidgetHIDUSBConnection, *PhidgetHIDUSBConnectionHandle;


/*
 * PhidgetUSB class device support
 */

typedef uint8_t PhidgetUSBPacketType;

// Phidget Vendor defined descriptors
#define USB_DESC_TYPE_PHIDGET_DEVICE			(0x40 | 0x00)
#define USB_DESC_TYPE_PHIDGET_INTERFACE			(0x40 | 0x01)
#define USB_DESC_TYPE_PHIDGET_ENDPOINT			(0x40 | 0x02)
#define USB_DESC_TYPE_VINT_PORTS_DESC			(0x40 | 0x03)
#define USB_DESC_TYPE_VINT_DEVICE_DESC			(0x40 | 0x04)
#define USB_DESC_TYPE_VINT_PORT_DESC			(0x40 | 0x05)

// Setup requests
typedef enum {
	PHIDGETUSB_REQ_CHANNEL_WRITE	= 0x00,
	PHIDGETUSB_REQ_CHANNEL_READ		= 0x01,
	PHIDGETUSB_REQ_DEVICE_WRITE		= 0x02,
	PHIDGETUSB_REQ_DEVICE_READ		= 0x03,
	PHIDGETUSB_REQ_GPP_WRITE		= 0x04,
	PHIDGETUSB_REQ_GPP_READ			= 0x05,
	PHIDGETUSB_REQ_BULK_WRITE		= 0x06
} PhidgetUSBRequest;

#define USB_PHIDGET_CH_REQ_MAKE_TYPE_AND_INDEX(d, i) ((uint16_t)((uint16_t)d<<8 | i))

// Used by the OS - ID 0x20 is defined in firmware
#define	USB_REQ_GET_OS_FEATURE_DESCRIPTOR		0x20

// Pack structures
#ifdef _WINDOWS
#pragma pack(1)
#define __STRUCTPACK__
#else
#define __STRUCTPACK__ __attribute__((packed))
#endif

// Version 1.1 of the PhidgetUSB Protocol
// This version applies to all Phidget Descriptors, and the communication protocol
#define USBD_PHIDGET_PROTO_VERSION			0x0110
typedef struct __STRUCTPACK__ {
	uint8_t	  bLength;
	uint8_t   bDescriptorType;
	uint16_t  bcdVersion;
	uint8_t   iLabel;
	uint8_t   iSKU;
	uint16_t  wMaxPacketSize;
} USBD_PhidgetDeviceDescStruct;

typedef struct __STRUCTPACK__ {
	uint8_t	  bLength;
	uint8_t   bDescriptorType;
	uint16_t  wMaxPacketSize;
} USBD_PhidgetEndpointDescStruct;

typedef struct __STRUCTPACK__ {
	uint8_t		bLength;
	uint8_t		bDescriptorType;
	struct __STRUCTPACK__ {
		uint16_t	wID;
		uint16_t	wVersion;
	} port[6];
	struct __STRUCTPACK__ {
		uint16_t	wID;
		uint16_t	wVersion;
	} portModeDevice[4];
} USBD_VINTPortsDescStruct_6_4;

typedef struct __STRUCTPACK__ {
	uint8_t		bLength;
	uint8_t		bDescriptorType;
	uint16_t	wID;
	uint16_t	wVersion;
	uint8_t		bVINTProtocolVersion;
	uint8_t		VINTProperties[32];
} USBD_VINTDeviceDescStruct;

typedef struct __STRUCTPACK__ {
	uint8_t		bLength;
	uint8_t		bDescriptorType;
	uint8_t		bPortMode;					// CPhidgetVINT_PortMode
	uint8_t		bPowered;					// whether power is enabled
	uint32_t	dwSpeedHz;					// VINT Speed
	uint8_t		bVINTProtocolVersion;		// VINT Protocol version
	uint8_t		VINTProperties[10];
} USBD_VINTPortDescStruct;

#ifdef _WINDOWS
// Assert structures are properly packed
C_ASSERT(sizeof(USBD_PhidgetDeviceDescStruct) == 8);
C_ASSERT(sizeof(USBD_PhidgetEndpointDescStruct) == 4);
// Reset packing
#pragma pack()
#endif

typedef enum {
	PHID_EP_UNAVAILABLE = 0,
	PHID_EP_BULK,
	PHID_EP_INTERRUPT
} PhidgetUSBEndpointType;

typedef struct {
	uint8_t labelIndex;
	uint8_t skuIndex;

	// Control
	uint16_t maxPacketEP0;

	// IN
	uint16_t maxPacketEP1;		// Max from config.h in firmware
	uint16_t wMaxPacketSizeEP1;	// Max from Endpoint desc

	// OUT
	uint16_t maxPacketEP2;

	PhidgetUSBEndpointType ep1type;
	PhidgetUSBEndpointType ep2type;
} PhidgetUSBDeviceParams, *PhidgetUSBDeviceParamsHandle;

typedef struct {
	PHIDGET_USB_CONN_STRUCT_START
	PHIDGET_USB_CONN_STRUCT_ASYNC
#ifdef _WINDOWS
	HANDLE winusbHandle;
	WCHAR  DeviceParentPath[MAX_PATH];
	DWORD port_nr; /* port number on (usb) hub */
#endif
	PhidgetUSBDeviceParams pusbParams;
} PhidgetPHIDUSBConnection, *PhidgetPHIDUSBConnectionHandle;


/*
 * API
 */

PhidgetReturnCode PhidgetHIDUSBConnectionCreate(PhidgetHIDUSBConnectionHandle *conn);
PhidgetHIDUSBConnectionHandle PhidgetHIDUSBConnectionCast(void *);

PhidgetReturnCode PhidgetPHIDUSBConnectionCreate(PhidgetPHIDUSBConnectionHandle *conn);
PhidgetPHIDUSBConnectionHandle PhidgetPHIDUSBConnectionCast(void *);

PhidgetUSBConnectionHandle PhidgetUSBConnectionCast(void *);

PhidgetReturnCode openAttachedUSBDevice(PhidgetDeviceHandle);
void joinUSBReadThread(PhidgetUSBConnectionHandle);
void stopUSBReadThread(PhidgetUSBConnectionHandle);

void PhidgetUSBError(PhidgetDeviceHandle device);

PhidgetReturnCode PhidgetUSBScanDevices(void);
PhidgetReturnCode PhidgetUSBOpenHandle(PhidgetDeviceHandle device);
PhidgetReturnCode PhidgetUSBCloseHandle(PhidgetUSBConnectionHandle conn);
PhidgetReturnCode PhidgetUSBSetLabel(PhidgetDeviceHandle device, char *buffer);
void PhidgetUSBCleanup(void);
PhidgetReturnCode PhidgetUSBRefreshLabelString(PhidgetDeviceHandle device);
//NOTE: str must have a length of 256 bytes
PhidgetReturnCode PhidgetUSBGetString(PhidgetUSBConnectionHandle conn, int index, char *str);
PhidgetReturnCode PhidgetUSBReadPacket(PhidgetUSBConnectionHandle conn, unsigned char *buffer, size_t *length);
PhidgetReturnCode PhidgetUSBSendPacket(mosiop_t iop, PhidgetHIDUSBConnectionHandle conn, const unsigned char *buffer, size_t bufferLen);
PhidgetReturnCode PhidgetUSBTransferPhidgetPacket(mosiop_t iop, PhidgetPHIDUSBConnectionHandle conn, PhidgetUSBRequest PhidgetUSBRequest,
  PhidgetUSBPacketType packetType, uint8_t index, uint8_t *buffer, size_t *bufferLen, int timeout);
PhidgetReturnCode PhidgetUSBGetDeviceDescriptor(PhidgetUSBConnectionHandle conn, int type, int index, uint8_t *buf, size_t *bufLen);

#if defined(_IPHONE) || defined(_MACOSX)
PhidgetReturnCode PhidgetUSBResetDevice(PhidgetDeviceHandle device);
PhidgetReturnCode PhidgetUSBSetupNotifications(CFRunLoopRef runloop);
PhidgetReturnCode PhidgetUSBTeardownNotifications(void);
#endif

#if defined(_LINUX) || defined(_FREEBSD) && !defined(_ANDROID)
void PhidgetUSBUninit(void);
#endif

#if USB_ASYNC_READS
PhidgetReturnCode PhidgetUSBStartAsyncReads(PhidgetDeviceHandle device);

#ifdef _WINDOWS
PhidgetReturnCode PhidgetUSBStopAsyncReads(PhidgetPHIDUSBConnectionHandle conn);
void PhidgetUSBFreeAsyncBuffers(PhidgetPHIDUSBConnectionHandle conn);
#define PhidgetUSBInit()
#define PhidgetUSBFini()
#else //Linux
PhidgetReturnCode PhidgetUSBStopAsyncReads(PhidgetUSBConnectionHandle conn);
void PhidgetUSBFreeAsyncBuffers(PhidgetUSBConnectionHandle conn);
void PhidgetUSBInit(void);
void PhidgetUSBFini(void);
#endif

#else
#define PhidgetUSBInit()
#define PhidgetUSBFini()
#endif //USB_ASYNC_READS

PhidgetReturnCode encodeLabelString(char *buffer, char *out, size_t *outLen);
PhidgetReturnCode decodeLabelString(char *labelBuf, char *out, int serialNumber);
BOOL labelHasWrapError(int serialNumber, char *labelBuf);

PhidgetReturnCode CCONV UTF16toUTF8(char *in, int inBytes, char *out);

MOS_TASK_RESULT PhidgetUSBReadThreadFunction(void *arg);

#endif
