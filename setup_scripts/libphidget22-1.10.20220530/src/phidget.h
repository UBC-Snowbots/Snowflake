#ifndef EXTERNALPROTO
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
#endif

#ifndef __CPHIDGET
#define __CPHIDGET

#include "types.gen.h"
#include "object.h"

typedef void(CCONV *Phidget_AsyncCallback)(PhidgetHandle phid, void *ctx, PhidgetReturnCode returnCode);

/* Library Properties */
API_PRETURN_HDR Phidget_getLibraryVersion		(const char **libraryVersion);
API_PRETURN_HDR Phidget_getLibraryVersionNumber	(const char **libraryVersion);

/* Library Methods */
API_PRETURN_HDR Phidget_getErrorDescription		(PhidgetReturnCode errorCode, const char **errorString);
API_PRETURN_HDR Phidget_getLastError			(PhidgetReturnCode *errorCode, const char **errorString, char *errorDetail, size_t *errorDetailLen);
API_PRETURN_HDR Phidget_finalize				(int flags);
API_PRETURN_HDR Phidget_resetLibrary			(void);

/* Methods */
API_PRETURN_HDR Phidget_open					(PhidgetHandle phid);
API_PRETURN_HDR Phidget_openWaitForAttachment	(PhidgetHandle phid, uint32_t timeoutMs);
API_PRETURN_HDR Phidget_close					(PhidgetHandle phid);
API_PRETURN_HDR Phidget_delete					(PhidgetHandle *phid);
API_PRETURN_HDR Phidget_writeDeviceLabel		(PhidgetHandle phid, const char *deviceLabel);
API_PRETURN_HDR Phidget_retain					(PhidgetHandle phid);
API_PRETURN_HDR Phidget_release					(PhidgetHandle *phid);
API_PRETURN_HDR Phidget_getChildDevices			(PhidgetHandle phid, PhidgetHandle *arr, size_t *arrCnt);
API_PRETURN_HDR Phidget_releaseDevices			(PhidgetHandle *arr, size_t arrCnt);
API_PRETURN_HDR Phidget_getDataInterval			(PhidgetHandle phid, uint32_t *di);
API_PRETURN_HDR Phidget_getMinDataInterval		(PhidgetHandle phid, uint32_t *min);
API_PRETURN_HDR Phidget_getMaxDataInterval		(PhidgetHandle phid, uint32_t *max);
API_PRETURN_HDR Phidget_setDataInterval			(PhidgetHandle phid, uint32_t di);
API_PRETURN_HDR Phidget_getDataRate				(PhidgetHandle phid, double *dr);
API_PRETURN_HDR Phidget_getMinDataRate			(PhidgetHandle phid, double *min);
API_PRETURN_HDR Phidget_getMaxDataRate			(PhidgetHandle phid, double *max);
API_PRETURN_HDR Phidget_setDataRate				(PhidgetHandle phid, double dr);

/* General Properties */
API_PRETURN_HDR Phidget_getAttached				(PhidgetHandle phid, int *attached);
API_PRETURN_HDR Phidget_getIsChannel			(PhidgetHandle phid, int *isChannel);
API_PRETURN_HDR Phidget_getIsLocal				(PhidgetHandle phid, int *isLocal);
API_PRETURN_HDR Phidget_setIsLocal				(PhidgetHandle phid, int isLocal);
API_PRETURN_HDR Phidget_getIsRemote				(PhidgetHandle phid, int *isRemote);
API_PRETURN_HDR Phidget_setIsRemote				(PhidgetHandle phid, int isRemote);
API_PRETURN_HDR Phidget_getParent				(PhidgetHandle phid, PhidgetHandle *parent);
API_PRETURN_HDR Phidget_getServerName			(PhidgetHandle phid, const char **serverName);
API_PRETURN_HDR Phidget_getServerUniqueName		(PhidgetHandle phid, const char **serverUniqueName);
API_PRETURN_HDR Phidget_setServerName			(PhidgetHandle phid, const char *serverName);
API_PRETURN_HDR Phidget_getServerPeerName		(PhidgetHandle phid, const char **serverPeerName);
API_PRETURN_HDR Phidget_getServerHostname		(PhidgetHandle phid, const char **serverHostname);
API_PRETURN_HDR Phidget_getServerVersion		(PhidgetHandle deviceOrChannel, int *major, int *minor);
API_PRETURN_HDR Phidget_getClientVersion		(PhidgetHandle deviceOrChannel, int *major, int *minor);

/* Channel Properties */
API_PRETURN_HDR Phidget_getChannel				(PhidgetHandle phid, int *channel);
API_PRETURN_HDR Phidget_setChannel				(PhidgetHandle phid, int channel);
API_PRETURN_HDR Phidget_getChannelClass			(PhidgetHandle phid, Phidget_ChannelClass *channelClass);
API_PRETURN_HDR Phidget_getChannelClassName		(PhidgetHandle phid, const char **channelClassName);
API_PRETURN_HDR Phidget_getChannelName			(PhidgetHandle phid, const char **channelName);
API_PRETURN_HDR Phidget_getChannelSubclass		(PhidgetHandle phid, Phidget_ChannelSubclass *channelSubclass);

/* Device Properties */
API_PRETURN_HDR Phidget_getDeviceClass			(PhidgetHandle phid, Phidget_DeviceClass *deviceClass);
API_PRETURN_HDR Phidget_getDeviceClassName		(PhidgetHandle phid, const char **deviceClassName);
API_PRETURN_HDR Phidget_getDeviceID				(PhidgetHandle phid, Phidget_DeviceID *deviceID);
API_PRETURN_HDR Phidget_getDeviceLabel			(PhidgetHandle phid, const char **deviceLabel);
API_PRETURN_HDR Phidget_setDeviceLabel			(PhidgetHandle phid, const char *deviceLabel);
API_PRETURN_HDR Phidget_getDeviceName			(PhidgetHandle phid, const char **deviceName);
API_PRETURN_HDR Phidget_getDeviceSerialNumber	(PhidgetHandle phid, int32_t *deviceSerialNumber);
API_PRETURN_HDR Phidget_setDeviceSerialNumber	(PhidgetHandle phid, int32_t deviceSerialNumber);
API_PRETURN_HDR Phidget_getDeviceSKU			(PhidgetHandle phid, const char **deviceSKU);
API_PRETURN_HDR Phidget_getDeviceVersion		(PhidgetHandle phid, int *deviceVersion);
API_PRETURN_HDR Phidget_getDeviceChannelCount	(PhidgetHandle phid, Phidget_ChannelClass cls, uint32_t *count);

/* VINT Hub Properties */
API_PRETURN_HDR Phidget_getIsHubPortDevice		(PhidgetHandle phid, int *isHubPortDevice);
API_PRETURN_HDR Phidget_setIsHubPortDevice		(PhidgetHandle phid, int isHubPortDevice);
API_PRETURN_HDR Phidget_getHub					(PhidgetHandle phid, PhidgetHandle *hub);
API_PRETURN_HDR Phidget_getHubPort				(PhidgetHandle phid, int *hubPort);
API_PRETURN_HDR Phidget_setHubPort				(PhidgetHandle phid, int hubPort);
API_PRETURN_HDR Phidget_getHubPortCount			(PhidgetHandle phid, int *hubPortCount);

API_PRETURN_HDR Phidget_getHubPortSpeed					(PhidgetHandle phid, uint32_t *speed);
API_PRETURN_HDR Phidget_setHubPortSpeed					(PhidgetHandle phid, uint32_t speed);
API_PRETURN_HDR Phidget_getHubPortSupportsSetSpeed		(PhidgetHandle phid, int *supportsSetSpeed);
API_PRETURN_HDR Phidget_getMaxHubPortSpeed				(PhidgetHandle phid, uint32_t *maxSpeed);
API_PRETURN_HDR Phidget_getVINTDeviceSupportsSetSpeed	(PhidgetHandle phid, int *supportsSetSpeed);
API_PRETURN_HDR Phidget_getMaxVINTDeviceSpeed			(PhidgetHandle phid, uint32_t *maxSpeed);

/* Mesh device properties */
API_PRETURN_HDR Phidget_setMeshMode				(PhidgetHandle phid, Phidget_MeshMode mode);
API_PRETURN_HDR Phidget_getMeshMode				(PhidgetHandle phid, Phidget_MeshMode *mode);

/* Events */
typedef void(CCONV *Phidget_OnAttachCallback)	(PhidgetHandle phid, void *ctx);
typedef void(CCONV *Phidget_OnDetachCallback)	(PhidgetHandle phid, void *ctx);
typedef void(CCONV *Phidget_OnErrorCallback)	(PhidgetHandle phid, void *ctx, Phidget_ErrorEventCode errorCode, const char *errorString);
typedef void(CCONV *Phidget_OnPropertyChangeCallback)(PhidgetHandle phid, void *ctx, const char *property);

API_PRETURN_HDR Phidget_setOnDetachHandler		(PhidgetHandle phid, Phidget_OnDetachCallback fptr, void *ctx);
API_PRETURN_HDR Phidget_setOnAttachHandler		(PhidgetHandle phid, Phidget_OnAttachCallback fptr, void *ctx);
API_PRETURN_HDR Phidget_setOnErrorHandler		(PhidgetHandle phid, Phidget_OnErrorCallback fptr, void *ctx);
API_PRETURN_HDR Phidget_setOnPropertyChangeHandler(PhidgetHandle phid, Phidget_OnPropertyChangeCallback fptr, void *ctx);

API_IRETURN_HDR Phidget_validDictionaryKey(const char *);
typedef void(CCONV *PhidgetDictionary_OnChangeCallback)(int, const char *, void *, int, const char *, const char *);
API_PRETURN_HDR PhidgetDictionary_setOnChangeCallbackHandler(PhidgetDictionary_OnChangeCallback, void *);

#ifdef _MACOSX
API_PRETURN_HDR Phidget_setOnWillSleepHandler	(void (CCONV *fptr)(void *ctx), void *ctx);
API_PRETURN_HDR Phidget_setOnWakeupHandler		(void (CCONV *fptr)(void *ctx), void *ctx);
#endif

#ifdef INCLUDE_PRIVATE

/* Hidden API */
#ifdef DEBUG
#define chlog(...) PhidgetLog_loge(NULL, 0, __func__, "_phidget22channel", PHIDGET_LOG_INFO, __VA_ARGS__)
#else
#define chlog(...)
#endif

API_PRETURN_HDR Phidget_getDeviceVINTID					(PhidgetHandle deviceOrChannel, uint32_t *VINTID);
API_PRETURN_HDR Phidget_getDeviceFirmwareUpgradeString	(PhidgetHandle deviceOrChannel, const char **buffer);
API_PRETURN_HDR Phidget_reboot							(PhidgetHandle phid);
API_PRETURN_HDR Phidget_rebootFirmwareUpgrade			(PhidgetHandle phid, uint32_t upgradeTimeout);

API_PRETURN_HDR Phidget_writeFlash(PhidgetHandle phid);

#endif //#ifdef INCLUDE_PRIVATE


#ifndef EXTERNALPROTO

#include "devices.h"
#include "constantsinternal.h"
#include "constants.h"
#include "bridge.h"
#include "mos/bsdqueue.h"
#include "mos/mos_rwrlock.h"

#include "util/packettracker.h"
#include "util/packing.h"
#include "util/utils.h"

#include "network/network.h"
#include "object.h"
#include "enumutil.gen.h"
#include "errorstrings.gen.h"

#ifdef CHK_FORMAT_STRINGS
#undef PRIphid
#define PRIphid "p"
#endif

#define CHANNELID_ISHUBPORT		0x01
#define CHANNELID_ISVINTDEVICE	0x02
struct _channelid_parts {
	uint32_t	serial;
	uint8_t		flags;
	uint8_t		class;
	uint8_t		port;
	uint8_t		index;
};

typedef union {
	#define c_flags parts.flags
	#define c_type parts.type
	#define c_serial parts.serial
	#define c_port parts.port
	#define c_index parts.index
	#define c_class parts.class
	struct _channelid_parts parts;
	uint64_t c_id;
} channelid_t;

typedef enum {
	EVENTMODE_DATARATE=1, EVENTMODE_CHANGETRIGGER=2
} Phidget_EventMode;

typedef struct _PhidgetChannel PhidgetChannel;
typedef struct _PhidgetDevice PhidgetDevice;

#include "vint.h"
#include "usb.h"
#include "spi.h"
#include "lightning.h"
#include "mesh.h"
#include "virtual.h"

typedef MTAILQ_HEAD(phidgetchannnelnetconnlist, _PhidgetChannelNetConn) phidgetchannelnetconnlist_t;

typedef struct {
	Phidget_DeviceClass class;

	const PhidgetUniqueDeviceDef *UDD;
	int version;

	//Properties which only exist for a USB Device
	char label[MAX_LABEL_STORAGE];
	int serialNumber;

	//Properties which only exist for a VINT Device
	unsigned char isHubPort;
	int hubPort;

	//Properties which exist only for mesh devices
	Phidget_MeshMode meshMode;

	//Properties which exist only for remote devices
	char serverName[256];
	char serverUniqueName[256];
	char serverPeerName[256];
	char serverHostName[256];

	int uniqueIndex;	//firmware index (index into ->children[] array of parent)
} PhidgetDeviceInfo, *PhidgetDeviceInfoHandle;

typedef struct {
	int openFlags;
	PhidgetHub_PortMode hubPortMode;

	int channel;
	BOOL isLocal;

	//USB Phidget options
	int serialNumber;
	char *label;

	//VINT Device options
	int hubPort;
	unsigned char isHubPort;

	//Network options
	char *serverName;
	BOOL isRemote;
	uint64_t devid;

	//Open type, timeout
	unsigned char async;
	uint32_t timeout;

	//open attempt feedback
	int openAttempts;
	PhidgetReturnCode lastOpenRes;
	PhidgetDeviceHandle lastOpenDevice;
} PhidgetOpenInfo, *PhidgetOpenInfoHandle;

typedef struct _PhidgetChannelNetConn {
	PhidgetNetConnHandle					nc;
	MTAILQ_ENTRY(_PhidgetChannelNetConn)	link;
	uint16_t								setstatusrep;	/* reply seq for setstatus (0 if sent already) */
} PhidgetChannelNetConn, *PhidgetChannelNetConnHandle;

struct _PhidgetChannel {
	PHIDGET_STRUCT_START

	Phidget_ChannelClass class;
	const PhidgetUniqueChannelDef *UCD;
	int uniqueIndex;						/* firmware index (index into ->channels[] array of parent) */
	int index;								/* channel id */

	MTAILQ_ENTRY(_PhidgetChannel) link;		/* open channel  linkage */
	MTAILQ_ENTRY(_PhidgetChannel) match;	/* attach matching list linkage */

	phidgetchannelnetconnlist_t netconns;	/* list of network connections to channel */
	mos_mutex_t					netconnslk;	/* lock for network connections */
	int netconnscnt;

	PhidgetOpenInfoHandle openInfo;
	mosiop_t iop;

	void *private;		/* private pointer for extending code */

	PhidgetReturnCode(*initAfterOpen)(PhidgetChannelHandle);
	PhidgetReturnCode(*setDefaults)(PhidgetChannelHandle);
	PhidgetReturnCode(*bridgeInput)(PhidgetChannelHandle, BridgePacket *);
	void(*errorHandler)(PhidgetChannelHandle, Phidget_ErrorEventCode);
	PhidgetReturnCode(*getStatus)(PhidgetChannelHandle, BridgePacket **);
	PhidgetReturnCode(*setStatus)(PhidgetChannelHandle, BridgePacket *);
	void (*fireInitialEvents)(PhidgetChannelHandle);
	int(*hasInitialState)(PhidgetChannelHandle);

	/* User events */
	Phidget_OnAttachCallback Attach;
	void *AttachCtx;
	Phidget_OnDetachCallback Detach;
	void *DetachCtx;
	Phidget_OnErrorCallback Error;
	void *ErrorCtx;
	Phidget_OnPropertyChangeCallback PropertyChange;
	void *PropertyChangeCtx;
	PhidgetReturnCode(*_closing)(PhidgetChannelHandle);
};

#define PHIDGET_DEVICE_LAST_ERROR_STR_LEN	256
struct _PhidgetDevice {
	PHIDGET_STRUCT_START

	mos_tlock_t *__memberlock;

	MTAILQ_ENTRY(_PhidgetDevice) link;		/* phidgetDevices linkage */
	PhidgetConnectionType connType;
	PhidgetHandle conn;

#define devudd_id		deviceInfo.UDD->id
#define dev_hub			deviceInfo.UDD->channelCnts.hub

	PhidgetDeviceInfo deviceInfo;

	PhidgetDeviceHandle child[PHIDGET_MAXCHILDREN];		/* RUNLOCK */
	PhidgetChannelHandle channel[PHIDGET_MAXCHANNELS];	/* RUNLOCK */
	uint32_t readCount;

	PhidgetPacketTrackersHandle packetTracking;

	uint8_t GPPResponse;

	PhidgetReturnCode (CCONV *bridgeInput)(PhidgetChannelHandle, BridgePacket *);
	mos_mutex_t bridgeInputLock;

	PhidgetReturnCode (CCONV *initAfterCreate)(PhidgetDeviceHandle);
	PhidgetReturnCode (CCONV *initAfterOpen)(PhidgetDeviceHandle);
	PhidgetReturnCode (CCONV *_closing)(PhidgetDeviceHandle);
	PhidgetReturnCode (CCONV *dataInput)(PhidgetDeviceHandle, uint8_t *buffer, size_t length);
	const VINTIO_t *vintIO;

	char fwstr[64];
	char firmwareUpgradeName[128];

	//open attempt feedback
	int openAttempts;
	PhidgetReturnCode lastOpenRes;
	char *lastOpenErrstr;
};

typedef struct _NetAttachDetachEntry {
	int flags;
	PhidgetDeviceHandle dev;
	MTAILQ_ENTRY(_NetAttachDetachEntry) link;		/* netAttachDetachQueue linkage */
} NetAttachDetachEntry, *NetAttachDetachEntryHandle;

typedef struct _PhidetErrorDetail {
	PhidgetReturnCode code;
	char *detail;
} PhidgetErrorDetail, *PhidgetErrorDetailHandle;

/* Globals */

extern const char *LibraryVersion;
extern const char *LibraryVersionNumber;
extern const char *LibrarySystem;

extern const PhidgetUniqueDeviceDef Phidget_Unique_Device_Def[];

void phidget_init(PhidgetHandle, PhidgetStructType, PhidgetDelete_t);

PhidgetChannelHandle PhidgetChannelCast(void *);
PhidgetDeviceHandle PhidgetDeviceCast(void *);

void PhidgetWriteLockChannels(void);
void PhidgetReadLockChannels(void);
void PhidgetUnlockChannels(void);

void PhidgetWriteLockDevices(void);
void PhidgetReadLockDevices(void);
void PhidgetUnlockDevices(void);

void PhidgetLockNetAttachDetachQueue(void);
void PhidgetUnlockNetAttachDetachQueue(void);

PhidgetReturnCode dispatchUserRequest(PhidgetChannelHandle, BridgePacket *, Phidget_AsyncCallback, void *);
PhidgetReturnCode dispatchBridgePacket(void *_phid, BridgePacket *);
PhidgetReturnCode dispatchVintData(void *_phid, const uint8_t *, size_t);
void clearPhidgetDispatch(PhidgetHandle);
void freeDispatchHandle(PhidgetHandle);
void startDispatch(PhidgetHandle);
void stopDispatch(PhidgetHandle, int);
void wakeDispatch(void);

PhidgetChannelHandle getAttachedChannel(void *_device, int index);
PhidgetChannelHandle getChannel(void *_device, int index);
PhidgetReturnCode setChannel(PhidgetDeviceHandle, int index, void *_channel);
PhidgetDeviceHandle getParent(void *phid);
void setParent(void *_phid, void *_parent);
PhidgetDeviceHandle getChild(PhidgetDeviceHandle, int index);
void setChild(PhidgetDeviceHandle device, int index, void *_childdevice);

const PhidgetChannelAttributeDef *getPhidgetChannelAttributesByClass(Phidget_ChannelClass);
const PhidgetChannelAttributeDef *getPhidgetChannelAttributes(PhidgetChannelHandle);

PhidgetReturnCode PhidgetDevice_usbDataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length);

PhidgetReturnCode PhidgetDevice_read(PhidgetDeviceHandle device);

BOOL isVintChannel(void *);
BOOL isNetworkPhidget(void *);
uint64_t PhidgetDeviceGetNetId(PhidgetDeviceHandle device);

typedef PhidgetReturnCode(*deviceChannelVisitor_t)(PhidgetDeviceHandle dev,
  const PhidgetUniqueChannelDef *ucd, int index, int uniqueIndex, void *ctx);

void matchOpenChannels(void);

PhidgetReturnCode addDictionary(int, const char *);

PhidgetOpenInfoHandle mallocPhidgetOpenInfo(void);
void freePhidgetOpenInfo(PhidgetOpenInfoHandle item);

size_t getMaxOutPacketSize(PhidgetDeviceHandle device);
PhidgetReturnCode PhidgetDevice_sendpacket(mosiop_t iop, PhidgetDeviceHandle device, const unsigned char *buf, size_t length);
PhidgetReturnCode PhidgetDevice_sendpacketTransaction(mosiop_t iop, PhidgetDeviceHandle device,
  const unsigned char *bufferIn, size_t bufferInLen, PhidgetTransactionHandle trans);
PhidgetReturnCode PhidgetDevice_beginTransaction(PhidgetDeviceHandle device, PhidgetTransactionHandle trans);
PhidgetReturnCode PhidgetChannel_endTransaction(PhidgetChannelHandle channel, PhidgetTransactionHandle trans);
PhidgetReturnCode PhidgetChannel_beginTransaction(PhidgetChannelHandle channel, PhidgetTransactionHandle trans);
PhidgetReturnCode PhidgetDevice_transferpacket(mosiop_t iop, PhidgetDeviceHandle device, int transferType, int packetType, int index,
  unsigned char *buffer, size_t *bufferLen, int timeout);

PhidgetReturnCode StartCentralThread(void);
void NotifyCentralThread(void);

PhidgetReturnCode VINTPacketStatusCode_to_PhidgetReturnCode(VINTPacketStatusCode code);

PhidgetReturnCode _addDevice(PhidgetDeviceHandle); /* no lock */
PhidgetReturnCode addDevice(PhidgetDeviceHandle);
PhidgetReturnCode _removeDevice(PhidgetDeviceHandle); /* no lock */
PhidgetReturnCode getNetworkDevice(PhidgetNetConnHandle, uint64_t, PhidgetDeviceHandle *);
PhidgetDeviceHandle getDeviceById(uint64_t);
PhidgetReturnCode addChannel(PhidgetChannelHandle);
PhidgetReturnCode removeChannel(PhidgetChannelHandle);
PhidgetChannelHandle getChannelById(uint64_t);
uint64_t mkChannelId(int chindex, int chclass, int serialNumber, int vint, int port, int hubport);
uint64_t getChannelId(PhidgetChannelHandle);

PhidgetHandle getPhidgetConnection(void *phid);

int getTimeout(PhidgetDeviceHandle device);
PhidgetReturnCode attachChannel(PhidgetDeviceHandle, int index, PhidgetChannelHandle);
PhidgetReturnCode openDevice(PhidgetDeviceHandle attachedDevice);
void closeDevice(PhidgetDeviceHandle device, int forceClose);
PhidgetReturnCode PhidgetChannel_create(PhidgetChannelHandle *phid);

int phidgetdevice_compare(PhidgetDeviceHandle , PhidgetDeviceHandle);
typedef MTAILQ_HEAD(PhidgetDevices, _PhidgetDevice)		phidgetdevices_t;
extern phidgetdevices_t									phidgetDevices;
extern mos_tlock_t *devicesLock;
#define FOREACH_DEVICE_SAFE(dev, tmp)	MTAILQ_FOREACH_SAFE(dev, &phidgetDevices, link, tmp)
#define FOREACH_DEVICE(dev)				MTAILQ_FOREACH(dev, &phidgetDevices, link)
#define DEVICES_EMPTY					MTAILQ_EMPTY(&phidgetDevices)

typedef MTAILQ_HEAD(PhidgetChannels, _PhidgetChannel)	phidgetchannels_t;
extern phidgetchannels_t		phidgetChannels;
extern mos_rwrlock_t channelsLock;
#define FOREACH_CHANNEL_SAFE(ch, tmp)	MTAILQ_FOREACH_SAFE(ch, &phidgetChannels, link, tmp)
#define FOREACH_CHANNEL(ch)				MTAILQ_FOREACH(ch, &phidgetChannels, link)
#define CHANNELS_EMPTY					MTAILQ_EMPTY(&phidgetChannels)

#define NETATTACHQUEUE_FLAG		0x00002000
#define NETDETACHQUEUE_FLAG		0x00004000
typedef MTAILQ_HEAD(NetAttachDetachEntries, _NetAttachDetachEntry)		netattachdetachentries_t;
extern netattachdetachentries_t netAttachDetachQueue;
extern mos_tlock_t *netAttachDetachQueueLock;
#define FOREACH_NET_ATTACHDETACH_QUEUE_DEVICE(dev)	MTAILQ_FOREACH(dev, &netAttachDetachQueue, link)
#define NET_ATTACHDETACH_QUEUE_DEVICE_EMPTY			MTAILQ_EMPTY(&netAttachDetachQueue)

typedef MTAILQ_HEAD(phidgets, _Phidget)				phidgets_t;

PhidgetReturnCode matchUniqueDevice(PhidgetUniqueDeviceType, int, int, int, int, int *);

PhidgetReturnCode createPhidgetDevice(PhidgetConnectionType connType, const PhidgetUniqueDeviceDef *pdd,
  int version, const char *label, int serialNumber, PhidgetDeviceHandle *device);

PhidgetReturnCode createPhidgetVirtualDevice(const PhidgetUniqueDeviceDef *, int, const char *, int,
  PhidgetDeviceHandle *);

PhidgetReturnCode createPhidgetHIDUSBDevice(const PhidgetUniqueDeviceDef *, int, const char *, int,
  const void *, const char *, PhidgetDeviceHandle *);

PhidgetReturnCode createPhidgetPHIDUSBDevice(const PhidgetUniqueDeviceDef *pdd, int version, const char *label, int serialNumber,
  const void *devpath, const char *skuString, PhidgetDeviceHandle *device);

PhidgetReturnCode createPhidgetSPIDevice(const PhidgetUniqueDeviceDef *, int version, const char *label,
  int serialNumber, const char *skuString, PhidgetDeviceHandle *);

PhidgetReturnCode createPhidgetVINTDevice(const PhidgetUniqueDeviceDef *, int, const char *, int,
  PhidgetDeviceHandle *);

PhidgetReturnCode createPhidgetMeshDevice(const PhidgetUniqueDeviceDef *pdd, int version,
  const char *label, int serialNumber, PhidgetDeviceHandle *device);

PhidgetReturnCode createPhidgetNetDevice(const PhidgetUniqueDeviceDef *, int, const char *, int,
  const char *, PhidgetNetConnHandle, uint64_t, PhidgetDeviceHandle *);

//Exported but never appears in phidget22.h
const char * CCONV deviceInfo(PhidgetDeviceHandle, char *, uint32_t);
const char * CCONV channelInfo(PhidgetChannelHandle, char *, uint32_t);

PhidgetReturnCode waitForReads(PhidgetDeviceHandle, uint32_t numReads, uint32_t ms);

PhidgetReturnCode PhidgetChannel_bridgeInput(PhidgetChannelHandle channel, BridgePacket *bp);

void PhidgetInit(void);
void PhidgetFini(void);

typedef void(CCONV *Phidget_OnThreadExitCallback)	(void *ctx);
API_PRETURN_HDR Phidget_setOnThreadExitHandle(Phidget_OnThreadExitCallback fptr, void *ctx);

const char *clientGetHostName(PhidgetNetConnHandle nc);

uint32_t HANDLE_DATAINTERVAL_PKT(BridgePacket *bp, uint32_t interruptRate);

PhidgetReturnCode Phidget_setLastError(PhidgetReturnCode code, const char *fmt, ...);

const char *getPhidgetServerName(PhidgetDeviceHandle);

#endif //#ifndef EXTERNALPROTO

#endif //#ifndef __CPHIDGET
