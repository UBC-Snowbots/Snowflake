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

#define _PHIDGET_NETWORKCODE

#include "phidgetbase.h"
#include "network/network.h"
#include "device/hubdevice.h"
#include "device/vintdevice.h"
#include "mos/mos_task.h"
#include "mos/mos_lock.h"

#include "util/json.h"

#include "manager.h"
#include "phidget.h"
#include "stats.h"

#ifndef _WINDOWS
#include <netinet/tcp.h>	/* TCP_NODELAY */
#endif

extern uint32_t network_keepalive_client;

static void
queueNetDeviceAttach(PhidgetDeviceHandle device) {
	NetAttachDetachEntryHandle entry;

		/* callers do not always check the return value */
	MOS_ASSERT(device);

	PhidgetLockNetAttachDetachQueue();

	entry = (NetAttachDetachEntryHandle)mos_malloc(sizeof(NetAttachDetachEntry));
	memset(entry, 0, sizeof(NetAttachDetachEntry));
	entry->flags = NETATTACHQUEUE_FLAG;
	entry->dev = device;

	MTAILQ_INSERT_TAIL(&netAttachDetachQueue, entry, link);
	PhidgetRetain(device);

	PhidgetUnlockNetAttachDetachQueue();

	NotifyCentralThread();
}

static void
queueNetDeviceDetach(PhidgetDeviceHandle device) {
	NetAttachDetachEntryHandle entry;

	/* callers do not always check the return value */
	MOS_ASSERT(device);

	PhidgetLockNetAttachDetachQueue();

	entry = (NetAttachDetachEntryHandle)mos_malloc(sizeof(NetAttachDetachEntry));
	memset(entry, 0, sizeof(NetAttachDetachEntry));
	entry->flags = NETDETACHQUEUE_FLAG;
	entry->dev = device;

	MTAILQ_INSERT_TAIL(&netAttachDetachQueue, entry, link);
	PhidgetRetain(device);

	PhidgetUnlockNetAttachDetachQueue();

	NotifyCentralThread();
}

void runNetAttachDetachQueue(void);
void
runNetAttachDetachQueue() {
	NetAttachDetachEntryHandle entry;
	PhidgetDeviceHandle phid;
	PhidgetReturnCode res;

	PhidgetLockNetAttachDetachQueue();
	PhidgetWriteLockDevices();

	while ((entry = MTAILQ_FIRST(&netAttachDetachQueue)) != NULL) {
		phid = entry->dev;
		if (entry->flags == NETATTACHQUEUE_FLAG) {
			res = deviceAttach(phid, 0);
			if (res != EPHIDGET_OK)
				netlogerr("%"PRIphid": Net device attach failed with error: "PRC_FMT, PRC_ARGS(res));
		} else if (entry->flags == NETDETACHQUEUE_FLAG) {
			deviceDetach(phid);
		} else {
			MOS_PANIC("Bad State!");
		}

		MTAILQ_REMOVE(&netAttachDetachQueue, entry, link);
		mos_free(entry, sizeof(NetAttachDetachEntry));
		PhidgetRelease(&phid);
	}
	MOS_ASSERT(NET_ATTACHDETACH_QUEUE_DEVICE_EMPTY);
	MTAILQ_INIT(&netAttachDetachQueue);

	PhidgetUnlockDevices();
	PhidgetUnlockNetAttachDetachQueue();
}

static PhidgetReturnCode
addNetVintDevice(PhidgetDeviceHandle device, int childIndex, int vintID, int version, const char *label,
  int port, int vintProtocol, int supportsSetSpeed, uint32_t maxSpeed, uint32_t commSpeed,
  uint64_t pid, PhidgetNetConnHandle nc, PhidgetDeviceHandle *rvint) {
	const PhidgetUniqueDeviceDef *pdd;
	PhidgetVINTDeviceHandle vint;
	PhidgetHubDeviceHandle hub;
	PhidgetDeviceHandle phid;
	PhidgetReturnCode res;

again:
	for (pdd = Phidget_Unique_Device_Def; (int)(pdd->type) != END_OF_LIST; pdd++) {
		if (pdd->type != PHIDTYPE_VINT)
			continue;
		if (pdd->vintID != vintID)
			continue;
		if (version >= pdd->versionHigh || version < pdd->versionLow)
			continue;

		phid = getChild(device, childIndex);
		if (phid != NULL) {
			PhidgetRelease(&phid);
			setChild(device, childIndex, NULL);
		}

		res = createPhidgetNetDevice(pdd, version, label, device->deviceInfo.serialNumber, NULL, nc, pid, &phid);
		if (res != EPHIDGET_OK)
			return (res);

		PhidgetSetFlags(phid, PHIDGET_SCANNED_FLAG);
		phid->deviceInfo.uniqueIndex = childIndex;
		phid->deviceInfo.hubPort = port;
		phid->deviceInfo.isHubPort = vintID <= HUB_PORT_ID_MAX ? PTRUE : PFALSE;

		setParent(phid, device);
		setChild(device, childIndex, phid);

		vint = (PhidgetVINTDeviceHandle)phid;

		// VINT props
		vint->deviceProtocolVersion = vintProtocol;
		vint->deviceSupportsSetSpeed = supportsSetSpeed;
		vint->deviceMaxSpeed = maxSpeed;
		vint->vintCommSpeed = commSpeed;

		// Grab the port properties from the hub
		if (device->deviceInfo.UDD->class == PHIDCLASS_HUB) {
			hub = (PhidgetHubDeviceHandle)device;
			vint->portProtocolVersion = hub->portProtocolVersion[port];
			vint->portSupportsSetSpeed = hub->portSupportsSetSpeed[port];
			vint->portMaxSpeed = hub->portMaxSpeed[port];
		}

		queueNetDeviceAttach(phid);

		if (rvint)
			*rvint = phid;
		else
			PhidgetRelease(&phid);

		return (EPHIDGET_OK);
	}

	logwarn("A Network VINT Phidget (ID: 0x%03x Version: %d Hub Port: %d) was found which is not supported "
		"by the library. A library upgrade is probably required to work with this Phidget",
		vintID, version, port);

	// Attach device as the special unknown device
	if (vintID != VINTID_0xff0) {
		vintID = VINTID_0xff0;
		goto again;
	}

	return (EPHIDGET_NOENT);
}

static PhidgetReturnCode
populateHubProps(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req, PhidgetDeviceHandle phid) {
	PhidgetHubDeviceHandle hub;
	uint32_t cnt;
	int err;
	int i;

	// version 2.3 - VINT2 props
	uint32_t portProtocolVersionCnt = VINTHUB_MAXPORTS;
	uint32_t portProtocolVersion[VINTHUB_MAXPORTS] = { 1,1,1,1,1,1 };
	uint32_t portSupportsSetSpeedCnt = VINTHUB_MAXPORTS;
	uint32_t portSupportsSetSpeed[VINTHUB_MAXPORTS] = { PFALSE, PFALSE, PFALSE, PFALSE ,PFALSE, PFALSE };
	uint32_t portMaxSpeedCnt = VINTHUB_MAXPORTS;
	uint32_t portMaxSpeed[VINTHUB_MAXPORTS] = { 100000, 100000, 100000, 100000, 100000, 100000 };

	if (phid->deviceInfo.UDD->class != PHIDCLASS_HUB)
		return (EPHIDGET_OK);

	if ((nc->ppmajor > 2) || (nc->ppmajor == 2 && nc->ppminor >= 3)) {
		// Starts in 2.3
		err = parseJSON((char *)req->nr_data, req->nr_len, NULL, 0, "%O,hubPortsInfo=%O,portProto=%Au,portSuppSetSpeed=%Au,portMaxSpeed=%Au",
			&cnt, &cnt, &portProtocolVersionCnt, portProtocolVersion, &portSupportsSetSpeedCnt, portSupportsSetSpeed, &portMaxSpeedCnt, portMaxSpeed);
		if (err <= 0)
			return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "invalid json in device attach"));
		if ((int)portProtocolVersionCnt != phid->dev_hub.numVintPorts)
			return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "invalid json in device attach - wrong number of ports"));
	}

	hub = (PhidgetHubDeviceHandle)phid;

	// Populate port properties
	for (i = 0; i < phid->dev_hub.numVintPorts; i++) {
		hub->portProtocolVersion[i] = portProtocolVersion[i];
		hub->portSupportsSetSpeed[i] = portSupportsSetSpeed[i];
		hub->portMaxSpeed[i] = portMaxSpeed[i];
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
handleDeviceAttach(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {
	PhidgetDeviceHandle phid;
	PhidgetDeviceHandle vint;
	PhidgetReturnCode res;
	int err;

	char allocbuf[512];
	const char *label;
	const char *fwstr;
	const char *type;
	const char *desc;
	int interfaceNum;
	int serialNumber;
	uint64_t ppid;
	uint64_t pid;
	int productID;
	int vendorID;
	int version;
	int hubPort;
	int vintID;
	int index;
	int uid;

	uint32_t cnt;

	netlogdebug("%"PRIphid"", nc);

	if ((nc->ppmajor > 2) || (nc->ppmajor == 2 && nc->ppminor >= 1)) {
		// Starts in 2.1
		err = parseJSON((char *)req->nr_data, req->nr_len, allocbuf, sizeof(allocbuf), "%O,type=%s,"
		  "phid=%lu,parent=%lu,vendorID=%d,productID=%d,interfaceNum=%d,version=%d,serialNumber=%d,label=%s,"
		  "index=%d,vintID=%d,hubPort=%d,desc=%s,fwstr=%s",
		  &cnt, &type,
		  &pid, &ppid, &vendorID, &productID, &interfaceNum, &version, &serialNumber, &label,
		  &index, &vintID, &hubPort, &desc, &fwstr);

		if (err > 0 && mos_strlen(fwstr) == 0)
			fwstr = NULL;
	} else {
		err = parseJSON((char *)req->nr_data, req->nr_len, allocbuf, sizeof(allocbuf), "%O,type=%s,"
			"phid=%lu,parent=%lu,vendorID=%d,productID=%d,interfaceNum=%d,version=%d,serialNumber=%d,label=%s,"
			"index=%d,vintID=%d,hubPort=%d,desc=%s",
			&cnt, &type,
			&pid, &ppid, &vendorID, &productID, &interfaceNum, &version, &serialNumber, &label,
			&index, &vintID, &hubPort, &desc);
		fwstr = NULL;
	}
	if (err <= 0)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "invalid json in device attach"));

	if (mos_strlen(label) == 0)
		label = NULL;


/* USB */
	if (mos_strcasecmp(type, "USB") == 0) {
		res = matchUniqueDevice(PHIDTYPE_USB, vendorID, productID, interfaceNum, version, &uid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to find matching USB device"));

		res = createPhidgetNetDevice(&Phidget_Unique_Device_Def[uid], version, label, serialNumber, fwstr, nc, pid, &phid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to create network device"));

		res = populateHubProps(iop, nc, req, phid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to populate hub props"));

		queueNetDeviceAttach(phid);
		netlogverbose("attached %"PRIx64"/%"PRIx64" %"PRIphid"", pid, ppid, phid);
		PhidgetRelease(&phid);

/* VINT */
	} else if (mos_strcasecmp(type, "VINT") == 0) {

		// version 2.3 - VINT2 props
		int deviceProtocolVersion = 1;
		int deviceSupportsSetSpeed = PFALSE;
		uint32_t deviceMaxSpeed = PUNK_UINT32;
		uint32_t vintCommSpeed = PUNK_UINT32;

		if ((nc->ppmajor > 2) || (nc->ppmajor == 2 && nc->ppminor >= 3)) {
			// Starts in 2.3
			err = parseJSON((char *)req->nr_data, req->nr_len, NULL, 0, "%O,vintProto=%d,suppSetSpeed=%d,maxSpeed=%u,commSpeed=%u",
				&cnt, &deviceProtocolVersion, &deviceSupportsSetSpeed, &deviceMaxSpeed, &vintCommSpeed);
			if (err <= 0)
				return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "invalid json in device attach"));
		}

		res = getNetworkDevice(nc, ppid, &phid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "unable to find parent device %x", ppid));

		res = addNetVintDevice(phid, index, vintID, version, label, hubPort, deviceProtocolVersion, deviceSupportsSetSpeed,
			deviceMaxSpeed, vintCommSpeed, pid, nc, &vint);
		PhidgetRelease(&phid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to add VINT device"));
		netlogverbose("attached %"PRIphid"", vint);
		PhidgetRelease(&vint);
/* VIRTUAL */
	} else if (mos_strcasecmp(type, "VIRTUAL") == 0) {
		res = matchUniqueDevice(PHIDTYPE_VIRTUAL, vendorID, productID, interfaceNum, version, &uid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to find matching VIRTUAL device"));

		res = createPhidgetNetDevice(&Phidget_Unique_Device_Def[uid], version, label, serialNumber, fwstr, nc, pid, &phid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to create network device"));

		res = populateHubProps(iop, nc, req, phid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to populate hub props"));

		queueNetDeviceAttach(phid);
		netlogverbose("attached %"PRIx64"/%"PRIx64" %"PRIphid"", pid, ppid, phid);
		PhidgetRelease(&phid);
/* SPI */
	} else if (mos_strcasecmp(type, "SPI") == 0) {
		res = matchUniqueDevice(PHIDTYPE_SPI, vendorID, productID, interfaceNum, version, &uid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to find matching SPI device"));

		res = createPhidgetNetDevice(&Phidget_Unique_Device_Def[uid], version, label, serialNumber, fwstr, nc, pid, &phid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to create network device"));

		res = populateHubProps(iop, nc, req, phid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to populate hub props"));

		queueNetDeviceAttach(phid);
		netlogverbose("attached %"PRIx64"/%"PRIx64" %"PRIphid"", pid, ppid, phid);
		PhidgetRelease(&phid);
/* MESH */
	} else if (mos_strcasecmp(type, "MESH") == 0) {
		res = getNetworkDevice(nc, ppid, &phid);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "unable to find parent device %llx", ppid));
		PhidgetRelease(&phid);

		netlogwarn("Saw a MESH device, but not yet supported.");

		//TODO: Implement addNetMeshDevice()
		//res = addNetMeshDevice(phid, index, productID, version, pid, nc, &mesh);
		//if (res != EPHIDGET_OK)
		//	return (MOS_ERROR(iop, res, "failed to add MESH device"));
		//netloginfo("attached %"PRIphid"", mesh);
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
handleDeviceDetach(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {
	PhidgetDeviceHandle phid;
	PhidgetReturnCode res;
	int err;

	uint64_t ppid;
	uint64_t pid;
	uint32_t cnt;

	err = parseJSON((char *)req->nr_data, req->nr_len, NULL, 0, "%O,phid=%lu,parent=%lu", &cnt, &pid, &ppid);
	if (err <= 0)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "invalid json in device detach"));

	res = getNetworkDevice(nc, pid, &phid);
	if (res != EPHIDGET_OK) {
		netlogerr("unable to find device %"PRIu64"\n", pid);
		return (MOS_ERROR(iop, res, "unable to find device %llu", pid));
	}

	netlogverbose("%"PRIphid"", phid);

	queueNetDeviceDetach(phid);
	PhidgetRelease(&phid);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
handleBridgePacket(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {
	PhidgetReturnCode res;
	BridgePacket *bp;

	netlogverbose("%"PRIphid": %s/%s", nc, strmsgtype(req->nr_type), strmsgsubtype(req->nr_stype));

	res = parseBridgePacketJSON(nc->tokens, &bp, (char *)req->nr_data, req->nr_len);
	if (res != EPHIDGET_OK) {
		netlogerr("client failed to parse bridge packet: "PRC_FMT, PRC_ARGS(res));
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "invalid json in device attach"));
	}

	bridgePacketSetIsFromNet(bp, nc);

	/*
	 * Flags as an event so we do not dispatch as a command.
	 */
	if (req->nr_flags & NRF_EVENT)
		bridgePacketSetIsEvent(bp);

	return (dispatchClientBridgePacket(iop, nc, bp, 0, req->nr_reqseq));
}

static PhidgetReturnCode
handleDeviceMessage(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {

	netlogverbose("%"PRIphid": %s/%s", nc, strmsgtype(req->nr_type), strmsgsubtype(req->nr_stype));

	switch (req->nr_stype) {
	case SMSG_DEVATTACH:
		return (handleDeviceAttach(iop, nc, req));
	case SMSG_DEVDETACH:
		return (handleDeviceDetach(iop, nc, req));
	case SMSG_DEVBRIDGEPKT:
		return (handleBridgePacket(iop, nc, req));
	default:
		netlogerr("unsupported submsg: %d\n", req->nr_stype);
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "unsupported device message: %d\n", req->nr_stype));
	}
}

static PhidgetReturnCode
handleCommandMessage(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {
	mostime_t tm;

	switch (req->nr_stype) {
	case SMSG_KEEPALIVE:
		/*
		 * Deadline must be set before the writeReplyL() as writeReplyL() may block.
		 */
		tm = mos_gettime_usec();
		netlogdebug("%"PRIphid" Got keepalive and sending reply. Client deadline remaining: %"PRId64" us of %"PRId64" us", nc,
			(nc->keepalive_dl - tm), nc->keepalive);
		nc->keepalive_dl = tm + nc->keepalive;
		return (writeReplyL(iop, nc, req->nr_reqseq, MSG_COMMAND, SMSG_KEEPALIVE, NULL, 0));
	default:
		netlogerr("unknown command smsg: %d", req->nr_stype);
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "unknown command msg stype: %d", req->nr_stype));
	}
}

PhidgetReturnCode
handleDeviceClientRequest(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req, int *stop) {

	netlogverbose("%"PRIphid": %s/%s", nc, strmsgtype(req->nr_type), strmsgsubtype(req->nr_stype));

	switch (req->nr_type) {
	case MSG_CONNECT:
		if (req->nr_stype == SMSG_DGRAMSTART) {
			netloginfo("server %"PRIphid" starting DATAGRAM as requested", nc);
			NetConnWriteLock(nc);
			writeEvent(iop, nc, MSG_CONNECT, SMSG_DGRAMSTARTOK, NULL, 0, DATAGRAM_DENY);
			NetConnWriteUnlock(nc);
			return (EPHIDGET_OK);
		}
		break;
	case MSG_DEVICE:
		return (handleDeviceMessage(iop, nc, req));
	case MSG_COMMAND:
		return (handleCommandMessage(iop, nc, req));
	}

	netlogerr("unknown msg: %d", req->nr_type);
	return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "unknown msg type: %d", req->nr_type));
}

PhidgetReturnCode
openNetworkChannel(PhidgetChannelHandle channel, PhidgetDeviceHandle device, int uniqueIndex, char **errBuf) {
	PhidgetNetworkConnectionHandle netConn;
	const PhidgetChannelAttributeDef *def;
	PhidgetReturnCode res, rres;
	WaitForReply *wfr;
	BridgePacket *bp;
	uint32_t len;
	int version;

	netlogdebug("");

	def = getPhidgetChannelAttributesByClass(channel->class);
	MOS_ASSERT(def != NULL);
	version = def->version;

again:
	netConn = PhidgetNetworkConnectionCast(getPhidgetConnection(device));
	MOS_ASSERT(netConn != NULL);

	NetConnWriteLock(netConn->nc);
	len = mkJSON(netConn->nc->databuf, netConn->nc->databufsz,
	  "{phid:%lu,channel:%lu,class:%d,index:%d,version:%d}",
	  netConn->id, PHIDID(channel), channel->class, uniqueIndex, version);
	res = writeRequest(MOS_IOP_IGNORE, netConn->nc, 0, MSG_DEVICE, SMSG_DEVOPEN, NULL, len, &wfr);
	NetConnWriteUnlock(netConn->nc);

	if (res != EPHIDGET_OK) {
		PhidgetRelease(&netConn);
		return (res);
	}

	res = waitForReply(wfr);
	if (res != EPHIDGET_OK) {
		closeWaitForReply(&wfr);
		MOS_ASSERT(wfr == NULL);
		PhidgetRelease(&netConn);
		return (res);
	}

	/* error occured */
	if (wfr->req.nr_type == MSG_COMMAND) {
		PhidgetRelease(&netConn);
		res = simpleWaitForReply(&wfr, &rres, errBuf, NULL, NULL);
		if (res != EPHIDGET_OK)
			return (res);

		// We will try with versions from current down to zero because older servers enforce matching channel class version
		if (rres == EPHIDGET_BADVERSION && version > 0) {
			version--;
			goto again;
		}

		return (rres);
	}

	/* Should get DEVICE/BRIDGEPKT */
	if (wfr->req.nr_type != MSG_DEVICE || wfr->req.nr_stype != SMSG_DEVBRIDGEPKT) {
		closeWaitForReply(&wfr);
		MOS_ASSERT(wfr == NULL);
		PhidgetRelease(&netConn);
		return (EPHIDGET_INVALIDARG);
	}

	res = parseBridgePacketJSON(NULL, &bp, (char *)wfr->req.nr_data, wfr->req.nr_len);
	closeWaitForReply(&wfr);
	MOS_ASSERT(wfr == NULL);
	if (res != EPHIDGET_OK) {
		PhidgetRelease(&netConn);
		return (res);
	}

	bridgePacketSetIsFromNet(bp, netConn->nc);

	res = channel->setStatus(channel, bp);

	PhidgetRelease(&netConn);
	destroyBridgePacket(&bp);

	return (res);
}

PhidgetReturnCode
closeNetworkChannel(PhidgetChannelHandle channel) {
	PhidgetNetworkConnectionHandle netConn;
	PhidgetReturnCode res, rres;
	WaitForReply *wfr;
	uint32_t len;

	netlogdebug("");

	netConn = PhidgetNetworkConnectionCast(getPhidgetConnection(channel));
	if (!netConn)
		return EPHIDGET_OK;

	NetConnWriteLock(netConn->nc);
	len = mkJSON(netConn->nc->databuf, netConn->nc->databufsz, "{phid:%lu,index:%d}",
	  netConn->id, channel->uniqueIndex);
	res = writeRequest(MOS_IOP_IGNORE, netConn->nc, 0, MSG_DEVICE, SMSG_DEVCLOSE, NULL, len, &wfr);
	NetConnWriteUnlock(netConn->nc);
	PhidgetRelease(&netConn);
	if (res != EPHIDGET_OK)
		return (res);

	res = simpleWaitForReply(&wfr, &rres, NULL, NULL, NULL);
	MOS_ASSERT(wfr == NULL);
	if (res != EPHIDGET_OK)
		return (res);
	return (rres);
}

static MOS_TASK_RESULT
runClient(void *arg) {
	PhidgetNetConnHandle nc;
	PhidgetReturnCode res;
	mosiop_t iop;
	int stop;


	iop = mos_iop_alloc();

	/*
	 * Should have been retained by our creator.
	 */
	nc = arg;

	mos_task_setname("Phidget22 Network Client Thread - %s", nc->peername);
	netlogdebug("network client thread started - '%s': 0x%08x", nc->peername, mos_self());

	nc->keepalive = network_keepalive_client;

	res = EPHIDGET_OK;

	startKeepAliveTask(nc);

	for (stop = 0; stop == 0;) {
		stop = PhidgetCKFlags(nc, PNCF_STOP);
		if (stop || res != EPHIDGET_OK)
			break;

		if (nc->errcondition != 0) {
			res = nc->errcondition;
			break;
		}

		res = handleNetworkRequest(iop, nc, &stop);
		if (res == EPHIDGET_TIMEOUT) {
			res = EPHIDGET_OK;
			continue;
		}
	}
	PhidgetBroadcast(nc);	/* wake anybody waiting for us: namely the keep alive task */

	if (stop == 0 && res != EPHIDGET_OK)
		netlogerr("clientHandleMessage() for %s failed: "PRC_FMT"\n%N", nc->peername, PRC_ARGS(res), iop);

	netloginfo("'%s': client thread closing: %sby request", nc->peername, stop ? "" : "not ");
	mos_iop_release(&iop);

	PhidgetCLRFlags(nc, PNCF_HASTHREAD);
	PhidgetNetConnClose(nc);
	PhidgetRelease(&nc);

	decPhidgetStat("client.tasks");
	MOS_TASK_EXIT(res);
}

PhidgetReturnCode
clientConnect(mos_af_t af, const char *address, int port, const char *passwd, const char *proto,
  int pmajor, int pminor, handleRequest_t handleRequest, void *private, PhidgetNetConnHandle *_nc) {
	PhidgetNetConnHandle nc;
	PhidgetReturnCode err;
	mos_sockaddr_t sa;
	mosiop_t iop;

	if (passwd == NULL)
		return (EPHIDGET_INVALIDARG);

	if (af != MOS_AF_INET4 && af != MOS_AF_INET6)
		return (EPHIDGET_INVALIDARG);

	createPhidgetNetConn(NULL, &nc);
	nc->conntype = PHIDGETCONN_REMOTE;
	nc->handleRequest = handleRequest;
	nc->sock = MOS_INVALID_SOCKET;
	nc->protocol = mos_strdup(proto, NULL);
	nc->pmajor = pmajor;
	nc->pminor = pminor;
	nc->private = private;

	iop = mos_iop_alloc();

	err = mos_netop_getbyname(iop, address, af, &nc->addr);
	if (err != 0) {
		netlogwarn("failed to resolve name '%s'\n%N", address, iop);
		goto bad;
	}

	switch (af) {
	case MOS_AF_INET4:
		nc->addr.s4.sin_port = htons(port);
		nc->addr.s4.sin_family = af;
		nc->dgaddr.s4.sin_port = 0;	/* random port */
		nc->dgaddr.s4.sin_family = af;
		break;
	case MOS_AF_INET6:
		nc->addr.s6.sin6_port = htons(port);
		nc->addr.s6.sin6_family = af;
		nc->dgaddr.s6.sin6_port = 0;	/* random port */
		nc->dgaddr.s6.sin6_family = af;
		break;
	}

	err = mos_netop_tcp_opensocket(iop, &nc->sock, &nc->addr);
	if (err != 0) {
		netlogverbose("failed to open client socket to %s:%d\n%N", address, port, iop);
		goto bad;
	}

	mos_netop_setnodelay(&nc->sock);

	err = mos_netop_getpeername(MOS_IOP_IGNORE, &nc->sock, &sa);
	if (err != EPHIDGET_OK)
		nc->peername = mos_strdup(address, NULL);
	else
		nc->peername = mos_strdup(mos_getaddrinfo(&sa, NULL, 0), NULL);

	if (af == MOS_AF_INET4) {
		netlogdebug("Creating DataGram socket");
		err = mos_netop_getsockname(MOS_IOP_IGNORE, &nc->sock, &sa);
		if (err != 0) {
			netlogwarn("Failed to start datagram communication: unable to determine local bind address");
			goto startconnection;
		}
		nc->dgaddr.s4.sin_addr = sa.s4.sin_addr;
		err = mos_netop_udp_openserversocket(iop, &nc->dgsock, &nc->dgaddr);
		if (err != 0) {
			netlogwarn("Failed to start datagram communication: unable to create socket");
			goto startconnection;
		}
		mos_netop_udp_setnonblocking(iop, &nc->dgsock, 1);
		mos_netop_setrecvbufsize(iop, &nc->dgsock, 65536);

		/*
		 * Flag that we support datagram if we opened the socket.
		 * We never send, so we do not enable datagram.
		 */
		PhidgetSetFlags(nc, PNCF_DGRAM);
	} else {
		netlogwarn("DataGram only supported with INET4");
	}

startconnection:
	netloginfo("Starting client connection to '%s'", nc->peername);
	NetConnWriteLock(nc);
	err = startClientConnection(iop, nc, passwd);
	NetConnWriteUnlock(nc);
	if (err != EPHIDGET_OK) {
		netlogerr("failed to start client connection to '%s'\n%N", nc->peername, iop);
		if (err == EPHIDGET_ACCESS)
			clientAuthFailed(nc);
		if (err == EPHIDGET_BADVERSION)
			clientProtocolFailure(nc);
		goto bad;
	}

	PhidgetRetain(nc);
	PhidgetSetFlags(nc, PNCF_HASTHREAD);
	err = mos_task_create(&nc->rself, runClient, nc);
	if (err != 0) {
		PhidgetCLRFlags(nc, PNCF_HASTHREAD);
		goto bad;
	}
	incPhidgetStat("client.tasks_ever");
	incPhidgetStat("client.tasks");

	netlogdebug("client started ok: %s:%d ", address, port);

	mos_iop_release(&iop);

	if (_nc)
		*_nc = nc;
	else
		PhidgetRelease(&nc);

	return (0);

bad:

	PhidgetNetConnClose(nc);
	PhidgetRelease(&nc);

	mos_iop_release(&iop);
	return (err);
}
