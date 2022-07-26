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

#include "manager.h"
#include "network/network.h"
#include "network/zeroconf.h"
#include "mos/mos_assert.h"
#include "mos/mos_task.h"
#include "mos/mos_lock.h"
#include "mos/mos_random.h"
#include "mos/mos_base64.h"
#include "mos/mos_sha2.h"
#include "mos/mos_byteorder.h"
#include "util/json.h"

#define NET_IDENT		"phidgetclient"

static void devicerelease(PhidgetNetConnHandle *);

extern int _allowDataGram;

void
PhidgetNetInit() {

	NetworkControlInit();
	ServersInit();
	ServerInit();
#if ZEROCONF_SUPPORT
	ZeroconfInit();
#endif
}

void
PhidgetNetFini() {

	ServerFini();
	ServersFini();
#if ZEROCONF_SUPPORT
	ZeroconfFini();
#endif
	NetworkControlFini();
}

static int networkStartRefCnt;
static int networkStarted;

int
PhidgetNet_isStarted() {
	int started;

	mos_glock((void *)1);
	started = networkStarted;
	mos_gunlock((void *)1);

	return (started);
}

#define CHECKSTARTED do {			\
	if (!PhidgetNet_isStarted())	\
		return (EPHIDGET_CLOSED);	\
} while (0)

void
PhidgetNet_start() {
	int runStart;

	runStart = 0;

	mos_glock((void *)1);
	if (networkStartRefCnt == 0) {
		networkStartRefCnt++;
		runStart = 1;
	} else if (networkStarted) {
		networkStartRefCnt++;
		mos_gunlock((void *)1);
		return;
	}
	mos_gunlock((void *)1);

	if (runStart) {
		netloginfo("Starting Networking");
		ServerStart();
		ServersStart();
#if ZEROCONF_SUPPORT
		ZeroconfStart();
#endif
		NetworkControlStart();

		mos_glock((void *)1);
		networkStarted = 1;
		mos_gunlock((void *)1);
		return;
	}

	/*
	 * Do not return until networking is started.
	 */
	for (;;) {
		mos_glock((void *)1);
		if (networkStarted) {
			mos_gunlock((void *)1);
			return;
		}
		mos_gunlock((void *)1);
		mos_yield();
	}
}

void
PhidgetNet_stop() {
	int runStop;

	runStop = 0;

	mos_glock((void *)1);
	MOS_ASSERT(networkStartRefCnt > 0);
	if (networkStartRefCnt == 1)
		runStop = 1;
	networkStartRefCnt--;
	mos_gunlock((void *)1);

	if (runStop) {
		netloginfo("Stopping Networking");
		ServersStop();
		ServerStop();
		NetworkControlStop();
#if ZEROCONF_SUPPORT
		ZeroconfStop();
#endif
		mos_glock((void *)1);
		networkStartRefCnt = 0;
		networkStarted = 0;
		mos_gunlock((void *)1);
	}
}

const char * CCONV
netConnInfo(PhidgetNetConnHandle nc, char *ubuf, uint32_t buflen) {
	static char sbuf[128];
	const char *type;
	char *buf;

	buf = ubuf;
	if (buf == NULL) {
		buflen = sizeof (sbuf);
		buf = sbuf;
	}

	type = nc->conntypestr;
	if (type == NULL) {
		switch (nc->conntype) {
		case PHIDGETCONN_REMOTE:
			type = "netconnclient";
			break;
		case PHIDGETCONN_LOCAL:
			type = "netconnserver";
			break;
		default:
			type = "netconn";
			break;
		}
	}

	mos_snprintf(buf, buflen, "%s://%s", type, nc->peername);
	return (buf);
}

static PhidgetReturnCode
netConnDetachChannels(PhidgetDeviceHandle phid, PhidgetNetConnHandle nc) {
	PhidgetChannelHandle channel;
	PhidgetReturnCode res, rres;
	int cnt;
	int i;

	rres = EPHIDGET_NOENT;

	for (i = 0; i < PHIDGET_MAXCHANNELS; i++) {
		channel = getChannel(phid, i);
		if (channel == NULL)
			continue;

		/* Is set if the channel was opened over the network (server side) */
		if (PhidgetCKFlags(channel, PHIDGET_OPENBYNETCLIENT_FLAG) == 0) {
			PhidgetRelease(&channel);
			continue;
		}

		res = removeChannelNetworkConnection(channel, nc, &cnt);
		if (res == EPHIDGET_OK)
			rres = EPHIDGET_OK;

		/*
		 * If there are no remaining network references, close the channel.
		 * This is a noop if the channel is already closed, and will end up
		 * releasing the network reference in channelDetach().
		 */
		if (res == EPHIDGET_OK && cnt == 0) {
			loginfo("%"PRIphid" last network reference removed: closing", channel);
			(void)Phidget_close((PhidgetHandle)channel);
		}

		PhidgetRelease(&channel);
	}

	return (rres);
}

static PhidgetReturnCode
netConnDetachDevice(PhidgetNetConnHandle nc) {
	PhidgetDeviceHandle phid, tmp;
	PhidgetNetworkConnectionHandle conn;

	PhidgetWriteLockDevices();
	FOREACH_DEVICE(phid) {
		/*
		 * If the device is not open, there cannot be any attached channels.
		 */
		if (!ISOPEN(phid))
			continue;

		/*
		 * Remove any references this connection may have to open channels.
		 */
		netConnDetachChannels(phid, nc);
	}

	FOREACH_DEVICE_SAFE(phid, tmp) {
		if (!isNetworkPhidget(phid))
			continue;

		/*
		 * If the device is network attached on this connection, detach it.
		 */
		conn = PhidgetNetworkConnectionCast(phid->conn);
		assert(conn);
		if (conn->nc == nc) {
			chlog("%"PRIphid"", phid);
			deviceDetach(phid);
		}
	}
	PhidgetUnlockDevices();

	return (EPHIDGET_OK);
}

static void
devicerelease(PhidgetNetConnHandle *nc) {

	mos_free(*nc, sizeof(PhidgetNetConn));
	*nc = NULL;
}

PhidgetReturnCode
openWaitForReply(uint16_t repseq, PhidgetNetConnHandle nc, WaitForReply **_wfr) {
	WaitForReply *wfr;

	wfr = mos_zalloc(sizeof(*wfr));
	mos_tlock_init(wfr->lock, P22LOCK_WFRLOCK, P22LOCK_FLAGS);
	mos_cond_init(&wfr->cond);
	wfr->waittime = WFR_WAITTIME;
	wfr->nc = nc;
	PhidgetRetain(nc);
	wfr->flags |= WFR_WAITING;
	wfr->req.nr_repseq = repseq;

	PhidgetLock(nc);
	MTAILQ_INSERT_HEAD(&nc->waitforreply, wfr, link);
	wfr->flags |= WFR_ONLIST;
	PhidgetUnlock(nc);

	netlogverbose("%d", repseq);

	*_wfr = wfr;
	return (EPHIDGET_OK);
}

void
cancelWaitForReply(WaitForReply *wfr) {

	if (wfr == NULL)
		return;

	mos_tlock_lock(wfr->lock);
	wfr->flags &= ~WFR_WAITING;
	mos_tlock_unlock(wfr->lock);
}

/*
 * wfr must be locked.
 */
static PhidgetReturnCode
handleWaitForReply(WaitForReply *wfr, netreq_t *req) {

	if ((wfr->flags & WFR_WAITING) == 0 || wfr->flags & WFR_CANCELLED)
		return (EPHIDGET_UNEXPECTED);

	assert(wfr->req.nr_repseq == req->nr_repseq);
	wfr->req = *req;
	wfr->flags |= WFR_RECEIVED;
	mos_cond_broadcast(&wfr->cond);
	return (EPHIDGET_OK);
}

PhidgetReturnCode
waitForReply(WaitForReply *wfr) {
	mostime_t waittime;
	mostime_t starttm;
	mostime_t tm;

	starttm = mos_gettime_usec() / 1000;
	waittime = wfr->waittime;

	mos_tlock_lock(wfr->lock);
	for (;;) {

		/*
		 * The reply was received, and the result is in wfr.
		 */
		if (wfr->flags & WFR_RECEIVED) {
			wfr->flags &= ~WFR_WAITING;	/* clear waiting so we can close */
			mos_tlock_unlock(wfr->lock);
			return (EPHIDGET_OK);
		}

		/*
		 * The reply was cancelled (probably because the connection was closed), or there was an error.
		 */
		if (wfr->flags & (WFR_CANCELLED | WFR_ERROR)) {
			wfr->flags &= ~WFR_WAITING;	/* clear waiting so we can close */
			mos_tlock_unlock(wfr->lock);
			return (EPHIDGET_UNEXPECTED);
		}

		tm = mos_gettime_usec() / 1000;

		/*
		 * Check for timeout
		 */
		if (waittime != 0 && tm - starttm >= waittime) {
			wfr->flags &= ~WFR_WAITING;	/* clear waiting so we can close */
			mos_tlock_unlock(wfr->lock);
			return (EPHIDGET_TIMEOUT);
		}

		tm = waittime - (tm - starttm);

		mos_tlock_timedwait(&wfr->cond, wfr->lock, tm * 1000000); //ms -> ns
	}
}

void
closeWaitForReply(WaitForReply **_wfr) {
	WaitForReply *wfr;

	if (_wfr == NULL)
		return;

	wfr = *_wfr;

	PhidgetLock(wfr->nc);
	mos_tlock_lock(wfr->lock);

	if (wfr->flags & WFR_ONLIST) {
		MTAILQ_REMOVE(&wfr->nc->waitforreply, wfr, link);
		wfr->flags &= ~WFR_ONLIST;
	}

	wfr->flags |= WFR_CANCELLED;

	/*
	 * If a thread is waiting on this, wake it and return.
	 */
	if (wfr->flags & WFR_WAITING) {
		mos_cond_broadcast(&wfr->cond);
		mos_tlock_unlock(wfr->lock);
		PhidgetUnlock(wfr->nc);
		return;
	}

	mos_tlock_unlock(wfr->lock);
	PhidgetUnlock(wfr->nc);
	PhidgetRelease(&wfr->nc);

	mos_tlock_destroy(&wfr->lock);
	mos_cond_destroy(&wfr->cond);

	mos_free(wfr, sizeof(*wfr));

	*_wfr = NULL;
}

PhidgetReturnCode
handleReply(PhidgetNetConnHandle nc, netreq_t *req) {
	PhidgetReturnCode res;
	WaitForReply *wfr;

	if (req->nr_type == MSG_COMMAND && req->nr_stype == SMSG_KEEPALIVE) {
		if (nc->server) {
			nc->keepalive_last = mos_gettime_usec();
			netlogdebug("keepalive reply received (%"PRIphid") in %"PRId64" us (at %"PRId64")", nc,
				(nc->keepalive_last - (nc->keepalive_dl - nc->keepalive)), nc->keepalive_last);
			nc->keepalive_dl = 0;
		} else {
			netlogdebug("keepalive reply received (%"PRIphid")", nc);
		}
		return (EPHIDGET_OK);
	}

	netlogdebug("reply %d", req->nr_repseq);
	PhidgetLock(nc);
	MTAILQ_FOREACH(wfr, &nc->waitforreply, link) {
		mos_tlock_lock(wfr->lock);
		if (wfr->req.nr_repseq == req->nr_repseq) {
			res = handleWaitForReply(wfr, req);
			mos_tlock_unlock(wfr->lock);
			PhidgetUnlock(nc);
			return (res);
		}
		mos_tlock_unlock(wfr->lock);
	}
	PhidgetUnlock(nc);

	netloginfo("handleReply(): no match for reqseq %d", req->nr_repseq);
	return (EPHIDGET_NOENT);
}

PhidgetReturnCode
simpleWaitForReply(WaitForReply **_wfr, PhidgetReturnCode *rres, char **ureply, uint8_t **dataReply, size_t *dataReplyLen) {
	PhidgetReturnCode res;
	WaitForReply *wfr;
	uint32_t cnt;
	char *reply;
	char *base64Data;
	uint32_t dlen;
	int err;

	wfr = *_wfr;

	if (ureply != NULL)
		*ureply = NULL;

	if (dataReply != NULL)
		*dataReply = NULL;

	res = waitForReply(wfr);
	if (res != EPHIDGET_OK) {
		closeWaitForReply(_wfr);
		return (res);
	}

	reply = NULL;
	base64Data = NULL;

	err = parseJSON((char *)wfr->req.nr_data, wfr->req.nr_len, NULL, 0, "%O,E=%uR?=%sD?=%s", &cnt, rres, &reply, &base64Data);
	if (err == 4 && reply != NULL && ureply != NULL) {
		*ureply = reply;
	} else if (reply != NULL) {
		mos_free(reply, MOSM_FSTR);
	}
	if (err == 4 && base64Data != NULL && dataReply != NULL && dataReplyLen != NULL) {
		*dataReply = mos_base64_decode((uint8_t *)base64Data, (uint32_t)strlen(base64Data), &dlen);
		*dataReplyLen = dlen;
	} else if (base64Data != NULL) {
		mos_free(base64Data, MOSM_FSTR);
	}
	closeWaitForReply(_wfr);
	if (err <= 0)
		return (EPHIDGET_INVALIDARG);

	return (EPHIDGET_OK);
}

/*
 * nc must be locked
 */
static PhidgetReturnCode
_sendSimpleReply(PhidgetNetConnHandle nc, uint16_t repseq, PhidgetReturnCode rcode, const char *reply) {
	PhidgetReturnCode res;
	uint32_t len;
	int err;

	if (reply != NULL)
		err = mkJSON(nc->databuf, nc->databufsz, "{E=%u,R=%s}", rcode, reply);
	else
		err = mkJSON(nc->databuf, nc->databufsz, "{E=%u}", rcode);
	if (err < 0) {
		netlogerr("failed to render json for reply to %"PRIphid"", nc);
		return (EPHIDGET_UNEXPECTED);
	}

	len = (uint32_t)mos_strlen(nc->databuf);
	MOS_ASSERT(len == (uint32_t)err);

	res = writeReply(MOS_IOP_IGNORE, nc, repseq, MSG_COMMAND, SMSG_REPLY, NULL, len);
	return (res);
}

PhidgetReturnCode
sendSimpleReply(PhidgetNetConnHandle nc, uint16_t repseq, PhidgetReturnCode rcode, const char *reply) {
	PhidgetReturnCode res;

	NetConnWriteLock(nc);
	res = _sendSimpleReply(nc, repseq, rcode, reply);
	NetConnWriteUnlock(nc);

	return (res);
}

/*
* nc must be locked
*/
static PhidgetReturnCode
_sendReply(PhidgetNetConnHandle nc, uint16_t repseq, PhidgetReturnCode rcode, const void *data, uint32_t dlen) {
	PhidgetReturnCode res;
	uint32_t base64Len;
	uint8_t *base64;
	uint32_t len;
	int err;

	if (data != NULL) {
		base64 = mos_base64_encode(data, dlen, &base64Len);
		if (base64 == NULL)
			return (EPHIDGET_UNEXPECTED);
		err = mkJSON(nc->databuf, nc->databufsz, "{E=%u,D=%s}", rcode, base64);
		mos_free(base64, base64Len);
	} else {
		err = mkJSON(nc->databuf, nc->databufsz, "{E=%u}", rcode);
	}
	if (err < 0) {
		netlogerr("failed to render json for reply to %"PRIphid"", nc);
		return (EPHIDGET_UNEXPECTED);
	}

	len = (uint32_t)mos_strlen(nc->databuf);
	MOS_ASSERT(len == (uint32_t)err);

	res = writeReply(MOS_IOP_IGNORE, nc, repseq, MSG_COMMAND, SMSG_REPLY, NULL, len);
	return (res);
}

PhidgetReturnCode
sendReply(PhidgetNetConnHandle nc, uint16_t repseq, PhidgetReturnCode rcode, const void *data, uint32_t dlen) {
	PhidgetReturnCode res;

	NetConnWriteLock(nc);
	res = _sendReply(nc, repseq, rcode, data, dlen);
	NetConnWriteUnlock(nc);

	return (res);
}

/*********************************************** PROTOCOL ************************************************/

PhidgetReturnCode
deviceread(mosiop_t iop, PhidgetNetConnHandle nc, void *vbuf, uint32_t *len) {

	return (pnread(iop, nc, vbuf, len));
}

API_PRETURN
pnread(mosiop_t iop, PhidgetNetConnHandle nc, void *vbuf, uint32_t *len) {
	mos_socket_t s;
	uint32_t nread;
	size_t n;
	int err;

	s = nc->sock;

	for (nread = 0; nread < *len; nread += (uint32_t)n) {
		if (nc->errcondition != EPHIDGET_OK)
			return (nc->errcondition);

		n = (size_t)(*len - nread);
		err = mos_netop_tcp_read(MOS_IOP_IGNORE, &s, ((uint8_t *)vbuf) + nread, &n);
		if (err != 0) {
			if (err == MOSN_AGAIN) {
				n = 0;
				continue;
			}
			return (MOS_ERROR(iop, err, "TCP read failed"));
		}
		if (n == 0)
			break;
	}
	*len = nread;

	return (0);
}

PhidgetReturnCode
devicewrite(mosiop_t iop, PhidgetNetConnHandle nc, const void *vbuf, uint32_t len) {

	return (pnwrite(iop, nc, vbuf, len));
}

API_PRETURN
pnwrite(mosiop_t iop, PhidgetNetConnHandle nc, const void *vbuf, uint32_t len) {
	mos_socket_t s;
	uint32_t nwr;
	size_t n;
	int err;

	s = nc->sock;

	for (nwr = 0; nwr < len; nwr += (uint32_t)n) {
		if (nc->errcondition != EPHIDGET_OK)
			return (nc->errcondition);

		n = len - nwr;
		err = mos_netop_tcp_write(iop, &s, ((const uint8_t *)vbuf) + nwr, &n);
		if (err != 0) {
			if (err == MOSN_AGAIN) {
				n = 0;
				continue;
			}
			return (MOS_ERROR(iop, err, "TCP write failed"));
		}
		if (n == 0)
			return (MOS_ERROR(iop, EPHIDGET_IO, "stream handled %u bytes", len - nwr));
	}

	return (0);
}

PhidgetReturnCode
readRequestHeader(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {
	PhidgetReturnCode res;
	uint32_t len;

	len = NR_HEADERLEN;
	res = nc->read(iop, nc, req->nr_buf, &len);
	if (res != 0)
		return (MOS_ERROR(iop, res, "failed to read request header"));

	nc->io_in += len;

	if (len != NR_HEADERLEN)
		return (MOS_ERROR(iop, EPHIDGET_IO, "failed to read complete request header"));

	req->nr_magic = mos_le32toh(req->nr_magic);
	req->nr_len = mos_le32toh(req->nr_len);
	req->nr_flags = mos_le16toh(req->nr_flags);
	req->nr_reqseq = mos_le16toh(req->nr_reqseq);
	req->nr_repseq = mos_le16toh(req->nr_repseq);

	if (req->nr_magic != NR_HEADERMAGIC) {
		netlogerr("bad magic read from request header: %x", req->nr_magic);
		return (MOS_ERROR(iop, EPHIDGET_IO, "invalid magic in request header"));
	}

	if (req->nr_len > NR_MAXDATALEN)
		return (MOS_ERROR(iop, EPHIDGET_IO, "invalid length %d", req->nr_len));

	len = req->nr_len;
	res = nc->read(iop, nc, req->nr_data, &len);
	req->nr_data[len] = '\0';

	if (res != EPHIDGET_OK || len != req->nr_len)
		return (MOS_ERROR(iop, EPHIDGET_IO, "failed to read data"));

	nc->io_in += len;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
readDGRequestHeader(mosiop_t iop, PhidgetNetConnHandle nc, netreq_t *req) {
	PhidgetReturnCode res;
	uint64_t dgseq;
	size_t len;

	if (PhidgetCKFlags(nc, PNCF_DGRAM) == 0)
		return (EPHIDGET_UNSUPPORTED);

	len = NR_MAXDGDATALEN + 8;	/* we should never see anything bigger */
	res = mos_netop_udp_recv(iop, &nc->dgsock, req->nr_buf, &len);
	if (res != EPHIDGET_OK) {
		if (res == EPHIDGET_AGAIN)
			return (res);
		return (MOS_ERROR(iop, res, "failed to read from dgram socket"));
	}

	nc->io_in += len;

	if (len < NR_HEADERLEN)
		return (MOS_ERROR(iop, EPHIDGET_IO, "failed to read complete request header"));

	req->nr_magic = mos_le32toh(req->nr_magic);
	req->nr_len = mos_le32toh(req->nr_len);
	req->nr_flags = mos_le16toh(req->nr_flags);
	req->nr_reqseq = mos_le16toh(req->nr_reqseq);
	req->nr_repseq = mos_le16toh(req->nr_repseq);

	if (req->nr_magic != NR_HEADERMAGIC) {
		netlogerr("bad magic read from request header: %x", req->nr_magic);
		return (MOS_ERROR(iop, EPHIDGET_IO, "invalid magic in request header"));
	}

	if (req->nr_len > NR_MAXDGDATALEN) {
		netlogerr("invalid length: %d", req->nr_len);
		return (MOS_ERROR(iop, EPHIDGET_IO, "invalid length %d", req->nr_len));
	}

	if (len != NR_HEADERLEN + req->nr_len + 8) {
		netlogerr("length does not calcuate: %zu", len);
		return (MOS_ERROR(iop, EPHIDGET_IO, "packet length does not calculate"));
	}

	/*
	 * Do not accept out of order packets.
	 */
	memcpy(&dgseq, req->nr_data + req->nr_len, 8);
	dgseq = mos_le64toh(dgseq);
	if (dgseq <= nc->dgseq) {
		netlogwarn("packet out of order %"PRIu64" vs %"PRIu64, dgseq, nc->dgseq);
		return (EPHIDGET_INVALIDPACKET);
	}
	if (dgseq > nc->dgseq + 1)
		netlogwarn("%"PRIu64" packets lost: %"PRIu64" vs %"PRIu64, dgseq - nc->dgseq, dgseq, nc->dgseq);
	nc->dgseq = dgseq;

	req->nr_data[req->nr_len] = '\0';

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
makeRequestHeader(mosiop_t iop, PhidgetNetConnHandle nc, uint32_t len, uint16_t flags, uint16_t reqseq,
  uint16_t repseq, msgtype_t type, msgsubtype_t stype) {
	netreq_t req;

	if (nc->hdrbufsz != NR_HEADERLEN)
		return (MOS_ERROR(iop, EPHIDGET_NOSPC, "invalid header buffer size"));

	req.nr_magic = mos_htole32(NR_HEADERMAGIC);
	req.nr_len = mos_htole32(len);
	req.nr_flags = mos_htole16(flags);
	req.nr_type = type;
	req.nr_stype = stype;
	req.nr_reqseq = mos_htole16(reqseq);
	req.nr_repseq = mos_htole16(repseq);

	memcpy(nc->hdrbuf, req.nr_buf, NR_HEADERLEN);

	return (0);
}

/*
 * Writes a packet to the network.
 *
 * If dgram is 1, and the connection has dgram enabled an attempt is made to write to the UDP socket.
 * If dgram is 2, an attempt is always made to write to the UDP socket.
 */
static PhidgetReturnCode
ncwrite(mosiop_t iop, PhidgetNetConnHandle nc, uint32_t datalen, int dgram) {
	PhidgetReturnCode res;
	uint64_t dgseq;
	uint32_t flags;
	size_t len;

	/*
	 * Ensure the connection was not closed after we got the write lock.
	 *
	 * The writelock is used between the real write and PhidetNetConnClose() to ensure the connection
	 * isn't torn out from under the write.
	 */
	flags = PhidgetCKFlags(nc, PNCF_CLOSED | PNCF_DGRAMENABLED);
	if (flags & PNCF_CLOSED)
		return (EPHIDGET_OK);

	/*
	 * If dgram is true, and dgram is enabled on the connection, try to send a data gram.
	 */
	if (dgram == DATAGRAM_FORCE ||
	  (_allowDataGram && dgram != DATAGRAM_DENY && flags & PNCF_DGRAMENABLED && datalen <= NR_MAXDGDATALEN)) {
		nc->dgseq++;
		dgseq = mos_htole64(nc->dgseq);
		memcpy(nc->databuf + datalen, &dgseq, sizeof (dgseq));
		len = NR_HEADERLEN + datalen + sizeof (dgseq);
		res = mos_netop_udp_send(iop, &nc->dgsock, nc->iobuf, &len);
		if (res == EPHIDGET_OK)
			goto done;
		netlogwarn("udp send failed: %N", iop);
		if (dgram == 2)
			return (MOS_ERROR(iop, res, "failed to write forced datagram packet"));
	}

	res = nc->write(iop, nc, nc->iobuf, NR_HEADERLEN + datalen);

done:
	if (res == EPHIDGET_OK)
		nc->io_out += datalen;

	return (res);
}


static PhidgetReturnCode
writeNetConn(mosiop_t iop, PhidgetNetConnHandle nc, int flags, uint16_t reqseq, uint16_t repseq,
	msgtype_t type, msgsubtype_t stype, const void *data, uint32_t dlen, int datagram, WaitForReply **wfr) {
	PhidgetReturnCode res;

	if (dlen > NR_MAXDATALEN)
		return (MOS_ERROR(iop, EPHIDGET_NOSPC, "data too large (%u > %u)", dlen, NR_MAXDATALEN));

	if (wfr) {
		res = openWaitForReply(nc->reqseq, nc, wfr);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to open WaitForReply"));
	}

	res = makeRequestHeader(iop, nc, dlen, flags, nc->reqseq, repseq, type, stype);
	if (res != EPHIDGET_OK) {
		MOS_ERROR(iop, res, "failed to create request header");
		goto bad;
	}

	if (data)
		memcpy(nc->databuf, data, dlen);

	res = ncwrite(iop, nc, dlen, datagram);
	if (res != EPHIDGET_OK) {
		MOS_ERROR(iop, res, "failed to write to network connection");
		goto bad;
	}

	return (EPHIDGET_OK);

bad:
	if (wfr) {
		cancelWaitForReply(*wfr);
		closeWaitForReply(wfr);
	}
	return (res);
}

PhidgetReturnCode
writeEvent(mosiop_t iop, PhidgetNetConnHandle nc, msgtype_t type, msgsubtype_t stype,
  const void *data, uint32_t dlen, int datagram) {
	PhidgetReturnCode res;

	res = writeNetConn(iop, nc, NRF_EVENT, nc->reqseq++, 0, type, stype, data, dlen, datagram, NULL);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to write event"));

	nc->io_ev++;
	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeRequest(mosiop_t iop, PhidgetNetConnHandle nc, int flags, msgtype_t type, msgsubtype_t stype,
  const void *data, uint32_t dlen, WaitForReply **wfr) {
	PhidgetReturnCode res;

	if (flags & ~NRF_USERMASK)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "invalid flags specified: 0x%x", flags));

	res = writeNetConn(iop, nc, NRF_REQUEST | flags, nc->reqseq++, 0, type, stype, data, dlen, 0, wfr);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to write request"));
	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeReplyL(mosiop_t iop, PhidgetNetConnHandle nc, uint16_t repseq, msgtype_t type, msgsubtype_t stype,
  const void *data, uint32_t dlen) {
	PhidgetReturnCode res;

	NetConnWriteLock(nc);
	res = writeReply(iop, nc, repseq, type, stype, data, dlen);
	NetConnWriteUnlock(nc);

	return (res);
}

PhidgetReturnCode
writeReply(mosiop_t iop, PhidgetNetConnHandle nc, uint16_t repseq, msgtype_t type, msgsubtype_t stype,
  const void *data, uint32_t dlen) {
	PhidgetReturnCode res;

	res = writeNetConn(iop, nc, NRF_REPLY, nc->reqseq++, repseq, type, stype, data, dlen, 0, NULL);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to write reply"));
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
createSalt(mosiop_t iop, char *buf, uint32_t buflen) {
	mosrandom_t *rdm;
	uint8_t rbuf[16];
	uint32_t b64len;
	uint8_t *b64;
	int err;

	err = mosrandom_alloc(iop, NULL, 0, &rdm);
	if (err != 0)
		return (EPHIDGET_UNEXPECTED);

	err = mosrandom_getbytes(rdm, iop, rbuf, sizeof(rbuf));
	mosrandom_free(&rdm);
	if (err != 0)
		return (EPHIDGET_UNEXPECTED);

	b64 = mos_base64_encode(rbuf, sizeof(rbuf), &b64len);
	mos_strlcpy(buf, (char *)b64, buflen);
	mos_free(b64, b64len);

	return (EPHIDGET_OK);
}

static void
hmac_sha256(const uint8_t *text, size_t text_len, uint8_t digest[SHA256_DIGEST_LENGTH]) {
	SHA256_CTX ctx;

	mos_SHA256_Init(&ctx);
	mos_SHA256_Update(&ctx, text, text_len);
	mos_SHA256_Final(digest, &ctx);
}

/*
 * Used by the client code to handshake with a server.
 *
 * nc must be locked.
 *
 * Handshake
 * ----------------------------------------------------
 * Client -> name, type, proto major, proto minor, ...
 * Server -> name, type, proto major, proto minor, result, ...
 *
 * Authentication
 * ----------------------------------------------------
 * Client -> identity : nonceC
 * Server -> result : nonceC : nonceS : salt : count
 * Client -> nonceC : nonceS : proof
 * Server -> result
 */
PhidgetReturnCode
startClientConnection(mosiop_t iop, PhidgetNetConnHandle nc, const char *passwd) {
	PhidgetReturnCode rres, res;
	const char *srvname;
	char allocbuf[128];
	char json[256];
	netreq_t req;
	uint32_t cnt;
	int err;

	uint8_t digest[SHA256_DIGEST_LENGTH];
	uint32_t b64len;
	uint8_t *b64;

	char challenge[128];
	const char *type;
	char nonceC[16];
	char *nonceSc;
	char *nonceS;
	char *salt;
	int count;

	if (passwd == NULL)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "missing password"));

	/*
	 * HANDSHAKE
	 */
	if (PhidgetCKFlags(nc, PNCF_DGRAM)) {
		cnt = mkJSON(json, sizeof(json), "{type=%s,pmajor=%d,pminor=%d,dgram=%d,port=%d}", nc->protocol, nc->pmajor,
		  nc->pminor, 1, ntohs(nc->dgaddr.s4.sin_port));
		res = writeRequest(iop, nc, 0, MSG_CONNECT, SMSG_HANDSHAKEC0, json, cnt, NULL);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to write client handshake request"));
	} else {
		cnt = mkJSON(json, sizeof(json), "{type=%s,pmajor=%d,pminor=%d}", nc->protocol, nc->pmajor, nc->pminor);
		res = writeRequest(iop, nc, 0, MSG_CONNECT, SMSG_HANDSHAKEC0, json, cnt, NULL);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to write client handshake request"));
	}

	res = readRequestHeader(iop, nc, &req);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to read server reply header"));
	if (req.nr_type != MSG_CONNECT || req.nr_stype != SMSG_HANDSHAKES0)
		return (MOS_ERROR(iop, EPHIDGET_INVALID, "invalid server handshake header"));

	err = parseJSON((char *)req.nr_data, req.nr_len, allocbuf, sizeof (allocbuf),
	  "%O,type=%s,pmajor=%d,pminor=%d,result=%d", &cnt, &type, &nc->ppmajor, &nc->ppminor, &res);
	if (err <= 0)
		return (MOS_ERROR(iop, res, "failed to parse server handshake"));
	if (res != 0) {
		netloginfo("server '%s' '%s' %d.%d rejected handshake", nc->peername, type, nc->ppmajor, nc->ppminor);
		if (nc->pmajor != nc->ppmajor)
			netlogerr("server protocol version '%d' does not match client version '%d'", nc->pmajor, nc->ppmajor);

		return (MOS_ERROR(iop, res, "server '%s' %d.%d rejected handshake", type, nc->ppmajor, nc->ppminor));
	}
	if (nc->pmajor != nc->ppmajor) {
		netlogerr("server protocol version '%d' does not match client version '%d'", nc->pmajor, nc->ppmajor);
		return (MOS_ERROR(iop, EPHIDGET_BADVERSION,
			"server protocol version '%d' does not match client version '%d'", nc->pmajor, nc->ppmajor));
	}
	netloginfo("server handshake '%s' %d.%d", type, nc->ppmajor, nc->ppminor);

	/*
	 * AUTHENTICATION
	 */

	res = createSalt(iop, nonceC, sizeof(nonceC));
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to generate client nonce"));

	mos_snprintf(json, sizeof(json), "{\"ident\":\"%s\",\"nonceC\":\"%s\"}", NET_IDENT, nonceC);

	netlogdebug("C=> %s", json);
	res = writeRequest(iop, nc, 0, MSG_CONNECT, SMSG_AUTHC0, json, (uint32_t)mos_strlen(json), NULL);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to write client auth 0"));

	res = readRequestHeader(iop, nc, &req);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to read server reply to client auth 0"));

	if (req.nr_len >= sizeof(json))
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "server reply to client auth 0 too long"));

	memcpy(json, req.nr_data, req.nr_len);
	json[req.nr_len] = '\0';
	netlogdebug("C<= %s", json);
	err = parseJSON(json, (uint32_t)mos_strlen(json), NULL, 0, "%O,result=%u", &cnt, &rres);
	if (err <= 0)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "Failed to parse server response to client auth 0"));

	if (rres != EPHIDGET_OK)
		return (MOS_ERROR(iop, rres, "Server rejected client auth 0"));

	err = parseJSON((char *)req.nr_data, req.nr_len, allocbuf, sizeof (allocbuf),
	  "%O,srvname=%s,nonceC=%s,nonceS=%s,salt=%s,count=%d", &cnt, &srvname, &nonceSc, &nonceS, &salt, &count);
	if (err <= 0)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "Failed to parse server response to client auth 0"));

	if (mos_strcmp(nonceSc, nonceC) != 0)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "Server response client nonce does not match"));

	mos_snprintf(challenge, sizeof(challenge), "%s%s%s%s%s", NET_IDENT, passwd, nonceC,
	  nonceS, salt);

	hmac_sha256((const uint8_t *)challenge, mos_strlen(challenge), digest);
	b64 = mos_base64_encode(digest, sizeof(digest), &b64len);

	mos_snprintf(json, sizeof(json), "{\"nonceC\":\"%s\",\"nonceS\":\"%s\",\"proof\":\"%s\"}",
	  nonceC, nonceS, b64);
	mos_free(b64, b64len);

	netlogdebug("C==> %s", json);
	res = writeRequest(iop, nc, 0, MSG_CONNECT, SMSG_AUTHC1, json, (uint32_t)mos_strlen(json), NULL);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to write client auth 1"));

	res = readRequestHeader(iop, nc, &req);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to read server reply to client auth 1"));

	netlogdebug("C<== %s", req.nr_data);
	err = parseJSON((char *)req.nr_data, req.nr_len, NULL, 0, "%O,E=%u", &cnt, &rres);
	if (err <= 0)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "failed to parse server reply to client auth 1"));
	if (rres != EPHIDGET_OK)
		return (MOS_ERROR(iop, rres, "server rejected client auth"));

	netloginfo("connected to server '%s'", srvname);
	nc->rsrvname = mos_strdup(srvname, NULL);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
serverAuthenticateClient(mosiop_t iop, const char *srvname, const char *passwd, PhidgetNetConnHandle nc) {
	PhidgetReturnCode res;
	char allocbuf[512];
	netreq_t req;
	uint32_t cnt;
	int err;

	uint8_t digest[SHA256_DIGEST_LENGTH];
	uint32_t b64len;
	uint8_t *b64;

	char challenge[128];
	char nonceS[16];
	char nonceC[16];
	char json[256];
	char salt[16];
	char *nonceC2;
	char *nonceSc;
	char *nonceCp;
	char *ident;
	char *proof;
	char *p;

	/*
	 * AUTHENTICATION
	 */
	res = readRequestHeader(iop, nc, &req);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to read client authentication request header"));
	if (req.nr_type != MSG_CONNECT || req.nr_stype != SMSG_AUTHC0)
		return (MOS_ERROR(iop, EPHIDGET_INVALID, "client did not send an auth C0 request (%s/%s)",
			strmsgtype(req.nr_type), strmsgsubtype(req.nr_stype)));

	netlogdebug("S<= %s", req.nr_data);
	err = parseJSON((char *)req.nr_data, req.nr_len, allocbuf, sizeof (allocbuf),
	  "%O,ident=%s,nonceC=%s", &cnt, &ident, &nonceCp);
	if (err <= 0)
		return (MOS_ERROR(iop, res, "failed to parse client authentication request header"));
	mos_strlcpy(nonceC, nonceCp, sizeof(nonceC));

	if (mos_strcmp(ident, NET_IDENT) != 0)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "invalid client authentication request ident"));

	res = createSalt(iop, nonceS, sizeof(nonceS));
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to generate server nonce"));
	res = createSalt(iop, salt, sizeof(salt));
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to generate server challenge salt"));

	/* do not send the '-#' from the server name */
	mos_strlcpy(challenge, srvname, sizeof(challenge));
	p = mos_strrchr(challenge, '-');
	*p = '\0';
	mos_snprintf(json, sizeof(json), "{\"srvname\":\"%s\",\"result\":0,\"nonceC\":\"%s\",\"nonceS\":\"%s\""
	  ",\"salt\":\"%s\",\"count\":1}", challenge, nonceC, nonceS, salt);

	netlogdebug("S=> %s", json);
	cnt = (uint32_t)mos_strlen(json);
	res = writeRequest(iop, nc, 0, MSG_CONNECT, SMSG_AUTHS0, json, cnt, NULL);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to write initial connection request"));

	res = readRequestHeader(iop, nc, &req);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to read client response to server auth S0"));
	if (req.nr_type != MSG_CONNECT || req.nr_stype != SMSG_AUTHC1)
		return (MOS_ERROR(iop, EPHIDGET_INVALID, "client did not send an auth C1 request (%s/%s)",
			strmsgtype(req.nr_type), strmsgsubtype(req.nr_stype)));

	netlogdebug("S<== %s", req.nr_data);
	err = parseJSON((char *)req.nr_data, req.nr_len, allocbuf, sizeof (allocbuf),
	  "%O,nonceC=%s,nonceS=%s,proof=%s", &cnt, &nonceC2, &nonceSc, &proof);
	if (err <= 0)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "failed to parse client response to server auth S0"));

	if (mos_strcmp(nonceC2, nonceC) != 0)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "client response client nonce does not match original"));

	if (mos_strcmp(nonceSc, nonceS) != 0)
		return (MOS_ERROR(iop, EPHIDGET_UNEXPECTED, "client response server nonce does not match"));

	mos_snprintf(challenge, sizeof(challenge), "%s%s%s%s%s", NET_IDENT, passwd, nonceC, nonceS, salt);
	hmac_sha256((const uint8_t *)challenge, mos_strlen(challenge), digest);
	b64 = mos_base64_encode(digest, sizeof(digest), &b64len);

	if (mos_strcmp((const char *)b64, proof) == 0) {
		res = EPHIDGET_OK;
		netloginfo("%"PRIphid" authenticated", nc);
	} else {
		res = EPHIDGET_ACCESS;
	}
	mos_free(b64, b64len);

	_sendSimpleReply(nc, req.nr_reqseq, res, NULL);
	return (res);
}

/*
 * Called by the server to handshake with a client.
 *
 * nc must be locked
 *
 * Handshake
 * ----------------------------------------------------
 * Client -> name, type, proto major, proto minor, ...
 * Server -> name, type, proto major, proto minor, result, ...
 *
 * Authentication
 * -----------------------------------------------------
 * Client -> identity, nonceC
 * Server -> result, nonceC, nonceS, salt, count
 * Client -> nonceC, nonceS, proof
 * Server -> result
 */
API_PRETURN
startServerConnection(mosiop_t iop, IPhidgetServerHandle server) {
	PhidgetReturnCode res;
	netreq_t req;
	uint32_t cnt;
	int dgram;
	int port;
	int err;

	char json[256];
	char *type;

	CHECKSTARTED;

	if (server->passwd == NULL)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "missing password"));

	/*
	 * HANDSHAKE
	 */
	res = readRequestHeader(iop, server->nc, &req);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to read client request header"));

	if (req.nr_type != MSG_CONNECT || req.nr_stype != SMSG_HANDSHAKEC0)
		return (MOS_ERROR(iop, EPHIDGET_INVALID, "client did not send a handshake request (%s/%s)",
			strmsgtype(req.nr_type), strmsgsubtype(req.nr_stype)));

	netlogdebug("S<= %s", req.nr_data);

	dgram = 0;

	/* use json buffer to store type.. type cannot be used past mkJSON() below */
	err = parseJSON((char *)req.nr_data, req.nr_len, json, sizeof (json),
	  "%O,type=%s,pmajor=%d,pminor=%d,dgram?=%d,port?=%d",
	  &cnt, &type, &server->nc->ppmajor, &server->nc->ppminor, &dgram, &port);
	if (err <= 0)
		return (MOS_ERROR(iop, res, "failed to parse client handshake json"));

	if (dgram) {
		PhidgetSetFlags(server->nc, PNCF_DGRAM);
		server->nc->dgaddr.s4.sin_family = server->nc->addr.s4.sin_family;
		server->nc->dgaddr.s4.sin_addr = server->nc->addr.s4.sin_addr;
		server->nc->dgaddr.s4.sin_port = htons(port);
	}

	if (server->nc->conntypestr != NULL)
		mos_free(server->nc->conntypestr, MOSM_FSTR);
	server->nc->conntypestr = mos_strdup(type, NULL);

	if (mos_strncmp(type, "www", 3) == 0)
		PhidgetSetFlags(server->nc, PNCF_SENDCHANNELS);

	if (server->nc->pmajor != server->nc->ppmajor) {
		netlogerr("'%s' client indicated unsupported protocol version %d", type, server->nc->ppmajor);
		cnt = mkJSON(json, sizeof(json), "{type=%s,pmajor=%d,pminor=%d,result=%d}",
		  server->nc->protocol, server->nc->pmajor, server->nc->pminor, EPHIDGET_UNSUPPORTED);
		writeRequest(iop, server->nc, 0, MSG_CONNECT, SMSG_HANDSHAKES0, json, cnt, NULL);
		return (MOS_ERROR(iop, EPHIDGET_BADVERSION,
			"'%s' client indicated unsupported protocol version %d", type, server->nc->ppmajor));
	}

	cnt = mkJSON(json, sizeof(json), "{type=%s,pmajor=%d,pminor=%d,result=%d}",
	  server->nc->protocol, server->nc->pmajor, server->nc->pminor, 0);

	res = writeRequest(iop, server->nc, 0, MSG_CONNECT, SMSG_HANDSHAKES0, json, cnt, NULL);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to write handshake reply S0"));

	res = serverAuthenticateClient(iop, server->name, server->passwd, server->nc);
	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to authenticate client"));

	return (0);
}

API_VRETURN
stopPhidgetNetConn(PhidgetNetConnHandle nc) {

	PhidgetSetFlags(nc, PNCF_STOP);
	mos_netop_tcp_closesocket(MOS_IOP_IGNORE, &nc->sock);
}

API_VRETURN
waitForPhidgetNetConnThread(PhidgetNetConnHandle nc) {

	PhidgetLock(nc);
	while (nc->__flags & PNCF_HASTHREAD)
		PhidgetWait(nc);
	PhidgetUnlock(nc);
}

API_VRETURN
stopAndWaitForPhidgetNetConnThread(PhidgetNetConnHandle nc) {

	stopPhidgetNetConn(nc);
	waitForPhidgetNetConnThread(nc);
}

void
PhidgetNetConnClose(PhidgetNetConnHandle nc) {
	WaitForReply *wfr1, *wfr2;
	PhidgetReturnCode res;

	NetConnWriteLock(nc);
	res = PhidgetCKandSetFlags(nc, PNCF_CLOSED);
	NetConnWriteUnlock(nc);

	if (res != EPHIDGET_OK)
		return;

	if (nc->close)
		nc->close(nc);

	netConnDetachDevice(nc);
	if (nc->server == NULL)
		clientClosed(nc);

#if ZEROCONF_SUPPORT
	if (nc->pubhandle)
		Zeroconf_unpublish(&nc->pubhandle);
#endif

	if (nc->sock != MOS_INVALID_SOCKET) {
		mos_netop_tcp_closesocket(MOS_IOP_IGNORE, &nc->sock);
		nc->sock = MOS_INVALID_SOCKET;
	}

	if (nc->dgsock != MOS_INVALID_SOCKET) {
		mos_netop_udp_closesocket(MOS_IOP_IGNORE, &nc->dgsock);
		nc->dgsock = MOS_INVALID_SOCKET;
	}

	wfr1 = MTAILQ_FIRST(&nc->waitforreply);
	while (wfr1 != NULL) {
		wfr2 = MTAILQ_NEXT(wfr1, link);
		closeWaitForReply(&wfr1);
		wfr1 = wfr2;
	}
}

static void
PhidgetNetConnDelete(PhidgetNetConnHandle *_nc) {
	PhidgetNetConnHandle nc;

	if (_nc == NULL)
		return;

	nc = *_nc;
	*_nc = NULL;

	/* ensure the connection has been closed properly */
	MOS_ASSERT(nc->__flags & PNCF_CLOSED);

	if (nc->peername) {
		mos_free(nc->peername, MOSM_FSTR);
		nc->peername = NULL;
	}
	if (nc->rsrvname) {
		mos_free(nc->rsrvname, MOSM_FSTR);
		nc->rsrvname = NULL;
	}
	if (nc->protocol) {
		mos_free(nc->protocol, MOSM_FSTR);
		nc->protocol = NULL;
	}

	if (nc->conntypestr) {
		mos_free(nc->conntypestr, MOSM_FSTR);
		nc->conntypestr= NULL;
	}

	mos_free(nc->tokens, sizeof (pjsmntok_t) * BRIDGE_JSON_TOKENS);
	mos_free(nc->iobuf, NR_HEADERLEN + NR_MAXDATALEN);

	/*
	 * This call is expected to free the memory.. if it is to be free'd
	 * Some implementations may allocate from a pool.
	 */
	nc->release(&nc);
}

API_PRETURN
createPhidgetNetConn(IPhidgetServerHandle server, PhidgetNetConnHandle *nc) {
	PhidgetReturnCode res;

	res = PhidgetNetConnCreate(nc);
	if (res != EPHIDGET_OK)
		return (res);

	(*nc)->conntype = PHIDGETCONN_LOCAL;
	(*nc)->sock = MOS_INVALID_SOCKET;
	(*nc)->server = server;

	/* defaults */
	(*nc)->read = deviceread;
	(*nc)->write = devicewrite;
	(*nc)->release = devicerelease;

	if (server)
		server->initNetConn(server, *nc);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetNetConnCreate(PhidgetNetConnHandle *nc) {

	TESTPTR(nc);

	*nc = mos_zalloc(sizeof(PhidgetNetConn));
	phidget_init((PhidgetHandle)*nc, PHIDGET_NETCONN, (PhidgetDelete_t)PhidgetNetConnDelete);
	(*nc)->__flags |= PHIDGET_ATTACHED_FLAG;	/* netconn is always attached */
	(*nc)->tokens = mos_malloc(sizeof (pjsmntok_t) * BRIDGE_JSON_TOKENS);
	(*nc)->iobuf = mos_malloc(NR_HEADERLEN + NR_MAXDATALEN);
	(*nc)->hdrbuf = (*nc)->iobuf;
	(*nc)->hdrbufsz = NR_HEADERLEN;
	(*nc)->databuf = (*nc)->iobuf + NR_HEADERLEN;
	(*nc)->databufsz = NR_MAXDATALEN;
	(*nc)->dgsock = MOS_INVALID_SOCKET;
	mostimestamp_localnow(&(*nc)->ctime);

	return (EPHIDGET_OK);
}

void
NetConnWriteLock(PhidgetNetConnHandle nc) {

	PhidgetRunLock(nc);
}

void
NetConnWriteUnlock(PhidgetNetConnHandle nc) {

	PhidgetRunUnlock(nc);
}

static void
PhidgetNetworkConnectionDelete(PhidgetNetworkConnectionHandle *conn) {

	if ((*conn)->nc)
		PhidgetRelease(&(*conn)->nc);

	mos_free(*conn, sizeof(PhidgetNetworkConnection));
}

PhidgetReturnCode
PhidgetNetworkConnectionCreate(PhidgetNetworkConnectionHandle *conn) {

	assert(conn);

	*conn = mos_zalloc(sizeof(PhidgetNetworkConnection));
	phidget_init((PhidgetHandle)*conn, PHIDGET_NET_CONNECTION, (PhidgetDelete_t)PhidgetNetworkConnectionDelete);

	return (EPHIDGET_OK);
}

API_PRETURN
netConnWrite(mosiop_t iop, PhidgetNetConnHandle nc, const void *v, size_t n) {

	return (mos_netop_tcp_writefully(iop, &nc->sock, v, n));
}

API_PRETURN
netConnRead(mosiop_t iop, PhidgetNetConnHandle nc, void *v, size_t *n) {

	return (mos_netop_tcp_readfully(iop, &nc->sock, v, n));
}

API_PRETURN
netConnReadLine(mosiop_t iop, PhidgetNetConnHandle nc, void *v, size_t *n) {

	return (mos_net_readline(iop, &nc->sock, v, n));
}

API_PRETURN
setNetConnConnTypeStr(PhidgetNetConnHandle nc, const char *str) {

	if (nc->conntypestr)
		mos_free(nc->conntypestr, MOSM_FSTR);
	nc->conntypestr = mos_strdup(str, NULL);

	return (EPHIDGET_OK);
}

API_VPRETURN
getNetConnPrivate(PhidgetNetConnHandle nc) {

	return (nc->private);
}

API_VRETURN
setNetConnPrivate(PhidgetNetConnHandle nc, void *v) {

	nc->private = v;
}

API_CRETURN
getNetConnPeerName(PhidgetNetConnHandle nc) {

	return (nc->peername);
}

API_PRETURN
setNetConnProtocol(PhidgetNetConnHandle nc, const char *proto, int major, int minor) {

	if (proto)
		nc->protocol = mos_strdup(proto, NULL);
	else
		nc->protocol = mos_strdup(PHIDGET_NET_PROTOCOL, NULL);

	if (major)
		nc->pmajor = major;
	else
		nc->pmajor = PHIDGET_NET_PROTOCOL_MAJOR;

	if (minor)
		nc->pminor = minor;
	else
		nc->pminor = PHIDGET_NET_PROTOCOL_MINOR;

	return (EPHIDGET_OK);
}

API_PRETURN
setNetConnHandlers(PhidgetNetConnHandle nc, NetConnClose cl, NetConnRelease rl, NetConnWrite wr,
  NetConnRead rd) {

	if (cl)
		nc->close = cl;
	if (rl)
		nc->release = rl;
	if (wr)
		nc->write = wr;
	if (rd)
		nc->read = rd;

	return (EPHIDGET_OK);
}

API_PRETURN
setNetConnConnectionTypeListener(PhidgetNetConnHandle nc) {

	nc->conntype = PHIDGETCONN_LISTENER;
	return (EPHIDGET_OK);
}

API_PRETURN
setNetConnConnectionTypeLocal(PhidgetNetConnHandle nc) {

	nc->conntype = PHIDGETCONN_LOCAL;
	return (EPHIDGET_OK);
}

PhidgetServerHandle CCONV
getPhidgetServerHandle(IPhidgetServerHandle isrv) {

		return (&isrv->info);
}

PhidgetNetConnHandle CCONV
getIPhidgetServerNetConn(IPhidgetServerHandle isrv) {

	return (isrv->nc);
}


const char *
strmsgtype(msgtype_t type) {

	switch (type) {
	case MSG_CONNECT:
		return ("MSG_CONNECT");
	case MSG_COMMAND:
		return ("MSG_COMMAND");
	case MSG_DEVICE:
		return ("MSG_DEVICE");
	default:
		return ("<UNKNOWN>");
	}
}

const char *
strmsgsubtype(msgsubtype_t type) {

	switch (type) {
	case SMSG_CLOSECONN:
		return ("SMSG_CLOSECONN");
	case SMSG_HANDSHAKEC0:
		return ("SMSG_HANDSHAKEC0");
	case SMSG_HANDSHAKES0:
		return ("SMSG_HANDSHAKES0");
	case SMSG_DGRAMSTART:
		return ("SMSG_DGRAMSTART");
	case SMSG_DGRAMSTARTOK:
		return ("SMSG_DGRAMSTARTOK");
	case SMSG_AUTHC0:
		return ("SMSG_AUTHC0");
	case SMSG_AUTHS0:
		return ("SMSG_AUTHS0");
	case SMSG_AUTHC1:
		return ("SMSG_AUTHC1");
	case SMSG_AUTHS1:
		return ("SMSG_AUTHS1");
	case SMSG_REPLY:
		return ("SMSG_REPLY");
	case SMSG_KEEPALIVE:
		return ("SMSG_KEEPALIVE");
	case SMSG_DEVATTACH:
		return ("SMSG_DEVATTACH");
	case SMSG_DEVDETACH:
		return ("SMSG_DEVDETACH");
	case SMSG_DEVOPEN:
		return ("SMSG_DEVOPEN");
	case SMSG_DEVCLOSE:
		return ("SMSG_DEVCLOSE");
	case SMSG_DEVBRIDGEPKT:
		return ("SMSG_DEVBRIDGEPKT");
	case SMSG_DEVCHANNEL:
		return ("SMSG_DEVCHANNEL");
	default:
		return ("<UNKNOWN>");
	}
}
