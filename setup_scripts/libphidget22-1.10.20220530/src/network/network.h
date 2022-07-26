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

#ifndef _PHIDGET_NETWORK_H_
#define _PHIDGET_NETWORK_H_

typedef struct {
	const char			*name;			/* The name of the server */
	const char			*stype;			/* The type name of the server */
	PhidgetServerType	type;			/* The type of server */
	int					flags;			/* Informational flags about the server */
	const char			*addr;
	const char			*host;
	int					port;
	const void			*handle;
} PhidgetServer, *PhidgetServerHandle;

/* Client API */

API_PRETURN_HDR PhidgetNet_addServer(const char *serverName, const char *address, int port, const char *password, int flags);
API_PRETURN_HDR PhidgetNet_removeServer(const char *serverName);
API_PRETURN_HDR PhidgetNet_removeAllServers(void);

API_PRETURN_HDR PhidgetNet_enableServer(const char *serverName);
API_PRETURN_HDR PhidgetNet_disableServer(const char *serverName, int flags);

API_PRETURN_HDR PhidgetNet_setServerPassword(const char *serverName, const char *password);

API_PRETURN_HDR PhidgetNet_enableServerDiscovery(PhidgetServerType serverType);
API_PRETURN_HDR PhidgetNet_disableServerDiscovery(PhidgetServerType serverType);

typedef void (CCONV *PhidgetNet_OnServerAddedCallback)(void *ctx, PhidgetServerHandle server, void *kv);
typedef void (CCONV *PhidgetNet_OnServerRemovedCallback)(void *ctx, PhidgetServerHandle server);

API_PRETURN_HDR PhidgetNet_setOnServerAddedHandler(PhidgetNet_OnServerAddedCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetNet_setOnServerRemovedHandler(PhidgetNet_OnServerRemovedCallback fptr, void *ctx);

API_PRETURN_HDR PhidgetNet_getServerAddressList(const char *hostname, int addressFamily, char *addressList[], uint32_t *count);
API_PRETURN_HDR PhidgetNet_freeServerAddressList(char *addressList[], uint32_t count);
/* Server API */

API_PRETURN_HDR PhidgetNet_startServer(int flags, int addressFamily, const char *serverName, const char *address, int port,
  const char *password, PhidgetServerHandle *server);
API_PRETURN_HDR PhidgetNet_stopServer(PhidgetServerHandle *server);

#ifdef INCLUDE_PRIVATE

API_PRETURN_HDR PhidgetNet_setProperty(const char *key, const char *property, ...);
API_PRETURN_HDR PhidgetNet_setPropertyv(const char *key, const char *property, va_list);

#endif

#ifndef EXTERNALPROTO
/******************************************************************************************************/

#define NETLS "phidget22net"

#ifdef NDEBUG
#define netlogdebug(...)
#define netlogcrit(...) PhidgetLog_loge(NULL, 0, __func__, NETLS, PHIDGET_LOG_CRITICAL, __VA_ARGS__)
#define netlogerr(...) PhidgetLog_loge(NULL, 0, __func__, NETLS, PHIDGET_LOG_ERROR, __VA_ARGS__)
#define netlogwarn(...) PhidgetLog_loge(NULL, 0, __func__, NETLS, PHIDGET_LOG_WARNING, __VA_ARGS__)
#define netloginfo(...) PhidgetLog_loge(NULL, 0, __func__, NETLS, PHIDGET_LOG_INFO, __VA_ARGS__)
#define netlogverbose(...)
#else
#define netlogcrit(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETLS, PHIDGET_LOG_CRITICAL, __VA_ARGS__)
#define netlogerr(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETLS, PHIDGET_LOG_ERROR, __VA_ARGS__)
#define netlogwarn(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETLS, PHIDGET_LOG_WARNING, __VA_ARGS__)
#define netloginfo(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETLS, PHIDGET_LOG_INFO, __VA_ARGS__)
#define netlogdebug(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETLS, PHIDGET_LOG_DEBUG, __VA_ARGS__)
#define netlogverbose(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETLS, PHIDGET_LOG_VERBOSE, __VA_ARGS__)
#endif /* NDEBUG */

#include "locks.h"
#include "network/zeroconf.h"
#include "mos/mos_netops.h"
#include "mos/bsdqueue.h"
#include "mos/bsdtree.h"
#include "mos/mos_time.h"
#include "util/jsmn.h"
#include "util/phidgetconfig.h"

#define PHIDGET_NETWORK_MDNS_DEVICE		"_phidget22server._tcp"
#define PHIDGET_NETWORK_MDNS_WWW		"_phidget_www._tcp"		/* also in network server */
#define PHIDGET_NETWORK_MDNS_SBC		"_phidget_sbc._tcp"

/* Mask must be within PHIDGET_OBJFLAG_MASK */
#define PHIDGET_NETWORK_FLAGMASK	0xF0000000
#define PHIDGET_NETWORK_PUBLISHMDNS	0x10000000
#define PHIDGET_NETWORK_MDNSDETACH	0x20000000
#define PHIDGETSERVERTYPE_CNT 11

typedef enum {
	PHIDGETCONN_NONE = 0,				/* placeholder */
	PHIDGETCONN_LISTENER,				/* listener socket */
	PHIDGETCONN_LOCAL,					/* server side connection */
	PHIDGETCONN_REMOTE,					/* client side connection */
} PhidgetConnType;

/* External server definition */
#define PHIDGET_SERVER_FROMMDNS	0x01

typedef struct _PhidgetMDNSPublish {
	void *handle;
} PhidgetMDNSPublish, *PhidgetMDNSPublishHandle;

struct _netreq {
	uint32_t 	magic;
	uint32_t	len;
	uint16_t	flags;
	uint16_t	reqseq;
	uint16_t	repseq;
	uint8_t		type;
	uint8_t		stype;
};

#define NR_HEADERLEN	16
#define NR_MAXDATALEN	(128 * 1024)
#define NR_MAXDGDATALEN	(500)
#define NR_HEADERMAGIC	0x50484930
#define NRF_REQUEST		0x0001	/* payload is a request: reply is expected */
#define NRF_REPLY		0x0002	/* payload is a reply */
#define NRF_EVENT		0x0004	/* payload is an event: no reply expected */
#define NRF_USER1		0x0100	/* user flag 1 */
#define NRF_USER2		0x0200	/* user flag 2 */
#define NRF_USER3		0x0400	/* user flag 3 */
#define NRF_USER4		0x0800	/* user flag 4 */
#define NRF_USERMASK	0x0F00

#define NRFDICT_PERSIST	NRF_USER1	/* dictionary persist flag */

typedef union netreqhdr {
	uint8_t			_nr_buf[NR_HEADERLEN];
	struct _netreq	_nr_req;
} netreqhdr_t;

typedef struct netreq {
	netreqhdr_t		nr_hdr;
	uint8_t			nr_data[NR_MAXDATALEN];

#define nr_buf		nr_hdr._nr_buf
#define nr_magic	nr_hdr._nr_req.magic
#define nr_len		nr_hdr._nr_req.len
#define nr_flags	nr_hdr._nr_req.flags
#define nr_reqseq	nr_hdr._nr_req.reqseq
#define nr_repseq	nr_hdr._nr_req.repseq
#define nr_type		nr_hdr._nr_req.type
#define nr_stype	nr_hdr._nr_req.stype
} netreq_t;

#define WFR_WAITING		0x01
#define WFR_CANCELLED	0x02
#define WFR_ONLIST		0x04
#define WFR_RECEIVED	0x10
#define WFR_ERROR		0x20
#define WFR_TIMEDOUT	0x40

typedef struct _PhidgetNetConn *PhidgetNetConnHandle;

#define WFR_WAITTIME		5000	/* 5 seconds default */

typedef struct _WaitForReply {
	int					flags;
	mostime_t			waittime;
	mos_tlock_t			*lock;
	mos_cond_t			cond;
	netreq_t			req;
	PhidgetReturnCode	res;
	PhidgetNetConnHandle nc;
	MTAILQ_ENTRY(_WaitForReply)	link;
} WaitForReply;

typedef MTAILQ_HEAD(waitforreplylist, _WaitForReply) waitforreplylist_t;

typedef PhidgetReturnCode(CCONV *NetConnWrite)(mosiop_t, PhidgetNetConnHandle, const void *, uint32_t);
typedef PhidgetReturnCode(CCONV *NetConnRead)(mosiop_t, PhidgetNetConnHandle, void *, uint32_t *);
typedef void(CCONV *NetConnClose)(PhidgetNetConnHandle);
typedef void(CCONV *NetConnRelease)(PhidgetNetConnHandle *);

typedef struct _IPhidgetServer *IPhidgetServerHandle;

typedef void (CCONV *initPhidgetNetConn_t)(IPhidgetServerHandle, PhidgetNetConnHandle);
typedef PhidgetReturnCode (CCONV *handlePhidgetNetConn_t)(mosiop_t, IPhidgetServerHandle);
typedef PhidgetReturnCode (CCONV *handleRequest_t)(mosiop_t, PhidgetNetConnHandle, netreq_t *, int *);

/* Internal server definition */

extern uint32_t network_keepalive;

typedef struct _IPhidgetServer {
	PhidgetServer			info;
	IPhidgetServerHandle	this;			/* non-const pointer to ourself */
	uint32_t				conncnt;		/* count of total client connections */
	mos_task_t				self;			/* active thread */
	int						state;			/* thread state */
	int						flags;
	char					*name;			/* real name, info.name points to this */
	char					*passwd;
	int						af;
	char					*hostname;
	char					*address;
	int						port;
	PhidgetNetConnHandle	nc;
	mos_tlock_t				*lock;
	mos_cond_t				cond;
	initPhidgetNetConn_t	initNetConn;	/* called to initalize per server type network connections */
	handlePhidgetNetConn_t	handleClient;	/* called to handle a client connection to the server */
	RB_ENTRY(_IPhidgetServer) link;
} IPhidgetServer;

/* Must be in the PHIDGET_OBJFLAG_MASK range */
#define PNCF_STOP			0x01000000			/* flag that the connection is closing */
#define PNCF_HASTHREAD		0x02000000			/* A thread is running for this connection */
#define PNCF_SENDCHANNELS	0x04000000			/* The connection wants channel events */
#define PNCF_CLOSED			0x08000000			/* The connection has been closed */
#define PNCF_DGRAM			0x10000000			/* DGRAM communication supported by client */
#define PNCF_DGRAMENABLED	0x20000000			/* DGRAM communication enabled on connection */

typedef struct _PhidgetNetConn {
	PHIDGET_STRUCT_START				/* Net connections are phidgets */
	IPhidgetServerHandle server;		/* Internal server handle */
	PhidgetConnType		conntype;		/* connection type */
	char				*protocol;		/* protocol type */
	int					pmajor;			/* protocol major */
	int					pminor;			/* protocol minor */
	int					ppmajor;		/* peer protocol major */
	int					ppminor;		/* peer protocol minor */
	char				*peername;		/* the address and port of the peer */
	char				*rsrvname;		/* remote server name for REMOTE connections */
	char				*conntypestr;	/* connection type string */
	mos_task_t			rself;			/* task handle for REMOTE connections */
	mos_sockaddr_t		addr;			/* bound/connected address */
	mos_socket_t		sock;			/* socket */
	mos_sockaddr_t		dgaddr;			/* datagram address */
	mos_socket_t		dgsock;			/* datagram socket */
	uint64_t			dgseq;			/* datagram sequence number */
	PhidgetReturnCode	errcondition;	/* if the connection is in an error state, and is closing */
	mostime_t			keepalive;		/* keepalive interval (usec) -- 0 is off */
	mostime_t			keepalive_last;	/* last time a keepalive was sent */
	mostime_t			keepalive_dl;	/* deadline to receive keepalive reply */
	uint16_t			reqseq;			/* request sequence number; protected by wrlock */
	waitforreplylist_t	waitforreply;	/* list of threads waiting for a reply; protected by wrlock */
#if ZEROCONF_SUPPORT
	ZeroconfPublishHandle pubhandle;	/* zeroconf publish handle for the main server thread */
#endif
	NetConnClose		close;			/* called just before (as) the connection is closed */
	NetConnWrite		write;			/* write handler for connection */
	NetConnRead			read;			/* read handler for connection */
	NetConnRelease		release;		/* called after the connection has been closed */
	handleRequest_t		handleRequest;	/* called to handle an incoming request on the connection */
	void				*private;		/* private pointer for extending code */
	mostimestamp_t		ctime;			/* creation time */
	uint32_t			openchannels;	/* open channel count */
	uint64_t			io_in;			/* bytes read */
	uint64_t			io_out;			/* bytes send */
	uint64_t			io_ev;			/* events sent */
	void				*tokens;		/* json parser tokens */
	uint32_t			hdrbufsz;		/* size of header buffer */
	char				*hdrbuf;		/* header buffer */
	uint32_t			databufsz;		/* data buffer size */
	char				*databuf;		/* data buffer */
	char				*iobuf;			/* full IO buffer */
	MTAILQ_ENTRY(_PhidgetNetConn)	openlink;	/* linkage for server events */
} PhidgetNetConn;

typedef MTAILQ_HEAD(phidgetnetconnlist, _PhidgetNetConn) phidgetnetconnlist_t;

/*
 * This structure exists in each network attached device.
 * 'id' is the address of the device on the server
 * 'conn' is the network connection the device is attached over
 */
typedef struct _PhidgetNetworkConnection {
	PHIDGET_STRUCT_START
	PhidgetNetConnHandle	nc;
	uint64_t				id;
} PhidgetNetworkConnection, *PhidgetNetworkConnectionHandle;

/*
 * Server Side Events/API
 */
PhidgetReturnCode sendNetDeviceAttached(PhidgetDeviceHandle, PhidgetNetConnHandle);
PhidgetReturnCode sendNetDeviceDetached(PhidgetDeviceHandle);
PhidgetReturnCode sendToNetworkConnections(PhidgetChannelHandle, BridgePacket *, PhidgetNetConnHandle);

/* initializes locks etc. */
void PhidgetNetInit(void);
void PhidgetNetFini(void);
int PhidgetNet_isStarted(void);

/* starts threads etc. */
void PhidgetNet_start(void);
void PhidgetNet_stop(void);

/*
 * Client Side API
 */
PhidgetReturnCode openNetworkChannel(PhidgetChannelHandle, PhidgetDeviceHandle, int index, char **reply);
PhidgetReturnCode closeNetworkChannel(PhidgetChannelHandle);

/*
 * Support Functions
 */
void removeChannelNetworkConnections(PhidgetChannelHandle);

PhidgetNetworkConnectionHandle PhidgetNetworkConnectionCast(void *);
PhidgetReturnCode PhidgetNetworkConnectionCreate(PhidgetNetworkConnectionHandle *);
PhidgetNetConnHandle PhidgetNetConnCast(void *);
PhidgetReturnCode PhidgetNetConnCreate(PhidgetNetConnHandle *);
const char *netConnInfo(PhidgetNetConnHandle, char *, uint32_t);

const char *getNetworkControlEntryName(void *nce);

/*
 * INTERNAL CODE AFTER HERE
 */
#ifdef _PHIDGET_NETWORKCODE

#include "mos/mos_os.h"
#include "mos/mos_netops.h"
#include "mos/mos_task.h"
#include "mos/mos_time.h"

#define WRLOCK(netInfo)		mos_mutex_lock(&(netInfo)->conn->wrlock)
#define WRUNLOCK(netInfo)	mos_mutex_unlock(&(netInfo)->conn->wrlock)

#define DATAGRAM_DENY	0
#define DATAGRAM_ALLOW	1
#define DATAGRAM_FORCE	2

#define MAX_SERVERS	4
#define MAX_CLIENTS	64

#define TS_FREE		0	/* Task structure is free: for when we cache structures */
#define TS_RUN		1	/* Task should run */
#define TS_RUNNING	2	/* Task is running */
#define TS_STOP		3	/* Task should stop */
#define TS_STOPPED	4	/* Task has stopped */

// XXX unused?
//#define PHIDGET_NET_SRVNAME			"phidget22server"
//#define PHIDGET_NET_SRVTYPE			"device server"
//#define PHIDGET_NET_CLINAME			"phidget22client"
//#define PHIDGET_NET_CLITYPE			"heavy client"

#define PHIDGET_NET_PROTOCOL		"phid22device"

/*
 * 1.1 - added keepalive support.
 * 2.0 - changed bridge packet ID numbers. Method packets are not forwarded to all clients.
 * 2.1 - added fwstr to SMSG_DEVATTACH packet
 * 2.2 - stop requiring class version to match on channel open
 * 2.3 - support VINT2 hubs - extra fields in SMSG_DEVATTACH, etc.
 */
#define PHIDGET_NET_PROTOCOL_MAJOR	2
#define PHIDGET_NET_PROTOCOL_MINOR	3

#define PHIDGET_MDNS_TXTVER			1

/*
 * 8 bit type
 */
typedef enum msgtype {
	MSG_CONNECT		= 10,	/* authentication */
	MSG_COMMAND		= 20,	/* requests */
	MSG_DEVICE 		= 30,	/* device/channel */
} msgtype_t;

/*
 * 8 bit subtype
 */
typedef enum msgsubtype {
	SMSG_CLOSECONN		= 1,
	/* MSG_CONNECT */
	SMSG_HANDSHAKEC0	= 10,
	SMSG_HANDSHAKES0	= 11,
	SMSG_DGRAMSTART		= 20,	/* datagram events start */
	SMSG_DGRAMSTARTOK	= 21,	/* datagram events start reply */
	SMSG_AUTHC0			= 30,	/* authentication states */
	SMSG_AUTHS0			= 31,
	SMSG_AUTHC1			= 32,
	SMSG_AUTHS1			= 33,
	/* MSG_COMMAND */
	SMSG_REPLY			= 40,	/* reply to a request */
	SMSG_KEEPALIVE		= 41,	/* keep alive request */
	/* MSG_DEVICE */
	SMSG_DEVATTACH		= 50,	/* device attached at source */
	SMSG_DEVDETACH		= 55,	/* device detached from source */
	SMSG_DEVOPEN		= 60,	/* device open request */
	SMSG_DEVCLOSE		= 65,	/* device close request */
	SMSG_DEVBRIDGEPKT	= 70,
	SMSG_DEVCHANNEL		= 80,	/* channel info for light clients */
} msgsubtype_t;

#define PHIDID(phid)	((uint64_t)(uintptr_t)(phid))

void NetworkControlInit(void);
void NetworkControlFini(void);
void ServersInit(void);
void ServersFini(void);
void ServerInit(void);
void ServerFini(void);

void NetworkControlStart(void);
void NetworkControlStop(void);
void ServersStart(void);
void ServersStop(void);
void ServerStart(void);
void ServerStop(void);

#if ZEROCONF_SUPPORT
void ZeroconfInit(void);
void ZeroconfFini(void);
void ZeroconfStart(void);
void ZeroconfStop(void);
void deviceServerAdded(const char *name, const char *host, mos_sockaddr_list_t *addrlist, int port, kv_t *txt);
#endif

PhidgetReturnCode openServerChannel(mosiop_t iop, uint64_t, Phidget_ChannelClass, int, PhidgetNetConnHandle,
  PhidgetChannelHandle *, uint16_t);
PhidgetReturnCode closeServerChannel(uint64_t, int, PhidgetNetConnHandle);

PhidgetReturnCode addChannelNetworkConnection(PhidgetChannelHandle, PhidgetNetConnHandle, uint16_t);
PhidgetReturnCode removeChannelNetworkConnection(PhidgetChannelHandle, PhidgetNetConnHandle, int *);
PhidgetReturnCode channelDeliverBridgePacket(PhidgetChannelHandle, BridgePacket *, PhidgetNetConnHandle, int);

PhidgetReturnCode openWaitForReply(uint16_t, PhidgetNetConnHandle, WaitForReply **);
PhidgetReturnCode waitForReply(WaitForReply *);
void closeWaitForReply(WaitForReply **);
void cancelWaitForReply(WaitForReply *);
PhidgetReturnCode handleReply(PhidgetNetConnHandle, netreq_t *);

PhidgetReturnCode simpleWaitForReply(WaitForReply **, PhidgetReturnCode *, char **, uint8_t **, size_t *);

#if ZEROCONF_SUPPORT
PhidgetReturnCode PhidgetNet_discoveredServer(ZeroconfListenerHandle, int, int, PhidgetServerType,
  const char *, const char *, const char *, const char *, int);
PhidgetReturnCode PhidgetNet_undiscoveredServer(const char *);
#endif

PhidgetReturnCode startNetworkServerConnection(mosiop_t, mos_socket_t, mos_sockaddr_t, PhidgetNetConnHandle);
PhidgetReturnCode startClientConnection(mosiop_t, PhidgetNetConnHandle, const char *);
void clientAuthFailed(PhidgetNetConnHandle);
void clientProtocolFailure(PhidgetNetConnHandle);
void clientClosed(PhidgetNetConnHandle);
void PhidgetNetConnClose(PhidgetNetConnHandle);
void NetConnWriteLock(PhidgetNetConnHandle);
void NetConnWriteUnlock(PhidgetNetConnHandle);

PhidgetReturnCode deviceread(mosiop_t, PhidgetNetConnHandle, void *, uint32_t *);

PhidgetReturnCode devicewrite(mosiop_t, PhidgetNetConnHandle, const void *, uint32_t);

PhidgetReturnCode readDGRequestHeader(mosiop_t, PhidgetNetConnHandle, netreq_t *);
PhidgetReturnCode readRequestHeader(mosiop_t, PhidgetNetConnHandle, netreq_t *);

PhidgetReturnCode writeRequest(mosiop_t, PhidgetNetConnHandle, int, msgtype_t, msgsubtype_t, const void *,
  uint32_t, WaitForReply **);
PhidgetReturnCode writeReplyL(mosiop_t, PhidgetNetConnHandle, uint16_t, msgtype_t, msgsubtype_t,
  const void *data, uint32_t);
PhidgetReturnCode writeReply(mosiop_t, PhidgetNetConnHandle, uint16_t, msgtype_t, msgsubtype_t,
  const void *data, uint32_t);
PhidgetReturnCode writeEvent(mosiop_t, PhidgetNetConnHandle, msgtype_t, msgsubtype_t, const void *,
  uint32_t, int);

PhidgetReturnCode sendSimpleReply(PhidgetNetConnHandle, uint16_t, PhidgetReturnCode, const char *);
PhidgetReturnCode sendReply(PhidgetNetConnHandle nc, uint16_t repseq, PhidgetReturnCode rcode, const void *data, uint32_t dlen);

PhidgetReturnCode dispatchClientBridgePacket(mosiop_t, PhidgetNetConnHandle, BridgePacket *, int, int);
PhidgetReturnCode dispatchServerBridgePacket(mosiop_t, PhidgetNetConnHandle, BridgePacket *, int, int);
PhidgetReturnCode initDispatch(void);

PhidgetReturnCode clientConnect(mos_af_t, const char *, int, const char *, const char *, int, int,
  handleRequest_t, void *, PhidgetNetConnHandle *);
PhidgetReturnCode handleDeviceClientRequest(mosiop_t, PhidgetNetConnHandle, netreq_t *, int *);
#if ZEROCONF_SUPPORT
void releaseMDNSControlEntries(ZeroconfListenerHandle);
#endif
int validServerName(const char *);
PhidgetReturnCode startKeepAliveTask(PhidgetNetConnHandle);
PhidgetReturnCode netConnToPConf(PhidgetNetConnHandle, pconf_t **);
PhidgetReturnCode openServersToPConf(pconf_t **);
const char *strmsgtype(msgtype_t type);
const char *strmsgsubtype(msgsubtype_t type);


/* Exported hidden */

API_PRETURN_HDR pnwrite(mosiop_t, PhidgetNetConnHandle, const void *, uint32_t);
API_PRETURN_HDR pnread(mosiop_t, PhidgetNetConnHandle, void *, uint32_t *);

API_PRETURN_HDR setNetConnConnTypeStr(PhidgetNetConnHandle, const char *);
API_CRETURN_HDR getNetConnPeerName(PhidgetNetConnHandle);
API_VRETURN_HDR setNetConnPrivate(PhidgetNetConnHandle, void *);
API_VPRETURN_HDR getNetConnPrivate(PhidgetNetConnHandle);
API_PRETURN_HDR setNetConnProtocol(PhidgetNetConnHandle, const char *, int, int);
API_PRETURN_HDR setNetConnHandlers(PhidgetNetConnHandle, NetConnClose, NetConnRelease, NetConnWrite,
  NetConnRead);
API_PRETURN_HDR setNetConnConnectionTypeListener(PhidgetNetConnHandle);
API_PRETURN_HDR setNetConnConnectionTypeLocal(PhidgetNetConnHandle);

API_PRETURN_HDR netConnWrite(mosiop_t, PhidgetNetConnHandle, const void *, size_t);
API_PRETURN_HDR netConnRead(mosiop_t, PhidgetNetConnHandle, void *, size_t *);
API_PRETURN_HDR netConnReadLine(mosiop_t, PhidgetNetConnHandle, void *, size_t *);

PhidgetServerHandle CCONV getPhidgetServerHandle(IPhidgetServerHandle);
PhidgetNetConnHandle CCONV getIPhidgetServerNetConn(IPhidgetServerHandle);

/* device server handlers */
API_PRETURN_HDR handleDeviceRequest(mosiop_t, PhidgetNetConnHandle, netreq_t *, int *);
API_PRETURN_HDR handleDeviceClient(mosiop_t, IPhidgetServerHandle);
API_PRETURN_HDR handleNetworkRequest(mosiop_t, PhidgetNetConnHandle, int *);

API_PRETURN_HDR createPhidgetNetConn(IPhidgetServerHandle, PhidgetNetConnHandle *);
API_VRETURN_HDR stopPhidgetNetConn(PhidgetNetConnHandle);
API_VRETURN_HDR waitForPhidgetNetConnThread(PhidgetNetConnHandle);
API_VRETURN_HDR stopAndWaitForPhidgetNetConnThread(PhidgetNetConnHandle);

API_PRETURN_HDR PhidgetNet_startServer2(PhidgetServerType, int, int, const char *, const char *, int,
  const char *, initPhidgetNetConn_t, handlePhidgetNetConn_t, handleRequest_t, PhidgetServerHandle *);

API_PRETURN_HDR startServerConnection(mosiop_t, IPhidgetServerHandle);

#endif /* _PHIDGET_NETWORKCODE */


API_PRETURN_HDR PhidgetNet_publishmdns(PhidgetMDNSPublishHandle *, const char *, const char *,
  const char *, int, kv_t *);
API_PRETURN_HDR PhidgetNet_unpublishmdns(PhidgetMDNSPublishHandle *);

#endif /* !EXTERNALPROTO */

#endif /* _PHIDGET_NETWORK_H_ */
