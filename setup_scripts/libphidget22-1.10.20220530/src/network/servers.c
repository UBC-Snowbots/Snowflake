#define _PHIDGET_NETWORKCODE

#include "phidgetbase.h"
#include "network/network.h"
#include "mos/bsdtree.h"
#include "mos/mos_assert.h"
#include "util/phidgetconfig.h"
#include "stats.h"

static const char *PHIDGETSERVER_NAMES[] = {
	"Unknown Server",
	"Phidget22 Server listener",
	"Phidget22 Server connection",
	"Phidget22 Server",
	"Phidget22 Web Server listener",
	"Phiget22 Web Server connection",
	"Phiget22 Web Server",
	"Phidget SBC",
	NULL
};

static uint32_t network_flags;

int _allowDataGram = 1;
uint32_t network_keepalive_client;
uint32_t network_keepalive;

#define NETFLAG_RESOLVEADDRS	0x01			/* if set, we resolve host names (disable for performance) */

#define CLIENTCONTROL_ALLOW		0x00000001
#define CLIENTCONTROL_BLOCK		0x00000002
#define CLIENTCONTROL_VERBOSE	0x08000000
typedef struct _ClientControl {
	mos_sockaddr_t				addr;
	int							flags;
	RB_ENTRY(_ClientControl)	link;
} ClientControl, *ClientControlHandle;

static int
clientcontrolcompare(ClientControlHandle a, ClientControlHandle b) {

	return (mos_sockaddr_cmpaddr(&a->addr, &b->addr));
}

static PhidgetReturnCode
setResolveAddrs(const char *b) {

	if (istrue(b))
		network_flags |= NETFLAG_RESOLVEADDRS;
	else
		network_flags &= ~NETFLAG_RESOLVEADDRS;

	return (EPHIDGET_OK);
}

/*
 * Allow/Deny default
 */
static int clientcontroldefault = 1;

RB_HEAD(clients, _ClientControl) clients;
RB_PROTOTYPE(clients, _ClientControl, link, clientcontrolcompare)
RB_GENERATE(clients, _ClientControl, link, clientcontrolcompare)
static mos_tlock_t *clientcontrollock;

static int
servercompare(IPhidgetServerHandle a, IPhidgetServerHandle b) {

	return (strcmp(a->info.name, b->info.name));
}

RB_HEAD(servers, _IPhidgetServer) servers;
RB_PROTOTYPE(servers, _IPhidgetServer, link, servercompare)
RB_GENERATE(servers, _IPhidgetServer, link, servercompare)
static mos_tlock_t *serverslock;
static mos_cond_t serverscond;
static int serverscount;

#define SERVERS_MAX	100	/* limit clients to ~100 */

#if ZEROCONF_SUPPORT
static ZeroconfListenerHandle zcl[PHIDGETSERVERTYPE_CNT];
static mos_tlock_t	*zclock;
#endif

static PhidgetNet_OnServerRemovedCallback serverRemoved;
static void *serverRemovedCtx;
static PhidgetNet_OnServerAddedCallback serverAdded;
static void *serverAddedCtx;

static int servers_initialized;
static int servers_started;

static void closeIPhidgetServer(IPhidgetServerHandle *);
static void stopIPhidgetServer(IPhidgetServerHandle);

void
ServersInit() {

	mos_glock((void *)1);
	if (servers_initialized) {
		mos_gunlock((void *)1);
		return;
	}

	mos_tlock_init(clientcontrollock, P22LOCK_SRVSCCLOCK, P22LOCK_FLAGS);
	mos_tlock_init(serverslock, P22LOCK_SRVSLOCK, P22LOCK_FLAGS);
#if ZEROCONF_SUPPORT
	mos_tlock_init(zclock, P22LOCK_SRVSZCLOCK, P22LOCK_FLAGS);
#endif
	mos_cond_init(&serverscond);
	RB_INIT(&servers);
	RB_INIT(&clients);

	servers_initialized = 1;

	/*
	 * The client keepalive defaults to 2x the server keep alive as the round trip between server
	 * packets will be at least the server keep alive.
	 */
	network_keepalive = 4 * 1000000;	/* 4 seconds */
	network_keepalive_client = network_keepalive * 2;
	network_flags = 0;

	mos_gunlock((void *)1);
}

void
ServersFini() {

	mos_glock((void *)1);
	if (!servers_initialized) {
		mos_gunlock((void *)1);
		return;
	}
	servers_initialized = 0;
	mos_gunlock((void *)1);

	mos_tlock_destroy(&serverslock);
	mos_tlock_destroy(&clientcontrollock);
#if ZEROCONF_SUPPORT
	mos_tlock_destroy(&zclock);
#endif
}

void
ServersStart() {

	mos_glock((void *)1);
	if (!servers_initialized || servers_started) {
		mos_gunlock((void *)1);
		return;
	}

	servers_started = 1;
	mos_gunlock((void *)1);
}

/*
 * It is not safe to touch any server handles etc. after this is called.
 * Users should have closed all of their handles before this, but if not, this will close them.
 */
void
ServersStop() {
	ClientControlHandle cca, ccb;
	IPhidgetServerHandle sa;

	mos_glock((void *)1);
	if (!servers_initialized || !servers_started) {
		mos_gunlock((void *)1);
		return;
	}
	mos_gunlock((void *)1);

	mos_tlock_lock(serverslock);
	RB_FOREACH(sa, servers, &servers)
		stopIPhidgetServer(sa);

	while (serverscount > 0)
		mos_tlock_timedwait(&serverscond, serverslock, MOS_SEC);

	mos_tlock_unlock(serverslock);

	RB_FOREACH_SAFE(cca, clients, &clients, ccb) {
		RB_REMOVE(clients, &clients, cca);
		mos_free(cca, sizeof(*cca));
	}

	mos_glock((void *)1);
	servers_started = 0;
	mos_gunlock((void *)1);
}

static int
allowClient(mos_sockaddr_t *ca) {
	ClientControlHandle ch;
	ClientControl chs;
	int flags;

	chs.addr = *ca;

	mos_tlock_lock(clientcontrollock);
	ch = RB_FIND(clients, &clients, &chs);
	if (ch == NULL)
		flags = 0;
	else
		flags = ch->flags;
	mos_tlock_unlock(clientcontrollock);

	if (flags & CLIENTCONTROL_BLOCK) {
		if (flags & CLIENTCONTROL_VERBOSE)
			netlogerr("client '%s' blocked", mos_getaddrinfo(ca, NULL, 0));
		return (0);
	} else if (flags & CLIENTCONTROL_ALLOW) {
		if (flags & CLIENTCONTROL_VERBOSE)
			netloginfo("client '%s' allowed", mos_getaddrinfo(ca, NULL, 0));
		return (1);
	}

	return (clientcontroldefault);
}

static void
devicerelease(PhidgetNetConnHandle *nc) {

	mos_free(*nc, sizeof(PhidgetNetConn));
	*nc = NULL;
}

static void
initDeviceNetConn(IPhidgetServerHandle server, PhidgetNetConnHandle nc) {

	/* defaults, but set here anyway */
	nc->read = deviceread;
	nc->write = devicewrite;
	nc->release = devicerelease;
	nc->protocol = mos_strdup(PHIDGET_NET_PROTOCOL, NULL);
	nc->pmajor = PHIDGET_NET_PROTOCOL_MAJOR;
	nc->pminor = PHIDGET_NET_PROTOCOL_MINOR;

	switch (server->info.type) {
	case PHIDGETSERVER_NONE:
		nc->conntype = PHIDGETCONN_NONE;
		break;
	case PHIDGETSERVER_DEVICELISTENER:
	case PHIDGETSERVER_WWWLISTENER:
		nc->conntype = PHIDGETCONN_LISTENER;
		break;
	case PHIDGETSERVER_DEVICE:
	case PHIDGETSERVER_WWW:
	case PHIDGETSERVER_SBC:
		nc->conntype = PHIDGETCONN_LOCAL;
		break;
	default:
		MOS_PANIC("invalid server type");
	}
}

static PhidgetReturnCode
addServer(PhidgetServerType srvtype, const char *srvname, int af, const char *address, int port,
  const char *passwd, initPhidgetNetConn_t initnc, handlePhidgetNetConn_t handlenc, handleRequest_t handlereq,
  IPhidgetServerHandle *_is) {
	IPhidgetServerHandle is;
	PhidgetReturnCode res;
	IPhidgetServer sis;
	char hostname[128];
	int err;

	TESTPTR(srvname);
	TESTPTR(passwd);
	TESTPTR(initnc);
	TESTPTR(handlenc);
	TESTPTR(handlereq);

	sis.info.name = srvname; /* search key */

	netlogdebug("%s", srvname);

	mos_tlock_lock(serverslock);
	is = RB_FIND(servers, &servers, &sis);
	mos_tlock_unlock(serverslock);
	if (is != NULL)
		return (EPHIDGET_DUPLICATE);

	is = mos_zalloc(sizeof(*is));
	mos_tlock_init(is->lock, P22LOCK_SRVSIPHIDLOCK, P22LOCK_FLAGS);
	mos_cond_init(&is->cond);
	is->this = is;
	is->name = mos_strdup(srvname, NULL);
	is->passwd = mos_strdup(passwd, NULL);
	if (address)
		is->address = mos_strdup(address, NULL);
	else
		is->address = mos_strdup("0.0.0.0", NULL);
	is->port = port;
	is->af = af;

	is->handleClient = handlenc;
	is->initNetConn = initnc;
	res = createPhidgetNetConn(is, &is->nc);
	if (res != EPHIDGET_OK) {
		netlogerr("failed to create netconn");
		goto bad;
	}

	if (is->address != NULL) {
		netlogdebug("%s: looking up address (%s)", srvname, is->address);
		err = mos_netop_getbyname(MOS_IOP_IGNORE, is->address, is->af, &is->nc->addr);
		netlogdebug("%s: looked up address", srvname);
		if (err != 0) {
			netlogerr("failed to resolve server address from '%s'", is->address);
			goto bad;
		}
	}

	switch (is->af) {
	case MOS_AF_INET4:
		is->nc->addr.s4.sin_family = MOS_AF_INET4;
		is->nc->addr.s4.sin_port = htons(is->port);
		if (is->address == NULL)
			is->nc->addr.s4.sin_addr.s_addr = htonl(INADDR_ANY);
		break;
	case MOS_AF_INET6:
		is->nc->addr.s6.sin6_family = MOS_AF_INET6;
		is->nc->addr.s6.sin6_port = htons(is->port);
		if (is->address == NULL)
			is->nc->addr.s6.sin6_addr = in6addr_any;
		break;
	}

	res = mos_netop_gethostname(MOS_IOP_IGNORE, hostname, sizeof(hostname));
	if (res == 0)
		is->hostname = mos_strdup(hostname, NULL);
	else
		is->hostname = mos_strdup("localhost", NULL);

	is->info.handle = is;
	is->info.name = is->name;
	is->info.type = srvtype;
	is->info.stype = PHIDGETSERVER_NAMES[srvtype];
	is->info.host = is->hostname;
	is->info.addr = is->address;
	is->info.port = port;

	/*
	 * handleRequest hangs off of the connection so we can use the same
	 * code for client and servers.
	 */
	is->nc->handleRequest = handlereq;

	switch (srvtype) {
	case PHIDGETSERVER_DEVICELISTENER:
	case PHIDGETSERVER_WWWLISTENER:
		mos_tlock_lock(serverslock);
		RB_INSERT(servers, &servers, is);
		serverscount++;
		mos_tlock_unlock(serverslock);
		mos_cond_broadcast(&serverscond);
		break;
	}

	*_is = is;

	return (EPHIDGET_OK);

bad:

	mos_tlock_unlock(serverslock);
	mos_tlock_destroy(&is->lock);
	mos_cond_destroy(&is->cond);
	mos_free(is->name, MOSM_FSTR);
	mos_free(is->passwd, MOSM_FSTR);
	if (is->address)
		mos_free(is->address, MOSM_FSTR);
	if (is->hostname)
		mos_free(is->hostname, MOSM_FSTR);
	mos_free(is, sizeof(*is));
	return (res);
}

static void
freeServer(IPhidgetServerHandle *is) {

	MOS_ASSERT((*is)->nc == NULL);	/* make sure the network connection was closed! */

	switch ((*is)->info.type) {
	case PHIDGETSERVER_DEVICELISTENER:
	case PHIDGETSERVER_WWWLISTENER:
		mos_tlock_lock(serverslock);
		MOS_ASSERT(serverscount > 0);

		RB_REMOVE(servers, &servers, *is);
		serverscount--;
		mos_cond_broadcast(&serverscond);
		mos_tlock_unlock(serverslock);
	}

	mos_tlock_destroy(&(*is)->lock);
	mos_cond_destroy(&(*is)->cond);

	mos_free((*is)->name, MOSM_FSTR);
	if ((*is)->address) {
		if ((*is)->hostname && (*is)->hostname != (*is)->address)
			mos_free((*is)->hostname, MOSM_FSTR);
		mos_free((*is)->address, MOSM_FSTR);
	}
	mos_free((*is)->passwd, MOSM_FSTR);
	mos_free(*is, sizeof(**is));
	*is = NULL;
}

#if ZEROCONF_SUPPORT
static void
wwwServerListener(ZeroconfListenerHandle handle, void *ctx, int added, int interface, Zeroconf_Protocol proto,
	const char *name, const char *host, const char *type, const char *domain) {
	mos_sockaddr_list_t *addrlist;
	PhidgetReturnCode res;
	PhidgetServer server;
	char addrbuf[64];
	uint16_t port;
	kv_t *kv;

	netlogverbose("%s", host);
	if (!added) {
		if (serverRemoved) {
			server.name = name;
			server.type = PHIDGETSERVER_WWWLISTENER;
			server.stype = PHIDGETSERVER_NAMES[server.type];
			server.handle = NULL;
			server.addr = NULL;
			server.port = 0;
			server.host = host;
			server.flags = 0;
			serverRemoved(serverRemovedCtx, &server);
		}
		return;
	}

	if (serverAdded) {
		res = Zeroconf_lookup(handle, interface, proto, name, host, type, domain, ZCP_IPv4, &addrlist, &port, &kv);
		if (res != 0) {
			netlogerr("Zeroconf_lookup() failed for %s", name);
			return;
		}
		server.name = name;
		server.type = PHIDGETSERVER_WWWLISTENER;
		server.stype = PHIDGETSERVER_NAMES[server.type];
		server.handle = NULL;
		server.addr = mos_ntoa(&addrlist->addr, addrbuf, sizeof(addrbuf));
		server.port = port;
		server.host = host;
		server.flags = 0;

		serverAdded(serverAddedCtx, &server, kv);
		mos_freeaddrlist(addrlist);
		if (kv)
			kvfree(&kv);
	}
}

static void
sbcServerListener(ZeroconfListenerHandle handle, void *ctx, int added, int interface, Zeroconf_Protocol proto,
	const char *name, const char *host, const char *type, const char *domain) {
	mos_sockaddr_list_t *addrlist;
	PhidgetReturnCode res;
	PhidgetServer server;
	char addrbuf[64];
	uint16_t port;
	kv_t *kv;

	netlogverbose("%s", host);
	if (!added) {
		if (serverRemoved) {
			server.name = name;
			server.type = PHIDGETSERVER_SBC;
			server.stype = PHIDGETSERVER_NAMES[server.type];
			server.handle = NULL;
			server.addr = NULL;
			server.host = host;
			server.port = 0;
			server.flags = 0;
			serverRemoved(serverRemovedCtx, &server);
		}
		return;
	}

	if (serverAdded) {
		res = Zeroconf_lookup(handle, interface, proto, name, host, type, domain, ZCP_IPv4, &addrlist, &port, &kv);
		if (res != 0) {
			netlogerr("Zeroconf_lookup() failed for %s [%s]", name, host);
			return;
		}

		server.name = name;
		server.type = PHIDGETSERVER_SBC;
		server.stype = PHIDGETSERVER_NAMES[server.type];
		server.handle = NULL;
		server.addr = mos_ntoa(&addrlist->addr, addrbuf, sizeof(addrbuf));
		server.port = port;
		server.host = host;
		server.flags = 0;
		serverAdded(serverAddedCtx, &server, kv);
		mos_freeaddrlist(addrlist);
		if (kv)
			kvfree(&kv);
	}
}

void deviceServerAdded(const char *name, const char *host, mos_sockaddr_list_t *addrlist, int port, kv_t *txt) {
	PhidgetServer server;
	char addrbuf[64];
	const char *v;

	if (serverAdded) {

		server.name = name;
		server.type = PHIDGETSERVER_DEVICEREMOTE;
		server.stype = PHIDGETSERVER_NAMES[server.type];
		server.handle = NULL;
		server.addr = mos_ntoa(&addrlist->addr, addrbuf, sizeof(addrbuf));
		server.port = port;
		server.host = host;
		server.flags = 0;

		if (txt) {
			v = kvgetstrc(txt, "auth", NULL);
			if (v && *v == 'y')
				server.flags = PHIDGETSERVER_AUTHREQUIRED;
		}

		serverAdded(serverAddedCtx, &server, txt);
	}
}

static void
deviceServerListener(ZeroconfListenerHandle hdl, void *ctx, int added, int interface, Zeroconf_Protocol proto,
	const char *name, const char *host, const char *type, const char *domain) {
	PhidgetReturnCode res;
	PhidgetServer server;

	if (!added) {
		PhidgetNet_undiscoveredServer(name);
		if (serverRemoved) {
			server.name = name;
			server.type = PHIDGETSERVER_DEVICEREMOTE;
			server.stype = PHIDGETSERVER_NAMES[server.type];
			server.handle = NULL;
			server.addr = NULL;
			server.host = host;
			server.port = 0;
			server.flags = 0;
			serverRemoved(serverRemovedCtx, &server);
		}
		return;
	}

	netloginfo("Discovered server '%s' [%s] (interface 0x%x)", name, host, interface);

	res = PhidgetNet_discoveredServer(hdl, 0, interface, PHIDGETSERVER_DEVICEREMOTE, name, host, type, domain, -1);
	if (res != 0)
		netlogerr("failed to add MDNS server '%s': "PRC_FMT, host, PRC_ARGS(res));
}
#endif

static PhidgetReturnCode
checkKeepAlive(PhidgetNetConnHandle nc) {
	mostime_t tm;

	if (nc->errcondition != 0 || nc->keepalive == 0)
		return (nc->errcondition);

	switch (nc->conntype) {
	case PHIDGETCONN_REMOTE:	/* Client */
		/*
		 * If the client has never received a keepalive, do not anticipate one.
		 */
		if (nc->keepalive_dl == 0) {
			netlogdebug("%"PRIphid": keepalive check passed because client has never recieved a keepalive.", nc);
			return (nc->errcondition);
		}

	case PHIDGETCONN_LOCAL:		/* Server */
		tm = mos_gettime_usec();
		if (nc->keepalive_dl != 0 && nc->keepalive_dl < tm) {
			netlogerr("%"PRIphid": keepalive check failed - keepalive: [%"PRId64" usec] overshot by: [%"PRId64" usec]",
			  nc, nc->keepalive, (tm - nc->keepalive_dl));
			nc->errcondition = EPHIDGET_KEEPALIVE;
		} else {
			if (nc->keepalive_dl == 0) {
				netlogdebug("%"PRIphid": keepalive check passed - keepalive: [%"PRId64" usec]", nc, nc->keepalive);
			} else {
				netlogdebug("%"PRIphid": keepalive check passed - keepalive: [%"PRId64" usec] remaining: [%"PRId64" usec]", nc,
					nc->keepalive, (nc->keepalive_dl - tm));
			}
		}
		return (nc->errcondition);

	case PHIDGETCONN_NONE:		/* FALL THROUGH */
	case PHIDGETCONN_LISTENER:
		break;
	}
	return (0);
}

/*
 * nc is retained prior to being passed to the task.
 */
static MOS_TASK_RESULT
runKeepAlive(void *arg) {
	PhidgetNetConnHandle nc;
	mostime_t tm;

	nc = arg;

	mos_task_setname("Phidget22 Network Keepalive Thread - %s", nc->peername);
	netlogdebug("network keepalive thread started - %s: 0x%08x", nc->peername, mos_self());

	PhidgetLock(nc);
	while (nc->errcondition == 0) {
		/*
		 * If the network is stopping, or the thread goes away, we are no longer required.
		 */
		if (PhidgetCKFlagsNoLock(nc, PNCF_STOP) != 0)
			break;

		if (PhidgetCKFlagsNoLock(nc, PNCF_HASTHREAD) == 0)
			break;

		tm = mos_gettime_usec();	/* must be stored prior to checkKeepAlive() call */

		if (checkKeepAlive(nc) != EPHIDGET_OK)
			continue;

		if (nc->keepalive_dl == 0) {
			netlogdebug("%"PRIphid": keepalive check sleeping(1) for %"PRId64" us", nc, nc->keepalive);
			PhidgetTimedWait(nc, (uint32_t)(nc->keepalive / 1000));			/* usec to msec */
		} else {
			netlogdebug("%"PRIphid": keepalive check sleeping(2) for %"PRId64" us", nc, (nc->keepalive_dl - tm));
			PhidgetTimedWait(nc, (uint32_t)(nc->keepalive_dl - tm) / 1000);	/* usec to msec */
		}
	}

	PhidgetUnlock(nc);

	netlogdebug("keepalive task exiting:%s", nc->peername);
	decPhidgetStat("server.keepalivetasks");

	PhidgetRelease(&nc);

	MOS_TASK_EXIT(0);
}

PhidgetReturnCode
startKeepAliveTask(PhidgetNetConnHandle nc) {
	PhidgetReturnCode res;

	PhidgetRetain(nc);
	res = mos_task_create(NULL, runKeepAlive, nc);
	if (res != EPHIDGET_OK)
		PhidgetRelease(&nc);
	incPhidgetStat("server.keepalivetasks_ever");
	incPhidgetStat("server.keepalivetasks");
	return (res);
}

API_PRETURN
handleNetworkRequest(mosiop_t iop, PhidgetNetConnHandle nc, int *stop) {
	PhidgetReturnCode res;
	mosiop_t iop2;
	netreq_t req;
	int socks;

	/*
	 * Poll for data on the TCP socket and the UDP socket.
	 *
	 * If both have pending IO, we always read TCP first as the UDP events
	 * are of lower priority.
	 */
	res = mos_netop_tcp_rpoll2(iop, &nc->sock, &nc->dgsock, &socks, 500);
	if (res != 0) {
		if (res != EPHIDGET_TIMEOUT)
			return (MOS_ERROR(iop, res, "failed to poll for IO"));
		return (res);
	}

	if (socks & 0x01) {
		res = readRequestHeader(iop, nc, &req);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to read request"));
		goto readcomplete;
	}

	if (nc->conntype == PHIDGETCONN_REMOTE && socks & 0x2) {
		res = readDGRequestHeader(iop, nc, &req);
		if (res != EPHIDGET_OK)
			return (MOS_ERROR(iop, res, "failed to read datagram request"));
		goto readcomplete;
	}

	return (EPHIDGET_TIMEOUT);

readcomplete:

	if (req.nr_flags & NRF_REPLY) {
		res = handleReply(nc, &req);
		if (res != EPHIDGET_OK) {
			netlogerr("reqseq:%d msgtype: %s/%s failed: "PRC_FMT,
			  req.nr_repseq, strmsgtype(req.nr_type), strmsgsubtype(req.nr_stype), PRC_ARGS(res));
			if (res != EPHIDGET_NOENT)
				return (MOS_ERROR(iop, res, "handleReply() failed"));
		}
		return (EPHIDGET_OK);
	}

	iop2 = mos_iop_alloc();
	res = nc->handleRequest(iop2, nc, &req, stop);
	if (res != EPHIDGET_OK)
		netlogerr("%"PRIphid": msgtype: %s/%s failed: "PRC_FMT"\n%N",
		  nc, strmsgtype(req.nr_type), strmsgsubtype(req.nr_stype), PRC_ARGS(res), iop2);
	mos_iop_release(&iop2);

	/*
	 * We only fail the whole connection for network errors.
	 */
	if (mos_notice_isnetworkrelated(res))
		return (MOS_ERROR(iop, res, "network error on %s/%s", strmsgtype(req.nr_type), strmsgsubtype(req.nr_stype)));

	return (EPHIDGET_OK);
}

static MOS_TASK_RESULT
runClientConnection(void *arg) {
	IPhidgetServerHandle server;
	PhidgetReturnCode res;
	mosiop_t iop;

	res = EPHIDGET_OK;

	server = arg;

	mos_task_setname("Phidget22 Network Server Client Thread - %"PRIphid, server->nc);
	netlogdebug("'%s' network server client thread started - %"PRIphid": 0x%08x", server->name, server->nc, mos_self());

	mos_tlock_lock(server->lock);
	if (server->state != TS_RUN)
		goto done;

	server->state = TS_RUNNING;
	mos_tlock_unlock(server->lock);

	iop = mos_iop_alloc();
	res = server->handleClient(iop, server);
	if (res != EPHIDGET_OK) {
		if (res == EPHIDGET_IO || res == EPHIDGET_PIPE || res == EPHIDGET_ACCESS) // we expect network errors
			netlogverbose("'%s' failed for client %"PRIphid"\n%N", server->name, server->nc, iop);
		else
			netlogerr("'%s' failed for client %"PRIphid"\n%N", server->name, server->nc, iop);
	}
	mos_iop_release(&iop);

	if (res != EPHIDGET_ACCESS && server->info.type != PHIDGETSERVER_WWW)
		netloginfo("%"PRIphid" disconnected", server->nc);

	PhidgetNetConnClose(server->nc);
	PhidgetCLRFlags(server->nc, PNCF_HASTHREAD);
	PhidgetBroadcast(server->nc);
	mos_tlock_lock(server->lock);

done:

	server->state = TS_STOPPED;
	mos_cond_broadcast(&server->cond);
	mos_tlock_unlock(server->lock);

	/*
	 * This is where the handle and netconn are freed.
	 */
	closeIPhidgetServer(&server);

	decPhidgetStat("server.clienttasks");
	MOS_TASK_EXIT(res);
}

#define SERVER_FMT	"'%s %s':"
#define SERVER_ARG	server->info.stype, server->name

static MOS_TASK_RESULT
runAcceptServer(void *arg) {
	IPhidgetServerHandle server;
	IPhidgetServerHandle conn;
	mos_sockaddr_t caddr;
	char connname[64];
	mos_socket_t cs;
	mosiop_t iop;
	int err;

	server = arg;

	mos_task_setname("Phidget22 Network Server Accept Thread - "SERVER_FMT, SERVER_ARG);
	netlogdebug(SERVER_FMT" network nerver accept thread started: 0x%08x", SERVER_ARG, mos_self());

	mos_tlock_lock(server->lock);

	/*
	 * This would only happen if we were started and then stopped right after.
	 * In that case, server->nc would be NULL, and we would crash.
	 */
	if (server->state != TS_RUN)
		goto done;

	MOS_ASSERT(server->nc != NULL);
	server->state = TS_RUNNING;

	for (;;) {
		if (server->state == TS_STOP)
			break;

		mos_tlock_unlock(server->lock);

		iop = mos_iop_alloc();

		netlogverbose(SERVER_FMT "polling for connection", SERVER_ARG);
		err = mos_netop_tcp_rpoll(iop, &server->nc->sock, 1000);
		if (err) {
			if (err != EPHIDGET_TIMEOUT)
				netlogerr(SERVER_FMT " failed to poll for client connection\n%N", SERVER_ARG, iop);
			goto next;
		}

		/*
		 * Allow a reasonable number of clients.
		 */
		if (serverscount > SERVERS_MAX) {
			netlogwarn(SERVER_FMT "too many connections (%d): dropping connection", SERVER_ARG, serverscount);
			goto next;
		}

		netlogdebug(SERVER_FMT "calling accept for connection", SERVER_ARG);
		err = mos_netop_tcp_accept(iop, &server->nc->sock, &cs, &caddr);
		if (err != 0) {
			netlogerr(SERVER_FMT " failed to accept client connection\n%N", SERVER_ARG, iop);
			goto next;
		}

		/*
		 * We simply drop the client.  No logging should be done here.
		 */
		netlogdebug(SERVER_FMT "calling allowClient()", SERVER_ARG);
		if (!allowClient(&caddr)) {
			netlogdebug(SERVER_FMT "client not allowed", SERVER_ARG);
			mos_netop_tcp_closesocket(MOS_IOP_IGNORE, &cs);
			goto next;
		}
		netlogdebug(SERVER_FMT "called allowClient()", SERVER_ARG);

		mos_netop_setnodelay(&cs);

		/*
		 * The connection name is the server name + the connection number.
		 * The connection type is +1 the server type (server should be the listener, +1 the conn)
		 */
		server->conncnt++;
		mos_snprintf(connname, sizeof(connname), "%s-%u", server->name, server->conncnt);
		netlogdebug(SERVER_FMT "adding server: %s", SERVER_ARG, connname);
		err = addServer(server->info.type + 1, connname, server->af, server->address, server->port,
		  server->passwd, server->initNetConn, server->handleClient, server->nc->handleRequest, &conn);
		netlogdebug(SERVER_FMT "added server: %s (%d)", SERVER_ARG, connname, err);
		if (err != 0) {
			netlogerr(SERVER_FMT "failed to add server for client connection", SERVER_ARG);
			mos_netop_tcp_closesocket(iop, &cs);
			goto next;
		}

		conn->nc->peername = mos_strdup(mos_getaddrinfo(&caddr, NULL, 0), NULL);
		conn->nc->conntype = PHIDGETCONN_LOCAL;
		conn->nc->addr = caddr;
		conn->nc->sock = cs;

		netlogverbose(SERVER_FMT "%s accepted connection [%s -> %s]", SERVER_ARG, conn->name,
		  conn->nc->peername, mos_getaddrinfo(&server->nc->addr, NULL, 0));

		netlogdebug(SERVER_FMT "about to create task to handle client", SERVER_ARG);
		PhidgetLock(conn->nc);
		conn->nc->__flags |= PNCF_HASTHREAD;
		conn->state = TS_RUN;
		err = mos_task_create(&conn->self, runClientConnection, conn);
		if (err != 0) {
			netlogerr(SERVER_FMT "failed to start server task", SERVER_ARG);
			conn->nc->__flags &= ~PNCF_HASTHREAD;
			PhidgetUnlock(conn->nc);
			PhidgetRelease(&conn->nc);
			goto next;
		}
		PhidgetUnlock(conn->nc);

		incPhidgetStat("server.clienttasks_ever");
		incPhidgetStat("server.clienttasks");
		netlogdebug(SERVER_FMT "connection thread started", SERVER_ARG);

	next:
		mos_iop_release(&iop);
		mos_tlock_lock(server->lock);
	}
done:

	netloginfo(SERVER_FMT "no longer accepting connections", SERVER_ARG);

	server->state = TS_STOPPED;
	mos_cond_broadcast(&server->cond);
	mos_tlock_unlock(server->lock);

	/*
	 * This is where the handle and netconn are freed.
	 */
	closeIPhidgetServer(&server);

	decPhidgetStat("server.accepttasks");
	MOS_TASK_EXIT(0);
}

static PhidgetReturnCode
startIPhidgetServer(int flags, IPhidgetServerHandle server) {
	PhidgetReturnCode err;
	mosiop_t iop;

	server->nc->peername = mos_strdup("<listener>", NULL);

	iop = mos_iop_alloc();

	err = mos_netop_tcp_openserversocket(iop, &server->nc->sock, &server->nc->addr);
	if (err != 0) {
		netlogerr("failed to open server socket\n%N", iop);
		goto bad;
	}

	PhidgetSetFlags(server->nc, flags & PHIDGET_NETWORK_FLAGMASK);
	server->state = TS_RUN;
	err = mos_task_create(&server->self, runAcceptServer, server);
	if (err != 0)
		goto bad;

	incPhidgetStat("server.accepttasks_ever");
	incPhidgetStat("server.accepttasks");

	netlogdebug("server started ok");

	mos_iop_release(&iop);
	return (0);

bad:

	mos_free(server->nc, sizeof(PhidgetNetConn));
	server->nc = NULL;

	mos_iop_release(&iop);

	return (err);
}

static void
stopIPhidgetServer(IPhidgetServerHandle server) {

	mos_tlock_lock(server->lock);

	if (server->state == TS_RUNNING) {
		server->state = TS_STOP;
		mos_cond_broadcast(&server->cond);
	}

	mos_tlock_unlock(server->lock);
}

static void
closeIPhidgetServer(IPhidgetServerHandle *_server) {
	IPhidgetServerHandle server;

	server = *_server;

	mos_tlock_lock(server->lock);

	MOS_ASSERT(server->state != TS_RUNNING);
	if (server->nc) {
		PhidgetNetConnClose(server->nc);
		PhidgetRelease(&server->nc);
	}

	mos_tlock_unlock(server->lock);

	if (serverRemoved)
		serverRemoved(serverRemovedCtx, &server->info);

	freeServer(_server);
}

static PhidgetReturnCode
startServer(PhidgetServerType srvtype, int flags, int af, const char *srvname, const char *address, int port,
  const char *passwd, initPhidgetNetConn_t initnc, handlePhidgetNetConn_t handlenc, handleRequest_t handlereq,
  PhidgetServerHandle *_server) {
	IPhidgetServerHandle server;
	PhidgetReturnCode res;
	const char *sname;
	int dupsrvname;
	int timeoutcnt;
	kv_t *keys;

#if ZEROCONF_SUPPORT
	char srvname2[128];
#endif

	TESTPTR(srvname);
	TESTPTR(_server);

	sname = srvname;
	dupsrvname = 1;
	timeoutcnt = 0;

	if (af != MOS_AF_INET4 && af != MOS_AF_INET6)
		return (EPHIDGET_INVALIDARG);

	if (!validServerName(srvname)) {
		netlogerr("server name contains invalid characters ('\"' or '\\')? '%s'", srvname);
		return (EPHIDGET_INVALIDARG);
	}

	res = newkv(&keys);
	if (res != 0) {
		netlogerr("failed to create kv for server keys");
		return (EPHIDGET_UNEXPECTED);
	}

	kvvadd(keys, NULL, "txtvers", "%d", PHIDGET_MDNS_TXTVER);
	kvadd(keys, NULL, "srvname", srvname);
	kvvadd(keys, NULL, "protocolmajor", "%d", PHIDGET_NET_PROTOCOL_MAJOR);
	kvvadd(keys, NULL, "protocolpminor", "%d", PHIDGET_NET_PROTOCOL_MINOR);
	if (passwd == NULL || mos_strlen(passwd) == 0)
		kvadd(keys, NULL, "auth", "n");
	else
		kvadd(keys, NULL, "auth", "y");

	if (passwd == NULL)
		passwd = "";

	res = addServer(srvtype, srvname, AF_INET, address, port, passwd, initnc, handlenc, handlereq, &server);
	if (res != 0) {
		kvfree(&keys);
		netlogerr("failed to add server");
		return (res);
	}

	res = startIPhidgetServer(flags, server);
	if (res != 0) {
		freeServer(&server);
		kvfree(&keys);
		netlogerr("failed to start server");
		return (res);
	}

#if ZEROCONF_SUPPORT
dup :
timeout:

	if (flags & PHIDGET_NETWORK_PUBLISHMDNS) {
		res = Zeroconf_publish(&server->nc->pubhandle, sname, NULL, PHIDGET_NETWORK_MDNS_DEVICE,
			server->port, keys);
		if (res == EPHIDGET_OK) {
			netloginfo("Published '%s' on port %d for discovery", PHIDGET_NETWORK_MDNS_DEVICE, server->port);
		} else if (res == EPHIDGET_DUPLICATE) {
			dupsrvname++;
			if (dupsrvname >= 100) {
				netlogerr("failed to publish '%s' (too many duplicates)", srvname);
			} else {
				mos_snprintf(srvname2, sizeof(srvname2), "%s (%d)", srvname, dupsrvname);
				netloginfo("duplicate server name (%s) - trying '%s'", srvname, srvname2);
				sname = srvname2;
				goto dup;
			}
		} else if (res == EPHIDGET_TIMEOUT) {
			timeoutcnt++;
			if (timeoutcnt >= 30) {
				netlogerr("failed to publish '%s' (too many timeouts)", srvname);
			} else {
				goto timeout;
			}
		} else {
			netlogerr("failed to publish '%s' on port %d: "PRC_FMT,
				PHIDGET_NETWORK_MDNS_DEVICE, server->port, PRC_ARGS(res));
		}
	}
#endif
	kvfree(&keys);

	*_server = &server->info;

	if (serverAdded)
		serverAdded(serverAddedCtx, &server->info, NULL);

	return (0);
}

API_PRETURN
PhidgetNet_startServer2(PhidgetServerType srvtype, int flags, int af, const char *srvname,
  const char *address, int port, const char *passwd, initPhidgetNetConn_t initnc,
  handlePhidgetNetConn_t handlenc, handleRequest_t handlereq, PhidgetServerHandle *_server) {
	PhidgetReturnCode res;

	PhidgetNet_start();
	res = startServer(srvtype, flags, af, srvname, address, port, passwd, initnc, handlenc, handlereq, _server);
	if (res != 0)
		PhidgetNet_stop();
	return (PHID_RETURN(res));
}

static const char *badname = "\"\\";

int
validServerName(const char *srvname) {
	const char *c, *b;

	for (c = srvname; *c; c++) {
		if (!mos_isprint_utf8((uint8_t)*c))
			return (0);
		for (b = badname; *b; b++)
			if (*b == *c)
				return (0);
	}

	return (1);
}

/** Exported Public API **/

API_PRETURN
PhidgetNet_enableServerDiscovery(PhidgetServerType srvt) {
#if ZEROCONF_SUPPORT
	PhidgetReturnCode res;

	switch (srvt) {
	case PHIDGETSERVER_DEVICELISTENER:
		srvt++;
	case PHIDGETSERVER_DEVICE:
		srvt++;
		break;
	case PHIDGETSERVER_WWWLISTENER:
		srvt++;
	case PHIDGETSERVER_WWW:
		srvt++;
		break;
	}

	mos_tlock_lock(zclock);
	if (zcl[srvt]) {
		mos_tlock_unlock(zclock);
		return (EPHIDGET_OK);
	}

	/*
	 * Create a reference to the network code.
	 */
	PhidgetNet_start();

	switch (srvt) {
	case PHIDGETSERVER_DEVICEREMOTE:
		res = Zeroconf_listen(&zcl[srvt], PHIDGET_NETWORK_MDNS_DEVICE, deviceServerListener, NULL);
		if (res != EPHIDGET_OK) {
			mos_tlock_unlock(zclock);
			PhidgetNet_stop();
			return (PHID_RETURN(res));
		}
		break;
	case PHIDGETSERVER_WWWREMOTE:
		res = Zeroconf_listen(&zcl[srvt], PHIDGET_NETWORK_MDNS_WWW, wwwServerListener, NULL);
		if (res != EPHIDGET_OK) {
			mos_tlock_unlock(zclock);
			PhidgetNet_stop();
			return (PHID_RETURN(res));
		}
		break;
	case PHIDGETSERVER_SBC:
		res = Zeroconf_listen(&zcl[srvt], PHIDGET_NETWORK_MDNS_SBC, sbcServerListener, NULL);
		if (res != EPHIDGET_OK) {
			mos_tlock_unlock(zclock);
			PhidgetNet_stop();
			return (PHID_RETURN(res));
		}
		break;
	case PHIDGETSERVER_NONE:
	default:
		mos_tlock_unlock(zclock);
		PhidgetNet_stop();
		return (PHID_RETURN(EPHIDGET_INVALIDARG));
	}

	mos_tlock_unlock(zclock);
	return (EPHIDGET_OK);
#else
	return (PHID_RETURN_ERRSTR(EPHIDGET_UNSUPPORTED, "Server discovery is disabled in this build."));
#endif
}

API_PRETURN
PhidgetNet_disableServerDiscovery(PhidgetServerType srvt) {
#if ZEROCONF_SUPPORT

	switch (srvt) {
	case PHIDGETSERVER_DEVICELISTENER:
		srvt++;
	case PHIDGETSERVER_DEVICE:
		srvt++;
		break;
	case PHIDGETSERVER_WWWLISTENER:
		srvt++;
	case PHIDGETSERVER_WWW:
		srvt++;
		break;
	}

	mos_tlock_lock(zclock);
	if (!zcl[srvt]) {
		mos_tlock_unlock(zclock);
		return (EPHIDGET_OK);
	}

	switch (srvt) {
	case PHIDGETSERVER_DEVICEREMOTE:
	case PHIDGETSERVER_WWWREMOTE:
	case PHIDGETSERVER_SBC:
		Zeroconf_listenclose(&zcl[srvt]);
		mos_tlock_unlock(zclock);
		PhidgetNet_stop();	/* Release our reference on the network code */
		return (EPHIDGET_OK);

	case PHIDGETSERVER_NONE:
	default:
		mos_tlock_unlock(zclock);
		return (PHID_RETURN(EPHIDGET_INVALIDARG));
	}
#else
	return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
#endif
}

API_PRETURN
PhidgetNet_publishmdns(PhidgetMDNSPublishHandle *handle, const char *name, const char *host,
  const char *regtype, int _port, kv_t *TXTRecords) {
#if ZEROCONF_SUPPORT
	ZeroconfPublishHandle zch;
	PhidgetReturnCode res;

	TESTPTR_PR(handle);

	res = Zeroconf_publish(&zch, name, host, regtype, _port, TXTRecords);
	if (res != 0)
		return (PHID_RETURN(res));

	*handle = mos_malloc(sizeof(PhidgetMDNSPublishHandle));
	(*handle)->handle = zch;

	return (EPHIDGET_OK);
#else
	return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
#endif
}

API_PRETURN
PhidgetNet_unpublishmdns(PhidgetMDNSPublishHandle *handle) {
#if ZEROCONF_SUPPORT
	ZeroconfPublishHandle zch;

	TESTPTR(handle);

	zch = (*handle)->handle;

	Zeroconf_unpublish(&zch);

	mos_free(*handle, sizeof(PhidgetMDNSPublishHandle));
	*handle = NULL;

	return (EPHIDGET_OK);
#else
	return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
#endif
}

API_PRETURN
PhidgetNet_setOnServerRemovedHandler(PhidgetNet_OnServerRemovedCallback func, void *ctx) {

	serverRemoved = func;
	serverRemovedCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetNet_setOnServerAddedHandler(PhidgetNet_OnServerAddedCallback func, void *ctx) {

	serverAdded = func;
	serverAddedCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetNet_startServer(int flags, int af, const char *srvname, const char *address, int port,
  const char *passwd, PhidgetServerHandle *server) {

	return (PhidgetNet_startServer2(PHIDGETSERVER_DEVICELISTENER, flags, af, srvname, address, port, passwd,
		initDeviceNetConn, handleDeviceClient, handleDeviceRequest, server));
}

API_PRETURN
PhidgetNet_stopServer(PhidgetServerHandle *_server) {
	IPhidgetServerHandle server;

	TESTPTR_PR(_server);
	TESTPTR_PR(*_server);

	/* GCC seems to have trouble with the typedef and const.. */
	server = ((const struct _IPhidgetServer *)(*_server))->this;

	stopIPhidgetServer(server);

	*_server = NULL;

	PhidgetNet_stop();

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
setBlockClient(const char *json) {
	ClientControlHandle chtmp;
	ClientControlHandle ch;
	PhidgetReturnCode res;
	const char *addr;
	pconf_t *pc;
	int verbose;
	int af;
#ifdef Windows
	IN6_ADDR a6;
	IN_ADDR a4;
#else
	struct in6_addr a6;
	struct in_addr a4;
#endif

	res = pconf_parsejson(&pc, json, mos_strlen(json));
	if (res != EPHIDGET_OK)
		return (res);

	af = pconf_getu32(pc, AF_INET, "family");
	verbose = pconf_getbool(pc, 1, "verbose");
	addr = pconf_getstr(pc, NULL, "addr");
	if (addr == NULL) {
		pconf_release(&pc);
		return (EPHIDGET_INVALIDARG);
	}

	switch (af) {
	case AF_INET:
		if (!inet_pton(af, addr, &a4))
			return (EPHIDGET_INVALIDARG);
		ch = mos_malloc(sizeof(*ch));
		ch->addr.s4.sin_addr = a4;
		break;
	case AF_INET6:
		if (!inet_pton(af, addr, &a6))
			return (EPHIDGET_INVALIDARG);
		ch = mos_malloc(sizeof(*ch));
		ch->addr.s6.sin6_addr = a6;
		break;
	default:
		pconf_release(&pc);
		return (EPHIDGET_INVALIDARG);
	}
	pconf_release(&pc);

	ch->addr.sa.sa_family = af;
	ch->flags = CLIENTCONTROL_BLOCK;
	if (verbose)
		ch->flags |= CLIENTCONTROL_VERBOSE;

	mos_tlock_lock(clientcontrollock);
	chtmp = RB_INSERT(clients, &clients, ch);
	mos_tlock_unlock(clientcontrollock);

	if (chtmp == NULL)
		return (0);

	mos_free(ch, sizeof(*ch));
	return (EPHIDGET_DUPLICATE);
}

static PhidgetReturnCode
setAllowClient(const char *json) {
	ClientControlHandle chtmp;
	ClientControlHandle ch;
	PhidgetReturnCode res;
	const char *addr;
	pconf_t *pc;
	int verbose;
	int af;

#ifdef Windows
	IN6_ADDR a6;
	IN_ADDR a4;
#else
	struct in6_addr a6;
	struct in_addr a4;
#endif

	res = pconf_parsejson(&pc, json, mos_strlen(json));
	if (res != EPHIDGET_OK)
		return (res);

	af = pconf_getu32(pc, AF_INET, "family");
	verbose = pconf_getbool(pc, 1, "verbose");
	addr = pconf_getstr(pc, NULL, "addr");
	if (addr == NULL) {
		pconf_release(&pc);
		return (EPHIDGET_INVALIDARG);
	}

	switch (af) {
	case AF_INET:
		if (!inet_pton(af, addr, &a4))
			return (EPHIDGET_INVALIDARG);
		ch = mos_malloc(sizeof(*ch));
		ch->addr.s4.sin_addr = a4;
		break;
	case AF_INET6:
		if (!inet_pton(af, addr, &a6))
			return (EPHIDGET_INVALIDARG);
		ch = mos_malloc(sizeof(*ch));
		ch->addr.s6.sin6_addr = a6;
		break;
	default:
		pconf_release(&pc);
		return (EPHIDGET_INVALIDARG);
	}
	pconf_release(&pc);

	ch->addr.sa.sa_family = af;
	ch->flags = CLIENTCONTROL_ALLOW;
	if (verbose)
		ch->flags |= CLIENTCONTROL_VERBOSE;

	mos_tlock_lock(clientcontrollock);
	chtmp = RB_INSERT(clients, &clients, ch);
	mos_tlock_unlock(clientcontrollock);

	if (chtmp == NULL)
		return (0);

	mos_free(ch, sizeof(*ch));
	return (EPHIDGET_DUPLICATE);
}

static PhidgetReturnCode
setAllowClients(const char *val) {
	int32_t ccdefault;

	if (mos_strto32(val, 0, &ccdefault) != 0)
		return (EPHIDGET_INVALIDARG);

	if (ccdefault)
		netloginfo("Client network connections allowed by default");
	else
		netloginfo("Client network connections denied by default");

	clientcontroldefault = ccdefault;

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
setKeepAlive(const char *val) {
	uint32_t ms;

	if (mos_strtou32(val, 0, &ms) != 0)
		return (EPHIDGET_INVALIDARG);

	network_keepalive = ms * 1000;
	network_keepalive_client = network_keepalive;

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
setAllowDataGram(const char *val) {
	int32_t i;

	if (mos_strto32(val, 0, &i) == 0) {
		_allowDataGram = (i != 0);
		return (EPHIDGET_OK);
	}
	if (mos_strcasecmp(val, "false") == 0)
		_allowDataGram = 0;
	else if (mos_strcasecmp(val, "true") == 0)
		_allowDataGram = 1;

	return (EPHIDGET_INVALIDARG);
}

API_PRETURN
PhidgetNet_setPropertyv(const char *key, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	char val[1024];
	int len;

	len = mos_vsnprintf(val, sizeof(val), fmt, va);
	if ((unsigned)len >= sizeof(val))
		return (PHID_RETURN(EPHIDGET_NOSPC));

	if (mos_strcmp(key, "keepalive") == 0)
		res =setKeepAlive(val);
	else if (mos_strcmp(key, "allowclients") == 0)
		res =setAllowClients(val);
	else if (mos_strcmp(key, "allowclient") == 0)
		res =setAllowClient(val);
	else if (mos_strcmp(key, "blockclient") == 0)
		res =setBlockClient(val);
	else if (mos_strcmp(key, "allowdatagram") == 0)
		res =setAllowDataGram(val);
	else if (mos_strcmp(key, "resolveaddrs") == 0)
		res =setResolveAddrs(val);
	else
		res = EPHIDGET_INVALIDARG;

	return (PHID_RETURN(res));
}

API_PRETURN
PhidgetNet_setProperty(const char *key, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = PhidgetNet_setPropertyv(key, fmt, va);
	va_end(va);

	// lastError set by PhidgetNet_setPropertyv
	return (res);
}

