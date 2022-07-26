#define _PHIDGET_NETWORKCODE

#include "phidgetbase.h"
#include "network/network.h"
#include "locks.h"
#include "stats.h"

#define NETCTLS "phidget22netctl"

#ifdef NDEBUG
#define nclogdebug(...)
#define nclogcrit(...) PhidgetLog_loge(NULL, 0, __func__, NETCTLS, PHIDGET_LOG_CRITICAL, __VA_ARGS__)
#define nclogerr(...) PhidgetLog_loge(NULL, 0, __func__, NETCTLS, PHIDGET_LOG_ERROR, __VA_ARGS__)
#define nclogwarn(...) PhidgetLog_loge(NULL, 0, __func__, NETCTLS, PHIDGET_LOG_WARNING, __VA_ARGS__)
#define ncloginfo(...) PhidgetLog_loge(NULL, 0, __func__, NETCTLS, PHIDGET_LOG_INFO, __VA_ARGS__)
#define nclogverbose(...)
#else
#define nclogcrit(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETCTLS, PHIDGET_LOG_CRITICAL, __VA_ARGS__)
#define nclogerr(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETCTLS, PHIDGET_LOG_ERROR, __VA_ARGS__)
#define nclogwarn(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETCTLS, PHIDGET_LOG_WARNING, __VA_ARGS__)
#define ncloginfo(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETCTLS, PHIDGET_LOG_INFO, __VA_ARGS__)
#define nclogdebug(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETCTLS, PHIDGET_LOG_DEBUG, __VA_ARGS__)
#define nclogverbose(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NETCTLS, PHIDGET_LOG_VERBOSE, __VA_ARGS__)
#endif /* NDEBUG */

#define WAITTIME_MAX	(60 * 60 * MOS_SEC)	/* 1 hour between connect attempts: nsec */
#define WAITTIME_MAXMAN	(16 * MOS_SEC)		/* max wait time for server that were not discovered with mDNS */
#define WAITTIME_INIT	(2 * MOS_SEC)		/* initial delay before reconnecting: nsec */

#define NCE_MAGIC		0x34f434f4	/* magic to identify the structure in a PhidgetServer handle */
#define NCE_ENABLED		0x00000001	/* Has a thread, and will attempt to connect */
#define NCE_CONNECTED	0x00000002	/* Is connected to the server */
#define NCE_DELETE		0x00000004	/* Is to be deleted */
#define NCE_MDNS		0x00000008	/* From mDNS */
#define NCE_AUTHFAILED	0x00000010	/* Authentication failed */
#define NCE_VIRTUAL		0x00000020	/* A placehold: holds a password etc. */
#define NCE_RUNNING		0x00000040	/* Thread is running */
#define NCE_WASCONN		0x00000080	/* We have been connected before. */
#define NCE_BADVERSION	0x00000100	/* Protocol version did not match */
#define NCE_LOOKUPF		0x00000200	/* Lookup failed */
#define NCE_DISCOVERED	0x00000400	/* Set if we were dicovered, cleared when un-discovered */
#define NCE_MANUAL		0x00001000	/* Server was added with addServer() */
#define NCE_EVADDED		0x00002000	/* ServerAdded event was called */

#define NCE_ERRORS	(NCE_LOOKUPF | NCE_BADVERSION | NCE_AUTHFAILED)

typedef struct _NetworkControlEntry {
	uint32_t				magic;
	PhidgetServerType		serverType;
	char					*name;
	char					*host;
	char					*type;
	char					*domain;
	char					*passwd;
	int						port;
	mos_af_t				proto;
	int						flags;		/* lock required */
	int						interface;
	mos_task_t				self;
	mos_tlock_t				*lock;
	mos_cond_t				cond;
#if ZEROCONF_SUPPORT
	ZeroconfListenerHandle	handle;
#endif
	PhidgetNetConnHandle	nc;
	uint64_t				waittime;
	int						failedconn;
	MTAILQ_ENTRY(_NetworkControlEntry)	link;
} NetworkControlEntry;

static MTAILQ_HEAD(ncentry_list, _NetworkControlEntry) entries;

static int			ncinitialized;
static int 			ncstarted;
static int			state;
static mos_tlock_t	*nclock;
static mos_cond_t	nccond;
static mos_task_t	nctask;

static void freeNetworkControlEntry(NetworkControlEntry **);
static MOS_TASK_RESULT runNetworkControl(void *);

static int
isStarted() {
	int ret;

	mos_glock((void *)1);
	ret = ncstarted;
	mos_gunlock((void *)1);

	return (ret);
}
#define CHECKSTARTED do {			\
	if (!isStarted())				\
		return (EPHIDGET_CLOSED);	\
} while (0)
#define CHECKSTARTED_PR do {			\
	if (!isStarted())				\
		return (PHID_RETURN_ERRSTR(EPHIDGET_CLOSED, "Networking has not started."));	\
} while (0)

void
NetworkControlInit() {

	mos_glock((void *)1);
	if (ncinitialized) {
		mos_gunlock((void *)1);
		return;
	}

	MTAILQ_INIT(&entries);

	nclock = mos_tlock_create(P22LOCK_NCTLLOCK, P22LOCK_FLAGS);
	mos_cond_init(&nccond);

	ncinitialized = 1;
	mos_gunlock((void *)1);
}

void
NetworkControlFini() {

	mos_glock((void *)1);
	if (!ncinitialized || ncstarted) {
		mos_gunlock((void *)1);
		return;
	}
	ncinitialized = 0;
	mos_gunlock((void *)1);

	mos_tlock_destroy(&nclock);
	mos_cond_destroy(&nccond);
}

void
NetworkControlStart() {

	mos_glock((void *)1);
	if (!ncinitialized || ncstarted) {
		mos_gunlock((void *)1);
		return;
	}
	ncstarted = 1;
	mos_gunlock((void *)1);

	state = TS_RUN;
	mos_task_create(&nctask, runNetworkControl, NULL);
}

void
NetworkControlStop() {
	NetworkControlEntry *tmpnce;
	NetworkControlEntry *nce;

	mos_glock((void *)1);
	if (!ncinitialized || !ncstarted) {
		mos_gunlock((void *)1);
		return;
	}
	ncstarted = 0;
	mos_gunlock((void *)1);

	mos_tlock_lock(nclock);
	state = TS_STOP;
	mos_cond_broadcast(&nccond);
	while (state != TS_STOPPED)
		mos_tlock_wait(&nccond, nclock);
	mos_tlock_unlock(nclock);

	/*
	 * Disable them all before we start waiting for them one at a time.
	 */
	MTAILQ_FOREACH(nce, &entries, link) {
		mos_tlock_lock(nce->lock);
		nce->flags &= ~NCE_ENABLED;
		if (nce->nc)
			stopPhidgetNetConn(nce->nc);
		mos_cond_broadcast(&nce->cond);
		mos_tlock_unlock(nce->lock);
	}

	MTAILQ_FOREACH_SAFE(nce, &entries, link, tmpnce) {
		mos_tlock_lock(nce->lock);
		nce->flags &= ~NCE_ENABLED;
		mos_cond_broadcast(&nce->cond);

		while (nce->flags & NCE_RUNNING || nce->flags & NCE_CONNECTED)
			mos_tlock_wait(&nce->cond, nce->lock);

		MTAILQ_REMOVE(&entries, nce, link);
		mos_tlock_unlock(nce->lock);
		freeNetworkControlEntry(&nce);
	}
}

static MOS_TASK_RESULT
runNetworkControl(void *arg) {
	NetworkControlEntry *tmpnce;
	NetworkControlEntry *nce;

	mos_task_setname("Phidget22 Network Control Central Thread");
	nclogdebug("network control central thread started: 0x%08x", mos_self());

	mos_tlock_lock(nclock);
	if (state != TS_RUN) {
		state = TS_STOPPED;
		mos_cond_broadcast(&nccond);
		mos_tlock_unlock(nclock);
		MOS_TASK_EXIT(0);
	}
	state = TS_RUNNING;

	while (state == TS_RUNNING) {
		MTAILQ_FOREACH_SAFE(nce, &entries, link, tmpnce) {
			mos_tlock_lock(nce->lock);
			if (nce->flags & NCE_DELETE) {
				/* force disabled just to be safe */
				nce->flags &= ~NCE_ENABLED;

				/* If it is still running, ignore; otherwise, delete it */
				if ((nce->flags & NCE_RUNNING) == 0 && (nce->flags & NCE_CONNECTED) == 0) {
					MTAILQ_REMOVE(&entries, nce, link);
					mos_tlock_unlock(nce->lock);
					freeNetworkControlEntry(&nce);
					continue; /* goto next */
				}
			}
			mos_tlock_unlock(nce->lock);
		}
		mos_tlock_timedwait(&nccond, nclock, 5 * MOS_SEC /* nsec */);
	}

	state = TS_STOPPED;
	mos_cond_broadcast(&nccond);
	mos_tlock_unlock(nclock);
	MOS_TASK_EXIT(0);
}

static MOS_TASK_RESULT
runNetworkControlEntry(void *arg) {
	NetworkControlEntry *nce;
	PhidgetReturnCode res;

#if ZEROCONF_SUPPORT
	mos_sockaddr_list_t *addrlist;
	const char *taddr;
	uint16_t port;
	char tmp[32];
	kv_t *txt;
	int i;
#endif

	nce = arg;

	mos_tlock_lock(nce->lock);
	nce->flags |= NCE_RUNNING;

	mos_task_setname("Phidget22 Network Control Entry Thread - %s", nce->name);
	ncloginfo("network control entry thread started - %s: 0x%08x", nce->name, mos_self());

	while (nce->flags & NCE_ENABLED) {
		if (nce->flags & NCE_CONNECTED)
			goto donelocked;

		/* we only autoconnect to device servers now */
		if (nce->serverType != PHIDGETSERVER_DEVICEREMOTE)
			goto donelocked;
		mos_tlock_unlock(nce->lock);

		/*
		 * If authentication failed, do not try to connect again.
		 */
		if (nce->flags & NCE_AUTHFAILED)
			goto done;

		ncloginfo("network control entry %s not connected (failures=%d)", nce->name, nce->failedconn);

		/*
		 * If the protocol versions do not match, do not try to connect again.
		 */
		if (nce->flags & NCE_BADVERSION)
			goto done;

#if ZEROCONF_SUPPORT
		if ((nce->flags & NCE_MDNS) && nce->handle) {
			/*
			 * We always lookup the name as it may change.
			 * If the lookup fails, we do not fall back to the old address.
			 */
			nce->flags &= ~NCE_LOOKUPF;
			addrlist = NULL;
			res = Zeroconf_lookup(nce->handle, nce->interface, ZCP_IPv4, nce->name, nce->host, nce->type,
			  nce->domain, ZCP_IPv4, &addrlist, &port, &txt);
			if (res != EPHIDGET_OK) {
				nce->failedconn++;
				nce->flags |= NCE_LOOKUPF;
				nclogerr("Zeroconf_lookup() failed for host '%s [%s]' on interface 0x%x: "PRC_FMT, nce->name, nce->host, nce->interface, PRC_ARGS(res));
				goto done;
			}

			// Update the port
			nce->port = port;
			taddr = mos_ntoa(&addrlist->addr, tmp, sizeof(tmp));

			// Now that we've completed the lookup, run the server added event
			if ((nce->flags & NCE_EVADDED) == 0) {
				deviceServerAdded(nce->name, nce->host, addrlist, port, txt);
				nce->flags |= NCE_EVADDED;
			}

			mos_freeaddrlist(addrlist);

			i = kvgeti32(txt, "txtvers", 0);
			if (i == PHIDGET_MDNS_TXTVER) {
				i = kvgeti32(txt, "protocolmajor", 0);
				if (i != PHIDGET_NET_PROTOCOL_MAJOR) {
					ncloginfo("server %s (%s:%d) boardcasting protocol version %d: ignoring server", nce->name, taddr, nce->port, i);
					nce->flags |= NCE_BADVERSION;
					goto done;
				}
			} else {
				ncloginfo("server %s (%s:%d) boardcasting txtvers version %d: ignoring server", nce->name, taddr, nce->port, i);
				nce->flags |= NCE_BADVERSION;
				goto done;
			}

			ncloginfo("network control entry %s: connecting to %s (%s:%d)", nce->name, nce->host, taddr, nce->port);
			res = clientConnect(nce->proto, taddr, nce->port, nce->passwd, PHIDGET_NET_PROTOCOL,
			  PHIDGET_NET_PROTOCOL_MAJOR, PHIDGET_NET_PROTOCOL_MINOR, handleDeviceClientRequest,
			  nce, &nce->nc);
			if (res != EPHIDGET_OK) {
				nce->failedconn++;
				ncloginfo("failed to connect to MDNS server %s [%s] (%s:%d): "PRC_FMT, nce->name, nce->host, taddr, nce->port, PRC_ARGS(res));
				goto done;
			}
		} else
#endif
		{	/* ! NCE_MDNS */
			res = clientConnect(nce->proto, nce->host, nce->port, nce->passwd, PHIDGET_NET_PROTOCOL,
			  PHIDGET_NET_PROTOCOL_MAJOR, PHIDGET_NET_PROTOCOL_MINOR, handleDeviceClientRequest,
			  nce, &nce->nc);
			if (res != EPHIDGET_OK) {
				nclogwarn("failed to connect to server %s (%s:%d): "PRC_FMT, nce->name, nce->host, nce->port, PRC_ARGS(res));
				nce->failedconn++;
				goto done;
			}
		}
		nce->waittime = WAITTIME_INIT;
		nce->failedconn = 0;
		nce->nc->private = nce;	/* for clientClosed() */
		nce->flags |= NCE_CONNECTED;
		nce->flags |= NCE_WASCONN;
		ncloginfo("network control entry %s connected to %s:%d", nce->name, nce->host, nce->port);

	done:	mos_tlock_lock(nce->lock);

		if ((nce->flags & NCE_CONNECTED) == 0) {
			/*
			 * Stop trying to connect if we were unable to lookup the server.
			 */
			// XXX - this could happen if server machine IP address is changed. Keep trying to lookup.
			//if (nce->flags & NCE_LOOKUPF)
			//	break;

			/*
			 * Stop if an MDNS discovered server was undiscovered, and we failed to connect.
			 */
			if ((nce->flags & (NCE_MDNS | NCE_DISCOVERED)) == NCE_MDNS)
				break;
		}

	donelocked:
		if (nce->flags & NCE_ENABLED) {
			if (nce->flags & NCE_CONNECTED) {
				mos_tlock_wait(&nce->cond, nce->lock);
			} else {
				if (nce->waittime < WAITTIME_INIT)
					nce->waittime = WAITTIME_INIT;
				nclogdebug("%s: waiting %u (0x%x)", nce->name, (uint32_t)(nce->waittime / 1000000), nce->flags);
				mos_tlock_timedwait(&nce->cond, nce->lock, nce->waittime);
				if (nce->flags & NCE_MANUAL) {
					nce->waittime += MOS_SEC;
					if (nce->waittime > WAITTIME_MAXMAN)
						nce->waittime = WAITTIME_MAXMAN;
				} else {
					nce->waittime *= 2;
					if (nce->waittime > WAITTIME_MAX)
						nce->waittime = WAITTIME_MAX;
				}
				nclogverbose("%s: woke %u", nce->name, (uint32_t)(nce->waittime / 1000000));
			}
			nclogdebug("%s: woke", nce->name);
		}
	}

	mos_tlock_unlock(nce->lock);
	if ((nce->flags & NCE_ENABLED) == 0)
		ncloginfo("network control entry %s disabled (0x%x)", nce->name, nce->flags);
	else
		ncloginfo("network control entry %s failing (0x%x)", nce->name, nce->flags);

	/*
	 * We close after unlocking because clientClosed() may be called as a result.
	 */
	if (nce->nc) {
		stopAndWaitForPhidgetNetConnThread(nce->nc);
		PhidgetRelease(&nce->nc);
	}

	mos_tlock_lock(nce->lock);
	nce->flags &= ~NCE_RUNNING;
	mos_cond_broadcast(&nce->cond);
	mos_tlock_unlock(nce->lock);

	decPhidgetStat("server.netcontrol.entrytasks");
	MOS_TASK_EXIT(0);
}

/*
 * Called when a connection to a server closes.
 */
void
clientClosed(PhidgetNetConnHandle nc) {
	NetworkControlEntry *nce;

	nce = nc->private;

	/*
	 * Ensure this is actually for us.
	 */
	if (nce == NULL || nce->magic != NCE_MAGIC)
		return;

	nclogdebug("%s: closed", nce->name);

	mos_tlock_lock(nce->lock);
	nce->flags &= ~NCE_CONNECTED;
	nce->nc = NULL;
	nc->private = NULL;
	mos_cond_broadcast(&nce->cond);
	mos_tlock_unlock(nce->lock);
}

/*
 * Called when a client failed to authenticate with the server.
 */
void
clientAuthFailed(PhidgetNetConnHandle nc) {
	NetworkControlEntry *nce;

	nce = nc->private;

	/*
	 * Ensure this is actually for us.
	 */
	if (nce == NULL || nce->magic != NCE_MAGIC) {
		ncloginfo("client auth failed");
		return;
	}
	ncloginfo("client auth failed for %s", nce->name);

	mos_tlock_lock(nce->lock);
	nce->flags |= NCE_AUTHFAILED;
	mos_tlock_unlock(nce->lock);
}

/*
 * Called when a client protocol version did not match the servers.
 */
void
clientProtocolFailure(PhidgetNetConnHandle nc) {
	NetworkControlEntry *nce;

	nce = nc->private;

	/*
	 * Ensure this is actually for us.
	 */
	if (nce == NULL || nce->magic != NCE_MAGIC)
		return;

	mos_tlock_lock(nce->lock);
	nce->flags |= NCE_BADVERSION;
	mos_tlock_unlock(nce->lock);
}

const char *
clientGetHostName(PhidgetNetConnHandle nc) {
	NetworkControlEntry *nce;

	nce = nc->private;

	/*
	* Ensure this is actually for us.
	*/
	if (nce == NULL || nce->magic != NCE_MAGIC)
		return (NULL);

	return (nce->host);
}

/*
 * Returns the control entry locked.
 */
static void
getNetworkControlEntry(const char *name, NetworkControlEntry **nce) {

	mos_tlock_lock(nclock);
	MTAILQ_FOREACH(*nce, &entries, link) {
		if (strcmp((*nce)->name, name) == 0) {
			mos_tlock_unlock(nclock);
			mos_tlock_lock((*nce)->lock);
			return;
		}
	}
	mos_tlock_unlock(nclock);
	*nce = NULL;
}

#if ZEROCONF_SUPPORT
void
releaseMDNSControlEntries(ZeroconfListenerHandle handle) {
	NetworkControlEntry *nce, *tmpnce;

	mos_tlock_lock(nclock);
	MTAILQ_FOREACH_SAFE(nce, &entries, link, tmpnce) {
		mos_tlock_lock(nce->lock);
		if ((nce->flags & NCE_MDNS) && nce->handle == handle) {
			nce->flags &= ~NCE_ENABLED;
			nce->flags |= NCE_DELETE;
		}
		mos_tlock_unlock(nce->lock);
	}
	mos_tlock_unlock(nclock);
}
#endif

/* nce->lock held by caller */
static PhidgetReturnCode
enableNetworkControlEntry(NetworkControlEntry *nce) {
	int err;

	nce->flags |= NCE_ENABLED;
	if (nce->flags & NCE_RUNNING)
		return (EPHIDGET_OK);

	err = mos_task_create(&nce->self, runNetworkControlEntry, nce);
	if (err == 0) {
		incPhidgetStat("server.netcontrol.entrytasks_ever");
		incPhidgetStat("server.netcontrol.entrytasks");
	}
	return (err);
}

/*
 * Returns _nce locked if 'lock' is non-zero.
 */
static PhidgetReturnCode
_addPhidgetServer(PhidgetServerType srvt, int flags, int lock, mos_af_t proto, const char *name,
  const char *host, int port, const char *passwd, NetworkControlEntry **_nce) {
	NetworkControlEntry *nce;
	PhidgetReturnCode res;

	TESTPTR(name);
	TESTPTR(passwd);
	TESTPTR(_nce);

	if (!validServerName(name)) {
		nclogerr("invalid server name '%s'", name);
		return (EPHIDGET_INVALIDARG);
	}

	getNetworkControlEntry(name, &nce);
	if (nce) {
		mos_tlock_unlock(nce->lock);
		return (EPHIDGET_DUPLICATE);
	}

	nce = mos_zalloc(sizeof(*nce));
	nce->waittime = WAITTIME_INIT;
	nce->magic = NCE_MAGIC;
	nce->flags = flags;
	nce->serverType = srvt;
	nce->passwd = mos_strdup(passwd, NULL);
	nce->proto = proto;
	nce->lock = mos_tlock_create(P22LOCK_NCTLENTLOCK, P22LOCK_FLAGS);
	mos_cond_init(&nce->cond);
	nce->name = mos_strdup(name, NULL);
	if (host) {
		nce->host = mos_strdup(host, NULL);
		nce->type = mos_strdup(PHIDGET_NETWORK_MDNS_DEVICE, NULL);
		nce->domain = mos_strdup("local.", NULL);
	} else {
		nce->host = NULL;
		nce->type = NULL;
		nce->domain = NULL;
	}
	nce->port = port;
	nce->nc = NULL;

	if (nce->flags & NCE_ENABLED) {
		/*
		 * Lock not required here, since we are not in the list yet and can only be see by this thread.
		 */
		res = enableNetworkControlEntry(nce);
		if (res != 0) {
			freeNetworkControlEntry(&nce);
			return (res);
		}
	}

	/* Task started before being added to the list */
	mos_tlock_lock(nclock);
	MTAILQ_INSERT_HEAD(&entries, nce, link);
	mos_tlock_unlock(nclock);

	/* Entry is returned locked, if requested */
	if (lock)
		mos_tlock_lock(nce->lock);

	*_nce = nce;

	return (EPHIDGET_OK);
}

#if ZEROCONF_SUPPORT
PhidgetReturnCode
PhidgetNet_undiscoveredServer(const char *name) {
	NetworkControlEntry *nce;

	ncloginfo("UnDiscovered Server: %s", name);
	getNetworkControlEntry(name, &nce);
	if (nce == NULL)
		return (EPHIDGET_NOENT);

	nce->flags &= ~NCE_DISCOVERED;
	mos_cond_broadcast(&nce->cond);
	mos_tlock_unlock(nce->lock);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
PhidgetNet_discoveredServer(ZeroconfListenerHandle handle, int flags, int interface, PhidgetServerType srvt,
  const char *name, const char *host, const char *type, const char *domain, int port) {
	NetworkControlEntry *nce;
	PhidgetReturnCode res;

	TESTPTR(host);
	TESTPTR(type);
	TESTPTR(domain);

	ncloginfo("Discovered Server: %s", name);
	getNetworkControlEntry(name, &nce);
	if (nce != NULL) {
		nce->flags |= NCE_DISCOVERED;

		if ((nce->flags & NCE_VIRTUAL) == 0) {
			/*
			 * If it is an MDNS server reappearing, enable it and move on.
			 * Set the waittime to fast as the server might just be restarting.
			 */
			if (nce->flags & NCE_MDNS) {
				nce->waittime = WAITTIME_INIT;
				nce->failedconn = 0;
				nce->flags &= ~NCE_ERRORS;	/* clear if we see the server again */
				nce->flags &= ~NCE_EVADDED;	/* clear if we see the server again */

				if ((nce->flags & (NCE_ENABLED | NCE_RUNNING)) == (NCE_ENABLED | NCE_RUNNING)) {
					mos_cond_broadcast(&nce->cond);
					mos_tlock_unlock(nce->lock);
					ncloginfo("network control entry for %s rediscovered", name);
					return (EPHIDGET_OK);
				}

				ncloginfo("network control entry for %s rediscovered (enabling)", name);
				if (nce->name && mos_strcmp(nce->name, name) != 0) {
					mos_free(nce->name, MOSM_FSTR);
					mos_free(nce->host, MOSM_FSTR);
					mos_free(nce->type, MOSM_FSTR);
					mos_free(nce->domain, MOSM_FSTR);
					nce->name = mos_strdup(name, NULL);
					nce->host = mos_strdup(host, NULL);
					nce->type = mos_strdup(type, NULL);
					nce->domain = mos_strdup(domain, NULL);
				}
				nce->port = port;
				nce->interface = interface;
				nce->handle = handle;
				enableNetworkControlEntry(nce);
				mos_tlock_unlock(nce->lock);
				return (EPHIDGET_OK);
			}
			mos_tlock_unlock(nce->lock);
			nclogerr("non-MDNS server already exists '%s'", host);
			return (EPHIDGET_EXIST);
		}

		/*
		 * Virtual upgrade.
		 */
		nce->waittime = WAITTIME_INIT;
		nce->failedconn = 0;
		nce->flags &= ~NCE_VIRTUAL;
		nce->flags |= NCE_MDNS;
		nce->serverType = srvt;
		nce->interface = interface;
		nce->handle = handle;
		nce->proto = MOS_AF_INET4;
		nce->name = mos_strdup(name, NULL);
		nce->host = mos_strdup(host, NULL);
		nce->type = mos_strdup(type, NULL);
		nce->domain = mos_strdup(domain, NULL);
		nce->port = port;
		nce->nc = NULL;

		enableNetworkControlEntry(nce);
		mos_tlock_unlock(nce->lock);

		return (EPHIDGET_OK);
	}

	flags &= PHIDGET_NETWORK_FLAGMASK;
	res = _addPhidgetServer(srvt, flags | NCE_MDNS | NCE_DISCOVERED, 1,
	  MOS_AF_INET4, name, host, port, "", &nce);
	if (res != EPHIDGET_OK) {
		nclogerr("failed to add MDNS discovered server '%s'", host);
		return (res);
	}

	nce->waittime = WAITTIME_INIT;
	nce->failedconn = 0;
	nce->interface = interface;
	nce->handle = handle;
	nce->proto = MOS_AF_INET4;

	enableNetworkControlEntry(nce);
	mos_tlock_unlock(nce->lock);

	return (EPHIDGET_OK);
}
#endif

static void
freeNetworkControlEntry(NetworkControlEntry **_nce) {
	NetworkControlEntry *nce;

	nce = *_nce;

	if (nce->passwd)
		mos_free(nce->passwd, MOSM_FSTR);
	if (nce->name)
		mos_free(nce->name, MOSM_FSTR);
	if (nce->host)
		mos_free(nce->host, MOSM_FSTR);
	if (nce->type)
		mos_free(nce->type, MOSM_FSTR);
	if (nce->domain)
		mos_free(nce->domain, MOSM_FSTR);
	mos_tlock_destroy(&nce->lock);
	mos_cond_destroy(&nce->cond);
	mos_free(nce, sizeof(*nce));
	*_nce = NULL;
}

const char *
getNetworkControlEntryName(void *nce) {

	if (nce == NULL)
		return (NULL);

	return (((NetworkControlEntry *)nce)->name);
}

/** Exported Public API **/

API_PRETURN
PhidgetNet_addServer(const char *name, const char *host, int port, const char *passwd, int flags) {
	NetworkControlEntry *nce;
	PhidgetReturnCode res;

	TESTPTR_PR(host);
	TESTPTR_PR(passwd);

	PhidgetNet_start(); /* reference the network layer */

	flags &= PHIDGET_NETWORK_FLAGMASK;

	res = _addPhidgetServer(PHIDGETSERVER_DEVICEREMOTE, flags | NCE_ENABLED | NCE_MANUAL, 0, MOS_AF_INET4,
	  name, host, port, passwd, &nce);
	if (res != EPHIDGET_OK) {
		nclogerr("failed to add server '%s'", host);
		return (PHID_RETURN(res));
	}

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetNet_removeServer(const char *name) {
	NetworkControlEntry *nce;

	getNetworkControlEntry(name, &nce);
	if (nce == NULL)
		return (EPHIDGET_OK);

	nce->flags &= ~NCE_ENABLED;
	nce->flags |= NCE_DELETE;
	mos_cond_broadcast(&nce->cond);
	mos_tlock_unlock(nce->lock);

	/*
	 * Remove reference to the network layer if the server was added with _addServer().
	 */
	if ((nce->flags & NCE_MDNS) == 0)
		PhidgetNet_stop();

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetNet_removeAllServers() {
	NetworkControlEntry *nce;

next:
	mos_tlock_lock(nclock);
	MTAILQ_FOREACH(nce, &entries, link) {
		if ((nce->flags & NCE_DELETE) == NCE_DELETE)
			continue;

		mos_tlock_unlock(nclock);

		mos_tlock_lock(nce->lock);
		nce->flags &= ~NCE_ENABLED;
		nce->flags |= NCE_DELETE;
		mos_cond_broadcast(&nce->cond);
		mos_tlock_unlock(nce->lock);

		/*
		* Remove reference to the network layer if the server was added with _addServer().
		*/
		if ((nce->flags & NCE_MDNS) == 0)
			PhidgetNet_stop();

		goto next;
	}
	mos_tlock_unlock(nclock);

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetNet_setServerPassword(const char *name, const char *passwd) {
	NetworkControlEntry *nce;
	PhidgetReturnCode res;

	CHECKSTARTED_PR;

	TESTPTR_PR(passwd);
	TESTPTR_PR(name);

	getNetworkControlEntry(name, &nce);
	if (nce) {
		nce->flags &= ~NCE_AUTHFAILED;
		if (nce->passwd)
			mos_free(nce->passwd, mos_strlen(nce->passwd) + 1);
		nce->passwd = mos_strdup(passwd, NULL);
		mos_cond_broadcast(&nce->cond);
		mos_tlock_unlock(nce->lock);
		return (EPHIDGET_OK);
	}

	/* returns new nce locked if successful */
	res = _addPhidgetServer(PHIDGETSERVER_NONE, NCE_VIRTUAL, 0, MOS_AF_INET4, name, NULL, 0, passwd, &nce);
	if (res != 0)
		return (PHID_RETURN(res));

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetNet_enableServer(const char *name) {
	NetworkControlEntry *nce;
	PhidgetReturnCode res;

	CHECKSTARTED_PR;

	getNetworkControlEntry(name, &nce);
	if (nce == NULL)
		return (PHID_RETURN(EPHIDGET_NOENT));

	if (nce->flags & NCE_VIRTUAL) {
		mos_tlock_unlock(nce->lock);
		return (EPHIDGET_OK);
	}

	nce->flags &= ~NCE_AUTHFAILED;
	nce->flags &= ~NCE_BADVERSION;
	nce->waittime = WAITTIME_INIT;
	nce->failedconn = 0;
	res = enableNetworkControlEntry(nce);
	mos_cond_broadcast(&nce->cond);	/* wake up the nce task is it is sleeping */
	mos_tlock_unlock(nce->lock);
	return (PHID_RETURN(res));
}

API_PRETURN
PhidgetNet_disableServer(const char *name, int flags) {
	NetworkControlEntry *nce;
	PhidgetReturnCode res;

	CHECKSTARTED_PR;

	getNetworkControlEntry(name, &nce);
	if (nce) {
		nce->flags &= ~NCE_ENABLED;

		/* clear error flags if the server unpublished */
		if (flags & PHIDGET_NETWORK_MDNSDETACH) {
			nce->flags &= ~NCE_AUTHFAILED;
			nce->flags &= ~NCE_BADVERSION;
		}

		mos_cond_broadcast(&nce->cond);
		mos_tlock_unlock(nce->lock);
		return (EPHIDGET_OK);
	}

	/* Attempt to add a placeholder if it doesn't exist already */
	res = _addPhidgetServer(PHIDGETSERVER_NONE, 0, 0, MOS_AF_INET4, name, NULL, 0, "", &nce);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetNet_getServerAddressList(const char *hostname, int addressFamily, char *addressList[], uint32_t *count) {

#if ZEROCONF_SUPPORT
	mos_sockaddr_list_t *addrlist, *ptr;
	Zeroconf_Protocol proto;
	PhidgetReturnCode res;
	char addrbuf[64];
	uint32_t cnt, i;

	switch (addressFamily) {
	case AF_UNSPEC:
		proto = ZCP_UNSPEC;
		break;
	case AF_INET:
		proto = ZCP_IPv4;
		break;
	case AF_INET6:
		proto = ZCP_IPv6;
		break;
	default:
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Invalid addressFamily."));
	}

	res = Zeroconf_addr_lookup(hostname, proto, &addrlist);
	if (res != EPHIDGET_OK)
		return (PHID_RETURN(res));

	ptr = addrlist;
	cnt = 0;
	while (ptr) {
		cnt++;
		ptr = ptr->next;
	}

	if (addressList != NULL) {
		ptr = addrlist;
		i = 0;
		while (ptr) {
			addressList[i] = mos_strdup(mos_ntoa(&ptr->addr, addrbuf, sizeof(addrbuf)), NULL);
			ptr = ptr->next;
			i++;
			if (i >= *count)
				break;
		}
		*count = i;
	} else {
		*count = cnt;
	}

	mos_freeaddrlist(addrlist);

	return (EPHIDGET_OK);
#else
	return (EPHIDGET_UNSUPPORTED);
#endif
}

API_PRETURN
PhidgetNet_freeServerAddressList(char *addressList[], uint32_t count) {
	uint32_t i;

	for (i = 0; i < count; i++)
		if (addressList[i] != NULL)
			mos_free(addressList[i], MOSM_FSTR);

	return (EPHIDGET_OK);
}
