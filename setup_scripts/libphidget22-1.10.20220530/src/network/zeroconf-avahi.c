/*
 * This file is part of libphidget21
 *
 * Copyright 2006-2015 Phidgets Inc <patrick@phidgets.com>
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
#include "zeroconf.h"

#include "avahi-common/thread-watch.h"
#include "avahi-common/malloc.h"
#include "avahi-client/client.h"
#include "avahi-client/lookup.h"

#include <dlfcn.h>

static AvahiThreadedPoll *threaded_poll;
static AvahiClient		*client;

static int			initialized;
static int			zcstarted;
static void 		*libHandle;
static int			avstate;
static mos_mutex_t	lock;
static mos_cond_t	cond;

static void entry_group_callback(AvahiEntryGroup *, AvahiEntryGroupState, void *);

typedef struct _wrapper {
	PhidgetReturnCode		res;
	int						more;
	ZeroconfListenerHandle	handle;
	void					*arg0;
	void					*arg1;
	uint16_t				port;
	mos_mutex_t				lock;
	mos_cond_t				cond;
} wrapper_t;

typedef struct zcdisp {
	ZeroconfListener_t	listener;
	void				 *handle;
	void				*ctx;
	int					added;
	int					interface;
	Zeroconf_Protocol	protocol;
	char				*name;
	char				*host;
	char				*type;
	char				*domain;
	MSLIST_ENTRY(zcdisp)	link;
} zcdisp_t;

MSLIST_HEAD(displisthead, zcdisp) displist = MSLIST_HEAD_INITIALIZER(displist);
static int			disprunning;
static mos_mutex_t	displock;
static mos_cond_t	dispcond;

#ifdef ZEROCONF_RUNTIME_LINKING
typedef AvahiClient *(*avahi_client_new_t)(const AvahiPoll *, AvahiClientFlags, AvahiClientCallback, void *,
  int *);
typedef void (*avahi_client_free_t)(AvahiClient *);
typedef const char *(*avahi_client_get_host_name_t)(AvahiClient *);
typedef AvahiServiceBrowser *(*avahi_service_browser_new_t)(AvahiClient *, AvahiIfIndex, AvahiProtocol,
  const char *, const char *, AvahiLookupFlags, AvahiServiceBrowserCallback, void *);
typedef int (*avahi_service_browser_free_t)(AvahiServiceBrowser *);
typedef AvahiServiceResolver *(*avahi_service_resolver_new_t)(AvahiClient *, AvahiIfIndex, AvahiProtocol,
  const char *, const char *, const char *, AvahiProtocol, AvahiLookupFlags, AvahiServiceResolverCallback,
  void *);
typedef int (*avahi_service_resolver_free_t)(AvahiServiceResolver *);
typedef AvahiRecordBrowser *(*avahi_record_browser_new_t)(AvahiClient *, AvahiIfIndex, AvahiProtocol,
  const char *, uint16_t, uint16_t, AvahiLookupFlags, AvahiRecordBrowserCallback , void *);
typedef int (*avahi_record_browser_free_t)(AvahiRecordBrowser *);
typedef int (*avahi_service_name_join_t)(char *, size_t, const char *, const char *, const char *);
typedef const char *(*avahi_strerror_t)(int);
typedef int (*avahi_client_errno_t) (AvahiClient *);
typedef AvahiThreadedPoll *(*avahi_threaded_poll_new_t)(void);
typedef const AvahiPoll *(*avahi_threaded_poll_get_t)(AvahiThreadedPoll *);
typedef void (*avahi_threaded_poll_free_t)(AvahiThreadedPoll *);
typedef int (*avahi_threaded_poll_start_t)(AvahiThreadedPoll *);
typedef int (*avahi_threaded_poll_stop_t)(AvahiThreadedPoll *);
typedef void (*avahi_threaded_poll_lock_t)(AvahiThreadedPoll *);
typedef void (*avahi_threaded_poll_unlock_t)(AvahiThreadedPoll *);
typedef const char *(*avahi_client_get_version_string_t)(AvahiClient *);
typedef void (*avahi_free_t)(void *p);
typedef AvahiStringList *(*avahi_string_list_new_t)(const char *, ...);
typedef void (*avahi_string_list_free_t)(AvahiStringList *);
typedef AvahiStringList *(*avahi_string_list_get_next_t)(AvahiStringList *);
typedef int (*avahi_string_list_get_pair_t)(AvahiStringList *, char **, char **, size_t *);
typedef AvahiStringList *(*avahi_string_list_add_pair_t)(AvahiStringList *, const char *, const char *);
typedef AvahiEntryGroup *(*avahi_entry_group_new_t)(AvahiClient *, AvahiEntryGroupCallback, void *);
typedef int (*avahi_entry_group_free_t)(AvahiEntryGroup *);
typedef int (*avahi_entry_group_commit_t)(AvahiEntryGroup *);
typedef int (*avahi_entry_group_reset_t)(AvahiEntryGroup *);
typedef AvahiClient *(*avahi_entry_group_get_client_t)(AvahiEntryGroup *);
typedef int (*avahi_entry_group_get_state_t)(AvahiEntryGroup *);
typedef int (*avahi_entry_group_add_service_t)(AvahiEntryGroup *, AvahiIfIndex, AvahiProtocol,
  AvahiPublishFlags, const char *, const char *, const char *, const char *, uint16_t, ...);
typedef int (*avahi_entry_group_add_service_strlst_t)(AvahiEntryGroup *, AvahiIfIndex, AvahiProtocol,
  AvahiPublishFlags, const char *, const char *, const char *, const char *, uint16_t, AvahiStringList *);
typedef char *(*avahi_alternative_service_name_t)(const char *);

static avahi_service_browser_new_t			_service_browser_new;
static avahi_service_browser_free_t			_service_browser_free;
static avahi_service_resolver_new_t			_service_resolver_new;
static avahi_service_resolver_free_t		_service_resolver_free;
static avahi_record_browser_new_t			_record_browser_new;
static avahi_record_browser_free_t			_record_browser_free;
static avahi_service_name_join_t			_service_name_join;
static avahi_client_new_t					_client_new;
static avahi_client_free_t					_client_free;
static avahi_strerror_t						_strerror;
static avahi_client_errno_t					_client_errno;
static avahi_threaded_poll_new_t			_threaded_poll_new;
static avahi_threaded_poll_get_t			_threaded_poll_get;
static avahi_threaded_poll_free_t			_threaded_poll_free;
static avahi_threaded_poll_start_t			_threaded_poll_start;
static avahi_threaded_poll_stop_t			_threaded_poll_stop;
static avahi_threaded_poll_lock_t			_threaded_poll_lock;
static avahi_threaded_poll_unlock_t			_threaded_poll_unlock;
static avahi_client_get_version_string_t	_client_get_version_string;
static avahi_free_t							_free;
static avahi_string_list_new_t				_string_list_new;
static avahi_string_list_free_t				_string_list_free;
static avahi_string_list_get_next_t			_string_list_get_next;
static avahi_string_list_get_pair_t			_string_list_get_pair;
static avahi_string_list_add_pair_t			_string_list_add_pair;
static avahi_entry_group_new_t				_entry_group_new;
static avahi_entry_group_free_t				_entry_group_free;
static avahi_entry_group_commit_t			_entry_group_commit;
static avahi_entry_group_reset_t			_entry_group_reset;
static avahi_entry_group_add_service_t		_entry_group_add_service;
static avahi_entry_group_add_service_strlst_t	_entry_group_add_service_strlst;
static avahi_entry_group_get_client_t		_entry_group_get_client;
static avahi_entry_group_get_state_t		_entry_group_get_state;

#else /* ZEROCONF_RUNTIME_LINKING */

#define _service_browser_new			avahi_service_browser_new
#define _service_browser_free			avahi_service_browser_free
#define _service_resolver_new			avahi_service_resolver_new
#define _service_resolver_free			avahi_service_resolver_free
#define _record_browser_new				avahi_record_browser_new
#define _record_browser_free			avahi_record_browser_free
#define _service_name_join				avahi_service_name_join
#define _client_new						avahi_client_new
#define _client_free					avahi_client_free
#define _strerror						avahi_strerror
#define _client_errno					avahi_client_errno
#define _threaded_poll_new				avahi_threaded_poll_new
#define _threaded_poll_get				avahi_threaded_poll_get
#define _threaded_poll_free				avahi_threaded_poll_free
#define _threaded_poll_start			avahi_threaded_poll_start
#define _threaded_poll_stop				avahi_threaded_poll_stop
#define _threaded_poll_lock				avahi_threaded_poll_lock
#define _threaded_poll_unlock			avahi_threaded_poll_unlock
#define _client_get_version_string		avahi_client_get_version_string
#define _free							avahi_free
#define _string_list_new				avahi_string_list_new
#define _string_list_free				avahi_string_list_free
#define _string_list_get_next			avahi_string_list_get_next
#define _string_list_get_pair			avahi_string_list_get_pair
#define _string_list_add_pair			avahi_string_list_add_pair
#define _entry_group_new				avahi_entry_group_new
#define _entry_group_free				avahi_entry_group_free
#define _entry_group_commit				avahi_entry_group_commit
#define _entry_group_get_client			avahi_entry_group_get_client
#define _entry_group_get_state			avahi_entry_group_get_state
#define _entry_group_reset				avahi_entry_group_reset
#define _entry_group_add_service		avahi_entry_group_add_service
#define _entry_group_add_service_strlst	avahi_entry_group_add_service_strlst

#endif /* ZEROCONF_RUNTIME_LINKING */

static void client_callback(AvahiClient *, AvahiClientState, AVAHI_GCC_UNUSED void *);

static PhidgetReturnCode ZeroconfLoad(void);

static void
listener_dispatch_thread(void *arg) {
	zcdisp_t *dp;

	mos_mutex_lock(&displock);
	while (disprunning) {
		if (MSLIST_EMPTY(&displist)) {
			mos_cond_timedwait(&dispcond, &displock, MOS_SEC);
			continue;
		}

		dp = MSLIST_FIRST(&displist);
		MSLIST_REMOVE(&displist, dp, zcdisp, link);
		mos_mutex_unlock(&displock);
		dp->listener(dp->handle, dp->ctx, dp->added, dp->interface, dp->protocol, dp->name, dp->host,
		  dp->type, dp->domain);
		mos_free(dp->name, MOSM_FSTR);
		mos_free(dp->host, MOSM_FSTR);
		mos_free(dp->type, MOSM_FSTR);
		mos_free(dp->domain, MOSM_FSTR);
		mos_free(dp, sizeof (*dp));
		mos_mutex_lock(&displock);
	}
	disprunning = -1;
	mos_cond_broadcast(&dispcond);
	mos_mutex_unlock(&displock);
	mos_task_exit(0);
}

void
ZeroconfInit() {

	mos_glock((void *)1);
	if (initialized) {
		mos_gunlock((void *)1);
		return;
	}

	mos_mutex_init(&displock);
	mos_cond_init(&dispcond);
	mos_mutex_init(&lock);
	mos_cond_init(&cond);

	if (ZeroconfLoad() != 0) {
		mos_gunlock((void *)1);
		goto bad;
	}

	initialized = 1;
	mos_gunlock((void *)1);
	return;

bad:

	mos_mutex_destroy(&displock);
	mos_cond_destroy(&dispcond);
	mos_mutex_destroy(&lock);
	mos_cond_destroy(&cond);
}

void
ZeroconfFini() {

	mos_glock((void *)1);
	if (!initialized || zcstarted) {
		mos_gunlock((void *)1);
		return;
	}
	mos_gunlock((void *)1);

	loginfo("fini");

	initialized = 0;

	mos_mutex_destroy(&displock);
	mos_cond_destroy(&dispcond);
	mos_mutex_destroy(&lock);
	mos_cond_destroy(&cond);

#ifdef ZEROCONF_RUNTIME_LINKING
	dlclose(libHandle);
#endif
}

void
ZeroconfStart() {
	mos_task_t task;
	int error;

	mos_glock((void *)1);
	if (!initialized || zcstarted) {
		mos_gunlock((void *)1);
		return;
	}
	zcstarted = 1;
	mos_gunlock((void *)1);

	threaded_poll = _threaded_poll_new();
	if (threaded_poll == NULL) {
		logerr("Failed to create Avahi poll object");
		goto bad;
	}

	/*
	 * Locking shouldn't be required here, but we are being paranoid with Avahi.
	 */
	_threaded_poll_lock(threaded_poll);
	client = _client_new(_threaded_poll_get(threaded_poll), 0, client_callback, NULL, &error);
	_threaded_poll_unlock(threaded_poll);
	if (client == NULL) {
		logerr("Failed to create client: %s", _strerror(error));
		goto bad;
	}

	if (_threaded_poll_start(threaded_poll)) {
		logerr("Failed to start threaded_poll");
		goto bad;
	}

	disprunning = 1;
	if (mos_task_create(&task, listener_dispatch_thread, NULL) != 0) {
		logerr("Failed to create dns listener dispatch thread");
		goto bad;
	}

	return;

bad:
	mos_glock((void *)1);
	if (threaded_poll) {
		_threaded_poll_stop(threaded_poll);
		if (client) {
			_client_free(client);
			client = NULL;
		}
		_threaded_poll_free(threaded_poll);
		threaded_poll = NULL;
	}
	disprunning = 0;
	zcstarted = 0;
	mos_gunlock((void *)1);
}

void
ZeroconfStop() {

	mos_glock((void *)1);
	if (!initialized || !zcstarted) {
		mos_gunlock((void *)1);
		return;
	}
	mos_gunlock((void *)1);

	mos_mutex_lock(&displock);
	if (disprunning == 1) {
		disprunning = 0;
		while (disprunning != -1)
			mos_cond_wait(&dispcond, &displock);
	}
	mos_mutex_unlock(&displock);

	_threaded_poll_stop(threaded_poll);
	_client_free(client);
	client = NULL;
	_threaded_poll_free(threaded_poll);
	threaded_poll = NULL;

	mos_glock((void *)1);
	zcstarted = 0;
	mos_gunlock((void *)1);
}


static void
dispatch(ZeroconfListener_t listener, void *handle, void *ctx, int added, int interface,
  Zeroconf_Protocol proto, const char *name, const char *host, const char *type, const char *domain) {
	zcdisp_t *dp;

	dp = mos_malloc(sizeof (*dp));
	dp->listener = listener;
	dp->handle = handle;
	dp->ctx = ctx;
	dp->added = added;
	dp->interface = interface;
	dp->protocol = proto;
	dp->name = mos_strdup(name, NULL);
	dp->host = mos_strdup(host, NULL);
	dp->type = mos_strdup(type, NULL);
	dp->domain = mos_strdup(domain, NULL);

	mos_mutex_lock(&displock);
	MSLIST_INSERT_HEAD(&displist, dp, link);
	mos_cond_broadcast(&dispcond);
	mos_mutex_unlock(&displock);
}

static PhidgetReturnCode
ZeroconfLoad() {

	if (initialized)
		return (EPHIDGET_OK);

#define CK(stmt) do {										\
	if ((stmt) == NULL) {                                   \
		loginfo("'%s' failed", #stmt);                      \
		loginfo("'%s' failed:%s", #stmt, dlerror());        \
		return (EPHIDGET_UNEXPECTED);                       \
	}                                                       \
} while (0)

	libHandle = dlopen("libavahi-client.so", RTLD_LAZY);
	if (!libHandle) {
		libHandle = dlopen("libavahi-client.so.3", RTLD_LAZY);
		if (!libHandle) {
			loginfo("dlopen() failed: %s", dlerror());
			loginfo("Zeroconf is not supported");
			return (EPHIDGET_UNSUPPORTED);
		}
	}

	CK(_service_browser_new = (avahi_service_browser_new_t)dlsym(libHandle, "avahi_service_browser_new"));
	CK(_service_browser_free = (avahi_service_browser_free_t)dlsym(libHandle, "avahi_service_browser_free"));
	CK(_service_resolver_new = (avahi_service_resolver_new_t)dlsym(libHandle, "avahi_service_resolver_new"));
	CK(_service_resolver_free = (avahi_service_resolver_free_t)dlsym(libHandle,
	  "avahi_service_resolver_free"));
	CK(_record_browser_new = (avahi_record_browser_new_t)dlsym(libHandle, "avahi_record_browser_new"));
	CK(_record_browser_free = (avahi_record_browser_free_t)dlsym(libHandle, "avahi_record_browser_free"));
	CK(_service_name_join = (avahi_service_name_join_t)dlsym(libHandle, "avahi_service_name_join"));
	CK(_client_new = (avahi_client_new_t)dlsym(libHandle, "avahi_client_new"));
	CK(_client_free = (avahi_client_free_t)dlsym(libHandle, "avahi_client_free"));
	CK(_strerror = (avahi_strerror_t)dlsym(libHandle, "avahi_strerror"));
	CK(_client_errno = (avahi_client_errno_t)dlsym(libHandle, "avahi_client_errno"));
	CK(_threaded_poll_new = (avahi_threaded_poll_new_t)dlsym(libHandle, "avahi_threaded_poll_new"));
	CK(_threaded_poll_get = (avahi_threaded_poll_get_t)dlsym(libHandle, "avahi_threaded_poll_get"));
	CK(_threaded_poll_free = (avahi_threaded_poll_free_t)dlsym(libHandle, "avahi_threaded_poll_free"));
	CK(_threaded_poll_start = (avahi_threaded_poll_start_t)dlsym(libHandle, "avahi_threaded_poll_start"));
	CK(_threaded_poll_stop = (avahi_threaded_poll_stop_t)dlsym(libHandle, "avahi_threaded_poll_stop"));
	CK(_threaded_poll_lock = (avahi_threaded_poll_lock_t)dlsym(libHandle, "avahi_threaded_poll_lock"));
	CK(_threaded_poll_unlock = (avahi_threaded_poll_lock_t)dlsym(libHandle, "avahi_threaded_poll_unlock"));
	CK(_client_get_version_string = (avahi_client_get_version_string_t)dlsym(libHandle,
	  "avahi_client_get_version_string"));
	CK(_free = (avahi_free_t)dlsym(libHandle, "avahi_free"));
	CK(_string_list_new = (avahi_string_list_new_t)dlsym(libHandle, "avahi_string_list_new"));
	CK(_string_list_free = (avahi_string_list_free_t)dlsym(libHandle, "avahi_string_list_free"));
	CK(_string_list_get_next = (avahi_string_list_get_next_t)dlsym(libHandle, "avahi_string_list_get_next"));
	CK(_string_list_get_pair = (avahi_string_list_get_pair_t)dlsym(libHandle, "avahi_string_list_get_pair"));
	CK(_string_list_add_pair = (avahi_string_list_add_pair_t)dlsym(libHandle, "avahi_string_list_add_pair"));

	CK(_entry_group_new = (avahi_entry_group_new_t)dlsym(libHandle, "avahi_entry_group_new"));
	CK(_entry_group_free = (avahi_entry_group_free_t)dlsym(libHandle, "avahi_entry_group_free"));
	CK(_entry_group_commit = (avahi_entry_group_commit_t)dlsym(libHandle, "avahi_entry_group_commit"));
	CK(_entry_group_reset = (avahi_entry_group_reset_t)dlsym(libHandle, "avahi_entry_group_reset"));
	CK(_entry_group_add_service = (avahi_entry_group_add_service_t)dlsym(libHandle,
	  "avahi_entry_group_add_service"));
	CK(_entry_group_add_service_strlst = (avahi_entry_group_add_service_strlst_t)dlsym(libHandle,
	  "avahi_entry_group_add_service_strlst"));
	CK(_entry_group_get_client = (avahi_entry_group_get_client_t)dlsym(libHandle,
	  "avahi_entry_group_get_client"));
	CK(_entry_group_get_state = (avahi_entry_group_get_state_t)dlsym(libHandle,
	  "avahi_entry_group_get_state"));

	return (EPHIDGET_OK);
}

static void
client_callback(AvahiClient *c, AvahiClientState state, AVAHI_GCC_UNUSED void *userdata) {

	assert(c);

	/* Called whenever the client or server state changes */

	mos_mutex_lock(&lock);
	avstate = state;
	mos_cond_broadcast(&cond);
	mos_mutex_unlock(&lock);

	switch (state) {
	case AVAHI_CLIENT_S_RUNNING:
		/*
		 * The server has startup successfully and registered its host
		 * name on the network
		 */
		logdebug("Avahi client is running");
		break;
	case AVAHI_CLIENT_FAILURE:
		logerr("Avahi client failure: %s", _strerror(_client_errno(c)));
		break;
	case AVAHI_CLIENT_S_COLLISION:
		/*
		 * Let's drop our registered services. When the server is back
		 * in AVAHI_SERVER_RUNNING state we will register them
		 * again with the new host name.
		 */
		logdebug("Avahi client collision");
		break;
	case AVAHI_CLIENT_S_REGISTERING:
		/* The server records are now being established. This
		 * might be caused by a host name change. We need to wait
		 * for our own records to register until the host name is
		 * properly esatblished.
		 */
		logdebug("Avahi client registering");
		break;
	case AVAHI_CLIENT_CONNECTING:
		logdebug("Avahi client connecting");
		break;
	}
}

static void
DNSServiceBrowse_Callback(AvahiServiceBrowser *b, AvahiIfIndex interface, AvahiProtocol protocol,
  AvahiBrowserEvent event, const char *name, const char *type, const char *domain,
  AVAHI_GCC_UNUSED AvahiLookupResultFlags flags, void *ctx) {
	ZeroconfListenerHandle handle;

	handle = ctx;

	switch (event) {
	case AVAHI_BROWSER_FAILURE:
		logwarn("%s", _strerror(_client_errno(client)));
		return;

	case AVAHI_BROWSER_NEW:
		logdebug("NEW: service '%s' of type '%s' in domain '%s'", name, type, domain);
		dispatch(handle->listener, handle, handle->listenerctx, 1, interface, protocol, name, name,
		  type, domain);
		break;

	case AVAHI_BROWSER_REMOVE:
		logdebug("REMOVE: service '%s' of type '%s' in domain '%s'", name, type, domain);
		dispatch(handle->listener, handle, handle->listenerctx, 0, interface, protocol, name, name,
		  type, domain);
		break;

	case AVAHI_BROWSER_ALL_FOR_NOW:
	case AVAHI_BROWSER_CACHE_EXHAUSTED:
		logverbose("%s", event == AVAHI_BROWSER_CACHE_EXHAUSTED ? "CACHE_EXHAUSTED" : "ALL_FOR_NOW");
		break;
	}
}

PhidgetReturnCode
Zeroconf_listen(ZeroconfListenerHandle *_handle, const char *type, ZeroconfListener_t fptr, void *ctx) {
	ZeroconfListenerHandle handle;

	if (client == NULL) {
		logerr("client is not initialized");
		return (EPHIDGET_UNEXPECTED);
	}

	mos_mutex_lock(&lock);
	if (avstate != AVAHI_CLIENT_S_RUNNING) {
		mos_mutex_unlock(&lock);
		logerr("client is not connected to server");
		return (EPHIDGET_UNEXPECTED);
	}
	mos_mutex_unlock(&lock);

	handle = mos_malloc(sizeof (*handle));
	handle->flags = ZCL_RUN;
	handle->listener = fptr;
	handle->listenerctx = ctx;
	handle->type = mos_strdup(type, NULL);

	_threaded_poll_lock(threaded_poll);
	handle->sb = _service_browser_new(client, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC, type, NULL, 0,
	  DNSServiceBrowse_Callback, handle);
	_threaded_poll_unlock(threaded_poll);
	if (handle->sb == NULL) {
		Zeroconf_listenclose(&handle);
		logerr("failed to create service browser for '%s': %s", type, _strerror(_client_errno(client)));
		return (EPHIDGET_UNEXPECTED);
	}

	*_handle = handle;

	return (EPHIDGET_OK);
}

void
Zeroconf_listenclose(ZeroconfListenerHandle *_handle) {
	ZeroconfListenerHandle handle;

	if (_handle == NULL || *_handle == NULL)
		return;

	handle = *_handle;
	if (handle->sb) {
		_threaded_poll_lock(threaded_poll);
		_service_browser_free(handle->sb);
		_threaded_poll_unlock(threaded_poll);
	}

	mos_free(handle->type, MOSM_FSTR);
	mos_free(handle, sizeof (*handle));

	*_handle = NULL;
}

static void
DNSServiceResolve_Callback(AvahiServiceResolver *r, AVAHI_GCC_UNUSED AvahiIfIndex interface,
  AVAHI_GCC_UNUSED AvahiProtocol protocol, AvahiResolverEvent event, const char *name, const char *type,
  const char *domain, const char *host_name, const AvahiAddress *address, uint16_t port, AvahiStringList *txt,
  AvahiLookupResultFlags flags, void *ctx) {
	mos_sockaddr_list_t *addri, **next;
	mos_sockaddr_list_t **addrlist;
	AvahiStringList *cur;
	wrapper_t *wrapper;
	char *key, *val;
	kv_t **_kv, *kv;
	size_t size;

	wrapper = ctx;
	addrlist = wrapper->arg0;
	_kv = wrapper->arg1;

	mos_mutex_lock(&wrapper->lock);

	switch (event) {
	case AVAHI_RESOLVER_FAILURE:
		loginfo("Failed to resolve service '%s' of type '%s' in domain '%s': %s", name, type, domain,
		  _strerror(_client_errno(client)));
		wrapper->res = EPHIDGET_UNEXPECTED;
		break;
	case AVAHI_RESOLVER_FOUND:
		if (addrlist) {
			if (address->proto != AVAHI_PROTO_INET) {
				wrapper->res = EPHIDGET_UNSUPPORTED;
				break;
			}
			addri = mos_malloc(sizeof(mos_sockaddr_list_t));
			memset(addri, 0, sizeof(mos_sockaddr_list_t));

			if (*addrlist == NULL) {
				*addrlist = addri;
			} else {
				next = &(*addrlist)->next;
				while (*next)
					next = &(*next)->next;
				*next = addri;
			}

			// XXX - IPv6?
			addri->family = AF_INET;
			addri->addr.s4.sin_family = AF_INET;
			addri->addr.s4.sin_addr.s_addr = address->data.ipv4.address;
			addri->addr.s4.sin_port = port;
		}
		if (_kv) {
			if (txt != NULL) {
				newkv(&kv);
				cur = txt;
				do {
					_string_list_get_pair(cur, &key, &val, &size);
					if (val)
						kvset(kv, MOS_IOP_IGNORE, key, val);
					_free(key);
					if (val)
						_free(val);
				} while ((cur = _string_list_get_next(cur)) != NULL);
				*_kv = kv;
			}
		}
		wrapper->port = port;
		wrapper->res = EPHIDGET_OK;
		break;
	default:
		logerr("unexpected event:%d", event);
		wrapper->res = EPHIDGET_UNEXPECTED;
	}

	wrapper->more = 0;
	mos_cond_broadcast(&wrapper->cond);
	mos_mutex_unlock(&wrapper->lock);
}

PhidgetReturnCode
Zeroconf_addr_lookup(const char *host, Zeroconf_Protocol proto, mos_sockaddr_list_t **addrlist) {

	// XXX - maybe support at some point
	return (EPHIDGET_UNSUPPORTED);
}

PhidgetReturnCode
Zeroconf_lookup(ZeroconfListenerHandle handle, int interface, Zeroconf_Protocol proto, const char *name,
  const char *host, const char *type, const char *domain, Zeroconf_Protocol reqproto, mos_sockaddr_list_t **addrlist,
  uint16_t *port, kv_t **kv) {
	AvahiServiceResolver *r;
	PhidgetReturnCode res;
	wrapper_t wrapper;

	if (client == NULL) {
		logerr("client is not initialized");
		return (EPHIDGET_UNEXPECTED);
	}

	mos_mutex_lock(&lock);
	if (avstate != AVAHI_CLIENT_S_RUNNING) {
		mos_mutex_unlock(&lock);
		logerr("client is not connected to server");
		return (EPHIDGET_UNEXPECTED);
	}
	mos_mutex_unlock(&lock);

	if (addrlist == NULL && kv == NULL)
		return (EPHIDGET_INVALIDARG);

	if (proto != ZCP_IPv4)
		return (EPHIDGET_UNSUPPORTED);

	if (addrlist)
		*addrlist = NULL;

	if (kv)
		*kv = NULL;

	wrapper.handle = handle;
	wrapper.arg0 = addrlist;
	wrapper.arg1 = kv;
	wrapper.more = 1;
	mos_mutex_init(&wrapper.lock);
	mos_cond_init(&wrapper.cond);

	_threaded_poll_lock(threaded_poll);
	r = _service_resolver_new(client, interface, proto, host, type, domain, reqproto, 0,
	  DNSServiceResolve_Callback, &wrapper);
	_threaded_poll_unlock(threaded_poll);
	if (r == NULL) {
		logerr("_service_resolver_new() failed on service '%s': %s", host, _strerror(_client_errno(client)));
		res = EPHIDGET_UNEXPECTED;
		goto error_exit;
	}

	mos_mutex_lock(&wrapper.lock);
	while (wrapper.more)
		mos_cond_wait(&wrapper.cond, &wrapper.lock);
	mos_mutex_unlock(&wrapper.lock);

	mos_mutex_destroy(&wrapper.lock);
	mos_cond_destroy(&wrapper.cond);

	_threaded_poll_lock(threaded_poll);
	_service_resolver_free(r);
	_threaded_poll_unlock(threaded_poll);

	if (wrapper.res != EPHIDGET_OK) {
		res = wrapper.res;
		goto error_exit;
	}

	*port = wrapper.port;

	return (EPHIDGET_OK);

error_exit:

	if (addrlist && *addrlist) {
		mos_freeaddrlist(*addrlist);
		*addrlist = NULL;
	}

	return (res);
}

static void
entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state, void *ctx) {
	ZeroconfPublishHandle hdl;

	hdl = ctx;

	/* Called whenever the entry group state changes */
	switch (state) {
	case AVAHI_ENTRY_GROUP_ESTABLISHED:
		break;
	case AVAHI_ENTRY_GROUP_COLLISION :
		break;
	case AVAHI_ENTRY_GROUP_FAILURE :
		break;
	case AVAHI_ENTRY_GROUP_UNCOMMITED:
	case AVAHI_ENTRY_GROUP_REGISTERING:
		break;
	}

	mos_mutex_lock(&hdl->lock);
	mos_cond_broadcast(&hdl->cond);
	mos_mutex_unlock(&hdl->lock);
}

PhidgetReturnCode
Zeroconf_publish(ZeroconfPublishHandle *hdl, const char *name, const char *host, const char *type,
  int port, kv_t *txtRecords) {
	PhidgetReturnCode res;
	AvahiStringList *sl;
	kvent_t *e;
	int err;

	TESTPTR(hdl);
	TESTPTR(name);
	TESTPTR(type);

	if (client == NULL) {
		logerr("client is not initialized");
		return (EPHIDGET_UNEXPECTED);
	}

	sl = NULL;

	*hdl = mos_zalloc(sizeof (**hdl));
	(*hdl)->host = mos_strdup(name, NULL);
	(*hdl)->type = mos_strdup(type, NULL);
	mos_mutex_init(&(*hdl)->lock);
	mos_cond_init(&(*hdl)->cond);

	_threaded_poll_lock(threaded_poll);
	(*hdl)->ref = _entry_group_new(client, entry_group_callback, *hdl);
	if ((*hdl)->ref == NULL) {
		_threaded_poll_unlock(threaded_poll);
		logerr("failed to create entry group");
		return (EPHIDGET_UNEXPECTED);
	}

	if (txtRecords) {
		KV_FOREACH(e, txtRecords)
			sl = _string_list_add_pair(sl, e->key, e->val);
	}

	if (sl) {
		err = _entry_group_add_service_strlst((*hdl)->ref, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC, 0,
		  name, type, NULL, NULL, port, sl);
		_string_list_free(sl);
	} else {
		err = _entry_group_add_service((*hdl)->ref, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC, 0,
		  name, type, NULL, NULL, port, NULL);
	}

	if (err != 0) {
		_threaded_poll_unlock(threaded_poll);
		logerr("failed to add service '%s'", name);
		res = EPHIDGET_UNEXPECTED;
		goto bad;
	}

	err = _entry_group_commit((*hdl)->ref);
	_threaded_poll_unlock(threaded_poll);
	if (err != 0) {
		logerr("failed to commit service '%s'", name);
		res = EPHIDGET_UNEXPECTED;
		goto bad;
	}

	mos_mutex_lock(&(*hdl)->lock);
	for (;;) {
		switch(_entry_group_get_state((*hdl)->ref)) {
		case AVAHI_ENTRY_GROUP_ESTABLISHED:
			logdebug("entry group established for '%s.%s'", (*hdl)->host, (*hdl)->type);
			mos_mutex_unlock(&(*hdl)->lock);
			return (EPHIDGET_OK);
		case AVAHI_ENTRY_GROUP_COLLISION :
			logerr("collision on name '%s'", (*hdl)->host);
			mos_mutex_unlock(&(*hdl)->lock);
			res = EPHIDGET_DUPLICATE;
			goto bad;
		case AVAHI_ENTRY_GROUP_FAILURE :
			logerr("entry group '%s' failure: %s", (*hdl)->host, _strerror(_client_errno(client)));
			mos_mutex_unlock(&(*hdl)->lock);
			res = EPHIDGET_UNEXPECTED;
			goto bad;
		case AVAHI_ENTRY_GROUP_UNCOMMITED:
		case AVAHI_ENTRY_GROUP_REGISTERING:
			break;
		}
		res = mos_cond_timedwait(&(*hdl)->cond, &(*hdl)->lock, 2 * MOS_SEC);
		if (res == EPHIDGET_TIMEOUT) {
			logerr("entry group '%s' timed out", (*hdl)->host);
			mos_mutex_unlock(&(*hdl)->lock);
			goto bad;
		}
	}

	return (EPHIDGET_OK);

bad:

	Zeroconf_unpublish(hdl);
	return (res);
}

PhidgetReturnCode
Zeroconf_unpublish(ZeroconfPublishHandle *hdl) {

	TESTPTR(hdl);

	if ((*hdl)->ref) {	/* might not be set if called in failure case */
		_threaded_poll_lock(threaded_poll);
		_entry_group_free((*hdl)->ref);
		_threaded_poll_unlock(threaded_poll);
	}

	mos_free((*hdl)->host, MOSM_FSTR);
	mos_free((*hdl)->type, MOSM_FSTR);
	mos_mutex_destroy(&(*hdl)->lock);
	mos_cond_destroy(&(*hdl)->cond);
	mos_free(*hdl, sizeof (**hdl));
	*hdl = NULL;

	return (EPHIDGET_OK);
}
