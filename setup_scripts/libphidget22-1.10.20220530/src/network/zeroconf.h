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

#ifndef __CZEROCONF
#define __CZEROCONF

#include "mos/mos_net.h"
#include "mos/kv/kv.h"

#if ZEROCONF_SUPPORT

typedef struct _ZeroconfListener *ZeroconfListenerHandle;

#define ZEROCONF_MAX_TXTRECORD	1024

#if defined(_LINUX) || defined(_FREEBSD)
#define _AVAHI	1
#include "avahi-common/address.h"
#include "avahi-client/lookup.h"
#include "avahi-client/publish.h"
#else /* LINUX */
#include "ext/include/dns_sd.h"
#endif

#ifdef _AVAHI
typedef enum {
	ZCP_IPv4 = AVAHI_PROTO_INET,
	ZCP_IPv6 = AVAHI_PROTO_INET6,
	ZCP_UNSPEC = AVAHI_PROTO_UNSPEC
} Zeroconf_Protocol;
#else
typedef enum {
	ZCP_IPv4 = kDNSServiceProtocol_IPv4,
	ZCP_IPv6 = kDNSServiceProtocol_IPv6,
	ZCP_UNSPEC = 0
} Zeroconf_Protocol;
#endif

typedef struct _ZeroconfPublish {
#ifdef _AVAHI
	AvahiEntryGroup		*ref;
	AvahiEntryGroupState state;
	mos_mutex_t			lock;
	mos_cond_t			cond;
#else
	DNSServiceRef		ref;
#endif
	char				*host;
	char				*type;
} ZeroconfPublish, *ZeroconfPublishHandle;

#define ZCL_RUN		0x01
#define ZCL_RUNNING	0x02
#define ZCL_STOPPED	0x04
#define ZCL_ERROR	0x08

/* handle, context, added, interface, protocol, name, type, domain */
typedef void(*ZeroconfListener_t)(ZeroconfListenerHandle, void *, int, int, Zeroconf_Protocol, const char *,
  const char *, const char *, const char *);

typedef struct _ZeroconfListener {
	int 				flags;
#ifdef _AVAHI
	AvahiServiceBrowser	*sb;
#else
	DNSServiceRef		ref;		/* browser */
	mos_mutex_t			lock;
	mos_cond_t			cond;
#endif
	char				*type;
	mos_task_t			task;
	ZeroconfListener_t	listener;
	void				*listenerctx;
} ZeroconfListener;

char *Zeroconf_unescape_label(const char **name, char *dest, size_t size);

/*
 * We take the name, hostname, type and domain in lookup because bonjour and avahi differ in the browse
 * and resolve provide, and it hasn't been worth the effort to fix after the initial implementation.
 *
 * Really, the hostname should be resolved and return by this call, not taken as a parameter.
 */
PhidgetReturnCode Zeroconf_lookup(ZeroconfListenerHandle, int, Zeroconf_Protocol, const char *, const char *,
  const char *, const char *, Zeroconf_Protocol, mos_sockaddr_list_t **, uint16_t *, kv_t **);
PhidgetReturnCode Zeroconf_addr_lookup(const char *, Zeroconf_Protocol, mos_sockaddr_list_t **);
PhidgetReturnCode Zeroconf_listen(ZeroconfListenerHandle *, const char *, ZeroconfListener_t, void *);
void Zeroconf_listenclose(ZeroconfListenerHandle *);
PhidgetReturnCode Zeroconf_publish(ZeroconfPublishHandle *, const char *, const char *, const char *, int, kv_t *);
PhidgetReturnCode Zeroconf_unpublish(ZeroconfPublishHandle *handle);
#endif
#endif
