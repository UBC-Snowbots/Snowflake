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
#include "phidget.h"
#include "mos/mos_os.h"
#include "mos/mos_time.h"
#include "util/json.h"
#include "util/phidgetconfig.h"
#include "debug.h"

typedef PhidgetReturnCode (*phiddebugset_t)(const char *, const char *);
typedef PhidgetReturnCode (*phiddebugget_t)(const char *, char *, size_t);

typedef struct _phiddebug {
	const char		*name;
	phiddebugget_t	get;
	phiddebugset_t	set;
} phiddebug_t;

static char mallocset_dst[MOS_PATH_MAX] = "mallocset.dmp";
static FILE *mallocsetfp;
static uint32_t mallocset;

#define CK(stmt)	do {			\
	res = (stmt);					\
	if (res != EPHIDGET_OK)			\
		return (res);				\
} while (0)
#define CKBAD(stmt)	do {			\
	res = (stmt);					\
	if (res != EPHIDGET_OK)			\
		goto bad;					\
} while (0)

static PhidgetReturnCode
db_mallocset_get(const char *key, char *data, size_t datasz) {
	mos_snprintf(data, datasz, "%u", mallocset);
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
db_mallocset_set(const char *key, const char *data) {
	uint32_t tmp;

	if (mos_strtou32(data, 10, &tmp) != 0)
		return (EPHIDGET_INVALIDARG);

	mallocset = tmp;
	mos_mallocset_set(mallocset);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
db_mallocsetdest_get(const char *key, char *data, size_t datasz) {

	mos_strlcpy(data, mallocset_dst, datasz);
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
db_mallocsetdest_set(const char *key, const char *data) {

	mos_strlcpy(mallocset_dst, data, sizeof (mallocset_dst));
	return (EPHIDGET_OK);
}

static int
mprinter(void *ctx, const char *fmt, ...) {
	char buf[4096];
	va_list va;
	FILE *fp;

	fp = ctx;

	va_start(va, fmt);
	mos_vsnprintf(buf, sizeof (buf), fmt, va);
	va_end(va);

	fprintf(fp, "%s", buf);
	return (0);
}

static PhidgetReturnCode
openmallocsetfp() {

	mallocsetfp = fopen(mallocset_dst, "w");
	if (mallocsetfp == NULL)
		return (EPHIDGET_IO);
	return (EPHIDGET_OK);
}

static void
closemallocsetfp() {

	fclose(mallocsetfp);
	mallocsetfp = NULL;
}

static PhidgetReturnCode
db_mallocset_dump(const char *key, const char *data) {
	PhidgetReturnCode res;

	res = openmallocsetfp();
	if (res != EPHIDGET_OK)
		return (res);
	mos_dump_allocation_set(mallocset, mprinter, mallocsetfp);
	closemallocsetfp();
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
db_setloglevel(const char *key, const char *data) {
	char source[32];
	const char *c;
	uint32_t ll;

	TESTPTR(data);

	c = mos_strchrc(data, ':');
	if (c == NULL) {
		if (mos_strtou32(data, 0, &ll) != 0)
			return (EPHIDGET_INVALIDARG);
		return (PhidgetLog_setLevel(ll));
	}

	if (mos_strtou32(c + 1, 0, &ll) != 0)
		return (EPHIDGET_INVALIDARG);

	mos_strlcpy(source, data, MOS_MIN(sizeof (source), (unsigned)((c - data) + 1)));
	return (PhidgetLog_setSourceLevel(source, ll));
}

static PhidgetReturnCode
db_getloglevel(const char *key, char *data, size_t datasz) {
	Phidget_LogLevel ll;

	TESTPTR(data);

	PhidgetLog_getLevel(&ll);
	mos_snprintf(data, datasz, "%d", ll);
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
db_getlogsources(const char *key, char *data, size_t datasz) {
	PhidgetReturnCode res;
	const char **sources;
	Phidget_LogLevel ll;
	uint32_t i, cnt;
	char buf[64];
	size_t n;

	res = PhidgetLog_getSources(NULL, &cnt);
	if (res != EPHIDGET_OK)
		return (res);

	n = sizeof (const char *) * cnt;
	sources = mos_malloc(n);

	res = PhidgetLog_getSources(sources, &cnt);
	if (res != EPHIDGET_OK) {
		mos_free((void *)sources, n);
		return (res);
	}

	data[0] = '\0';
	for (i = 0; i < cnt; i++) {
		PhidgetLog_getSourceLevel(sources[i], &ll);
		mos_snprintf(buf, sizeof (buf), "%s:%d", sources[i], ll);
		mos_strlcat(data, buf, datasz);
		if ((i + 1) < cnt)
			mos_strlcat(data, ";", datasz);
	}
	mos_free((void *)sources, n);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
db_getopenconnections(const char *key, char *data, size_t datasz) {
	PhidgetReturnCode res;
	pconf_t *pc;

	TESTPTR(data);

	res = openServersToPConf(&pc);
	if (res != EPHIDGET_OK)
		return (res);

	res = pconf_renderjson(pc, data, datasz);
	pconf_release(&pc);
	if (res != EPHIDGET_OK)
		return (res);

	return (EPHIDGET_OK);
}

/*
 * This function reaches into the parent because we cannot access the runlock of the parent of
 * the dictionary control channel without recursing.  Also, this should be safe since the channel
 * must be attached; otherwise, we wouldn't be here.
 */
static int
channelToPConf(PhidgetChannelHandle ch, pconf_t **pc) {
	PhidgetChannelNetConnHandle cnc;
	PhidgetDeviceHandle dev;
	PhidgetReturnCode res;
	const char *conntype;
	const char *cclass;
	const char *label;
	pconf_t *ncpc;
	int net;

	res = pconf_create(pc);
	if (res != EPHIDGET_OK)
		return (res);

	net = isNetworkPhidget(ch);

	switch(ch->parent->connType) {
	case PHIDCONN_HIDUSB:
		conntype = "HIDUSB";
		break;
	case PHIDCONN_VINT:
		conntype = "VINT";
		break;
	case PHIDCONN_MESH:
		conntype = "MESH";
		break;
	case PHIDCONN_SPI:
		conntype = "SPI";
		break;
	case PHIDCONN_LIGHTNING:
		conntype = "LIGHTNING";
		break;
	case PHIDCONN_VIRTUAL:
		conntype = "VIRTUAL";
		break;
	case PHIDCONN_NETWORK:
		conntype = "NET";
		break;
	case PHIDCONN_PHIDUSB:
		conntype = "PHIDUSB";
		break;
	default:
		conntype = "UNKN";
	}

	Phidget_getChannelClassName((PhidgetHandle)ch, &cclass);

	dev = ch->parent;
	label = (const char *)dev->deviceInfo.label;

	if (label == NULL)
		label = "";

	mos_mutex_lock(&ch->netconnslk);
	if (ch->netconnscnt > 0) {
		CKBAD(pconf_addstr(*pc, ch->UCD->name, "name"));
		CKBAD(pconf_addstr(*pc, cclass, "class"));
		CKBAD(pconf_addstr(*pc, label, "label"));
		CKBAD(pconf_addstr(*pc, conntype, "conntype"));
		CKBAD(pconf_addi(*pc, dev->deviceInfo.serialNumber, "sn"));
		CKBAD(pconf_addi(*pc, dev->deviceInfo.hubPort, "port"));
		CKBAD(pconf_addi(*pc, dev->deviceInfo.isHubPort, "ishubport"));
		CKBAD(pconf_addi(*pc, ch->index, "ch"));
		CKBAD(pconf_addi(*pc, net, "network"));
		CKBAD(pconf_addi(*pc, ch->netconnscnt, "netconncnt"));

		CKBAD(pconf_addarray(*pc, "conn"));
		MTAILQ_FOREACH(cnc, &ch->netconns, link) {
			CKBAD(netConnToPConf(cnc->nc, &ncpc));
			res = pconf_merge(*pc, &ncpc, NULL, "conn");
			if (res != EPHIDGET_OK) {
				pconf_release(&ncpc);
				goto bad;
			}
		}

		mos_mutex_unlock(&ch->netconnslk);
		return (EPHIDGET_OK);
	}

	CKBAD(pconf_addstr(*pc, ch->UCD->name, "name"));
	CKBAD(pconf_addstr(*pc, cclass, "class"));
	CKBAD(pconf_addstr(*pc, conntype, "conntype"));
	CKBAD(pconf_addi(*pc, net, "network"));
	mos_mutex_unlock(&ch->netconnslk);
	return (EPHIDGET_OK);

bad:
	mos_mutex_unlock(&ch->netconnslk);
	pconf_release(pc);
	return (res);
}

static PhidgetReturnCode
db_getopenchannels(const char *key, char *data, size_t datasz) {
	PhidgetDeviceHandle dev;
	PhidgetChannelHandle ch;
	PhidgetReturnCode res;
	pconf_t *cpc;
	pconf_t *pc;
	int ccnt;
	int i;


	TESTPTR(data);

	res = pconf_create(&pc);
	if (res != EPHIDGET_OK)
		return (res);

	/*
	 * Don't even try with a small buffer.
	 */
	if (datasz < 1024)
		return (EPHIDGET_NOSPC);

	CKBAD(pconf_addi(pc, 1, "ver"));
	CKBAD(pconf_addarray(pc, "channels"));

	ccnt = 0;
	PhidgetReadLockDevices();
	FOREACH_DEVICE(dev) {
		if (!ISOPEN(dev))
			continue;
		for (i = 0; i < PHIDGET_MAXCHANNELS; i++) {
			ch = getChannel(dev, i);
			if (ch == NULL)
				continue;
			if (!ISATTACHED(ch)) {
				PhidgetRelease(&ch);
				continue;
			}

			CKBAD(channelToPConf(ch, &cpc));
			res = pconf_merge(pc, &cpc, NULL, "channels");
			if (res != EPHIDGET_OK) {
				pconf_release(&cpc);
				goto bad;
			}
			ccnt++;
			PhidgetRelease(&ch);
			if (res != EPHIDGET_OK)
				goto bad;
		}
	}
	PhidgetUnlockDevices();

	CKBAD(pconf_addi(pc, ccnt, "cnt"));
	CKBAD(pconf_renderjson(pc, data, datasz));

	return (EPHIDGET_OK);

bad:

	logerr("failed to generate open channels pconf");
	PhidgetUnlockDevices();
	pconf_release(&pc);
	return (res);
}

/************************************************************************************/

static const char *phiddebugactions =
  "mallocset\n"
  "mallocset_destination\n"
  "mallocset_dump\n"
  "loglevel\n"
  "logsources\n"
  "openconnections\n"
  "openchannels\n";

static phiddebug_t phiddebuggers[] = {
	{"mallocset", db_mallocset_get, db_mallocset_set },
	{"mallocset_destination", db_mallocsetdest_get, db_mallocsetdest_set },
	{"mallocset_dump", NULL, db_mallocset_dump },
	{"loglevel", db_getloglevel, db_setloglevel },
	{"logsources", db_getlogsources, NULL },
	{"openconnections", db_getopenconnections, NULL},
	{"openchannels", db_getopenchannels, NULL},
	{ NULL, NULL }
};

static phiddebug_t *
finddebug(const char *key) {
	phiddebug_t *debug;

	for (debug = phiddebuggers; debug->name != NULL; debug++)
		if (mos_strcmp(debug->name, key) == 0)
			return (debug);
	return (NULL);
}

PhidgetReturnCode
getPhidgetDebugKey(const char *key, char *buf, size_t bufsz) {
	phiddebug_t *debug;

	debug = finddebug(key);
	if (debug == NULL)
		return (EPHIDGET_NOENT);

	if (debug->get == NULL) {
		buf[0] = '\0';
		return (EPHIDGET_OK);
	}

	return (debug->get(key, buf, bufsz));
}

PhidgetReturnCode
setPhidgetDebugKey(const char *key, const char *buf) {
	phiddebug_t *debug;

	debug = finddebug(key);
	if (debug == NULL)
		return (EPHIDGET_NOENT);

	if (debug->set == NULL)
		return (EPHIDGET_OK);

	return (debug->set(key, buf));
}

/*
 * Very simple implementation assumes a small number of keys, and does not support middle key query.
 */
PhidgetReturnCode
getPhidgetDebugKeys(const char *start, char *keys, size_t keysz) {

	if (start == NULL || mos_strlen(start) == 0)
		mos_strlcpy(keys, phiddebugactions, keysz);
	else
		keys[0] = '\0';

	return (EPHIDGET_OK);
}
