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

#include "phidgetbase.h"
#include "phidget.h"
#include "mos/mos_os.h"
#include "mos/mos_time.h"
#include "mos/mos_assert.h"
#include "mos/mos_atomic.h"

#include "stats.h"

static phidstat_t _phidstats[] = {
	{ "dispatch.max_dispatchers", 0 },
	{ "dispatch.desired_idle_dispatchers", 0 },
	{ "dispatch.dispatchers_ever", 0 },
	{ "dispatch.dispatchers_running", 0 },
	{ "dispatch.dispatchers", 0 },
	{ "dispatch.max_entries", 0 },
	{ "dispatch.desired_entries", 0 },
	{ "dispatch.entries", 0 },
	{ "device.attached", 0 },
	{ "usb.readthreads_ever", 0 },
	{ "usb.readthreads", 0 },
	{ "spi.readthreads_ever", 0 },
	{ "spi.readthreads", 0 },
	{ "bridge.readthreads_ever", 0 },
	{ "bridge.readthreads", 0 },
	{ "discovery.listeners", 0 },
	{ "discovery.dispatchers", 0 },
	{ "server.accepttasks_ever", 0 },
	{ "server.accepttasks", 0 },
	{ "server.clienttasks_ever", 0 },
	{ "server.clienttasks", 0 },
	{ "server.keepalivetasks_ever", 0 },
	{ "server.keepalivetasks", 0 },
	{ "server.netcontrol.entrytasks_ever", 0 },
	{ "server.netcontrol.entrytasks", 0 },
	{ "client.tasks_ever", 0 },
	{ "client.tasks", 0 },
	{ NULL, 0 }
};

int
phidstat_compare(phidstat_t *a, phidstat_t *b) {

	return (mos_strcmp(a->name, b->name));
}

RB_GENERATE(phidstats, _phidstat, link, phidstat_compare)

static phidstats_t stats;
static mos_mutex_t lock;

void
PhidgetStatsInit() {
	phidstat_t *ps;

	RB_INIT(&stats);

	for (ps = _phidstats; ps->name != NULL; ps++)
		RB_INSERT(phidstats, &stats, ps);

	mos_mutex_init(&lock);
}

void
PhidgetStatsFini() {

	mos_mutex_destroy(&lock);
}

PhidgetReturnCode
incPhidgetStat(const char *key) {
	phidstat_t psk;
	phidstat_t *ps;

	psk.name = key;
	ps = RB_FIND(phidstats, &stats, &psk);
	if (ps == NULL)
		return (EPHIDGET_NOENT);

	mos_mutex_lock(&lock);
	ps->count++;
	mos_mutex_unlock(&lock);
	return (EPHIDGET_OK);
}

PhidgetReturnCode
decPhidgetStat(const char *key) {
	phidstat_t psk;
	phidstat_t *ps;

	psk.name = key;
	ps = RB_FIND(phidstats, &stats, &psk);
	if (ps == NULL)
		return (EPHIDGET_NOENT);

	mos_mutex_lock(&lock);
	ps->count--;
	mos_mutex_unlock(&lock);
	return (EPHIDGET_OK);
}

PhidgetReturnCode
getPhidgetStat(const char *key, uint32_t *cnt) {
	phidstat_t psk;
	phidstat_t *ps;

	psk.name = key;
	ps = RB_FIND(phidstats, &stats, &psk);
	if (ps == NULL)
		return (EPHIDGET_NOENT);

	mos_mutex_lock(&lock);
	*cnt = ps->count;
	mos_mutex_unlock(&lock);
	return (EPHIDGET_OK);
}

PhidgetReturnCode
setPhidgetStat(const char *key, uint32_t cnt) {
	phidstat_t psk;
	phidstat_t *ps;

	psk.name = key;
	ps = RB_FIND(phidstats, &stats, &psk);
	if (ps == NULL)
		return (EPHIDGET_NOENT);

	mos_mutex_lock(&lock);
	ps->count = cnt;
	mos_mutex_unlock(&lock);
	return (EPHIDGET_OK);
}


PhidgetReturnCode
getPhidgetStatKeys(const char *start, char *keys, size_t keyssz) {
	phidstat_t psk;
	phidstat_t *ps;
	char *keysp;
	size_t len;

	if (start != NULL && mos_strlen(start) > 0) {
		psk.name = start;
		ps = RB_FIND(phidstats, &stats, &psk);
		if (ps == NULL)
			return (EPHIDGET_NOENT);

		/*
		 * Get the key following the given key.
		 */
		ps = RB_NEXT(phidstats, &stats, ps);
		if (ps == NULL) {
			keys[0] = '\0';
			return (EPHIDGET_OK);
		}
	} else {
		ps = RB_MIN(phidstats, &stats);
		if (ps == NULL) {
			keys[0] = '\0';
			return (EPHIDGET_OK);
		}
	}

#define KLEN (size_t)(keyssz - (keysp - keys))

	keysp = keys;
	do {
		len = mos_strlcpy(keysp, ps->name, KLEN);
		if (len >= KLEN) {
			*keysp = '\0';
			break;
		}
		keysp += len;
		len = mos_strlcpy(keysp, "\n", KLEN);
		if (len >= KLEN) {
			*keysp = '\0';
			break;
		}
		keysp += len;
	} while ((ps = RB_NEXT(phidstats, &stats, ps)) != NULL);

	return (EPHIDGET_OK);
}