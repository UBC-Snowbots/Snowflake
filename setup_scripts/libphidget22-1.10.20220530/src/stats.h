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

#ifndef _STATS_H_
#define _STATS_H_

#include "phidget.h"

typedef struct _phidstat {
	const char			*name;
	uint32_t			count;
	RB_ENTRY(_phidstat)	link;
} phidstat_t;

int phidstat_compare(phidstat_t *a, phidstat_t *b);

typedef RB_HEAD(phidstats, _phidstat) phidstats_t;
RB_PROTOTYPE(phidstats, _phidstat, link, phidstat_compare);

void PhidgetStatsInit(void);
void PhidgetStatsFini(void);

PhidgetReturnCode incPhidgetStat(const char *key);
PhidgetReturnCode decPhidgetStat(const char *key);
PhidgetReturnCode getPhidgetStat(const char *key, uint32_t *cnt);
PhidgetReturnCode setPhidgetStat(const char *key, uint32_t cnt);
PhidgetReturnCode getPhidgetStatKeys(const char *startkey, char *keys, size_t keyssz);

#endif /* _STATS_H_ */
