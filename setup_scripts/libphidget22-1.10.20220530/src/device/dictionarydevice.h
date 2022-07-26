/*
 * This file is part of libphidget22
 *
 * Copyright 2016 Phidgets Inc <patrick@phidgets.com>
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

#ifndef _PHIDGET_DICTIONARYDEVICE_H_
#define _PHIDGET_DICTIONARYDEVICE_H_

typedef union {
	const char	*ckey;
	char		*key;
} tkey_t;

typedef struct _dictent {
#define de_key key.key
	tkey_t				key;
	char				*de_val;
	RB_ENTRY(_dictent)	de_link;
} dictent_t;

int dictent_compare(dictent_t *a, dictent_t *b);

typedef RB_HEAD(dictionary, _dictent) dictionary_t;
RB_PROTOTYPE(dictionary, _dictent, de_link, dictent_compare)

typedef struct _PhidgetDictionaryDevice *PhidgetDictionaryDeviceHandle;
PhidgetReturnCode PhidgetDictionaryDevice_create(PhidgetDictionaryDeviceHandle *phid);

struct _PhidgetDictionaryDevice {
	PhidgetDevice	phid;
	int				initialized;
	dictionary_t	dict;
	uint32_t		cnt;
} typedef PhidgetDictionaryDeviceInfo;

#endif /* _PHIDGET_DICTIONARYDEVICE_H_ */
