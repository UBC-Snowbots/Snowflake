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

#define INCLUDE_PRIVATE

#include "phidgetbase.h"
#include "device/dictionarydevice.h"

#include "stats.h"
#include "debug.h"
#include "mos/bsdtree.h"

#define DICTIONARY_KEYMAX	255
#define DICTIONARY_VALMAX	65536


static PhidgetDictionary_OnChangeCallback onChangeCallback;
static void *onChangeCallbackCtx;

API_PRETURN
PhidgetDictionary_setOnChangeCallbackHandler(PhidgetDictionary_OnChangeCallback cb, void *ctx) {

	onChangeCallback = cb;
	onChangeCallbackCtx = ctx;

	return (EPHIDGET_OK);
}

#define ONCHANGE(phid, act, key, val) do {						\
	if (onChangeCallback != NULL)								\
		onChangeCallback((phid)->phid.deviceInfo.serialNumber,	\
		  (phid)->phid.deviceInfo.label,						\
		  onChangeCallbackCtx, (act), (key), (val));			\
} while (0)

/* [/_A-Za-z][/_A-Za-z0-9]* */
API_IRETURN
Phidget_validDictionaryKey(const char *key) {
	const char *c;

	if (key == NULL || key[0] == '\0')
		return (0);

	for (c = key; *c; c++) {
		switch (*c) {
		case '/':
		case '_':
			continue;
		default:
			break;
		}
		if (*c < 48 || *c > 122)
			return (0);
		if (*c > 57 && *c < 65)
			return (0);
		if (*c > 90 && *c < 97)
			return (0);

		if (c == key)
			if (*c >= 48 && *c <= 57)
				return (0);
	}
	return (1);
}

int
dictent_compare(dictent_t *a, dictent_t *b) {

	return (mos_strcmp(a->de_key, b->de_key));
}

RB_GENERATE(dictionary, _dictent, de_link, dictent_compare)

static void
freedictent(dictent_t *ent) {

	mos_free(ent->de_key, MOSM_FSTR);
	mos_free(ent->de_val, MOSM_FSTR);
	mos_free(ent, sizeof (*ent));
}

static PhidgetReturnCode CCONV
PhidgetDictionaryDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetDictionaryDeviceHandle dd;

	dd = (PhidgetDictionaryDeviceHandle)device;

	if (dd->initialized == 0) {
		RB_INIT(&dd->dict);
		dd->initialized = 1;
	}

	return (EPHIDGET_OK);
}

/*
 * Currently should never be called for a dictionary device.
 */
static PhidgetReturnCode CCONV
PhidgetDictionaryDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
stats_bridgeInput(PhidgetDictionaryDeviceHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;
	char keysbuf[DICTIONARY_VALMAX];
	const char *key;
	uint32_t stat;
	char val[32];

	switch (bp->vpkt) {
	case BP_DICTIONARYGET:
		key = getBridgePacketString(bp, 0);
		res = getPhidgetStat(key, &stat);
		if (res != EPHIDGET_OK)
			return (res);
		mos_snprintf(val, sizeof (val), "%u", stat);
		bp->reply_bpe = bridgeCreateReplyBPEfromString(mos_strdup(val, NULL));
		return (EPHIDGET_OK);
	case BP_DICTIONARYSCAN:
		key = getBridgePacketString(bp, 0);
		res = getPhidgetStatKeys(key, keysbuf, sizeof (keysbuf));
		if (res != EPHIDGET_OK)
			return (res);
		bp->reply_bpe = bridgeCreateReplyBPEfromString(mos_strdup(keysbuf, NULL));
		return (EPHIDGET_OK);
	case BP_OPENRESET:
	case BP_CLOSERESET:
	case BP_ENABLE:
	case BP_DICTIONARYADD:
	case BP_DICTIONARYUPDATE:
	case BP_DICTIONARYSET:
	case BP_DICTIONARYREMOVE:
	case BP_DICTIONARYREMOVEALL:
		return (EPHIDGET_OK);	// XXX Should we do this?
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

static PhidgetReturnCode
debug_bridgeInput(PhidgetDictionaryDeviceHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;
	char buf[DICTIONARY_VALMAX];
	const char *key;
	const char *val;

	switch (bp->vpkt) {
	case BP_DICTIONARYGET:
		key = getBridgePacketString(bp, 0);
		res = getPhidgetDebugKey(key, buf, sizeof (buf));
		if (res != EPHIDGET_OK)
			return (res);
		bp->reply_bpe = bridgeCreateReplyBPEfromString(mos_strdup(buf, NULL));
		return (EPHIDGET_OK);
	case BP_DICTIONARYUPDATE:
	case BP_DICTIONARYSET:
		key = getBridgePacketString(bp, 0);
		val = getBridgePacketString(bp, 1);
		res = setPhidgetDebugKey(key, val);
		if (res != EPHIDGET_OK)
			return (res);
		return (EPHIDGET_OK);
	case BP_DICTIONARYSCAN:
		key = getBridgePacketString(bp, 0);
		res = getPhidgetDebugKeys(key, buf, sizeof (buf));
		if (res != EPHIDGET_OK)
			return (res);
		bp->reply_bpe = bridgeCreateReplyBPEfromString(mos_strdup(buf, NULL));
		return (EPHIDGET_OK);
	case BP_OPENRESET:
	case BP_CLOSERESET:
	case BP_ENABLE:
	case BP_DICTIONARYADD:
	case BP_DICTIONARYREMOVE:
	case BP_DICTIONARYREMOVEALL:
		return (EPHIDGET_OK);	// XXX Should we do this?
	default:
		MOS_PANIC("Unexpected packet type");
	}
}


static PhidgetReturnCode CCONV
PhidgetDictionaryDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetDictionaryDeviceHandle phid = (PhidgetDictionaryDeviceHandle)ch->parent;
	char keys[DICTIONARY_VALMAX];
	dictent_t *ent, *tent;
	const char *key;
	const char *val;
	dictent_t skey;
	char *keysp;
	size_t len;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_DICTIONARY);
	assert(ch->class == PHIDCHCLASS_DICTIONARY);

	if (phid->phid.deviceInfo.serialNumber == 1)
		return (stats_bridgeInput(phid, bp));
	if (phid->phid.deviceInfo.serialNumber == 2)
		return (debug_bridgeInput(phid, bp));

	switch (bp->vpkt) {
	case BP_OPENRESET:
	case BP_CLOSERESET:
	case BP_ENABLE:
		return (EPHIDGET_OK);
	case BP_DICTIONARYGET:
		skey.key.ckey = getBridgePacketString(bp, 0);
		ent = RB_FIND(dictionary, &phid->dict, &skey);
		if (ent == NULL)
			return (MOS_ERROR(bp->iop, EPHIDGET_NOENT, "Specified key: '%s' does not exist in the dictionary", skey.key.ckey));
		bp->reply_bpe = bridgeCreateReplyBPEfromString(mos_strdup(ent->de_val, NULL));
		return (EPHIDGET_OK);
	case BP_DICTIONARYADD:
		key = getBridgePacketString(bp, 0);
		if (!Phidget_validDictionaryKey(key))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Invalid key name. Key names can contain: / _ A-Z a-z 0-9 and cannot start with a number."));

		ent = mos_malloc(sizeof (*ent));
		ent->de_key = mos_strdup(key, NULL);
		ent->de_val = mos_strdup(getBridgePacketString(bp, 1), NULL);
		tent = RB_INSERT(dictionary, &phid->dict, ent);
		if (tent != NULL) {
			freedictent(ent);
			return (MOS_ERROR(bp->iop, EPHIDGET_DUPLICATE, "Key already exists."));
		}
		phid->cnt++;
		bridgeSendToChannelNC(ch, bridgePacketGetNetConn(bp), BP_DICTIONARYADDED, "%s%s",
		  ent->de_key, ent->de_val);

		ONCHANGE(phid, BP_DICTIONARYADDED, ent->de_key, ent->de_val);
		return (EPHIDGET_OK);
	case BP_DICTIONARYUPDATE:
		if (bp->entrycnt != 2)
			return (MOS_ERROR(bp->iop, EPHIDGET_UNEXPECTED, "Malformed bridge packet."));

		skey.key.ckey = getBridgePacketString(bp, 0);
		val = getBridgePacketString(bp, 1);	/* make sure it is there before we do anything else */

		ent = RB_FIND(dictionary, &phid->dict, &skey);
		if (ent == NULL)
			return (MOS_ERROR(bp->iop, EPHIDGET_NOENT, "Specified key: '%s' does not exist in the dictionary", skey.key.ckey));
		mos_free(ent->de_val, MOSM_FSTR);
		ent->de_val = mos_strdup(val, NULL);
		bridgeSendToChannelNC(ch, bridgePacketGetNetConn(bp), BP_DICTIONARYUPDATED, "%s%s",
		  ent->de_key, ent->de_val);
		ONCHANGE(phid, BP_DICTIONARYUPDATED, ent->de_key, ent->de_val);
		return (EPHIDGET_OK);
	case BP_DICTIONARYSET:
		if (bp->entrycnt != 2)
			return (MOS_ERROR(bp->iop, EPHIDGET_UNEXPECTED, "Malformed bridge packet."));

		skey.key.ckey = getBridgePacketString(bp, 0);
		val = getBridgePacketString(bp, 1);	/* make sure it is there before we do anything else */

		ent = RB_FIND(dictionary, &phid->dict, &skey);
		if (ent == NULL) {
			key = getBridgePacketString(bp, 0);
			if (!Phidget_validDictionaryKey(key))
				return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG, "Invalid key name. Key names can contain: / _ A-Z a-z 0-9 and cannot start with a number."));
			ent = mos_malloc(sizeof (*ent));
			ent->de_key = mos_strdup(key, NULL);
			ent->de_val = mos_strdup(getBridgePacketString(bp, 1), NULL);
			RB_INSERT(dictionary, &phid->dict, ent);
			phid->cnt++;
			bridgeSendToChannelNC(ch, bridgePacketGetNetConn(bp), BP_DICTIONARYADDED, "%s%s",
			  ent->de_key, ent->de_val);
			ONCHANGE(phid, BP_DICTIONARYADDED, ent->de_key, ent->de_val);
		} else {
			mos_free(ent->de_val, MOSM_FSTR);
			ent->de_val = mos_strdup(val, NULL);
			bridgeSendToChannelNC(ch, bridgePacketGetNetConn(bp), BP_DICTIONARYUPDATED, "%s%s",
			  ent->de_key, ent->de_val);
			ONCHANGE(phid, BP_DICTIONARYUPDATED, ent->de_key, ent->de_val);
		}
		return (EPHIDGET_OK);
	case BP_DICTIONARYREMOVE:
		skey.key.ckey = getBridgePacketString(bp, 0);
		ent = RB_FIND(dictionary, &phid->dict, &skey);
		if (ent) {
			RB_REMOVE(dictionary, &phid->dict, ent);
			bridgeSendToChannelNC(ch, bridgePacketGetNetConn(bp), BP_DICTIONARYREMOVED, "%s", ent->de_key);
			ONCHANGE(phid, BP_DICTIONARYREMOVED, ent->de_key, NULL);
			freedictent(ent);
			phid->cnt--;
		}
		return (EPHIDGET_OK);
	case BP_DICTIONARYREMOVEALL:
		RB_FOREACH_SAFE(ent, dictionary, &phid->dict, tent) {
			RB_REMOVE(dictionary, &phid->dict, ent);
			bridgeSendToChannelNC(ch, bridgePacketGetNetConn(bp), BP_DICTIONARYREMOVED, "%s", ent->de_key);
			ONCHANGE(phid, BP_DICTIONARYREMOVED, ent->de_key, NULL);
			freedictent(ent);
		}
		return (EPHIDGET_OK);
	case BP_DICTIONARYSCAN:
		val = getBridgePacketString(bp, 0);
		if (mos_strlen(val) > 0) {
			skey.key.ckey = val;
			ent = RB_FIND(dictionary, &phid->dict, &skey);
			if (ent == NULL)
				return (MOS_ERROR(bp->iop, EPHIDGET_NOENT, "Specified key: '%s' does not exist in the dictionary", skey.key.ckey));

			/*
			 * Get the key following the given key.
			 */
			ent = RB_NEXT(dictionary, &phid->dict, ent);
			if (ent == NULL) {
				bp->reply_bpe = bridgeCreateReplyBPEfromString(mos_strdup("", NULL));
				return (EPHIDGET_OK);
			}
		} else {
			ent = RB_MIN(dictionary, &phid->dict);
			if (ent == NULL) {
				bp->reply_bpe = bridgeCreateReplyBPEfromString(mos_strdup("", NULL));
				return (EPHIDGET_OK);
			}
		}
		keysp = keys;
#define KLEN (size_t)(DICTIONARY_VALMAX - (keysp - keys))

		do {
			len = mos_strlcpy(keysp, ent->de_key, KLEN);
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
		} while ((ent = RB_NEXT(dictionary, &phid->dict, ent)) != NULL);

		bp->reply_bpe = bridgeCreateReplyBPEfromString(mos_strdup(keys, NULL));
		return (EPHIDGET_OK);
	default:
		MOS_PANIC("Unexpected packet type");
	}
}

static void CCONV
PhidgetDictionaryDevice_free(PhidgetDeviceHandle *phid) {
	PhidgetDictionaryDeviceHandle dict = (PhidgetDictionaryDeviceHandle)*phid;
	dictent_t *ent, *tent;

	if (dict->initialized) {
		RB_FOREACH_SAFE(ent, dictionary, &dict->dict, tent) {
			RB_REMOVE(dictionary, &dict->dict, ent);
			mos_free(ent->de_key, MOSM_FSTR);
			mos_free(ent->de_val, MOSM_FSTR);
			mos_free(ent, sizeof (*ent));
		}
	}

	mos_free(*phid, sizeof(struct _PhidgetDictionaryDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetDictionaryDevice_create(PhidgetDictionaryDeviceHandle *phidp) {
	DEVICECREATE_BODY(DictionaryDevice, PHIDCLASS_DICTIONARY);
	return (EPHIDGET_OK);
}