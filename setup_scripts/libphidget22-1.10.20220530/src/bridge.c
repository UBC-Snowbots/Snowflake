#define _PHIDGET_NETWORKCODE

#include "phidgetbase.h"
#include "util/jsmn.h"	/* JSON */
#include "mos/mos_base64.h"
#include "mos/mos_assert.h"
#include "mos/bsdqueue.h"
#include "bridge.h"
#include "network/network.h"
#include "util/json.h"
#include "stats.h"

#define LK_BTLIST	(void *)3

#ifdef DEBUG
#define bridgeloginfo(...) PhidgetLog_loge(NULL, 0, __func__, "_phidget22bridge", PHIDGET_LOG_INFO, __VA_ARGS__)
#else
#define bridgeloginfo(...)
#endif

/************************************************************************************************************
* Bridge Packet
*
* Bridge packets are used primarily over the network, and can be constructed either directly via a printf
* style constructor, or by parsing JSON.
*
* Printf like formatters supported are:
*	%c	- uint8_t
*	%h	- int16_t
*	%uh	- uint16_t
*	%d	- int32_t
*	%u	- uint32_t
*	%l	- int64_t
*	%ul	- uint64_t
*	%f	- float
*	%g	- double
*	%s	- string			(max 64 chars)
*	%p	- ptr				(will not be rendered by JSON)
*	%#R	- uint8_t array		(# is the number of bytes)
*	%*R - uint8_t array		(takes two args, int length followed by the pointer)
*	%#H	- int16_t array		(# is the count of ints)
*	%*H - int16_t array		(takes two args, int count, followed by the pointer)
*	%#UH - uint16_t array	(# is the count of ints)
*	%*UH - uint16_t array	(takes two args, int count, followed by the pointer)
*	%#I	- int32_t array		(# is the count of ints)
*	%*I - int32_t array		(takes two args, int count, followed by the pointer)
*	%#U	- uint32_t array	(# is the count of ints)
*	%*U - uint32_t array	(takes two args, int count, followed by the pointer)
*	%#L	- int64_t array		(# is the count of ints)
*	%*L - int64_t array		(takes two args, int count, followed by the pointer)
*	%#UL - uint64_t array	(# is the count of ints)
*	%*UL - uint64_t array	(takes two args, int count, followed by the pointer)
*	%#G - double array		(# is the count of doubles)
*	%*G - double array		(takes two args, int count, followed by the pointer)
*	%J	- inline JSON object
*
* JSON of the form:
* { "v" : 0, "s" : 0, "f" : 0, "p" : 1, "I" : 1, "O" : 0, "X" : 1, "c" : 0, "e" : {
*	"0" : { "t" : "h", "v" : -134},
*	"1" : { "t" : "uh", "v" : 135},
*	"2" : { "t" : "d", "v" : 16},
*	"3" : { "t" : "u", "v" : 33},
*	"4" : { "t" : "f", "v" : 44.099998},
*	"5" : { "t" : "g", "v" : -128.44000},
*	"6" : { "t" : "s", "v" : "-- test str --"},
*	"7" : { "t" : "I", "v" : [0, 1, 2, 3]},
*	"8" : { "t" : "J", "v" : {"a" : 1}},
* }}
*
* v is the packet version
* s is the last packet source (JSON or BRIDGE)
* f is any flags in base10
* p is the bridge packet type
* I is the phidget id
* O is the client-side channel ID
* X is the channel index
* c is the entry count
* e is the entry object
* 0 is the first entry
*	t is the type (same as the printf formatter)
*	v is the value
*/

static PhidgetReturnCode
allocBridgePacket(BridgePacket **bp) {

	*bp = mos_zalloc(sizeof(BridgePacket));
	mos_tlock_init((*bp)->lock, P22LOCK_BPLOCK, P22LOCK_FLAGS);
	(*bp)->_refcnt = 1;

	return (0);
}

/*
 * Allocates an array at the current offset, and does NOT adjust that offset.
 * Called by generated code.
 */
static void
allocArray(BridgePacket *bp, size_t cnt, BridgePacketEntryType type, int offset) {
	size_t bsize;

	if (offset == -1)
		offset = bp->entrycnt;

	switch (type) {
	case BPE_UI8ARRAY:
		bsize = cnt;
		break;
	case BPE_I16ARRAY:
	case BPE_UI16ARRAY:
		bsize = cnt * sizeof(uint16_t);
		break;
	case BPE_I32ARRAY:
	case BPE_UI32ARRAY:
		bsize = cnt * sizeof(uint32_t);
		break;
	case BPE_I64ARRAY:
	case BPE_UI64ARRAY:
		bsize = cnt * sizeof(uint64_t);
		break;
	case BPE_DBLARRAY:
		bsize = cnt * sizeof(double);
		break;
	default:
		MOS_PANIC("Unsupported array type");
	}

	bp->entry[offset].type = type;
	bp->entry[offset].bpe_len = (uint16_t)bsize;
	if (bsize == 0)
		bp->entry[offset].bpe_ptr = NULL;
	else
		bp->entry[offset].bpe_ptr = mos_malloc(bsize);

	switch (type) {
	case BPE_UI8ARRAY:
		bp->entry[offset].bpe_ui8array = bp->entry[offset].bpe_ptr;
		bp->entry[offset].bpe_cnt = (uint16_t)bsize;
		break;
	case BPE_I16ARRAY:
		bp->entry[offset].bpe_i16array = bp->entry[offset].bpe_ptr;
		bp->entry[offset].bpe_cnt = bp->entry[offset].bpe_len / sizeof(int16_t);
		MOS_ASSERT((bp->entry[offset].bpe_len % sizeof(int16_t)) == 0);
		break;
	case BPE_UI16ARRAY:
		bp->entry[offset].bpe_ui16array = bp->entry[offset].bpe_ptr;
		bp->entry[offset].bpe_cnt = bp->entry[offset].bpe_len / sizeof(uint16_t);
		MOS_ASSERT((bp->entry[offset].bpe_len % sizeof(uint16_t)) == 0);
		break;
	case BPE_I32ARRAY:
		bp->entry[offset].bpe_i32array = bp->entry[offset].bpe_ptr;
		bp->entry[offset].bpe_cnt = bp->entry[offset].bpe_len / sizeof(int32_t);
		MOS_ASSERT((bp->entry[offset].bpe_len % sizeof(int32_t)) == 0);
		break;
	case BPE_UI32ARRAY:
		bp->entry[offset].bpe_ui32array = bp->entry[offset].bpe_ptr;
		bp->entry[offset].bpe_cnt = bp->entry[offset].bpe_len / sizeof(uint32_t);
		MOS_ASSERT((bp->entry[offset].bpe_len % sizeof(uint32_t)) == 0);
		break;
	case BPE_I64ARRAY:
		bp->entry[offset].bpe_i64array = bp->entry[offset].bpe_ptr;
		bp->entry[offset].bpe_cnt = bp->entry[offset].bpe_len / sizeof(int64_t);
		MOS_ASSERT((bp->entry[offset].bpe_len % sizeof(int64_t)) == 0);
		break;
	case BPE_UI64ARRAY:
		bp->entry[offset].bpe_ui64array = bp->entry[offset].bpe_ptr;
		bp->entry[offset].bpe_cnt = bp->entry[offset].bpe_len / sizeof(uint64_t);
		MOS_ASSERT((bp->entry[offset].bpe_len % sizeof(uint64_t)) == 0);
		break;
	case BPE_DBLARRAY:
		bp->entry[offset].bpe_dblarray = bp->entry[offset].bpe_ptr;
		bp->entry[offset].bpe_cnt = bp->entry[offset].bpe_len / sizeof(double);
		MOS_ASSERT((bp->entry[offset].bpe_len % sizeof(double)) == 0);
		break;
	}
}
static BridgePacketEntryType
decodeBridgePacketEntryType(const char *str) {

	switch (str[0]) {
	case '0':
		return (BPE_NONE);
	case 'c':
		return (BPE_UI8);
	case 'd':
		return (BPE_I32);
	case 'f':
		return (BPE_FLOAT);
	case 'g':
		return (BPE_DBL);
	case 'h':
		return (BPE_I16);
	case 'l':
		return (BPE_I64);
	case 's':
		return (BPE_STR);
	case 'u':
		switch (str[1]) {
		case 0:
			return (BPE_UI32);
		case 'h':
			return (BPE_UI16);
		case 'l':
			return (BPE_UI64);
		default:
			return (BPE_NONE);
		}
	case 'G':
		return (BPE_DBLARRAY);
	case 'H':
		return (BPE_I16ARRAY);
	case 'I':
		return (BPE_I16ARRAY);
	case 'J':
		return (BPE_JSON);
	case 'L':
		return (BPE_I64ARRAY);
	case 'R':
		return (BPE_UI8ARRAY);
	case 'U':
		switch (str[1]) {
		case 0:
			return (BPE_UI32ARRAY);
		case 'H':
			return (BPE_UI16ARRAY);
		case 'L':
			return (BPE_UI64ARRAY);
		default:
			return (BPE_NONE);
		}
	}

	return (BPE_NONE);
}

static const char *
bridgePacketEntryType(BridgePacketEntryType type) {

	switch (type) {
	case BPE_NONE:
		return ("0");
	case BPE_UI8:
		return ("c");
	case BPE_I16:
		return ("h");
	case BPE_UI16:
		return ("uh");
	case BPE_I32:
		return ("d");
	case BPE_UI32:
		return ("u");
	case BPE_I64:
		return ("l");
	case BPE_UI64:
		return ("ul");
	case BPE_FLOAT:
		return ("f");
	case BPE_DBL:
		return ("g");
	case BPE_STR:
		return ("s");
	case BPE_PTR:
		return ("p");
	case BPE_UI8ARRAY:
		return ("R");
	case BPE_I16ARRAY:
		return ("H");
	case BPE_UI16ARRAY:
		return ("UH");
	case BPE_I32ARRAY:
		return ("I");
	case BPE_UI32ARRAY:
		return ("U");
	case BPE_I64ARRAY:
		return ("L");
	case BPE_UI64ARRAY:
		return ("UL");
	case BPE_DBLARRAY:
		return ("G");
	case BPE_JSON:
		return ("J");
	}
	return ("");
}

/*
 * Validates the header portion of the parsed json.
 * { "v" : 0, "s" : 0, "f" : 0, "p" : 1, "I" : 123, "X" : 1, "c" : 0, "e" {
 *
 * Returns the offset of the "e' object, or < 0 if error.
 */
static int
processBPHeader(pjsmntok_t *token, const char *text, BridgePacket *bp) {
	char strbuf[JSON_STRING_MAX];
	const char *str;
	uint64_t u64;
	int64_t i64;
	int size;
	int off;

	/* Object */
	if (token->type != JSMN_OBJECT)
		return (-1);

	size = token->size;
	token++;
	for (off = 0; off < size * 2 + 1; off++, token++) {
		str = pjsmn_string(text, token, strbuf, sizeof(strbuf));
		if (str == NULL)
			return (-1);

		off++;
		token++;
		switch (str[0]) {
		case 'p':
			if (pjsmn_uint64(text, token, &u64) != 0)
				return (-1);
			if (u64 > 0xffffffff)
				return (-1);
			bp->vpkt = (uint32_t)u64;
			break;
		case 'v':
			if (pjsmn_uint64(text, token, &u64) != 0)
				return (-1);
			if (u64 != 0)		/* version must be 0 */
				return (-1);
			break;
		case 's':
			if (pjsmn_uint64(text, token, &u64) != 0)
				return (-1);
			if (u64 != BPS_BRIDGE && u64 != BPS_JSON)
				return (-1);
			bp->source = (BridgePacketSource)u64;
			break;
		case 'f':
			if (pjsmn_uint64(text, token, &u64) != 0)
				return (-1);
			bp->flags = (uint32_t)u64;
			break;
		case 'c':
			if (pjsmn_uint64(text, token, &u64) != 0)
				return (-1);
			if (u64 > BRIDGE_PACKET_ENTRY_MAX)
				return (-1);
			bp->entrycnt = (uint16_t)u64;
			break;
		case 'I':
			if (pjsmn_uint64(text, token, &u64) != 0)
				return (-1);
			bp->phidid = u64;
			break;
		case 'O':
			if (pjsmn_uint64(text, token, &u64) != 0)
				return (-1);
			bp->ocid = u64;
			break;
		case 'X':
			if (pjsmn_int64(text, token, &i64) != 0)
				return (-1);
			bp->chidx = (int)i64;
			break;
		case 'e':
			return (off + 2);
		}
	}
	return (-1);	/* should return via 'e' */
}

static int
processBPEntry(pjsmntok_t *token, int entry, const char *text, BridgePacket *bp) {
	char strbuf[JSON_STRING_MAX];
	uint64_t u64;
	int64_t i64;
	double dbl;
	char *str;
	int jsonend;
	int size;
	int off;
	int err;
	int sz;
	int i;

	if (token->type != JSMN_STRING)
		return (-1);

	str = pjsmn_string(text, token, strbuf, sizeof(strbuf));
	if (str == NULL)
		return (-1);

	bp->entry[entry].name = mos_strdup(str, NULL);

	token++;
	if (token->type != JSMN_OBJECT)
		return (-1);

	size = token->size;

	token++;

	u64 = 0;
	i64 = 0;
	dbl = 0;

	for (off = 0; off < size * 2; off++, token++) {
		str = pjsmn_string(text, token, strbuf, sizeof(strbuf));
		if (str == NULL)
			return (-1);

		off++;
		token++;
		switch (str[0]) {
		case 't':
			str = pjsmn_string(text, token, strbuf, sizeof(strbuf));
			if (str == NULL) {
				logerr("invalid type value in bridge packet entry");
				return (-1);
			}
			bp->entry[entry].type = decodeBridgePacketEntryType(str);
			break;
		case 'v':
			switch (bp->entry[entry].type) {
			case BPE_PTR:	/* Unexpected */
				return (-1);
			case BPE_NONE:
				bp->entry[entry].bpe_ui64 = 0;
				break;
			case BPE_UI8:
			case BPE_UI16:
			case BPE_UI32:
			case BPE_UI64:
				err = pjsmn_uint64(text, token, &bp->entry[entry].bpe_ui64);
				if (err != 0) {
					logerr("invalid unsigned integer value in bridge packet entry");
					return (-1);
				}
				break;
			case BPE_I16:
			case BPE_I32:
			case BPE_I64:
				err = pjsmn_int64(text, token, &bp->entry[entry].bpe_i64);
				if (err != 0) {
					logerr("invalid integer value in bridge packet entry");
					return (-1);
				}
				break;
			case BPE_FLOAT:
			case BPE_DBL:
				err = pjsmn_double(text, token, &bp->entry[entry].bpe_dbl);
				if (err != 0) {
					logerr("invalid double value in bridge packet entry");
					return (-1);
				}
				break;
			case BPE_STR:
				str = pjsmn_string(text, token, strbuf, sizeof(strbuf));
				if (str == NULL || strlen(str) > JSON_STRING_MAX) {
					logerr("invalid string value in bridge packet entry");
					return (-1);
				}
				json_unescape(str);
				bp->entry[entry].bpe_ptr = mos_strdup(str, NULL);
				break;
			case BPE_UI8ARRAY:
			case BPE_I16ARRAY:
			case BPE_UI16ARRAY:
			case BPE_I32ARRAY:
			case BPE_UI32ARRAY:
			case BPE_I64ARRAY:
			case BPE_UI64ARRAY:
			case BPE_DBLARRAY:
				if (token->size < 0 || token->size > BPE_MAXARRAY_LEN)
					return (-1);

				allocArray(bp, token->size, bp->entry[entry].type, entry);
				if (token->size > 0) {
					token++;
					for (i = 0; i < bp->entry[entry].bpe_cnt; i++, token++, off++) {
						err = 0;

						switch (bp->entry[entry].type) {
						case BPE_UI8ARRAY:
						case BPE_UI16ARRAY:
						case BPE_UI32ARRAY:
						case BPE_UI64ARRAY:
							err = pjsmn_uint64(text, token, &u64);
							break;
						case BPE_I16ARRAY:
						case BPE_I32ARRAY:
						case BPE_I64ARRAY:
							err = pjsmn_int64(text, token, &i64);
							break;
						case BPE_DBLARRAY:
							err = pjsmn_double(text, token, &dbl);
							break;
						}
						if (err != 0) {
							mos_free(bp->entry[entry].bpe_ptr, bp->entry[entry].bpe_len);
							bp->entry[entry].bpe_len = 0;
							return (-1);
						}
						switch (bp->entry[entry].type) {
						case BPE_UI8ARRAY:
							bp->entry[entry].bpe_ui8array[i] = (uint8_t)u64;
							break;
						case BPE_I16ARRAY:
							bp->entry[entry].bpe_i16array[i] = (int16_t)i64;
							break;
						case BPE_UI16ARRAY:
							bp->entry[entry].bpe_ui16array[i] = (uint16_t)u64;
							break;
						case BPE_I32ARRAY:
							bp->entry[entry].bpe_i32array[i] = (int32_t)i64;
							break;
						case BPE_UI32ARRAY:
							bp->entry[entry].bpe_ui32array[i] = (uint32_t)u64;
							break;
						case BPE_I64ARRAY:
							bp->entry[entry].bpe_i64array[i] = (int64_t)i64;
							break;
						case BPE_UI64ARRAY:
							bp->entry[entry].bpe_ui64array[i] = u64;
							break;
						case BPE_DBLARRAY:
							bp->entry[entry].bpe_dblarray[i] = dbl;
							break;
						}
					}
				}
				break;
			case BPE_JSON:
				sz = token->end - token->start + 1;
				jsonend = token->end;
				if (sz <= 0 || sz > BPE_MAXARRAY_LEN)
					return (-1);
				bp->entry[entry].bpe_len = (uint16_t)sz;
				bp->entry[entry].bpe_ptr = (uint8_t *)mos_malloc(sz);
				mos_strlcpy((char *)bp->entry[entry].bpe_ptr, &text[token->start], sz);
				if (token->start < jsonend) {
					while (token->start < jsonend) {
						token++;
						off++;
					}
					/* we go one too far above */
					token--;
					off--;
				}
				break;
			}
		}
	}
	return (off + 1);
}

/*
 * Uses the token array from the network connection.
 * Network connections have a single read thread so this is expected to be safe.
 */
PhidgetReturnCode
parseBridgePacketJSON(void *tokensIn, BridgePacket **bridgePacket, const char *json,
  uint32_t jsonlen) {
	pjsmntok_t *tokens;
	pjsmn_parser p;
	int res, n;

	BridgePacket *bp;
	uint16_t entry;

	if (tokensIn == NULL)
		tokens = mos_malloc(sizeof(pjsmntok_t) * BRIDGE_JSON_TOKENS);
	else
		tokens = tokensIn;

	*bridgePacket = (BridgePacket *)NULL;
	allocBridgePacket(&bp);
	bp->source = BPS_JSON;

	pjsmn_init(&p);
	res = pjsmn_parse(&p, json, jsonlen, tokens, BRIDGE_JSON_TOKENS);
	if (res < 0)
		goto error;

	res = processBPHeader(tokens, json, bp);
	if (res < 0)
		goto error;

	for (entry = 0; entry < bp->entrycnt; entry++) {
		n = processBPEntry(&tokens[entry + res], entry, json, bp);
		if (n < 0)
			goto error;
		res += n;
	}

	if (tokensIn == NULL)
		mos_free(tokens, sizeof(pjsmntok_t) * BRIDGE_JSON_TOKENS);

	*bridgePacket = bp;
	bridgeloginfo("BPJSON: 0x%x", bp->vpkt);
	return (EPHIDGET_OK);

error:
	if (tokensIn == NULL)
		mos_free(tokens, sizeof(pjsmntok_t) * BRIDGE_JSON_TOKENS);
	destroyBridgePacket(&bp);
	return (EPHIDGET_UNEXPECTED);
}

/*
 * Note: snprintf() on windows is often faster the mos_snprintf(), so use it when we do not need
 * special formatting (this can be called thousands of times a second).
 */
PhidgetReturnCode
renderBridgePacketJSON(BridgePacket *bp, char *buf, uint32_t *bufsz) {
	char tmpbuf[JSON_STRING_MAX];
	const char *je;
	uint32_t off;
	uint64_t u64;
	int64_t i64;
	uint32_t n;
	char *p;
	int w;
	int i;

	/*
	 * The max payload size is NR_MAXDATALEN, so do not bother trying to render any larger.
	 */
	if (*bufsz > NR_MAXDATALEN)
		return (EPHIDGET_INVALIDARG);

#define ADJUSTPTRANDLEN do {						\
	if (w < 0)										\
		return (EPHIDGET_UNEXPECTED);					\
	if (w > (int)n)									\
		return (EPHIDGET_INVALIDARG);						\
	p += w;											\
	n -= w;											\
} while (0)

	p = &buf[0];
	n = *bufsz;
	w = snprintf(p, n,
	  "{\"v\":0,\"s\":%d,\"f\":%u,\"p\":%d,\"I\":%"PRIu64",\"O\":%"PRIu64",\"X\":%d,\"c\":%u,\"e\":{\n",
	  bp->source, bp->flags, bp->vpkt, bp->phidid, bp->ocid, bp->chidx, bp->entrycnt);
	ADJUSTPTRANDLEN;

	for (off = 0; off < bp->entrycnt; off++) {
		if (bp->entry[off].name)
			w = snprintf(p, n, "\"%s\":", bp->entry[off].name);
		else
			w = snprintf(p, n, "\"%u\":", off);
		ADJUSTPTRANDLEN;

		switch (bp->entry[off].type) {
		case BPE_NONE:
			w = snprintf(p, n, "{\"t\":\"%s\"}", bridgePacketEntryType(bp->entry[off].type));
			break;
		case BPE_I16:
		case BPE_I32:
		case BPE_I64:
			w = snprintf(p, n, "{\"t\":\"%s\",\"v\":%"PRId64"}", bridgePacketEntryType(bp->entry[off].type),
			  bp->entry[off].bpe_i64);
			break;
		case BPE_UI8:
		case BPE_UI16:
		case BPE_UI32:
		case BPE_UI64:
			w = snprintf(p, n, "{\"t\":\"%s\",\"v\":%"PRIu64"}", bridgePacketEntryType(bp->entry[off].type),
			  bp->entry[off].bpe_ui64);
			break;
		case BPE_FLOAT:
		case BPE_DBL:
			w = phid_snprintf_c(p, n, "{\"t\":\"%s\",\"v\":%e}", bridgePacketEntryType(bp->entry[off].type),
			  bp->entry[off].bpe_dbl);
			break;
		case BPE_STR:
			je = json_escape(bp->entry[off].bpe_str, tmpbuf, sizeof(tmpbuf));
			if (je == NULL)
				return (EPHIDGET_INVALID);
			w = snprintf(p, n, "{\"t\":\"%s\",\"v\":\"%s\"}", bridgePacketEntryType(bp->entry[off].type), je);
			break;
		case BPE_UI8ARRAY:
		case BPE_UI16ARRAY:
		case BPE_UI32ARRAY:
		case BPE_UI64ARRAY:
			w = snprintf(p, n, "{\"t\":\"%s\",\"v\":[", bridgePacketEntryType(bp->entry[off].type));
			ADJUSTPTRANDLEN;

			u64 = 0;
			for (i = 0; i < bp->entry[off].bpe_cnt; i++) {
				switch (bp->entry[off].type) {
				case BPE_UI8ARRAY:
					u64 = bp->entry[off].bpe_ui8array[i];
					break;
				case BPE_UI16ARRAY:
					u64 = bp->entry[off].bpe_ui16array[i];
					break;
				case BPE_UI32ARRAY:
					u64 = bp->entry[off].bpe_ui32array[i];
					break;
				case BPE_UI64ARRAY:
					u64 = bp->entry[off].bpe_ui64array[i];
					break;
				}
				if (i < bp->entry[off].bpe_cnt - 1)
					w = snprintf(p, n, "%" PRIu64 ",", u64);
				else
					w = snprintf(p, n, "%" PRIu64, u64);
				ADJUSTPTRANDLEN;
			}
			w = snprintf(p, n, "]}");
			break;
		case BPE_I16ARRAY:
		case BPE_I32ARRAY:
		case BPE_I64ARRAY:
			w = snprintf(p, n, "{\"t\":\"%s\",\"v\":[", bridgePacketEntryType(bp->entry[off].type));
			ADJUSTPTRANDLEN;

			i64 = 0;
			for (i = 0; i < bp->entry[off].bpe_cnt; i++) {
				switch (bp->entry[off].type) {
				case BPE_I16ARRAY:
					i64 = bp->entry[off].bpe_i16array[i];
					break;
				case BPE_I32ARRAY:
					i64 = bp->entry[off].bpe_i32array[i];
					break;
				case BPE_I64ARRAY:
					i64 = bp->entry[off].bpe_i64array[i];
					break;
				}
				if (i < bp->entry[off].bpe_cnt - 1)
					w = snprintf(p, n, "%" PRId64 ",", i64);
				else
					w = snprintf(p, n, "%" PRId64, i64);
				ADJUSTPTRANDLEN;
			}

			w = snprintf(p, n, "]}");
			break;
		case BPE_DBLARRAY:
			w = snprintf(p, n, "{\"t\":\"%s\",\"v\":[", bridgePacketEntryType(bp->entry[off].type));
			ADJUSTPTRANDLEN;

			for (i = 0; i < bp->entry[off].bpe_cnt; i++) {
				if (i < bp->entry[off].bpe_cnt - 1)
					w = phid_snprintf_c(p, n, "%e,", bp->entry[off].bpe_dblarray[i]);
				else
					w = phid_snprintf_c(p, n, "%e", bp->entry[off].bpe_dblarray[i]);
				ADJUSTPTRANDLEN;
			}

			w = snprintf(p, n, "]}");
			break;
		case BPE_JSON:
			w = snprintf(p, n, "{\"t\":\"%s\",\"v\":%s}",
			  bridgePacketEntryType(bp->entry[off].type), (const char *)bp->entry[off].bpe_ptr);
			break;
		case BPE_PTR:	/* not rendered in JSON */
			break;
		default:
			return (EPHIDGET_UNSUPPORTED);
		}
		ADJUSTPTRANDLEN;
		if (off < (uint32_t)(bp->entrycnt - 1))
			w = snprintf(p, n, ",\n");
		else
			w = snprintf(p, n, "\n");
		ADJUSTPTRANDLEN;
	}

	if (n < 3)
		return (EPHIDGET_INVALIDARG);

	*p = '}';
	p++;
	*p = '}';
	p++;
	*p = '\0';
	*bufsz = (uint32_t)(p - &buf[0]);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
createBridgePacket(BridgePacket **bridgePacket, bridgepacket_t pkt, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = createBridgePacketv(bridgePacket, pkt, fmt, va);
	va_end(va);

	return (res);
}

PhidgetReturnCode
createBridgePacketv(BridgePacket **bridgePacket, bridgepacket_t pkt, const char *fmt, va_list va) {
	BridgePacket *bp;
	const char *cptr;
	char name[64];
	void *arrptr;
	int nameoff;
	int ch;
	int n;
	int sz;

	*bridgePacket = (BridgePacket *)NULL;

	allocBridgePacket(&bp);
	bp->source = BPS_BRIDGE;
	bp->vpkt = pkt;

	if (fmt == NULL) {
		*bridgePacket = bp;
		return (EPHIDGET_OK);
	}

	for (bp->entrycnt = 0; bp->entrycnt < BRIDGE_PACKET_ENTRY_MAX; bp->entrycnt++) {
		memset(name, 0, sizeof(name));
		nameoff = 0;

		while ((ch = (uint8_t)*fmt++) != '%') {
			if (ch == '\0') {
				*bridgePacket = bp;
				return (EPHIDGET_OK);
			}
			if (ch == ',' || ch == '=')
				continue;
			name[nameoff] = (char)ch;
			nameoff++;
			if (nameoff >= (int)sizeof(name)) {
				destroyBridgePacket(&bp);
				return (EPHIDGET_INVALIDARG);
			}
		}

		if (nameoff != 0)
			bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

		n = 0;

	reswitch:
		switch (ch = (uint8_t)*fmt++) {
		case '*':
			n = va_arg(va, int);
			goto reswitch;
		case '0': case '1': case '2': case '3': case '4':
		case '5': case '6': case '7': case '8': case '9':
			for (n = 0;; ++fmt) {
				n = n * 10 + ch - '0';
				ch = *fmt;
				if (ch < '0' || ch > '9')
					break;
			}
			goto reswitch;
		case 'u':
			switch ((uint8_t)(*fmt)) {
			case 'h':
				bp->entry[bp->entrycnt].type = BPE_UI16;
				bp->entry[bp->entrycnt].bpe_ui64 = va_arg(va, int);
				fmt++;
				break;
			case 'l':
				bp->entry[bp->entrycnt].type = BPE_UI64;
				bp->entry[bp->entrycnt].bpe_ui64 = va_arg(va, uint64_t);
				fmt++;
				break;
			default:
				bp->entry[bp->entrycnt].type = BPE_UI32;
				bp->entry[bp->entrycnt].bpe_ui64 = va_arg(va, uint32_t);
				break;
			}
			break;
		case 'c':
			bp->entry[bp->entrycnt].type = BPE_UI8;
			bp->entry[bp->entrycnt].bpe_ui64 = va_arg(va, int);
			break;
		case 'h':
			bp->entry[bp->entrycnt].type = BPE_I16;
			bp->entry[bp->entrycnt].bpe_i64 = va_arg(va, int);
			break;
		case 'd':
			bp->entry[bp->entrycnt].type = BPE_I32;
			bp->entry[bp->entrycnt].bpe_i64 = va_arg(va, int);
			break;
		case 'l':
			bp->entry[bp->entrycnt].type = BPE_I64;
			bp->entry[bp->entrycnt].bpe_i64 = va_arg(va, int64_t);
			break;
		case 'f':
			bp->entry[bp->entrycnt].type = BPE_FLOAT;
			bp->entry[bp->entrycnt].bpe_dbl = va_arg(va, double);
			break;
		case 'g':
			bp->entry[bp->entrycnt].type = BPE_DBL;
			bp->entry[bp->entrycnt].bpe_dbl = va_arg(va, double);
			break;
		case 's':
			bp->entry[bp->entrycnt].type = BPE_STR;
			cptr = va_arg(va, const char *);
			if (strlen(cptr) > BPE_STR_LEN)
				return (EPHIDGET_INVALIDARG);
			bp->entry[bp->entrycnt].bpe_ptr = mos_strdup(cptr, NULL);
			break;
		case 'p':
			bp->entry[bp->entrycnt].type = BPE_PTR;
			bp->entry[bp->entrycnt].bpe_ptr = va_arg(va, void *);
			break;
		case 'R':
			if (n > BPE_MAXARRAY_LEN) {
				destroyBridgePacket(&bp);
				return (EPHIDGET_INVALIDARG);
			}
			allocArray(bp, n, BPE_UI8ARRAY, bp->entrycnt);
			arrptr = va_arg(va, uint8_t *);
			if (n > 0)
				memcpy(bp->entry[bp->entrycnt].bpe_ui8array, arrptr, n);
			break;
		case 'H':
			sz = n * sizeof(int16_t);
			if (sz > BPE_MAXARRAY_LEN) {
				destroyBridgePacket(&bp);
				return (EPHIDGET_INVALIDARG);
			}
			allocArray(bp, n, BPE_I16ARRAY, bp->entrycnt);
			arrptr = va_arg(va, int16_t *);
			if (n > 0)
				memcpy(bp->entry[bp->entrycnt].bpe_i16array, arrptr, sz);
			break;
		case 'I':
			sz = n * sizeof(int32_t);
			if (sz > BPE_MAXARRAY_LEN) {
				destroyBridgePacket(&bp);
				return (EPHIDGET_INVALIDARG);
			}
			allocArray(bp, n, BPE_I32ARRAY, bp->entrycnt);
			arrptr = va_arg(va, int32_t *);
			if (n > 0)
				memcpy(bp->entry[bp->entrycnt].bpe_i32array, arrptr, sz);
			break;
		case 'U':
			switch ((uint8_t)(*fmt)) {
			case 'H':
				sz = n * sizeof(uint16_t);
				if (sz > BPE_MAXARRAY_LEN) {
					destroyBridgePacket(&bp);
					return (EPHIDGET_INVALIDARG);
				}
				allocArray(bp, n, BPE_UI16ARRAY, bp->entrycnt);
				arrptr = va_arg(va, uint16_t *);
				if (n > 0)
					memcpy(bp->entry[bp->entrycnt].bpe_ui16array, arrptr, sz);
				fmt++;
				break;
			case 'L':
				sz = n * sizeof(uint64_t);
				if (sz > BPE_MAXARRAY_LEN) {
					destroyBridgePacket(&bp);
					return (EPHIDGET_INVALIDARG);
				}
				allocArray(bp, n, BPE_UI64ARRAY, bp->entrycnt);
				arrptr = va_arg(va, uint64_t *);
				if (n > 0)
					memcpy(bp->entry[bp->entrycnt].bpe_ui64array, arrptr, sz);
				fmt++;
				break;
			default:
				sz = n * sizeof(uint32_t);
				if (sz > BPE_MAXARRAY_LEN) {
					destroyBridgePacket(&bp);
					return (EPHIDGET_INVALIDARG);
				}
				allocArray(bp, n, BPE_UI32ARRAY, bp->entrycnt);
				arrptr = va_arg(va, uint32_t *);
				if (n > 0)
					memcpy(bp->entry[bp->entrycnt].bpe_ui32array, arrptr, sz);
				break;
			}
			break;
		case 'L':
			sz = n * sizeof(int64_t);
			if (sz > BPE_MAXARRAY_LEN) {
				destroyBridgePacket(&bp);
				return (EPHIDGET_INVALIDARG);
			}
			allocArray(bp, n, BPE_I64ARRAY, bp->entrycnt);
			arrptr = va_arg(va, int64_t *);
			if (n > 0)
				memcpy(bp->entry[bp->entrycnt].bpe_i64array, arrptr, sz);
			break;
		case 'G':
			sz = n * sizeof(double);
			if (sz > BPE_MAXARRAY_LEN) {
				destroyBridgePacket(&bp);
				return (EPHIDGET_INVALIDARG);
			}
			allocArray(bp, n, BPE_DBLARRAY, bp->entrycnt);
			arrptr = va_arg(va, double *);
			if (n > 0)
				memcpy(bp->entry[bp->entrycnt].bpe_dblarray, arrptr, sz);
			break;
		case 'J':
			cptr = va_arg(va, const char *);
			n = (int)strlen(cptr);
			if (n == 0 || n > BPE_MAXARRAY_LEN) {
				destroyBridgePacket(&bp);
				return (EPHIDGET_INVALIDARG);
			}
			bp->entry[bp->entrycnt].type = BPE_JSON;
			bp->entry[bp->entrycnt].bpe_len = (uint16_t)(n + 1);
			bp->entry[bp->entrycnt].bpe_ptr = (uint8_t *)mos_malloc(n + 1);
			mos_strlcpy((char *)bp->entry[bp->entrycnt].bpe_ptr, cptr, n + 1);
			break;
		default:
			destroyBridgePacket(&bp);
			return (EPHIDGET_INVALIDARG);
		}
	}

	destroyBridgePacket(&bp);

	return (EPHIDGET_2BIG);
}

void
freeBridgePacketEntry(BridgePacketEntry *bpe, int freename) {

	if (freename && bpe->name) {
		mos_free(bpe->name, MOSM_FSTR);
		bpe->name = NULL;
	}

	switch (bpe->type) {
	case BPE_STR:
		mos_free(bpe->bpe_ptr, MOSM_FSTR);
		bpe->bpe_ptr = NULL;
		break;
	case BPE_JSON:
	case BPE_UI8ARRAY:
	case BPE_I16ARRAY:
	case BPE_UI16ARRAY:
	case BPE_I32ARRAY:
	case BPE_UI32ARRAY:
	case BPE_I64ARRAY:
	case BPE_UI64ARRAY:
	case BPE_DBLARRAY:
		if (bpe->bpe_len != 0)
			mos_free(bpe->bpe_ptr, bpe->bpe_len);
		bpe->bpe_ptr = NULL;
		break;
	default:
		break;
	}
}

void
retainBridgePacket(BridgePacket *bp) {

	mos_tlock_lock(bp->lock);
	bp->_refcnt++;
	mos_tlock_unlock(bp->lock);
}

void
destroyBridgePacket(BridgePacket **_bp) {
	BridgePacket *bp;
	int i;

	if (_bp == NULL || *_bp == NULL)
		return;

	bp = *_bp;

	mos_tlock_lock(bp->lock);

	MOS_ASSERT(bp->_refcnt > 0);

	bp->_refcnt--;
	if (bp->_refcnt != 0) {
		mos_tlock_unlock(bp->lock);
		*_bp = NULL;
		return;
	}

	mos_tlock_unlock(bp->lock);
	mos_tlock_destroy(&bp->lock);

	if (bp->nc)
		PhidgetRelease(&bp->nc);

	if (bp->reply_bpe != NULL) {
		freeBridgePacketEntry(bp->reply_bpe, 1);
		mos_free(bp->reply_bpe, sizeof(BridgePacketEntry));
	}

	for (i = 0; i < bp->entrycnt; i++)
		freeBridgePacketEntry(&bp->entry[i], 1);

	if (bp->iop)
		mos_iop_release(&bp->iop);

	mos_free(bp, sizeof(BridgePacket));
	*_bp = (BridgePacket *)NULL;
}

static int
getBridgePacketEntryOffsetByName(BridgePacket *bp, const char *name) {
	int off;

	for (off = 0; off < bp->entrycnt; off++) {
		if (bp->entry[off].name && mos_strcmp(bp->entry[off].name, name) == 0)
			return (off);
	}
	return (-1);
}

int
getBridgePacketArrayCnt(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);

	switch (bp->entry[off].type) {
	case BPE_UI8ARRAY:
	case BPE_I16ARRAY:
	case BPE_UI16ARRAY:
	case BPE_I32ARRAY:
	case BPE_UI32ARRAY:
	case BPE_I64ARRAY:
	case BPE_UI64ARRAY:
	case BPE_DBLARRAY:
		return (bp->entry[off].cnt);
	default:
		MOS_PANIC("Not an array type");
	}
}

int
getBridgePacketArrayLen(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);

	switch (bp->entry[off].type) {
	case BPE_UI8ARRAY:
	case BPE_I16ARRAY:
	case BPE_UI16ARRAY:
	case BPE_I32ARRAY:
	case BPE_UI32ARRAY:
	case BPE_I64ARRAY:
	case BPE_UI64ARRAY:
	case BPE_DBLARRAY:
		return (bp->entry[off].len);
	default:
		MOS_PANIC("Not an array type");
	}
}

int
getBridgePacketArrayCntByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	off = getBridgePacketEntryOffsetByName(bp, name);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);

	switch (bp->entry[off].type) {
	case BPE_UI8ARRAY:
	case BPE_I16ARRAY:
	case BPE_UI16ARRAY:
	case BPE_I32ARRAY:
	case BPE_UI32ARRAY:
	case BPE_I64ARRAY:
	case BPE_UI64ARRAY:
	case BPE_DBLARRAY:
		return (bp->entry[off].cnt);
	default:
		MOS_PANIC("Not an array type");
	}
}

int
getBridgePacketArrayLenByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	off = getBridgePacketEntryOffsetByName(bp, name);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);

	switch (bp->entry[off].type) {
	case BPE_UI8ARRAY:
	case BPE_I16ARRAY:
	case BPE_UI16ARRAY:
	case BPE_I32ARRAY:
	case BPE_UI32ARRAY:
	case BPE_I64ARRAY:
	case BPE_UI64ARRAY:
	case BPE_DBLARRAY:
		return (bp->entry[off].len);
	default:
		MOS_PANIC("Not an array type");
	}
}

/*
 * Encodes base64 data
 * 0: data len
 * 1: encoded length
 * 2: encoded data ...
 */
int
bridgePacketBase64Encode(BridgePacket *bp, const void *data, size_t dataLen) {
	PhidgetReturnCode res;
	char buf[BPE_STR_LEN];
	uint32_t base64Len;
	uint8_t *base64;
	char name[16];
	uint32_t n;
	size_t cnt;
	int i;

	uint32_t bn;
	char *b;

	MOS_ASSERT(bp != NULL);

	base64 = mos_base64_encode(data, (uint32_t)dataLen, &base64Len);
	if (base64 == NULL)
		return (EPHIDGET_UNEXPECTED);

	b = (char *)base64;
	bn = (uint32_t)strlen(b);

	cnt = bn / BPE_STR_LEN;
	if (cnt >= BRIDGE_PACKET_ENTRY_MAX - 1) {
		mos_free(base64, base64Len);
		return (EPHIDGET_NOSPC);
	}

	res = addBridgePacketUInt32(bp, (uint32_t)dataLen, "base64datalen");
	if (res != EPHIDGET_OK) {
		mos_free(base64, base64Len);
		return (res);
	}

	res = addBridgePacketUInt32(bp, bn, "base64enclen");
	if (res != EPHIDGET_OK) {
		mos_free(base64, base64Len);
		return (res);
	}

	for (i = 0; bn > 0; i++) {
		n = MOS_MIN(sizeof(buf) - 1, bn);
		memcpy(buf, b, n);
		buf[n] = '\0';
		bn -= n;
		b += n;

		mos_snprintf(name, sizeof(name), "base64data%d", i);
		res = addBridgePacketString(bp, buf, name);
		if (res != EPHIDGET_OK) {
			mos_free(base64, base64Len);
			return (res);
		}
	}

	return (EPHIDGET_OK);
}

int
bridgePacketBase64Decode(BridgePacket *bp, void *data, size_t *dataLen, int *off) {
	const char *str;
	uint8_t *encbuf;
	uint32_t enclen;
	uint32_t dlen;
	size_t outlen;
	uint32_t len;
	uint8_t *buf;
	uint8_t *out;
	size_t n;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(dataLen != NULL);
	MOS_ASSERT(off != NULL);
	MOS_ASSERT(*off >= 0 && *off < BRIDGE_PACKET_ENTRY_MAX);

	if (*off >= bp->entrycnt - 2)
		return (EPHIDGET_INVALIDARG);

	len = getBridgePacketUInt32(bp, *off);
	if (data == NULL) {
		*dataLen = len;
		return (EPHIDGET_OK);
	}

	if (len > *dataLen)
		return (EPHIDGET_NOSPC);

	(*off)++;

	enclen = getBridgePacketUInt32(bp, *off);
	(*off)++;

	encbuf = mos_malloc(enclen + 1);
	for (out = encbuf, outlen = enclen; outlen > 0 && *off < bp->entrycnt; (*off)++) {
		str = getBridgePacketString(bp, *off);
		n = strlen(str);
		memcpy(out, str, n);
		out += n;
		outlen -= n;
	}
	*out = '\0';

	/*
	 * Make sure we got all of the data.
	 */
	if (outlen != 0) {
		mos_free(encbuf, enclen + 1);
		return (EPHIDGET_INVALID);
	}

	buf = mos_base64_decode(encbuf, enclen, &dlen);
	if (buf == NULL) {
		mos_free(encbuf, enclen + 1);
		return (EPHIDGET_INVALID);
	}

	if (dlen != len) {
		mos_free(encbuf, enclen + 1);
		return (EPHIDGET_UNEXPECTED);
	}

	memcpy(data, buf, dlen);
	*dataLen = dlen;

	mos_free(encbuf, enclen + 1);

	return (EPHIDGET_OK);
}

uint64_t
bridgePacketGetPhidId(BridgePacket *bp) {

	return (bp->phidid);
}

void
bridgePacketSetPhidId(BridgePacket *bp, uint64_t phidid) {

	bp->phidid = phidid;
}

uint64_t
bridgePacketGetOpenChannelId(BridgePacket *bp) {

	return (bp->ocid);
}

void
bridgePacketSetOpenChannelId(BridgePacket *bp, uint64_t ocid) {

	bp->ocid = ocid;
}

int
bridgePacketGetChannelIndex(BridgePacket *bp) {

	return (bp->chidx);
}

void
bridgePacketSetChannelIndex(BridgePacket *bp, int chidx) {

	bp->chidx = chidx;
}

int
bridgePacketIsEvent(BridgePacket *bp) {

	return (bp->flags & BPE_ISEVENT_FLAG);
}

void
bridgePacketSetIsEvent(BridgePacket *bp) {

	bp->flags |= BPE_ISEVENT_FLAG;
}

int
bridgePacketIsReply(BridgePacket *bp) {

	return (bp->flags & BPE_ISREPLY_FLAG);
}

void
bridgePacketSetIsReply(BridgePacket *bp, uint16_t repseq) {

	bp->flags |= BPE_ISREPLY_FLAG;
	bp->repseq = repseq;
}

int
bridgePacketIsFromNet(BridgePacket *bp) {

	return (bp->flags & BPE_ISFROMNET_FLAG);
}

void
bridgePacketSetIsFromNet(BridgePacket *bp, PhidgetNetConnHandle nc) {

	bridgePacketSetNetConn(bp, nc);
	bp->flags |= BPE_ISFROMNET_FLAG;
}

void
bridgePacketSetNetConn(BridgePacket *bp, PhidgetNetConnHandle nc) {

	if (bp->nc)
		PhidgetRelease(&bp->nc);
	bp->nc = nc;
	if (bp->nc)
		PhidgetRetain(bp->nc);
}

/*
 * DOES NOT RETAIN THE CONNECTION HANDLE.
 * The caller must ensure the bridge packet lives as long as their reference.. or create their own.
 */
PhidgetNetConnHandle
bridgePacketGetNetConn(BridgePacket *bp) {

	return (bp->nc);
}

#include "bridge.gen.c"
#include "structio.gen.c"

/***********************************************************************************************************
 * Bridge Communication
 */

static int
allowDataGram(BridgePacket *bp) {

	return (bridgePacketSupportsDataGram(bp->vpkt));
}

/*
 * Renders and sends a bridge packet to the specified network connection.
 */
PhidgetReturnCode
networkSendBridgePacket(PhidgetChannelHandle channel, BridgePacket *bp, PhidgetNetConnHandle nc) {
	PhidgetReturnCode res, rres;
	WaitForReply *wfr;
	uint32_t len;
	char *reply;
	uint8_t *data;
	size_t dataLen;

	wfr = NULL;

	NetConnWriteLock(nc);

	len = nc->databufsz;
	res = renderBridgePacketJSON(bp, nc->databuf, &len);
	if (res != EPHIDGET_OK) {
		NetConnWriteUnlock(nc);
		return (res);
	}

	/*
	 * There is no reply sent for events.
	 */
	if (bridgePacketIsEvent(bp))
		res = writeEvent(MOS_IOP_IGNORE, nc, MSG_DEVICE, SMSG_DEVBRIDGEPKT, NULL, len, allowDataGram(bp));
	else if (bridgePacketIsReply(bp))
		res = writeReply(MOS_IOP_IGNORE, nc, bp->repseq, MSG_DEVICE, SMSG_DEVBRIDGEPKT, NULL, len);
	else
		res = writeRequest(MOS_IOP_IGNORE, nc, 0, MSG_DEVICE, SMSG_DEVBRIDGEPKT, NULL, len, &wfr);
	NetConnWriteUnlock(nc);

	if (res != EPHIDGET_OK)
		return (res);

	if (wfr == NULL)
		return (EPHIDGET_OK);

	if (bp->vpkt == BP_SENDFIRMWARE)
		wfr->waittime = 60000;	/* wait up to 60 seconds for the upgrade to complete */

	res = simpleWaitForReply(&wfr, &rres, &reply, &data, &dataLen);

	MOS_ASSERT(wfr == NULL);
	if (res != EPHIDGET_OK) {
		return (res);
	}

	if (reply != NULL)
		bp->reply_bpe = bridgeCreateReplyBPEfromString(reply);
	else if (data != NULL) {
		bp->reply_bpe = mos_malloc(sizeof(BridgePacketEntry));
		memset(bp->reply_bpe, 0, sizeof(BridgePacketEntry));

		bp->reply_bpe->type = BPE_UI8ARRAY;
		bp->reply_bpe->bpe_len = (uint16_t)dataLen;
		if (dataLen == 0)
			bp->reply_bpe->bpe_ptr = NULL;
		else
			bp->reply_bpe->bpe_ptr = mos_malloc(dataLen);
		bp->reply_bpe->bpe_ui8array = bp->reply_bpe->bpe_ptr;
		bp->reply_bpe->bpe_cnt = (uint16_t)dataLen;

		memcpy(bp->reply_bpe->bpe_ui8array, data, dataLen);

		mos_free(data, dataLen);
	}

	if (rres != EPHIDGET_OK) {
		// in error case, reply contains the error details. Insert into bp iop to report to user.
		if (reply != NULL)
			MOS_ERROR(bp->iop, rres, reply);
	}

	return (rres);
}

/*
 * If the channel is opened by one or more remote clients, forward the bridge packet on to
 * every client other than the one the bridge packet originated from.
 */
PhidgetReturnCode
bridgeSendBPToNetworkChannelsNoWait(PhidgetChannelHandle channel, BridgePacket *bp) {

	/*
	 * Only for channels opened by a network client.
	 */
	if (PhidgetCKFlags(channel, PHIDGET_OPENBYNETCLIENT_FLAG) == 0)
		return (EPHIDGET_OK);

	bridgePacketSetPhidId(bp, (uint64_t)(uintptr_t)channel->parent);
	bridgePacketSetChannelIndex(bp, channel->uniqueIndex);
	bridgePacketSetIsEvent(bp); /* we will not wait for a reply.. */

	return (sendToNetworkConnections(channel, bp, bridgePacketGetNetConn(bp)));
}

/*************************************************************************************************************
 * DEVICE DELIVERY FUNCTIONS
 * Called by channels to deliver bridge packets to the device.
 *
 * All of these bridge packets are dispatch on the inbound queue of the channel.
 *
 * Eventually this results in:
 *	res = PhidgetChannel_bridgeInput(channel, bp);
 *	if (res && networkPhidget(channel))
 *		networkSendBridgePacket(channel, bp, nc);
 *
 * If a callback was provided, the callback function will be called with the result and these functions
 * will not block (or wait for a reply).
 * If a callback was not provided, these functions will block for up to 60 seconds, and check for a reply.
 */
PhidgetReturnCode
bridgeSendBPToDeviceWithReply(PhidgetChannelHandle channel, Phidget_AsyncCallback cb, void *ctx,
  BridgePacket *bp, uint8_t *reply, uint32_t *replylen) {
	PhidgetReturnCode res;
	mosiop_t iop;

	iop = mos_iop_alloc();

	// Give the bridgepacket it's own ref to the iop. It will free in destroyBridgePacket
	mos_iop_retain(iop);
	bp->iop = iop;

	if (isNetworkPhidget(channel))
		bridgePacketSetPhidId(bp, PhidgetDeviceGetNetId(channel->parent));
	else
		bridgePacketSetPhidId(bp, (uint64_t)(uintptr_t)channel->parent);

	retainBridgePacket(bp);
	bridgePacketSetChannelIndex(bp, channel->uniqueIndex);

	res = dispatchUserRequest(channel, bp, cb, ctx);
	if (res != EPHIDGET_OK) {
		destroyBridgePacket(&bp);

		if (cb == NULL)
			return (PHID_RETURN_IOP(res, iop));

		mos_iop_release(&iop);
		return (res);
	}

	if (cb == NULL && reply && replylen && bp->reply_bpe != NULL) {
		if (bp->reply_bpe->type == BPE_STR) {
			mos_strlcpy((char *)reply, bp->reply_bpe->val.str, *replylen);
			*replylen = (int)strlen((char *)reply);
		} else if (bp->reply_bpe->type == BPE_UI8ARRAY) {
			if (*replylen < bp->reply_bpe->len) {
				logerr("Not enough space provided for an array reply.");
				*replylen = 0;
			} else {
				memcpy(reply, bp->reply_bpe->bpe_ui8array, bp->reply_bpe->len);
				*replylen = bp->reply_bpe->len;
			}
		} else {
			logerr("Invalid reply type returned.");
		}
	}

	destroyBridgePacket(&bp);
	mos_iop_release(&iop);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
bridgeSendBPToDevice(PhidgetChannelHandle channel, Phidget_AsyncCallback cb, void *ctx, BridgePacket *bp) {

	return (bridgeSendBPToDeviceWithReply(channel, cb, ctx, bp, NULL, NULL));
}

PhidgetReturnCode
bridgeSendToDeviceWithReply(PhidgetChannelHandle channel, bridgepacket_t pkt, Phidget_AsyncCallback cb,
  void *ctx, uint8_t *reply, uint32_t *replylen, const char *fmt, ...) {
	PhidgetReturnCode res;
	BridgePacket *bp;
	va_list va;

	if (cb)
		TESTPTR(reply);
	else
		TESTPTR_PR(reply);

	va_start(va, fmt);
	res = createBridgePacketv(&bp, pkt, fmt, va);
	va_end(va);

	// Only set last error for non-async
	if (res != EPHIDGET_OK) {
		if (cb)
			return (res);
		return (PHID_RETURN(res));
	}

	return (bridgeSendBPToDeviceWithReply(channel, cb, ctx, bp, reply, replylen));
}

PhidgetReturnCode
bridgeSendToDevice(PhidgetChannelHandle channel, bridgepacket_t pkt, Phidget_AsyncCallback cb, void *ctx,
  const char *fmt, ...) {
	PhidgetReturnCode res;
	BridgePacket *bp;
	va_list va;

	va_start(va, fmt);
	res = createBridgePacketv(&bp, pkt, fmt, va);
	va_end(va);

	// Only set last error for non-async
	if (res != EPHIDGET_OK) {
		if (cb)
			return (res);
		return (PHID_RETURN(res));
	}

	return (bridgeSendBPToDevice(channel, cb, ctx, bp));
}


/*************************************************************************************************************
 * CHANNEL DELIVERY FUNCTIONS
 * These functions are intended to be called by device code delivering data to channels.
 *
 * Bridge packets are only delivered to channels that are attached.
 *
 * Bridge packet references are assumed by these functions, and a caller who wishes to maintain a
 * reference must do so before hand.
 *
 * Results in:
 *	res = PhidgetChannel_bridgeInput(channel, bp)
 *	if (res == EPHIDGET_OK)
 *		bridgeSendBPToNetworkChannelsNoWait(channel, bp);
 */

PhidgetReturnCode
bridgeSendBPToChannelNC(PhidgetChannelHandle channel, PhidgetNetConnHandle nc, BridgePacket *bp) {

	if (!ISATTACHED(channel))
		return (EPHIDGET_OK);

	if (nc)
		bridgePacketSetNetConn(bp, nc);

	/* dispatcher is responsible for destroying the bridge packet */
	return (dispatchChannelBridgePacket(channel, bp));
}

PhidgetReturnCode
bridgeSendBPToChannel(PhidgetChannelHandle channel, BridgePacket *bp) {

	return (bridgeSendBPToChannelNC(channel, NULL, bp));
}

PhidgetReturnCode
bridgeSendToChannelNC(PhidgetChannelHandle channel, PhidgetNetConnHandle nc, bridgepacket_t pkt,
  const char *fmt, ...) {
	PhidgetReturnCode err;
	BridgePacket *bp;
	va_list va;

	va_start(va, fmt);
	err = createBridgePacketv(&bp, pkt, fmt, va);
	va_end(va);
	if (err)
		return (err);

	return (bridgeSendBPToChannelNC(channel, nc, bp));
}

PhidgetReturnCode
bridgeSendToChannel(PhidgetChannelHandle channel, bridgepacket_t pkt, const char *fmt, ...) {
	PhidgetReturnCode err;
	BridgePacket *bp;
	va_list va;

	va_start(va, fmt);
	err = createBridgePacketv(&bp, pkt, fmt, va);
	va_end(va);
	if (err)
		return (err);

	return (bridgeSendBPToChannelNC(channel, NULL, bp));
}

BridgePacketEntry *
bridgeCreateReplyBPEfromString(char *str) {

	BridgePacketEntry *reply_bpe = mos_malloc(sizeof(BridgePacketEntry));
	memset(reply_bpe, 0, sizeof(BridgePacketEntry));
	reply_bpe->type = BPE_STR;
	reply_bpe->bpe_str = str;

	return reply_bpe;
}