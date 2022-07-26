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

#ifndef __BRIDGE_H_
#define __BRIDGE_H_

#include <stdint.h>

#include "mos/mos_tlock.h"
#include "constants.h"
#include "macros.h"
#include "phidget.h"
#include "object.h"

#include "util/json.h"

#include "bridgepackets.gen.h"

#if 0
#define BP_PRINTF_LIKE(a, b) PRINTF_LIKE(a, b)
#else
#define BP_PRINTF_LIKE(a, b)
#endif

int supportedBridgePacket(PhidgetChannelHandle, bridgepacket_t);

#define BRIDGE_JSON_TOKENS		10000	/* Allocated inside of netconn, used by jsmn parser */

#define BRIDGE_PACKET_ENTRY_MAX	36		/* makes each packet ~1k in memory, with 32B entries */

#define BPE_MAXARRAY_LEN		8192	/* cannot be more the how many tokens we allow */
#define BPE_STR_LEN				(JSON_STRING_MAX / 2)

#define BPE_ISEVENT_FLAG		0x01
#define BPE_ISFROMNET_FLAG		0x02
#define BPE_ISREPLY_FLAG		0x04	/* Bridge Packet was read from the network */

/* Order is important */
typedef enum _BridgePacketEntryType {
	BPE_NONE = 0,
	BPE_UI8,
	BPE_I16,
	BPE_UI16,
	BPE_I32,
	BPE_UI32,
	BPE_I64,
	BPE_UI64,
	BPE_FLOAT,
	BPE_DBL,
	BPE_STR,
	BPE_PTR,
	BPE_UI8ARRAY,
	BPE_I16ARRAY,
	BPE_UI16ARRAY,
	BPE_I32ARRAY,
	BPE_UI32ARRAY,
	BPE_I64ARRAY,
	BPE_UI64ARRAY,
	BPE_DBLARRAY,
	BPE_JSON
} BridgePacketEntryType;

typedef enum _BridgePacketSource {
	BPS_BRIDGE = 1,
	BPS_JSON = 2
} BridgePacketSource;

typedef struct _BridgePacketUInt8Array {
	uint8_t		*buf;
} BridgePacketUInt8Array;

typedef struct _BridgePacketInt16Array {
	int16_t		*buf;
} BridgePacketInt16Array;

typedef struct _BridgePacketUInt16Array {
	uint16_t	*buf;
} BridgePacketUInt16Array;

typedef struct _BridgePacketInt32Array {
	int32_t		*buf;
} BridgePacketInt32Array;

typedef struct _BridgePacketUInt32Array {
	uint32_t	*buf;
} BridgePacketUInt32Array;

typedef struct _BridgePacketInt64Array {
	int64_t		*buf;
} BridgePacketInt64Array;

typedef struct _BridgePacketUInt64Array {
	uint64_t	*buf;
} BridgePacketUInt64Array;

typedef struct _BridgePacketDblArray {
	double		*buf;
} BridgePacketDblArray;

typedef union _BridgePacketValue {
	const char				*str;
	int64_t					i64;
	uint64_t				ui64;
	double					dbl;
	void					*ptr;
	BridgePacketUInt8Array	ui8array;
	BridgePacketInt16Array	i16array;
	BridgePacketInt32Array	i32array;
	BridgePacketInt64Array	i64array;
	BridgePacketUInt16Array	ui16array;
	BridgePacketUInt32Array	ui32array;
	BridgePacketUInt64Array	ui64array;
	BridgePacketDblArray	dblarray;
} BridgePacketValue;

typedef struct _BridgePacketEntry {
	BridgePacketEntryType	type;
	char					*name;
	uint16_t				cnt;	/* the number of types elements */
	uint16_t				len;	/* ptr length (number of bytes allocated) */
	BridgePacketValue		val;
#define bpe_str val.str
#define bpe_i64 val.i64
#define bpe_ui64 val.ui64
#define bpe_dbl val.dbl
#define bpe_ptr val.ptr
#define bpe_len len
#define bpe_cnt cnt
#define bpe_ui8array val.ui8array.buf
#define bpe_i16array val.i16array.buf
#define bpe_ui16array val.ui16array.buf
#define bpe_i32array val.i32array.buf
#define bpe_ui32array val.ui32array.buf
#define bpe_i64array val.i64array.buf
#define bpe_ui64array val.ui64array.buf
#define bpe_dblarray val.dblarray.buf
} BridgePacketEntry;

typedef struct _BridgePacket {
	BridgePacketSource	source;
	bridgepacket_t		vpkt;
	uint32_t			flags;
	uint64_t			phidid;		/* phidget id */
	uint64_t			ocid;		/* open channel id */
	int					chidx;		/* channel index */
	uint16_t			entrycnt;
	BridgePacketEntry	entry[BRIDGE_PACKET_ENTRY_MAX];
	uint16_t			_refcnt;	/* should not be modified without the lock: flag private */
	mos_tlock_t			*lock;
	uint16_t			repseq;		/* used to handle reply (so we reply to the correct command) */
	BridgePacketEntry	*reply_bpe;	/* used within the library to return results from a packet command */
	void				*nc;		/* network connection that packet came from */
	mosiop_t			iop;
} BridgePacket;

#include "network/network.h" /* include after structure definitions */

PhidgetReturnCode createBridgePacketv(BridgePacket **, bridgepacket_t, const char *, va_list);
PhidgetReturnCode createBridgePacket(BridgePacket **, bridgepacket_t, const char *, ...) BP_PRINTF_LIKE(3, 4);
PhidgetReturnCode renderBridgePacketJSON(BridgePacket *, char *, uint32_t *);
PhidgetReturnCode parseBridgePacketJSON(void *tokens, BridgePacket **, const char *, uint32_t);
void freeBridgePacketEntry(BridgePacketEntry *, int);
void destroyBridgePacket(BridgePacket **);
void retainBridgePacket(BridgePacket *);

PhidgetReturnCode bridgeSendBPToDeviceWithReply(PhidgetChannelHandle, Phidget_AsyncCallback, void *,
  BridgePacket *, uint8_t *, uint32_t *);
PhidgetReturnCode bridgeSendBPToDevice(PhidgetChannelHandle, Phidget_AsyncCallback, void *, BridgePacket *);
PhidgetReturnCode bridgeSendToDevice(PhidgetChannelHandle, bridgepacket_t, Phidget_AsyncCallback, void *,
  const char *, ...) BP_PRINTF_LIKE(5, 6);
PhidgetReturnCode bridgeSendToDeviceWithReply(PhidgetChannelHandle, bridgepacket_t, Phidget_AsyncCallback,
  void *uptr, uint8_t *reply, uint32_t *replylen, const char *, ...) BP_PRINTF_LIKE(7, 8);
PhidgetReturnCode bridgeSendBPToNetworkChannelsNoWait(PhidgetChannelHandle, BridgePacket *);
PhidgetReturnCode bridgeSendToChannel(PhidgetChannelHandle, bridgepacket_t, const char *, ...) BP_PRINTF_LIKE(3, 4);
PhidgetReturnCode bridgeSendBPToChannel(PhidgetChannelHandle, BridgePacket *);
PhidgetReturnCode bridgeSendToChannelNC(PhidgetChannelHandle, PhidgetNetConnHandle, bridgepacket_t,
  const char *, ...) BP_PRINTF_LIKE(4, 5);
PhidgetReturnCode bridgeSendBPToChannelNC(PhidgetChannelHandle, PhidgetNetConnHandle, BridgePacket *);
PhidgetReturnCode networkSendBridgePacket(PhidgetChannelHandle, BridgePacket *, PhidgetNetConnHandle);

PhidgetReturnCode dispatchChannelBridgePacket(PhidgetChannelHandle, BridgePacket *);

int getBridgePacketArrayCnt(BridgePacket *bp, int off);
int getBridgePacketArrayLen(BridgePacket *bp, int off);
int getBridgePacketArrayCntByName(BridgePacket *bp, const char *);
int getBridgePacketArrayLenByName(BridgePacket *bp, const char *);

int bridgePacketBase64Encode(BridgePacket *, const void *, size_t);
int bridgePacketBase64Decode(BridgePacket *, void *, size_t *, int *);

void bridgePacketSetPhidId(BridgePacket *, uint64_t);
uint64_t bridgePacketGetPhidId(BridgePacket *);
void bridgePacketSetOpenChannelId(BridgePacket *, uint64_t);
uint64_t bridgePacketGetOpenChannelId(BridgePacket *);
void bridgePacketSetChannelIndex(BridgePacket *, int);
void bridgePacketSetSendId(BridgePacket *bp, int sendid);
int bridgePacketGetChannelIndex(BridgePacket *);

int bridgePacketIsReply(BridgePacket *);
void bridgePacketSetIsReply(BridgePacket *, uint16_t);
int bridgePacketIsEvent(BridgePacket *);
void bridgePacketSetIsEvent(BridgePacket *);
int bridgePacketIsFromNet(BridgePacket *);
void bridgePacketSetIsFromNet(BridgePacket *, PhidgetNetConnHandle);

PhidgetNetConnHandle bridgePacketGetNetConn(BridgePacket *);
void bridgePacketSetNetConn(BridgePacket *, PhidgetNetConnHandle);

BridgePacketEntry *bridgeCreateReplyBPEfromString(char *str);

#include "bridge.gen.h"
#include "structio.gen.h"

#endif /* __BRIDGE_H_ */
