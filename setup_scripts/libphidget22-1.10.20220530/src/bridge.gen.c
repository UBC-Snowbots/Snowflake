
int
hasBridgePacket(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	return (off >= 0 && off < bp->entrycnt);
}

int
hasBridgePacketByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);
	return (off >= 0 && off < bp->entrycnt);
}

const char *
getBridgePacketString(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_STR);

	return ((const char *)bp->entry[off].bpe_str);
}

const char *
getBridgePacketStringByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_STR);

	return ((const char *)bp->entry[off].bpe_str);
}

PhidgetReturnCode
addBridgePacketString(BridgePacket *bp, const char * val, const char *name) {

	TESTPTR(bp);
	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	bp->entry[bp->entrycnt].type = BPE_STR;
	bp->entry[bp->entrycnt].bpe_str = mos_strdup(val, NULL);
	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketString(BridgePacket *bp, const char * val, int off) {

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	TESTPTR(bp);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_STR;
	bp->entry[off].bpe_str = mos_strdup(val, NULL);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketStringByName(BridgePacket *bp, const char * val, const char *name) {
int off;

	TESTPTR(name);
	TESTPTR(bp);

	if ((off = getBridgePacketEntryOffsetByName(bp, name)) == -1)
		return (EPHIDGET_NOENT);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_STR;
	bp->entry[off].bpe_str = mos_strdup(val, NULL);

	return (EPHIDGET_OK);
}

void *
getBridgePacketPtr(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_PTR);

	return ((void *)bp->entry[off].bpe_ptr);
}

void *
getBridgePacketPtrByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_PTR);

	return ((void *)bp->entry[off].bpe_ptr);
}

PhidgetReturnCode
addBridgePacketPtr(BridgePacket *bp, void * val, const char *name) {

	TESTPTR(bp);
	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	bp->entry[bp->entrycnt].type = BPE_PTR;
	bp->entry[bp->entrycnt].bpe_ptr = val;
	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketPtr(BridgePacket *bp, void * val, int off) {

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	TESTPTR(bp);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_PTR;
	bp->entry[off].bpe_ptr = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketPtrByName(BridgePacket *bp, void * val, const char *name) {
int off;

	TESTPTR(name);
	TESTPTR(bp);

	if ((off = getBridgePacketEntryOffsetByName(bp, name)) == -1)
		return (EPHIDGET_NOENT);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_PTR;
	bp->entry[off].bpe_ptr = val;

	return (EPHIDGET_OK);
}

uint8_t
getBridgePacketUInt8(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI8);

	return ((uint8_t)bp->entry[off].bpe_ui64);
}

uint8_t
getBridgePacketUInt8ByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI8);

	return ((uint8_t)bp->entry[off].bpe_ui64);
}

const uint8_t *
getBridgePacketUInt8Array(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI8ARRAY);

	return (bp->entry[off].bpe_ui8array);
}

const uint8_t *
getBridgePacketUInt8ArrayByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI8ARRAY);

	return (bp->entry[off].bpe_ui8array);
}

PhidgetReturnCode
addBridgePacketUInt8(BridgePacket *bp, uint8_t val, const char *name) {

	TESTPTR(bp);
	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	bp->entry[bp->entrycnt].type = BPE_UI8;
	bp->entry[bp->entrycnt].bpe_ui64 = val;
	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketUInt8(BridgePacket *bp, uint8_t val, int off) {

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	TESTPTR(bp);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_UI8;
	bp->entry[off].bpe_ui64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketUInt8ByName(BridgePacket *bp, uint8_t val, const char *name) {
int off;

	TESTPTR(name);
	TESTPTR(bp);

	if ((off = getBridgePacketEntryOffsetByName(bp, name)) == -1)
		return (EPHIDGET_NOENT);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_UI8;
	bp->entry[off].bpe_ui64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
addBridgePacketUInt8Array(BridgePacket *bp, uint8_t *val, uint32_t cnt, const char *name) {

	TESTPTR(bp);
	TESTPTR(val);

	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	allocArray(bp, cnt, BPE_UI8ARRAY, -1);
	MOS_ASSERT(bp->entry[bp->entrycnt].bpe_cnt == cnt);
	memcpy(bp->entry[bp->entrycnt].bpe_ui8array, val, cnt * sizeof (uint8_t));

	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

int16_t
getBridgePacketInt16(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I16);

	return ((int16_t)bp->entry[off].bpe_i64);
}

int16_t
getBridgePacketInt16ByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I16);

	return ((int16_t)bp->entry[off].bpe_i64);
}

const int16_t *
getBridgePacketInt16Array(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I16ARRAY);

	return (bp->entry[off].bpe_i16array);
}

const int16_t *
getBridgePacketInt16ArrayByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I16ARRAY);

	return (bp->entry[off].bpe_i16array);
}

PhidgetReturnCode
addBridgePacketInt16(BridgePacket *bp, int16_t val, const char *name) {

	TESTPTR(bp);
	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	bp->entry[bp->entrycnt].type = BPE_I16;
	bp->entry[bp->entrycnt].bpe_i64 = val;
	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketInt16(BridgePacket *bp, int16_t val, int off) {

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	TESTPTR(bp);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_I16;
	bp->entry[off].bpe_i64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketInt16ByName(BridgePacket *bp, int16_t val, const char *name) {
int off;

	TESTPTR(name);
	TESTPTR(bp);

	if ((off = getBridgePacketEntryOffsetByName(bp, name)) == -1)
		return (EPHIDGET_NOENT);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_I16;
	bp->entry[off].bpe_i64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
addBridgePacketInt16Array(BridgePacket *bp, int16_t *val, uint32_t cnt, const char *name) {

	TESTPTR(bp);
	TESTPTR(val);

	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	allocArray(bp, cnt, BPE_I16ARRAY, -1);
	MOS_ASSERT(bp->entry[bp->entrycnt].bpe_cnt == cnt);
	memcpy(bp->entry[bp->entrycnt].bpe_i16array, val, cnt * sizeof (int16_t));

	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

uint16_t
getBridgePacketUInt16(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI16);

	return ((uint16_t)bp->entry[off].bpe_ui64);
}

uint16_t
getBridgePacketUInt16ByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI16);

	return ((uint16_t)bp->entry[off].bpe_ui64);
}

const uint16_t *
getBridgePacketUInt16Array(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI16ARRAY);

	return (bp->entry[off].bpe_ui16array);
}

const uint16_t *
getBridgePacketUInt16ArrayByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI16ARRAY);

	return (bp->entry[off].bpe_ui16array);
}

PhidgetReturnCode
addBridgePacketUInt16(BridgePacket *bp, uint16_t val, const char *name) {

	TESTPTR(bp);
	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	bp->entry[bp->entrycnt].type = BPE_UI16;
	bp->entry[bp->entrycnt].bpe_ui64 = val;
	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketUInt16(BridgePacket *bp, uint16_t val, int off) {

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	TESTPTR(bp);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_UI16;
	bp->entry[off].bpe_ui64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketUInt16ByName(BridgePacket *bp, uint16_t val, const char *name) {
int off;

	TESTPTR(name);
	TESTPTR(bp);

	if ((off = getBridgePacketEntryOffsetByName(bp, name)) == -1)
		return (EPHIDGET_NOENT);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_UI16;
	bp->entry[off].bpe_ui64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
addBridgePacketUInt16Array(BridgePacket *bp, uint16_t *val, uint32_t cnt, const char *name) {

	TESTPTR(bp);
	TESTPTR(val);

	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	allocArray(bp, cnt, BPE_UI16ARRAY, -1);
	MOS_ASSERT(bp->entry[bp->entrycnt].bpe_cnt == cnt);
	memcpy(bp->entry[bp->entrycnt].bpe_ui16array, val, cnt * sizeof (uint16_t));

	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

int32_t
getBridgePacketInt32(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I32);

	return ((int32_t)bp->entry[off].bpe_i64);
}

int32_t
getBridgePacketInt32ByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I32);

	return ((int32_t)bp->entry[off].bpe_i64);
}

const int32_t *
getBridgePacketInt32Array(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I32ARRAY);

	return (bp->entry[off].bpe_i32array);
}

const int32_t *
getBridgePacketInt32ArrayByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I32ARRAY);

	return (bp->entry[off].bpe_i32array);
}

PhidgetReturnCode
addBridgePacketInt32(BridgePacket *bp, int32_t val, const char *name) {

	TESTPTR(bp);
	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	bp->entry[bp->entrycnt].type = BPE_I32;
	bp->entry[bp->entrycnt].bpe_i64 = val;
	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketInt32(BridgePacket *bp, int32_t val, int off) {

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	TESTPTR(bp);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_I32;
	bp->entry[off].bpe_i64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketInt32ByName(BridgePacket *bp, int32_t val, const char *name) {
int off;

	TESTPTR(name);
	TESTPTR(bp);

	if ((off = getBridgePacketEntryOffsetByName(bp, name)) == -1)
		return (EPHIDGET_NOENT);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_I32;
	bp->entry[off].bpe_i64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
addBridgePacketInt32Array(BridgePacket *bp, int32_t *val, uint32_t cnt, const char *name) {

	TESTPTR(bp);
	TESTPTR(val);

	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	allocArray(bp, cnt, BPE_I32ARRAY, -1);
	MOS_ASSERT(bp->entry[bp->entrycnt].bpe_cnt == cnt);
	memcpy(bp->entry[bp->entrycnt].bpe_i32array, val, cnt * sizeof (int32_t));

	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

uint32_t
getBridgePacketUInt32(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI32);

	return ((uint32_t)bp->entry[off].bpe_ui64);
}

uint32_t
getBridgePacketUInt32ByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI32);

	return ((uint32_t)bp->entry[off].bpe_ui64);
}

const uint32_t *
getBridgePacketUInt32Array(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI32ARRAY);

	return (bp->entry[off].bpe_ui32array);
}

const uint32_t *
getBridgePacketUInt32ArrayByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI32ARRAY);

	return (bp->entry[off].bpe_ui32array);
}

PhidgetReturnCode
addBridgePacketUInt32(BridgePacket *bp, uint32_t val, const char *name) {

	TESTPTR(bp);
	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	bp->entry[bp->entrycnt].type = BPE_UI32;
	bp->entry[bp->entrycnt].bpe_ui64 = val;
	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketUInt32(BridgePacket *bp, uint32_t val, int off) {

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	TESTPTR(bp);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_UI32;
	bp->entry[off].bpe_ui64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketUInt32ByName(BridgePacket *bp, uint32_t val, const char *name) {
int off;

	TESTPTR(name);
	TESTPTR(bp);

	if ((off = getBridgePacketEntryOffsetByName(bp, name)) == -1)
		return (EPHIDGET_NOENT);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_UI32;
	bp->entry[off].bpe_ui64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
addBridgePacketUInt32Array(BridgePacket *bp, uint32_t *val, uint32_t cnt, const char *name) {

	TESTPTR(bp);
	TESTPTR(val);

	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	allocArray(bp, cnt, BPE_UI32ARRAY, -1);
	MOS_ASSERT(bp->entry[bp->entrycnt].bpe_cnt == cnt);
	memcpy(bp->entry[bp->entrycnt].bpe_ui32array, val, cnt * sizeof (uint32_t));

	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

int64_t
getBridgePacketInt64(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I64);

	return ((int64_t)bp->entry[off].bpe_i64);
}

int64_t
getBridgePacketInt64ByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I64);

	return ((int64_t)bp->entry[off].bpe_i64);
}

const int64_t *
getBridgePacketInt64Array(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I64ARRAY);

	return (bp->entry[off].bpe_i64array);
}

const int64_t *
getBridgePacketInt64ArrayByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_I64ARRAY);

	return (bp->entry[off].bpe_i64array);
}

PhidgetReturnCode
addBridgePacketInt64(BridgePacket *bp, int64_t val, const char *name) {

	TESTPTR(bp);
	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	bp->entry[bp->entrycnt].type = BPE_I64;
	bp->entry[bp->entrycnt].bpe_i64 = val;
	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketInt64(BridgePacket *bp, int64_t val, int off) {

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	TESTPTR(bp);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_I64;
	bp->entry[off].bpe_i64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketInt64ByName(BridgePacket *bp, int64_t val, const char *name) {
int off;

	TESTPTR(name);
	TESTPTR(bp);

	if ((off = getBridgePacketEntryOffsetByName(bp, name)) == -1)
		return (EPHIDGET_NOENT);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_I64;
	bp->entry[off].bpe_i64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
addBridgePacketInt64Array(BridgePacket *bp, int64_t *val, uint32_t cnt, const char *name) {

	TESTPTR(bp);
	TESTPTR(val);

	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	allocArray(bp, cnt, BPE_I64ARRAY, -1);
	MOS_ASSERT(bp->entry[bp->entrycnt].bpe_cnt == cnt);
	memcpy(bp->entry[bp->entrycnt].bpe_i64array, val, cnt * sizeof (int64_t));

	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

uint64_t
getBridgePacketUInt64(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI64);

	return ((uint64_t)bp->entry[off].bpe_ui64);
}

uint64_t
getBridgePacketUInt64ByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI64);

	return ((uint64_t)bp->entry[off].bpe_ui64);
}

const uint64_t *
getBridgePacketUInt64Array(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI64ARRAY);

	return (bp->entry[off].bpe_ui64array);
}

const uint64_t *
getBridgePacketUInt64ArrayByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_UI64ARRAY);

	return (bp->entry[off].bpe_ui64array);
}

PhidgetReturnCode
addBridgePacketUInt64(BridgePacket *bp, uint64_t val, const char *name) {

	TESTPTR(bp);
	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	bp->entry[bp->entrycnt].type = BPE_UI64;
	bp->entry[bp->entrycnt].bpe_ui64 = val;
	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketUInt64(BridgePacket *bp, uint64_t val, int off) {

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	TESTPTR(bp);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_UI64;
	bp->entry[off].bpe_ui64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketUInt64ByName(BridgePacket *bp, uint64_t val, const char *name) {
int off;

	TESTPTR(name);
	TESTPTR(bp);

	if ((off = getBridgePacketEntryOffsetByName(bp, name)) == -1)
		return (EPHIDGET_NOENT);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_UI64;
	bp->entry[off].bpe_ui64 = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
addBridgePacketUInt64Array(BridgePacket *bp, uint64_t *val, uint32_t cnt, const char *name) {

	TESTPTR(bp);
	TESTPTR(val);

	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	allocArray(bp, cnt, BPE_UI64ARRAY, -1);
	MOS_ASSERT(bp->entry[bp->entrycnt].bpe_cnt == cnt);
	memcpy(bp->entry[bp->entrycnt].bpe_ui64array, val, cnt * sizeof (uint64_t));

	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

double
getBridgePacketDouble(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_DBL);

	return ((double)bp->entry[off].bpe_dbl);
}

double
getBridgePacketDoubleByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_DBL);

	return ((double)bp->entry[off].bpe_dbl);
}

const double *
getBridgePacketDoubleArray(BridgePacket *bp, int off) {

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_DBLARRAY);

	return (bp->entry[off].bpe_dblarray);
}

const double *
getBridgePacketDoubleArrayByName(BridgePacket *bp, const char *name) {
	int off;

	MOS_ASSERT(bp != NULL);
	MOS_ASSERT(name != NULL);

	off = getBridgePacketEntryOffsetByName(bp, name);

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	MOS_ASSERT(bp->entry[off].type == BPE_DBLARRAY);

	return (bp->entry[off].bpe_dblarray);
}

PhidgetReturnCode
addBridgePacketDouble(BridgePacket *bp, double val, const char *name) {

	TESTPTR(bp);
	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	bp->entry[bp->entrycnt].type = BPE_DBL;
	bp->entry[bp->entrycnt].bpe_dbl = val;
	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketDouble(BridgePacket *bp, double val, int off) {

	MOS_ASSERT(off >= 0 && off < bp->entrycnt);
	TESTPTR(bp);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_DBL;
	bp->entry[off].bpe_dbl = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
setBridgePacketDoubleByName(BridgePacket *bp, double val, const char *name) {
int off;

	TESTPTR(name);
	TESTPTR(bp);

	if ((off = getBridgePacketEntryOffsetByName(bp, name)) == -1)
		return (EPHIDGET_NOENT);

	freeBridgePacketEntry(&bp->entry[off], 0);
	bp->entry[off].type = BPE_DBL;
	bp->entry[off].bpe_dbl = val;

	return (EPHIDGET_OK);
}

PhidgetReturnCode
addBridgePacketDoubleArray(BridgePacket *bp, double *val, uint32_t cnt, const char *name) {

	TESTPTR(bp);
	TESTPTR(val);

	if (bp->entrycnt >= BRIDGE_PACKET_ENTRY_MAX)
		return (EPHIDGET_NOSPC);

	if (name != NULL && getBridgePacketEntryOffsetByName(bp, name) != -1)
		return (EPHIDGET_DUPLICATE);

	allocArray(bp, cnt, BPE_DBLARRAY, -1);
	MOS_ASSERT(bp->entry[bp->entrycnt].bpe_cnt == cnt);
	memcpy(bp->entry[bp->entrycnt].bpe_dblarray, val, cnt * sizeof (double));

	if (name != NULL)
		bp->entry[bp->entrycnt].name = mos_strdup(name, NULL);

	bp->entrycnt++;

	return (EPHIDGET_OK);
}
