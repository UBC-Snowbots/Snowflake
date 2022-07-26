int hasBridgePacket(BridgePacket *bp, int off);
int hasBridgePacketByName(BridgePacket *bp, const char *name);
const char * getBridgePacketString(BridgePacket *bp, int off);
const char * getBridgePacketStringByName(BridgePacket *bp, const char *name);
PhidgetReturnCode addBridgePacketString(BridgePacket *bp, const char * val, const char *name);
PhidgetReturnCode setBridgePacketString(BridgePacket *bp, const char * val, int off);
PhidgetReturnCode setBridgePacketStringByName(BridgePacket *bp, const char * val, const char *name);
void * getBridgePacketPtr(BridgePacket *bp, int off);
void * getBridgePacketPtrByName(BridgePacket *bp, const char *name);
PhidgetReturnCode addBridgePacketPtr(BridgePacket *bp, void * val, const char *name);
PhidgetReturnCode setBridgePacketPtr(BridgePacket *bp, void * val, int off);
PhidgetReturnCode setBridgePacketPtrByName(BridgePacket *bp, void * val, const char *name);
uint8_t getBridgePacketUInt8(BridgePacket *bp, int off);
uint8_t getBridgePacketUInt8ByName(BridgePacket *bp, const char *name);
const uint8_t *getBridgePacketUInt8Array(BridgePacket *bp, int off);
const uint8_t *getBridgePacketUInt8ArrayByName(BridgePacket *bp, const char *name);
PhidgetReturnCode addBridgePacketUInt8(BridgePacket *bp, uint8_t val, const char *name);
PhidgetReturnCode setBridgePacketUInt8(BridgePacket *bp, uint8_t val, int off);
PhidgetReturnCode setBridgePacketUInt8ByName(BridgePacket *bp, uint8_t val, const char *name);
PhidgetReturnCode addBridgePacketUInt8Array(BridgePacket *bp, uint8_t *val, uint32_t cnt,
  const char *name);
int16_t getBridgePacketInt16(BridgePacket *bp, int off);
int16_t getBridgePacketInt16ByName(BridgePacket *bp, const char *name);
const int16_t *getBridgePacketInt16Array(BridgePacket *bp, int off);
const int16_t *getBridgePacketInt16ArrayByName(BridgePacket *bp, const char *name);
PhidgetReturnCode addBridgePacketInt16(BridgePacket *bp, int16_t val, const char *name);
PhidgetReturnCode setBridgePacketInt16(BridgePacket *bp, int16_t val, int off);
PhidgetReturnCode setBridgePacketInt16ByName(BridgePacket *bp, int16_t val, const char *name);
PhidgetReturnCode addBridgePacketInt16Array(BridgePacket *bp, int16_t *val, uint32_t cnt,
  const char *name);
uint16_t getBridgePacketUInt16(BridgePacket *bp, int off);
uint16_t getBridgePacketUInt16ByName(BridgePacket *bp, const char *name);
const uint16_t *getBridgePacketUInt16Array(BridgePacket *bp, int off);
const uint16_t *getBridgePacketUInt16ArrayByName(BridgePacket *bp, const char *name);
PhidgetReturnCode addBridgePacketUInt16(BridgePacket *bp, uint16_t val, const char *name);
PhidgetReturnCode setBridgePacketUInt16(BridgePacket *bp, uint16_t val, int off);
PhidgetReturnCode setBridgePacketUInt16ByName(BridgePacket *bp, uint16_t val, const char *name);
PhidgetReturnCode addBridgePacketUInt16Array(BridgePacket *bp, uint16_t *val, uint32_t cnt,
  const char *name);
int32_t getBridgePacketInt32(BridgePacket *bp, int off);
int32_t getBridgePacketInt32ByName(BridgePacket *bp, const char *name);
const int32_t *getBridgePacketInt32Array(BridgePacket *bp, int off);
const int32_t *getBridgePacketInt32ArrayByName(BridgePacket *bp, const char *name);
PhidgetReturnCode addBridgePacketInt32(BridgePacket *bp, int32_t val, const char *name);
PhidgetReturnCode setBridgePacketInt32(BridgePacket *bp, int32_t val, int off);
PhidgetReturnCode setBridgePacketInt32ByName(BridgePacket *bp, int32_t val, const char *name);
PhidgetReturnCode addBridgePacketInt32Array(BridgePacket *bp, int32_t *val, uint32_t cnt,
  const char *name);
uint32_t getBridgePacketUInt32(BridgePacket *bp, int off);
uint32_t getBridgePacketUInt32ByName(BridgePacket *bp, const char *name);
const uint32_t *getBridgePacketUInt32Array(BridgePacket *bp, int off);
const uint32_t *getBridgePacketUInt32ArrayByName(BridgePacket *bp, const char *name);
PhidgetReturnCode addBridgePacketUInt32(BridgePacket *bp, uint32_t val, const char *name);
PhidgetReturnCode setBridgePacketUInt32(BridgePacket *bp, uint32_t val, int off);
PhidgetReturnCode setBridgePacketUInt32ByName(BridgePacket *bp, uint32_t val, const char *name);
PhidgetReturnCode addBridgePacketUInt32Array(BridgePacket *bp, uint32_t *val, uint32_t cnt,
  const char *name);
int64_t getBridgePacketInt64(BridgePacket *bp, int off);
int64_t getBridgePacketInt64ByName(BridgePacket *bp, const char *name);
const int64_t *getBridgePacketInt64Array(BridgePacket *bp, int off);
const int64_t *getBridgePacketInt64ArrayByName(BridgePacket *bp, const char *name);
PhidgetReturnCode addBridgePacketInt64(BridgePacket *bp, int64_t val, const char *name);
PhidgetReturnCode setBridgePacketInt64(BridgePacket *bp, int64_t val, int off);
PhidgetReturnCode setBridgePacketInt64ByName(BridgePacket *bp, int64_t val, const char *name);
PhidgetReturnCode addBridgePacketInt64Array(BridgePacket *bp, int64_t *val, uint32_t cnt,
  const char *name);
uint64_t getBridgePacketUInt64(BridgePacket *bp, int off);
uint64_t getBridgePacketUInt64ByName(BridgePacket *bp, const char *name);
const uint64_t *getBridgePacketUInt64Array(BridgePacket *bp, int off);
const uint64_t *getBridgePacketUInt64ArrayByName(BridgePacket *bp, const char *name);
PhidgetReturnCode addBridgePacketUInt64(BridgePacket *bp, uint64_t val, const char *name);
PhidgetReturnCode setBridgePacketUInt64(BridgePacket *bp, uint64_t val, int off);
PhidgetReturnCode setBridgePacketUInt64ByName(BridgePacket *bp, uint64_t val, const char *name);
PhidgetReturnCode addBridgePacketUInt64Array(BridgePacket *bp, uint64_t *val, uint32_t cnt,
  const char *name);
double getBridgePacketDouble(BridgePacket *bp, int off);
double getBridgePacketDoubleByName(BridgePacket *bp, const char *name);
const double *getBridgePacketDoubleArray(BridgePacket *bp, int off);
const double *getBridgePacketDoubleArrayByName(BridgePacket *bp, const char *name);
PhidgetReturnCode addBridgePacketDouble(BridgePacket *bp, double val, const char *name);
PhidgetReturnCode setBridgePacketDouble(BridgePacket *bp, double val, int off);
PhidgetReturnCode setBridgePacketDoubleByName(BridgePacket *bp, double val, const char *name);
PhidgetReturnCode addBridgePacketDoubleArray(BridgePacket *bp, double *val, uint32_t cnt,
  const char *name);
