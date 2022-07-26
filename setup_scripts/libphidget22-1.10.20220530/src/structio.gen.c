
PhidgetReturnCode
readUnitInfo(BridgePacket *bp, Phidget_UnitInfoHandle dst) {

	dst->unit = getBridgePacketInt32ByName(bp, "UnitInfo.unit");
	dst->name = getBridgePacketStringByName(bp, "UnitInfo.name");
	dst->symbol = getBridgePacketStringByName(bp, "UnitInfo.symbol");

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeUnitInfo(const Phidget_UnitInfoHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketInt32(bp, src->unit, "UnitInfo.unit");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketString(bp, src->name, "UnitInfo.name");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketString(bp, src->symbol, "UnitInfo.symbol");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readPhidgetServer(BridgePacket *bp, PhidgetServerHandle dst) {

	dst->name = getBridgePacketStringByName(bp, "PhidgetServer.name");
	dst->stype = getBridgePacketStringByName(bp, "PhidgetServer.stype");
	dst->type = getBridgePacketInt32ByName(bp, "PhidgetServer.type");
	dst->flags = getBridgePacketInt32ByName(bp, "PhidgetServer.flags");
	dst->addr = getBridgePacketStringByName(bp, "PhidgetServer.addr");
	dst->host = getBridgePacketStringByName(bp, "PhidgetServer.host");
	dst->port = getBridgePacketInt32ByName(bp, "PhidgetServer.port");

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writePhidgetServer(const PhidgetServerHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketString(bp, src->name, "PhidgetServer.name");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketString(bp, src->stype, "PhidgetServer.stype");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt32(bp, src->type, "PhidgetServer.type");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt32(bp, src->flags, "PhidgetServer.flags");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketString(bp, src->addr, "PhidgetServer.addr");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketString(bp, src->host, "PhidgetServer.host");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt32(bp, src->port, "PhidgetServer.port");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readGPSTime(BridgePacket *bp, PhidgetGPS_TimeHandle dst) {

	dst->tm_ms = getBridgePacketInt16ByName(bp, "GPSTime.tm_ms");
	dst->tm_sec = getBridgePacketInt16ByName(bp, "GPSTime.tm_sec");
	dst->tm_min = getBridgePacketInt16ByName(bp, "GPSTime.tm_min");
	dst->tm_hour = getBridgePacketInt16ByName(bp, "GPSTime.tm_hour");

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeGPSTime(const PhidgetGPS_TimeHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketInt16(bp, src->tm_ms, "GPSTime.tm_ms");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt16(bp, src->tm_sec, "GPSTime.tm_sec");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt16(bp, src->tm_min, "GPSTime.tm_min");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt16(bp, src->tm_hour, "GPSTime.tm_hour");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readGPSDate(BridgePacket *bp, PhidgetGPS_DateHandle dst) {

	dst->tm_mday = getBridgePacketInt16ByName(bp, "GPSDate.tm_mday");
	dst->tm_mon = getBridgePacketInt16ByName(bp, "GPSDate.tm_mon");
	dst->tm_year = getBridgePacketInt16ByName(bp, "GPSDate.tm_year");

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeGPSDate(const PhidgetGPS_DateHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketInt16(bp, src->tm_mday, "GPSDate.tm_mday");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt16(bp, src->tm_mon, "GPSDate.tm_mon");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt16(bp, src->tm_year, "GPSDate.tm_year");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readGPGGA(BridgePacket *bp, PhidgetGPS_GPGGAHandle dst) {

	dst->latitude = getBridgePacketDoubleByName(bp, "GPGGA.latitude");
	dst->longitude = getBridgePacketDoubleByName(bp, "GPGGA.longitude");
	dst->fixQuality = getBridgePacketInt16ByName(bp, "GPGGA.fixQuality");
	dst->numSatellites = getBridgePacketInt16ByName(bp, "GPGGA.numSatellites");
	dst->horizontalDilution = getBridgePacketDoubleByName(bp, "GPGGA.horizontalDilution");
	dst->altitude = getBridgePacketDoubleByName(bp, "GPGGA.altitude");
	dst->heightOfGeoid = getBridgePacketDoubleByName(bp, "GPGGA.heightOfGeoid");

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeGPGGA(const PhidgetGPS_GPGGAHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketDouble(bp, src->latitude, "GPGGA.latitude");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->longitude, "GPGGA.longitude");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt16(bp, src->fixQuality, "GPGGA.fixQuality");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt16(bp, src->numSatellites, "GPGGA.numSatellites");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->horizontalDilution, "GPGGA.horizontalDilution");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->altitude, "GPGGA.altitude");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->heightOfGeoid, "GPGGA.heightOfGeoid");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readGPGSA(BridgePacket *bp, PhidgetGPS_GPGSAHandle dst) {

	dst->mode = getBridgePacketUInt8ByName(bp, "GPGSA.mode");
	dst->fixType = getBridgePacketInt16ByName(bp, "GPGSA.fixType");
	memcpy(&dst->satUsed, getBridgePacketInt16ArrayByName(bp, "GPGSA.satUsed"), sizeof (int16_t) * 12);
	dst->posnDilution = getBridgePacketDoubleByName(bp, "GPGSA.posnDilution");
	dst->horizDilution = getBridgePacketDoubleByName(bp, "GPGSA.horizDilution");
	dst->vertDilution = getBridgePacketDoubleByName(bp, "GPGSA.vertDilution");

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeGPGSA(const PhidgetGPS_GPGSAHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketUInt8(bp, src->mode, "GPGSA.mode");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt16(bp, src->fixType, "GPGSA.fixType");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt16Array(bp, src->satUsed, 12, "GPGSA.satUsed");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->posnDilution, "GPGSA.posnDilution");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->horizDilution, "GPGSA.horizDilution");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->vertDilution, "GPGSA.vertDilution");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readGPRMC(BridgePacket *bp, PhidgetGPS_GPRMCHandle dst) {

	dst->status = getBridgePacketUInt8ByName(bp, "GPRMC.status");
	dst->latitude = getBridgePacketDoubleByName(bp, "GPRMC.latitude");
	dst->longitude = getBridgePacketDoubleByName(bp, "GPRMC.longitude");
	dst->speedKnots = getBridgePacketDoubleByName(bp, "GPRMC.speedKnots");
	dst->heading = getBridgePacketDoubleByName(bp, "GPRMC.heading");
	dst->magneticVariation = getBridgePacketDoubleByName(bp, "GPRMC.magneticVariation");
	dst->mode = getBridgePacketUInt8ByName(bp, "GPRMC.mode");

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeGPRMC(const PhidgetGPS_GPRMCHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketUInt8(bp, src->status, "GPRMC.status");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->latitude, "GPRMC.latitude");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->longitude, "GPRMC.longitude");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->speedKnots, "GPRMC.speedKnots");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->heading, "GPRMC.heading");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->magneticVariation, "GPRMC.magneticVariation");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt8(bp, src->mode, "GPRMC.mode");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readGPVTG(BridgePacket *bp, PhidgetGPS_GPVTGHandle dst) {

	dst->trueHeading = getBridgePacketDoubleByName(bp, "GPVTG.trueHeading");
	dst->magneticHeading = getBridgePacketDoubleByName(bp, "GPVTG.magneticHeading");
	dst->speedKnots = getBridgePacketDoubleByName(bp, "GPVTG.speedKnots");
	dst->speed = getBridgePacketDoubleByName(bp, "GPVTG.speed");
	dst->mode = getBridgePacketUInt8ByName(bp, "GPVTG.mode");

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeGPVTG(const PhidgetGPS_GPVTGHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketDouble(bp, src->trueHeading, "GPVTG.trueHeading");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->magneticHeading, "GPVTG.magneticHeading");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->speedKnots, "GPVTG.speedKnots");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->speed, "GPVTG.speed");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt8(bp, src->mode, "GPVTG.mode");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readNMEAData(BridgePacket *bp, PhidgetGPS_NMEADataHandle dst) {
	PhidgetReturnCode res;

	res = readGPGGA(bp, &dst->GGA);
	if (res != EPHIDGET_OK)
		return (res);

	res = readGPGSA(bp, &dst->GSA);
	if (res != EPHIDGET_OK)
		return (res);

	res = readGPRMC(bp, &dst->RMC);
	if (res != EPHIDGET_OK)
		return (res);

	res = readGPVTG(bp, &dst->VTG);
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeNMEAData(const PhidgetGPS_NMEADataHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = writeGPGGA(&src->GGA, bp);
	if (res != EPHIDGET_OK)
		return (res);

	res = writeGPGSA(&src->GSA, bp);
	if (res != EPHIDGET_OK)
		return (res);

	res = writeGPRMC(&src->RMC, bp);
	if (res != EPHIDGET_OK)
		return (res);

	res = writeGPVTG(&src->VTG, bp);
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readSpatialQuaternion(BridgePacket *bp, PhidgetSpatial_SpatialQuaternionHandle dst) {

	dst->x = getBridgePacketDoubleByName(bp, "SpatialQuaternion.x");
	dst->y = getBridgePacketDoubleByName(bp, "SpatialQuaternion.y");
	dst->z = getBridgePacketDoubleByName(bp, "SpatialQuaternion.z");
	dst->w = getBridgePacketDoubleByName(bp, "SpatialQuaternion.w");

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeSpatialQuaternion(const PhidgetSpatial_SpatialQuaternionHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketDouble(bp, src->x, "SpatialQuaternion.x");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->y, "SpatialQuaternion.y");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->z, "SpatialQuaternion.z");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->w, "SpatialQuaternion.w");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readSpatialEulerAngles(BridgePacket *bp, PhidgetSpatial_SpatialEulerAnglesHandle dst) {

	dst->pitch = getBridgePacketDoubleByName(bp, "SpatialEulerAngles.pitch");
	dst->roll = getBridgePacketDoubleByName(bp, "SpatialEulerAngles.roll");
	dst->heading = getBridgePacketDoubleByName(bp, "SpatialEulerAngles.heading");

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeSpatialEulerAngles(const PhidgetSpatial_SpatialEulerAnglesHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketDouble(bp, src->pitch, "SpatialEulerAngles.pitch");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->roll, "SpatialEulerAngles.roll");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->heading, "SpatialEulerAngles.heading");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}

PhidgetReturnCode
readCodeInfo(BridgePacket *bp, PhidgetIR_CodeInfoHandle dst) {

	dst->bitCount = getBridgePacketUInt32ByName(bp, "CodeInfo.bitCount");
	dst->encoding = getBridgePacketInt32ByName(bp, "CodeInfo.encoding");
	dst->length = getBridgePacketInt32ByName(bp, "CodeInfo.length");
	dst->gap = getBridgePacketUInt32ByName(bp, "CodeInfo.gap");
	dst->trail = getBridgePacketUInt32ByName(bp, "CodeInfo.trail");
	memcpy(&dst->header, getBridgePacketUInt32ArrayByName(bp, "CodeInfo.header"),
	  sizeof (uint32_t) * 2);
	memcpy(&dst->one, getBridgePacketUInt32ArrayByName(bp, "CodeInfo.one"), sizeof (uint32_t) * 2);
	memcpy(&dst->zero, getBridgePacketUInt32ArrayByName(bp, "CodeInfo.zero"), sizeof (uint32_t) * 2);
	memcpy(&dst->repeat, getBridgePacketUInt32ArrayByName(bp, "CodeInfo.repeat"),
	  sizeof (uint32_t) * 26);
	dst->minRepeat = getBridgePacketUInt32ByName(bp, "CodeInfo.minRepeat");
	dst->dutyCycle = getBridgePacketDoubleByName(bp, "CodeInfo.dutyCycle");
	dst->carrierFrequency = getBridgePacketUInt32ByName(bp, "CodeInfo.carrierFrequency");
	memcpy(&dst->toggleMask, getBridgePacketUInt8ArrayByName(bp, "CodeInfo.toggleMask"),
	  sizeof (char) * 33);

	return (EPHIDGET_OK);
}

PhidgetReturnCode
writeCodeInfo(const PhidgetIR_CodeInfoHandle src, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = addBridgePacketUInt32(bp, src->bitCount, "CodeInfo.bitCount");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt32(bp, src->encoding, "CodeInfo.encoding");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketInt32(bp, src->length, "CodeInfo.length");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt32(bp, src->gap, "CodeInfo.gap");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt32(bp, src->trail, "CodeInfo.trail");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt32Array(bp, src->header, 2, "CodeInfo.header");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt32Array(bp, src->one, 2, "CodeInfo.one");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt32Array(bp, src->zero, 2, "CodeInfo.zero");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt32Array(bp, src->repeat, 26, "CodeInfo.repeat");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt32(bp, src->minRepeat, "CodeInfo.minRepeat");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketDouble(bp, src->dutyCycle, "CodeInfo.dutyCycle");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt32(bp, src->carrierFrequency, "CodeInfo.carrierFrequency");
	if (res != EPHIDGET_OK)
		return (res);

	res = addBridgePacketUInt8Array(bp, (uint8_t *)src->toggleMask, 33, "CodeInfo.toggleMask");
	if (res != EPHIDGET_OK)
		return (res);


	return (EPHIDGET_OK);
}
