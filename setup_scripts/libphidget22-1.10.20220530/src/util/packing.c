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

/*
 * This file is used by firmware, please be careful with includes etc.
 */

#ifndef MOS_BYTE_ORDER
// This should be the firmware case
#define	MOS_LITTLE_ENDIAN	1234
#define	MOS_BIG_ENDIAN		4321
#define MOS_BYTE_ORDER MOS_LITTLE_ENDIAN
#else
// phidget22 library case
#include "mos/mos_os.h"
#include "mos/mos_byteorder.h"
#if !defined(MOS_BYTE_ORDER) || (MOS_BYTE_ORDER != MOS_LITTLE_ENDIAN && MOS_BYTE_ORDER != MOS_BIG_ENDIAN)
#error Define MOS_BYTE_ORDER to be equal to either MOS_LITTLE_ENDIAN or MOS_BIG_ENDIAN
#endif
#endif

#include <string.h>
#include <assert.h>
#include "packing.h"
#include "constants.h"

void
pack(size_t sz, uint8_t *buf, uint64_t val) {

	switch (sz) {
	case 1:
		buf[0] = (uint8_t)val;
		break;
	case 2:
		pack16(buf, (uint16_t)val);
		break;
	case 4:
		pack32(buf, (uint32_t)val);
		break;
	case 8:
		pack64(buf, (uint64_t)val);
		break;
	}
}

void
pack16(uint8_t *buf, uint16_t val) {

	buf[0] = (val >> 8) & 0xff;
	buf[1] = val & 0xff;
}

uint16_t
unpack16(const uint8_t *buf) {

	return ((uint16_t)buf[0] << 8 | (uint16_t)buf[1]);
}

void
pack32(uint8_t *buf, uint32_t val) {

	buf[0] = (val >> 24) & 0xff;
	buf[1] = (val >> 16) & 0xff;
	buf[2] = (val >> 8) & 0xff;
	buf[3] = val & 0xff;
}

uint32_t
unpack32(const uint8_t *buf) {

	return ((uint32_t)buf[0] << 24 |
			(uint32_t)buf[1] << 16 |
			(uint32_t)buf[2] << 8 |
			(uint32_t)buf[3]);
}

void
pack64(uint8_t *buf, uint64_t val) {

	buf[0] = (val >> 56) & 0xff;
	buf[1] = (val >> 48) & 0xff;
	buf[2] = (val >> 40) & 0xff;
	buf[3] = (val >> 32) & 0xff;
	buf[4] = (val >> 24) & 0xff;
	buf[5] = (val >> 16) & 0xff;
	buf[6] = (val >> 8) & 0xff;
	buf[7] = val & 0xff;
}

uint64_t
unpack64(const uint8_t *buf) {

	return ((uint64_t)buf[0] << 56 |
			(uint64_t)buf[1] << 48 |
			(uint64_t)buf[2] << 40 |
			(uint64_t)buf[3] << 32 |
			(uint64_t)buf[4] << 24 |
			(uint64_t)buf[5] << 16 |
			(uint64_t)buf[6] << 8 |
			(uint64_t)buf[7]);
}

#if MOS_BYTE_ORDER == MOS_LITTLE_ENDIAN

void
packfloat(uint8_t *buf, float val) {

	memcpy(buf, &val, sizeof(float));
}

float
unpackfloat(const uint8_t *buf) {
	float ret;

	memcpy(&ret, buf, sizeof(float));
	return (ret);
}

void
packdouble(uint8_t *buf, double val) {

	memcpy(buf, &val, sizeof(double));
}

double
unpackdouble(const uint8_t *buf) {
	double ret;

	memcpy(&ret, buf, sizeof(double));
	return (ret);
}

#else

void
packfloat(uint8_t *buf, float val) {

	char *valBuf = (char*)& val;

	// byte swap so buf is little-endian
	buf[0] = valBuf[3];
	buf[1] = valBuf[2];
	buf[2] = valBuf[1];
	buf[3] = valBuf[0];
}

float
unpackfloat(const uint8_t *buf) {
	float ret;
	char *retBuf = (char*)& ret;

	// byte swap so retBuf is big-endian
	retBuf[0] = buf[3];
	retBuf[1] = buf[2];
	retBuf[2] = buf[1];
	retBuf[3] = buf[0];

	return (ret);
}

void
packdouble(uint8_t *buf, double val) {

	char *valBuf = (char*)& val;

	// byte swap so buf is little-endian
	buf[0] = valBuf[7];
	buf[1] = valBuf[6];
	buf[2] = valBuf[5];
	buf[3] = valBuf[4];
	buf[4] = valBuf[3];
	buf[5] = valBuf[2];
	buf[6] = valBuf[1];
	buf[7] = valBuf[0];
}

double
unpackdouble(const uint8_t *buf) {
	double ret;
	char *retBuf = (char*)& ret;

	// byte swap so retBuf is big-endian
	retBuf[0] = buf[7];
	retBuf[1] = buf[6];
	retBuf[2] = buf[5];
	retBuf[3] = buf[4];
	retBuf[4] = buf[3];
	retBuf[5] = buf[2];
	retBuf[6] = buf[1];
	retBuf[7] = buf[0];

	return (ret);
}

#endif

double
unpack16xS(const uint8_t *buf, unsigned int shift) {
	int16_t tmp;

	tmp = (((int16_t)buf[0] << 8) | buf[1]);
	return (((double)tmp) / (1 << shift));
}

double
unpacku16xS(const uint8_t *buf, unsigned int shift) {
	uint16_t tmp;

	tmp = (((uint16_t)buf[0] << 8) | buf[1]);
	return (((double)tmp) / (1 << shift));
}

void
pack16to16xS(uint8_t *buf, uint16_t val, unsigned int shift, int scale) {

	packfltto16xS(buf, ((float)val) / scale, shift);
}

void
packfltto16xS(uint8_t *buf, float val, unsigned int shift) {
	int16_t sval;

	assert(shift < 15);

	sval = (int16_t)(val * (1 << shift));
	buf[0] = sval >> 8;
	buf[1] = sval & 0xff;
}

double
unpack32xS(const uint8_t *buf, unsigned int shift) {
	int32_t tmp;
	tmp = (((int32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint16_t)buf[2] << 8) | buf[3]);
	return (((double)tmp) / (1 << shift));
}

double
unpacku32xS(const uint8_t *buf, unsigned int shift) {
	uint32_t tmp;
	tmp = (((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint16_t)buf[2] << 8) | buf[3]);
	return (((double)tmp) / (1 << shift));
}

void
doubleToUnsignedFixedPoint(double value, unsigned char *buffer, int offset, int numBytes,
  int numFractionalBits) {
	uint64_t fixed;
	int shift;

	if (value == PUNK_DBL) {
		//Set to fixed point UNK - maximum value
		memset(buffer + offset, 0xFF, numBytes);
		return;
	}

	fixed = ((uint64_t)((value)* ((uint64_t)1 << numFractionalBits)));
	for (shift = 0; numBytes; numBytes--, shift += 8)
		buffer[numBytes - 1 + offset] = (unsigned char)(fixed >> shift);
}
