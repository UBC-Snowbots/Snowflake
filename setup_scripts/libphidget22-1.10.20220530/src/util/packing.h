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

#ifndef __CPACKING
#define __CPACKING

/*
 * This file is used by firmware, please be careful with includes etc.
 */

#ifdef _WINDOWS
#include "phidgetbase.h"
#else
#include <stddef.h>
#endif

#include <stdint.h>

void pack(size_t, uint8_t *, uint64_t);

void pack16(uint8_t *, uint16_t);
uint16_t unpack16(const uint8_t *);

void pack32(uint8_t *, uint32_t);
uint32_t unpack32(const uint8_t *);

void pack64(uint8_t *, uint64_t);
uint64_t unpack64(const uint8_t *);

void packfloat(uint8_t *, float);
float unpackfloat(const uint8_t *);

void packdouble(uint8_t *, double);
double unpackdouble(const uint8_t *);

void pack16to16xS(uint8_t *, uint16_t, unsigned int, int);
void packfltto16xS(uint8_t *, float, unsigned int);

double unpacku16xS(const uint8_t *, unsigned int);
double unpack16xS(const uint8_t *, unsigned int);

double unpack32xS(const uint8_t *buf, unsigned int shift);
double unpacku32xS(const uint8_t *buf, unsigned int shift);

void doubleToUnsignedFixedPoint(double, unsigned char *, int, int, int);

#endif /* __CPACKING */
