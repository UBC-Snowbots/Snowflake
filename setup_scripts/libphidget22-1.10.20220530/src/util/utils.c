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

#include "phidgetbase.h"
#include "util/phidgetlog.h"
#include "util/utils.h"

int
istrue(const char *b) {
	int32_t i;

	if (b == NULL)
		return (0);

	if (mos_strcasecmp(b, "true") == 0)
		return (1);
	if (mos_strcasecmp(b, "false") == 0)
		return (0);
	if (b[0] == 't' || b[0] == 'T')
		return (1);
	if (b[0] == 'n' || b[0] == 'N')
		return (0);

	if (mos_strto32(b, 0, &i) != 0)
		return (0);

	return (i != 0);
}

unsigned long
upper_power_of_two(unsigned long v) {
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

double
round_double(double x, int decimals) {

	return ((double)((double)roundl(x * (double)(pow(10, decimals))) / (double)(pow(10, decimals))));
}
