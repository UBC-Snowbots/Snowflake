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
#ifndef _PHIDEBUG_H_
#define _PHIDEBUG_H_

#include "phidget.h"

PhidgetReturnCode getPhidgetDebugKeys(const char *start, char *keys, size_t keysz);
PhidgetReturnCode getPhidgetDebugKey(const char *, char *, size_t);
PhidgetReturnCode setPhidgetDebugKey(const char *, const char *);

#endif /* _PHIDEBUG_H_ */
