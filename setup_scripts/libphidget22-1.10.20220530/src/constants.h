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

#ifndef _PHIDGET_CONSTANTS_H_
#define _PHIDGET_CONSTANTS_H_

#define PUNK_BOOL					0x02			/* Unknown Boolean */
#define PUNK_INT8					INT8_MAX		/* Unknown Short   (8-bit) */
#define PUNK_UINT8					UINT8_MAX		/* Unknown Short   (8-bit unsigned) */
#define PUNK_INT16					INT16_MAX		/* Unknown Short   (16-bit) */
#define PUNK_UINT16					UINT16_MAX		/* Unknown Short   (16-bit unsigned) */
#define PUNK_INT32					INT32_MAX		/* Unknown Integer (32-bit) */
#define PUNK_UINT32					UINT32_MAX		/* Unknown Integer (32-bit unsigned) */
#define PUNK_INT64					INT64_MAX		/* Unknown Integer (64-bit) */
#define PUNK_UINT64					UINT64_MAX		/* Unknown Integer (64-bit unsigned) */
#define PUNK_DBL					1e300			/* Unknown Double */
#define PUNK_FLT					1e30			/* Unknown Float */
#define PUNK_ENUM					INT32_MAX		/* Unknown Enum */
#define PUNK_SIZE					SIZE_MAX		/* Unknown size_t */

#define PFALSE		0x00	/* False. Used for boolean values. */
#define PTRUE		0x01	/* True. Used for boolean values. */

#define PRIphid "P"			/* mos_printf format string for printing a PhidgetHandle */

#endif /* _PHIDGET_CONSTANTS_H_ */
