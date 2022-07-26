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

#ifndef PHIDGET22_H
#define PHIDGET22_H

#ifdef _WINDOWS
#ifdef PHIDGET22_EXPORTS
#ifndef PHIDGET22_API
#define PHIDGET22_API
#endif
#else
#ifndef PHIDGET22_API
#define PHIDGET22_API __declspec(dllimport)
#endif
#endif
#ifndef CCONV
#define CCONV __stdcall
#endif
#ifndef PRINTF_LIKE
#define PRINTF_LIKE(a, b)
#endif

#else
#ifndef PHIDGET22_API
#define PHIDGET22_API
#endif
#ifndef CCONV
#define CCONV
#endif
#ifndef PRINTF_LIKE
#ifdef CHK_FORMAT_STRINGS
#define PRINTF_LIKE(a, b) __attribute__((format(printf, a, b)))
#else
#define PRINTF_LIKE(a, b)
#endif
#endif

#endif

#ifndef API_PRETURN_HDR
#define API_PRETURN_HDR PHIDGET22_API PhidgetReturnCode CCONV
#endif
#ifndef API_VRETURN_HDR
#define API_VRETURN_HDR PHIDGET22_API void CCONV
#endif
#ifndef API_CRETURN_HDR
#define API_CRETURN_HDR PHIDGET22_API const char * CCONV
#endif
#ifndef API_IRETURN_HDR
#define API_IRETURN_HDR PHIDGET22_API int CCONV
#endif

/* Hide private and unreleased APIs from release builds */
#if !defined(EXTERNALPROTO) || defined(DEBUG) || defined(INTERNAL)
#define INCLUDE_PRIVATE
#else
#undef INCLUDE_PRIVATE
#endif
#if defined(DEBUG) || defined(INTERNAL)
#define INCLUDE_UNRELEASED
#else
#undef INCLUDE_UNRELEASED
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "macros.h"
#include "phidget.h"
#include "manager.h"
#include "util/phidgetlog.h"
#include "network/network.h"
#include "enumutil.gen.h"
#include "phidget22int.gen.h"

#ifdef __cplusplus
}
#endif

#endif
