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

#ifndef _PHIDGET_BASE_H_
#define _PHIDGET_BASE_H_

// This needs to be BEFORE stdlib gets included
#if THREAD_LOCALE_LOCALE_H
#define _XOPEN_SOURCE 700   /* needed for glibc newlocale() support */
#define _GNU_SOURCE         /* ditto, for older versions of glibc */
#endif

#include "mos/mos_os.h"
#include "mos/mos_iop.h"
#include "mos/mos_task.h"
#include "mos/mos_lock.h"

#define USE_PHIDGET22_LOGGING

#define INCLUDE_PRIVATE
#if defined(DEBUG) || defined(INTERNAL)
#define INCLUDE_UNRELEASED
#endif

#ifdef _WINDOWS // Defines & Includes for Windows

#define PHIDAPIPUBLIC
#define PHIDAPIPRIVATE

#ifdef _WINDOWSUAP
// UAP cannot LoadLibrary
#ifndef ZEROCONF_SUPPORT
#define ZEROCONF_SUPPORT 0
#endif
#else
//use runtime linking for zeroconf
#ifndef ZEROCONF_RUNTIME_LINKING
#define ZEROCONF_RUNTIME_LINKING
#endif
#ifndef ZEROCONF_SUPPORT
#define ZEROCONF_SUPPORT 1
#endif
#endif

#include <locale.h>
#define LOCALE_SUPPORT 1

#define CCONV __stdcall
#define CCONV_CDECL __cdecl

#define PRINTF_LIKE(a, b)
#define SCANF_LIKE(a, b)

#ifndef USB_ASYNC_READS
#define USB_ASYNC_READS 1
#endif

#else /* WINDOWS */

/* NON-Windows PLATFORMS */

typedef void * HANDLE;

/* Always use internal iconv */
#define USE_INTERNAL_UNICONV
#include "cvtutf.h"

#define CCONV
#define CCONV_CDECL
#ifndef PHIDGET22_API
#define PHIDGET22_API
#endif

#ifdef CHK_FORMAT_STRINGS
#define PRINTF_LIKE(a, b) __attribute__((format(printf, a, b)))
#define SCANF_LIKE(a, b) __attribute__((format(scanf, a, b)))
#else
#define PRINTF_LIKE(a, b)
#define SCANF_LIKE(a, b)
#endif

#if THREAD_LOCALE_LOCALE_H
#include <locale.h>
#define LOCALE_SUPPORT 1
#elif THREAD_LOCALE_XLOCALE_H
#include <xlocale.h>
#define LOCALE_SUPPORT 1
#endif

#ifdef _MACOSX // Defines & Includes for Mac
#include <CoreFoundation/CoreFoundation.h>
#include <mach/mach.h>

#define PHIDAPIPUBLIC
#define PHIDAPIPRIVATE

#ifndef ZEROCONF_SUPPORT
#define ZEROCONF_SUPPORT 1
#endif

#ifndef _IPHONE
#include <IOKit/IOKitLib.h>
#endif

#include <xlocale.h>
#ifndef LOCALE_SUPPORT
#define LOCALE_SUPPORT 1
#endif

#elif _FREEBSD // Defines & Includes for FreeBSD

//use runtime linking for zeroconf
#ifndef ZEROCONF_RUNTIME_LINKING
#define ZEROCONF_RUNTIME_LINKING
#endif
#ifndef ZEROCONF_SUPPORT
#define ZEROCONF_SUPPORT 1
#endif

#define PHIDAPIPUBLIC
#define PHIDAPIPRIVATE

#elif _LINUX // Defines & Includes for Linux

//use runtime linking for zeroconf
#ifndef ZEROCONF_RUNTIME_LINKING
#define ZEROCONF_RUNTIME_LINKING
#endif
#ifndef ZEROCONF_SUPPORT
#define ZEROCONF_SUPPORT 1
#endif

// Use libusb on Linux in Async mode for reads
//  NOTE: Enabled by Makefile. Disable with ./configure --disable-libusbasync
#ifdef LIBUSB_ASYNC
#define USB_ASYNC_READS 1
#endif

#define PHIDAPIPUBLIC __attribute__ ((visibility("default")))
#define PHIDAPIPRIVATE __attribute__ ((visibility("hidden")))

#elif _ANDROID // Defines & Include for Android Only
#include <android/log.h>

#ifndef ZEROCONF_SUPPORT
#define ZEROCONF_SUPPORT 1
#endif

#define PHIDAPIPUBLIC
#define PHIDAPIPRIVATE

#endif

#endif /* WINDOWS */

// Default is disabled
#ifndef LOCALE_SUPPORT
#define LOCALE_SUPPORT 0
#endif

// Default is disabled
#ifndef USB_ASYNC_READS
#define USB_ASYNC_READS 0
#endif

// Default is disabled
#ifndef ZEROCONF_SUPPORT
#define ZEROCONF_SUPPORT 0
#endif

#ifndef round
#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))
#endif
#ifndef roundl
#define roundl(x) ((x)>=0?(int64_t)((x)+0.5):(int64_t)((x)-0.5))
#endif
#ifndef roundu
#define roundu(x) ((unsigned)((x)+0.5))
#endif

/* For exported API in the source files only */
#define API_PRETURN PHIDAPIPUBLIC PhidgetReturnCode CCONV
#define API_VPRETURN PHIDAPIPUBLIC void * CCONV
#define API_VRETURN PHIDAPIPUBLIC void CCONV
#define API_CRETURN PHIDAPIPUBLIC const char * CCONV
#define API_DRETURN PHIDAPIPUBLIC double CCONV
#define API_IRETURN PHIDAPIPUBLIC int CCONV
#define API_I32RETURN PHIDAPIPUBLIC int32_t CCONV
#define API_U32RETURN PHIDAPIPUBLIC uint32_t CCONV
#define API_I64RETURN PHIDAPIPUBLIC int64_t CCONV
#define API_U64RETURN PHIDAPIPUBLIC uint64_t CCONV

/* For exported API in the header files only */
#define API_PRETURN_HDR API_PRETURN
#define API_VPRETURN_HDR API_VPRETURN
#define API_VRETURN_HDR API_VRETURN
#define API_CRETURN_HDR API_CRETURN
#define API_DRETURN_HDR API_DRETURN
#define API_IRETURN_HDR API_IRETURN
#define API_I32RETURN_HDR API_I32RETURN
#define API_U32RETURN_HDR API_U32RETURN
#define API_I64RETURN_HDR API_I64RETURN
#define API_U64RETURN_HDR API_U64RETURN

#include "util/phidgetlog.h"
#include "phidget.h"

double phid_strtod_c(const char* str, char** endptr);
int phid_snprintf_c(char * s, size_t n, const char * format, ...);

extern void(CCONV *fptrJavaDetachCurrentThread)(void);

#endif /* _PHIDGET_BASE_H_ */
