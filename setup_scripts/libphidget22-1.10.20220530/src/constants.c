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
#include "constantsinternal.h"
#include "constants.h"

#define LIBRARY_VERSION "1.10"

#ifdef DEBUG
const char *LibraryVersion = "Phidget22 Debug - Version " LIBRARY_VERSION " - Built " __DATE__ " " __TIME__;
#else
const char *LibraryVersion = "Phidget22 - Version " LIBRARY_VERSION " - Built " __DATE__ " " __TIME__;
#endif

#ifdef P22_LIB_VERSION
const char *LibraryVersionNumber = "" P22_LIB_VERSION;
#else
const char *LibraryVersionNumber = "" LIBRARY_VERSION;
#endif

#if UINTPTR_MAX == 0xffffffff
#define P22_WIDTH "32-bit"
#elif UINTPTR_MAX == 0xffffffffffffffff
#define P22_WIDTH "64-bit"
#else
#define P22_WIDTH "?""?-bit"
#endif

#if defined(_WINDOWS)

#if defined(_WINDOWSUAP)
#define P22_OS "Windows (UWP)"
#else
#define P22_OS "Windows"
#endif

#elif defined(_MACOSX)

#if defined(_IPHONE)
#define P22_OS "iOS"
#else
#define P22_OS "macOS"
#endif

#elif defined(_ANDROID)
#define P22_OS "Android"
#elif defined(_FREEBSD)
#define P22_OS "FreeBSD"
#elif defined(_LINUX)
#define P22_OS "Linux"
#else
#define P22_OS "Unknown"
#endif

const char *LibrarySystem = "" P22_OS " (" P22_WIDTH ")";
