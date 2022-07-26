#ifndef EXTERNALPROTO
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
#endif

#ifndef _PHIDGETLOG_H_
#define _PHIDGETLOG_H_

#ifndef EXTERNALPROTO
#include "constants.h"
#include "types.gen.h"
#endif

API_PRETURN_HDR PhidgetLog_enable(Phidget_LogLevel level, const char *destination);
API_PRETURN_HDR PhidgetLog_disable(void);
API_PRETURN_HDR PhidgetLog_enableNetwork(const char *address, int port);
API_PRETURN_HDR PhidgetLog_disableNetwork(void);

API_PRETURN_HDR PhidgetLog_log(Phidget_LogLevel level, const char *message, ...) PRINTF_LIKE(2, 3);
API_PRETURN_HDR PhidgetLog_loge(const char *file, int line, const char *func,
  const char *src, Phidget_LogLevel level, const char *message, ...) PRINTF_LIKE(6, 7);
API_PRETURN_HDR PhidgetLog_logs(Phidget_LogLevel level, const char *message);
API_PRETURN_HDR PhidgetLog_loges(Phidget_LogLevel level, const char *source, const char *message);

API_PRETURN_HDR PhidgetLog_rotate(void);
API_PRETURN_HDR PhidgetLog_enableRotating(void);
API_PRETURN_HDR PhidgetLog_disableRotating(void);
API_PRETURN_HDR PhidgetLog_isRotating(int *isrotating);
API_PRETURN_HDR PhidgetLog_getRotating(uint64_t *size, int *keepCount);
API_PRETURN_HDR PhidgetLog_setRotating(uint64_t size, int keepCount);
API_PRETURN_HDR PhidgetLog_getLevel(Phidget_LogLevel *level);
API_PRETURN_HDR PhidgetLog_setLevel(Phidget_LogLevel level);
API_PRETURN_HDR PhidgetLog_addSource(const char *, Phidget_LogLevel level);
API_PRETURN_HDR PhidgetLog_getSourceLevel(const char *source, Phidget_LogLevel *level);
API_PRETURN_HDR PhidgetLog_setSourceLevel(const char *source, Phidget_LogLevel level);
API_PRETURN_HDR PhidgetLog_getSources(const char *sources[], uint32_t *count);

#ifndef EXTERNALPROTO

#define LOGF_STDERR		0x08000
#define LOGF_DEBUGGER	0x10000

API_PRETURN_HDR PhidgetLog_logv(const char *file, int line, const char *func,
  const char *src, Phidget_LogLevel level, const char *fmt, va_list va);

void PhidgetLogInit(void);
void PhidgetLogFini(void);

void logStackTrace(Phidget_LogLevel, const char *);

/* logs to visual studio output... or info  */
#define logvs(...) \
  PhidgetLog_loge(__FILE__, __LINE__, __func__, NULL, PHIDGET_LOG_INFO | LOGF_DEBUGGER, __VA_ARGS__)

#ifdef NDEBUG
#define logdebug(...)
#define logcrit(...) PhidgetLog_loge(NULL, 0, __func__, NULL, PHIDGET_LOG_CRITICAL, __VA_ARGS__)
#define logerr(...) PhidgetLog_loge(NULL, 0, __func__, NULL, PHIDGET_LOG_ERROR, __VA_ARGS__)
#define logwarn(...) PhidgetLog_loge(NULL, 0, __func__, NULL, PHIDGET_LOG_WARNING, __VA_ARGS__)
#define loginfo(...) PhidgetLog_loge(NULL, 0, __func__, NULL, PHIDGET_LOG_INFO, __VA_ARGS__)
#define logverbose(...)
#else
#define logcrit(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NULL, PHIDGET_LOG_CRITICAL, __VA_ARGS__)
#define logerr(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NULL, PHIDGET_LOG_ERROR, __VA_ARGS__)
#define logwarn(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NULL, PHIDGET_LOG_WARNING, __VA_ARGS__)
#define loginfo(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NULL, PHIDGET_LOG_INFO, __VA_ARGS__)
#define logdebug(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NULL, PHIDGET_LOG_DEBUG, __VA_ARGS__)
#define logverbose(...) PhidgetLog_loge(__FILE__, __LINE__, __func__, NULL, PHIDGET_LOG_VERBOSE, __VA_ARGS__)
#endif /* NDEBUG */

#endif /* EXTERNALPROTO */

#endif /* _PHIDGETLOG_H_ */
