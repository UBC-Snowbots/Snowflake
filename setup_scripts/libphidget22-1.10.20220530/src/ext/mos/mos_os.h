#ifndef _MOS_OS_H_
#define _MOS_OS_H_

/*
 * Forward declarations for anybody who uses moh/mohpac.
 *
 * These functions are declared in lib/mohpac/mohpac.c, but are
 * prototyped here so that we do not have to include another
 * header everywhere.
 */
void moh_init(void);
void moh_fini(void);

typedef void (*mos_cleanup_t)(void);
int mos_register_cleanup(mos_cleanup_t);
void mos_cleanup(void);

#include "mos_basic.h"

#if defined(Windows)
#if defined(_KERNEL)
#include "mos_os-Windows-kern.h"
#else /* !_KERNEL */
#include "mos_os-Windows-user.h"
#endif /* _KERNEL */
#endif /* Windows */

#if defined(FreeBSD)
#if defined(_KERNEL)
#include "mos_os-FreeBSD-kern.h"
#else /* !_KERNEL */
#include "mos_os-FreeBSD-user.h"
#endif /* _KERNEL */
#endif /* FreeBSD */

#if defined(Darwin)
#if defined(_KERNEL)
#include "mos_os-Darwin-kern.h"
#else /* !_KERNEL */
#include "mos_os-Darwin-user.h"
#endif /* _KERNEL */
#endif /* Darwin */

#if defined(SunOS)
#if defined(_KERNEL)
#include "mos_os-SunOS-kern.h"
#else /* !_KERNEL */
#include "mos_os-SunOS-user.h"
#endif /* _KERNEL */
#endif /* SunOS */

#if defined(Linux)
#if defined(_KERNEL)
#error "Linux Kernel Not Supported"
#else /* !_KERNEL */
#include "mos_os-Linux-user.h"
#endif /* _KERNEL */
#endif /* Linux */

MOSAPI void MOSCConv mos_init(void);
MOSAPI void MOSCConv mos_fini(void);
extern void _mos_malloc_os_init(void);
extern void _mos_malloc_os_fini(void);

MOSAPI const char * MOSCConv mos_sysname(void);
MOSAPI const char * MOSCConv mos_machine(void);
MOSAPI const char * MOSCConv mos_sysrev(void);
MOSAPI const char * MOSCConv mos_sysrel(void);
MOSAPI const char * MOSCConv mos_osfamily(void);
MOSAPI const char * MOSCConv mos_getfeatures(void);
MOSAPI const char * MOSCConv mos_getfeatures_C_define(void);

MOSAPI void MOSCConv mos_task_exiting(void);
typedef void(MOSCConv *mos_task_thread_exit_callback)	(void *ctx);
MOSAPI void MOSCConv mos_task_set_thread_exit_handler(mos_task_thread_exit_callback fptr, void *ctx);

#endif /* _MOS_OS_H_ */
