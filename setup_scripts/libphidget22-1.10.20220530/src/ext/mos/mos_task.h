#ifndef _MOS_TASK_H_
#define _MOS_TASK_H_

#include "mos_os.h"

#ifdef _MOS_WINUSER_THREADS
#define MOS_TASK_RESULT	DWORD
#define MOS_TASK_EXIT(val)	do { mos_task_exiting(); mos_task_exit(val); /* not reached */ return (0); } while(0);
#else
#define MOS_TASK_RESULT	void
#define MOS_TASK_EXIT(val)	do { mos_task_exiting(); mos_task_exit(val); } while(0);
#endif

/****************************************************************************
 * PTHREADS
 ****************************************************************************/
#ifdef _MOS_PTHREADS
#include <pthread.h>

#if defined(Linux)
#define MOS_TASK_NONE	0UL
#else
#define MOS_TASK_NONE	NULL
#endif

typedef	pthread_t		mos_task_t;

#endif	/* _MOS_PTHREADS */

/****************************************************************************
 * FREEBSD KERNEL
 ****************************************************************************/
#ifdef _MOS_FREEBSDKERNTHREADS

typedef struct thread *	mos_task_t;
#define MOS_TASK_NONE	NULL

#endif	/* _MOS_FREEBSDKERNTHREADS */

/****************************************************************************
 * DARWIN KERNEL
 ****************************************************************************/
#ifdef _MOS_DARWINKERNTHREADS

typedef thread_t 		mos_task_t;
#define MOS_TASK_NONE	NULL

#endif	/* _MOS_FREEBSDKERNTHREADS */
/****************************************************************************
 * SOLARIS KERNEL
 ****************************************************************************/
#ifdef _MOS_SOLARISKERNTHREADS

typedef kthread_t *		mos_task_t;
#define MOS_TASK_NONE	NULL

#endif /* _MOS_SOLARISKERNTHREADS */

/****************************************************************************
 * WINDOWS KERNEL
 ****************************************************************************/
#ifdef _MOS_WINKERNTHREADS

typedef PKTHREAD		mos_task_t;
#define MOS_TASK_NONE	NULL

#endif /* _MOS_WINKERNTHREADS */

/****************************************************************************
 * WINDOWS USERLAND
 ****************************************************************************/
#ifdef _MOS_WINUSER_THREADS

typedef DWORD mos_task_t;
#define MOS_TASK_NONE ((mos_task_t)-1)

#endif /* _MOS_WINUSER_THREADS */

/****************************************************************************
 * GLOBAL PROTOTYPES
 ****************************************************************************/

MOSAPI int MOSCConv mos_task_create(mos_task_t *, MOS_TASK_RESULT (*)(void *), void *);
MOSAPI int MOSCConv mos_task_create2(mos_task_t *, uint32_t, MOS_TASK_RESULT (*)(void *), void *);
MOSAPI int MOSCConv mos_task_exit(int);
MOSAPI int MOSCConv mos_task_equal(mos_task_t, mos_task_t);
MOSAPI mos_task_t MOSCConv mos_self(void);
MOSAPI void mos_task_setname(const char *fmt, ...);

#endif /* _MOS_TASK_H_ */
