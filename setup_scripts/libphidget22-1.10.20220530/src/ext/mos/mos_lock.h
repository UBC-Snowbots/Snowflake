#ifndef _MOS_LOCK_H_
#define _MOS_LOCK_H_

#include "mos_os.h"

void _mos_lock_init(void);
void _mos_lock_fini(void);

/****************************************************************************
 * PTHREADS
 ****************************************************************************/
#ifdef _MOS_PTHREADS
#define MOS_THREAD_IMPL "ptheads"

#define __USE_UNIX98 1

#include <sys/types.h>
#include <pthread.h>
#include <semaphore.h>

typedef	pthread_rwlock_t	mos_rwlock_t;
typedef pthread_mutex_t		mos_mutex_t;
typedef pthread_cond_t		mos_cond_t;

#define MOS_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER

typedef struct mos_namedlock {
	sem_t *sem;
	char *name;
	int locked;
} mos_namedlock_t;

#endif	/* _MOS_PTHREADS */

/****************************************************************************
 * FREEBSD KERNEL
 ****************************************************************************/
#ifdef _MOS_FREEBSDKERNTHREADS
#define MOS_THREAD_IMPL "freebsdkernthreads"

#include <sys/param.h>
#include <sys/condvar.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rwlock.h>

typedef struct rwlock mos_rwlock_t;
typedef struct mtx mos_mutex_t;
typedef struct cv mos_cond_t;

#endif	/* _MOS_FREEBSDKERNTHREADS */

/****************************************************************************
 * SOLARIS KERNEL
 ****************************************************************************/
#ifdef _MOS_SOLARISKERNTHREADS
#define MOS_THREAD_IMPL "solariskernthreads"

#include <sys/ksynch.h>

typedef krwlock_t	mos_rwlock_t;
typedef kmutex_t 	mos_mutex_t;
typedef kcondvar_t	mos_cond_t;

#endif /* _MOS_SOLARISKERNTHREADS */

/****************************************************************************
 * DARWIN KERNEL
 ****************************************************************************/
#ifdef _MOS_DARWINKERNTHREADS
#define MOS_THREAD_IMPL "darwinkernthreads"

#include "kern/locks.h"

#define MOS_USE_GENERIC_RWLOCK
typedef lck_mtx_t * mos_mutex_t;
typedef void * mos_cond_t;

#endif	/* _MOS_DARWINKERNTHREADS */

/****************************************************************************
 * WINDOWS KERNEL
 ****************************************************************************/
#ifdef _MOS_WINKERNTHREADS
#define MOS_THREAD_IMPL "winkernthreads"

#define MOS_USE_GENERIC_RWLOCK
typedef KMUTEX *mos_mutex_t;
typedef struct mos_cond mos_cond_t;

/*
 * A condition-variable implementation, based on that described in
 * "Strategies for Implementing POSIX Condition Variables on Win32" by
 * Douglas C. Schmidt and Irfan Pyarali.  This implementation has the
 * unfortunate requirement that the external mutex be HELD DURING
 * SIGNALLING/BROADCASTING, though the interface doesn't imply this
 * restriction.
 */
struct mos_cond {

	/* number of waiters */
	int					mc_nwaiters;

	/* waiters lock */
	mos_mutex_t			mc_nwaiters_lock;

	/* number of threads to release via a signal/broadcast */
	int					mc_release_count;

	/* wait generation number */
	int					mc_generation;

	/* an event to block/release waiting threads */
	KEVENT				*mc_event;
};

#endif /* _MOS_WINKERNTHREADS */

/****************************************************************************
 * WINDOWS USERLAND
 ****************************************************************************/
#ifdef _MOS_WINUSER_THREADS
#define MOS_THREAD_IMPL "Windows usermode threads"

#define MOS_USE_GENERIC_RWLOCK
typedef CRITICAL_SECTION mos_mutex_t;
typedef struct mos_cond mos_cond_t;

/*
 * A condition-variable implementation, based on that described in
 * "Strategies for Implementing POSIX Condition Variables on Win32" by
 * Douglas C. Schmidt and Irfan Pyarali.  This implementation has the
 * unfortunate requirement that the external mutex be HELD DURING
 * SIGNALLING/BROADCASTING, though the interface doesn't imply this
 * restriction.
 */
typedef struct mos_cond {

	/* number of waiters */
	int					mc_nwaiters;

	/* waiters lock */
	mos_mutex_t			mc_nwaiters_lock;

	/* number of threads to release via a signal/broadcast */
	int					mc_release_count;

	/* wait generation number */
	int					mc_generation;

	/* a manual-release event to block/release waiting threads */
	HANDLE				mc_event;
} mos_cond_t;

typedef struct mos_namedlock {
	HANDLE handle;
	char *name;
	int locked;
} mos_namedlock_t;

#endif /* _MOS_WINUSER_THREADS */

/****************************************************************************
 * ERROR CASE
 ****************************************************************************/
#ifndef MOS_THREAD_IMPL
#error "No threading implementation defined"
#endif

#ifdef MOS_USE_GENERIC_RWLOCK

#include "mos_task.h"

typedef struct {
	uint32_t		rw_nreader;
	uint32_t		rw_nreaderwaiting;
	uint32_t		rw_nwriter;
	mos_task_t		rw_writer;
	mos_mutex_t		rw_lock;
	mos_cond_t		rw_rcond;
	mos_cond_t		rw_wcond;
} mos_rwlock_t;

#endif /* MOS_USE_GENERIC_RWLOCK */

/****************************************************************************
 * GLOBAL PROTOTYPES
 ****************************************************************************/
MOSAPI int MOSCConv mos_cond_init(mos_cond_t *);
MOSAPI void MOSCConv mos_cond_destroy(mos_cond_t *);

MOSAPI void MOSCConv mos_cond_wait(mos_cond_t *, mos_mutex_t *);
#define MOS_USEC	1000LL
#define MOS_MSEC	1000000LL
#define MOS_SEC		1000000000LL
MOSAPI int MOSCConv mos_cond_timedwait(mos_cond_t *, mos_mutex_t *, uint64_t);
MOSAPI void MOSCConv mos_cond_signal(mos_cond_t *);
MOSAPI void MOSCConv mos_cond_broadcast(mos_cond_t *);

/* readers/writer lock (does not necessarily allow recursion as reader) */
MOSAPI int MOSCConv mos_rwlock_init(mos_rwlock_t *);
MOSAPI void MOSCConv mos_rwlock_destroy(mos_rwlock_t *);
MOSAPI void MOSCConv mos_rwlock_rdlock(mos_rwlock_t *);
MOSAPI void MOSCConv mos_rwlock_wrlock(mos_rwlock_t *);
MOSAPI int MOSCConv mos_rwlock_trywrlock(mos_rwlock_t *);
MOSAPI int MOSCConv mos_rwlock_tryrdlock(mos_rwlock_t *);
MOSAPI void MOSCConv mos_rwlock_unlock(mos_rwlock_t *);

MOSAPI int MOSCConv mos_mutex_init(mos_mutex_t *);
MOSAPI void MOSCConv mos_mutex_destroy(mos_mutex_t *);

MOSAPI void MOSCConv mos_mutex_lock(mos_mutex_t *);
MOSAPI int MOSCConv mos_mutex_trylock(mos_mutex_t *);
MOSAPI void MOSCConv mos_mutex_unlock(mos_mutex_t *);
MOSAPI int MOSCConv mos_mutex_lockedbyme(mos_mutex_t *);

MOSAPI int MOSCConv mos_namedlock_init(mos_namedlock_t **, const char *, int);
MOSAPI int MOSCConv mos_namedlock_fini(mos_namedlock_t **);
MOSAPI int MOSCConv mos_namedlock_lock(mos_namedlock_t *, uint64_t ns);
MOSAPI int MOSCConv mos_namedlock_unlock(mos_namedlock_t *);

MOSAPI void MOSCConv mos_yield(void);

#endif /* _MOS_LOCK_H_ */
