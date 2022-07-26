
#ifndef _MOS_TLOCK_H_
#define _MOS_TLOCK_H_

#define MOSLOCK_TRACK			0x00000001	/* enable tracking */
#define MOSLOCK_RECURSE_CLASS	0x00000002	/* allow recurse on same class */
#define MOSLOCK_LOCKED			0x00000100	/* lock is held */
#define MOSLOCK_RLOCKED			0x00000200	/* lock is held for reading */
#define MOSLOCK_WAITING			0x00000400	/* lock is waiting on cond */
#define MOSLOCK_RWLOCK			0x10000000	/* lock is a rwlock */

#include "mos_lock.h"

typedef struct mos_tlock mos_tlock_t;

#if defined(MOS_DEBUG_LOCKING)

#define mos_tlock_init(a, b, c) a = _mos_tlock_create((b), (c), __FILE__, __func__, __LINE__)
#define mos_tlock_create(a, b) _mos_tlock_create((a), (b), __FILE__, __func__, __LINE__)

#define mos_tlock_destroy(l)	_mos_tlock_destroy((l), __FILE__, __func__)

#define mos_tlock_lock(l)		_mos_tlock_lock((l), __FILE__, __LINE__, __func__)
#define mos_tlock_wlock(l)		_mos_tlock_wlock((l), __FILE__, __LINE__, __func__)
#define mos_tlock_rlock(l)		_mos_tlock_rlock((l), __FILE__, __LINE__, __func__)
#define mos_tlock_unlock(l)		_mos_tlock_unlock((l), __FILE__, __LINE__, __func__)
#define mos_tlock_rwunlock(l)	_mos_tlock_rwunlock((l), __FILE__, __LINE__, __func__)

#define mos_tlock_trylock(l) _mos_tlock_lock((l), __FILE__, __LINE__, __func__)

#else

#define mos_tlock_init(a, b, c) a = _mos_tlock_create((b), (c), NULL, NULL, 0)
#define mos_tlock_create(a, b) _mos_tlock_create((a), (b), NULL, NULL, 0)

#define mos_tlock_destroy(l)	_mos_tlock_destroy((l), NULL, NULL)

#define mos_tlock_lock(l)		_mos_tlock_lock((l), NULL, 0, NULL)
#define mos_tlock_wlock(l)		_mos_tlock_wlock((l), NULL, 0, NULL)
#define mos_tlock_rlock(l)		_mos_tlock_rlock((l), NULL, 0, NULL)
#define mos_tlock_unlock(l)		_mos_tlock_unlock((l), NULL, 0, NULL)
#define mos_tlock_rwunlock(l)	_mos_tlock_rwunlock((l), NULL, 0, NULL)

#define mos_tlock_trylock(l) _mos_tlock_lock((l), NULL, 0, NULL)

#endif

MOSAPI mos_tlock_t * MOSCConv _mos_tlock_create(int, int, const char *, const char *, int);
MOSAPI void MOSCConv _mos_tlock_destroy(mos_tlock_t **, const char *, const char *);

MOSAPI int MOSCConv _mos_tlock_lock(mos_tlock_t *, const char *, int line, const char *);
MOSAPI int MOSCConv _mos_tlock_wlock(mos_tlock_t *, const char *, int line, const char *);
MOSAPI int MOSCConv _mos_tlock_rlock(mos_tlock_t *, const char *, int line, const char *);
MOSAPI int MOSCConv _mos_tlock_unlock(mos_tlock_t *, const char *, int, const char *);
MOSAPI int MOSCConv _mos_tlock_rwunlock(mos_tlock_t *, const char *, int, const char *);
MOSAPI int MOSCConv _mos_tlock_trylock(mos_tlock_t *, const char *, int, const char *);
MOSAPI int MOSCConv mos_tlock_islocked(mos_tlock_t *);
MOSAPI int MOSCConv mos_tlock_isrdlocked(mos_tlock_t *);

MOSAPI int MOSCConv mos_tlock_hasreader(mos_tlock_t *, mos_task_t);
MOSAPI int MOSCConv mos_tlock_haswriter(mos_tlock_t *, mos_task_t);

#if defined(MOS_DEBUG_LOCKING)
MOSAPI const char * MOSCConv mos_tlock_getfunction(mos_tlock_t *);
MOSAPI const char * MOSCConv mos_tlock_getfile(mos_tlock_t *);
MOSAPI int MOSCConv mos_tlock_getline(mos_tlock_t *);

MOSAPI int MOSCConv mos_tlock_getid(mos_tlock_t *);
MOSAPI int MOSCConv mos_tlock_setid(mos_tlock_t *, int);
#endif

MOSAPI int MOSCConv mos_tlock_isrwlock(mos_tlock_t *);

MOSAPI void MOSCConv mos_tlock_wait(mos_cond_t *, mos_tlock_t *);
MOSAPI int MOSCConv mos_tlock_timedwait(mos_cond_t *, mos_tlock_t *, uint64_t);

/* Fast tlock just calls into regular locks directly when not debugging, no support for rwlock */
#if defined(MOS_DEBUG_LOCKING)
typedef mos_tlock_t * mos_fasttlock_t;

#define mos_fasttlock_init(a, b, c)			mos_tlock_init((*a), (b), (c))
#define mos_fasttlock_destroy(l)			mos_tlock_destroy((l))
#define mos_fasttlock_lock(l)				mos_tlock_lock((*l))
#define mos_fasttlock_unlock(l)				mos_tlock_unlock((*l))
#define mos_fasttlock_wait(c, l)			mos_tlock_wait((c), (*l))
#define mos_fasttlock_timedwait(c, l, t)	mos_tlock_timedwait((c), (*l), (t))
#else
typedef mos_mutex_t mos_fasttlock_t;

#define mos_fasttlock_init(a, b, c)			mos_mutex_init((a))
#define mos_fasttlock_destroy(l)			mos_mutex_destroy((l))
#define mos_fasttlock_lock(l)				mos_mutex_lock((l))
#define mos_fasttlock_unlock(l)				mos_mutex_unlock((l))
#define mos_fasttlock_wait(c, l)			mos_cond_wait((c), (l))
#define mos_fasttlock_timedwait(c, l, t)	mos_cond_timedwait((c), (l), (t))
#endif

#endif /* _MOS_TLOCK_H_ */
