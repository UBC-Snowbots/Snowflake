#ifndef _MOS_PTHREADS
#define _MOS_PTHREADS
#endif

#include <sys/time.h>

#include <assert.h>
#include <stdio.h>
#include <time.h>
#include <fcntl.h>

#include "mos_assert.h"
#include "mos_lock.h"
#include "mos_iop.h"
#include "mos_error-errno.h"
#include "mos_time.h"

pthread_mutexattr_t *mos_default_mutexattrp = NULL;

#if defined(SunOS) && defined(DEBUG)
void
_mos_lock_init(void) {
	static pthread_mutexattr_t attr;

	/* enable deadlock checking on Solaris */

	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);

	mos_default_mutexattrp = &attr;
}
#else
void
_mos_lock_init(void) {
}
#endif

void
_mos_lock_fini(void) {
}

MOSAPI int MOSCConv
mos_namedlock_init(mos_namedlock_t **_lk, const char *name, int flags) {
	mos_namedlock_t *lk;
	char sname[128];

	if (_lk == NULL)
		return (MOSN_FAULT);

	/*
	 * Some implementations require / as the first character.
	 */
	if (name[0] == '/')
		mos_strlcpy(sname, name, sizeof (sname));
	else
		mos_snprintf(sname, sizeof (sname), "/%s", name);

	lk = mos_malloc(sizeof (mos_namedlock_t));
	lk->sem = sem_open(sname, O_CREAT, 0666, 1);
	if (lk->sem == SEM_FAILED)
		return (mos_fromerrno(errno));

	lk->name = mos_strdup(sname, NULL);
	lk->locked = 0;

	*_lk = lk;

	return (0);
}

MOSAPI int MOSCConv
mos_namedlock_fini(mos_namedlock_t **_lk) {
	mos_namedlock_t *lk;

	if (_lk == NULL)
		return (MOSN_FAULT);
	lk = *_lk;
	if (lk == NULL)
		return (0);

	MOS_ASSERT(lk->locked == 0);
	mos_free(lk->name, MOSM_FSTR);
	sem_close(lk->sem);
	mos_free(lk, sizeof (*lk));

	*_lk = NULL;
	return (0);
}

/*
 * MOSN_TIMEDOUT is returned if a timeout is specified and exceeded.
 */
MOSAPI int MOSCConv
mos_namedlock_lock(mos_namedlock_t *lk, uint64_t nsec) {
	int err;
#if defined(Darwin)
	mostime_t dl;
#else	
	struct timespec ts;
	uint64_t psec;
	uint64_t fsec;
#endif

	MOS_ASSERT(lk->locked == 0);

	if (nsec == 0) {
		err = sem_wait(lk->sem);
		if (err != 0)
			return (mos_fromerrno(errno));
		lk->locked = 1;
		return (0);
	}

#if defined(Darwin)

	err = sem_trywait(lk->sem);
	if (err == 0)
		return (0);

	if (errno != EINTR && errno != EAGAIN)
		return (mos_fromerrno(errno));

	dl = mos_gettime_usec() + nsec / 1000;	/* deadline */
	while (mos_gettime_usec() < dl) {
		err = sem_trywait(lk->sem);
		if (err == 0) {
			lk->locked = 1;
			return (0);
		}
		if (errno != EINTR && errno != EAGAIN)
			return (mos_fromerrno(errno));
		mos_usleep(1000 * 25);	/* 25 ms */
	}
	return (MOSN_TIMEDOUT);

#else /* !Darwin */

	err = clock_gettime(CLOCK_REALTIME, &ts);
	if (err < 0)
		return (MOSN_ERR);

	fsec = nsec / 1000000000;
	psec = nsec % 1000000000;

	ts.tv_sec += fsec;
	ts.tv_nsec += psec;
	if (ts.tv_nsec >= 1000000000) {
		ts.tv_nsec -= 1000000000;
		ts.tv_sec++;
	}

again:

	err = sem_timedwait(lk->sem, &ts);
	if (err == 0) {
		lk->locked = 1;
		return (0);
	}
	if (errno == EINTR)
		goto again;

	return (mos_fromerrno(errno));
#endif
}

MOSAPI int MOSCConv
mos_namedlock_unlock(mos_namedlock_t *lk) {

	MOS_ASSERT(lk->locked == 1);

	if (lk == NULL)
		return (MOSN_FAULT);

	sem_post(lk->sem);
	lk->locked = 0;

	return (0);
}

MOSAPI int MOSCConv
mos_cond_init(mos_cond_t *cond) {

	if (pthread_cond_init((pthread_cond_t *)cond, NULL) != 0)
		return (MOSN_ERR);
	return (0);
}

MOSAPI void MOSCConv
mos_cond_destroy(mos_cond_t *cond) {
#ifndef NDEBUG
	int res;
	res = pthread_cond_destroy((pthread_cond_t *)cond);
	assert(res == 0);
#else
	pthread_cond_destroy((pthread_cond_t *)cond);
#endif
}

MOSAPI void MOSCConv
mos_cond_wait(mos_cond_t *cond, mos_mutex_t *mutex) {

	(void)pthread_cond_wait((pthread_cond_t *)cond, (pthread_mutex_t *)mutex);
}

MOSAPI int MOSCConv
mos_cond_timedwait(mos_cond_t *cond, mos_mutex_t *mutex, uint64_t nsec) {
	struct timespec ts;
	uint64_t psec;
	int err;

#if defined(Darwin)
	struct timeval tv;

	gettimeofday(&tv, NULL);
	
	psec = tv.tv_usec * 1000ULL + nsec;
	ts.tv_sec = tv.tv_sec + psec / 1000000000ULL;
	ts.tv_nsec = psec % 1000000000ULL;

#else /* !Darwin */

	uint64_t fsec;

	err = clock_gettime(CLOCK_REALTIME, &ts);
	if (err < 0)
		return (MOSN_ERR);

	fsec = nsec / 1000000000;
	psec = nsec % 1000000000;

	ts.tv_sec += fsec;
	ts.tv_nsec += psec;
	if (ts.tv_nsec >= 1000000000) {
		ts.tv_nsec -= 1000000000;
		ts.tv_sec++;
	}
#endif

	err = pthread_cond_timedwait((pthread_cond_t *)cond,
	  (pthread_mutex_t *)mutex, &ts);
	switch(err) {
	case ETIMEDOUT:
		return (MOSN_TIMEDOUT);
	case 0:
		return (0);
	default:
		MOS_PANIC("pthread_cond_timedwait failed");
	}
}

MOSAPI void MOSCConv
mos_cond_signal(mos_cond_t *cond) {
#ifndef NDEBUG
	int err;

	err = pthread_cond_signal((pthread_cond_t *)cond);
	MOS_ASSERT(err == 0);
#else
	pthread_cond_signal((pthread_cond_t *)cond);
#endif
}

MOSAPI void MOSCConv
mos_cond_broadcast(mos_cond_t *cond) {
#ifndef NDEBUG
	int err;

	err = pthread_cond_broadcast((pthread_cond_t *)cond);
	MOS_ASSERT(err == 0);
#else
	pthread_cond_broadcast((pthread_cond_t *)cond);
#endif
}

MOSAPI int MOSCConv
mos_mutex_init(mos_mutex_t *mutex) {
	int err;

	err = pthread_mutex_init((pthread_mutex_t *)mutex, mos_default_mutexattrp);
	if (err != 0)
		return (MOSN_ERR);
	return (0);
}

MOSAPI void MOSCConv
mos_mutex_destroy(mos_mutex_t *mutex) {
	int res;

	res = pthread_mutex_destroy((pthread_mutex_t *)mutex);
	if (res != 0)
		fprintf(stderr, "mos_mutex_destroy() failed %d\n", res);
	assert(res == 0);
}

MOSAPI void MOSCConv
mos_mutex_lock(mos_mutex_t *mutex) {
	int res;

	res = pthread_mutex_lock((pthread_mutex_t *)mutex);
	if (res != 0)
		fprintf(stderr, "lock %p failed with %d (%s)\n", (void *)mutex, res,
		  strerror(res));
	assert(res == 0);
}

MOSAPI int MOSCConv
mos_mutex_trylock(mos_mutex_t *mutex) {
	int err;

	err = pthread_mutex_trylock((pthread_mutex_t *)mutex);
	switch(err) {
	case EBUSY:
		return (MOSN_BUSY);
	case 0:
		return (0);
	default:
		MOS_PANIC("pthread_mutex_trylock() failed");
	}
}

MOSAPI void MOSCConv
mos_mutex_unlock(mos_mutex_t *mutex) {
	int res;

	res = pthread_mutex_unlock((pthread_mutex_t *)mutex);
	if (res != 0)
		fprintf(stderr, "unlock %p failed with %d\n", (void *)mutex, res);
	assert(res == 0);
}

MOSAPI int MOSCConv
mos_rwlock_init(mos_rwlock_t *mutex) {

	if (pthread_rwlock_init((pthread_rwlock_t *)mutex, NULL) != 0)
		return (MOSN_ERR);
	return (0);
}

MOSAPI void MOSCConv
mos_rwlock_destroy(mos_rwlock_t *mutex) {
#ifndef NDEBUG
	int err;

	err = pthread_rwlock_destroy((pthread_rwlock_t *)mutex);
	assert(err == 0);
#else
	pthread_rwlock_destroy((pthread_rwlock_t *)mutex);
#endif
}

MOSAPI void MOSCConv
mos_rwlock_rdlock(mos_rwlock_t *mutex) {
#ifndef NDEBUG
	int err;

	err = pthread_rwlock_rdlock((pthread_rwlock_t *)mutex);
	assert(err == 0);
#else
	pthread_rwlock_rdlock((pthread_rwlock_t *)mutex);
#endif
}

MOSAPI void MOSCConv
mos_rwlock_wrlock(mos_rwlock_t *mutex) {
#ifndef NDEBUG
	int err;

	err = pthread_rwlock_wrlock((pthread_rwlock_t *)mutex);
	assert(err == 0);
#else
	pthread_rwlock_wrlock((pthread_rwlock_t *)mutex);
#endif
}


MOSAPI void MOSCConv
mos_rwlock_unlock(mos_rwlock_t *mutex) {
#ifndef NDEBUG
	int err;

	err = pthread_rwlock_unlock((pthread_rwlock_t *)mutex);
	assert(err == 0);
#else
	pthread_rwlock_unlock((pthread_rwlock_t *)mutex);
#endif
}

MOSAPI int MOSCConv
mos_rwlock_tryrdlock(mos_rwlock_t *mutex) {
	int res;

	res = pthread_rwlock_tryrdlock((pthread_rwlock_t *)mutex);
	switch(res) {
	case 0:
		return (0);
	case EBUSY:
		return (MOSN_BUSY);
	default:
		MOS_PANIC("pthread_rwlock_tryrdlock() failed");
	}
}

MOSAPI int MOSCConv
mos_rwlock_trywrlock(mos_rwlock_t *mutex) {
	int res;

	res = pthread_rwlock_trywrlock((pthread_rwlock_t *)mutex);
	switch(res) {
	case 0:
		return (0);
	case EBUSY:
		return (MOSN_BUSY);
	default:
		MOS_PANIC("pthread_rwlock_trywrlock() failed");
	}
}

MOSAPI void MOSCConv
mos_yield() {

	sched_yield();
}

MOSAPI int MOSCConv
mos_mutex_lockedbyme(mos_mutex_t *ml)
{
	return 1; //TODO: not implemented on Linux.
}
