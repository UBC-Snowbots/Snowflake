#include "mos_assert.h"
#include "mos_lock.h"
#include "mos_task.h"
#include "mos_rwrlock.h"
#include "mos_tlock.h"

typedef union {
	mos_mutex_t		mtx;
	mos_rwrlock_t	rw;
} tlock_lock_t;

struct mos_tlock {
	int 			ml_flags;		/* Lock flags */
	tlock_lock_t	ml_lock;		/* The Real Lock */
	mos_mutex_t 	ml_mlock;		/* Meta Data Lock */
	mos_task_t		ml_owner;

#if defined(MOS_DEBUG_LOCKING)
	int				ml_id;			/* Lock ID for reversal tracking etc. */
	int				ml_line;
	const char		*ml_file;
	const char		*ml_func;
#endif
};

#define ISRWLK(l)	((l)->ml_flags & MOSLOCK_RWLOCK)

#define TLOCKTRYLK(l)	(ISRWLK(l) ? 1 :					\
		mos_mutex_trylock(&((l)->ml_lock.mtx)))

#define MOSLOCK_THREADS_MAX		512
#define MOSLOCK_HELD_MAX		15

#define MOSLOCK_TOOMANYTHREADS	0
#define MOSLOCK_TOOMANYLOCKS	1
#define MOSLOCK_HELD			2
#define MOSLOCK_NOTHELD			3
#define MOSLOCK_UNLOCKORDER		4

#define MOSLOCK_LOCK			1
#define MOSLOCK_TRYLOCK			2
#define MOSLOCK_UNLOCK			3

const char MOSLOCK_ABORT_REASON[5][24] = {
	{"Too Many Threads"},
	{"Too Many Locks"},
	{"Already Held"},
	{"Not Held"},
	{"Not Unlocked In Order"},
};

static mos_mutex_t		slock;

#if defined(MOS_DEBUG_LOCKING)

typedef struct {
	mos_task_t	lt_tid;
	int32_t		lt_count;
	mos_tlock_t	*lt_lock[MOSLOCK_HELD_MAX];
} moslock_thread_t;

static moslock_thread_t	sthread[MOSLOCK_THREADS_MAX];

static void
print_lock_details(mos_tlock_t *this) {

	if (this == NULL) {
		mos_printef("unknown lock\n");
		return;
	}

	mos_printef("tlock(ptr=%p, id=%d) : %s+%d(%s)\n", (void *)this, this->ml_id,
	  this->ml_file, this->ml_line, this->ml_func);
}

static void
mos_tlock_abort(mos_tlock_t *this, int reason, const char *file, int line,
  const char *func, mos_tlock_t *other) {
	char tbuf[64];

	mos_printef("LOCK ABORT: [%s]\n", MOSLOCK_ABORT_REASON[reason]);
	print_lock_details(this);
	if (other) {
		mos_printef(" other lock:\n");
		print_lock_details(other);
	}

	mos_snprintf(tbuf, sizeof (tbuf), "Lock abort: %s",
	  MOSLOCK_ABORT_REASON[reason]);
	MOS_PANIC("%s", tbuf);
}

static void
mos_tlock_reversal(mos_tlock_t *this, mos_tlock_t *other, const char *file,
  int line, const char *func) {

	mos_printef("** LOCK ORDER REVERSAL\n");
	print_lock_details(other);
	mos_printef("  locked before\n");
	print_lock_details(this);

#if !defined(MOS_DEBUG_LOCKING_NO_PANIC)
	MOS_PANIC("Lock order Reversal");
#endif
}

static moslock_thread_t *
getthread(mos_tlock_t *this) {
	mos_task_t tid;
	int avail;
	int i;

	tid = mos_self();
	avail = -1;

	mos_mutex_lock(&slock);

	for (i = 0; i < MOSLOCK_THREADS_MAX; i++) {
		if (mos_task_equal(sthread[i].lt_tid, tid)) {
			mos_mutex_unlock(&slock);
			return (&(sthread[i]));
		}
		if (sthread[i].lt_tid == MOS_TASK_NONE && avail == -1)
			avail = i;
	}

	if (avail == -1) {
		mos_mutex_unlock(&slock);
		mos_tlock_abort(this, MOSLOCK_TOOMANYTHREADS, NULL, -1, NULL, NULL);
		return (NULL);
	}

	sthread[avail].lt_tid = tid;
	mos_mutex_unlock(&slock);

	return (&sthread[avail]);
}

static void
retain_thread(moslock_thread_t *thr) {

	mos_mutex_lock(&slock);
	thr->lt_count++;
	mos_mutex_unlock(&slock);
}

static void
release_thread(moslock_thread_t *thr) {

	mos_mutex_lock(&slock);
	thr->lt_count--;

	MOS_ASSERT(thr->lt_count >= 0);

	if (thr->lt_count == 0)
		thr->lt_tid = MOS_TASK_NONE;

	mos_mutex_unlock(&slock);
}

static void
check_lock_order(mos_tlock_t *this, moslock_thread_t *thr, const char *file,
  int line, const char *func) {
	int i;

	for (i = 0; thr->lt_lock[i] != NULL && i < MOSLOCK_HELD_MAX; i++) {
		if (thr->lt_lock[i]->ml_id >= this->ml_id) {
			if (this->ml_flags & MOSLOCK_RECURSE_CLASS) {
				if (thr->lt_lock[i]->ml_id == this->ml_id)
					continue;
			}
			mos_tlock_reversal(this, thr->lt_lock[i], file, line, func);
			return; /* If reached */
		}
	}
}

static int
mos_tlock_track(mos_tlock_t *this, int action, const char *file, int line,
  const char *func) {
	moslock_thread_t *thr;
	int i;

	thr = getthread(this);
	if (thr == NULL)
		return (1);

	switch(action) {
	case MOSLOCK_LOCK:		/* FALL THROUGH */
	case MOSLOCK_TRYLOCK:
		check_lock_order(this, thr, file, line, func);
		for (i = 0; i < MOSLOCK_HELD_MAX; i++) {
			if (thr->lt_lock[i] == NULL)
				break;
			if (thr->lt_lock[i] == this) {
				mos_tlock_abort(this, MOSLOCK_HELD, file, line, func,
				  thr->lt_lock[i]);
				return (1);
			}
		}
		if (i == MOSLOCK_HELD_MAX) {
			mos_tlock_abort(this, MOSLOCK_TOOMANYLOCKS, file, line, func, NULL);
			return (1);
		}

		thr->lt_lock[i] = this;
		retain_thread(thr);
		return (0);
	case MOSLOCK_UNLOCK:
		for (i = 0; i < MOSLOCK_HELD_MAX; i++) {
			if (thr->lt_lock[i] == NULL) {
				mos_tlock_abort(this, MOSLOCK_NOTHELD, file, line, func, NULL);
				return (1);
			}

			if (thr->lt_lock[i] == this) {
				if (i + 1 < MOSLOCK_HELD_MAX && thr->lt_lock[i + 1] != NULL) {
					mos_tlock_abort(this, MOSLOCK_UNLOCKORDER, file, line, func,
					  thr->lt_lock[i + 1]);
					return (1);
				}
				thr->lt_lock[i] = NULL;
				break;
			}
		}
		if (i == MOSLOCK_HELD_MAX) {
			mos_tlock_abort(this, MOSLOCK_NOTHELD, file, line, func, NULL);
			return (1);
		}
		release_thread(thr);
		return (0);
	}
	return (1);
}
#endif

MOSAPI mos_tlock_t * MOSCConv
_mos_tlock_create(int id, int flags, const char *file, const char *func,
  int line) {
	mos_tlock_t *this;

	this = _mos_alloc(sizeof (*this), MOSM_DEFAULT, file, func, line);

	this->ml_flags = flags;
	this->ml_owner = MOS_TASK_NONE;

#if defined(MOS_DEBUG_LOCKING)
	this->ml_id = id;
	this->ml_file = "";
	this->ml_line = -1;
	this->ml_func = "<unlocked>";
#endif

	mos_mutex_init(&this->ml_mlock);
	if (ISRWLK(this))
		mos_rwrlock_init(&this->ml_lock.rw);
	else
		mos_mutex_init(&this->ml_lock.mtx);

	return (this);
}

MOSAPI void MOSCConv
_mos_tlock_destroy(mos_tlock_t **this, const char *file, const char *func) {

	mos_mutex_destroy(&(*this)->ml_mlock);
	if (ISRWLK(*this))
		mos_rwrlock_destroy(&(*this)->ml_lock.rw);
	else
		mos_mutex_destroy(&(*this)->ml_lock.mtx);

	mos_free(*this, sizeof (mos_tlock_t));
	*this = NULL;
}

MOSAPI int MOSCConv
_mos_tlock_lock(mos_tlock_t *this, const char *file, int line, const char *func) {

	MOS_ASSERT(!ISRWLK(this));

	mos_mutex_lock(&this->ml_lock.mtx);
	mos_mutex_lock(&this->ml_mlock);

	MOS_ASSERT((this->ml_flags & MOSLOCK_LOCKED) == 0);
	this->ml_flags |= MOSLOCK_LOCKED;
	this->ml_owner = mos_self();

#if defined(MOS_DEBUG_LOCKING)
	/*
	 * Update the meta data early, so we can print it in case of a
	 * tracking error.
	 */
	if (file)
		this->ml_file = file;
	this->ml_line = line;
	this->ml_func = func;

	if (this->ml_flags & MOSLOCK_TRACK)
		mos_tlock_track(this, MOSLOCK_LOCK, file, line, func);
#endif

	mos_mutex_unlock(&this->ml_mlock);

	return (0);
}

MOSAPI int MOSCConv
_mos_tlock_wlock(mos_tlock_t *this, const char *file, int line, const char *func) {

	MOS_ASSERT(ISRWLK(this));

	mos_rwrlock_wrlock(&this->ml_lock.rw);
	mos_mutex_lock(&this->ml_mlock);

	MOS_ASSERT((this->ml_flags & MOSLOCK_LOCKED) == 0);
	this->ml_flags |= MOSLOCK_LOCKED;
	this->ml_owner = mos_self();

#if defined(MOS_DEBUG_LOCKING)
	/*
	* Update the meta data early, so we can print it in case of a
	* tracking error.
	*/
	if (file)
		this->ml_file = file;
	this->ml_line = line;
	this->ml_func = func;

	if (this->ml_flags & MOSLOCK_TRACK)
		mos_tlock_track(this, MOSLOCK_LOCK, file, line, func);
#endif

	mos_mutex_unlock(&this->ml_mlock);

	return (0);
}

MOSAPI int MOSCConv
_mos_tlock_rlock(mos_tlock_t *this, const char *file, int line, const char *func) {

	MOS_ASSERT(ISRWLK(this));

	mos_rwrlock_rdlock(&this->ml_lock.rw);
	mos_mutex_lock(&this->ml_mlock);

	/*
	 * For R/W locks, since they are recursible, we only track first
	 * lock/last unlock.
	 */
	if ((this->ml_flags & MOSLOCK_RLOCKED) &&
	  mos_rwrlock_getreaderholdcount(&this->ml_lock.rw, mos_self()) > 1) {
		MOS_ASSERT(this->ml_flags & MOSLOCK_LOCKED);
		mos_mutex_unlock(&this->ml_mlock);
		return (0);
	}

	this->ml_flags |= MOSLOCK_LOCKED;
	this->ml_flags |= MOSLOCK_RLOCKED;

	this->ml_owner = mos_self();

#if defined(MOS_DEBUG_LOCKING)
	/*
	 * Update the meta data early, so we can print it in case of a
	 * tracking error.
	 */
	if (file)
		this->ml_file = file;
	this->ml_line = line;
	this->ml_func = func;

	if (this->ml_flags & MOSLOCK_TRACK)
		mos_tlock_track(this, MOSLOCK_LOCK, file, line, func);
#endif

	mos_mutex_unlock(&this->ml_mlock);

	return (0);
}

MOSAPI int MOSCConv
_mos_tlock_unlock(mos_tlock_t *this, const char *file, int line, const char *func) {

	MOS_ASSERT(!ISRWLK(this));
	mos_mutex_lock(&this->ml_mlock);

	MOS_ASSERT(this->ml_flags & MOSLOCK_LOCKED);

#if defined(MOS_DEBUG_LOCKING)
	if (this->ml_flags & MOSLOCK_TRACK)
		mos_tlock_track(this, MOSLOCK_UNLOCK, file, line, func);

	this->ml_file = "";
	this->ml_line = -1;
	this->ml_func = "<unlocked>";
#endif

	this->ml_owner = MOS_TASK_NONE;
	this->ml_flags &= ~MOSLOCK_LOCKED;

	mos_mutex_unlock(&this->ml_lock.mtx);
	mos_mutex_unlock(&this->ml_mlock);

	return (0);
}

MOSAPI int MOSCConv
_mos_tlock_rwunlock(mos_tlock_t *this, const char *file, int line, const char *func) {

	MOS_ASSERT(ISRWLK(this));
	mos_mutex_lock(&this->ml_mlock);

	MOS_ASSERT(this->ml_flags & MOSLOCK_LOCKED);

	/*
	* For R/W locks, since they are recursible, we only track first
	* lock/last unlock.
	*/
	if ((this->ml_flags & MOSLOCK_RLOCKED) &&
		mos_rwrlock_getreaderholdcount(&this->ml_lock.rw, mos_self()) > 1) {
		mos_mutex_unlock(&this->ml_mlock);
		mos_rwrlock_unlock(&this->ml_lock.rw);
		return (0);
	}

	/*
	* The caller is unlocking their first reader for this thread, so
	* remove the rwlock from the list of held locks now.
	*/
#if defined(MOS_DEBUG_LOCKING)
	if (this->ml_flags & MOSLOCK_TRACK)
		mos_tlock_track(this, MOSLOCK_UNLOCK, file, line, func);
#endif

	if ((this->ml_flags & MOSLOCK_RLOCKED) && mos_rwrlock_getreadercount(&this->ml_lock.rw) > 1) {
		/*
		* Other readers are still using the lock, so don't mark as unlocked.
		*/
		goto skipstate;
	}

#if defined(MOS_DEBUG_LOCKING)
	this->ml_file = "";
	this->ml_line = -1;
	this->ml_func = "<unlocked>";
#endif

	this->ml_owner = MOS_TASK_NONE;
	this->ml_flags &= ~(MOSLOCK_LOCKED | MOSLOCK_RLOCKED);

skipstate:

	/*
	* In order to ensure that the reader count (if RW) is accurate
	* for the check above, we must unlock the rwrlock with the mlock
	* held.  Obviously this matches the order they were acquired in,
	* which isn't the best practice, but it should work.  If it
	* doesn't, adding a reader count to this (protected by mlock)
	* would also ensure the above check will give correct results, so
	* we'll mark the lock as unlocked iff it is the last reader to
	* exit.
	*/
	mos_rwrlock_unlock(&this->ml_lock.rw);
	mos_mutex_unlock(&this->ml_mlock);

	return (0);
}

MOSAPI int MOSCConv
_mos_tlock_trylock(mos_tlock_t *this, const char *file, int line, const char *func) {
	int res;

	/* try*lock() for rwrlocks is not supported */
	MOS_ASSERT(!ISRWLK(this));

	res = mos_mutex_trylock(&this->ml_lock.mtx);

	if (res != 0)
		return (res);

	mos_mutex_lock(&this->ml_mlock);

	MOS_ASSERT((this->ml_flags & MOSLOCK_LOCKED) == 0);
	this->ml_flags |= MOSLOCK_LOCKED;
	this->ml_owner = mos_self();

#if defined(MOS_DEBUG_LOCKING)
	/*
	 * Update the meta data early, so we can print it in case of an error.
	 */
	if (file)
		this->ml_file = file;
	this->ml_line = line;
	this->ml_func = func;

	if (this->ml_flags & MOSLOCK_TRACK)
		mos_tlock_track(this, MOSLOCK_TRYLOCK, file, line, func);
#endif

	mos_mutex_unlock(&this->ml_mlock);

	return (res);
}

MOSAPI void MOSCConv
mos_tlock_wait(mos_cond_t *cond, mos_tlock_t *this) {

	MOS_ASSERT(!ISRWLK(this));

	mos_mutex_lock(&this->ml_mlock);
	this->ml_flags |= MOSLOCK_WAITING;
	this->ml_flags &= ~MOSLOCK_LOCKED;
	mos_mutex_unlock(&this->ml_mlock);

	mos_cond_wait(cond, &this->ml_lock.mtx);

	mos_mutex_lock(&this->ml_mlock);
	this->ml_flags &= ~MOSLOCK_WAITING;
	this->ml_flags |= MOSLOCK_LOCKED;
	mos_mutex_unlock(&this->ml_mlock);
}

MOSAPI int MOSCConv
mos_tlock_timedwait(mos_cond_t *cond, mos_tlock_t *this, uint64_t nsec) {
	int res;

	MOS_ASSERT(!ISRWLK(this));

	mos_mutex_lock(&this->ml_mlock);
	this->ml_flags |= MOSLOCK_WAITING;
	this->ml_flags &= ~MOSLOCK_LOCKED;
	mos_mutex_unlock(&this->ml_mlock);

	res = mos_cond_timedwait(cond, &this->ml_lock.mtx, nsec);

	mos_mutex_lock(&this->ml_mlock);
	this->ml_flags &= ~MOSLOCK_WAITING;
	this->ml_flags |= MOSLOCK_LOCKED;
	mos_mutex_unlock(&this->ml_mlock);

	return (res);
}

MOSAPI int MOSCConv
mos_tlock_islocked(mos_tlock_t *this) {
	int locked;

	mos_mutex_lock(&this->ml_mlock);
	locked = this->ml_flags & MOSLOCK_LOCKED;
	mos_mutex_unlock(&this->ml_mlock);

	if (locked)
		return (1);
	return (0);
}

MOSAPI int MOSCConv
mos_tlock_isrdlocked(mos_tlock_t *this) {
	int locked;

	mos_mutex_lock(&this->ml_mlock);
	locked = this->ml_flags & MOSLOCK_RLOCKED;
	mos_mutex_unlock(&this->ml_mlock);

	return (locked);
}

MOSAPI int MOSCConv
mos_tlock_hasreader(mos_tlock_t *this, mos_task_t thread) {

	if (!ISRWLK(this))
		return (0); /* no readers */

	return (mos_rwrlock_getreaderholdcount(&this->ml_lock.rw, thread) > 0);
}

MOSAPI int MOSCConv
mos_tlock_haswriter(mos_tlock_t *this, mos_task_t thread) {
	int res;

	mos_mutex_lock(&this->ml_mlock);
	if (this->ml_flags & MOSLOCK_RLOCKED) {
		/* there are no writers */
		mos_mutex_unlock(&this->ml_mlock);
		return (0);
	}

	/* see if the thread is the current owner/writer */
	res = (this->ml_owner == thread);

	/* sanity check: owner shouldn't be set when unlocked */
	if (res != 0)
		MOS_ASSERT(this->ml_flags & MOSLOCK_LOCKED);

	mos_mutex_unlock(&this->ml_mlock);

	return (res);
}

#if defined(MOS_DEBUG_LOCKING)
MOSAPI int MOSCConv
mos_tlock_getid(mos_tlock_t *this) {

	return (this->ml_id);
}

MOSAPI int MOSCConv
mos_tlock_setid(mos_tlock_t *this, int id) {

	this->ml_id = id;
	return (0);
}

MOSAPI const char * MOSCConv
mos_tlock_getfile(mos_tlock_t *this) {

	return (this->ml_file);
}

MOSAPI int MOSCConv
mos_tlock_getline(mos_tlock_t *this) {

	return (this->ml_line);
}

MOSAPI const char * MOSCConv
mos_tlock_getfunction(mos_tlock_t *this) {

	return (this->ml_func);
}
#endif

void _mos_tlock_init(void);
void _mos_tlock_fini(void);

void
_mos_tlock_init(void) {
#if defined(MOS_DEBUG_LOCKING)
	int i;

	for (i = 0; i < MOSLOCK_THREADS_MAX; i++)
		sthread[i].lt_tid = MOS_TASK_NONE;

#endif /* MOS_DEBUG_LOCKING */

	mos_mutex_init(&slock);
}

void
_mos_tlock_fini(void) {

	mos_mutex_destroy(&slock);
}
