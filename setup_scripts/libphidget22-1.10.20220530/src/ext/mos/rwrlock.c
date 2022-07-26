#include "mos_assert.h"
#include "mos_lock.h"
#include "mos_rwrlock.h"

MOSAPI int MOSCConv
mos_rwrlock_init(mos_rwrlock_t *rwr) {
	int res;

	rwr->rwr_writer = MOS_TASK_NONE;
	rwr->rwr_nreader = 0;

	MSLIST_INIT(&rwr->rwr_readers);

	res = mos_rwlock_init(&rwr->rwr_rwlock);
	if (res != 0)
		return (res);

	res = mos_mutex_init(&rwr->rwr_lock);
	if (res != 0) {
		mos_rwlock_destroy(&rwr->rwr_rwlock);
		return (res);
	}

	return (0);
}

MOSAPI void MOSCConv
mos_rwrlock_destroy(mos_rwrlock_t *rwr) {
	mos_rwr_reader_t *r;
	mos_rwr_reader_t *t;

	MSLIST_FOREACH_SAFE(r, &rwr->rwr_readers, r_link, t)
		mos_free(r, sizeof (*r));

	mos_rwlock_destroy(&rwr->rwr_rwlock);
	mos_mutex_destroy(&rwr->rwr_lock);
}

MOSAPI void MOSCConv
mos_rwrlock_rdlock(mos_rwrlock_t *rwr) {
	mos_rwr_reader_t *freeslot;
	mos_rwr_reader_t *r;
	mos_task_t thread;

	thread = mos_self();
	freeslot = NULL;

	/*
	 * If the thread is already a reader, allow it to recurse.  Don't
	 * use rdlock() on the underlying lock, if it would deadlock due
	 * to a waiting writer.
	 */

	mos_mutex_lock(&rwr->rwr_lock);
	MSLIST_FOREACH(r, &rwr->rwr_readers, r_link) {
		if (r->r_owner == thread) {
			r->r_count++;

			if (r->r_count > 1) {
				/*
				 * The rdlock has already been obtained, so we can just return.
				 */
				mos_mutex_unlock(&rwr->rwr_lock);
				return;
			}

			/*
			 * Reacquire the rdlock to lock out writers.
			 */
			goto dordlock;

		} else if (r->r_count == 0) {
			freeslot = r;
		}
	}

	if (r == NULL && freeslot != NULL) {
		/* recycle an old entry */
		r = freeslot;
	} else if (freeslot == NULL) {
		/* need a new entry */
		r = mos_malloc(sizeof (*r));
		MSLIST_INSERT_HEAD(&rwr->rwr_readers, r, r_link);
	}

	r->r_owner = thread;
	r->r_count = 1;

dordlock:

	mos_mutex_unlock(&rwr->rwr_lock);

	mos_rwlock_rdlock(&rwr->rwr_rwlock);

	/* now that we have acquired the rdlock, bump nreader */
	mos_mutex_lock(&rwr->rwr_lock);
	rwr->rwr_nreader++;
	mos_mutex_unlock(&rwr->rwr_lock);
}

MOSAPI void MOSCConv
mos_rwrlock_wrlock(mos_rwrlock_t *rwr) {

	mos_rwlock_wrlock(&rwr->rwr_rwlock);

	mos_mutex_lock(&rwr->rwr_lock);
	MOS_ASSERT(rwr->rwr_writer == MOS_TASK_NONE);
	rwr->rwr_writer = mos_self();
	mos_mutex_unlock(&rwr->rwr_lock);
}

MOSAPI void MOSCConv
mos_rwrlock_unlock(mos_rwrlock_t *rwr) {
	mos_rwr_reader_t *r;
	mos_task_t thread;

	thread = mos_self();

	mos_mutex_lock(&rwr->rwr_lock);

	/* unlocking writer */
	if (rwr->rwr_writer == thread) {
		rwr->rwr_writer = MOS_TASK_NONE;

		mos_mutex_unlock(&rwr->rwr_lock);
		mos_rwlock_unlock(&rwr->rwr_rwlock);
		return;
	}

	/* identify which reader */
	MSLIST_FOREACH(r, &rwr->rwr_readers, r_link) {
		if (r->r_owner == thread)
			break;
	}

	MOS_ASSERT(r != NULL);

	/* unwind one level of recursion */
	MOS_ASSERT(r->r_count > 0);
	r->r_count--;

	/* if this reader is now completely out, unlock the underlying rwlock */
	if (r->r_count == 0) {
		MOS_ASSERT(rwr->rwr_nreader > 0);
		rwr->rwr_nreader--;

		mos_rwlock_unlock(&rwr->rwr_rwlock);
	}

	mos_mutex_unlock(&rwr->rwr_lock);
}

MOSAPI uint32_t MOSCConv
mos_rwrlock_getreadercount(mos_rwrlock_t *rwr) {
	uint32_t res;

	mos_mutex_lock(&rwr->rwr_lock);
	res = rwr->rwr_nreader;
	mos_mutex_unlock(&rwr->rwr_lock);

	return (res);
}

MOSAPI uint32_t MOSCConv
mos_rwrlock_getreaderholdcount(mos_rwrlock_t *rwr, mos_task_t thread) {
	mos_rwr_reader_t *r;
	uint32_t res;

	mos_mutex_lock(&rwr->rwr_lock);

	MSLIST_FOREACH(r, &rwr->rwr_readers, r_link) {
		if (r->r_owner == thread)
			break;
	}

	if (r != NULL)
		res = r->r_count;
	else
		res = 0;

	mos_mutex_unlock(&rwr->rwr_lock);

	return (res);
}
