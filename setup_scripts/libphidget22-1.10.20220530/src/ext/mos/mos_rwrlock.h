#ifndef _MOS_RWRLOCK_H_
#define _MOS_RWRLOCK_H_

#include "bsdqueue.h"
#include "mos_os.h"
#include "mos_task.h"
#include "mos_lock.h"

/* generic recursible-reader r/w lock */

typedef struct mos_rwr_reader {
	mos_task_t						r_owner;		/* owning thread */
	uint32_t						r_count;		/* level of recursion */
	MSLIST_ENTRY(mos_rwr_reader)	r_link;			/* link to next thread */
} mos_rwr_reader_t;

typedef struct rwrlock {
	MSLIST_HEAD(rwr_readers, mos_rwr_reader)	rwr_readers;
	mos_task_t									rwr_writer;
	uint32_t									rwr_nreader;
	mos_rwlock_t								rwr_rwlock;
	mos_mutex_t									rwr_lock;	/* protects list */
} mos_rwrlock_t;

MOSAPI int MOSCConv mos_rwrlock_init(mos_rwrlock_t *);
MOSAPI void MOSCConv mos_rwrlock_destroy(mos_rwrlock_t *);
MOSAPI void MOSCConv mos_rwrlock_rdlock(mos_rwrlock_t *);
MOSAPI void MOSCConv mos_rwrlock_wrlock(mos_rwrlock_t *);
MOSAPI void MOSCConv mos_rwrlock_unlock(mos_rwrlock_t *);

MOSAPI uint32_t MOSCConv mos_rwrlock_getreadercount(mos_rwrlock_t *);
MOSAPI uint32_t MOSCConv mos_rwrlock_getreaderholdcount(mos_rwrlock_t *, mos_task_t);

#endif /* _MOS_RWRLOCK_H_ */
