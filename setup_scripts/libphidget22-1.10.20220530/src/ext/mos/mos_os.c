#include "mos_os.h"
#include "mos_atomic.h"
#include "mos_lock.h"
#include "mos_iop.h"
#include "mos_netops.h"

extern void _mos_base_init(void);
extern void _mos_base_fini(void);
extern void _mos_glock_init(void);
extern void _mos_glock_fini(void);
extern void _mos_tlock_init(void);
extern void _mos_tlock_fini(void);
extern void _mos_task_init(void);
extern void _mos_task_fini(void);

mos_mutex_t				*mos_global_iop_id_lock;
static mos_mutex_t		real_global_iop_id_lock;

mos_cond_t				*mos_error_walker_released_cv;
static mos_cond_t		real_error_walker_released_cv;

#define MAX_CLEANUP	8

static mos_cleanup_t cleanup[MAX_CLEANUP];

int
mos_register_cleanup(mos_cleanup_t func) {
	int i;

	mos_glock(cleanup);

	for (i = 0; i < MAX_CLEANUP; i++) {
		if (cleanup[i] == NULL) {
			cleanup[i] = func;
			mos_gunlock(cleanup);
			return (0);
		}
	}
	mos_gunlock(cleanup);
	return (MOSN_NOSPC);
}

MOSAPI void MOSCConv
mos_init() {

	_mos_lock_init();
	_mos_base_init();
	_mos_glock_init();
	_mos_task_init();
	_mos_netops_init();
	_mos_tlock_init();
	_mos_atomic_init();

	mos_global_iop_id_lock = &real_global_iop_id_lock;
	mos_mutex_init(mos_global_iop_id_lock);

	mos_error_walker_released_cv = &real_error_walker_released_cv;
	mos_cond_init(mos_error_walker_released_cv);
}

void
mos_cleanup(void) {
	int i;

	for (i = 0; i < MAX_CLEANUP; i++) {
		if (cleanup[i] != NULL) {
			cleanup[i]();
			cleanup[i] = NULL;
		}
	}
}

MOSAPI void MOSCConv
mos_fini() {

	mos_cleanup();

	mos_mutex_destroy(mos_global_iop_id_lock);
	mos_global_iop_id_lock = NULL;

	mos_cond_destroy(mos_error_walker_released_cv);
	mos_error_walker_released_cv = NULL;

	_mos_glock_fini();
	_mos_base_fini();
	_mos_task_fini();
	_mos_lock_fini();
	_mos_tlock_fini();
}

static mos_task_thread_exit_callback threadExit;
static void *threadExitCtx;

MOSAPI void MOSCConv
mos_task_exiting() {

	if (threadExit)
		threadExit(threadExitCtx);
}

MOSAPI void MOSCConv
mos_task_set_thread_exit_handler(mos_task_thread_exit_callback fptr, void *ctx) {

	threadExit = fptr;
	threadExitCtx = ctx;
}
