
#include <pthread.h>
#include <limits.h>

#include "mos_os.h"
#include "mos_task.h"
#include "mos_iop.h"

#define DEFAULT_STACK_SIZE	(1048576 * 4)

/*
 * Creates a task.  There is a race condition here if multiple tasks
 * are created close together, but since we only create on task in the
 * filesystem this shouldn't be an issue.
 */
MOSAPI int MOSCConv
mos_task_create(mos_task_t *task, void (*start)(void *), void *arg) {
	
	return (mos_task_create2(task, DEFAULT_STACK_SIZE, start, arg));
}

MOSAPI int MOSCConv
mos_task_create2(mos_task_t *task, uint32_t stacksz, MOS_TASK_RESULT (*start)(void *), void *arg) {
	pthread_t thrd;
	pthread_attr_t attr;
	int ret;
	
	ret = pthread_attr_init(&attr);
	if (ret)
		return (MOSN_ERR);
	
	if (stacksz) {
		if (stacksz < PTHREAD_STACK_MIN)
			stacksz = PTHREAD_STACK_MIN;
		ret = pthread_attr_setstacksize(&attr, stacksz);
		if (ret)
			return (MOSN_ERR);
	}
	
	ret = pthread_create(&thrd, &attr, (void *(*)(void *))start, arg);
	switch(ret) {
		case 0:
			if (task != NULL)
				*task = thrd;
			pthread_detach(thrd);
			return (0);
		case EAGAIN:
			return (MOSN_AGAIN);	/* over current thread limit */
		case EPERM:
			return (MOSN_PERM);		/* shouldn't happen - scheduling issue */
		case EINVAL:
			return (MOSN_INVAL);	/* invalid parameters */
		default:
			return (MOSN_ERR);
	}
}

MOSAPI int MOSCConv
mos_task_exit(int _dummy) {

	pthread_exit(NULL);
}

MOSAPI mos_task_t MOSCConv
mos_self(void) {

	return (pthread_self());
}

MOSAPI int MOSCConv
mos_task_equal(mos_task_t t1, mos_task_t t2) {

	return (pthread_equal(t1, t2));
}

void _mos_task_init(void);

void
_mos_task_init(void) {
}

void _mos_task_fini(void);

void
_mos_task_fini(void) {
}

void
mos_task_setname(const char *fmt, ...) {
#if defined(Darwin)
	char threadName[16];
	va_list va;

	va_start(va, fmt);
	mos_vsnprintf(threadName, sizeof(threadName), fmt, va);
	va_end(va);

	pthread_setname_np(threadName);
#endif
// XXX - Figure this out for Linux - but sometimes pthread_setname_np
//       doesn't exist, sometimes it takes 2 args, etc..
}
