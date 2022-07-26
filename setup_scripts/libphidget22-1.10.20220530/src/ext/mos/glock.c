#include "mos_basic.h"
#include "mos_lock.h"

#define GLOCK_COUNT	8

static mos_mutex_t glock[GLOCK_COUNT];

MOSAPI void MOSCConv _mos_glock_init(void);
MOSAPI void MOSCConv
_mos_glock_init() {
	int i;

	for (i = 0; i < GLOCK_COUNT; i++)
		mos_mutex_init(&glock[i]);
}

void _mos_glock_fini(void);
void
_mos_glock_fini() {
	int i;

	for (i = 0; i < GLOCK_COUNT; i++)
		mos_mutex_destroy(&glock[i]);
}

MOSAPI int MOSCConv
mos_glock(void *ptr) {
	int i;

	i = ((int)(intptr_t)ptr) % GLOCK_COUNT;
	mos_mutex_lock(&glock[i]);

	return (0);
}

MOSAPI int MOSCConv
mos_gunlock(void *ptr) {
	int i;

	i = ((int)(intptr_t)ptr) % GLOCK_COUNT;
	mos_mutex_unlock(&glock[i]);

	return (0);
}
