#include <pthread.h>
#include "mos_atomic.h"

static pthread_mutex_t sync_lock = PTHREAD_MUTEX_INITIALIZER;

void
mos_atomic_add_32(uint32_t *dst, int32_t delta) {

	pthread_mutex_lock(&sync_lock);
	*dst += delta;
	pthread_mutex_unlock(&sync_lock);
}

uint32_t
mos_atomic_add_32_nv(uint32_t *dst, int32_t delta) {
	uint32_t res;

	pthread_mutex_lock(&sync_lock);
	res = *dst + delta;
	*dst = res;
	pthread_mutex_unlock(&sync_lock);

	return (res);
}

void
mos_atomic_add_64(uint64_t *dst, int64_t delta) {

	pthread_mutex_lock(&sync_lock);
	*dst += delta;
	pthread_mutex_unlock(&sync_lock);
}

uint64_t
mos_atomic_add_64_nv(uint64_t *dst, int64_t delta) {
	uint64_t res;

	pthread_mutex_lock(&sync_lock);
	res = *dst + delta;
	*dst = res;
	pthread_mutex_unlock(&sync_lock);

	return (res);
}

uint32_t
mos_atomic_get_32(uint32_t *dst) {
	uint32_t res;

	pthread_mutex_lock(&sync_lock);
	res = *dst;
	pthread_mutex_unlock(&sync_lock);

	return (res);
}


uint64_t
mos_atomic_get_64(uint64_t *dst) {
	uint64_t res;

	pthread_mutex_lock(&sync_lock);
	res = *dst;
	pthread_mutex_unlock(&sync_lock);

	return (res);
}

uint32_t
mos_atomic_swap_32(uint32_t *dst, uint32_t nv) {
	uint32_t ov;

	pthread_mutex_lock(&sync_lock);
	ov = *dst;
	*dst = nv;
	pthread_mutex_unlock(&sync_lock);

	return (ov);
}

uint64_t
mos_atomic_swap_64(uint64_t *dst, uint64_t nv) {
	uint64_t ov;

	pthread_mutex_lock(&sync_lock);
	ov = *dst;
	*dst = nv;
	pthread_mutex_unlock(&sync_lock);

	return (ov);
}

void
_mos_atomic_init(void) {
}
