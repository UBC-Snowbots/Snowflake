#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>

#include "mos_os.h"
#include "mos_assert.h"

void
_mos_malloc_os_init(void) {
}

void
_mos_malloc_os_fini(void) {
}

MOSAPI void * MOSCConv 
mos__alloc(size_t size, int flags) {
	void *ret;

again:

	MOS_ASSERT(flags & (MOSM_SLP | MOSM_NSLP));

	ret = malloc(size);
	if (ret == NULL) {
		if (flags & MOSM_NSLP)
			return (NULL);
		mos_raw_printf("malloc failure; retrying\n");
		sleep(1);
		goto again;
	}

	if ((flags & MOSM_ZERO) != 0)
		mos_bzero(ret, size);

	return (ret);
}

MOSAPI void MOSCConv
mos__free(void *ptr, size_t sz /* UNUSED */) {

	free(ptr);
}
