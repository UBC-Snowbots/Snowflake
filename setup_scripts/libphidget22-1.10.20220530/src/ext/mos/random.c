#include "mos_random.h"

int
mosrandom_getu64(mosiop_t iop, uint64_t *res) {
	mosrandom_t *r;
	int err;

	err = mosrandom_alloc(iop, NULL, 0, &r);
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to get secure random source"));

	err = mosrandom_getbytes(r, iop, (uint8_t *)res, sizeof (*res));
	mosrandom_free(&r);
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to get random bytes"));

	return (0);
}
