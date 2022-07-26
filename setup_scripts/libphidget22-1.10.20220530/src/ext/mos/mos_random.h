#ifndef _MOS_RANDOM_H_
#define _MOS_RANDOM_H_

#include "mos_iop.h"

typedef struct mosrandom mosrandom_t;

MOSAPI int MOSCConv mosrandom_alloc(mosiop_t, const uint8_t *, size_t, mosrandom_t **);
MOSAPI int MOSCConv mosrandom_getbytes(mosrandom_t *, mosiop_t, uint8_t *, size_t);
MOSAPI void MOSCConv mosrandom_free(mosrandom_t **);
MOSAPI int MOSCConv mosrandom_getu64(mosiop_t, uint64_t *);

MOSAPI int32_t MOSCConv mos_jrand48(uint16_t[3]);
MOSAPI int32_t MOSCConv mos_mrand48(void);
MOSAPI int32_t MOSCConv mos_nrand48(uint16_t[3]);
MOSAPI int32_t MOSCConv mos_lrand48(void);
MOSAPI uint16_t * MOSCConv mos_seed48(uint16_t[3]);
MOSAPI void MOSCConv mos_srand48(int32_t);
MOSAPI void MOSCConv mos_lcong48(uint16_t[7]);

#endif /* _MOS_RANDOM_H_ */
