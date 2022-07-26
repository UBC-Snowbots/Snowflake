#ifndef _MOS_ATOMIC_H_
#define _MOS_ATOMIC_H_

#include "mos_os.h"

void _mos_atomic_init(void);

MOSAPI void MOSCConv mos_atomic_add_32(uint32_t *, int32_t);
MOSAPI void MOSCConv mos_atomic_add_64(uint64_t *, int64_t);

MOSAPI uint32_t MOSCConv mos_atomic_add_32_nv(uint32_t *, int32_t);
MOSAPI uint64_t MOSCConv mos_atomic_add_64_nv(uint64_t *, int64_t);

MOSAPI uint32_t MOSCConv mos_atomic_get_32(uint32_t *);
MOSAPI uint64_t MOSCConv mos_atomic_get_64(uint64_t *);

MOSAPI uint32_t MOSCConv mos_atomic_swap_32(uint32_t *, uint32_t);
MOSAPI uint64_t MOSCConv mos_atomic_swap_64(uint64_t *, uint64_t);

#endif /* _MOS_ATOMIC_H_ */
