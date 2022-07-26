#ifndef _MOS_BASE_TYPES_H_
#define _MOS_BASE_TYPES_H_

#if defined(Windows)

#if defined(_KERNEL)
#ifndef EXCLUDE_INT_LIMIT_DEFS
#include <intsafe.h>
#endif
#else /* USER */
#include <wtypes.h>
#ifndef EXCLUDE_INT_LIMIT_DEFS
#include <ntintsafe.h>
#endif
#endif
#include "mos_basic_types-win.h"

#else /* UNIX */

#include <sys/types.h>
#include <sys/time.h>
#if !defined(Darwin) && !defined(_KERNEL)
#include <inttypes.h>
#include <limits.h>
#endif /* !Darwin && !_KERNEL */

#if defined(Darwin)
#include <stdint.h>
#endif /* Darwin */

#endif /* UNIX */

#include <stdarg.h>

#ifndef MOSExport
#define MOSExport
#endif

#ifndef MOSImport
#define MOSImport
#endif

#if defined(MOS_MOS_SOURCE)
#define MOSAPI					MOSExport
#else
#define MOSAPI					MOSImport
#endif

#if !defined(NULL)
#define NULL (void *)0
#endif

#if !defined(__restrict)
#define __restrict
#endif

#if defined(Windows)
#define MOSCConv	WINAPI
#else
#define MOSCConv
#endif

#define MOS_UINT8_MAX	((uint8_t)0xffUL)
#define MOS_INT8_MAX	((int8_t)0x7fUL)
#define MOS_INT8_MIN	((int8_t)(-0x7fL - 1))

#define MOS_UINT16_MAX	((uint16_t)0xffffUL)
#define MOS_INT16_MAX	((int16_t)0x7fffL)
#define MOS_INT16_MIN	((int16_t)(-0x7fffL - 1))

#define MOS_UINT32_MAX	((uint32_t)0xffffffffUL)
#define MOS_INT32_MAX	((int32_t)0x7fffffffL)
#define MOS_INT32_MIN	((int32_t)(-0x7fffffffL - 1))

#define MOS_UINT64_MAX	((uint64_t)0xffffffffffffffffULL)
#define MOS_INT64_MAX	((uint64_t)0x7fffffffffffffffLL)
#define MOS_INT64_MIN	((int64_t)(-0x7fffffffffffffffLL - 1))

typedef unsigned long long ull_t;

#if !defined(Windows)
#ifndef BOOL
#define BOOL _Bool
#endif
#endif

MOSAPI int MOSCConv mosdata_string(const uint8_t *, size_t, char *, size_t);
MOSAPI int MOSCConv mos_hex_string(const uint8_t *, size_t, char *, size_t);

#ifndef NBBY
#define NBBY 8 /* bits in a byte */
#endif

/* (BSD) Bit map related macros. */

#define mos_setbit(a,i) ((a)[(i)/NBBY] |= 1<<((i)%NBBY))
#define mos_clrbit(a,i) ((a)[(i)/NBBY] &= ~(1<<((i)%NBBY)))
#define mos_isset(a,i)  ((a)[(i)/NBBY] & (1<<((i)%NBBY)))
#define mos_isclr(a,i)  (((a)[(i)/NBBY] & (1<<((i)%NBBY))) == 0)

/* (BSD) Macros for counting and rounding. */

#define mos_powerof2(x) ((((x)-1)&(x)) == 0)
#define mos_howmany(x, y)   (((x)+((y)-1))/(y))
#define mos_rounddown(x, y) (((x)/(y))*(y))
#define mos_roundup(x, y)   ((((x)+((y)-1))/(y))*(y))  /* to any y */
#define mos_roundup2(x, y)  (((x)+((y)-1))&(~((y)-1))) /* if y is powers of two */

/* Macros for min/max. */
#define MOS_MIN(a,b) (((a)<(b))?(a):(b))
#define MOS_MAX(a,b) (((a)>(b))?(a):(b))
#define MOS_ABS(a) (((a) < 0) ? -(a) : (a))

#define MOS_ABSDIFF(a,b)  (((a)>(b))?(a)-(b):(b)-(a))

/* ASCII macros */
#ifndef isletter
#define isletter(x) ((x >= 65 && x <= 90) || (x >= 97 && x <= 122))
#endif

#endif /* _MOS_BASE_TYPES_H_ */
