#ifndef _MOS_ASSERT_H_
#define _MOS_ASSERT_H_

#if defined(NDEBUG)

#define MOS_ASSERT(v)	((void)0)

#endif /* NDEBUG */

/******************************************************************************
 * KERNEL
 ******************************************************************************/
#if defined(_KERNEL)

/******************************************************************************
 * SUNOS KERNEL
 ******************************************************************************/
#if defined(SunOS)

#include <sys/debug.h>

#if !defined(NDEBUG)
#define _MOS_ASSERT(v) ASSERT(v)
#endif

#define _MOS_PANIC(...) panic(__VA_ARGS__)

#endif

/******************************************************************************
 * FreeBSD KERNEL
 ******************************************************************************/
#if defined(FreeBSD)

#if !defined(NDEBUG)
#define _MOS_ASSERT(v) KASSERT(v, (__FILE__))
#endif

#define _MOS_PANIC(...) panic(__VA_ARGS__)

#endif

/******************************************************************************
 * DARWIN KERNEL
 ******************************************************************************/
#if defined(Darwin)

#define MACH_ASSERT 1

#include "kern/assert.h"
#include "kern/debug.h"

#if !defined(NDEBUG)
#define _MOS_ASSERT(v) assert((v))
#endif

#undef panic

#define _MOS_PANIC(...)panic(__VA_ARGS__)

#endif

/******************************************************************************
 * Windows KERNEL
 ******************************************************************************/
#if defined(Windows)

#if !defined(NDEBUG)
/* for breakpoints */
static void mos_assfail(void) {}

#define _MOS_ASSERT(v) ((v) || (ASSERT(v), mos_assfail(), 1))

#endif

#define _MOS_PANIC(...) (ASSERTMSG(0, !"MOS PANIC"), DbgBreakPoint())

#endif  /* Windows */

/******************************************************************************
 * USERLAND
 ******************************************************************************/
#else /* !_KERNEL */

#include <assert.h>
#include <stdlib.h>

#if !defined(NDEBUG)
#define _MOS_ASSERT(v) (assert((v)))
#endif

void mos_log_err(const char *, ...);
#define _MOS_PANIC(...) mos_log_err(__VA_ARGS__), abort()

#endif	/* _KERNEL */

/******************************************************************************
 * MAIN MACROS
 ******************************************************************************/
#ifdef MOS_HOLD_FAILURES
#if !defined(NDEBUG)
#define MOS_ASSERT(v) if (!(v)) {										\
	mos_printef("HOLD ASSERT: %s at %s:%d\n", #v, __FILE__, __LINE__);	\
	while (1)															\
		;																\
}
#endif

#define MOS_PANIC(...) {		\
	mos_printef(__VA_ARGS__);	\
	while (1)					\
		;						\
}
#else /* !MOS_HOLD_FAILURES */

#if !defined(NDEBUG)
#define MOS_ASSERT(v) _MOS_ASSERT(v)
#endif

#define MOS_PANIC(...) _MOS_PANIC(__VA_ARGS__)
#endif /* MOS_HOLD_FAILURES */

#endif /* _MOS_ASSERT_H_ */
