#ifndef _MOS_STACKTRACE_H_
#define _MOS_STACKTRACE_H_

#include "mos_os.h"

/*
 * Fills in the given buffer with the return program counters from each
 * stack frame.  Returns the number of valid frames, or 0 if not
 * determined.
 */
MOSAPI size_t MOSCConv mos_stacktrace(void **, size_t);

/*
 * Fills in the given buffer with the name of the symbol at the given
 * address.  Returns the actual number of bytes in the name so the
 * caller can tell if their buffer was too small.  Returns 0 if no
 * translation is possible or implemented.
 */
MOSAPI size_t MOSCConv mos_getsymbolname(const void *, char *, size_t);

#endif /* _MOS_STACKTRACE_H_ */
