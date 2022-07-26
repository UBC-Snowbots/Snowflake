#ifndef _MOS_DL_H_
#define _MOS_DL_H_

#include "mos_os.h"

#if defined(UNIX)
#include <dlfcn.h>

#define MOS_RTLD_LAZY	RTLD_LAZY
#define MOS_RTLD_NOW	RTLD_NOW
#define MOS_RTLD_GLOBAL	RTLD_GLOBAL
#define MOS_RTLD_LOCAL	RTLD_LOCAL
#else /* flags not supported on other OSes */
#define MOS_RTLD_LAZY	0
#define MOS_RTLD_NOW	0
#define MOS_RTLD_GLOBAL	0
#define MOS_RTLD_LOCAL	0
#endif

MOSAPI void * MOSCConv mos_dlopen(const char *, int);
MOSAPI int MOSCConv mos_dlclose(void *);

MOSAPI void * MOSCConv mos_dlsym(void * __restrict, const char * __restrict);
MOSAPI const char * MOSCConv mos_dlerror(char *, size_t);

#endif /* _MOS_DL_H_ */
