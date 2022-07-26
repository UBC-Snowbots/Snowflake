#include "mos_basic.h"

#if defined(Windows)
#include <direct.h>
#endif

#if defined(UNIX)
#include <unistd.h>
#endif

#include "mos_error-errno.h"

MOSAPI int MOSCConv
mos_getcwd(char *dbuf, size_t dbufsz) {
	const char *ret;

#if defined(Windows)
	ret = _getcwd(dbuf, (int)dbufsz);
#else
	ret = getcwd(dbuf, dbufsz);
#endif

	if (ret == NULL)
		return (mos_fromerrno(errno));

	return (0);
}
