#include <stdlib.h>
#include "mos_os.h"

MOSAPI int MOSCConv
mos_setenv(const char *name, const char *val, int overwrite) {

	return (setenv(name, val, overwrite));
}

MOSAPI int MOSCConv
mos_unsetenv(const char *name) {

#if defined(Darwin) && !__DARWIN_UNIX03
	unsetenv(name);
	return 0;
#else
	return (unsetenv(name));
#endif
}
