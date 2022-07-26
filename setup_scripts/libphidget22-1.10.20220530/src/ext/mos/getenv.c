#include "mos_os.h"

MOSAPI const char * MOSCConv
mos_getenv(const char *name) {

	return (getenv(name));
}
