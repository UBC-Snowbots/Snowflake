#include "mos_os.h"

MOSAPI char * MOSCConv
mos__strdup(const char *str, uint32_t *len, int flags, const char *file, const char *func, int line) {
	uint32_t lenp;
	char *ret;

	if (len == NULL)
		len = &lenp;

	*len = (uint32_t)mos_strlen(str) + 1;
	if ((ret = _mos_alloc(*len, flags, file, func, line)) == NULL)
		return (NULL);

	memcpy(ret, str, *len);

	return (ret);
}
