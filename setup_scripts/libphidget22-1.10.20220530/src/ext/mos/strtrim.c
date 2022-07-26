
#include "mos_os.h"
#include "mos_iop.h"

MOSAPI const char * MOSCConv
mos_strtrim(const char *src, char *dst, size_t dsz) {
	const char *start;
	char *end;

	if (src == NULL || dst == NULL)
		return ("");

	for (start = src; mos_isspace(*start) && *start; start++)
		;

	if (*start == '\0')
		goto empty;

	mos_strlcpy(dst, start, dsz);

	for (end = (dst + (mos_strlen(dst) - 1)); end >= dst; end--) {
		if (!mos_isspace(*end)) {
			*(end + 1) = '\0';
			return (dst);
		}
	}

empty:

	dst[0] = '\0';
	return (dst);
}
