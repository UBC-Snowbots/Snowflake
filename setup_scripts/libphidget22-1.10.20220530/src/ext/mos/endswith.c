#include "mos_os.h"

MOSAPI int MOSCConv
mos_endswith(const char *str, const char *e) {
	const char *c;
	size_t elen;
	size_t slen;

	slen = mos_strlen(str);
	elen = mos_strlen(e);

	if (elen > slen)
		return (0);

	c = str + (slen - elen);
	if (mos_strcmp(c, e) == 0)
		return (1);
	return (0);
}
