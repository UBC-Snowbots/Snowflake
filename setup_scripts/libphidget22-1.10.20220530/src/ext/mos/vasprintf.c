#include "mos_os.h"

MOSAPI int MOSCConv
mos_asprintf(char **ret, uint32_t *len, const char *fmt, ...) {
	va_list ap;
	int err;

	va_start(ap, fmt);
	err = mos_vasprintf(ret, len, fmt, ap);
	va_end(ap);

	return (err);
}

MOSAPI int MOSCConv
mos_vasprintf(char **ret, uint32_t *_len, const char *fmt, va_list ap) {
	char tbuf[12];
	uint32_t *lenp;
	va_list ap2;
	uint32_t len;
	uint32_t n;

	if (_len != NULL)
		lenp = _len;
	else
		lenp = &len;

	*lenp = 0;

	va_copy(ap2, ap);
	n = mos_vsnprintf(tbuf, sizeof(tbuf), fmt, ap);

	*lenp = n + 1;
	*ret = mos_malloc(*lenp);

	return (mos_vsnprintf(*ret, *lenp, fmt, ap2));
}
