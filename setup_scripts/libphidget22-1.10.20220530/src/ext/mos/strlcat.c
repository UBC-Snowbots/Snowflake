#include "mos_os.h"

/*
 * Copied from FreeBSD5 strlcat.c, written by Todd Miller.
 */
MOSAPI size_t MOSCConv
mos_strlcat(char *dst, const char *src, size_t siz) {
	char *d;
	const char *s;
	size_t n;
	size_t dlen;

	d = dst;
	s = src;
	n = siz;

	while (n-- != 0 && *d != '\0')
		d++;
	dlen = d - dst;
	n = siz - dlen;

	if (n == 0)
		return (dlen + mos_strlen(s));
	while (*s != '\0') {
		if (n != 1) {
			*d++ = *s;
			n--;
		}
		s++;
	}
	*d = '\0';

	return (dlen + (s - src));
}

