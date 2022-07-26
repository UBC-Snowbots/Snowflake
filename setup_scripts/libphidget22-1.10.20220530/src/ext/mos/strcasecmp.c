#include "mos_os.h"

/*
 * Copied from FreeBSD strcasecmp.c.
 */

MOSAPI int MOSCConv
mos_strcasecmp(const char *s1, const char *s2) {
	const unsigned char *us1;
	const unsigned char *us2;

	us1 = (const unsigned char *)s1;
	us2 = (const unsigned char *)s2;

	while (mos_tolower(*us1) == mos_tolower(*us2)) {
		us2++;
		if (*us1++ == '\0')
			return (0);
	}
	return (mos_tolower(*us1) - mos_tolower(*us2));
}

MOSAPI int MOSCConv
mos_strncasecmp(const char *s1, const char *s2, size_t n) {
	const unsigned char *us1;
	const unsigned char *us2;

	if (n == 0)
		return (0);

	(void)(us1 = (const unsigned char *)s1),
	us2 = (const unsigned char *)s2;

	do {
		if (mos_tolower(*us1) != mos_tolower(*us2))
			return (mos_tolower(*us1) - mos_tolower(*us2));
		us2++;
		if (*++us1 == '\0')
			break;
	} while (--n != 0);

	return (0);
}
