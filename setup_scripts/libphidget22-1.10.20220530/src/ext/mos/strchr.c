#include "mos_os.h"

/*
 * Safer, prefered strchr().  Does not return a const.
 */
MOSAPI char * MOSCConv
mos_strchr(char *p, int ch) {
	char c;

	c = (char)ch;
	for (;; ++p) {
		if (*p == c)
			return (p);
		if (*p == '\0')
			return (NULL);
	}
}

/*
 * Safer, prefered strchr().  Does return a const.
 */
MOSAPI const char * MOSCConv
mos_strchrc(const char *p, int ch) {
	char c;

	c = (char)ch;
	for (;; ++p) {
		if (*p == c)
			return (p);
		if (*p == '\0')
			return (NULL);
	}
}
