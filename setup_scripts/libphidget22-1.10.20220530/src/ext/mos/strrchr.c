#include "mos_os.h"

/*
 * Safer, prefered strrchr().  Does not return a const.
 */
MOSAPI char* MOSCConv
mos_strrchr(char *p, int ch) {
	char *result;
	char c;

	result = NULL;
	c = (char)ch;
	for (;; ++p) {
		if (*p == c)
			result = (p);
		if (*p == '\0')
			return (result);
	}
}

/*
 * Safer, prefered strrchr().  Does return a const.
 */
MOSAPI const char * MOSCConv
mos_strrchrc(const char *p, int ch) {
	const char *result;
	char c;

	result = NULL;
	c = (char)ch;
	for (;; ++p) {
		if (*p == c)
			result = (p);
		if (*p == '\0')
			return (result);
	}
}
