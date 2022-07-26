#include "mos_os.h"

/*
 * Reverses a string.  Taken from a public domain example by Bob Stout
 * at http://gd.tuwien.ac.at/languages/c/cref-mleslie/CONTRIB/SNIP/strrev.c.
 */
MOSAPI char* MOSCConv
mos_strrev(char *p) {
	char *p1, *p2;

	if (!p || ! *p)
		return p;
	for (p1 = p, p2 = p + mos_strlen(p) - 1; p2 > p1; ++p1, --p2) {
		*p1 ^= *p2;
		*p2 ^= *p1;
		*p1 ^= *p2;
	}
	return (p);
}
