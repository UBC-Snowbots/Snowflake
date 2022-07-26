/*
 * Copyright (c) 1989, 1993
 *  The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Guido van Rossum.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *  This product includes software developed by the University of
 *  California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
 /*
  * Taken from FreeBSD:/usr/src/lib/libc/gen/glob.c.
  */

#include "mos_os.h"

#define M_MASK	0xffffffffffULL

#define META(c)	((char)((c)))

#define M_ALL	META('*')
#define M_ONE	META('?')
#define M_SET	META('[')
#define M_RNG	META('-')
#define M_END	META(']')
#define M_NOT	META('!')

#define EOS		'\0'

static int
match(const char *name, const char *pat, const char *patend) {
	int ok, negate_range;
	char c, k;

	while (pat < patend) {
		c = *pat++;
		switch (c) {
		case M_ALL:
			if (pat == patend)
				return (1);
			do {
				if (match(name, pat, patend))
				return (1);
			} while (*name++ != EOS);
			return (0);
		case M_ONE:
			if (*name++ == EOS)
				return (0);
			break;
		case M_SET:
			ok = 0;
			if ((k = *name++) == EOS)
				return (0);
			if ((negate_range = ((*pat & M_MASK) == M_NOT)) != EOS)
				++pat;
			while (((c = *pat++) & M_MASK) != M_END) {
				if ((*pat & M_MASK) == M_RNG) {
					if (c <= k && k <= pat[1])
						ok = 1;
					pat += 2;
				} else if (c == k) {
					ok = 1;
				}
			}
			if (ok == negate_range)
				return (0);
			break;
		default:
			if (*name++ != c)
				return (0);
			break;
		}
	}
	return (*name == EOS);
}

MOSAPI int MOSCConv
mos_globmatch(const char *str, const char *pattern) {
	const char *patend;

	patend = pattern + mos_strlen(pattern);

	return (match(str, pattern, patend));
}

#if 0
int
main(int argc, char **argv) {

	mos_printf("%s matches %s => %d\n", argv[1], argv[2],
	  mos_globmatch(argv[1], argv[2]));

	exit(0);
}
#endif
