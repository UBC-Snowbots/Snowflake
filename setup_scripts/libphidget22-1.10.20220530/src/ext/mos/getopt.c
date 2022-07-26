/*	$NetBSD: getopt.c,v 1.26 2003/08/07 16:43:40 agc Exp $	*/

/*
 * Copyright (c) 1987, 1993, 1994
 *	The Regents of the University of California.  All rights reserved.
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
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
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
#define MOS_GETOPT 1

#include "mos_os.h"

int	mos_opterr = 1,		/* if error message should be printed */
	mos_optind = 1,		/* index into parent argv vector */
	mos_optopt,			/* character checked for validity */
	mos_optreset;		/* reset getopt */
char *mos_optarg;		/* argument associated with option */

#define	BADCH	(int)'?'
#define	BADARG	(int)':'
#define	EMSG	""

/*
 * getopt --
 *	Parse argc/argv argument vector.
 */
MOSAPI int MOSCConv
mos_getopt(int nargc, char * const nargv[], const char *ostr) {
	static char *place = NULL;		/* option letter processing */
	char *oli;				/* option letter list index */

	if (mos_optreset || place == NULL || *place == 0) {		/* update scanning pointer */
		mos_optreset = 0;
		place = nargv[mos_optind];
		if (mos_optind >= nargc || *place++ != '-') {
			/* Argument is absent or is not an option */
			place = NULL;
			return (-1);
		}
		mos_optopt = *place++;
		if (mos_optopt == '-' && *place == 0) {
			/* "--" => end of options */
			++mos_optind;
			place = NULL;
			return (-1);
		}
		if (mos_optopt == 0) {
			/* Solitary '-', treat as a '-' option
			   if the program (eg su) is looking for it. */
			place = NULL;
			if (strchr(ostr, '-') == NULL)
				return (-1);
			mos_optopt = '-';
		}
	} else {
		mos_optopt = *place++;
	}

	/* See if option letter is one the caller wanted... */
	if (mos_optopt == ':' || (oli = strchr(ostr, mos_optopt)) == NULL) {
		if (place == NULL || *place == 0)
			++mos_optind;
		if (mos_opterr && *ostr != ':')
			(void)fprintf(stderr, "illegal option -- %c\n", mos_optopt);
		return (BADCH);
	}

	/* Does this option need an argument? */
	if (oli[1] != ':') {
		/* don't need argument */
		mos_optarg = NULL;
		if (place == NULL || *place == 0)
			++mos_optind;
	} else {
		/* Option-argument is either the rest of this argument or the
		   entire next argument. */
		if (place != NULL && *place)
			mos_optarg = place;
		else if (nargc > ++mos_optind)
			mos_optarg = nargv[mos_optind];
		else {
			/* option-argument absent */
			place = NULL;
			if (*ostr == ':')
				return (BADARG);
			if (mos_opterr)
				(void)fprintf(stderr, "option requires an argument -- %c\n", mos_optopt);
			return (BADCH);
		}
		place = NULL;
		++mos_optind;
	}
	return (mos_optopt);			/* return option letter */
}
