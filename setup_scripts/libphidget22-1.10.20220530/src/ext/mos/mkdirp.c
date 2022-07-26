/*
 * Copyright (c) 1988, 1989, 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
 * Copyright (c) 1989 by Berkeley Softworks
 * All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Adam de Boor.
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

/*
 * mkdirp() that should work on Windows too.  Based on mkdirp() in pmake.
 */

#include <stdio.h>
#include <errno.h>

#ifdef Windows
#include <direct.h>
#define mkdir(dir, mode)	_mkdir(dir)
#define chdir(dir)			_chdir(dir)
#else
#include <unistd.h>
#include <sys/stat.h>
#endif

#include "mos_os.h"

static int
_mkdirp(const char *dir, int mode) {

	if (chdir(dir) == 0)
		return (0);

	if (errno != ENOENT)
		return (-1);

	if (mkdir(dir, mode) != 0)
		return (-1);

	if (chdir(dir) != 0)
		return (-1);

	return (0);
}

MOSAPI int MOSCConv
mos_mkdirp(const char *dir, int mode) {
	char oldwd[1024];
	char dbuf[1024];
	char drive[4];
	char *s, *e;
	int err;

	if (strlen(dir) > sizeof (dbuf))
		return (-1);
	err = mos_getcwd(oldwd, sizeof (oldwd));
	if (err != 0)
		return (-1);

	mos_strlcpy(dbuf, dir, sizeof (dbuf));

	s = dbuf;

	if (dbuf[0] == '/') {
		err = chdir("/");
		if (err != 0)
			return (err);
		s++;
	} else if (strlen(dbuf) >= 3 && dbuf[1] == ':' && (dbuf[2] == '/' || dbuf[2] == '\\')) {
		/* handle absolute path with DOS drive */
		mos_snprintf(drive, sizeof (drive), "%c:/", dbuf[0]);
		err = chdir(drive);
		if (err != 0)
			return (err);
		s += 3;
	}

	for (e = s; *e != '\0'; e++) {
		if (*e != '/' && *e != '\\')
			continue;

		if (s == e) {
			s++;
			continue;
		}

		*e = '\0';
		if (_mkdirp(s, mode) != 0) {
			fprintf(stderr, "failed to create %s of %s\n", s, dir);
			return (-1);
		}

		s = e + 1;
	}

	if (s != e) {
		if (_mkdirp(s, mode) != 0) {
			fprintf(stderr, "failed to create %s of %s\n", s, dir);
			return (-1);
		}
	}

	err = chdir(oldwd);
	if (err != 0) {
		fprintf(stderr, "failed to change to '%s'\n", oldwd);
		return (-1);
	}

	return (0);
}
