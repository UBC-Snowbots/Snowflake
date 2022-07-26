
#ifdef Windows
#include <direct.h>
#include <errno.h>
#define mkdir(dir, mode) _mkdir(dir)
#else
#if !defined (_KERNEL)
#include <unistd.h>
#include <sys/stat.h>
#endif
#endif

#include "mos_os.h"
#include "mos_iop.h"

#if defined(Windows)
#define issep(c)	((c) == '/' || (c) == '\\')
#define issepstr(c)	(c[0] == '/' || c[0] == '\\')

static const char *specials[] = { "\\\\.\\", "\\\\?\\", NULL };

static int
isspecial(const char *path) {
	int i;

	if (path[0] == '\0')
		return (0);

	if (((path[0] >= 'a' && path[0] <= 'z') ||
	 (path[0] >= 'A' && path[0] <= 'Z')) && path[1] == ':')
		return (1);

	for (i = 0; specials[i] != NULL; i++) {
		if (mos_strncmp(path, specials[i], mos_strlen(specials[i])) == 0)
			return (1);
	}

	return (0);
}

/*
 * Puts any "special" path beginnings (e.g. "c:/") into comp[0] and returns a
 * pointer to the part of the path that starts after the "special".
 */
static char *
getspecial(char *path, const char **comps, uint32_t *cnt) {
	int i;

	*cnt = 0;

	if (((path[0] >= 'a' && path[0] <= 'z') ||
	 (path[0] >= 'A' && path[0] <= 'Z')) && path[1] == ':') {
		if (path[2] == '\0') {
			comps[0] = path;
			path += 2;
			*cnt = 1;
		} else if (issep(path[2]) && path[3] == '\0') {
			comps[0] = path;
			path += 3;
			*cnt = 1;
		} else if (issep(path[2]) && issep(path[3])) {
			/*
			 * If the special has at least "c://", consume separators until we
			 * hit something that isn't a separator.
			 */
			comps[0] = path;
			path += 3;
			for (;issep(*path); path++)
				*path = '\0';
			*cnt = 1;
		} else {
			path[2] = '\0';
			comps[0] = path;
			path += 3;
			*cnt = 1;
		}
	}

	for (i = 0; specials[i] != NULL; i++) {
		if (mos_strncmp(path, specials[i], mos_strlen(specials[i])) == 0) {
			comps[0] = specials[i];
			path += mos_strlen(specials[i]);
			*cnt = 1;
			break;
		}
	}

	return (path);
}

#else
#define issep(c)				(c == '/')
#define issepstr(c)				(c[0] == '/')
#define getspecial(p, c, n)		(*(n) = 0, (p))
#define isspecial(p)			(0)
#endif


/*
 * Determines if the path is '..' or ends with '/..'
 */
MOSAPI int MOSCConv
mos_path_isdotdot(const char *path) {
	const char *c;
	size_t len;

	len = mos_strlen(path);
	if (len < 1)
		return (0);

	c = &(path[len - 1]);
	if (*c != '.')
		return (0);

	c--;
	if (c < path)
		return (0);

	if (*c != '.')
		return (0);

	c--;
	if (c < path || issep(*c))
		return (1);

	return (0);
}

MOSAPI int MOSCConv
mos_path_isdot(const char *path) {
	const char *c;
	size_t len;

	len = mos_strlen(path);
	if (len < 1)
		return (0);

	c = &(path[len - 1]);
	if (*c != '.')
		return (0);

	c--;
	if (c < path || issep(*c))
		return (1);

	return (0);
}

MOSAPI int MOSCConv
mos_path_isabsolute(const char *path) {

	if (path == NULL || path[0] == '\0')
		return (0);

	if (isspecial(path))
		return (1);

	if (path[0] == '/')
		return (1);

	return (0);
}

static int
_parse_path(char *path, const char **comps, uint32_t *cnt, int *lastdir) {
	uint32_t ocnt;
	char *c, *s;
	int ss;

	if (path == NULL || comps == NULL || cnt == NULL || lastdir == NULL)
		return (MOSN_INVALARG);

	ocnt = *cnt;

	*lastdir = 0;
	ss = 0;

	comps[0] = NULL;

	path = getspecial(path, comps, cnt);

	for (c = s = path; *c; c++) {
		if (issep(*c)) {
			if (ss) {
				s++;

				if (*(c + 1) == '\0')
					*lastdir = 1;
				continue;
			} else {
				ss = 1;

				*c = '\0';

				if (mos_strlen(s) == 0)
					comps[*cnt] = NULL;
				else
					comps[*cnt] = s;
				(*cnt)++;
				if (*cnt + 1 >= ocnt)
					return (MOSN_NOSPC);

				s = c + 1;

				if (*(c + 1) == '\0')
					*lastdir = 1;
			}
		} else {
			ss = 0;
		}
	}

	if (s != c) {
		comps[*cnt] = s;
		(*cnt)++;
	}

	if (comps[0] == NULL)
		comps[0] = "/";

	return (0);
}

MOSAPI int MOSCConv
mos_path_count(const char *pathp, uint32_t *cnt) {
	const char *comps[MOS_PATH_COMPONENTS_MAX];
	char path[MOS_PATH_MAX];
	int lastdir;
	int err;

	if (mos_strlcpy(path, pathp, sizeof (path)) >= sizeof (path))
		return (MOSN_INVAL);

	*cnt = MOS_PATH_COMPONENTS_MAX;
	err = _parse_path(path, comps, cnt, &lastdir);
	if (err != 0)
		return (err);

	return (0);
}

MOSAPI const char * MOSCConv
mos_path_get(const char *pathp, uint32_t idx, char *gbuf, uint32_t gbufsz) {
	const char *comps[MOS_PATH_COMPONENTS_MAX];
	char path[MOS_PATH_MAX];
	uint32_t cnt;
	int lastdir;
	int err;

	if (mos_strlcpy(path, pathp, sizeof (path)) >= sizeof (path)) {
		gbuf[0] = '\0';
		return (NULL);
	}

	cnt = MOS_PATH_COMPONENTS_MAX;
	err = _parse_path(path, comps, &cnt, &lastdir);
	if (err != 0) {
		gbuf[0] = '\0';
		return (NULL);
	}

	if (cnt <= idx) {
		gbuf[0] = '\0';
		return (NULL);
	}

	mos_strlcpy(gbuf, comps[idx], gbufsz);
	return (gbuf);
}

MOSAPI const char * MOSCConv
mos_path_build(const char *pathp, uint32_t start, uint32_t end, char *pbuf, uint32_t bsize) {
	const char *comps[MOS_PATH_COMPONENTS_MAX];
	char path[MOS_PATH_MAX];
	uint32_t cnt;
	int lastdir;
	uint32_t i;
	char *p;
	int err;

	if (mos_strlcpy(path, pathp, sizeof (path)) >= sizeof (path)) {
		pbuf[0] = '\0';
		return (NULL);
	}

	cnt = MOS_PATH_COMPONENTS_MAX;
	err = _parse_path(path, comps, &cnt, &lastdir);
	if (err != 0) {
		pbuf[0] = '\0';
		return (NULL);
	}

	if (end >= cnt)
		end = cnt - 1;

	for (i = start, p = pbuf; i <= end; i++) {
		if (((i != 0 && i + 1 <= end) || (i == 0 && !issepstr(comps[i]))) &&
		  end != 0) {
#if defined (Windows)
			if (i == 0 && isspecial(comps[i]))
				p += mos_snprintf(p, bsize - (p - pbuf), "%s\\", comps[i]);
			else
				p += mos_snprintf(p, bsize - (p - pbuf), "%s/", comps[i]);
#else
			p += mos_snprintf(p, bsize - (p - pbuf), "%s/", comps[i]);
#endif
		} else {
			p += mos_strlcpy(p, comps[i], bsize - (p - pbuf));
		}
		if (p >= pbuf + bsize) {
			pbuf[0] = '\0';
			return (NULL);
		}
	}

	return (pbuf);
}

#ifndef _KERNEL
/*
 * A simple (not fast, but effective) mkdir, that creates any missing
 * directories without using chdir().
 */
MOSAPI int MOSCConv
mos_path_mkdir(const char *pathp, uint16_t mode) {
	const char *comps[MOS_PATH_COMPONENTS_MAX];
	char npath[MOS_PATH_MAX];
	char path[MOS_PATH_MAX];
	struct stat st;
	const char *c;
	uint32_t cnt;
	int lastdir;
	uint32_t i;
	char *p;
	int err;

	if (mos_strlcpy(path, pathp, sizeof (path)) >= sizeof (path))
		return (MOSN_NOSPC);

	cnt = MOS_PATH_COMPONENTS_MAX;
	err = _parse_path(path, comps, &cnt, &lastdir);
	if (err != 0)
		return (err);

	/*
	 * By default, skip the first comp in the for() assuming
	 * we are absolute or special.
	 */
	i = 1;

	if (isspecial(comps[0])) {
		mos_strlcpy(npath, comps[0], sizeof (npath));
		/*
		 * If npath now contains a "special" component, we need to make sure it
		 * ends with a "/".  Otherwise it would be possible to end up with paths
		 * like "C:Program Files/..." instead of "C:/Program Files/...".
		 */
		if (mos_strlen(npath) > 0) {
			c = npath + mos_strlen(npath) - 1;
			if (!issepstr(c))
				mos_strlcat(npath, "/", sizeof (npath));
		}
	} else if (issep(comps[0][0])) {
		mos_snprintf(npath, sizeof (npath), "/");
	} else {
		err = mos_getcwd(npath, sizeof (npath));
		if (err != 0)
			return (err);
		mos_strlcat(npath, "/", sizeof (npath));

		/*
		 * Relative path, so start with the first comp.
		 */
		i = 0;
	}

	p = npath + mos_strlen(npath);

	/* i was set above depending on if we are absolute or relative */
	for (; i < cnt; i++) {
		p += mos_strlcpy(p, comps[i], sizeof (npath) - (p - &(npath[0])));
		if ((unsigned)(p - &(npath[0])) >= sizeof (npath))
			return (MOSN_NOSPC);

		err = stat(npath, &st);
		/* skip existing component */
		if (err == 0)
			goto next;

		if (errno != ENOENT)
			return (MOSN_ERR);

		err = mkdir(npath, (mode_t)mode);
		if (err != 0) {
			if (errno != EEXIST)
				return (MOSN_ERR);
		}

next:

		p += mos_strlcpy(p, "/", sizeof (npath) - (p - &(npath[0])));
		if ((unsigned)(p - &(npath[0])) >= sizeof (npath))
			return (MOSN_NOSPC);
	}

	return (0);
}
#endif /* _KERNEL */

/*
 * Copyright (c) 1997, 2004 Todd C. Miller <Todd.Miller@courtesan.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
MOSAPI const char * MOSCConv
mos_dirname(const char *path, char *dname, uint32_t dnamesz) {
	const char *endp;
	uint32_t len;

	if (dname == NULL || dnamesz < 2)
		return ("");

	/* Empty or NULL string gets treated as "." */
	if (path == NULL || *path == '\0') {
		dname[0] = '.';
		dname[1] = '\0';
		return (dname);
	}

	/* Strip any trailing slashes */
	endp = path + mos_strlen(path) - 1;
	while (endp > path && issep(*endp))
		endp--;

	/* Find the start of the dir */
	while (endp > path && !issep(*endp))
		endp--;

	/* Either the dir is "/" or there are no slashes */
	if (endp == path) {
		dname[0] = *endp == '/' ? '/' : '.';
		dname[1] = '\0';
		return (dname);
	} else {
		/* Move forward past the separating slashes */
		do {
			endp--;
		} while (endp > path && issep(*endp));
	}

	len = (uint32_t)(endp - path) + 1;
	if (len >= dnamesz)
		return ("");

	memcpy(dname, path, len);
	dname[len] = '\0';

	return (dname);
}

MOSAPI const char * MOSCConv
mos_basename(const char *arg0) {
	const char *binary;

	if (arg0 == NULL || *arg0 == '\0')
		return (".");

	binary = mos_strrchrc(arg0, '/');
#if defined(Windows)
	if (binary == NULL)
		binary = mos_strrchrc(arg0, '\\');
#endif
	if (binary == NULL)
		binary = arg0;
	else
		binary++;

	return (binary);
}

#if !defined(_KERNEL)
MOSAPI const char * MOSCConv
mos_path_getcanonical(const char *src, char *dst, uint32_t dstsz) {

#if !defined(Windows)
	char *tmp;
#endif

	if (src == NULL || dst == NULL || dstsz == 0)
		return (NULL);

#if defined(Windows)

	if (!GetFullPathNameA(src, dstsz, dst, NULL))
		return (NULL);

#else	/* UNIX */
	tmp = realpath(src, NULL);
	
	if (tmp == NULL)
		return (NULL);

	if (mos_strlcpy(dst, tmp, dstsz) >= dstsz)
		return (NULL);
		
	free(tmp);

#endif

	return (dst);
}
#endif
