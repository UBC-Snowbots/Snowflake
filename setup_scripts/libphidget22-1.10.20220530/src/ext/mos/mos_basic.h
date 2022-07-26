#ifndef _MOS_BASE_H_
#define _MOS_BASE_H_

#if !defined(_KERNEL)

#if !defined(MOS_STANDALONE)
#if defined(Windows)
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#endif

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <math.h>
#if !defined(Windows)
#include <errno.h>
#include <unistd.h>
#endif
#endif

#else /* _KERNEL */

#if defined(Windows)
#include <stdio.h>
#include <string.h>
#endif /* Windows */

#endif	/* _KERNEL */

#include "mos_basic_types.h"
#include "mos_macrocompat.h"

/*
 * MOS_PATH_MAX
 * Used in place of MAX_PATH which is 1024 in unix and 260 in Windows.
 * Describes the maximum number of characters that can be used to describe a
 * path to an entity on a filesystem.
 *
 * Windows directory creation calls allow a maximum directory length of 254
 * characters.  This detail has no impact on the fact that the path length is
 * 260, however.
 *
 * See "http://msdn.microsoft.com/en-us/library/aa365247.aspx".
 *
 *
 * MOS_PATH_COMPONENT_MAX
 * Describes the maximum number of characters that a single component of a path
 * can have (a file or directory) can have.  This is 1024 in unix, 254 in
 * Windows.
 *
 *
 * MOS_PATH_COMPONENTS_MAX
 * Describes the maximum number of components it is possible to represent in a
 * path.  If you imagine a path formed of directories with a single-character
 * name, then the value is half of MOS_PATH_MAX.
 */
#if !defined(Windows) || defined(_KERNEL)
#define MOS_PATH_MAX            1024
#define MOS_PATH_COMPONENT_MAX  1024
#define MOS_PATH_COMPONENTS_MAX 512

#else
#define MOS_PATH_MAX			260
#define MOS_PATH_COMPONENT_MAX	254
#define MOS_PATH_COMPONENTS_MAX	130

#endif

#if defined(Windows)
#define MOS_PRINTF_FORMAT _Printf_format_string_
#define MOS_PRINTF_LIKE(a, b)
#define MOS_SCANF_LIKE(a, b)
#else
#define MOS_PRINTF_FORMAT
#ifdef CHK_FORMAT_STRINGS
#define MOS_PRINTF_LIKE(a, b) __attribute__ ((format (printf, a, b)))
#define MOS_SCANF_LIKE(a, b) __attribute__ ((format (scanf, a, b)))
#else
#define MOS_PRINTF_LIKE(a, b)
#define MOS_SCANF_LIKE(a, b)
#endif
#endif /* Windows */

#define mos_bcopy(s, d, l)		memcpy((d), (s), (l))
#define mos_bzero(d, l)			memset((d), 0, (l))

MOSAPI int MOSCConv mos_snprintf(char *, size_t, MOS_PRINTF_FORMAT const char *, ...) MOS_PRINTF_LIKE(3, 4);
MOSAPI int MOSCConv mos_vsnprintf(char *, size_t, const char *, va_list);
MOSAPI int MOSCConv mos_kvprintf(char const *, void (*)(int, void*), void *, int,
  va_list);
MOSAPI int MOSCConv mos_sscanf(const char *, char const *, ...) MOS_SCANF_LIKE(2, 3);
MOSAPI int MOSCConv mos_vsscanf(const char *, char const *, va_list);

MOSAPI int MOSCConv mos_printf(MOS_PRINTF_FORMAT const char *, ...) MOS_PRINTF_LIKE(1, 2);
MOSAPI int MOSCConv mos_printef(MOS_PRINTF_FORMAT const char *, ...) MOS_PRINTF_LIKE(1, 2);
MOSAPI int MOSCConv mos_vprintf(const char *, va_list);
MOSAPI int MOSCConv mos_vprintef(const char *, va_list);

MOSAPI const char * MOSCConv mos_basename(const char *);
MOSAPI const char * MOSCConv mos_dirname(const char *, char *, uint32_t);
MOSAPI int MOSCConv mos_mkdirp(const char *, int);
MOSAPI const char * MOSCConv mos_path_getcanonical(const char *, char *, uint32_t);

MOSAPI int MOSCConv mos_globmatch(const char *, const char *);

/* error printing */
MOSAPI int MOSCConv mos_printe(const char *, ...) MOS_PRINTF_LIKE(1, 2);

#define MOSM_FSTR		(size_t)0x7f6f5f4f	/* freeing a null terminated string */
#define MOSM_SLP		0x01	/* sleeping allocation */
#define MOSM_NSLP		0x02	/* non-sleeping allocation */
#define MOSM_PG			0x04	/* pageable allocation */
#define MOSM_NPG		0x08	/* non-pageable allocation */
#define MOSM_ZERO		0x10	/* zero returned memory */
#define MOSM_DEFAULT	(MOSM_SLP | MOSM_PG)
#define MOSM_SAFE		(MOSM_NSLP | MOSM_NPG)
#define MOSM_TAG_MASK	0xff000000
#define MOSM_TAG_SHIFT	24
#define MOSM_TAG(n)		((n) << MOSM_TAG_SHIFT)

/*
 * MOS allocation tags for internal allocations
 *
 * They start at 56, so they'll be printed with recognizable letters
 * on Windows.
 */
#define MOSM_TAG_MUTEX			MOSM_TAG(56)
#define MOSM_TAG_CV				MOSM_TAG(57)
#define MOSM_TAG_ALLOCINFO		MOSM_TAG(58)

typedef int (*mos_malloc_printer_t)(void *, const char *, ...);
MOSAPI void MOSCConv mos_dump_allocation_set(uint32_t, mos_malloc_printer_t, void *);
MOSAPI void MOSCConv mos_dump_allocations(void);
MOSAPI size_t MOSCConv mos_print_allocated_bytes(void);
MOSAPI size_t MOSCConv mos_allocated_bytes(void);
MOSAPI void MOSCConv mos_mallocset_set(uint32_t);
MOSAPI uint32_t MOSCConv mos_mallocset_get(void);

#define mos_alloc(s, f)	_mos_alloc((s), (f), __FILE__, __func__, __LINE__)
#define mos_malloc(s)	mos_alloc((s), MOSM_DEFAULT)
#define mos_zalloc(s)	mos_alloc((s), MOSM_DEFAULT | MOSM_ZERO)
MOSAPI void * MOSCConv _mos_alloc(size_t, int, const char *, const char *, int);
MOSAPI void * MOSCConv mos__alloc(size_t, int);

#define mos_free(p,s)		_mos_free((p), (s), __FILE__, __func__, __LINE__)
MOSAPI void MOSCConv _mos_free(void *, size_t, const char *, const char *, int);
MOSAPI void MOSCConv mos__free(void *, size_t);

MOSAPI int MOSCConv mos_asprintf(char **, uint32_t *, const char *, ...) MOS_PRINTF_LIKE(3, 4);
MOSAPI int MOSCConv mos_vasprintf(char **, uint32_t *, const char *, va_list);

MOSAPI int MOSCConv mos_memcmp(const void *, const void *, size_t);
MOSAPI const void * MOSCConv mos_memchr(const void *, int, size_t);
MOSAPI const void * MOSCConv mos_memmem(const void *, size_t, const void *, size_t);
MOSAPI int MOSCConv mos_strncasecmp(const char *, const char *, size_t);
MOSAPI int MOSCConv mos_strcasecmp(const char *, const char *);

MOSAPI int MOSCConv mos_strcmp(const char *, const char*);
MOSAPI const char * MOSCConv mos_strchrc(const char *, int);
MOSAPI const char * MOSCConv mos_strrchrc(const char *, int);
MOSAPI char* MOSCConv mos_strstr(char *, const char *);
MOSAPI char* MOSCConv mos_strnstr(char *, const char *, size_t);
MOSAPI const char * MOSCConv mos_strnstrc(const char *, const char *, size_t);
MOSAPI const char * MOSCConv mos_strstrc(const char *, const char *);
MOSAPI char* MOSCConv mos_strcasestr(char *, const char *);
MOSAPI const char * MOSCConv mos_strcasestrc(const char *, const char *);
MOSAPI char* MOSCConv mos_strchr(char *, int);
MOSAPI char* MOSCConv mos_strrchr(char *, int);
MOSAPI char* MOSCConv mos_strrev(char *);
MOSAPI size_t MOSCConv mos_strlen(const char *);
MOSAPI size_t MOSCConv mos_strnlen(const char *, size_t);
#define mos_strdup(s, l) mos__strdup((s), (l), MOSM_DEFAULT, __FILE__, __func__, __LINE__)
MOSAPI char* MOSCConv mos__strdup(const char *, uint32_t *, int, const char *, const char *, int);
MOSAPI int MOSCConv mos_endswith(const char *, const char *);

MOSAPI size_t MOSCConv mos_strlcat(char *, const char *, size_t);
MOSAPI size_t MOSCConv mos_strlcpy(char *, const char *, size_t);
MOSAPI const char * MOSCConv mos_strtrim(const char *, char *, size_t);
MOSAPI char* MOSCConv mos_strncpy(char * __restrict, const char * __restrict, size_t);
MOSAPI int MOSCConv mos_strncmp(const char *, const char *, size_t);
MOSAPI char* MOSCConv mos_strnsep(char **, const char *, ssize_t *);

MOSAPI uint64_t MOSCConv _mos_strtou64(const char *, const char **, int);
MOSAPI int MOSCConv mos_strtou64(const char *, int, uint64_t *);
MOSAPI int64_t MOSCConv _mos_strto64(const char *, const char **, int);
MOSAPI int MOSCConv mos_strto64(const char *, int, int64_t *);
MOSAPI int32_t MOSCConv _mos_strto32(const char *, const char **, int);
MOSAPI int MOSCConv mos_strtou32(const char *, int, uint32_t *);
MOSAPI int MOSCConv mos_strto32(const char *, int, int32_t *);
MOSAPI uint16_t MOSCConv mos_bytes2units(uint64_t, const char **);

MOSAPI int MOSCConv mos_getcwd(char *, size_t);

MOSAPI int MOSCConv mos_isspace(int);
MOSAPI int MOSCConv mos_isascii(int);
MOSAPI int MOSCConv mos_isupper(int);
MOSAPI int MOSCConv mos_islower(int);
MOSAPI int MOSCConv mos_isalpha(int);
MOSAPI int MOSCConv mos_isdigit(int);
MOSAPI int MOSCConv mos_isxdigit(int);
MOSAPI int MOSCConv mos_isprint(int);
MOSAPI int MOSCConv mos_isprint_utf8(int);

MOSAPI int MOSCConv mos_tolower(int);
MOSAPI int MOSCConv mos_toupper(int);
MOSAPI const char * MOSCConv mos_uppercase(char *);

MOSAPI int MOSCConv mos_ffs(int);

MOSAPI const char * MOSCConv mos_getenv(const char *);
MOSAPI int MOSCConv mos_setenv(const char *, const char *, int);
MOSAPI int MOSCConv mos_unsetenv(const char *);

MOSAPI int MOSCConv mos_path_isdot(const char *);
MOSAPI int MOSCConv mos_path_isdotdot(const char *);
MOSAPI int MOSCConv mos_path_isabsolute(const char *);
MOSAPI int MOSCConv mos_path_count(const char *, uint32_t *);
MOSAPI const char * MOSCConv mos_path_get(const char *, uint32_t, char *, uint32_t);
MOSAPI const char * MOSCConv mos_path_build(const char *, uint32_t, uint32_t, char *,
  uint32_t);
MOSAPI int MOSCConv mos_path_mkdir(const char *, uint16_t);

MOSAPI int MOSCConv mos_gunlock(void *);
MOSAPI int MOSCConv mos_glock(void *);

#if !defined(_KERNEL)
MOSAPI int MOSCConv mos_getprocessid(void);
#endif

#if !defined(_KERNEL)
#if defined(Windows)
MOSAPI int MOSCConv mos_mkstemp(char *);
#else
#define mos_mkstemp(a)	mkstemp(a)
#endif /* Windows */
#endif /* !_KERNEL */

#if !defined(MOS_STANDALONE)
#if		defined(Windows)
#if			!defined(_KERNEL)
#if				!defined(MOS_GETOPT)
					__declspec(dllimport) const char *mos_optarg;
					__declspec(dllimport) int mos_optind;
					__declspec(dllimport) int mos_optopt;
					__declspec(dllimport) int mos_opterr;
					__declspec(dllimport) int mos_optreset;
#endif			/* MOS_GETOPT */

				MOSAPI int MOSCConv mos_getopt(int argc, char * const argv[], const char *);

#define			MOS_SRANDOM(s)	srand((unsigned)s)
#define			MOS_RANDOM(s)	rand(s)
#endif /* !_KERNEL */

#else	/* Windows */

#define mos_optarg		optarg
#define mos_optind		optind
#define mos_optopt		optopt
#define mos_opterr		opterr
#define mos_optreset	optreset
#define mos_getopt		getopt

#if !defined(_KERNEL)
#define MOS_SRANDOM(s)	srandom((unsigned)s)
#define MOS_RANDOM(s)	random(s)
#endif

#endif	/* !Windows */
#endif /* MOS_STANDALONE */
#endif /* _MOS_BASE_H_ */
