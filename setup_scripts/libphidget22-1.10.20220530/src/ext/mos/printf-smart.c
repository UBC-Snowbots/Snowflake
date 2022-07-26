#include "mos_os.h"

#if !defined(_KERNEL)
#include <stdio.h>
#else
#if defined(Windows)
#include "mos_task.h"
#endif
#if defined(FreeBSD)
#include <sys/syslog.h>
#endif
#endif

/*
 * Provide a printf() wrapper for our snprintf(), which understands
 * negative width specifiers and other pretty things.
 */

void _mos_printf_init(void);

void
_mos_printf_init(void) {

	/* prevent buffering stdout in userland programs */
#if defined(Windows) && !defined(_KERNEL)
	setbuf(stdout, NULL);
#endif
}

/***** WINDOWS *****/
#if defined(Windows)

# if defined(_KERNEL)
MOSAPI int MOSCConv
mos_vprintef(const char *fmt, va_list ap) {

	return (mos_vprintf(fmt, ap));
}
# else /* !_KERNEL */

MOSAPI int MOSCConv
mos_vprintef(const char *fmt, va_list ap) {
	va_list ap2;
	size_t len;
	char *tbuf;

	va_copy(ap2, ap);

	len = mos_vsnprintf(NULL, 0, fmt, ap);
	tbuf = mos_malloc(len + 1);

	mos_vsnprintf(tbuf, len + 1, fmt, ap2);

#if defined(Windows) && defined(MOS_USE_OUTPUTDEBUGSTRING)
	OutputDebugStringA(tbuf);
#else
	fprintf(stderr, "%s", tbuf);
#endif

	mos_free(tbuf, len + 1);

	return ((int)len);
}
# endif /* !_KERNEL */

MOSAPI int MOSCConv
mos_vprintf(const char *fmt, va_list ap) {
	va_list ap2;
	int len;
	char *tbuf;
#if defined(_KERNEL)
	static int eol = 1;
	char *next;
	char *cur;
	char *end;
#endif

	va_copy(ap2, ap);

	len = mos_vsnprintf(NULL, 0, fmt, ap);

	tbuf = mos_malloc(len + 1);

	mos_vsnprintf(tbuf, len + 1, fmt, ap2);

#if defined(_KERNEL)
	/* DbgPrint() truncates strings >512 bytes, so split output into lines */

	/* if starting a new line, prepend with thread ID */
	if (eol) {
		DbgPrint("%p ", mos_self());
		eol = 0;
	}
	for (cur = next = tbuf; next != NULL; cur = end + 1) {
		end = mos_strchr(cur, '\n');
		if (end != NULL) {
			*end = '\0';
			next = end + 1;
			if (mos_strlen(next) > 0) {
				DbgPrint("%s\n%p ", cur, mos_self());
			} else {
				DbgPrint("%s\n", cur);
				eol = 1; /* put thread ID on next line */
			}
		} else {
			DbgPrint("%s", cur);
			next = NULL;
		}
	}
#else
	mos_raw_printf("%s", tbuf);
#endif

	mos_free(tbuf, len + 1);

	return (len);
}

/***** UNIX *****/
#else /* !Windows */

int
mos_vprintef(const char *fmt, va_list ap) {
	char sbuf[64];
	va_list ap2;
	int len;
	char *tbuf;

	va_copy(ap2, ap);

	len = mos_vsnprintf(NULL, 0, fmt, ap);

	if (len < (int)sizeof (sbuf)) {
		tbuf = sbuf;
	} else {
		if ((tbuf = mos_alloc(len + 1, MOSM_NSLP)) == NULL)
			return (-1);
	}

	mos_vsnprintf(tbuf, len + 1, fmt, ap2);

#if defined(_KERNEL)
	/* log to system log and console */
#if defined(FreeBSD)
	mos_raw_printf("%s\n", tbuf);
#else
	mos_raw_printf("%s", tbuf);
#endif
#else
	fprintf(stderr, "%s", tbuf);
#endif

	if (tbuf != sbuf)
		mos_free(tbuf, len + 1);

	return (len);
}

int
mos_vprintf(const char *fmt, va_list ap) {
	char sbuf[64];
	va_list ap2;
	size_t len;
	char *tbuf;

	va_copy(ap2, ap);

	len = mos_vsnprintf(NULL, 0, fmt, ap);

	if (len < sizeof (sbuf)) {
		tbuf = sbuf;
	} else {
		if ((tbuf = mos_alloc(len + 1, MOSM_NSLP | MOSM_PG)) == NULL)
			return (-1);
	}

	mos_vsnprintf(tbuf, len + 1, fmt, ap2);

#if defined(_KERNEL)
#if defined(FreeBSD)
	log(LOG_WARNING, "%s\n", tbuf);
#else
	/* print to the system log only */
	mos_raw_printf("!%s", tbuf);
#endif
#else
	mos_raw_printf("%s", tbuf);
#endif

	if (tbuf != sbuf)
		mos_free(tbuf, len + 1);

	return ((int)len);
}
#endif /* !Windows */

MOSAPI int MOSCConv
mos_printef(MOS_PRINTF_FORMAT const char *fmt, ...) {
	va_list ap;
	int len;

	va_start(ap, fmt);
	len = mos_vprintef(fmt, ap);
	va_end(ap);

	return (len);
}

MOSAPI int MOSCConv
mos_printf(MOS_PRINTF_FORMAT const char *fmt, ...) {
	va_list ap;
	int len;

	va_start(ap, fmt);
	len = mos_vprintf(fmt, ap);
	va_end(ap);

	return (len);
}
