/*-
 * Copyright (c) 1986, 1988, 1991, 1993
 *	The Regents of the University of California.  All rights reserved.
 * (c) UNIX System Laboratories, Inc.
 * All or some portions of this file are derived from material licensed
 * to the University of California by American Telephone and Telegraph
 * Co. or Unix System Laboratories, Inc. and are reproduced herein with
 * the permission of UNIX System Laboratories, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
 *
 *	@(#)subr_prf.c	8.3 (Berkeley) 1/21/94
 */

/*
 * Some portions copyright ISSci Inc:
 *
 * $@Copyright ISSci_copyright:ISSci:BSD; YEARS 2003-2004
 *
 * Copyright (C) 2003-2004 by Industrial System Sciences, Inc.
 * All Rights Reserved.
 *
 * www.issci.ca:@$
 *
 * $@License ISSci_licenses:ISSci:BSD;
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice(s), this list of conditions and the following disclaimer as
 *    the first lines of this file unmodified other than the possible
 *    addition of one or more copyright notices.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice(s), this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER(S) ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER(S) BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *:@$
 *
 * $Id: snprintf.c,v 1.32 2009/02/19 17:24:19 rhyasonj Exp $
 */

/*
 * This file includes the bare essentials for the snprintf() family of
 * functions, taken from the FreeBSD kernel (subr_prf.c).
 */

#include "mos_os.h"
#include "mos_iop.h"
#include "mos_fmt.h"
#include "mos_time.h"
#include "mos_assert.h"
#include "mos_hexdump.h"
#include "mos_urlencode.h"
#include "mos_base64.h"

/* Max number conversion buffer length: a u_quad_t in base 2, plus NUL byte. */
#define MAXNBUF	(sizeof(intmax_t) * NBBY + 1)

#define hex2ascii(hex)  (hex2ascii_data[hex])

#define FMT_HANDLERS	6

#define FMT_DEFAULT_LEN	4096

static struct {
	mos_fmt_char_t	fh_ch;
	formatter_t		fh_fmt;
} fh[FMT_HANDLERS];

struct snprintf_arg {
	char	*str;
	size_t	remain;
};

static char *ksprintn(char *nbuf, uintmax_t num, int base, int *len);
static void snprintf_func(int ch, void *arg);

char const hex2ascii_data[] = "0123456789abcdefghijklmnopqrstuvwxyz";

static int
fmt_char_valid(mos_fmt_char_t ch) {

	switch (ch) {
	case FMT_A:
	case FMT_a:
	case FMT_B:
	case FMT_C:
	case FMT_E:
	case FMT_e:
	case FMT_F:
	case FMT_f:
	case FMT_G:
	case FMT_g:
	case FMT_H:
	case FMT_I:
	case FMT_J:
	case FMT_K:
	case FMT_k:
	case FMT_L:
	case FMT_M:
	case FMT_m:
	case FMT_O:
	case FMT_P:
	case FMT_Q:
	case FMT_q:
	case FMT_R:
	case FMT_U:
	case FMT_V:
	case FMT_v:
	case FMT_W:
	case FMT_w:
	case FMT_Y:
	case FMT_Z:
		return (1);
	}

	return (0);
}

static int
handle_registered_format(mos_fmt_char_t ch, void *arg, char **pp, size_t *len) {
	size_t deslen;
	int i;

	for (i = 0; i < FMT_HANDLERS; i++)
		if (fh[i].fh_ch == ch)
			break;

	if (i == FMT_HANDLERS)
		return (MOSN_NOENT);

	*len = FMT_DEFAULT_LEN;

again:

	*pp = mos_malloc(*len);

	deslen = *len;
	fh[i].fh_fmt(arg, *pp, &deslen);

	if (deslen > *len) {
		mos_free(*pp, *len);
		*len = deslen + 1; /* deslen describes desired string length only */
		goto again;
	}

	return (0);
}

typedef struct {
	size_t fn_dstbuflen;
	size_t fn_fmtlen;
	char *fn_dst;
	int omitsrcinfo;
} fmt_N_cb_arg_t;

static void
fmt_N_notes_cb(const mos_iop_note_t *note, void *arg, size_t sub) {
	fmt_N_cb_arg_t *fn;
	const char *nt;
	size_t dstlen;
	char *dst;

	fn = arg;

	nt = mos_iop_note_getnote(note);
	dst = fn->fn_dst;
	dst += MOS_MIN(fn->fn_fmtlen, fn->fn_dstbuflen);
	dstlen = fn->fn_dstbuflen - MOS_MIN(fn->fn_fmtlen, fn->fn_dstbuflen);
	dstlen = mos_snprintf(dst, dstlen, "==> %s <==\n", nt);
	fn->fn_fmtlen += dstlen;
}

static void
fmt_N_cb(const mos_notice_t *mn, void *arg, size_t sub) {
	fmt_N_cb_arg_t *fn;
	const char *filenm;
	const char *errstr;
	size_t dstlen;
	char pre[12];
	char *dst;
	size_t i;

	fn = arg;

	MOS_ASSERT(mn != NULL);

	errstr = mos_notice_string(mos_notice_get_notice(mn));

	dst = fn->fn_dst;
	dst += MOS_MIN(fn->fn_fmtlen, fn->fn_dstbuflen);
	dstlen = fn->fn_dstbuflen - MOS_MIN(fn->fn_fmtlen, fn->fn_dstbuflen);

	for (i = 0; i < sub && i < sizeof (pre) - 1; i++)
		pre[i] = '*';
	pre[i] = '\0';

	/*
	 * If we just walked a subiop, add a line showing the two are
	 * related.
	 */
	if (mos_notice_get_subiop(mn) != NULL) {
		fn->fn_fmtlen += mos_snprintf(dst, dstlen, "%s[%p]subiop complete %p\n",
		  pre, mos_notice_get_iop(mn), mos_notice_get_subiop(mn));
		dst = fn->fn_dst + MOS_MIN(fn->fn_fmtlen, fn->fn_dstbuflen);
		dstlen = fn->fn_dstbuflen - MOS_MIN(fn->fn_fmtlen, fn->fn_dstbuflen);
	}

	filenm = mos_basename(mos_notice_get_file(mn));

	if (fn->omitsrcinfo) {
		if (errstr != NULL)
			fn->fn_fmtlen += mos_snprintf(dst, dstlen, "%s\n", mos_notice_get_message(mn));
		else
			fn->fn_fmtlen += mos_snprintf(dst, dstlen, "%s\n", mos_notice_get_message(mn));
	} else {
		if (errstr != NULL)
			fn->fn_fmtlen += mos_snprintf(dst, dstlen,
			  "%s%s+%d %s() : (%s) %s\n", pre, filenm, mos_notice_get_line(mn),
			  mos_notice_get_func(mn), errstr, mos_notice_get_message(mn));
		else
			fn->fn_fmtlen += mos_snprintf(dst, dstlen,
			  "%s%s+%d %s() : (%u) %s\n", pre, filenm, mos_notice_get_line(mn),
			  mos_notice_get_func(mn), mos_notice_get_notice(mn),
			  mos_notice_get_message(mn));
	}
}

static void
fmt_N(mosiop_t iop, char **abufp, size_t *abuflenp, int omitsrcinfo) {
	fmt_N_cb_arg_t fn;

	if (iop == MOS_IOP_IGNORE) {
		*abufp = NULL;
		*abuflenp = 0;
		return;
	}

	memset(&fn, 0, sizeof (fn));

	fn.fn_fmtlen = fn.fn_dstbuflen = 1024; /* initial buffer size to use */
	fn.fn_dst = NULL;
	fn.omitsrcinfo = omitsrcinfo;

	do {
		if (fn.fn_dst != NULL) {
			mos_free(fn.fn_dst, fn.fn_dstbuflen);
			fn.fn_dst = NULL;
		}

		/* allocate buffer */
		fn.fn_dstbuflen = fn.fn_fmtlen; /* reset second time through */
		MOS_ASSERT(fn.fn_dstbuflen > 0);
		fn.fn_dst = mos_malloc(fn.fn_dstbuflen);
		fn.fn_dst[0] = '\0';
		fn.fn_fmtlen = 0;

		/* format buffer */
		mos_iop_walknotes(iop, fmt_N_notes_cb, &fn, 0);
		mos_iop_walknotices(iop, fmt_N_cb, &fn, 0);
	} while (fn.fn_fmtlen > fn.fn_dstbuflen); /* try again if it didn't fit */

	*abufp = fn.fn_dst;
	*abuflenp = fn.fn_dstbuflen;
}

/*
 * Scaled down version of snprintf(3).
 */
MOSAPI int MOSCConv
mos_snprintf(char *str, size_t size, MOS_PRINTF_FORMAT const char *format, ...) {
	int retval;
	va_list ap;

	va_start(ap, format);
	retval = mos_vsnprintf(str, size, format, ap);
	va_end(ap);
	return(retval);
}

/*
 * Scaled down version of vsnprintf(3).
 */
MOSAPI int MOSCConv
mos_vsnprintf(char *str, size_t size, const char *format, va_list ap) {
	struct snprintf_arg info;
	int retval;

	info.str = str;
	info.remain = size;
	retval = mos_kvprintf(format, snprintf_func, &info, 10, ap);
	if (info.remain >= 1)
		*info.str++ = '\0';
	return (retval);
}

static void
snprintf_func(int ch, void *arg) {
	struct snprintf_arg *const info = arg;

	if (info->remain >= 2) {
		*info->str++ = (char)ch;
		info->remain--;
	}
}

/*
 * Put a NUL-terminated ASCII number (base <= 36) in a buffer in reverse
 * order; return an optional length and a pointer to the last character
 * written in the buffer (i.e., the first character of the string).
 * The buffer pointed to by `nbuf' must have length >= MAXNBUF.
 */
static char *
ksprintn(char *nbuf, uintmax_t num, int base, int *lenp) {
	char *p;

	p = nbuf;
	*p = '\0';
	do {
		*++p = hex2ascii(num % base);
	} while (num /= base);
	if (lenp)
		*lenp = (int)(p - nbuf);
	return (p);
}

static int
needsescape(char c) {

	switch (c) {
	case '\\':
	case '%':
	case '\"':
	case '\n':
	case '\r':
	case '\t':
		return (1);
	}
	return (0);
}

static const char *
escape(char c) {

	switch (c) {
	case '\\':
		return ("\\\\");
	case '%':
		return ("%%");
	case '\"':
		return ("\\\"");
	case '\n':
		return ("\\n");
	case '\r':
		return ("\\r");
	case '\t':
		return ("\\t");
	default:
		return (NULL);
	}
}

#ifdef Windows
#define SNPR	_snprintf
#else
#define SNPR	snprintf
#endif

/*
 * Scaled down version of printf(3).
 *
 * Two additional formats:
 *
 * The format %b is supported to decode error registers.
 * Its usage is:
 *
 *	printf("reg=%b\n", regval, "<base><arg>*");
 *
 * where <base> is the output base expressed as a control character, e.g.
 * \10 gives octal; \20 gives hex.  Each arg is a sequence of characters,
 * the first of which gives the bit number to be inspected (origin 1), and
 * the next characters (up to a control character, i.e. a character <= 32),
 * give the name of the register.  Thus:
 *
 *	mos_kvprintf("reg=%b\n", 3, "\10\2BITTWO\1BITONE\n");
 *
 * would produce output:
 *
 *	reg=3<BITTWO,BITONE>
 *
 * XXX:  %D  -- Hexdump, takes pointer and separator string:
 *		("%6D", ptr, ":")   -> XX:XX:XX:XX:XX:XX
 *		("%*D", len, ptr, " " -> XX XX XX XX ...
 */
MOSAPI int MOSCConv
mos_kvprintf(char const *fmt, void (*func)(int, void*), void *arg, int radix,
  va_list ap) {

#define PCHAR(c) do {int cc=(c); if (func) (*func)(cc,arg); else *d++ = (char)cc; retval++; } while (0)
	char nbuf[MAXNBUF];
	char tbuf[80];
	mostimestamp_t *mts;
	mostimestamp_t now;
	uint32_t u;
	char *d;
	const char *p, *percent, *q;
	unsigned char *up;
	int ch, m, n;
	uintmax_t num;
	int base, lflag, tmp, width, ladjust, sharpflag, neg, sign, dot, sharpflagN;
	int cflag, hflag, jflag, tflag, zflag;
	int dwidth;
	char padc;
	int retval = 0;
	size_t abuflen;
	const char *esc;
#ifndef _KERNEL
	double dbl;
#endif

	char *abuf;
	void *v;

#if defined(Windows)
	const wchar_t *ws;
	int escaped;
#endif

	abuf = NULL;
	abuflen = 0;

	num = 0;
	if (!func)
		d = (char *) arg;
	else
		d = NULL;

	if (fmt == NULL)
		fmt = "(fmt null)\n";

	if (radix < 2 || radix > 36)
		radix = 10;

	for (;;) {
		padc = ' ';
		width = 0;
		while ((ch = (unsigned char)*fmt++) != '%') {
			if (ch == '\0')
				return (retval);
			PCHAR(ch);
		}
		percent = fmt - 1;
		lflag = 0; ladjust = 0; sharpflag = 0; neg = 0;
		sign = 0; dot = 0; dwidth = 0;
		cflag = 0; hflag = 0; jflag = 0; tflag = 0; zflag = 0;
reswitch:	switch (ch = (unsigned char)*fmt++) {
		case '.':
			dot = 1;
			goto reswitch;
		case '#':
			sharpflag = 1;
			goto reswitch;
		case '+':
			sign = 1;
			goto reswitch;
		case '-':
			ladjust = 1;
			goto reswitch;
		case '%':
			PCHAR(ch);
			break;
		case '*':
			if (!dot) {
				width = va_arg(ap, int);
				if (width < 0) {
					ladjust = !ladjust;
					width = -width;
				}
			} else {
				dwidth = va_arg(ap, int);
			}
			goto reswitch;
		case '0':
			if (!dot) {
				padc = '0';
				goto reswitch;
			}
		case '1': case '2': case '3': case '4':
		case '5': case '6': case '7': case '8': case '9':
				for (n = 0;; ++fmt) {
					n = n * 10 + ch - '0';
					ch = *fmt;
					if (ch < '0' || ch > '9')
						break;
				}
			if (dot)
				dwidth = n;
			else
				width = n;
			goto reswitch;
		case 'B':
			p = va_arg(ap, char *);
			abuflen = mos_escape_string((const uint8_t *)p, width, NULL, 0) + 1;
			abuf = mos_malloc(abuflen);
			mos_escape_string((const uint8_t *)p, width, (uint8_t *)abuf,
			  abuflen);
			p = abuf;
			goto dostring;
		case 'b':
			num = (unsigned int)va_arg(ap, int);
			p = va_arg(ap, char *);
			for (q = ksprintn(nbuf, num, *p++, NULL); *q;)
				PCHAR(*q--);

			if (num == 0)
				break;

			for (tmp = 0; *p;) {
				n = *p++;
				if (num & (1LL << (n - 1))) {
					PCHAR(tmp ? ',' : '<');
					for (; (n = *p) > ' '; ++p)
						PCHAR(n);
					tmp = 1;
				} else
					for (; *p > ' '; ++p)
						continue;
			}
			if (tmp)
				PCHAR('>');
			break;
		case 'c':
			PCHAR(va_arg(ap, int));
			break;
		case 'D':
			up = va_arg(ap, unsigned char *);
			p = va_arg(ap, char *);
			if (!width)
				width = 16;
			while(width--) {
				PCHAR(hex2ascii(*up >> 4));
				PCHAR(hex2ascii(*up & 0x0f));
				up++;
				if (width)
					for (q=p;*q;q++)
						PCHAR(*q);
			}
			break;
		case 'd':
			goto i;
#ifndef _KERNEL
		case 'e':
			dbl = va_arg(ap, double);
			if (width != 0 && dwidth != 0)
				SNPR(tbuf, sizeof (tbuf), "%*.*e", width, dwidth, dbl);
			else if (width != 0)
				SNPR(tbuf, sizeof (tbuf), "%*e", width, dbl);
			else if (dwidth != 0)
				SNPR(tbuf, sizeof (tbuf), "%.*e", dwidth, dbl);
			else
				SNPR(tbuf, sizeof (tbuf), "%e", dbl);

			for (p = tbuf; *p; p++)
				PCHAR(*p);
			break;
#endif
		case 'E':
			p = va_arg(ap, char *);
			abuf = (char *)mos_base64_encode((const uint8_t *)p, width, &u);
			abuflen = u;
			p = abuf;
			goto dostring;
#ifndef _KERNEL
		case 'f':
			dbl = va_arg(ap, double);
			if (width != 0 && dwidth != 0)
				SNPR(tbuf, sizeof (tbuf), "%*.*f", width, dwidth, dbl);
			else if (width != 0)
				SNPR(tbuf, sizeof (tbuf), "%*f", width, dbl);
			else if (dwidth != 0)
				SNPR(tbuf, sizeof (tbuf), "%.*f", dwidth, dbl);
			else
				SNPR(tbuf, sizeof (tbuf), "%f", dbl);

			for (p = tbuf; *p; p++)
				PCHAR(*p);
			break;
		case 'g':
			dbl = va_arg(ap, double);
			if (width != 0 && dwidth != 0)
				SNPR(tbuf, sizeof (tbuf), "%*.*g", width, dwidth, dbl);
			else if (width != 0)
				SNPR(tbuf, sizeof (tbuf), "%*g", width, dbl);
			else if (dwidth != 0)
				SNPR(tbuf, sizeof (tbuf), "%.*g", dwidth, dbl);
			else
				SNPR(tbuf, sizeof (tbuf), "%g", dbl);

			for (p = tbuf; *p; p++)
				PCHAR(*p);
			break;
#endif
		case 'i':
i:
			base = 10;
			sign = 1;
			goto handle_sign;
		case 'h':
			if (hflag) {
				hflag = 0;
				cflag = 1;
			} else
				hflag = 1;
			goto reswitch;
		case 'j':
			jflag = 1;
			goto reswitch;
		case 'l':
			lflag++;
			goto reswitch;
		case 'n':
			if (jflag)
				*(va_arg(ap, intmax_t *)) = retval;
			else if (lflag == 1)
				*(va_arg(ap, long *)) = retval;
			else if (lflag == 2)
				*(va_arg(ap, long long *)) = retval;
			else if (zflag)
				*(va_arg(ap, size_t *)) = retval;
			else if (hflag)
				*(va_arg(ap, short *)) = (short)retval;
			else if (cflag)
				*(va_arg(ap, char *)) = (char)retval;
			else
				*(va_arg(ap, int *)) = retval;
			break;
		case 'N':
			sharpflagN = sharpflag;
			sharpflag = 0;
			fmt_N(va_arg(ap, mosiop_t), &abuf, &abuflen, sharpflagN);
			p = abuf;
			goto dostring;
		case 'o':
			base = 8;
			goto handle_nosign;
		case 'p':
			base = 16;
			sharpflag = (width == 0);
			sign = 0;
			num = (uintptr_t)va_arg(ap, void *);
			goto number;
		case 'r':
			base = radix;
			if (sign)
				goto handle_sign;
			goto handle_nosign;
		case 'R': /* urlencode */
			p = va_arg(ap, char *);
			if (dwidth == 0)
				dwidth = (int)mos_strlen(p);
			abuf = mos_urlencode(p, dwidth, &u);
			abuflen = u;
			p = abuf;
			dwidth = 0;
			goto dostring;

		case 's':
			p = va_arg(ap, char *);
dostring:
			if (p == NULL)
				p = "(null)";
			if (!dot) {
				if (sharpflag) {
					for (m = 0, n = 0; p[m] != '\0'; m++, n++)
						if (needsescape(p[m]))
							n++;
				} else {
					m = n = (int)mos_strlen(p);
				}
			} else {
				for (m = 0, n = 0; n < dwidth && p[m] != '\0'; m++, n++) {
					if (sharpflag && needsescape(p[m])) {
						n++;
						if (n == dwidth)
							break;
					}
					continue;
				}
			}

			width -= n;

			if (!ladjust && width > 0)
				while (width--)
					PCHAR(padc);
			while (m--) {
				if (sharpflag && needsescape(*p)) {
					esc = escape(*p++);
					PCHAR(esc[0]);
					if (n-- > 0)
						PCHAR(esc[1]);
				} else {
					PCHAR(*p++);
					n--;
				}
			}
			if (ladjust && width > 0)
				while (width--)
					PCHAR(padc);

			if (abuf != NULL) {
				mos_free(abuf, abuflen);
				abuf = NULL;
			}

			break;

#if defined(Windows)
		case 'S':
			ws = va_arg(ap, wchar_t *);
			escaped = 0;
			if (ws == NULL) {
				ws = L"(null)";
				n = 6;
				if (dot && n > dwidth)
					n = dwidth;
			} else {
				for (n = 0; !dot || n < dwidth; n++) {
					if (!ws[n])
						break;
					if (sharpflag && needsescape((char)ws[n]))
						escaped++;
				}
			}

			n += escaped;
			width -= n;

			if (!ladjust && width > 0)
				while (width--)
					PCHAR(padc);
			while (n--) {
				if (*ws > 255)
					PCHAR('?');
				else if (!*ws)
					PCHAR('!');
				if (sharpflag && needsescape((char)*ws)) {
					esc = escape((char)*ws);
					PCHAR(esc[0]);
					if (n-- > 0)
						PCHAR(esc[1]);
				} else {
					PCHAR((char)*ws);
				}
				ws++;
			}
			if (ladjust && width > 0)
				while (width--)
					PCHAR(padc);

			break;
#endif
		case 't':
			tflag = 1;
			goto reswitch;
		case 'T':
			mts = va_arg(ap, mostimestamp_t *);
			if (mts == NULL) {
				if (sharpflag)
					mostimestamp_localnow(&now);
				else
					mostimestamp_now(&now);
				mts = &now;
			}
			if (lflag) {
				mostimestamp_torfc1123date(mts, &abuf, &u);
				abuflen = u;
				p = abuf;
			} else {
				mostimestamp_string(mts, tbuf, sizeof (tbuf));
				p = tbuf;
			}
			goto dostring;
		case 'u':
			base = 10;
			goto handle_nosign;
		case 'x':
		case 'X':
			base = 16;
			goto handle_nosign;
		case 'y':
			base = 16;
			sign = 1;
			goto handle_sign;
		case 'z':
			zflag = 1;
			goto reswitch;
handle_nosign:
			sign = 0;
			if (jflag)
				num = va_arg(ap, uintmax_t);
			else if (tflag)
				num = va_arg(ap, long);	/* XXX ptrdiff_t */
			else if (lflag == 1)
				num = va_arg(ap, unsigned long);
			else if (lflag == 2)
				num = va_arg(ap, unsigned long long);
			else if (zflag)
				num = va_arg(ap, size_t);
			else if (hflag)
				num = (unsigned short)va_arg(ap, int);
			else if (cflag)
				num = (unsigned char)va_arg(ap, int);
			else
				num = va_arg(ap, unsigned int);
			goto number;
handle_sign:
			if (jflag)
				num = va_arg(ap, intmax_t);
			else if (tflag)
				num = va_arg(ap, long);	/* XXX ptrdiff_t */
			else if (lflag == 1)
				num = va_arg(ap, long);
			else if (lflag == 2)
				num = va_arg(ap, long long);
			else if (zflag)
				num = va_arg(ap, size_t);
			else if (hflag)
				num = (short)va_arg(ap, int);
			else if (cflag)
				num = (char)va_arg(ap, int);
			else
				num = va_arg(ap, int);
number:
			if (sign && (intmax_t)num < 0) {
				neg = 1;
				num = -(intmax_t)num;
			}
			p = ksprintn(nbuf, num, base, &tmp);
			if (sharpflag && num != 0) {
				if (base == 8)
					tmp++;
				else if (base == 16)
					tmp += 2;
			}
			if (neg)
				tmp++;

			if (!ladjust && padc != '0' && width
			    && (width -= tmp) > 0)
				while (width--)
					PCHAR(padc);
			if (neg)
				PCHAR('-');
			if (sharpflag && num != 0) {
				if (base == 8) {
					PCHAR('0');
				} else if (base == 16) {
					PCHAR('0');
					PCHAR('x');
				}
			}
			if (!ladjust && width && (width -= tmp) > 0)
				while (width--)
					PCHAR(padc);

			while (*p)
				PCHAR(*p--);

			if (ladjust && width && (width -= tmp) > 0)
				while (width--)
					PCHAR(padc);

			break;
		default:
			v = va_arg(ap, void *);
			if (handle_registered_format(ch, v, &abuf, &abuflen) == 0) {
				p = abuf;
				goto dostring;
			}

			while (percent < fmt)
				PCHAR(*percent++);
			break;
		}
	}
#undef PCHAR
}

MOSAPI int MOSCConv
mos_register_formatter(mosiop_t iop, mos_fmt_char_t ch, formatter_t cb) {
	int avail;
	int i;

	avail = FMT_HANDLERS;

	if (!fmt_char_valid(ch))
		return (MOS_ERROR(iop, MOSN_INVALARG, "invalid format character 0x%x",
		  ch));

	for (i = 0; i < FMT_HANDLERS; i++) {
		if (fh[i].fh_ch == ch)
			return (MOS_ERROR(iop, MOSN_EXIST,
			  "handler already exists for 0x%x", ch));
		if (fh[i].fh_ch == '\0' && avail > i)
			avail = i;
	}

	if (avail == FMT_HANDLERS)
		return (MOS_ERROR(iop, MOSN_NOSPC, "too many registered handlers"));

	fh[avail].fh_ch = (mos_fmt_char_t)ch;
	fh[avail].fh_fmt = cb;

	return (0);
}

MOSAPI int MOSCConv
mos_unregister_formatter(mosiop_t iop, mos_fmt_char_t ch) {
	int i;

	for (i = 0; i < FMT_HANDLERS; i++) {
		if (fh[i].fh_ch == ch) {
			fh[i].fh_ch = '\0';
			fh[i].fh_fmt = NULL;

			return (0);
		}
	}

	return (MOS_ERROR(iop, MOSN_NOENT, "no such handler for 0x%x", ch));
}
