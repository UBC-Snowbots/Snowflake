/*-
 * Copyright (c) 2003-2007 Tim Kientzle
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR(S) ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR(S) BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mos_os.h"
#include "mos_urlencode.h"

MOSAPI char* MOSCConv
mos_urlencode(const char *in, uint32_t insz, uint32_t *outszp) {
	const char *s;
	int out_len;
	uint32_t n;
	char *out;
	char *d;

	out_len = 0;

	for (s = in, n = 0; n < insz; s++, n++) {
		if (*s < 33 || *s > 126 || *s == '%' || *s == '=')
			out_len += 3;
		else
			out_len++;
	}

	*outszp = out_len + 1;
	out = mos_malloc(*outszp);

	for (n = 0, s = in, d = out; n < insz; s++, n++) {
		/* encode any non-printable ASCII character or '%' or '=' */
		if (*s < 33 || *s > 126 || *s == '%' || *s == '=') {
			/* URL encoding is '%' followed by two hex digits */
			*d++ = '%';
			*d++ = "0123456789ABCDEF"[0x0f & (*s >> 4)];
			*d++ = "0123456789ABCDEF"[0x0f & *s];
		} else {
			*d++ = *s;
		}
	}
	*d = '\0';
	return (out);
}

static int
ishex(char c) {

	return ((mos_toupper(c) >= 'A' && mos_toupper(c) <= 'F') || (c >= '0' && c <= '9'));
}

MOSAPI int MOSCConv
mos_isurlencoded(const char *in, uint32_t insz) {
	const char *s;
	uint32_t n;

	s = in;
	for (n = 0; n < insz; s++, n++) {
		if (*s < 33 || *s > 126 || *s == '%' || *s == '=') {
			/* recognize urlencoded characters */
			if (*s == '%' && ishex(s[1]) && ishex(s[2])) {
				s += 2;
				n += 2;
				continue;
			}
			return (0);
		}
	}

	return (1);
}
