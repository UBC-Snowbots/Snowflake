/*-
 * Copyright (c) 1997-2009 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Luke Mewburn.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Scott Aaron Bamford.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mos_os.h"
#include "mos_urldecode.h"

/*
 * Decode %xx escapes in given string, `in-place'.  The new, possibly
 * shorter, length is returned.  The buffer is never assumed to be
 * NUL- terminated, and the result should not be assumed to be
 * terminated either.
 */
MOSAPI uint32_t MOSCConv
mos_urldecode(void *url, uint32_t max) {
	unsigned char *p, *q;

	if (max == 0 || url == NULL)
		return (0);

	p = q = (unsigned char *)url;

#define	HEXTOINT(x) (x - (mos_isdigit(x) ? '0' : (mos_islower(x) ? 'a' : 'A') - 10))

	while (*p && max-- > 0) {
		if (*p == '+') {
			*q++ = ' ';
			p++;
		} else if (max >= 2 && p[0] == '%' &&
		  p[1] && mos_isxdigit((unsigned char)p[1]) &&
		  p[2] && mos_isxdigit((unsigned char)p[2])) {
			*q++ = HEXTOINT(p[1]) * 16 + HEXTOINT(p[2]);
			p+=3;
			max -= 2;
		} else {
			*q++ = *p++;
		}
	}

	return ((int)(q - (unsigned char *)url));
}
