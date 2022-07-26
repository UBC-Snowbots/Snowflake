/*-
 * Copyright (c) 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * Chris Torek.
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
 */

#include "mos_os.h"
#include "mos_iop.h"

#define MAX10SHIFT			0xffc0000000000000ULL

/*
 * Convert a string to an unsigned quad integer.
 *
 * Ignores `locale' stuff.  Assumes that the upper and lower case
 * alphabets and digits are each contiguous.
 */
MOSAPI uint64_t MOSCConv
_mos_strtou64(const char *nptr, const char **endptr, int base) {
	uint64_t qbase, cutoff;
	int neg, any, cutlim;
	unsigned char c;
	const char *s;
	uint64_t acc;

	for (s = nptr; *s; s++)
		if (!mos_isspace(*s))
			break;

	if (*s == '\0') {
		if (endptr != NULL)
			*endptr = nptr;
		return (0);
	}

	c = *s++;
	if (c == '-') {
		neg = 1;
		c = *s++;
	} else {
		neg = 0;
		if (c == '+')
			c = *s++;
	}

	if ((base == 0 || base == 16) &&
	    c == '0' && (*s == 'x' || *s == 'X')) {
		c = s[1];
		s += 2;
		base = 16;
	}

	if (base == 0)
		base = c == '0' ? 8 : 10;

	qbase = (unsigned)base;
	cutoff = (uint64_t)(MOS_UINT64_MAX / qbase);
	cutlim = (int)(MOS_UINT64_MAX % qbase);

	for (acc = 0, any = 0;; c = *s++) {
		if (!mos_isascii(c))
			break;

		if (mos_isdigit(c))
			c -= '0';
		else if (mos_isalpha(c))
			c -= mos_isupper(c) ? 'A' - 10 : 'a' - 10;
		else
			break;

		if (c >= base)
			break;

		if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim)) {
			any = -1;
		} else {
			any = 1;
			acc *= qbase;
			acc += c;
		}
	}

	/*
	 * May point beyond the end of endptr.
	 */
	if (s > nptr)
		s--;

	if (any < 0) {
		acc = MOS_UINT64_MAX;
		if (*s == '\0')
			s--;
	} else if (neg) {
		acc = (uint64_t)-(int64_t)acc;
	}

	if (endptr != 0)
		*(endptr) = any ? s : nptr;

	return (acc);
}

MOSAPI int MOSCConv
mos_strtou64(const char *nptr, int base, uint64_t *ret) {
	const char *end;
	size_t len;

	len = mos_strlen(nptr);
	if (len == 0)
		return (MOSN_INVAL);

	*ret = _mos_strtou64(nptr, &end, base);

	/*
	 * We must use all of the characters.
	 */
	if (end != nptr + len && end != nptr + len - 1)
		return (MOSN_INVAL);

	if (*ret == 0 || *ret == MOS_UINT64_MAX) {
		if (end != nptr + len)
			return (MOSN_INVAL);
	}

	if (end == nptr + (len - 1)) {
		switch (*end) {
			case 'e':
			case 'E':
				if ((*ret & MAX10SHIFT) != 0)
					return (MOSN_INVAL);
				*ret *= 1024;
			case 'p':
			case 'P':
				if ((*ret & MAX10SHIFT) != 0)
					return (MOSN_INVAL);
				*ret *= 1024;
			case 't':
			case 'T':
				if ((*ret & MAX10SHIFT) != 0)
					return (MOSN_INVAL);
				*ret *= 1024;
			case 'g':
			case 'G':
				if ((*ret & MAX10SHIFT) != 0)
					return (MOSN_INVAL);
				*ret *= 1024;
			case 'm':
			case 'M':
				if ((*ret & MAX10SHIFT) != 0)
					return (MOSN_INVAL);
				*ret *= 1024;
			case 'k':
			case 'K':
				if ((*ret & MAX10SHIFT) != 0)
					return (MOSN_INVAL);
				*ret *= 1024;
			case 'b':
			case 'B':
				break;
			default:
				return (MOSN_INVAL);
		}
	}

	return (0);
}

MOSAPI int64_t MOSCConv
_mos_strto64(const char *nptr, const char **endptr, int base) {
	uint64_t qbase, cutoff;
	int neg, any, cutlim;
	unsigned char c;
	const char *s;
	uint64_t acc;

	for (s = nptr; *s; s++)
		if (!mos_isspace(*s))
			break;

	if (*s == '\0') {
		if (endptr != NULL)
			*endptr = nptr;
		return (0);
	}

	c = *s++;
	if (c == '-') {
		neg = 1;
		c = *s++;
	} else {
		neg = 0;
		if (c == '+')
			c = *s++;
	}

	if ((base == 0 || base == 16) &&
	    c == '0' && (*s == 'x' || *s == 'X')) {
		c = s[1];
		s += 2;
		base = 16;
	}
	if (base == 0)
		base = c == '0' ? 8 : 10;

	/*
	 * Compute the cutoff value between legal numbers and illegal
	 * numbers.  That is the largest legal value, divided by the
	 * base.  An input number that is greater than this value, if
	 * followed by a legal input character, is too big.  One that
	 * is equal to this value may be valid or not; the limit
	 * between valid and invalid numbers is then based on the last
	 * digit.  For instance, if the range for quads is
	 * [-9223372036854775808..9223372036854775807] and the input base
	 * is 10, cutoff will be set to 922337203685477580 and cutlim to
	 * either 7 (neg==0) or 8 (neg==1), meaning that if we have
	 * accumulated a value > 922337203685477580, or equal but the
	 * next digit is > 7 (or 8), the number is too big, and we will
	 * return a range error.
	 *
	 * Set any if any `digits' consumed; make it negative to indicate
	 * overflow.
	 *
	 * We do the two's complement ourselves just to avoid range
	 * warnings; this seems to be the easiest way to not cause
	 * problems with WDK compiler and gcc.
	 */
	qbase = (unsigned)base;
	cutoff = neg ? (uint64_t)~MOS_INT64_MIN + 1 : MOS_INT64_MAX;
	cutlim = (int)(cutoff % qbase);
	cutoff /= qbase;
	for (acc = 0, any = 0;; c = *s++) {
		if (!mos_isascii(c))
			break;

		if (mos_isdigit(c))
			c -= '0';
		else if (mos_isalpha(c))
			c -= mos_isupper(c) ? 'A' - 10 : 'a' - 10;
		else
			break;

		if (c >= base)
			break;

		if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim)) {
			any = -1;
		} else {
			any = 1;
			acc *= qbase;
			acc += c;
		}
	}

	if (s > nptr)
		s--;

	if (any < 0) {
		acc = neg ? (uint64_t)MOS_INT64_MIN : (uint64_t)MOS_INT64_MAX;
		if (*s == '\0')
			s--;
	} else if (neg) {
		acc = (uint64_t)-(int64_t)acc;
	}

	if (endptr != 0)
		*(endptr) = any ? s : nptr;

	return (acc);
}

MOSAPI int MOSCConv
mos_strto64(const char *nptr, int base, int64_t *ret) {
	const char *end;
	uint64_t maxu;
	uint64_t tmp;
	size_t len;
	int neg;

	len = mos_strlen(nptr);
	if (len == 0)
		return (MOSN_INVAL);

	*ret = _mos_strto64(nptr, &end, base);

	/*
	 * We must use all of the characters.
	 */
	if (end != nptr + len && end != nptr + len - 1)
		return (MOSN_INVAL);

	if (*ret == 0 || *ret == MOS_INT64_MAX || *ret == MOS_INT64_MIN) {
		if (end != nptr + len)
			return (MOSN_INVAL);
	}

	if (*ret < 0) {
		neg = 1;
		tmp = (uint64_t)(*ret * -1);
		maxu = (~(uint64_t)MOS_INT64_MIN + 1) >> 10;
	} else {
		neg = 0;
		tmp = (uint64_t)(*ret);
		maxu = MOS_INT64_MAX >> 10;
	}

	if (end == nptr + (len - 1)) {
		switch (*end) {
			case 'e':
			case 'E':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 'p':
			case 'P':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 't':
			case 'T':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 'g':
			case 'G':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 'm':
			case 'M':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 'k':
			case 'K':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 'b':
			case 'B':
				break;
			default:
				return (MOSN_INVAL);
		}
	}

	if (neg)
		*ret = ((int64_t)tmp) * -1;
	else
		*ret = (int64_t)tmp;

	return (0);
}

MOSAPI int32_t MOSCConv
_mos_strto32(const char *nptr, const char **endptr, int base) {
	const char *s;
	int neg, any, cutlim;
	unsigned char c;
	uint32_t cutoff;
	uint32_t acc;

	neg = 0;

	for (s = nptr; *s; s++)
		if (!mos_isspace(*s))
			break;

	if (*s == '\0') {
		if (endptr != NULL)
			*endptr = nptr;
		return (0);
	}

	c = *s++;
	if (c == '-') {
		neg = 1;
		c = *s++;
	} else if (c == '+') {
		c = *s++;
	}

	if ((base == 0 || base == 16) &&
	    c == '0' && (*s == 'x' || *s == 'X')) {
		c = s[1];
		s += 2;
		base = 16;
	}
	if (base == 0)
		base = c == '0' ? 8 : 10;

	/*
	 * Compute the cutoff value between legal numbers and illegal
	 * numbers.  That is the largest legal value, divided by the
	 * base.  An input number that is greater than this value, if
	 * followed by a legal input character, is too big.  One that
	 * is equal to this value may be valid or not; the limit
	 * between valid and invalid numbers is then based on the last
	 * digit.  For instance, if the range for longs is
	 * [-2147483648..2147483647] and the input base is 10,
	 * cutoff will be set to 214748364 and cutlim to either
	 * 7 (neg==0) or 8 (neg==1), meaning that if we have accumulated
	 * a value > 214748364, or equal but the next digit is > 7 (or 8),
	 * the number is too big, and we will return a range error.
	 *
	 * Set any if any `digits' consumed; make it negative to indicate
	 * overflow.
	 */
	cutoff = neg ? (uint32_t)-(int64_t)MOS_INT32_MIN : MOS_INT32_MAX;
	cutlim = cutoff % (uint32_t)base;
	cutoff /= (uint32_t)base;
	for (acc = 0, any = 0;; c = *s++) {
		if (!mos_isascii(c))
			break;

		if (mos_isdigit(c))
			c -= '0';
		else if (mos_isalpha(c))
			c -= mos_isupper(c) ? 'A' - 10 : 'a' - 10;
		else
			break;

		if (c >= base)
			break;

		if (any < 0 || acc > cutoff || (acc == cutoff && c > cutlim)) {
			any = -1;
		} else {
			any = 1;
			acc *= base;
			acc += c;
		}
	}

	if (s > nptr)
		s--;

	if (any < 0) {
		acc = neg ? MOS_INT32_MIN : MOS_INT32_MAX;
		if (*s == '\0')
			s--;
	} else if (neg) {
		acc = (uint32_t)-(int32_t)acc;
	}

	if (endptr != 0)
		*((const char **)endptr) = any ? s : nptr;

	return (acc);
}

MOSAPI int MOSCConv
mos_strtou32(const char *nptr, int base, uint32_t *ret) {
	uint64_t val;
	int err;

	err = mos_strtou64(nptr, base, &val);
	if (err != 0)
		return (err);

	if (val > MOS_UINT32_MAX)
		return (MOSN_INVAL);

	*ret = (uint32_t)val;

	return (0);
}

MOSAPI int MOSCConv
mos_strto32(const char *nptr, int base, int32_t *ret) {
	const char *end;
	uint32_t maxu;
	uint32_t tmp;
	size_t len;
	int neg;

	len = mos_strlen(nptr);
	if (len == 0)
		return (MOSN_INVAL);

	*ret = _mos_strto32(nptr, &end, base);

	/*
	 * We must use all of the characters.
	 */
	if (end != nptr + len && end != nptr + len - 1)
		return (MOSN_INVAL);

	if (*ret == 0 || *ret == MOS_INT32_MAX || *ret == MOS_INT32_MIN) {
		if (end != nptr + len)
			return (MOSN_INVAL);
	}

	if (*ret < 0) {
		neg = 1;
		tmp = (uint32_t)(*ret * -1);
		maxu = (~(uint32_t)MOS_INT32_MIN + 1) >> 10;
	} else {
		neg = 0;
		tmp = (uint32_t)(*ret);
		maxu = MOS_INT32_MAX >> 10;
	}

	if (end == nptr + (len - 1)) {
		switch (*end) {
			case 'e':
			case 'E':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 'p':
			case 'P':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 't':
			case 'T':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 'g':
			case 'G':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 'm':
			case 'M':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 'k':
			case 'K':
				if (tmp > maxu)
					return (MOSN_INVAL);
				tmp *= 1024;
			case 'b':
			case 'B':
				break;
			default:
				return (MOSN_INVAL);
		}
	}

	if (neg)
		*ret = ((int32_t)tmp) * -1;
	else
		*ret = (int32_t)tmp;

	return (0);
}

/*
 * Approximate given quantity in [BKMGTP] units, striving for 4
 * significant digits.  Sets *unitp to the units, and returns a
 * value in the range [0,9999].
 */
MOSAPI uint16_t MOSCConv
mos_bytes2units(uint64_t i, const char **unitp) {
	const char *unit[] = { "", "K", "M", "G", "T", "P", "E", NULL };
	int roundup;
	int u;

	roundup = 0;

	for (u = 0; i > 9999 && unit[u + 1] != NULL; u++) {
		roundup = (i % 1024) > 511;
		i /= 1024;
	}

	if (roundup)
		i++;

	*unitp = unit[u];

	return ((uint16_t)i);
}
