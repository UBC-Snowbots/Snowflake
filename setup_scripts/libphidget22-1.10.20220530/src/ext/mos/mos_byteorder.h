/*-
 * Portions:
 *
 * Copyright (c) 2002 Thomas Moestl <tmm@FreeBSD.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/sys/endian.h,v 1.6 2003/10/15 20:05:57 obrien Exp $
 */

#ifndef _MOS_BYTEORDER_H_
#define _MOS_BYTEORDER_H_

#if defined(_MACHINE) && _MACHINE == i386

#include "mos_endian-i386.h"

#else

/* machine independent bytes swaps */

#define	MOS_LITTLE_ENDIAN	1234	/* LSB first: i386, vax */
#define	MOS_BIG_ENDIAN	4321		/* MSB first: 68000, ibm, net */

static inline uint64_t
__mos_bswap64(uint64_t _x) {

	return ((_x >> 56) | ((_x >> 40) & 0xff00) | ((_x >> 24) & 0xff0000) |
	    ((_x >> 8) & 0xff000000) | ((_x << 8) & ((__uint64_t)0xff << 32)) |
	    ((_x << 24) & ((__uint64_t)0xff << 40)) |
	    ((_x << 40) & ((__uint64_t)0xff << 48)) | ((_x << 56)));
}

static inline uint32_t
__mos_bswap32(uint32_t _x) {

	return ((_x >> 24) | ((_x >> 8) & 0xff00) | ((_x << 8) & 0xff0000) |
	    ((_x << 24) & 0xff000000));
}

static inline uint16_t
__mos_bswap16(uint16_t _x) {

	return ((_x >> 8) | ((_x << 8) & 0xff00));
}

#endif

#ifndef MOS_BYTE_ORDER
#error MOS_BYTE_ORDER must be set in make.inc
#endif

MOSAPI void * MOSCConv mos_revcpy(void *, const void *, size_t);

#include "mos_os.h"

/*
 * General byte order swapping functions.
 */
#define	mos_bswap16(x)	__mos_bswap16(x)
#define	mos_bswap32(x)	__mos_bswap32(x)
#define	mos_bswap64(x)	__mos_bswap64(x)

/*
 * Host to big endian, host to little endian, big endian to host, and little
 * endian to host byte order functions as detailed in byteorder(9).
 */
#if MOS_BYTE_ORDER == MOS_LITTLE_ENDIAN
#define	mos_htobe16(x)	mos_bswap16((x))
#define	mos_htobe32(x)	mos_bswap32((x))
#define	mos_htobe64(x)	mos_bswap64((x))
#define	mos_htole16(x)	((uint16_t)(x))
#define	mos_htole32(x)	((uint32_t)(x))
#define	mos_htole64(x)	((uint64_t)(x))

#define	mos_be16toh(x)	mos_bswap16((x))
#define	mos_be32toh(x)	mos_bswap32((x))
#define	mos_be64toh(x)	mos_bswap64((x))
#define	mos_le16toh(x)	((uint16_t)(x))
#define	mos_le32toh(x)	((uint32_t)(x))
#define	mos_le64toh(x)	((uint64_t)(x))
#else /* MOS_BYTE_ORDER != MOS_LITTLE_ENDIAN */

#define	mos_htobe16(x)	((uint16_t)(x))
#define	mos_htobe32(x)	((uint32_t)(x))
#define	mos_htobe64(x)	((uint64_t)(x))
#define	mos_htole16(x)	mos_bswap16((x))
#define	mos_htole32(x)	mos_bswap32((x))
#define	mos_htole64(x)	mos_bswap64((x))

#define	mos_be16toh(x)	((uint16_t)(x))
#define	mos_be32toh(x)	((uint32_t)(x))
#define	mos_be64toh(x)	((uint64_t)(x))
#define	mos_le16toh(x)	mos_bswap16((x))
#define	mos_le32toh(x)	mos_bswap32((x))
#define	mos_le64toh(x)	mos_bswap64((x))
#endif /* MOS_BYTE_ORDER == MOS_LITTLE_ENDIAN */

/* Alignment-agnostic encode/decode bytestream to/from little/big endian. */

static __inline uint16_t
mos_be16dec(const void *pp) {
	unsigned char const *p = (unsigned char const *)pp;

	return ((p[0] << 8) | p[1]);
}

static __inline uint32_t
mos_be32dec(const void *pp) {
	unsigned char const *p = (unsigned char const *)pp;

	return ((p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]);
}

static __inline uint64_t
mos_be64dec(const void *pp) {
	unsigned char const *p = (unsigned char const *)pp;

	return (((uint64_t)mos_be32dec(p) << 32) | mos_be32dec(p + 4));
}

static __inline uint16_t
mos_le16dec(const void *pp) {
	unsigned char const *p = (unsigned char const *)pp;

	return ((p[1] << 8) | p[0]);
}

static __inline uint32_t
mos_le32dec(const void *pp) {
	unsigned char const *p = (unsigned char const *)pp;

	return ((p[3] << 24) | (p[2] << 16) | (p[1] << 8) | p[0]);
}

static __inline uint64_t
mos_le64dec(const void *pp) {
	unsigned char const *p = (unsigned char const *)pp;

	return (((uint64_t)mos_le32dec(p + 4) << 32) | mos_le32dec(p));
}

static __inline void
mos_be16enc(void *pp, uint16_t u) {
	unsigned char *p = (unsigned char *)pp;

	p[0] = (unsigned char)((u >> 8) & 0xff);
	p[1] = (unsigned char)(u & 0xff);
}

static __inline void
mos_be32enc(void *pp, uint32_t u) {
	unsigned char *p = (unsigned char *)pp;

	p[0] = (unsigned char)((u >> 24) & 0xff);
	p[1] = (unsigned char)((u >> 16) & 0xff);
	p[2] = (unsigned char)((u >> 8) & 0xff);
	p[3] = (unsigned char)(u & 0xff);
}

static __inline void
mos_be64enc(void *pp, uint64_t u) {
	unsigned char *p = (unsigned char *)pp;

	mos_be32enc(p, (uint32_t) (u >> 32));
	mos_be32enc(p + 4, (uint32_t) (u & 0xffffffff));
}

static __inline void
mos_le16enc(void *pp, uint16_t u) {
	unsigned char *p = (unsigned char *)pp;

	p[0] = (unsigned char)(u & 0xff);
	p[1] = (unsigned char)((u >> 8) & 0xff);
}

static __inline void
mos_le32enc(void *pp, uint32_t u) {
	unsigned char *p = (unsigned char *)pp;

	p[0] = (unsigned char)(u & 0xff);
	p[1] = (unsigned char)((u >> 8) & 0xff);
	p[2] = (unsigned char)((u >> 16) & 0xff);
	p[3] = (unsigned char)((u >> 24) & 0xff);
}

static __inline void
mos_le64enc(void *pp, uint64_t u) {
	unsigned char *p = (unsigned char *)pp;

	mos_le32enc(p, (uint32_t) (u & 0xffffffff));
	mos_le32enc(p + 4, (uint32_t) (u >> 32));
}

#endif /* _MOS_BYTEORDER_H_ */
