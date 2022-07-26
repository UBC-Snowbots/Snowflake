/*-
 * Copyright (c) 1987, 1991 Regents of the University of California.
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
 *	@(#)endian.h	7.8 (Berkeley) 4/3/91
 * $FreeBSD: src/sys/i386/include/endian.h,v 1.39 2004/04/07 20:46:05 imp Exp $
 */

#ifndef _MOS_ENDIAN_I386_H_
#define	_MOS_ENDIAN_I386_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mos_basic_types.h"

#if !defined(Windows) && !defined(Darwin)
#include "inttypes.h"
#else
#define inline __inline
#endif

/*
 * Define the order of 32-bit words in 64-bit words.
 */
#define	MOS_QUAD_HIGHWORD 1
#define	MOS_QUAD_LOWWORD 0

/*
 * Definitions for byte order, according to byte significance from low
 * address to high.
 */
#define	MOS_LITTLE_ENDIAN	1234	/* LSB first: i386, vax */
#define	MOS_BIG_ENDIAN	4321		/* MSB first: 68000, ibm, net */
#define	MOS_PDP_ENDIAN	3412		/* LSB first in word, MSW first in long */

#define	MOS_BYTE_ORDER	MOS_LITTLE_ENDIAN

#if defined(__INTEL_COMPILER)
#if defined(__cplusplus)
#if __INTEL_COMPILER >= 800
#define __INTEL_COMPILER_with_FreeBSD_endian 1
#endif
#else
#define __INTEL_COMPILER_with_FreeBSD_endian 1
#endif
#endif

#if defined(__GNUC__) || defined(__INTEL_COMPILER_with_FreeBSD_endian)

#define __mos_word_swap_int_var(x) \
__extension__ ({ register uint32_t __X = (x); \
   __asm ("rorl $16, %0" : "+r" (__X)); \
   __X; })

#ifdef __OPTIMIZE__

#define	__mos_word_swap_int_const(x) \
	((((x) & 0xffff0000) >> 16) | \
	 (((x) & 0x0000ffff) << 16))
#define	__mos_word_swap_int(x) (__builtin_constant_p(x) ? \
	__mos_word_swap_int_const(x) : __mos_word_swap_int_var(x))

#else	/* __OPTIMIZE__ */

#define	__mos_word_swap_int(x) __mos_word_swap_int_var(x)

#endif	/* __OPTIMIZE__ */

#if defined(_KERNEL) && (defined(I486_CPU) || defined(I586_CPU) || defined(I686_CPU)) && !defined(I386_CPU)

#define __mos_byte_swap_int_var(x) \
__extension__ ({ register uint32_t __X = (x); \
   __asm ("bswap %0" : "+r" (__X)); \
   __X; })
#else

#define __mos_byte_swap_int_var(x) \
__extension__ ({ register uint32_t __X = (x); \
   __asm ("xchgb %h0, %b0\n\trorl $16, %0\n\txchgb %h0, %b0" \
       : "+q" (__X)); \
   __X; })
#endif

#ifdef __OPTIMIZE__

#define	__mos_byte_swap_int_const(x) \
	((((x) & 0xff000000) >> 24) | \
	 (((x) & 0x00ff0000) >>  8) | \
	 (((x) & 0x0000ff00) <<  8) | \
	 (((x) & 0x000000ff) << 24))
#define	__mos_byte_swap_int(x) (__builtin_constant_p(x) ? \
	__mos_byte_swap_int_const(x) : __mos_byte_swap_int_var(x))

#else	/* __OPTIMIZE__ */

#define	__mos_byte_swap_int(x) __mos_byte_swap_int_var(x)

#endif	/* __OPTIMIZE__ */

#define __mos_byte_swap_word_var(x) \
__extension__ ({ register uint16_t __X = (x); \
   __asm ("xchgb %h0, %b0" : "+q" (__X)); \
   __X; })

#ifdef __OPTIMIZE__

#define	__mos_byte_swap_word_const(x) \
	((((x) & 0xff00) >> 8) | \
	 (((x) & 0x00ff) << 8))

#define	__mos_byte_swap_word(x) (__builtin_constant_p(x) ? \
	__mos_byte_swap_word_const(x) : __mos_byte_swap_word_var(x))

#else	/* __OPTIMIZE__ */

#define	__mos_byte_swap_word(x) __mos_byte_swap_word_var(x)

#endif	/* __OPTIMIZE__ */

static inline uint64_t
__mos_bswap64(uint64_t _x) {

	return ((_x >> 56) | ((_x >> 40) & 0xff00) | ((_x >> 24) & 0xff0000) |
	    ((_x >> 8) & 0xff000000) | ((_x << 8) & ((uint64_t)0xff << 32)) |
	    ((_x << 24) & ((uint64_t)0xff << 40)) |
	    ((_x << 40) & ((uint64_t)0xff << 48)) | ((_x << 56)));
}

static inline uint32_t
__mos_bswap32(uint32_t _x) {

	return (__mos_byte_swap_int(_x));
}

static inline uint16_t
__mos_bswap16(uint16_t _x) {

	return (__mos_byte_swap_word(_x));
}

#else /* !(__GNUC__ || __INTEL_COMPILER_with_FreeBSD_endian) */

/*
 * No optimizations are available for this compiler.  Fall back to
 * non-optimized functions by defining the constant usually used to prevent
 * redefinition.
 */
#define	_BYTEORDER_FUNC_DEFINED

static inline uint64_t
__mos_bswap64(uint64_t _x) {

	return ((_x >> 56) | ((_x >> 40) & 0xff00) | ((_x >> 24) & 0xff0000) |
	    ((_x >> 8) & 0xff000000) | ((_x << 8) & ((uint64_t)0xff << 32)) |
	    ((_x << 24) & ((uint64_t)0xff << 40)) |
	    ((_x << 40) & ((uint64_t)0xff << 48)) | ((_x << 56)));
}

static inline uint32_t
__mos_bswap32(uint32_t _x) {

	return ((_x >> 24) |
	  ((_x >> 8) & 0xff00) |
	  ((_x << 8) & ((uint32_t)0xff << 16)) |
	  ((_x << 24) & ((uint32_t)0xff << 24)));
}

static inline uint16_t
__mos_bswap16(uint16_t _x) {

	return (((_x >> 8) & 0xff) | ((_x << 8) & ((uint16_t)0xff << 8)));
}

#endif /* __GNUC__ || __INTEL_COMPILER_with_FreeBSD_endian */

#ifdef __cplusplus
}
#endif

#endif /* !_MOS_ENDIAN_I386_H_ */
