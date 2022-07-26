/* MD5.H - header file for MD5C.C
 * $FreeBSD: src/sys/sys/md5.h,v 1.16.8.1 2005/01/31 23:26:56 imp Exp $
 */

/*-
 Copyright (C) 1991-2, RSA Data Security, Inc. Created 1991. All
rights reserved.

License to copy and use this software is granted provided that it
is identified as the "RSA Data Security, Inc. MD5 Message-Digest
Algorithm" in all material mentioning or referencing this software
or this function.

License is also granted to make and use derivative works provided
that such works are identified as "derived from the RSA Data
Security, Inc. MD5 Message-Digest Algorithm" in all material
mentioning or referencing the derived work.

RSA Data Security, Inc. makes no representations concerning either
the merchantability of this software or the suitability of this
software for any particular purpose. It is provided "as is"
without express or implied warranty of any kind.

These notices must be retained in any copies of any part of this
documentation and/or software.
 */

#ifndef _MOS_MD5_H_
#define _MOS_MD5_H_

#include "mos_os.h"

#define MD5_DIGEST_LENGTH	16
#define MD5_BLOCK_LENGTH	64

/* MD5 context. */
typedef struct MD5Context {
  uint32_t state[4];	/* state (ABCD) */
  uint32_t count[2];	/* number of bits, modulo 2^64 (lsb first) */
  unsigned char buffer[64];	/* input buffer */
} MD5_CTX;

MOSAPI void MOSCConv mos_MD5Init(MD5_CTX *);
MOSAPI void MOSCConv mos_MD5Update(MD5_CTX *, const unsigned char *, unsigned int);
MOSAPI void MOSCConv mos_MD5Pad(MD5_CTX *);
MOSAPI void MOSCConv mos_MD5Final(unsigned char [MD5_DIGEST_LENGTH], MD5_CTX *);
MOSAPI void MOSCConv mos_MD5End(MD5_CTX *, char *);

#endif /* _MOS_MD5_H_ */
