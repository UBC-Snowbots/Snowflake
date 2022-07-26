/*	$OpenBSD: sha1.h,v 1.4 2004/04/28 20:39:35 hshoexer Exp $	*/

/*
 * SHA-1 in C
 * By Steve Reid <steve@edmweb.com>
 * 100% Public Domain
 */

#ifndef _SHA1_H_
#define _SHA1_H_

#include "mos_os.h"

#define	SHA1_BLOCK_LENGTH		64
#define	SHA1_DIGEST_LENGTH		20

typedef unsigned char SHA1_DIGEST[SHA1_DIGEST_LENGTH];

typedef struct {
	uint32_t	state[5];
	uint64_t	count;
	unsigned char	buffer[SHA1_BLOCK_LENGTH];
} SHA1_CTX;

MOSAPI void MOSCConv mos_SHA1_Init(SHA1_CTX * context);
MOSAPI void MOSCConv mos_SHA1_Transform(uint32_t state[5],
  const unsigned char buffer[SHA1_BLOCK_LENGTH]);
MOSAPI void MOSCConv mos_SHA1_Update(SHA1_CTX *context, const unsigned char *data,
  unsigned int len);
MOSAPI void MOSCConv mos_SHA1_Final(unsigned char digest[SHA1_DIGEST_LENGTH],
  SHA1_CTX *context);

#endif /* _SHA1_H_ */
