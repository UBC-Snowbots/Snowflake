/*-
 * Copyright (c) 2008 Damien Bergamini <damien.bergamini@free.fr>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "mos_os.h"
#include "mos_sha1.h"
#include "pkcs5_pbkdf2.h"

/*
 * HMAC-SHA-1 (from RFC 2202).
 */
static void
hmac_sha1(const uint8_t *text, size_t text_len, const uint8_t *key,
  size_t key_len, uint8_t digest[SHA1_DIGEST_LENGTH]) {
	uint8_t k_pad[SHA1_BLOCK_LENGTH];
	uint8_t tk[SHA1_DIGEST_LENGTH];
	SHA1_CTX ctx;
	int i;

	if (key_len > SHA1_BLOCK_LENGTH) {
		mos_SHA1_Init(&ctx);
		mos_SHA1_Update(&ctx, key, (int)key_len);
		mos_SHA1_Final(tk, &ctx);

		key = tk;
		key_len = SHA1_DIGEST_LENGTH;
	}

	memset(k_pad, 0, sizeof(k_pad));
	memcpy(k_pad, key, key_len);
	for (i = 0; i < SHA1_BLOCK_LENGTH; i++)
		k_pad[i] ^= 0x36;

	mos_SHA1_Init(&ctx);
	mos_SHA1_Update(&ctx, k_pad, SHA1_BLOCK_LENGTH);
	mos_SHA1_Update(&ctx, text, (int)text_len);
	mos_SHA1_Final(digest, &ctx);

	memset(k_pad, 0, sizeof k_pad);
	memcpy(k_pad, key, key_len);
	for (i = 0; i < SHA1_BLOCK_LENGTH; i++)
		k_pad[i] ^= 0x5c;
	mos_SHA1_Init(&ctx);

	mos_SHA1_Update(&ctx, k_pad, SHA1_BLOCK_LENGTH);
	mos_SHA1_Update(&ctx, digest, SHA1_DIGEST_LENGTH);
	mos_SHA1_Final(digest, &ctx);
}

/*
 * Password-Based Key Derivation Function 2 (PKCS #5 v2.0).
 * Code based on IEEE Std 802.11-2007, Annex H.4.2.
 */
int
pkcs5_pbkdf2(const char *pass, size_t pass_len, const char *salt,
  size_t salt_len, uint8_t *key, size_t key_len, unsigned int rounds) {
	uint8_t *asalt, obuf[SHA1_DIGEST_LENGTH];
	uint8_t d1[SHA1_DIGEST_LENGTH], d2[SHA1_DIGEST_LENGTH];
	unsigned int i, j;
	unsigned int count;
	size_t r;

	if (rounds < 1 || key_len == 0)
		return (-1);

	if (salt_len == 0 || salt_len > MOS_UINT32_MAX - 4)
		return (-1);

	if ((asalt = mos_malloc(salt_len + 4)) == NULL)
		return (-1);

	memcpy(asalt, salt, salt_len);

	for (count = 1; key_len > 0; count++) {
		asalt[salt_len + 0] = (count >> 24) & 0xff;
		asalt[salt_len + 1] = (count >> 16) & 0xff;
		asalt[salt_len + 2] = (count >> 8) & 0xff;
		asalt[salt_len + 3] = count & 0xff;
		hmac_sha1(asalt, salt_len + 4, (const uint8_t *)pass, pass_len, d1);
		memcpy(obuf, d1, sizeof(obuf));
		for (i = 1; i < rounds; i++) {
			hmac_sha1(d1, sizeof(d1), (const uint8_t *)pass, pass_len, d2);
			memcpy(d1, d2, sizeof(d1));
			for (j = 0; j < sizeof(obuf); j++)
				obuf[j] ^= d1[j];
		}

		r = MOS_MIN(key_len, SHA1_DIGEST_LENGTH);
		memcpy(key, obuf, r);
		key += r;
		key_len -= r;
	}

	memset(asalt, 0, salt_len + 4);
	mos_free(asalt, salt_len + 4);
	memset(d1, 0, sizeof(d1));
	memset(d2, 0, sizeof(d2));
	memset(obuf, 0, sizeof(obuf));

	return 0;
}
