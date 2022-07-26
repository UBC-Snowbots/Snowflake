/*
 * Base64 encoding/decoding (RFC1341)
 * Copyright (c) 2005, Jouni Malinen <j@w1.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 * See README and COPYING for more details.
 */

#include "mos_os.h"

#include "mos_base64.h"

static const unsigned char base64_table[65] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/**
 * base64_encode - Base64 encode
 * @src: Data to be encoded
 * @len: Length of the data to be encoded
 * @out_len: Pointer to output length variable, or %NULL if not used
 * Returns: Allocated buffer of out_len bytes of encoded data,
 * or %NULL on failure
 *
 * Caller is responsible for freeing the returned buffer. Returned buffer is
 * nul terminated to make it easier to use as a C string.  *out_len is the
 * size to free.
 */
MOSAPI uint8_t * MOSCConv
mos_base64_encode(const uint8_t *src, uint32_t len, uint32_t *out_len) {
	const uint8_t *end, *in;
	uint8_t *out, *pos;
	uint32_t olen;
	int line_len;

	olen = len * 4 / 3 + 4; /* 3-byte blocks to 4-byte */
	olen += olen / 72; /* line feeds */
	olen++; /* nul termination */
	if (olen < len)
		return (NULL); /* integer overflow */
	out = mos_malloc(olen);
	if (out == NULL)
		return (NULL);

	end = src + len;
	in = src;
	pos = out;
	line_len = 0;
	while (end - in >= 3) {
		*pos++ = base64_table[in[0] >> 2];
		*pos++ = base64_table[((in[0] & 0x03) << 4) | (in[1] >> 4)];
		*pos++ = base64_table[((in[1] & 0x0f) << 2) | (in[2] >> 6)];
		*pos++ = base64_table[in[2] & 0x3f];
		in += 3;
		line_len += 4;
		if (line_len >= 72) {
			*pos++ = '\n';
			line_len = 0;
		}
	}

	if (end - in) {
		*pos++ = base64_table[in[0] >> 2];
		if (end - in == 1) {
			*pos++ = base64_table[(in[0] & 0x03) << 4];
			*pos++ = '=';
		} else {
			*pos++ = base64_table[((in[0] & 0x03) << 4) | (in[1] >> 4)];
			*pos++ = base64_table[(in[1] & 0x0f) << 2];
		}
		*pos++ = '=';
		line_len += 4;
	}

	*pos = '\0';
	if (out_len)
		*out_len = olen;
	return (out);
}


/**
 * Base64 decoder. Caller is responsible for freeing the returned buffer.
 *
 * %param src - data to be decoded
 * %param len - length of the data to be decoded
 * %param out_len - pointer to output length variable
 * %return allocated buffer of out_len bytes of decoded data,
 * or <code>NULL</code> on failure
 *
 */
MOSAPI uint8_t * MOSCConv
mos_base64_decode(const uint8_t *src, uint32_t len, uint32_t *out_len) {
	uint8_t dtable[256], *out, *pos;
	uint8_t in[4], block[4], tmp;
	uint32_t i, count, olen;

	memset(dtable, 0x80, 256);
	for (i = 0; i < sizeof(base64_table) - 1; i++)
		dtable[base64_table[i]] = (uint8_t) i;
	dtable['='] = 0;

	count = 0;
	olen = 0;
	for (i = 0; i < len; i++) {
		tmp = dtable[(uint8_t)src[i]];
		if (tmp == 0x80)
			continue;

		in[count] = src[i];
		block[count] = tmp;
		count++;
		if (count == 4) {
			olen += 3;
			count = 0;
		}
	}

	if (olen == 0 || count % 4)
		return (NULL);

	if (in[2] == '=')
		olen -= 2;
	else if (in[3] == '=')
		olen--;

	*out_len = olen;
	pos = out = mos_malloc(olen);
	if (out == NULL)
		return (NULL);

	count = 0;
	for (i = 0; i < len; i++) {
		tmp = dtable[(uint8_t)src[i]];
		if (tmp == 0x80)
			continue;

		in[count] = src[i];
		block[count] = tmp;
		count++;
		if (count == 4) {
			if (olen > 0) {
				*pos++ = (block[0] << 2) | (block[1] >> 4);
				olen--;
			}
			if (olen > 0) {
				*pos++ = (block[1] << 4) | (block[2] >> 2);
				olen--;
			}
			if (olen > 0) {
				*pos++ = (block[2] << 6) | block[3];
				olen--;
			}
			count = 0;
		}
	}

	return (out);
}
