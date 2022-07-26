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

#ifndef _MOS_BASE64_H_
#define _MOS_BASE64_H_

#include "mos_basic.h"

MOSAPI uint8_t * MOSCConv mos_base64_encode(const uint8_t *, uint32_t, uint32_t *);
MOSAPI uint8_t * MOSCConv mos_base64_decode(const uint8_t *, uint32_t, uint32_t *);

#endif /* _MOS_BASE64_H_ */
