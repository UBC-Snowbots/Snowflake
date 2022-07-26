#include "mos_os.h"
#include "mos_hexdump.h"

MOSAPI void MOSCConv
mos_hexdump(const void *start, size_t sz) {
	const uint8_t *cur;
	const uint8_t *end;
	unsigned n;

	cur = start;
	end = cur + sz;

	for (n = 0; cur < end; cur++) {
		if (n % 16 == 0)
			mos_printf("%p  ", cur);
		mos_printf("%02x ", *cur);
		n++;
		if (n % 16 == 0)
			mos_printf("\n");
	}
	mos_printf("\n");
}

MOSAPI void MOSCConv
mos_hexdumpstr(const void *start, size_t sz, char *dst, size_t dstsz) {
	const uint8_t *cur;
	const uint8_t *end;
	unsigned n;
	uint32_t s;

	cur = start;
	end = cur + sz;

	for (n = 0; cur < end; cur++) {
		if (n % 16 == 0) {
			s = mos_snprintf(dst, dstsz, "%p  ", cur);
			if (s > dstsz)
				return;
			dstsz -= s;
			dst += s;
		}
		s = mos_snprintf(dst, dstsz, "%02x ", *cur);
		if (s > dstsz)
			return;
		dstsz -= s;
		dst += s;
		n++;
		if (n % 16 == 0) {
			s = mos_snprintf(dst, dstsz, "\n");
			if (s > dstsz)
				return;
			dstsz -= s;
			dst += s;
		}
	}
}

MOSAPI size_t MOSCConv
mos_data2hex(const uint8_t *data, size_t dlen, uint8_t *hex, size_t hlen) {
	const char hexnibble[] = "0123456789abcdef";
	size_t i;
	size_t j;

	for (i = 0, j = 0; i < dlen && j < hlen; j++) {
		if (j % 2 == 0)
			hex[j] = hexnibble[(uint8_t)data[i] >> 4];
		else
			hex[j] = hexnibble[data[i++] & 0xf];
	}

	return (dlen * 2);
}

MOSAPI size_t MOSCConv
mos_hex2data(const uint8_t *hex, size_t hlen, uint8_t *data, size_t dlen) {
	uint8_t n;
	size_t i;
	size_t j;
	char h;

	for (i = 0, j = 0; i < hlen && j < dlen; i++) {
		h = hex[i];
		if (h >= '0' && h <= '9')
			n = h - '0';
		else if (h >= 'a' && h <= 'f')
			n = h - 'a' + 10;
		else if (h >= 'A' && h <= 'F')
			n = h - 'A' + 10;
		else
			n = 0;
		if (i % 2 == 0)
			data[j] = n;
		else
			data[j++] |= n << 4;
	}

	return (hlen / 2);
}

static int
needsescape(char c) {

	switch (c) {
	case '\\':
	case '\"':
	case '\n':
	case '\r':
	case '\t':
	case '\0':
		return (1);
	}
	return (0);
}

static const char *
escape(char c) {

	switch (c) {
	case '\\':
		return ("\\\\");
	case '\"':
		return ("\\\"");
	case '\n':
		return ("\\n");
	case '\r':
		return ("\\r");
	case '\t':
		return ("\\t");
	case '\0':
		return ("\\0");
	default:
		return (NULL);
	}
}

MOSAPI size_t MOSCConv
mos_escape_string(const uint8_t *data, ssize_t dlen, uint8_t *tbuf,
  ssize_t blen) {
	static const char hexdigs[] = "0123456789ABCDEF";
	const char *esc;
	int wouldprint;
	int bufpos;
	int i;

	wouldprint = 0;
	bufpos = 0;

	for (i = 0; i < dlen; i++) {
		if (data[i] > 31 && data[i] < 128 && !needsescape(data[i])) {
			wouldprint++;
			if (bufpos < blen - 1)
				tbuf[bufpos++] = data[i];
		} else {
			esc = escape(data[i]);
			if (esc != NULL) {
				wouldprint += (int)mos_strlen(esc);
				if (bufpos < blen - 1 && esc[0] != '\0')
					tbuf[bufpos++] = esc[0];
				if (bufpos < blen - 1 && esc[1] != '\0')
					tbuf[bufpos++] = esc[1];
			} else {
				/* hex-escape */
				wouldprint += 4;
				if (bufpos < blen - 1)
					tbuf[bufpos++] = '\\';
				if (bufpos < blen - 1)
					tbuf[bufpos++] = 'x';
				if (bufpos < blen - 1)
					tbuf[bufpos++] = hexdigs[data[i] / 16];
				if (bufpos < blen - 1)
					tbuf[bufpos++] = hexdigs[data[i] % 16];
			}
		}
	}

	if (wouldprint > bufpos && bufpos > 5)
		tbuf[bufpos - 1] = tbuf[bufpos - 2] = tbuf[bufpos - 3] = '.';

	if (bufpos < blen)
		tbuf[bufpos++] = '\0';

	return (wouldprint);
}
