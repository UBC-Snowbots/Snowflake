#include "mos_os.h"
#include "mos_readline.h"

MOSAPI int MOSCConv
mos_readline(mosiop_t iop, mos_readfunc_t *readfunc, void *readarg,
  char **obuf, uint32_t *olen, uint32_t *osz) {
	uint32_t len, sz, n;
	int got_cr;
	char *nbuf;
	char *rbuf;
	uint8_t c;
	int err;

	got_cr = 0;
	len = 0;

	sz = 128;	/* starting size should be plenty for most requests */
	rbuf = mos_malloc(sz);
	if (rbuf == NULL)
		return (MOS_ERROR(iop, MOSN_MEM, "failed allocating buffer"));

	for (;;) {
		n = 1;
		err = readfunc(iop, readarg, &c, &n);
		if (err != 0) {
			mos_free(rbuf, sz);
			return (MOS_ERROR(iop, err, "failed to read"));
		}
		if (n == 0)
			break;

		if (len >= (sz - 1)) {
			nbuf = mos_malloc(sz * 2);
			memcpy(nbuf, rbuf, sz);
			mos_free(rbuf, sz);
			sz *= 2;
			rbuf = nbuf;
		}

		rbuf[len++] = c;
		if (c == '\r') {
			got_cr = 1;
			continue;
		} else if (c == '\n') {
			/*
			 * HTTP/1.1 spec says to ignore CR and treat
			 * LF as the real line terminator.  even though
			 * the same spec defines CRLF as the line
			 * terminator, it is recommended in section 19.3
			 * to do the LF trick for tolerance.
			 */
			if (got_cr)
				len -= 2;
			else
				len -= 1;
			break;
		}
	}
	rbuf[len] = '\0';

	if (len == 0) {
		mos_free(rbuf, sz);
		*obuf = NULL;
	} else {
		*obuf = rbuf;
	}

	*olen = len;
	*osz = sz;

	return (0);
}
