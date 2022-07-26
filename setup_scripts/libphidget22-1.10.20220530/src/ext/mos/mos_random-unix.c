#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "mos_os.h"
#include "mos_random.h"

struct mosrandom {
	int devrandomfd;
};

int
mosrandom_alloc(mosiop_t iop, const uint8_t *seed, size_t seedlen,
  mosrandom_t **randp) {

	if (seedlen > 0)
		return (MOS_ERROR(iop, MOSN_INVAL,
		  "seedable source not implemented"));

	*randp = mos_malloc(sizeof (*randp));

/*
 * /dev/random is way too slow on Linux. If Linux becomes a supported
 * platform we will need to address this.
 */
#if defined(Linux)
	(*randp)->devrandomfd = open("/dev/urandom", O_RDONLY);
#else
	(*randp)->devrandomfd = open("/dev/random", O_RDONLY);
#endif
	if ((*randp)->devrandomfd == -1) {
		mos_free(*randp, sizeof (*randp));
		return (MOS_ERROR(iop, MOSN_ERR, "failed to open /dev/random: %s",
		  strerror(errno)));
	}

	return (0);
}

int
mosrandom_getbytes(mosrandom_t *this, mosiop_t iop, uint8_t *buf, size_t len) {
	size_t off;
	ssize_t n;

	off = 0;
	
	for (;;) {
		n = read(this->devrandomfd, buf + off, len);
		if (n <= 0)
			return (MOS_ERROR(iop, MOSN_IO, "failed to read"));
		off += n;
		len -= n;
		if (len == 0)
			return (0);
	}

	return (0);
}

void
mosrandom_free(mosrandom_t **randp) {

	if ((*randp)->devrandomfd > 0) {
		close((*randp)->devrandomfd);
		(*randp)->devrandomfd = 0;
	}

	mos_free(*randp, sizeof (*randp));

	*randp = NULL;
}
