
#ifndef _KERNEL
#include <errno.h>
#else
#include <sys/errno.h>
#endif

#include "mos_error-errno.h"
#include "mos_iop.h"

int
mos_fromerrno(int _errno) {
	int err;

	switch(_errno) {
	case EEXIST:
		err = MOSN_EXIST;
		break;
	case ENOENT:
		err = MOSN_NOENT;
		break;
	case EINTR:
		err = MOSN_INTR;
		break;
	case EPERM:
		err = MOSN_PERM;
		break;
	case EACCES:
		err = MOSN_ACCESS;
		break;
	case EPIPE:
		err = MOSN_PIPE;
		break;
#ifdef ECONNREFUSED
	case ECONNREFUSED:
		err = MOSN_CONNREF;
		break;
#endif
#ifdef ETIMEDOUT
	case ETIMEDOUT:
		err = MOSN_TIMEDOUT;
		break;
#endif
#ifdef ECONNRESET
	case ECONNRESET:
		err = MOSN_CONNRESET;
		break;
#endif
#ifdef ECONNABORTED
	case ECONNABORTED:
		err = MOSN_CONNABORTED;
		break;
#endif
	case EWOULDBLOCK:	/* EAGAIN */
		err = MOSN_AGAIN;
		break;
	default:
		err = MOSN_IO;
	}

	return (err);
}
