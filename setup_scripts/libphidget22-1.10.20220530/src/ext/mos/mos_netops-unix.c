#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>
#include <netdb.h>
#include <fcntl.h>

#if defined(SunOS)
#include <sys/filio.h>
#define USE_GETIPNODEBYNAME
#endif

#if defined(FreeBSD)
#include <sys/filio.h>
#define USE_GETIPNODEBYNAME
#endif

#if defined(Darwin)
#include <sys/ioctl.h>
#define USE_GETIPNODEBYNAME
#endif

#if defined(Linux)
#include <sys/ioctl.h>
#include <sys/stat.h>
#endif

#include "mos/mos_os.h"
#include "mos/mos_netops.h"
#include "mos/mos_assert.h"
#include "mos/mos_error-errno.h"

#define CHECKTCPSOCKET	do {										\
	if (sock == NULL)												\
		return (MOS_ERROR(iop, MOSN_INVALARG, "socket is null"));	\
	if (*sock < 0)									\
		return (MOS_ERROR(iop, MOSN_INVAL, "socket is closed"));	\
} while (0)

void
_mos_netops_init() {
}

int
mos_netop_setsockopt(mos_socket_t *sock, int level, int opt, void *optval, socklen_t optlen) {

		return (setsockopt(*sock, level, opt, optval, optlen));
}

int
mos_netop_getbyname(mosiop_t iop, const char *nm, mos_af_t af, mos_sockaddr_t *addr) {
	struct addrinfo hints;
	struct addrinfo *ai;
	int res;

	if (nm == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "null name specified"));

	memset(&hints, 0, sizeof (hints));
	switch(af) {
	case MOS_AF_INET4:
		hints.ai_family = AF_INET;
		break;
	case MOS_AF_INET6:
		hints.ai_family = AF_INET6;
		break;
	default:
		return (MOS_ERROR(iop, MOSN_NOSUP, "unsupported address family %d", af));
	}

	res = getaddrinfo(nm, NULL, &hints, &ai);
	if (res != 0)
		return (MOS_ERROR(iop, MOSN_ERR, "failed to get address info for %s: %s", nm, gai_strerror(res)));
	addr->sa = *ai->ai_addr;	/* take the first */
	freeaddrinfo(ai);

	return (0);
}

int
mos_netop_addrmatchesname(mosiop_t iop, mos_sockaddr_t *addr, const char *nm, mos_af_t af, int *res) {
	struct addrinfo hints;
	struct addrinfo *walk;
	struct addrinfo *ai;
	int res0;

	*res = 0;

	memset(&hints, 0, sizeof (hints));
	switch(af) {
	case MOS_AF_INET4:
		hints.ai_family = AF_INET;
		break;
	case MOS_AF_INET6:
		hints.ai_family = AF_INET;
		break;
	default:
		return (MOS_ERROR(iop, MOSN_NOSUP, "unsupported address family %d", af));
	}

	res0 = getaddrinfo(nm, NULL, &hints, &ai);
	if (res0 != 0)
		return (MOS_ERROR(iop, MOSN_ERR, "failed to get address info: %s", gai_strerror(res0)));

	for (walk = ai; walk != NULL; walk = walk->ai_next) {
		if (memcmp(&addr->sa, walk->ai_addr, walk->ai_addrlen) == 0) {
			*res = 1;
			break;
		}
	}
	freeaddrinfo(ai);

	return (0);
}


MOSAPI int MOSCConv
mos_netop_getnameinfo(mos_sockaddr_t *addr, char *host, size_t hostn, char *svc, size_t svcn) {
	int err;

	err = getnameinfo(&addr->sa, sizeof (addr->sa), host, (socklen_t)hostn, svc, (socklen_t)svcn, 0);
	switch (err) {
	case 0:
		return (0);
	case EAI_SYSTEM:
		return (mos_fromerrno(errno));
	case EAI_AGAIN:
		return (MOSN_AGAIN);
	case EAI_OVERFLOW:
	case EAI_BADFLAGS:
	case EAI_FAMILY:
	case EAI_NONAME:
		return (MOSN_INVAL);
	case EAI_FAIL:
	default:
		return (MOSN_ERR);
	}
}

int
mos_netop_tcp_opensocket(mosiop_t iop, mos_socket_t *sock, mos_sockaddr_t *addr) {
	int s;
	int err;

	switch(addr->sa.sa_family) {
	case MOS_AF_INET4:
	case MOS_AF_INET6:
		s = socket(addr->sa.sa_family, SOCK_STREAM, IPPROTO_TCP);
		if (s < 0)
			return (MOS_ERROR(iop, mos_fromerrno(errno), "socket() failed"));

		err = connect(s, &addr->sa, sizeof (addr->sa));
		if (err != 0) {
			err = errno;
			close(s);
			return (MOS_ERROR(iop, mos_fromerrno(errno), "failed to connect: %s", strerror(errno)));
		}

		*sock = s;
		return (0);
	default:
		return (MOS_ERROR(iop, MOSN_NOSUP, "address family not supported"));
	}
}

int
mos_netop_tcp_openserversocket(mosiop_t iop, mos_socket_t *sock, mos_sockaddr_t *addr) {
	int s;
	int err;
	int on;

	s = -1;

	switch(addr->sa.sa_family) {
	case MOS_AF_INET4:
	case MOS_AF_INET6:
		s = socket(addr->sa.sa_family, SOCK_STREAM, IPPROTO_TCP);
		if (s < 0)
			return (MOS_ERROR(iop, mos_fromerrno(errno), "socket() failed"));

		on = 1;
		err = setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char *)&on, sizeof (on));
		if (err != 0) {
			err = MOS_ERROR(iop, mos_fromerrno(errno),
			  "failed to set SO_REUSEADDR: %s", strerror(errno));
			goto fail;
		}

		switch(addr->sa.sa_family) {
		case MOS_AF_INET4:
			err = bind(s, &addr->sa, sizeof (addr->s4));
			break;
		case MOS_AF_INET6:
			err = bind(s, &addr->sa, sizeof (addr->s6));
			break;
		}
		if (err != 0) {
			err = MOS_ERROR(iop, mos_fromerrno(errno),
			  "failed to bind socket: %s", strerror(errno));
			goto fail;
		}

		listen(s, SOMAXCONN);
		*sock = s;

		return (0);

	default:
		return (MOS_ERROR(iop, MOSN_NOSUP, "address family not supported"));
	}

fail:

	if (s >= 0)
		close(s);

	return (err);
}

int
mos_netop_tcp_closesocket(mosiop_t iop, mos_socket_t *sock) {

	CHECKTCPSOCKET;

	close(*sock);
	*sock = -1;

	return (0);
}

int
mos_netop_tcp_accept(mosiop_t iop, mos_socket_t *sock, mos_socket_t *s, mos_sockaddr_t *addr) {
	struct sockaddr sa;
	socklen_t alen;
	int cs;

	CHECKTCPSOCKET;

	if (sock == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "server socket is NULL"));

again:

	alen = sizeof (struct sockaddr);
	cs = accept(*sock, &sa, &alen);
	if (cs < 0) {
		switch(errno) {
		case EINTR:
			goto again;
		default:
			return (MOS_ERROR(iop, mos_fromerrno(errno),
			  "failed to accept connection:%s", strerror(errno)));
		}
	}

	*s = cs;
	if (addr != NULL)
		memcpy(&addr->sa, &sa, sizeof (sa));

	return (0);
}

int
mos_netop_tcp_rpoll(mosiop_t iop, mos_socket_t *sock, uint32_t msec) {
	struct timeval timeval;
	fd_set rfds;
	int res;

	FD_ZERO(&rfds);
	FD_SET(*sock, &rfds);
	timeval.tv_sec = msec / 1000;
	timeval.tv_usec = (msec % 1000) * 1000;

	res = select(((int)*sock)+1, &rfds, NULL, NULL, &timeval);
	if (res < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "select() failed:%s",
		  strerror(errno)));
	if (!FD_ISSET(*sock, &rfds))
		return (MOSN_TIMEDOUT);

	return (0);
}

MOSAPI int MOSCConv
mos_netop_tcp_rpoll2(mosiop_t iop, mos_socket_t *s1, mos_socket_t *s2, int *socks, uint32_t msec) {
	struct timeval timeval;
	fd_set rfds;
	int nfds;
	int res;

	FD_ZERO(&rfds);
	if (*s1 != MOS_INVALID_SOCKET)
		FD_SET(*s1, &rfds);
	if (*s2 != MOS_INVALID_SOCKET)
		FD_SET(*s2, &rfds);
	timeval.tv_sec = msec / 1000;
	timeval.tv_usec = (msec % 1000) * 1000;

	nfds = MOS_MAX(*s1, *s2);
	res = select(nfds + 1, &rfds, NULL, NULL, &timeval);
	if (res < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "select()"));

	*socks = 0;
	if (*s1 != MOS_INVALID_SOCKET && FD_ISSET(*s1, &rfds))
		*socks |= 1;
	if (*s2 != MOS_INVALID_SOCKET && FD_ISSET(*s2, &rfds))
		*socks |= 2;

	if (*socks == 0)
		return (MOSN_TIMEDOUT);

	return (0);
}

int
mos_netop_tcp_available(mosiop_t iop, mos_socket_t *sock, int *len) {
	int res;

	CHECKTCPSOCKET;

	res = ioctl(*sock, FIONREAD, len);
	if (res < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno),
		  "ioctl(FIONREAD) failed:%s", strerror(errno)));

	return (0);
}

int
mos_netop_tcp_setnonblocking(mosiop_t iop, mos_socket_t *sock, int on) {
	int res;
	int val;

	CHECKTCPSOCKET;

	val = fcntl(*sock, F_GETFL, 0);
	if (on)
		res = fcntl(*sock, F_SETFL, val | O_NONBLOCK);
	else
		res = fcntl(*sock, F_SETFL, val & ~O_NONBLOCK);
	if (res != 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "fcntl(F_SETFL, O_NONBLOCK) failed:%s", strerror(errno)));
	return (0);
}

int
mos_netop_tcp_read(mosiop_t iop, mos_socket_t *sock, void *v, size_t *len) {
	ssize_t res;

	CHECKTCPSOCKET;

	res = recv(*sock, v, (int)*len, 0);
	if (res < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "recv() failed:%s",
		  strerror(errno)));

	*len = (size_t)res;

	return (0);
}

int
mos_netop_tcp_write(mosiop_t iop, mos_socket_t *sock, const void *v, size_t *len) {
	ssize_t res;

	CHECKTCPSOCKET;

	res = send(*sock, v, (int)*len, 0);
	if (res < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "send() failed%s",
		  strerror(errno)));

	*len = (size_t)res;

	return (0);
}

int
mos_netop_gethostname(mosiop_t iop, char *name, size_t namelen) {

	return (gethostname(name, (int)namelen));
}

int
mos_netop_getsockname(mosiop_t iop, mos_socket_t *sock, mos_sockaddr_t *addr) {
	struct sockaddr sa;
	socklen_t saln;
	int err;

	CHECKTCPSOCKET;

	saln = sizeof (sa);
	err = getsockname(*sock, &sa, &saln);
	if (err != 0)
		return (MOS_ERROR(iop, MOSN_ERR, "getsockname() failed: %s", strerror(errno)));

	addr->sa = sa;

	return (0);
}

int
mos_netop_getpeername(mosiop_t iop, mos_socket_t *sock, mos_sockaddr_t *addr) {
	struct sockaddr sa;
	socklen_t saln;
	int err;

	CHECKTCPSOCKET;

	saln = sizeof (sa);
	err = getpeername(*sock, &sa, &saln);
	if (err != 0)
		return (MOS_ERROR(iop, MOSN_ERR, "getpeername() failed: %s", strerror(errno)));

	addr->sa = sa;

	return (0);
}

int
mos_netop_usekeepalive(mosiop_t iop, mos_socket_t *sock, uint32_t intervalms) {
	int err;
	BOOL on;

	CHECKTCPSOCKET;
	on = 1;

	err = setsockopt(*sock, SOL_SOCKET, SO_KEEPALIVE, (void *)&on, sizeof (on));
	if (err != 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "failed to set SO_KEEPALIVE: %s", strerror(errno)));

	return (0);
}

int
mos_netop_setrecvtimeout(mosiop_t iop, mos_socket_t *sock, uint32_t ms) {
	int d;
	int err;

	CHECKTCPSOCKET;

	d = ms;
	err = setsockopt(*sock, SOL_SOCKET, SO_RCVTIMEO, (void *)&d, sizeof (d));
	if (err != 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "setsockopt(SO_RCVTIMEO): %s", strerror(errno)));

	return (0);
}

MOSAPI int MOSCConv
mos_netop_setrecvbufsize(mosiop_t iop, mos_socket_t *sock, uint32_t sz) {
	int err;

	err = setsockopt(*sock, SOL_SOCKET, SO_RCVBUF, &sz, sizeof (sz));
	if (err != 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "setsockopt(SO_RCVBUF, %u): %s", sz, strerror(errno)));

	return (0);
}

int
mos_netop_setsendtimeout(mosiop_t iop, mos_socket_t *sock, uint32_t ms) {
	int d;
	int err;

	CHECKTCPSOCKET;

	d = ms;
	err = setsockopt(*sock, SOL_SOCKET, SO_SNDTIMEO, (void *)&d, sizeof (d));
	if (err != 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "setsockopt(SO_SNDTIMEO): %s", strerror(errno)));

	return (0);
}

MOSAPI int MOSCConv
mos_netop_udp_openserversocket(mosiop_t iop, mos_socket_t *sock, mos_sockaddr_t *addr) {
	struct sockaddr_in sa;
	socklen_t saln;
	int err;
	int s;

	if (addr->sa.sa_family != AF_INET)
		return (MOS_ERROR(iop, MOSN_NOSUP, "unsupported address family"));

	s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (s == MOS_INVALID_SOCKET)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "socket(SOCK_DGRAM): %s", strerror(errno)));

	err = bind(s, &addr->sa, sizeof (addr->s4));
	if (err != 0) {
		err = mos_fromerrno(errno);
		close(s);
		return (MOS_ERROR(iop, err, "bind()"));
	}

	/*
	 * If the supplied port was 0, the system will assign a random port: find that port.
	 */
	if (addr->s4.sin_port == 0) {
		saln = sizeof (sa);
		err = getsockname(s, (struct sockaddr *)&sa, &saln);
		if (err != 0) {
			close(s);
			return (MOS_ERROR(iop, MOSN_ERR, "getsockname()"));
		}
		addr->s4.sin_port = sa.sin_port;
	}

	*sock = s;

	return (0);
}

MOSAPI int MOSCConv
mos_netop_udp_opensocket(mosiop_t iop, mos_socket_t *sock, mos_sockaddr_t *addr) {
	int s;
	int err;

	switch(addr->sa.sa_family) {
	case MOS_AF_INET4:
	case MOS_AF_INET6:
		s = socket(addr->sa.sa_family, SOCK_DGRAM, IPPROTO_UDP);
		if (s < 0)
			return (MOS_ERROR(iop, mos_fromerrno(errno), "socket() failed"));

		err = connect(s, &addr->sa, sizeof (addr->sa));
		if (err != 0) {
			err = errno;
			close(s);
			return (MOS_ERROR(iop, mos_fromerrno(errno), "failed to connect: %s", strerror(errno)));
		}

		*sock = s;
		return (0);
	default:
		return (MOS_ERROR(iop, MOSN_NOSUP, "address family not supported"));
	}
}

MOSAPI int MOSCConv
mos_netop_udp_closesocket(mosiop_t iop, mos_socket_t *sock) {

	close(*sock);
	*sock = -1;

	return (0);
}

MOSAPI int MOSCConv
mos_netop_udp_send(mosiop_t iop, mos_socket_t *sock, const void *v, size_t *len) {
	ssize_t res;

	res = send(*sock, v, (int)*len, 0);
	if (res < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "send() failed%s", strerror(errno)));

	*len = (size_t)res;

	return (0);
}

MOSAPI int MOSCConv
mos_netop_udp_recv(mosiop_t iop, mos_socket_t *sock, void *v, size_t *len) {
	ssize_t res;

	res = recv(*sock, v, (int)*len, 0);
	if (res < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "recv() failed:%s",
		  strerror(errno)));

	*len = (size_t)res;

	return (0);
}

MOSAPI int MOSCConv
mos_netop_udp_setnonblocking(mosiop_t iop, mos_socket_t *sock, int on) {
	int res;
	int val;

	CHECKTCPSOCKET;

	val = fcntl(*sock, F_GETFL, 0);
	if (on)
		res = fcntl(*sock, F_SETFL, val | O_NONBLOCK);
	else
		res = fcntl(*sock, F_SETFL, val & ~O_NONBLOCK);
	if (res != 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "fcntl(F_SETFL, O_NONBLOCK) failed:%s", strerror(errno)));
	return (0);
}
