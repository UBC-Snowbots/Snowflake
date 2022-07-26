#include "mos_os.h"
#include "mos_net.h"
#include "mos_netops.h"
#include "mos_assert.h"

#ifndef _WINDOWS
#include <netinet/tcp.h> /* TCP_NODELAY */
#endif

MOSAPI int MOSCConv
mos_sockaddr_cmpaddr(const mos_sockaddr_t *a, const mos_sockaddr_t *b) {

	MOS_ASSERT(a->sa.sa_family == AF_INET || a->sa.sa_family == AF_INET6);

	if (a->sa.sa_family != b->sa.sa_family)
		return (a->sa.sa_family - b->sa.sa_family);

	if (a->sa.sa_family == AF_INET)
		return (a->s4.sin_addr.s_addr - b->s4.sin_addr.s_addr);

	return (memcmp(&a->s6.sin6_addr, &b->s6.sin6_addr, sizeof (struct in6_addr)));
}

MOSAPI const char * MOSCConv
mos_getaddrinfo(mos_sockaddr_t *addr, char *ubuf, size_t bufsz) {
	static char _buf[32];
	char tmp[64];
	char *buf;

	buf = ubuf;
	if (buf == NULL) {
		buf = _buf;
		bufsz = sizeof (_buf);
	}

	switch(addr->sa.sa_family) {
	case MOS_AF_INET4:
		mos_snprintf(buf, bufsz, "%s:%u", inet_ntop(AF_INET, &addr->s4.sin_addr, tmp, sizeof (tmp)),
		  ntohs(addr->s4.sin_port));
		break;
	case MOS_AF_INET6:
		mos_snprintf(buf, bufsz, "%s:%u", inet_ntop(AF_INET6, &addr->s4.sin_addr, tmp, sizeof (tmp)),
		  ntohs(addr->s4.sin_port));
		break;
	default:
		mos_snprintf(buf, bufsz, "<unsupported address>");
	}

	return (buf);
}

MOSAPI void MOSCConv
mos_freeaddrlist(mos_sockaddr_list_t *addrlist) {
	mos_sockaddr_list_t *next, *list;

	list = addrlist;
	while (list) {
		next = list->next;
		mos_free(list, sizeof(mos_sockaddr_list_t));
		list = next;
	}
}

MOSAPI const char * MOSCConv
mos_ntoa(mos_sockaddr_t *addr, char *ubuf, size_t bufsz) {
	static char _buf[32];
	char tmp[64];
	char *buf;

	buf = ubuf;
	if (buf == NULL) {
		buf = _buf;
		bufsz = sizeof (_buf);
	}

	switch(addr->sa.sa_family) {
	case MOS_AF_INET4:
		mos_strlcpy(buf, inet_ntop(AF_INET, &addr->s4.sin_addr, tmp, sizeof (tmp)), bufsz);
		break;
	case MOS_AF_INET6:
		mos_strlcpy(buf, inet_ntop(AF_INET6, &addr->s4.sin_addr, tmp, sizeof (tmp)), bufsz);
		break;
	default:
		mos_snprintf(buf, bufsz, "<unsupported address family:%d>", addr->sa.sa_family);
	}

	return (buf);
}

MOSAPI int MOSCConv
mos_net_readto(mosiop_t iop, mos_socket_t *sock, void *vbuf, size_t *len, int rs) {
	size_t count;
	size_t n;
	char *c;
	int err;

	for (count = 0, c = vbuf; count < *len; c++, count++) {
		n = 1;
		err = mos_netop_tcp_read(iop, sock, c, &n);
		if (err != 0)
			return (MOS_ERROR(iop, err, "failed to read byte from socket"));

		/* eof */
		if (n == 0) {
			if (count == 0) {
				*len = 0;
				return (MOSN_EOF);
			}
			break;
		}

		if (*c == rs)
			break;
	}

	*len = count;

	return (0);
}

MOSAPI int MOSCConv
mos_net_readline(mosiop_t iop, mos_socket_t *sock, void *vbuf, size_t *len) {
	int err;

	err = mos_net_readto(iop, sock, vbuf, len, '\n');
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to read to newline"));

	if (*len > 0 && ((char *)vbuf)[(*len) - 1] == '\r')
		(*len)--;

	return (0);
}

MOSAPI int MOSCConv
mos_net_skip(mosiop_t iop, mos_socket_t *sock, size_t count) {
	char tbuf[128];
	size_t r;
	size_t n;
	int err;

	if (count == 0)
		return (0);

	for (n = count; n > 0;) {
		r = (uint32_t)MOS_MIN(n, sizeof (tbuf));
		err = mos_netop_tcp_read(iop, sock, tbuf, &r);
		if (err != 0)
			return (MOS_ERROR(iop, err, "read failed"));
		if (r == 0)
			break;
		n -= r;
	}

	return (0);
}

MOSAPI int MOSCConv
mos_netop_tcp_readfully(mosiop_t iop, mos_socket_t *s, void *vbuf, size_t *len) {
	uint32_t nread;
	size_t n;
	int err;

	for (nread = 0; nread < *len; nread += (uint32_t)n) {
		n = (size_t) (*len - nread);
		err = mos_netop_tcp_read(iop, s, ((uint8_t *)vbuf) + nread, &n);
		if (err != 0)
			return (MOS_ERROR(iop, err, "TCP read failed"));
		if (n == 0)
			break;
	}
	*len = nread;

	return (0);
}

MOSAPI int MOSCConv
mos_netop_tcp_writefully(mosiop_t iop, mos_socket_t *s, const void *vbuf, size_t len) {
	uint32_t nwr;
	size_t n;
	int err;

	for (nwr = 0; nwr < len; nwr += (uint32_t)n) {
		n = len - nwr;
		err = mos_netop_tcp_write(iop, s, ((const uint8_t *)vbuf) + nwr, &n);
		if (err != 0)
			return (MOS_ERROR(iop, err, "TCP write failed"));
		if (n == 0)
			return (MOS_ERROR(iop, MOSN_IO, "stream handled %u bytes", len - nwr));
	}

	return (0);
}

MOSAPI int MOSCConv
mos_netop_setnodelay(mos_socket_t *sock) {
	int on;

	on = 1;

	return (mos_netop_setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (void *)&on, sizeof (on)));
}
