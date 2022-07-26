
#ifndef _MOS_NETOPS_H_
#define _MOS_NETOPS_H_

#include "mos_net.h"
#include "mos_iop.h"

void _mos_netops_init(void);

MOSAPI int MOSCConv mos_netop_gethostname(mosiop_t, char *, size_t);
MOSAPI int MOSCConv mos_netop_getbyname(mosiop_t, const char *, mos_af_t, mos_sockaddr_t *);
MOSAPI int MOSCConv mos_netop_addrmatchesname(mosiop_t, mos_sockaddr_t *, const char *, mos_af_t, int *);
MOSAPI int MOSCConv mos_netop_setsockopt(mos_socket_t *, int, int, void *, socklen_t);

MOSAPI int MOSCConv mos_netop_tcp_openserversocket(mosiop_t, mos_socket_t *, mos_sockaddr_t *);
MOSAPI int MOSCConv mos_netop_tcp_accept(mosiop_t, mos_socket_t *, mos_socket_t *, mos_sockaddr_t *);
MOSAPI int MOSCConv mos_netop_tcp_closeserversocket(mosiop_t, mos_socket_t *);

MOSAPI int MOSCConv mos_netop_tcp_opensocket(mosiop_t, mos_socket_t *, mos_sockaddr_t *);
MOSAPI int MOSCConv mos_netop_tcp_closesocket(mosiop_t, mos_socket_t *);
MOSAPI int MOSCConv mos_netop_tcp_available(mosiop_t, mos_socket_t *, int *);
MOSAPI int MOSCConv mos_netop_tcp_setnonblocking(mosiop_t, mos_socket_t *, int);
MOSAPI int MOSCConv mos_netop_tcp_rpoll(mosiop_t, mos_socket_t *, uint32_t);
MOSAPI int MOSCConv mos_netop_tcp_rpoll2(mosiop_t, mos_socket_t *, mos_socket_t *, int *, uint32_t);
MOSAPI int MOSCConv mos_netop_tcp_read(mosiop_t, mos_socket_t *, void *, size_t *);
MOSAPI int MOSCConv mos_netop_tcp_readfully(mosiop_t, mos_socket_t *, void *, size_t *);
MOSAPI int MOSCConv mos_netop_tcp_write(mosiop_t, mos_socket_t *, const void *, size_t *);
MOSAPI int MOSCConv mos_netop_tcp_writefully(mosiop_t, mos_socket_t *, const void *, size_t);
MOSAPI int MOSCConv mos_netop_getsockname(mosiop_t, mos_socket_t *, mos_sockaddr_t *);
MOSAPI int MOSCConv mos_netop_getpeername(mosiop_t, mos_socket_t *, mos_sockaddr_t *);
MOSAPI int MOSCConv mos_netop_usekeepalive(mosiop_t, mos_socket_t *, uint32_t);
MOSAPI int MOSCConv mos_netop_setrecvtimeout(mosiop_t, mos_socket_t *, uint32_t);
MOSAPI int MOSCConv mos_netop_setsendtimeout(mosiop_t, mos_socket_t *, uint32_t);
MOSAPI int MOSCConv mos_netop_setrecvbufsize(mosiop_t, mos_socket_t *, uint32_t);

MOSAPI int MOSCConv mos_net_readto(mosiop_t, mos_socket_t *, void *, size_t *, int);
MOSAPI int MOSCConv mos_net_readline(mosiop_t, mos_socket_t *, void *, size_t *);
MOSAPI int MOSCConv mos_net_skip(mosiop_t, mos_socket_t *, size_t);

MOSAPI int MOSCConv mos_netop_udp_openserversocket(mosiop_t, mos_socket_t *, mos_sockaddr_t *);
MOSAPI int MOSCConv mos_netop_udp_opensocket(mosiop_t, mos_socket_t *, mos_sockaddr_t *);
MOSAPI int MOSCConv mos_netop_udp_closesocket(mosiop_t, mos_socket_t *);
MOSAPI int MOSCConv mos_netop_udp_send(mosiop_t, mos_socket_t *, const void *, size_t *);
MOSAPI int MOSCConv mos_netop_udp_recv(mosiop_t, mos_socket_t *, void *, size_t *);
MOSAPI int MOSCConv mos_netop_udp_setnonblocking(mosiop_t, mos_socket_t *, int);

#endif /* _MOS_NETOPS_H_ */
