
#ifndef _MOS_NET_H_
#define _MOS_NET_H_

#if defined(Windows)
#include <winsock2.h>
#include <ws2tcpip.h>

typedef SOCKET mos_socket_t;

#if (WINVER < 0x600)
int mos_netop_inet_pton(int, const char *, void *);
const char *mos_netop_inet_ntop(int, const void *, char *, socklen_t);
#define inet_pton mos_netop_inet_pton
#define inet_ntop mos_netop_inet_ntop
#endif
#endif /* Windows */

#include "mos_os.h"

#if !defined(Windows)
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
typedef int mos_socket_t;
#define MOS_INVALID_SOCKET	-1
#else /* Windows */
#define MOS_INVALID_SOCKET	INVALID_SOCKET
#endif

typedef union mos_sockaddr {
	struct sockaddr			sa;
	struct sockaddr_in		s4;
	struct sockaddr_in6		s6;
	struct sockaddr_storage	ss;
} mos_sockaddr_t;

typedef struct mos_sockaddr_list {
	int family;
	mos_sockaddr_t addr;
	struct mos_sockaddr_list *next;
} mos_sockaddr_list_t;

typedef enum {
	MOS_AF_INET4 = AF_INET,
	MOS_AF_INET6 = AF_INET6
} mos_af_t;

MOSAPI const char * MOSCConv mos_getaddrinfo(mos_sockaddr_t *, char *, size_t);
MOSAPI void MOSCConv mos_freeaddrlist(mos_sockaddr_list_t *);
MOSAPI const char * MOSCConv mos_ntoa(mos_sockaddr_t *, char *, size_t);
MOSAPI int MOSCConv mos_sockaddr_cmpaddr(const mos_sockaddr_t *, const mos_sockaddr_t *);
MOSAPI int MOSCConv mos_netop_getnameinfo(mos_sockaddr_t *, char *, size_t, char *, size_t);
MOSAPI int MOSCConv mos_netop_setnodelay(mos_socket_t *);

#endif /* _MOS_NET_H_ */
