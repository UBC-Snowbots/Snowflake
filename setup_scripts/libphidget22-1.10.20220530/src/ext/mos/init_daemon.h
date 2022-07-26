#ifndef _INIT_DAEMON_H_
#define _INIT_DAEMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#define NOCHDIR			0x01
#define NOUMASK			0x02
#define NOCLOSEFD		0x04
#define NOSIGPIPEIGN	0x08

int init_daemon(int);

#ifdef __cplusplus
}
#endif

#endif /* _INIT_DAEMON_H_ */
