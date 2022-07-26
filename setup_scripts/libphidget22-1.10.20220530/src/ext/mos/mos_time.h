#ifndef _MOS_TIME_H_
#define _MOS_TIME_H_

#if !defined(Darwin) || !defined(_KERNEL)
#include <time.h>
#endif

#include "mos_os.h"
#include "mos_iop.h"

#define MOSTIME_DURATION			0x01
#define MOSTIME_LOCAL				0x02

#define MOSTIMESTAMP_SIZE			8
#define MOSTIMESTAMP_STRING_LEN		21

#if defined(SunOS) && defined(_KERNEL)
typedef hrtime_t	mostime_t;
#else
typedef int64_t		mostime_t;
#endif

#define MOSTIME_MAXYEAR		8191 /* ISO8601 is 0-9999 ... close enough */
#define MOSTIME_MAXDAY		4095 /* for durations */

typedef struct mostimestamp {
	unsigned int		mts_flags:8;		/* absolute/duration*/
	unsigned int		mts_year:13; 		/* 0-8191 */
	unsigned int		mts_month:4; 		/* 1-12 */
	unsigned int		mts_day:12; 		/* 1-31abs / 0-4095dur */
	unsigned int		mts_hour:5;			/* 0-23 */
	unsigned int		mts_minute:6;		/* 0-59 */
	unsigned int		mts_second:6;		/* 0-59 */
	unsigned int		mts_msecond:10;		/* 0-999 */
} mostimestamp_t;

extern const mostimestamp_t mostimestamp_infinity;
extern const mostimestamp_t mostimestamp_zero;
extern const mostimestamp_t mostimestamp_zero_duration;

MOSAPI int MOSCConv mostimestamp_isduration(const mostimestamp_t *);
MOSAPI int MOSCConv mostimestamp_islocal(const mostimestamp_t *);
MOSAPI int MOSCConv mostimestamp_isabsolute(const mostimestamp_t *);

MOSAPI const char * MOSCConv mostimestamp_string(const mostimestamp_t *, char *, size_t);
MOSAPI int MOSCConv mostimestamp_fromstring(mosiop_t, const char *, mostimestamp_t *);

MOSAPI int MOSCConv mostimestamp_validate(const mostimestamp_t *, mosiop_t);

MOSAPI int MOSCConv mostimestamp_localnow(mostimestamp_t *);
MOSAPI int MOSCConv mostimestamp_now(mostimestamp_t *);
MOSAPI mostime_t MOSCConv mos_gettime_usec(void);
MOSAPI int MOSCConv mos_gettzbias(int32_t *);

#if !defined(_KERNEL)
/**
 * usec since epoch
 */
MOSAPI mostime_t MOSCConv mos_getsystime_usec(void);
#endif

MOSAPI int MOSCConv mostimestamp_cmp(const mostimestamp_t *, const mostimestamp_t *);
MOSAPI int MOSCConv mostimestamp_add(mosiop_t, const mostimestamp_t *,
  const mostimestamp_t *, mostimestamp_t *);
MOSAPI int MOSCConv mostimestamp_addseconds(mosiop_t, const mostimestamp_t *, int64_t,
  mostimestamp_t *);
MOSAPI int MOSCConv mostimestamp_timepassed(mosiop_t, const mostimestamp_t *,
  const mostimestamp_t *, int *);

MOSAPI const char * MOSCConv mostimestamp_dayofweek(mostimestamp_t *);
MOSAPI const char * MOSCConv mostimestamp_monthstring(mostimestamp_t *);
MOSAPI int MOSCConv mostimestamp_ndayofweek(mostimestamp_t *);
MOSAPI int MOSCConv mostimestamp_torfc1123date(mostimestamp_t *, char **, uint32_t *);

#if !defined(_KERNEL)
MOSAPI void MOSCConv mos_usleep(mostime_t);
#endif

/*
 * Build everywhere, but FreeBSD and Darwin kernel.
 */
#if defined(FreeBSD) || defined(Darwin)
#if defined(_KERNEL)
#define _MOSTIMESTAMP_FROM_TM_	0
#else
#define _MOSTIMESTAMP_FROM_TM_	1
#endif
#else
#define _MOSTIMESTAMP_FROM_TM_	1
#endif

#if _MOSTIMESTAMP_FROM_TM_ == 1
MOSAPI int MOSCConv mostimestamp_fromtm(mosiop_t, const struct tm *, mostimestamp_t *);
MOSAPI int MOSCConv mostimestamp_totm(mosiop_t, const mostimestamp_t *, struct tm *);
MOSAPI int MOSCConv mostimestamp_toepoch(mosiop_t, const mostimestamp_t *, time_t *);
#endif

#endif /* _MOS_TIME_H_ */
