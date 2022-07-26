#include <sys/types.h>
#include <sys/time.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#include "mos_assert.h"
#include "mos_time.h"
#include "mos_os.h"

int
mostimestamp_now(mostimestamp_t *mts) {
	struct timeval tv;
	struct tm tm;
	time_t epoch;
	
	if (gettimeofday(&tv, NULL) != 0)
		/* gettimeofday error */
		return (-1);

	epoch = tv.tv_sec;

	if (gmtime_r(&epoch, &tm) == NULL)
		/* gmtime_r error */
		return (-1);

	mts->mts_flags = 0;
	mts->mts_year = 1900 + tm.tm_year;
	mts->mts_month = tm.tm_mon + 1;
	mts->mts_day = tm.tm_mday;
	mts->mts_hour = tm.tm_hour;
	mts->mts_minute = tm.tm_min;
	mts->mts_second = tm.tm_sec;
	mts->mts_msecond = 0;

	return (0);
}

MOSAPI int MOSCConv
mostimestamp_localnow(mostimestamp_t *mts) {
	struct timeval tv;
	struct tm ptm;
	time_t epoch;

	if (gettimeofday(&tv, NULL) != 0)
		return (-1);

	epoch = tv.tv_sec;
	if (localtime_r(&epoch, &ptm) == NULL)
		return (-1);

	mts->mts_flags = MOSTIME_LOCAL;
	mts->mts_year = 1900 + ptm.tm_year;
	mts->mts_month = ptm.tm_mon + 1;
	mts->mts_day = ptm.tm_mday;
	mts->mts_hour = ptm.tm_hour;
	mts->mts_minute = ptm.tm_min;
	mts->mts_second = ptm.tm_sec;
	mts->mts_msecond = 0;

	return (0);
}

MOSAPI int MOSCConv
mos_gettzbias(int32_t *min) {
	struct tm ltm, utm;
	time_t tm;

	tm = time(NULL);

	localtime_r(&tm, &ltm);
	gmtime_r(&tm, &utm);

	*min = (utm.tm_hour - ltm.tm_hour) * 60;
	*min += utm.tm_min - ltm.tm_min;

	return (0);
}

#if defined(SunOS)
mostime_t
mos_gettime_usec() {

	return (gethrtime() / 1000);
}

mostime_t
mos_getsystime_usec() {

	/*
	 * mos_getsystime_usec() not yet implemented on SunOS
	 */
	MOS_ASSERT(0);
	return(0);
}

#elif defined(Darwin)

#include <mach/mach_time.h>

uint64_t gettime_nsec(void);

uint64_t 
gettime_nsec() {
	static mach_timebase_info_data_t info;

	if (info.denom == 0)
		(void) mach_timebase_info(&info);
	
	return (mach_absolute_time() * info.numer / info.denom);
}

mostime_t
mos_gettime_usec() {

	return (gettime_nsec() / 1000);
}

mostime_t
mos_getsystime_usec() {
	static mostime_t epoch;
	mostime_t now;

	now = mos_gettime_usec();

	if (epoch == 0)
		epoch = now - time(NULL) * 1000000ULL;

	return (now - epoch);
}

#else

mostime_t
mos_gettime_usec() {
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
		return (0);

	return ((ts.tv_sec * 1000000ULL) + (ts.tv_nsec / 1000ULL));
}

mostime_t
mos_getsystime_usec() {
	struct timespec ts;

	if (clock_gettime(CLOCK_REALTIME, &ts) != 0)
		return (0);

	return ((ts.tv_sec * 1000000ULL) + (ts.tv_nsec / 1000ULL));
}

#endif

void
mos_usleep(mostime_t usec) {

	usleep((unsigned int)usec);
}
