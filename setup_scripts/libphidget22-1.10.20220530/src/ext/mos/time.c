#include "mos_os.h"
#include "mos_time.h"
#include "mos_assert.h"

typedef enum mos_tsunit {
	MTS_YEAR		= 0x001,
	MTS_MONTH		= 0x002,
	MTS_WEEK		= 0x004,
	MTS_DAY			= 0x008,
	MTS_HOUR		= 0x010,
	MTS_MINUTE		= 0x020,
	MTS_SECOND		= 0x040,
	MTS_MSECOND		= 0x080
} mos_tsunit_t;

/* number of days in each month (0-indexed) in non-leap years */
static const unsigned days_per_month_nl[] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

const mostimestamp_t mostimestamp_infinity = {
	0, MOSTIME_MAXYEAR, 12, 31, 23, 59, 59, 999
};

const mostimestamp_t mostimestamp_zero = {
	0, 0, 0, 0, 0, 0, 0, 0
};

const mostimestamp_t mostimestamp_zero_duration = {
	MOSTIME_DURATION, 0, 0, 0, 0, 0, 0, 0
};

static const char *mostimestamp_months[] = {
	"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

static const char *mostimestamp_days[] = {
	"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"
};

static int
is_leap_year(unsigned int year) {

	if (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))
		return (1);

	return (0);
}

static unsigned
days_per_month(unsigned int year, unsigned int month) {

	if (month > 12 || month < 1)
		return (0);

	if (month == 2 && is_leap_year(year))
		return (29);

	return (days_per_month_nl[month - 1]);
}

/*
 * Adds a, b, and any carry-forward amount, indicating how much to
 * carry if the result is greater than mod.
 */
static int32_t
addmodcarry(uint64_t a, uint64_t b, uint64_t mod, uint64_t *carryp) {
	int64_t n;

	MOS_ASSERT(mod < MOS_UINT32_MAX);

	n = a + b + *carryp;

	if (mod == 0) {
		MOS_ASSERT(n <= MOS_UINT32_MAX);
		return ((uint32_t)n);
	}

	*carryp = n / mod;

	return ((uint32_t)(n % mod));
}


/*
 * Adds the specified unit of time to the given timestamp.
 */
static int
addunit(mosiop_t iop, mostimestamp_t *mts, mos_tsunit_t unit, uint64_t val) {
	uint64_t carry;
	uint64_t wval;
	unsigned dpm;	/* days per month */
	int month;
	int year;

	carry = 0;

	wval = val;

	/*
	 * Add the specified unit, and fall-through to the next unit if there
	 * is any carry. Special cases are the week, which is simply treated
	 * as 7 days and falls-through to days, and days themselves, which
	 * never fall-through, but recurse into addunit() for each month worth
	 * of days encountered.
	 *
	 * In the case of a duration, we do not reduce days to months and years
	 * as months are variable, and we need to walk over each individual month
	 * to account for leap years etc.  We only support 4095 days, which
	 * should be a reasonable limit.
	 */
	switch(unit) {
	case MTS_MSECOND:
		mts->mts_msecond = addmodcarry(mts->mts_msecond, wval, 1000, &carry);
		if (carry == 0)
			return (0);
		wval = 0;
	case MTS_SECOND:
		mts->mts_second = addmodcarry(mts->mts_second, wval, 60, &carry);
		if (carry == 0)
			return (0);
		wval = 0;
	case MTS_MINUTE:
		mts->mts_minute = addmodcarry(mts->mts_minute, wval, 60, &carry);
		if (carry == 0)
			return (0);
		wval = 0;
	case MTS_HOUR:
		mts->mts_hour = addmodcarry(mts->mts_hour, wval, 24, &carry);
		if (carry == 0)
			return (0);
		wval = 0;
	case MTS_WEEK:
		if (unit == MTS_WEEK)
			wval *= 7;	/* fall through to days */
	case MTS_DAY:
		wval += mts->mts_day + carry;

		if (wval > MOSTIME_MAXDAY)
			return (MOS_ERROR(iop, MOSN_INVAL, "bad days: %llu > %d",
			  wval, MOSTIME_MAXDAY));

		/*
		 * If this is a duration, just set the days and we are done.
		 */
		if (mostimestamp_isduration(mts)) {
			mts->mts_day = (uint32_t)wval;
			return (0);
		}

		/*
		 * Start the days_per_month walk based on the given absolute time.
		 */
		year = mts->mts_year;
		month = mts->mts_month;

		while (wval > 0) {
			dpm = days_per_month(year, month);
			if (wval > dpm) {
				wval -= dpm;
				if (addunit(iop, mts, MTS_MONTH, 1) != 0)
					return (MOSN_INVAL);

				month++;
				if (month > 12) {
					month = 1;
					year++;
				}
			} else {
				mts->mts_day = (uint32_t)wval;
				wval = 0;
				break;
			}
		}
		return (0);
	case MTS_MONTH:
		mts->mts_month = addmodcarry(mts->mts_month, wval, 12, &carry);
		if (carry == 0)
			return (0);
		wval = 0;
	case MTS_YEAR:
		if (mts->mts_year + carry + wval > MOSTIME_MAXYEAR)
			return (MOS_ERROR(iop, MOSN_INVAL, "%llu > %d",
			  mts->mts_year + carry + wval, MOSTIME_MAXYEAR));
		mts->mts_year = addmodcarry(mts->mts_year, wval, 0, &carry);
		return (0);
	}

	return (MOSN_INVAL);
}

/*
 * "Spreads" sec seconds across res so that res will have the correct number of
 * days, hours, minutes and seconds represented by sec.
 */
static void
makedayrelativetimestamp(uint64_t sec, mostimestamp_t *res) {
	uint64_t asec;

	asec = sec;

	res->mts_flags = MOSTIME_DURATION;
	res->mts_year = 0;
	res->mts_month = 0;
	res->mts_msecond = 0;

	res->mts_day = (unsigned)(asec / 86400);
	asec = asec % 86400;

	res->mts_hour = (unsigned)(asec / 3600);
	asec = asec % 3600;

	res->mts_minute = (unsigned)(asec / 60);
	asec = asec % 60;

	res->mts_second = (unsigned)asec;
}

/*
 * Adds sec (in seconds) to mts, storing the result in res.
 */
static int
subtractseconds(mosiop_t iop, const mostimestamp_t *mts, int64_t sec,
  mostimestamp_t *res) {
	mostimestamp_t rel;
	unsigned month;
	uint64_t asec;
	unsigned year;
	unsigned tmp;
	int64_t n;

	asec = (uint64_t)MOS_ABS(sec);

	makedayrelativetimestamp(asec, &rel);
	res->mts_flags = 0;
	res->mts_msecond = 0;

	if (rel.mts_second <= mts->mts_second) {
		res->mts_second = mts->mts_second - rel.mts_second;
	} else {
		if (addunit(iop, &rel, MTS_MINUTE, 1) != 0)
			return (MOSN_INVAL);
		n = (int64_t)mts->mts_second - (int64_t)rel.mts_second;
		res->mts_second = (unsigned)(60 + n);
	}

	if (rel.mts_minute <= mts->mts_minute) {
		res->mts_minute = mts->mts_minute - rel.mts_minute;
	} else {
		if (addunit(iop, &rel, MTS_HOUR, 1) != 0)
			return (MOSN_INVAL);
		n = (int64_t)mts->mts_minute - (int64_t)rel.mts_minute;
		res->mts_minute = (unsigned)(60 + n);
	}

	if (rel.mts_hour <= mts->mts_hour) {
		res->mts_hour = mts->mts_hour - rel.mts_hour;
	} else {
		if (addunit(iop, &rel, MTS_DAY, 1) != 0)
			return (MOSN_INVAL);
		n = (int64_t)mts->mts_hour - (int64_t)rel.mts_hour;
		res->mts_hour = (unsigned)(24 + n);
	}

	tmp = mts->mts_day - 1;
	month = mts->mts_month;
	year = mts->mts_year;
	if (rel.mts_day <= tmp) {
		res->mts_day = tmp - rel.mts_day + 1;
	} else {
		n = rel.mts_day;
		while (n > 0) {
			if (n <= tmp) {
				res->mts_day = (unsigned)(tmp - n) + 1;
				break;
			} else {
				n -= tmp;

				month--;
				if (month == 0) {
					month = 12;
					year--;
				}

				tmp = days_per_month(year, month);
			}
		}
	}
	res->mts_month = month;
	res->mts_year = year;

	if (res->mts_day > days_per_month(year, month))
		return (MOS_ERROR(iop, MOSN_INVAL, "invalid day %u > %u", res->mts_day,
		  days_per_month(year, month)));

	if (res->mts_month == 0 || res->mts_month > 12)
		return (MOS_ERROR(iop, MOSN_INVAL, "invalid month %u",
		  res->mts_month));

	if (res->mts_year > MOSTIME_MAXYEAR)
		return (MOS_ERROR(iop, MOSN_INVAL, "invalid year %llu > %d",
		  rel.mts_year, MOSTIME_MAXYEAR));

	return (0);
}

/*
 * Determines if the timestamp is for an absolute date in time.
 */
MOSAPI int MOSCConv
mostimestamp_isabsolute(const mostimestamp_t *mts) {

	if (!(mts->mts_flags & MOSTIME_DURATION))
		return (1);
	return (0);
}

/*
 * Determines if the timestamp is for a duration of time.
 */
MOSAPI int MOSCConv
mostimestamp_isduration(const mostimestamp_t *mts) {

	if (mts->mts_flags & MOSTIME_DURATION)
		return (1);
	return (0);
}

/*
 * Determines if the timestamp is local or UTC
 */
MOSAPI int MOSCConv
mostimestamp_islocal(const mostimestamp_t *mts) {

	if (mts->mts_flags & MOSTIME_LOCAL)
		return (1);
	return (0);
}

/*
 * Validates a timestamp for correctness.
 *
 * For absolute timestamps, the values must be within correct range.
 * For durations, days may be as high as 4095 as we need to account for
 * leap years when incrementing an absolute timestamp.
 *
 */
MOSAPI int MOSCConv
mostimestamp_validate(const mostimestamp_t *mts, mosiop_t iop) {

	if (mts == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "timestamp is null"));

	if (mostimestamp_isabsolute(mts)) {
		if (mts->mts_day > days_per_month(mts->mts_year, mts->mts_month))
			return (MOS_ERROR(iop, MOSN_INVAL, "invalid day of the month (%T)",
			  mts));
	}

	if (mts->mts_month > 12)
		return (MOS_ERROR(iop, MOSN_INVAL, "invalid month (%T)", mts));
	if (mts->mts_hour > 23)
		return (MOS_ERROR(iop, MOSN_INVAL, "invalid hour (%T)", mts));
	if (mts->mts_minute > 59)
		return (MOS_ERROR(iop, MOSN_INVAL, "invalid minute (%T)", mts));
	if (mts->mts_second > 59)
		return (MOS_ERROR(iop, MOSN_INVAL, "invalid second (%T)", mts));
	if (mts->mts_msecond > 999)
		return (MOS_ERROR(iop, MOSN_INVAL, "invalid millisecond (%T)", mts));

	return (0);
}

/*
 * Converts the timestamp to a string.
 */
MOSAPI const char * MOSCConv
mostimestamp_string(const mostimestamp_t *mts, char *tbuf, size_t buflen) {
	char year[6];
	char month[5];
	char day[6];
	char hour[5];
	char minute[5];
	char second[5];
	int intime;

	if (mts == NULL)
		return (NULL);

	if (tbuf == NULL)
		return (NULL);

	intime = 0;

	if (mostimestamp_isduration(mts)) {
		if (mts->mts_year > 0)
			mos_snprintf(year, sizeof (year), "%uY", mts->mts_year);
		else
			year[0] = '\0';
		if (mts->mts_month > 0)
			mos_snprintf(month, sizeof (month), "%uM", mts->mts_month);
		else
			month[0] = '\0';
		if (mts->mts_day > 0)
			mos_snprintf(day, sizeof (day), "%uD", mts->mts_day);
		else
			day[0] = '\0';
		if (mts->mts_hour > 0) {
			mos_snprintf(hour, sizeof (hour), "T%uH", mts->mts_hour);
			intime++;
		} else {
			hour[0] = '\0';
		}

		if (mts->mts_minute > 0) {
			if (intime)
				mos_snprintf(minute, sizeof (minute), "%uM", mts->mts_minute);
			else
				mos_snprintf(minute, sizeof (minute), "T%uM", mts->mts_minute);
			intime++;
		} else {
			minute[0] = '\0';
		}

		if (mts->mts_second > 0) {
			if (intime)
				mos_snprintf(second, sizeof (second), "%uS", mts->mts_second);
			else
				mos_snprintf(second, sizeof (second), "T%uS", mts->mts_second);
			intime++;
		} else {
			second[0] = '\0';
		}
		mos_snprintf(tbuf, buflen, "P%s%s%s%s%s%s", year, month, day, hour,
		  minute, second);
	} else {
		if (mostimestamp_islocal(mts)) {
			if (mts->mts_msecond != 0)
				mos_snprintf(tbuf, buflen, "%04u-%02u-%02uT%02u:%02u:%02u.%03u",
				  mts->mts_year, mts->mts_month, mts->mts_day, mts->mts_hour,
				  mts->mts_minute, mts->mts_second, mts->mts_msecond);
			else
				mos_snprintf(tbuf, buflen, "%04u-%02u-%02uT%02u:%02u:%02u",
				  mts->mts_year, mts->mts_month, mts->mts_day, mts->mts_hour,
				  mts->mts_minute, mts->mts_second);
		} else {
			if (mts->mts_msecond != 0)
				mos_snprintf(tbuf, buflen, "%04u-%02u-%02uT%02u:%02u:%02u.%03uZ",
				  mts->mts_year, mts->mts_month, mts->mts_day, mts->mts_hour,
				  mts->mts_minute, mts->mts_second, mts->mts_msecond);
			else
				mos_snprintf(tbuf, buflen, "%04u-%02u-%02uT%02u:%02u:%02uZ",
				  mts->mts_year, mts->mts_month, mts->mts_day, mts->mts_hour,
				  mts->mts_minute, mts->mts_second);
		}
	}

	return (tbuf);
}

#if _MOSTIMESTAMP_FROM_TM_ == 1
MOSAPI int MOSCConv
mostimestamp_fromtm(mosiop_t iop, const struct tm *tm, mostimestamp_t *mts) {

	if (tm == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "struct tm is null"));

	if (mts == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "timestamp is null"));

	mts->mts_year = tm->tm_year + 1900;
	mts->mts_month = tm->tm_mon + 1;
	mts->mts_day = tm->tm_mday;
	mts->mts_hour = tm->tm_hour;
	mts->mts_minute = tm->tm_min;
	mts->mts_second = tm->tm_sec;
	mts->mts_msecond = 0;
	mts->mts_flags = 0;

	return (0);
}

MOSAPI int MOSCConv
mostimestamp_totm(mosiop_t iop, const mostimestamp_t *mts, struct tm *tm) {

	if (tm == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "struct tm is null"));

	if (mts == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "timestamp is null"));

	if (mts->mts_year < 1900)
		return (MOS_ERROR(iop, MOSN_INVAL, "timestamp is prior to 1900"));

	tm->tm_year = mts->mts_year - 1900;
	tm->tm_mon = mts->mts_month - 1;
	tm->tm_mday = mts->mts_day;
	tm->tm_hour = mts->mts_hour;
	tm->tm_min = mts->mts_minute;
	tm->tm_sec = mts->mts_second;
	tm->tm_isdst = -1;

	return (0);
}

MOSAPI int MOSCConv
mostimestamp_toepoch(mosiop_t iop, const mostimestamp_t *mts, time_t *t) {
	struct tm tm;
	int err;

	err = mostimestamp_totm(iop, mts, &tm);
	if (err != 0)
		return (err);

	*t = mktime(&tm);
	if (*t == -1)
		return (MOS_ERROR(iop, MOSN_ERR, "mktime() failed"));

	return (0);
}
#endif

/*
 * Parse an ISO8601 duration timestamp.
 * PnnYnnMnnWnnDTnnHnnMnnS
 */
static int
fromstring_duration(mosiop_t iop, const char *tbuf, mostimestamp_t *mts) {
	mos_tsunit_t unit;
	const char *ss;
	const char *s;
	int32_t val;
	int intime;
	int seen;

	memset(mts, 0, sizeof (*mts));
	mts->mts_flags = MOSTIME_DURATION;
	intime = 0;
	seen = 0;

#define CKUNIT													\
	if ((seen & unit) != 0) {									\
		MOS_ERROR(iop, MOSN_INVAL, "duplicate unit '%c'", *s);	\
		goto fmterr;											\
	} else {													\
		seen |= unit;											\
	}


	s = tbuf;

	if (*s != 'P')
		goto fmterr;
	s++;

	while (*s != '\0') {
		if (*s == 'T') {
			intime = 1;
			s++;
		}

		val = _mos_strto32(s, &ss, 10);
		if (val == MOS_INT32_MAX || val == MOS_INT32_MIN) {
			MOS_ERROR(iop, MOSN_INVAL, "mos_strto32() failed");
			goto fmterr;
		}
		s = ss;
		switch(*s) {
		case 'S':
			if (!intime) {
				MOS_ERROR(iop, MOSN_INVAL, "unit 'S' before 'T'");
				goto fmterr;
			}
			unit = MTS_SECOND;
			CKUNIT
			break;
		case 'M':
			if (intime)
				unit = MTS_MINUTE;
			else
				unit = MTS_MONTH;
			CKUNIT
			break;
		case 'H':
			if (!intime) {
				MOS_ERROR(iop, MOSN_INVAL, "unit 'H' before 'T'");
				goto fmterr;
			}
			unit = MTS_HOUR;
			CKUNIT
			break;
		case 'D':
			unit = MTS_DAY;
			CKUNIT
			break;
		case 'W':
			unit = MTS_WEEK;
			CKUNIT
			break;
		case 'Y':
			unit = MTS_YEAR;
			if (val > MOSTIME_MAXYEAR) {
				MOS_ERROR(iop, MOSN_INVAL, "year %llu > %d", val,
				  MOSTIME_MAXYEAR);
				goto fmterr;
			}
			CKUNIT
			break;
		default:
			goto fmterr;
		}
		if (addunit(iop, mts, unit, val) != 0)
			goto fmterr;
		s++;
	}

	return (0);

fmterr:

	return (MOS_ERROR(iop, MOSN_INVAL, "invalid duration '%s'; "
	  "expected ISO8601 periodic", tbuf));
}

/*
 * Only parses UTC times.
 */
MOSAPI int MOSCConv
mostimestamp_fromstring(mosiop_t iop, const char *tbuf, mostimestamp_t *mts) {
	int year, month, day, hour, minute, second, msecond;
	const char *s;
	uint32_t val;
	char tmp[5];
	int state;
	int err;

#define TOVAL(len, sep) do {			\
	mos_strncpy(tmp, s, (len));			\
	tmp[(len)] = '\0';					\
	if (mos_strlen(tmp) != (len))		\
		goto fmterr;					\
	err = mos_strtou32(tmp, 10, &val);	\
	if (err != 0)						\
		goto fmterr;					\
	s += (len);							\
	if (*s == (sep))					\
		s++;							\
} while (0)

	if (mts == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG,
		  "timestamp return pointer is null"));

	if (tbuf[0] == 'P')
		return (fromstring_duration(iop, tbuf, mts));

	year = 0;
	month = 0;
	day = 0;
	hour = 0;
	minute = 0;
	second = 0;
	msecond = 0;

	s = tbuf;
	for (state = 0; state < 7; state++) {
		switch(state) {
		case 0: /* Y */
			TOVAL(4, '-');
			year = val;
			break;
		case 1: /* M */
			TOVAL(2, '-');
			month = val;
			break;
		case 2: /* D */
			TOVAL(2, 'T');
			if (*s == ' ')
				s++;
			day = val;
			if (*s == '\0')
				goto done;
			break;
		case 3: /* H */
			TOVAL(2, ':');
			hour = val;
			break;
		case 4: /* M */
			TOVAL(2, ':');
			minute = val;
			if (*s == 'Z')
				s++;
			if (*s == '\0')
				goto done;
			break;
		case 5: /* S */
			TOVAL(2, 'Z');
			if (*s == '.')
				s++;
			second = val;
			if (*s == '\0')
				goto done;
			break;
		case 6: /* MS */
			TOVAL(3, 'Z');
			msecond = val;
			if (*s == '\0')
				goto done;
			break;
		}
	}

done:

	if (year > MOSTIME_MAXYEAR)
		return (MOS_ERROR(iop, MOSN_INVAL, "year %u exceeds range limit", year));
	if (month > 12)
		return (MOS_ERROR(iop, MOSN_INVAL, "month %u exceeds range limit", month));
	if (day > 31)
		return (MOS_ERROR(iop, MOSN_INVAL, "day %u exceeds range limit", day));
	if (hour > 23)
		return (MOS_ERROR(iop, MOSN_INVAL, "hour %u exceeds range limit", hour));
	if (minute > 59)
		return (MOS_ERROR(iop, MOSN_INVAL, "minute %u exceeds range limit", minute));
	if (second > 59)
		return (MOS_ERROR(iop, MOSN_INVAL, "second %u exceeds range limit", second));
	if (msecond > 999)
		return (MOS_ERROR(iop, MOSN_INVAL, "msecond %u exceeds range limit", msecond));

	memset(mts, 0, sizeof (*mts));
	mts->mts_year = year;
	mts->mts_month = month;
	mts->mts_day = day;
	mts->mts_hour = hour;
	mts->mts_minute = minute;
	mts->mts_second = second;
	mts->mts_msecond = msecond;

	return (mostimestamp_validate(mts, iop));

fmterr:

	return (MOS_ERROR(iop, MOSN_INVAL, "invalid date '%s'; expected ISO8601", tbuf));
}

MOSAPI int MOSCConv
mostimestamp_cmp(const mostimestamp_t *mts1, const mostimestamp_t *mts2) {

	if (mts1->mts_year != mts2->mts_year)
		return (mts1->mts_year - mts2->mts_year);
	if (mts1->mts_month != mts2->mts_month)
		return (mts1->mts_month - mts2->mts_month);
	if (mts1->mts_day != mts2->mts_day)
		return (mts1->mts_day - mts2->mts_day);
	if (mts1->mts_hour != mts2->mts_hour)
		return (mts1->mts_hour - mts2->mts_hour);
	if (mts1->mts_minute != mts2->mts_minute)
		return (mts1->mts_minute - mts2->mts_minute);
	if (mts1->mts_second != mts2->mts_second)
		return (mts1->mts_second - mts2->mts_second);
	if (mts1->mts_msecond != mts2->mts_msecond)
		return (mts1->mts_msecond - mts2->mts_msecond);

	return (0);
}

MOSAPI int MOSCConv
mostimestamp_addseconds(mosiop_t iop, const mostimestamp_t *mts, int64_t sec,
  mostimestamp_t *res) {
	int err;

	if (sec < 0)
		return(subtractseconds(iop, mts, sec, res));

	*res = *mts;
	err = addunit(iop, res, MTS_SECOND, sec);
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to add seconds to timestamp"));

	return (0);
}

MOSAPI int MOSCConv
mostimestamp_add(mosiop_t iop, const mostimestamp_t *mts,
  const mostimestamp_t *dur, mostimestamp_t *res) {

	if (mts == NULL || !mostimestamp_isabsolute(mts))
		return (MOS_ERROR(iop, MOSN_INVALARG,
		  "first argument must be an absolute timestamp"));

	if (dur == NULL || !mostimestamp_isduration(dur))
		return (MOS_ERROR(iop, MOSN_INVALARG,
		  "second argument must be a duration timestamp"));

	*res = *mts;
	if (dur->mts_second != 0)
		addunit(iop, res, MTS_SECOND, dur->mts_second);
	if (dur->mts_minute != 0)
		addunit(iop, res, MTS_MINUTE, dur->mts_minute);
	if (dur->mts_hour != 0)
		addunit(iop, res, MTS_HOUR, dur->mts_hour);
	if (dur->mts_day != 0)
		addunit(iop, res, MTS_DAY, dur->mts_day);
	if (dur->mts_month != 0)
		addunit(iop, res, MTS_MONTH, dur->mts_month);
	if (dur->mts_year != 0)
		addunit(iop, res, MTS_YEAR, dur->mts_year);

	return (0);

#if 0


	memset(res, 0, sizeof (*res));
	carry = 0;
	a = *ap;
	b = *bp;

	res->mts_msecond = addmodcarry(a.mts_msecond, b.mts_msecond, 1000, &carry);
	res->mts_second = addmodcarry(a.mts_second, b.mts_second, 60, &carry);
	res->mts_minute = addmodcarry(a.mts_minute, b.mts_minute, 60, &carry);
	res->mts_hour = addmodcarry(a.mts_hour, b.mts_hour, 24, &carry);

	dpm = days_per_month(a.mts_year, a.mts_month);

again:
	res->mts_day = addmodcarry(a.mts_day, b.mts_day, dpm, &carry);
	res->mts_month = addmodcarry(a.mts_month, b.mts_month, 12, &carry);
	res->mts_year = addmodcarry(a.mts_year, b.mts_year, 0, &carry);

	/*
	 * After advancing month and year, may need to round res->mts_day
	 * by dpm.  Since we can only be at most 4 days off (mts_day is :5
	 * and dpm is at least 28), we'll only do this once.
	 */
	dpm = days_per_month(res->mts_year, res->mts_month);
	if (res->mts_day > dpm) {
		memset(&b, 0, sizeof (b));
		carry = 0;
		goto again;
	}

	return (0);
#endif
}

MOSAPI int MOSCConv
mostimestamp_timepassed(mosiop_t iop, const mostimestamp_t *basetime,
  const mostimestamp_t *durtime, int *resp) {
	mostimestamp_t now;
	mostimestamp_t mts;
	int err;

	err = mostimestamp_add(iop, basetime, durtime, &mts);
	if (err != 0)
		return (MOS_ERROR(iop, err, "mostimestamp_add() failed"));

	if (mostimestamp_now(&now) != 0)
		return (MOS_ERROR(iop, MOSN_ERR, "failed to get current time"));

	*resp = mostimestamp_cmp(&now, &mts) >= 0;

	return (0);
}

MOSAPI const char * MOSCConv
mostimestamp_dayofweek(mostimestamp_t *mts) {
	int dow;

	dow = mostimestamp_ndayofweek(mts);
	if (dow == -1)
		return ("");
	return (mostimestamp_days[dow]);
}

MOSAPI const char * MOSCConv
mostimestamp_monthstring(mostimestamp_t *mts) {

	if (mts == NULL || mts->mts_month < 1 || mts->mts_month > 12)
		return ("");
	return (mostimestamp_months[mts->mts_month - 1]);
}

/*
 * Calculate the day of the week using Sakamoto's algorithm.
 * This is valid from Sept 14, 1752 - 9999.
 * Sept 14, 1752 begins the Gregorian Calendar.
 */
MOSAPI int MOSCConv
mostimestamp_ndayofweek(mostimestamp_t *mts) {
	static uint8_t t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
	int y, m, d;

	if (mts == NULL)
		return (-1);

	if (mts->mts_month > 11)
		return (-1);

	if (!mostimestamp_isabsolute(mts))
		return (-1);

	if (mts->mts_year < 1752)
		return (-1);
	if (mts->mts_year == 1752) {
		if (mts->mts_month < 9)
			return (-1);
		if (mts->mts_month == 9) {
			if (mts->mts_day < 14)
				return (-1);
		}
	}

	y = mts->mts_year;
	m = mts->mts_month;
	d = mts->mts_day - 1;

	y -= m < 3;
	return ((y + y/4 - y/100 + y/400 + t[m-1] + d) % 7);
}

MOSAPI int MOSCConv
mostimestamp_torfc1123date(mostimestamp_t *mts, char **datep,
  uint32_t *datelenp) {
	const char *dow;

	if (mts == NULL)
		return (MOSN_INVAL);

	dow = mostimestamp_dayofweek(mts);

	mos_asprintf(datep, datelenp, "%s, %02u %s %04u %02u:%02u:%02u GMT",
	  dow, mts->mts_day, mostimestamp_months[mts->mts_month - 1], mts->mts_year,
	  mts->mts_hour, mts->mts_minute, mts->mts_second);

	return (0);
}
