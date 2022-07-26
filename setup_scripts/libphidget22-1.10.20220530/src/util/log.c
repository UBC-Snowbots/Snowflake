/*
 * This file is part of libphidget22
 *
 * Copyright 2015 Phidgets Inc <patrick@phidgets.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/>
 */

 /*
  * Phidget logging logs to a single file, at various log levels.  The log level of a message
  * determines if it will be logged to the file, or thrown away.  Users can control the log level
  * at which the system will accept messages.
  *
  * Each message is from a log source.  The default log source is the 'phidget' log source.
  * Each log source has its own log level.
  *
  * New log sources are dynamically created when a message is received from a source that has not
  * previously been seen.  There is curently no way to create a log source prior sending a message
  * from that source.  The recommended practive is to send a inital 'setup' message.  The inital log
  * level will be the level of that message.
  * Log sources cannot be deleted.
  *
  * Messages that repeat will be detected, and not be written to the log file.  When a new message is
  * received the log system will generate a message indicating how many times the message repeated.
  *
  * Setting the log level with setLogLevel() will set the log level of every log source.
  */

#include "phidgetbase.h"

#include "mos/mos_time.h"
#include "mos/bsdtree.h"
#include "mos/mos_stacktrace.h"
#include "mos/mos_readdir.h"
#include "mos/mos_fileio.h"

struct internal_logsource {
	const char			*name;
	Phidget_LogLevel	level;
	int					flags;
};
struct internal_logsource internal_logsources[] = {
	// These are controlled by public setLogLevel API
	{ "phidget22usb", PHIDGET_LOG_INFO, 0 },
	{ "phidget22net", PHIDGET_LOG_INFO, 0 },
	{ "phidget22netctl", PHIDGET_LOG_INFO, 0 },
	// These are meant for internal use
#ifdef DEBUG
	{ "_phidget22match", PHIDGET_LOG_ERROR, 0 },
	{ "_phidget22channel", PHIDGET_LOG_ERROR, 0 },
	{ "_phidget22bridge", PHIDGET_LOG_ERROR, 0 },
	{ "_phidget22disp", PHIDGET_LOG_ERROR, 0 },
#endif
	{ "_pconf", PHIDGET_LOG_ERROR, 0 },
	{ 0 }
};

#define LOGMSG_MAX		1024

#define NETWORK_LOGGING	"_PHIDGET_LOG_NETWORK_"
#define LOG_PORT		5771

#define LOGF_KEY		0x01
#define LOGF_USEBASE	0x02
#define LOGF_OPEN		0x04
#define LOGF_USERSRC	0x08

typedef struct _logmfile {
	char				*fname;
	uint32_t			fsize;
	uint32_t			flags;
	RB_ENTRY(_logmfile)	link;
} logmfile_t;

typedef RB_HEAD(logmfiles, _logmfile) logmfiles_t;

static mos_socket_t logsrvsock;
static mos_socket_t logclisock;
static int			logmfilecnt;
static logmfiles_t	logmfiles;

static int
logmfile_compare(logmfile_t *a, logmfile_t *b) {
	mostimestamp_t ats, bts;
	char atm[32], btm[32];
	const char *c;
	char *t;
	int err;

	c = mos_strrchrc(a->fname, '.');
	if (c == NULL)
		return (-1);
	mos_strlcpy(atm, c + 1, sizeof (atm));

	for (t = atm; *t; t++)
		if (*t == '_')
			*t = ':';

	err = mostimestamp_fromstring(MOS_IOP_IGNORE, atm, &ats);
	if (err != 0)
		return (-1);

	c = mos_strrchrc(b->fname, '.');
	if (c == NULL)
		return (1);
	mos_strlcpy(btm, c + 1, sizeof (btm));
	for (t = btm; *t; t++)
		if (*t == '_')
			*t = ':';

	err = mostimestamp_fromstring(MOS_IOP_IGNORE, btm, &bts);
	if (err != 0)
		return (1);

	return (mostimestamp_cmp(&ats, &bts));
}

RB_PROTOTYPE(logmfiles, _logmfile, link, logmfile_compare)
RB_GENERATE(logmfiles, _logmfile, link, logmfile_compare)

typedef struct _logsrc {
	const char			*sname;
	char				*name;
	int					flags;
	Phidget_LogLevel	level;
	RB_ENTRY(_logsrc)	link;
} logsrc_t;

static int
logsrc_compare(logsrc_t *a, logsrc_t *b) {
	const char *an, *bn;

	if (a->sname != NULL)
		an = a->sname;
	else
		an = a->name;

	if (b->sname != NULL)
		bn = b->sname;
	else
		bn = b->name;

	return (mos_strcmp(an, bn));
}

#define LOGLEVEL(l)	((l) & ~(LOGF_STDERR | LOGF_DEBUGGER))
#define PHIDGET_LOGSRC "phidget22"

typedef RB_HEAD(logsrc, _logsrc) logsrctree_t;
RB_PROTOTYPE(logsrc, _logsrc, link, logsrc_compare)
RB_GENERATE(logsrc, _logsrc, link, logsrc_compare)

static Phidget_LogLevel	defLevel;						/* The default log level */
static logsrctree_t		srctree;
static mos_mutex_t		lock;
static int				initialized;
static int				enabled;
static mos_file_t		*logmf;
static mos_file_t		*stderrf;
static int				stderrio;
static logsrc_t			*psrc;							/* phidget logsrc */
static char				*logbasename;					/* log file base name */
static char				*dirname;						/* log directory name */

static int				logAutoRotate = 1;				/* Rotate log file automatically */
static uint64_t			logRotateSize = 1048576 * 10;	/* 10 MB */
static int				logRotationKeep = 1;			/* how many log files to keep when rotating */
static uint64_t			logSize;						/* estimated log size */

static int
isInitialized() {
	int init;

	mos_glock((void *)4);
	init = initialized;
	mos_gunlock((void *)4);

	return (init);
}

#define CHECKINITIALIZED_PR do {			\
	if (!isInitialized())				\
		return (PHID_RETURN(EPHIDGET_CLOSED));		\
} while (0)

#define CHECKENABLED do {			\
	if (!enabled)				\
		return (EPHIDGET_OK);		\
} while (0)

void
PhidgetLogInit() {

	mos_glock((void *)4);
	if (initialized == 1) {
		mos_gunlock((void *)4);
		return;
	}

	mos_mutex_init(&lock);
	RB_INIT(&logmfiles);
	logmfilecnt = 0;

	RB_INIT(&srctree);
	logmf = NULL;

	initialized = 1;
	logsrvsock = MOS_INVALID_SOCKET;
	logclisock = MOS_INVALID_SOCKET;

	mos_gunlock((void *)4);
}

static int
removeLogFile(int dounlink) {
	logmfile_t *lfile;

	lfile = RB_MIN(logmfiles, &logmfiles);
	if (lfile == NULL)
		return (0);

	RB_REMOVE(logmfiles, &logmfiles, lfile);
	MOS_ASSERT(logmfilecnt > 0);
	logmfilecnt--;

	if (dounlink)
		mos_file_unlink("%s", lfile->fname);

	mos_free(lfile->fname, MOSM_FSTR);
	mos_free(lfile, sizeof (*lfile));

	return (1);
}

static const char *
getFileDate() {
	static char buf[32];
	static char tmp[32];
	const char *s;
	char *d;

	mos_snprintf(tmp, sizeof (tmp), "%#T", NULL);

	for (s = tmp, d = buf; *s; s++, d++) {
		if (*s == ':') {
			*d = '_';
			continue;
		}
		*d = *s;
	}
	*d = '\0';

	return (buf);
}

static PhidgetReturnCode
addLogFile(const char *logfile, uint32_t sz, int renamelog) {
	char rlogfile[MOS_PATH_MAX];
	logmfile_t *lfile;
	uint64_t fsize;
	int err;

	err = mos_file_getsizex(MOS_IOP_IGNORE, &fsize, "%s", logfile);
	if (err != 0)
		return (err);

	if (renamelog) {
		mos_snprintf(rlogfile, sizeof (rlogfile), "%s.%s", logfile, getFileDate());
		rename(logfile, rlogfile);
	} else {
		mos_strlcpy(rlogfile, logfile, sizeof (rlogfile));
	}

	lfile = mos_zalloc(sizeof (*lfile));
	lfile->fname = mos_strdup(rlogfile, NULL);
	lfile->fsize = (uint32_t)fsize;

	RB_INSERT(logmfiles, &logmfiles, lfile);
	logmfilecnt++;

	return (EPHIDGET_OK);
}

void
PhidgetLogFini() {
	logsrc_t *src, *nxt;

	mos_glock((void *)4);
	if (initialized == 0) {
		mos_gunlock((void *)4);
		return;
	}

	mos_mutex_lock(&lock);

	if (logsrvsock != MOS_INVALID_SOCKET)
		mos_netop_udp_closesocket(MOS_IOP_IGNORE, &logsrvsock);
	if (logclisock != MOS_INVALID_SOCKET)
		mos_netop_udp_closesocket(MOS_IOP_IGNORE, &logclisock);

	if (stderrf && stderrf != logmf)
		mos_file_close(MOS_IOP_IGNORE, &stderrf);
	if (logmf)
		mos_file_close(MOS_IOP_IGNORE, &logmf);

	for (src = RB_MIN(logsrc, &srctree); src != NULL; src = nxt) {
		nxt = RB_NEXT(logsrc, &srctree, src);
		RB_REMOVE(logsrc, &srctree, src);
		mos_free(src->name, MOSM_FSTR);
		mos_free(src, sizeof (*src));
	}

	for (;;)
		if (removeLogFile(0) == 0)
			break;

	if (logbasename)
		mos_free(logbasename, MOSM_FSTR);
	logbasename = NULL;

	if (dirname)
		mos_free(dirname, MOSM_FSTR);
	dirname = NULL;

	psrc = NULL;

	mos_mutex_unlock(&lock);
	mos_mutex_destroy(&lock);

	initialized = 0;
	mos_gunlock((void *)4);
}

#define MAX_LVL_TO_STR_LEN	"7"
static const char *
lvlToStr(Phidget_LogLevel l) {

	switch (LOGLEVEL(l)) {
	case PHIDGET_LOG_CRITICAL:
		return ("<CRIT>");
	case PHIDGET_LOG_ERROR:
		return ("<ERROR>");
	case PHIDGET_LOG_WARNING:
		return ("<Warn>");
	case PHIDGET_LOG_DEBUG:
		return ("<Debug>");
	case PHIDGET_LOG_INFO:
		return ("<Info>");
	case PHIDGET_LOG_VERBOSE:
		return ("<Verb>");
	default:
		return ("");
	}
}

static int
validLogLevel(Phidget_LogLevel l) {

	switch (LOGLEVEL(l)) {
	case PHIDGET_LOG_CRITICAL:
	case PHIDGET_LOG_ERROR:
	case PHIDGET_LOG_WARNING:
	case PHIDGET_LOG_DEBUG:
	case PHIDGET_LOG_INFO:
	case PHIDGET_LOG_VERBOSE:
		return (1);
	default:
		return (0);
	}
}

API_PRETURN
PhidgetLog_enableRotating() {

	mos_mutex_lock(&lock);
	logAutoRotate = 1;
	mos_mutex_unlock(&lock);

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLog_disableRotating() {

	mos_mutex_lock(&lock);
	logAutoRotate = 0;
	mos_mutex_unlock(&lock);

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLog_isRotating(int *autorotate) {

	TESTPTR_PR(autorotate);

	mos_mutex_lock(&lock);
	*autorotate = logAutoRotate;
	mos_mutex_unlock(&lock);

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLog_setRotating(uint64_t rotatesz, int keepcnt) {

	if (rotatesz < 32768)
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "rotatesz must be >= 32768"));

	if (keepcnt < 0 || keepcnt > 64)
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "keepcnt must between 0 and 64."));

	mos_mutex_lock(&lock);
	if (keepcnt == 0) {
		logRotationKeep = 0;
		logRotationKeep = 0;
	} else {
		logRotationKeep = 0;
		logRotationKeep = keepcnt;
	}

	logRotateSize = rotatesz;
	mos_mutex_unlock(&lock);

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLog_getRotating(uint64_t *rotatesz, int *keep) {

	mos_mutex_lock(&lock);
	if (rotatesz)
		*rotatesz = logRotateSize;
	if (keep)
		*keep = logRotationKeep;
	mos_mutex_unlock(&lock);

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLog_getSources(const char *names[], uint32_t *count) {
	logsrc_t *src;
	uint32_t i;

	TESTPTR_PR(count);

	i = 0;

	mos_mutex_lock(&lock);
	RB_FOREACH(src, logsrc, &srctree) {
		if (names)
			names[i] = src->name;
		i++;
		if (names && i >= *count)
			break;
	}

	*count = i;
	mos_mutex_unlock(&lock);

	return (EPHIDGET_OK);
}

static logsrc_t *
_getLogSource(const char *name) {
	logsrc_t key;

	key.sname = name;
	return (RB_FIND(logsrc, &srctree, &key));
}

static logsrc_t *
_addLogSource(const char *name, Phidget_LogLevel level) {
	logsrc_t *src;

	src = _getLogSource(name);
	if (src != NULL)
		return (src);

	src = mos_zalloc(sizeof (*src));
	src->name = mos_strdup(name, NULL);
	src->level = level;

	if (RB_INSERT(logsrc, &srctree, src) != NULL)
		MOS_PANIC("Implosible duplicate log source");

	return (src);
}

/*
 * Adds and returns the source if it does not exist; otherwise, the existing source is returned.
 */
static logsrc_t *
PhidgetLog_addLogSource(const char *name, Phidget_LogLevel level) {
	logsrc_t *src;

	mos_mutex_lock(&lock);
	src = _addLogSource(name, level);
	mos_mutex_unlock(&lock);
	return (src);
}

API_PRETURN
PhidgetLog_addSource(const char *name, Phidget_LogLevel level) {

	if (PhidgetLog_addLogSource(name, level) == NULL)
		return (PHID_RETURN(EPHIDGET_UNEXPECTED));
	return (EPHIDGET_OK);
}


static uint32_t lastmsgdupcnt;
static char lastmsg[1024];

static PhidgetReturnCode
_writelog(mos_file_t *mf, const char *hdr, const char *msg) {
	char mbuf[LOGMSG_MAX];
	const char *p;
	int isdup;
	int len;
	int err;
	char *m;

	if (mos_strlen(hdr) + mos_strlen(msg) >= LOGMSG_MAX)
		return (EPHIDGET_NOSPC);

	isdup = mos_strcmp(msg, lastmsg) == 0;
	if (isdup)
		lastmsgdupcnt++;
	else
		mos_strlcpy(lastmsg, msg, sizeof (lastmsg));

	if (isdup && lastmsgdupcnt > 3)
		return (EPHIDGET_OK);

	if (!isdup && lastmsgdupcnt > 3) {
		len = mos_snprintf(mbuf, sizeof (mbuf), "last message repeated %u times\n", lastmsgdupcnt + 1);
		if (len >= (int)sizeof (mbuf))
			return (EPHIDGET_NOSPC);

		err = mos_file_write(MOS_IOP_IGNORE, mf, mbuf, len);
		if (err == 0)
			logSize += len;
		lastmsgdupcnt = 0;
	}

	m = mbuf;
	len = 0;

	// hdr -> mbuf
	p = hdr;
	while (*p) {
		*m++ = *p++;
		len++;
	}

	// msg -> mbuf
	p = msg;
	while (*p) {

		if ((len + 3) >= LOGMSG_MAX)
			return (EPHIDGET_NOSPC);

		switch (*p) {

		case '\n':
			p++;
#ifdef _WINDOWS
			// Windows style newlines
			*m++ = '\r';
			len++;
#endif
			*m++ = '\n';
			len++;
			// Indent newlines (not last newline)
			if (*p != '\0') {
				*m++ = '\t';
				len++;
			}
			break;

		case '\r':
			// Ignore
			p++;
			break;

		default:
		*m++ = *p++;
		len++;
			break;
		}
	}

	*m = '\0';

	err = mos_file_write(MOS_IOP_IGNORE, mf, mbuf, len);
	if (err != 0)
		return (err);

	logSize += len;

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
_rotateLogFile(uint64_t rotatesz, int keep) {
	char logfile[MOS_PATH_MAX];
	int err;

	if (logmf == NULL || stderrio)
		return (EPHIDGET_OK);

	if (logSize < rotatesz)
		return (EPHIDGET_OK);

	mos_snprintf(logfile, sizeof (logfile), "%s/%s", dirname, logbasename);
	mos_file_close(MOS_IOP_IGNORE, &logmf);

	if (keep) {
		addLogFile(logfile, (uint32_t)logSize, 1);
	} else {
		err = mos_file_unlink("%s", logfile);
		if (err)
			fprintf(stderr, "failed to unlink '%s': %d\n", logfile, err);
	}

	err = mos_file_open(MOS_IOP_IGNORE, &logmf, MOS_FILE_CREATE | MOS_FILE_WRITE | MOS_FILE_READ, "%s", logfile);
	if (err != 0) {
		fprintf(stderr, "failed to reopen '%s': %s\n", logfile, strerror(errno));
		return (err);
	}

	mos_file_write(MOS_IOP_IGNORE, logmf, "Log File Rotated\n", 17);
	logSize = 17;

	while (logmfilecnt > logRotationKeep)
		removeLogFile(1);

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLog_rotate() {
	PhidgetReturnCode res;

	CHECKINITIALIZED_PR;

	mos_mutex_lock(&lock);
	res = _rotateLogFile(logRotateSize, logRotationKeep);
	mos_mutex_unlock(&lock);
	if (res != 0)
		return (PHID_RETURN(res));

	PhidgetLog_logs(PHIDGET_LOG_INFO, "Log Rotated");
	return (EPHIDGET_OK);
}
#ifndef NDEBUG
static PhidgetReturnCode
_netlog(Phidget_LogLevel level, const char *srcname, const char *file, int line, const char *func,
  const char *msg, size_t msglen) {
	PhidgetReturnCode res;
	char buf[2048];
	pconf_t *pc;
	size_t len;

	if (msg == NULL || msglen > 1024)
		return (EPHIDGET_INVALIDARG);

	res = pconf_create(&pc);
	if (res != EPHIDGET_OK)
		return (res);

	pconf_addi(pc, 1, "V");				/* version */
	pconf_addstr(pc, "logkey", "K");	/* pass phrase */
	pconf_addi(pc, 0, "F");				/* flags */
	pconf_addi(pc, level, "ll");		/* log level */
	pconf_addstr(pc, srcname, "sn");	/* source name */
	pconf_addstr(pc, file == NULL ? "" : file, "fn");
	pconf_addi(pc, line, "ln");
	pconf_addstr(pc, func == NULL ? "" : func, "fc");
	pconf_addstr(pc, msg, "mg");

	res = pconf_renderjson(pc, buf, sizeof (buf));
	pconf_release(&pc);
	if (res != EPHIDGET_OK)
		return (res);

	len = mos_strlen(buf);
	return (mos_netop_udp_send(MOS_IOP_IGNORE, &logclisock, buf, &len));

}
#endif
static PhidgetReturnCode
_log(Phidget_LogLevel level, logsrc_t *src, const char *hdr, size_t hdrlen, const char *msg, size_t msglen) {
	int err;

#ifdef _ANDROID
	switch(level) {
		case PHIDGET_LOG_CRITICAL:
			__android_log_print(ANDROID_LOG_FATAL, "Phidget22", "%s%s", hdr, msg);
			break;
		case PHIDGET_LOG_ERROR:
			__android_log_print(ANDROID_LOG_ERROR, "Phidget22", "%s%s", hdr, msg);
			break;
		case PHIDGET_LOG_WARNING:
			__android_log_print(ANDROID_LOG_WARN, "Phidget22", "%s%s", hdr, msg);
			break;
		case PHIDGET_LOG_INFO:
			__android_log_print(ANDROID_LOG_INFO, "Phidget22", "%s%s", hdr, msg);
			break;
		case PHIDGET_LOG_VERBOSE:
			__android_log_print(ANDROID_LOG_VERBOSE, "Phidget22", "%s%s", hdr, msg);
			break;
		case PHIDGET_LOG_DEBUG:
			__android_log_print(PHIDGET_LOG_DEBUG, "Phidget22", "%s%s", hdr, msg);
			break;

	}
	return (EPHIDGET_OK);
#endif

#if _WINDOWS
	if (level & LOGF_DEBUGGER || src->flags & LOGF_DEBUGGER) {
		OutputDebugStringA(hdr);
		OutputDebugStringA(msg);
		return (EPHIDGET_OK);
	}
#endif

	if (level & LOGF_STDERR || src->flags & LOGF_STDERR) {
		if (stderrf == NULL) {
			err = mos_file_open(MOS_IOP_IGNORE, &stderrf, 0, MOS_FILE_STDERR);
			if (err != 0)
				return (err);
		}
		return (_writelog(stderrf, hdr, msg));
	}

	if (logmf != NULL)
		return (_writelog(logmf, hdr, msg));

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLog_loges(Phidget_LogLevel level, const char *srcname, const char *str) {

	return (PhidgetLog_loge(NULL, 0, __func__, srcname, level, "%s", str));
}

API_PRETURN
PhidgetLog_logs(Phidget_LogLevel level, const char *str) {

	return (PhidgetLog_log(level, "%s", str));
}

API_PRETURN
PhidgetLog_log(Phidget_LogLevel level, const char *fmt, ...) {
	va_list va;
	int ret;

	CHECKENABLED;
	CHECKINITIALIZED_PR;

	va_start(va, fmt);
	ret = PhidgetLog_logv(NULL, 0, NULL, NULL, level, fmt, va);
	va_end(va);

	return (ret);
}

API_PRETURN
PhidgetLog_loge(const char *file, int line, const char *func, const char *srcname, Phidget_LogLevel level,
  const char *fmt, ...) {
	va_list va;
	int ret;

	CHECKENABLED;
	CHECKINITIALIZED_PR;

	va_start(va, fmt);
	ret = PhidgetLog_logv(file, line, func, srcname, level, fmt, va);
	va_end(va);

	return (ret);
}

API_PRETURN
PhidgetLog_logv(const char *file, int line, const char *func, const char *srcname, Phidget_LogLevel level,
  const char *fmt, va_list va) {
	PhidgetReturnCode res;
	char buf[4096];	/* do not allow crazy amounts of data in the log.. */
	char hdr[128];
	logsrc_t *src;
	size_t hdrlen;
	size_t buflen;

#ifndef NDEBUG
	const char *c;
	char fbuf[256];
#endif

	CHECKENABLED;
	CHECKINITIALIZED_PR;

	/*
	 * If the srcname is NULL or is PHIDGET_LOGSRC, and we have already assigned psrc, then short
	 * circuit the addition of the logsrc and the locking as it is unneeded.
	 *
	 * This should be the most common case.
	 */
	if ((srcname == NULL || mos_strcmp(srcname, PHIDGET_LOGSRC) == 0) && psrc != NULL) {
		srcname = PHIDGET_LOGSRC;
		src = psrc;
	} else {
		if (srcname == NULL)
			srcname = PHIDGET_LOGSRC;

		src = PhidgetLog_addLogSource(srcname, defLevel);
		if (mos_strcmp(srcname, PHIDGET_LOGSRC) == 0)
			psrc = src;
	}

	if (src->level < LOGLEVEL(level))
		return (EPHIDGET_OK);

	buflen = mos_vsnprintf(buf, sizeof (buf), fmt, va);
	if (buflen >= sizeof (buf) - 3)
		buflen = sizeof (buf) - 3;

	// Add a newline if we didn't pass one in.
	if (buf[buflen - 1] != '\n')
		buf[buflen++] = '\n';

	buf[buflen] = '\0';

#if NDEBUG
	if (func == NULL)
		hdrlen = mos_snprintf(hdr, sizeof(hdr), "%#T %" MAX_LVL_TO_STR_LEN "s %s : ", NULL, lvlToStr(level), srcname);
	else
		hdrlen = mos_snprintf(hdr, sizeof (hdr), "%#T %" MAX_LVL_TO_STR_LEN "s %s[%s()] : ", NULL, lvlToStr(level), srcname, func);
#else
	// Truncate the filename from any path information for the log header
	if (file != NULL) {
#ifdef _WINDOWS
		c = mos_strrchrc(file, '\\');
#else
		c = mos_strrchrc(file, '/');
#endif /* _WINDOWS */
		if (c != NULL) {
			mos_strlcpy(fbuf, c + 1, sizeof(fbuf));
			file = fbuf;
		}
	}

	if (logclisock != MOS_INVALID_SOCKET) {
		mos_mutex_lock(&lock);
		res = _netlog(level, srcname, file, line, func, buf, buflen);
		mos_mutex_unlock(&lock);
		return (PHID_RETURN(res));
	}

	if (file == NULL)
		hdrlen = mos_snprintf(hdr, sizeof(hdr), "%#T %" MAX_LVL_TO_STR_LEN "s %s : ", NULL, lvlToStr(level), srcname);
	else
		hdrlen = mos_snprintf(hdr, sizeof(hdr), "%#T %" MAX_LVL_TO_STR_LEN "s %s[%.32s+%d %s()] : ",
			NULL, lvlToStr(level), srcname, file, line, func);

#endif /* NDEBUG */

	mos_mutex_lock(&lock);
	res = _log(level, src, hdr, hdrlen, buf, buflen);

	if (logAutoRotate)
		_rotateLogFile(logRotateSize, logRotationKeep);
	mos_mutex_unlock(&lock);

	return (PHID_RETURN(res));
}

API_PRETURN
PhidgetLog_setSourceLevel(const char *name, Phidget_LogLevel level) {
	logsrc_t *src;

	CHECKINITIALIZED_PR;

	if (!validLogLevel(level))
		return (PHID_RETURN(EPHIDGET_INVALIDARG));

	mos_mutex_lock(&lock);
	src = _getLogSource(name);
	if (src == NULL) {
		mos_mutex_unlock(&lock);
		return (PHID_RETURN(EPHIDGET_NOENT));
	}
	src->level = level;
	mos_mutex_unlock(&lock);

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLog_getSourceLevel(const char *name, Phidget_LogLevel *level) {
	logsrc_t *src;

	mos_mutex_lock(&lock);
	src = _getLogSource(name);
	if (src == NULL) {
		mos_mutex_unlock(&lock);
		return (PHID_RETURN(EPHIDGET_NOENT));
	}

	*level = src->level;
	mos_mutex_unlock(&lock);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLog_getLevel(Phidget_LogLevel *level) {

	return (PhidgetLog_getSourceLevel(PHIDGET_LOGSRC, level));
}

/*
 * Sets the log level of every source to maintain POLA.
 * Sources that begin with _phidget22 are NOT adjusted by this call.
 */
API_PRETURN
PhidgetLog_setLevel(Phidget_LogLevel level) {
	logsrc_t *src;

	CHECKINITIALIZED_PR;

	if (!validLogLevel(level))
		return (PHID_RETURN(EPHIDGET_INVALIDARG));

	mos_mutex_lock(&lock);
	defLevel = level;
	RB_FOREACH(src, logsrc, &srctree) {
		if (mos_strncmp(src->name, "_phidget22", 10) == 0)
			continue;
		src->level = level;
	}
	mos_mutex_unlock(&lock);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
addExistingLogFile(const char *fname) {
	char logfile[MOS_PATH_MAX];
	uint64_t sz;
	size_t n;
	int err;

	n = mos_snprintf(logfile, sizeof (logfile), "%s/%s", dirname, fname);
	if (n >= sizeof (logfile))
		return (EPHIDGET_NOSPC);

	err = mos_file_getsizex(MOS_IOP_IGNORE, &sz, "%s", logfile);
	if (err != 0)
		return (err);

	return (addLogFile(logfile, (uint32_t)sz, 0));
}

static PhidgetReturnCode
scanExistingLogFiles() {
	mos_dirinfo_t *di;
	int err;

	err = mos_opendir(MOS_IOP_IGNORE, dirname, &di);
	if (err != 0)
		return (err);


	for (;;) {
		err = mos_readdir(MOS_IOP_IGNORE, di);
		if (err != 0)
			break;

		if (di->errcode == MOSN_NOENT)
			break;

		if (di->flags & MOS_DIRINFO_ISDIR)
			continue;

		if (mos_strstr(di->filename, logbasename) == NULL)
			continue;

		if (mos_strcmp(di->filename, logbasename) == 0)
			continue;

		addExistingLogFile(di->filename);
	}

	mos_closedir(&di);

	return (err);

}

static PhidgetReturnCode
openNetworkClient(const char *dest) {
	PhidgetReturnCode res;
	mos_sockaddr_t caddr;
	const char *c;
	int port;

	c = mos_strchrc(dest, ':');
	if (c == NULL) {
		port = LOG_PORT;
	} else {
		res = mos_strto32(c + 1, 0, &port);
		if (res != 0)
			return (EPHIDGET_INVALIDARG);
	}
	memset(&caddr, 0, sizeof (caddr));
	caddr.s4.sin_family = AF_INET;
	inet_pton(AF_INET, "127.0.0.1", &caddr.s4.sin_addr);
	caddr.s4.sin_port = htons(port);

	return (mos_netop_udp_opensocket(MOS_IOP_IGNORE, &logclisock, &caddr));
}

API_PRETURN
PhidgetLog_enable(Phidget_LogLevel level, const char *dest) {
	struct internal_logsource *il;
	char dname[MOS_PATH_MAX];
	PhidgetReturnCode res;
	const char *bname;
	mosiop_t iop;

	CHECKINITIALIZED_PR;

	if (!validLogLevel(level))
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Invalid log level."));

	mos_mutex_lock(&lock);

	if (logmf != NULL) {
		mos_mutex_unlock(&lock);
		return (PHID_RETURN_ERRSTR(EPHIDGET_BUSY, "Logging already enabled."));
	}

	defLevel = level;
	iop = mos_iop_alloc();

	if (dest == NULL) {
		res = mos_file_open(iop, &logmf, 0, MOS_FILE_STDERR);
		if (res != 0) {
			mos_mutex_unlock(&lock);
			return (PHID_RETURN_IOP(res, iop));
		}
		stderrf = logmf;
		stderrio = 1;
		goto addsources;
	}

	if (mos_strncmp(dest, NETWORK_LOGGING, mos_strlen(NETWORK_LOGGING)) == 0) {
		res = openNetworkClient(dest);
		if (res != EPHIDGET_OK) {
			mos_iop_release(&iop);
			mos_mutex_unlock(&lock);
			return (PHID_RETURN_ERRSTR(res, "Failed to enable network logging."));
		}
		goto addsources;
	}

	bname = mos_basename(dest);
	if (mos_strlen(bname) < 2) {
		mos_iop_release(&iop);
		mos_mutex_unlock(&lock);
		return (PHID_RETURN_ERRSTR(EPHIDGET_INVALIDARG, "Invalid filename."));
	}
	logbasename = mos_strdup(bname, NULL);

	bname = mos_dirname(dest, dname, sizeof (dname));
	if (mos_strlen(bname) < 1) {
		res = EPHIDGET_INVALIDARG;
		MOS_ERROR(iop, res, "Invalid filename.");
		goto bad;
	}
	dirname = mos_strdup(bname, NULL);

	res = mos_file_open(iop, &logmf, MOS_FILE_CREATE | MOS_FILE_WRITE | MOS_FILE_READ, "%s", dest);
	if (res != 0)
		goto bad;

	res = mos_file_getsize(iop, logmf, &logSize);
	if (res != 0) {
		fprintf(stderr, "failed to determine log file size: '%s'", dest);
		goto bad;
	}

	res = mos_file_seek(iop, logmf, logSize);
	if (res != 0) {
		fprintf(stderr, "failed to seek to end of log file: '%s'", dest);
		goto bad;
	}

	scanExistingLogFiles();

addsources:

	enabled = 1;

	mos_iop_release(&iop);
	mos_mutex_unlock(&lock);

	for (il = internal_logsources; il->name != NULL; il++)
		PhidgetLog_addSource(il->name, il->level);

	PhidgetLog_setLevel(level);
	if (dest != NULL)
		PhidgetLog_log(level,
			"\n******************************  Logging Enabled  ******************************"
			"\n* %s%*s *"
			"\n* Release %s - %s%*s *"
			"\n*******************************************************************************"
		, LibraryVersion, (75 - strlen(LibraryVersion)), " "
		, LibraryVersionNumber, LibrarySystem, (64 - strlen(LibraryVersionNumber) - strlen(LibrarySystem)), " ");

	return (EPHIDGET_OK);

bad:

	if (logmf != NULL) {
		mos_file_close(MOS_IOP_IGNORE, &logmf);
		stderrf = NULL;
		stderrio = 0;
	}

	if (logbasename) {
		mos_free(logbasename, MOSM_FSTR);
		logbasename = NULL;
	}

	if (dirname) {
		mos_free(dirname, MOSM_FSTR);
		dirname = NULL;
	}

	mos_mutex_unlock(&lock);

	return (PHID_RETURN_IOP(res, iop));
}

API_PRETURN
PhidgetLog_disable() {

	mos_mutex_lock(&lock);

	enabled = 0;

	if (stderrf && stderrf != logmf)
		mos_file_close(MOS_IOP_IGNORE, &stderrf);
	stderrf = NULL;
	stderrio = 0;

	if (logmf)
		mos_file_close(MOS_IOP_IGNORE, &logmf);

	if (dirname) {
		mos_free(dirname, MOSM_FSTR);
		dirname = NULL;
	}

	if (logbasename) {
		mos_free(logbasename, MOSM_FSTR);
		logbasename = NULL;
	}

	mos_mutex_unlock(&lock);

	return (EPHIDGET_OK);
}

void
logStackTrace(Phidget_LogLevel level, const char *msg) {
	void *stack[32];
	char sname[128];
	char buf[4096];
	uint32_t n;
	size_t cnt;
	size_t i;

	n = mos_snprintf(buf, sizeof (buf), "::stacktrace [%s]\n", msg);

	cnt = mos_stacktrace(stack, 32);
	for (i = 0; i < cnt; i++) {
		mos_getsymbolname(stack[i], sname, sizeof (sname));
		n += mos_snprintf(buf + n, sizeof (buf) - n, "\t%s\n", sname);
		if (n >= sizeof (buf))
			break;
	}

	PhidgetLog_log(level, "%s", buf);
}

static MOS_TASK_RESULT
runNetworkLogging(void *arg) {
	PhidgetReturnCode res;
	Phidget_LogLevel ll;
	const char *srcname;
	const char *msg;
	char buf[2048];
	char hdr[128];
	pconf_t *pc;
	size_t n;
	int err;

#ifndef NDEBUG
	const char *file;
#endif

	mos_task_setname("Phidget22 Network Logging Thread");
	logdebug("network logging thread started: 0x%08x", mos_self());

	while (logsrvsock != MOS_INVALID_SOCKET) {
		res = mos_netop_tcp_rpoll(MOS_IOP_IGNORE, &logsrvsock, 1000);
		if (res != 0) {
			if (res != EPHIDGET_TIMEOUT) {
				mos_printef("Failed to poll network logging socket: stopping\n");
				break;
			}
			continue;
		}

		n = sizeof (buf);
		res = mos_netop_udp_recv(MOS_IOP_IGNORE, &logsrvsock, buf, &n);
		if (res != EPHIDGET_OK)
			continue;
		buf[n] = '\0';

		res = pconf_parsejson(&pc, buf, n);
		if (res != EPHIDGET_OK)
			continue;

		/*
		 * V  version
		 * K  pass phrase
		 * F  flags
		 * ll log level
		 * sn srcname
		 * fn filename
		 * ln line number
		 * fc function
		 * mg message
		 */
		srcname = pconf_getstr(pc, NULL, "sn");
		if (srcname == NULL)
			goto badmsg;

		msg = pconf_getstr(pc, NULL, "mg");
		if (msg == NULL)
			goto badmsg;

		ll = pconf_get32(pc, PHIDGET_LOG_INFO, "ll");

#if NDEBUG
		mos_snprintf(hdr, sizeof(hdr), "%#T %" MAX_LVL_TO_STR_LEN "s %s : ", NULL, lvlToStr(ll), srcname);
#else

		file = pconf_getstr(pc, NULL, "fn");

		if (file == NULL)
			mos_snprintf(hdr, sizeof(hdr), "%#T %" MAX_LVL_TO_STR_LEN "s %s : ", NULL, lvlToStr(ll), srcname);
		else
			mos_snprintf(hdr, sizeof(hdr), "%#T %" MAX_LVL_TO_STR_LEN "s %s[%.32s+%d %s()] : ",
				NULL, lvlToStr(ll), srcname, file, pconf_get32(pc, -1, "ln"), pconf_getstr(pc, "()", "fn"));
#endif /* NDEBUG */

#if _WINDOWS
		if (ll & LOGF_DEBUGGER) {
			OutputDebugStringA(hdr);
			OutputDebugStringA(msg);
		}
#endif

		if (ll & LOGF_STDERR) {
			if (stderrf == NULL) {
				err = mos_file_open(MOS_IOP_IGNORE, &stderrf, 0, MOS_FILE_STDERR);
				if (err != 0)
					MOS_TASK_EXIT(err);
			}
			_writelog(stderrf, hdr, msg);
		} else if (logmf) {
			_writelog(logmf, hdr, msg);
		}

badmsg:
		pconf_release(&pc);
	}

	if (logsrvsock != MOS_INVALID_SOCKET)
		mos_netop_udp_closesocket(MOS_IOP_IGNORE, &logsrvsock);

	MOS_TASK_EXIT(0);
}

static PhidgetReturnCode
enableNetworkLogging(mos_sockaddr_t *saddr) {
	PhidgetReturnCode res;

	res = mos_netop_udp_openserversocket(MOS_IOP_IGNORE, &logsrvsock, saddr);
	if (res!= 0) {
		mos_printef("Failed to enable network logging: unable to create socket\n");
		return (res);
	}
	mos_netop_udp_setnonblocking(MOS_IOP_IGNORE, &logsrvsock, 1);
	mos_netop_setrecvbufsize(MOS_IOP_IGNORE, &logsrvsock, 65536);

	res = mos_task_create(NULL, runNetworkLogging, NULL);
	if (res != EPHIDGET_OK) {
		mos_printef("Failed to create network logging task\n");
		mos_netop_udp_closesocket(MOS_IOP_IGNORE, &logsrvsock);
		return (res);
	}

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLog_enableNetwork(const char *addr, int port) {
	PhidgetReturnCode res;
	mos_sockaddr_t saddr;

	/*
	 * We only support logging to localhost.
	 */
	if (addr != NULL)
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));

	memset(&saddr, 0, sizeof (saddr));
	saddr.s4.sin_family = AF_INET;
	inet_pton(AF_INET, "127.0.0.1", &saddr.s4.sin_addr);
	saddr.s4.sin_port = htons(port);

	res = enableNetworkLogging(&saddr);
	return (PHID_RETURN(res));
}

API_PRETURN
PhidgetLog_disableNetwork() {

	if (logsrvsock != MOS_INVALID_SOCKET)
		mos_netop_udp_closesocket(MOS_IOP_IGNORE, &logsrvsock);
	return (EPHIDGET_OK);
}
