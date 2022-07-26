#if defined(SunOS)
#include <sys/varargs.h>
#include <sys/ddi.h>
#include <sys/sunddi.h>
#endif

#include "bsdqueue.h"
#include "mos_os.h"
#include "mos_lock.h"
#include "mos_assert.h"
#include "mos_typeio.h"
#include "mos_iop.h"

#define MOS_IOP_INIT	0x0100		/* initialized */
#define MOS_IOP_FINI	0x0200		/* freed */

/* serialized notice flags */
#define MOS_NOTICE_HAS_SUBIOP	0x0001

struct mos_notice {
	mosiop_t			mn_iop; 	/* I/O operation */
	mosiop_t			mn_subiop;	/* child I/O operation */
	uint16_t			mn_seq;		/* sequence number (in iop) */
	uint16_t			mn_msgbuflen;
	uint32_t			mn_line;
	const char			*mn_file;
	const char			*mn_func;
	char				*mn_msg;
	uint32_t			mn_notice;

	/* if deserialized */
	char				*mn_filebuf;
	size_t				mn_filebufsz;
	char				*mn_funcbuf;
	size_t				mn_funcbufsz;

	MTAILQ_ENTRY(mos_notice)	mn_linkage;		/* notice linkage */
};

struct mos_iop_note {
	char						*min_note;
	MTAILQ_ENTRY(mos_iop_note)	min_linkage;
};

typedef struct {
	size_t		it_lastsub;
	size_t		it_usedlen;
	size_t		it_dstsz;
	size_t		it_count;
	char		*it_dst;
	int			it_err;
} ioptoissconfarg_t;

typedef MTAILQ_HEAD(mos_notice_list, mos_notice) mos_notice_list_t;
typedef MTAILQ_HEAD(mos_note_list, mos_iop_note) mos_note_list_t;

struct mos_iop {
	uint16_t			mi_magic;
	mos_mutex_t			mi_lock;
	mos_note_list_t		mi_notes;	/* list of notes */
	uint16_t			mi_nnotes;	/* count of notes */
	mos_notice_list_t	mi_notice;	/* list of notices */
	uint16_t			mi_nnotice;	/* count of notices */
	int					mi_opcode;	/* user operational code */
	uint16_t			mi_refcnt;	/* reference count */
	uint16_t			mi_flags;
};

static const char *(*mos_notice_localfamily_string_cb)(int);
static const char *(*mos_notice_localfamily_name_cb)(int);
static int mos_notice_localfamily = -1;

int mos_notice_tracing = 0;
int mos_notice_tracing_verbose = 0;

static void
mos_iop_free(mosiop_t mi) {
	mos_iop_note_t *tnote;
	mos_iop_note_t *note;
	mos_notice_t *next;
	mos_notice_t *mn;

	/* free notices and their messages */
	for (mn = MTAILQ_FIRST(&mi->mi_notice); mn != NULL; mn = next) {
		next = MTAILQ_NEXT(mn, mn_linkage);
		MOS_ASSERT(mn->mn_iop == mi); /* verify notice's iop matches us */
		mos_free(mn->mn_msg, mn->mn_msgbuflen);
		if (mn->mn_filebuf != NULL)
			mos_free(mn->mn_filebuf, mn->mn_filebufsz);
		if (mn->mn_funcbuf != NULL)
			mos_free(mn->mn_funcbuf, mn->mn_funcbufsz);
		if (mn->mn_subiop != NULL)
			mos_iop_release(&mn->mn_subiop);
		mos_free(mn, sizeof (*mn));
	}

	for (note = MTAILQ_FIRST(&mi->mi_notes); note != NULL; note = tnote) {
		tnote = MTAILQ_NEXT(note, min_linkage);
		mos_free(note->min_note, mos_strlen(note->min_note) + 1);
		mos_free(note, sizeof (*note));
	}

	mos_mutex_destroy(&mi->mi_lock);

	mos_free(mi, sizeof (*mi));
}

MOSAPI void MOSCConv
mos_iop_retain(mosiop_t mi) {

	mos_mutex_lock(&mi->mi_lock);
	MOS_ASSERT(mi->mi_magic == MOS_IOP_MAGIC);
	MOS_ASSERT((mi->mi_flags & MOS_IOP_INIT) && !(mi->mi_flags & MOS_IOP_FINI));
	mi->mi_refcnt++;
	mos_mutex_unlock(&mi->mi_lock);
}

MOSAPI void MOSCConv
mos_iop_release(mosiop_t *mip) {
	int lastref;

	if (*mip == NULL)
		return;

	mos_mutex_lock(&(*mip)->mi_lock);
	MOS_ASSERT((*mip)->mi_magic == MOS_IOP_MAGIC);
	MOS_ASSERT(((*mip)->mi_flags & MOS_IOP_INIT) && !((*mip)->mi_flags &
	  MOS_IOP_FINI));
	MOS_ASSERT((*mip)->mi_refcnt > 0);
	(*mip)->mi_refcnt--;
	lastref = (*mip)->mi_refcnt == 0;
	if (lastref)
		(*mip)->mi_flags |= MOS_IOP_FINI;
	mos_mutex_unlock(&(*mip)->mi_lock);

	if (lastref)
		mos_iop_free(*mip);

	*mip = NULL;
}

MOSAPI int MOSCConv
mos_iop_addnotev(mosiop_t iop, const char *fmt, ...) {
	mos_iop_note_t *lnote;
	mos_iop_note_t *note;
	uint32_t len;
	va_list va;

	va_start(va, fmt);

	note = mos_malloc(sizeof (*note));
	mos_vasprintf(&note->min_note, &len, fmt, va);
	va_end(va);

	mos_mutex_lock(&iop->mi_lock);

	MOS_ASSERT(iop->mi_magic == MOS_IOP_MAGIC);
	MOS_ASSERT((iop->mi_flags & MOS_IOP_INIT) && !(iop->mi_flags &
	  MOS_IOP_FINI));

	MTAILQ_FOREACH(lnote, &iop->mi_notes, min_linkage) {
		if (mos_strcmp(lnote->min_note, note->min_note) == 0) {
			mos_mutex_unlock(&iop->mi_lock);
			mos_free(note->min_note, mos_strlen(note->min_note) + 1);
			mos_free(note, sizeof (*note));
			return (MOSN_EXIST);
		}
	}
	MTAILQ_INSERT_TAIL(&iop->mi_notes, note, min_linkage);
	iop->mi_nnotes++;

	mos_mutex_unlock(&iop->mi_lock);

	return (0);
}

MOSAPI int MOSCConv
mos_iop_walknotes(mosiop_t mi, void(*cb)(const mos_iop_note_t *, void *,
  size_t), void *arg, size_t sub) {
	mos_iop_note_t *note;

	mos_mutex_lock(&mi->mi_lock);
	MOS_ASSERT(mi->mi_magic == MOS_IOP_MAGIC);
	MOS_ASSERT((mi->mi_flags & MOS_IOP_INIT) && !(mi->mi_flags & MOS_IOP_FINI));
	for (note = MTAILQ_FIRST(&mi->mi_notes); note != NULL; ) {
		mos_mutex_unlock(&mi->mi_lock);
		cb(note, arg, sub);
		mos_mutex_lock(&mi->mi_lock);

		note = MTAILQ_NEXT(note, min_linkage);
	}
	mos_mutex_unlock(&mi->mi_lock);

	return (0);
}

MOSAPI int MOSCConv
mos_iop_addnotice(mosiop_t iop, mosiop_t subiop, uint32_t notice,
  const char *const file, int line, const char *func, const char *fmt, ...) {
	va_list va;
	int res;

	va_start(va, fmt);
	res = mos_iop_vaddnotice(iop, subiop, notice, file, line, func, fmt, va);
	va_end(va);

	return (res);
}

MOSAPI int MOSCConv
mos_iop_vaddnotice(mosiop_t mi, mosiop_t subiop, uint32_t notice,
  const char *const file, int line, const char *func, const char *fmt,
  va_list va) {
	uint32_t msgbuflen;
	mos_notice_t *mn;
	char *msg;

	/* format message */
	mos_vasprintf(&msg, &msgbuflen, fmt, va);

	/*
	 * Show notices if mos_notice_tracing is enabled, but don't show
	 * ignored notices unless mos_notice_tracing_verbose is defined.
	 */
	if (mos_notice_tracing_verbose || (mi != NULL && mos_notice_tracing))
		mos_printf("%p %s+%d %s: %s%s\n", mi, file, line,
		  mos_notice_string(notice), msg, mi == NULL ? " (ignored)" : "");

	if (mi == NULL) {
		mos_free(msg, msgbuflen);
		return (notice);
	}

	/* allocate notice buffer big enough to have message follow */
	mn = mos_malloc(sizeof (*mn));

	mn->mn_iop = mi;
	mn->mn_subiop = subiop;
	if (subiop != NULL)
		mos_iop_retain(subiop);
	mn->mn_file = file;
	mn->mn_line = line;
	mn->mn_func = func;
	mn->mn_msgbuflen = (uint16_t)msgbuflen;
	mn->mn_msg = msg;
	mn->mn_notice = notice;

	mn->mn_filebuf = NULL;
	mn->mn_filebufsz = 0;
	mn->mn_funcbuf = NULL;
	mn->mn_funcbufsz = 0;

	memset(&mn->mn_linkage, 0, sizeof (mn->mn_linkage));

	mos_mutex_lock(&mi->mi_lock);
	MOS_ASSERT(mi->mi_magic == MOS_IOP_MAGIC);
	MOS_ASSERT((mi->mi_flags & MOS_IOP_INIT) && !(mi->mi_flags & MOS_IOP_FINI));
	MTAILQ_INSERT_HEAD(&mi->mi_notice, mn, mn_linkage);
	mn->mn_seq = mi->mi_nnotice++;
	mos_mutex_unlock(&mi->mi_lock);

	return (notice);
}

MOSAPI void MOSCConv
mos_iop_walknotices(mosiop_t mi, void(*cb)(const mos_notice_t *, void *,
  size_t), void *arg, size_t sub) {
	mos_notice_t *mn;

	mos_mutex_lock(&mi->mi_lock);
	MOS_ASSERT(mi->mi_magic == MOS_IOP_MAGIC);
	MOS_ASSERT((mi->mi_flags & MOS_IOP_INIT) && !(mi->mi_flags & MOS_IOP_FINI));
	for (mn = MTAILQ_FIRST(&mi->mi_notice); mn != NULL; ) {

		mos_mutex_unlock(&mi->mi_lock);

		MOS_ASSERT(mn->mn_iop == mi); /* verify notice's iop matches us */

		/* recurse through subiop, if set */
		if (mn->mn_subiop != NULL)
			mos_iop_walknotices(mn->mn_subiop, cb, arg, sub + 1);

		cb(mn, arg, sub);

		mos_mutex_lock(&mi->mi_lock);

		mn = MTAILQ_NEXT(mn, mn_linkage);
	}
	mos_mutex_unlock(&mi->mi_lock);
}

static void
issconfstuffslashes(char *dst, uint32_t dstsz, const char *in) {
	const char *s;
	char *d;

	for (s = in, d = dst; (uint32_t)(d - dst) < (dstsz - 2); s++) {
		if (*s == '\\') {
			*d++ = '\\';
			*d++ = '\\';
		} else {
			*d++ = *s;
		}
		if (*s == '\0')
			return;
	}

	dst[dstsz - 1] = '\0';
}

static void
issconfprepmessage(char *dst, uint32_t dstsz, const char *in) {
	const char *s;
	uint32_t n;
	char *d;

	/*
	 * Replace double-quotes with single-quotes because we render fields using
	 * outer doubles (e.g.: mosmsg=\"%s\";).
	 */
	for (n = 0, s = in, d = dst; *s != '\0' && n < dstsz; n++, s++, d++) {
		if (*s == '"')
			*d = '\'';
		else
			*d = *s;
	}
	*d = '\0';
}

static void
ioptoissconf_cb(const mos_notice_t *mn, void *iarg, size_t sub) {
	ioptoissconfarg_t *arg;
	const char *errstr;
	const char *filenm;
	char file[1024];
	char tmp[2048];
	char msg[2048];

	arg = iarg;

	MOS_ASSERT(mn != NULL);

	/*
	 * If we've already filled the buffer there's no point in continuing.
	 */
	if (arg->it_usedlen >= (arg->it_dstsz - 1)) {
		arg->it_err = MOSN_NOSPC;
		return;
	}

	if (sub > arg->it_lastsub) {
		/*
		 * Begin new subiop blocks until we reach the correct level.
		 */
		for (; sub != arg->it_lastsub; arg->it_lastsub++) {
			mos_snprintf(tmp, sizeof (tmp), " siop%zu { ", arg->it_count);
			arg->it_count++;
			arg->it_usedlen = mos_strlcat(arg->it_dst, tmp, arg->it_dstsz);
		}
	} else if (sub < arg->it_lastsub) {
		/*
		 * End subiop blocks until we reach the correct level.
		 */
		for (; sub != arg->it_lastsub; arg->it_lastsub--)
			arg->it_usedlen = mos_strlcat(arg->it_dst, " } ", arg->it_dstsz);
	}

	if (arg->it_usedlen >= (arg->it_dstsz - 1)) {
		arg->it_err = MOSN_NOSPC;
		return;
	}

	errstr = mos_notice_string(mos_notice_get_notice(mn));
	filenm = mos_basename(mos_notice_get_file(mn));
	issconfstuffslashes(file, sizeof (file), filenm);
	issconfprepmessage(msg, sizeof (msg), mos_notice_get_message(mn));

	if (errstr != NULL) {
		mos_snprintf(tmp, sizeof (tmp),
		  "%zu { file=\"%s\"; line=%d; func=\"%s\"; mosmsg=\"%s\"; "
		  "msg=[%zu]%s; } ", arg->it_count, file, mos_notice_get_line(mn),
		  mos_notice_get_func(mn), errstr, mos_strlen(msg), msg);
	} else {
		mos_snprintf(tmp, sizeof (tmp),
		  "%zu { file=\"%s\"; line=%d; func=\"%s\"; mosmsg=\"%d\"; "
		  "msg=[%zu]%s; } ", arg->it_count, file, mos_notice_get_line(mn),
		  mos_notice_get_func(mn), mos_notice_get_notice(mn), mos_strlen(msg),
		  msg);
	}
	arg->it_usedlen = mos_strlcat(arg->it_dst, tmp, arg->it_dstsz);
	arg->it_count++;

	if (arg->it_usedlen >= (arg->it_dstsz - 1))
		arg->it_err = MOSN_NOSPC;
}

int
mosiop_to_issconf(mosiop_t iop, int err, const char *summary, char *ibuf,
  uint32_t ibufsz) {
	ioptoissconfarg_t arg;
	const char *moscode;
	const char *mosmsg;
	const char *s;
	uint32_t b;

	if (ibuf == NULL)
		return (0);

	if (summary == NULL)
		s = "no summary";
	else
		s = summary;

	moscode = mos_notice_name(err);
	mosmsg = mos_notice_string(err);

	b = (uint32_t)mos_snprintf(ibuf, ibufsz,
	  "error { moscode=\"%s\"; mosmsg=\"%s\"; summary=[%zu]%s; iop { ",
	  moscode, mosmsg, mos_strlen(s), s);
	if (b >= (ibufsz - 1))
		return (MOSN_NOSPC);

	memset(&arg, 0, sizeof (arg));

	arg.it_dst = ibuf;
	arg.it_dstsz = ibufsz;
	arg.it_usedlen = (uint32_t)b;
	arg.it_err = 0;

	mos_iop_walknotices(iop, ioptoissconf_cb, &arg, 0);

	if (arg.it_err != 0)
		return (arg.it_err);

	arg.it_usedlen = mos_strlcat(ibuf, "} } ", ibufsz);

	if (arg.it_usedlen >= ibufsz)
		return (MOSN_NOSPC);

	return (0);
}

MOSAPI const char * MOSCConv
mos_iop_note_getnote(const mos_iop_note_t *note) {

	return (note->min_note);
}

MOSAPI const char * MOSCConv
mos_notice_get_file(const mos_notice_t *mn) {

	return (mn->mn_file);
}

MOSAPI const char * MOSCConv
mos_notice_get_func(const mos_notice_t *mn) {

	return (mn->mn_func);
}

MOSAPI const char * MOSCConv
mos_notice_get_message(const mos_notice_t *mn) {

	return (mn->mn_msg);
}

MOSAPI int MOSCConv
mos_notice_get_line(const mos_notice_t *mn) {

	return (mn->mn_line);
}

MOSAPI uint32_t MOSCConv
mos_notice_get_notice(const mos_notice_t *mn) {

	return (mn->mn_notice);
}

MOSAPI int MOSCConv
mos_notice_get_sequence(const mos_notice_t *mn) {

	return (mn->mn_seq);
}

MOSAPI mosiop_t MOSCConv
mos_notice_get_iop(const mos_notice_t *mn) {

	return (mn->mn_iop);
}

MOSAPI mosiop_t MOSCConv
mos_notice_get_subiop(const mos_notice_t *mn) {

	return (mn->mn_subiop);
}

MOSAPI const char * MOSCConv
mos_notice_name(int notice) {

	if (!MOSN_ISERROR(notice)) {
		switch (notice) {
		case 0:
			return ("SUCCESS");
		default:
			return ("UNKNOWN");
		}
	}

	if (MOSN_FAMILY(notice) == mos_notice_localfamily) {
		if (mos_notice_localfamily_name_cb != NULL)
			return (mos_notice_localfamily_name_cb(notice));
		return ("LOCAL");
	}

	switch (MOSN_NOTICE(notice)) {
	case MOSN_N_PERM:
		return ("MOSN_PERM");
	case MOSN_N_NOENT:
		return ("MOSN_NOENT");
	case MOSN_N_TIMEDOUT:
		return ("MOSN_TIMEDOUT");
	case MOSN_N_INTR:
		return ("MOSN_INTR");
	case MOSN_N_IO:
		return ("MOSN_IO");
	case MOSN_N_MEM:
		return ("MOSN_MEM");
	case MOSN_N_ACCESS:
		return ("MOSN_ACCESS");
	case MOSN_N_FAULT:
		return ("MOSN_FAULT");
	case MOSN_N_BUSY:
		return ("MOSN_BUSY");
	case MOSN_N_EXIST:
		return ("MOSN_EXIST");
	case MOSN_N_NOTDIR:
		return ("MOSN_NOTDIR");
	case MOSN_N_ISDIR:
		return ("MOSN_ISDIR");
	case MOSN_N_INVAL:
		return ("MOSN_INVAL");
	case MOSN_N_NFILE:
		return ("MOSN_NFILE");
	case MOSN_N_MFILE:
		return ("MOSN_MFILE");
	case MOSN_N_NOSPC:
		return ("MOSN_NOSPC");
	case MOSN_N_FBIG:
		return ("MOSN_FBIG");
	case MOSN_N_ROFS:
		return ("MOSN_ROFS");
	case MOSN_N_RO:
		return ("MOSN_RO");
	case MOSN_N_NOSUP:
		return ("MOSN_NOSUP");
	case MOSN_N_INVALARG:
		return ("MOSN_INVALARG");
	case MOSN_N_AGAIN:
		return ("MOSN_AGAIN");
	case MOSN_N_NEVENT:
		return ("MOSN_NEVENT");
	case MOSN_N_INCONST:
		return ("MOSN_INCONST");
	case MOSN_N_ADDR:
		return ("MOSN_ADDR");
	case MOSN_N_NOTEMPTY:
		return ("MOSN_NOTEMPTY");
	case MOSN_N_DUP:
		return ("MOSN_DUP");
	case MOSN_N_ERR:
		return ("MOSN_ERR");
	case MOSN_N_HASH:
		return ("MOSN_HASH");
	case MOSN_N_CONTENT:
		return ("MOSN_CONTENT");
	case MOSN_N_EOF:
		return ("MOSN_EOF");
	case MOSN_N_POLICY:
		return ("MOSN_POLICY");
	case MOSN_N_LICENSE:
		return ("MOSN_LICENSE");
	case MOSN_N_TASTE:
		return ("MOSN_TASTE");
	case MOSN_N_CONNREF:
		return ("MOSN_CONNREF");
	case MOSN_N_CONNFAIL:
		return ("MOSN_CONNFAIL");
	case MOSN_N_BADCRED:
		return ("MOSN_BADCRED");
	case MOSN_N_BADKEY:
		return ("MOSN_BADKEY");
	case MOSN_N_SIGNATURE:
		return ("MOSN_SIGNATURE");
	case MOSN_N_NODEV:
		return ("MOSN_NODEV");
	case MOSN_N_PIPE:
		return ("MOSN_PIPE");
	case MOSN_N_REVOKED:
		return ("MOSN_REVOKED");
	case MOSN_N_BADTIME:
		return ("MOSN_BADTIME");
	case MOSN_N_RESOLV:
		return ("MOSN_RESOLV");
	case MOSN_N_NETUNAVAIL:
		return ("MOSN_NETUNAVAIL");
	case MOSN_N_CONNRESET:
		return ("MOSN_CONNRESET");
	case MOSN_N_CONNABORTED:
		return ("MOSN_CONNABORTED");
	case MOSN_N_HOSTUNREACH:
		return ("MOSN_HOSTUNREACH");
	case MOSN_N_HOSTDOWN:
		return ("MOSN_HOSTDOWN");
	case MOSN_N_WRONGDEV:
		return ("MOSN_WRONGDEV");
	case MOSN_N_UNKNOWNVAL:
		return ("MOSN_UNKNOWNVAL");
	case MOSN_N_NOTATTACHED:
		return ("MOSN_NOTATTACHED");
	case MOSN_N_INVALPACKET:
		return ("MOSN_INVALPACKET");
	case MOSN_N_2BIG:
		return ("MOSN_2BIG");
	case MOSN_N_BADVER:
		return ("MOSN_BADVER");
	case MOSN_N_CLOSED:
		return ("MOSN_CLOSED");
	case MOSN_N_NOTCONFIGURED:
		return ("MOSN_NOTCONFIGURED");
	case MOSN_N_KEEPALIVE:
		return ("MOSN_KEEPALIVE");
	}
	return ("MOSN_UNKNOWN");
}

MOSAPI const char * MOSCConv
mos_notice_string(int notice) {

	if (!MOSN_ISERROR(notice)) {
		switch (notice) {
		case 0:
			return ("No error (0)");
		case -1:
			return ("Unknown error (-1)");
		default:
			return ("Unknown non-error notice");
		}
	}

	if (MOSN_FAMILY(notice) == mos_notice_localfamily) {
		if (mos_notice_localfamily_string_cb != NULL)
			return (mos_notice_localfamily_string_cb(notice));
		else
			return ("unknown local family error");
	}

	switch (MOSN_NOTICE(notice)) {
	case MOSN_N_PERM:
		return ("Permission Denied");
	case MOSN_N_NOENT:
		return ("No Such Entity");
	case MOSN_N_TIMEDOUT:
		return ("Timed Out");
	case MOSN_N_INTR:
		return ("Operation Interrupted");
	case MOSN_N_IO:
		return ("IO Failure");
	case MOSN_N_MEM:
		return ("Memory Failure");
	case MOSN_N_ACCESS:
		return ("Access Denied");
	case MOSN_N_FAULT:
		return ("Invalid Address");
	case MOSN_N_BUSY:
		return ("Busy");
	case MOSN_N_EXIST:
		return ("Object Exists");
	case MOSN_N_NOTDIR:
		return ("Object Not A Directory");
	case MOSN_N_ISDIR:
		return ("Object Is A Directory");
	case MOSN_N_INVAL:
		return ("Invalid");
	case MOSN_N_NFILE:
		return ("Too Many Files");
	case MOSN_N_MFILE:
		return ("Too Many Files");
	case MOSN_N_NOSPC:
		return ("Not Enough Space");
	case MOSN_N_FBIG:
		return ("File Too Large");
	case MOSN_N_ROFS:
		return ("Read Only Filesystem");
	case MOSN_N_RO:
		return ("Read Only");
	case MOSN_N_NOSUP:
		return ("Not Supported");
	case MOSN_N_INVALARG:
		return ("Invalid Argument");
	case MOSN_N_AGAIN:
		return ("Try Again");
	case MOSN_N_NEVENT:
		return ("Not an Event");
	case MOSN_N_INCONST:
		return ("Inconsistent State Encountered");
	case MOSN_N_ADDR:
		return ("Invalid Address");
	case MOSN_N_NOTEMPTY:
		return ("Not Empty");
	case MOSN_N_DUP:
		return ("Duplicate");
	case MOSN_N_ERR:
		return ("Error");
	case MOSN_N_HASH:
		return ("Hash Failure");
	case MOSN_N_CONTENT:
		return ("Invalid Content");
	case MOSN_N_EOF:
		return ("End of File");
	case MOSN_N_POLICY:
		return ("Policy Failure");
	case MOSN_N_LICENSE:
		return ("License Check");
	case MOSN_N_TASTE:
		return ("Bad Taste");
	case MOSN_N_CONNREF:
		return ("Connection Refused");
	case MOSN_N_CONNFAIL:
		return ("Connection Failed");
	case MOSN_N_BADCRED:
		return ("Bad Credentials");
	case MOSN_N_BADKEY:
		return ("Bad Key");
	case MOSN_N_SIGNATURE:
		return ("Signature");
	case MOSN_N_NODEV:
		return ("No Such Device");
	case MOSN_N_PIPE:
		return ("Broken Pipe");
	case MOSN_N_REVOKED:
		return ("Revoked");
	case MOSN_N_BADTIME:
		return ("Bad Time");
	case MOSN_N_RESOLV:
		return ("Name Resolution Failure");
	case MOSN_N_NETUNAVAIL:
		return ("Network Unavailable");
	case MOSN_N_CONNRESET:
		return ("Connection Reset");
	case MOSN_N_CONNABORTED:
		return ("Connection Aborted");
	case MOSN_N_HOSTUNREACH:
		return ("No Route To Host");
	case MOSN_N_HOSTDOWN:
		return ("Host is Down");
	case MOSN_N_WRONGDEV:
		return ("Wrong Device");
	case MOSN_N_UNKNOWNVAL:
		return ("Unknown or Invalid Value");
	case MOSN_N_NOTATTACHED:
		return ("Device not Attached");
	case MOSN_N_INVALPACKET:
		return ("Invalid or Unexpected Packet");
	case MOSN_N_2BIG:
		return ("Argument List Too Long");
	case MOSN_N_BADVER:
		return ("Bad Version");
	case MOSN_N_CLOSED:
		return ("Closed");
	case MOSN_N_NOTCONFIGURED:
		return ("Channel Not Fully Configured");
	case MOSN_N_KEEPALIVE:
		return ("Keep Alive");
	default:
		return ("Unknown Notice");
	}
}

MOSAPI mosiop_t MOSCConv
mos_iop_alloc(void) {
	mosiop_t mi;

	mi = mos_malloc(sizeof (*mi));

	mos_mutex_init(&mi->mi_lock);
	MTAILQ_INIT(&mi->mi_notice);
	MTAILQ_INIT(&mi->mi_notes);
	mi->mi_magic = MOS_IOP_MAGIC;
	mi->mi_nnotes = 0;
	mi->mi_nnotice = 0;
	mi->mi_refcnt = 1;
	mi->mi_opcode = 0;
	mi->mi_flags = MOS_IOP_INIT;

	return (mi);
}

MOSAPI void MOSCConv
mos_notice_addfamily(int family, const char *(*family_notice_name_cb)(int),
  const char *(*family_notice_string_cb)(int)) {

	MOS_ASSERT(family != MOSN_F_SYSTEM);
	MOS_ASSERT(family != MOSN_F_PRIVATE);
	MOS_ASSERT(family != MOSN_F_USER);

	if (family == mos_notice_localfamily)
		return;

	/* limit of one family for now */
	if (mos_notice_localfamily != 0)
		MOS_PANIC("limit of one added notice family");

	mos_notice_localfamily = family;
	mos_notice_localfamily_name_cb = family_notice_name_cb;
	mos_notice_localfamily_string_cb = family_notice_string_cb;
}

static int
mos_iop_note_rw(mosiop_t iop, int dowrite, uint8_t *tbuf, uint32_t tbuflen,
  uint32_t *indexp, mos_iop_note_t **notep) {
	mos_iop_note_t *note;
	uint16_t len;
	int res;

	len = 0;

	if (!dowrite) {
		/* allocate note to read into */
		*notep = mos_malloc(sizeof (**notep));
	}

	note = *notep;

	if (dowrite)
		len = (uint16_t)mos_strlen(note->min_note) + 1;
	TYPEIO_CK(MOS_UINT16_RW(dowrite, tbuf, tbuflen, indexp, &len));
	if (!dowrite)
		note->min_note = mos_malloc(len);

	TYPEIO_CK(MOS_UINT8_ARRAY_RW(dowrite, tbuf, tbuflen, indexp,
	  note->min_note, len));

	return (0);
}

static int
mos_notice_rw(mosiop_t iop, int dowrite, uint8_t *tbuf, uint32_t tbuflen,
  uint32_t *indexp, mos_notice_t **mnp) {
	mos_notice_t *mn;
	uint32_t flags;
	uint16_t sz;
	int res;

	union {
		const void *cv;
		void *v;
	} deconstify;

	flags = 0;
	sz = 0;

	if (!dowrite) {
		/* allocate notice to read into */
		*mnp = mos_malloc(sizeof (**mnp));
		memset(*mnp, 0, sizeof (**mnp));
	}

	mn = *mnp;

	/* r/w flags and subiop, if specified */
	if (dowrite) {
		/* initialize flags to write */
		if (mn->mn_subiop != NULL)
			flags = MOS_NOTICE_HAS_SUBIOP;
	}
	TYPEIO_CK(MOS_UINT16_RW(dowrite, tbuf, tbuflen, indexp, &flags));
	if (flags & MOS_NOTICE_HAS_SUBIOP)
		TYPEIO_CK(mos_iop_rw(iop, dowrite, tbuf, tbuflen, indexp,
		  &mn->mn_subiop));
	TYPEIO_CK(MOS_UINT16_RW(dowrite, tbuf, tbuflen, indexp, &mn->mn_msgbuflen));
	if (!dowrite)
		mn->mn_msg = mos_malloc(mn->mn_msgbuflen);
	TYPEIO_CKGOTO(MOS_UINT8_ARRAY_RW(dowrite, tbuf, tbuflen, indexp, mn->mn_msg,
	  mn->mn_msgbuflen));
	TYPEIO_CKGOTO(MOS_UINT32_RW(dowrite, tbuf, tbuflen, indexp, &mn->mn_line));
	TYPEIO_CKGOTO(MOS_UINT32_RW(dowrite, tbuf, tbuflen, indexp,
	  &mn->mn_notice));

	/* r/w file */
	if (dowrite)
		sz = (uint16_t)mos_strlen(mn->mn_file);
	TYPEIO_CKGOTO(MOS_UINT16_RW(dowrite, tbuf, tbuflen, indexp, &sz));
	if (!dowrite) {
		mn->mn_filebufsz = sz + 1;
		mn->mn_filebuf = mos_malloc(mn->mn_filebufsz);
		deconstify.v = mn->mn_filebuf;
	} else {
		deconstify.cv = mn->mn_file;
	}
	TYPEIO_CKGOTO(MOS_UINT8_ARRAY_RW(dowrite, tbuf, tbuflen, indexp,
	  deconstify.v, sz));
	if (!dowrite) {
		mn->mn_filebuf[sz] = '\0';
		mn->mn_file = mn->mn_filebuf;
	}

	/* r/w func */
	if (dowrite)
		sz = (uint16_t)mos_strlen(mn->mn_func);
	TYPEIO_CKGOTO(MOS_UINT16_RW(dowrite, tbuf, tbuflen, indexp, &sz));
	if (!dowrite) {
		mn->mn_funcbufsz = sz + 1;
		mn->mn_funcbuf = mos_malloc(mn->mn_funcbufsz);
		deconstify.v = mn->mn_funcbuf;
	} else {
		deconstify.cv = mn->mn_func;
	}
	TYPEIO_CKGOTO(MOS_UINT8_ARRAY_RW(dowrite, tbuf, tbuflen, indexp,
	  deconstify.v, sz));
	if (!dowrite) {
		mn->mn_funcbuf[sz] = '\0';
		mn->mn_func = mn->mn_funcbuf;
	}

done:

	/* if we failed during reading, free allocations made thus far */
	if (res != 0 && !dowrite) {
		if (mn->mn_filebuf != NULL)
			mos_free(mn->mn_filebuf, mn->mn_filebufsz);
		if (mn->mn_funcbuf != NULL)
			mos_free(mn->mn_funcbuf, mn->mn_funcbufsz);
		if (mn->mn_msg != NULL)
			mos_free(mn->mn_msg, mn->mn_msgbuflen);
		mos_free(mn, sizeof (*mn));
	}

	return (res);
}

MOSAPI int MOSCConv
mos_iop_rw(mosiop_t iop, int dowrite, uint8_t *tbuf, uint32_t tbuflen,
  uint32_t *indexp, mosiop_t *siopp) {
	mos_iop_note_t *note;
	mos_notice_t *mn;
	uint16_t magic;
	mosiop_t siop;
	uint16_t i;
	int res;

	if (!dowrite)
		*siopp = mos_iop_alloc();

	siop = *siopp;

	/*
	 * We don't bother locking when writing since the caller's iop
	 * isn't going away, and anything that gets added after we start
	 * is too late.  If reading, the iop's not visible to anyone
	 * until we return it anyway, so locking is unnecessary.
	 */

	/* add/verify magic */
	magic = MOS_IOP_MAGIC;
	TYPEIO_CKGOTO(MOS_UINT16_RW(dowrite, tbuf, tbuflen, indexp, &magic));
	if (!dowrite && magic != MOS_IOP_MAGIC)
		return (MOS_ERROR(iop, MOSN_INVAL,
		  "iop magic mismatch (got 0x%x, expecting 0x%x)", magic,
		  MOS_IOP_MAGIC));

	TYPEIO_CKGOTO(MOS_UINT16_RW(dowrite, tbuf, tbuflen, indexp,
	  &siop->mi_nnotes));

/* NOTES */
	/* if writing, initialize note to start of walk */
	if (dowrite)
		note = MTAILQ_FIRST(&siop->mi_notes);
	else
		note = NULL;

	for (i = 0; i < siop->mi_nnotes; i++) {
		/*
		 * R/W a single note.
		 */
		res = mos_iop_note_rw(iop, dowrite, tbuf, tbuflen, indexp, &note);
		if (res != 0) {
			MOS_ERROR(iop, res, "failed to handle note %u", i);
			goto done;
		}

		/*
		 * Initialize containing iop-related fields.
		 */
		if (!dowrite) {
			MTAILQ_INSERT_TAIL(&siop->mi_notes, note, min_linkage);
		} else {
			note = MTAILQ_NEXT(note, min_linkage);
		}
	}

/* NOTICES */
	TYPEIO_CKGOTO(MOS_UINT16_RW(dowrite, tbuf, tbuflen, indexp,
	  &siop->mi_nnotice));

	/* if writing, initialize mn to start of walk */
	if (dowrite)
		mn = MTAILQ_FIRST(&siop->mi_notice);
	else
		mn = NULL;

	for (i = 0; i < siop->mi_nnotice; i++) {
		/*
		 * R/W a single notice.  If reading, mn will have been allocated and
		 * initialized with everything but mn_seq etc. which happens next.
		 */
		res = mos_notice_rw(iop, dowrite, tbuf, tbuflen, indexp, &mn);
		if (res != 0) {
			MOS_ERROR(iop, res, "failed to handle notice %u", i);
			goto done;
		}

		/*
		 * Initialize containing iop-related fields.
		 */
		if (!dowrite) {
			mn->mn_seq = i;
			mn->mn_iop = siop;
			MTAILQ_INSERT_TAIL(&siop->mi_notice, mn, mn_linkage);
		} else {
			mn = MTAILQ_NEXT(mn, mn_linkage);
		}
	}

done:

	if (res != 0 && !dowrite)
		mos_iop_release(siopp);

	return (res);
}

MOSAPI uint16_t MOSCConv
mos_iop_getnoticecount(mosiop_t mi) {

	return (mi->mi_nnotice);
}

MOSAPI uint32_t MOSCConv
mos_iop_getlastnotice(mosiop_t mi) {
	mos_notice_t *mn;
	uint32_t notice;

	notice = 0;

	for (mn = MTAILQ_FIRST(&mi->mi_notice); mn != NULL; mn = MTAILQ_NEXT(mn,
	  mn_linkage))
		notice = mn->mn_notice;

	return (notice);
}

/*
 * Sets the opcode if the opcode has not already been set.
 */
MOSAPI void MOSCConv
mos_iop_setopcodens(mosiop_t mi, int opcode) {

	if (mi == NULL)
		return;

	mos_mutex_lock(&mi->mi_lock);
	MOS_ASSERT(mi->mi_magic == MOS_IOP_MAGIC);
	MOS_ASSERT((mi->mi_flags & MOS_IOP_INIT) && !(mi->mi_flags & MOS_IOP_FINI));
	if (mi->mi_opcode == 0)
		mi->mi_opcode = opcode;
	mos_mutex_unlock(&mi->mi_lock);
}

MOSAPI void MOSCConv
mos_iop_setopcode(mosiop_t mi, int opcode) {

	if (mi == NULL)
		return;

	mos_mutex_lock(&mi->mi_lock);
	MOS_ASSERT(mi->mi_magic == MOS_IOP_MAGIC);
	MOS_ASSERT((mi->mi_flags & MOS_IOP_INIT) && !(mi->mi_flags & MOS_IOP_FINI));
	mi->mi_opcode = opcode;
	mos_mutex_unlock(&mi->mi_lock);
}

MOSAPI int MOSCConv
mos_iop_getopcode(mosiop_t mi) {
	int ret;

	if (mi == NULL)
		return (0);

	mos_mutex_lock(&mi->mi_lock);
	MOS_ASSERT(mi->mi_magic == MOS_IOP_MAGIC);
	MOS_ASSERT((mi->mi_flags & MOS_IOP_INIT) && !(mi->mi_flags & MOS_IOP_FINI));
	ret = mi->mi_opcode;
	mos_mutex_unlock(&mi->mi_lock);
	return (ret);
}

int
mos_notice_isnetworkrelated(int err) {

	switch(err) {
	case MOSN_PIPE:
	case MOSN_CONNREF:
	case MOSN_CONNFAIL:
	case MOSN_TIMEDOUT:
	case MOSN_RESOLV:
	case MOSN_NETUNAVAIL:
	case MOSN_CONNRESET:
	case MOSN_CONNABORTED:
	case MOSN_HOSTUNREACH:
	case MOSN_HOSTDOWN:
		return (1);
	}

	return (0);
}
