#ifndef _KV_H_
#define _KV_H_

#include "mos/mos_os.h"
#include "mos/bsdqueue.h"
#include "mos/mos_iop.h"

#define KV_FOREACH(e, kv)	MTAILQ_FOREACH((e), &kv->list, link)

typedef struct kvent kvent_t;
typedef struct kv kv_t;

typedef MTAILQ_HEAD(kv_list, kv) kv_list_t;
typedef MTAILQ_HEAD(kv_elist, kvent) kv_elist_t;

struct kv {
	int					magic;
	kv_elist_t			list;
	uint32_t			cnt;
	uint32_t			flags;
	kv_t				*_curns;	/* current namespace */
	kv_list_t			namespaces;
	MTAILQ_ENTRY(kv) 	nslink;
	char				*kvnamespace;
};

MOSAPI int MOSCConv kv_read(kv_t **, mosiop_t, const char *);
MOSAPI int MOSCConv kv_vread(kv_t **, mosiop_t, const char *, ...) MOS_PRINTF_LIKE(3, 4);
MOSAPI int MOSCConv kv_vawrite(kv_t *, mosiop_t, const char *, va_list);
MOSAPI int MOSCConv kv_vwrite(kv_t *, mosiop_t, const char *, ...) MOS_PRINTF_LIKE(3, 4);
MOSAPI int MOSCConv kv_write(kv_t *, mosiop_t, const char *);
MOSAPI int MOSCConv kv_loadf(kv_t *, mosiop_t, const char *);

MOSAPI int MOSCConv newkv(kv_t **);
MOSAPI int MOSCConv newkv_ns(kv_t **, const char *);
MOSAPI int MOSCConv newkvbuf(kv_t **, mosiop_t, const char *);
MOSAPI void MOSCConv kvfree(kv_t **);
MOSAPI void MOSCConv kvsetcaseinsensitive(kv_t *, int);
MOSAPI int MOSCConv kvcount(kv_t *);
MOSAPI int MOSCConv kvgetentity(kv_t *, mosiop_t, const char *, kvent_t **);
MOSAPI int MOSCConv kvadd(kv_t *, mosiop_t, const char *, const char *);
MOSAPI int MOSCConv kvvadd(kv_t *, mosiop_t, const char *, const char *, ...) MOS_PRINTF_LIKE(4, 5);
MOSAPI int MOSCConv kvvaadd(kv_t *, mosiop_t, const char *, const char *, va_list);
MOSAPI int MOSCConv kvset(kv_t *, mosiop_t, const char *, const char *);
MOSAPI int MOSCConv kvvset(kv_t *, mosiop_t, const char *, const char *, ...) MOS_PRINTF_LIKE(4, 5);
MOSAPI int MOSCConv kvvaset(kv_t *, mosiop_t, const char *, const char *, va_list);
MOSAPI int MOSCConv kvremove(kv_t *, mosiop_t, const char *);
MOSAPI int MOSCConv kvgetstr(kv_t *, mosiop_t, const char *, char *, uint32_t);
MOSAPI const char * MOSCConv kvgetstrc(kv_t *, const char *, const char *);
MOSAPI uint64_t MOSCConv kvgetui64(kv_t *, const char *, uint64_t);
MOSAPI uint32_t MOSCConv kvgetui32(kv_t *, const char *, uint32_t);
MOSAPI int64_t MOSCConv kvgeti64(kv_t *, const char *, int64_t);
MOSAPI int32_t MOSCConv kvgeti32(kv_t *, const char *, int32_t);
MOSAPI int MOSCConv kvgetbool(kv_t *, const char *, int);
MOSAPI int MOSCConv kvhasvalue(kv_t *, const char *);
MOSAPI const char * MOSCConv kvgetnamespace(kv_t *);
MOSAPI int MOSCConv kvgetnamespacekv(kv_t *, const char *ns, kv_t **kv);
MOSAPI int MOSCConv kvaddnamespacekv(kv_t *, mosiop_t, kv_t *kv);

struct kvent {
	int8_t					type;
	char					*key;
	char					*val;
	MTAILQ_ENTRY(kvent) link;
};

#define MOSKV_MAXKEYLEN	128
#define MOSKV_MAXVALLEN	32768

#define MOSKV_INT	1
#define MOSKV_DBL	2
#define MOSKV_STR	3
#define MOSKV_BOOL	4
#define MOSKV_DATE	5
#define MOSKV_TKN	6

MOSAPI int MOSCConv mkkvent(kvent_t **, const char *);
MOSAPI int MOSCConv mkkvent_str(kvent_t **, const char *, const char *);
MOSAPI int MOSCConv mkkvent_tkn(kvent_t **, const char *, const char *);
MOSAPI int MOSCConv mkkvent_int(kvent_t **, const char *, int);
MOSAPI int MOSCConv mkkvent_int64(kvent_t **, const char *, int64_t);
MOSAPI int MOSCConv mkkvent_bool(kvent_t **, const char *, int);
MOSAPI int MOSCConv mkkvent_kv(kvent_t **, int, char *, char *);
MOSAPI void MOSCConv kventfree(kvent_t **);
MOSAPI void MOSCConv kvent_setvalue(kvent_t *, const char *);
MOSAPI int MOSCConv kvent_getstr(kvent_t *,  char *, uint32_t);
MOSAPI const char * MOSCConv kvent_getstrc(kvent_t *);
MOSAPI uint64_t MOSCConv kvent_getui64(kvent_t *, uint64_t);
MOSAPI int64_t MOSCConv kvent_geti64(kvent_t *, int64_t);
MOSAPI uint32_t MOSCConv kvent_getui32(kvent_t *, uint32_t);
MOSAPI int32_t MOSCConv kvent_geti32(kvent_t *, int32_t);
MOSAPI int MOSCConv kvent_getbool(kvent_t *, int);
MOSAPI int MOSCConv kvent_gettext(kvent_t *, char *, uint32_t, uint32_t *);
MOSAPI const char * MOSCConv kvent_getkey(kvent_t *);

#endif /* _KV_H_ */
