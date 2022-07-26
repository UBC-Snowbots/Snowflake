#include "phidgetbase.h"
#include "kv.h"

#include <stdio.h>

#include "mos/mos_error-errno.h"
#include "sys/stat.h"
#include "parse.h"
#include "scan.h"

#define KV_CASE_INSENSITIVE	0x01
#define KV_MAGIC 0x44789121

int kv_reduce(mosiop_t, scanstate_t *, scanresult_t *, void *);
int kv_rw(kv_t *, mosiop_t, int, uint8_t *, uint32_t, uint32_t *);

int
kv_reduce(mosiop_t iop, scanstate_t *ss, scanresult_t *sres, void *private) {
	kvent_t *e;
	kv_t *pkv;
	kv_t *kv;
	int err;

	pkv = private;
	if (pkv->magic != KV_MAGIC)
		return (MOS_ERROR(iop, MOSN_INVALARG, "private is not a kv"));

	if (sres[0].type == SCAN_NAMESPACE) {
		err = newkv_ns(&kv, sres[0].value);
		if (err != 0)
			return (MOS_ERROR(iop, err, "failed to construct namespace kv"));
		MTAILQ_INSERT_TAIL(&pkv->namespaces, kv, nslink);
		pkv->_curns = kv;

		return (0);
	}

	if (pkv->_curns != NULL)
		kv = pkv->_curns;
	else
		kv = pkv;

	switch(sres[2].type) {
	case SCAN_STRING:
		err = mkkvent_str(&e, sres[0].value, sres[2].value);
		break;
	case SCAN_TOKEN:
		err = mkkvent_tkn(&e, sres[0].value, sres[2].value);
		break;
	case SCAN_INT:
		err = mkkvent_int64(&e, sres[0].value, sres[2].ivalue);
		break;
	case SCAN_TRUE:
		err = mkkvent_bool(&e, sres[0].value, 1);
		break;
	case SCAN_FALSE:
		err = mkkvent_bool(&e, sres[0].value, 0);
		break;
	default:
		return (MOS_ERROR(iop, MOSN_INVAL, "invalid value type [%s] line %d+%d",
		  scantoken(sres[2].type), ss->lineno, ss->charno));
	}
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to consruct kv entity"));

	MTAILQ_INSERT_TAIL(&kv->list, e, link);
	kv->cnt++;

	return (0);
}

static kvent_t *
getentity(kv_t *thiskv, const char *key) {
	kvent_t *e;

	MTAILQ_FOREACH(e, &thiskv->list, link) {
		if (thiskv->flags & KV_CASE_INSENSITIVE) {
			if (mos_strcasecmp(e->key, key) == 0)
				return (e);
		} else {
			if (mos_strcmp(e->key, key) == 0)
				return (e);
		}
	}
	return (NULL);
}

/*
 * Value must be NULL, or already malloced.
 */
static int
addentity(kv_t *thiskv, mosiop_t iop, const char *key, const char *val) {
	kvent_t *e;
	int err;

	err = mkkvent_str(&e, key, val);
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to construct kvent"));

	MTAILQ_INSERT_TAIL(&thiskv->list, e, link);
	thiskv->cnt++;

	return (0);
}

static int
writekv(kv_t *thiskv, mosiop_t iop, FILE *fp) {
	uint32_t len;
	kvent_t *e;
	size_t res;
	char *kv;

	MTAILQ_FOREACH(e, &thiskv->list, link) {
		switch(e->type) {
		case MOSKV_STR:
			mos_asprintf(&kv, &len, "%s=\"%s\"\n", e->key, e->val);
			break;
		case MOSKV_BOOL:
			if (mos_strcmp(e->val, "0") == 0)
				mos_asprintf(&kv, &len, "%s=false\n", e->key);
			else
				mos_asprintf(&kv, &len, "%s=true\n", e->key);
			break;
		default:
			mos_asprintf(&kv, &len, "%s=%s\n", e->key, e->val);
			break;
		}

		res = fwrite(kv, 1, len - 1, fp);
		if (res != (len - 1))
			return (MOS_ERROR(iop, (uint32_t)res, "failed to write entry '%s'", e->key));

		mos_free(kv, len);
	}

	return (0);
}

static int
kv_parse_file(kv_t *thiskv, mosiop_t iop, FILE *fp, uint32_t maxsz, kv_elist_t *elist) {
	char *kvbuf;
	size_t sz;
	int err;

	if (maxsz < 2)
		return (MOS_ERROR(iop, MOSN_INVALARG, "maxsz (%u) is too small", maxsz));

	kvbuf = mos_malloc(maxsz);

	sz = fread(kvbuf, 1, maxsz - 1, fp);
	if (sz == 0) {
		mos_free(kvbuf, maxsz);
		if (ferror(fp))
			return (MOS_ERROR(iop, MOSN_IO, "failed to read file"));
		return (0);
	}

	kvbuf[sz] = '\0';
	err = parse(iop, kvbuf, (uint32_t)sz, kv_reduce, thiskv);
	mos_free(kvbuf, maxsz);
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to parse file"));
	return (0);
}

MOSAPI int MOSCConv
kv_read(kv_t **thiskv, mosiop_t iop, const char *path) {
	int err;

	err = newkv(thiskv);
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to construct kv"));

	err = kv_loadf(*thiskv, iop, path);
	if (err != 0) {
		kvfree(thiskv);
		return (MOS_ERROR(iop, err, "failed to parse kv from %s", path));
	}

	return (0);
}

MOSAPI int MOSCConv
kv_vread(kv_t **thiskv, mosiop_t iop, const char *fmt, ...) {
	uint32_t plen;
	va_list va;
	char *path;
	int err;

	err = newkv(thiskv);
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to construct kv"));

	va_start(va, fmt);
	mos_vasprintf(&path, &plen, fmt, va);
	va_end(va);
	err = kv_loadf(*thiskv, iop, path);
	if (err != 0) {
		MOS_ERROR(iop, err, "failed to parse kv from %s", path);
		mos_free(path, plen);
		kvfree(thiskv);
		return (err);
	}
	mos_free(path, plen);

	return (0);
}

MOSAPI int MOSCConv
kv_vawrite(kv_t *thiskv, mosiop_t iop, const char *fmt, va_list va) {
	uint32_t plen;
	char *path;
	int err;

	mos_vasprintf(&path, &plen, fmt, va);
	err = kv_write(thiskv, iop, path);
	mos_free(path, plen);
	return (err);
}

MOSAPI int MOSCConv
kv_vwrite(kv_t *thiskv, mosiop_t iop, const char *fmt, ...) {
	uint32_t plen;
	va_list va;
	char *path;
	int err;

	va_start(va, fmt);
	mos_vasprintf(&path, &plen, fmt, va);
	va_end(va);

	err = kv_write(thiskv, iop, path);
	mos_free(path, plen);
	return (err);
}

MOSAPI int MOSCConv
kv_write(kv_t *thiskv, mosiop_t iop, const char *path) {
	char nsbuf[128];
	kv_t *kv;
	FILE *fp;
	int err;

	fp = fopen(path, "w");
	if (fp == NULL)
		return (MOS_ERROR(iop, MOSN_NOENT, "failed to open file '%s'", path));

	err = writekv(thiskv, iop, fp);

	MTAILQ_FOREACH(kv, &thiskv->namespaces, nslink) {
		mos_snprintf(nsbuf, sizeof (nsbuf), "[%s]\n", kv->kvnamespace);
		fwrite(nsbuf, 1, mos_strlen(nsbuf), fp);
		err = writekv(kv, iop, fp);
		if (err != 0) {
			MOS_ERROR(iop, err, "failed to write namespace kv");
			goto done;
		}
	}

done:
	return (err);
}

MOSAPI int MOSCConv
kv_loadf(kv_t *thiskv, mosiop_t iop, const char *path) {
	struct stat st;
	uint32_t maxsz;
	FILE *fp;
	int err;
	int res;

	maxsz = 65536;

	MTAILQ_INIT(&thiskv->list);
	thiskv->cnt = 0;

	if (path == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "null path"));

	res = stat(path, &st);
	if (res != 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "failed to stat() '%s'",
		  path));
	if ((ull_t)st.st_size >= (ull_t)maxsz)
		return (MOS_ERROR(iop, MOSN_NOSPC,
		  "%llu-byte kv file '%s' is too large (limit %u)", (ull_t)st.st_size,
		  path, maxsz));

	fp = fopen(path, "r");
	if (fp == NULL)
		return (MOS_ERROR(iop, MOSN_NOENT, "failed to open file '%s'", path));

	err = kv_parse_file(thiskv, iop, fp, maxsz, &thiskv->list);
	fclose(fp);

	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to parse file '%s'", path));

	return (0);
}

MOSAPI int MOSCConv
newkv(kv_t **thiskv) {

	*thiskv = mos_zalloc(sizeof (kv_t));
	(*thiskv)->magic = KV_MAGIC;

	MTAILQ_INIT(&(*thiskv)->list);
	MTAILQ_INIT(&(*thiskv)->namespaces);
	(*thiskv)->cnt = 0;

	return (0);
}

MOSAPI int MOSCConv
newkv_ns(kv_t **thiskv, const char *kvnamespace) {
	int err;

	err = newkv(thiskv);
	if (err != 0)
		return (err);

	(*thiskv)->kvnamespace = mos_strdup(kvnamespace, NULL);

	return (0);
}

MOSAPI int MOSCConv
newkvbuf(kv_t **thiskv, mosiop_t iop, const char *kvbuf) {
	int err;

	err = newkv(thiskv);
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to create kv"));

	err = parse(iop, kvbuf, 0, kv_reduce, *thiskv);
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to parse kv buf"));
	return (0);
}

MOSAPI void MOSCConv
kvfree(kv_t **_thiskv) {
	kvent_t *e1, *e2;
	kv_t *k1, *k2;
	kv_t *thiskv;

	if (_thiskv == NULL)
		return;

	thiskv = *_thiskv;

	if (thiskv->kvnamespace != NULL)
		mos_free(thiskv->kvnamespace, mos_strlen(thiskv->kvnamespace) + 1);

	k1 = MTAILQ_FIRST(&thiskv->namespaces);
	while (k1 != NULL) {
		k2 = MTAILQ_NEXT(k1, nslink);
		kvfree(&k1);
		k1 = k2;
	}

	e1 = MTAILQ_FIRST(&thiskv->list);
	while (e1 != NULL) {
		e2 = MTAILQ_NEXT(e1, link);
		kventfree(&e1);
		e1 = e2;
	}

	/* for crash debugging sanity */
	MTAILQ_INIT(&thiskv->namespaces);
	MTAILQ_INIT(&thiskv->list);

	mos_free(*_thiskv, sizeof (kv_t));
	*_thiskv = NULL;
}

MOSAPI int MOSCConv
kvcount(kv_t *thiskv) {

	return (thiskv->cnt);
}

MOSAPI void MOSCConv
kvsetcaseinsensitive(kv_t *thiskv, int on) {

	if (on)
		thiskv->flags |= KV_CASE_INSENSITIVE;
	else
		thiskv->flags &= ~KV_CASE_INSENSITIVE;
}

MOSAPI int MOSCConv
kvgetentity(kv_t *thiskv, mosiop_t iop, const char *key, kvent_t **ent) {

	if (ent == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "return pointer is null"));

	*ent = getentity(thiskv, key);
	if (*ent == NULL)
		return (MOS_ERROR(iop, MOSN_NOENT, "no such entity '%s'", key));

	return (0);
}

/**
 * Adds the key and value.  It is an error if the key already exists.
 */
MOSAPI int MOSCConv
kvadd(kv_t *thiskv, mosiop_t iop, const char *key, const char *val) {
	kvent_t *e;
	int err;

	if (key == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "key is null"));
	if (val == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "val is null"));

	e = getentity(thiskv, key);
	if (e != NULL)
		return (MOS_ERROR(iop, MOSN_EXIST, "'%s' already exists", key));

	err = addentity(thiskv, iop, key, val);
	if (err != 0)
		return (MOS_ERROR(iop, err, "failed to add entitiy '%s'", key));

	return (0);
}

MOSAPI int MOSCConv
kvvadd(kv_t *thiskv, mosiop_t iop, const char *key, const char *fmt, ...) {
	va_list va;
	int err;

	va_start(va, fmt);
	err = kvvaadd(thiskv, iop, key, fmt, va);
	va_end(va);

	return (err);
}

MOSAPI int MOSCConv
kvvaadd(kv_t *thiskv, mosiop_t iop, const char *key, const char *fmt, va_list va) {
	kvent_t *e;

	if (key == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "key is null"));
	if (fmt == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "fmt is null"));

	e = getentity(thiskv, key);
	if (e != NULL)
		return (MOS_ERROR(iop, MOSN_EXIST, "'%s' already exists", key));

	return (kvvaset(thiskv, iop, key, fmt, va));
}

/**
 * Sets the value of the given key to <code>val</code>.  If the key doesn't
 * already exist, it is created.
 */
MOSAPI int MOSCConv
kvset(kv_t *thiskv, mosiop_t iop, const char *key, const char *val) {
	kvent_t *e;
	int err;

	if (key == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "key is null"));
	if (val == NULL)
		val = "";

	e = getentity(thiskv, key);
	if (e != NULL) {
		kvent_setvalue(e, val);
	} else {
		err = addentity(thiskv, iop, key, val);
		if (err != 0)
			return (MOS_ERROR(iop, err, "failed to add entitiy '%s'", key));
	}

	return (0);
}

MOSAPI int MOSCConv
kvvset(kv_t *thiskv, mosiop_t iop, const char *key, const char *fmt, ...) {
	va_list va;
	int err;

	va_start(va, fmt);
	err = kvvaset(thiskv, iop, key, fmt, va);
	va_end(va);

	return (err);
}

MOSAPI int MOSCConv
kvvaset(kv_t *thiskv, mosiop_t iop, const char *key, const char *fmt, va_list va) {
	kvent_t *e;
	uint32_t len;
	char *val;
	int err;

	if (key == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "key is null"));
	if (fmt == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "fmt is null"));

	mos_vasprintf(&val, &len, fmt, va);

	e = getentity(thiskv, key);
	if (e != NULL) {
		kvent_setvalue(e, val);
		mos_free(val, len);
	} else {
		err = addentity(thiskv, iop, key, val);
		mos_free(val, len);
		if (err != 0)
			return (MOS_ERROR(iop, err, "failed to add entitiy '%s'", key));
	}

	return (0);
}

MOSAPI int MOSCConv
kvremove(kv_t *thiskv, mosiop_t iop, const char *key) {
	kvent_t *e;

	e = getentity(thiskv, key);
	if (e == NULL)
		return (MOS_ERROR(iop, MOSN_NOENT, "no such entity '%s'", key));

	MTAILQ_REMOVE(&thiskv->list, e, link);
	kventfree(&e);
	thiskv->cnt--;

	return (0);
}

MOSAPI int MOSCConv
kvgetstr(kv_t *thiskv, mosiop_t iop, const char *key, char *vbuf, uint32_t vbufsz) {
	kvent_t *e;
	int err;

	e = getentity(thiskv, key);
	if (e == NULL)
		return (MOS_ERROR(iop, MOSN_NOENT, "no such entry '%s'", key));

	err = kvent_getstr(e, vbuf, vbufsz);
	if (err != 0)
		return (MOS_ERROR(iop, err, "getstr() failed"));
	return (0);
}

MOSAPI const char * MOSCConv
kvgetstrc(kv_t *thiskv, const char *key, const char *def) {
	kvent_t *e;

	e = getentity(thiskv, key);
	if (e == NULL)
		return (def);

	return (kvent_getstrc(e));
}

MOSAPI uint64_t MOSCConv
kvgetui64(kv_t *thiskv, const char *key, uint64_t def) {
	kvent_t *e;

	e = getentity(thiskv, key);
	if (e == NULL)
		return (def);

	return (kvent_getui64(e, def));
}

MOSAPI uint32_t MOSCConv
kvgetui32(kv_t *thiskv, const char *key, uint32_t def) {
	kvent_t *e;

	e = getentity(thiskv, key);
	if (e == NULL)
		return (def);

	return (kvent_getui32(e, def));
}

MOSAPI int64_t MOSCConv
kvgeti64(kv_t *thiskv, const char *key, int64_t def) {
	kvent_t *e;

	e = getentity(thiskv, key);
	if (e == NULL)
		return (def);

	return (kvent_geti64(e, def));
}

MOSAPI int32_t MOSCConv
kvgeti32(kv_t *thiskv, const char *key, int32_t def) {
	kvent_t *e;

	e = getentity(thiskv, key);
	if (e == NULL)
		return (def);

	return (kvent_geti32(e, def));
}

MOSAPI int MOSCConv
kvgetbool(kv_t *thiskv, const char *key, int def) {
	kvent_t *e;

	e = getentity(thiskv, key);
	if (e == NULL)
		return (def);

	return (kvent_getbool(e, def));
}

MOSAPI int MOSCConv
kvhasvalue(kv_t *thiskv, const char *key) {

	return (getentity(thiskv, key) != NULL);
}

MOSAPI const char * MOSCConv
kvgetnamespace(kv_t *thiskv) {

	if (thiskv->kvnamespace == NULL)
		return ("");
	return (thiskv->kvnamespace);
}

MOSAPI int MOSCConv
kvgetnamespacekv(kv_t *thiskv, const char *ns, kv_t **kv) {

	MTAILQ_FOREACH(*kv, &thiskv->namespaces, nslink)
		if (mos_strcmp(ns, (*kv)->kvnamespace) == 0)
			return (0);

	*kv = NULL;
	return (0);
}

MOSAPI int MOSCConv
kvaddnamespacekv(kv_t *thiskv, mosiop_t iop, kv_t *kv) {

	if (kv == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "null kv"));

	if (kv->kvnamespace == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "kv namespace name not set"));

	MTAILQ_INSERT_HEAD(&thiskv->namespaces, kv, nslink);

	return (0);
}
