#include "phidgetconfig.h"

RB_GENERATE(pconfentries, pconfentry, link, phidgetentrycompare)

static void pconf_releaseentry(pconfentry_t *);

static const char *
strescape(int strchar, const char *in, char *out, size_t len) {
	const char *p;
	char *o;

	for (p = in, o = out; *p && (size_t)(o - out) < len; p++, o++) {
		if ((unsigned char)*p > 0x1F && (unsigned char)*p != strchar) {
			*o = *p;
			continue;
		}
		*o++ = '\\';

		switch (*p) {
		case '\'':
		case '"':
		case '\\':
			*o = *p;
			break;
		case '\n':
			*o = 'n';
			break;
		case '\r':
			*o = 'r';
			break;
		case '\b':
			*o = 'b';
			break;
		case '\f':
			*o = 'f';
			break;
		case '\t':
			*o = 't';
			break;
		default:
			mos_snprintf(o, len - (o - out), "%04d", *p);
			o += 3;
			p++;	/* broken for multi-byte chars? */
		}
	}

	if ((size_t)(o - out) >= len)
		return (NULL);

	*o = '\0';
	return (out);
}

int
phidgetentrycompare(pconfentry_t *a, pconfentry_t *b) {

	return (mos_strcmp(a->key, b->key));
}

static char *
getcomponent(const char *path, int off, char *buf, size_t bufsz) {
	const char *c;
	char *e;
	int i;

	for (i = 0, c = path; i < off; i++) {
		c = mos_strchrc(c, '.');
		if (c == NULL)
			return (NULL);
		c++;
	}

	if (*c == '\0')
		return (NULL);

	mos_strlcpy(buf, c, bufsz);
	e = mos_strchr(buf, '.');
	if (e)
		*e = '\0';

	return (buf);
}

static char *
getbase(const char *path, char *buf, size_t bufsz) {
	const char *c;

	c = mos_strrchrc(path, '.');
	if (c == NULL)
		return (NULL);
	mos_strlcpy(buf, path, bufsz);
	buf[c - path] = '\0';
	return (buf);
}

static char *
getlastcomponent(const char *path, char *buf, size_t bufsz) {
	const char *c;

	c = mos_strrchrc(path, '.');
	if (c == NULL) {
		mos_strlcpy(buf, path, bufsz);
		return (buf);
	}
	mos_strlcpy(buf, c + 1, bufsz);
	return (buf);
}


static PhidgetReturnCode
pconf_mkentry(pconfentry_t **entry, const char *key, pconftype_t type, int flags) {

	*entry = mos_zalloc(sizeof (pconfentry_t));
	(*entry)->type = type;
	(*entry)->flags = flags;
	if (key)
		(*entry)->key = mos_strdup(key, NULL);

	if (type == PHIDGETCONFIG_BLOCK || type == PHIDGETCONFIG_ARRAY)
		RB_INIT(&(*entry)->value.entries);

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
pconf_getblock(pconf_t *pc, int cm, pconfentry_t **_entry, const char *path) {
	char base[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *entry;

	if (getbase(path, base, sizeof (base)) == NULL) {
		*_entry = pc->root;
	} else {
		res = pconf_getentry(pc, cm, &entry, base);
		if (res != EPHIDGET_OK)
			return (res);
		if (entry->type != PHIDGETCONFIG_BLOCK && entry->type != PHIDGETCONFIG_ARRAY)
			return (EPHIDGET_INVALID);
		*_entry = entry;
	}
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
pconf_addtoentry(pconfentry_t *parent, pconfentry_t *entry ) {
	char nkey[12];

	if (parent->type == PHIDGETCONFIG_BLOCK)
		MOS_ASSERT(entry->key != NULL);
	if (parent->type == PHIDGETCONFIG_ARRAY) {
		MOS_ASSERT(entry->key == NULL);
		mos_snprintf(nkey, sizeof (nkey), "%d", parent->cnt);
		entry->key = mos_strdup(nkey, NULL);
	}

	if (RB_INSERT(pconfentries, &parent->value.entries, entry) != NULL)
		return (EPHIDGET_DUPLICATE);

	parent->cnt++;

	return (EPHIDGET_OK);
}

#define MAX_DEPTH 32
typedef enum { START, KEY, VALUE, STOP } parse_state_t;
struct json_stack {
	pconfentry_t	*entry;
	int				objtoks;
};

#define JSONLEN ((int)((jsonsz) - ((c) - (json))))

#define JSONADD(...) do {								\
	int __len__;										\
	int __n__;											\
	__len__ = JSONLEN;									\
	__n__ = mos_snprintf(c, __len__, __VA_ARGS__);		\
	if (__n__ >= __len__)								\
		return (EPHIDGET_NOSPC);						\
	c += __n__;											\
} while (0)

static char *
jsonnumber(double num, char *buf, size_t bufsz) {
	char *c;

	mos_snprintf(buf, bufsz, "%f", num);

	for (c = &buf[mos_strlen(buf) - 1]; c != &buf[1] && *(c - 1) != '.'; c--)
		if (*c == '0')
			*c = '\0';

	return (buf);
}

static PhidgetReturnCode
pconf_renderentryjson(pconfentry_t *entry, char **_c, char *json, size_t jsonsz, int oecnt, int array) {
	char buf[PHIDGETCONFIG_STR_MAX];
	PhidgetReturnCode res;
	pconfentry_t *ent;
	int ecnt;
	char *c;

	c = *_c;

	if (oecnt > 0)
		JSONADD(",");

	if (!array)
		JSONADD("\"%s\":", entry->key);

	switch (entry->type) {
	case PHIDGETCONFIG_BLOCK:
		JSONADD("{");
		ecnt = 0;
		RB_FOREACH(ent, pconfentries, &entry->value.entries) {
			res = pconf_renderentryjson(ent, &c, json, jsonsz, ecnt, 0);
			if (res != EPHIDGET_OK)
				return (res);
			ecnt++;
		}
		JSONADD("}");
		break;
	case PHIDGETCONFIG_ARRAY:
		JSONADD("[");
		ecnt = 0;
		RB_FOREACH(ent, pconfentries, &entry->value.entries) {
			res = pconf_renderentryjson(ent, &c, json, jsonsz, ecnt, 1);
			if (res != EPHIDGET_OK)
				return (res);
			ecnt++;
		}
		JSONADD("]");
		break;
	case PHIDGETCONFIG_BOOLEAN:
		JSONADD("%s", entry->e_bool ? "true" : "false");
		break;
	case PHIDGETCONFIG_STRING:
		JSONADD("\"%s\"", json_escape(entry->e_str, buf, sizeof (buf)));
		break;
	case PHIDGETCONFIG_NUMBER:
		JSONADD("%s", jsonnumber(entry->e_num, buf, sizeof (buf)));
		break;
	case PHIDGETCONFIG_I64:
		JSONADD("%"PRId64, entry->e_i64);
		break;
	case PHIDGETCONFIG_U64:
		JSONADD("%"PRIu64, entry->e_u64);
		break;
	case PHIDGETCONFIG_NULL:
		JSONADD("null");
		break;
	default:
		return (EPHIDGET_UNEXPECTED);
	}

	*_c = c;
	return (EPHIDGET_OK);
}

API_PRETURN
pconf_renderjson(pconf_t *pc, char *json, size_t jsonsz) {
	PhidgetReturnCode res;
	pconfentry_t *entry;
	int ecnt;
	char *c;

	if (jsonsz < 3)
		return (EPHIDGET_NOSPC);

	c = json;
	if (pc->root->type == PHIDGETCONFIG_BLOCK)
		JSONADD("{");
	else if (pc->root->type == PHIDGETCONFIG_ARRAY)
		JSONADD("[");

	ecnt = 0;
	RB_FOREACH(entry, pconfentries, &pc->pc_root) {
		res = pconf_renderentryjson(entry, &c, json, jsonsz, ecnt, pc->root->type == PHIDGETCONFIG_ARRAY);
		if (res != EPHIDGET_OK)
			return (res);
		ecnt++;
	}

	if (pc->root->type == PHIDGETCONFIG_BLOCK)
		JSONADD("}");
	else if (pc->root->type == PHIDGETCONFIG_ARRAY)
		JSONADD("]");

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_create(pconf_t **upc) {
	pconf_t *pc;

	pc = mos_zalloc(sizeof (*pc));
	pc->root = mos_zalloc(sizeof (pconfentry_t));
	pc->root->type = PHIDGETCONFIG_BLOCK;
	RB_INIT(&pc->pc_root);

	*upc = pc;

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_setcreatemissing(pconf_t *pc, int enable) {

	if (enable)
		pc->flags |= PHIDGETCONFIG_CREATEMISSING;
	else
		pc->flags &= ~PHIDGETCONFIG_CREATEMISSING;

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_parsejson(pconf_t **upc, const char *json, size_t jsonsz) {
	PhidgetReturnCode res;
	parse_state_t state;
	pjsmntok_t *tokens;
	pjsmn_parser p;
	pjsmntok_t *t;
	pconf_t *pc;
	int flags;

	struct json_stack stack[MAX_DEPTH];
	pconfentry_t *entry;
	pconftype_t type;
	char valbuf[512];
	char keybuf[64];
	int64_t ival;
	double nval;
	char *sval;
	char *key;
	int depth;
	int bval;

	int i, j;
	int err;

	TESTPTR(upc);
	TESTPTR(json);

	nval = 0;
	bval = 0;

	tokens = mos_malloc(sizeof (*tokens) * PHIDGETCONFIG_TOKEN_MAX);
	pjsmn_init(&p);
	err = pjsmn_parse(&p, json, jsonsz, tokens, PHIDGETCONFIG_TOKEN_MAX);
	if (err < 0) {
		mos_free(tokens, sizeof (*tokens) * PHIDGETCONFIG_TOKEN_MAX);
		return (EPHIDGET_UNEXPECTED);
	}

	res = pconf_create(&pc);
	if (res != EPHIDGET_OK) {
		mos_free(tokens, sizeof (*tokens) * PHIDGETCONFIG_TOKEN_MAX);
		return (res);
	}

	depth = 0;

	state = START;
	key = NULL;

	for (i = 0, j = 1; j > 0; i++, j--) {
		t = &tokens[i];

		if (t->type == JSMN_ARRAY)
			j += t->size;
		if (t->type == JSMN_OBJECT)
			j += t->size * 2;

		switch (state) {
		case START:

			stack[depth].objtoks = t->size;
			stack[depth].entry = pc->root;

			switch (t->type) {
			case JSMN_OBJECT:
				pc->root->type = PHIDGETCONFIG_BLOCK;
				state = KEY;
				break;
			case JSMN_ARRAY:
				pc->root->type = PHIDGETCONFIG_ARRAY;
				state = VALUE;
				break;
			default:
				goto parseerror;
			}

			if (stack[depth].objtoks == 0)
				state = STOP;
			break;
		case STOP:
			goto parsedone;
		case KEY:
			stack[depth].objtoks--;

			if (t->type != JSMN_STRING)
				goto parseerror;

			key = pjsmn_string(json, t, keybuf, sizeof (keybuf));
			if (key == NULL)
				goto parseerror;

			state = VALUE;
			break;
		case VALUE:
			if (stack[depth].entry->type == PHIDGETCONFIG_ARRAY) {
				flags = PHIDGETCONFIG_INARRAY;
				stack[depth].objtoks--;
			} else {
				flags = 0;
			}

			switch (t->type) {
			case JSMN_STRING:
				sval = pjsmn_string(json, t, valbuf, sizeof (valbuf));
				if (sval == NULL)
					goto parseerror;

				err = pconf_mkentry(&entry, key, PHIDGETCONFIG_STRING, flags);
				if (err != EPHIDGET_OK)
					goto parseerror;
				entry->e_str = mos_strdup(json_unescape(sval), NULL);
				res = pconf_addtoentry(stack[depth].entry, entry);
				if (res != EPHIDGET_OK)
					goto parseerror;
				break;
			case JSMN_PRIMITIVE:
				type = PHIDGETCONFIG_I64;
				err = pjsmn_int64(json, t, &ival);
				if (err != 0) {
					type = PHIDGETCONFIG_NUMBER;
					err = pjsmn_double(json, t, &nval);
					if (err != 0) {
						type = PHIDGETCONFIG_BOOLEAN;
						bval = pjsmn_boolean(json, t);
						if (bval == -1) {
							type = PHIDGETCONFIG_NULL;
							if (t->end - t->start == 4 && mos_strncmp(&json[t->start], "null", 4) != 0)
								goto parseerror;
						}
					}
				}

				switch (type) {
				case PHIDGETCONFIG_I64:
					err = pconf_mkentry(&entry, key, PHIDGETCONFIG_I64, flags);
					entry->e_i64 = ival;
					break;
				case PHIDGETCONFIG_NUMBER:
					err = pconf_mkentry(&entry, key, PHIDGETCONFIG_NUMBER, flags);
					entry->e_num = nval;
					break;
				case PHIDGETCONFIG_BOOLEAN:
					err = pconf_mkentry(&entry, key, PHIDGETCONFIG_BOOLEAN, flags);
					entry->e_bool = bval;
					break;
				case PHIDGETCONFIG_NULL:
					err = pconf_mkentry(&entry, key, PHIDGETCONFIG_NULL, flags);
					break;
				default:
					goto parseerror;
				}
				if (err != EPHIDGET_OK)
					goto parseerror;

				res = pconf_addtoentry(stack[depth].entry, entry);
				if (res != EPHIDGET_OK)
					goto parseerror;
				break;
			case JSMN_OBJECT:
				err = pconf_mkentry(&entry, key, PHIDGETCONFIG_BLOCK, flags);
				if (err != EPHIDGET_OK)
					goto parseerror;

				depth++;
				if (depth >= MAX_DEPTH)
					goto parseerror;
				stack[depth].objtoks = t->size;
				stack[depth].entry = entry;
				res = pconf_addtoentry(stack[depth - 1].entry, entry);
				if (res != EPHIDGET_OK)
					goto parseerror;
				break;
			case JSMN_ARRAY:
				err = pconf_mkentry(&entry, key, PHIDGETCONFIG_ARRAY, flags);
				if (err != EPHIDGET_OK)
					goto parseerror;

				depth++;
				if (depth >= MAX_DEPTH)
					goto parseerror;
				stack[depth].objtoks = t->size;
				stack[depth].entry = entry;
				res = pconf_addtoentry(stack[depth - 1].entry, entry);
				if (res != EPHIDGET_OK)
					goto parseerror;
				break;
			default:
				goto parseerror;
			}

			while (stack[depth].objtoks == 0 && depth > 0)
				depth--;
			if (stack[depth].entry->type == PHIDGETCONFIG_ARRAY)
				state = VALUE;
			else
				state = KEY;
			key = NULL;
			break;
		}
	}

parsedone:

	*upc = pc;
	mos_free(tokens, sizeof (*tokens) * PHIDGETCONFIG_TOKEN_MAX);
	return (EPHIDGET_OK);

parseerror:

	logerr("json parse failed token %d: char:%d", i, t->start);
	pconf_release(&pc);
	mos_free(tokens, sizeof (*tokens) * PHIDGETCONFIG_TOKEN_MAX);
	return (EPHIDGET_INVALID);
}

/*
 * Returns the name of the entry at the specified offset.
 */
API_CRETURN
pconf_getentryname(pconf_t *pc, int off, const char *fmt, ...) {
	pconfentry_t *ent, *cent;
	PhidgetReturnCode res;
	va_list va;
	int i;

	va_start(va, fmt);
	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	va_end(va);
	if (res != EPHIDGET_OK)
		return (NULL);

	if (ent->type != PHIDGETCONFIG_BLOCK && ent->type != PHIDGETCONFIG_ARRAY)
		return (NULL);

	i = 0;
	RB_FOREACH(cent, pconfentries, &ent->value.entries) {
		if (i == off)
			return (cent->key);
		i++;
	}

	return (NULL);
}

API_I32RETURN
pconf_getcount(pconf_t *pc, const char *fmt, ...) {
	PhidgetReturnCode res;
	pconfentry_t *ent;
	va_list va;

	va_start(va, fmt);
	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	va_end(va);

	if (res != EPHIDGET_OK)
		return (-1);

	return (ent->cnt);
}

API_PRETURN
pconf_getentryv(pconf_t *pc, int cm, pconfentry_t **_ent, const char *fmt, va_list va) {
	char path[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentries_t *entries;
	pconfentry_t *pent;
	pconfentry_t sent;
	pconfentry_t *ent;
	char comp[32];
	int len;
	int i;

	len = mos_vsnprintf(path, sizeof (path), fmt, va);
	if ((unsigned)len >= sizeof (path))
		return (EPHIDGET_INVALIDARG);

	for (len = 0;; len++) {
		if (getcomponent(path, len, comp, sizeof (comp)) == NULL)
			break;
	}

	entries = &pc->pc_root;
	pent = pc->root;
	ent = NULL;

	for (i = 0; i < len - 1; i++) {
		sent.key = getcomponent(path, i, comp, sizeof (comp));
		if (sent.key == NULL)
			return (EPHIDGET_UNEXPECTED);

		ent = RB_FIND(pconfentries, entries, &sent);
		if (ent == NULL) {
			if (cm == 0 || (pc->flags & PHIDGETCONFIG_CREATEMISSING) == 0)
				return (EPHIDGET_NOENT);

			res = pconf_mkentry(&ent, sent.key, PHIDGETCONFIG_BLOCK, pent->type == PHIDGETCONFIG_ARRAY);
			if (res != EPHIDGET_OK)
				return (res);

			if (RB_INSERT(pconfentries, &pent->value.entries, ent) != NULL) {
				pconf_releaseentry(ent);
				return (EPHIDGET_DUPLICATE);
			}
			pent->cnt++;
		}
		if (ent->type != PHIDGETCONFIG_BLOCK && ent->type != PHIDGETCONFIG_ARRAY)
			return (EPHIDGET_NOENT);
		entries = &ent->value.entries;
		pent = ent;
	}
	sent.key = getcomponent(path, i, comp, sizeof (comp));
	ent = RB_FIND(pconfentries, entries, &sent);
	if (ent == NULL) {
		if (cm == 0 || (pc->flags & PHIDGETCONFIG_CREATEMISSING) == 0)
			return (EPHIDGET_NOENT);

		res = pconf_mkentry(&ent, sent.key, PHIDGETCONFIG_BLOCK, pent->type == PHIDGETCONFIG_ARRAY);
		if (res != EPHIDGET_OK)
			return (res);

		if (RB_INSERT(pconfentries, &pent->value.entries, ent) != NULL) {
			pconf_releaseentry(ent);
			return (EPHIDGET_DUPLICATE);
		}
		pent->cnt++;
	}

	*_ent = ent;

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_getentry(pconf_t *pc, int cm, pconfentry_t **ent, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_getentryv(pc, cm, ent, fmt, va);
	va_end(va);

	return (res);
}

static void
pconf_releaseentry(pconfentry_t *ent) {
	pconfentry_t *fent, *tent;

	if (ent->key)
		mos_free(ent->key, MOSM_FSTR);

	switch (ent->type) {
	case PHIDGETCONFIG_BLOCK:
	case PHIDGETCONFIG_ARRAY:
		RB_FOREACH_SAFE(fent, pconfentries, &ent->value.entries, tent) {
			RB_REMOVE(pconfentries, &ent->value.entries, fent);
			pconf_releaseentry(fent);
			ent->cnt--;
			MOS_ASSERT(ent->cnt >= 0);
		}
		break;
	case PHIDGETCONFIG_STRING:
		mos_free(ent->e_str, MOSM_FSTR);
		break;
	}
	mos_free(ent, sizeof (*ent));
}

API_PRETURN_HDR
pconf_removev(pconf_t *pc, const char *fmt, va_list va) {
	char path[PHIDGETCONFIG_PATH_MAX];
	char base[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *fent;
	pconfentry_t sent;
	pconfentry_t *ent;
	size_t n;

	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	if (n >= sizeof (path))
		return (EPHIDGET_NOSPC);

	if (getbase(path, base, sizeof (base)) == NULL)
		return (EPHIDGET_INVALIDARG);

	res = pconf_getentry(pc, 0, &ent, "%s", base);
	if (res != EPHIDGET_OK)
		return (res);

	if (ent->type != PHIDGETCONFIG_BLOCK && ent->type != PHIDGETCONFIG_ARRAY)
		return (EPHIDGET_INVALIDARG);

	sent.key = getlastcomponent(path, base, sizeof (base));
	if (sent.key == NULL)
		return (EPHIDGET_UNEXPECTED);

	fent = RB_FIND(pconfentries, &ent->value.entries, &sent);
	if (fent == NULL)
		return (EPHIDGET_NOENT);

	RB_REMOVE(pconfentries, &ent->value.entries, fent);
	pconf_releaseentry(fent);
	ent->cnt--;
	MOS_ASSERT(ent->cnt >= 0);

	return (EPHIDGET_OK);
}

API_PRETURN_HDR
pconf_remove(pconf_t *pc, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_removev(pc, fmt, va);
	va_end(va);

	return (res);
}

API_PRETURN
pconf_release(pconf_t **upc) {
	pconfentry_t *ent, *tent;
	pconf_t *pc;

	TESTPTR(upc);

	pc = *upc;
	TESTPTR(pc);
	*upc = NULL;

	RB_FOREACH_SAFE(ent, pconfentries, &pc->pc_root, tent) {
		RB_REMOVE(pconfentries, &pc->pc_root, ent);
		pc->root->cnt--;
		MOS_ASSERT(ent->cnt >= 0);
		pconf_releaseentry(ent);
	}

	mos_free(pc->root, sizeof (pconfentry_t));
	mos_free(pc, sizeof (*pc));

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_merge(pconf_t *dpc, pconf_t **spc, const char *name, const char *fmt, ...) {
	PhidgetReturnCode res;
	pconfentry_t *ent;
	va_list va;

	TESTPTR(dpc);
	TESTPTR(spc);

	va_start(va, fmt);
	res = pconf_getentryv(dpc, 0, &ent, fmt, va);
	va_end(va);
	if (res != EPHIDGET_OK)
		return (res);

	switch (ent->type) {
	case PHIDGETCONFIG_ARRAY:
		res = pconf_addtoentry(ent, (*spc)->root);
		break;
	case PHIDGETCONFIG_BLOCK:
		(*spc)->root->key = mos_strdup(name, NULL);
		res = pconf_addtoentry(ent, (*spc)->root);
	}
	if (res == EPHIDGET_OK) {
		mos_free(*spc, sizeof (pconf_t));
		*spc = NULL;
	}

	return (res);
}

API_PRETURN
pconf_tostring(pconf_t *pc, char *buf, size_t bufsz, const char *fmt, ...) {
	PhidgetReturnCode res;
	pconfentry_t *ent;
	va_list va;

	va_start(va, fmt);
	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	va_end(va);
	if (res != EPHIDGET_OK)
		return (res);

	switch (ent->type) {
	case PHIDGETCONFIG_I64:
		mos_snprintf(buf, bufsz, "%"PRId64, ent->e_i64);
		break;
	case PHIDGETCONFIG_U64:
		mos_snprintf(buf, bufsz, "%"PRIu64, ent->e_u64);
		break;
	case PHIDGETCONFIG_NUMBER:
		mos_snprintf(buf, bufsz, "%f", ent->e_num);
		break;
	case PHIDGETCONFIG_BOOLEAN:
		mos_snprintf(buf, bufsz, "%d", ent->e_bool);
		break;
	case PHIDGETCONFIG_NULL:
		mos_strlcpy(buf, "<null>", bufsz);
		break;
	case PHIDGETCONFIG_STRING:
		mos_strlcpy(buf, ent->e_str, bufsz);
		break;
	case PHIDGETCONFIG_BLOCK:
		mos_strlcpy(buf, "<block>", bufsz);
		break;
	case PHIDGETCONFIG_ARRAY:
		mos_strlcpy(buf, "<array>", bufsz);
		break;
	default:
		return (EPHIDGET_UNEXPECTED);
	}

	return (EPHIDGET_OK);
}

/*
 * Tries to convert val to each supported type, and returns the detect type and converted value.
 * Does NOT allocate memory.
 *
 * Doesn't fail, as we can always create a string.
 */
API_PRETURN
pconf_detecttype(const char *val, pconftype_t *type, pconfvalue_t *eval) {
	PhidgetReturnCode res;
	uint64_t u64;
	int64_t i64;
	double dbl;
	int32_t b;
	char *end;

	if (val == NULL) {
		*type = PHIDGETCONFIG_NULL;
		eval->u64 = 0;
		return (EPHIDGET_OK);
	}

	res = mos_strtou64(val, 0, &u64);
	if (res == 0) {
		*type = PHIDGETCONFIG_U64;
		eval->u64 = u64;
		return (EPHIDGET_OK);
	}

	res = mos_strto64(val, 0, &i64);
	if (res == 0) {
		*type = PHIDGETCONFIG_I64;
		eval->i64 = i64;
		return (EPHIDGET_OK);
	}

	dbl = strtod(val, &end);
	if (dbl != 0 && end != val) {
		*type = PHIDGETCONFIG_NUMBER;
		eval->num = dbl;
		return (EPHIDGET_OK);
	}

	b = -1;
	if (mos_strcasecmp(val, "true") == 0)
		b = 1;
	else if (mos_strcasecmp(val, "false") == 0)
		b = 0;
	if (b == 1 || b == 0) {
		*type = PHIDGETCONFIG_BOOLEAN;
		eval->bl = b;
		return (EPHIDGET_OK);
	}

	*type = PHIDGETCONFIG_STRING;
	eval->cstr = val;

	return (EPHIDGET_OK);
}

/*
 * Casts 'val' to 'type', and assigns it to to 'eval'.
 */
API_PRETURN
pconf_cast(const char *val, pconftype_t type, pconfvalue_t *eval) {
	PhidgetReturnCode res;
	uint64_t u64;
	int64_t i64;
	double dbl;
	int32_t b;
	char *end;

	switch (type) {
	case PHIDGETCONFIG_I64:
		res = mos_strto64(val, 0, &i64);
		if (res != 0)
			return (EPHIDGET_INVALIDARG);
		eval->i64 = i64;
		break;
	case PHIDGETCONFIG_U64:
		res = mos_strtou64(val, 0, &u64);
		if (res != 0)
			return (EPHIDGET_INVALIDARG);
		eval->u64 = u64;
		break;
	case PHIDGETCONFIG_NUMBER:
		dbl = strtod(val, &end);
		if (dbl == 0 && end == val)
			return (EPHIDGET_INVALIDARG);
		eval->num = dbl;
		break;
	case PHIDGETCONFIG_BOOLEAN:
		if (mos_strncasecmp(val, "true", 4) == 0) {
			b = 1;
		} else if (mos_strncasecmp(val, "false", 5) == 0) {
			b = 0;
		} else {
			res = mos_strto32(val, 0, &b);
			if (res != 0)
				return (EPHIDGET_INVALIDARG);
		}
		eval->bl = b ? 1 : 0;
		break;
	case PHIDGETCONFIG_NULL:
		eval->u64 = 0;
		break;
	case PHIDGETCONFIG_STRING:
		eval->cstr = val;
		break;
	case PHIDGETCONFIG_BLOCK:
		return (EPHIDGET_INVALID);
	case PHIDGETCONFIG_ARRAY:
		return (EPHIDGET_INVALID);
	default:
		return (EPHIDGET_UNEXPECTED);
	}
	return (EPHIDGET_OK);
}

/*
 * Updates an existing ent to the new value, attempting to cast the new value to the existing type.
 * If the cast fails, the update will fail.
 */
API_PRETURN
pconf_updatev(pconf_t *pc, const char *val, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;
	char *oval;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res != EPHIDGET_OK)
		return (res);

	/*
	 * The cast will overwrite the string with the new one, so save the pointer so we can free it
	 * and allocate new memory for the new value.
	 */
	oval = ent->e_str;

	res = pconf_cast(val, ent->type, &ent->value.val);
	if (res != EPHIDGET_OK)
		return (res);

	if (ent->type == PHIDGETCONFIG_STRING) {
		mos_free(oval, MOSM_FSTR);
		ent->e_str = mos_strdup(ent->e_cstr, NULL);
	}

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_update(pconf_t *pc, const char *val, const char *fmt, ...) {
	uint32_t res;
	va_list va;

	va_start(va, fmt);
	res = pconf_updatev(pc, val, fmt, va);
	va_end(va);

	return (res);
}

API_U32RETURN
pconf_getu32v(pconf_t *pc, uint32_t def, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res != EPHIDGET_OK)
		return (def);

	switch (ent->type) {
	case PHIDGETCONFIG_I64:
		return ((uint32_t)ent->e_i64);
	case PHIDGETCONFIG_U64:
		return ((uint32_t)ent->e_u64);
	case PHIDGETCONFIG_NUMBER:
		return ((uint32_t)ent->e_num);
	case PHIDGETCONFIG_BOOLEAN:
		return ((uint32_t)ent->e_bool);
	case PHIDGETCONFIG_NULL:
		return (0);
	default:
		return (def);
	}
}

API_U32RETURN
pconf_getu32(pconf_t *pc, uint32_t def, const char *fmt, ...) {
	uint32_t res;
	va_list va;

	va_start(va, fmt);
	res = pconf_getu32v(pc, def, fmt, va);
	va_end(va);

	return (res);
}

API_I32RETURN
pconf_get32v(pconf_t *pc, int32_t def, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res != EPHIDGET_OK)
		return (def);

	switch (ent->type) {
	case PHIDGETCONFIG_I64:
		return ((int32_t)ent->e_i64);
	case PHIDGETCONFIG_U64:
		return ((int32_t)ent->e_u64);
	case PHIDGETCONFIG_NUMBER:
		return ((int32_t)ent->e_num);
	case PHIDGETCONFIG_BOOLEAN:
		return ((int32_t)ent->e_bool);
	case PHIDGETCONFIG_NULL:
		return (0);
	default:
		return (def);
	}
}

API_I32RETURN
pconf_get32(pconf_t *pc, int32_t def, const char *fmt, ...) {
	uint32_t res;
	va_list va;

	va_start(va, fmt);
	res = pconf_get32v(pc, def, fmt, va);
	va_end(va);

	return (res);
}

API_U64RETURN
pconf_getu64v(pconf_t *pc, uint64_t def, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res != EPHIDGET_OK)
		return (def);

	switch (ent->type) {
	case PHIDGETCONFIG_I64:
		return ((uint64_t)ent->e_i64);
	case PHIDGETCONFIG_U64:
		return ((uint64_t)ent->e_u64);
	case PHIDGETCONFIG_NUMBER:
		return ((uint64_t)ent->e_num);
	case PHIDGETCONFIG_BOOLEAN:
		return ((uint64_t)ent->e_bool);
	case PHIDGETCONFIG_NULL:
		return (0);
	default:
		return (def);
	}
}

API_U64RETURN
pconf_getu64(pconf_t *pc, uint64_t def, const char *fmt, ...) {
	uint64_t res;
	va_list va;

	va_start(va, fmt);
	res = pconf_getu64v(pc, def, fmt, va);
	va_end(va);

	return (res);
}

API_I64RETURN
pconf_get64v(pconf_t *pc, int64_t def, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res != EPHIDGET_OK)
		return (def);

	switch (ent->type) {
	case PHIDGETCONFIG_I64:
		return ((int64_t)ent->e_i64);
	case PHIDGETCONFIG_U64:
		return ((int64_t)ent->e_u64);
	case PHIDGETCONFIG_NUMBER:
		return ((int64_t)ent->e_num);
	case PHIDGETCONFIG_BOOLEAN:
		return ((int64_t)ent->e_bool);
	case PHIDGETCONFIG_NULL:
		return (0);
	default:
		return (def);
	}
}

API_I64RETURN
pconf_get64(pconf_t *pc, int64_t def, const char *fmt, ...) {
	uint64_t res;
	va_list va;

	va_start(va, fmt);
	res = pconf_get64v(pc, def, fmt, va);
	va_end(va);

	return (res);
}

API_DRETURN
pconf_getdblv(pconf_t *pc, double def, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res != EPHIDGET_OK)
		return (def);

	switch (ent->type) {
	case PHIDGETCONFIG_I64:
		return ((double)ent->e_i64);
	case PHIDGETCONFIG_U64:
		return ((double)ent->e_u64);
	case PHIDGETCONFIG_NUMBER:
		return (ent->e_num);
	case PHIDGETCONFIG_BOOLEAN:
		return ((double)ent->e_bool);
	case PHIDGETCONFIG_NULL:
		return (0);
	default:
		return (def);
	}
}

API_DRETURN
pconf_getdbl(pconf_t *pc, double def, const char *fmt, ...) {
	double res;
	va_list va;

	va_start(va, fmt);
	res = pconf_getdblv(pc, def, fmt, va);
	va_end(va);

	return (res);
}

API_CRETURN
pconf_getstrv(pconf_t *pc, const char *def, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res != EPHIDGET_OK)
		return (def);

	if (ent->type != PHIDGETCONFIG_STRING)
		return (def);

	return (ent->e_str);
}

API_CRETURN
pconf_getstr(pconf_t *pc, const char *def, const char *fmt, ...) {
	const char *res;
	va_list va;

	va_start(va, fmt);
	res = pconf_getstrv(pc, def, fmt, va);
	va_end(va);

	return (res);
}

API_IRETURN
pconf_getboolv(pconf_t *pc, int def, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res != EPHIDGET_OK)
		return (def);

	switch (ent->type) {
	case PHIDGETCONFIG_I64:
		return (ent->e_i64 != 0);
	case PHIDGETCONFIG_U64:
		return (ent->e_u64 != 0);
	case PHIDGETCONFIG_NUMBER:
		return (ent->e_num != 0);
	case PHIDGETCONFIG_BOOLEAN:
		return (ent->e_bool);
	case PHIDGETCONFIG_NULL:
		return (0);
	default:
		return (def);
	}
}

API_IRETURN
pconf_getbool(pconf_t *pc, int def, const char *fmt, ...) {
	va_list va;
	int res;

	va_start(va, fmt);
	res = pconf_getboolv(pc, def, fmt, va);
	va_end(va);

	return (res);
}

API_IRETURN
pconf_existsv(pconf_t *pc, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res == EPHIDGET_OK)
		return (1);
	return (0);
}

API_IRETURN
pconf_exists(pconf_t *pc, const char *fmt, ...) {
	va_list va;
	int res;

	va_start(va, fmt);
	res = pconf_existsv(pc, fmt, va);
	va_end(va);

	return (res);
}

API_IRETURN
pconf_isblockv(pconf_t *pc, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res != EPHIDGET_OK)
		return (0);

	return (ent->type == PHIDGETCONFIG_BLOCK);
}

API_IRETURN
pconf_isblock(pconf_t *pc, const char *fmt, ...) {
	va_list va;
	int res;

	va_start(va, fmt);
	res = pconf_isblockv(pc, fmt, va);
	va_end(va);

	return (res);
}

API_IRETURN
pconf_isarrayv(pconf_t *pc, const char *fmt, va_list va) {
	PhidgetReturnCode res;
	pconfentry_t *ent;

	res = pconf_getentryv(pc, 0, &ent, fmt, va);
	if (res != EPHIDGET_OK)
		return (0);

	return (ent->type == PHIDGETCONFIG_ARRAY);
}

API_IRETURN
pconf_isarray(pconf_t *pc, const char *fmt, ...) {
	va_list va;
	int res;

	va_start(va, fmt);
	res = pconf_isarrayv(pc, fmt, va);
	va_end(va);

	return (res);
}

/****************************************************************************************************
 * CREATION
 *
 * Creates and inserts the entry.  The value is NOT assigned.
 */
static PhidgetReturnCode
pconf_addentry(pconf_t *pc, pconfentry_t **entry, pconftype_t type, const char *path) {
	char comp[PHIDGETCONFIG_COMP_MAX];
	pconfentry_t *dent;
	PhidgetReturnCode res;

	res = pconf_getblock(pc, 1, &dent, path);
	if (res != EPHIDGET_OK)
		return (res);

	if (getlastcomponent(path, comp, sizeof (comp)) == NULL)
		return (EPHIDGET_INVALIDARG);

	res = pconf_mkentry(entry, comp, type, dent->type == PHIDGETCONFIG_ARRAY);
	if (res != EPHIDGET_OK)
		return (res);

	if (RB_INSERT(pconfentries, &dent->value.entries, *entry) != NULL) {
		pconf_releaseentry(*entry);
		return (EPHIDGET_DUPLICATE);
	}
	dent->cnt++;

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_addblockv(pconf_t *pc, const char *fmt, va_list va) {
	char path[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *entry;
	int n;

	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	if (n >= (int)sizeof (path))
		return (EPHIDGET_NOSPC);

	res = pconf_addentry(pc, &entry, PHIDGETCONFIG_BLOCK, path);
	if (res != EPHIDGET_OK)
		return (res);

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_addblock(pconf_t *pc, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_addblockv(pc, fmt, va);
	va_end(va);

	return (res);
}

API_PRETURN
pconf_addarrayv(pconf_t *pc, const char *fmt, va_list va) {
	char path[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *entry;
	int n;

	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	if (n >= (int)sizeof (path))
		return (EPHIDGET_NOSPC);

	res = pconf_addentry(pc, &entry, PHIDGETCONFIG_ARRAY, path);
	if (res != EPHIDGET_OK)
		return (res);

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_addarray(pconf_t *pc, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_addarrayv(pc, fmt, va);
	va_end(va);

	return (res);
}

API_PRETURN
pconf_addstrv(pconf_t *pc, const char *val, const char *fmt, va_list va) {
	char path[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *entry;
	int n;

	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	if (n >= (int)sizeof (path))
		return (EPHIDGET_NOSPC);

	res = pconf_addentry(pc, &entry, PHIDGETCONFIG_STRING, path);
	if (res != EPHIDGET_OK)
		return (res);

	entry->e_str = mos_strdup(val, NULL);

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_addstr(pconf_t *pc, const char *val, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_addstrv(pc, val, fmt, va);
	va_end(va);

	return (res);
}

API_PRETURN
pconf_addnumv(pconf_t *pc, double val, const char *fmt, va_list va) {
	char path[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *entry;
	int n;

	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	if (n >= (int)sizeof (path))
		return (EPHIDGET_NOSPC);

	res = pconf_addentry(pc, &entry, PHIDGETCONFIG_NUMBER, path);
	if (res != EPHIDGET_OK)
		return (res);

	entry->e_num = val;

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_addnum(pconf_t *pc, double val, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_addnumv(pc, val, fmt, va);
	va_end(va);

	return (res);
}

API_PRETURN
pconf_addboolv(pconf_t *pc, int val, const char *fmt, va_list va) {
	char path[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *entry;
	int n;

	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	if (n >= (int)sizeof (path))
		return (EPHIDGET_NOSPC);

	res = pconf_addentry(pc, &entry, PHIDGETCONFIG_BOOLEAN, path);
	if (res != EPHIDGET_OK)
		return (res);

	entry->e_bool = val;

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_addbool(pconf_t *pc, int val, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_addboolv(pc, val, fmt, va);
	va_end(va);

	return (res);
}


API_PRETURN
pconf_addiv(pconf_t *pc, int64_t val, const char *fmt, va_list va) {
	char path[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *entry;
	int n;

	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	if (n >= (int)sizeof (path))
		return (EPHIDGET_NOSPC);

	res = pconf_addentry(pc, &entry, PHIDGETCONFIG_I64, path);
	if (res != EPHIDGET_OK)
		return (res);

	entry->e_i64 = val;

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_addi(pconf_t *pc, int64_t val, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_addiv(pc, val, fmt, va);
	va_end(va);

	return (res);
}

API_PRETURN
pconf_adduv(pconf_t *pc, uint64_t val, const char *fmt, va_list va) {
	char path[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *entry;
	int n;

	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	if (n >= (int)sizeof (path))
		return (EPHIDGET_NOSPC);

	res = pconf_addentry(pc, &entry, PHIDGETCONFIG_U64, path);
	if (res != EPHIDGET_OK)
		return (res);

	entry->e_u64 = val;

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_addu(pconf_t *pc, uint64_t val, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_adduv(pc, val, fmt, va);
	va_end(va);

	return (res);
}

/*
 * Creates an new entry at the given path, and tries to figure out the type of the value.
 */
API_PRETURN
pconf_addv(pconf_t *pc, const char *val, const char *fmt, va_list va) {
	char path[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *entry;
	pconfvalue_t eval;
	pconftype_t type;
	size_t n;

	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	if (n >= (int)sizeof (path))
		return (EPHIDGET_NOSPC);

	res = pconf_addentry(pc, &entry, PHIDGETCONFIG_NULL, path);
	if (res != EPHIDGET_OK)
		return (res);

	res = pconf_detecttype(val, &type, &eval);
	if (res != EPHIDGET_OK)
		return (res);

	entry->value.val = eval;
	entry->type = type;

	if (type == PHIDGETCONFIG_STRING)
		entry->e_str = mos_strdup(val, NULL);

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_add(pconf_t *pc, const char *val, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_addv(pc, val, fmt, va);
	va_end(va);

	return (res);
}

/*
 * Creates the entry, and casts the type to that detected.
 * If the entry already exists, its type is updated.
 */
API_PRETURN
pconf_setv(pconf_t *pc, const char *val, const char *fmt, va_list va) {
	char key[PHIDGETCONFIG_PATH_MAX];
	PhidgetReturnCode res;
	pconfentry_t *entry;
	pconftype_t type;
	pconfvalue_t eval;
	size_t n;

	n = mos_vsnprintf(key, sizeof (key), fmt, va);
	if (n >= sizeof (key))
		return (EPHIDGET_NOSPC);

	res = pconf_add(pc, val, "%s", key);
	if (res == EPHIDGET_OK)
		return (res);
	if (res != EPHIDGET_DUPLICATE)
		return (res);

	res = pconf_detecttype(val, &type, &eval);
	if (res != EPHIDGET_OK)
		return (res);

	res = pconf_getentry(pc, 0, &entry, "%s", key);
	if (res != EPHIDGET_OK)
		return (res);

	if (entry->type == PHIDGETCONFIG_BLOCK || entry->type == PHIDGETCONFIG_ARRAY)
		return (EPHIDGET_INVALID);

	if (entry->type == PHIDGETCONFIG_STRING)
		mos_free(entry->e_str, MOSM_FSTR);

	entry->type = type;
	entry->value.val = eval;

	if (entry->type == PHIDGETCONFIG_STRING)
		entry->e_str = mos_strdup(eval.cstr, NULL);

	return (EPHIDGET_OK);
}

API_PRETURN
pconf_set(pconf_t *pc, const char *val, const char *fmt, ...) {
	PhidgetReturnCode res;
	va_list va;

	va_start(va, fmt);
	res = pconf_setv(pc, val, fmt, va);
	va_end(va);

	return (res);
}

#define PCLEN ((int)((pcfsz) - ((c) - (pcf))))

#define PCADD(...) do {									\
	int __len__;										\
	int __n__;											\
	__len__ = PCLEN;									\
	__n__ = mos_snprintf(c, __len__, __VA_ARGS__);		\
	if (__n__ >= __len__)								\
		return (EPHIDGET_NOSPC);						\
	c += __n__;											\
} while (0)

#define PCADDDEPTH() do {								\
	int _i;												\
	for (_i = 0; _i < dep; _i++)						\
		PCADD("\t");									\
} while (0)

static PhidgetReturnCode
pconf_renderentrypc(pconfentry_t *entry, char **_c, char *pcf, size_t pcfsz, int oecnt, int array, int dep) {
	char buf[PHIDGETCONFIG_STR_MAX];
	PhidgetReturnCode res;
	pconfentry_t *ent;
	int ecnt;
	char *c;

	c = *_c;

	PCADDDEPTH();

	if (array && oecnt > 0)
		PCADD(", ");

	if (!array) {
		if (entry->type == PHIDGETCONFIG_BLOCK)
			PCADD("%s ", entry->key);
		else
			PCADD("%s: ", entry->key);
	}

	switch (entry->type) {
	case PHIDGETCONFIG_BLOCK:
		PCADD("{\n");
		ecnt = 0;
		RB_FOREACH(ent, pconfentries, &entry->value.entries) {
			res = pconf_renderentrypc(ent, &c, pcf, pcfsz, ecnt, 0, dep + 1);
			if (res != EPHIDGET_OK)
				return (res);
			ecnt++;
		}
		PCADDDEPTH();
		PCADD("}");

		break;
	case PHIDGETCONFIG_ARRAY:
		PCADD("[");
		ecnt = 0;
		RB_FOREACH(ent, pconfentries, &entry->value.entries) {
			res = pconf_renderentrypc(ent, &c, pcf, pcfsz, ecnt, 1, 0);
			if (res != EPHIDGET_OK)
				return (res);
			ecnt++;
		}

		PCADD("]");
		break;
	case PHIDGETCONFIG_BOOLEAN:
		PCADD("%s", entry->e_bool ? "true" : "false");
		break;
	case PHIDGETCONFIG_STRING:
		PCADD("'%s'", strescape('\'', entry->e_str, buf, sizeof (buf)));
		break;
	case PHIDGETCONFIG_NUMBER:
		PCADD("%s", jsonnumber(entry->e_num, buf, sizeof (buf)));
		break;
	case PHIDGETCONFIG_I64:
		PCADD("%"PRId64, entry->e_i64);
		break;
	case PHIDGETCONFIG_U64:
		PCADD("%"PRIu64, entry->e_u64);
		break;
	case PHIDGETCONFIG_NULL:
		PCADD("null");
		break;
	default:
		return (EPHIDGET_UNEXPECTED);
	}
	if (!array && dep >= 1)
		PCADD("\n");

	*_c = c;
	return (EPHIDGET_OK);
}

API_PRETURN
pconf_renderpc(pconf_t *pc, char *pcf, size_t pcfsz) {
	PhidgetReturnCode res;
	pconfentry_t *entry;
	int ecnt;
	char *c;

	if (pcfsz < 3)
		return (EPHIDGET_NOSPC);

	c = pcf;

	ecnt = 0;
	RB_FOREACH(entry, pconfentries, &pc->pc_root) {
		res = pconf_renderentrypc(entry, &c, pcf, pcfsz, ecnt, pc->root->type == PHIDGETCONFIG_ARRAY, 0);
		if (res != EPHIDGET_OK)
			return (res);
		ecnt++;
	}

	return (EPHIDGET_OK);
}
