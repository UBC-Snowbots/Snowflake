#include "kv.h"

int
mkkvent(kvent_t **kve, const char *key) {

	*kve = mos_malloc(sizeof (kvent_t));
	(*kve)->key = mos_strdup(key, NULL);
	(*kve)->type = MOSKV_TKN;

	return (0);
}

int
mkkvent_str(kvent_t **kve, const char *key, const char *val) {

	*kve = mos_malloc(sizeof (kvent_t));
	(*kve)->key = mos_strdup(key, NULL);
	(*kve)->val = mos_strdup(val, NULL);
	(*kve)->type = MOSKV_STR;

	return (0);
}

int
mkkvent_tkn(kvent_t **kve, const char *key, const char *val) {
	char tmp[512];

	*kve = mos_malloc(sizeof (kvent_t));
	(*kve)->key = mos_strdup(key, NULL);
	(*kve)->val = mos_strdup(mos_strtrim(val, tmp, sizeof (tmp)), NULL);
	(*kve)->type = MOSKV_TKN;

	return (0);
}

int
mkkvent_int(kvent_t **kve, const char *key, int val) {
	uint32_t len;

	*kve = mos_malloc(sizeof (kvent_t));
	(*kve)->key = mos_strdup(key, NULL);
	mos_asprintf(&(*kve)->val, &len, "%d", val);
	(*kve)->type = MOSKV_INT;

	return (0);
}

int
mkkvent_int64(kvent_t **kve, const char *key, int64_t val) {
	uint32_t len;

	*kve = mos_malloc(sizeof (kvent_t));
	(*kve)->key = mos_strdup(key, NULL);
	mos_asprintf(&(*kve)->val, &len, "%"PRId64, val);
	(*kve)->type = MOSKV_INT;

	return (0);
}

int
mkkvent_bool(kvent_t **kve, const char *key, int val) {
	uint32_t len;

	*kve = mos_malloc(sizeof (kvent_t));
	(*kve)->key = mos_strdup(key, NULL);
	mos_asprintf(&(*kve)->val, &len, "%d", val);
	(*kve)->type = MOSKV_BOOL;

	return (0);
}

int
mkkvent_kv(kvent_t **kve, int type, char *key, char *val) {

	*kve = mos_malloc(sizeof (kvent_t));
	(*kve)->type = (uint8_t)type;
	(*kve)->key = key;
	(*kve)->val = val;

	return (0);
}

void
kventfree(kvent_t **kve) {

	mos_free((*kve)->key, mos_strlen((*kve)->key) + 1);
	if ((*kve)->val != NULL)
		mos_free((*kve)->val, mos_strlen((*kve)->val) + 1);
	mos_free(*kve, sizeof (kvent_t));
	*kve = NULL;
}

void
kvent_setvalue(kvent_t *this, const char *val) {

	if (this->val != NULL)
		mos_free(this->val, mos_strlen(this->val) + 1);

	if (val != NULL)
		this->val = mos_strdup(val, NULL);
	else
		this->val = NULL;
}

int
kvent_getstr(kvent_t *this,  char *vbuf, uint32_t vbufsz) {

	if (this->val == NULL) {
		vbuf[0] = '\0';
		return (0);
	}
	mos_strlcpy(vbuf, this->val, vbufsz);

	return (0);
}

const char *
kvent_getstrc(kvent_t *this) {

	return (this->val);
}

uint64_t
kvent_getui64(kvent_t *this, uint64_t def) {
	uint64_t ret;
	int err;

	if (this->val == NULL)
		return (def);

	err = mos_strtou64(this->val, 0, &ret);
	if (err != 0)
		return (def);

	return (ret);
}

uint32_t
kvent_getui32(kvent_t *this, uint32_t def) {
	uint32_t ret;
	int err;

	if (this->val == NULL)
		return (def);

	err = mos_strtou32(this->val, 0, &ret);
	if (err != 0)
		return (def);

	return (ret);
}

int64_t
kvent_geti64(kvent_t *this, int64_t def) {
	int64_t ret;
	int err;

	if (this->val == NULL)
		return (def);

	err = mos_strto64(this->val, 0, &ret);
	if (err != 0)
		return (def);

	return (ret);
}

int32_t
kvent_geti32(kvent_t *this, int32_t def) {
	int32_t ret;
	int err;

	if (this->val == NULL)
		return (def);

	err = mos_strto32(this->val, 0, &ret);
	if (err != 0)
		return (def);

	return (ret);
}

int
kvent_getbool(kvent_t *this, int def) {
	int32_t ret;
	int err;

	if (this->val == NULL)
		return (def);

	if (mos_strcasecmp(this->val, "true") == 0)
		return (1);
	if (mos_strcasecmp(this->val, "yes") == 0)
		return (1);

	err = mos_strto32(this->val, 0, &ret);
	if (err != 0)
		return (0);

	return (ret);
}

int
kvent_gettext(kvent_t *this, char *tbuf, uint32_t tbuflen, uint32_t *lenp) {
	uint32_t len;

	switch(this->type) {
	case MOSKV_BOOL:
		if (kvent_getbool(this, 0))
			len = mos_snprintf(tbuf, tbuflen, "%s=true", this->key);
		else
			len = mos_snprintf(tbuf, tbuflen, "%s=false", this->key);
		break;
	case MOSKV_STR:
		len = mos_snprintf(tbuf, tbuflen, "%s=\"%s\"", this->key, this->val);
		break;
	default:
		len = mos_snprintf(tbuf, tbuflen, "%s=%s", this->key, this->val);
		break;
	}

	if (lenp != NULL)
		*lenp = len;

	if (len > tbuflen)
		return (MOSN_NOSPC);

	return (0);
}

const char *
kvent_getkey(kvent_t *this) {

	return (this->key);
}
