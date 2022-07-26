#include "util/json.h"
#include "util/jsmn.h"

/*
 * %c		- uint8_t
 * %h		- int16_t
 * %uh		- uint16_t
 * %d		- int32_t
 * %u		- uint32_t
 * %ld		- int64_t
 * %lu		- uint64_t
 * %f		- float
 * %g		- double
 * %s		- str
 * %O		- OBJECT
 * %Au		- ARRAY of uint32_t - 2 args, a uint32 length and the array
 */

API_CRETURN
json_escape(const char *in, char *out, size_t len) {
	const char *p;
	char *o;

	for (p = in, o = out; *p && (size_t)(o - out) < len; p++, o++) {
		if ((uint8_t)*p > 0x1F && *p != '"' && *p != '\\') {
			*o = *p;
			continue;
		}
		*o++ = '\\';

		switch (*p) {
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
			mos_snprintf(o, len - (o - out), "u%04x", *p);
			o += 4;
		}
	}

	if ((size_t)(o - out) >= len)
		return (NULL);

	*o = '\0';
	return (out);
}

API_CRETURN_HDR
json_unescape(char *in) {
	uint32_t c;
	char val[5];
	char *p;
	char *o;

	for (p = o = in; *p; p++, o++) {
		if (*p != '\\') {
			*o = *p;
			continue;
		}
		p++;
		switch (*p) {
		default:
		case '"':
		case '\\':
		case '/':
			*o = *p;
			break;
		case 'b':
			*o = '\b';
			break;
		case 'f':
			*o = '\f';
			break;
		case 'n':
			*o = '\n';
			break;
		case 'r':
			*o = '\r';
			break;
		case 't':
			*o = '\t';
			break;
		case 'u':
			p++;
			mos_strncpy(val, p, 4);
			val[4] = '\0';
			if (mos_strtou32(val, 16, &c) == 0)
				*o = (char)c;
			else
				*o = '!';
			p += 3;
		}
	}

	*o = '\0';
	return (in);
}

int
parseJSON(const char *json, uint32_t jsonlen, char *allocbuf, size_t allocbufsz, const char *fmt, ...) {
	va_list va;
	int res;

	va_start(va, fmt);
	res = parseJSONv(json, jsonlen, allocbuf, allocbufsz, fmt, va);
	va_end(va);

	return (res);
}

const char *
mkJSONc(char *json, uint32_t jsonlen, const char *fmt, ...) {
	va_list va;
	int res;

	va_start(va, fmt);
	res = mkJSONv(json, jsonlen, fmt, va);
	va_end(va);

	if (res > (int)jsonlen)
		return ("");

	return (json);
}

int
mkJSON(char *json, uint32_t jsonlen, const char *fmt, ...) {
	va_list va;
	int res;

	va_start(va, fmt);
	res = mkJSONv(json, jsonlen, fmt, va);
	va_end(va);

	return (res);
}

int
mkJSONv(char *json, uint32_t jsonlen, const char *fmt, va_list va) {
	char buf[65536];	/* must be large enough to handle dictionary reply */
	char str[512];

	const char *tmp;
	char *jsonp;
	char *p;
	int ch;
	int n;
	int i;

	uint32_t userArrSize;
	uint32_t *userArr;

#define JA(c) do {								\
	if ((unsigned)(jsonp - json) < jsonlen - 1)	\
		*jsonp = (char)(c);						\
	jsonp++;									\
	*jsonp = '\0';								\
} while (0)

#define JAP(...) do {																		\
	jsonp += phid_snprintf_c(jsonp,															\
	  (unsigned)(jsonp - json) < jsonlen ? jsonlen - (jsonp - json) : 0, __VA_ARGS__);		\
} while (0)

	jsonp = json;

	for (;;) {
		memset(str, 0, sizeof(str));
		p = str;

		while ((ch = (unsigned char)*fmt++) != '%') {
			switch (ch) {
			case '\0':
				return ((int)(jsonp - json));
			case '[':
				JA(ch);
				break;
			case '{':
				JA(ch);
				break;
			case '}':
			case ']':
			case ',':
			case ':':
			case '=':
				if (p != str) {
					JAP("\"%s\"", str);
					memset(str, 0, sizeof(str));
					p = str;
				}
				if (ch == '=')
					JA(':');
				else
					JA(ch);
				break;
			default:
				if (p - str < (int)sizeof(str)) {
					*p = ch;
					p++;
				}
			}
		}

	reswitch:switch (ch = (unsigned char)*fmt++) {
	case '0': case '1': case '2': case '3': case '4':
	case '5': case '6': case '7': case '8': case '9':
		for (n = 0;; ++fmt) {
			n = n * 10 + ch - '0';
			ch = *fmt;
			if (ch < '0' || ch > '9')
				break;
		}
		goto reswitch;
	case 'c':
		JAP("%c", (uint8_t)va_arg(va, int));
		break;
	case 'l':
		switch ((uint8_t)*fmt) {
		case 'u':
			JAP("%"PRIu64, va_arg(va, uint64_t));
			fmt++;
			break;
		case 'd':
			fmt++;
			JAP("%"PRId64, va_arg(va, int64_t));
			break;
		default:
			return (-1);
		}
		break;
	case 'd':
		JAP("%d", va_arg(va, int));
		break;
	case 'u':
		JAP("%u", va_arg(va, uint32_t));
		break;
	case 'A':
		switch ((uint8_t)*fmt) {
		case 'u':
			userArrSize = va_arg(va, uint32_t);
			userArr = va_arg(va, uint32_t *);
			JA('[');
			for (i = 0; i < (int)userArrSize; i++) {
				JAP("%u", userArr[i]);
				if (i < (int)(userArrSize - 1))
					JA(',');
			}
			JA(']');
			fmt++;
			break;
		default:
			return (-1);
		}
		break;
	case 'f':
	case 'g':
		JAP("%g", va_arg(va, double));
		break;
	case 's':
		tmp = json_escape(va_arg(va, const char *), buf, sizeof(buf));
		if (tmp == NULL)
			return (-1);
		JAP("\"%s\"", tmp);
		break;
	}
	}
}

/*
 * Returns the value token following the first token in the object whose string value matches 'find'.
 */
static pjsmntok_t *
findTokenInObject(const char *json, pjsmntok_t *token, const char *find) {
	char strbuf[JSON_STRING_MAX];
	const char *str;
	int size;
	int off;

	if (token->type != JSMN_OBJECT)
		return (NULL);

	size = token->size;
	token++;

	for (off = 0; off < size; off++, token++) {

		str = pjsmn_string(json, token, strbuf, sizeof(strbuf));
		if (str == NULL)
			return (NULL);

		if (strcmp(str, find) == 0)
			return (++token);

		token++;	/* skip the value */

		// Jump over arrays
		if (token->type == JSMN_ARRAY)
			token += token->size;

		// XXX - this won't handle embedded objects properly
	}

	return (NULL);
}

#define GETU64(t) do {								\
	token = findTokenInObject(json, tobj, find);	\
	if (token == NULL) {							\
		if (!optional)								\
			return (-1);							\
		break;										\
	}												\
	res = pjsmn_uint64(json, token, &u64);			\
	if (res != 0)									\
		return (-1 * off);							\
	*(va_arg(va, t *)) = (t)u64;					\
} while (0)

#define GETI64(t) do {								\
	token = findTokenInObject(json, tobj, find);	\
	if (token == NULL) {							\
		if (!optional)								\
			return (-1);							\
		break;										\
	}												\
	res = pjsmn_int64(json, token, &i64);			\
	if (res != 0)									\
		return (-1 * off);							\
	*(va_arg(va, t *)) = (t)i64;					\
} while (0)

#define GETDBL(t) do {								\
	token = findTokenInObject(json, tobj, find);	\
	if (token == NULL) {							\
		if (!optional)								\
			return (-1);							\
		break;										\
	}												\
	res = pjsmn_double(json, token, &dbl);			\
	if (res != 0)									\
		return (-1 * off);							\
	*va_arg(va, t *) = (t)dbl;						\
} while (0)

int
parseJSONv(const char *json, uint32_t jsonlen, char *allocbuf, size_t allocbufsz,
  const char *fmt, va_list va) {
	char strbuf[JSON_STRING_MAX];
	pjsmntok_t tokens[256];
	pjsmntok_t *token;
	pjsmntok_t *tobj;
	pjsmn_parser p;
	int optional;
	char find[32];
	int findoff;
	int arrsize;
	size_t len;
	int size;
	int off;
	int res;
	int ch;
	int n;
	int i;

	uint32_t *userArrSize;
	uint32_t *userArr;

	uint64_t u64;
	int64_t i64;
	double dbl;
	char *str;

	pjsmn_init(&p);
	res = pjsmn_parse(&p, json, jsonlen, tokens, 256);
	if (res < 0)
		return (-1);

	/* must start with an object */
	if (tokens[0].type != JSMN_OBJECT)
		return (-1);

	tobj = token = &tokens[0];
	size = res;
	len = 0;
	res = 0;

	for (off = 0; off < size; off++) {
		memset(find, 0, sizeof(find));
		findoff = 0;

		optional = 0;

		while ((ch = (unsigned char)*fmt++) != '%') {
			if (ch == '\0')
				return (off);
			/*
			 * Ignore these just to make the input easier to handle for the user.
			 */
			if (ch == ' ' || ch == '\t' || ch == ',' || ch == '=' || ch == ':' || ch == '{' || ch == '}')
				continue;
			if (ch == '?') {
				optional = 1;
				continue;
			}
			find[findoff] = (char)ch;
			findoff++;
			if (findoff >= (int)sizeof(find))
				return (-1);
		}

	reswitch:switch (ch = (unsigned char)*fmt++) {
	case '0': case '1': case '2': case '3': case '4':
	case '5': case '6': case '7': case '8': case '9':
		for (n = 0;; ++fmt) {
			n = n * 10 + ch - '0';
			ch = *fmt;
			if (ch < '0' || ch > '9')
				break;
		}
		goto reswitch;
	case 'O':
		if (token->type != JSMN_OBJECT) {
			token = findTokenInObject(json, tobj, find);
			if (token == NULL || token->type != JSMN_OBJECT)
				return (-1);
		}
		*(va_arg(va, uint32_t *)) = token->size;
		tobj = token;
		token++;
		break;
	case 'A':
		switch ((uint8_t)*fmt) {
		case 'u':
			token = findTokenInObject(json, tobj, find);
			if (token == NULL) {
				if (!optional)
					return (-1);
				break;
			}
			if (token->type != JSMN_ARRAY)
				return (-1);

			arrsize = token->size;

			userArrSize = (va_arg(va, uint32_t *));
			userArr = (va_arg(va, uint32_t *));

			// array passed in is too small
			if (arrsize > (int)*userArrSize)
				return (-1);

			// Return the array count
			*userArrSize = arrsize;

			if (arrsize > 0) {
				token++;

				for (i = 0; i < arrsize; i++, token++, off++) {
					res = pjsmn_uint64(json, token, &u64);
					if (res != 0)
						return (-1 * off);
					userArr[i] = (uint32_t)u64;
				}
			}

			fmt++;
			break;
		default:
			return (-1);
		}
		break;
	case 'c':
		GETU64(uint8_t);
		break;
	case 'h':
		GETI64(int16_t);
		break;
	case 'd':
		GETI64(int32_t);
		break;
	case 'u':
		GETU64(uint32_t);
		break;
	case 'l':
		switch ((uint8_t)*fmt) {
		case 'd':
			GETI64(int64_t);
			fmt++;
			break;
		case 'u':
			GETU64(uint64_t);
			fmt++;
			break;
		default:
			return (-1);
		}
		break;
	case 'f':
		GETDBL(float);
		break;
	case 'g':
		GETDBL(double);
		break;
	case 's':
		token = findTokenInObject(json, tobj, find);
		if (token == NULL) {
			if (!optional)
				return (-1);
			*(va_arg(va, const char **)) = NULL;
			break;
		}
		str = pjsmn_string(json, token, strbuf, sizeof(strbuf));
		if (str == NULL)
			return (-1 * off);
		if (allocbuf == NULL) {
			*(va_arg(va, const char **)) = mos_strdup(json_unescape(str), NULL);
		} else {
			*(va_arg(va, const char **)) = allocbuf;
			len = mos_strlcpy(allocbuf, json_unescape(str), allocbufsz - len);
			len++;
			if (len >= allocbufsz) {
				allocbuf = NULL;
				allocbufsz = 0;
			} else {
				allocbufsz -= len;
				allocbuf += len;
			}
		}
		break;
	}
	}

	return (off);
}
