#include <stdlib.h>

#include "mos/mos_os.h"
#include "jsmn.h"

#include "phidgetbase.h"

char *
pjsmn_string(const char *jsonsrc, pjsmntok_t *tok, char *out, size_t outlen) {
	size_t n;

	if (tok->type != JSMN_STRING)
		return (NULL);

	n = MOS_MIN((uint32_t)(tok->end - tok->start), outlen);

	mos_strncpy(out, &jsonsrc[tok->start], n);
	out[n] = '\0';

	return (out);
}

int
pjsmn_boolean(const char *jsonsrc, pjsmntok_t *tok) {

	if (mos_strncmp(jsonsrc + tok->start, "true", tok->end - tok->start) == 0)
		return (1);
	if (mos_strncmp(jsonsrc + tok->start, "false", tok->end - tok->start) == 0)
		return (0);

	return (-1);
}

int
pjsmn_uint64(const char *jsonsrc, pjsmntok_t *tok, uint64_t *ret) {
	char buf[32];
	size_t n;
	int err;

	if (tok->type != JSMN_PRIMITIVE)
		return (-1);

	n = MOS_MIN((uint32_t)(tok->end - tok->start), sizeof (buf));

	mos_strncpy(buf, &jsonsrc[tok->start], n);
	buf[n] = '\0';
	err = mos_strtou64(buf, 10, ret);
	if (err != 0)
		return (-1);

	return (0);
}

int
pjsmn_int64(const char *jsonsrc, pjsmntok_t *tok, int64_t *ret) {
	char buf[32];
	size_t n;
	int err;

	if (tok->type != JSMN_PRIMITIVE)
		return (-1);

	n = MOS_MIN((uint32_t)(tok->end - tok->start), sizeof (buf));

	mos_strncpy(buf, &jsonsrc[tok->start], n);
	buf[n] = '\0';

	err = mos_strto64(buf, 10, ret);
	if (err != 0)
		return (-1);

	return (0);
}

int
pjsmn_double(const char *jsonsrc, pjsmntok_t *tok, double *ret) {
	char buf[64];
	size_t n;
	char *end;

	if (tok->type != JSMN_PRIMITIVE)
		return (-1);

	n = MOS_MIN((uint32_t)(tok->end - tok->start), sizeof (buf));

	mos_strncpy(buf, &jsonsrc[tok->start], n);
	buf[n] = '\0';

	*ret = phid_strtod_c(buf, &end);
	if (*ret == 0 && end == buf)
		return (-1);

	return (0);
}

/**
 * Allocates a fresh unused token from the token pull.
 */
static pjsmntok_t *
jsmn_alloc_token(pjsmn_parser *parser, pjsmntok_t *tokens, size_t num_tokens) {
	pjsmntok_t *tok;

	if (parser->toknext >= num_tokens)
		return (NULL);

	tok = &tokens[parser->toknext++];
	tok->start = tok->end = -1;
	tok->size = 0;
#ifdef JSMN_PARENT_LINKS
	tok->parent = -1;
#endif
	return (tok);
}

/**
 * Fills token type and boundaries.
 */
static void
jsmn_fill_token(pjsmntok_t *token, pjsmntype_t type, int start, int end) {

	token->type = type;
	token->start = start;
	token->end = end;
	token->size = 0;
}

/**
 * Fills next available token with JSON primitive.
 */
static pjsmnerr_t
jsmn_parse_primitive(pjsmn_parser *parser, const char *js, size_t len,
  pjsmntok_t *tokens, size_t num_tokens) {
	pjsmntok_t *token;
	int start;

	start = parser->pos;

	for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
		switch (js[parser->pos]) {
#ifndef JSMN_STRICT
		/* In strict mode primitive must be followed by "," or "}" or "]" */
		case ':':
#endif
		case '\t': case '\r': case '\n': case ' ':
		case ',': case ']': case '}':
			goto found;
		}
		if (js[parser->pos] < 32 || js[parser->pos] >= 127) {
			parser->pos = start;
			return JSMN_ERROR_INVAL;
		}
	}
#ifdef JSMN_STRICT
	/* In strict mode primitive must be followed by a comma/object/array */
	parser->pos = start;
	return (JSMN_ERROR_PART);
#endif

found:
	if (tokens == NULL) {
		parser->pos--;
		return 0;
	}
	token = jsmn_alloc_token(parser, tokens, num_tokens);
	if (token == NULL) {
		parser->pos = start;
		return JSMN_ERROR_NOMEM;
	}
	jsmn_fill_token(token, JSMN_PRIMITIVE, start, parser->pos);
#ifdef JSMN_PARENT_LINKS
	token->parent = parser->toksuper;
#endif
	parser->pos--;
	return (0);
}

/**
 * Fills next token with JSON string.
 */
static pjsmnerr_t
jsmn_parse_string(pjsmn_parser *parser, const char *js, size_t len,
  pjsmntok_t *tokens, size_t num_tokens) {
	pjsmntok_t *token;
	int start;
	char c;
	int i;

	start = parser->pos;
	parser->pos++;

	/* Skip starting quote */
	for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
		c = js[parser->pos];

		/* Quote: end of string */
		if (c == '\"') {
			if (tokens == NULL)
				return (0);
			token = jsmn_alloc_token(parser, tokens, num_tokens);
			if (token == NULL) {
				parser->pos = start;
				return (JSMN_ERROR_NOMEM);
			}
			jsmn_fill_token(token, JSMN_STRING, start + 1, parser->pos);
#ifdef JSMN_PARENT_LINKS
			token->parent = parser->toksuper;
#endif
			return 0;
		}

		/* Backslash: Quoted symbol expected */
		if (c == '\\' && parser->pos + 1 < len) {
			parser->pos++;
			switch (js[parser->pos]) {
			/* Allowed escaped symbols */
			case '\"': case '/': case '\\': case 'b':
			case 'f': case 'r': case 'n': case 't':
				break;
			/* Allows escaped symbol \uXXXX */
			case 'u':
				parser->pos++;
				for (i = 0;
				i < 4 && parser->pos < len && js[parser->pos] != '\0'; i++) {
				  /* If it isn't a hex character we have an error */
					/* 0-9 */
					if (!((js[parser->pos] >= 48 && js[parser->pos] <= 57) ||
					  /* A-F */
						(js[parser->pos] >= 65 && js[parser->pos] <= 70) ||
						/* a-f */
						(js[parser->pos] >= 97 && js[parser->pos] <= 102))) {
						parser->pos = start;
						return (JSMN_ERROR_INVAL);
					}
					parser->pos++;
				}
				parser->pos--;
				break;
			/* Unexpected symbol */
			default:
				parser->pos = start;
				return (JSMN_ERROR_INVAL);
			}
		}
	}
	parser->pos = start;

	return (JSMN_ERROR_PART);
}

/**
 * Parse JSON string and fill tokens.
 */
pjsmnerr_t
pjsmn_parse(pjsmn_parser *parser, const char *js, size_t len, pjsmntok_t *tokens,
  unsigned int num_tokens) {
	pjsmntok_t *token;
	pjsmntype_t type;
	pjsmnerr_t r;
	int count;
	char c;
	int i;

	count = 0;
	for (; parser->pos < len && js[parser->pos] != '\0'; parser->pos++) {
		c = js[parser->pos];
		switch (c) {
		case '{': case '[':
			count++;
			if (tokens == NULL)
				break;
			token = jsmn_alloc_token(parser, tokens, num_tokens);
			if (token == NULL)
				return JSMN_ERROR_NOMEM;
			if (parser->toksuper != -1) {
				tokens[parser->toksuper].size++;
#ifdef JSMN_PARENT_LINKS
				token->parent = parser->toksuper;
#endif
			}
			token->type = (c == '{' ? JSMN_OBJECT : JSMN_ARRAY);
			token->start = parser->pos;
			parser->toksuper = parser->toknext - 1;
			break;
		case '}': case ']':
			if (tokens == NULL)
				break;
			type = (c == '}' ? JSMN_OBJECT : JSMN_ARRAY);
#ifdef JSMN_PARENT_LINKS
			if (parser->toknext < 1)
				return JSMN_ERROR_INVAL;
			token = &tokens[parser->toknext - 1];
			for (;;) {
				if (token->start != -1 && token->end == -1) {
					if (token->type != type)
						return (JSMN_ERROR_INVAL);
					token->end = parser->pos + 1;
					parser->toksuper = token->parent;
					break;
				}
				if (token->parent == -1)
					break;
				token = &tokens[token->parent];
			}
#else
			for (i = parser->toknext - 1; i >= 0; i--) {
				token = &tokens[i];
				if (token->start != -1 && token->end == -1) {
					if (token->type != type)
						return (JSMN_ERROR_INVAL);
					parser->toksuper = -1;
					token->end = parser->pos + 1;
					break;
				}
			}
			/* Error if unmatched closing bracket */
			if (i == -1) return JSMN_ERROR_INVAL;
			for (; i >= 0; i--) {
				token = &tokens[i];
				if (token->start != -1 && token->end == -1) {
					parser->toksuper = i;
					break;
				}
			}
#endif
			break;
		case '\"':
			r = jsmn_parse_string(parser, js, len, tokens, num_tokens);
			if (r < 0)
				return (r);
			count++;
			if (parser->toksuper != -1 && tokens != NULL)
				tokens[parser->toksuper].size++;
			break;
		case '\t': case '\r': case '\n': case ' ':
			break;
		case ':':
			parser->toksuper = parser->toknext - 1;
			break;
		case ',':
			if (tokens != NULL &&
			  tokens[parser->toksuper].type != JSMN_ARRAY &&
			  tokens[parser->toksuper].type != JSMN_OBJECT) {
#ifdef JSMN_PARENT_LINKS
				parser->toksuper = tokens[parser->toksuper].parent;
#else
				for (i = parser->toknext - 1; i >= 0; i--) {
					if (tokens[i].type == JSMN_ARRAY ||
					  tokens[i].type == JSMN_OBJECT) {
						if (tokens[i].start != -1 && tokens[i].end == -1) {
							parser->toksuper = i;
							break;
						}
					}
				}
#endif
			}
			break;
#ifdef JSMN_STRICT
			/* In strict mode primitives are: numbers and booleans */
		case '-': case '0': case '1': case '2': case '3': case '4':
		case '5': case '6': case '7': case '8': case '9':
		case 't': case 'f': case 'n':
			/* And they must not be keys of the object */
			if (tokens != NULL) {
				pjsmntok_t *t = &tokens[parser->toksuper];
				if (t->type == JSMN_OBJECT ||
						(t->type == JSMN_STRING && t->size != 0)) {
					return JSMN_ERROR_INVAL;
				}
			}
#else
		/* In non-strict mode every unquoted value is a primitive */
		default:
#endif
			r = jsmn_parse_primitive(parser, js, len, tokens, num_tokens);
			if (r < 0)
				return (r);
			count++;
			if (parser->toksuper != -1 && tokens != NULL)
				tokens[parser->toksuper].size++;
			break;

#ifdef JSMN_STRICT
		/* Unexpected char in strict mode */
		default:
			return (JSMN_ERROR_INVAL);
#endif
		}
	}

	for (i = parser->toknext - 1; i >= 0; i--) {
		/* Unmatched opened object or array */
		if (tokens[i].start != -1 && tokens[i].end == -1)
			return (JSMN_ERROR_PART);
	}

	return (count);
}

/**
 * Creates a new parser based over a given buffer with an array of tokens
 * available.
 */
void
pjsmn_init(pjsmn_parser *parser) {

	parser->pos = 0;
	parser->toknext = 0;
	parser->toksuper = -1;
}