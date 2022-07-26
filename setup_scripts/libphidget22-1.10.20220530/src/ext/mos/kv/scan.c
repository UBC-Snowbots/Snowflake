#include "scan.h"

static int istokenstart(int);
static int istokenchar(int);
static int isintstart(int);
static int isintchar(int);
static int readstring(mosiop_t, scanstate_t *, scanresult_t *, int);
static int readnamespace(mosiop_t, scanstate_t *, scanresult_t *);
static int _scan(mosiop_t, scanstate_t *, scanresult_t *);

#define CPOS		(*ss->cpos)
#define CPOSN(n)	(ss->cpos + (n))
#define EPOS		(*ss->epos)

const char *
scantoken(int id) {

	switch(id) {
	case SCAN_TOKEN:
		return ("TOKEN");
	case SCAN_STRING:
		return ("STRING");
	case SCAN_INT:
		return ("INT");
	case SCAN_TRUE:
		return ("TRUE");
	case SCAN_FALSE:
		return ("FALSE");
	case SCAN_EQUAL:
		return ("EQUAL");
	default:
		return ("unknown");
	}
}

void
initscanresult(scanresult_t *sres) {

	sres->type = 0;
	sres->value = NULL;
	sres->valuesz = 0;
}

void
finiscanresult(scanresult_t *sres) {

	if (sres->value != NULL)
		mos_free(sres->value, sres->valuesz);

	memset(sres, 0, sizeof (*sres));
}

void
setscanstate(scanstate_t *ss, const char *sbuf, uint32_t sbuflen) {

	if (sbuflen == 0) {
		ss->sbuf = mos_strdup(sbuf, &ss->sbufsz);
	} else {
		ss->sbuf = mos_malloc(sbuflen + 1);
		ss->sbufsz = sbuflen + 1;
		memcpy(ss->sbuf, sbuf, sbuflen);
		ss->sbuf[sbuflen] = '\0';
	}
	ss->cpos = ss->sbuf;
	ss->epos = NULL;
	ss->lineno = 1;
	ss->charno = 0;
	ss->state = 0;
}

void
freescanstate(scanstate_t *ss) {

	if (ss->sbuf != NULL)
		mos_free(ss->sbuf, ss->sbufsz);
}

static void
skipline(scanstate_t *ss) {

	while (CPOS != '\0' && CPOS != '\n')
		ss->cpos++;

	if (CPOS == '\n')
		ss->cpos++;

	ss->lineno++;
	ss->charno = 0;
}

static void
setepos(scanstate_t *ss) {
	char *c;

	for (c = ss->cpos; *c != '\0'; c++) {
		ss->epos = c;

		if (*c == '\n') {
			if (mos_isspace(*(c + 1)))
				continue;
			else
				return;
		}
	}
}

#define SS_OPEN		0
#define SS_TOKEN	1
#define SS_INT		2

#ifdef SCAN_MULTILINE
/**
 * We were forced to add support for a special case that violates our kv syntax
 * rules. In mrep, we were generating collaboration KV files with mos_snprintf,
 * and in so doing, we ended up with kv files that contained the following kv
 * syntax violation:
 *
 * <code>mtpt="E:\"</code>
 *
 * In this case, the backslash should not have been included, or should've been
 * a double escape. Since there were many collaboration kv files in the wild
 * with this problem, we had to update scanml to tolerate the above violation.
 */
static int
scanml(mosiop_t iop, scanstate_t *ss, const char *spos, scanresult_t *sres) {
	const char *c;
	int escape;
	char *d;

	finiscanresult(sres);
	sres->type = SCAN_STRING;
	if (ss->epos - spos > 0) {
		sres->valuesz = (uint32_t)(ss->epos - spos) + 1;
		sres->value = mos_zalloc(sres->valuesz);
	}

	d = sres->value;
	escape = 0;

	for (c = spos; c < ss->epos; c++) {

		if (*c == '\r')
			if (*(c + 1) == '\n')
				c++;

		if (*c == '\n') {
			c++;
			if (mos_isspace(*c)) {
				c++;
				*d = ' ';	/* replace \nWS with a space */
				d++;
			}
		}

		if ((escape == 0) && (*c == '\\')) {
			/*
			 * see function documentation
			 */
			if (((ss->epos - c) != 2) || *(c + 1) != '\"')
				escape = 1;
			continue;
		}

		if (escape == 0 && *c == '"') {
			escape = 0;
			continue;
		}

		escape = 0;

		/* skip leading whitespace */
		if (mos_isspace(*c) && d == sres->value)
			continue;

		*d = *c;
		d++;
	}

	*d = '\0';

	return (0);
}

#endif

int
scan(mosiop_t iop, scanstate_t *ss, scanresult_t *sres) {
	scanstate_t tss;
	int err;
#ifdef SCAN_MULTILINE
	char *oldcpos;
	char *c;
#endif

	/*
	 * States 0 and 1 are 'key' '=', the remaining states are for values.
	 * Find the end of the value, in case it is a more complex (rfc822)
	 * multi-line value.
	 */
	if (ss->state > 1)
		setepos(ss);

#ifdef SCAN_MULTILINE
	oldcpos = ss->cpos;
#endif
	tss = *ss;
	err = _scan(iop, &tss, sres);

	/*
	 * Determine if we matched the whole value, or only part of it.
	 * If we only matched part, scan as a multi-line value.
	 */
#ifdef SCAN_MULTILINE
	if (ss->state > 1) {
		if (tss.cpos != ss->epos - 1) {
			for (c = tss.cpos; c != ss->epos; c++) {
				if (mos_isspace(*c)) {
					continue;
				} else {
					err = scanml(iop, &tss, oldcpos, sres);
					break;
				}
			}
			tss.cpos = ss->epos;
		}
	}
#endif

	*ss = tss;
	return (err);
}

static int
_scan(mosiop_t iop, scanstate_t *ss, scanresult_t *sres) {
	char sbuf[1024];
	int prevcpos;
	int state;
	size_t i;
	int err;

	finiscanresult(sres);

	state = SS_OPEN;
	i = 0;

	prevcpos = CPOS;
	for (;;) {
		if (prevcpos != CPOS) /* only increment if we have consumed the char */
			ss->charno++;

		switch(state) {

		/*
		 * Open state eats white space, and attempts to detect the type
		 * of the next token.  Once the type is determined, we changed
		 * state, and read that token type.
		 *
		 * Strings begin with ' or " are read directly in the open state.
		 */
		case SS_OPEN:
			switch(CPOS) {
			case '\0':
				return (SCAN_DONE);
			case '#':
				skipline(ss);
				break;
			case ' ':
			case '\r':
			case '\t':
				ss->cpos++;
				break;
			case '\n':
				ss->cpos++;
				ss->lineno++;
				ss->charno = 0;
				break;
			case '=':
			case ':':
				ss->cpos++;
				sres->type = SCAN_EQUAL;
				return (SCAN_OK);
			case '"':
			case '\'':
				return (readstring(iop, ss, sres, CPOS));
			case '[':
				return (readnamespace(iop, ss, sres));
			default:
				if (istokenstart(CPOS)) {
					state = SS_TOKEN;
					i = 0;
					break;
				}
				if (isintstart(CPOS)) {
					state = SS_INT;
					sbuf[0] = CPOS;
					ss->cpos++;	/* skip possible '-' */
					i = 1;
					break;
				}
				goto invalid;
			}
			break;

		/*
		 * Reads a token.  Tokens cannot contain whitespace, and have a
		 * limited character set (istokenchar()).
		 */
		case SS_TOKEN:
			if (istokenchar(CPOS)) {
				if (i >= sizeof (sbuf) - 1)
					return (MOS_ERROR(iop, MOSN_INVAL, "token is too large"));
				sbuf[i] = CPOS;
				ss->cpos++;
				i++;
			} else {
				sbuf[i] = '\0';
				if (mos_strcasecmp(sbuf, "true") == 0) {
						sres->type = SCAN_TRUE;
				} else if (mos_strcasecmp(sbuf, "false") == 0) {
						sres->type = SCAN_FALSE;
				} else {
					sres->type = SCAN_TOKEN;
					sres->value = mos_strdup(sbuf, &sres->valuesz);
				}
				state = SS_OPEN;
				return (SCAN_OK);
			}
			break;

		/*
		 * Reads a signed 64 bit integer.
		 */
		case SS_INT:
			if (isintchar(CPOS)) {
				if (i >= sizeof (sbuf) - 1)
					return (MOS_ERROR(iop, MOSN_INVAL, "integer is too large"));
				sbuf[i] = CPOS;
				ss->cpos++;
				i++;
			} else {
				sbuf[i] = '\0';
				sres->type = SCAN_INT;
				err = mos_strto64(sbuf, 10, &sres->ivalue);
				if (err != 0) {
					MOS_ERROR(iop, err, "invalid integer '%s'", sbuf);
					return (SCAN_ERR);
				}
				state = SS_OPEN;
				return (SCAN_OK);
			}
			break;
		default:
			goto invalid;
		}
	}

invalid:

	MOS_ERROR(iop, MOSN_INVAL, "invalid scan state: line %d+%d char [%c]",
	  ss->lineno, ss->charno, CPOS);
	return (SCAN_ERR);
}

static int
isintstart(int c) {

	if (c == '-')
		return (1);
	if (c >= '0' && c <= '9')
		return (1);
	return (0);
}

static int
isintchar(int c) {

	if (c >= '0' && c <= '9')
		return (1);
	return (0);
}

static int
istokenstart(int c) {

	switch(c) {
	case '%':
	case '@':
	case '_':
		return (1);
	default:
		if (c >= 'A' && c <= 'Z')
			return (1);
		if (c >= 'a' && c <= 'z')
			return (1);
		return (0);
	}
}

static int
istokenchar(int c) {

	switch(c) {
	case '%':
	case '-':
	case '.':
	case '@':
	case '_':
	case '/':
		return (1);
	default:
		if (c >= 'A' && c <= 'Z')
			return (1);
		if (c >= 'a' && c <= 'z')
			return (1);
		if (c >= '0' && c <= '9')
			return (1);
		return (0);
	}
}

static int
readnamespace(mosiop_t iop, scanstate_t *ss, scanresult_t *sres) {
	char nsbuf[64];
	size_t i;

	ss->cpos++;

	for (i = 0; CPOS != '\0' && i < sizeof (nsbuf); ss->cpos++, i++) {
		if (CPOS == '\n') {
			MOS_ERROR(iop, MOSN_INVAL, "bad namespace: line %d+%d", ss->lineno, ss->charno);
			return (SCAN_ERR);
		} else if (CPOS == ']') {
			ss->cpos += 2;
			nsbuf[i] = '\0';
			sres->type = SCAN_NAMESPACE;
			sres->value = mos_strdup(nsbuf, &sres->valuesz);
			return (SCAN_OK);
		}
		nsbuf[i] = CPOS;
		ss->charno++;
	}

	MOS_ERROR(iop, MOSN_INVAL, "bad namespace: line %d+%d", ss->lineno, ss->charno, CPOS);
	return (SCAN_ERR);
}

static int
readstring(mosiop_t iop, scanstate_t *ss, scanresult_t *sres, int term) {
	char sbuf[1024];
	int escape;
	size_t i;

	escape = 0;
	i = 0;

	ss->cpos++;

	for (i = 0, escape = 0; CPOS != '\0' && i < sizeof (sbuf) - 2; ss->cpos++, i++) {
		if (CPOS == '\n') {
			ss->charno = 0;
			ss->lineno++;
		} else {
			ss->charno++;
		}

		if (escape) {
			/*
			 * Simple implementation of ANSI X3.159-1989 (ANSI C)
			 * escape sequences... at least the ones most likely
			 * to be used.
			 */
			switch(CPOS) {
			case 'a':
				sbuf[i] = '\a';
				break;
			case 'b':
				sbuf[i] = '\b';
				break;
			case 'f':
				sbuf[i] = '\f';
				break;
			case 'n':
				sbuf[i] = '\n';
				break;
			case 'r':
				sbuf[i] = '\r';
				break;
			case 't':
				sbuf[i] = '\t';
				break;
			case 'v':
				sbuf[i] = '\v';
				break;
			case '\\':
				sbuf[i] = '\\';
				break;
			default:
				sbuf[i] = CPOS;
				break;
			}
			escape = 0;
		} else {
			if (CPOS == term) {
				ss->cpos++;
				goto done;
			}
			switch(CPOS) {
			case '\\':
				escape = 1;
				i--;
				break;
			default:
				sbuf[i] = CPOS;
				break;
			}
		}
	}

	MOS_ERROR(iop, MOSN_INVAL, "string does not terminate");
	return (SCAN_ERR);

done:

	sbuf[i] = '\0';
	sres->type = SCAN_STRING;
	sres->value = mos_strdup(sbuf, &sres->valuesz);

	return (SCAN_OK);
}

void
printscanresult(scanresult_t *sres) {

	switch(sres->type) {
	case SCAN_TOKEN:
		mos_printf("token [%s]", sres->value);
		break;
	case SCAN_EQUAL:
		mos_printf("[=]");
		break;
	case SCAN_TRUE:
		mos_printf("[TRUE]");
		break;
	case SCAN_FALSE:
		mos_printf("[FALSE]");
		break;
	case SCAN_STRING:
		mos_printf("string [%s]", sres->value);
		break;
	case SCAN_INT:
		mos_printf("integer [%"PRId64"]", sres->ivalue);
		break;
	case SCAN_NAMESPACE:
		mos_printf("namespace [%s]", sres->value);
		break;
	default:
		mos_printf("unknown [%d]", sres->type);
		break;
	}
}
