#ifndef _SCAN_H_
#define _SCAN_H_

#include "mos/mos_os.h"
#include "mos/mos_iop.h"

typedef struct scanstate {
	char		*sbuf;		/* full scan buf */
	uint32_t	sbufsz;		/* buf size */
	char		*cpos;		/* current character position */
	char		*epos;		/* end of the current 'line' */
	int			lineno;		/* current line number */
	int			charno;		/* current character position */
	int			state;		/* parser state number */
} scanstate_t;

typedef struct scanresult {
	int				type;
	char			*value;
	int64_t			ivalue;
	uint32_t		valuesz;
} scanresult_t;

#define SCAN_OK		0
#define SCAN_DONE	1
#define SCAN_ERR	2

#define SCAN_TOKEN		1
#define SCAN_STRING		2
#define SCAN_INT		3
#define SCAN_TRUE		4
#define SCAN_FALSE		5
#define SCAN_EQUAL		6
#define SCAN_NAMESPACE	128

void initscanresult(scanresult_t *);
void finiscanresult(scanresult_t *);
void setscanstate(scanstate_t *, const char *, uint32_t);
void freescanstate(scanstate_t *);
int scan(mosiop_t, scanstate_t *, scanresult_t *);
void printscanresult(scanresult_t *);
const char *scantoken(int);

#endif /*_SCAN_H_*/
