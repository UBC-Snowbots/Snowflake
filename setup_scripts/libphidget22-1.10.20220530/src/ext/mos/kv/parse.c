#include "parse.h"

#define MAX_STACK_SIZE	4

static int debug;

static int state0[] = { SCAN_TOKEN, 1, 0, SCAN_NAMESPACE, 0, 1, -1, -1, -1};
static int state1[] = { SCAN_EQUAL, 2, 0, -1, -1, -1 };
static int state2[] = {
	SCAN_TOKEN, 0, 1,
	SCAN_STRING, 0, 1,
	SCAN_INT, 0, 1,
	SCAN_TRUE, 0, 1,
	SCAN_FALSE, 0, 1,
	-1, -1, -1
};

typedef struct parsestate {
	uint32_t state;
	int	*accepts;
} parsestate_t;

#define PARSE_STATES 3
static parsestate_t states[] = {
	{ 0, state0 },
	{ 1, state1 },
	{ 2, state2 },
};

static int
handletoken(mosiop_t iop, scanstate_t *ss, scanresult_t *sres, int *reduces) {
	parsestate_t *ps;
	int *i;

	if (ss->state >= PARSE_STATES)
		return (MOS_ERROR(iop, MOSN_INVAL, "invalid state %u line %d+%d",
		  ss->state, ss->lineno, ss->charno));

	/*
	 * Accepts is an array of tokens and state; offset 0 is the token,
	 * +1 the next state, and +2 if the state reduces.  -1, -1, -1 terminates.
	 */
	ps = &states[ss->state];
	for (i = ps->accepts; *i != -1; i += 3) {
		if (debug > 1)
			mos_printf("%d: %d vs %d  => %d\n", ss->state, sres->type, *i,
			  *(i + 1));
		if (*i == sres->type) {
			*reduces = *(i + 2);
			if (debug)
				mos_printf("accept %d => %d%s\n", ss->state, *(i + 1),
				  *reduces ? " (reduces)" : "");
			ss->state = *(i + 1);
			return (0);
		}
	}

	/*
	 * token does not match the current state
	 */
	if (sres->type == SCAN_TOKEN)
		return (MOS_ERROR(iop, MOSN_INVAL,
		  "parse error: line %d+%d unexpected token [%s] in state %u",
		    ss->lineno, ss->charno + 1, sres->value, ss->state));
	else
		return (MOS_ERROR(iop, MOSN_INVAL,
		  "parse error: line %d+%d unexpected token [%s] in state %u",
		    ss->lineno, ss->charno + 1, scantoken(sres->type), ss->state));
}

void
setparsedebug() {
	debug++;
}

int
parse(mosiop_t iop, const char *sbuf, uint32_t sbuflen, parsereduce_t reduce,
  void *private) {
	scanresult_t stack[MAX_STACK_SIZE];
	scanstate_t ss;
	int reduces;
	int depth;
	int err;
	int n;

	setscanstate(&ss, sbuf, sbuflen);
	reduces = 0;

	for (n = 0; n < MAX_STACK_SIZE; n++)
		initscanresult(&stack[n]);

	depth = 0;
	while ((err = scan(iop, &ss, &stack[depth])) == SCAN_OK) {
		if (debug) {
			printscanresult(&stack[depth]);
			mos_printf("\n");
		}

		err = handletoken(iop, &ss, &stack[depth], &reduces);
		if (err != 0)
			goto bad;

		depth++;
		if (reduces) {
			if (ss.state != 0) {
				err = MOS_ERROR(iop, MOSN_INVAL, "parse state error: line %d+5d"
				  "depth:%d state:%u", ss.lineno, ss.charno, depth, ss.state);
				goto bad;
			}

			if (reduce) {
				err = reduce(iop, &ss, stack, private);
				if (err != 0) {
					MOS_ERROR(iop, err, "reduce() failed");
					goto bad;
				}
			}
			depth = 0;
		}
	}

	if (err == SCAN_DONE)
		err = 0;
	if (err == SCAN_ERR)
		err = MOSN_ERR;

bad:

	for (n = 0; n < MAX_STACK_SIZE; n++)
		finiscanresult(&stack[n]);
	freescanstate(&ss);

	return (err);
}
