#ifndef _PARSE_H_
#define _PARSE_H_

#include "mos/mos_os.h"
#include "mos/mos_iop.h"
#include "scan.h"

typedef int (*parsereduce_t)(mosiop_t, scanstate_t *, scanresult_t *, void *);

/*
 * iop, buf, buflen, reducefunc, private
 *
 * If the buf is null terminated, buflen can be set to 0; otherwise, only
 * buflen bytes from the buf will be used.
 */
int parse(mosiop_t, const char *, uint32_t, parsereduce_t, void *private);
void setparsedebug(void);

#endif /* _PARSE_H_ */
