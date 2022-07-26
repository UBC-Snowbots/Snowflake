#ifndef _MOS_READLINE_H_
#define _MOS_READLINE_H_

#include "mos_os.h"
#include "mos_iop.h"

MOSAPI typedef int(mos_readfunc_t)(mosiop_t, void *, uint8_t *, uint32_t *);
MOSAPI int MOSCConv mos_readline(mosiop_t, mos_readfunc_t *, void *, char **, uint32_t *,
  uint32_t *);

#endif /* _MOS_READLINE_H_ */
