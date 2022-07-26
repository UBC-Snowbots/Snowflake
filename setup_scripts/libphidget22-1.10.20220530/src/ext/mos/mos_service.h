#ifndef _MOS_SERVICE_H_
#define _MOS_SERVICE_H_

#include "mos_iop.h"

#ifdef __cplusplus
extern "C" {
#endif

MOSAPI int MOSCConv mos_service_install(mosiop_t, const char *, const char *,
  const char *);

#ifdef __cplusplus
}
#endif

#endif /* _MOS_SERVICE_H_ */
