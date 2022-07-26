#ifndef _MOS_URLENCODE_H_
#define _MOS_URLENCODE_H_

MOSAPI int MOSCConv mos_isurlencoded(const char *, uint32_t);
MOSAPI char* MOSCConv mos_urlencode(const char *, uint32_t, uint32_t *);

#endif /* _MOS_URLENCODE_H_ */
