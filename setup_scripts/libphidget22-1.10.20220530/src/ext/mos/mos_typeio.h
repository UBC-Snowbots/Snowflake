#ifndef _MOS_TYPEIO_H_
#define _MOS_TYPEIO_H_

#include <sys/types.h>

//#include "mos_byteorder.h"
#include "mos_iop.h"
#include "mos_time.h"

/*
 * TYPES
 */

/*
 * General RW contract
 *
 * rw(class, iop, dowrite, buf, buflen, indexp)
 */ 
typedef int (*mos_typeio_rw_t)(void *, mosiop_t, int, uint8_t *, uint32_t,
  uint32_t *);

/*
 * On-Disk Varchar.
 */
#define MOS_VARCHAR_MAX_SIZE	65536
struct mos_odvarchar {
	uint32_t	vc_maxlen;
	uint16_t	vc_len;
	uint8_t		*vc_data;
};

typedef struct mos_odvarchar mos_odvarchar_t;

/*
 * MACROS
 */
#define TYPEIO_CK(stmt)		do {											\
	if ((res = (stmt)) != 0)												\
   		return (res);														\
  } while (0)
#define TYPEIO_CKGOTO(stmt)	do {											\
	if ((res = (stmt)) != 0)												\
		goto done;															\
  } while (0)

#define MOS_VAR_SP_RW(dowrite, tbuf, tbuflen, indexp, addr, len, cpfunc) \
	  ((dowrite) ? \
	    (((tbuflen) < *(indexp) + (len)) ? \
	      MOS_ERROR(iop, MOSN_NOSPC, "target tbuffer too small (%d)", (len)) : \
	      ((cpfunc)((tbuf) + *(indexp), (addr), (len)), *(indexp) += (len), 0))\
		: (((tbuflen) < *(indexp) + (len)) ? \
	      MOS_ERROR(iop, MOSN_NOSPC, "source tbuffer too small (%d)", (len)) : \
	      ((cpfunc)((addr), (tbuf) + *(indexp), (len)), *(indexp) += (len), 0)))

#define MOS_VAR_NATIVE_RW(dowrite, tbuf, tbuflen, indexp, addr, len) \
	  MOS_VAR_SP_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), (len), \
	    memcpy)
#define MOS_VAR_REV_RW(dowrite, tbuf, tbuflen, indexp, addr, len) \
	  MOS_VAR_SP_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), (len), \
	    mos_revcpy)

#if (MOS_BYTE_ORDER == MOS_LITTLE_ENDIAN)

#define MOS_VAR_RW(dowrite, tbuf, tbuflen, indexp, addr, len) \
	  MOS_VAR_NATIVE_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), (len))
#define MOS_UINT8_ARRAY_RW(dowrite, tbuf, tbuflen, indexp, addr, len) \
	  MOS_VAR_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), (len))
#define MOS_UINT32_ARRAY_RW(dowrite, tbuf, tbuflen, indexp, addr, n) \
	  MOS_VAR_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), (n) * 4)
#define MOS_UINT64_ARRAY_RW(dowrite, tbuf, tbuflen, indexp, addr, n) \
	  MOS_VAR_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), (n) * 8)

#elif (MOS_BYTE_ORDER == MOS_BIG_ENDIAN)

#define MOS_VAR_RW(dowrite, tbuf, tbuflen, indexp, addr, len) \
	  MOS_VAR_REV_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), (len))
#define MOS_UINT8_ARRAY_RW(dowrite, tbuf, tbuflen, indexp, addr, n) \
	  MOS_VAR_NATIVE_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), (n))
#define MOS_UINT32_ARRAY_RW(dowrite, tbuf, tbuflen, indexp, addr, n) \
	  mos_uint32_array_rw(iop, (dowrite), (tbuf), (tbuflen), (indexp), \
	    (addr), (n))
#define MOS_UINT64_ARRAY_RW(dowrite, tbuf, tbuflen, indexp, addr, n) \
	  mos_uint64_array_rw(iop, (dowrite), (tbuf), (tbuflen), (indexp), \
	    (addr), (n))

#else

#error MOS_BYTE_ORDER should be MOS_BIG_ENDIAN or MOS_LITTLE_ENDIAN

#endif

#define MOS_UINT8_RW(dowrite, tbuf, tbuflen, indexp, addr) \
	  MOS_VAR_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), 1)
#define MOS_UINT16_RW(dowrite, tbuf, tbuflen, indexp, addr) \
	  MOS_VAR_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), 2)
#define MOS_UINT32_RW(dowrite, tbuf, tbuflen, indexp, addr) \
	  MOS_VAR_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), 4)
#define MOS_UINT64_RW(dowrite, tbuf, tbuflen, indexp, addr) \
	  MOS_VAR_RW((dowrite), (tbuf), (tbuflen), (indexp), (addr), 8)

#define MOSADDR_RW(dowrite, tbuf, tbuflen, indexp, addr) \
  mosaddr_rw(iop, (dowrite), (tbuf), (tbuflen), (indexp), (addr)) 
  
MOSAPI int MOSCConv mosodvarchar_rw(mosiop_t, mos_odvarchar_t *, int, uint8_t *,
  uint32_t, uint32_t *);

MOSAPI int MOSCConv mostimestamp_rw(mosiop_t, int, uint8_t *, uint32_t, uint32_t *,
  mostimestamp_t *);
MOSAPI int MOSCConv mos_uint32_array_rw(mosiop_t, int, uint8_t *, uint32_t, uint32_t *,
  uint32_t *, uint32_t);
MOSAPI int MOSCConv mos_uint64_array_rw(mosiop_t, int, uint8_t *, uint32_t, uint32_t *,
  uint64_t *, uint32_t);

MOSAPI int MOSCConv mos_typeio_write(void *, mosiop_t, uint8_t *, uint32_t,
  uint32_t *, mos_typeio_rw_t);

#endif /* _MOS_TYPEIO_H_ */
