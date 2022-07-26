#ifndef _MOS_IOP_H_
#define _MOS_IOP_H_

#ifndef EXTERNALPROTO
#include <stdarg.h>
#include "mos_basic_types.h"
#include "mos_macrocompat.h"

#define MOS_IOP_MAGIC	0xf0f9

typedef struct mos_iop_note		mos_iop_note_t;
typedef struct mos_notice		mos_notice_t;
typedef struct mos_iop			*mosiop_t;
#define MOS_IOP_IGNORE			NULL

#define MOS_ERROR(iop, err, ...) 											\
  (mos_iop_addnotice((iop), NULL, (err), __FILE__, __LINE__, __func__,		\
    __VA_ARGS__))
#define MOS_ERROR_CHAIN(iop, subiop, err, ...) 								\
  (mos_iop_addnotice((iop), (subiop), (err), __FILE__, __LINE__, __func__,	\
    __VA_ARGS__))

#define MOSIOP_ISSCONF_BUFSZ	131072

MOSAPI int MOSCConv mos_iop_addnotice(mosiop_t, mosiop_t, uint32_t, const char *const,
  int, const char *, const char *, ...);
MOSAPI int MOSCConv mos_iop_vaddnotice(mosiop_t, mosiop_t, uint32_t, const char *const,
  int, const char *, const char *, va_list);
MOSAPI void MOSCConv mos_iop_walknotices(mosiop_t,
  void(*)(const mos_notice_t *, void *, size_t), void *, size_t);

MOSAPI int MOSCConv mos_iop_addnotev(mosiop_t, const char *, ...);
MOSAPI int MOSCConv mos_iop_walknotes(mosiop_t,
  void (*)(const mos_iop_note_t *, void *, size_t), void *, size_t);
MOSAPI const char * MOSCConv mos_iop_note_getnote(const mos_iop_note_t *);

MOSAPI const char * MOSCConv mos_notice_get_message(const mos_notice_t *);
MOSAPI const char * MOSCConv mos_notice_get_file(const mos_notice_t *);
MOSAPI const char * MOSCConv mos_notice_get_func(const mos_notice_t *);
MOSAPI mosiop_t MOSCConv mos_notice_get_subiop(const mos_notice_t *);
MOSAPI mosiop_t MOSCConv mos_notice_get_iop(const mos_notice_t *);
MOSAPI uint32_t MOSCConv mos_notice_get_notice(const mos_notice_t *);
MOSAPI int MOSCConv mos_notice_get_sequence(const mos_notice_t *);
MOSAPI int MOSCConv mos_notice_get_line(const mos_notice_t *);
MOSAPI int MOSCConv mos_notice_isnetworkrelated(int); /* e.g. MOSN_CONNREF/TIMEOUT/... */
MOSAPI const char * MOSCConv mos_notice_name(int); /* e.g. "MOSN_INVALARG" */
MOSAPI const char * MOSCConv mos_notice_string(int); /* e.g. "Invalid Argument" */
MOSAPI void MOSCConv mos_iop_setopcodens(mosiop_t, int);
MOSAPI void MOSCConv mos_iop_setopcode(mosiop_t, int);
MOSAPI int MOSCConv mos_iop_getopcode(mosiop_t);

MOSAPI uint16_t MOSCConv mos_iop_getnoticecount(mosiop_t);
MOSAPI uint32_t MOSCConv mos_iop_getlastnotice(mosiop_t);

MOSAPI int MOSCConv mos_iop_rw(mosiop_t, int, uint8_t *, uint32_t, uint32_t *,
  mosiop_t *);

MOSAPI int MOSCConv mosiop_to_issconf(mosiop_t, int, const char *, char *, uint32_t);
#endif

#if MOS_IOPSUPPORT == 2
#define MOSN_FAMILY(n)		((n) & 0x0F000000)
#define MOSN_F_SYSTEM		0x01000000	/* System Family of Notices */
#define MOSN_F_PRIVATE		0x02000000	/* Private Family of Notices */
#define MOSN_F_USER			0x03000000	/* User Family of Notices */

#define MOSN_ISSYSTEM(n)	(MOSN_FAMILY((n)) == MOSN_F_SYSTEM)
#define MOSN_ISPRIVATE(n)	(MOSN_FAMILY((n)) == MOSN_F_PRIVATE)
#define MOSN_ISUSER(n)		(MOSN_FAMILY((n)) == MOSN_F_USER)

#define MOSN_TYPE(n)		((n) & 0x00FF0000)

#define MOSN_T_ERROR		0x00010000	/* Error Notice */
#define MOSN_T_WARNING		0x00020000	/* Warning Notice */
#define MOSN_T_INFO			0x00030000	/* Informational Notice */
#define MOSN_T_DEBUG		0x00040000	/* Debug Notice */

#define MOSN_ISERROR(n)		(MOSN_TYPE((n)) == (MOSN_T_ERROR))
#define MOSN_ISWARNING(n)	(MOSN_TYPE((n)) == MOSN_T_WARNING)
#define MOSN_ISINFO(n)		(MOSN_TYPE((n)) == MOSN_T_INFO)
#define MOSN_ISDEBUG(n)		(MOSN_TYPE((n)) == MOSN_T_DEBUG)

#define MOSN_NOTICE(n)		((n) & 0x0000FF00)
#define MOSN_SNOTICE(n)		((n) & 0x000000FF)

#define MOSN_SHIFT	0

#else /* MOS_IOPSUPPORT */

#define MOSN_T_ERROR	0

#define MOSN_F_SYSTEM	0
#define MOSN_F_USER		0
#define MOSN_F_PRIVATE	0

#define MOSN_ISERROR(n)	(1)
#define MOSN_FAMILY(n)	(0)
#define MOSN_NOTICE(n)	(n)

#define MOSN_SHIFT	8

#endif /* MOS_IOPSUPPORT */


#define MOSN_N_PERM			(0x00000100 >> MOSN_SHIFT)	/* Not Permitted*/
#define MOSN_N_NOENT		(0x00000200 >> MOSN_SHIFT)	/* No Such Entity */
#define MOSN_N_TIMEDOUT		(0x00000300 >> MOSN_SHIFT)	/* Timed Out */
#define MOSN_N_INTR			(0x00000400 >> MOSN_SHIFT)	/* Op Interrupted */
#define MOSN_N_IO			(0x00000500 >> MOSN_SHIFT)	/* IO Issue */
#define MOSN_N_MEM			(0x00000600 >> MOSN_SHIFT)	/* Memory Issue */
#define MOSN_N_ACCESS		(0x00000700 >> MOSN_SHIFT)	/* Access (Permission) Issue */
#define MOSN_N_FAULT		(0x00000800 >> MOSN_SHIFT)	/* Address Issue */
#define MOSN_N_BUSY			(0x00000900 >> MOSN_SHIFT)	/* Resource Busy */
#define MOSN_N_EXIST		(0x00000A00 >> MOSN_SHIFT)	/* Object Exists */
#define MOSN_N_NOTDIR		(0x00000B00 >> MOSN_SHIFT)	/* Object is not a directory */
#define MOSN_N_ISDIR		(0x00000C00 >> MOSN_SHIFT)	/* Object is a directory */
#define MOSN_N_INVAL		(0x00000D00 >> MOSN_SHIFT)	/* Invalid */
#define MOSN_N_NFILE		(0x00000E00 >> MOSN_SHIFT)	/* Too many open files in system */
#define MOSN_N_MFILE		(0x00000F00 >> MOSN_SHIFT)	/* Too many open files */
#define MOSN_N_NOSPC		(0x00001000 >> MOSN_SHIFT)	/* Space Issue */
#define MOSN_N_FBIG			(0x00001100 >> MOSN_SHIFT)	/* File too big */
#define MOSN_N_ROFS			(0x00001200 >> MOSN_SHIFT)	/* Read Only FS */
#define MOSN_N_RO			(0x00001300 >> MOSN_SHIFT)	/* Read Only Object */
#define MOSN_N_NOSUP		(0x00001400 >> MOSN_SHIFT)	/* Not Supported */
#define MOSN_N_INVALARG		(0x00001500 >> MOSN_SHIFT)	/* Invalid Argument */
#define MOSN_N_AGAIN		(0x00001600 >> MOSN_SHIFT)	/* Try Again */
#define MOSN_N_NEVENT		(0x00001700 >> MOSN_SHIFT)	/* Not an event */
#define MOSN_N_INCONST		(0x00001800 >> MOSN_SHIFT)	/* Inconsistent State */
#define MOSN_N_ADDR			(0x00001900 >> MOSN_SHIFT)	/* Invalid Address */
#define MOSN_N_NOTEMPTY		(0x00001A00 >> MOSN_SHIFT)	/* Not Empty */
#define MOSN_N_DUP			(0x00001B00 >> MOSN_SHIFT)	/* Duplicate */
#define MOSN_N_ERR			(0x00001C00 >> MOSN_SHIFT)	/* Generic Unexpected Error */
#define MOSN_N_HASH			(0x00001D00 >> MOSN_SHIFT)	/* Hash verify failed */
#define MOSN_N_CONTENT		(0x00001E00 >> MOSN_SHIFT)	/* Invalid content read */
#define MOSN_N_EOF			(0x00001F00 >> MOSN_SHIFT)	/* End of File*/
#define MOSN_N_POLICY		(0x00002000 >> MOSN_SHIFT)	/* Policy Issue */
#define MOSN_N_LICENSE		(0x00002100 >> MOSN_SHIFT)	/* License Issue */
#define MOSN_N_TASTE		(0x00002200 >> MOSN_SHIFT)	/* Input not acceptable */
#define MOSN_N_CONNREF		(0x00002300 >> MOSN_SHIFT)	/* Connection refused */
#define MOSN_N_CONNFAIL		(0x00002400 >> MOSN_SHIFT)	/* Connection failed */
#define MOSN_N_BADCRED		(0x00002500 >> MOSN_SHIFT)	/* Bad username/password */
#define MOSN_N_BADKEY		(0x00002600 >> MOSN_SHIFT)	/* enc/decryption key inappropriate */
#define MOSN_N_SIGNATURE	(0x00002700 >> MOSN_SHIFT)	/* signature verification failure */
#define MOSN_N_NODEV		(0x00002800 >> MOSN_SHIFT)	/* no such device */
#define MOSN_N_PIPE			(0x00002900 >> MOSN_SHIFT)	/* broken pipe */
#define MOSN_N_REVOKED		(0x00002A00 >> MOSN_SHIFT)	/* e.g. license revoked */
#define MOSN_N_BADTIME		(0x00002B00 >> MOSN_SHIFT)	/* time inappropriate */
#define MOSN_N_RESOLV		(0x00002C00 >> MOSN_SHIFT)	/* name resolution failure */
#define MOSN_N_NETUNAVAIL	(0x00002D00 >> MOSN_SHIFT)	/* network unavailability (cached?) */
#define MOSN_N_CONNRESET	(0x00002E00 >> MOSN_SHIFT)	/* connection reset */
#define MOSN_N_CONNABORTED	(0x00002F00 >> MOSN_SHIFT)	/* connection aborted */
#define MOSN_N_HOSTUNREACH	(0x00003000 >> MOSN_SHIFT)	/* no route to host */
#define MOSN_N_HOSTDOWN		(0x00003100 >> MOSN_SHIFT)	/* host is down */
#define MOSN_N_WRONGDEV		(0x00003200 >> MOSN_SHIFT)	/* wrong device */
#define MOSN_N_UNKNOWNVAL	(0x00003300 >> MOSN_SHIFT)	/* unknown value */
#define MOSN_N_NOTATTACHED	(0x00003400 >> MOSN_SHIFT)	/* not attached */
#define MOSN_N_INVALPACKET	(0x00003500 >> MOSN_SHIFT)	/* Invalid Packet */
#define MOSN_N_2BIG			(0x00003600 >> MOSN_SHIFT)	/* Argument List Too Long */
#define MOSN_N_BADVER		(0x00003700 >> MOSN_SHIFT)	/* Bad Version */
#define MOSN_N_CLOSED		(0x00003800 >> MOSN_SHIFT)	/* Closed */
#define MOSN_N_NOTCONFIGURED (0x00003900 >> MOSN_SHIFT) /* Not Configured */
#define MOSN_N_KEEPALIVE	(0x00003A00 >> MOSN_SHIFT)	/* Keep Alive Failure */


#define MOSN_PERM		(MOSN_N_PERM				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NOENT		(MOSN_N_NOENT				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_TIMEDOUT	(MOSN_N_TIMEDOUT			| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_INTR		(MOSN_N_INTR				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_IO			(MOSN_N_IO					| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_MEM		(MOSN_N_MEM					| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_ACCESS		(MOSN_N_ACCESS				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_FAULT		(MOSN_N_FAULT				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_BUSY		(MOSN_N_BUSY				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_EXIST		(MOSN_N_EXIST				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NOTDIR		(MOSN_N_NOTDIR				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_ISDIR		(MOSN_N_ISDIR				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_INVAL		(MOSN_N_INVAL				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NFILE		(MOSN_N_NFILE				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_MFILE		(MOSN_N_MFILE				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NOSPC		(MOSN_N_NOSPC				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_FBIG		(MOSN_N_FBIG				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_ROFS		(MOSN_N_ROFS				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_RO			(MOSN_N_RO					| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NOSUP		(MOSN_N_NOSUP				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_INVALARG	(MOSN_N_INVALARG			| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_AGAIN		(MOSN_N_AGAIN				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NEVENT		(MOSN_N_NEVENT				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_INCONST	(MOSN_N_INCONST				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_ADDR		(MOSN_N_ADDR				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NOTEMPTY	(MOSN_N_NOTEMPTY			| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_DUP		(MOSN_N_DUP					| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_ERR		(MOSN_N_ERR					| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_HASH		(MOSN_N_HASH				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_CONTENT	(MOSN_N_CONTENT				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_EOF		(MOSN_N_EOF					| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_POLICY		(MOSN_N_POLICY				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_LICENSE	(MOSN_N_LICENSE				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_TASTE		(MOSN_N_TASTE				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_CONNREF	(MOSN_N_CONNREF				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_CONNFAIL	(MOSN_N_CONNFAIL			| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_BADCRED	(MOSN_N_BADCRED				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_BADKEY		(MOSN_N_BADKEY				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_SIGNATURE	(MOSN_N_SIGNATURE			| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NODEV		(MOSN_N_NODEV				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_PIPE		(MOSN_N_PIPE				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_REVOKED	(MOSN_N_REVOKED				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_BADTIME	(MOSN_N_BADTIME				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_RESOLV		(MOSN_N_RESOLV				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NETUNAVAIL	(MOSN_N_NETUNAVAIL			| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_CONNRESET	(MOSN_N_CONNRESET			| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_CONNABORTED (MOSN_N_CONNABORTED		| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_HOSTUNREACH (MOSN_N_HOSTUNREACH		| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_HOSTDOWN	 (MOSN_N_HOSTDOWN			| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_WRONGDEV	(MOSN_N_WRONGDEV			| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_UNKNOWNVAL	(MOSN_N_UNKNOWNVAL			| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NOTATTACHED (MOSN_N_NOTATTACHED		| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_INVALPACKET (MOSN_N_INVALPACKET		| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_2BIG		(MOSN_N_2BIG				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_BADVER		(MOSN_N_BADVER				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_CLOSED		(MOSN_N_CLOSED				| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_NOTCONFIGURED (MOSN_N_NOTCONFIGURED	| MOSN_T_ERROR | MOSN_F_SYSTEM)
#define MOSN_KEEPALIVE	(MOSN_N_KEEPALIVE			| MOSN_T_ERROR | MOSN_F_SYSTEM)

#ifndef EXTERNALPROTO

MOSAPI mosiop_t MOSCConv mos_iop_alloc(void);
MOSAPI void MOSCConv mos_iop_retain(mosiop_t);
MOSAPI void MOSCConv mos_iop_release(mosiop_t *);

MOSAPI void MOSCConv mos_notice_addfamily(int, const char *(*)(int),
  const char *(*)(int));
#endif

#endif /* _MOS_IOP_H_ */
