#ifndef _MOS_READDIR_H_
#define _MOS_READDIR_H_

#include "mos_os.h"
#include "mos_iop.h"

#define MOS_DIRINFO_ISDIR		0x0001
#define MOS_DIRINFO_ISHIDDEN	0x0002

typedef struct mos_dirinfo {
	int			errcode;
	int			first;
	uint32_t	flags;
	void		*dirobj;	/* private impl. pointer */
	char		*path;		/* path opened in opendir() */
	char		filename[MOS_PATH_COMPONENT_MAX];
} mos_dirinfo_t;

MOSAPI int MOSCConv mos_opendir(mosiop_t, const char *, mos_dirinfo_t **);
MOSAPI void MOSCConv mos_closedir(mos_dirinfo_t **);
MOSAPI int MOSCConv mos_readdir(mosiop_t, mos_dirinfo_t *);

#endif /* _MOS_READDIR_H_ */
