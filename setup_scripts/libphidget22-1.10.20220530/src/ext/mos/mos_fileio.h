#ifndef _MOS_FILEIO_H_
#define _MOS_FILEIO_H_

#include "mos_os.h"
#include "mos_iop.h"

typedef struct _mos_file {
#ifdef Windows
	HANDLE	handle;
#else
	int		fd;
#endif
	int		special;
} mos_file_t;

#define MOS_FILE_READ	0x01
#define MOS_FILE_WRITE	0x02
#define MOS_FILE_CREATE	0x04
#define MOS_FILE_EXCL	0x08
#define MOS_FILE_TRUNC	0x10
#define MOS_FILE_LOCK	0x20

#define MOS_FILE_STDIN	"<_stdin_>"
#define MOS_FILE_STDOUT	"<_stdout_>"
#define MOS_FILE_STDERR	"<_stderr_>"

MOSAPI int MOSCConv mos_file_mkdir(const char *, ...);
MOSAPI int MOSCConv mos_file_rmdir(const char *, ...);
MOSAPI int MOSCConv mos_file_exists(const char *, ...);
MOSAPI int MOSCConv mos_file_isdirectory(const char *, ...);
MOSAPI int MOSCConv mos_file_unlink(const char *, ...);
MOSAPI int MOSCConv mos_file_open(mosiop_t, mos_file_t **, int, const char *, ...);
MOSAPI int MOSCConv mos_file_close(mosiop_t, mos_file_t **);
MOSAPI int MOSCConv mos_file_read(mosiop_t, mos_file_t *, void *, size_t *);
MOSAPI int MOSCConv mos_file_write(mosiop_t, mos_file_t *, const void *, size_t);
MOSAPI int MOSCConv mos_file_getsize(mosiop_t, mos_file_t *, uint64_t *);
MOSAPI int MOSCConv mos_file_getoffset(mosiop_t, mos_file_t *, uint64_t *);
MOSAPI int MOSCConv mos_file_trunc(mosiop_t, mos_file_t *, uint64_t);
MOSAPI int MOSCConv mos_file_seek(mosiop_t, mos_file_t *, uint64_t);
MOSAPI int MOSCConv mos_file_getsizex(mosiop_t, uint64_t *, const char *, ...);
MOSAPI int MOSCConv mos_file_readx(mosiop_t, void *, size_t *, const char *, ...);
MOSAPI int MOSCConv mos_file_writex(mosiop_t, const void *, size_t, const char *, ...);

#endif /* _MOS_FILEIO_H_ */
