#include <sys/file.h>
#include <sys/stat.h>
#include "unistd.h"
#include "fcntl.h"

#include "mos_fileio.h"
#include "mos_error-errno.h"

MOSAPI int MOSCConv
mos_file_mkdir(const char *fmt, ...) {
	char path[MOS_PATH_MAX];
	va_list va;
	size_t n;
	int err;

	va_start(va, fmt);
	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	va_end(va);
	if (n >= sizeof (path))
		return (0);

	err = mkdir(path, 0775);
	if (err != 0)
		return (0);
	return (1);
}

MOSAPI int MOSCConv
mos_file_rmdir(const char *fmt, ...) {
	char path[MOS_PATH_MAX];
	va_list va;
	size_t n;
	int err;

	va_start(va, fmt);
	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	va_end(va);
	if (n >= sizeof (path))
		return (0);

	err = rmdir(path);
	if (err != 0)
		return (0);
	return (1);
}

MOSAPI int MOSCConv
mos_file_exists(const char *fmt, ...) {
	char path[MOS_PATH_MAX];
	struct stat sb;
	va_list va;
	size_t n;
	int err;

	va_start(va, fmt);
	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	va_end(va);
	if (n >= sizeof (path))
		return (0);

	err = stat(path, &sb);
	if (err != 0)
		return (0);
	return (1);
}

MOSAPI int MOSCConv
mos_file_isdirectory(const char *fmt, ...) {
	char path[MOS_PATH_MAX];
	struct stat sb;
	va_list va;
	size_t n;
	int err;

	va_start(va, fmt);
	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	va_end(va);
	if (n >= sizeof (path))
		return (0);

	err = stat(path, &sb);
	if (err != 0)
		return (0);

	if (S_ISDIR(sb.st_mode))
		return (1);

	return (0);
}

MOSAPI int MOSCConv
mos_file_unlink(const char *fmt, ...) {
	char path[MOS_PATH_MAX];
	va_list va;
	size_t n;
	int err;

	va_start(va, fmt);
	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	va_end(va);
	if (n >= sizeof (path))
		return (MOSN_NOSPC);

	err = unlink(path);
	if (err != 0)
		return (mos_fromerrno(errno));

	return (0);
}


MOSAPI int MOSCConv
mos_file_open(mosiop_t iop, mos_file_t **mf, int flags, const char *fmt, ...) {
	char path[MOS_PATH_MAX];
	int oflags;
	va_list va;
	size_t n;
	int err;

	if (mf == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "file pointer is null"));

	va_start(va, fmt);
	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	va_end(va);
	if (n >= sizeof (path))
		return (MOS_ERROR(iop, MOSN_NOSPC, "path too long"));

	if (mos_strcmp(path, MOS_FILE_STDIN) == 0) {
		*mf = mos_malloc(sizeof (mos_file_t));
		(*mf)->fd = STDIN_FILENO;
		(*mf)->special = 1;
		return (0);
	}
	if (mos_strcmp(path, MOS_FILE_STDOUT) == 0) {
		*mf = mos_malloc(sizeof (mos_file_t));
		(*mf)->fd = STDOUT_FILENO;
		(*mf)->special = 1;
		return (0);
	}
	if (mos_strcmp(path, MOS_FILE_STDERR) == 0) {
		*mf = mos_malloc(sizeof (mos_file_t));
		(*mf)->fd = STDERR_FILENO;
		(*mf)->special = 1;
		return (0);
	}

	oflags = 0;
	if ((flags & (MOS_FILE_READ | MOS_FILE_WRITE)) == (MOS_FILE_READ | MOS_FILE_WRITE))
		oflags |= O_RDWR;
	else if (flags & MOS_FILE_READ)
		oflags |= O_RDONLY;
	else if (flags & MOS_FILE_WRITE)
		oflags |= O_WRONLY;

	if (flags & MOS_FILE_CREATE) {
		oflags |= O_CREAT;
		if (flags & MOS_FILE_EXCL)
			oflags |= O_EXCL;
	} else if (flags & MOS_FILE_TRUNC) {
		oflags |= O_TRUNC;
	}

	*mf = mos_malloc(sizeof (mos_file_t));

	(*mf)->special = 0;
	(*mf)->fd = open(path, oflags, 0644);
	if ((*mf)->fd < 0) {
		mos_free(*mf, sizeof (mos_file_t));
		*mf = NULL;
		return (MOS_ERROR(iop, mos_fromerrno(errno), "open(%s) failed", path));
	}

	if (flags & MOS_FILE_LOCK) {
		err = flock((*mf)->fd, LOCK_EX | LOCK_NB);
		if (err == -1) {
			if (errno == EWOULDBLOCK) {
				close((*mf)->fd);
				mos_free(*mf, sizeof (mos_file_t));
				*mf = NULL;
				return (MOSN_BUSY);
			}
			close((*mf)->fd);
			mos_free(*mf, sizeof (mos_file_t));
			*mf = NULL;
			return (MOS_ERROR(iop, mos_fromerrno(errno), "flock() failed"));
		}
	}

	return (0);
}

MOSAPI int MOSCConv
mos_file_close(mosiop_t iop, mos_file_t **mf) {

	if (mf == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "file pointer is null"));

	/* releases flock, if one existed */
	if ((*mf)->special == 0)
		close((*mf)->fd);
	mos_free(*mf, sizeof (mos_file_t));
	*mf = NULL;
	return (0);
}

MOSAPI int MOSCConv
mos_file_read(mosiop_t iop, mos_file_t *mf, void *buf, size_t *bufsz) {
	ssize_t n;

	if (mf == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "mf is null"));

	if (buf == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "buf is null"));

	if (bufsz == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "bufsz is null"));

	n = read(mf->fd, buf, *bufsz);
	if (n < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "read() failed"));
	if (n == 0)
		return (MOSN_EOF);

	*bufsz = n;
	return (0);
}

MOSAPI int MOSCConv
mos_file_write(mosiop_t iop, mos_file_t *mf, const void *buf, size_t bufsz) {
	ssize_t n;

	if (mf == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "mf is null"));

	if (buf == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "buf is null"));

	n = write(mf->fd, buf, bufsz);
	if (n < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "write() failed"));
	return (0);
}

MOSAPI int MOSCConv
mos_file_seek(mosiop_t iop, mos_file_t *mf, uint64_t off) {
	off_t err;

	if (mf == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "mf is null"));

	err = lseek(mf->fd, (off_t)off, SEEK_SET);
	if (err == -1)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "lseek(%llu, SEEK_SET) failed", off));

	return (0);
}

MOSAPI int MOSCConv
mos_file_trunc(mosiop_t iop, mos_file_t *mf, uint64_t length) {
	int err;

	if (mf == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "mf is null"));

	err = ftruncate(mf->fd, (off_t)length);
	if (err != 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "truncate(%llu) failed", length));

	return (0);
}

MOSAPI int MOSCConv
mos_file_getsize(mosiop_t iop, mos_file_t *mf, uint64_t *sz) {
	struct stat sb;
	int err;

	if (mf == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "mf is null"));

	err = fstat(mf->fd, &sb);
	if (err != 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "failed to stat file"));

	*sz = (uint64_t)sb.st_size;
	return (0);
}

MOSAPI int MOSCConv
mos_file_getoffset(mosiop_t iop, mos_file_t *mf, uint64_t *off) {
	off_t soff;

	if (mf == NULL)
		return (MOS_ERROR(iop, MOSN_INVALARG, "mf is null"));

	soff = lseek(mf->fd, 0, SEEK_CUR);
	if (soff == -1)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "lseek() failed"));

	*off = (uint64_t)soff;
	return (0);
}

MOSAPI int MOSCConv
mos_file_readx(mosiop_t iop, void *buf, size_t *bufsz, const char *fmt, ...) {
	char path[MOS_PATH_MAX];
	va_list va;
	ssize_t n;
	int err;
	int fd;

	va_start(va, fmt);
	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	va_end(va);
	if (n >= (ssize_t)sizeof (path))
		return (MOS_ERROR(iop, MOSN_NOSPC, "path is too long"));

	fd = open(path, O_RDONLY);
	if (fd < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "open(%s) failed", path));

	err = flock(fd, LOCK_EX | LOCK_NB);
	if (err == -1) {
		if (errno == EWOULDBLOCK) {
			close(fd);
			return (MOSN_BUSY);
		}
		err = MOS_ERROR(iop, mos_fromerrno(errno), "flock() failed");
		close(fd);
		return (err);
	}

	n = read(fd, buf, *bufsz);
	if (n < 0) {
		err = MOS_ERROR(iop, mos_fromerrno(errno), "read() failed");
	} else {
		*bufsz = n;
		err = 0;
	}
	flock(fd, LOCK_UN);
	close(fd);

	return (err);
}

MOSAPI int MOSCConv
mos_file_writex(mosiop_t iop, const void *buf, size_t bufsz, const char *fmt, ...) {
	char path[MOS_PATH_MAX];
	va_list va;
	ssize_t n;
	int err;
	int fd;

	va_start(va, fmt);
	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	va_end(va);
	if (n >= (ssize_t)sizeof (path))
		return (MOS_ERROR(iop, MOSN_NOSPC, "path is too long"));

	fd = open(path, O_WRONLY | O_TRUNC | O_CREAT, 0644);
	if (fd < 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "open(%s) failed", path));

	err = flock(fd, LOCK_EX | LOCK_NB);
	if (err == -1) {
		if (errno == EWOULDBLOCK) {
			close(fd);
			return (MOSN_BUSY);
		}
		err = MOS_ERROR(iop, mos_fromerrno(errno), "open(%s) failed", path);
		close(fd);
		return (err);
	}

	n = write(fd, buf, bufsz);
	if (n < 0)
		err = MOS_ERROR(iop, mos_fromerrno(errno), "write() failed");
	else
		err = 0;

	flock(fd, LOCK_UN);
	close(fd);

	return (err);
}

MOSAPI int MOSCConv
mos_file_getsizex(mosiop_t iop, uint64_t *sz, const char *fmt, ...) {
	char path[MOS_PATH_MAX];
	struct stat sb;
	va_list va;
	size_t n;
	int err;

	va_start(va, fmt);
	n = mos_vsnprintf(path, sizeof (path), fmt, va);
	va_end(va);
	if (n >= sizeof (path))
		return (MOS_ERROR(iop, MOSN_NOSPC, "path is too long"));

	err = stat(path, &sb);
	if (err != 0)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "failed to stat %s", path));

	*sz = (uint64_t)sb.st_size;
	return (0);
}
