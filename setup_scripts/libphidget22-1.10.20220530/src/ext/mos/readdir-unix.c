
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

#include "mos_readdir.h"
#include "mos_error-errno.h"

int
mos_opendir(mosiop_t iop, const char *path, mos_dirinfo_t **di) {
	DIR	*dptr;

	dptr = opendir(path);
	if (dptr == NULL)
		return (MOS_ERROR(iop, mos_fromerrno(errno), "opendir() failed"));

	*di = mos_malloc(sizeof (mos_dirinfo_t));
	memset(*di, 0, sizeof (mos_dirinfo_t));
	(*di)->dirobj = dptr;
	(*di)->path = mos_strdup(path, NULL);

	return (0);
}

void
mos_closedir(mos_dirinfo_t **di) {

	closedir((DIR *)((*di)->dirobj));
	mos_free((*di)->path, mos_strlen((*di)->path) + 1);
	mos_free(*di, sizeof (mos_dirinfo_t));
	*di = NULL;
}

int
mos_readdir(mosiop_t iop, mos_dirinfo_t *di) {
	char epath[MOS_PATH_MAX];
	struct dirent *de;
	struct stat sb;
	int err;

	di->errcode = 0;

next:

	errno = 0;
	de = readdir((DIR *) (di->dirobj));
	if (de == NULL) {
		if (errno == 0) {
			di->errcode = MOSN_NOENT;
			return (0);
		}

		di->errcode = MOS_ERROR(iop, mos_fromerrno(errno),
		  "readdir() failed: %s", strerror(errno));
		return (0);
	}

	if (mos_strcmp(de->d_name, ".") == 0 || mos_strcmp(de->d_name, "..") == 0)
		goto isdir;

	mos_snprintf(epath, sizeof (epath), "%s/%s", di->path, de->d_name);
	err = stat(epath, &sb);
	if (err != 0) {
		if (errno == ENOENT)
			goto next;
		return (MOS_ERROR(iop, mos_fromerrno(errno),
		  "failed to stat dirent '%s': %s", epath, strerror(errno)));
	}

	if (S_ISDIR(sb.st_mode))
isdir:
		di->flags |= MOS_DIRINFO_ISDIR;
	else
		di->flags &= ~MOS_DIRINFO_ISDIR;

	mos_strlcpy(di->filename, de->d_name, sizeof(di->filename));

	return (0);
}
