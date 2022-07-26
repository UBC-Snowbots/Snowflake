#include <dlfcn.h>

#include "mos_dl.h"

void *
mos_dlopen(const char *path, int flags) {

	return (dlopen(path, flags));
}

int
mos_dlclose(void *hdl) {

	return (dlclose(hdl));
}

void *
mos_dlsym(void * __restrict hdl, const char * __restrict symname) {

	return (dlsym(hdl, symname));
}

const char *
mos_dlerror(char *msgbuf, size_t bufsz) {

	mos_strlcpy(msgbuf, dlerror(), bufsz);
	return (msgbuf);
}
