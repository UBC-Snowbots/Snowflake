#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include "mos_os.h"
#include "mos_iop.h"
#include "mos_getpasswd.h"

int
mos_getpasswd(mosiop_t iop, const char *prompt, char *pwd, size_t pwdsz) {
	char *c;

	c = getpass(prompt);
	if (c == NULL)
		return (MOS_ERROR(iop, MOSN_ERR, "failed to get password: %s",
		  strerror(errno)));

	if (mos_strlen(c) >= pwdsz)
		return (MOS_ERROR(iop, MOSN_NOSPC, "input larger than buffer"));

	mos_strlcpy(pwd, c, pwdsz);

	return (0);
}
