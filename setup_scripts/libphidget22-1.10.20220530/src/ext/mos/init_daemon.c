/*
 * $@Copyright ISSci_copyright:ISSci:BSD; YEARS 2003-2005
 *
 * Copyright (C) 2003-2005 by Industrial System Sciences, Inc.
 * All Rights Reserved.
 *
 * www.issci.ca:@$
 *
 * $@License ISSci_licenses:ISSci:BSD;
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice(s), this list of conditions and the following disclaimer as
 *    the first lines of this file unmodified other than the possible
 *    addition of one or more copyright notices.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice(s), this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER(S) ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER(S) BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *:@$
 *
 * $Id: init_daemon.c,v 1.1 2009/03/03 21:35:37 davidc Exp $
 */

/*
 *	Written by: Chad David  ACNS Inc. 1996.
 *	davidc@acns.ab.ca
 *
 */
#include <sys/types.h>
#include <sys/errno.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <signal.h>

#include "mos_os.h"
#include "init_daemon.h"

MOSAPI int MOSCConv
init_daemon(int flags) {
	pid_t pid;
	int fd;

	/*
	 * Fork a child and kill the parent.  This does a number of things:
	 *	1) makes the shell think the command is done.
	 *	2) Child inherits the pgid, but gets a new pid, this makes sure
	 *	  that the child is not a group leader (hate to see it die by accident).
	 */
	if ((pid = fork()) < 0)
		return(1);
	else if (pid != 0)
		exit(0);	/* Parent is killed */

	/* setsid() will create a new session, and change our pgroup id */
	setsid();

	if ((pid = fork()) < 0)
		return(1);
	else if (pid != 0)
		exit(0);

	/*
	 * Change the daemons working directory to the root directory.
	 * This is done to make sure the daemon isn't running on 
	 * a mounted filesystem.
	 * (tough to unmount the filesystem if so)
	 */
	if ((flags & NOCHDIR) == 0) {
		if (chdir("/") != 0)
			return(1);
	}

	/*
	 * Set the file creation mask to 0.  The file mode creation mask that is
	 * inherited could cause some problems with files created.
	 */
	if ((flags & NOUMASK) == 0)
		umask((mode_t)(int)0);

	/* Map stdin, stdout, and stderr to /dev/null */
	if ((flags & NOCLOSEFD) == 0) {
		if ((fd = open("/dev/null", O_RDWR, 0)) != -1) {
			(void)dup2(fd, STDIN_FILENO);
			(void)dup2(fd, STDOUT_FILENO);
			(void)dup2(fd, STDERR_FILENO);
			if (fd > 2)
				(void)close(fd);
		}
	}

	/*
	 * Handle write()s to disconnected sockets where they happen,
	 * instead of in a signal handler.
	 */
	if ((flags & NOSIGPIPEIGN) == 0)
		(void)signal(SIGPIPE, SIG_IGN);

	return(0);
}

/* EOF */
