/*
 * Copyright (c) 2012 Jeff Rhyason <jeff@rhyason.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#if defined(Linux)
#define _GNU_SOURCE
#endif

#ifndef _KERNEL
#include <dlfcn.h>
#include <stdio.h>
#endif

#include "mos_stacktrace.h"

static void *getreturnaddr(int) __attribute__((noinline));
static void *getframeaddr(int) __attribute__((noinline));

#if defined(Linux)
/*
 * the builtins are not very safe, and like to crash on Linux.
 */
#define	STACK_LIMIT 7
#else
#define	STACK_LIMIT 32
#endif

size_t
mos_stacktrace(void **buffer, size_t size) {
    size_t i;

    for (i = 0; getframeaddr((int)i) != NULL && i < size && i < STACK_LIMIT ; i++) {
        buffer[i] = getreturnaddr((int)i);
        if (buffer[i] == NULL)
            break;
    }

    return (i - 0);
}

size_t
mos_getsymbolname(const void *caddr, char *buf, size_t len) {

#ifdef _KERNEL
	mos_snprintf(buf, len, "%p", caddr);

#else

    Dl_info info;
	int offset;
	void *addr;

	union {
		const void *cv;
		void *v;
	} deconstify;

	deconstify.cv = caddr;
	addr = deconstify.v;

	if (dladdr(addr, &info) == 0)
		return (snprintf(buf, len, "%p <%s>", addr, dlerror()));

	if (info.dli_sname == NULL)
		info.dli_sname = "???";
	if (info.dli_saddr == NULL)
		offset = 0;
	else
		offset = (int)((uintptr_t)addr - (uintptr_t)info.dli_saddr);

	snprintf(buf, len, "%s`%s+0x%x <%p>", mos_basename(info.dli_fname),
	  info.dli_sname, offset, addr);

#endif /* !_KERNEL */

	return (0);
}

static void *
getreturnaddr(int level) {

	return (NULL);
}

static void *
getframeaddr(int level) {

	return (NULL);
}
