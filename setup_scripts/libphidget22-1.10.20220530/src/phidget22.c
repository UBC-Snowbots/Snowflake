/*
 * This file is part of libphidget22
 *
 * Copyright 2015 Phidgets Inc <patrick@phidgets.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/>
 */

#include "phidgetbase.h"
#include "mos/mos_os.h"
#include "mos/bsdqueue.h"

#include "network/network.h"

#include "manager.h"
#include "phidget.h"

#if defined(_MACOSX) && !defined(_IPHONE)
#include "plat/mac/macusb.h"
#endif

void FormatInit(void);
void PhidgetDispatchInit(void);
void PhidgetDispatchFini(void);
void PhidgetDispatchStop(void);
void PhidgetObjectInit(void);
void PhidgetObjectFini(void);
void PhidgetStatsInit(void);
void PhidgetStatsFini(void);

static void joinCentralThread(void);

static MOS_TASK_RESULT CentralThreadFunction(void *);
static mos_task_t centralThread;
static int centralThreadRun;
static mos_mutex_t centralThreadLock;
static mos_cond_t centralThreadCond;

extern void runNetAttachDetachQueue(void);

/* Called by MOS_PANIC to log reason */
void
mos_log_err(const char *fmt, ...) {
	char buf[256];
	va_list va;

	va_start(va, fmt);
	mos_vsnprintf(buf, sizeof(buf), fmt, va);
	va_end(va);

	fprintf(stderr, "%s\n", buf);
	logcrit("%s", buf);
}

#ifdef _MACOSX
static CFRunLoopRef runLoop;
static int macInitDone = PFALSE;
#endif

static void
_Phidget22Initialize() {
	mos_mutex_init(&centralThreadLock);
	mos_cond_init(&centralThreadCond);
}

#ifdef _WINDOWS
#define _CONSTRUCTOR_
#define _DESTRUCTOR_
#else
#if defined(__GNUC__) || defined(__clang__)
#define _CONSTRUCTOR_ __attribute__((constructor))
#define _DESTRUCTOR_ __attribute__((destructor))
#else
#error "compiler does not support constructor/destructor"
#endif
#endif /* !_WINDOWS */

#if LOCALE_SUPPORT
static void LocaleInit(void);
static void LocaleFini(void);
#else
#define LocaleInit()
#define LocaleFini()
#endif

static void LastErrorInit(void);

#ifdef _WINDOWS
static DWORD lastErrorTLSIndex;
static LPVOID allocLastErrorStorage(void);
static void freeLastErrorStorage(void);
#else

#include <pthread.h>
static pthread_key_t lastErrorTLSKey;
static pthread_once_t lastErrorTLSKeyOnce = PTHREAD_ONCE_INIT;
static int lastErrorTLSKeyValid = 0;

static PhidgetErrorDetailHandle GetLastError() {
	PhidgetErrorDetailHandle threadLastErr;
	void *data;

	if (!lastErrorTLSKeyValid)
		return NULL;

	if ((data = pthread_getspecific(lastErrorTLSKey)) == NULL) {
		data = malloc(sizeof(PhidgetErrorDetail));
		memset(data, 0, sizeof(PhidgetErrorDetail));
		pthread_setspecific(lastErrorTLSKey, data);
	}

	threadLastErr = (PhidgetErrorDetailHandle)data;
	return (threadLastErr);
}
static void LastErrorFree(void *data) {
	PhidgetErrorDetailHandle threadLastErr;

	if (data) {
		threadLastErr = (PhidgetErrorDetailHandle)data;

		if (threadLastErr->detail) {
			free(threadLastErr->detail);
			threadLastErr->detail = NULL;
		}

		free(threadLastErr);
	}
}
static void LastErrorMakeKey() {

	if (!pthread_key_create(&lastErrorTLSKey, LastErrorFree))
		lastErrorTLSKeyValid = 1;
}
static void LastErrorInit() {

	pthread_once(&lastErrorTLSKeyOnce, LastErrorMakeKey);
}

#endif

static int
mallocprint(void *arg, const char *fmt, ...) {
	char buf[1024];
	va_list va;
	int res;

	va_start(va, fmt);
	res = mos_raw_vsnprintf(buf, sizeof(buf), fmt, va);
	va_end(va);

	if (res < 0 || res >(int)sizeof(buf) - 1)
		buf[sizeof(buf) - 1] = '\0';

#ifdef _WINDOWS
	OutputDebugStringA(buf);
#else
	fprintf(stderr, "%s", buf);
#endif

	return (0);
}

mos_malloc_printer_t mallocprinter = mallocprint;

_CONSTRUCTOR_
static void
Phidget22Initialize() {

	mos_init();
	LocaleInit();
	LastErrorInit();
	FormatInit();
	PhidgetStatsInit();
	PhidgetLogInit();
	PhidgetObjectInit();
	_Phidget22Initialize();
	PhidgetManagerInit();
	PhidgetInit();
	PhidgetDispatchInit();
	PhidgetUSBInit();
	PhidgetNetInit();
}

//_DESTRUCTOR_
static void
Phidget22Finalize() {

	logdebug("Finalizing library");

	joinCentralThread();

	PhidgetDispatchFini();
	PhidgetNetFini();
	PhidgetUSBFini();
	PhidgetManagerFini();
	PhidgetFini();
	PhidgetObjectFini();
	PhidgetLogFini();
	PhidgetStatsFini();

#ifdef _WINDOWS
	freeLastErrorStorage();
#endif

#ifdef MOS_TRACK_ALLOCATIONS
	if (mos_print_allocated_bytes())
		mos_dump_allocation_set(0, mallocprinter, NULL);
#endif

	mos_mutex_destroy(&centralThreadLock);
	mos_cond_destroy(&centralThreadCond);

	LocaleFini();

	mos_fini();
}

PhidgetReturnCode
Phidget_finalize(int flags) {

	Phidget22Finalize();
	return (EPHIDGET_OK);
}

#if LOCALE_SUPPORT
#ifdef _WINDOWS

static _locale_t c_locale;

static void LocaleInit(void) {
	c_locale = _create_locale(LC_NUMERIC, "C");
}

static void LocaleFini(void) {
	_free_locale(c_locale);
}

double phid_strtod_c(const char* str, char** endptr) {

	return (_strtod_l(str, endptr, c_locale));
}
int phid_snprintf_c(char * s, size_t n, const char * format, ...) {
	va_list args;
	int ret;

	va_start(args, format);
	ret = _vsnprintf_l(s, n, format, c_locale, args);
	va_end(args);

	return (ret);
}

#else

static locale_t c_locale;

static void LocaleInit(void) {
	c_locale = newlocale(LC_NUMERIC_MASK, "C", (locale_t)0);
}

static void LocaleFini(void) {
	freelocale(c_locale);
}

double phid_strtod_c(const char* str, char** endptr) {

	return (strtod_l(str, endptr, c_locale));
}

#if THREAD_LOCALE_XLOCALE_H
// xlocale on freebsd/macos have vsnprintf_l
int phid_snprintf_c(char * s, size_t n, const char * format, ...) {
	va_list args;
	int ret;

	va_start(args, format);
	ret = vsnprintf_l(s, n, c_locale, format, args);
	va_end(args);

	return (ret);
}
#else
// On linux, we'll have to actually switch the thread-locale
int phid_snprintf_c(char * s, size_t n, const char * format, ...) {
	locale_t oldLocale;
	va_list args;
	int ret;

	oldLocale = uselocale((locale_t)0);
	uselocale(c_locale);
	va_start(args, format);
	ret = vsnprintf(s, n, format, args);
	va_end(args);
	uselocale(oldLocale);

	return (ret);
}
#endif

#endif
#else

// No locale support

double phid_strtod_c(const char* str, char** endptr) {

	return (strtod(str, endptr));
}
int phid_snprintf_c(char * s, size_t n, const char * format, ...) {
	va_list args;
	int ret;

	va_start(args, format);
	ret = vsnprintf(s, n, format, args);
	va_end(args);

	return (ret);
}

#endif

#ifdef _WINDOWS
#ifdef _MANAGED
#pragma managed(push, off)
#endif

static void LastErrorInit(void) {

	allocLastErrorStorage();
}
static LPVOID allocLastErrorStorage(void) {
	LPVOID data;

	data = (LPVOID)LocalAlloc(LPTR, sizeof(PhidgetErrorDetail));
	memset(data, 0, sizeof(PhidgetErrorDetail));
	if (data != NULL)
		TlsSetValue(lastErrorTLSIndex, data);

	return data;
}
static void freeLastErrorStorage(void) {
	PhidgetErrorDetailHandle threadLastErr;
	LPVOID data;

	data = TlsGetValue(lastErrorTLSIndex);
	if (data != NULL) {
		threadLastErr = (PhidgetErrorDetailHandle)data;

		if (threadLastErr->detail) {
			free(threadLastErr->detail);
			threadLastErr->detail = NULL;
		}

		LocalFree((HLOCAL)data);
	}
}

BOOL APIENTRY
DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {

	switch (ul_reason_for_call) {
	case DLL_PROCESS_ATTACH:

		// Claim TLS index for storing last error
		if ((lastErrorTLSIndex = TlsAlloc()) == TLS_OUT_OF_INDEXES)
			return FALSE;
		Phidget22Initialize();
		break;

	case DLL_THREAD_ATTACH:
		allocLastErrorStorage();
		logdebug("Windows thread attach: 0x%08x", mos_self());
		break;

	case DLL_THREAD_DETACH:
		freeLastErrorStorage();
		logdebug("Windows thread detach: 0x%08x", mos_self());
		break;

	case DLL_PROCESS_DETACH:
		freeLastErrorStorage();
		// Release TLS index
		TlsFree(lastErrorTLSIndex);
		break;
	}
	return TRUE;
}

#ifdef _MANAGED
#pragma managed(pop)
#endif
#endif


#ifdef _MACOSX
void macPeriodicTimerFunction(CFRunLoopTimerRef timer, void *Handle);
void macFindOpenChannelsSource(void *nothing);
CFRunLoopTimerRef timer = NULL;
CFRunLoopSourceRef findOpenChannelsSource = NULL;
#endif

void
NotifyCentralThread() {

	mos_cond_broadcast(&centralThreadCond);
}

PhidgetReturnCode
StartCentralThread() {
	PhidgetReturnCode res;

	mos_mutex_lock(&centralThreadLock);
	if (centralThreadRun == 1) {
		mos_cond_broadcast(&centralThreadCond);
		mos_mutex_unlock(&centralThreadLock);
		return (EPHIDGET_OK);
	}

#ifdef _MACOSX
	if (findOpenChannelsSource == NULL) {
		CFRunLoopSourceContext sourceContext = { 0 };
		sourceContext.perform = macFindOpenChannelsSource;
		findOpenChannelsSource = CFRunLoopSourceCreate(kCFAllocatorDefault, 0, &sourceContext);
	}
	if (timer == NULL)
		timer = CFRunLoopTimerCreate(kCFAllocatorDefault, 0, 0.250, 0, 0, macPeriodicTimerFunction, NULL);
#endif

	centralThreadRun = 1;
	res = mos_task_create(&centralThread, CentralThreadFunction, NULL);
	if (res != 0) {
		centralThreadRun = 0;
		mos_mutex_unlock(&centralThreadLock);
		return (res);
	}

	mos_mutex_unlock(&centralThreadLock);

#ifdef _MACOSX
		//make sure mac thread stuff is initialized
	while (!macInitDone)
		mos_usleep(10000);
	//run findOpenChannels in the context of the central thread
	CFRunLoopSourceSignal(findOpenChannelsSource);
	CFRunLoopWakeUp(runLoop);
#endif

	return (EPHIDGET_OK);
}

static int
needCentralThread() {

	if (centralThreadRun != 1)
		return (0);

	if (CHANNELS_EMPTY && MTAILQ_EMPTY(&phidgetManagerList) && !PhidgetNet_isStarted())
		return (0);
	return (1);
}

static void
joinCentralThread() {

#ifdef _MACOSX
	if (macInitDone) {
		PhidgetUSBTeardownNotifications();

#ifdef LIGHTNING_SUPPORT
		PhidgetLightning_teardown();
#endif

		CFRunLoopRemoveTimer(runLoop, timer, kCFRunLoopDefaultMode);
		CFRunLoopRemoveSource(runLoop, findOpenChannelsSource, kCFRunLoopDefaultMode);

		CFRunLoopStop(runLoop);

		macInitDone = PFALSE;
	}
#endif

	mos_mutex_lock(&centralThreadLock);
	while (centralThreadRun != 0) {
		centralThreadRun = 2;
		mos_cond_broadcast(&centralThreadCond);
		mos_cond_wait(&centralThreadCond, &centralThreadLock);
	}
	mos_mutex_unlock(&centralThreadLock);

}

#ifdef _MACOSX
//this is run every 250ms from the CentralThread runLoop
void
macPeriodicTimerFunction(CFRunLoopTimerRef timer, void *Handle) {
	PhidgetDeviceHandle device;

	logverbose("running macPeriodicTimerFunction()");
	PhidgetWriteLockDevices();

	FOREACH_DEVICE(device) {
		switch (device->deviceInfo.class) {
		case PHIDCLASS_MESHDONGLE:
			addMeshHubs(device);
			break;
		case PHIDCLASS_HUB:
			scanVintDevices(device);
			break;
		default:
			break;
		}
	}

	PhidgetUnlockDevices();

	mos_mutex_lock(&centralThreadLock);
	if (needCentralThread()) {
		matchOpenChannels(); //this looks for attached active devices and opens them
		runNetAttachDetachQueue();
	}
	mos_mutex_unlock(&centralThreadLock);

	return;
}

//This is run when a new Phidget is registered when the CentralThread is already running
void
macFindOpenChannelsSource(void *ctx) {
	logverbose("running macFindOpenChannelsSource()");
	mos_mutex_lock(&centralThreadLock);
	if (needCentralThread()) {
		matchOpenChannels(); //this looks for attached active devices and opens them
	}
	mos_mutex_unlock(&centralThreadLock);
	return;
}
#endif

//The central thread should stop itself when there are no more active devices...?
//Or we can stop it in unregisterlocaldevice

static MOS_TASK_RESULT
CentralThreadFunction(void *_param) {
	PhidgetDeviceHandle device;

	mos_task_setname("Phidget22 Central Thread");
	logdebug("central thread started: 0x%08x", mos_self());

#ifdef SPI_SUPPORT
	populateAttachedSPIDevices();
#endif

#ifdef _MACOSX
	PhidgetReturnCode res;

	runLoop = CFRunLoopGetCurrent();

	CFRunLoopAddTimer(runLoop, timer, kCFRunLoopDefaultMode);
	CFRunLoopAddSource(runLoop, findOpenChannelsSource, kCFRunLoopDefaultMode);

	macInitDone = PTRUE;

	//setup notifications of Phidget attach/detach
	res = PhidgetUSBSetupNotifications(runLoop);
	if (res != EPHIDGET_OK) {
		logerr("PhidgetUSBSetupNotifications failed: %d", res);
	}

#ifdef LIGHTNING_SUPPORT
	PhidgetLightning_setup();
#endif

	//start run loop - note that this blocks until JoinCentralThread() is called.
	logverbose("Starting CFRunLoopRun()...");
	CFRunLoopRun();
	mos_mutex_lock(&centralThreadLock);

#else

	mos_mutex_lock(&centralThreadLock);
	while (needCentralThread()) {
		PhidgetManager_poll();
		matchOpenChannels();
		runNetAttachDetachQueue();

		mos_cond_timedwait(&centralThreadCond, &centralThreadLock, 250000000); //250ms
	}
#endif

#ifdef SPI_SUPPORT
	clearAttachedSPIDevices();
#endif

	PhidgetWriteLockDevices();
	while ((device = MTAILQ_FIRST(&phidgetDevices)) != NULL) {
		chlog("centralthread %"PRIphid"", device);
		deviceDetach(device);
	}
	MOS_ASSERT(DEVICES_EMPTY);
	MTAILQ_INIT(&phidgetDevices);
	PhidgetUnlockDevices();

	/* Shut down all dispatchers */
	PhidgetDispatchStop();

#if defined(_LINUX) || defined(_FREEBSD) && !defined(_ANDROID)
	//Shut down USB
	PhidgetUSBUninit();
#endif

	logverbose("Central Thread exiting");

	centralThreadRun = 0;
	mos_cond_broadcast(&centralThreadCond);
	mos_mutex_unlock(&centralThreadLock);

	MOS_TASK_EXIT(EPHIDGET_OK);
}

API_PRETURN
Phidget_resetLibrary() {
	PhidgetChannelHandle channel, channeltmp;
	PhidgetManagerHandle manager, managertmp;

	logdebug("Resetting library");

	channel = MTAILQ_FIRST(&phidgetChannels);
	while (channel != NULL) {
		channeltmp = MTAILQ_NEXT(channel, link);
		Phidget_close((PhidgetHandle)channel);
		PhidgetRelease(&channel);
		channel = channeltmp;
	}

	manager = MTAILQ_FIRST(&phidgetManagerList);
	while (manager != NULL) {
		managertmp = MTAILQ_NEXT(manager, link);
		PhidgetManager_close(manager);
		PhidgetManager_delete(&manager);
		manager = managertmp;
	}

	PhidgetNet_disableServerDiscovery(PHIDGETSERVER_DEVICEREMOTE);
	PhidgetNet_disableServerDiscovery(PHIDGETSERVER_WWWREMOTE);
	PhidgetNet_disableServerDiscovery(PHIDGETSERVER_SBC);
	PhidgetNet_removeAllServers();

	Phidget22Finalize();
	Phidget22Initialize();

	return (EPHIDGET_OK);
}

PhidgetReturnCode
Phidget_setLastError(PhidgetReturnCode code, const char *fmt, ...) {
	PhidgetErrorDetailHandle threadLastErr;
	char detail[4096];
	va_list va;

	// Don't store success as a last error
	if (code == EPHIDGET_OK)
		return (EPHIDGET_OK);

#ifdef _WINDOWS
	LPVOID lpvData;

	lpvData = TlsGetValue(lastErrorTLSIndex);
	if (lpvData == NULL) {
		lpvData = allocLastErrorStorage();
		if (lpvData == NULL)
			return (EPHIDGET_NOMEMORY);
	}

	threadLastErr = (PhidgetErrorDetailHandle)lpvData;
#else
	// Linux/macOS
	threadLastErr = GetLastError();
	if (threadLastErr == NULL)
		return (EPHIDGET_NOMEMORY);
#endif

	// NOTE: NOT using mos_strdup/mos_free, etc. because these allocations can persist beyond finalize
	//  and crash the mos memory tracker.
	threadLastErr->code = code;
	if (threadLastErr->detail) {
		free(threadLastErr->detail);
		threadLastErr->detail = NULL;
	}
	if (fmt != NULL) {
		va_start(va, fmt);
		mos_vsnprintf(detail, sizeof(detail), fmt, va);
		va_end(va);
#ifdef _WINDOWS
		threadLastErr->detail = _strdup(detail);
#else
		threadLastErr->detail = strdup(detail);
#endif
	}

	return (EPHIDGET_OK);
}

PhidgetReturnCode
Phidget_getLastError(PhidgetReturnCode *code, const char **desc, char *detailOut, size_t *detailLen) {
	PhidgetErrorDetailHandle threadLastErr;
	const char *detail;

	TESTPTR(detailLen);

#ifdef _WINDOWS
	LPVOID lpvData;

	lpvData = TlsGetValue(lastErrorTLSIndex);
	if (lpvData == NULL)
		return (EPHIDGET_UNEXPECTED);

	threadLastErr = (PhidgetErrorDetailHandle)lpvData;
#else
	// Linux/macOS
	threadLastErr = GetLastError();
	if (threadLastErr == NULL) {
		if (!lastErrorTLSKeyValid)
			return (EPHIDGET_UNSUPPORTED);
		return (EPHIDGET_UNEXPECTED);
	}
#endif

	*code = threadLastErr->code;

	if (desc)
		*desc = Phidget_strerror(threadLastErr->code);

	if (threadLastErr->detail != NULL && threadLastErr->detail[0] != '\0')
		detail = threadLastErr->detail;
	else
		detail = Phidget_strerrordetail(threadLastErr->code);

	if (detailOut == NULL) {
		*detailLen = (mos_strlen(detail) + 1);
		return (EPHIDGET_OK);
	}

	if (*detailLen > 0) {
		if (detail == NULL || strlen(detail) <= 0) {
			detailOut[0] = '\0';
		} else {
			mos_strlcpy(detailOut, detail, *detailLen);
			// Trim trailing newline
			if (detailOut[strlen(detailOut) - 1] == '\n')
				detailOut[strlen(detailOut) - 1] = '\0';
		}
	}

	return (EPHIDGET_OK);
}
