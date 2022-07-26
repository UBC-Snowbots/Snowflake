#define _PHIDGET_NETWORKCODE

#include "phidgetbase.h"
#include "manager.h"
#include "bridge.h"
#include "stats.h"

#include "mos/mos_os.h"
#include "mos/mos_time.h"
#include "mos/mos_assert.h"

#ifdef DEBUG
#define displog(...) PhidgetLog_loge(NULL, 0, __func__, "_phidget22disp", PHIDGET_LOG_INFO, __VA_ARGS__)
#else
#define displog(...)
#endif

#define DISPATCHENTRY_PHID_IMAX		64		/* inbound limit to allow on a given phid */
#define DISPATCHENTRY_PHID_OSOFTMAX	200		/* soft outbound limit to allow on a given phid */
#define DISPATCHENTRY_PHID_OHARDMAX	4096	/* hard outbound limit to allow on a given phid */
#define DISPATCHENTRY_MAX			32768	/* max entries to create before failing */
#define DISPATCHENTRY_DESIRED		256		/* entries to try and balance the system at */
#define DISPATCHENTRY_MAXDATA		64		/* data buffer size */

#define DISPATCHERS_MAX				32	/* max dispatch threads we will allow at one time */
#define DISPATCHERS_DESIRED_IDLE	4	/* the number of idle threads we'd like to have */

extern uint32_t phidgetChannelsCount;

#define DISPATCHERS_DESIRED (phidgetChannelsCount + DISPATCHERS_DESIRED_IDLE)

typedef enum dispatchtype {
	DE_NOTHING			= 0,
	DE_MGR_ATTACH,			/* notify new manager of existing devices */
	DE_MGR_ATTACHCH,		/* notify manager of a channel attach */
	DE_MGR_DETACHCH,		/* notify manager of a channel detach */
	DE_DEVICE_ATTACH,		/* device has attached: notifies managers of each channel */
	DE_DEVICE_DETACH,		/* device has detached: notifies managers of each channel */
	DE_CHANNEL_ATTACH,		/* channel has attached: notifies user */
	DE_CHANNEL_DETACH,		/* channel has detached: notifies user */
	DE_CHANNEL_SETSTATUS,	/* channel setstatus */
	DE_CHANNEL_INITEVENTS,	/* channel initial events */
	DE_USERREQ,				/* user request to the channel - includes setStatus to attaching */

	DE_NOT_DETACHING,		/* EVENTS AFTER THIS ARE NOT TO BE DISPATCHED TO DETACHING OR DETACHED PHIDGETS */
	DE_USERREQCALLBACK,		/* handle async callback from a user request */
	DE_THROW_AWAY,			/* EVENTS AFTER THIS CAN BE THROWN AWAY */

	DE_CHANNEL_BRIDGEPKT,	/* deliver bridge packet to channel */
	DE_CLIENTBRIDGEPACKET,	/* deliver bridge packet from server */
	DE_SERVERBRIDGEPACKET,	/* deliver bridge packet from client */
} dispatchtype_t;

#define DISPATCHENTRY_ONLIST	0x01
#define DISPATCHENTRY_INBOUND	0x02		/* flag the entry as coming from a user call */
#define DISPATCHENTRY_WAITING	0x04		/* cleared when an entry is handled */
#define DISPATCHENTRY_NORETURN	0x08		/* tell dispatcher not to return de (returned by waiter) */

/* By default, everything is outbound */
#define OUTBOUND(de)	(((de)->flags & DISPATCHENTRY_INBOUND) == 0)

typedef struct bpdispatchentry {
	BridgePacket				*bp;
	PhidgetNetConnHandle		nc;
	int							forward;
	int							reqseq;
} bpdispatchentry_t;

typedef struct userreqdispatchentry {
	BridgePacket			*bp;
	Phidget_AsyncCallback	cb;
	void					*ctx;
	PhidgetReturnCode		res;
	PhidgetChannelHandle	channel;
} userreqdispatchentry_t;

typedef struct _DispatchEntry {
	dispatchtype_t					type;
	uint8_t							flags;
	uint8_t							len;
	mos_cond_t						cond;
	mos_mutex_t						lock;
	MTAILQ_ENTRY(_DispatchEntry)	link;

#define de_buf	value.buf
#define de_phid value.phid
#define de_device value.device
#define de_channel value.channel
#define de_manager value.manager
#define de_bpe value.dbpe
#define de_bp value.bp
#define de_ureq value.ureq
	union {
		PhidgetHandle			phid;
		PhidgetDeviceHandle		device;
		PhidgetChannelHandle	channel;
		PhidgetManagerHandle	manager;
		BridgePacket			*bp;
		uint8_t					buf[DISPATCHENTRY_MAXDATA];
		bpdispatchentry_t		dbpe;
		userreqdispatchentry_t	ureq;
	} value;
} DispatchEntry, *DispatchEntryHandle;

static MOS_TASK_RESULT PhidgetDispatcher(void *);

typedef MTAILQ_HEAD(dispatchentry_list, _DispatchEntry) dispatchentry_list_t;

mos_fasttlock_t					dispatchLock;
mos_cond_t						dispatchCond;

static dispatchentry_list_t		dispatchEntryList;
static uint32_t					dispatchOutListCount;
static phidgets_t				dispatchOutList;
static uint32_t					dispatchInListCount;
static phidgets_t				dispatchInList;

typedef struct _Dispatch {
	struct dispatchentry_list	list;
	uint16_t					count;
	uint16_t					max;
} Dispatch, *DispatchHandle;

static int initialized;				/* init flag */

static uint32_t entryCount;			/* how many have been allocated */
static uint32_t dispatchers;		/* how many dispatch threads exist */
static uint32_t dispatchersRunning;	/* how many dispatch threads are busy */

static void cleanDispatchEntry(DispatchEntryHandle);

void PhidgetDispatchStop(void);


static DispatchEntryHandle
allocDispatchEntry(void) {
	DispatchEntryHandle de;
	entryCount++;
	incPhidgetStat("dispatch.entries");
	de = mos_zalloc(sizeof(DispatchEntry));
	mos_mutex_init(&de->lock);
	mos_cond_init(&de->cond);

	return (de);
}

static void
freeDispatchEntry(DispatchEntryHandle de) {

	mos_mutex_destroy(&de->lock);
	mos_cond_destroy(&de->cond);

	mos_free(de, sizeof(*de));
	entryCount--;
	decPhidgetStat("dispatch.entries");
}

static void
entryDispatched(void) {
	mos_task_t task;
	int idle;
	int res;

	mos_fasttlock_lock(&dispatchLock);
	idle = dispatchers - dispatchersRunning;
	if (idle < 1 && dispatchers < DISPATCHERS_MAX) {
		logdebug("creating dispatcher");
		res = mos_task_create(&task, PhidgetDispatcher, NULL);
		if (res) {
			logerr("error creating dispatcher: 0x%08x", res);
		} else {
			incPhidgetStat("dispatch.dispatchers");
			incPhidgetStat("dispatch.dispatchers_ever");
			dispatchers++;
			logdebug("created dispatcher - dispatchers: %d", dispatchers);
		}
	}
	mos_cond_signal(&dispatchCond);
	mos_fasttlock_unlock(&dispatchLock);
}

/*
 * Requires dispatchLock
 */
static int
dispatchTaskNeeded(void) {

	if (initialized == 0 && dispatchOutListCount == 0 && dispatchInListCount == 0)
		return (0);

	return (dispatchers - dispatchersRunning <= DISPATCHERS_DESIRED);
}

/*
 * Requires dispatchLock
 */
static PhidgetHandle
getDispatchOutPhidget() {
	PhidgetHandle tphid;
	PhidgetHandle phid;

	/*
	 * Network channels are started before we flag ATTACHED so that events are not lost and
	 * the attach event is queued first.
	 *
	 * Local channels will always be ATTACHED or DETACHING.
	 */
	MTAILQ_FOREACH_SAFE(phid, &dispatchOutList, dispatchOutLink, tphid) {
		if (ISATTACHEDORDETACHING(phid)) {
			MTAILQ_REMOVE(&dispatchOutList, phid, dispatchOutLink);
			dispatchOutListCount--;
			return (phid); // reference assigned to the caller
		} else if (!ISOPEN(phid)) {
			MTAILQ_REMOVE(&dispatchOutList, phid, dispatchOutLink);
			dispatchOutListCount--;
			// Flag must be cleared here because we don't return the phid to the Dispatcher
			PhidgetCLRFlags(phid, PHIDGET_DISPATCHOUT_FLAG);
		}
	}

	return (NULL);
}

/*
 * Requires dispatchLock
 */
static PhidgetHandle
getDispatchInPhidget() {
	PhidgetHandle phid;

	phid = MTAILQ_FIRST(&dispatchInList);
	if (phid == NULL)
		return (NULL);

	MTAILQ_REMOVE(&dispatchInList, phid, dispatchInLink);
	dispatchInListCount--;
	return (phid); // reference assigned to the caller
}

void PhidgetDispatchInit(void);

void
PhidgetDispatchInit(void) {

	mos_glock((void *)0);
	if (initialized) {
		mos_gunlock((void *)0);
		return;
	}
	initialized = 1;
	mos_gunlock((void *)0);

	mos_fasttlock_init(&dispatchLock, P22LOCK_DISPATCHLOCK, P22LOCK_FLAGS);
	mos_cond_init(&dispatchCond);

	MTAILQ_INIT(&dispatchEntryList);
	entryCount = 0;

	MTAILQ_INIT(&dispatchOutList);
	dispatchOutListCount = 0;

	MTAILQ_INIT(&dispatchInList);
	dispatchInListCount = 0;

	dispatchers = 0;
	dispatchersRunning = 0;

	setPhidgetStat("dispatch.max_dispatchers", DISPATCHERS_MAX);
	setPhidgetStat("dispatch.desired_idle_dispatchers", DISPATCHERS_DESIRED_IDLE);
	setPhidgetStat("dispatch.max_entries", DISPATCHENTRY_MAX);
	setPhidgetStat("dispatch.desired_entries", DISPATCHENTRY_DESIRED);
}

void PhidgetDispatchFini(void);

void
PhidgetDispatchFini(void) {
	DispatchEntryHandle de, de2;

	mos_glock((void *)0);
	if (initialized == 0) {
		mos_gunlock((void *)0);
		return;
	}

	initialized = 2;	/* stopping: for asserts */
	PhidgetDispatchStop();
	initialized = 0;

	mos_fasttlock_lock(&dispatchLock);

	de = MTAILQ_FIRST(&dispatchEntryList);
	while (de != NULL) {
		de2 = MTAILQ_NEXT(de, link);
		freeDispatchEntry(de);
		de = de2;
	}
	mos_fasttlock_unlock(&dispatchLock);
	mos_fasttlock_destroy(&dispatchLock);
	mos_gunlock((void *)0);

	MOS_ASSERT(entryCount == 0);
	MTAILQ_INIT(&dispatchEntryList);
}

void
PhidgetDispatchStop(void) {
	/* wait for the dispatchers to exit */

	displog("stopping dispatch");
	mos_fasttlock_lock(&dispatchLock);
	initialized = 0;
	while (dispatchers > 0) {
		mos_cond_broadcast(&dispatchCond);
		mos_fasttlock_timedwait(&dispatchCond, &dispatchLock, 1000000000);
	}
	initialized = 1;
	mos_fasttlock_unlock(&dispatchLock);
}

static PhidgetReturnCode
getDispatchEntry(DispatchEntryHandle *de) {

	MOS_ASSERT(initialized == 1);
	mos_fasttlock_lock(&dispatchLock);
	*de = MTAILQ_FIRST(&dispatchEntryList);
	if (*de == NULL) {
		if (entryCount >= DISPATCHENTRY_MAX) {
			logwarn("entry count (%d) >= %d, cannot dispatch", entryCount, DISPATCHENTRY_MAX);
			mos_fasttlock_unlock(&dispatchLock);
			return (EPHIDGET_AGAIN);	/* try again, and hopefully somebody will return one */
		}
		*de = allocDispatchEntry();
	} else {
		MTAILQ_REMOVE(&dispatchEntryList, *de, link);
		(*de)->flags &= ~DISPATCHENTRY_ONLIST;
	}

	(*de)->flags |= DISPATCHENTRY_WAITING; /* cleared when the entry is actually added to a channel's list */
	mos_fasttlock_unlock(&dispatchLock);
	return (EPHIDGET_OK);
}

static void
cleanDispatchEntry(DispatchEntryHandle de) {

	/*
	 * Make sure any referenced objects are freed when the dispatch thread exits.
	 */
	switch (de->type) {
	case DE_SERVERBRIDGEPACKET:
	case DE_CLIENTBRIDGEPACKET:
		if (de->de_bpe.bp)
			destroyBridgePacket(&de->de_bpe.bp);
		if (de->de_bpe.nc)
			PhidgetRelease(&de->de_bpe.nc); /* should already be closed if we hold the last reference */
		break;
	case DE_CHANNEL_ATTACH:
	case DE_CHANNEL_DETACH:
	case DE_CHANNEL_SETSTATUS:
	case DE_CHANNEL_INITEVENTS:
		if (de->de_channel)
			PhidgetRelease(&de->de_channel);
		break;
	case DE_CHANNEL_BRIDGEPKT:
		if (de->de_bpe.bp)
			destroyBridgePacket(&de->de_bpe.bp);
		break;
	case DE_DEVICE_ATTACH:
	case DE_DEVICE_DETACH:
		if (de->de_device)
			PhidgetRelease(&de->de_device);
		break;
	case DE_MGR_ATTACHCH:
	case DE_MGR_DETACHCH:
		if (de->de_channel)
			PhidgetRelease(&de->de_channel);
		break;
	case DE_MGR_ATTACH:
		if (de->de_manager)
			PhidgetRelease(&de->de_manager);
		break;
	case DE_USERREQ:
		if (de->de_ureq.bp)
			destroyBridgePacket(&de->de_ureq.bp);
		if (de->de_ureq.channel)
			PhidgetRelease(&de->de_ureq.channel);
		break;
	case DE_USERREQCALLBACK:
		if (de->de_ureq.channel)
			PhidgetRelease(&de->de_ureq.channel);
		break;
	default:
		MOS_PANIC("dispatch type not cleaned");
	}

	de->type = DE_NOTHING;
	de->flags = 0;
}

static PhidgetReturnCode
returnDispatchEntry(DispatchEntryHandle de) {

	MOS_ASSERT((de->flags & DISPATCHENTRY_NORETURN) == 0);

	if (de->type != DE_NOTHING)
		cleanDispatchEntry(de);
	mos_fasttlock_lock(&dispatchLock);
	if (entryCount >= DISPATCHENTRY_DESIRED) {
		freeDispatchEntry(de);
	} else {
		MTAILQ_INSERT_HEAD(&dispatchEntryList, de, link);
		de->flags |= DISPATCHENTRY_ONLIST;
	}
	mos_fasttlock_unlock(&dispatchLock);

	return (EPHIDGET_OK);
}

static DispatchHandle
getDispatchHandle(PhidgetHandle phid, int out) {
	DispatchHandle dph;

	if (out) {
		if (phid->dispatchOutHandle)
			return (phid->dispatchOutHandle);
		phid->dispatchOutHandle = dph = mos_malloc(sizeof(*dph));
	} else {
		if (phid->dispatchInHandle)
			return (phid->dispatchInHandle);
		phid->dispatchInHandle = dph = mos_malloc(sizeof(*dph));
	}

	MTAILQ_INIT(&dph->list);
	dph->count = 0;
	dph->max = 0;

	return (dph);
}

static void
fireDispatchErrorEvent(PhidgetHandle phid, int hard, int outbound) {

	if (outbound) {
		if (!hard)
			return;
		FIRE_ERROR(phid, EEPHIDGET_DISPATCH, "event queue is full: data rate too fast?");
	} else {
		FIRE_ERROR(phid, EEPHIDGET_DISPATCH, "command queue is full");
	}
}

static void
clearPhidgetDispatchOut(PhidgetHandle phid, int dataOnly) {
	DispatchEntryHandle de;
	DispatchHandle dph;
	int cnt;

	cnt = 0;

	PhidgetLock(phid);
	for (;;) {
		if (phid->dispatchOutHandle != NULL) {
			dph = phid->dispatchOutHandle;
			de = MTAILQ_FIRST(&dph->list);

			// list is empty
			if (de == NULL)
				break;

			MTAILQ_REMOVE(&dph->list, de, link);
			MOS_ASSERT(dph->count > 0);
			dph->count--;
			cnt++;
			PhidgetUnlock(phid);
			returnDispatchEntry(de);
			PhidgetLock(phid);
		}
	}
	PhidgetUnlock(phid);
	loginfo("cleared %d packets", cnt);
}

/*
 * Always returns the dispatch entry: even in the case of error.
 * Callers must realize that the payload cannot be referenced after this call returns.
 */
static PhidgetReturnCode
insertDispatchEntry(PhidgetHandle phid, DispatchEntryHandle de) {
	uint16_t soft, hard;
	DispatchHandle dph;

#ifndef NDEBUG
	/*
	 * If NORETURN is set, WAITING must also be set.
	 */
	if (de->flags & DISPATCHENTRY_NORETURN)
		MOS_ASSERT((de->flags & DISPATCHENTRY_WAITING) == DISPATCHENTRY_WAITING);
#endif

	PhidgetLock(phid);

	/*
	 * Do not allow data events to be dispatched after we begin detaching the channel.
	 * This is done so that upcalls are made with the ATTACHED flag set which allows property
	 * gets to succeed.
	 */
	if (de->type > DE_NOT_DETACHING && (_ISDETACHING(phid) || !_ISATTACHEDORATTACHING(phid))) {
		PhidgetUnlock(phid);
		logwarn("%"PRIphid": dropping dispatch as channel is not attached or detaching", phid);
		clearPhidgetDispatchOut(phid, 1);
		returnDispatchEntry(de); /* releases the reference passed to us */
		return (EPHIDGET_NOTATTACHED);
	}

	if (OUTBOUND(de)) {
		soft = DISPATCHENTRY_PHID_OSOFTMAX;
		hard = DISPATCHENTRY_PHID_OHARDMAX;
	} else {
		soft = hard = DISPATCHENTRY_PHID_IMAX;
	}
	dph = getDispatchHandle(phid, OUTBOUND(de));

	/*
	 * At the soft limit, only allow some event types to be thrown away
	 */
	if (de->type > DE_THROW_AWAY && dph->count >= soft) {
		PhidgetUnlock(phid);
		fireDispatchErrorEvent(phid, 0, OUTBOUND(de));
		logwarn("%"PRIphid": too many dispatch entries queued (soft):%d dropping entry (type=%d)", phid,
		  dph->count, de->type);
		returnDispatchEntry(de);
		/*
		 * If this happens on the server while trying to send to a client, clear the dispatch
		 * queue as we are falling behind and this will not get better..
		 */
		if (PhidgetCKFlags(phid, PHIDGET_OPENBYNETCLIENT_FLAG)) {
			logwarn("throwing away outbound data events");
			clearPhidgetDispatchOut(phid, 1);
		}
		return (EPHIDGET_NOSPC);
	}

	/*
	 * At the hard limit we assume something is wrong, so stop queuing entries.
	 */
	if (dph->count > hard) {
		PhidgetUnlock(phid);
		fireDispatchErrorEvent(phid, 1, OUTBOUND(de));
		logwarn("%"PRIphid": too many dispatch entries queued (hard):%d dropping entry (type=%d)", phid,
		  dph->count, de->type);
		returnDispatchEntry(de);
		return (EPHIDGET_NOSPC);
	}

	MTAILQ_INSERT_TAIL(&dph->list, de, link);
	dph->count++;
	if (dph->count > dph->max)
		dph->max = dph->count;

	PhidgetUnlock(phid);

	if (OUTBOUND(de)) {
		if (PhidgetCKandSetFlags(phid, PHIDGET_DISPATCHOUT_FLAG) == EPHIDGET_OK) {
			PhidgetRetain(phid);
			mos_fasttlock_lock(&dispatchLock);
			MTAILQ_INSERT_TAIL(&dispatchOutList, phid, dispatchOutLink);
			dispatchOutListCount++;
			mos_fasttlock_unlock(&dispatchLock);
		}
	} else {
		if (PhidgetCKandSetFlags(phid, PHIDGET_DISPATCHIN_FLAG) == EPHIDGET_OK) {
			PhidgetRetain(phid);
			mos_fasttlock_lock(&dispatchLock);
			MTAILQ_INSERT_TAIL(&dispatchInList, phid, dispatchInLink);
			dispatchInListCount++;
			mos_fasttlock_unlock(&dispatchLock);
		}
	}

	entryDispatched();

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
dispatchUserRequestCallback(PhidgetChannelHandle ch, Phidget_AsyncCallback cb, void *ctx,
  PhidgetReturnCode ret) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	MOS_ASSERT(cb != NULL);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_USERREQCALLBACK;
	PhidgetRetain(ch);
	de->de_ureq.channel = ch;
	de->de_ureq.cb = cb;
	de->de_ureq.ctx = ctx;
	de->de_ureq.res = ret;

	return (insertDispatchEntry((PhidgetHandle)ch, de));
}

PhidgetReturnCode
dispatchUserRequest(PhidgetChannelHandle ch, BridgePacket *bp, Phidget_AsyncCallback cb, void *ctx) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;
	mostime_t tm;

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_USERREQ;
	de->flags |= DISPATCHENTRY_INBOUND;
	PhidgetRetain(ch);
	de->de_ureq.channel = ch;
	de->de_ureq.bp = bp;
	de->de_ureq.cb = cb;
	de->de_ureq.ctx = ctx;

	/*
	 * Flag that we are waiting for the dispatch if a callback wasn't provided.
	 * NORETURN tells the dispather not to return the de as we will after WAITING is cleared.
	 */
	if (cb == NULL)
		de->flags |= DISPATCHENTRY_WAITING | DISPATCHENTRY_NORETURN;

	/*
	 * In the failure case, the entry is freed by insert.
	 */
	res = insertDispatchEntry((PhidgetHandle)ch, de);
	if (res != EPHIDGET_OK)
		return (res);

	if (cb == NULL) {
		tm = mos_gettime_usec();
		mos_mutex_lock(&de->lock);
		while (de->flags & DISPATCHENTRY_WAITING) {
			mos_cond_timedwait(&de->cond, &de->lock, MOS_SEC /* nsec */);
			/*
			 * A timeout should not occur, but if it does de will probably get leaked.
			 * The only way this can happen is if the entry has not been dispatched for some reason (bug),
			 * or the actual command really did timeout.  If it did timeout (bug), the dispatcher will
			 * probably not return the de as we flagged NORETURN.
			 */
			if (mos_gettime_usec() - tm > 60000000) {
				de->flags &= ~DISPATCHENTRY_NORETURN; /* probably won't help, but do it anyway */
				mos_mutex_unlock(&de->lock);
				/*
				 * The dispatch entry might get leaked, but there is nothing we can do about it
				 * as something is clearly broken.
				 */
				return (EPHIDGET_TIMEOUT);
			}
		}
		mos_mutex_unlock(&de->lock);

		/*
		 * Once WAITING is cleared, the dispatcher should never touch the de again.  We need to
		 * clear the flag and return it ourself.
		 */
		de->flags &= ~DISPATCHENTRY_NORETURN;
		res = de->de_ureq.res;
		returnDispatchEntry(de);

		return (res);
	}

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
dispatchChannelAttach(PhidgetChannelHandle channel) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	TESTPTR(channel);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_CHANNEL_ATTACH;
	de->de_channel = channel;
	PhidgetRetain(channel);
	return (insertDispatchEntry((PhidgetHandle)channel, de));
}

static PhidgetReturnCode
dispatchChannelInitialEvents(PhidgetChannelHandle channel) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	TESTPTR(channel);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_CHANNEL_INITEVENTS;
	de->de_channel = channel;
	PhidgetRetain(channel);
	return (insertDispatchEntry((PhidgetHandle)channel, de));
}

PhidgetReturnCode dispatchChannelSetStatus(PhidgetChannelHandle);
PhidgetReturnCode
dispatchChannelSetStatus(PhidgetChannelHandle channel) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	TESTPTR(channel);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_CHANNEL_SETSTATUS;
	de->de_channel = channel;
	PhidgetRetain(channel);
	return (insertDispatchEntry((PhidgetHandle)channel, de));
}

static PhidgetReturnCode
dispatchChannelDetach(PhidgetChannelHandle channel) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	TESTPTR(channel);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_CHANNEL_DETACH;
	de->de_channel = channel;
	PhidgetRetain(channel);
	return (insertDispatchEntry((PhidgetHandle)channel, de));
}

/*
 * Always responsible for destroying the bridge packet: even in the case of error.
 */
PhidgetReturnCode
dispatchChannelBridgePacket(PhidgetChannelHandle channel, BridgePacket *bp) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	TESTPTR(channel);
	TESTPTR(bp);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK) {
		destroyBridgePacket(&bp);
		return (res);
	}

	de->type = DE_CHANNEL_BRIDGEPKT;
	de->de_bp = bp;
	return (insertDispatchEntry((PhidgetHandle)channel, de));
}

static PhidgetReturnCode
dispatchDeviceAttach(PhidgetDeviceHandle device) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	TESTPTR(device);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_DEVICE_ATTACH;
	de->de_device = device;
	PhidgetRetain(device);
	return (insertDispatchEntry((PhidgetHandle)device, de));
}

static PhidgetReturnCode
dispatchDeviceDetach(PhidgetDeviceHandle device) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	TESTPTR(device);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_DEVICE_DETACH;
	de->de_device = device;
	PhidgetRetain(device);
	return (insertDispatchEntry((PhidgetHandle)device, de));
}

static PhidgetReturnCode
dispatchManagerAttach(PhidgetManagerHandle manager) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	TESTPTR(manager);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_MGR_ATTACH;
	de->de_manager = manager;
	PhidgetRetain(manager);
	return (insertDispatchEntry((PhidgetHandle)manager, de));
}

PhidgetReturnCode dispatchManagerAttachChannel(PhidgetManagerHandle manager, PhidgetChannelHandle channel);

PhidgetReturnCode
dispatchManagerAttachChannel(PhidgetManagerHandle manager, PhidgetChannelHandle channel) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	TESTPTR(manager);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_MGR_ATTACHCH;
	de->de_channel = channel;
	PhidgetRetain(channel);
	return (insertDispatchEntry((PhidgetHandle)manager, de));
}

PhidgetReturnCode dispatchManagerDetachChannel(PhidgetManagerHandle manager, PhidgetChannelHandle channel);

PhidgetReturnCode
dispatchManagerDetachChannel(PhidgetManagerHandle manager, PhidgetChannelHandle channel) {
	DispatchEntryHandle de;
	PhidgetReturnCode res;

	TESTPTR(manager);

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK)
		return (res);

	de->type = DE_MGR_DETACHCH;
	de->de_channel = channel;
	PhidgetRetain(channel);
	return (insertDispatchEntry((PhidgetHandle)manager, de));
}

/*
 * Always destroys the bridge packet: even in the case of error.
 */
static PhidgetReturnCode
_dispatchBridgePacket(mosiop_t iop, PhidgetNetConnHandle nc, int server, BridgePacket *bp, int forward,
  int reqseq) {
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle phid;
	DispatchEntryHandle de;
	PhidgetReturnCode res;
	uint64_t ocid;
	uint64_t pid;
	int chidx;

	TESTPTR(nc);

	chidx = bridgePacketGetChannelIndex(bp);
	ocid = bridgePacketGetOpenChannelId(bp);
	pid = bridgePacketGetPhidId(bp);

	logverbose("pid:%"PRIu64" chidx:%d", pid, chidx);

	if (chidx < 0 || chidx >= PHIDGET_MAXCHANNELS) {
		destroyBridgePacket(&bp);
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "invalid channel index:%d", chidx));
	}

	res = getDispatchEntry(&de);
	if (res != EPHIDGET_OK) {
		destroyBridgePacket(&bp);
		return (MOS_ERROR(iop, res, "failed to get dispatch entry"));
	}

	/*
	 * On the client, we should have been given the open channel id.
	 * On the server, we use the device and channel index.
	 */
	if (ocid != 0) {
		channel = getChannelById(ocid);
		if (channel == NULL) {
			destroyBridgePacket(&bp);
			returnDispatchEntry(de);
			return (MOS_ERROR(iop, EPHIDGET_NOENT, "no such channel:%llu", ocid));
		}
		assert(isNetworkPhidget(channel));
	} else {
		phid = getDeviceById(pid);
		if (phid == NULL) {
			destroyBridgePacket(&bp);
			returnDispatchEntry(de);
			return (MOS_ERROR(iop, EPHIDGET_NOENT, "no such device in device list: 0x%llx", pid));
		}

		channel = getChannel(phid, chidx);
		PhidgetRelease(&phid);

		if (channel == NULL) {
			destroyBridgePacket(&bp);
			returnDispatchEntry(de);
			return (MOS_ERROR(iop, EPHIDGET_NOENT, "no channel at index:%d", chidx));
		}
		assert(!isNetworkPhidget(channel));
	}

	if (server)
		de->type = DE_SERVERBRIDGEPACKET;
	else
		de->type = DE_CLIENTBRIDGEPACKET;

	PhidgetRetain(nc);

	if (server)
		de->flags |= DISPATCHENTRY_INBOUND;

	de->de_bpe.bp = bp;
	de->de_bpe.nc = nc;
	de->de_bpe.forward = forward;
	de->de_bpe.reqseq = reqseq;

	/*
	 * This may fail, but the normal cause is that we have hit a limit or are detaching.
	 * Either way, there is nothing we can do about it, and it will be logged.
	 */
	res = insertDispatchEntry((PhidgetHandle)channel, de);
	PhidgetRelease(&channel);

	if (res != EPHIDGET_OK)
		return (MOS_ERROR(iop, res, "failed to insert dispatch entry"));

	return (res);
}

PhidgetReturnCode
dispatchServerBridgePacket(mosiop_t iop, PhidgetNetConnHandle nc, BridgePacket *bp, int forward, int reqseq) {

	return (_dispatchBridgePacket(iop, nc, 1, bp, forward, reqseq));
}

PhidgetReturnCode
dispatchClientBridgePacket(mosiop_t iop, PhidgetNetConnHandle nc, BridgePacket *bp, int forward, int reqseq) {

	return (_dispatchBridgePacket(iop, nc, 0, bp, forward, reqseq));
}

static void
dispatchEntry(PhidgetHandle phid, DispatchEntryHandle de) {
	PhidgetNetworkConnectionHandle netConn;
	PhidgetChannelNetConnHandle cnc;
	PhidgetManagerHandle manager;
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle device;
	PhidgetReturnCode res;
	BridgePacket *bp;
	uint16_t repseq;

	displog("0x%x", de->type);

	device = PhidgetDeviceCast(phid);
	if (device) {
		switch (de->type) {
		case DE_DEVICE_ATTACH:
			walkDeviceChannels(device, sendAttachVisitor, NULL);
			break;
		case DE_DEVICE_DETACH:
			walkDeviceChannels(device, sendDetachVisitor, NULL);
			break;
		default:
			MOS_PANIC("invalid dispatch entry for device (%"PRIphid") %x", device, de->type);
		}
		return;
	}

	channel = PhidgetChannelCast(phid);
	if (channel) {
		switch (de->type) {
		case DE_CHANNEL_INITEVENTS:
			if (channel->fireInitialEvents)
				channel->fireInitialEvents(channel);
			break;
		case DE_CHANNEL_SETSTATUS:
			mos_mutex_lock(&channel->netconnslk);
			MTAILQ_FOREACH(cnc, &channel->netconns, link) {
				/*
				 * The status reply id was assigned when that channels was attached to the netconn.
				 * We set it to 0 after we dispatch it.
				 */
				if (cnc->setstatusrep == 0)
					continue;

				repseq = cnc->setstatusrep;
				cnc->setstatusrep = 0;

				res = channel->getStatus(channel, &bp);
				if (res != EPHIDGET_OK) {
					mos_mutex_unlock(&channel->netconnslk);
					logerr("%"PRIphid": failed to get status", channel);
					return;
				}
				bp->vpkt = BP_SETSTATUS;	/* Should already be this, but be pedantic */
				bridgePacketSetIsReply(bp, repseq);

				bridgePacketSetOpenChannelId(bp, getChannelId(channel));
				bridgePacketSetPhidId(bp, PHIDID(channel)); /* set for web client so they can match the reply */
				res = networkSendBridgePacket(channel, bp, cnc->nc);
				destroyBridgePacket(&bp);
				if (res != EPHIDGET_OK) {
					mos_mutex_unlock(&channel->netconnslk);
					return;
				}
			}
			mos_mutex_unlock(&channel->netconnslk);
			break;

		case DE_CHANNEL_ATTACH:
		{
			// Only for local attach
			if (!isNetworkPhidget(channel)) {
				res = createBridgePacket(&bp, BP_OPENRESET, NULL);
				if (res == EPHIDGET_OK) {
					res = DEVBRIDGEINPUT(channel, bp);
					destroyBridgePacket(&bp);
				}
				if (res != EPHIDGET_OK) {
					logerr("Failed to send BP_OPENRESET to device for %"PRIphid": "PRC_FMT, channel, PRC_ARGS(res));
					goto attach_error;
				}

				res = channel->initAfterOpen(channel);
				if (res != EPHIDGET_OK) {
					logerr("Channel Initialization failed for %"PRIphid": "PRC_FMT, channel, PRC_ARGS(res));
					goto attach_error;
				}

				res = channel->setDefaults(channel);
				if (res != EPHIDGET_OK) {
					logerr("failed to set defaults for %"PRIphid": "PRC_FMT, channel, PRC_ARGS(res));
					goto attach_error;
				}

				if (channel->hasInitialState(channel)) {
					PhidgetSetFlags(channel, PHIDGET_HASINITIALSTATE_FLAG);
				}
			}

			if (channel->Attach)
				channel->Attach((PhidgetHandle)channel, channel->AttachCtx);

			// Only for local attach
			if (!isNetworkPhidget(channel)) {
				res = createBridgePacket(&bp, BP_ENABLE, NULL);
				if (res == EPHIDGET_OK) {
					res = DEVBRIDGEINPUT(channel, bp);
					destroyBridgePacket(&bp);
				}
				if (res != EPHIDGET_OK) {
					logerr("Failed to send BP_ENABLE to device for %"PRIphid": "PRC_FMT, channel, PRC_ARGS(res));
					if (channel->Detach) {
						PhidgetSetFlags(channel, PHIDGET_DETACHING_FLAG);
						PhidgetCLRFlags(channel, PHIDGET_ATTACHED_FLAG);
						channel->Detach((PhidgetHandle)channel, channel->DetachCtx);
						PhidgetCLRFlags(channel, PHIDGET_DETACHING_FLAG);
					}
					goto attach_error;
				}
			}

			// Signals that attach is finished. openWaitForAttachment can return
			PhidgetCLRFlags(channel, PHIDGET_ATTACHING_FLAG);
			break;

		attach_error:
			// We don't want to process anything else in the dispatch queue after the attach fails
			clearPhidgetDispatch(phid);
			PhidgetCLRFlags(channel, PHIDGET_ATTACHED_FLAG | PHIDGET_ATTACHING_FLAG);
			device = getParent(channel);
			if (device) {
				closeDevice(device, PFALSE);
				setChannel(device, channel->uniqueIndex, NULL);
				PhidgetRelease(&device);
			}
			break;
		}
		case DE_CHANNEL_DETACH:
			PhidgetCLRFlags(channel, PHIDGET_ATTACHED_FLAG);
			if (channel->Detach)
				channel->Detach((PhidgetHandle)channel, channel->DetachCtx);
			PhidgetCLRFlags(channel, PHIDGET_DETACHING_FLAG | PHIDGET_NETWORK_FLAG);
			break;
		case DE_USERREQ:
			res = PhidgetChannel_bridgeInput(channel, de->de_ureq.bp);
			if (res == EPHIDGET_OK) {
				if (isNetworkPhidget(channel)) {
					netConn = PhidgetNetworkConnectionCast(getPhidgetConnection(channel));
					MOS_ASSERT(netConn != NULL);
					res = networkSendBridgePacket(channel, de->de_ureq.bp, netConn->nc);
					PhidgetRelease(&netConn);
				}
			}
			if (de->de_ureq.cb) {
				// Don't just call the callback here - dispatch so it's called from the dispatch out context along with all other events.
				res = dispatchUserRequestCallback(channel, de->de_ureq.cb, de->de_ureq.ctx, res);
				// Not great - but call the callback directly if the insert fails
				if (res != EPHIDGET_OK)
					de->de_ureq.cb((PhidgetHandle)channel, de->de_ureq.ctx, res);
			} else {
				/*
				 * Once we clear WAITING and unlock, the de cannot be touched by this thread again.
				 */
				mos_mutex_lock(&de->lock);
				de->de_ureq.res = res;
				de->flags &= ~DISPATCHENTRY_WAITING;
				mos_cond_broadcast(&de->cond);
				mos_mutex_unlock(&de->lock);
			}
			break;
		case DE_USERREQCALLBACK:
			de->de_ureq.cb((PhidgetHandle)channel, de->de_ureq.ctx, de->de_ureq.res);
			break;
		case DE_CHANNEL_BRIDGEPKT:
			/*
			 * We allow the device to change the values in the bridge packet, so the packet must be
			 * delivered to the channel first (who will deliver to the device before storing its values),
			 * and finally we will forward across the network.
			 *
			 * Example: we round the device data interval sets to the interrupt rate.
			 */
			if (PhidgetChannel_bridgeInput(channel, de->de_bp) == EPHIDGET_OK)
				bridgeSendBPToNetworkChannelsNoWait(channel, de->de_bp);
			break;
		case DE_CLIENTBRIDGEPACKET:
			if (bridgePacketIsEvent(de->de_bpe.bp)) {
				res = channelDeliverBridgePacket(channel, de->de_bpe.bp, de->de_bpe.nc, de->de_bpe.forward);
			} else {
				/*
				 * This packet came from the server as a result of another client's set.  We do not care
				 * about the result code as the service isn't going to wait to hear about it.
				 */
				bridgePacketSetIsEvent(de->de_bpe.bp);	// never wait as we do not care about the result
				res = channelDeliverBridgePacket(channel, de->de_bpe.bp, de->de_bpe.nc, de->de_bpe.forward);

				// XXX - what about not-string replies?
				const char *reply = NULL;
				if (de->de_bpe.bp->reply_bpe && de->de_bpe.bp->reply_bpe->type == BPE_STR)
					reply = de->de_bpe.bp->reply_bpe->val.str;

				sendSimpleReply(de->de_bpe.nc, de->de_bpe.reqseq, res, reply);
			}
			break;
		case DE_SERVERBRIDGEPACKET:
			if (bridgePacketIsEvent(de->de_bpe.bp)) {
				res = channelDeliverBridgePacket(channel, de->de_bpe.bp, de->de_bpe.nc, de->de_bpe.forward);
			} else {
				mosiop_t iop;
				/*
				 * This is a client command, being procedded by the server-side.
				 * Pass any error details back to the client
				 */
				MOS_ASSERT(de->de_bpe.bp->iop == NULL);

				iop = mos_iop_alloc();
				// Give the bridgepacket it's own ref to the iop. It will free in destroyBridgePacket
				mos_iop_retain(iop);
				de->de_bpe.bp->iop = iop;

				res = channelDeliverBridgePacket(channel, de->de_bpe.bp, de->de_bpe.nc, de->de_bpe.forward);
				if (res != EPHIDGET_OK) {
					char *err;

					MOS_ASSERT(de->de_bpe.bp->reply_bpe == NULL);
					err = mos_malloc(4096);
					mos_snprintf(err, 4096, "%#N", de->de_bpe.bp->iop);
					// Trim trailing newline
					if (err[mos_strlen(err) - 1] == '\n')
						err[mos_strlen(err) - 1] = '\0';
					if (mos_strlen(err) > 0)
						de->de_bpe.bp->reply_bpe = bridgeCreateReplyBPEfromString(mos_strdup(err, NULL));
					mos_free(err, 4096);
				}
				mos_iop_release(&iop);

				if (de->de_bpe.bp->reply_bpe && de->de_bpe.bp->reply_bpe->type == BPE_STR)
					sendSimpleReply(de->de_bpe.nc, de->de_bpe.reqseq, res, de->de_bpe.bp->reply_bpe->val.str);
				else if (de->de_bpe.bp->reply_bpe && de->de_bpe.bp->reply_bpe->type == BPE_UI8ARRAY)
					sendReply(de->de_bpe.nc, de->de_bpe.reqseq, res, de->de_bpe.bp->reply_bpe->bpe_ui8array, de->de_bpe.bp->reply_bpe->len);
				else
					sendSimpleReply(de->de_bpe.nc, de->de_bpe.reqseq, res, NULL);
			}
			break;
		default:
			logerr("invalid dispatch entry type for (%"PRIphid"): %x", phid, de->type);
		}

		return;
	}

	manager = PhidgetManagerCast(phid);
	if (manager) {
		switch (de->type) {
		case DE_MGR_ATTACHCH:
			if (PhidgetCKFlags(manager, PHIDGET_OPEN_FLAG) != 0 && manager->onAttach)
				manager->onAttach(manager, manager->onAttachCtx, de->de_phid);
			break;
		case DE_MGR_DETACHCH:
			if (PhidgetCKFlags(manager, PHIDGET_OPEN_FLAG) != 0 && manager->onDetach)
				manager->onDetach(manager, manager->onDetachCtx, de->de_phid);
			break;
		case DE_MGR_ATTACH:
			PhidgetReadLockDevices();
			FOREACH_DEVICE(device)
				walkDeviceChannels(device, sendAttachVisitor, manager);
			PhidgetUnlockDevices();
			break;
		default:
			logerr("invalid dispatch entry type for (%"PRIphid"): %x", phid, de->type);
		}
		return;
	}

	MOS_PANIC("Not a supported phidget handle");
}

static MOS_TASK_RESULT
PhidgetDispatcher(void *arg) {
	DispatchEntryHandle de;
	PhidgetHandle phid;
	DispatchHandle dph;
	int returnde;
	int out;

	mos_task_setname("Phidget22 Dispatcher Thread");
	logdebug("dispatcher thread started: 0x%08x", mos_self());

	mos_fasttlock_lock(&dispatchLock);
	while (dispatchTaskNeeded()) {

		/*
		 * Threads will handle dispatches from the user first, and to the user second.
		 */
		out = 0;
		phid = getDispatchInPhidget();
		if (phid == NULL) {
			phid = getDispatchOutPhidget();
			if (phid == NULL) {
				mos_fasttlock_wait(&dispatchCond, &dispatchLock);
				continue;
			}

			out = 1;	/* handle the outbound queue */
		}

		dispatchersRunning++;
		mos_fasttlock_unlock(&dispatchLock);

		incPhidgetStat("dispatch.dispatchers_running");

		displog("+dispatching %"PRIphid"", phid);
		PhidgetLock(phid);

		for (;;) {
			if (out) {
				if (!(phid->__flags & PHIDGET_ATTACHED_FLAG) && !(phid->__flags & PHIDGET_DETACHING_FLAG)) {
					break;
				}
			} else {
				if (!(phid->__flags & PHIDGET_OPEN_FLAG))
					break;
			}

			dph = getDispatchHandle(phid, out);
			de = MTAILQ_FIRST(&dph->list);
			if (de == NULL)
				break;
			MTAILQ_REMOVE(&dph->list, de, link);
			dph->count--;

			/*
			 * If NORETURN is flagged, the thread that dispatched the de will return it after
			 * WAITING has been cleared.  It is not safe for us to touch the de after WAITING is
			 * cleared.
			 */
			mos_mutex_lock(&de->lock);
			returnde = (de->flags & DISPATCHENTRY_NORETURN) == 0;
			mos_mutex_unlock(&de->lock);

			phid->dispatchThread = mos_self();

			PhidgetBroadcast(phid);
			PhidgetUnlock(phid);
			dispatchEntry(phid, de);
			if (returnde)
				returnDispatchEntry(de);
			PhidgetLock(phid);
		}

		/* clear dispatch flag so it will be added to list */
		if (out)
			phid->__flags &= ~PHIDGET_DISPATCHOUT_FLAG;
		else
			phid->__flags &= ~PHIDGET_DISPATCHIN_FLAG;

		PhidgetBroadcast(phid);
		PhidgetUnlock(phid);
		displog("-dispatching %"PRIphid"", phid);
		PhidgetRelease(&phid);

		decPhidgetStat("dispatch.dispatchers_running");
		mos_fasttlock_lock(&dispatchLock);
		dispatchersRunning--;
	}

	dispatchers--;
	logdebug("dispatcher thread exiting: 0x%08x - dispatchers: %d", mos_self(), dispatchers);
	decPhidgetStat("dispatch.dispatchers");

	mos_cond_signal(&dispatchCond);
	mos_fasttlock_unlock(&dispatchLock);

	MOS_TASK_EXIT(0);
}

void
startDispatch(PhidgetHandle phid) {
	PhidgetManagerHandle manager;
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle device;

	displog("start %"PRIphid"", phid);

	/*
	 * For devices, we notify each manager of the new device channels
	 */
	device = PhidgetDeviceCast(phid);
	if (device) {
		dispatchDeviceAttach(device);
		return;
	}

	/*
	 * For channels, fire the attach first.
	 */
	channel = PhidgetChannelCast(phid);
	if (channel) {
		dispatchChannelAttach(channel);			/* Only does anything on the client side */
		dispatchChannelSetStatus(channel);		/* Only does anything on the server side */
		dispatchChannelInitialEvents(channel);	/* Only does anything on the client side */
		return;
	}

	/*
	 * For managers, send the initial attach events for each device.
	 */
	manager = PhidgetManagerCast(phid);
	if (manager) {
		dispatchManagerAttach(manager);
		return;
	}
}

void
stopDispatch(PhidgetHandle phid, int sendEvents) {
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle device;

	displog("stop %"PRIphid"", phid);

	clearPhidgetDispatch(phid);

	if (sendEvents == 0)
		return;

	/*
	 * For devices, we notify each manager of the detaching channels
	 */
	device = PhidgetDeviceCast(phid);
	if (device) {
		dispatchDeviceDetach(device);
		return;
	}

	/*
	 * For channels, fire the detach last.
	 */
	channel = PhidgetChannelCast(phid);
	if (channel) {
		dispatchChannelDetach(channel);
		return;
	}
}

void
wakeDispatch(void) {

	mos_fasttlock_lock(&dispatchLock);
	mos_cond_signal(&dispatchCond);
	mos_fasttlock_unlock(&dispatchLock);
}

void
clearPhidgetDispatch(PhidgetHandle phid) {
	PhidgetChannelHandle channel;
	DispatchEntryHandle de;
	DispatchHandle dph;

	PhidgetLock(phid);

	channel = PhidgetChannelCast(phid);

	if (phid->dispatchOutHandle != NULL) {
		dph = phid->dispatchOutHandle;

		for (;;) {
			de = MTAILQ_FIRST(&dph->list);
			if (de == NULL)
				break;
			MTAILQ_REMOVE(&dph->list, de, link);
			MOS_ASSERT(dph->count > 0);
			dph->count--;
			PhidgetUnlock(phid);
			returnDispatchEntry(de);
			PhidgetLock(phid);
		}
	}

	if (phid->dispatchInHandle != NULL) {
		dph = phid->dispatchInHandle;

		for (;;) {
			de = MTAILQ_FIRST(&dph->list);
			if (de == NULL)
				break;
			MTAILQ_REMOVE(&dph->list, de, link);
			MOS_ASSERT(dph->count > 0);
			dph->count--;
			PhidgetUnlock(phid);

			mos_mutex_lock(&de->lock);
			if (de->flags & DISPATCHENTRY_NORETURN) {
				de->flags &= ~DISPATCHENTRY_WAITING;
				de->de_ureq.res = EPHIDGET_NOTATTACHED;
				mos_cond_broadcast(&de->cond);
				mos_mutex_unlock(&de->lock);
				PhidgetLock(phid);
				continue;
			}
			mos_mutex_unlock(&de->lock);

			// If there is a callback for an async user request - call it here
			if (channel != NULL && ((de->type == DE_USERREQ && de->de_ureq.cb) || de->type == DE_USERREQCALLBACK)) {
				de->de_ureq.cb((PhidgetHandle)channel, de->de_ureq.ctx, EPHIDGET_NOTATTACHED);
			}

			returnDispatchEntry(de);
			PhidgetLock(phid);
		}
	}

	PhidgetUnlock(phid);
}

void
freeDispatchHandle(PhidgetHandle phid) {

	PhidgetLock(phid);
	if (phid->dispatchOutHandle) {
		mos_free(phid->dispatchOutHandle, sizeof(Dispatch));
		phid->dispatchOutHandle = NULL;
	}
	if (phid->dispatchInHandle) {
		mos_free(phid->dispatchInHandle, sizeof(Dispatch));
		phid->dispatchInHandle = NULL;
	}
	PhidgetUnlock(phid);
}
