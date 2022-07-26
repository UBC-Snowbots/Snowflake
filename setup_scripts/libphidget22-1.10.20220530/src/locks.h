#ifndef _PHIDGET22_LOCKS_H_
#define _PHIDGET22_LOCKS_H_

#include "mos/mos_tlock.h"
#include "mos/mos_assert.h"

#define P22HELD(tl)			mos_tlock_haswriter((tl), mos_self())
#define P22_ASSERTHELD(tl)	MOS_ASSERT(P22HELD((tl)))

#ifdef MOS_DEBUG_LOCKING
#define P22LOCK_FLAGS					MOSLOCK_TRACK
#else
#define P22LOCK_FLAGS					0
#endif

/*
 * Lock ordering is lower to higher.
 * A high numbered lock cannot be held while attempting to aquire a lower lock (per thread).
 */

#define P22LOCK_SRVSZCLOCK				10020
#define P22LOCK_NCTLLOCK				10030	/* network control lock */
#define P22LOCK_NCTLENTLOCK				10031	/* network control entry lock */
#define P22LOCK_SRVSCCLOCK				10040
#define P22LOCK_SRVSLOCK				10041
#define P22LOCK_SRVSIPHIDLOCK			10043
#define P22LOCK_MANAGERLISTLOCK			11001
#define P22LOCK_NETQUEUELISTLOCK		11002
#define P22LOCK_DEVICELISTLOCK			11003
#define P22LOCK_MANAGERRUNLOCK			12051
#define P22LOCK_CHANNELRUNLOCK			12052
#define P22LOCK_DEVICERUNLOCK			12053
#define P22LOCK_CONNECTIONRUNLOCK		12054
#define P22LOCK_DISPATCHLOCK			13000
#define P22LOCK_DEVICEMEMBERLOCK		13053
#define P22LOCK_CONNECTIONLOCK			14004
#define P22LOCK_NETCONNWRLOCK			14050
#define P22LOCK_MANAGERLOCK				14051
#define P22LOCK_CHANNELLOCK				14052
#define P22LOCK_DEVICELOCK				14053
#define P22LOCK_NETCONNLOCK				15044
#define P22LOCK_WFRLOCK					15100
#define P22LOCK_BPLOCK					15200

#endif /* _PHIDGET22_LOCKS_H_ */
