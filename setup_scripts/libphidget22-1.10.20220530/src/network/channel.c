#define _PHIDGET_NETWORKCODE

#include "phidgetbase.h"
#include "phidget.h"
#include "network/network.h"
#include "bridgepackets.gen.h"

PhidgetReturnCode dispatchChannelSetStatus(PhidgetChannelHandle);

PhidgetReturnCode
addChannelNetworkConnection(PhidgetChannelHandle channel, PhidgetNetConnHandle nc, uint16_t reqseq) {
	PhidgetChannelNetConn *cnc;

	mos_mutex_lock(&channel->netconnslk);
	MTAILQ_FOREACH(cnc, &channel->netconns, link) {
		if (cnc->nc == nc) {
			mos_mutex_unlock(&channel->netconnslk);
			return (EPHIDGET_OK);
		}
	}

	cnc = mos_malloc(sizeof(*cnc));
	cnc->nc = nc;
	cnc->setstatusrep = reqseq;
	PhidgetRetain(cnc->nc);
	cnc->nc->openchannels++;
	MTAILQ_INSERT_HEAD(&channel->netconns, cnc, link);
	channel->netconnscnt++;
	mos_mutex_unlock(&channel->netconnslk);

	netloginfo("%"PRIphid" linked to %"PRIphid"", nc, channel);

	return (EPHIDGET_OK);
}

/*
 * Removes a specific network connection from a channel.
 *
 * Connection is closing...
 */
PhidgetReturnCode
removeChannelNetworkConnection(PhidgetChannelHandle channel, PhidgetNetConnHandle nc, int *cnt) {
	PhidgetChannelNetConnHandle cnc;

	mos_mutex_lock(&channel->netconnslk);
	MTAILQ_FOREACH(cnc, &channel->netconns, link) {
		if (cnc->nc == nc) {
			MTAILQ_REMOVE(&channel->netconns, cnc, link);
			netloginfo("%"PRIphid" unlinked from %"PRIphid" (cnt=%d)", cnc->nc, channel, channel->netconnscnt - 1);

			cnc->nc->openchannels--;
			PhidgetRelease(&cnc->nc);
			mos_free(cnc, sizeof(*cnc));
			channel->netconnscnt--;
			MOS_ASSERT(channel->netconnscnt >= 0);
			*cnt = channel->netconnscnt;
			mos_mutex_unlock(&channel->netconnslk);
			return (EPHIDGET_OK);
		}
	}
	mos_mutex_unlock(&channel->netconnslk);
	return (EPHIDGET_NOENT);
}

/*
 * Removes all of the network connections associated with the channel.
 *
 * Channel is going away...
 */
void
removeChannelNetworkConnections(PhidgetChannelHandle channel) {
	PhidgetChannelNetConnHandle cnc1, cnc2;

	mos_mutex_lock(&channel->netconnslk);
	cnc1 = MTAILQ_FIRST(&channel->netconns);
	while (cnc1 != NULL) {
		cnc2 = MTAILQ_NEXT(cnc1, link);

		/*
		 * We are releasing our reference to the connection, not closing the connection.
		 * Just because a channel detaches does not mean the client connection closes.
		 */
		netloginfo("%"PRIphid" unlinked from %"PRIphid"", cnc1->nc, channel);
		PhidgetRelease(&cnc1->nc);
		mos_free(cnc1, sizeof(*cnc1));
		cnc1 = cnc2;
	}
	MTAILQ_INIT(&channel->netconns);
	channel->netconnscnt = 0;
	mos_mutex_unlock(&channel->netconnslk);
}

PhidgetReturnCode
openServerChannel(mosiop_t iop, uint64_t pid, Phidget_ChannelClass chclass, int chidx, PhidgetNetConnHandle nc,
  PhidgetChannelHandle *openChannel, uint16_t reqseq) {
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle device;
	PhidgetReturnCode res;
	int cnt;

	TESTPTR(openChannel);
	TESTPTR(nc);

	if (chidx < 0 || chidx >= PHIDGET_MAXCHANNELS)
		return (MOS_ERROR(iop, EPHIDGET_INVALIDARG, "Invalid channel index. Must be >= 0 and <= %d", PHIDGET_MAXCHANNELS));

	device = getDeviceById(pid);
	if (device == NULL)
		return (MOS_ERROR(iop, EPHIDGET_NOENT, "Specified device ID was not found."));

	/*
	 * If there is already an open channel, and it is not a network channel we are done.
	 * We do not support opening a channel locally as well as over the network.
	 *
	 * There is a race where the channel could get opened while are are creating ours,
	 * but attachChannel() will double check.
	 *
	 * The reference created here will belong to the system.  The system will release the reference when
	 * the client closes the channel, or when the device is detached.  An additional reference is created
	 * and assigned to the caller.  This is done for two reasons 1) because the caller needs a reference
	 * and 2) because the channel could detach while we are attaching it, and get freed out from under us.
	 */
	channel = getChannel(device, chidx);
	if (channel == NULL) {
		res = createTypedPhidgetChannelHandle(&channel, chclass);
		if (res != EPHIDGET_OK) {
			netlogerr("Failed to create channel of class '%x' for %"PRIphid"", chclass, nc);
			MOS_ERROR(iop, res, "Failed to create channel of class '%x' for %"PRIphid"", chclass, nc);
			goto bad;
		}

		if (device->deviceInfo.isHubPort) {
			switch (channel->class) {
			case PHIDCHCLASS_DIGITALINPUT:
				channel->openInfo->hubPortMode = PORT_MODE_DIGITAL_INPUT;
				break;
			case PHIDCHCLASS_DIGITALOUTPUT:
				channel->openInfo->hubPortMode = PORT_MODE_DIGITAL_OUTPUT;
				break;
			case PHIDCHCLASS_VOLTAGEINPUT:
				channel->openInfo->hubPortMode = PORT_MODE_VOLTAGE_INPUT;
				break;
			case PHIDCHCLASS_VOLTAGERATIOINPUT:
				channel->openInfo->hubPortMode = PORT_MODE_VOLTAGE_RATIO_INPUT;
				break;
			default:
				res = EPHIDGET_INVALIDARG;
				MOS_ERROR(iop, res, "Invalid channel class for isHubPort channel.");
				goto bad;
			}
		}
	} else {
		if (PhidgetCKFlags(channel, PHIDGET_OPEN_FLAG) && !PhidgetCKFlags(channel, PHIDGET_OPENBYNETCLIENT_FLAG)) {
			netlogwarn("'%"PRIphid"': %"PRIphid" busy", nc, channel);
			res = EPHIDGET_BUSY;
			MOS_ERROR(iop, res, "Channel is opened locally in the server process. Cannot be opened over network.");
			goto bad;
		}
	}
	PhidgetRetain(channel);	/* create the callers reference */

	if (!allowNetworkAccess(channel, 0)) {
		netloginfo("Access to '%"PRIphid"' by '%"PRIphid"' denied", channel, nc);
		res = EPHIDGET_ACCESS;
		MOS_ERROR(iop, res, "Network access to this channel has been disabled.");
		goto bad;
	}

	res = addChannelNetworkConnection(channel, nc, reqseq);
	if (res != EPHIDGET_OK) {
		netlogerr("failed to add network connection '%"PRIphid"' to device '%"PRIphid"': "PRC_FMT, nc, device, PRC_ARGS(res));
		MOS_ERROR(iop, res, "failed to add network connection '%"PRIphid"' to device '%"PRIphid"': "PRC_FMT, nc, device, PRC_ARGS(res));
		goto bad;
	}

	if (!PhidgetCKFlags(channel, PHIDGET_OPEN_FLAG)) {
		PhidgetSetFlags(channel, PHIDGET_OPEN_FLAG | PHIDGET_OPENBYNETCLIENT_FLAG);
		channel->iop = iop;
		// This will set any errors in the channel iop
		res = attachChannel(device, chidx, channel);
		channel->iop = NULL;
		if (res != EPHIDGET_OK) {
			if (res != EPHIDGET_BUSY) // do not spam the log file
				netlogerr("attachChannel(%"PRIphid") failed for '%"PRIphid"': "PRC_FMT, channel, nc, PRC_ARGS(res));
			removeChannelNetworkConnection(channel, nc, &cnt);
			goto bad;
		}
	} else {
		if (!allowNetworkAccess(channel, 1)) {
			netlogwarn("Network access to channel <%"PRIphid"> by multiple clients not allowed", channel);
			res = EPHIDGET_ACCESS;
			removeChannelNetworkConnection(channel, nc, &cnt);
			MOS_ERROR(iop, res, "Network access to channel <%"PRIphid"> by multiple clients not allowed; this channel is associated with another client.", channel);
			goto bad;
		}
		res = dispatchChannelSetStatus(channel);
		if (res != EPHIDGET_OK) {
			netlogerr("dispatchChannelSetStatus(%"PRIphid") failed for '%"PRIphid"': "PRC_FMT, channel, nc, PRC_ARGS(res));
			removeChannelNetworkConnection(channel, nc, &cnt);
			MOS_ERROR(iop, res, "dispatchChannelSetStatus(%"PRIphid") failed for '%"PRIphid"': "PRC_FMT, channel, nc, PRC_ARGS(res));
			goto bad;
		}
	}

	netloginfo("%"PRIphid" opened %"PRIphid"", nc, channel);

	*openChannel = channel;

	PhidgetRelease(&device);
	return (EPHIDGET_OK);

bad:

	netlogerr("Failed to open channel %x:%d for %"PRIphid"", chclass, chidx, nc);

	if (channel != NULL)
		PhidgetRelease(&channel);

	PhidgetRelease(&device);
	return (res);
}

PhidgetReturnCode
closeServerChannel(uint64_t pid, int chidx, PhidgetNetConnHandle nc) {
	PhidgetChannelHandle channel;
	PhidgetDeviceHandle device;
	PhidgetReturnCode res;
	int cnt;

	if (chidx < 0 || chidx >= PHIDGET_MAXCHANNELS)
		return (EPHIDGET_INVALIDARG);

	device = getDeviceById(pid);
	if (device == NULL)
		return (EPHIDGET_NOENT);

	channel = getAttachedChannel(device, chidx);
	if (channel == NULL) {
		PhidgetRelease(&device);
		return (EPHIDGET_INVALIDARG);
	}

	res = removeChannelNetworkConnection(channel, nc, &cnt);
	if (res != EPHIDGET_OK)
		goto done;

	if (cnt != 0)
		goto done;

	/*
	 * If there are no remaining network references, close the channel.
	 * This will end up releasing the network reference in channelDetach().
	 */
	Phidget_close((PhidgetHandle)channel);

	netloginfo("%"PRIphid" closed %"PRIphid"", nc, channel);

done:

	PhidgetRelease(&device);
	PhidgetRelease(&channel);

	return (res);
}

/*
 * XXX We need to figure out what to do with single errors in these bulk sends..
 * probably something like shutting down the network connection and causing a complete
 * rebuild.
 */
PhidgetReturnCode
sendToNetworkConnections(PhidgetChannelHandle channel, BridgePacket *bp, PhidgetNetConnHandle ignoreNC) {
	PhidgetChannelNetConn *cnc;
	PhidgetReturnCode res;

	res = EPHIDGET_OK;

	bridgePacketSetOpenChannelId(bp, getChannelId(channel));
	bridgePacketSetPhidId(bp, PHIDID(channel)); /* set for web client so they can match the reply */

	mos_mutex_lock(&channel->netconnslk);
	MTAILQ_FOREACH(cnc, &channel->netconns, link) {
		if (cnc->nc == ignoreNC)
			continue;

		if (PhidgetCKFlags(cnc->nc, PNCF_CLOSED) != 0)
			continue;

		/*
		 * Potential optimization would be to only render the json once.
		 */
		res = networkSendBridgePacket(channel, bp, cnc->nc);
		if (res != EPHIDGET_OK)
			break;
	}
	mos_mutex_unlock(&channel->netconnslk);

	return (res);
}

PhidgetReturnCode
channelDeliverBridgePacket(PhidgetChannelHandle channel, BridgePacket *bp, PhidgetNetConnHandle nc,
  int forward) {
	PhidgetReturnCode res;

	TESTPTR(channel);

	res = PhidgetChannel_bridgeInput(channel, bp);
	netlogverbose("bridgeInput(%"PRIphid") [%s]: "PRC_FMT, channel, bridgepacketinfo[bp->vpkt].name, PRC_ARGS(res));
	if (res != EPHIDGET_OK)
		return (res);

	if (bp->vpkt < BRIDGEPACKET_COUNT)
		if (bridgepacketinfo[bp->vpkt].flags & BP_FLAG_NOFORWARD)
			forward = 0;

	if (forward)
		sendToNetworkConnections(channel, bp, nc);

	return (EPHIDGET_OK);
}