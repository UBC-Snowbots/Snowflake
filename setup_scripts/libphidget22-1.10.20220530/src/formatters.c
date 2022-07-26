#include "phidgetbase.h"
#include "mos/mos_fmt.h"

static const char *
phidget_format(void *obj, char *fbuf, size_t *blen) {
	PhidgetHandle phid;
	char buf[128];

	if (obj == NULL) {
		mos_strlcpy(fbuf, "(null)", *blen);
		*blen = 6;
		return (fbuf);
	}

	phid = (PhidgetHandle)obj;
	switch (phid->type) {
	default:
		mos_strlcpy(fbuf, "<not a phidget>", *blen);
		break;
	case PHIDGET_BASE:
		mos_strlcpy(fbuf, "<not a device or user channel>", *blen);
		break;
	case PHIDGET_CHANNEL:
		*blen = mos_snprintf(fbuf, *blen, "%s", channelInfo((PhidgetChannelHandle)phid, buf, sizeof(buf)));
		break;
	case PHIDGET_DEVICE:
		*blen = mos_snprintf(fbuf, *blen, "%s", deviceInfo((PhidgetDeviceHandle)phid, buf, sizeof(buf)));
		break;
	case PHIDGET_MANAGER:
		*blen = mos_snprintf(fbuf, *blen, "MANAGER (0x%x)", phid->__flags);
		break;
	case PHIDGET_NETCONN:
		*blen = mos_snprintf(fbuf, *blen, "%s", netConnInfo((PhidgetNetConnHandle)phid, buf, sizeof(buf)));
		break;
	}

	*blen = strlen(fbuf);
	return (fbuf);
}

void FormatInit(void);

void
FormatInit() {

	mos_register_formatter(MOS_IOP_IGNORE, FMT_P, phidget_format);
}
