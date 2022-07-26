#ifndef _MOS_MACROCOMPAT_H_
#define _MOS_MACROCOMPAT_H_

#if defined(Windows)

#if defined(_KERNEL)
#define	mos_raw_printf				DbgPrint
#define	mos_raw_vprintf(fmt, va)	vDbgPrintEx(DPFLTR_IHVDRIVER_ID, 0, (fmt), \
									  (va))
#define	mos_raw_vsnprintf			_vsnprintf
#else
#define	mos_raw_printf				printf
#define	mos_raw_vprintf				vprintf
#define	mos_raw_vsnprintf			_vsnprintf
#define sleep(x)			(Sleep((x) * 1000))
#endif

#define __func__			__FUNCTION__

#else /* !Windows */

#define mos_raw_printf				printf
#define mos_raw_vprintf				vprintf
#define mos_raw_vsnprintf			vsnprintf

#endif /* Windows */

#endif /* _MOS_MACROCOMPAT_H_ */
