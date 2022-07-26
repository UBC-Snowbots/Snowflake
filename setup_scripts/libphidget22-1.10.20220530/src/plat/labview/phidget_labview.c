#include "phidgetbase.h"
#include "phidget22int.h"

#ifdef COMPILE_PHIDGETS_LABVIEW

#ifdef _WINDOWS
#include "windows/extcode.h"
#elif _LINUX
#include "extcode.h"
#elif _MACOSX
#include "macos/2010/extcode.h"
#endif
#include "plat/labview/phidget_labview.gen.h"

#else

typedef uint32_t LVUserEventRef;

#endif

#include "plat/labview/phidget_labview.gen.c"
