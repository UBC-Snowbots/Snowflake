AC_INIT([phidget22 C Library],
  m4_esyscmd_s([cat library_version]),
  [support@phidgets.com],
  [libphidget22],
  [www.phidgets.com])
AC_CONFIG_AUX_DIR([build])
AM_INIT_AUTOMAKE([-Wall subdir-objects])
AC_CONFIG_MACRO_DIR([m4])
m4_ifdef([AM_PROG_AR],[AM_PROG_AR])

AC_DEFINE(P22_LIB_VERSION, m4_esyscmd_s([awk '{ print "\""$0"\""}' library_version]))

AC_CANONICAL_HOST
AS_CASE([$host_cpu],
  [x86_64],
	AC_DEFINE_UNQUOTED(_HOST_CPU, [$host_cpu])
	AC_DEFINE_UNQUOTED(_MACHINE, [$host_cpu])
  [
  ],
  [i?86],
  [
	AC_DEFINE_UNQUOTED(_HOST_CPU, [$host_cpu])
	AC_DEFINE(_MACHINE, [i386])
  ],
  [*],
  [
	AC_DEFINE_UNQUOTED(_HOST_CPU, [$host_cpu])
	AC_DEFINE_UNQUOTED(_MACHINE, [$host_cpu])
  ]
)

LT_INIT
AC_CONFIG_SRCDIR([src/phidget22int.h])
m4_ifdef([AM_SILENT_RULES],[AM_SILENT_RULES([yes])])

# Install in /usr/local
AC_PREFIX_DEFAULT([/usr/local])

AC_PROG_CC_C99
if test x$ac_cv_prog_c99 = xno ; then
	AC_MSG_ERROR([A c99 compiler is required])
fi

AC_PROG_LIBTOOL
AC_C_BIGENDIAN(
  [AC_DEFINE(MOS_BYTE_ORDER, MOS_BIG_ENDIAN, [bigendian machine])],
  [AC_DEFINE(MOS_BYTE_ORDER, MOS_LITTLE_ENDIAN, [littleendian machine])],
  [AC_MSG_ERROR(unknown endianess)]
)

AC_ARG_VAR([LDCONFIG], [ldconfig utility])
AC_PATH_PROG([LDCONFIG], [ldconfig], [true], [/sbin /usr/sbin $PATH])

case "$host_os" in
*linux*)
	# Checks for libraries.
	AC_SEARCH_LIBS([dlopen], [dl], [], 
		AC_MSG_ERROR([Missing libdl!]))
	AC_SEARCH_LIBS([sqrt], [m], [], 
		AC_MSG_ERROR([Missing libm!]))
	AC_SEARCH_LIBS([pthread_create], [pthread], 
		[AC_DEFINE(_MOS_PTHREADS, 1)],
		AC_MSG_ERROR([Missing libpthread!]))
	AC_SEARCH_LIBS([libusb_init], [usb-1.0], [],
		AC_MSG_ERROR([Missing libusb!]))
	AC_DEFINE(UNIX, 1)
	AC_DEFINE(_LINUX, 1)
	AC_DEFINE(Linux, 1)
	OS=Linux
	AC_SUBST(OS)
	;;
freebsd*)
	AC_SEARCH_LIBS([sqrt], [m], [], 
		AC_MSG_ERROR([Missing libm!]))
	AC_SEARCH_LIBS([pthread_create], [pthread], 
		[AC_DEFINE(_MOS_PTHREADS, 1)],
		AC_MSG_ERROR([Missing libpthread!]))
	AC_SEARCH_LIBS([libusb_init], [usb], [],
		AC_MSG_ERROR([Missing libusb!]))
	AC_DEFINE(UNIX, 1)
	AC_DEFINE(_FREEBSD, 1)
	AC_DEFINE(FreeBSD, 1)
	OS=FreeBSD
	AC_SUBST(OS)
	;;
*)
	AC_MSG_ERROR(Unrecognized OS $host_os)
	;;
esac

#
# SPI
#
AC_ARG_ENABLE([spi],
  [AS_HELP_STRING([--enable-spi], [Define to compile in SPI Hub support])],,
  [enable_spi=no]
)
AS_IF([test "x$enable_spi" == "xyes"],
  [AC_DEFINE([SPI_SUPPORT], 1, [Define to enable SPI Hub support])]
)
AM_CONDITIONAL([SPI], [test "x$enable_spi" == "xyes"])

#
# Async libusb
#
AC_ARG_ENABLE([libusbasync],
  [AS_HELP_STRING([--disable-libusbasync], [Define to use libusb in synchronous mode])],,
  [enable_libusbasync=yes]
)
AS_IF([test "x$enable_libusbasync" == "xyes"],
  [AC_DEFINE([LIBUSB_ASYNC], 1, [Define to use libusb in Async mode])]
)

#
# DEBUG
#
AC_ARG_ENABLE([debug],
  [AS_HELP_STRING([--enable-debug], [Define to enable debugging])],,
  [enable_debug=no]
)
AS_IF([test "x$enable_debug" == "xyes"], [
  AC_DEFINE([DEBUG], 1, [Define to enable debugging])
  AC_DEFINE([_DEBUG], 1, [Define to enable debugging])
  AC_DEFINE([MOS_TRACK_ALLOCATIONS], 1, [Define to track memory allocations])
  AC_DEFINE([MOS_TRACK_ALLOCATION_DETAILS], 1, [Define to track memory allocation details])
  AC_DEFINE([MOS_DEBUG_LOCKING], 1, [Define to debug lock ordering])
])
AS_IF([test "x$enable_debug" == "xno"], [
  AC_DEFINE([NDEBUG], 1, [Define to disable debugging])
])

#
# Labview
#
AC_ARG_ENABLE([labview],
  [AS_HELP_STRING([--enable-labview], [Define to turn on Labview support])],,
  [enable_labview=no]
)
AS_IF([test "x$enable_labview" == "xyes"], [
  AC_DEFINE([LABVIEW], 1, [Define to support Labview])
])
AM_CONDITIONAL([LABVIEW], [test "x$enable_labview" == "xyes"])

#
# Check for newlocale() et al.
#
AC_MSG_CHECKING(for newlocale in xlocale.h)
AC_TRY_COMPILE([
    #include <xlocale.h>
  ], [newlocale(LC_ALL_MASK, "C", NULL)], use_xlocale=yes, use_xlocale=no)
if test "$use_xlocale" = "yes"; then
  AC_DEFINE(THREAD_LOCALE_XLOCALE_H, 1, [Define to 1 if newlocale() is defined in xlocale.h (e.g. on Mac OS X / BSD)])
  AC_MSG_RESULT(yes)
else
  AC_MSG_RESULT(no)
  AC_MSG_CHECKING(for newlocale in locale.h)
  AC_TRY_COMPILE([
      #define _XOPEN_SOURCE 700
      #define _GNU_SOURCE
      #include <locale.h>
    ], [newlocale(LC_ALL_MASK, "C", NULL)], use_locale=yes, use_locale=no)
  if test "$use_locale" = "yes"; then
    AC_DEFINE(THREAD_LOCALE_LOCALE_H, 1, [Define to 1 if newlocale() is defined in locale.h (e.g. in glibc)])
    AC_MSG_RESULT(yes)
  else
    AC_MSG_WARN([newlocale()/uselocale() not found, network clients may fail in non-C locales])
    AC_MSG_RESULT(no)
  fi
fi

#
# ldconfig
#
AC_ARG_ENABLE([ldconfig],
  [AS_HELP_STRING([--disable-ldconfig], [do not update dynamic linker cache using ldconfig])],,
  [enable_ldconfig=yes]
)
AM_CONDITIONAL([RUN_LDCONFIG], [test "x$enable_ldconfig" = "xyes"])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT([libphidget22.pc])
