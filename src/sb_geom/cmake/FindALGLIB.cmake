# - Find ALGLIB
# Find the native ALGLIB includes and library
#
#  ALGLIB_INCLUDE_DIRS    - where to find fftw3.h
#  ALGLIB_LIBRARIES   - List of libraries when using FFTW.
#  ALGLIB_FOUND       - True if FFTW found.

if (ALGLIB_INCLUDE_DIRS)
  # Already in cache, be silent
  set (ALGLIB_FIND_QUIETLY TRUE)
endif (ALGLIB_INCLUDE_DIRS)

find_path (ALGLIB_INCLUDE_DIRS
    alglibinternal.h
    alglibmisc.h
    ap.h
    dataanalysis.h
    diffequations.h
    fasttransforms.h
    integration.h
    interpolation.h
    linalg.h
    optimization.h
    solvers.h
    specialfunctions.h
    statistics.h
    stdafx.h
    PATHS
    /usr/include/libalglib/
    )

find_library (ALGLIB_LIBRARIES NAMES alglib)

# handle the QUIETLY and REQUIRED arguments and set ALGLIB_FOUND to TRUE if
# all listed variables are TRUE
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (ALGLIB DEFAULT_MSG ALGLIB_LIBRARIES ALGLIB_INCLUDE_DIRS)

mark_as_advanced (ALGLIB_LIBRARIES ALGLIB_INCLUDE_DIRS)
