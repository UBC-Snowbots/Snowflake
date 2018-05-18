# - Find dlib
# Find the native dlib includes and library
#
#  DLIB_INCLUDE_DIRS - dlib include directories
#  DLIB_LIBRARIES - dlib libraries

INCLUDE(FindPackageHandleStandardArgs)

set(DLIB_INCLUDE_DIRS ../dlib/include)

SET(DLIB_LibrarySearchPaths
    /usr/local/lib
    /usr/lib
    )

FIND_LIBRARY(DLIB_LIBRARY
    NAMES dlib
    PATHS ${DLIB_LibrarySearchPaths}
    )

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
# The package is found if all variables listed are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(DLIB
    FAIL_MESSAGE "Could NOT find dlib library, is libdlib-dev installed?"
    REQUIRED_VARS DLIB_LIBRARY DLIB_INCLUDE_DIRS
    )

SET(DLIB_LIBRARIES)
LIST(APPEND DLIB_LIBRARIES ${DLIB_LIBRARY})

MARK_AS_ADVANCED(
    DLIB_INCLUDE_DIRS
    DLIB_LIBRARIES
    DLIB_FOUND
)