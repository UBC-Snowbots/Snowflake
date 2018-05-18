cmake_minimum_required(VERSION 2.8.3)
project(sb_geom)

# Add our custom cmake files to the `CMAKE_MODULE_PATH` so that we can use them here
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Since this is a highly performance sensitive section of the codebase
# build in "Release" (with lots of compiler optimizations) by default
# (If built in "Debug", some functions can take up to 250x longer)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

add_definitions(-std=gnu++14)

# Make code position independent. This is required to let us link against
# the dlib shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sb_geom_msgs
        )
find_package(ALGLIB REQUIRED)
find_package(GSL REQUIRED)
find_package(DLIB REQUIRED)

# We currently have dlib downloaded because we need a newer version then
# the Ubuntu system version (we need >19.6)
# Eventually we should switch back to using the Ubuntu system version (once
# it's updated)
#add_subdirectory(./dlib dlib_build)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sb_geom
        CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${ALGLIB_INCLUDE_DIRS}
        ${GSL_INCLUDE_DIRS}
        # We have this include because the system version of dlib on Ubuntu
        # is out of date and is missing the "global_optimization" part of the
        # library. Eventually, if the system version of dlib is updated to >19.6,
        # we should be able to remove this line and delete the `dlib` folder.
        # Current system version of dlib in Ubuntu is indicated here:
        #   https://launchpad.net/ubuntu/+source/dlib
        ./dlib
        ./include
)

# Put all the libraries we use into a single variable
set(ALL_LIBRARIES
    ${catkin_LIBRARIES}
    ${ALGLIB_LIBRARIES}
    ${GSL_LIBRARIES}
    ${DLIB_LIBRARIES}
    )

## Declare C++ libraries

# Find all the `.h` and `.cpp` files for the `sb_geom` library
file(GLOB sb_geom_SRCS
        "src/*.cpp"
        # We have this include because the system version of dlib on Ubuntu
        # is out of date and is missing the "global_optimization" part of the
        # library. Eventually, if the system version of dlib is updated to >19.6,
        # we should be able to remove this line and delete the `dlib` folder.
        # Current system version of dlib in Ubuntu is indicated here:
        #   https://launchpad.net/ubuntu/+source/dlib
        "dlib/dlib/global_optimization/*.cpp"
        )
file(GLOB sb_geom_HEADERS
        "include/sb_geom/*.h"
        )
# NOTE: We have to make this library static so we can link against dlib
add_library(sb_geom STATIC
        ${sb_geom_SRCS}
        ${sb_geom_HEADERS}
        )
target_link_libraries(sb_geom ${ALL_LIBRARIES})
add_dependencies(sb_geom ${catkin_EXPORTED_TARGETS})


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    catkin_add_gtest(Polynomial-test
            test/Polynomial-test.cpp
            ${sb_geom_SRCS}
            )
    target_link_libraries(Polynomial-test ${ALL_LIBRARIES})
    add_dependencies(Polynomial-test ${catkin_EXPORTED_TARGETS})

    catkin_add_gtest(Spline-test
            test/Spline-test.cpp
            ${sb_geom_SRCS}
            )
    target_link_libraries(Spline-test ${ALL_LIBRARIES})
    add_dependencies(Spline-test ${catkin_EXPORTED_TARGETS})

    catkin_add_gtest(PolynomialSegment-test
        test/PolynomialSegment-test.cpp
        ${sb_geom_SRCS}
        )
    target_link_libraries(PolynomialSegment-test ${ALL_LIBRARIES})
    add_dependencies(PolynomialSegment-test ${catkin_EXPORTED_TARGETS})

    catkin_add_gtest(utils-test
        test/utils-test.cpp
        ${sb_geom_SRCS}
        )
    target_link_libraries(utils-test ${ALL_LIBRARIES})
    add_dependencies(utils-test ${catkin_EXPORTED_TARGETS})

endif()

