cmake_minimum_required(VERSION 2.8.3)

project(sb_utils)

# Build in "Release" (with lots of compiler optimizations) by default
# (If built in "Debug", some functions can take orders of magnitude longer)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

add_definitions(-std=c++14)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mapping_msgs_urc
)


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
    LIBRARIES sb_utils
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
    ./include
)

## Declare a C++ library
add_library(sb_utils
        include/sb_utils.h
        include/RvizUtils.h
        src/sb_utils.cpp
        src/RvizUtils.cpp
)

## Specify libraries to link a library or executable target against
# target_link_libraries(utils_lib_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(sb_utils ${catkin_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    catkin_add_gtest(rviz_utils_rostest
            test/rviz_utils_rostest.test
            test/rviz_utils_rostest.cpp
            src/RvizUtils.cpp
            include/RvizUtils.h
            )

    target_link_libraries(rviz_utils_rostest ${catkin_LIBRARIES})

endif()
