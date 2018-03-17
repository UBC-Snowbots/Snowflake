cmake_minimum_required(VERSION 2.8.3)
project(sb_geom)

add_definitions(-std=c++14)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sb_geom_msgs
        ecl_geometry
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
        ${sb_geom_INCLUDE_DIRS}
        ./include
)

## Declare C++ libraries

# Find all the `.h` and `.cpp` files for the `sb_geom` library
file(GLOB sb_geom_SRCS
        "src/*.cpp"
        )
file(GLOB sb_geom_HEADERS
        "include/sb_geom/*.h"
        )
add_library(sb_geom
        ${sb_geom_SRCS}
        ${sb_geom_HEADERS}
        )
add_dependencies(sb_geom ${catkin_EXPORTED_TARGETS})


#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)

    catkin_add_gtest(polyline-test
            test/polyline-test.cpp
            src/PolyLine.cpp
            )
    target_link_libraries(polyline-test ${catkin_LIBRARIES})
    add_dependencies(polyline-test ${catkin_EXPORTED_TARGETS})

endif()

