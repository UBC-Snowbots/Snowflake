cmake_minimum_required(VERSION 2.8.3)
project(sb_geom)

# Add our custom cmake files to the `CMAKE_MODULE_PATH`
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_definitions(-std=c++14)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sb_geom_msgs
        )
find_package(ALGLIB REQUIRED)


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
        test/polynomial-test.cpp
        src/Polynomial.cpp
            )
    target_link_libraries(polyline-test ${catkin_LIBRARIES})
    add_dependencies(polyline-test ${catkin_EXPORTED_TARGETS})

    catkin_add_gtest(splineline-test
            test/splineline-test.cpp
        src/Spline.cpp
            )
    target_link_libraries(splineline-test
        ${catkin_LIBRARIES}
        ${ALGLIB_LIBRARIES}
        )
    add_dependencies(splineline-test ${catkin_EXPORTED_TARGETS})

endif()

