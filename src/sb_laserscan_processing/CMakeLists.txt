cmake_minimum_required(VERSION 2.8.3)
project(sb_laserscan_processing)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
)
find_package(sb_utils REQUIRED)
find_package(mapping_igvc REQUIRED) #Contains cone obstacle

catkin_package(
        INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ./include
        ${catkin_INCLUDE_DIRS}
        ${sb_utils_INCLUDE_DIRS}
        ${mapping_igvc_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(sample_package_node src/sample_package_node.cpp)
add_executable(sb_laserscan_processing
        src/cone_extractor_node.cpp
        src/ConeExtractorNode.cpp
        include/ConeExtractorNode.h
        )


## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(sb_laserscan_processing
        ${catkin_LIBRARIES}
        ${sb_utils_LIBRARIES}
        ${mapping_igvc_LIBRARIES}
        )


#############
## Testing ##
#############

#if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    #catkin_add_gtest(laserscan-cone-manager-test test/laserscan-cone-manager-test.cpp src/LaserscanConeManager.cpp)
    #target_link_libraries(laserscan-cone-manager-test ${catkin_LIBRARIES})

    # Adding rostest to the package
    #find_package(rostest REQUIRED)
    # name the test and link it to the .test file and the .cpp file itself, this will allow
    # "catkin_make run_tests" to be able to find and run this rostest
    #add_rostest_gtest(laserscan_cone_manager_rostest test/sb_laserscan_processing_test.test test/laserscan_cone_manager_rostest.cpp)
    #target_link_libraries(laserscan_cone_manager_rostest ${catkin_LIBRARIES})
#endif()