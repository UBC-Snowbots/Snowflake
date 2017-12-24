cmake_minimum_required(VERSION 2.8.3)
project(localisation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(sb_utils REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions(-std=c++14)

catkin_package(
    INCLUDE_DIRS include
    DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${sb_utils_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ./include
)

## Declare a C++ executable
# add_executable(sample_package_node src/sample_package_node.cpp)
add_executable(ekf_node src/ekf_node.cpp src/EKFNode.cpp src/EKF.cpp include/EKFNode.h include/EKF.h)

## Specify libraries to link a library or executable target against
# target_link_libraries(sample_package_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(ekf_node 
	${catkin_LIBRARIES} 
	${sb_utils_LIBRARIES}
	${EIGEN3_LIBRARIES}
	)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)

    # Adding gtests to the package
    catkin_add_gtest(ekf-node-test test/ekf-node-test.cpp src/EKFNode.cpp src/EKF.cpp)
    target_link_libraries(ekf-node-test ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

    # Adding rostest to the package
    find_package(rostest REQUIRED)
    add_rostest_gtest(ekf_node_rostest test/ekf_node_test.test test/ekf_node_rostest.cpp src/EKFNode.cpp src/EKF.cpp)
    target_link_libraries(ekf_node_rostest ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
endif()

