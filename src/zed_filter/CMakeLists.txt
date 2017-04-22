cmake_minimum_required(VERSION 2.8.3)
project(zed_filter)

find_package(catkin REQUIRED COMPONENTS roscpp rospy pcl_ros tf2_sensor_msgs)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zed_filter
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
    ./include
)


link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(zedfilter src/zed_filter.cpp src/ZedFilter.cpp include/ZedFilter.h)
target_link_libraries(zedfilter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    #gTest
    catkin_add_gtest(zed-filter-test test/zed-filter-test.cpp src/ZedFilter.cpp)
    target_link_libraries(zed-filter-test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
endif()