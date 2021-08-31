cmake_minimum_required(VERSION 2.8.3)
project(motor_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_motor src/move_motor.cpp src/MoveMotor.cpp include/MoveMotor.h)

target_link_libraries(move_motor
   ${catkin_LIBRARIES}
   phidget22
)
