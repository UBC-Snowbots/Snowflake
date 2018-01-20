cmake_minimum_required(VERSION 2.8.3)
project(mapping_igvc_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES ConeObstacle.msg LineObstacle.msg Point2D.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

#cmake_minimum_required(VERSION 2.8.3)
#project(geometry_msgs)
#
#find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
#
#add_message_files(
#  DIRECTORY msg
#  FILES
#  Accel.msg
#  AccelStamped.msg
#  AccelWithCovariance.msg
#  AccelWithCovarianceStamped.msg
#  Inertia.msg
#  InertiaStamped.msg
#  Point.msg
#  Point32.msg
#  PointStamped.msg
#  Polygon.msg
#  PolygonStamped.msg
#  Pose2D.msg
#  Pose.msg
#  PoseArray.msg
#  PoseStamped.msg
#  PoseWithCovariance.msg
#  PoseWithCovarianceStamped.msg
#  Quaternion.msg
#  QuaternionStamped.msg
#  Transform.msg
#  TransformStamped.msg
#  Twist.msg
#  TwistStamped.msg
#  TwistWithCovariance.msg
#  TwistWithCovarianceStamped.msg
#  Vector3.msg
#  Vector3Stamped.msg
#  Wrench.msg
#  WrenchStamped.msg)
#
#generate_messages(DEPENDENCIES std_msgs)
#
#catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
