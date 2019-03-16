cmake_minimum_required(VERSION 2.8.3)
project(state_machine_circ)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
        "src/state_machine.py"
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )