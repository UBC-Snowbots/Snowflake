# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "hardware_interface;controller_manager;roscpp;urdf;joint_limits_interface;arm_hardware_driver".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-larm_hardware_interface".split(';') if "-larm_hardware_interface" != "" else []
PROJECT_NAME = "arm_hardware_interface"
PROJECT_SPACE_DIR = "/workfiles/snowbots_arm/install"
PROJECT_VERSION = "0.0.0"
