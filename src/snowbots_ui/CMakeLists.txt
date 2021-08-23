##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(snowbots_ui)

set(CMAKE_AUTOUIC ON)
##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp) 

include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml

##############################################################################
# Sections
##############################################################################


set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets)
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})##############################################################################
# Sources
##############################################################################
include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    )

set(QT_SOURCES
#Main Window
    src/main.cpp
    src/mainwindow.cpp
    include/mainwindow.h
    src/mainwindow.ui
#Dialog Window
    #src/Manual_Window/manual_window.cpp
    #src/Manual_Window/manual_window.h
    #src/Manual_Window/manual_window.ui
 #ROS SRC
    include/RosIntegration.h
    src/RosIntegration.cpp

    )

##############################################################################
# Binaries
##############################################################################
add_executable(snowbots_ui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(snowbots_ui ${QT_LIBRARIES} ${catkin_LIBRARIES}
        Qt5::Core     Qt5::Widgets)
install(TARGETS snowbots_ui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

