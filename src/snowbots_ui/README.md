# Snowbot_UI
This is a UI package built for the UBC Snowbot Team


## Installation
### Install qt build for ROS Melodic.
```bash
sudo apt-get install ros-melodic-qt-build
```
### Create a catkin workspace or cd if you have already one
```bash
mkdir -p ~/catkin_ws/src
cd catkin_ws/src/
```
### Clone repository to src folder
```bash
git clone https://github.com/alexspirou/ROS_Melodic_Qt_GUI_Template.git
```
### Install turtlesim
```bash
sudo apt-get install ros-melodic-turtlesim
```
### Catkin make
```bash
cd ~/catkin_ws/
catkin_make
```


# Run GUI
### First run a rosmaster
```bash
roscore
```
### Run turtlesim in a new terminal
```bash
rosrun turtlesim turtlesim_node
```
### Open another terminal, source your bashrc file and RUN GUI
```bash
source devel/setup.bash
rosrun ros_qt_gui_template ros_qt_gui_template 
```

NOTE: If you want to edit your project from qtcreator watch this tutorial
```bash
https://www.youtube.com/watch?v=Can7zppN-Kg&t=519s
```
