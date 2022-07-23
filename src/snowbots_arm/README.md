Steps:

1) ./install-snowbots-arm-dep.sh
2) ./install_new_cmake.sh
3) ./pro_controller_moveit_fix.sh

Install these dependencies:

```
sudo apt-get update && apt-get install -y build-essential rviz python-rosdep \
	ros-melodic-joy ros-melodic-joint-state-publisher vim \
	ros-melodic-robot-state-publisher ros-melodic-joystick-drivers ros-melodic-joy \
	ros-melodic-ros-control ros-melodic-ros-controllers \
	ros-melodic-urdf-tutorial ros-melodic-rqt
```
Then apply the proController driver by running the pro controller fix and driver.

Be sure to have a CMake version of 3.13 or higher.

If you don't have a CMake version that is 3.13 or higher, you can run my install cmake script. Do this before running the pro controller fix script.

Then run

```
./install_new_cmake.sh
```

Then run

```
./pro_controller_moveit_fix.sh
```

Or run the install script in the root of this folder 

```
install-snowbots-arm-dep.sh
```

To run Moveit:

Run the command:

```
roslaunch snowbot_arm_moveit_configs demo.launch
```

To enable Joystick Control:

```
roslaunch snowbot_arm_moveit_configs joystick_control.launch
```
