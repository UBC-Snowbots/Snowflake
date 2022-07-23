Install these dependencies:

```
sudo apt-get update && apt-get install -y build-essential rviz python-rosdep \
	ros-melodic-joy ros-melodic-joint-state-publisher vim \
	ros-melodic-robot-state-publisher ros-melodic-joystick-drivers ros-melodic-joy \
	ros-melodic-ros-control ros-melodic-ros-controllers \
	ros-melodic-urdf-tutorial ros-melodic-rqt
```
Then apply the proController driver by running the script:

```
./pro_controller_moveit_fix.sh
```

Or run the install script in the root of this folder 

```
install-snobots-arm-dep-.sh
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
