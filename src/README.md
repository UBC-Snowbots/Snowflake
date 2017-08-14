# DIRECTORY 
```
.
├── challenge: The Snowbots Software Initiation Challenge 
├── decision: The main decision logic. Analyzes all the sensor inputs (GPS, LIDAR, vision) to create commands to send to the robot
├── drivers: Contains all the drivers that interface sensors/controllers with ROS
├── firmware: Various arduino (.ino) files that make up the firmware, for various external sensors/controller (GPS, IMU, etc.)
├── gps_common: provides a node for conversion of `sensor_msgs/NavSatFix` to `nav_msgs/Odometry` 
├── localisation: houses all nodes/launch files pretaining to robot localisation 
├── razor_imu_9dof: ROS driver for interfacing with the razor IMU
├── ros_arduino_bridge: Mainly contains the encoder driver and firmware
├── sample_package: Probably the only well-formatted package we have. Reference this for package syntax & formatting
├── sb_gazebo: Everything to do with simulation. Contains models, launch files, and custom nodes designed to work with Gazebo.
├── sb_utils: utility package with useful general functions
├── sicktoolbox: An SDK for the SICK lidar module
├── sicktoolbox_wrapper: A ROS wrapper for the SICK SDK
├── vision: Contains everything to do with vision (camera imagery) processing
└── zed-ros-wrapper: Contains nodes for working with the ZED Stereo Camera

```
