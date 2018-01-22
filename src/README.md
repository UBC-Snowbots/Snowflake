# DIRECTORY 
## challenge
The Snowbots Software Initiation Challenge. This is the challenge that new members must complete in order to join the team. Remember, it's more about perserverance then anything else!
## decision_igvc
Originally created for IGVC 2017. Contains nodes that make up a purely reactive system for navigating the IGVC autonomous navigation course.
## drag_race_iarrc
Originally created for IARRC 2017. Contains nodes for running the drag race course at IARRC.
## drivers
Contains any custom drivers developed in house. Usually these take the form of a ROS node that interacts with hardware external to the computer.
## firmware
Contains any firmware developed in house. Usually arduino code.
## localisation_igvc
Originally created for IGVC 2017. Contains nodes used to localise the robot (figure out where we are) based on sensor data.
## nmea_navsat_driver
Contains a driver that parses NMEA strings (basically a specification for how GPS messages are passed over USB), and publishes them as ROS `NavSat` messages. This is used for interfacing with our Swift GPS.
## razor_imu_9dof
Contains a driver for interfacing with the [Razor 9DOF imu](https://www.sparkfun.com/products/retired/10736) (now discontinued). Note, this is scheduled for removal with PR #260.
## ros_arduino_bridge
Contains driver(s) for interfacing with our wheel encoders for the IGVC 2017 robot (model: https://www.sparkfun.com/products/11102). Driver(s) should work for most encoder models.
## sample_package
Our "best practices" package, should contain a simple example for creating a node and associated tests. Reference this for package syntax, formatting, and structure.
## sb_gazebo
Contains all models and launch files used to simulate our robots in [Gazebo](http://gazebosim.org/).
## sb_utils
Contains general utility functions and classes that we commonly need to use across packages.
## sb_vision
Contains general nodes used for vision processing, such as HSV and IPM filters.
## sicktoolbox
An SDK for our SICK lidar.
## sicktoolbox_wrapper
A ROS wrapper for the SICK SDK.
## zed-ros-wrapper
Contains drivers and nodes for interfacing with and working with the [ZED Stereo Camera](https://www.stereolabs.com/zed/).
