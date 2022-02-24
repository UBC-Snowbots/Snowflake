# Snowflake
UBC Snowbots Repository for competitions.

![alt tag](https://travis-ci.org/UBC-Snowbots/Snowflake.svg?branch=core)

## Table of Contents
- [Installation and Setup](#installation-and-setup)
  - [Important Notes](#important-notes)
  - [Zed Configuration](#zed-configuration)
  - [External Packages](#external-packages)
  - [Nodelets](#nodelets-quick-guide)
- [New Members](#new-members)
- [Conventions](#conventions)
  - [Github](#github-conventions)
  - [Pull Requests](#pull-requests)
  - [Coding Conventions](#coding-conventions)
  - [Coordinate Systems](#coordinate-systems)
- [Creating a New Node](#creating-a-new-node)
- [Creating a New Simple Node](#creating-a-new-simple-node)
- [Launch Files](#launch-files)
- [Testing](#testing)
  - [GTest](#gtest)
  - [Rostest](#rostest)
- [Simulating with Gazebo](#simulating-with-gazebo)
- [Arduino Development](#arduino-development)
- [Intel Realsense](#intel-realsense)
- [Debugging Tips](#debugging-tips)

## Installation and Setup

You will be downloading an Ubuntu ISO and multiple ROS packages with their respective dependencies.
It is highly recommended that you have access to high speed internet while doing this entire setup; 
if you're on campus use the `ubcsecure` or `resnet` networks for best results.

1. Install Ubuntu 18.04 (**Note**: Download 18.04, **not** another version, as other Ubuntu versions will make setting up this repo much more painful) (**Backup your important data first**) (We recommend you use a minimum of 30GB of space) 
    - For dual-booting: [Windows Instructions](https://askubuntu.com/questions/1031993/how-to-install-ubuntu-18-04-alongside-windows-10), [Mac Instructions](https://www.maketecheasier.com/install-dual-boot-ubuntu-mac)
2. If you haven't done so already, setup your UBC alumni email account [here](https://id.ubc.ca/) 
3. Using your UBC email account, get a JetBrains education account [here](https://www.jetbrains.com/shop/eform/students)
    - _JetBrains will send an initial email to confirm the UBC email you inputted, 
    once you've confirmed another email will be sent to activate your new education account; 
    you will use this account to set up CLion later on_
4. Boot into Ubuntu for the remaining steps
5. Install git by running `sudo apt-get install git`
6. Fork this repository by heading over to the GitHub page and click the fork button on the top right
7. Clone your server-side repository from the terminal by running `git clone --recursive https://github.com/YOUR_USERNAME/Snowflake.git` (`YOUR_USERNAME` is your github username)
8. To start the setup run `cd ~/Snowflake && ./setup_scripts/install_tools.sh` (Do not run this script as root).
    - *Just choose yes and enter your password when the terminal prompts you*
9. Run `source /opt/ros/melodic/setup.sh`.
10. To build the workspace run `catkin_make`. If everything compiles correctly and you don't get any errors, then you're good to go!
11. If catkin_make fails, and mentions missing .cmake files, run `./setup_scripts/install_dependencies.sh`.

### Important Notes:
- To run CLion with ROS, you must first go in to terminal, navigate to your project (`cd ~/Snowflake`), run `source devel/setup.sh` and then **from the same terminal** run `clion`
- CLion will not support auto-completion in your *.cpp* and *.h* files until you've added them to the CMake file

### Zed Configuration
- Follow the instructions on [this github page](https://github.com/stereolabs/zed-ros-wrapper) This repository already contains `zed_ros_wrapper` as a ~~submodule~~ standalone package in `src/external_pkgs/zed_ros_wrapper`(As of 4/27/2018 submodules have been migrated to wstool)
- Download ZED calibration file from the link indicated when you run `zed.launch` and place it in the folder `/usr/local/zed/settings/`

### External Packages
Note: External packages are defined as packages which does not have an installable build from apt.

To manually install external packages using wstool, run `wstool set src/external_pkgs/<pkg_name> https://github.com/some_org/<pkg_name>.git --git` in the workspace root.
The full syntax and options can be found with `wstool set -h`

To pull in the external package we call `wstool update` 

 
Note: `wstool` is addressed and run in `install_dependencies.sh`, should there be any need to run manual, `wstool` should be run after all steps from installation and setup has been completed. 

Current packages included under wstool: 
- Sicktoolbox: Drivers for interacting with Sick Lidars 
- Sicktoolbox_wrapper: Interface for Sick Lidar drivers integration into ROS
- ROS_Arduino_bridge: Arduino driver for ROS
- NMEA_Navsat_Driver: NMEA msg driver
- Zed_ROS_Wrapper: Wrapper for interfacing Zed Stereo Cam with ROS
- realsense: Wrapper for interfacing the Realsense D415 with ROS

### Nodelets Quick Guide
See [here](nodelets.md)

## Tips for New Members

Tutorials and advice for new members.

### Terminal

First of all, if you're new to the [Ubuntu command line](https://lifehacker.com/how-can-i-quickly-learn-terminal-commands-1494082178): it is primarily used over the graphical user interface as it allows you to do more.

### Github

In case you're new to GitHub, get started with this [tutorial](https://guides.github.com/activities/hello-world/)

Pretty cool right? Now you're wondering how you can use GitHub from the terminal/command line, it's called [Git](https://try.github.io/levels/1/challenges/1)

### Workflow

Now you're wondering how we develop our software, the first place to start is understanding how our workflow **works.** We use the Forking Workflow, which you can learn more about under **Github Conventions** in `README.md` (located in the same directory you opened this readme from) and other important information you should know. If you have questions, you may find your answers over there. 

### C++

This is the language we use to develop most of our software, so if you are new with C++, understand the fundamentals from this [tutorial](https://www.tutorialspoint.com/cplusplus/), but there are much more resources online as well.

### ROS

By now you understand the basics of ROS, but to learn more about it, you can complete the intermediate tutorials or brush up on the basics if you need to [here](http://wiki.ros.org/ROS/Tutorials)

### General Advice

Rarely will your Github issues contain all the information that you need to resolve them. Using google to search for that missing piece you need is sometimes useful (think like an optimist when you're understanding/interpreting your project/issue: "If only I knew how to do...", then search "how to do..."), or of course don't hesitate to ask for help, clarification, or just more information from any of the software leads. 

Remember it is much better to get help when you're stuck than to waste time trying to figure out what's wrong. This way, you're more or less continuously progressing, and as a whole, we develop efficiently and our software stack progresses.

## Conventions

### Github Conventions
- We follow the Forking Workflow:
    - [what it is](https://www.atlassian.com/git/tutorials/comparing-workflows#forking-workflow)
    - [how to use it](https://gist.github.com/Chaser324/ce0505fbed06b947d962)
- Only commit files that are essential for the system to run; do not put any photos or videos in here
- All files **must** be formatted properly. Formatting will be enforced with the `clang-format` tool. 
    - To check and fix formatting, from the `Snowflake` folder run `./clang_format/fix_formatting.sh BRANCH_NAME`, where `BRANCH_NAME` is the name of the branch you intend to merge your code into (ex. `iarrc` or `core`). This script will fix any improperly formatted code, but will refuse to change any files with uncommited changes (to prevent you losing work)
- Once your pull request has been reviewed and revised until it looks good from both your and the reviewers' sides, go ahead and Squash and Merge it, which will squash all the commits on your pull request into one and merge it to the target branch.

### Pull Requests
Below are some notes we would like to highlight about pull requests: 
- before opening a pull request, quickly run through the [pull request checklist](pull-request-checklist.md) and make sure you've satisfied everything
- Once your pull request has been approved, please proceed to merge and close the pull request yourself
- When your pull requests receive comments, please reply to each comment individually. 
- If there were any fixes that were specific to resources you found (eg. stackoverflow thread), please comment them into the PR for future reference. 
- On a similar note, if you made design decisions, please document them in the comments of the PR. We often go back to close PRs to look at why things were done a certain way. It is very helpful for us to know how you came up with your brilliant solution. 

### Coding Conventions
- Every **.cpp** and **.h** file should start with 
```
/*
 * Created By: Someone
 * Created On: December 1st, 2000
 * Description: A quick description of what this file does/is for
 */
```

- Functions should be commented a la JavaDoc
- The Javadoc comment (below) should be directly above every function in the header file
```
/**
 * One line description of the function
 *
 * A longer and more in depth description of the function
 * if it is needed.
 * 
 * @param param_one the first parameter of the function
 * @param param_two the second parameter of the function whose
 *                  description goes longer than one line
 * @return what the function returns if it returns anything
 * 
 */
```

- Classes are **CamelCase**
- Variables are **non_camel_case**
- Constants are **ALL_CAPS_WITH_UNDERSCORES**
- Functions are **camelCase**
- Indentations are 4 spaces

- `CMakeLists.txt` files should be reduced to only contain the minimum amount of comments. The version in `sample_package` has all the comments left in (for the sake of verbosity), so for a more representative example of what yours should look like, see `sb_vision/CMakeLists.txt` (or really any package aside from `sample_package`)

### Coordinate Systems
- We try to follow ROS standards, which can be found [here](http://www.ros.org/reps/rep-0103.html)
- x : forward
- y : left
- z : up
```
              +X
              ^
              |
      +θ  +<----->+ -θ
          |   |   |
          V   |   V
+Y <---------------------> -Y
              |
              |
              |
              V
              -X
```

## Creating a new node
- If your node is at all complicated, then this format should be followed. For simple nodes, please [see below](#creating-a-new-simple-node)
- Each node should be class based
- **MyNode.h** should contain your class declaration
- **MyNode.cpp** should contain your class definition
- **my_node.cpp** should be relatively small, and should just contain a **main** function to run your node
- **my-node-test.cpp** should contain all your gtest (unit test)
- **my_node_rostest.cpp** should contain your rostest (integrated test)
- For an example of this, please see `src/sample_package`
<pre>
some_ros_package
|  CMakeLists.txt
|  package.xml
└───src
|   | <b>MyNode.cpp</b> 
|   | <b>my_node.cpp</b>
|
└───launch
|   | <b>my_node.launch</b>
|
└───include
|   | <b>MyNode.h</b>
| 
└───test
|   | <b>my-node-test.cpp</b>
|   | <b>my_node_rostest.cpp</b>
|   | <b>sample_package_test.test</b>
</pre>

## Creating a new **simple** node
- You will not be able to write unit tests for this type of node, so it must be extremely simple
- **my_node.cpp** will contain the entire node, and will probably contain a `while(ros::ok()){}` loop
<pre>
some_ros_package
|  CMakeLists.txt
|  package.xml
└───src
|   | <b>my_node.cpp</b>
</pre>

## Launch files
- A launch file (ends in .launch) is an easy way to run multiple nodes with specified parameters with one single command. This is useful if a package has multiple nodes that needs to be run. A launch file can also launch other smaller launch files so it can be used to start every node needed for the robot to run properly. Reference [here](http://wiki.ros.org/roslaunch).  
- To use the launch file, the command is: `roslaunch package_name package_launch_file.launch`

## Testing
### GTest
- GTest is our primary testing tool at the moment. The ROS wiki has a quick intro to it [here](http://wiki.ros.org/gtest), and we also strongly recommend you read Google's introduction to it [here] (https://github.com/google/googletest/blob/master/googletest/docs/Primer.md), then setup and write a few example tests before you start using it with ROS.
- Once you've setup your tests in ROS, run `catkin_make run_tests` to run them
- To test a specific package, run `catkin_make run_tests_MY_PACKAGE_NAME`

### Rostest
- For tests which require more than one active node, i.e. integrated testing, the rostest framework provides a way to launch your test alongside all the nodes it requires. This is an extension on roslaunch enabling it to run test nodes. Special test nodes are nested within a `<test></test>` tag. This also needs a special entry under CMakelists as shown in the sample package. See more details [here](http://wiki.ros.org/rostest)
- Similar to launch files, the command is: `rostest package_name package_test_file.test`.

## Simulating with Gazebo
- You will always need to source the project before running gazebo, by moving to the project directory with `cd ~/Snowflake` and then `source devel/setup.sh`
- You will probably need a computer with an dedicated gpu, as gazebo **sometimes** works with intel integrated graphics, but generally not. If you do end up using a computer without a dedicated gpu, make sure to go in to `sb_gazebo/urdf/**ROBOT_NAME**.gazebo` and switch around the lidar settings (see comments in said file)
- All worlds should go in the `sb_gazebo/worlds` folder
- To launch a world, simply run the appropriate launch file in `sb_gazebo/launch`
- Once the world has launched, it is common for the robot to be initially unable to move. Just lift it up a bit in gazebo and drop it for this to be fixed
- You can manually control the robot with your keyboard, logitech, ps3, or xbox controller. To do so, simply `cd ~/Snowflake` and then `source devel/setup.sh`, then run the appriate launchfile from `src/sb_gazebo/launch` by running `roslaunch LAUNCH_FILE.launch` (from within the launch folder). **Note:** At least with PS3 controllers, you have to hold down `L1` while using the joystick for the controller to work

## Arduino Development
- When developing the firmware/Arduino parts of the software, we've made a complete arduino workspace in `src/firmware`. This way you don't need to worry about downloading the libraries yourself!
- In order to use this, go to your Arduino IDE's Preferences dialog box and use `/your/path/to/Snowflake/src/firmware` as your sketchbook directory. Open arduino sketches in the workspace and they will work!

## Intel Realsense
Intel Realsense is a sensor that captures a 3D depth map of the environment in front of the sensor. This 3D depth map is called a **point cloud**, which is essentially just a list (vector in c++) that contains a bunch of points, where each point contains the x, y, and z coordinates, as well as a color value associated to it. 
- **Visualizing the input from the realsense**: Run `roslaunch realsense2_camera rs_rgbd.launch` to launch realsense. You should be able to see the point cloud at the topic named `/camera/depth_registered/points` in `rviz`.
- **Recording a rosbag from the Realsense**: Recording a rosbag  from the Realsense is useful, because you can record the input from the sensor and play it as many time as you want after recording it. That way, you don't need the physical sensor and you don't to "capture" the same thing every time you run it.
    1. Launch realsense using `roslaunch realsense2_camera rs_rgbd.launch`.
    2. Record a rosbag by running `rosbag record /camera/depth_registered/points <options>`. See `rosbag record -h` for the list of options. To stop the recording, just press ctrl+c. E.g. `rosbag record /camera/depth_registered/points --split --duration 1m --max-splits 3 -o ../rosbags/tennis_ball/` - split files every 1 minute, split for maximum of 3 times, and store the rosbag in ` ../rosbags/tennis_ball/` folder.
    3. Stop the realsense node (ctrl+c the process launched in step 1)
    4. Run `rosbag play path/to/rosbag/file -l`. `path/to/rosbag/file` is the path to the rosbag generated in step 2. Note that the `-l` flag loops the recording so it will play forever until you terminate it manually (using ctrl+c).
    5. Open `rviz`. You should be able to see the recording of the point cloud at `/camera/depth_registered/points`.

## Debugging Tips
- If something is happening that does not seem to correspond to your code, (ex. if your node is subscribing to the totally wrong topic) try deleting the `build` and `devel` folders (if present) to remove any "cached" information

