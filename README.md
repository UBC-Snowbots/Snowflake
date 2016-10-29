# IGVC-2017
UBC Snowbots Repository for the 2017 Intelligent Ground Vehicle Competition.

![alt tag](https://travis-ci.org/UBC-Snowbots/IGVC-2017.svg?branch=master)

## Installation and Setup

You will be downloading an Ubuntu ISO and multiple ROS packages with their respective dependencies.
It is highly recommended that you have access to high speed internet while doing this entire setup; 
if you're on campus use the `ubcsecure` or `resnet` networks for best results.

1. Install Ubuntu 16.04 (We recommend you use a minimum of 30GB of space) 
    - For dual-booting: [Windows Instructions](http://www.tecmint.com/install-ubuntu-16-04-alongside-with-windows-10-or-8-in-dual-boot/), [Mac Instructions](http://www.howtogeek.com/187410/how-to-install-and-dual-boot-linux-on-a-mac/)
    - _Note_: You should always choose the "install alongside Windows/OSX" during your installation (step 7 in the windows tutorial)
2. If you haven't done so already, setup your UBC alumni email account [here](https://id.ubc.ca/) 
3. Using your UBC email account, get a JetBrains education account [here](https://www.jetbrains.com/shop/eform/students)
    - _JetBrains will send an initial email to confirm the UBC email you inputted, 
    once you confirm another email will be sent to activate your new education account; 
    you will use this account to set up CLion later on_
4. Boot into Ubuntu for the remaining steps
5. Install git by running `sudo apt-get install git`
6. Clone this repository by running `git clone https://github.com/UBC-Snowbots/IGVC-2017.git ~/IGVC-2017`
7. To start set-up run `cd ~/IGVC-2017 && ./get_started.sh` **(Do not run this script as root)**
    - _Just choose yes and enter your password when the terminal prompts you_ 
8. Build the ROS project by running `source /opt/ros/kinetic/setup.bash` and `cd ~/IGVC-2017 && catkin_make` 
    - If everything compiles correctly and you don't get any errors, then you're good to go!

## Important Notes:
- To run CLion with ROS, you must first go in to terminal, navigate to your project (`cd ~/IGVC-2017`), run `source devel/setup.sh` and then **from the same terminal** run `clion`
- CLion will not support auto-completion in your *.cpp* and *.h* files until you've added them to the CMake file

## What Should **NOT** Go In This Repo
- Photos or videos (that aren't needed for the system to run)

## Conventions
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
- Functions are **camelCase**
- Indentations are 4 spaces

## Creating a new node
- If your node is at all complicated, then this format should be followed. For simple nodes, please see below
- Each node should be class based
- **MyNode.h** should contain your class declaration
- **MyNode.cpp** should contain your class definition
- **my_node.cpp** should be relatively small, and should just contain a **main** function to run your node
- **my-node-test.cpp** should contain all your tests
- For an example of this, please see `src/sample_package`
<pre>
some_ros_package
|  CMakeLists.txt
|  package.xml
└───src
|   | <b>MyNode.cpp</b> 
|   | <b>my_node.cpp</b>
|
└───include
|   | <b>MyNode.h</b>
| 
└───test
|   | <b>my-node-test.cpp</b>
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

## Testing
- GTest is our primary testing tool at the moment. We strongly recommend you read Google's introduction to it [here] (https://github.com/google/googletest/blob/master/googletest/docs/Primer.md), then setup and write a few example tests before you start using it with ROS
- Once you've setup your tests in ROS, run `catkin_make run_tests` to run them
- To run the tests for a specific package, run `catkin_make run_tests_MY_PACKAGE_NAME`

## Using Gazebo
- You will always need to source the project before running gazebo, by moving to the project directory with `cd ~/IGVC-2017` and then `source devel/setup.sh`
- You will probably need a computer with an dedicated gpu, as gazebo **sometimes** works with intel integrated graphics, but generally not. If you do end up using a computer without a dedicated gpu, make sure to go in to `elsa_gazebo/urdf/elsa.gazebo` and switch around the lidar settings (see comments in said file)
- All worlds should go in the `elsa_gazebo/worlds` folder
- To launch a world, simply run the appropriate launch file in `elsa_gazebo/launch`
- To create a launch file for your world, create one in `elsa_gazebo/launch`, using `elsa_gazebo/launch/sample.launch` as a guide
- To manually control the robot, run `rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/elsa/cmd_vel`

## Github Procedure
- We follow the "Feature Branch Workflow"
- A good tutorial can be found [here](https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow)

## Arduino Development
- When developing the firmware/Arduino parts of the software, we've made a complete arduino workspace in `src/firmware`. This way you don't need to worry about downloading the libraries yourself!
- In order to use this, go to your Arduino IDE's Preferences dialog box and use `/your/path/to/IGVC-2017/src/firmware` as your sketchbook directory. Open arduino sketches in the workspace and they will work!
