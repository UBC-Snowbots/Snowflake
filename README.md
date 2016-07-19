# IGVC-2017
UBC Snowbots repo for the 2017 intelligent ground vehicle competition


## Setup
- Install Ubuntu (We recommend you use a minimum of 30GB of space) ([Windows Instructions](http://www.tecmint.com/install-ubuntu-16-04-alongside-with-windows-10-or-8-in-dual-boot/), [Mac Instructions](http://www.howtogeek.com/187410/how-to-install-and-dual-boot-linux-on-a-mac/))
- Boot into Ubuntu
- Setup your UBC alumni email account [here](https://id.ubc.ca/) and then get a JetBrains education accout [here](https://www.jetbrains.com/shop/eform/students). You will use this to active CLion later on
- Clone this repository by running `git clone https://github.com/UBC-Snowbots/IGVC-2017.git ~/IGVC-2017`
- Install everything you'll need to get started by running `cd ~/IGVC-2017 && ./install_required.sh` (Just choose yes and enter your password when needed throughout) **(Do not run this script as root)**
- Build the ROS project by running `cd ~/IGVC-2017 && catkin_make`. If everything compiles correctly and you don't get any error's, then you're good to go!

## Important Notes:
- To run CLion with ROS, you must first go in to terminal, navigate to your project(cd ~/IGVC-2017), run `source devel/setup.sh` and then **from the same terminal** run `clion`
- CLion will not support autocompletion in your *.cpp* and *.h* files until you've added them to the CMake file

## What Should **NOT** Go In This Repo
- photos or videos (that aren't needed for the system to run)

## Conventions
- every **.cpp** and **.h** file should start with 
```
/*
 * Created By: Someone
 * Created On: December 1st, 2000
 * Description: A quick description of what this file does/is for
 */
```
- classes are **CamelCase**
- variables are **non_camelcase**
- functions are **camelCase**

## Creating a new node
- If your node is at all complicated, then this format should be followed. For simple nodes, please see below
- Each node should be class based
- **MyNode.h** should contain your class declaration
- **MyNode.cpp** should contain your class definition.
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
- you will not be able to write unit tests for this type of node, so it must be extremely simple
- **my_node.cpp** will contain the entire node, and will probably contain a `while(ros::ok()){}` loop
<pre>
some_ros_package
|  CMakeLists.txt
|  package.xml
└───src
|   | <b>my_node.cpp</b>
</pre>

## Testing
- ROS has native support for gtest (c++ unit testing). To run your tests, run `catkin_make run_tests`

## Using Gazebo
- You will probably need a computer with an dedicated gpu, as gazebo **sometimes** works with intel integrated graphics, but generally not. If you do end up using a computer without a dedicated gpu, make sure to go in to `elsa_gazebo/urdf/elsa.gazebo` and switch around the lidar settings (see comments in said file)
- All worlds should go in the `elsa_gazebo/worlds` folder
- To launch a world, simply run the appropriate launch file in `elsa_gazebo/launch`
- To create a launch file for your world, create one in `elsa_gazebo/launch`, using `elsa_gazebo/launch/sample.launch` as a guide
