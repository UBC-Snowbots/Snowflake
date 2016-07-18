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

## Using Gazebo
- You will probably need a computer with an dedicated gpu, as gazebo **sometimes** works with intel integrated graphics, but generally not. If you do end up using a computer without a dedicated gpu, make sure to go in to `elsa_gazebo/urdf/elsa.gazebo` and switch around the lidar settings (see comments in said file)
- All worlds should go in the `elsa_gazebo/worlds` folder
- To launch a world, simply run the appropriate launch file in `elsa_gazebo/launch`
- To create a launch file for your world, create one in `elsa_gazebo/launch`, using `elsa_gazebo/launch/sample.launch` as a guide
