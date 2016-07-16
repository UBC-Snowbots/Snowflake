# IGVC-2017
UBC Snowbots repo for the 2017 intelligent ground vehicle competition


## Setup
- Install Ubuntu (We recommend you use a minimum of 30GB of space) ([Windows](http://www.tecmint.com/install-ubuntu-16-04-alongside-with-windows-10-or-8-in-dual-boot/), [Mac](http://www.howtogeek.com/187410/how-to-install-and-dual-boot-linux-on-a-mac/))
- Boot into Ubuntu
- Setup your UBC alumni email account [here](https://id.ubc.ca/) and then get a JetBrains education accout [here](https://www.jetbrains.com/shop/eform/students). You will use this to active CLion later on
- Clone this repository by running `git clone https://github.com/UBC-Snowbots/IGVC-2017.git`
- Install everything you'll need to get started by running `cd IGVC-2017 && ./install_required.sh` (Just choose yes and enter your password when needed throughout) **(Do not run this script as root)**


## Important Notes:
- To run CLion, you must first go in to terminal, navigate to your project, run `source devel/setup.sh` and then **from the same terminal** run `clion`
- CLion will not support autocompletion in your *.cpp* and *.h* files until you've added them to the CMake file
