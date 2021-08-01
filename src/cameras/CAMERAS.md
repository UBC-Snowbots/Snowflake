# Cameras
This package is implemented in 2021 to view both Intel cameras that UBC Snowbots owns: the Intel Realsense D415 and D435i

The serial number of the D415 is 739112061033

The serial number of the D435i is 116622072291

These serial numbers are reflected in the launch files in this directory. 

The launch files simply call the launch file from external_pkgs/realsense2_camera with specific argments

Running it without argments would connect to the first realsense camera found, but to connect to two those argments
need to be specified.

Both cameras can be launched using realsense_both.launch.

Lastly, camera_viewer.launch is a file that is supposed to work with any usb camera, copied from the internet. 
For the realsense cameras, it's probably better to use the ros wrapper as above, but for a new camera it might be worth
trying the camera_viewer.launch. 