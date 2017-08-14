# JFrost Launch Steps

## IMPORTANT
Run `roscore` on its own terminal.

## Base Robot Control:
1. Firmware flashed to the controlling arduino should be under: 
```
/ros_arduino_bridge/ros_arduino_firmware/src/libraries/ROSArduinoBridge/RosArduinoBridge.ino
```
2. To launch the corresponding ROS node which talks to the firmware:
```
roslaunch ros_arduino_python arduino.launch
```

The config file is hosted at: 

```
Shortcut:
rosed ros_arduino_python my_arduino_params.yaml

Full Path:
ros_arduino_bridge/ros_arduino_python/config/my_arduino_params.yaml
```


## Sensors

### Vision

1. Launch entire vision stack:
```
roslaunch vision vision_stack.launch
```

2. That's it.

### GPS
Ignore the crummy version numbering.

1. Launch the gps manager:
```
roslaunch decision gps_manager3.launch
```

2. Launch gps decision:
```
roslaunch decision gps_decision2.launch
```

### Lidar
1. Launch lidar:
```
roslaunch drivers sick_lidar.launch
```

### IMU
```
roslaunch razor_imu_9dof razor-pub-and-display.launch
```

### EKF
```
roslaunch localisation ekf.launch
```









