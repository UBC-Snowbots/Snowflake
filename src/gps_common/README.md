# gps_common
- This package was taken from https://github.com/swri-robotics/gps_umd. 
- Added for the use of the `utm_odometry_node`, which allows the conversion of `sensor_msgs/GPSNavSatFix` messages to `nav_msgs/Odometry` messages
- For an example of using `utm_odometry_node`, please see the `launch` folder
- `Odometry` messages are in the UTM coordinate system ([Wikipedia](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system))
