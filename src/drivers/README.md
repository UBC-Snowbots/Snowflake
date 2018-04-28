# Drivers

## Determining Variance for encoder_to_odometry node
The `linear.x`, `linear.y`, and `angular.z` in the `Odometry` msg that we publish based on encoder data all have associated variances that are stored along the diagonal of the `pose.covariance` matrix (for full documentation, see [Odometry msg documentation](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)). These are set as parameters on the `encoder_to_odometry` node. These values are determined experimentally, and are different for every robot. Here are the steps to determine them:

1. Start the `encoder_to_odometry` node, and push the robot in a curve with a radius of at least 1 meters (the larger the better, just make sure you are pushing it in the exact same curve every time). Repeat this at least 10 times, re-starting the node each time, and recording the end `linear.x`, `linear.y`, `angular.z`, and `angular.w` values. (**Note:** The orientation in the `Odometry` msg is a quaternion. To get `theta` in radians, use an [online quaternion-radian converter](http://quaternions.online/) or use the math [from wikipedia](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code_2)).
![calibration-path](https://user-images.githubusercontent.com/9075711/39399650-7cd944b4-4ad6-11e8-88fa-50dd55bb051f.jpg)
2. Calculate the *standard deviation* for `linear.x`, `linear.y`, and `angular.z`. You can do this by putting them into a spreadsheet and calculating `STDEV` for a column (see photo below). Take the square root of each of these values to get the *variance*.
TODO: Photo of spreadsheet here
![variance-spreadsheet](https://user-images.githubusercontent.com/9075711/39399651-7e162676-4ad6-11e8-97a4-a91289ef4d16.png)


