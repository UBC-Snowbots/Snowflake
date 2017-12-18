/*
 * Created By: Marcus Swift
 * Created On: November 20th, 2017
 * Description: An extended kalman filter node that takes in sensor data from
 * the GPS, encoders and IMU and returns the bots estimated postion and
 * orientation
 */

#include <EKF.h>

int main(int argc, char** argv) {
    EKF ekf(argc, argv, "EKF");
    ros::spin();

    return EXIT_SUCCESS;
}