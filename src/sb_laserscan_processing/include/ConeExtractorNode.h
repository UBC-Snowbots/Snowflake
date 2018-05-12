/**
 * Created by William Gu on 24/03/18.
 * Class declaration for a Cone Extractor Node that identifies cones from a laser msg
 */

#ifndef LASERSCAN_CONE_MANAGER_H
#define LASERSCAN_CONE_MANAGER_H

#include <ConeIdentification.h>
#include <iostream>
#include <ros/ros.h>
#include <sb_utils.h>
#include <sensor_msgs/LaserScan.h>

class ConeExtractorNode {
    public:
        ConeExtractorNode(int argc, char** argv, std::string node_name);

    private:
        ros::Subscriber laser_subscriber;
        ros::Publisher cone_publisher;

        /**
         * Callback function for receiving laser scan msgs. Publishes the cones found in the laserscan to
         * the correct topic one by one. Note that the coordinates for cones are in the base-link (robot frame)
         * @param ptr
         */
        void laserCallBack(const sensor_msgs::LaserScan::ConstPtr& ptr);

        double cone_dist_tol; //Distance tolerance between cones in cluster
        double cone_rad_exp; //Expected cone radius
        double cone_rad_tol; //Tolerance for cone radius (max diff between calculated and expected values)
        int min_points_in_cone; //Index difference between points used in edge cluster splitting algorithm
        double ang_threshold; //Max angle needed to split edge clusters
};

#endif //LASERSCAN_CONE_MANAGER_H
