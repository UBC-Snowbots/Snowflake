/*
 * Created by William Gu on 29/04/18.
 * Helper class for populating laser scan messages given x and y coordinates
 */

#ifndef PROJECT_LASERSCANBUILDER_H
#define PROJECT_LASERSCANBUILDER_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <mapping_igvc/Point2D.h>
#include <mapping_igvc/ConeObstacle.h>
#include <math.h>
#include <ConeIdentification.h>

namespace LaserscanBuilder {
    class LaserscanBuilder {
    public:
        LaserscanBuilder() {
            point_tol = 0.01;
            num_rays = 1000;
            laser_msg.range_min = 0.1;
            laser_msg.range_max = 5.0;
            laser_msg.angle_min = (float) -M_PI / 2;
            laser_msg.angle_max = (float) M_PI / 2;
            laser_msg.angle_increment = (laser_msg.angle_max - laser_msg.angle_min) / num_rays;
            // Set all the ranges to 10 initially (out of range)
            laser_msg.ranges = std::vector<float>(num_rays, 10.0);
        }

        /**
         * Adds a cone to the laserscan messasge (only the edge points closest to the origin of the scan is considered)
         * @param x coordinate of cone (center)
         * @param y coordinate of cone (center)
         * @param radius of cone
         */
        void addCone(double x, double y, double radius) {
            mapping_igvc::Point2D coneCenter; //The point of the center of the cone to add
            coneCenter.x = x;
            coneCenter.y = y;

            for (int i = 0; i < num_rays; i++) {
                double ang = laser_msg.angle_min + i * laser_msg.angle_increment;

                // If we can't find a close enough point, don't add any
                for (double range = laser_msg.range_min; range <= laser_msg.range_max; range += 0.01) {
                    mapping_igvc::Point2D point = ConeIdentification::laserToPoint(range, ang);
                    if (fabs(ConeIdentification::getDist(coneCenter, point) - radius) <= point_tol) {
                        if (range < laser_msg.ranges[i]) { //Only add if not currently occupied by a closer range
                            laser_msg.ranges[i] = range;
                            //std::cout<<"("<<point.x<<","<<point.y<<")"<<std::endl;
                        }
                        break; //If we find the edge, we are done for this ray
                    }
                }
            }
        }

        /**
         * Get the laserscan with the cones previously added
         * @return a laserscan message
         */
        sensor_msgs::LaserScan getLaserscan() {
            return laser_msg;
        }

    private:
        sensor_msgs::LaserScan laser_msg;
        ulong num_rays;
        double point_tol;
    };
}

#endif //PROJECT_LASERSCANBUILDER_H
