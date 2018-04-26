/**
 * Created by William Gu on 24/03/18.
 * Class declaration for Cone Identification, which contains static methods for identifying cones in a laser message
 */
#ifndef CONEIDENTIFICATION_H
#define CONEIDENTIFICATION_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <mapping_igvc/Point2D.h>
#include <mapping_igvc/ConeObstacle.h>
#include <math.h>



class ConeIdentification {
    public:
        /**
         * Identifies the cones in a given laserscan message. Note that all cones will have radius of the expected radius, and only cones that
         * have a calculated radius within radius_tol of expected radius will be added to the resulting identified cones.
         * @param laser_msg laserscan message to analyze
         * @param dist_tol max distance between points to be considered that are part of one cone
         * @param radius_exp expected radius of cone
         * @param radius_tol maximum difference between calculated cone radius and expected cone radius
         * @return a vector of cone obstacle messages identified
         */
        static std::vector<mapping_igvc::ConeObstacle> identifyCones(const sensor_msgs::LaserScan &laser_msg, float dist_tol, float radius_exp, float radius_tol);

        /**
        * Converts a cluster of edge points to a cone obstacle with a predicted radius
        * @param edge_points should have size >= 3 and be in the order they appear in the object
        * @return a cone formed by edge points
        */
        static mapping_igvc::ConeObstacle edgeToCone(const std::vector<mapping_igvc::Point2D> &edge_points);


    private:
        /**
         * Converts a laserscan reading to a point
         * @param dist distance reading, should be in valid min-max range of laser scan
         * @param ang angle reading, should be in valid min-max angle of laser scan
         * @return point in 2d
         */
        static mapping_igvc::Point2D laserToPoint(float dist, float ang);

        /**
         * Gets the distance between 2 points
         * @param p1 point 1
         * @param p2 point 2
         * @return distance between points
         */
        static float getDist(const mapping_igvc::Point2D &p1, const mapping_igvc::Point2D &p2);
};



#endif //CONEIDENTIFICATION_H
