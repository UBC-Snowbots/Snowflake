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
         * Identifies the cones in a given laserscan message
         * @param laser_msg laserscan message to analyze
         * @param tolerance max distance between points to be considered within a cluster
         * @return a vector of cone obstacle messages identified
         */
        static std::vector<mapping_igvc::ConeObstacle> identifyCones(const sensor_msgs::LaserScan &laser_msg, float tolerance);


    private:
        /**
         * Converts a laserscan reading to a point
         * @param dist distance reading, should be in valid min-max range of laser scan
         * @param ang angle reading, should be in valid min-max angle of laser scan
         * @return point in 2d
         */
        static mapping_igvc::Point2D laserToPoint(float dist, float ang); //TODO: Points must be in global frame?

        static float getDist(const mapping_igvc::Point2D &p1, const mapping_igvc::Point2D &p2);

        static mapping_igvc::ConeObstacle clusterToCone(const std::vector<mapping_igvc::Point2D> &cluster_points);
};



#endif //CONEIDENTIFICATION_H
