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
         * @param min_points_in_cone the minimum number of points that can represent a valid cone
         * @param ang_tol the max angle for a split to be considered (in radians)
         * @return a vector of cone obstacle messages identified
         */
        static std::vector<mapping_igvc::ConeObstacle> identifyCones(const sensor_msgs::LaserScan &laser_msg, double dist_tol, double radius_exp, double radius_tol, int min_points_in_cone, double ang_threshold);

        /**
         * Identifies valid cones that can be made from points in edge_points and adds it to the identified_cones input vector
         * Note that multiple cones may be added in this function call
         * @param identified_cones cones identified so far
         * @param edge_points edge points (in a cluster) to analyze
         * @param radius_exp expected radius of cone
         * @param radius_tol maximum difference between calculated and expected cone radius
         * @param min_points_in_cone the minimum number of points that can represent a valid cone
         * @param ang_threshold the max angle for a split to be considered (in radians)
         * @param frame_id frame id used to generate edge_points (typically same as the laserscan msg)
         */
        static void addConesInEdgeCluster(std::vector<mapping_igvc::ConeObstacle> &identified_cones, std::vector<mapping_igvc::Point2D> &edge_points, double radius_exp, double radius_tol, int min_points_in_cone, double ang_threshold, std::string frame_id);

        /**
        * Splits a cluster of edge points such that points forming multiple cones extremely close to one another will be returned
        * as vectors of points where each vector represents one cone
        * @param edge_points
        * @param line_point_dist the number of points to take before and after each edge point to form a line and test angle
        * @param ang_tol the max angle for a split to be considered (in radians)
        * @return a vector consisting of split edge point vectors
        */
        static std::vector<std::vector<mapping_igvc::Point2D>> splitEdge(const std::vector<mapping_igvc::Point2D> &edge_points, int line_point_dist, double ang_threshold);

        /**
        * Converts a cluster of edge points to a cone obstacle with a predicted radius
        * @param edge_points should have size >= 3 and be in the order they appear in the object
        * @return a cone formed by edge points
        */
        static mapping_igvc::ConeObstacle edgeToCone(const std::vector<mapping_igvc::Point2D> &edge_points);

         /**
         * Converts a laserscan reading to a point
         * @param dist distance reading, should be in valid min-max range of laser scan
         * @param ang angle reading, should be in valid min-max angle of laser scan
         * @return point in 2d
         */
        static mapping_igvc::Point2D laserToPoint(double dist, double ang);

        /**
         * Gets the distance between 2 points
         * @param p1 point 1
         * @param p2 point 2
         * @return distance between points
         */
        static double getDist(const mapping_igvc::Point2D &p1, const mapping_igvc::Point2D &p2);

        /**
         * Get the mean x coordinate of points in edge_points
         * @param edge_points
         * @return mean x coordinate
         */
        static double getMeanX(const std::vector<mapping_igvc::Point2D> &edge_points);

        /**
         * Get the mean y coordinate of points in edge_points
         * @param edge_points
         * @return mean y coordinate
         */
        static double getMeanY(const std::vector<mapping_igvc::Point2D> &edge_points);

        /**
         * Get the slope of the regression line formed by points in edge_points
         * @param edge_points
         * @return slope of line
         */
        static double getRegressionSlope(const std::vector<mapping_igvc::Point2D> &edge_points);

};



#endif //CONEIDENTIFICATION_H
