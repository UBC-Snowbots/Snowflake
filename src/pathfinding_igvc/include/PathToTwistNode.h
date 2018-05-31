/*
 * Created By: William Gu
 * Created On: Jan 20, 2018
 * Description: A path finding node which converts a given path into a Twist
 * message to send to the robot
 */

#ifndef PATHFINDING_IGVC_PATHFINDING_H
#define PATHFINDING_IGVC_PATHFINDING_H

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sb_utils.h"
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>

class PathToTwistNode {
  public:
    // The constructor
    PathToTwistNode(int argc, char** argv, std::string node_name);

    /**
     * Produces a twist message for the robot from the path message and
     * current robot coordinates + orientation
     * @param path_msg given path message to process
     * @param x_pos current robot x cood
     * @param y_pos current robot y cood
     * @param orientation current robot orientation in global frame
     * @param num_poses number of poses to process including initial robot pos
     * @return calculated path message
     */
    geometry_msgs::Twist pathToTwist(nav_msgs::Path path_msg,
                                            double x_pos,
                                            double y_pos,
                                            double orientation,
                                            bool valid_cood);

    /**
     * Adds geometric vector values to two empty vectors, based on contents of
     * an array of poses
     * @param poses : Array of pose messages
     * @param x_vectors Empty vector to represent x values of geometric vectors
     * @param y_vectors Empty vector to represent y values of geometric vectors
     * @param x_pos Current robot x position in global system
     * @param y_pos Current robot y position in global system
     */
    static void
    calcVectors(const std::vector<geometry_msgs::PoseStamped>& poses,
                std::vector<double>& x_vectors,
                std::vector<double>& y_vectors,
                double x_pos,
                double y_pos);

    /**
     * Calculates a weighted value given a std::vector of geometric vectors
     * The values near the front of the list are given a higher weight (since
     * they are imminent)
     * Currently the scaling for the weights are INVERSE
     * @param vectors represents geometric vector values for one dimension, i.e.
     * x
     * values in some geometric vectors
     * @param numToSum number of elements to sum, should be less than or equal
     * to
     * size of vectors
     * @return weighted values of given geometric vectors
     */
    static double weightedSum(const std::vector<double>& vectors, int path_dropoff_factor);

  private:
    /**
     * Produces twist msg based on path msg and publishes it
     * @param path_ptr
     */
    void pathCallBack(const nav_msgs::Path::ConstPtr& path_ptr);

    /**
     * Updates the current position and orientation of the robot in the global
     * frame,
     * and stores in member variables
     * @param tf_message
     */
    void tfCallBack(const tf2_msgs::TFMessageConstPtr tf_message);

    ros::Subscriber path_subscriber;
    ros::Subscriber tf_subscriber;
    ros::Publisher twist_publisher;

    std::string base_frame;   // The base frame of the robot ("base_link",
                              // "base_footprint", etc.)
    std::string global_frame; // The global frame ("map", "odom", etc.)

    /* Robot coordinates in global system */
    double robot_x_pos;
    double robot_y_pos;
    double robot_orientation;

    bool valid_cood; // Flag is set to true when we receive our first tf message

    double linear_speed_scaling_factor;
    double angular_speed_scaling_factor;

    double max_linear_speed;
    double max_angular_speed;

    int path_dropoff_factor; //larger number indicates more points in path to consider in path to twist
};

#endif // PATHFINDING_IGVC_PATHFINDING_H
