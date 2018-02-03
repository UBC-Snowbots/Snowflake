//
// Created by william on 20/01/18.
//

#ifndef PATHFINDING_IGVC_PATHFINDING_H
#define PATHFINDING_IGVC_PATHFINDING_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include "../../sb_utils/include/sb_utils.h"

class PathFinding {
public:
    // The constructor
    PathFinding(int argc, char **argv, std::string node_name);

private:
    void pathCallBack(const nav_msgs::Path::ConstPtr &path_ptr);
    void tfCallBack(const tf2_msgs::TFMessageConstPtr tf_message);

    geometry_msgs::Twist pathToTwist(nav_msgs::Path path_msg);
    void calcVectors(const std::vector<geometry_msgs::PoseStamped> &poses, std::vector<float> &x_vectors, std::vector<float> &y_vectors, int num_vectors);
    float weightedXSum (std::vector<float> &x_vectors, int numToSum);
    float weightedYSum (std::vector<float> &y_vectors, int num_to_sum);

    ros::Subscriber path_subscriber;
    ros::Subscriber tf_subscriber;
    ros::Publisher twist_publisher;

    std::string base_frame;   // The base frame of the robot ("base_link", "base_footprint", etc.)
    std::string global_frame; // The global frame ("map", "odom", etc.)

    double robotXPos;
    double robotYPos;
    double robotOrientation;


};


#endif //PATHFINDING_IGVC_PATHFINDING_H
