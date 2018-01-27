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

class PathFinding {
public:
    // The constructor
    PathFinding(int argc, char **argv, std::string node_name);

private:
    void pathCallBack(const nav_msgs::Path::ConstPtr &path_ptr);
    geometry_msgs::Twist pathToTwist(nav_msgs::Path path_msg);
    void calcVectors(const std::vector<geometry_msgs::PoseStamped> &poses, std::vector<float> &x_vectors, std::vector<float> &y_vectors, int num_vectors);
    float weightedXSum (std::vector<float> &x_vectors, int numToSum);
    float weightedYSum (std::vector<float> &y_vectors, int num_to_sum);

};


#endif //PATHFINDING_IGVC_PATHFINDING_H
