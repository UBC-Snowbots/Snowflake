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

class PathFinding {
public:
    // The constructor
    PathFinding(int argc, char** argv, std::string node_name);



private:
    void PathFinding::pathCallBack(const nav_msgs::Path::ConstPtr& path_ptr);
    geometry_msgs::Twist PathFinding::pathToTwist(nav_msgs::Path path_msg);
};



#endif //PATHFINDING_IGVC_PATHFINDING_H
