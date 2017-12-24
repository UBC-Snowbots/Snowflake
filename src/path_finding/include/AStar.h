/*
 * Created By: Marcus Swift
 * Created On: December 18th, 2017
 * Description: An A Star optimal path finding node
 * takes in a occupancy grid and goal point and returns
 * the optimal path using an A star algorithm
 */

#ifndef PATH_FINDING_ASTAR_H
#define PATH_FINDING_ASTAR_H

#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <functional>
#include <queue>
#include <vector>
#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <sb_utils.h>
#include <cmath>


using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using std::vector;

class AStar {
	
private:
	//Subscriber and publisher data types
	nav_msgs::OccupancyGrid occupancy_grid_data;
	nav_msgs::Path optimal_path;
	geometry_msgs::Point goal_data;
	//Subscribers and Publishers
    ros::Subscriber occupancy_grid_sub;
	ros::Subscriber goal_sub;
    ros::Publisher path_pub;

    /**
     * Callback function for recieving an updated occupancy grid
     *
     * @param the occupancy grid recieved in the callback
     */
    void occupancyGridCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);
	/**
     * Callback function for recieving a new goal position
     *
     * @param the goal position received in the callback
     */
    void goalCallBack(const geometry_msgs::Point::ConstPtr& msg);
    /**
     * Publishes an optimal path to the goal
     *
     * @param the optimal path that needs to be published
     */
    void publishOptimalPath(nav_msgs::Path path_to_publish);
	
public:
	//Constructor
    AStar(int argc, char **argv, std::string node_name);
    
	//function to convert occupancy into a more useful matrix
	
	//a function to retrieve the current position
	
	
	
	
	/**
     * Adds an exclamation point to a given string
     *
     * Some Longer explanation should go here
     *
     * @param input_string the string to add an exclamation point to
     *
     * @return input_string with an exclamation point added to it
     */
     static std::string addCharacterToString(std::string input_string, std::string suffix);
     std::string suffix;

};
#endif //PATH_FINDING_ASTAR_H
