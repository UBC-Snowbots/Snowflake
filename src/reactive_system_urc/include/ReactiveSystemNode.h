/**
 * Created by William Gu on Sept 29 2018
 * Class declaration for Reactive System Node
 * This node is a wrapper for ReactiveSystemPath logic, which subscribes to twist, current GPS goal, and risk area
 * nodes, and publishes a path message to head towards the goal while avoiding risky areas
 */

#ifndef REACTIVE_SYSTEM_NODE_H
#define REACTIVE_SYSTEM_NODE_H

#include "ReactiveSystemPath.h"
#include <iostream>
#include <mapping_msgs_urc/RiskArea.h>
#include <mapping_msgs_urc/RiskAreaArray.h>
#include <mapping_msgs_urc/RiskAreaStamped.h>
#include <ros/ros.h>
#include <sb_utils.h>

class ReactiveSystemNode {
  public:
    ReactiveSystemNode(int argc, char** argv, std::string node_name);

  private:
    ros::Subscriber risk_subscriber;
    ros::Subscriber goal_subscriber;
    ros::Publisher path_publisher;

    /**
    * Callback function for receiving an array of risks
    * Publishes an optimal twist command for the robot, avoiding risky areas
    * while tending towards the goal
    * @param ptr
    */
    void riskCallBack(const mapping_msgs_urc::RiskAreaArray::ConstPtr& ptr);

    /**
     * Callback for updating goal position
     * @param ptr
     */
    void goalCallBack(const sb_geom_msgs::Point2D::ConstPtr& ptr);

    /* ros params */
    // size of unit increments in a trajectory calculation
    float traj_time_inc;
    // number of total increments used in trajectory calculation
    int traj_num_incs;
    // current linear vel of robot (needs to be set initally)
    float linear_vel;
    // max turning velocity
    float max_angular_vel;
    // number of possible turning velocities to consider
    int num_angular_vel;
    // squared "radius" of the robot
    float risk_dist_tol_sq;

    // current goal position to head towards
    sb_geom_msgs::Point2D goal_pos;
};

#endif // REACTIVE_SYSTEM_NODE
