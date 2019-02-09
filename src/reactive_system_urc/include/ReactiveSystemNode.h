/**
 * Created by William Gu on Sept 29 2018
 * Class declaration for Reactive System Node
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
    float traj_time_inc; // size of unit increments in a trajectory calculation
    int
    traj_num_incs; // number of total increments used in trajectory calculation
    float linear_vel; // current linear vel of robot (needs to be set iniitally)
    float max_angular_vel;  // max turning velocity
    int num_angular_vel;    // number of possible turning velocities to consider
    float risk_dist_tol_sq; // squared "radius" of the robot

    sb_geom_msgs::Point2D goal_pos; // current goal position to head towards
};

#endif // REACTIVE_SYSTEM_NODE
