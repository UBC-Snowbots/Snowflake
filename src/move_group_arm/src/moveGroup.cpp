/*
 * Created By: Tate Kolton
 * Created On: August 9, 2022
 * Description: Uses move group interface to update current joint pose in moveit when switching
 * from joint space mode to cartesian mode (IK)
 */

#include "../include/moveGroup.h"

MoveGroupArm::MoveGroupArm(int argc, char** argv, string node_name) 
{

  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle;
  subPos = 
  node_handle.subscribe("/observed_pos_arm", 1, &updatePose, this); 
  subExecute = 
  node_handle.subscribe("/move_group_trigger", 1, &executePose, this);

  init();
}

void MoveGroupArm::executePose(const std_msgs::Bool::ConstPtr& inMsg) {

  // fetches current state of rover
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // writes new position command
  for(int i=0; i<num_joints_; i++)
  {
    joint_positions_[i] = degToRad(actuator_positions_[i]);
    joint_group_positions[i] = joint_positions_[i];
  }

  // plans movement
  move_group.setJointValueTarget(joint_group_positions);
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if(success)
  {
    ROS_INFO("Target pose successfully planned");
  }
  else
  {
    ROS_INFO("Target pose failed");
  }

  // executes current arm position in move group interface
  move_group.move();
}

void MoveGroupArm::updatePose(const sb_msgs::ArmPosition::ConstPtr& observed_msg)
{
  actuator_positions_.assign(observed_msg->positions.begin(), observed_msg->positions.end());
}

void MoveGroupArm::init()
{
  actuator_positions_.resize(num_joints_);
  joint_positions_.resize(num_joints_);
  joint_group_positions.resize(num_joints_);
  static const std::string PLANNING_GROUP = "arm";

  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
}

double ArmHardwareInterface::degToRad(double deg) 
{
  return deg / 180.0 * 3.14159;
}