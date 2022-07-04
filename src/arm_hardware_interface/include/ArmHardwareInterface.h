/*
 * Created By: Ihsan Olawale, Tate Kolton
 * Created On: June 28th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#ifndef ARM_HARDWARE_INTERFACE_ARMHARDWAREINTERFACE_H
#define ARM_HARDWARE_INTERFACE_ARMHARDWAREINTERFACE_H

// STD Includes
#include <vector>

// ROS Includes
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Snowbots Includes
#include <sb_utils.h>

class ArmHardwareInterface : public hardware_interface::RobotHW {
public:
    ArmHardwareInterface(int argc, char **argv, std::string node_name);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

private:
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface position_state_interface;
    std::vector<double> commands;
    std::vector<double> velocities;
    std::vector<double> positions;
    std::vector<double> efforts;
    std::vector<hardware_interface::JointStateHandle> joint_state_handles;
    std::vector<hardware_interface::JointHandle> joint_handles;
    int num_joints;
};
#endif //ARM_HARDWARE_INTERFACE_ARMHARDWAREINTERFACE_H
