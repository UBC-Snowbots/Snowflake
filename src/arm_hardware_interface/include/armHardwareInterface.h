#ifndef ARM_HARDWARE_INTERFACE_H
#define ARM_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include "arm_hardware_driver/armHardwareDriver.h"

#include <chrono>
#include <thread>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;


	class ArmHardwareInterface: public hardware_interface::RobotHW
	{
		public:
			ArmHardwareInterface(int argc, char** argv, std::string node_name);
			~ArmHardwareInterface();

			void init();
			void update(const ros::TimerEvent& e);
			void read();
			void write(ros::Duration elapsed_time);

		private:
			ros::NodeHandle nh_;
			ros::Timer non_realtime_loop_;
			ros::Duration control_period_;
			ros::Duration elapsed_time_;
			PositionJointInterface positionJointInterface;
			PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
			double loop_hz_;
			boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
			double p_error_, v_error_, e_error_;

			// Motor driver
			ArmHardwareDriver driver_;
			std::vector<double> actuator_commands_;
			std::vector<double> actuator_positions_;

			// Interfaces
			hardware_interface::JointStateInterface joint_state_interface_;
			hardware_interface::PositionJointInterface position_joint_interface_;

			joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface_;
			joint_limits_interface::PositionJointSoftLimitsInterface position_joint_limits_interface_;

			// Shared memory
			int num_joints_;
			std::vector<std::string> joint_names_;
			std::vector<double> joint_offsets_;
			std::vector<double> joint_positions_;
			std::vector<double> joint_velocities_;
			std::vector<double> joint_efforts_;
			std::vector<double> joint_position_commands_;
			std::vector<double> joint_velocity_commands_;
			std::vector<double> joint_effort_commands_;
			std::vector<double> joint_lower_limits_;
			std::vector<double> joint_upper_limits_;
			std::vector<double> velocity_limits_;
			std::vector<double> acceleration_limits_;

			std::Boolean xbox_mode = false;

			// Misc
			double degToRad(double deg);
			double radToDeg(double rad);

			void controllerModeCallBack(const std_msgs::Bool::ConstPtr& inMsg)

			ros::Subscriber subMode;
	};

#endif // ARM_HARDWARE_INTERFACE_H
