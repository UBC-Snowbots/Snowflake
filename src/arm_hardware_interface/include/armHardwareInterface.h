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
#include <sb_msgs/ArmPosition.h>
#include <std_msgs/Bool.h>

#include <chrono>
#include <thread>

using namespace hardware_interface;

	class ArmHardwareInterface: public hardware_interface::RobotHW
	{
		public:
			ArmHardwareInterface(ros::NodeHandle& nh);
			~ArmHardwareInterface();

			void init();
			void read();
			void write(ros::Duration elapsed_time);
			void cmdArmPosition(const ros::TimerEvent& e);

		private:
			ros::NodeHandle nh;
			ros::Timer non_realtime_loop_;
			ros::Duration control_period_;
			ros::Duration elapsed_time_;
			ros::Time previous_time_;
			double loop_hz_;
			boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

			// Motor driver
			std::vector<double> actuator_commands_;
			std::vector<double> actuator_positions_;

			// Interfaces
			hardware_interface::JointStateInterface joint_state_interface_;
			hardware_interface::PositionJointInterface position_joint_interface_;

			// Shared memory
			int num_joints_;
			std::vector<std::string> joint_names_;
			std::vector<double> joint_offsets_;
			std::vector<double> joint_positions_;
			std::vector<double> joint_velocities_;
			std::vector<double> joint_efforts_;
			std::vector<double> joint_position_commands_;

			bool cartesian_mode = false;

			// Misc
			double degToRad(double deg);
			double radToDeg(double rad);

			void controllerModeCallBack(const std_msgs::Bool::ConstPtr& inMsg);
			void armPositionCallBack(const sb_msgs::ArmPosition::ConstPtr& observed_Msg);

			ros::Subscriber subMode;
			ros::Subscriber sub_arm_pos;
			ros::Publisher pub_arm_pos;
	};

#endif // ARM_HARDWARE_INTERFACE_H
