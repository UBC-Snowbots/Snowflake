#include <sstream>
#include <arm_hardware_interface/armHardwareInterface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>


using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::SoftJointLimits;

ArmHardwareInterface::ArmHardwareInterface(int argc, char** argv, string node_name)
{
	ros::init(argc, argv, node_name);
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");
	string modeSubscriber = "/moveit_toggle";
	subMode                    = nh.subscribe(
    modeSubscriber, 10, &ArmHardwareInterface::controllerModeCallBack, this);
        string arm_pos_subscriber = "/observed_pos_arm";
        sub_arm_pos = nh.subscribe(arm_pos_subscriber, 10, &ArmHardwareInterface::armPositionCallBack, this);
        string arm_pos_publisher = "/cmd_pos_arm";
        pub_arm_pos = private_nh.advertise<sb_msgs::ArmPosition>(arm_pos_publisher, 1);

	init();

	// init ros controller manager
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

	nh_.param("/arm/hardware_interface/loop_hz", loop_hz_, 0.1);
	ROS_DEBUG_STREAM_NAMED("constructor", "Using loop freqency of " << loop_hz_ << " hz");
	ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
	non_realtime_loop_ = nh_.createTimer(update_freq, &ArmHardwareInterface::cmdArmPosition, this);

	// initialize controller
	for (int i = 0; i < num_joints_; ++i)
	{
		ROS_DEBUG_STREAM_NAMED("constructor", "Loading joint name: " << joint_names_[i]);

		// Create joint state interface
		JointStateHandle jointStateHandle(joint_names_[i], &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]);
		joint_state_interface_.registerHandle(jointStateHandle);

		// Create position joint interface
		JointHandle jointPositionHandle(jointStateHandle, &joint_position_commands_[i]);
		position_joint_interface_.registerHandle(jointPositionHandle);
	}

	// get encoder calibration
	std::vector<double> enc_steps_per_deg(num_joints_);

	for (int i = 0; i < num_joints_; ++i)
	{
		if(!nh_.getParam("/arm/hardware_driver/encoder_steps_per_deg/" + joint_names_[i], enc_steps_per_deg[i]))
		{
			ROS_WARN("Failed to get params for /arm/hardware_driver/encoder_steps_per_deg/");
		}

		if (!nh_.getParam("/arm/hardware_interface/joint_offsets/" + joint_names_[i], joint_offsets_[i]))
		{
			ROS_WARN("Failed to get params for /arm/hardware_interface/joint_offsets/");
		}

	}

	// set velocity limits
	for (int i = 0; i < num_joints_; ++i)
	{
		nh_.getParam("/arm/joint_limits/" + joint_names_[i] + "/max_velocity", velocity_limits_[i]);
		nh_.getParam("/arm/joint_limits/" + joint_names_[i] + "/max_acceleration", acceleration_limits_[i]);
		velocity_limits_[i] = radToDeg(velocity_limits_[i]);
		acceleration_limits_[i] = radToDeg(acceleration_limits_[i]);
	}

	for (int i = 0; i < num_joints_; ++i)
	{
		// assign zero angles initially
		joint_positions_[i] = 0;
		joint_position_commands_[i] = 0;
	}

	registerInterface(&joint_state_interface_);
	registerInterface(&position_joint_interface_);
}

ArmHardwareInterface::~ArmHardwareInterface()
{
}

void ArmHardwareInterface::controllerModeCallBack(const std_msgs::Bool::ConstPtr& inMsg)
{
	cartesian_mode = inMsg->data;
    if (cartesian_mode)
        ROS_INFO("Enabling Cartesian Mode");
    else
        ROS_INFO("Disabling Cartesian Mode");
}

void ArmHardwareInterface::armPositionCallBack(const sb_msgs::ArmPosition::ConstPtr& observed_msg) {
    
    // TODO: ihsan implement snowbots message type
    //actuator_positions_ = ___________ 

    if (!cartesian_mode) 
    {
	    for (int i = 0; i < num_joints_; ++i)
		{
			// apply offsets, convert from deg to rad for moveit
			joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
			joint_position_commands_[i] = joint_positions_[i];
		}
    }
    else
    {
    	read();
    	controller_manager_->update(ros::Time::now(), elapsed_time_);
    }
}

void ArmHardwareInterface::cmdArmPosition(const ros::TimerEvent &e)
{
	elapsed_time_ = ros::Duration(e.current_real - e.last_real);
	write(elapsed_time_);

	// TODO: ihsan inspect lines below
	// sb_msgs::ArmPosition cmdPos = ________
	// pub_arm_pos.publish(cmdPos);
}

void ArmHardwareInterface::init()
{
	// get joint names
	nh_.getParam("/arm/hardware_interface/joints", joint_names_);
	if (joint_names_.size() == 0)
	{
		ROS_FATAL_STREAM_NAMED("init", "No joints found on parameter server for controller. Did you load the proper yaml file?");
	}
	num_joints_ = static_cast<int>(joint_names_.size());

	// resize vectors
	actuator_commands_.resize(num_joints_);
	actuator_positions_.resize(num_joints_);
	joint_positions_.resize(num_joints_);
	joint_velocities_.resize(num_joints_);
	joint_efforts_.resize(num_joints_);
	joint_position_commands_.resize(num_joints_);
	joint_velocity_commands_.resize(num_joints_);
	joint_effort_commands_.resize(num_joints_);
	joint_offsets_.resize(num_joints_);
	joint_lower_limits_.resize(num_joints_);
	joint_upper_limits_.resize(num_joints_);
	velocity_limits_.resize(num_joints_);
	acceleration_limits_.resize(num_joints_);
}

/* deprecated, for reference only

void ArmHardwareInterface::update(const ros::TimerEvent &e)
{

	if(cartesian_mode)
	{
		std::string logInfo = "\n";
		logInfo += "Joint Position Command:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			std::ostringstream jointPositionStr;
			jointPositionStr << radToDeg(joint_position_commands_[i]);
			logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str() + "\n";
		}

		elapsed_time_ = ros::Duration(e.current_real - e.last_real);
		
		write(elapsed_time_);
		read();

		logInfo += "Joint Position State:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			std::ostringstream jointPositionStr;
			jointPositionStr << radToDeg(joint_positions_[i]);
			logInfo += "  " + joint_names_[i] + ": " + jointPositionStr.str() + "\n";
		}

		controller_manager_->update(ros::Time::now(), elapsed_time_);

		ROS_INFO_STREAM(logInfo);
	}
}

*/

void ArmHardwareInterface::read()
{		
	// TODO: assign observed_msg components to actuator_positions_

	for (int i = 0; i < num_joints_; ++i)
	{
		// apply offsets, convert from deg to rad for moveit
		joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
	}
}

void ArmHardwareInterface::write(ros::Duration elapsed_time)
{
	for (int i = 0; i < num_joints_; ++i)
	{
		// convert from rad to deg, apply offsets
		actuator_commands_[i] = radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
	}
}

double ArmHardwareInterface::degToRad(double deg)
{
	return deg / 180.0 * M_PI;
}

double ArmHardwareInterface::radToDeg(double rad)
{
	return rad / M_PI * 180.0;
}
