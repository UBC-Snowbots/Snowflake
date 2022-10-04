#include <armHardwareInterface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <sstream>

ArmHardwareInterface::ArmHardwareInterface(ros::NodeHandle& nh) : nh(nh) {
    // ros::NodeHandle private_nh("~");
    std::string modeSubscriber = "/moveit_toggle";
    subMode                    = nh.subscribe(
    modeSubscriber, 10, &ArmHardwareInterface::controllerModeCallBack, this);
    std::string arm_pos_subscriber = "/observed_pos_arm";
    sub_arm_pos                    = nh.subscribe(
    arm_pos_subscriber, 10, &ArmHardwareInterface::armPositionCallBack, this);
    std::string arm_pos_publisher = "/cmd_pos_arm";
    pub_arm_pos = nh.advertise<sb_msgs::ArmPosition>(arm_pos_publisher, 1);

    ROS_INFO("1");

    nh.getParam("/hardware_interface/joints", joint_names_);
    if (joint_names_.size() == 0) {
        ROS_FATAL_STREAM_NAMED(
        "init",
        "No joints found on parameter server for controller. Did you load the "
        "proper yaml file?");
    }
    init();

    // init ros controller manager
    controller_manager_.reset(
    new controller_manager::ControllerManager(this, nh));

    nh.param("/hardware_interface/loop_hz", loop_hz_, 0.1);
    ROS_DEBUG_STREAM_NAMED("constructor",
                           "Using loop freqency of " << loop_hz_ << " hz");
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);
    non_realtime_loop_ =
    nh.createTimer(update_freq, &ArmHardwareInterface::cmdArmPosition, this);

    // initialize controller
    for (int i = 0; i < num_joints_; ++i) {
        ROS_DEBUG_STREAM_NAMED("constructor",
                               "Loading joint name: " << joint_names_[i]);

        // Create joint state interface
        JointStateHandle jointStateHandle(joint_names_[i],
                                          &joint_positions_[i],
                                          &joint_velocities_[i],
                                          &joint_efforts_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        JointHandle jointPositionHandle(jointStateHandle,
                                        &joint_position_commands_[i]);
        position_joint_interface_.registerHandle(jointPositionHandle);
    }

    for (int i = 0; i < num_joints_; ++i) {
        if (!nh.getParam("/hardware_interface/joint_offsets/" + joint_names_[i],
                         joint_offsets_[i])) {
            ROS_WARN(
            "Failed to get params for /hardware_interface/joint_offsets/%s",
            joint_names_[i].c_str());
            joint_offsets_[i] = 0;
        }
    }

    for (int i = 0; i < num_joints_; ++i) {
        // assign zero angles initially
        joint_positions_[i]         = 0;
        joint_position_commands_[i] = 0;
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
}

ArmHardwareInterface::~ArmHardwareInterface() {}

void ArmHardwareInterface::controllerModeCallBack(
const std_msgs::Bool::ConstPtr& inMsg) {
    cartesian_mode = inMsg->data;
    if (cartesian_mode)
        ROS_INFO("Enabling Cartesian Mode");
    else {
        ROS_INFO("Disabling Cartesian Mode");
        previous_time_ = ros::Time::now();
    }
}

void ArmHardwareInterface::armPositionCallBack(
const sb_msgs::ArmPosition::ConstPtr& observed_msg) {
    // TODO: ihsan implement snowbots message type
    // actuator_positions_ = ___________
    ROS_INFO("Received new message");
    actuator_positions_.assign(observed_msg->positions.begin(),
                               observed_msg->positions.end());

    if (!cartesian_mode) {
        elapsed_time_ = ros::Duration(observed_msg->header.stamp - previous_time_);
        previous_time_ = observed_msg->header.stamp;
        for (int i = 0; i < num_joints_; ++i) {
            // fake controller, set position state to equal command state
            joint_positions_[i] = joint_position_commands_[i];
        }
        controller_manager_->update(ros::Time::now(), elapsed_time_);
    } else {
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
    }
}

void ArmHardwareInterface::cmdArmPosition(const ros::TimerEvent& e) {
    if (cartesian_mode) {
        ROS_INFO("-I- Timer Initiated Position Exchange");

        for(int i=0; i<num_joints_; ++i)
        {
            if(abs(joint_positions_[i] - joint_position_commands_[i]) < 10)
            {
                joint_position_commands_[i] = joint_positions_[i];
            }
        }

        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        write(elapsed_time_);
        sb_msgs::ArmPosition cmdPos;
        cmdPos.positions.assign(actuator_commands_.begin(),
                actuator_commands_.end()); pub_arm_pos.publish(cmdPos);
    }
}

void ArmHardwareInterface::init() {
    // get joint names
    num_joints_ = static_cast<int>(joint_names_.size());

    // resize vectors
    actuator_commands_.resize(num_joints_);
    actuator_positions_.resize(num_joints_);
    joint_positions_.resize(num_joints_);
    joint_velocities_.resize(num_joints_);
    joint_efforts_.resize(num_joints_);
    joint_position_commands_.resize(num_joints_);
    joint_offsets_.resize(num_joints_);
}

void ArmHardwareInterface::read() {
    // TODO: assign observed_msg components to actuator_positions_

    for (int i = 0; i < num_joints_; ++i) {
        // apply offsets, convert from deg to rad for moveit
        joint_positions_[i] =
        degToRad(actuator_positions_[i] + joint_offsets_[i]);
    }
}

void ArmHardwareInterface::write(ros::Duration elapsed_time) {
    for (int i = 0; i < num_joints_; ++i) {
        // convert from rad to deg, apply offsets
        actuator_commands_[i] =
        radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
    }
}

double ArmHardwareInterface::degToRad(double deg) {
    return deg / 180.0 * M_PI;
}

double ArmHardwareInterface::radToDeg(double rad) {
    return rad / M_PI * 180.0;
}
