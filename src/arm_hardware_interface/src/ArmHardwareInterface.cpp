/*
 * Created By: Ihsan Olawale, Tate Kolton
 * Created On: June 28th, 2022
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <ArmHardwareInterface.h>

ArmHardwareInterface::ArmHardwareInterface(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle private_nh("~");
    SB_getParam(private_nh, "num_joints", num_joints, 7);

    commands.resize(num_joints);
    velocities.resize(num_joints);
    positions.resize(num_joints);
    efforts.resize(num_joints);
    for (int i = 0; i < num_joints; ++i) {
        hardware_interface::JointStateHandle state_handle(""+i, &positions[i], &velocities[i], &commands[i]);
        joint_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle position_handle(joint_state_interface.getHandle(""+i), &commands[0]);
        position_state_interface.registerHandle(position_handle);
    }
    registerInterface(&joint_state_interface);
    registerInterface(&position_state_interface);
}

void ArmHardwareInterface::read(const ros::Time& time, const ros::Duration& period) {
}

void ArmHardwareInterface::write(const ros::Time& time, const ros::Duration& period) {
}
