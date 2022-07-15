#include <arm_hardware_interface/armHardwareInterface.h>

int main(int argc, char** argv)
{

	std::string node_name = arm_hardware_interface

	ArmHardwareInterface moveitArm(argc, argv, node_name);

	ros::spin();

	return 0;
}
