#include <ros/callback_queue.h>
#include <arm_hardware_interface/arm_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_hardware_interface");
	ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
	nh.setCallbackQueue(&ros_queue);
  arm_hardware_interface::ArmHardwareInterface ar3(nh);

	ros::MultiThreadedSpinner spinner(0);
	spinner.spin(&ros_queue);

  return 0;
}
