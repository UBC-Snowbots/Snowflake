#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

class MoveGroupArm {
    public:
        MoveGroupArm(int argc, char** argv, string node_name);
        
    private: 
        void updatePose(const sb_msgs::ArmPosition::ConstPtr& observed_msg);
        void toggleMode(const std_msgs::Bool::ConstPtr& inMsg);
        double degToRad(double deg);
        void init();

        ros::Subscriber curPos;
        ros::Subscriber curMode;

        bool cartesian_mode = false;
        std::vector<double> actuator_positions_;
        std::vector<double> joint_positions_;
        std::vector<double> joint_group_positions;
        int num_joints_ = 6;
}