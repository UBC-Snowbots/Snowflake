#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

class TurtleController {
public:
    TurtleController();
private:
    void poseCallBack(const turtlesim::Pose::ConstPtr& msg);
    void sendNewCommand();

    ros::Subscriber turtle_pose;
    ros::Publisher cmd_vel_pub;

    turtlesim::Pose::ConstPtr current_pose;
};

// Constructor for the turtle controller
TurtleController::TurtleController(){
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    turtle_pose = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 10, &TurtleController::poseCallBack, this);

    ros::Rate loop_rate(10);
}

// This function is called when the turtle publishes a position
void TurtleController::poseCallBack(const turtlesim::Pose::ConstPtr &msg) {
    current_pose = msg;
    sendNewCommand();
}

// This function sends a new command to the turtle
void TurtleController::sendNewCommand(){
    // Create the command
    geometry_msgs::Twist command;

    // Figure out your turn and move values
    command.linear.x = 0.5;
    command.angular.z = sin(5 - current_pose->y);

    // Print some info for debugging purposes
    ROS_INFO("Turn Val: %f", command.angular.z);

    // Publish the command
    cmd_vel_pub.publish(command);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "solution");
    ros::NodeHandle nh;

    TurtleController turtleController;

    ros::spin();

    return 0;
}