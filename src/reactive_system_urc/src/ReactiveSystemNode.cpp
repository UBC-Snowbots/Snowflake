/**
 * Created by William Gu on Sept 29 2018
 * Implementation for Reactive System Node
 */

#include <ReactiveSystemNode.h>

ReactiveSystemNode::ReactiveSystemNode(int argc,
                                     char** argv,
                                     std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    uint32_t queue_size = 1;


    /* Get ros params */

    std::string traj_time_inc_param = "traj_time_inc";
    float default_traj_time_inc    = 0.01;
    SB_getParam(
            private_nh, traj_time_inc_param, traj_time_inc, default_traj_time_inc);

    std::string traj_num_incs_param = "traj_num_incs";
    int default_traj_num_incs    = 5;
    SB_getParam(
            private_nh, traj_num_incs_param, traj_num_incs, default_traj_num_incs);

    std::string linear_vel_param = "linear_vel";
    float default_linear_vel    = 5;
    SB_getParam(
            private_nh, linear_vel_param, linear_vel, default_linear_vel);

    std::string max_angular_vel_param = "max_angular_vel";
    float default_max_angular_vel    = 5;
    SB_getParam(
            private_nh, max_angular_vel_param, max_angular_vel, default_max_angular_vel);

    std::string num_angular_vel_param = "num_angular_vel";
    int default_num_angular_vel    = 10;
    SB_getParam(
            private_nh, num_angular_vel_param, num_angular_vel, default_num_angular_vel);

    std::string risk_dist_tol_sq_param = "risk_dist_tol_sq";
    float default_risk_dist_tol_sq    = 10;
    SB_getParam(
            private_nh, risk_dist_tol_sq_param, risk_dist_tol_sq, default_risk_dist_tol_sq);

    //Initial goal is at current position
    sb_geom_msgs::Point2D initial_goal;
    initial_goal.x = 0;
    initial_goal.y = 0;
    goal_pos = initial_goal;


    /* Setup subscribers and publishers */

    std::string risk_topic =
            "risk_areas";  //TODO: update topic
    risk_subscriber = nh.subscribe(
            risk_topic, queue_size, &ReactiveSystemNode::riskCallBack, this);

    std::string goal_topic =
            "goal";  //TODO: update topic
    goal_subscriber = nh.subscribe(
            goal_topic, queue_size, &ReactiveSystemNode::goalCallBack, this);

    std::string twist_topic = "twist"; // TODO: update placeholder
    twist_publisher = private_nh.advertise<geometry_msgs::Twist>(
            twist_topic, queue_size);


}

void ReactiveSystemNode::riskCallBack(const mapping_msgs_urc::RiskAreaArray::ConstPtr& ptr) {
    mapping_msgs_urc::RiskAreaArray risk_areas = *ptr;

    geometry_msgs::Twist twist_msg = ReactiveSystemTwist::getTwist(risk_areas,
            goal_pos, traj_time_inc, traj_num_incs, linear_vel, max_angular_vel,
            num_angular_vel, risk_dist_tol_sq);

    twist_publisher.publish(twist_msg);
}

void ReactiveSystemNode::goalCallBack(const sb_geom_msgs::Point2D::ConstPtr& ptr) {
    sb_geom_msgs::Point2D goal = *ptr;
    goal_pos = goal; //update current goal

}