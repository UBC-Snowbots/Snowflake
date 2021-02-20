/*
 * Created By: Vijeeth Vijhaipranith
 * Created On: Oct 30th, 2020
 * Description: An example node that subscribes to a topic publishing strings,
 *              and re-publishes everything it receives to another topic with
 *              a "!" at the end
 */

#include <IntegrationNode.h>


MyClass::MyClass(int argc, char **argv, std::string node_name, float dist, float max_speed) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    distBetweenWheels = dist;
    maximum_speed = max_speed;

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "cmd_vel";
    int queue_size = 10;
    my_subscriber = nh.subscribe(topic_to_subscribe_to, queue_size, &MyClass::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string lwheels_topic = private_nh.resolveName("lwheels_pub_topic");
    int lwheels_queue_size = 1;
    lwheels_publisher = private_nh.advertise<geometry_msgs::Twist>(lwheels_topic, lwheels_queue_size);

    std::string rwheels_topic = private_nh.resolveName("rwheels_pub_topic");
    int rwheels_queue_size = 1;
    rwheels_publisher = private_nh.advertise<geometry_msgs::Twist>(rwheels_topic, rwheels_queue_size);
}

/*
 * Requires:
 *      -1.0 <= linear.x 1.0
 *      -Z_SENSITIVITY <= angular.x Z_SENSITIVITY
 * Returns:
 *      Publishes the linear velocity of left and right wheel in a Twist message
 */
void MyClass::subscriberCallBack(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received message");

    float zero_threshold = 0.01;

    float linear_vel = maximum_speed * msg->linear.x;
    float angular_vel = msg->angular.z;

    float lwheel_vel, rwheel_vel;

    if(abs(angular_vel) < zero_threshold) {
        lwheel_vel = linear_vel;
        rwheel_vel = linear_vel;
    }
    else {
        float radius_of_curvature = linear_vel/angular_vel;

        rwheel_vel = angular_vel * (radius_of_curvature + distBetweenWheels/2);
        lwheel_vel = angular_vel * (radius_of_curvature - distBetweenWheels/2);
    }

    geometry_msgs::Twist lwheels_msg;
    geometry_msgs::Twist rwheels_msg;

    lwheels_msg.linear.x = lwheel_vel;
    lwheels_msg.angular.z = 0.0;
    rwheels_msg.linear.x = rwheel_vel;
    rwheels_msg.angular.z = 0.0;

    lwheels_publisher.publish(lwheels_msg);
    rwheels_publisher.publish(rwheels_msg);

    ROS_INFO("Published messages");
}



