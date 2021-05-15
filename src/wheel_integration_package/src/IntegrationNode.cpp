/*
 * Created By: Vijeeth Vijhaipranith
 * Created On: Oct 30th, 2020
 * Description: This node subscribes to a topic publishing Geometry Twist
 * messages that indicate the direction to move the rover in,
 *              and publishes velocity values to the left and right wheels of
 * the rover.
 *              The velocity of the left wheels is published to the topic
 * "/integration_node/lwheels_pub_topic"
 *              The velocity of the right wheels is published to the topic
 * "/integration_node/rwheels_pub_topic"
 */

#include <IntegrationNode.h>

MyClass::MyClass(
int argc, char** argv, std::string node_name, float dist, float max_speed) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    distBetweenWheels = dist;
    maximum_speed     = max_speed;

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "cmd_vel";
    int queue_size                    = 10;
    my_subscriber                     = nh.subscribe(
    topic_to_subscribe_to, queue_size, &MyClass::subscriberCallBack, this);

    // Setup Publisher(s)
    std::string lwheels_topic = private_nh.resolveName("lwheels_pub_topic");
    int lwheels_queue_size    = 1;
    lwheels_publisher         = private_nh.advertise<geometry_msgs::Twist>(
    lwheels_topic, lwheels_queue_size);

    std::string rwheels_topic = private_nh.resolveName("rwheels_pub_topic");
    int rwheels_queue_size    = 1;
    rwheels_publisher         = private_nh.advertise<geometry_msgs::Twist>(
    rwheels_topic, rwheels_queue_size);
}

/*
 * Requires:
 *      -1.0 <= linear.x <= 1.0
 *      -Z_SENSITIVITY <= angular.x <= Z_SENSITIVITY
 * Returns:
 *      Publishes the linear velocity of left and right wheel in a Twist message
 */
void MyClass::subscriberCallBack(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("Received message");

    /* This variable is created to avoid division by 0 error */
    float zero_threshold = 0.01;

    /* Calculate the linear and angular velocities required by the Pro
     * Controller */
    float linear_vel  = maximum_speed * msg->linear.x;
    float angular_vel = msg->angular.z;

    float lwheel_vel, rwheel_vel;

    /*
     * If the angular velocty is 0, then the rover moves straight
     * and we set the velocities of both of its wheels equal to the
     * linear velocity required by the Pro Controller
     */
    if (abs(angular_vel) < zero_threshold) {
        lwheel_vel = linear_vel;
        rwheel_vel = linear_vel;
    } else {
        /* Calculate the radius of curvature of required by Pro Controller */
        float radius_of_curvature = linear_vel / angular_vel;

        /* Calculate the velocities of the left and right wheel according to the
         * formula on README.md */
        rwheel_vel =
        angular_vel * (radius_of_curvature + distBetweenWheels / 2);
        lwheel_vel =
        angular_vel * (radius_of_curvature - distBetweenWheels / 2);
    }

    geometry_msgs::Twist lwheels_msg;
    geometry_msgs::Twist rwheels_msg;

    /* Create Geometry Twist messages by assigning the velocities of the wheels
     * as the linear.x component */
    lwheels_msg.linear.x  = lwheel_vel;
    lwheels_msg.angular.z = 0.0;
    rwheels_msg.linear.x  = rwheel_vel;
    rwheels_msg.angular.z = 0.0;

    /* Publish the created Geometry Twist messages to their appropriate topics
     */
    lwheels_publisher.publish(lwheels_msg);
    rwheels_publisher.publish(rwheels_msg);

    ROS_INFO("Published messages");
}
