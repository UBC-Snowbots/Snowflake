/*
 * Created By: Gareth Ellis
 * Created On: March 29th, 2018
 * Description:
 */


// Snowbots Includes
#include <EncoderOdometryNode.h>

// GTest Includes
#include <gtest/gtest.h>

// ROS Includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>


/**
 * This is the helper class which will publish and subscribe messages which will test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the subscriber
 *      getter function - to provide a way for gtest to check for equality of the message recieved
 */
class EncoderOdometryNodeTest : public testing::Test{
protected:
    virtual void SetUp(){
        // TODO: Delete me
        //test_publisher = nh_.advertise<std_msgs::String>("subscribe_topic", 1);
        //test_subscriber = nh_.subscribe("/my_node/publish_topic", 1, &EncoderOdometryNodeTest::odomMsgCallback, this);
        // The publisher for our JointState message containing the encoder ticks

        // Setup Publishers and Subscribers
        encoder_joint_state_publisher = nh.advertise<sensor_msgs::JointState>("/encoders/joint_states", 10);
        // Note: We seem to need to set the queue size to be really large here in order to get the most recent
        //      odometry estimate....
        odom_msg_subscriber = nh.subscribe("/encoders/odom", 5000, &EncoderOdometryNodeTest::odomMsgCallback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    /**
     * The callback function for received Odometry messages
     * @param odom_msg_ptr
     */
    void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& odom_msg_ptr){
        odom_msg = *odom_msg_ptr;
        std::cerr << "Got Into callback: " << std::endl << odom_msg.pose.pose << std::endl;
    }

    /**
     * Simulate moving the robot
     *
     * @param left_ticks the number of ticks to move the left wheel
     * @param right_ticks the number of ticks to move the right wheel
     * @param dt the time over which to perform the given movement
     */
    void driveSimulator(int left_ticks, int right_ticks, double dt){
        // We want to increment at least as fast as we're ticking
        double time_increment = dt/std::max(std::abs(left_ticks), std::abs(right_ticks));

        // The time between left and right wheel ticks
        double left_wheel_tick_time_increment = dt/std::abs(left_ticks);
        double right_wheel_tick_time_increment = dt/std::abs(right_ticks);

        // The last time we incremented the left and right wheel ticks
        double last_left_wheel_increment_time = ros::Time::now().toSec();
        double last_right_wheel_increment_time = ros::Time::now().toSec();

        // The current number of ticks of the left and right wheel encoders
        int left_encoder_tick_count = 0;
        int right_encoder_tick_count = 0;

        double start_time = ros::Time::now().toSec();
        double end_time = start_time + dt + time_increment;
        while(ros::Time::now().toSec() < end_time){
            double curr_time = ros::Time::now().toSec();

            // Check if we should increment the left or right wheels
            if ((curr_time - last_left_wheel_increment_time) > left_wheel_tick_time_increment){
                left_encoder_tick_count = (int)std::floor((curr_time - start_time) * (left_ticks/dt));
                last_left_wheel_increment_time = ros::Time::now().toSec();
            }
            if ((curr_time - last_right_wheel_increment_time) > right_wheel_tick_time_increment){
                right_encoder_tick_count = (int)std::floor((curr_time - start_time) * (right_ticks/dt));
                last_right_wheel_increment_time = ros::Time::now().toSec();
            }

            // Publish the JointState msg
            sensor_msgs::JointState joint_state;
            joint_state.name = {"left_encoder", "right_encoder"};
            joint_state.position = {(double)left_encoder_tick_count, (double)right_encoder_tick_count};

            encoder_joint_state_publisher.publish(joint_state);

            // Sleep for our time increment
            ros::Duration(time_increment).sleep();
        }

    }

    ros::NodeHandle nh;

    // The most recently received odom message from the encoder -> odom node
    nav_msgs::Odometry odom_msg;

    // The publisher for publishing JointState messages containing the encoder ticks
    ros::Publisher encoder_joint_state_publisher;

    // The subscriber for received Odometry messages estimated from the encoders
    ros::Subscriber odom_msg_subscriber;

public:

};

TEST_F(EncoderOdometryNodeTest, simulator_test){
    // drive forward 1 rotation of the wheels
    driveSimulator(1000, 1000, 5);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    // We should now be ~62.83 cm ahead (wheel radius is 10cm)
    EXPECT_NEAR(0.6283, odom_msg.pose.pose.position.x, 0.01);
    EXPECT_NEAR(0, odom_msg.pose.pose.position.y, 0.01);
    EXPECT_NEAR(0, tf::getYaw(odom_msg.pose.pose.orientation), 0.01);
}

TEST_F(EncoderOdometryNodeTest, driving_straight_backwards){
    // drive forward 1 rotation of the wheels
    driveSimulator(-1000, -1000, 5);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    // We should now be ~62.83 cm backwards (wheel radius is 10cm)
    EXPECT_NEAR(-0.6283, odom_msg.pose.pose.position.x, 0.01);
    EXPECT_NEAR(0, odom_msg.pose.pose.position.y, 0.01);
    EXPECT_NEAR(0, tf::getYaw(odom_msg.pose.pose.orientation), 0.01);
}

// Test turning 90 degrees right on the spot
TEST_F(EncoderOdometryNodeTest, turn_90_right){
    // drive forward 1 rotation of the wheels
    driveSimulator(-1000, -1000, 5);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    // We should now be ~62.83 cm backwards (wheel radius is 10cm)
    EXPECT_NEAR(-0.6283, odom_msg.pose.pose.position.x, 0.01);
    EXPECT_NEAR(0, odom_msg.pose.pose.position.y, 0.01);
    EXPECT_NEAR(0, tf::getYaw(odom_msg.pose.pose.orientation), 0.01);
}


int main(int argc, char **argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "encoder_odometry_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}