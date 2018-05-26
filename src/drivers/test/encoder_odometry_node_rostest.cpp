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
 * This is the helper class which will publish and subscribe messages which will
 * test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the
 * subscriber
 *      getter function - to provide a way for gtest to check for equality of
 * the message recieved
 */
class EncoderOdometryNodeTest : public testing::Test {
  protected:
    EncoderOdometryNodeTest()
      : wheel_radius(0.1), ticks_per_rotation(1000), wheelbase(0.5) {
        // Setup Publishers and Subscribers
        encoder_joint_state_publisher =
        nh.advertise<sensor_msgs::JointState>("/encoders/joint_states", 1);
        // Note: We seem to need to set the queue size to be really large here
        // in order to get the most recent
        //      odometry estimate....
        odom_msg_subscriber =
        nh.subscribe("/encoders/odom",
                     5000,
                     &EncoderOdometryNodeTest::odomMsgCallback,
                     this);
        reset_msg_publisher =
        nh.advertise<std_msgs::Empty>("/encoder_odometry_node/reset", 10);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    virtual void SetUp() {
        // Let callbacks process
        ros::Rate loop_rate(1);
        loop_rate.sleep();
        ros::spinOnce();

        // Reset the Encoder to Odometry Node by publishing an empty message
        // the the node's "reset" topic
        reset_msg_publisher.publish(std_msgs::Empty());

        ros::spinOnce();
    }

    /**
     * The callback function for received Odometry messages
     * @param odom_msg_ptr
     */
    void odomMsgCallback(const nav_msgs::Odometry::ConstPtr& odom_msg_ptr) {
        odom_msg = *odom_msg_ptr;
    }

    /**
     * Simulate moving the robot by simulating encoder values
     *
     * Publishes the given number of left and right encoder ticks over the
     * given time frame
     *
     * @param left_ticks the number of ticks to move the left wheel
     * @param right_ticks the number of ticks to move the right wheel
     * @param dt the time over which to perform the given movement
     */
    void driveSimulator(int left_ticks, int right_ticks, double dt) {
        // We want to increment at least as fast as we're ticking
        double time_increment =
        dt / std::max(std::abs(left_ticks), std::abs(right_ticks));

        // The time between left and right wheel ticks
        double left_wheel_tick_time_increment  = dt / std::abs(left_ticks);
        double right_wheel_tick_time_increment = dt / std::abs(right_ticks);

        // The last time we incremented the left and right wheel ticks
        double last_left_wheel_increment_time  = ros::Time::now().toSec();
        double last_right_wheel_increment_time = ros::Time::now().toSec();

        // The current number of ticks of the left and right wheel encoders
        int left_encoder_tick_count  = 0;
        int right_encoder_tick_count = 0;

        double start_time = ros::Time::now().toSec();
        double end_time   = start_time + dt + time_increment;
        while (ros::Time::now().toSec() < end_time) {
            double curr_time = ros::Time::now().toSec();

            // Check if we should increment the left or right wheels
            if ((curr_time - last_left_wheel_increment_time) >
                left_wheel_tick_time_increment) {
                left_encoder_tick_count =
                (int) std::floor((curr_time - start_time) * (left_ticks / dt));
                last_left_wheel_increment_time = ros::Time::now().toSec();
            }
            if ((curr_time - last_right_wheel_increment_time) >
                right_wheel_tick_time_increment) {
                right_encoder_tick_count =
                (int) std::floor((curr_time - start_time) * (right_ticks / dt));
                last_right_wheel_increment_time = ros::Time::now().toSec();
            }

            // Publish the JointState msg
            sensor_msgs::JointState joint_state;
            joint_state.name     = {"left_encoder", "right_encoder"};
            joint_state.position = {(double) left_encoder_tick_count,
                                    (double) right_encoder_tick_count};

            encoder_joint_state_publisher.publish(joint_state);

            // Let callbacks process
            ros::spinOnce();

            // Sleep for our time increment
            ros::Duration(time_increment).sleep();
        }
    }

    ros::NodeHandle nh;

    // The most recently received odom message from the encoder -> odom node
    nav_msgs::Odometry odom_msg;

    // The publisher for publishing JointState messages containing the encoder
    // ticks
    ros::Publisher encoder_joint_state_publisher;

    // The publisher for publishing empty "reset" messages to reset the
    // encoder to odometry node
    ros::Publisher reset_msg_publisher;

    // The subscriber for received Odometry messages estimated from the encoders
    ros::Subscriber odom_msg_subscriber;

    // Things that we've set for the Encoder -> Odometry node
    double wheel_radius;
    double ticks_per_rotation;
    double wheelbase;

  public:
};

TEST_F(EncoderOdometryNodeTest, driving_straight_forward) {
    // drive forward 1 rotation of the wheels
    driveSimulator(1000, 1000, 3);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    // We should now be ~62.83 cm ahead (wheel radius is 10cm)
    EXPECT_NEAR(0.6283, odom_msg.pose.pose.position.x, 0.05);
    EXPECT_NEAR(0, odom_msg.pose.pose.position.y, 0.01);
    EXPECT_NEAR(0, tf::getYaw(odom_msg.pose.pose.orientation), 0.01);
}

TEST_F(EncoderOdometryNodeTest, driving_straight_backwards) {
    // drive forward 1 rotation of the wheels
    driveSimulator(-1000, -1000, 3);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    // We should now be ~62.83 cm backwards (wheel radius is 10cm)
    EXPECT_NEAR(-0.6283, odom_msg.pose.pose.position.x, 0.05);
    EXPECT_NEAR(0, odom_msg.pose.pose.position.y, 0.01);
    EXPECT_NEAR(0, tf::getYaw(odom_msg.pose.pose.orientation), 0.01);
}

// Test turning 90 degrees by traveling in an arc
TEST_F(EncoderOdometryNodeTest, turn_90_right) {
    double inner_turn_radius = 2;
    double outer_turn_radius = inner_turn_radius + wheelbase;

    // Calculate how far the left and right wheel will have to move
    double dl = (2 * M_PI / 4) * outer_turn_radius;
    double dr = (2 * M_PI / 4) * inner_turn_radius;

    // Figure out how many ticks this will be per wheel
    double wheel_circumference = 2 * M_PI * wheel_radius;
    double ticks_per_rotation  = 1000;
    auto left_wheel_ticks =
    (int) std::floor(dl * (ticks_per_rotation / wheel_circumference));
    auto right_wheel_ticks =
    (int) std::floor(dr * (ticks_per_rotation / wheel_circumference));

    // Simulate driving in an arc
    driveSimulator(left_wheel_ticks, right_wheel_ticks, 5);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_NEAR(2.25, odom_msg.pose.pose.position.x, 0.2);
    EXPECT_NEAR(-2.25, odom_msg.pose.pose.position.y, 0.2);
    EXPECT_NEAR(-(M_PI / 2), tf::getYaw(odom_msg.pose.pose.orientation), 0.08);
}

int main(int argc, char** argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros
    // framework !!
    ros::init(argc, argv, "encoder_odometry_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
