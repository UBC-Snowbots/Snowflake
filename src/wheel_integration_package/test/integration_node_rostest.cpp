/*
 * Created By: Valerian Ratu
 * Created On: January 29, 2017
 * Description: Integration testing for MyNode
 */

#include <IntegrationNode.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

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
class MyNodeTest : public testing::Test {
  protected:
    virtual void SetUp() {
        test_publisher = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        test_lwheels_subscriber =
        nh_.subscribe("/integration_node/lwheels_pub_topic",
                      1,
                      &MyNodeTest::lwheels_callback,
                      this);
        test_rwheels_subscriber =
        nh_.subscribe("/integration_node/rwheels_pub_topic",
                      1,
                      &MyNodeTest::rwheels_callback,
                      this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    float lwheels_output, rwheels_output;
    ros::Publisher test_publisher;
    ros::Subscriber test_lwheels_subscriber, test_rwheels_subscriber;

  public:
    void lwheels_callback(const geometry_msgs::Twist::ConstPtr msg) {
        lwheels_output = msg->linear.x;
    }
    void rwheels_callback(const geometry_msgs::Twist::ConstPtr msg) {
        rwheels_output = msg->linear.x;
    }
};

TEST_F(MyNodeTest, straight_fullspeed_test) {
    // publishes straight moving Twist message to the test node
    geometry_msgs::Twist msg;
    msg.linear.x  = 1.0;
    msg.angular.z = 0.0;
    test_publisher.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(1.0, lwheels_output);
    EXPECT_FLOAT_EQ(1.0, rwheels_output);
}

TEST_F(MyNodeTest, straight_halfspeed_test) {
    // publishes straight moving Twist message to the test node
    geometry_msgs::Twist msg;
    msg.linear.x  = 0.5;
    msg.angular.z = 0.0;
    test_publisher.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(0.5, lwheels_output);
    EXPECT_FLOAT_EQ(0.5, rwheels_output);
}

TEST_F(MyNodeTest, straight_halt_test) {
    // publishes straight moving Twist message to the test node
    geometry_msgs::Twist msg;
    msg.linear.x  = 0.0;
    msg.angular.z = 0.0;
    test_publisher.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(0.0, lwheels_output);
    EXPECT_FLOAT_EQ(0.0, rwheels_output);
}

TEST_F(MyNodeTest, straight_back_halfspeed_test) {
    // publishes straight moving Twist message to the test node
    geometry_msgs::Twist msg;
    msg.linear.x  = -0.5;
    msg.angular.z = 0.0;
    test_publisher.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(-0.5, lwheels_output);
    EXPECT_FLOAT_EQ(-0.5, rwheels_output);
}

TEST_F(MyNodeTest, straight_back_fullspeed_test) {
    // publishes straight moving Twist message to the test node
    geometry_msgs::Twist msg;
    msg.linear.x  = -1.0;
    msg.angular.z = 0.0;
    test_publisher.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(-1.0, lwheels_output);
    EXPECT_FLOAT_EQ(-1.0, rwheels_output);
}

TEST_F(MyNodeTest, curve_fullspeed_test) {
    // publishes straight moving Twist message to the test node
    geometry_msgs::Twist msg;
    msg.linear.x  = 1.0;
    msg.angular.z = 0.3;
    test_publisher.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(0.8605, lwheels_output);
    EXPECT_FLOAT_EQ(1.1395, rwheels_output);
}

TEST_F(MyNodeTest, curve_halfspeed_test) {
    // publishes straight moving Twist message to the test node
    geometry_msgs::Twist msg;
    msg.linear.x  = 0.5;
    msg.angular.z = 0.5;
    test_publisher.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(0.2675, lwheels_output);
    EXPECT_FLOAT_EQ(0.7325, rwheels_output);
}

TEST_F(MyNodeTest, spin_test) {
    // publishes straight moving Twist message to the test node
    geometry_msgs::Twist msg;
    msg.linear.x  = 0;
    msg.angular.z = 0.8;
    test_publisher.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(-0.372, lwheels_output);
    EXPECT_FLOAT_EQ(0.372, rwheels_output);
}

TEST_F(MyNodeTest, curve_back_halfspeed_test) {
    // publishes straight moving Twist message to the test node
    geometry_msgs::Twist msg;
    msg.linear.x  = -0.5;
    msg.angular.z = 0.8;
    test_publisher.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(-0.872, lwheels_output);
    EXPECT_FLOAT_EQ(-0.128, rwheels_output);
}

TEST_F(MyNodeTest, curve_back_fullspeed_test) {
    // publishes straight moving Twist message to the test node
    geometry_msgs::Twist msg;
    msg.linear.x  = -1.0;
    msg.angular.z = -1.0;
    test_publisher.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious:
    // http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(-0.535, lwheels_output);
    EXPECT_FLOAT_EQ(-1.465, rwheels_output);
}

int main(int argc, char** argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros
    // framework !!
    ros::init(argc, argv, "integration_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}