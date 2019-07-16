/**
 * Created by William Gu on Jun 16 2019
 * Contains some simple ROS tests to ensure broker node is forwarding messages correctly
 */

#include <RCBrokerNode.h>
#include <gtest/gtest.h>

class RCBrokerTest : public testing::Test {
protected:
    virtual void SetUp() {
        test_publisher_client =
                nh_.advertise<std_msgs::String>("/client_sub", 1);
        test_publisher_server =
                nh_.advertise<std_msgs::String>("/server_sub", 1);
        test_subscriber_client =
                nh_.subscribe("/client_pub", 1, &RCBrokerTest::callback1, this);
        test_subscriber_server =
                nh_.subscribe("/server_pub", 1, &RCBrokerTest::callback2, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    ros::Publisher test_publisher_client;
    ros::Publisher test_publisher_server;
    ros::Subscriber test_subscriber_client;
    ros::Subscriber test_subscriber_server;

    std_msgs::String rcv_client_msg;
    std_msgs::String rcv_server_msg;

public:
    void callback1(const std_msgs::String::ConstPtr& ptr) {
        rcv_client_msg = *ptr;
    }
    void callback2(const std_msgs::String::ConstPtr& ptr) {
        rcv_server_msg = *ptr;
    }
};

TEST_F(RCBrokerTest, testEZ) {
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rc_broker_rostest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}