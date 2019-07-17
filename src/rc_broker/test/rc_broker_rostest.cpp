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

TEST_F(RCBrokerTest, testServerToClient) {
    std_msgs::String msg;
    msg.data = "Hello Client!";
    test_publisher_server.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    EXPECT_TRUE(rcv_client_msg.data == "Hello Client!");
}

TEST_F(RCBrokerTest, testClientToServer) {
    std_msgs::String msg;
    msg.data = "Hello Server!";
    test_publisher_client.publish(msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    EXPECT_TRUE(rcv_server_msg.data == "Hello Server!");
}

TEST_F(RCBrokerTest, testSimMessages) {
    std_msgs::String msgToServer;
    msgToServer.data = "Hello Server!";

    std_msgs::String msgToClient;
    msgToServer.data = "Hello Client!";

    test_publisher_client.publish(msgToServer);
    test_publisher_server.publish(msgToClient);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    EXPECT_TRUE(rcv_server_msg.data == "Hello Server!");
    EXPECT_TRUE(rcv_client_msg.data == "Hello Client!");
}

TEST_F(RCBrokerTest, testMultiMessages) {

    for (int i = 0; i < 100; i ++){
        std_msgs::String msg;
        msg.data = std::to_string(i);
        test_publisher_client.publish(msg);

        ros::Rate loop_rate(1);
        loop_rate.sleep();
        ros::spinOnce();

        EXPECT_TRUE(rcv_server_msg.data == std::to_string(i));
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rc_broker_rostest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}