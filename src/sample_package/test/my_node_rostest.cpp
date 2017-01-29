//
// Created by valerian on 28/01/17.
//


#include <MyNode.h>
#include <gtest/gtest.h>
#include <ros/ros.h>


class MyClassTest : public testing::Test{
protected:
    virtual void SetUp(){
        testPublisher = nh_.advertise<std_msgs::String>("subscribe_topic", 1);
        testSubscriber = nh_.subscribe("my_node/publish_topic", 1, &MyClassTest::callback, this);
        recieved = false;
    }

    ros::NodeHandle nh_;
    std::string messageOutput;
    ros::Publisher testPublisher;
    ros::Subscriber testSubscriber;

public:
    void callback(const std_msgs::String::ConstPtr msg){
        messageOutput = msg->data.c_str();
        recieved = true;
    }

    std::string getMessage(){
        return messageOutput;
    }

    bool recieved;

};

TEST_F(MyClassTest, MyNodeTest){
    std_msgs::String msg;
    ros::Rate loop_rate(1);
    msg.data = "Hello";
    loop_rate.sleep();

    testPublisher.publish(msg);
    loop_rate.sleep();

    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    loop_rate.sleep();

    std::string message = getMessage();
    loop_rate.sleep();

    EXPECT_EQ("Hello!", message);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "my_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}