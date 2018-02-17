/*
#include <PathFinding.h>
#include <gtest/gtest.h>

class PathFindingTest : public testing::Test{
protected:
    virtual void SetUp(){
        test_publisher = nh_.advertise<nav_msgs::Path>("/path", 1);
        test_subscriber = nh_.subscribe("/cmd_vel", 1, &PathFindingTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;
    double speed;
    double turn_rate;

public:
    void callback(const geometry_msgs::Twist::ConstPtr vel){
        speed = vel->linear.x;
        turn_rate = vel->angular.z;
    }
};

TEST_F(PathFindingTest, testStraightPathFinding){//Note that without given orientation/position, initial is 0,0 (center facing x)
    //Construct a path message to send to the test node
    //Path is currently a straight line
    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i=1; i<10; i++){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = i;
        pose.pose.position.y = i;
        poses.push_back(pose);
    }
    path_msg.poses = poses;

    test_publisher.publish(path_msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_NEAR(0.88, speed, 0.1);
    EXPECT_NEAR(M_PI/4, turn_rate, 0.1);
}

TEST_F(PathFindingTest, testCurvedPathFinding){//Note that without given orientation/position, initial is 0,0 (center, facing x)

    //Construct a path message to send to the test node
    //Path is currently a straight line
    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i=1; i<10; i++){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = i;
        if (i<=5){
            pose.pose.position.y = i;
        }
        else{
            pose.pose.position.y = 10-i;
        }
        poses.push_back(pose);
    }
    path_msg.poses = poses;

    test_publisher.publish(path_msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_NEAR(0.70, speed, 0.1);
    EXPECT_FLOAT_EQ(M_PI/4, turn_rate);
}

TEST_F(PathFindingTest, testSharpUTurn){//Note that without given orientation/position, initial is 0,0 (center, facing x)
    //Construct a path message to send to the test node
    //Path is currently a straight line
    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i=1; i<10; i++){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.y = i;
        if (i<=3){
            pose.pose.position.x = i;
        }
        else{
            pose.pose.position.x = 6-i;
        }
        poses.push_back(pose);
    }
    path_msg.poses = poses;

    test_publisher.publish(path_msg);

    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();

    // spinOnce allows ros to actually process your callbacks
    // for the curious: http://answers.ros.org/question/11887/significance-of-rosspinonce/
    ros::spinOnce();

    EXPECT_FLOAT_EQ(0.875, speed);
    EXPECT_FLOAT_EQ(M_PI/4, turn_rate);
}

int main(int argc, char **argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "path_finding_rostest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}*/