#include <PathFinding.h>
#include <gtest/gtest.h>

class PathFindingTest : public testing::Test{
protected:
    virtual void SetUp(){
        test_publisher = nh_.advertise<nav_msgs::Path>("/path", 1);
        test_subscriber = nh_.subscribe("/cmd_vel", 1,
        &PathFindingTest::callback, this);

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

TEST_F(PathFindingTest, testStraightPathFinding){
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

    ros::spinOnce();

    EXPECT_NEAR(0.88, speed, 0.1);
    EXPECT_NEAR(M_PI/4, turn_rate, 0.1);
}

TEST_F(PathFindingTest, testCurvedPathFinding){

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

    ros::spinOnce();

    EXPECT_NEAR(0.89, speed, 0.1);
    EXPECT_NEAR(0.70, turn_rate,0.1);
}

TEST_F(PathFindingTest, testSharpUTurn){
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

    ros::spinOnce();

    EXPECT_NEAR(0.80, speed, 0.1);
    EXPECT_NEAR(1.00, turn_rate, 0.1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_finding_rostest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}