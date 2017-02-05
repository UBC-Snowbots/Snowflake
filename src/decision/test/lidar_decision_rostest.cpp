/*
 * Created By: Gareth Ellis
 * Created On: February 4th, 2016
 * Description: Integration testing for the lidar_decision node
 */

#include <LidarDecision.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

class LidarDecisionTest : public testing::Test{
protected:
    virtual void SetUp(){
        laserScanPublisher = nh_.advertise<sensor_msgs::LaserScan>("/robot/scan", 1);
        twistSubscriber = nh_.subscribe("/lidar_decision/command", 1, &LidarDecisionTest::callback, this);

        ros::spinOnce();
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    ros::Publisher laserScanPublisher;
    ros::Subscriber twistSubscriber;

    geometry_msgs::Twist command;

public:
    void callback(const geometry_msgs::Twist::ConstPtr msg){
        command = *msg;
    }
};

TEST_F(LidarDecisionTest, overallTest){
    // Create a fake lidar scan to publish
    sensor_msgs::LaserScan scan;
    ulong num_rays = 300;
    scan.angle_min = 0;
    scan.angle_max = (float)M_PI;
    scan.angle_increment = (scan.angle_max - scan.angle_min)/num_rays;
    // Set all the ranges to 0 initially
    scan.ranges = std::vector<float>(num_rays, 0);
    scan.range_min = 2;
    scan.range_max = 40;
    // Add a large obstacle directly in front
    std::fill(scan.ranges.begin()+140, scan.ranges.begin()+220, 3);

    laserScanPublisher.publish(scan);

    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    // With the given laserscan, we would want to be turning right
    EXPECT_GE(abs(command.angular.z), 0);

    // We would also not want to be going forward
    EXPECT_EQ(0, command.linear.x);

    // Everything else should always be 0
    EXPECT_EQ(0, command.linear.y);
    EXPECT_EQ(0, command.linear.z);
    EXPECT_EQ(0, command.angular.x);
    EXPECT_EQ(0, command.angular.y);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_decision_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
