#include <PathFinding.h>
#include <gtest/gtest.h>


TEST(PathFinding, testWeightedSum1){

    std::vector<float> testVec;
    testVec.push_back(3);
    testVec.push_back(3);
    testVec.push_back(3);

    EXPECT_EQ(5.5, PathFinding::weightedSum(testVec, 3));
}

TEST(PathFinding, testWeightedSum2){

    std::vector<float> testVec;
    testVec.push_back(1);
    testVec.push_back(2);
    testVec.push_back(3);

    EXPECT_EQ(1.0, PathFinding::weightedSum(testVec, 1));
}

//Case where robot starting orientation is forward and must travel a straight path diagonal (along y=x line)
TEST(PathFinding, testStraightPathToTwist){
    double xPos = 0;
    double yPos = 0;
    double orientation = 0;

    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i=1; i<10; i++){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = i;
        pose_stamped.pose.position.y = i;
        poses.push_back(pose_stamped);
    }
    path_msg.poses = poses;

    geometry_msgs::Twist twist_msg = PathFinding::pathToTwist(path_msg, xPos, yPos, orientation);
    std::cout<<"Straight Path"<<std::endl;
    std::cout<<twist_msg.linear.x<<std::endl;
    std::cout<<twist_msg.angular.z<<std::endl;
    std::cout<<std::endl;
    EXPECT_NEAR(0.875, twist_msg.linear.x, 0.01);
    EXPECT_NEAR(0.785, twist_msg.angular.z, 0.01);
}

//Case where robot starting orientation is perpendicular to a straight path
TEST(PathFinding, testPerpenPathToTwist){
    double xPos = 0;
    double yPos = 0;
    double orientation = M_PI/2.0;

    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i=1; i<10; i++){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = i;
        pose_stamped.pose.position.y = 0;
        poses.push_back(pose_stamped);
    }
    path_msg.poses = poses;

    geometry_msgs::Twist twist_msg = PathFinding::pathToTwist(path_msg, xPos, yPos, orientation);
    std::cout<<"Perpen Path"<<std::endl;
    std::cout<<twist_msg.linear.x<<std::endl;
    std::cout<<twist_msg.angular.z<<std::endl;
    std::cout<<std::endl;
    EXPECT_NEAR(0.750, twist_msg.linear.x, 0.01);
    EXPECT_NEAR(-1.57, twist_msg.angular.z, 0.01);
}

//Case where robot starting orientation is opposite to a straight path
TEST(PathFinding, testOppPathToTwist){
    double xPos = 0;
    double yPos = 0;
    double orientation = M_PI;

    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i=1; i<10; i++){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = i;
        pose_stamped.pose.position.y = 0;
        poses.push_back(pose_stamped);
    }
    path_msg.poses = poses;

    geometry_msgs::Twist twist_msg = PathFinding::pathToTwist(path_msg, xPos, yPos, orientation);
    std::cout<<"Opp Path"<<std::endl;
    std::cout<<twist_msg.linear.x<<std::endl;
    std::cout<<twist_msg.angular.z<<std::endl;
    std::cout<<std::endl;
    EXPECT_NEAR(0.5, twist_msg.linear.x, 0.01);
    EXPECT_NEAR(M_PI, twist_msg.angular.z, 0.01);
}

//Case where robot is far from beginning of path and facing wrong direction
TEST(PathFinding, testFarStartPos){
    double xPos = -100;
    double yPos = -100;
    double orientation = -M_PI/2;

    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i=1; i<10; i++){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = i;
        pose_stamped.pose.position.y = i;
        poses.push_back(pose_stamped);
    }
    path_msg.poses = poses;

    geometry_msgs::Twist twist_msg = PathFinding::pathToTwist(path_msg, xPos, yPos, orientation);
    std::cout<<"Far Start Pos"<<std::endl;
    std::cout<<twist_msg.linear.x<<std::endl;
    std::cout<<twist_msg.angular.z<<std::endl;
    std::cout<<std::endl;
    EXPECT_NEAR(0.63, twist_msg.linear.x, 0.01);
    EXPECT_NEAR(2.36, twist_msg.angular.z, 0.01);
}

//Case where path consists of 2 sharp turns, one 90 deg left and one 90 deg right later
TEST(PathFinding, testSharpTurns){
    double xPos = 0;
    double yPos = 0;
    double orientation = 0;

    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i=1; i<10; i++){
        geometry_msgs::PoseStamped pose_stamped;
        if (i<=5) {
            pose_stamped.pose.position.x = 0;
            pose_stamped.pose.position.y = i;
        }
        else {
            pose_stamped.pose.position.x = i-5;
            pose_stamped.pose.position.y = 5;
        }
        poses.push_back(pose_stamped);
    }
    path_msg.poses = poses;

    geometry_msgs::Twist twist_msg = PathFinding::pathToTwist(path_msg, xPos, yPos, orientation);
    std::cout<<"Sharp turns"<<std::endl;
    std::cout<<twist_msg.linear.x<<std::endl;
    std::cout<<twist_msg.angular.z<<std::endl;
    std::cout<<std::endl;
    EXPECT_NEAR(0.763, twist_msg.linear.x, 0.01);
    EXPECT_NEAR(1.49, twist_msg.angular.z, 0.01);
}

//Case where path consists of erratic direction and magnitude changes
TEST(PathFinding, testErraticPath){
    double xPos = 0;
    double yPos = 0;
    double orientation = 0;

    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i=1; i<10; i++){
        geometry_msgs::PoseStamped pose_stamped;
        if (i%2==0) {
            pose_stamped.pose.position.x = 2*i;
            pose_stamped.pose.position.y = -2*i;
        }
        else {
            pose_stamped.pose.position.x = i;
            pose_stamped.pose.position.y = 2*i;
        }
        poses.push_back(pose_stamped);
    }
    path_msg.poses = poses;

    geometry_msgs::Twist twist_msg = PathFinding::pathToTwist(path_msg, xPos, yPos, orientation);
    std::cout<<"Erratic Path"<<std::endl;
    std::cout<<twist_msg.linear.x<<std::endl;
    std::cout<<twist_msg.angular.z<<std::endl;
    std::cout<<std::endl;
    EXPECT_NEAR(0.96, twist_msg.linear.x, 0.01);
    EXPECT_NEAR(0.25, twist_msg.angular.z, 0.01);
}

//Case where position/orientation of robot is ahead of the path
TEST(PathFinding, testPathLag){
    double xPos = 2.5;
    double yPos = 0;
    double orientation = 0;

    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i=1; i<10; i++){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = i;
        pose_stamped.pose.position.y = 0;
        poses.push_back(pose_stamped);
    }
    path_msg.poses = poses;

    geometry_msgs::Twist twist_msg = PathFinding::pathToTwist(path_msg, xPos, yPos, orientation);
    std::cout<<"Path Lag"<<std::endl;
    std::cout<<twist_msg.linear.x<<std::endl;
    std::cout<<twist_msg.angular.z<<std::endl;
    std::cout<<std::endl;
    EXPECT_NEAR(1.0, twist_msg.linear.x, 0.01);
    EXPECT_NEAR(0, twist_msg.angular.z, 0.01);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}