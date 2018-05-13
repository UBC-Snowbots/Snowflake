#include <PathToTwist.h>
#include <gtest/gtest.h>

class PathFindingTest : public testing::Test {
  protected:
    virtual void SetUp() {
        test_publisher_path =
        nh_.advertise<nav_msgs::Path>("/path", 1); // Path publisher
        test_publisher_tf =
        nh_.advertise<tf2_msgs::TFMessage>("/tf", 1); // TF publisher
        test_subscriber =
        nh_.subscribe("/cmd_vel", 1, &PathFindingTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    ros::Publisher test_publisher_path;
    ros::Publisher test_publisher_tf;
    ros::Subscriber test_subscriber;
    double speed;
    double turn_rate;

  public:
    void callback(const geometry_msgs::Twist::ConstPtr vel) {
        speed     = vel->linear.x;
        turn_rate = vel->angular.z;
    }
};

TEST_F(PathFindingTest, testStraightPathFinding) {
    tf2_msgs::TFMessage tf_msg;
    std::vector<geometry_msgs::TransformStamped> all_transforms;
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.frame_id         = "GLOBAL_FRAME";
    transform_stamped.child_frame_id          = "BASE_FRAME";
    transform_stamped.transform.translation.x = 0;
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.rotation.x    = 0;
    transform_stamped.transform.rotation.y    = 0;
    transform_stamped.transform.rotation.z    = 0;
    transform_stamped.transform.rotation.w    = 1;
    all_transforms.push_back(transform_stamped);
    tf_msg.transforms = all_transforms;
    test_publisher_tf.publish(tf_msg);
    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    // Construct a path message to send to the test node
    // Path is currently a straight line
    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i = 1; i < 10; i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = i;
        pose.pose.position.y = i;
        poses.push_back(pose);
    }
    path_msg.poses = poses;

    test_publisher_path.publish(path_msg);

    // Wait for the message to get passed around
    loop_rate.sleep();

    ros::spinOnce();

    EXPECT_NEAR(0.88, speed, 0.1);
    EXPECT_NEAR(M_PI / 4, turn_rate, 0.1);
}

TEST_F(PathFindingTest, testCurvedPathFinding) {
    tf2_msgs::TFMessage tf_msg;
    std::vector<geometry_msgs::TransformStamped> all_transforms;
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.frame_id         = "GLOBAL_FRAME";
    transform_stamped.child_frame_id          = "BASE_FRAME";
    transform_stamped.transform.translation.x = 0;
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.rotation.x    = 0;
    transform_stamped.transform.rotation.y    = 0;
    transform_stamped.transform.rotation.z    = 0;
    transform_stamped.transform.rotation.w    = 1;
    all_transforms.push_back(transform_stamped);
    tf_msg.transforms = all_transforms;
    test_publisher_tf.publish(tf_msg);
    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();
    // Construct a path message to send to the test node
    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i = 1; i < 10; i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = i;
        if (i <= 5) {
            pose.pose.position.y = i;
        } else {
            pose.pose.position.y = 10 - i;
        }
        poses.push_back(pose);
    }
    path_msg.poses = poses;

    test_publisher_path.publish(path_msg);

    // Wait for the message to get passed around
    loop_rate.sleep();

    ros::spinOnce();

    EXPECT_NEAR(0.89, speed, 0.1);
    EXPECT_NEAR(0.55, turn_rate, 0.1);
}

TEST_F(PathFindingTest, testSharpUTurn) {
    tf2_msgs::TFMessage tf_msg;
    std::vector<geometry_msgs::TransformStamped> all_transforms;
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.frame_id         = "GLOBAL_FRAME";
    transform_stamped.child_frame_id          = "BASE_FRAME";
    transform_stamped.transform.translation.x = 0;
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.rotation.x    = 0;
    transform_stamped.transform.rotation.y    = 0;
    transform_stamped.transform.rotation.z    = 0;
    transform_stamped.transform.rotation.w    = 1;
    all_transforms.push_back(transform_stamped);
    tf_msg.transforms = all_transforms;
    test_publisher_tf.publish(tf_msg);
    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    // Construct a path message to send to the test node
    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i = 1; i < 10; i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.y = i;
        if (i <= 3) {
            pose.pose.position.x = i;
        } else {
            pose.pose.position.x = 6 - i;
        }
        poses.push_back(pose);
    }
    path_msg.poses = poses;

    test_publisher_path.publish(path_msg);

    // Wait for the message to get passed around
    loop_rate.sleep();

    ros::spinOnce();

    EXPECT_NEAR(0.80, speed, 0.1);
    EXPECT_NEAR(1.28, turn_rate, 0.1);
}

TEST_F(PathFindingTest, testReceiveTF1) {
    // Send custom tf to update robot position to pos x=0, y=100, facing in pos
    // x direction
    // Then send a path msg (straight line) with the robot's new position in
    // mind
    tf2_msgs::TFMessage tf_msg;
    std::vector<geometry_msgs::TransformStamped> all_transforms;
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.frame_id         = "GLOBAL_FRAME";
    transform_stamped.child_frame_id          = "BASE_FRAME";
    transform_stamped.transform.translation.x = 0;
    transform_stamped.transform.translation.y = 100;
    transform_stamped.transform.rotation.x    = 0;
    transform_stamped.transform.rotation.y    = 0;
    transform_stamped.transform.rotation.z    = 0;
    transform_stamped.transform.rotation.w    = 1;
    all_transforms.push_back(transform_stamped);
    tf_msg.transforms = all_transforms;
    test_publisher_tf.publish(tf_msg);
    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i = 1; i < 10; i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.y = 100;
        pose.pose.position.x = i;
        poses.push_back(pose);
    }
    path_msg.poses = poses;

    test_publisher_path.publish(path_msg);

    // Wait for the message to get passed around
    loop_rate.sleep();
    ros::spinOnce();

    EXPECT_NEAR(1, speed, 0.1);
    EXPECT_NEAR(0, turn_rate, 0.1);
}

TEST_F(PathFindingTest, testReceiveTF2) {
    // Send custom tf to update robot position to pos x=50, y=50, facing in pos
    // y direction
    // Then send a path msg (straight line) with the robot's new position in
    // mind
    tf2_msgs::TFMessage tf_msg;
    std::vector<geometry_msgs::TransformStamped> all_transforms;
    geometry_msgs::TransformStamped transform_stamped;

    transform_stamped.header.frame_id         = "GLOBAL_FRAME";
    transform_stamped.child_frame_id          = "BASE_FRAME";
    transform_stamped.transform.translation.x = 50;
    transform_stamped.transform.translation.y = 50;
    transform_stamped.transform.rotation.x    = 0;
    transform_stamped.transform.rotation.y    = 0;
    transform_stamped.transform.rotation.z    = 0.707;
    transform_stamped.transform.rotation.w    = 0.707;
    all_transforms.push_back(transform_stamped);
    tf_msg.transforms = all_transforms;
    test_publisher_tf.publish(tf_msg);
    // Wait for the message to get passed around
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    nav_msgs::Path path_msg;
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i = 1; i < 10; i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.y = 50 + i;
        pose.pose.position.x = 50;
        poses.push_back(pose);
    }
    path_msg.poses = poses;

    test_publisher_path.publish(path_msg);

    // Wait for the message to get passed around
    loop_rate.sleep();
    ros::spinOnce();

    EXPECT_NEAR(1, speed, 0.1);
    EXPECT_NEAR(0, turn_rate, 0.1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_finding_rostest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}