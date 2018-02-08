/*
 * Created By: Marcus Swift
 * Created On: November 20th, 2017
 * Description: Intergration testing for EKFNode
 */

#include <EKFNode.h>
#include <gtest/gtest.h>
#include <random>
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
class EKFNodeTest : public testing::Test {
  protected:
    virtual void SetUp() {
        // set up publsihers and subscribers
        test_gps_publisher =
        nh_.advertise<nav_msgs::Odometry>("/gps_driver/odom", 1000);
        test_encoder_publisher =
        nh_.advertise<nav_msgs::Odometry>("/odom", 1000);
        test_imu_publisher = nh_.advertise<sensor_msgs::Imu>("/imu", 1000);
        test_pose_subscriber =
        nh_.subscribe("/cmd_pose", 1, &EKFNodeTest::poseCallback, this);
        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(100);
        loop_rate.sleep();
    }

    // node handler, publsiher and subscriber definitions
    ros::NodeHandle nh_;
    geometry_msgs::Pose pose_data;
    ros::Publisher test_gps_publisher;
    ros::Publisher test_encoder_publisher;
    ros::Publisher test_imu_publisher;
    ros::Subscriber test_pose_subscriber;

  public:
    void poseCallback(const geometry_msgs::Pose::ConstPtr pose_msg) {
        pose_data.position    = pose_msg->position;
        pose_data.orientation = pose_msg->orientation;
    }

    void driveSimulator(double x,
                        double y,
                        double angle,
                        double speed,
                        double ang_vel,
                        double seconds_traveling,
                        std::mt19937 e2) {
        // Set local variables
        int i                   = 0;
        int j                   = 0;
        double measured_x       = x;
        double measured_y       = y;
        double measured_angle   = angle;
        double measured_speed   = speed;
        double measured_ang_vel = ang_vel;
        std::normal_distribution<> gyro_noise(0, M_PI / 180);
        std::normal_distribution<> speed_noise(0, 0.15);
        std::normal_distribution<> gps_x_noise(0, 0.5);
        std::normal_distribution<> gps_y_noise(0, 0.5);
        std::normal_distribution<> angle_noise(0, M_PI / 180);
        sensor_msgs::Imu test_imu;
        nav_msgs::Odometry test_gps;
        nav_msgs::Odometry test_encoder;
        const int gps_refresh_rate     = 4;  // times per second
        const int encoder_refresh_rate = 20; // per seconds
        ros::Rate loop_rate(100);

        for (i = 0; i < gps_refresh_rate * seconds_traveling; i++) {
            for (j = 0; j < (int) encoder_refresh_rate / gps_refresh_rate;
                 j++) {
                // save every generated value as real position
                x += sin(angle) * speed * (1. / encoder_refresh_rate);
                y += cos(angle) * speed * (1. / encoder_refresh_rate);
                angle += ang_vel * (1. / encoder_refresh_rate);
                angle = EKF::constrainAngleInBounds(angle);

                measured_speed += speed_noise(e2);
                measured_ang_vel += gyro_noise(e2);

                // publish IMU
                test_imu.angular_velocity.z = measured_ang_vel;
                test_imu.orientation =
                tf::createQuaternionMsgFromYaw(measured_angle);
                test_imu_publisher.publish(test_imu);
                ros::spinOnce();

                // publish encoder
                test_encoder.twist.twist.linear.x = measured_speed;
                test_encoder_publisher.publish(test_encoder);
                sleep(0.05);
                loop_rate.sleep();
                ros::spinOnce();
            }

            // save every generated values as measured position
            measured_x +=
            sin(angle) * speed * (1. / gps_refresh_rate) + gps_x_noise(e2);
            measured_y +=
            cos(angle) * speed * (1. / gps_refresh_rate) + gps_y_noise(e2);
            measured_angle += ang_vel * (1. / gps_refresh_rate) +
                              angle_noise(e2); // add some noise
            measured_angle = EKF::constrainAngleInBounds(measured_angle);

            // publish GPS
            test_gps.pose.pose.position.x = measured_x;
            test_gps.pose.pose.position.y = measured_y;
            test_gps_publisher.publish(test_gps);
            sleep(0.01);
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
};

TEST_F(EKFNodeTest, testEKFNode) {
    // set up gaussian noise generator
    // std::random_device rd;
    std::mt19937 e2(99);   // std::mt19937 e2(rd()); for a random seed
    double tolerance = 30; // the maximum error allowed in the error test
    // Start at the engineering design centre (in UTM)
    double x       = 481917;  // starting positoin in metres
    double y       = 5456662; // starting position in metres
    double angle   = 0;       // starting angle in radians
    double speed   = 3;       // starting speed in metres per second
    double ang_vel = 0;       // starting angular velocity in radians per second
    double seconds_traveling = 50;

    // travel north for 50 seconds

    driveSimulator(x, y, angle, speed, ang_vel, seconds_traveling, e2);

    x += seconds_traveling * sin(angle) * speed;
    y += seconds_traveling * cos(angle) * speed;
    angle += seconds_traveling * ang_vel;

    EXPECT_NEAR(x, pose_data.position.x, tolerance);
    EXPECT_NEAR(y, pose_data.position.y, tolerance);

    // rotate to the East
    ang_vel           = -(1. / 16.) * M_PI;
    speed             = 0;
    seconds_traveling = 8;
    driveSimulator(x, y, angle, speed, ang_vel, seconds_traveling, e2);

    x += seconds_traveling * sin(angle) * speed;
    y += seconds_traveling * cos(angle) * speed;
    angle += seconds_traveling * ang_vel;

    EXPECT_NEAR(x, pose_data.position.x, tolerance);
    EXPECT_NEAR(y, pose_data.position.y, tolerance);

    // travel east for 30 seconds
    ang_vel           = 0;
    speed             = 4;
    seconds_traveling = 30;
    driveSimulator(x, y, angle, speed, ang_vel, seconds_traveling, e2);

    x += seconds_traveling * sin(angle) * speed;
    y += seconds_traveling * cos(angle) * speed;
    angle += seconds_traveling * ang_vel;

    EXPECT_NEAR(x, pose_data.position.x, tolerance);
    EXPECT_NEAR(y, pose_data.position.y, tolerance);

    // rotate to face the south west
    ang_vel           = (1. / 8.) * M_PI;
    speed             = 0;
    seconds_traveling = 10;
    driveSimulator(x, y, angle, speed, ang_vel, seconds_traveling, e2);

    x += seconds_traveling * sin(angle) * speed;
    y += seconds_traveling * cos(angle) * speed;
    angle += seconds_traveling * ang_vel;

    EXPECT_NEAR(x, pose_data.position.x, tolerance);
    EXPECT_NEAR(y, pose_data.position.y, tolerance);

    // travel south west for 40 seconds
    ang_vel           = 0;
    speed             = 2;
    seconds_traveling = 40;
    driveSimulator(x, y, angle, speed, ang_vel, seconds_traveling, e2);

    x += seconds_traveling * sin(angle) * speed;
    y += seconds_traveling * cos(angle) * speed;
    angle += seconds_traveling * ang_vel;

    EXPECT_NEAR(x, pose_data.position.x, tolerance);
    EXPECT_NEAR(y, pose_data.position.y, tolerance);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "EKFNode");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
