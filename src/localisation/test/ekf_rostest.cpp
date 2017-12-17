/*
 * Created By: Marcus Swift
 * Created On: November 20th, 2017
 * Description: Intergration testing for EKF
 */


#include <EKF.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <random>

geometry_msgs::Quaternion angToQuaternion(double angle);
double angleWrap(double angle);


/**
 * This is the helper class which will publish and subscribe messages which will test the node being instantiated
 * It contains at the minimum:
 *      publisher - publishes the input to the node
 *      subscriber - publishes the output of the node
 *      callback function - the callback function which corresponds to the subscriber
 *      getter function - to provide a way for gtest to check for equality of the message recieved
 */
class EKFTest : public testing::Test{
protected:
    virtual void SetUp(){
        test_gps_publisher = nh_.advertise<nav_msgs::Odometry>("/gps", 10);
		test_encoder_publisher = nh_.advertise<nav_msgs::Odometry>("/encoder", 10);
		test_imu_publisher = nh_.advertise<sensor_msgs::Imu>("/imu", 10);
        test_pose_subscriber = nh_.subscribe("/cmd_pose", 10, &EKFTest::poseCallback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(10);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    geometry_msgs::Pose pose_data;
	ros::Publisher test_gps_publisher;
	ros::Publisher test_encoder_publisher;
    ros::Publisher test_imu_publisher;
    ros::Subscriber test_pose_subscriber;

public:

    void poseCallback(const geometry_msgs::Pose::ConstPtr pose_msg){
        pose_data.position = pose_msg->position;
		pose_data.orientation = pose_msg->orientation;
    }
};

TEST_F(EKFTest, testEKF){
	std::random_device rd;
	std::mt19937 e2(rd());
	std::normal_distribution<> noise(0, 5);
	//Start at the engineering design centre (in UTM)
	double x = 482003;   //starting positoin in metres
	double y = 5456550;  //starting position in metres
	double angle = 0; //starting angle in radians
	double measured_x = x;
	double measured_y = y;
	double measured_angle = angle;
	double speed = 3; //starting speed in metres per second
	double ang_vel = 0; //starting angular velocity in radians per second
	double measured_speed = speed;
	double measured_ang_vel = ang_vel;
	sensor_msgs::Imu test_imu;
	nav_msgs::Odometry test_gps;
	nav_msgs::Odometry test_encoder;
	
	int i = 0;
	
	//travel north for 50 seconds
	for (i = 0; i < 50; i++) {
		
		//save every generated value as real position
		x += cos(angle)*speed;
		y += sin(angle)*speed;
		angle += ang_vel;
		angle = angleWrap(angle);
		
		//save every generated values as measured position
		measured_x += cos(angle)*speed + noise(e2); //add some noise
		measured_y += sin(angle)*speed + noise(e2); //add some noise
		measured_angle += ang_vel + noise(e2); //add some noise
		measured_angle = angleWrap(angle);
		
		measured_speed += noise(e2); //add some random noise
		measured_ang_vel += noise(e2); //add some random noise
		
		test_gps.pose.pose.position.x = measured_x;
		test_gps.pose.pose.position.y = measured_y;
		test_encoder.twist.twist.linear.x = measured_speed;
		test_imu.angular_velocity.z = measured_ang_vel;
		test_imu.orientation = angToQuaternion(measured_angle);
		
		test_gps_publisher.publish(test_gps);
		test_encoder_publisher.publish(test_encoder);
		test_imu_publisher.publish(test_imu);
		ros::Rate loop_rate(10);
		loop_rate.sleep();
		ros::spinOnce();
		
		std::cout << "curr pos" << x << ", " << y << std::endl;
		std::cout << "ekf pos" << pose_data.position.x << ", " << pose_data.position.y << std::endl;
				
	}
		EXPECT_NEAR(x, pose_data.position.x, 100);
		EXPECT_NEAR(y, pose_data.position.y, 100);

	//rotate to the East 
	ang_vel = -(1./16.)*M_PI;
	speed = 0;
	measured_speed = speed; //add random noise
	measured_ang_vel = ang_vel; //add random noise
	for (i = 0; i < 8; i++) {
		
		//save every generated value as real position
		x += cos(angle)*speed;
		y += sin(angle)*speed;
		angle += ang_vel;
		angle = angleWrap(angle);
		
		//save every generated values as measured position
		measured_x += cos(angle)*speed + noise(e2); //add some noise
		measured_y += sin(angle)*speed + noise(e2); //add some noise
		measured_angle += ang_vel + noise(e2); //add some noise
		measured_angle = angleWrap(angle);
		
		measured_speed += noise(e2); //add some random noise
		measured_ang_vel += noise(e2); //add some random noise
		
		test_gps.pose.pose.position.x = measured_x;
		test_gps.pose.pose.position.y = measured_y;
		test_encoder.twist.twist.linear.x = measured_speed;
		test_imu.angular_velocity.z = measured_ang_vel;
		test_imu.orientation = angToQuaternion(measured_angle);
		
		test_gps_publisher.publish(test_gps);
		test_encoder_publisher.publish(test_encoder);
		test_imu_publisher.publish(test_imu);
		ros::Rate loop_rate(10);
		loop_rate.sleep();
		ros::spinOnce();
		
		std::cout << "curr pos" << x << ", " << y << std::endl;
		std::cout << "ekf pos" << pose_data.position.x << ", " << pose_data.position.y << std::endl;
				
	}
	
		EXPECT_NEAR(x, pose_data.position.x, 100);
		EXPECT_NEAR(y, pose_data.position.y, 100);
	
	//travel east for 30 seconds
	ang_vel = 0;
	speed = 4;
	measured_speed = speed; //add random noise
	measured_ang_vel = ang_vel; //add random noise
	for (i = 0; i < 30; i++) {
		
		//save every generated value as real position
		x += sin(angle)*speed;
		y += cos(angle)*speed;
		angle += ang_vel;
		angle = angleWrap(angle);
		
		//save every generated values as measured position
		measured_x += sin(angle)*speed + noise(e2); //add some noise
		measured_y += cos(angle)*speed + noise(e2); //add some noise
		measured_angle += ang_vel + noise(e2); //add some noise
		measured_angle = angleWrap(angle);
		
		measured_speed += noise(e2); //add some random noise
		measured_ang_vel += noise(e2); //add some random noise
		
		test_gps.pose.pose.position.x = measured_x;
		test_gps.pose.pose.position.y = measured_y;
		test_encoder.twist.twist.linear.x = measured_speed;
		test_imu.angular_velocity.z = measured_ang_vel;
		test_imu.orientation = angToQuaternion(measured_angle);
		
		test_gps_publisher.publish(test_gps);
		test_encoder_publisher.publish(test_encoder);
		test_imu_publisher.publish(test_imu);
		ros::Rate loop_rate(10);
		loop_rate.sleep();
		ros::spinOnce();
		
		std::cout << "curr pos" << x << ", " << y << std::endl;
		std::cout << "ekf pos" << pose_data.position.x << ", " << pose_data.position.y << std::endl;
				
	}
	
		EXPECT_NEAR(x, pose_data.position.x, 100);
		EXPECT_NEAR(y, pose_data.position.y, 100);

	//rotate to face the south west
	ang_vel = (1./8.)*M_PI;
	speed = 0;
	measured_speed = speed; //add random noise
	measured_ang_vel = ang_vel; //add random noise
	for (i = 0; i < 10; i++) {
		
		//save every generated value as real position
		x += sin(angle)*speed;
		y += cos(angle)*speed;
		angle += ang_vel;
		angle = angleWrap(angle);
		
		//save every generated values as measured position
		measured_x += sin(angle)*speed + noise(e2); //add some noise
		measured_y += cos(angle)*speed + noise(e2); //add some noise
		measured_angle += ang_vel + noise(e2); //add some noise
		measured_angle = angleWrap(angle);
		
		measured_speed += noise(e2); //add some random noise
		measured_ang_vel += noise(e2); //add some random noise
		
		test_gps.pose.pose.position.x = measured_x;
		test_gps.pose.pose.position.y = measured_y;
		test_encoder.twist.twist.linear.x = measured_speed;
		test_imu.angular_velocity.z = measured_ang_vel;
		test_imu.orientation = angToQuaternion(measured_angle);
		
		test_gps_publisher.publish(test_gps);
		test_encoder_publisher.publish(test_encoder);
		test_imu_publisher.publish(test_imu);
		ros::Rate loop_rate(10);
		loop_rate.sleep();
		ros::spinOnce();
		
		std::cout << "curr pos" << x << ", " << y << std::endl;
		std::cout << "ekf pos" << pose_data.position.x << ", " << pose_data.position.y << std::endl;
				
	}
	
		EXPECT_NEAR(x, pose_data.position.x, 100);
		EXPECT_NEAR(y, pose_data.position.y, 100);
	
	//travel south west for 40 seconds
	ang_vel = 0;
	speed = 2;
	measured_speed = speed; //add random noise
	measured_ang_vel = ang_vel; //add random noise
	for (i = 0; i < 40; i++) {
		
		//save every generated value as real position
		x += sin(angle)*speed;
		y += cos(angle)*speed;
		angle += ang_vel;
		angle = angleWrap(angle);
		
		//save every generated values as measured position
		measured_x += sin(angle)*speed + noise(e2); //add some noise
		measured_y += cos(angle)*speed + noise(e2); //add some noise
		measured_angle += ang_vel + noise(e2); //add some noise
		measured_angle = angleWrap(angle);
		
		measured_speed += noise(e2); //add some random noise
		measured_ang_vel += noise(e2); //add some random noise
		
		test_gps.pose.pose.position.x = measured_x;
		test_gps.pose.pose.position.y = measured_y;
		test_encoder.twist.twist.linear.x = measured_speed;
		test_imu.angular_velocity.z = measured_ang_vel;
		test_imu.orientation = angToQuaternion(measured_angle);
		
		test_gps_publisher.publish(test_gps);
		test_encoder_publisher.publish(test_encoder);
		test_imu_publisher.publish(test_imu);
		ros::Rate loop_rate(10);
		loop_rate.sleep();
		ros::spinOnce();
		
		std::cout << "curr pos" << x << ", " << y << std::endl;
		std::cout << "ekf pos" << pose_data.position.x << ", " << pose_data.position.y << std::endl;
				
	}
	
	
	EXPECT_NEAR(x, pose_data.position.x, 150);
	EXPECT_NEAR(y, pose_data.position.y, 150);
	double test = EKF::defineAngleInBounds(180.);
}


int main(int argc, char **argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "EKF");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

geometry_msgs::Quaternion angToQuaternion(double angle) {
	geometry_msgs::Quaternion quat_angle;
	double def_angle = angleWrap(angle);
	quat_angle.w = cos(def_angle/2.);
	quat_angle.x = 0;
	quat_angle.y = 0;
	quat_angle.z = sin(def_angle/2.);
	return quat_angle;
}

double angleWrap(double angle) {
	return(fmod(angle, 2.*M_PI) == fmod(angle, M_PI)) ? fmod(angle, M_PI) : 
	  (fmod(angle, M_PI) > 0) ? fmod(angle, M_PI) - M_PI : fmod(angle, M_PI) + M_PI;
}