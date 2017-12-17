/*
 * Created By: Marcus Swift
 * Created On: November 20th, 2017
 * Description: An extended kalman filter node that takes in sensor data from
 * the GPS, encoders and IMU and returns the bots estimated postion and orientation	
 */


#include <EKF.h>


EKF::EKF(int argc, char **argv, std::string node_name) {
	// Setup NodeHandle
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Setup Subscribers
    gps_sub = nh.subscribe("/gps", 10, &EKF::gpsCallBack, this);
	encoder_sub = nh.subscribe("/encoder", 10, &EKF::encoderCallBack, this);
	imu_sub = nh.subscribe("/imu", 10, &EKF::imuCallBack, this);

    // Setup Publisher
    pose_pub = nh.advertise<geometry_msgs::Pose>("/cmd_pose", 10);
	
	p << 0, 0, 0,
		 0, 0, 0,
		 0, 0, 0;
	
	bot_position.position.x = 482003;
	bot_position.position.y = 5456550;
	bot_position.position.z = 0;
	bot_position.orientation.x = 0;
	bot_position.orientation.y = 0;
	bot_position.orientation.z = 0;
	bot_position.orientation.w = 1;

	time = ros::Time::now().toSec();
}
	

void EKF::gpsCallBack(const nav_msgs::Odometry::ConstPtr &gps_message) {
	
	gps_data.pose = gps_message->pose;
	gps_data.twist = gps_message->twist;
	position << bot_position.position.x, bot_position.position.y, quaternionToAngle(bot_position.orientation);
	//Evaluate residual (innovation)
	z  << gps_data.pose.pose.position.x, gps_data.pose.pose.position.y, quaternionToAngle(imu_data.orientation);

	s = r + h*p*h.transpose();
	k = p*h.transpose()*s.inverse();           //Kalman gain
	position = position + k*(z - h*position);  //update the  expected value
	p = p-p*h.transpose()*s.inverse()*h*p;     //update the Covariance
	
	bot_position.position.x = position(0);
	bot_position.position.y = position(1);
	bot_position.orientation = angleToQuaternion(position(2));
	
	publishPose(bot_position);

}
	

void EKF::encoderCallBack(const nav_msgs::Odometry::ConstPtr &encoder_message) {

	encoder_data.pose = encoder_message->pose;
	encoder_data.twist = encoder_message->twist;
	double angle = quaternionToAngle(bot_position.orientation);
	double speed = encoder_data.twist.twist.linear.x;
	double dt = 1;//time - ros::Time::now().toSec(); set to 1 for rostest
    time += dt;
           
    //Jacobian of process model:
    j << 1, 0, -dt*speed*sin(angle),
		 0, 1,  dt*speed*cos(angle),
         0, 0,  1;
    ju << dt*cos(angle), 0,
		  dt*sin(angle), 0,
		  0,             dt;
    q = ju*pu*ju.transpose() + q1;
    p = j*p*j.transpose()+ q;

	bot_position.position.x += dt*speed*cos(angle);
	bot_position.position.y += dt*speed*sin(angle);
	angle += dt*imu_data.angular_velocity.z;
	defineAngleInBounds(angle);
	bot_position.orientation = angleToQuaternion(angle);

}
	

void EKF::imuCallBack(const sensor_msgs::Imu::ConstPtr &imu_message) {
	
	imu_data.orientation = imu_message->orientation;
	imu_data.angular_velocity = imu_message->angular_velocity;
	imu_data.linear_acceleration = imu_message->linear_acceleration;
}
	 

void EKF::publishPose(geometry_msgs::Pose pose_msg) {
	pose_pub.publish(pose_msg);
}

	 
double EKF::defineAngleInBounds(double angle) {
	double rebounded_angle;
	
	if (fmod(angle, 2*M_PI) == fmod(angle, M_PI)) {
		rebounded_angle = fmod(angle, M_PI);
	} else if ((fmod(angle, 2*M_PI) > fmod(angle, M_PI)) && (fmod(angle, M_PI) != 0)) {
		rebounded_angle = fmod(angle, M_PI) - M_PI;
	} else {
		rebounded_angle = fmod(angle, M_PI) + M_PI;
	}
	
	return rebounded_angle;
}


geometry_msgs::Quaternion EKF::angleToQuaternion(double angle){
	
	angle = defineAngleInBounds(angle);
	
	geometry_msgs::Quaternion quat;
	quat.x = 0;
	quat.y = 0;
	quat.z = sin(angle/2);
	quat.w = cos(angle/2);
	
	return quat;
}


double EKF::quaternionToAngle(geometry_msgs::Quaternion quat_angle) {
	
	double angle;
	
	Eigen::Quaternion<double> eigen_quat(quat_angle.w,quat_angle.x, quat_angle.y, quat_angle.z);
	Eigen::AngleAxis<double> ang_axis(eigen_quat);
	
	if (ang_axis.axis()(2) < 0) {
		angle = -ang_axis.angle();
	} else {
		angle  = ang_axis.angle();
	}
	
	return defineAngleInBounds(angle);
}
