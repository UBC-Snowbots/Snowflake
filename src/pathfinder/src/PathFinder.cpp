/*
 * Created By: Sophie Chen
 * Created On: March 8, 2023
 * Description: Given current rover coordinates and destination coordinates, 
 * returns a vector that represents the shortest path to destination.
*/


/* Implementation notes: Use rover_movement_vector = a - b where a are the coordinates of the destination and b are the coordinates of the rover*/

#include <PathFinder.h>
#include <algorithm>

PathFinder::PathFinder(int argc, char **argv, std::string node_name) {
    // initializing variables (to be determined)
    last_coord_x = 0;
    last_coord_y = 0;
    angular_threshold = 4; 

    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "oscar_topic";
    int queue_size                    = 1;
    my_subscriber                     = nh.subscribe(topic_to_subscribe_to, queue_size, &PathFinder::CallBack, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("publish_topic");
    queue_size        = 1;
    my_publisher      = private_nh.advertise<geometry_msgs::Twist>(topic, queue_size); // tragectory vector
}

void PathFinder::update_movement_vector(float x_rover, float y_rover, float x_dest, float y_dest) {
    // Calculate the linear and angular components of the movement vector
    linear_x  = x_dest - x_rover;
    angular_y = y_dest - y_rover;

    // Calculate the angle to turn to reach the destination
    float temp1         = atan2(linear_x, angular_y); 
    float angle_to_turn = temp1 * 180/M_PI;

    // Adjust the angular component of the movement vector by the difference
    // between the angle to turn and the current cardinal direction
    if (abs(angle_to_turn - cardinal_direction) > angular_threshold) {
         if (abs(angle_to_turn - cardinal_direction) > 180) {
            angular_y = 360 - std::max(angle_to_turn, cardinal_direction) + std::min(angle_to_turn, cardinal_direction);
         } else {
            angular_y = angle_to_turn - cardinal_direction;
         }
    }

    // Create a twist message for the rover's trajectory vector
    geometry_msgs::Twist rover_traj_vector;

    // Set the angular component of the rover's trajectory vector
    rover_traj_vector.angular.y = angular_y;

    // Publish the rover's trajectory vector
    my_publisher.publish(rover_traj_vector);

    // Calculate the angle of the cardinal direction
    float lat_diff  = last_coord_x - x_rover;
    float long_diff = last_coord_y - y_rover;

    float temp         = atan2(lat_diff, long_diff); // in radians
    cardinal_direction = temp * 180/M_PI;

    // Store the current rover coordinates as the last coordinates
    last_coord_x = x_rover;
    last_coord_y = y_rover;

    //if (angle to desired angle < some threshold)
    //then go forward
    //else turn right/left based on +/- of angle

    // rover_traj_vector.linear.x  = linear_x;
//  float dot = desTraj.dot(currTraj);
//  float mag1 = norm(currTraj);
//  float mag2 = norm(desTraj);

//  float cos_theta = dot/(mag1 * mag2);
//  float angle = fastAtan2(sqrt(1 - cos_theta*cos_theta), cos_theta);

// float cross = currTraj.x * desTraj.y - currTraj.y * desTraj.x;



// if (cross > 0){
// std::string wee("angle is: ");
// wee += std::to_string(angle);
//     cv::putText(outputImage, wee, angle_to_go_point, font1, fontScale, cv::Scalar(0, 100, 0), thickness, lineType, false);
//     cmd.angular.z = -40;
// }else if (cross < 0){
// std::string wee("angle is: ");
// wee += std::to_string(-angle);
//     cv::putText(outputImage, wee, angle_to_go_point, font1, fontScale, cv::Scalar(110, 0, 0), thickness, lineType, false);

//     cmd.angular.z = 40;

// }
}

void PathFinder::CallBack(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    float x_rover = msg->latitude;
    float y_rover = msg->longitude;
    float x_dest  = 10.0000034;
    float y_dest  = 10.0000010;
    ROS_WARN("Received message, latitude: %f", x_rover);
    update_movement_vector(x_rover, y_rover, x_dest, y_dest);
}