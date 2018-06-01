/*
 * Created By: William Gu
 * Created On: Jan 20, 2018
 * Description: A path finding node which converts a given path into a Twist
 * message to send to the robot
 */

#include <PathToTwistNode.h>

PathToTwistNode::PathToTwistNode(int argc, char** argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    uint32_t queue_size = 1;

    /*No received coordinates from tf on launch so not valid */
    valid_cood = false;

    std::string path_subscribe_topic = "/path"; // Setup subscriber to path
    path_subscriber                  = nh.subscribe(
    path_subscribe_topic, queue_size, &PathToTwistNode::pathCallBack, this);

    std::string tf_subscribe_topic = "/tf"; // Setup subscriber to tf
    tf_subscriber                  = nh.subscribe(
    tf_subscribe_topic, queue_size, &PathToTwistNode::tfCallBack, this);

    std::string twist_publish_topic =
    private_nh.resolveName("/cmd_vel"); // setup Publisher to twist
    twist_publisher =
    nh.advertise<geometry_msgs::Twist>(twist_publish_topic, queue_size);

    // Get Params
    SB_getParam(
    private_nh, "base_frame", base_frame, (std::string) "base_link");
    SB_getParam(
    private_nh, "global_frame", global_frame, (std::string) "odom_combined");
    SB_getParam(private_nh, "linear_speed_scaling_factor", linear_speed_scaling_factor, 1.0);
    SB_getParam(private_nh, "angular_speed_scaling_factor", angular_speed_scaling_factor, 1.0);
    SB_getParam(private_nh, "max_linear_speed", max_linear_speed, 1.0);
    SB_getParam(private_nh, "max_angular_speed", max_angular_speed, 1.0);
    SB_getParam(private_nh, "path_dropoff_factor", path_dropoff_factor, 20);

}

void PathToTwistNode::pathCallBack(const nav_msgs::Path::ConstPtr& path_ptr) {
    nav_msgs::Path path_msg =
    *path_ptr; // Take required information from received message
    geometry_msgs::Twist twist_msg = pathToTwist(path_msg,
                                                 robot_x_pos,
                                                 robot_y_pos,
                                                 robot_orientation,
                                                 valid_cood);
    twist_publisher.publish(twist_msg);
}

void PathToTwistNode::tfCallBack(const tf2_msgs::TFMessageConstPtr tf_message) {
    // Check if the tf_message contains the transform we're looking for
    for (geometry_msgs::TransformStamped tf_stamped : tf_message->transforms) {
        if (tf_stamped.header.frame_id == global_frame &&
            tf_stamped.child_frame_id == base_frame) {
            double x_pos = tf_stamped.transform.translation.x;
            double y_pos = tf_stamped.transform.translation.y;

            double quatx = tf_stamped.transform.rotation.x;
            double quaty = tf_stamped.transform.rotation.y;
            double quatz = tf_stamped.transform.rotation.z;
            double quatw = tf_stamped.transform.rotation.w;
            tf::Quaternion quat(quatx, quaty, quatz, quatw);
            tf::Matrix3x3 rot_matrix(quat);
            double roll, pitch, yaw;
            rot_matrix.getRPY(roll, pitch, yaw);

            // Set member variables
            robot_x_pos = x_pos;
            robot_y_pos = y_pos;
            robot_orientation =
            yaw; // Orientation = rotation about z axis (yaw)
            valid_cood = true;
        }
    }
}

geometry_msgs::Twist PathToTwistNode::pathToTwist(nav_msgs::Path path_msg,
                                              double x_pos,
                                              double y_pos,
                                              double orientation,
                                              bool valid_cood) {
    geometry_msgs::Twist twist_msg; // Initialize velocity message

    if (!valid_cood || path_msg.poses.size() == 0) { // No TF received yet so don't move
        twist_msg.linear.x  = 0;
        twist_msg.angular.z = 0;
        return twist_msg;
    }

    std::vector<geometry_msgs::PoseStamped> inc_poses = path_msg.poses;

    std::vector<double> x_vectors; // holds x values for vectors
    std::vector<double> y_vectors; // holds y values for vectors
    calcVectors(inc_poses,
                x_vectors,
                y_vectors,
                x_pos,
                y_pos); // x_vectors and y_vectors now updated

    double x_sum = weightedSum(x_vectors, path_dropoff_factor); //-1 because number of vectors is one less than number of poses
    double y_sum = weightedSum(y_vectors, path_dropoff_factor);

    double desired_angle = atan(y_sum / x_sum);

    // Handle case where desired angle is behind us
    if (x_sum < 0) {
        desired_angle += M_PI;
    }

    double turn_rate = fmod(desired_angle - orientation,
                           2 * M_PI); // Keep turn_rate between -2pi and 2pi

    // If pi < turn < 2pi or -2pi < turn < -pi, turn in other direction instead
    if (turn_rate > M_PI)
        turn_rate -= 2 * M_PI;
    else if (turn_rate < -M_PI)
        turn_rate += 2 * M_PI;

    // At this point, turn rate should be between -pi and pi
    /*
    double speed =
    1.0 - fabs(fmod(turn_rate, M_PI) /
               (M_PI)); // Could multiply this by some factor to scale speed*/
    double speed = exp(-pow(turn_rate, 2) / 0.4);

    // Scale speeds
    speed *= linear_speed_scaling_factor;
    turn_rate *= angular_speed_scaling_factor;

    // Cap speeds
    speed = std::max(-(double)max_linear_speed, std::min(speed, (double)max_linear_speed));
    turn_rate = std::max(-(double)max_angular_speed, std::min(turn_rate, (double)max_angular_speed));

    twist_msg.linear.x  = speed;
    twist_msg.angular.z = turn_rate;
    return twist_msg;
}

void PathToTwistNode::calcVectors(
const std::vector<geometry_msgs::PoseStamped>& poses,
std::vector<double>& x_vectors,
std::vector<double>& y_vectors,
double x_pos,
double y_pos) {
    double x; //x magnitude in pending vector
    double y; //y magnitude in pending vector
    double mag; //absolute magnitude in pending vector

    for (int i = 0; i < poses.size(); i++) {
        x = poses[i].pose.position.x - x_pos;
        y = poses[i].pose.position.y - y_pos;
        mag = sqrt(pow(x, 2) + pow(y, 2));
        //Calculate unit vector
        x /= mag;
        y /= mag;
        x_vectors.push_back(x);
        y_vectors.push_back(y);
    }
}

double PathToTwistNode::weightedSum(const std::vector<double>& vectors, int path_dropoff_factor) {
    double weighted_sum = 0;
    for (int i = 0; i < vectors.size(); i++) {
        //weighted_sum += vectors[i] / (i + 1); // 1/x scaling
        weighted_sum += vectors[i] * exp (-pow(i+1, 2) / path_dropoff_factor) ;
    }
    return weighted_sum;
}
