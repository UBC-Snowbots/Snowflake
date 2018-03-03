/*
 * Created By: William Gu
 * Created On: Jan 20, 2018
 * Description: A path finding node which converts a given path into a Twist
 * message to send to the robot
 */

#include <PathFinding.h>

PathFinding::PathFinding(int argc, char** argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    uint32_t queue_size = 1;

    /*Assume initial coordinates: may need to change later*/
    robot_x_pos      = 0;
    robot_y_pos      = 0;
    robotOrientation = 0;

    std::string path_subscribe_topic = "/path"; // Setup subscriber to path
    path_subscriber                  = nh.subscribe(
    path_subscribe_topic, queue_size, &PathFinding::pathCallBack, this);

    std::string tf_subscribe_topic = "/tf"; // Setup subscriber to tf
    tf_subscriber                  = nh.subscribe(
    tf_subscribe_topic, queue_size, &PathFinding::tfCallBack, this);

    std::string twist_publish_topic =
    private_nh.resolveName("/cmd_vel"); // setup Publisher to twist
    twist_publisher =
    nh.advertise<geometry_msgs::Twist>(twist_publish_topic, queue_size);

    // Get Params
    SB_getParam(
    private_nh, "base_frame", base_frame, (std::string) "base_link");
    SB_getParam(
    private_nh, "global_frame", global_frame, (std::string) "odom_combined");
    SB_getParam(private_nh, "num_poses", num_poses, 10);
}

void PathFinding::pathCallBack(const nav_msgs::Path::ConstPtr& path_ptr) {
    nav_msgs::Path path_msg =
    *path_ptr; // Take required information from received message
    geometry_msgs::Twist twist_msg = pathToTwist(
    path_msg, robot_x_pos, robot_y_pos, robotOrientation, num_poses);
    twist_publisher.publish(twist_msg);
}

void PathFinding::tfCallBack(const tf2_msgs::TFMessageConstPtr tf_message) {
    // Check if the tf_message contains the transform we're looking for
    for (geometry_msgs::TransformStamped tf_stamped : tf_message->transforms) {
        // header frame might be base
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
            robot_x_pos      = x_pos;
            robot_y_pos      = y_pos;
            robotOrientation = yaw; // Orientation = rotation about z axis
        }
    }
}

geometry_msgs::Twist PathFinding::pathToTwist(nav_msgs::Path path_msg,
                                              double x_pos,
                                              double y_pos,
                                              double orientation,
                                              int num_poses) {
    geometry_msgs::Twist twist_msg; // Initialize velocity message

    std::vector<geometry_msgs::PoseStamped> inc_poses = path_msg.poses;

    std::vector<float> x_vectors; // holds x values for vectors
    std::vector<float> y_vectors; // holds y values for vectors
    calcVectors(inc_poses,
                x_vectors,
                y_vectors,
                num_poses,
                x_pos,
                y_pos); // x_vectors and y_vectors now updated

    float x_sum = weightedSum(x_vectors, num_poses);
    float y_sum = weightedSum(y_vectors, num_poses);

    float desired_angle = atan(y_sum / x_sum);
    float current_angle = orientation;

    float turn_rate = fmod(desired_angle - current_angle,
                           2 * M_PI); // Keep turn_rate between -2pi and 2pi

    // If pi < turn < 2pi or -2pi < turn < -pi, turn in other direction instead
    if (turn_rate > M_PI)
        turn_rate -= 2 * M_PI;
    else if (turn_rate < -M_PI)
        turn_rate += 2 * M_PI;

    float speed =
    1.0 - fabs(fmod(turn_rate, 2 * M_PI) /
               (2 * M_PI)); // Could multiply this by some factor to scale speed

    twist_msg.linear.x  = speed;
    twist_msg.angular.z = turn_rate;

    return twist_msg;
}

void PathFinding::calcVectors(
const std::vector<geometry_msgs::PoseStamped>& poses,
std::vector<float>& x_vectors,
std::vector<float>& y_vectors,
int num_poses,
double x_pos,
double y_pos) {
    x_vectors.push_back(poses[0].pose.position.x - x_pos);
    y_vectors.push_back(poses[0].pose.position.y - y_pos);
    for (int i = 1; i < num_poses; i++) {
        x_vectors.push_back(poses[i].pose.position.x -
                            poses[i - 1].pose.position.x);
        y_vectors.push_back(poses[i].pose.position.y -
                            poses[i - 1].pose.position.y);
    }
}

float PathFinding::weightedSum(const std::vector<float>& vectors,
                               int num_to_sum) {
    float weighted_sum = 0;
    for (int i = 0; i < num_to_sum; i++) {
        weighted_sum += vectors[i] / (i + 1); // 1/x scaling
    }
    return weighted_sum;
}
