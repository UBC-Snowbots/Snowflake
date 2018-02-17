/*
 * Created By: William Gu
 * Created On: Jan 20, 2018
 * Description: A path finding node which converts a given path into a Twist message to
 *              send to the robot
 */

#include <PathFinding.h>

PathFinding::PathFinding(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    uint32_t queue_size = 1;

    /*Assume initial coordinates: may need to change later*/
    robot_x_pos = 0;
    robot_y_pos = 0;
    robotOrientation = 0;

    // Setup Subscriber to path
    std::string path_subscribe_topic = "/path";
    path_subscriber = nh.subscribe(path_subscribe_topic, queue_size, &PathFinding::pathCallBack, this);

    std::string tf_subscribe_topic = "/tf";
    tf_subscriber = nh.subscribe(tf_subscribe_topic, queue_size, &PathFinding::tfCallBack, this);

    // Setup Publisher to twist
    std::string twist_publish_topic = private_nh.resolveName("/cmd_vel");
    twist_publisher = nh.advertise<geometry_msgs::Twist>(twist_publish_topic, queue_size);

    // Get Params
    SB_getParam(private_nh, "base_frame", base_frame, (std::string) "base_link");
    SB_getParam(private_nh, "global_frame", global_frame, (std::string) "odom_combined");
}


/*
 * Path call back. Produces twist msg and publishes it based on path and current position
 */
void PathFinding::pathCallBack(const nav_msgs::Path::ConstPtr& path_ptr) {
    nav_msgs::Path path_msg = *path_ptr; //Take required information from received message
    geometry_msgs::Twist twist_msg = pathToTwist(path_msg, robot_x_pos, robot_y_pos, robotOrientation);
    twist_publisher.publish(twist_msg);
}

/**
 * Stores the current position and orientation of the robot in the global frame, in member variables
 */
void PathFinding::tfCallBack(const tf2_msgs::TFMessageConstPtr tf_message) {
    // Check if the tf_message contains the transform we're looking for
    for (geometry_msgs::TransformStamped tf_stamped : tf_message->transforms) {

        //header frame might be base
        if (tf_stamped.header.frame_id == global_frame && tf_stamped.child_frame_id == base_frame) {
            // We've found the transform we're looking for, so see how close we
            // are to the waypoint
            double x_pos = tf_stamped.transform.translation.x;
            double y_pos = tf_stamped.transform.translation.y;

            double quatx = tf_stamped.transform.rotation.x;
            double quaty = tf_stamped.transform.rotation.y;
            double quatz = tf_stamped.transform.rotation.z;
            double quatw = tf_stamped.transform.rotation.w;
            tf::Quaternion q(quatx, quaty, quatz, quatw);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            //Set member variables
            robot_x_pos = x_pos;
            robot_y_pos = y_pos;
            robotOrientation = yaw; //Orientation = rotation about z axis
        }
    }
}

/**
 * Produces twist message from a path message and current robot coordinates/orientation
 * @param path_msg
 * @param x_pos
 * @param y_pos
 * @param orientation
 * @return a path message
 */
geometry_msgs::Twist PathFinding::pathToTwist(nav_msgs::Path path_msg, double x_pos, double y_pos, double orientation) {
    geometry_msgs::Twist twist_msg; //Initialize velocity message
    const int NUM_VECTORS = 10; //Number of vectors to process including initial robot position

    std::vector<geometry_msgs::PoseStamped> inc_poses = path_msg.poses;

    std::vector<float> x_vectors; //holds x values for vectors
    std::vector<float> y_vectors; //holds y values for vectors
    calcVectors(inc_poses, x_vectors, y_vectors, NUM_VECTORS, x_pos, y_pos); //x_vectors and y_vectors now updated

    float x_sum = weightedSum(x_vectors, NUM_VECTORS);
    float y_sum = weightedSum(y_vectors, NUM_VECTORS);

    float desired_angle = atan(y_sum/x_sum);
    float current_angle = orientation;

    float turn_rate = fmod(desired_angle - current_angle, 2*M_PI);
    if (turn_rate > M_PI)
        turn_rate -= 2 * M_PI;
    else if (turn_rate < -M_PI)
        turn_rate += 2 * M_PI;

    float speed = 1.0 - fabs(fmod(turn_rate,2*M_PI)/(2*M_PI));

    twist_msg.linear.x = speed;
    twist_msg.angular.z = turn_rate;

    return twist_msg;
}

/**
 * Adds geometric vector values to two empty vectors, based on contents of an array of poses
 * @param poses : Array of pose messages, containing x and y positions to move to
 * @param x_vectors : Empty vector to represent x values of multiple geometric vectors
 * @param y_vectors : Empty vector to represent y values of multiple geometric vectors
 * @param num_vectors : Number of vectors to calculate (including initial robot position), must be >=1 but <=size(x_vectors)-1
 */
void PathFinding::calcVectors(const std::vector<geometry_msgs::PoseStamped> &poses, std::vector<float> &x_vectors, std::vector<float> &y_vectors, int num_vectors, double x_pos, double y_pos) {
    x_vectors.push_back(poses[0].pose.position.x - x_pos);
    y_vectors.push_back(poses[0].pose.position.y - y_pos);
    for (int i = 1; i < num_vectors; i++){
        x_vectors.push_back(poses[i].pose.position.x - poses[i-1].pose.position.x);
        y_vectors.push_back(poses[i].pose.position.y - poses[i-1].pose.position.y);
    }
}

/**
 * Calculates a weighted value given a std::vector of geometric vectors
 * The values near the front of the list are given a higher weight (since they are imminent)
 * Currently the scaling for the weights are LINEAR
 * @param vectors (represents geometric vector values for one dimension, i.e. x values in some geometric vectors)
 * @param numToSum number of elements to sum, should be less than or equal to size of vectors
 * @return weighted values of given geometric vectors
 */
float PathFinding::weightedSum(const std::vector<float> &vectors, int num_to_sum) {
    float weighted_sum = 0;
    for (int i = 0; i < num_to_sum; i++){
        //weighted_sum += vectors[i] * ((float)(num_to_sum-i)/(num_to_sum)); (LINEAR SCALING)
        weighted_sum += vectors[i]/(i+1); //1/x scaling
    }
    return weighted_sum;
}
