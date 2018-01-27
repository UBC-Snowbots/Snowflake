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

    // *** Change to subscribe to path message
    // Setup Subscriber to path
    std::string topic_to_subscribe_to = "/CHANGE";
    int refresh_rate = 10;
    ros::Subscriber path_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &PathFinding::pathCallBack, this);

    // Setup Publisher to twist
    std::string topic_to_publish_to = "/cmd_vel";
    int queue_size = 1;
    ros::Publisher twist_publisher = nh.advertise<geometry_msgs::Twist>(topic_to_publish_to, queue_size);
}


//Subscriber callback
void PathFinding::pathCallBack(const nav_msgs::Path::ConstPtr& path_ptr) {
    nav_msgs::Path path_msg = *path_ptr; //Take required information from received message
    geometry_msgs::Twist twist_msg = pathToTwist(path_msg);


}

//Algorithm for processing path message and producing to twist message
geometry_msgs::Twist PathFinding::pathToTwist(nav_msgs::Path path_msg) {
    geometry_msgs::Twist twist_msg; //Initialize velocity message
    const int NUM_VECTORS = 50;

    std::vector<geometry_msgs::PoseStamped> inc_poses = path_msg.poses;
    std::vector<float> x_vectors; //holds x values for vectors
    std::vector<float> y_vectors; //holds y values for vectors
    calcVectors(inc_poses, x_vectors, y_vectors, NUM_VECTORS); //x_vectors and y_vectors now updated

    float x_sum = weightedXSum(x_vectors,NUM_VECTORS);
    float y_sum = weightedYSum(y_vectors,NUM_VECTORS);

    float desired_angle = atan(y_sum/x_sum);
    float current_angle; //Get orientation of robot somehow

    float turn_rate = desired_angle - current_angle;
    float speed; //The higher the needed turn rate, the lower the speed (for tight turns)

    twist_msg.linear.x = speed;
    twist_msg.angular.z = turn_rate;

    return twist_msg;
}

/**
 * Adds geometric vector values to two empty vectors, based on contents of an array of poses
 * @param poses : Array of pose messages, containing x and y positions to move to
 * @param x_vectors : Empty vector to represent x values of multiple geometric vectors
 * @param y_vectors : Empty vector to represent y values of multiple geometric vectors
 * @param num_vectors : Number of vectors to calculate
 */
void PathFinding::calcVectors(const std::vector<geometry_msgs::PoseStamped> &poses, std::vector<float> &x_vectors, std::vector<float> &y_vectors, int num_vectors) {

    for (int i = 0; i < num_vectors; i++){
        x_vectors.push_back(poses[1].pose.position.x - poses[0].pose.position.x);
        y_vectors.push_back(poses[1].pose.position.y - poses[0].pose.position.y);
    }
}

/**
 * Calculates a weighted x value given a list of x values in vectors
 * The x values near the front of the list are given a higher weight (since they are imminent)
 * Currently the scaling for the weights are LINEAR
 * @param x_vectors
 * @param numToSum number of x vectors to sum, should be less than or equal to NUM_VECTORS
 * @return weighted X values of given vectors
 */
float PathFinding::weightedXSum (std::vector<float> &x_vectors, int num_to_sum) {
    float weighted_x_sum = 0;
    for (int i = 0; i < num_to_sum; i++){
        weighted_x_sum += x_vectors[i] * ((num_to_sum-i)/num_to_sum);
    }
    return weighted_x_sum;
}

/**
 * Calculates a weighted y value given a list of y values in vectors
 * The y values near the front of the list are given a higher weight (since they are imminent)
 * @param y_vectors
 * @param numToSum number of y vectors to sum, should be less than or equal to NUM_VECTORS
 * @return weighted Y values of given vectors
 */
float PathFinding::weightedYSum (std::vector<float> &y_vectors, int num_to_sum) {
    float weighted_y_sum = 0;
    for (int i = 0; i < num_to_sum; i++){
        weighted_y_sum += y_vectors[i] * ((num_to_sum-i)/num_to_sum);
    }
    return weighted_y_sum;
}

//https://answers.ros.org/question/58742/quaternion-to-roll-pitch-yaw/
//https://stackoverflow.com/questions/1568568/how-to-convert-euler-angles-to-directional-vector Pitch/Yaw/