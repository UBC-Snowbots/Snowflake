/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Node implementation for Ball Extractor Node
 */

#include <BallExtractorNode.h>

BallExtractorNode::BallExtractorNode(int argc,
                                     char** argv,
                                     std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string min_neighbours_param = "min_neighbours";
    int default_min_neighbours       = 3;
    SB_getParam(private_nh,
                min_neighbours_param,
                this->minNeighbours,
                default_min_neighbours);

    std::string radius_param = "radius";
    float default_radius     = 0.1;
    SB_getParam(private_nh, radius_param, this->radius, default_radius);

    std::string scale_param = "scale";
    float default_scale     = 0.1;
    SB_getParam(private_nh, scale_param, this->scale, default_scale);

    std::string frame_id_param   = "frame_id";
    std::string default_frame_id = "ball_extractor_test";
    SB_getParam(private_nh, frame_id_param, this->frame_id, default_frame_id);

    if (areParamsInvalid()) {
        ROS_DEBUG(
        "Detected invalid params - make sure all params are positive");
        ros::shutdown();
    }

    std::string topic_to_subscribe_to = "input_pointcloud"; // dummy topic name
    int refresh_rate                  = 10;
    subscriber                        = nh.subscribe(
    topic_to_subscribe_to, refresh_rate, &BallExtractorNode::pclCallBack, this);

    std::string topic_to_publish_to =
    "output_ball_obstacle"; // dummy topic name
    uint32_t queue_size = 1;
    publisher =
    private_nh.advertise<geometry_msgs::Point>(topic_to_publish_to, queue_size);

    std::string rviz_cluster_topic = "debug/clusters";
    rviz_cluster_publisher = private_nh.advertise<visualization_msgs::Marker>(
    rviz_cluster_topic, queue_size);

    std::string rviz_ball_topic = "debug/ball";
    rviz_ball_publisher = private_nh.advertise<visualization_msgs::Marker>(
    rviz_ball_topic, queue_size);
}

void BallExtractorNode::pclCallBack(
const sensor_msgs::PointCloud2ConstPtr processed_pcl) {
    pcl::PCLPointCloud2 pcl_pc2;

    // convert sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl_conversions::toPCL(*processed_pcl, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(
    new pcl::PointCloud<pcl::PointXYZ>);

    // convert pcl::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    // store converted pointcloud for use
    this->pclPtr = temp_cloud;

    // extract ball from the pointcloud
    extractBall();

    return;
}

void BallExtractorNode::extractBall() {
    this->clusters =
    DBSCAN::getClusters(this->pclPtr, this->minNeighbours, this->radius, DBSCAN::YZ);

    if (this->clusters.size() < 1) { return; }

    geometry_msgs::Point center_of_ball = this->getCenterOfCluster(this->clusters[0]);

    this->publisher.publish(center_of_ball);

    visualizeClusters();
    visualizeBall(center_of_ball);

    return;
}

void BallExtractorNode::visualizeBall(geometry_msgs::Point ball) {
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.r = 0;
    color.b = 1;
    color.g = 1;

    std::vector<std_msgs::ColorRGBA> colors = {color};

    visualization_msgs::Marker::_scale_type scale =
    snowbots::RvizUtils::createrMarkerScale(
    this->scale, this->scale, this->scale);

    std::string ns = "debug";

    visualization_msgs::Marker marker =
    snowbots::RvizUtils::createMarker(ball, colors, scale, this->frame_id, ns);

    rviz_ball_publisher.publish(marker);
}

// TODO: duplicate function from LineExtractorNode - refactor this
void BallExtractorNode::visualizeClusters() {
    std::vector<geometry_msgs::Point> cluster_points;
    std::vector<std_msgs::ColorRGBA> colors;
    convertClustersToPointsWithColors(this->clusters, cluster_points, colors);

    visualization_msgs::Marker::_scale_type scale =
    snowbots::RvizUtils::createrMarkerScale(
    this->scale, this->scale, this->scale);

    std::string ns = "debug";

    visualization_msgs::Marker marker = snowbots::RvizUtils::createMarker(
    cluster_points, colors, scale, this->frame_id, ns);

    rviz_cluster_publisher.publish(marker);
}

void BallExtractorNode::convertClustersToPointsWithColors(
std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters,
std::vector<geometry_msgs::Point>& cluster_points,
std::vector<std_msgs::ColorRGBA>& colors) {
    std::vector<float> color_library_r = {1.0, 0.0, 0.0};
    std::vector<float> color_library_g = {0.0, 0.0, 1.0};
    std::vector<float> color_library_b = {0.0, 1.0, 0.0};

    for (unsigned int c = 0; c < clusters.size(); c++) {
        pcl::PointCloud<pcl::PointXYZ> cluster = clusters[c];

        // assign color to this cluster
        std_msgs::ColorRGBA color;

        color.r = color_library_r[c % color_library_r.size()];
        color.g = color_library_g[c % color_library_g.size()];
        color.b = color_library_b[c % color_library_b.size()];
        color.a = 1.0;

        // push all the points in this cluster along with its color
        for (unsigned int p = 0; p < cluster.size(); p++) {
            pcl::PointXYZ pcl_point = cluster[p];

            geometry_msgs::Point msg_point;
            msg_point.x = pcl_point.x;
            msg_point.y = pcl_point.y;

            cluster_points.push_back(msg_point);
            colors.push_back(color);
        }
    }
}

bool BallExtractorNode::areParamsInvalid() {
    return this->minNeighbours < 0 || this->radius < 0;
}

geometry_msgs::Point
BallExtractorNode::getCenterOfCluster(const pcl::PointCloud<pcl::PointXYZ>& cluster) {
    struct {
        bool operator()(pcl::PointXYZ a, pcl::PointXYZ b) const {
            return a.y < b.y;
        }
    } lessY;

    float y_min = std::min_element(cluster.begin(), cluster.end(), lessY)->y;
    float y_max = std::max_element(cluster.begin(), cluster.end(), lessY)->y;

    struct {
        bool operator()(pcl::PointXYZ a, pcl::PointXYZ b) const {
            return a.z < b.z;
        }
    } lessZ;

    float z_min = std::min_element(cluster.begin(), cluster.end(), lessZ)->z;
    float z_max = std::max_element(cluster.begin(), cluster.end(), lessZ)->z;

    struct {
        bool operator()(pcl::PointXYZ a, pcl::PointXYZ b) const {
            return a.x < b.x;
        }
    } lessX;

    float x_min = std::min_element(cluster.begin(), cluster.end(), lessX)->x;

    geometry_msgs::Point center;
    center.y = (y_min + y_max) / 2;
    center.z = (z_min + z_max) / 2;
    center.x = x_min;

    return center;
}
