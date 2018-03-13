/*
 * Created By: Min Gyo Kim
 * Created On: January 20, 2018
 * Description: Node implementation for Line Extractor Node
 */

#include <LineExtractorNode.h>

LineExtractorNode::LineExtractorNode(int argc,
                                     char** argv,
                                     std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string degree_polynomial_param = "degree_polynomial";
    int default_degree_polynomial       = 3;
    SB_getParam(private_nh,
                degree_polynomial_param,
                this->degreePoly,
                default_degree_polynomial);

    std::string lambda_param = "lambda";
    float default_lambda     = 0;
    SB_getParam(private_nh, lambda_param, this->lambda, default_lambda);

    std::string min_neighbours_param = "min_neighbours";
    int default_min_neighbours       = 3;
    SB_getParam(private_nh,
                min_neighbours_param,
                this->minNeighbours,
                default_min_neighbours);

    std::string radius_param = "radius";
    float default_radius     = 0.1;
    SB_getParam(private_nh, radius_param, this->radius, default_radius);

    std::string delta_x_param = "x_delta";
    float default_delta_x     = 0.01;
    SB_getParam(private_nh, delta_x_param, this->x_delta, default_delta_x);

    if (areParamsInvalid()) {
        ROS_DEBUG(
        "At least one of your parameters are negative; they should be "
        "positive!");
        ros::shutdown();
    }

    std::string topic_to_subscribe_to = "input_pointcloud"; // dummy topic name
    int refresh_rate                  = 10;
    subscriber                        = nh.subscribe(
    topic_to_subscribe_to, refresh_rate, &LineExtractorNode::pclCallBack, this);

    std::string topic_to_publish_to =
    "output_line_obstacle"; // dummy topic name
    uint32_t queue_size = 1;
    publisher           = private_nh.advertise<mapping_igvc::LineObstacle>(
    topic_to_publish_to, queue_size);

    std::string rviz_line_topic = "debug/output_line_obstacle";
    rviz_line_publisher = private_nh.advertise<visualization_msgs::Marker>(
    rviz_line_topic, queue_size);

    std::string rviz_cluster_topic = "debug/clusters";
    rviz_cluster_publisher = private_nh.advertise<visualization_msgs::Marker>(
            rviz_cluster_topic, queue_size);

    this->dbscan.setRadius(this->radius);
    this->dbscan.setMinNeighbours(this->minNeighbours);
}

void LineExtractorNode::pclCallBack(
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

    // extract lines from the pointcloud
    extractLines();

    return;
}

void LineExtractorNode::extractLines() {
    this->clusters = this->dbscan.findClusters(this->pclPtr);
    visualizeClusters();

    std::vector<Eigen::VectorXf> lines =
    regression.getLinesOfBestFit(this->clusters, this->degreePoly);

    std::vector<mapping_igvc::LineObstacle> line_obstacles =
    vectorsToMsgs(lines);

    for (unsigned int i = 0; i < line_obstacles.size(); i++) {
        publisher.publish(line_obstacles[i]);
    }

    visualizeLineObstacles(line_obstacles);

    return;
}

void LineExtractorNode::visualizeClusters() {
    std::vector<geometry_msgs::Point> cluster_points;
    std::vector<std_msgs::ColorRGBA> colors;
    convertClustersToPointsWithColors(this->clusters, cluster_points, colors);

    visualization_msgs::Marker::_scale_type scale =
            snowbots::RvizUtils::createrMarkerScale(1.0, 1.0, 1.0);

    std::string frame_id = "line_extractor_test";
    std::string ns       = "debug";

    visualization_msgs::Marker marker =
            snowbots::RvizUtils::displayPoints(cluster_points, colors, scale, frame_id, ns);

    rviz_cluster_publisher.publish(marker);
}

void LineExtractorNode::convertClustersToPointsWithColors(
        std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters,
        std::vector<geometry_msgs::Point> &cluster_points,
        std::vector<std_msgs::ColorRGBA> &colors) {
    std::vector<float> color_library_r = {1.0, 0.0, 0.0};
    std::vector<float> color_library_g = {0.0, 0.0, 1.0};
    std::vector<float> color_library_b = {0.0, 1.0, 0.0};

    for (unsigned int c = 0; c < clusters.size(); c++) {
        pcl::PointCloud<pcl::PointXYZ> cluster = clusters[c];

        std_msgs::ColorRGBA color;

        color.r = color_library_r[c % color_library_r.size()];
        color.g = color_library_g[c % color_library_g.size()];
        color.b = color_library_b[c % color_library_b.size()];
        color.a = 1.0;

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

void LineExtractorNode::visualizeLineObstacles(
std::vector<mapping_igvc::LineObstacle> line_obstacles) {
    std::vector<geometry_msgs::Point> points =
    convertLineObstaclesToPoints(line_obstacles);

    visualization_msgs::Marker::_color_type color =
    snowbots::RvizUtils::createMarkerColor(0.0, 1.0, 1.0, 1.0);
    visualization_msgs::Marker::_scale_type scale =
    snowbots::RvizUtils::createrMarkerScale(1.0, 1.0, 1.0);

    std::string frame_id = "line_extractor_test";
    std::string ns       = "debug";

    visualization_msgs::Marker marker =
    snowbots::RvizUtils::displayPoints(points, color, scale, frame_id, ns);

    rviz_line_publisher.publish(marker);
}

std::vector<geometry_msgs::Point>
LineExtractorNode::convertLineObstaclesToPoints(
std::vector<mapping_igvc::LineObstacle> line_obstacles) {
    std::vector<geometry_msgs::Point> points;

    for (unsigned int i = 0; i < line_obstacles.size(); i++) {
        mapping_igvc::LineObstacle line_obstacle = line_obstacles[i];

        for (float x = line_obstacle.x_min; x < line_obstacle.x_max;
             x += this->x_delta) {
            geometry_msgs::Point p;
            p.x = x;

            for (unsigned int i = 0; i < line_obstacle.coefficients.size();
                 i++) {
                p.y += line_obstacle.coefficients[i] * pow(x, i);
            }

            points.push_back(p);
        }
    }

    return points;
}

bool LineExtractorNode::areParamsInvalid() {
    return this->degreePoly < 0 || this->lambda < 0 ||
           this->minNeighbours < 0 || this->radius < 0;
}

std::vector<mapping_igvc::LineObstacle>
LineExtractorNode::vectorsToMsgs(std::vector<Eigen::VectorXf> vectors) {
    std::vector<mapping_igvc::LineObstacle> msgs;

    for (unsigned int i = 0; i < vectors.size(); i++) {
        msgs.push_back(vectorToLineObstacle(vectors[i], i));
    }

    return msgs;
}

mapping_igvc::LineObstacle
LineExtractorNode::vectorToLineObstacle(Eigen::VectorXf v,
                                        unsigned int cluster_index) {
    mapping_igvc::LineObstacle line_obstacle = mapping_igvc::LineObstacle();

    for (unsigned int i = 0; i < v.size(); i++) {
        line_obstacle.coefficients.push_back(v(i));
    }

    getClusterXRange(line_obstacle.x_min, line_obstacle.x_max, cluster_index);

    return line_obstacle;
}

void LineExtractorNode::getClusterXRange(double& xmin,
                                         double& xmax,
                                         unsigned int cluster_index) {
    pcl::PointCloud<pcl::PointXYZ> cluster = this->clusters[cluster_index];

    double min, max;

    if (cluster.size()) {
        min = max = cluster[0].x;
    } else {
        xmin = xmax = -1;
        return;
    }

    for (unsigned int i = 0; i < cluster.size(); i++) {
        if (cluster[i].x < min) { min = cluster[i].x; }
        if (cluster[i].x > max) { max = cluster[i].x; }
    }

    xmin = min;
    xmax = max;
}