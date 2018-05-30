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

    std::string scale_param = "scale";
    float default_scale     = 0.1;
    SB_getParam(private_nh, scale_param, this->scale, default_scale);

    std::string frame_id_param   = "frame_id";
    std::string default_frame_id = "base_link";
    SB_getParam(private_nh, frame_id_param, this->frame_id, default_frame_id);

    if (areParamsInvalid()) {
        ROS_DEBUG(
        "Detected invalid params - make sure all params are positive");
        ros::shutdown();
    }

    std::string topic_to_subscribe_to = "input_pointcloud"; // dummy topic name
    uint32_t queue_size                    = 1;
    subscriber                        = nh.subscribe(
    topic_to_subscribe_to, queue_size, &LineExtractorNode::pclCallBack, this);

    std::string topic_to_publish_to =
    "output_line_obstacle"; // dummy topic name
    queue_size = 10;
    publisher           = private_nh.advertise<mapping_igvc::LineObstacle>(
    topic_to_publish_to, queue_size);

    std::string rviz_line_topic = "debug/output_line_obstacle";
    rviz_line_publisher = private_nh.advertise<visualization_msgs::Marker>(
    rviz_line_topic, queue_size);

    std::string rviz_cluster_topic = "debug/clusters";
    rviz_cluster_publisher = private_nh.advertise<visualization_msgs::Marker>(
    rviz_cluster_topic, queue_size);
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

    // save the timestamp of the received pointcloud
    this->last_cloud_time = processed_pcl->header.stamp;

    // extract lines from the pointcloud
    extractLines();

    return;
}

void LineExtractorNode::extractLines() {
    DBSCAN dbscan(this->minNeighbours, this->radius);
    this->clusters = dbscan.findClusters(this->pclPtr);

    // TODO: Put some of this functionality in another function
    std::vector<std::pair<Eigen::VectorXf, double>> x_dependent_lines =
            regression.getLinesOfBestFitWithStandardError(this->clusters, this->degreePoly, this->lambda);

    std::vector<pcl::PointCloud<pcl::PointXYZ>> switched_clusters = switchXandYinClusters(this->clusters);

    std::vector<std::pair<Eigen::VectorXf, double>> y_dependent_lines =
            regression.getLinesOfBestFitWithStandardError(switched_clusters, this->degreePoly, this->lambda);

    // TODO: This expression is bit redundant
    // choose whichever of the x/y dependent lines fit each cluster the best
    std::vector<mapping_igvc::LineObstacle> line_obstacles;
    for (int i = 0; i < clusters.size(); i++){
        if (x_dependent_lines[i].second < y_dependent_lines[i].second) {
            mapping_igvc::LineObstacle line = vectorToLineObstacle(x_dependent_lines[i].first, clusters[i]);
            line.dependent_on = mapping_igvc::LineObstacle::DEPENDENT_ON_X;
            line_obstacles.emplace_back(line);
        } else {
            mapping_igvc::LineObstacle line = vectorToLineObstacle(y_dependent_lines[i].first, switched_clusters[i]);
            line.dependent_on = mapping_igvc::LineObstacle::DEPENDENT_ON_Y;
            line_obstacles.emplace_back(line);
        }
    }

    for (mapping_igvc::LineObstacle& line_obstacle : line_obstacles){
        publisher.publish(line_obstacle);
    }

    visualizeClusters();
    visualizeLineObstacles(line_obstacles);

    return;
}

void LineExtractorNode::visualizeClusters() {
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

void LineExtractorNode::convertClustersToPointsWithColors(
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

void LineExtractorNode::visualizeLineObstacles(
std::vector<mapping_igvc::LineObstacle> line_obstacles) {
    std::vector<geometry_msgs::Point> lines_points =
    convertLineObstaclesToPoints(line_obstacles);

    visualization_msgs::Marker::_color_type color =
    snowbots::RvizUtils::createMarkerColor(0.0, 1.0, 1.0, 1.0);
    visualization_msgs::Marker::_scale_type scale =
    snowbots::RvizUtils::createrMarkerScale(
    this->scale, this->scale, this->scale);

    std::string ns = "debug";

    visualization_msgs::Marker marker =
    snowbots::RvizUtils::createMarker(lines_points,
                                      color,
                                      scale,
                                      this->frame_id,
                                      ns,
                                      visualization_msgs::Marker::POINTS);

    rviz_line_publisher.publish(marker);
}

std::vector<geometry_msgs::Point>
LineExtractorNode::convertLineObstaclesToPoints(
std::vector<mapping_igvc::LineObstacle> line_obstacles) {
    std::vector<geometry_msgs::Point> line_points;
    // iterate through all lines
    for (unsigned int i = 0; i < line_obstacles.size(); i++) {
        mapping_igvc::LineObstacle line_obstacle = line_obstacles[i];

        // draw the line as a series of points
        for (float x = line_obstacle.min; x < line_obstacle.max;
             x += this->x_delta) {
            // Compute the point assuming the polynomial is in terms of x
            // ie. y(x)
            geometry_msgs::Point p;
            p.x = x;

            // calculate y with a polynomial function
            for (unsigned int i = 0; i < line_obstacle.coefficients.size();
                 i++) {
                p.y += line_obstacle.coefficients[i] * pow(x, i);
            }

            // Check if the point is in terms of y, if it is,
            // we need to switch x and y
            if (line_obstacle.dependent_on == mapping_igvc::LineObstacle::DEPENDENT_ON_Y){
                double y = p.y;
                double x = p.x;
                p.y = x;
                p.x = y;
            }
            line_points.push_back(p);
        }
    }

    return line_points;
}

bool LineExtractorNode::areParamsInvalid() {
    return this->degreePoly < 0 || this->lambda < 0 ||
           this->minNeighbours < 0 || this->radius < 0;
}

std::vector<mapping_igvc::LineObstacle>
LineExtractorNode::vectorsToMsgs(std::vector<Eigen::VectorXf> vectors,
                                 vector<pcl::PointCloud<pcl::PointXYZ>> &clusters) {
    std::vector<mapping_igvc::LineObstacle> msgs;

    for (unsigned int i = 0; i < clusters.size(); i++) {
        msgs.push_back(vectorToLineObstacle(vectors[i], clusters[i]));
    }

    return msgs;
}

mapping_igvc::LineObstacle
LineExtractorNode::vectorToLineObstacle(Eigen::VectorXf v,
                                        pcl::PointCloud<pcl::PointXYZ> &cluster) {
    mapping_igvc::LineObstacle line_obstacle = mapping_igvc::LineObstacle();

    for (unsigned int i = 0; i < v.size(); i++) {
        line_obstacle.coefficients.push_back(v(i));
    }

    getClusterRange(line_obstacle.min, line_obstacle.max, cluster, 0);

    line_obstacle.header.frame_id = this->frame_id;

    // Stamp each line with the time of the cloud it came from
    line_obstacle.header.stamp = this->last_cloud_time;

    return line_obstacle;
}

void LineExtractorNode::getClusterRange(double &min, double &max,
                                        pcl::PointCloud<pcl::PointXYZ> &cluster,
                                        mapping_igvc::LineObstacle::_dependent_on_type dependent_on) {
    // Lambda function to get the right member variable depending on what
    // the polynomial is in terms of
    auto getRangeVariable = [&](pcl::PointXYZ p){
        if (dependent_on == mapping_igvc::LineObstacle::DEPENDENT_ON_Y){
            return p.y;
        } else {
            return p.x;
        }
    };

    if (cluster.size()) {
        min = max = getRangeVariable(cluster[0]);
    } else {
        min = max = -1;
        return;
    }

    for (unsigned int i = 0; i < cluster.size(); i++) {
        if (getRangeVariable(cluster[i]) < min) { min = getRangeVariable(cluster[i]); }
        if (getRangeVariable(cluster[i]) > max) { max = getRangeVariable(cluster[i]); }
    }

    min = min;
    max = max;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>>
LineExtractorNode::switchXandYinClusters(
        std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters) {
    std::vector<pcl::PointCloud<pcl::PointXYZ>> switched_clusters;
    for (pcl::PointCloud<pcl::PointXYZ>& cluster : clusters){
        switched_clusters.emplace_back(switchXandYinCluster(cluster));
    }

    return switched_clusters;
}

pcl::PointCloud<pcl::PointXYZ>
LineExtractorNode::switchXandYinCluster(pcl::PointCloud<pcl::PointXYZ> cluster) {
    pcl::PointCloud<pcl::PointXYZ> switched_cluster = cluster;
    for (pcl::PointXYZ& point : switched_cluster){
        double x = point.x;
        double y = point.y;
        point.x = y;
        point.y = x;
    }

    return switched_cluster;
}
