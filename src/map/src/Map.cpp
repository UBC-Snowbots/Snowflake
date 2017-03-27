/*
 * Created By: Valerian Ratu
 * Created On: December 23, 2016
 * Description: A node which manages and organizes mapping information from various sources
 */

#include <Map.h>

Map::Map(int argc, char **argv, std::string node_name){

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");



    // Creates the relevant layers
    map.add("vision");
    map.add("lidar");
    map.add("footprint");

    // 40m x 40m map with 1 cell / cm
    double const INIT_MAP_WIDTH = 40;
    double const INIT_MAP_HEIGHT = 40;
    double const INIT_MAP_RES = 0.01;

    double const INIT_MAP_X_POSITION = 20;
    double const INIT_MAP_Y_POSITION = 20;

    // TODO: set this frame to "map" and create transform between "map" and "odom"
    map.setFrameId("odom");
    map.setGeometry(grid_map::Length(INIT_MAP_WIDTH, INIT_MAP_HEIGHT), INIT_MAP_RES);

    // Initialize subscribers
    // Buffers are bad because we might be processing old data
    vision_sub = nh.subscribe("vision", 1, &Map::visionCallback, this);
    lidar_sub = nh.subscribe("/robot/laser/scan", 1, &Map::lidarCallback, this);
    position_sub = nh.subscribe("/robot/odom_diffdrive", 1, &Map::positionCallback, this);

    // Initialize publishers
    vision_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("vision_map", 1, true);
    lidar_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("lidar_map", 1, true);
    location_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("position_map", 1, true);
    test_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_pointcloud", 1, true);

    // Initializes position to origin
    // TODO: Set initial position and orientation
    // geometry_msgs::Point

}

void Map::positionCallback(const nav_msgs::Odometry::ConstPtr& msg){
    pos = msg->pose.pose;
    //ROS_INFO("Position: (%f, %f, %f)", pos.position.x, pos.position.y, pos.position.z);
    //ROS_INFO("Orientation: (%f, %f, %f, %f)", pos.orientation.x, pos.orientation.w, pos.orientation.y, pos.orientation.z);
}

// TODO: Callbacks are super similar, maybe refactor to a generic callback with params.
void Map::visionCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    sensor_msgs::PointCloud2 transformed_ros_cloud;
    transformPointCloud(*msg, transformed_ros_cloud, "odom");

    pcl::PointCloud<pcl::PointXYZ> cloud;
    convertPointCloud(transformed_ros_cloud, cloud);

    // Integrate data
    pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
    for (it = cloud.points.begin(); it != cloud.points.end(); it++)
    {
        grid_map::Position pos(it->x, it->y);
        if (map.isInside(pos))
        {
            ROS_INFO("Inserting [%f] at: (%f, %f)", it->z, it->x, it->y);
            map.atPosition("vision", pos) = it->z;
        }
    }

    // Publish data
    nav_msgs::OccupancyGrid occ_grid;
    float occ_grid_min = 0;
    float occ_grid_max = 100;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "vision", occ_grid_min, occ_grid_max, occ_grid);
    vision_map_pub.publish(occ_grid);

}

void Map::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    // Take laserscan and magic it into a pointcloud with the right transformation
    sensor_msgs::PointCloud2 ros_cloud;
    projector.projectLaser(*msg, ros_cloud);
    ROS_INFO("Frame of input: %s", msg->header.frame_id.c_str());

    sensor_msgs::PointCloud2 transformed_ros_cloud;
    transformPointCloud(ros_cloud, transformed_ros_cloud, "odom");

    pcl::PointCloud<pcl::PointXYZ> cloud;
    convertPointCloud(transformed_ros_cloud, cloud);

    // Integrate data
    pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
    for (it = cloud.points.begin(); it != cloud.points.end(); it++)
    {
        grid_map::Position pos(it->x, it->y);
        if (map.isInside(pos))
        {
            ROS_INFO("Inserting [%f] at: (%f, %f)", it->z, it->x, it->y);
            map.atPosition("lidar", pos) = it->z;
        }

    }
    // Publish data
    nav_msgs::OccupancyGrid occ_grid;
    float occ_grid_min = 0;
    float occ_grid_max = 100;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "lidar", occ_grid_min, occ_grid_max, occ_grid);
    lidar_map_pub.publish(occ_grid);
}

void Map::transformPointCloud(const sensor_msgs::PointCloud2& input,
                              sensor_msgs::PointCloud2& output,
                              std::string target_frame){
    try {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped tstamped;
        tstamped = tfBuffer.lookupTransform(target_frame, input.header.frame_id, ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(input, output, tstamped);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void Map::convertPointCloud(const sensor_msgs::PointCloud2& input, pcl::PointCloud<pcl::PointXYZ>& output){
    pcl::PCLPointCloud2 temp;
    pcl_conversions::toPCL(input, temp);
    pcl::fromPCLPointCloud2(temp, output);
}

grid_map::Matrix Map::getVisionLayer(){
    return map.get("vision");
}

grid_map::Matrix Map::getLidarLayer(){
    return map.get("lidar");
}

grid_map::Matrix Map::getFootprintLayer(){
    return map.get("footprint");
}

std::pair<double, double> Map::getCurrentLocation(){
    return std::pair<double, double>(0, 0);
}






