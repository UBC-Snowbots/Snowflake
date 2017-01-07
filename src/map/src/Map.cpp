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

    map.setGeometry(grid_map::Length(INIT_MAP_WIDTH, INIT_MAP_HEIGHT), INIT_MAP_RES);

    // Initialize subscribers
    vision_sub = nh.subscribe("vision", 1000, &Map::visionCallback, this);
    lidar_sub = nh.subscribe("lidar", 1000, &Map::lidarCallback, this);
    position_sub = nh.subscribe("position", 1000, &Map::positionCallback, this);

    // Initialize publishers
    vision_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("vision_map", 1, true);
    lidar_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("lidar_map", 1, true);
    location_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("position_map", 1, true);

    // Initializes position to origin
    pos(0,0);
}

void Map::positionCallback(const geometry_msgs::Pose2DConstPtr msg){

}

void Map::visionCallback(const sensor_msgs::PointCloud2ConstPtr msg) {
    // Assume we obtain x,y,z + rgb value for each point
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Integrate data


    // Publish data
    nav_msgs::OccupancyGrid occ_grid;
    float occ_grid_min = 0;
    float occ_grid_max = 100;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "vision", occ_grid_min, occ_grid_max, occ_grid);
    vision_map_pub.publish(occ_grid);

}

void Map::lidarCallback(const sensor_msgs::PointCloud2ConstPtr msg) {
    // Get data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Integrate data


    // Publish data
    nav_msgs::OccupancyGrid occ_grid;
    float occ_grid_min = 0;
    float occ_grid_max = 100;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "lidar", occ_grid_min, occ_grid_max, occ_grid);
    vision_map_pub.publish(occ_grid);
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
    return std::pair<double, double>(pos(0,0), pos(0,1));
}




