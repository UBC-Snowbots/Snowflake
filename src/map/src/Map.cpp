/*
 * Created By: Valerian Ratu
 * Created On: December 23, 2016
 * Description: A node which manages and organizes mapping information from various sources
 */

#include <Map.h>

Map::Map(int argc, char **argv, std::string node_name){

    map = new grid_map::GridMap();

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // 40m x 40m map with 1 cell / cm
    map_width = 40;
    map_height = 40;
    map_res = 0.02;

    setUpMap(map);

    // Initialize subscribers
    // Buffers are bad because we might be processing old data
    vision_sub = nh.subscribe("vision", 1, &Map::visionCallback, this);
    lidar_sub = nh.subscribe("/robot/laser/scan", 1, &Map::lidarCallback, this);

    // Initialize publishers
    vision_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("vision_map", 1, true);
    lidar_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("lidar_map", 1, true);

    //Initialize TFs
    tfListener = new tf2_ros::TransformListener(tfBuffer);
}

void Map::visionCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    sensor_msgs::PointCloud2 transformed_ros_cloud;
    transformPointCloud(*msg, transformed_ros_cloud, "odom");

    pcl::PointCloud<pcl::PointXYZ> cloud;
    convertPointCloud(transformed_ros_cloud, cloud);

    plotPointCloudOnMap(cloud, "vision");

    // Publish data
    nav_msgs::OccupancyGrid occ_grid;
    float occ_grid_min = 0;
    float occ_grid_max = 100;
    grid_map::GridMapRosConverter::toOccupancyGrid(*map, "vision", occ_grid_min, occ_grid_max, occ_grid);
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

    plotPointCloudOnMap(cloud, "lidar");

    // Publish data
    nav_msgs::OccupancyGrid occ_grid;
    float occ_grid_min = 0;
    float occ_grid_max = 100;
    grid_map::GridMapRosConverter::toOccupancyGrid(*map, "lidar", occ_grid_min, occ_grid_max, occ_grid);
    lidar_map_pub.publish(occ_grid);
}

void Map::plotPointCloudOnMap(const pcl::PointCloud<pcl::PointXYZ>& cloud, std::string layer){
    pcl::PointCloud<pcl::PointXYZ>::const_iterator it;
    for (it = cloud.points.begin(); it != cloud.points.end(); it++)
    {
        grid_map::Position pos(it->x, it->y);
        if (map->isInside(pos))
        {
            map->atPosition(layer, pos) = it->z;
        }
        else
        {
            // Resize map and hope everything goes smoothly,
            // this can be expensive since we assume it won't happen too often
            grid_map::GridMap* resized_map = new grid_map::GridMap();
            map_width = 2*map_width;
            map_height = 2*map_height;
            setUpMap(resized_map);
            transferMap(map, resized_map);
            ROS_INFO("Resizing map to: %f x %f", map_width, map_height);
            delete map;
            map = resized_map;
        }

    }
}

void Map::setUpMap(grid_map::GridMap* map){
    map->add("vision");
    map->add("lidar");
    map->add("footprint");

    // TODO: set this frame to "map" and create transform between "map" and "odom"
    map->setFrameId("odom");
    map->setGeometry(grid_map::Length(map_width, map_height), map_res);
}

void Map::transferMap(grid_map::GridMap* fromMap, grid_map::GridMap* toMap){
    // Check for map size
    double from_map_res = fromMap->getResolution();
    double to_map_res = fromMap->getResolution();

    if (    from_map_res * fromMap->getSize()[0] > to_map_res * toMap->getSize()[0] ||
            from_map_res * fromMap->getSize()[1] > to_map_res * toMap->getSize()[1]     )
    {
        ROS_WARN("Map being transferred to has smaller size, data might be lost");
    }

    //TODO: Check for existing layers
    //std::vector<std::string> fromMapLayers = fromMap->getBasicLayers();

    //nuke the existing map
    toMap->clearAll();
    toMap->addDataFrom(*fromMap, true, true, true);
}

void Map::transformPointCloud(const sensor_msgs::PointCloud2& input,
                              sensor_msgs::PointCloud2& output,
                              std::string target_frame){
    try {
        geometry_msgs::TransformStamped tStamped;
        tStamped = tfBuffer.lookupTransform(target_frame, input.header.frame_id, input.header.stamp, ros::Duration(1.0));
        tf2::doTransform(input, output, tStamped);
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
    return map->get("vision");
}

grid_map::Matrix Map::getLidarLayer(){
    return map->get("lidar");
}

grid_map::Matrix Map::getFootprintLayer(){
    return map->get("footprint");
}






