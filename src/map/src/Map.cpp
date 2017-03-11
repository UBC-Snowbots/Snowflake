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

void Map::visionCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg) {


    // Assume we obtain x,y,z + rgb value for each point
    pcl::PointCloud<pcl::PointXYZRGB> cloud = *msg;

    // Integrate data


    // Publish data
    nav_msgs::OccupancyGrid occ_grid;
    float occ_grid_min = 0;
    float occ_grid_max = 100;
    grid_map::GridMapRosConverter::toOccupancyGrid(map, "vision", occ_grid_min, occ_grid_max, occ_grid);
    vision_map_pub.publish(occ_grid);

}

void Map::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    // Take laserscan and magic it into a pointcloud
    sensor_msgs::PointCloud2 ros_cloud;
    projector.projectLaser(*msg, ros_cloud);
    ROS_INFO("Frame of input: %s", msg->header.frame_id.c_str());
    pcl::PointCloud<pcl::PointXYZ> cloud;


    try {

        // Get the TF transformation
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped tstamped;
        tstamped = tfBuffer.lookupTransform("odom", "lidar", ros::Time(0), ros::Duration(3.0));
        sensor_msgs::PointCloud2 ros_transformed_cloud;
        tf2::doTransform(ros_cloud, ros_transformed_cloud, tstamped);
        ROS_INFO("New frame: %s", ros_transformed_cloud.header.frame_id.c_str());
        test_pub.publish(ros_transformed_cloud);

        // Convert into PCL pointcloud type
        pcl::PCLPointCloud2 temp;
        pcl_conversions::toPCL(ros_transformed_cloud, temp);
        pcl::fromPCLPointCloud2(temp, cloud);
        // ros_transformed_cloud.header.frame_id = "odom";

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    // Integrate data
    grid_map::Matrix& data = map["lidar"];
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




