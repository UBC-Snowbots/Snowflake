//
// Created by sb on 25/03/17.
//

#include <ZedFilter.h>

// The constructor
ZedFilter::ZedFilter(int argc, char **argv, std::string node_name)
{
    ros::init(argc, argv, node_name);

    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string camera_image_topic_name = "/zed/point_cloud/cloud_registered";
    int refresh_rate = 10;
    raw_image_subscriber = public_nh.subscribe(camera_image_topic_name, refresh_rate, &ZedFilter::imageCallBack, this);
    
    // Setup Publisher(s)
    std::string filtered_image_topic_name = "/zed_filter/filtered_point_cloud";
    uint32_t queue_size = 10;
    filtered_image_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > (filtered_image_topic_name, queue_size);
}

void ZedFilter::imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud;
    filtered_point_cloud = ZedFilter::filterImage(zed_camera_output);
    publishFilteredImage(*filtered_point_cloud);
}


void ZedFilter::publishFilteredImage(const pcl::PointCloud<pcl::PointXYZRGB> filtered_point_cloud)
{
    filtered_image_publisher.publish(filtered_point_cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ZedFilter::filterImage(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output)
{
    // Convert PointCloud2 into a pcl::PointCloud
    pcl::PCLPointCloud2 temp;
    pcl_conversions::toPCL(*zed_camera_output, temp);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_point_cloud;
    pcl::fromPCLPointCloud2(temp, *transformed_point_cloud);

    // Filter out non-white points from the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_point_cloud;
    // Create the filtering object;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(transformed_point_cloud);
    // Filter red
    pass.setFilterFieldName("r");
    pass.setFilterLimits(250.0, 255.0);
    pass.filter(*filtered_point_cloud);
    // Filter green
    pass.setInputCloud(filtered_point_cloud);
    pass.setFilterFieldName("g");
    pass.filter(*filtered_point_cloud);
    // Filter blue
    pass.setFilterFieldName("b");
    pass.filter(*filtered_point_cloud);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapped_and_filtered_point_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
    for(it = filtered_point_cloud->points.begin(); it != filtered_point_cloud->points.end(); it++) {
        pcl::PointXYZRGB curpoint_to_add;

        // Map all points to z = 0 plane.
        curpoint_to_add.z = 0;

        // All remaining points are mapped to white.
        curpoint_to_add.r = 255;
        curpoint_to_add.g = 255;
        curpoint_to_add.b = 255;

        // Retrieve the x and z values
        curpoint_to_add.x = it->x;
        curpoint_to_add.y = it->y;

        mapped_and_filtered_point_cloud->points.push_back(curpoint_to_add);
    }

    return mapped_and_filtered_point_cloud;
}