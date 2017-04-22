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
    int queue_size = 1;
    raw_image_subscriber = public_nh.subscribe(camera_image_topic_name, queue_size, &ZedFilter::imageCallBack, this);
    
    // Setup Publisher(s)
    std::string filtered_image_topic_name = "/zed_filter/filtered_point_cloud";
    filtered_image_publisher = nh.advertise<PointCloudColour>(filtered_image_topic_name, queue_size);
}

void ZedFilter::imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output) {

    PointCloudColour::Ptr filtered_point_cloud;
    filtered_point_cloud = ZedFilter::filterImage(zed_camera_output);
    publishFilteredImage(*filtered_point_cloud);
}


void ZedFilter::publishFilteredImage(const PointCloudColour filtered_point_cloud)
{
    filtered_image_publisher.publish(filtered_point_cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ZedFilter::filterImage(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output)
{
    // Convert PointCloud2 into a pcl::PointCloud
    pcl::PCLPointCloud2 temp;
    pcl_conversions::toPCL(*zed_camera_output, temp);
    PointCloudColour::Ptr transformed_point_cloud(new PointCloudColour);
    pcl::fromPCLPointCloud2(temp, *transformed_point_cloud);

    // Filter out non-white points from the point cloud
    PointCloudColour::Ptr filtered_point_cloud(new PointCloudColour);
    // Create the filtering object;
    pcl::PassThrough<Point> pass;
    pass.setInputCloud(transformed_point_cloud);

    // TODO: Convert to HSV (If not computationally expensive) before analyzing colour
    // Also should be pass.setFilterFieldName("rgb") methinks

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


    PointCloudColour::Ptr mapped_and_filtered_point_cloud(new PointCloudColour);

    PointCloudColour::const_iterator it;
    for(it = filtered_point_cloud->points.begin(); it != filtered_point_cloud->points.end(); it++) {
        Point curpoint_to_add;

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