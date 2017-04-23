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
    filtered_image_publisher = nh.advertise<PointCloudRGB>(filtered_image_topic_name, queue_size);
}

void ZedFilter::imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output) {

    PointCloudRGB::Ptr filtered_point_cloud;
    filtered_point_cloud = ZedFilter::filterImage(zed_camera_output);
    publishFilteredImage(*filtered_point_cloud);
}


void ZedFilter::publishFilteredImage(const PointCloudRGB filtered_point_cloud)
{
    filtered_image_publisher.publish(filtered_point_cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ZedFilter::filterImage(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output)
{
    // Convert PointCloud2 into a pcl::PointCloud
    pcl::PCLPointCloud2 temp;
    pcl_conversions::toPCL(*zed_camera_output, temp);
    PointCloudRGB::Ptr point_cloud_RGB(new PointCloudRGB);
    pcl::fromPCLPointCloud2(temp, *point_cloud_RGB);

    // Conversion to HSV colourspace
    PointCloudHSV::Ptr point_cloud_HSV(new PointCloudHSV);
    pcl::PointCloudXYZRGBtoXYZHSV(*point_cloud_RGB, *point_cloud_HSV);

    // Filter out non-white points from the point cloud
    PointCloudHSV::Ptr filtered_point_cloud(new PointCloudHSV);
    // Create the filtering object;
    pcl::PassThrough<PointHSV> pass;
    pass.setInputCloud(point_cloud_HSV);

    // Filter red
    pass.setFilterFieldName("h");
    pass.setFilterLimits(250.0, 255.0);
    pass.filter(*filtered_point_cloud);
    // Filter green
    pass.setInputCloud(filtered_point_cloud);
    pass.setFilterFieldName("s");
    pass.filter(*filtered_point_cloud);
    // Filter blue
    pass.setFilterFieldName("v");
    pass.filter(*filtered_point_cloud);

    PointCloudRGB::Ptr output(new PointCloudRGB);
    PointCloudHSV::const_iterator it;
    for(it = filtered_point_cloud->points.begin(); it != filtered_point_cloud->points.end(); it++) {
        PointRGB current_point;

        // Map all points to z = 0 plane.
        current_point.z = 0;

        // All remaining points are mapped to white.
        current_point.r = 255;
        current_point.g = 255;
        current_point.b = 255;

        // Retrieve the x and z values
        current_point.x = it->x;
        current_point.y = it->y;

        output->points.push_back(current_point);
    }

    return output;
}