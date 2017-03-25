//
// Created by sb on 25/03/17.
//

#ifndef PROJECT_ZED_FILTER_H
#define PROJECT_ZED_FILTER_H


class zed_filter {

public:
    ZedFilter(int argc, char **argv, std::string node_name);


private:
    ros::Subscriber raw_image_subscriber;
    ros::Publisher filtered_image_publisher;

    void imageCallBack(const sensor_msgs::PointCloud2::ConstPtr& zed_camera_output);
    void publishFilteredImage(const sensor_msgs::PointCloud2::ConstPtr& filtered_point_cloud);
};


#endif //PROJECT_ZED_FILTER_H
