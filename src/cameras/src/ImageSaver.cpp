/*
 * Created By: Rowan Zawadzki
 * Created On: Aug 8 2022
 * Description: A node that saves images
 */

#include <ImageSaver.h>

ImageSaver::ImageSaver(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    image_transport::ImageTransport it(nh);
    // Obtains character from the parameter server (or launch file), sets '!' as default
    //std::string parameter_name    = "character";
   // SB_getParam(private_nh, parameter_name, suffix, default_character);

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "cam_1/color/image_raw";
    int queue_size                    = 10;
    camera_subscribe                  = it.subscribe(topic_to_subscribe_to, queue_size, &ImageSaver::subscriberCallBack, this);
    
    std::string topic_to_subscribe_to2 = "shutter_button";
    int queue_size2                    = 2;
    shutter                  = nh.subscribe(topic_to_subscribe_to2, queue_size2, &ImageSaver::subscriberCallBack2, this);


    // Setup Publisher(s)
    //to automatically take photos
    //std::string topic = private_nh.resolveName("shutter_button");
   // queue_size        = 1;
   // test = private_nh.advertise<std_msgs::Int32>(topic, queue_size);
    //ROS_INFO("Virtual button has been pushed, taking photo(s) and saving to");
}

void ImageSaver::subscriberCallBack(const sensor_msgs::Image::ConstPtr& image) {
    
   
}

void ImageSaver::subscriberCallBack2(const std_msgs::String::ConstPtr& msg) {
    
   
}

