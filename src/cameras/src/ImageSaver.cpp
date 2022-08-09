/*
 * Created By: Rowan Zawadzki
 * Created On: Aug 8 2022
 * Description: A node that saves images from the realsense cameras CIRC2022
 */

#include <ImageSaver.h>

ImageSaver::ImageSaver(int argc, char **argv, std::string node_name) {
  
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    image_transport::ImageTransport it(nh);
    //decide which camera to save from
//std::string parameter1    = "camera";
   // SB_getParam(private_nh, parameter1, camera, 2);
    // Setup Subscriber(s)
    std::string cam1_topic;
   std::string cam2_topic;
    cam1_topic = "cam_1/color/image_raw";
   cam2_topic = "cam_2/color/image_raw";

       std::string shutter1_topic = "shutter1_button";
      std::string shutter2_topic = "shutter2_button";


    
    int queue_size                    = 10;
    camera1_subscribe                  = it.subscribe(cam1_topic, queue_size, &ImageSaver::subscriberCallBack1, this);
        camera2_subscribe                  = it.subscribe(cam2_topic, queue_size, &ImageSaver::subscriberCallBack2, this);

    int queue_size2                    = 4;
    shutter1                  = nh.subscribe(shutter1_topic, queue_size2, &ImageSaver::subscriberCallBackShutter1, this);
    shutter2                  = nh.subscribe(shutter2_topic, queue_size2, &ImageSaver::subscriberCallBackShutter2, this);


    
    
    
}

void ImageSaver::subscriberCallBack1(const sensor_msgs::Image::ConstPtr& img1) {
    if (PicsToTake1 > 0){
   

  camera = 1;
   
      

      cv::Mat image;
      image = cv_bridge::toCvShare(img1, "bgr8")->image;
      
       ROS_INFO("Camera %d shutter has been pushed ---%d--- time(s) so far, taking photo(s) and saving to your Snowflake/docs/CIRC_Images/cam%d--images file.", camera, picCounter1 + 1, camera);

      cv::imwrite(cv::format("../Snowflake/docs/CIRC_Images/cam1--images/%d.png", picCounter1), image);
      picCounter1 ++;
      PicsToTake1 -= 1;void subscriberCallBack(const sensor_msgs::Image::ConstPtr& image);
      
    }
    
   
}
void ImageSaver::subscriberCallBack2(const sensor_msgs::Image::ConstPtr& img2) {
    if (PicsToTake2 > 0){
   

      camera = 2;
   
      

      cv::Mat image;
      image = cv_bridge::toCvShare(img2, "bgr8")->image;
    
       ROS_INFO("Camera %d shutter has been pushed ---%d--- time(s) so far, taking photo(s) and saving to your Snowflake/docs/CIRC_Images/cam%d--images file.", camera, picCounter2 + 1, camera);

      cv::imwrite(cv::format("../Snowflake/docs/CIRC_Images/cam2--images/%d.png", picCounter2), image);
      picCounter2 ++;
      PicsToTake2 -= 1;
      

    }
   
}
void ImageSaver::subscriberCallBackShutter1(const std_msgs::Int32::ConstPtr& msg) {

   PicsToTake1 += msg->data; 
   
}
void ImageSaver::subscriberCallBackShutter2(const std_msgs::Int32::ConstPtr& msg) {

   PicsToTake2 += msg->data; 
   
}

