#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>


using namespace cv;
using namespace std;

int main(int argc, char** argv) {
 //   string inputWindow = "Camera";
  //  namedWindow(inputWindow, CV_WINDOW_AUTOSIZE);

    VideoCapture cap(0); // captures the first camera
    if (!cap.isOpened()) {
        cout << "Camera cannot be opened" << endl;
        return -1;
    }

    ros::init(argc, argv, "camera_pubber");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("cam_1/color/image_raw", 1);

    Mat inputImage;

    ros::Rate loop_rate(30);

    while (nh.ok()) {
        bool isRead = cap.read(inputImage);
        if (!isRead) {
            cout << "Failed to read image from camera" << endl;
            break;
        }
       // imshow(inputWindow, inputImage);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", inputImage).toImageMsg();
        pub.publish(msg);
        waitKey(1);
        ros::spinOnce();
        loop_rate.sleep();
    }


}









    