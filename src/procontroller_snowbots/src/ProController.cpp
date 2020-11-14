/*
 * Created By: Kevin Lin
 * Created On: December 21st, 2019
 * Description: Uses Libevdev to turn Nintendo Switch Pro Controller left joystick inputs into a ROS Twist message
 */


#include "../include/ProController.h"

// see header file for changing sensitivity, evtest path, and to see what every button's event code is.
//If you get "Failed to init libevdev (Bad file descriptor)", run evtest and see
//if the path to the controller listed is not between
//and change it if it is. If you run this node and there's no controller output, its likely because the evtest
//path has changed.
ProController::ProController(int argc, char **argv, string node_name) {
    setup();
    ros::init(argc, argv, node_name);
    ros::NodeHandle private_nh("~");
    pubmove = private_nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
    printf("Preparing to read inputs...");
    readInputs();
}

void ProController::setup() {
    int rc = 0;
    for(auto i=0;i<50;i++){
        string inttostring = to_string(i);
        string EVTEST_PATH = "/dev/input/event";
        string pathstring = EVTEST_PATH + inttostring;
        const char* path = (pathstring).c_str();
        int fd = open(path,O_RDONLY | O_NONBLOCK);
        rc = libevdev_new_from_fd(fd, &dev);
        if (rc >= 0) {
            printf("Succesfully set up with path \"%s\n",path);
            printf("Input device name: \"%s\"\n", libevdev_get_name(dev));
            printf("Input device ID: bus %#x vendor %#x product %#x\n",
                   libevdev_get_id_bustype(dev),
                   libevdev_get_id_vendor(dev),
                   libevdev_get_id_product(dev));
            break;
        }
    }
    if (rc < 0){
        ROS_INFO_COND(stderr, "Failed to init libevdev (%s)\n", strerror(-rc));
        exit(1);
    }
}

void ProController::readInputs() {
    double x = 0;
    double z = 0;
    double x_old = 0;
    double z_old = 0;
    int rc;
    do {
        geometry_msgs::Twist msg;
        struct input_event ev;
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
        if (rc == 0) {
            auto code = libevdev_event_code_get_name(ev.type, ev.code);
            auto type = libevdev_event_type_get_name(ev.type);
            if (ev.type == EV_ABS) {//ignore SYN_REPORT
                //uncomment to see the event code printouts to calibrate the z= and x= lines
                ROS_INFO("Event: Type: %s Code: %s Value: %d\n",
                      type, code, ev.value);
                if (ev.code == ABS_X) {// if the received event is the X axis of the joystick
                    //Some estimated bounds of what should count as 0 on the x axis
                    if (ev.value > 120 && ev.value < 135) {
                        z = 0;
                    } else {
                        //my calibration revealed 128 was the center, 255 was the max, so this normalizes the max to be 1
                        z = -(ev.value - 128) / 128.0 * Z_SENSITIVITY;
                    }
                } else if (ev.code == ABS_Y) {// if the received event is the Y axis of the joystick
                    //Some estimated bounds of what should count as 0 on the y axis
                    if (ev.value > 120 && ev.value < 135) {
                        x = 0;
                    } else {
                        //my calibration revealed 128 was the center, 255 was the max, so this normalizes the max to be 1
                        x = (ev.value - 128) / 128.0* X_SENSITIVITY;
                    }
                } else if (ev.code == BTN_WEST) {
                }
                //if this is a new input, publish it
                if (x_old != x || z_old != z) {
                    msg.linear.x = x;
                    msg.angular.z = z;
                    pubmove.publish(msg);
                    //ROS_INFO("X: %d Z: %d", x, z);
                    z_old = z;
                    x_old = x;
                }
            }
        }
    } while (rc == 1 || rc == 0 || rc == -EAGAIN);
}
