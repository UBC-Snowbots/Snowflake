/*
 * Created By: Kevin Lin
 * Created On: December 21st, 2019
 * Description: Uses Libevdev to turn Nintendo Switch Pro Controller left joystick inputs into a ROS Twist message
 */


#include "../include/ProController.h"

//see header file for changing sensitivity, controller, or ros topic

ProController::ProController(int argc, char **argv, string node_name) {
    setup();
    ros::init(argc, argv, node_name);
    ros::NodeHandle private_nh("~");
    pub = private_nh.advertise<geometry_msgs::Twist>(ROS_TOPIC, 1000);
    readInputs();
}

void ProController::setup() {

    int fd;
    fd = open(EVTEST_PATH, O_RDONLY | O_NONBLOCK);
    int rc = libevdev_new_from_fd(fd, &dev);
    if (rc < 0) {
        ROS_INFO_COND(stderr, "Failed to init libevdev (%s)\n", strerror(-rc));
        exit(1);
    }
    printf("Input device name: \"%s\"\n", libevdev_get_name(dev));
    printf("Input device ID: bus %#x vendor %#x product %#x\n",
           libevdev_get_id_bustype(dev),
           libevdev_get_id_vendor(dev),
           libevdev_get_id_product(dev));
}

void ProController::readInputs() {
    int x = 0;
    int z = 0;
    int x_old = 0;
    int z_old = 0;
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
                //ROS_INFO("Event: Type: %s Code: %s Value: %d\n",
                //       type, code, ev.value);
                if (ev.code == ABS_X) {// if the received event is the X axis of the joystick
                    //Some estimated bounds of what should count as 0 on the x axis
                    if (ev.value > 32000 && ev.value < 37000) {
                        z = 0;
                    } else {
                        //2500 is a magic number that reduces the event's value to an input that is smooth in turtlesim
                        z = -(ev.value - 32500) / 2500.0 * Z_SENSITIVITY;
                    }
                } else if (ev.code == ABS_Y) {// if the received event is the Y axis of the joystick
                    //Some estimated bounds of what should count as 0 on the y axis
                    if (ev.value > 34000 && ev.value < 37000) {
                        x = 0;
                    } else {
                        //4000 is a magic number that reduces the events value to something that is smooth in turtlesim
                        x = -(ev.value - 36341) / 4000.0 * X_SENSITIVITY;
                    }
                }
                //if this is a new input, publish it
                if (x_old != x || z_old != z) {
                    msg.linear.x = x;
                    msg.angular.z = z;
                    pub.publish(msg);
                    //ROS_INFO("X: %d Z: %d", x, z);
                    z_old = z;
                    x_old = x;
                }
            }
        }
    } while (rc == 1 || rc == 0 || rc == -EAGAIN);
}
