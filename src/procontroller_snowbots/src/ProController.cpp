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
    bool turtle = false;
    string publisher = "/cmd_vel";
    if (turtle) {
        publisher = "/turtle1/cmd_vel";
    }
    setup();
    ros::init(argc, argv, node_name);
    ros::NodeHandle private_nh("~");
    pubmove = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    printf("Preparing to read inputs...\n");
    x = 0;
    z = 0;
    readInputs();
}

void ProController::setup() {
    int rc = 0;
#   //check the first 50 event inputs to find the controller.
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
    double x_old = 0;
    double z_old = 0;
    int rc;
    do {
        struct input_event ev;
        rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
        if (rc == 0) {
            auto code = libevdev_event_code_get_name(ev.type, ev.code);
            auto type = libevdev_event_type_get_name(ev.type);
            if (ev.type != EV_SYN) {//ignore EV_SYN, keep track of ABS and KEY
                switch(ev.code) {
                    case ABS_X:
                        leftJoystickX(ev.value);
                        break;
                    case ABS_Y:
                        leftJoystickY(ev.value);
                        break;
                    case ABS_RX:
                        rightJoystickX(ev.value);
                        break;
                    case ABS_RY:
                        rightJoystickY(ev.value);
                        break;
                    case BTN_EAST:
                        A(ev.value);
                        break;
                    case BTN_SOUTH:
                        B(ev.value);
                        break;
                    case BTN_WEST:
                        X(ev.value);
                        break;
                    case BTN_NORTH:
                        Y(ev.value);
                        break;
                    case BTN_TL:
                        leftBumper(ev.value);
                        break;
                    case BTN_TR:
                        rightBumper(ev.value);
                        break;
                    case BTN_SELECT:
                        select(ev.value);
                        break;
                    case BTN_START:
                        start(ev.value);
                        break;
                    case BTN_MODE:
                        home(ev.value);
                        break;
                    case ABS_Z:
                        leftTrigger(ev.value);
                        break;
                    case ABS_RZ:
                        rightTrigger(ev.value);
                        break;
                    case ABS_HAT0X:
                        arrowsRorL(ev.value);
                        break;
                    case ABS_HAT0Y:
                        arrowsUorD(ev.value);
                        break;
                    case BTN_THUMBL:
                        leftJoystickPress(ev.value);
                        break;
                    case BTN_THUMBR:
                        rightJoystickPress(ev.value);
                        break;
                }
                //publish move command and update oldx, oldz
                tie(x_old,z_old) = publishMoveXZ(x, z, x_old, z_old);
                //uncomment to see all controller event code printouts
                //ROS_INFO("Event: Type: %s Code: %s Value: %d\n", type, code, ev.value);

            }
        }
    } while (rc == 1 || rc == 0 || rc == -EAGAIN);
}

tuple<double,double> ProController::publishMoveXZ(double x_new, double z_new, double x_old, double z_old){
    if (x_old != x_new || z_old != z_new) {
        geometry_msgs::Twist msg;
        msg.linear.x = x_new;
        msg.angular.z = z_new;
        pubmove.publish(msg);
        //return tuple
        return make_tuple(x_new,z_new);
    }
}

void ProController::leftJoystickX(int value) {
    if (value > 120 && value < 135) {
        z = 0;
    } else {
        //128 is the center, so this normalizes the result to [-1,1]*Z_SENSITIVITY
        z = -(value - 128) / 128.0 * Z_SENSITIVITY;
    }
}

void ProController::leftJoystickY(int value){
    if (value > 120 && value < 135) {
        x = 0;
    } else {
        //128 is the center, so this normalizes the result to [-1,1]*X_SENSITIVITY
        x = (value - 128) / 128.0* X_SENSITIVITY;
    }
}
void ProController::rightJoystickX(int value){
    if (value > 120 && value < 135) {
        // do nothing
    } else {ROS_INFO("Right Joystick X event with value: %d\n", value);}
}
void ProController::rightJoystickY(int value){
    if (value > 120 && value < 135) {
        // do nothing
    } else {ROS_INFO("Right Joystick Y event with value: %d\n", value);}
}
void ProController::A(int value){
    if (value == 1) { ROS_INFO("A button pressed"); }
    else if (value == 0) { ROS_INFO("A button released");}
}
void ProController::B(int value){
    if (value == 1) { ROS_INFO("B button pressed"); }
    else if (value == 0) { ROS_INFO("B button released");}
}
void ProController::X(int value){
    if (value == 1) { ROS_INFO("X button pressed"); }
    else if (value == 0) { ROS_INFO("X button released");}
}
void ProController::Y(int value){
    if (value == 1) { ROS_INFO("Y button pressed"); }
    else if (value == 0) { ROS_INFO("Y button released");}
}
void ProController::leftBumper(int value){
    if (value == 1) { ROS_INFO("Left bumper pressed"); }
    else if (value == 0) { ROS_INFO("Left bumper released");}
}
void ProController::rightBumper(int value){
    if (value == 1) { ROS_INFO("Right bumper pressed"); }
    else if (value == 0) { ROS_INFO("Right bumper released");}
}
void ProController::select(int value){
    if (value == 1) { ROS_INFO("Select button pressed"); }
    else if (value == 0) { ROS_INFO("Select button released");}
}
void ProController::start(int value){
    if (value == 1) { ROS_INFO("Start button pressed"); }
    else if (value == 0) { ROS_INFO("Start button released");}
}

void ProController::home(int value){
    if (value == 1) { ROS_INFO("Home button pressed"); }
    else if (value == 0) { ROS_INFO("Home button released");}
}
void ProController::leftTrigger(int value){
    if (value == 255) { ROS_INFO("Left trigger pressed"); }
    else if (value == 0) { ROS_INFO("Left trigger released");}
}
void ProController::rightTrigger(int value){
    if (value == 255) { ROS_INFO("Right trigger pressed"); }
    else if (value == 0) { ROS_INFO("Right trigger released");}
}
void ProController::arrowsRorL(int value){
    if (value == 1) { ROS_INFO("Right button pressed"); }
    else if (value == 0) { ROS_INFO("Arrow button released");}
    else { ROS_INFO("Left button pressed"); }
}
void ProController::arrowsUorD(int value){
    if (value == 1) { ROS_INFO("Up button pressed"); }
    else if (value == 0) { ROS_INFO("Arrow button released");}
    else { ROS_INFO("Down button pressed"); }
}
void ProController::leftJoystickPress(int value){
    if (value == 1) { ROS_INFO("Left Joystick pressed"); }
    else if (value == 0) { ROS_INFO("Left Joystick released");}
}
void ProController::rightJoystickPress(int value){
    if (value == 1) { ROS_INFO("Right Joystick pressed"); }
    else if (value == 0) { ROS_INFO("Right Joystick released");}
}