/*
 * Created By: Rowan Zawadkzki, modified from the old procontroller_snowbots
 * Created On: December 21st, 2019
 * Description: Uses Libevdev to turn Nintendo Switch Pro Controller left
 * joystick inputs into a ROS Twist message
 * old summary^^
 */

#include "../include/AllController.h"

int32_t buttons[11];
_Float32 axes[8];
// Read the master documentation if there's any issues with this package
AllController::AllController(int argc, char** argv, string node_name) {
    string publisher = "/cmd_vel";
    string armPublisher = "/cmd_arm";
    string modePublisher = "/moveit_toggle";
    string joyTopic = "/joy";
    // string moveGrpPublisher = "/move_group_trigger";
    ros::init(argc, argv, node_name);
    ros::NodeHandle private_nh("~");
    if (private_nh.param<double>("X", X_SENSITIVITY, 1.0)) {
        ROS_INFO("X sensitivity set to %f", X_SENSITIVITY);
    }
    if (private_nh.param<double>("Z", Z_SENSITIVITY, 1.0)) {
        ROS_INFO("Z sensitivity set to %f", Z_SENSITIVITY);
    }
    if (private_nh.param<bool>("debug", debug, false)) {
        ROS_INFO("Debug mode %s", (debug) ? "on" : "off");
    }
    setup();
    pubmove = private_nh.advertise<geometry_msgs::Twist>(publisher, 1);
    pubarm  = private_nh.advertise<std_msgs::String>(armPublisher, 1);
    pubmode = private_nh.advertise<std_msgs::Bool>(modePublisher, 1);
    joyinput = private_nh.subscribe(joyTopic, 10, &AllController::readJoyInputs, this);
    // pubmovegrp = private_nh.advertise<std_msgs::Bool>(moveGrpPublisher,1);

    true_message.data = true;
    false_message.data = false;

    ROS_INFO("Preparing to process inputs...\n");
    state = Mode::wheels;
    printState();
    x = 0;
    z = 0;
    processInputs();
}

void AllController::readJoyInputs(const sensor_msgs::Joy::ConstPtr& msg){
// int32_t A = msg->buttons[0];
// int32_t B = msg->buttons[1];
// int32_t X = msg->buttons[2];
// int32_t Y = msg->buttons[2];
// ROS_INFO("reacieved msg");
// ROS_INFO("A is %i", msg->buttons[0]);
// ROS_INFO("B is %i", msg->buttons[1]);
// ROS_INFO("X is %i", msg->buttons[2]);
// ROS_INFO("Y is %i", msg->buttons[3]);
// ROS_INFO("HOME is %f", msg->axes[0]);
for (int i = 0; i < 11; i ++ ){
buttons[i] = msg->buttons[i];
ROS_INFO("Buttons Recorded: %i", i);
}
for (int i = 0; i < 8; i++){
axes[i] = msg->axes[i];
ROS_INFO("Axes Recorded: %i", i);

}


}

  void AllController::setup() {
//     system(
//     "gnome-terminal --tab -- bash -c 'cd src/sb_AllController/src; sudo "
//     "./procon_driver; read;'");
//     ROS_INFO(
//     "Press enter after you've calibrated the Controller in the other "
//     "terminal...\n ");
//     ROS_INFO(
//     "If calibration is done properly, X and Y buttons should correctly print "
//     "out when pressed");
//     int c  = getchar();
//     int rc = 0;
// #//check the first 50 event inputs to find the controller.
//     for (auto i = 0; i < 50; i++) {
//         string inttostring = to_string(i);
//         string EVTEST_PATH = "/dev/input/event";
//         string pathstring  = EVTEST_PATH + inttostring;
//         const char* path   = (pathstring).c_str();
//         int fd             = open(path, O_RDONLY | O_NONBLOCK);
//         rc                 = libevdev_new_from_fd(fd, &dev);
//         if (rc >= 0) {
//             ROS_INFO("Succesfully set up %s with path \"%s\n",
//                      libevdev_get_name(dev),
//                      path);
//             if (debug) {
//                 ROS_DEBUG("Input device ID: bus %#x vendor %#x product %#x\n",
//                           libevdev_get_id_bustype(dev),
//                           libevdev_get_id_vendor(dev),
//                           libevdev_get_id_product(dev));
//             }
//             break;
//         }
//     }
//     if (rc < 0) {
//         ROS_INFO_COND(stderr, "Failed to init libevdev (%s)\n", strerror(-rc));
//         exit(1);
//     }
}

void AllController::processInputs() {
    double x_old = 0;
    double z_old = 0;
    int rc;
    
        if (rc == 0) {
            // EV_SYN types are useless, ABS and KEY are useful (see .h file for
            // details)
            
                // use rosrun procontroller_snowbots pro_controller
                // _debug:="true"
                if (debug) {
                   // printControllerDebug(ev.type, ev.code, ev.value);
                } else {

                    // vehicle for arm output message
                    armOutMsg = "";
                    armOutVal = "";
                    // handle all controller inputs using API functions
                    // switch (ev.code) {
                    //     case ABS_X: leftJoystickX(ev.value); break;
                    //     case ABS_Y: leftJoystickY(ev.value); break;
                    //     case ABS_RX: rightJoystickX(ev.value); break;
                    //     case ABS_RY: rightJoystickY(ev.value); break;
                    //     case BTN_EAST: A(ev.value); break;
                    //     case BTN_SOUTH: B(ev.value); break;
                    //     case BTN_WEST: X(ev.value); break;
                    //     case BTN_NORTH: Y(ev.value); break;
                    //     case BTN_TL: leftBumper(ev.value); break;
                    //     case BTN_TR: rightBumper(ev.value); break;
                    //     case BTN_SELECT: select(ev.value); break;
                    //     case BTN_START: start(ev.value); break;
                    //     case BTN_MODE: home(ev.value); break;
                    //     case ABS_Z: leftTrigger(ev.value); break;
                    //     case ABS_RZ: rightTrigger(ev.value); break;
                    //     case ABS_HAT0X: arrowsRorL(ev.value); break;
                    //     case ABS_HAT0Y: arrowsUorD(ev.value); break;
                    //     case BTN_THUMBL: leftJoystickPress(ev.value); break;
                    //     case BTN_THUMBR: rightJoystickPress(ev.value); break;
                    // }
                    // publish move command and update oldx, oldz
                    if (state == Mode::wheels) {
                        // Publish motion, update x and z old using tuple
                        tie(x_old, z_old) = publishMoveXZ(x, z, x_old, z_old);
                    }
                    else if (state == Mode::arm_joint_space) { // Joint space control of the arm
                        armOutMsg += jointMode;
                        armOutMsg += armOutVal;
                        publishArmMessage(armOutMsg);
                    }

                    else if (state == Mode::arm_cartesian) { // Inverse Kinematics mode i.e. cartesian motion
                        armOutMsg += IKMode;
                        armOutMsg += armOutVal;
                        publishArmMessage(armOutMsg);
                    }

                    else { // Drilling mode for arm 
                        armOutMsg += drillMode;
                        armOutMsg += armOutVal;
                        publishArmMessage(armOutMsg);
                    }
                
            }
        }

}

// Prints out a controller event using ROS_INFO
void AllController::printControllerDebug(int type, int code, int value) {
   // auto codeout = libevdev_event_code_get_name(type, code);
    //auto typeout = libevdev_event_type_get_name(type);
   // ROS_DEBUG("Event: Type: %s Code: %s Value: %d\n", typeout, codeout, value);
}

// Prints a status message detailing the current control mode
void AllController::printState() {
    if (state == Mode::wheels) {
        ROS_INFO("Current mode: controlling wheels");
    } else if (state == Mode::arm_joint_space) {
        ROS_INFO("Current mode: controlling arm in the joint space");
    } else if (state == Mode::arm_cartesian) {
        ROS_INFO("Current mode: controlling arm in the cartesian space");
    } else if (state == Mode::drilling) {
        ROS_INFO("Current mode: drilling with the arm");
    } else {
        ROS_INFO("There is no current mode, which is a problem");
    }
}

// If x and z are new commands, this function uses the global pubmove to publish
// a movement message and return the new or old xz to update readInputs()
tuple<double, double> AllController::publishMoveXZ(double x_new,
                                                   double z_new,
                                                   double x_old,
                                                   double z_old) {
    if (abs(x_old - x_new) > 0.0001 || abs(z_old - z_new) > 0.0001) {
        geometry_msgs::Twist msg;
        msg.linear.x  = x_new * speed / 100;
        msg.angular.z = z_new * speed / 100;
        pubmove.publish(msg);
        // return tuple
        return make_tuple(x_new, z_new);
    }
    return make_tuple(x_old, z_old);
}

// If controller recieves new commands and is in an arm mode, send message to arm
void AllController::publishArmMessage(std::string outMsg) {
    std_msgs::String outMsgWrapper;
    outMsg += '\n';
    outMsgWrapper.data = outMsg;
    pubarm.publish(outMsgWrapper);
}

bool AllController::inDeadzone(int value) {
    return state == Mode::wheels ? value > 108 && value < 148 :  value > 88 && value < 168 ;
}

// Updates z, which is then published by publish___XZ in readInputs()
void AllController::leftJoystickX(int value) {

    if (inDeadzone(value)) {
        if(state == Mode::wheels) {
        x = 0;
        }
    } 
    
    else {
        // 128 is the center, so this normalizes the result to
        // [-1,1]*Z_SENSITIVITY
        ROS_INFO("Left Joystick X event with value: %d\n", value);
        x = (value - 128) / 128.0 * X_SENSITIVITY;
    }
}

// Updates x, which is then published by publish___XZ in readInputs()
void AllController::leftJoystickY(int value) {
    if (inDeadzone(value)) {
        if(state == Mode::wheels) {
            z = 0;
        }

        else if(state == Mode::arm_joint_space){
            armOutVal = leftJSRel;
        }
    } else {

        // 128 is the center, so this normalizes the result to
        // [-1,1]*X_SENSITIVITY
        ROS_INFO("Left Joystick Y event with value: %d\n", value);
        z = -(value - 128) / 128.0 * Z_SENSITIVITY;
        if(z > 0) {
            armOutVal = leftJSU;
        }

        else {
            armOutVal = leftJSD;
        }
    }
}

// Currently doing nothing
void AllController::rightJoystickX(int value) {
    if (inDeadzone(value)) {
        // do nothing
    } else {
        ROS_INFO("Right Joystick X event with value: %d\n", value);
    }
}

void AllController::rightJoystickY(int value) {

    if (inDeadzone(value)) {
        if(state != Mode::wheels)
            armOutVal = rightJSRel;
    }
    
    else {

        // 128 is the center, so this normalizes the result to
        // [-1,1]*Z_SENSITIVITY
        ROS_INFO("Right Joystick Y event with value: %d\n", value);
        z = -(value - 128) / 128.0 * Z_SENSITIVITY;

        // Right joystick is only used in Y direction in all arm modes
        if(state != Mode::wheels) {

            if(z > 0) {
             armOutVal = rightJSU;
            }

            else {
             armOutVal = rightJSD;
            }
        }
    }
}

void AllController::A(int value) {
    if (value == 1) {
        ROS_INFO("A button pressed");
     armOutVal = buttonA;
    } else if (value == 0) {
        ROS_INFO("A button released");
     armOutVal = buttonARel;
    }
}

void AllController::B(int value) {
    if (value == 1) {
        ROS_INFO("B button pressed");
     armOutVal = buttonB;
    } else if (value == 0) {
        ROS_INFO("B button released");
     armOutVal = buttonBRel;
    }
}

void AllController::X(int value) {
    if (value == 1) {
        ROS_INFO("X button pressed");
        armOutVal = buttonX;
    } else if (value == 0) {
        ROS_INFO("X button released");
        armOutVal = buttonXRel;
    }
}

void AllController::Y(int value) {
    if (value == 1) {
        ROS_INFO("Y button pressed");
        armOutVal = buttonY;
    } else if (value == 0) {
        ROS_INFO("Y button released");
        armOutVal = buttonYRel;
    }
}

void AllController::leftBumper(int value) {
    if (value == 1) {
        ROS_INFO("Left bumper pressed");
        armOutVal = bumperL;
    } else if (value == 0) {
        ROS_INFO("Left bumper released");
        armOutVal = bumperLRel;
    }
}

void AllController::rightBumper(int value) {
    if (value == 1) {
        ROS_INFO("Right bumper pressed");
        armOutVal = bumperR;
    } else if (value == 0) {
        ROS_INFO("Right bumper released");
        armOutVal = bumperRRel;
    }
}

void AllController::select(int value) {
    if (value == 1) {
        ROS_INFO("Select button pressed");
    } else if (value == 0) {
        ROS_INFO("Select button released");
        armOutVal = homeValEE;
    }
}

void AllController::start(int value) {
    if (value == 1) {
        ROS_INFO("Start button pressed");
    } else if (value == 0) {
        ROS_INFO("Start button released");
        armOutVal = homeVal;
    }
}

// Currently switches between wheels and arm mode
void AllController::home(int value) {
    if (!debug) {
        if (value == 1) {
            ROS_INFO("Home button pressed");
        } else if (value == 0) {
            state = static_cast<Mode>((state + 1) % (Mode::num_modes));
            if (state == Mode::wheels || state == Mode::arm_joint_space) {
                pubmode.publish(false_message);
            } else {
                // pubmovegrp.publish(true_message);
                // sleep(8);
                pubmode.publish(true_message);
            }
            printState();
        }
    }
}

void AllController::leftTrigger(int value) {
    if (value == 255) {
        ROS_INFO("Left trigger pressed");
        armOutVal = triggerL;
    } else if (value == 0) {
        ROS_INFO("Left trigger released");
        armOutVal = triggerLRel;
    }
}

void AllController::rightTrigger(int value) {
    if (value == 255) {
        ROS_INFO("Right trigger pressed");
        armOutVal = triggerR;
    } else if (value == 0) {
        ROS_INFO("Right trigger released");
        armOutVal = triggerRRel;
    }
}

void AllController::arrowsRorL(int value) {
    if (value == 1) {
        ROS_INFO("Right button pressed");
        armOutVal = arrowR;
    } else if (value == 0) {
        ROS_INFO("Arrow button released");
        armOutVal = arrowRLRel;
    } else {
        ROS_INFO("Left button pressed");
        armOutVal = arrowL;
    }
}

void AllController::arrowsUorD(int value) {
    if (value == 1) {
        ROS_INFO("Up button pressed");
        armOutVal = arrowU;
	if (state == Mode::wheels) {
	    speed = speed < max_speed ? speed + increment : speed;
	    ROS_INFO("Speed increased to %d%% of max output", speed);
	}
    } else if (value == 0) {
        ROS_INFO("Arrow button released");
        armOutVal = arrowUDRel;
    } else {
        ROS_INFO("Down button pressed");
        armOutVal = arrowD;
	if (state == Mode::wheels) {
	    speed = speed > increment ? speed - increment : speed;
	    ROS_INFO("Speed decreased to %d%% of max output", speed);
	}
    }
}

// Currently doing nothing
void AllController::leftJoystickPress(int value) {
    if (value == 1) {
        ROS_INFO("Left Joystick pressed");
    } else if (value == 0) {
        ROS_INFO("Left Joystick released");
    }
}

// Currently doing nothing
void AllController::rightJoystickPress(int value) {
    if (value == 1) {
        ROS_INFO("Right Joystick pressed");
        armOutVal = rightJSPress;
    } else if (value == 0) {
        ROS_INFO("Right Joystick released");
        armOutVal = rightJSPressRel;
    }
}
