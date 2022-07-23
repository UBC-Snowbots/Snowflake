#ifndef TEENSY_DRIVER_H
#define TEENSY_DRIVER_H

#include <vector>
#include <unordered_map>
#include <string>
#include "math.h"
#include "time.h"
#include <ros/ros.h>
#include <boost/asio.hpp>

namespace arm_hardware_driver {

class TeensyDriver {
  public:
    void init(std::string port, int baudrate, int num_joints, std::vector<double>& enc_steps_per_deg);
    void setStepperSpeed(std::vector<double>& max_speed, std::vector<double>& max_accel);
    void update(std::vector<double>& pos_commands, std::vector<double>& joint_states);
    void getJointPositions(std::vector<double>& joint_positions);
    void calibrateJoints();

    TeensyDriver();

  private:
    bool initialised_;
    std::string version_;
    boost::asio::io_service io_service_;
    boost::asio::serial_port serial_port_;
    int num_joints_;
    std::vector<int> enc_commands_;
    std::vector<int> enc_steps_;
    std::vector<double> enc_steps_per_deg_;
    std::vector<int> enc_calibrations_;


    // Comms with teensy
    void exchange(std::string outMsg); // exchange joint commands/state
    bool transmit(std::string outMsg, std::string& err);
    bool receive(std::string &inMsg, std::string& err);
    void sendCommand(std::string outMsg); // send arbitrary commands

    void checkInit(std::string msg);
    void updateEncoderCalibrations(std::string msg);
    void updateEncoderSteps(std::string msg);

    // Convert between encoder steps and joint angle degrees
    void encStepsToJointPos(std::vector<int>& enc_steps, std::vector<double>& joint_positions);
    void jointPosToEncSteps(std::vector<double>& joint_positions, std::vector<int>& enc_steps);
};

} // namespace arm_hardware_drivers

#endif // TEENSY_DRIVER
