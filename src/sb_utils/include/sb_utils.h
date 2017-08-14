/*
 * Created By: Gareth Ellis
 * Created On: February 4th, 2017
 * Description:
 *  - A collection of useful functions used across multiple projects
 *  - All functions should be preface with "SB_" to indicate they are
 *  functions "owned" by our team, preventing confusion with other,
 *  similarly named functions in other packages
 */

#ifndef UTILS_UTILS_H
#define UTILS_UTILS_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * Get a param with a default value
 *
 * Gets a param with a given name and sets a given variable to it, or to a
 * default value if the param could not be retrieved. Also will warn if the
 * param could not be retrieved.
 *
 * Because this is a template function, it's definition must be in the header
 *
 * @param nh the nodehandle on which to look for the param,
 *        should be the private nodehandle
 * @param param_name the name of the param to get
 * @param param_val the variable to set with the retrieved param value
 * @param default_val the value to set param_val to if no param was found
 *
 * @return true if param was found, false otherwise
 */
template <typename T>
bool SB_getParam(ros::NodeHandle& nh, const std::string& param_name,
                 T& param_val, const T& default_val){
    if (!nh.param(param_name, param_val, default_val)){
        ROS_WARN_STREAM(nh.getNamespace() << ": " << param_name
                                          << " not set, defaulting to " << param_val);
        return false;
    }
    return true;
}
/**
 * Get a param with no default value
 *
 * Gets a param with a given name and sets a given variable to it, or to a
 * default value if the param could not be retrieved. Also will warn if the
 * param could not be retrieved.
 *
 * Because this is a template function, it's definition must be in the header
 *
 * @param nh the nodehandle on which to look for the param,
 *        should be the private nodehandle
 * @param param_name the name of the param to get
 * @param param_val the variable to set with the retrieved param value
 *
 * @return true if param was found, false otherwise
 */
template <typename T>
bool SB_getParam(ros::NodeHandle& nh, const std::string& param_name, T& param_val){
    if (!nh.getParam(param_name, param_val)){
        ROS_ERROR_STREAM(nh.getNamespace() << ": no value given for " << param_name <<
                                           " param, and no default value set!!");
        return false;
    }
    return true;
}

/**
 * Transforms a given message from it's current frame to a given frame
 * Note that input may be the same as output (ie. you can just change the
 * frame of a given object, rather then place the transformed object in a
 * new object)
 *
 * @param input the message to be transformed
 * @param output the transformed message will be placed in this
 * @param output_frame the frame to transform the input to
 */
template <typename T>
void SB_doTransform(const T& input, T& output, std::string output_frame){
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(
            output_frame, input.header.frame_id, ros::Time(0), ros::Duration(1.0)
    );
    tf2::doTransform(input, output, transformStamped);
}

#endif //UTILS_UTILS_H
