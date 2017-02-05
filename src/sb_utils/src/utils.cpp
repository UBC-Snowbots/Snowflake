/*
 * Created By: Gareth Ellis
 * Created On: February 4th, 2017
 * Description:
 *  - A collection of useful functions used across multiple projects
 *  - All functions should be preface with "SB_" to indicate they are
 *  functions "owned" by our team, preventing confusion with other,
 *  similarly named functions in other packages
 */

#include <utils.h>

template <typename T>
void SB_getParam(ros::NodeHandle& nh, const std::string& param_name,
                 T& param_val, const T& default_val){
    if (!nh.param(param_name, param_val, default_val)){
        // Casting to a double here is not great, but seems the
        // only option available with ros's warning system
        ROS_WARN("%s param not set, defaulting to %f", param_name, (double)param_val);
    }
}