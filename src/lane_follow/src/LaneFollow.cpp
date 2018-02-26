/*
 * Created By: Raad Khan
 * Created On: April 23, 2017
 * Description: Takes in an image feed and uses LineDetect to generate
 *              lane lines and destination point, then broadcasts a
 *              Twist message to stay within the lanes.
 */

#include <LaneFollow.h>

class Twist;

using namespace cv;

LaneFollow::LaneFollow(int argc, char **argv, std::string node_name) {
    // rewrite
}
