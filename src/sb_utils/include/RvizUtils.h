/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Utilities to create Rviz visualisation messages
 *
 */
#ifndef SB_UTILS_RVIZUTILS_H
#define SB_UTILS_RVIZUTILS_H

// Messages
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace snowbots {
class RvizUtils {
  public:
    /**
     *  Turn points into a marker for rviz
     *  Can specify a color for each point
     *
     *  @param points the points to be converted
     *  @param colors the color of each point
     *  @param frame_id the frame id
     *  @param ns the namespace
     *
     *  @return an rviz marker
     */
    static visualization_msgs::Marker
    displayPoints(std::vector<geometry_msgs::Point> points,
                  std::vector<std_msgs::ColorRGBA> colors,
                  visualization_msgs::Marker::_scale_type scale,
                  std::string frame_id,
                  std::string ns);

    /**
     *  Turn points into a marker for rviz
     *
     *  @param points the points to be converted
     *  @param color the color of the points
     *  @param frame_id the frame id
     *  @param ns the namespace
     *
     *  @return an rviz marker
     */
    static visualization_msgs::Marker
    displayPoints(std::vector<geometry_msgs::Point> points,
                  visualization_msgs::Marker::_color_type color,
                  visualization_msgs::Marker::_scale_type scale,
                  std::string frame_id,
                  std::string ns);

    /**
     *  Turn a point into a marker for rviz
     *
     *  @param point the point to be converted
     *  @param color the color of the point
     *  @param frame_id the frame id
     *  @param ns the namespace
     *
     *  @return an rviz marker
     */
    static visualization_msgs::Marker
    displayPoint(geometry_msgs::Point point,
                 visualization_msgs::Marker::_color_type color,
                 visualization_msgs::Marker::_scale_type scale,
                 std::string frame_id,
                 std::string ns);

    /**
     *  Create a marker color type based on given red, green, blue, alpha
     * values.
     *
     *  @param r red
     *  @param g green
     *  @param b blue
     *  @param a alpha
     *
     *  @return a marker color type
     */
    static visualization_msgs::Marker::_color_type
    createMarkerColor(float r, float g, float b, float a);

    /**
     *  Create a marker scale type based on given x, y, and z scales.
     *
     *  @param x the x scale
     *  @param y the y scale
     *  @param z the z scale
     *
     *  @return a marker scale type
     */
    static visualization_msgs::Marker::_scale_type
    createrMarkerScale(float x, float y, float z);

  private:
    /**
     *  Private constructor (No reason for someone to make an instance of this
     * class).
     */
    RvizUtils();

    /**
     *  Sets the header information value
     *
     *  @param marker the marker that will have its header initialised
     *  @param frame_id the frame id that the marker will get
     *  @param ns the name space that the marker will get
     */
    static void initialiseMarkerHeader(visualization_msgs::Marker& marker,
                                       std::string frame_id,
                                       std::string ns);
};
};
#endif // HOLE_TRACKER_RVIZUTILS_H
