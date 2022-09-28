/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Utilities to create Rviz visualisation messages
 *
 */
#ifndef SB_UTILS_RVIZUTILS_H
#define SB_UTILS_RVIZUTILS_H

// ROS
#include <ros/ros.h>

// Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "sb_geom_msgs/Point2D.h"
#include "sb_geom_msgs/Polygon2D.h"

namespace snowbots {
namespace RvizUtils {
    /**
     *  Turn points into a marker for rviz
     *
     *  @param points the points to be converted
     *  @param color the color of the points
     *  @param frame_id the frame id
     *  @param ns the namespace
     *  @param marker_id the marker_id that enables Rviz to differentiate
     * markers.
     *  @param type the type of marker specified by
     * visualization_msgs::Marker::*
     *
     *  @return an rviz marker
     */
    visualization_msgs::Marker
    createMarker(std::vector<geometry_msgs::Point> points,
                 visualization_msgs::Marker::_color_type color,
                 visualization_msgs::Marker::_scale_type scale,
                 std::string frame_id,
                 std::string ns,
                 int marker_id,
                 int type = visualization_msgs::Marker::POINTS);

    /**
     *  Turn points into a marker for rviz
     *  Can specify a color for each point
     *
     *  @param points the points to be converted
     *  @param colors the color of each point
     *  @param frame_id the frame id
     *  @param ns the namespace
     *  @param marker_id the marker_id that enables Rviz to differentiate
     * markers.
     *  @param type the type of marker specified by
     * visualization_msgs::Marker::*
     *
     *  @return an rviz marker
     */
    visualization_msgs::Marker
    createMarker(std::vector<geometry_msgs::Point> points,
                 std::vector<std_msgs::ColorRGBA> colors,
                 visualization_msgs::Marker::_scale_type scale,
                 std::string frame_id,
                 std::string ns,
                 int marker_id,
                 int type = visualization_msgs::Marker::POINTS);

    /**
     *  Turn a point into a marker for rviz
     *
     *  @param point the point to be converted
     *  @param color the color of the point
     *  @param frame_id the frame id
     *  @param ns the namespace
     *  @param marker_id the marker_id that enables Rviz to differentiate
     * markers.
     *  @param type the type of marker specified by
     * visualization_msgs::Marker::*
     *
     *  @return an rviz marker
     */
    visualization_msgs::Marker
    createMarker(geometry_msgs::Point point,
                 std::vector<std_msgs::ColorRGBA> colors,
                 visualization_msgs::Marker::_scale_type scale,
                 std::string frame_id,
                 std::string ns,
                 int type = visualization_msgs::Marker::POINTS,
                 int id   = 0);

    /**
     *  Turn a point into a marker for rviz
     *
     *  @param point the point to be converted
     *  @param color the color of the point
     *  @param frame_id the frame id
     *  @param ns the namespace
     *  @param marker_id the marker_id that enables Rviz to differentiate
     * markers.
     *  @param type the type of marker specified by
     * visualization_msgs::Marker::*
     *
     *  @return an rviz marker
     */
    visualization_msgs::Marker
    createMarker(geometry_msgs::Point point,
                 visualization_msgs::Marker::_color_type color,
                 visualization_msgs::Marker::_scale_type scale,
                 std::string frame_id,
                 std::string ns,
                 int marker_id,
                 int type = visualization_msgs::Marker::POINTS);

    /**
     *  Turn a polygon into a marker for rviz
     *
     *  @param polygon the polygon to be converted
     *  @param color the color of the polygon
     *  @param frame_id the frame id
     *  @param ns the namespace
     *  @param marker_id the marker_id that enables Rviz to differentiate
     * markers.
     *  @param type the type of marker specified by
     * visualization_msgs::Marker::*
     *
     *  @return an rviz marker
     */
    visualization_msgs::Marker
    createPolygonMarker2D(sb_geom_msgs::Polygon2D polygon,
                          visualization_msgs::Marker::_color_type color,
                          visualization_msgs::Marker::_scale_type scale,
                          std::string frame_id,
                          std::string ns,
                          int marker_id,
                          int type = visualization_msgs::Marker::LINE_STRIP);
    /**
     *  Turn a polygon into a marker for rviz
     *
     *  @param polygon the polygon to be converted
     *  @param color the color of the polygon
     *  @param frame_id the frame id
     *  @param ns the namespace
     *  @param marker_id the marker_id that enables Rviz to differentiate
     * markers.
     *  @param type the type of marker specified by
     * visualization_msgs::Marker::*
     *
     *  @return an rviz marker
     */
    visualization_msgs::Marker
    createPolygonMarker3D(geometry_msgs::Polygon polygon,
                          visualization_msgs::Marker::_color_type color,
                          visualization_msgs::Marker::_scale_type scale,
                          std::string frame_id,
                          std::string ns,
                          int marker_id,
                          int type = visualization_msgs::Marker::LINE_STRIP);

    /**
     * Creates a Marker Array (array of Markers)
     *
     * @param points_array each array inside corresponds to one marker
     * @param color color of the points in the array
     * @param frame_id frame id of the markers
     * @param ns namespace of the markers
     * @param type the type of marker specified by visualization_msgs::Marker::*
     *
     * @return an rviz marker array
     */
    visualization_msgs::MarkerArray createMarkerArray(
    std::vector<std::vector<geometry_msgs::Point>> points_array,
    visualization_msgs::Marker::_color_type color,
    visualization_msgs::Marker::_scale_type scale,
    std::string frame_id,
    std::string ns,
    int type = visualization_msgs::Marker::POINTS);

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
    visualization_msgs::Marker::_color_type
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
    visualization_msgs::Marker::_scale_type
    createMarkerScale(float x, float y, float z);
};
};
#endif // HOLE_TRACKER_RVIZUTILS_H
