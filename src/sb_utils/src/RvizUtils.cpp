/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Utilities to create Rviz visualisation messages
 *
 */
#include <RvizUtils.h>

using namespace std;
using namespace visualization_msgs;
using namespace snowbots;


Marker RvizUtils::displayPoints(vector<geometry_msgs::Point> points,
                                std::vector<std_msgs::ColorRGBA> colors,
                                Marker::_scale_type scale,
                                string frame_id,
                                string ns) {
    Marker marker;

    initialiseMarkerHeader(marker, frame_id, ns);

    // Set the shape & color
    marker.colors = colors;
    marker.scale = scale;

    // Set the points
    marker.points = points;

    return marker;
}

Marker RvizUtils::displayPoints(vector<geometry_msgs::Point> points,
                                Marker::_color_type color,
                                Marker::_scale_type scale,
                                string frame_id,
                                string ns) {
    Marker marker;

    initialiseMarkerHeader(marker, frame_id, ns);

    // Set the shape & color
    marker.color = color;
    marker.scale = scale;

    // Set the points
    marker.points = points;

    return marker;
}

Marker RvizUtils::displayPoint(geometry_msgs::Point point,
                               Marker::_color_type color,
                               Marker::_scale_type scale,
                               string frame_id,
                               string ns) {
    Marker marker;

    initialiseMarkerHeader(marker, frame_id, ns);

    // Set the display properties
    marker.color = color;
    marker.scale = scale;

    // Set the points
    marker.points.push_back(point);

    return marker;
}

Marker::_color_type
RvizUtils::createMarkerColor(float r, float g, float b, float a) {
    Marker::_color_type color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}

Marker::_scale_type RvizUtils::createrMarkerScale(float x, float y, float z) {
    Marker::_scale_type scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;

    return scale;
}

void RvizUtils::initialiseMarkerHeader(Marker& marker,
                                       string frame_id,
                                       string ns) {
    // Set up frame id and namespace
    marker.header.frame_id = frame_id;
    marker.ns              = ns;

    // Set up other values (TODO: Find out what these values do)
    marker.type               = Marker::POINTS;
    marker.header.stamp       = ros::Time::now();
    marker.action             = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id                 = 0;
}