/*
 * Created By: Robyn Castro
 * Created On: November 10th, 2017
 * Description: Utilities to create Rviz visualisation messages
 *
 */
#include <RvizUtils.h>

using namespace visualization_msgs;

namespace snowbots {
namespace RvizUtils {
    /**
     * Helper function that sets up common marker parameters
     *
     * @param scale the scale
     * @param frame_id the frame id
     * @param ns the namespace
     * @param type the type of marker
     * @param id the id of marker
     */
    void setupMarker(visualization_msgs::Marker& marker,
                     visualization_msgs::Marker::_scale_type scale,
                     std::string frame_id,
                     std::string ns,
                     int marker_id,
                     int type = visualization_msgs::Marker::POINTS);
}
}

Marker
snowbots::RvizUtils::createMarker(std::vector<geometry_msgs::Point> points,
                                  Marker::_color_type color,
                                  Marker::_scale_type scale,
                                  std::string frame_id,
                                  std::string ns,
                                  int marker_id,
                                  int type) {
    Marker marker;

    setupMarker(marker, scale, frame_id, ns, marker_id, type);

    // Set the color
    marker.color = color;

    // Set the points
    marker.points = points;

    return marker;
}

Marker
snowbots::RvizUtils::createMarker(std::vector<geometry_msgs::Point> points,
                                  std::vector<std_msgs::ColorRGBA> colors,
                                  Marker::_scale_type scale,
                                  std::string frame_id,
                                  std::string ns,
                                  int marker_id,
                                  int type) {
    Marker marker;

    setupMarker(marker, scale, frame_id, ns, marker_id, type);

    // Set the colors
    marker.colors = colors;

    // Set the points
    marker.points = points;

    return marker;
}

Marker
snowbots::RvizUtils::createMarker(geometry_msgs::Point point,
                                  std::vector<std_msgs::ColorRGBA> colors,
                                  Marker::_scale_type scale,
                                  std::string frame_id,
                                  std::string ns,
                                  int marker_id,
                                  int type) {
    Marker marker;

    setupMarker(marker, scale, frame_id, ns, marker_id, type);

    // Set the color
    marker.colors = colors;

    // Set the points
    marker.points.push_back(point);

    return marker;
}

Marker snowbots::RvizUtils::createMarker(geometry_msgs::Point point,
                                         Marker::_color_type color,
                                         Marker::_scale_type scale,
                                         std::string frame_id,
                                         std::string ns,
                                         int marker_id,
                                         int type) {
    Marker marker;

    setupMarker(marker, scale, frame_id, ns, marker_id, type);

    // Set the color
    marker.color = color;

    // Set the points
    marker.points.push_back(point);

    return marker;
}

visualization_msgs::Marker snowbots::RvizUtils::createPolygonMarker2D(
sb_geom_msgs::Polygon2D polygon,
visualization_msgs::Marker::_color_type color,
visualization_msgs::Marker::_scale_type scale,
std::string frame_id,
std::string ns,
int marker_id,
int type) {
    visualization_msgs::Marker marker;

    setupMarker(marker, scale, frame_id, ns, marker_id, type);

    // Set the color
    marker.color = color;

    // Setup the line strip
    for (int i = 0; i < polygon.points.size(); i++) {
        geometry_msgs::Point point;
        point.x = polygon.points[i].x;
        point.y = polygon.points[i].y;
        point.z = 0;
        marker.points.push_back(point);
    }

    geometry_msgs::Point point;
    point.x = polygon.points[0].x;
    point.y = polygon.points[0].y;
    point.z = 0;
    marker.points.push_back(point);

    return marker;
}

Marker snowbots::RvizUtils::createPolygonMarker3D(
geometry_msgs::Polygon polygon,
visualization_msgs::Marker::_color_type color,
visualization_msgs::Marker::_scale_type scale,
std::string frame_id,
std::string ns,
int marker_id,
int type) {
    Marker marker;

    setupMarker(marker, scale, frame_id, ns, marker_id, type);

    // Set the color
    marker.color = color;

    // Setup the line strip
    for (int i = 0; i < polygon.points.size(); i++) {
        geometry_msgs::Point point;
        point.x = polygon.points[i].x;
        point.y = polygon.points[i].y;
        point.z = polygon.points[i].z;
        marker.points.push_back(point);
    }
    geometry_msgs::Point point;
    point.x = polygon.points[0].x;
    point.y = polygon.points[0].y;
    point.z = polygon.points[0].z;
    marker.points.push_back(point);

    return marker;
}

MarkerArray snowbots::RvizUtils::createMarkerArray(
std::vector<std::vector<geometry_msgs::Point>> points_array,
Marker::_color_type color,
Marker::_scale_type scale,
std::string frame_id,
std::string ns,
int type) {
    visualization_msgs::MarkerArray markerArray;
    for (unsigned int i = 0; i < points_array.size(); i++) {
        Marker marker =
        createMarker(points_array[i], color, scale, frame_id, ns, type, i);
        markerArray.markers.push_back(marker);
    }

    return markerArray;
}

Marker::_color_type
snowbots::RvizUtils::createMarkerColor(float r, float g, float b, float a) {
    Marker::_color_type color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}

Marker::_scale_type
snowbots::RvizUtils::createMarkerScale(float x, float y, float z) {
    Marker::_scale_type scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;

    return scale;
}

void snowbots::RvizUtils::setupMarker(
Marker& marker,
visualization_msgs::Marker::_scale_type scale,
std::string frame_id,
std::string ns,
int marker_id,
int type) {
    marker.header.stamp       = ros::Time::now();
    marker.action             = Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.type = type;
    marker.id   = marker_id;

    marker.header.frame_id = frame_id;
    marker.ns              = ns;

    marker.scale = scale;
}