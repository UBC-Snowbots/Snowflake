/**
 * Created by William Gu on 24/03/18
 * Implementation of Cone Identification methods for identifying cones from a laser msg
 */

#include <ConeIdentification.h>


std::vector<mapping_igvc::ConeObstacle> ConeIdentification::identifyCones(const sensor_msgs::LaserScan &laser_msg, float tolerance){
    std::vector<mapping_igvc::ConeObstacle> identified_cones;
    std::vector<mapping_igvc::Point2D> edge_points; //Represents points in a potential cluster

    int numIndices = (laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment;
    for (int i=0; i<numIndices; i++){

        if (laser_msg.ranges[i] > laser_msg.range_max || laser_msg.ranges[i] < laser_msg.range_min) { //Check for invalid ranges (points)
            if (edge_points.size() >= 3) {
                identified_cones.push_back(edgeToCone(edge_points));
            }
            edge_points.clear();
            continue;
        }

        mapping_igvc::Point2D point = laserToPoint(laser_msg.ranges[i], laser_msg.angle_min + i * laser_msg.angle_increment);
        //Out of tolerance or end of points, analyze edge points so far to create cone, and clear edge_points
        if (getDist(edge_points.back(), point) > tolerance || i == numIndices - 1){
            if (edge_points.size() >= 3) { //Ignore edge clusters of 2 or less, can't accurately analyze
                identified_cones.push_back(edgeToCone(edge_points));
            }
            edge_points.clear();
        }
        edge_points.push_back(point);
    }
    return identified_cones;
}


mapping_igvc::Point2D ConeIdentification::laserToPoint(float dist, float ang){
    mapping_igvc::Point2D point = mapping_igvc::Point2D();
    point.x = dist * cos(ang);
    point.y = dist * sin(ang);
    return point;
}


float ConeIdentification::getDist(const mapping_igvc::Point2D &p1, const mapping_igvc::Point2D &p2){
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}


mapping_igvc::ConeObstacle ConeIdentification::edgeToCone(const std::vector<mapping_igvc::Point2D> &edge_points){
    mapping_igvc::ConeObstacle cone = mapping_igvc::ConeObstacle();

    //We can assume there's at least 3 points at the edge of the cone
    /*Indices for points to sample from edge cluster*/
    int i1 = 0;
    int i3 = edge_points.size()-1;
    int i2 = i3/2;

    /* Formula from http://paulbourke.net/geometry/circlesphere/ */
    float slopeA = (edge_points[i2].y - edge_points[i1].y) / (edge_points[i2].x - edge_points[i1].x);
    float slopeB = (edge_points[i3].y - edge_points[i2].y) / (edge_points[i3].x - edge_points[i2].x);
    cone.center.x = (((slopeA * slopeB) * (edge_points[i1].y - edge_points[i3].y)) +
                    slopeB * (edge_points[i1].x + edge_points[i2].x) -
                    slopeA * (edge_points[i2].x + edge_points[i3].x)) /
                    (2 * (slopeB - slopeA));
    cone.center.y = -((cone.center.x - (edge_points[i1].x + edge_points[i2].x) / 2) / slopeA) +
                    ((edge_points[i1].y + edge_points[i2].y) / 2);
    cone.radius = getDist(edge_points[i1], cone.center);

    //TODO: create header

    return cone;
}