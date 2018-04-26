/**
 * Created by William Gu on 24/03/18
 * Implementation of Cone Identification methods for identifying cones from a laser msg
 */

#include <ConeIdentification.h>


std::vector<mapping_igvc::ConeObstacle> ConeIdentification::identifyCones(const sensor_msgs::LaserScan &laser_msg, float dist_tol, float radius_exp, float radius_tol){
    std::vector<mapping_igvc::ConeObstacle> identified_cones;
    std::vector<mapping_igvc::Point2D> edge_points; //Represents points in a potential cluster

    int numIndices = (laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment;
    for (int i=0; i<numIndices; i++){
        if (laser_msg.ranges[i] > laser_msg.range_max || laser_msg.ranges[i] < laser_msg.range_min) { //Check if curr laserscan point is in invalid range
            if (edge_points.size() >= 3) { //Ignore edge clusters of 2 or less, consider edge points so far
                mapping_igvc::ConeObstacle potential_cone = edgeToCone(edge_points);
                if (fabs(potential_cone.radius - radius_exp <= radius_tol)){ //Within dist_tol
                    potential_cone.radius = radius_exp;
                    identified_cones.push_back(potential_cone);
                }
            }
            edge_points.clear();
            continue;
        }

        mapping_igvc::Point2D point = laserToPoint(laser_msg.ranges[i], laser_msg.angle_min + i * laser_msg.angle_increment); //Convert to x-y point
        edge_points.push_back(point);

        //Out of dist_tol or end of points, analyze edge points so far to create cone, and clear edge_points
        if ((getDist(edge_points.back(), point) > dist_tol) || i == numIndices - 1){
            if (edge_points.size() >= 3) { //Ignore edge clusters of 2 or less
                mapping_igvc::ConeObstacle potential_cone = edgeToCone(edge_points);
                if (fabs(potential_cone.radius - radius_exp <= radius_tol)){ //Within dist_tol
                    potential_cone.radius = radius_exp;
                    identified_cones.push_back(potential_cone);
                }
            }
            edge_points.clear();
        }
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
    int start_i = 0; //First index
    int end_i = edge_points.size()-1; //Last index

    float total_x = 0;
    float total_y = 0;
    float total_radius = 0;

    /* Formula from http://paulbourke.net/geometry/circlesphere/ */
    //Get avg center
    for (int i = 1; i < end_i; i++) {
        float slopeA = (edge_points[i].y - edge_points[start_i].y) / (edge_points[i].x - edge_points[start_i].x);
        float slopeB = (edge_points[end_i].y - edge_points[i].y) / (edge_points[end_i].x - edge_points[i].x);
        total_x += (((slopeA * slopeB) * (edge_points[start_i].y - edge_points[end_i].y)) +
                         slopeB * (edge_points[start_i].x + edge_points[i].x) -
                         slopeA * (edge_points[i].x + edge_points[end_i].x)) /
                        (2 * (slopeB - slopeA));
        total_y += -(((total_x / i) - (edge_points[start_i].x + edge_points[i].x) / 2) / slopeA) +
                        ((edge_points[start_i].y + edge_points[i].y) / 2);
    }

    cone.center.x = total_x / (end_i - 1);
    cone.center.y = total_y / (end_i - 1);

    //Get avg radius
    for (int i=0; i <= end_i; i++){
        total_radius += ConeIdentification::getDist(edge_points[i], cone.center);
    }
    cone.radius = total_radius / (end_i + 1);

    return cone;
    //TODO: create header
}