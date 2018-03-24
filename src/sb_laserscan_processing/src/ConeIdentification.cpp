/**
 * Created by William Gu on 24/03/18
 * Implementation of Cone Identification methods for identifying cones from a laser msg
 */

#include <ConeIdentification.h>

std::vector<mapping_igvc::ConeObstacle> ConeIdentification::identifyCones(const sensor_msgs::LaserScan &laser_msg, float tolerance){
    std::vector<mapping_igvc::ConeObstacle> identified_cones;
    std::vector<mapping_igvc::Point2D> cluster_points; //Represents points in a potential cluster

    int numIndices = (laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment;
    for (int i=0; i<numIndices; i++){

        if (laser_msg.ranges[i] > laser_msg.range_max || laser_msg.ranges[i] < laser_msg.range_min) { //Check for invalid ranges (points)
            //(Could also create some bounding radius for the condition to just ignore)
            if (cluster_points.size() >= 3) {
                identified_cones.push_back(clusterToCone(cluster_points));
            }
            cluster_points.clear();
            continue;
        }

        mapping_igvc::Point2D point = laserToPoint(laser_msg.ranges[i], laser_msg.angle_min + i * laser_msg.angle_increment);
        if (getDist(cluster_points.back(), point) > tolerance){ //Out of tolerance, end of prev cluster so analyze it first and dump cluster_points
            if (cluster_points.size() >= 3) { //Ignore clusters of 2 or less, consider them as outliers
                identified_cones.push_back(clusterToCone(cluster_points));
            }
            cluster_points.clear();
        }
        cluster_points.push_back(point);
    }
    return identified_cones;
}

mapping_igvc::Point2D ConeIdentification::laserToPoint(float dist, float ang){ //TODO: Points must be in global frame?
    //TODO: Ang is relative to robot, so after we find x + y in robot frame we could translate to global
    mapping_igvc::Point2D point = mapping_igvc::Point2D();
    point.x = dist * cos(ang);
    point.y = dist * sin(ang);
    return point;
}

float ConeIdentification::getDist(const mapping_igvc::Point2D &p1, const mapping_igvc::Point2D &p2){
    return sqrt(pow((p1.x - p2.x) + (p1.y-p2.y), 2));
}

mapping_igvc::ConeObstacle ConeIdentification::clusterToCone(const std::vector<mapping_igvc::Point2D> &cluster_points){
    //We can assume there's at least 3 points
    mapping_igvc::ConeObstacle cone = mapping_igvc::ConeObstacle();

    /*Sample 3 points method*/
    /*Indices for points to sample from cluster*/
    int i1 = 0;
    int i3 = cluster_points.size()-1;
    int i2 = i3/2;

    /* Formula from http://paulbourke.net/geometry/circlesphere/ */
    float slopeA = (cluster_points[i2].y - cluster_points[i1].y)/(cluster_points[i2].x - cluster_points[i1].x);
    float slopeB = (cluster_points[i3].y - cluster_points[i2].y)/(cluster_points[i3].x - cluster_points[i2].x);
    cone.center.x = ((slopeA * slopeB) * (cluster_points[i1].y - cluster_points[i3].y) +
                    slopeB * (cluster_points[i1].x - cluster_points[i2].x) -
                    slopeA * (cluster_points[i2].x + cluster_points[i3].x)) /
                    (2 * (slopeB - slopeA));
    cone.center.y = slopeA * (cone.center.x - cluster_points[i1].x) + cluster_points[i1].y;
    cone.radius = getDist(cluster_points[i1], cone.center);

    return cone;
}