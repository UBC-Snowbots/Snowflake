/**
 * Created by William Gu on 24/03/18
 * Implementation of Cone Identification methods for identifying cones from a laser msg
 */

#include <ConeIdentification.h>


std::vector<mapping_igvc::ConeObstacle> ConeIdentification::identifyCones(const sensor_msgs::LaserScan &laser_msg, double dist_tol, double radius_exp, double radius_tol, int line_point_dist, double ang_threshold){
    std::vector<mapping_igvc::ConeObstacle> identified_cones;
    std::vector<mapping_igvc::Point2D> edge_points; //Represents points in a potential cluster

    int numIndices = (laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment;
    for (int i=0; i<numIndices; i++){
        if (laser_msg.ranges[i] > laser_msg.range_max || laser_msg.ranges[i] < laser_msg.range_min) { //Check if curr laserscan point is in invalid range

            if (edge_points.size() >= line_point_dist * 2 + 1) { // Don't bother splitting edges with too few points
                std::vector<std::vector<mapping_igvc::Point2D>> split_edges = splitEdge(edge_points, line_point_dist, ang_threshold); //Split edge points if multiple cones detected (Temp values right now as params)
                for (int i = 0; i < split_edges.size(); i++){
                    mapping_igvc::ConeObstacle potential_cone = edgeToCone(split_edges[i]);

                    //Testing
                    std::cout<<"Split Result"<<std::endl;
                    std::cout<<"X: "<<potential_cone.center.x<<std::endl;
                    std::cout<<"Y: "<<potential_cone.center.y<<std::endl;
                    std::cout<<"RADIUS: "<<potential_cone.radius<<std::endl;
                    std::cout<<std::endl;

                    if (fabs(potential_cone.radius - radius_exp <= radius_tol)){ //Within expected radius
                        potential_cone.radius = radius_exp;
                        identified_cones.push_back(potential_cone);
                    }
                }
            }
            else if (edge_points.size() >= 3) { //Directly make the cone if not enough points to split (cone still needs at least 3 edge points though)
                mapping_igvc::ConeObstacle potential_cone = edgeToCone(edge_points);

                //Testing
                std::cout<<"No Split Result"<<std::endl;
                std::cout<<"X: "<<potential_cone.center.x<<std::endl;
                std::cout<<"Y: "<<potential_cone.center.y<<std::endl;
                std::cout<<"RADIUS: "<<potential_cone.radius<<std::endl;
                std::cout<<std::endl;

                if (fabs(potential_cone.radius - radius_exp <= radius_tol)){ //Within expected radius
                    potential_cone.radius = radius_exp;
                    identified_cones.push_back(potential_cone);
                }
            }

            edge_points.clear();
        }

        else { //Laserscan point in range
            mapping_igvc::Point2D point = laserToPoint(laser_msg.ranges[i], laser_msg.angle_min + i * laser_msg.angle_increment); //Convert to x-y point
            edge_points.push_back(point);

            //Out of dist_tol or end of points, analyze edge points so far to create cone, and clear edge_points
            if ((getDist(edge_points.back(), point) > dist_tol) || i == numIndices - 1){

                if (edge_points.size() >= line_point_dist * 2 + 1) { // Don't bother splitting edges with too few points
                    std::vector<std::vector<mapping_igvc::Point2D>> split_edges = splitEdge(edge_points, line_point_dist, ang_threshold); //Split edge points if multiple cones detected (Temp values right now as params)
                    for (int i = 0; i < split_edges.size(); i++){
                        mapping_igvc::ConeObstacle potential_cone = edgeToCone(split_edges[i]);

                        //Testing
                        std::cout<<"Split Result"<<std::endl;
                        std::cout<<"X: "<<potential_cone.center.x<<std::endl;
                        std::cout<<"Y: "<<potential_cone.center.y<<std::endl;
                        std::cout<<"RADIUS: "<<potential_cone.radius<<std::endl;
                        std::cout<<std::endl;

                        if (fabs(potential_cone.radius - radius_exp <= radius_tol)){ //Within expected radius
                            potential_cone.radius = radius_exp;
                            identified_cones.push_back(potential_cone);
                        }
                    }
                }
                else if (edge_points.size() >= 3) { //Directly make the cone if not enough points to split (cone still needs at least 3 edge points though)
                    mapping_igvc::ConeObstacle potential_cone = edgeToCone(edge_points);

                    //Testing
                    std::cout<<"No Split Result"<<std::endl;
                    std::cout<<"X: "<<potential_cone.center.x<<std::endl;
                    std::cout<<"Y: "<<potential_cone.center.y<<std::endl;
                    std::cout<<"RADIUS: "<<potential_cone.radius<<std::endl;
                    std::cout<<std::endl;

                    if (fabs(potential_cone.radius - radius_exp <= radius_tol)){ //Within expected radius
                        potential_cone.radius = radius_exp;
                        identified_cones.push_back(potential_cone);
                    }
                }

                edge_points.clear();
            }
        }
    }

    return identified_cones;
}


mapping_igvc::Point2D ConeIdentification::laserToPoint(double dist, double ang){
    mapping_igvc::Point2D point = mapping_igvc::Point2D();
    point.x = dist * cos(ang);
    point.y = dist * sin(ang);
    return point;
}


double ConeIdentification::getDist(const mapping_igvc::Point2D &p1, const mapping_igvc::Point2D &p2){
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}


std::vector<std::vector<mapping_igvc::Point2D>> ConeIdentification::splitEdge(const std::vector<mapping_igvc::Point2D> &edge_points, int line_point_dist, double ang_threshold){
    std::vector<std::vector<mapping_igvc::Point2D>> split_edges;

    std::vector<double> angles; //index 0 corresponds to index line_point_dist of edge_points vector
    for (int i = line_point_dist; i < edge_points.size() - line_point_dist; i++){

        //Calculate the angle between the two lines we form
        double AB_x = edge_points[i].x - edge_points[i - line_point_dist].x;
        double AB_y = edge_points[i].y - edge_points[i - line_point_dist].y;
        double CB_x = edge_points[i].x - edge_points[i + line_point_dist].x;
        double CB_y = edge_points[i].y - edge_points[i + line_point_dist].y;
        double dot = AB_x * CB_x + AB_y * CB_y;
        double cross = AB_x * CB_y - AB_y * CB_x;
        double ang = -atan2(cross, dot);
        if (ang < 0) ang += 2 * M_PI;

        angles.push_back(ang);
    }

    //Find local mins, then check if they are below threshold
    std::vector<size_t> splitIndices;
    for (int i = 1; i < angles.size() - 1; i++){
        //TODO: May have to enforce local minima harder and force angle to smaller than many around it
        if (angles[i] < angles[i-1] && angles[i] < angles[i+1] && angles[i] < ang_threshold)
            splitIndices.push_back(i + line_point_dist);
    }

    //Split edges based on qualified local mins
    size_t lastIndex = 0;
    for (int i = 0; i < splitIndices.size(); i++) {
        std::vector<mapping_igvc::Point2D> split(edge_points.begin() + lastIndex,
                                                 edge_points.begin() + splitIndices[i] + 1);
        split_edges.push_back(split);
        lastIndex = splitIndices[i];
    }
    std::vector<mapping_igvc::Point2D> lastSplit(edge_points.begin() + lastIndex, edge_points.end());// Add last cluster of edges
    split_edges.push_back(lastSplit);

    return split_edges;
}


mapping_igvc::ConeObstacle ConeIdentification::edgeToCone(const std::vector<mapping_igvc::Point2D> &edge_points){
    mapping_igvc::ConeObstacle cone = mapping_igvc::ConeObstacle();

    //We can assume there's at least 3 points at the edge of the cone
    int start_i = 0; //First index
    int end_i = edge_points.size()-1; //Last index

    double total_x = 0;
    double total_y = 0;
    double total_radius = 0;

    /* Formula from http://paulbourke.net/geometry/circlesphere/ */
    //Get avg center
    for (int i = 1; i < end_i; i++) {
        double slopeA = (edge_points[i].y - edge_points[start_i].y) / (edge_points[i].x - edge_points[start_i].x);
        double slopeB = (edge_points[end_i].y - edge_points[i].y) / (edge_points[end_i].x - edge_points[i].x);
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
        total_radius += getDist(edge_points[i], cone.center);
    }
    cone.radius = total_radius / (end_i + 1);

    return cone;
    //TODO: create header
}