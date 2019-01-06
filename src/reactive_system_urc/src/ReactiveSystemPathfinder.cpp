/**
 * Created by William Gu on Nov 3 2018
 * Implementation for Reactive System Pathfinder
 */

#include <ReactiveSystemPathfinder.h>

nav_msgs::Path ReactiveSystemPathfinder::pathFinder(mapping_msgs_urc::RiskAreaArray risk_areas, geometry_msgs::Point curr_pos, geometry_msgs::Point goal_pos) {

    nav_msgs::Path path;
    return path;

}


std::vector<geometry_msgs::Point> ReactiveSystemPathfinder::getArcTrajectory(double linear_vel, double angular_vel, double time_inc, int num_incs){

    std::vector<geometry_msgs::Point> arc_trajectory;

    if (abs(angular_vel) < 0.01){ //essentially a straight trajectory
        for (double i = 0; i <= time_inc * num_incs; i += time_inc){
            geometry_msgs::Point arc_point;
            arc_point.x = time_inc * i * linear_vel;
            arc_point.y = 0;
            arc_trajectory.push_back(arc_point);
        }
    }
    else{
        geometry_msgs::Point center;
        center.x = 0;
        center.y = linear_vel/angular_vel;

        for (double i = 0; i <= time_inc * num_incs; i += time_inc){
            double angle = time_inc * i * angular_vel;
            geometry_msgs::Point arc_point = ReactiveSystemPathfinder::getArcPoint(center, angle);
            arc_trajectory.push_back(arc_point);
        }

    }

    return arc_trajectory;
}


geometry_msgs::Point ReactiveSystemPathfinder::getArcPoint(geometry_msgs::Point center, double angle){

    geometry_msgs::Point arc_point;
    arc_point.x = center.x - center.x * cos(angle) + center.y * sin(angle);
    arc_point.y = center.y - center.x * sin(angle) - center.y * cos(angle);

    return arc_point;

}