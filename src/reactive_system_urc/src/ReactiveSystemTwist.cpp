/**
 * Created by William Gu on Nov 3 2018
 * Implementation for Reactive System Pathfinder
 */

#include <ReactiveSystemTwist.h>

geometry_msgs::Twist ReactiveSystemTwist::getTwist(mapping_msgs_urc::RiskAreaArray risk_areas, geometry_msgs::Point32 goal_pos, float traj_time_inc, int traj_num_incs, float linear_vel, float max_angular_vel, int num_angular_vel, float risk_dist_tol_sq) {

    geometry_msgs::Twist twist;
    twist.linear.x = linear_vel;
    twist.angular.z = 0;

    //Find lowest trajectory score given the range of angular velocities we search
    float min_traj_score = std::numeric_limits<float>::max();

    for (int i = -num_angular_vel; i <= num_angular_vel; i++){
        float angular_vel = i * (max_angular_vel/num_angular_vel);
        std::vector<geometry_msgs::Point32> trajectory = getArcTrajectory(linear_vel, angular_vel, traj_time_inc, traj_num_incs);
        float traj_score = getTrajectoryScore(trajectory, goal_pos, risk_areas, risk_dist_tol_sq);
        if (traj_score < min_traj_score) {
            min_traj_score = traj_score;
            twist.angular.z = angular_vel;
        }
    }

    return twist;
}


std::vector<geometry_msgs::Point32> ReactiveSystemTwist::getArcTrajectory(float linear_vel, float angular_vel, float time_inc, int num_incs){

    std::vector<geometry_msgs::Point32> arc_trajectory;

    if (abs(angular_vel) < 0.01){ //essentially a straight trajectory
        for (int i = 1; i <= num_incs; i++){
            geometry_msgs::Point32 arc_point;
            arc_point.x = time_inc * i * linear_vel;
            arc_point.y = 0;
            arc_trajectory.push_back(arc_point);
        }
    }
    else{ //use arc formula to obtain trajectory
        geometry_msgs::Point32 center;
        center.x = 0;
        center.y = linear_vel/angular_vel;

        for (int i = 1; i <= num_incs; i++){
            float angle = time_inc * i * angular_vel;
            geometry_msgs::Point32 arc_point = ReactiveSystemTwist::getArcPoint(center, angle);
            arc_trajectory.push_back(arc_point);
        }

    }

    return arc_trajectory;
}


geometry_msgs::Point32 ReactiveSystemTwist::getArcPoint(geometry_msgs::Point32 center, float angle){

    //Calculate the arc point in a given arc trajectory by rotating (0,0) about the center point, by angle
    geometry_msgs::Point32 arc_point;

    /*
    arc_point.x = center.x - center.x * cos(angle) + center.y * sin(angle);
    arc_point.y = center.y - center.x * sin(angle) - center.y * cos(angle);
    */

    //simplified version (since center.x is always 0):
    arc_point.x = center.y * sin(angle);
    arc_point.y = center.y - center.y * cos(angle);

    return arc_point;
}


float ReactiveSystemTwist::getTrajectoryScore(std::vector<geometry_msgs::Point32> trajectory, geometry_msgs::Point32 goal_pos, mapping_msgs_urc::RiskAreaArray risk_areas, float dist_tol_sq){

    float risk_score = 0;

    for (int i=0; i<trajectory.size(); i++){
        geometry_msgs::Point32 traj_point = trajectory[i];
        float traj_risk = -1; //risk score for a given trajectory point, set to -1 since actual risks should be >= 0
        for (mapping_msgs_urc::RiskArea area : risk_areas.areas){ //Check all points for all risk polygons
            for (geometry_msgs::Point32 risk_point : area.area.points){
                if (isWithinDistance(risk_point, traj_point, dist_tol_sq)){
                    if (area.score.data > traj_risk) //Keep track of the highest risk for this trajectory point
                        traj_risk = area.score.data;
                    break;
                }
            }
        }
        if (traj_risk < 0) //unknown risk, so just assign it the maximum risk
            traj_risk = 100;

        //Multiplier applied to points based on proximity to current position (closer = higher multiplier)
        traj_risk *= (trajectory.size() - i);
        risk_score += traj_risk;
    }

    //Apply a multiplier to the total risk score for a given trajectory, based on how close it heads towards the goal (closer = lower multiplier)
    float goal_ang = atan2(goal_pos.y, goal_pos.x);
    float traj_ang = atan2(trajectory[trajectory.size()-1].y, trajectory[trajectory.size()-1].x); //approximate trajectory angle from the line b/w last point in the trajectory and origin
    float ang_diff = abs(goal_ang - traj_ang);
    float goal_multiplier = ang_diff + 1; //TODO: may need to tweak this
    risk_score *= goal_multiplier;

    return risk_score;
}


bool ReactiveSystemTwist::isWithinDistance(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, float dist_tol_sq){

    return (pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) < dist_tol_sq);
}