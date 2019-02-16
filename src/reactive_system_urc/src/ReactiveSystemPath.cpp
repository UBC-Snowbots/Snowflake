/**
 * Created by William Gu on Nov 3 2018
 * Implementation for Reactive System Pathfinder
 */

#include <ReactiveSystemPath.h>

nav_msgs::Path
ReactiveSystemPath::getPath(mapping_msgs_urc::RiskAreaArray risk_areas,
                            sb_geom_msgs::Point2D goal_pos,
                            float traj_time_inc,
                            int traj_num_incs,
                            float linear_vel,
                            float max_angular_vel,
                            int num_angular_vel,
                            float risk_dist_tol_sq) {
    nav_msgs::Path path;

    float min_traj_score = std::numeric_limits<float>::max();
    std::vector<sb_geom_msgs::Point2D> best_traj;

    // Find lowest trajectory score in the range of [-max_angular_vel,
    // max_angular_vel],
    // out of a total of "num_angular_vel" trajectories
    for (int i = -num_angular_vel; i <= num_angular_vel; i++) {
        float angular_vel = i * (max_angular_vel / num_angular_vel);
        std::vector<sb_geom_msgs::Point2D> trajectory =
        getArcTrajectory(linear_vel, angular_vel, traj_time_inc, traj_num_incs);
        float traj_score =
        getTrajectoryScore(trajectory, goal_pos, risk_areas, risk_dist_tol_sq);

        if (traj_score < min_traj_score) {
            min_traj_score = traj_score;
            best_traj      = trajectory;
        }
    }

    // Convert best trajectory to a path
    for (int i = 0; i < best_traj.size(); i++) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = best_traj[i].x;
        pose_stamped.pose.position.y = best_traj[i].y;
        path.poses.push_back(pose_stamped);
    }

    return path;
}

std::vector<sb_geom_msgs::Point2D> ReactiveSystemPath::getArcTrajectory(
float linear_vel, float angular_vel, float time_inc, int num_incs) {
    std::vector<sb_geom_msgs::Point2D> arc_trajectory;

    if (abs(angular_vel) < 0.01) { // essentially a straight trajectory
        for (int i = 1; i <= num_incs; i++) {
            sb_geom_msgs::Point2D arc_point;
            arc_point.x = time_inc * i * linear_vel;
            arc_point.y = 0;
            arc_trajectory.push_back(arc_point);
        }
    } else { // use arc formula to obtain trajectory
        sb_geom_msgs::Point2D center;
        center.x = 0;
        center.y = linear_vel / angular_vel;

        for (int i = 1; i <= num_incs; i++) {
            float angle = time_inc * i * angular_vel;
            sb_geom_msgs::Point2D arc_point =
            ReactiveSystemPath::getArcPoint(center, angle);
            arc_trajectory.push_back(arc_point);
        }
    }

    return arc_trajectory;
}

sb_geom_msgs::Point2D
ReactiveSystemPath::getArcPoint(sb_geom_msgs::Point2D center, float angle) {
    // Calculate the arc point in a given arc trajectory by rotating (0,0) about
    // the center point, by angle
    sb_geom_msgs::Point2D arc_point;

    arc_point.x = center.y * sin(angle);
    arc_point.y = center.y - center.y * cos(angle);

    return arc_point;
}

float ReactiveSystemPath::getTrajectoryScore(
std::vector<sb_geom_msgs::Point2D> trajectory,
sb_geom_msgs::Point2D goal_pos,
mapping_msgs_urc::RiskAreaArray risk_areas,
float dist_tol_sq) {
    float risk_score = 0;

    // Give each point in the trajectory a riskiness based on proximity to risk
    // areas and immediate proximity to current position
    // Add all the risks for each point for a risk sum for the trajectory
    for (int i = 0; i < trajectory.size(); i++) {
        sb_geom_msgs::Point2D traj_point = trajectory[i];
        float traj_point_risk =
        -1; // risk score for a given trajectory point, set to
            // -1 since actual risks should be >= 0
        for (mapping_msgs_urc::RiskArea area :
             risk_areas.areas) { // Check all points for all risk polygons
            for (sb_geom_msgs::Point2D risk_point : area.area.points) {
                if (isWithinDistance(risk_point, traj_point, dist_tol_sq)) {
                    if (area.score.data > traj_point_risk) // Keep track of the
                        // highest risk for this
                        // trajectory point
                        traj_point_risk = area.score.data;
                    break;
                }
            }
        }
        if (traj_point_risk <
            0) // unknown risk, so just assign it the maximum risk
            traj_point_risk = MAX_RISK;

        // Multiplier applied to points based on proximity to current position
        // (closer = higher multiplier)
        traj_point_risk *= (trajectory.size() - i);
        risk_score += traj_point_risk;
    }

    // Apply an additive value to the total risk score for a given trajectory,
    // based on how close it heads towards the goal (closer = lower multiplier)
    float goal_ang = atan2(goal_pos.y, goal_pos.x);
    float traj_ang =
    atan2(trajectory[trajectory.size() / 2].y,
          trajectory[trajectory.size() / 2].x); // approximate trajectory angle
                                                // from the line b/w last point
                                                // in the trajectory and origin
    float ang_diff   = abs(goal_ang - traj_ang);
    float goal_adder = ang_diff * MAX_RISK; // apply a multiplier to the amount
                                            // that goal difference effects risk
                                            // level
    risk_score += goal_adder;

    return risk_score;
}

bool ReactiveSystemPath::isWithinDistance(sb_geom_msgs::Point2D p1,
                                          sb_geom_msgs::Point2D p2,
                                          float dist_tol_sq) {
    return (pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) < dist_tol_sq);
}
