/*
 * Created By: Gareth Ellis
 * Created On: July 4, 2017
 * Description: TODO
 */

#include <LidarObstacleManager.h>
#include <cmath>
#include <stack>

// TODO: We should probably get rid of the default constructor here.....
LidarObstacleManager::LidarObstacleManager()
  : max_obstacle_merging_distance(0.3), cone_grouping_tolerance(1.3){};

LidarObstacleManager::LidarObstacleManager(
double max_obstacle_merging_distance,
double max_distance_from_robot_accepted,
double cone_grouping_tolerance,
double min_wall_length,
double collision_distance,

double front_angle,
double side_angle_max,
double side_angle_min,
double region_fill_percentage,
bool front_collision_only)
  : max_obstacle_merging_distance(max_obstacle_merging_distance),
    max_distance_from_robot_accepted(max_distance_from_robot_accepted),
    cone_grouping_tolerance(cone_grouping_tolerance),
    min_wall_length(min_wall_length),
    collision_distance(collision_distance),
    front_angle(front_angle),
    side_angle_max(side_angle_max),
    side_angle_min(side_angle_min),
    region_fill_percentage(region_fill_percentage),
    front_collision_only(front_collision_only) {}

void LidarObstacleManager::addLaserScan(const sensor_msgs::LaserScan& scan) {
    // Number of hits for each region
    int left_side_hits  = 0;
    int right_side_hits = 0;
    int front_side_hits = 0;

    // Create an obstacle for every hit in the lidar scan
    for (int i = 0; i < scan.ranges.size(); ++i) {
        // Check that the lidar hit is within acceptable bounds
        double angle = scan.angle_min + i * scan.angle_increment;
        double range = scan.ranges[i];
        if (range < scan.range_max && range > scan.range_min) {
            addObstacle(LidarObstacle(min_wall_length, angle, range));

            double abs_angle = std::abs(angle);
            if (abs_angle < side_angle_max && abs_angle > side_angle_min) {
                if (range < collision_distance * 2) {
                    if (angle > 0)
                        left_side_hits++;
                    else
                        right_side_hits++;
                }
            }

            if (abs_angle < front_angle / 2) {
                if (range < collision_distance) front_side_hits++;
            }
        }
    }

    int side_region_total_size =
    (side_angle_max - side_angle_min) / scan.angle_increment;

    int front_region_total_size = front_angle / scan.angle_increment;

    if (front_collision_only) {
        collision_detected =
        (front_side_hits > (front_region_total_size * region_fill_percentage));
    } else {
        collision_detected =
        (left_side_hits > (side_region_total_size * region_fill_percentage)) &&
        (right_side_hits > (side_region_total_size * region_fill_percentage)) &&
        (front_side_hits > (front_region_total_size * region_fill_percentage));
    }
}

std::vector<LidarObstacle> LidarObstacleManager::getObstacles() {
    return obstacles;
}

void LidarObstacleManager::clearObstacles() {
    obstacles.clear();
}

void LidarObstacleManager::addObstacle(LidarObstacle obstacle) {
    // See if this obstacle is close enough to any other saved obstacle to be
    // the same
    // TODO: Should we be instead checking for the CLOSEST saved obstcle?
    for (LidarObstacle& saved_obstacle : obstacles) {
        if (minDistanceBetweenObstacles(saved_obstacle, obstacle) <
            max_obstacle_merging_distance) {
            saved_obstacle.mergeInLidarObstacle(obstacle);
            return;
        }
    }
    obstacles.emplace_back(obstacle);
}

double
LidarObstacleManager::minDistanceBetweenObstacles(LidarObstacle obstacle1,
                                                  LidarObstacle obstacle2) {
    // TODO: This is ABSURDLY ineffecient. We're doing n^2 operations on 2
    // potentially
    // TODO: very large objects. We should be able to do some sort of obstacle
    // approx to
    // TODO: circles or boxes and compare them that way (need to update
    // LidarObstacle for that approx)

    std::vector<Point> obstacle1_points = obstacle1.getReadingsAsPoints();
    std::vector<Point> obstacle2_points = obstacle2.getReadingsAsPoints();

    // Compare every point to.... *shudders slightly* every other point..
    double min_distance = -1;
    for (Point p1 : obstacle1_points) {
        for (Point p2 : obstacle2_points) {
            double dx       = p1.x - p2.x;
            double dy       = p1.y - p2.y;
            double distance = std::sqrt(std::pow(dx, 2.0) + std::pow(dy, 2.0));
            if (min_distance < 0 || distance < min_distance)
                min_distance = distance;
        };
    };

    return min_distance;
}

std::vector<LineOfBestFit> LidarObstacleManager::getConeLines() {
    // Get all our cones as points
    std::vector<Point> points;
    for (LidarObstacle obstacle : obstacles) {
        if (obstacle.getObstacleType() == CONE) {
            points.emplace_back(obstacle.getCenter());
        }
    }

    // Get groups of lines
    std::vector<std::vector<Point>> groups =
    getPointGroupings(points, cone_grouping_tolerance);

    // Fit a line of best fit to each group
    std::vector<LineOfBestFit> lines;
    for (std::vector<Point> group : groups)
        if (group.size() >= 2) lines.emplace_back(getLineOfBestFit(group));

    return lines;
}

std::vector<std::vector<Point>>
LidarObstacleManager::getPointGroupings(std::vector<Point> points,
                                        double tolerance) {
    std::vector<std::vector<Point>> groups;

    // Go through every point
    while (points.size() > 0) {
        // Start the current group off with the last point
        std::vector<Point> group;
        std::stack<Point> to_visit;
        to_visit.emplace(points.back());
        points.pop_back();
        while (to_visit.size() > 0) {
            // Visit the first point in to_visit
            Point curr_point = to_visit.top();
            to_visit.pop();

            // Figure out if there are any points within tolerance of the point
            // we're visiting
            for (int i = 0; i < points.size(); i++) {
                Point p = points[i];
                if (distanceBetweenPoints(curr_point, p) < tolerance) {
                    // Add point to to_visit and remove from the given list of
                    // points
                    to_visit.emplace(p);
                    points.erase(points.begin() + i);
                }
            }

            // Add the current point to the group
            group.emplace_back(curr_point);

            // Keep going if we've got more points to visit
        }

        groups.emplace_back(group);
    }

    return groups;
}

LineOfBestFit
LidarObstacleManager::getLineOfBestFit(const std::vector<Point>& points) {
    // Get line of best fit using linear regression formula
    // http://www.statisticshowto.com/how-to-find-a-linear-regression-equation/

    // TODO: Handle this more nicely
    // WE SHOULD NEVER GET HERE
    BOOST_ASSERT(points.size() >= 2);

    double x_sum = std::accumulate(
    points.begin(), points.end(), 0.0, [](double accum, Point p) {
        return accum + p.x;
    });
    double y_sum = std::accumulate(
    points.begin(), points.end(), 0.0, [](double accum, Point p) {
        return accum + p.y;
    });
    double x_squared_sum = std::accumulate(
    points.begin(), points.end(), 0.0, [](double accum, Point p) {
        return accum + std::pow(p.x, 2.0);
    });
    double x_y_product_sum = std::accumulate(
    points.begin(), points.end(), 0.0, [](double accum, Point p) {
        return accum + p.x * p.y;
    });
    double y_intercept = (y_sum * x_squared_sum - x_sum * x_y_product_sum) /
                         (points.size() * x_squared_sum - std::pow(x_sum, 2.0));
    double slope = (points.size() * x_y_product_sum - x_sum * y_sum) /
                   (points.size() * x_squared_sum - std::pow(x_sum, 2.0));

    // Calculate means
    double x_mean = x_sum / points.size();
    double y_mean = y_sum / points.size();

    // Calculate Variances
    double x_var =
    std::accumulate(points.begin(),
                    points.end(),
                    0.0,
                    [x_mean](double accum, Point p) {
                        return accum + std::pow((x_mean - p.x), 2.0);
                    }) /
    (points.size() - 1);
    double y_var =
    std::accumulate(points.begin(),
                    points.end(),
                    0.0,
                    [y_mean](double accum, Point p) {
                        return accum + std::pow((y_mean - p.y), 2.0);
                    }) /
    (points.size() - 1);
    // Calculate standard deviations
    double x_sd = sqrt(x_var);
    double y_sd = sqrt(y_var);

    // Calculate correlation coefficient using Slope = r*Sy/Sx
    double r = slope * x_sd / y_sd;

    return LineOfBestFit(slope, y_intercept, r);
}

LineOfBestFit LidarObstacleManager::getBestLine(bool lineToTheRight) {
    LineOfBestFit bestLine(0, 0, 0);

    std::vector<LineOfBestFit> lines = getConeLines();

    for (size_t i = 0; i < lines.size(); i++) {
        // Only check lines where the y-intercept is on the correct side.
        if ((lineToTheRight && lines[i].getYIntercept() < 0) ||
            (!lineToTheRight && lines[i].getYIntercept() >= 0)) {
            // Don't accept lines that are too far from the robot.
            if (fabs(lines[i].getYIntercept()) <=
                max_distance_from_robot_accepted) {
                // If correlation is stronger than the current best, update best
                // line.
                if (fabs(lines[i].correlation) > fabs(bestLine.correlation))
                    bestLine = lines[i];
            }
        }
    }
    return bestLine;
}

visualization_msgs::Marker LidarObstacleManager::getConeRVizMarker() {
    visualization_msgs::Marker points;

    // TODO: we shold differenetiat between walls and cones

    // TODO: Should be a param (currently in the default LaserScan frame)
    points.header.frame_id    = "laser";
    points.header.stamp       = ros::Time::now();
    points.ns                 = "debug";
    points.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id                 = 0;
    // TODO: Make these round, not blocks
    points.type = visualization_msgs::Marker::POINTS;
    // TODO: Make the scale a param
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    // Points are green
    // TODO: Make this a param
    points.color.g = 1.0f;
    points.color.a = 1.0;

    for (LidarObstacle obstacle : obstacles) {
        if (obstacle.getObstacleType() == CONE) {
            Point center = obstacle.getCenter();
            geometry_msgs::Point geom_point;
            geom_point.x = center.x;
            geom_point.y = center.y;
            points.points.push_back(geom_point);
        }
    }

    return points;
}

visualization_msgs::Marker LidarObstacleManager::getConeLinesRVizMarker() {
    visualization_msgs::Marker lines;

    // TODO: Should be a param (currently in the default LaserScan frame)
    lines.header.frame_id    = "laser";
    lines.header.stamp       = ros::Time::now();
    lines.ns                 = "debug";
    lines.action             = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.id                 = 0;
    lines.type               = visualization_msgs::Marker::LINE_LIST;
    lines.action             = visualization_msgs::Marker::ADD;
    // TODO: Make the scale a param
    lines.scale.x = 0.1;
    lines.scale.y = 0.1;
    // TODO: Make this a param
    lines.color.r = 1.0f;
    lines.color.a = 1.0;

    // Get the cone lines
    std::vector<LineOfBestFit> cone_lines = getConeLines();
    for (int i = 0; i < cone_lines.size(); i++) {
        // Get two points to represent the line
        geometry_msgs::Point p1, p2;
        p1.x = -10;
        p1.y = cone_lines[i].getYCoorAtX(p1.x);
        // TODO: make length here a param
        p2.x = 10;
        p2.y = cone_lines[i].getYCoorAtX(p2.x);
        lines.points.push_back(p1);
        lines.points.push_back(p2);
    }
    return lines;
}

// TODO: See comments in above visualisation functions
visualization_msgs::Marker
LidarObstacleManager::getBestConeLineRVizMarker(bool line_to_the_right) {
    // We'll just put a single line in this
    visualization_msgs::Marker line;

    // TODO: Should be a param (currently in the default LaserScan frame)
    line.header.frame_id    = "laser";
    line.header.stamp       = ros::Time::now();
    line.ns                 = "debug";
    line.action             = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.id                 = 0;
    line.type               = visualization_msgs::Marker::LINE_LIST;
    line.action             = visualization_msgs::Marker::ADD;
    // TODO: Make the scale a param
    line.scale.x = 0.1;
    line.scale.y = 0.1;
    // TODO: Make this a param
    line.color.b = 1.0f;
    line.color.a = 1.0;

    // TODO: Find a better way to sync this logic up. Copy pasting sucks
    LineOfBestFit best_line = getBestLine(line_to_the_right);
    // If no line found on desired side
    if (best_line.correlation == 0) best_line = getBestLine(!line_to_the_right);
    geometry_msgs::Point p1, p2;
    p1.x = -10;
    p1.y = best_line.getYCoorAtX(p1.x);
    p2.x = 10;
    p2.y = best_line.getYCoorAtX(p2.x);
    line.points.push_back(p1);
    line.points.push_back(p2);

    return line;
}

bool LidarObstacleManager::collisionDetected() {
    return collision_detected;
}
