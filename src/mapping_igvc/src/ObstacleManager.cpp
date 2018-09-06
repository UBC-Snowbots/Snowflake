/*
 * Created By: Gareth Ellis
 * Created On: January 9, 2018
 * Description: The Obstacle Manager takes in discrete obstacles and saves them,
 *              comparing them to newly received obstacles and checking if
 *              they're similar. If they are, it will merge/move known obstacles
 *              using the new obstacles as updated information
 */

// Snowbots Includes
#include <ObstacleManager.h>
#include <sb_geom/utils.h>

// STD Includes
#include <cmath>
#include <algorithm>
#include <tuple>

// ROS Includes
#include <std_msgs/Int8.h>
#include <tf/transform_datatypes.h>

using namespace sb_geom;

ObstacleManager::ObstacleManager(double cone_merging_tolerance,
                                 double line_merging_tolerance,
                                 double obstacle_inflation_buffer,
                                 double occ_grid_cell_size,
                                 unsigned int line_merging_max_iters,
                                 unsigned int closest_line_max_iters,
                                 double exp_coefficient,
                                 double occupied_radius
) :
cone_merging_tolerance(cone_merging_tolerance),
line_merging_tolerance(line_merging_tolerance),
obstacle_inflation_buffer(obstacle_inflation_buffer),
occ_grid_cell_size(occ_grid_cell_size),
spline_merging_max_iters(line_merging_max_iters),
closest_spline_max_iters(closest_line_max_iters),
exp_coefficient(exp_coefficient),
occupied_radius(occupied_radius)
{}

std::vector<mapping_igvc::ConeObstacle> ObstacleManager::getConeObstacles() {
    return cones;
}

std::vector<sb_geom::Spline> ObstacleManager::getLineObstacles() {
    return lines;
}

double ObstacleManager::distanceBetweenCones(mapping_igvc::ConeObstacle cone1, mapping_igvc::ConeObstacle cone2){
    return distance(cone1.center, cone2.center);
}


void ObstacleManager::addObstacle(mapping_igvc::ConeObstacle cone) {

    // Find the distance to every known cone from this one
    std::vector<std::pair<double,int>> distances;
    for (int known_cone_index = 0; known_cone_index < cones.size(); known_cone_index++){
        mapping_igvc::ConeObstacle& known_cone = cones[known_cone_index];
        double distance = distanceBetweenCones(cone, known_cone);
        distances.emplace_back(std::make_pair(distance, known_cone_index));
    }

    // NOTE: In the future we could use the cone radius as
    // further criteria for merging

    // Find the closest known cone (min element by distance)
    auto min_element = std::min_element(distances.begin(), distances.end(),
                               [&](auto pair1, auto pair2){return pair1.first < pair2.first;}
                        );

    // Make sure that we were at least able to find one cone
    if (min_element != distances.end()){
        // Get the distance and cone from the iterator
        double distance_to_closest_known_cone = min_element->first;
        int closest_known_cone_index = min_element->second;

        if (distance_to_closest_known_cone < cone_merging_tolerance){
            // Overwrite the known cone with our updated estimate of the cone
            cones[closest_known_cone_index] = cone;
        } else {
            // The closest cone to this one isn't close enough to merge into,
            // so just add this cone as a new cone
            cones.emplace_back(cone);
        }
    } else {
        // We don't know about any cones yet, so just add this one
        cones.emplace_back(cone);
    }
}

void ObstacleManager::addObstacle(mapping_igvc::LineObstacle line_obstacle) {
    // Convert the polynomial to a spline
    sb_geom::PolynomialSegment poly_segment(line_obstacle.coefficients, line_obstacle.min, line_obstacle.max);
    sb_geom::Spline spline(poly_segment);

    addObstacle(spline);
}

void ObstacleManager::addObstacle(Spline spline) {

    // Find the distance from this line_obstacle to every other known line_obstacle
    // (in parallel)
    std::vector<std::pair<double, int>> distances(lines.size());
    #pragma omp parallel for
    for (int line_index = 0; line_index < lines.size(); line_index++) {
        double distance = minDistanceBetweenSplines(lines[line_index], spline, closest_spline_max_iters);
        distances[line_index] = std::make_pair(distance, line_index);
    }

    // Find the closest known line_obstacle (min element by distance)
    auto min_element = std::min_element(distances.begin(), distances.end(),
                                        [&](auto pair1, auto pair2){
                                            return pair1.first < pair2.first;
                                        });

    // Make sure that we were at least able to find one line_obstacle
    if (min_element != distances.end()){
        // Get the distance and line_obstacle from the iterator
        double distance_to_closest_known_line = min_element->first;
        int closest_known_line_index = min_element->second;

        if (distance_to_closest_known_line < line_merging_tolerance){
            // Update our current knowledge of the line_obstacle based on the new line_obstacle
            Spline merged_line = updateLineWithNewLine(lines[closest_known_line_index], spline);
            // Overwrite the old line_obstacle with our new merged line_obstacle
            lines[closest_known_line_index] = merged_line;
        } else {
            // The closest line_obstacle to this one isn't close enough to merge into,
            // so just add this line_obstacle as a new line_obstacle
            lines.emplace_back(Spline(spline));
        }
    } else {
        // We don't know about any lines yet, so just add this one as it's own line_obstacle
        lines.emplace_back(Spline(spline));
    }
}

Spline ObstacleManager::updateLineWithNewLine(Spline current_line,
                                              Spline new_line) {

    // Find the closest points on the known line to the start and end of the new line
    double closest_point_to_start =
            findClosestPointOnSplineToPoint(current_line, new_line(0), spline_merging_max_iters);
    double closest_point_to_end =
            findClosestPointOnSplineToPoint(current_line, new_line(1), spline_merging_max_iters);
    double min_u = std::min(closest_point_to_start, closest_point_to_end);
    double max_u = std::max(closest_point_to_start, closest_point_to_end);

    // "replace" the section of the current line between `u1` and `u2` with the new line

    // Find the points "before" the new line (on the current line)
    std::vector<Point2D> points_before_new_line = {};
    // This catches the edge case where the closest point to the endpoint of the new spline
    // is the endpoint of the currently known spline. In this case there are *NO* points on
    // the known spline that we want in the merged spline, but `getInterpolationPointsInRange`
    // is inclusive, and so will return the endpoint when given the range: [0,0]
    if (min_u != 0){
        points_before_new_line = current_line.getInterpolationPointsInRange(0, min_u);
    }

    // Find the points "after" the new line (on the current line)
    std::vector<Point2D> points_after_new_line = {};
    // This catches the edge case where the closest point to the endpoint of the new spline
    // is the endpoint of the currently known spline. In this case there are *NO* points on
    // the known spline that we want in the merged spline, but `getInterpolationPointsInRange`
    // is inclusive, and so will return the endpoint when given the range: [1,1]
    if (max_u != 1){
        points_after_new_line = current_line.getInterpolationPointsInRange(max_u, 1);
    }

    // Get the interpolation points from the new line
    std::vector<Point2D> points_on_new_line = new_line.getInterpolationPointsInRange(0,1);

    // Check if the points of the new line are in the "opposite" order of those
    // in the known line. That is, is the closest point to the start of the new
    // line after the closest point to the end of the new line on the old line?
    if (closest_point_to_start > closest_point_to_end){
        std::reverse(points_on_new_line.begin(), points_on_new_line.end());
    }

    // Merge the 3 sets of interpolation points in order:
    // - points on the current line before the new line
    // - points on the new line
    // - points on the current line after the new line
    std::vector<Point2D> new_interpolation_points = points_before_new_line;
    new_interpolation_points.insert(
            new_interpolation_points.end(), points_on_new_line.begin(), points_on_new_line.end());
    new_interpolation_points.insert(
            new_interpolation_points.end(), points_after_new_line.begin(), points_after_new_line.end());

    // Return a spline interpolated through our new points
    return Spline(new_interpolation_points);
}

nav_msgs::OccupancyGrid ObstacleManager::generateOccupancyGrid() {
    // TODO: All this cleanup stuff should probably be in it's own function

    // TODO: Make this angle a param
    // TODO: Is this the right place to do this? (keep in mind how expensive it is)
    // splitLineSelfLoops(0.47);
    // TODO: Is this the right place to do this? (keep in mind how expensive it is)
    mergeCloseLines();
    mergeCloseCones();

    // Find what cells are directly occupied (ie. overlapping) with
    // known obstacles
    std::vector<Point2D> occupied_points;

    for (auto& line : lines){
        // Figure out how many points to sample
        auto num_sample_points = (int)std::ceil(line.approxLength() / occ_grid_cell_size) * 2;

        // Sample points from the line
        for (int i = 0; i <= num_sample_points; i++){
            occupied_points.emplace_back(line(i / (double)num_sample_points));
        }
    }

    for (auto& cone : cones){
        // Iterate over the cone in 2D
        auto cone_radius_num_cells = (int)std::ceil(cone.radius / occ_grid_cell_size);
        for (int x = -cone_radius_num_cells; x <= cone_radius_num_cells; x++){
            for (int y = -cone_radius_num_cells; y <= cone_radius_num_cells; y++) {
                // Check that the point is within the circle by using the equation of
                // a circle: (x^2 + y^2 = r^2)
                if ((std::pow(x, 2) + std::pow(y,2)) <= std::pow(cone_radius_num_cells,2)){
                    occupied_points.emplace_back(Point2D(
                            cone.center.x + x * occ_grid_cell_size,
                            cone.center.y + y * occ_grid_cell_size
                    ));
                }
            }
        }
    }

    nav_msgs::OccupancyGrid occ_grid;

    // Set MapMetaData
    occ_grid.info.map_load_time = ros::Time(0);
    occ_grid.info.resolution = (float)occ_grid_cell_size;
    // Initially, set all values to 0
    occ_grid.info.width = 0;
    occ_grid.info.height = 0;
    occ_grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
    occ_grid.info.origin.position.x = 0;
    occ_grid.info.origin.position.y = 0;

    // If we have occupied points, then expand and fill the occupancy
    // grid accordingly
    if (!occupied_points.empty()){
        // Find the max/min x and y values in the occupied points
        // these will define the extents of the generate occupancy grid
        // We also add a buffer around the min/max to account for edge points
        // being inflated
        Point2D point_max_x = *std::max_element(occupied_points.begin(), occupied_points.end(),
                                                [&](Point2D p1, Point2D p2){
                                                    return p1.x() < p2.x();
                                                });
        double max_x = point_max_x.x() + obstacle_inflation_buffer;
        Point2D point_max_y = *std::max_element(occupied_points.begin(), occupied_points.end(),
                                                [&](Point2D p1, Point2D p2){
                                                    return p1.y() < p2.y();
                                                });
        double max_y = point_max_y.y() + obstacle_inflation_buffer;
        Point2D point_min_x = *std::min_element(occupied_points.begin(), occupied_points.end(),
                                                [&](Point2D p1, Point2D p2){
                                                    return p1.x() < p2.x();
                                                });
        double min_x = point_min_x.x() - obstacle_inflation_buffer;
        Point2D point_min_y = *std::min_element(occupied_points.begin(), occupied_points.end(),
                                                [&](Point2D p1, Point2D p2){
                                                    return p1.y() < p2.y();
                                                });
        double min_y = point_min_y.y() - obstacle_inflation_buffer;


        // Convert our min/max extents into a width and height in # of cells
        occ_grid.info.width = (unsigned int)std::ceil((max_x - min_x) / occ_grid_cell_size)+1;
        occ_grid.info.height = (unsigned int)std::ceil((max_y - min_y) / occ_grid_cell_size)+1;
        occ_grid.info.origin.position.x = min_x;
        occ_grid.info.origin.position.y = min_y;

        // Populate the occupancy grid with all 0 initially (unoccupied)
        occ_grid.data = std::vector<int8_t>(occ_grid.info.width * occ_grid.info.height);
        std::fill(occ_grid.data.begin(), occ_grid.data.end(), 0);

        // NOTE: If there are performance issues, more intelligently inflating
        // obstacles here would be a good place to start

        // Inflate the obstacles (and thus mark sections of the grid as occupied)
        for (auto& point: occupied_points){
            inflatePoint(occ_grid, point, obstacle_inflation_buffer, exp_coefficient, occupied_radius);
        }
    }

    return occ_grid;
}

void ObstacleManager::inflatePoint(nav_msgs::OccupancyGrid &occ_grid, sb_geom::Point2D point, double inflation_radius, double exp_coefficient, double occupied_radius) {
    // Figure out how many cells the inflation radius corresponds to
    auto inflation_radius_num_of_cells = (int)std::ceil(inflation_radius / occ_grid.info.resolution);

    // This function returns the `y` value (in # of cells) of the inflation circle for a given
    // `x` value (also in # of cells) using the equation for a circle: `y = sqrt(r^2 - x^2)`
    auto inflationCircle = [&](int x) {
        // Calculate `y` as a floating point distance
        double y = std::sqrt(std::pow(inflation_radius_num_of_cells,2) - std::pow(x,2));
        // Return the equivalent number of cells
        return (int)std::floor(y);
    };

    // Find the closest cell to the given point
    // Note: this cell may not be on the graph if the point isn't, but we catch
    //       this case by checking that a given cell is on the graph before setting
    //       it in the `for` loops below
    double graph_rotation = tf::getYaw(occ_grid.info.origin.orientation);

    // Translate and rotate the given point to the frame of the occupancy grid
    Point2D translated_point(
            point.x() - occ_grid.info.origin.position.x,
            point.y() - occ_grid.info.origin.position.y
    );
    Point2D translated_and_rotated_point(
            cos(graph_rotation) * translated_point.x() - sin(graph_rotation) * translated_point.y(),
            sin(graph_rotation) * translated_point.x() + cos(graph_rotation) * translated_point.y()
    );

    // Round the point to the nearest cell
    auto center_cell_x = (int)std::round(translated_and_rotated_point.x() / occ_grid.info.resolution);
    auto center_cell_y = (int)std::round(translated_and_rotated_point.y() / occ_grid.info.resolution);

    // Iterate over the area this point is to be inflated to,
    // marking all the cells as "occupied"
    int min_x = center_cell_x - inflation_radius_num_of_cells;
    int max_x = center_cell_x + inflation_radius_num_of_cells;
    for (int x = min_x; x <= max_x; x++){
        int min_y = center_cell_y - inflationCircle(center_cell_x - x);
        int max_y = center_cell_y + inflationCircle(center_cell_x - x);
        for (int y = min_y; y <= max_y; y++){
            // Check that (x,y) is actually on the grid
            if (y >= 0 && y < occ_grid.info.height &&
                    x >= 0 && x < occ_grid.info.width){
                // Calculate the weight of the cell in a linear gradient from
                // 100->0 from the center to the edge of the inflation radius
                int dy = center_cell_y - y;
                int dx = center_cell_x - x ;
                
                double distance = std::sqrt(std::pow(dy,2) + std::pow(dx,2));
                signed char cell_weight;
                if ( distance < occupied_radius)
                    cell_weight = 100;
                else {
                    cell_weight = (int8_t) (100 * exp(-exp_coefficient * (distance - occupied_radius))) ;
                }
                // To account for overlaps, we only want to increase the cell weight
                signed char curr_cell_weight = occ_grid.data[y * occ_grid.info.width + x];

                occ_grid.data[y * occ_grid.info.width + x] = std::max(curr_cell_weight, cell_weight);
            }
        }
    }
}

void ObstacleManager::mergeCloseLines() {
    // Compare every line to every other line
    for (int i = 0; i < lines.size(); i++){
        for (int j = i+1; j < lines.size(); j++){
            // Check if the two lines are too close together
            if (minDistanceBetweenSplines(lines[i], lines[j], closest_spline_max_iters) < line_merging_tolerance){
                // Remove the second of the two lines
                Spline line = lines[j];
                lines.erase(lines.begin()+j);
                // Merge it back into the first
                lines[i] = updateLineWithNewLine(lines[i], line);
                // Restart merging
                i = 0;
                j = 0;
                break;
            }
        }
    }
}

void ObstacleManager::mergeCloseCones() {
    // Compare every cone to every other cone
    for (int i = 0; i < cones.size(); i++){
        for (int j = i+1; j < cones.size(); j++){
            // Check if the two cones are too close together
            if (distanceBetweenCones(cones[i], cones[j]) < cone_merging_tolerance){
                // Remove the older of the two cones
                if (cones[i].header.stamp < cones[j].header.stamp){
                    cones.erase(cones.begin() + i);
                } else {
                    cones.erase(cones.begin() + j);
                }
                // Restart merging
                i = 0;
                j = 0;
                break;
            }
        }
    }
}

void ObstacleManager::splitLineSelfLoops(double self_loop_max_angle) {
    for (int i = 0; i < lines.size(); i++){
        std::vector<sb_geom::Point2D> points = lines[i].getInterpolationPoints();
        // Check if any 3 consecutive points form an angle < `self_loop_max_angle`
        for (int j = 1; j < points.size()-1; j++){
            double angle = interiorAngle(points[j-1], points[j], points[j+1]);
            if (angle <= self_loop_max_angle){
                // Remove the offending line
                Spline line_to_split = lines[i];
                lines.erase(lines.begin()+i);

                // Split it into two lines at the point where the excessive angle occured
                std::vector<Point2D> interpolation_points = line_to_split.getInterpolationPoints();
                auto start = interpolation_points.begin();
                auto split = interpolation_points.begin() + j;
                auto end = interpolation_points.end();
                // Note: we include the split point in both splines
                std::vector<Point2D> s1_points(start, split+1);
                std::vector<Point2D> s2_points(split, end);
                lines.emplace_back(Spline(s1_points));
                lines.emplace_back(Spline(s2_points));

                // TODO: We don't need to go through all lines again, we can do something smarter here..
                i = 0;
                break;
            }
        }
    }
}

void ObstacleManager::pruneObstaclesOutsideCircle(sb_geom::Point2D center, double radius) {
    // TODO: Make cones and lines seperate functions here

    // Remove cones
    for (int i = 0; i < cones.size(); i++){
        if (distance(cones[i].center, center) > radius){
            cones.erase(cones.begin() + i);
        }
    }

    // Remove Lines
    for (int i = 0; i < lines.size(); i++){
        // Check for interpolation points outside of the radius to remove
        std::vector<sb_geom::Point2D> interpolation_points = lines[i].getInterpolationPoints();
        for (int j = 0; j < interpolation_points.size(); j++) {
            sb_geom::Point2D point = interpolation_points[j];
            if (distance(point, center) > radius){
                // Remove the line
                sb_geom::Spline spline = lines[i];
                lines.erase(lines.begin() + i);

                // Create two splines from the points before and after
                // the interpolation point outside the radius
                // (not including said interpolation point)
                auto start = interpolation_points.begin();
                auto split = interpolation_points.begin() + j;
                auto end = interpolation_points.end();
                // Note: we do NOT include the split point in either spline
                std::vector<Point2D> s1_points(start, split);
                std::vector<Point2D> s2_points(split+1, end);

                // Only add splines with more then one point
                if (s1_points.size() > 1){
                    lines.emplace_back(Spline(s1_points));
                }
                if (s2_points.size() > 1){
                    lines.emplace_back(Spline(s2_points));
                }

                // Decrement the line index counter since we just removed the line at `i` and
                // so the line that was at `i+1` is now at `i`
                i--;
                break;

            }
        }
    }

}


