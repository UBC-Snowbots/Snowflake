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
                                 double occ_grid_cell_size) :
cone_merging_tolerance(cone_merging_tolerance),
line_merging_tolerance(line_merging_tolerance),
obstacle_inflation_buffer(obstacle_inflation_buffer),
occ_grid_cell_size(occ_grid_cell_size)
{}

std::vector<mapping_igvc::ConeObstacle> ObstacleManager::getConeObstacles() {
    return cones;
}

std::vector<sb_geom::Spline> ObstacleManager::getLineObstacles() {
    return lines;
}

void ObstacleManager::addObstacle(mapping_igvc::ConeObstacle cone) {


    // Find the distance to every known cone from this one
    std::vector<std::pair<double,int>> distances;
    for (int known_cone_index = 0; known_cone_index < cones.size(); known_cone_index++){
        mapping_igvc::ConeObstacle& known_cone = cones[known_cone_index];
        double dx = cone.center.x - known_cone.center.x;
        double dy = cone.center.y - known_cone.center.y;
        double distance = std::sqrt(std::pow(dx,2) + std::pow(dy,2));
        distances.emplace_back(std::make_pair(distance, known_cone_index));
    }

    // TODO: Consider using the radius as further criteria for merging

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

// TODO: The `addObstacle` functions seem pretty similar.... maybe we could apply DRY here?
void ObstacleManager::addObstacle(Spline line_obstacle) {

    // TODO: maybe prune obstacles outside a given distance from us? (would likely improve performance of this function)

    // Find the distance from this line_obstacle to every other known line_obstacle
    // (in parallel)
    std::vector<std::pair<double, int>> distances(lines.size());
    #pragma omp parallel for
    for (int line_index = 0; line_index < lines.size(); line_index++) {
        // TODO: `max_iters` should be a class member we can change (of `ObstacleManager`)
        double distance = minDistanceBetweenSplines(lines[line_index], line_obstacle, 15);
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
            Spline merged_line = updateLineWithNewLine(lines[closest_known_line_index], line_obstacle);
            // Overwrite the old line_obstacle with our new merged line_obstacle
            lines[closest_known_line_index] = merged_line;
        } else {
            // The closest line_obstacle to this one isn't close enough to merge into,
            // so just add this line_obstacle as a new line_obstacle
            lines.emplace_back(Spline(line_obstacle));
        }
    } else {
        // We don't know about any lines yet, so just add this one as it's own line_obstacle
        lines.emplace_back(Spline(line_obstacle));
    }
}

Spline ObstacleManager::updateLineWithNewLine(Spline current_line,
                                              Spline new_line) {

    // Find the closest points on the known line to the start and end of the new line
    // TODO: `max_iters` should be a class member we can change (of `ObstacleManager`)
    double closest_point_to_start =
            findClosestPointOnSplineToPoint(current_line, new_line(0), 10);
    double closest_point_to_end =
            findClosestPointOnSplineToPoint(current_line, new_line(1), 10);
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

    // TODO: maybe prune obstacles outside a given distance from us? (might want a seperate function for this)

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

    // TODO: Set header

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

        // Inflate the obstacles (and thus mark sections of the grid as occupied)
        // TODO: If this is slow, we may want to more intelligently inflate lines and cones
        for (auto& point: occupied_points){
            inflatePoint(occ_grid, point, obstacle_inflation_buffer);
        }
    }

    return occ_grid;
}

void ObstacleManager::inflatePoint(nav_msgs::OccupancyGrid &occ_grid, sb_geom::Point2D point, double inflation_radius) {
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
                occ_grid.data[y * occ_grid.info.width + x] = 100;
            }
        }
    }
}

