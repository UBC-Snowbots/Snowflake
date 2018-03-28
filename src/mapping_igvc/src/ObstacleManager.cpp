/*
 * Created By: Gareth Ellis
 * Created On: January 9, 2018
 * Description: TODO
 */

// Snowbots Includes
#include <ObstacleManager.h>
#include <sb_geom/utils.h>

// STD Includes
#include <cmath>
#include <algorithm>
#include <tuple>

using namespace sb_geom;

ObstacleManager::ObstacleManager(double cone_merging_tolerance, double line_merging_tolerance) :
cone_merging_tolerance(cone_merging_tolerance),
line_merging_tolerance(line_merging_tolerance)
{}

std::vector<Cone> ObstacleManager::getConeObstacles() {
    return cones;
}

std::vector<sb_geom::Spline> ObstacleManager::getLineObstacles() {
    return lines;
}

void ObstacleManager::addObstacle(Cone cone) {
    // Find the distance to every known cone from this one
    std::vector<std::pair<double,int>> distances;
    for (int known_cone_index = 0; known_cone_index < cones.size(); known_cone_index++){
        Cone& known_cone = cones[known_cone_index];
        double dx = cone.x - known_cone.x;
        double dy = cone.y - known_cone.y;
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
    // Find the distance from this line_obstacle to every other known line_obstacle
    std::vector<std::pair<double, int>> distances;
    for (int line_index = 0; line_index < lines.size(); line_index++) {
        // TODO: `max_iters` should be a class member we can change (of `ObstacleManager`)
        double distance = minDistanceBetweenSplines(lines[line_index], line_obstacle, 20);
        distances.emplace_back(std::make_pair(distance, line_index));
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



