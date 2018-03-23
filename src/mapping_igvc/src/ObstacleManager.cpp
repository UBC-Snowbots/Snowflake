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

std::vector<sb_geom::Spline> ObstacleManager::getLineObstacle() {
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
void ObstacleManager::addObstacle(PolynomialSegment line) {
    // Find the distance from this line to every other known line
    std::vector<std::pair<double, int>> distances;
    for (int line_index = 0; line_index < lines.size(); line_index++) {
        double distance = distanceBetweenLines(lines[line_index], line, 0, 0);
        distances.emplace_back(std::make_pair(distance, line_index));
    }

    // Find the closest known line (min element by distance)
    auto min_element = std::min_element(distances.begin(), distances.end(),
                                        [&](auto pair1, auto pair2){
                                            return pair1.first < pair2.first;
                                        });

    // Make sure that we were at least able to find one line
    if (min_element != distances.end()){
        // Get the distance and line from the iterator
        double distance_to_closest_known_line = min_element->first;
        int closest_known_line_index = min_element->second;

        if (distance_to_closest_known_line < line_merging_tolerance){
            // Update our current knowledge of the line based on the new line
            Spline merged_line = updateLineWithNewLine(lines[closest_known_line_index], line);
            // Overwrite the old line with our new merged line
            lines[closest_known_line_index] = merged_line;
        } else {
            // The closest line to this one isn't close enough to merge into,
            // so just add this line as a new line
            lines.emplace_back(Spline(line));
        }
    } else {
        // We don't know about any lines yet, so just add this one as it's own line
        lines.emplace_back(Spline(line));
    }
}

double ObstacleManager::distanceBetweenLines(sb_geom::Spline spline,
                                             PolynomialSegment poly_line,
                                             unsigned int num_sample_points,
                                             double max_err) {
    // The start and end of the section of spline we're sampling
    // Initially just choose the entire spline
    double u1 = 0;
    double u2 = 1;
    double distance_to_u1 = minDistanceFromPointToPolynomialSegment(
            spline(u1), poly_line);
    double distance_to_u2 = minDistanceFromPointToPolynomialSegment(
            spline(u2), poly_line);

    do {
        // Calculate how much to step each time, depending on the length
        // of spline we're sampling
        double len_of_sub_spline = std::max(u1, u2) - std::min(u1, u2);
        double u_step = len_of_sub_spline/num_sample_points;

        for (int i = 0; i < num_sample_points; i++){
            double curr_u = i * u_step;
            // Find the distance to the polynomial from the sample point on the spline
            Point2D curr_p = spline(i * 1/num_sample_points);
            double dist_to_p = minDistanceFromPointToPolynomialSegment(curr_p, poly_line);

            // Check if this point is better then at least one of the points we have
            if (dist_to_p < std::max(distance_to_u1, distance_to_u2)){
                // If it is better, overwrite the furthest currently known point
                if (distance_to_u1 > distance_to_u2){
                    u1 = curr_u;
                    distance_to_u1 = dist_to_p;
                } else { // (distance_to_p1 <= distance_to_p2)
                    u2 = curr_u;
                    distance_to_u2 = dist_to_p;
                }
            }
        }
    } while (std::abs(distance_to_u1 - distance_to_u2) > max_err);

    // Return the average of the two closest points
    return (distance_to_u1 + distance_to_u2)/2;
}

Spline ObstacleManager::updateLineWithNewLine(Spline current_line,
                                              PolynomialSegment new_line) {
    Point2D min_endpoint(new_line.x_min(), new_line(new_line.x_min()));
    Point2D max_endpoint(new_line.x_max(), new_line(new_line.x_max()));

    // Find the closest points on the spline to the start and end points of the
    // polynomial
    // TODO: Make max_err and num_sample points here class members we can set
    double u1 =
            findClosestPointOnSplineToPoint(current_line, min_endpoint, 100, 0.01);
    double u2 =
            findClosestPointOnSplineToPoint(current_line, max_endpoint, 100, 0.01);

    // "replace" the interpolation points between `u1` and `u2` with interpolation
    // points created from the polynomial line

    // Find the points "before" the polynomial line segment (on the spline)
    std::vector<Point2D> points_before_poly =
            current_line.getInterpolationPointsInRange(0, std::min(u1,u2));
    // Find the points through the polynomial line segment (on the polynomial itself)
    std::vector<Point2D> points_in_poly =
            getInterpolationPointsFromPolySegment(new_line);
    // Find the points "after" the polynomial segment (on the spline)
    std::vector<Point2D> points_after_poly =
            current_line.getInterpolationPointsInRange(std::max(u1,u2), 1);

    // Merge the 3 sets of interpolation points
    std::vector<Point2D> new_interpolation_points = points_before_poly;
    new_interpolation_points.insert(
            new_interpolation_points.end(), points_in_poly.begin(), points_in_poly.end());
    new_interpolation_points.insert(
            new_interpolation_points.end(), points_after_poly.begin(), points_after_poly.end());

    // Return a spline interpolated through our new points
    return Spline(new_interpolation_points);
}



