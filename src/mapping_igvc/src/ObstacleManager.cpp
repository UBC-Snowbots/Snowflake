/*
 * Created By: Gareth Ellis
 * Created On: January 9, 2018
 * Description: TODO
 */

// Snowbots Includes
#include <ObstacleManager.h>
#include <sb_geom/util.h>

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
void ObstacleManager::addObstacle(Polynomial line) {
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
                                                  Polynomial new_line) {
    // TODO
//    - find closest point on spline to end points of poly
//    - remove points in spline between two found closest points
//    - re-fit spline through some points in poly and remaining points from spline2
}


