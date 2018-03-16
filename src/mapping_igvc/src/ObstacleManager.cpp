/*
 * Created By: Gareth Ellis
 * Created On: January 9, 2018
 * Description: TODO
 */

#include <ObstacleManager.h>
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
void ObstacleManager::addObstacle(PolyLine line) {
    // Find the distance from this line to every other known line
    std::vector<std::pair<double, int>> distances;
    for (int line_index = 0; line_index < lines.size(); line_index++) {
        double distance = distanceBetweenLines(lines[line_index], line);
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
            SplineLine merged_line = updateLineWithNewLine(lines[closest_known_line_index], line);
            // Overwrite the old line with our new merged line
            lines[closest_known_line_index] = merged_line;
        } else {
            // The closest line to this one isn't close enough to merge into,
            // so just add this line as a new line
            lines.emplace_back(SplineLine(line));
        }
    } else {
        // We don't know about any lines yet, so just add this one as it's own line
        lines.emplace_back(SplineLine(line));
    }
}

double ObstacleManager::distanceBetweenLines(SplineLine spline_line,
                                             PolyLine poly_line) {
    // TODO
    return 0;
}

SplineLine ObstacleManager::updateLineWithNewLine(SplineLine current_line,
                                                  PolyLine new_line) {
    // TODO
    return SplineLine(PolyLine());
}


