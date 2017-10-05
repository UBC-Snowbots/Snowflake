/*
 * Created By: Gareth Ellis
 * Created On: July 4, 2017
 * Description: TODO
 */

#ifndef DRAG_RACE_LIDAROBSTACLEMANAGER_H
#define DRAG_RACE_LIDAROBSTACLEMANAGER_H

// STD Includes
#include <vector>

// ROS Includes
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

// SB Includes
#include <LidarObstacle.h>

class SlopeInterceptLine {
  public:
    SlopeInterceptLine(double slope, double y_intercept)
      : slope(slope), y_intercept(y_intercept) {}

    // TODO: DOC functions
    inline double getSlope() { return slope; }

    inline double getYIntercept() { return y_intercept; }

    inline double getXIntercept() { return -y_intercept / slope; }

    inline double getXCoorAtY(double y) {
        if (slope == 0)
            return y_intercept;
        else
            return (y - y_intercept) / slope;
    }

    inline double getYCoorAtX(double x) { return slope * x + y_intercept; }

  protected:
    double slope;
    double y_intercept;
};

/**
 * A line with a Correlation Coefficient
 */
class LineOfBestFit : public SlopeInterceptLine {
  public:
    LineOfBestFit(double slope, double y_intercept, double correlation)
      : SlopeInterceptLine(slope, y_intercept), correlation(correlation) {}

    // TODO: Add getter function and make private
    double correlation;
};

struct FiniteLine {
    double start_x;
    double start_y;
    double slope;
    double length;
};

class LidarObstacleManager {
  public:
    LidarObstacleManager();

    LidarObstacleManager(double max_obstacle_merging_distance,
                         double cone_grouping_tolerance,
                         double max_distance_from_robot_accepted,
                         double min_wall_length,
                         double collision_distance,
                         double front_angle,
                         double side_angle_max,
                         double side_angle_min,
                         double region_fill_percentage,
                         bool front_collision_only);

    /**
     * Merges or adds the given obstacle to the already saved ones
     *
     * @param obstacle the obstacle to be added
     */
    void addObstacle(LidarObstacle obstacle);

    /**
     * Finds a saves obstacles from the given scan
     *
     * Gets obstacles from the given scan and merges or adds them to
     * the already saved obstacles
     *
     * @param scan the scan to be merged in
     */
    void addLaserScan(const sensor_msgs::LaserScan& scan);

    /**
     * Determines whether there is an obstacle in front of the robot
     * @param scan the laserscan containing the area data
     * @param min_obstacle_distance the distance of obstacle in front to compare
     * @return true if there is an obstancle in front, false otherwise
     */
    bool obstacleInFront(const sensor_msgs::LaserScan& scan,
                         double min_obstacle_distance);

    /**
     * Clears all saved obstacles
     */
    void clearObstacles();

    /**
     * Gets all obstacles
     * @return all saved obstacles
     */
    std::vector<LidarObstacle> getObstacles();

    /**
     * Gets all lines of cones in the saved obstacles
     * @return all lines of cones in the saved obstacles
     */
    std::vector<LineOfBestFit> getConeLines();

    /**
     * Gets a line of best fit for a given group of points
     *
     * @param points the points to fit the line to
     */
    static LineOfBestFit getLineOfBestFit(const std::vector<Point>& points);

    LineOfBestFit getBestLine(bool lineToTheRight);

    /**
     * Finds groups of points within a larger group of points
     *
     * @param points the points to find the groups in
     * @param tolerance the maximum distance between any two points in a group
     *
     * @return a vector of groups of points (as vectors)
     */
    std::vector<std::vector<Point>> getPointGroupings(std::vector<Point> points,
                                                      double tolerance);

    /**
     * Determines the minimum distance between two obstacles
     *
     * @param obstacle1
     * @param obstacle2
     * @return the minimum distance between obstacle1 and obstacle2
     */
    static double minDistanceBetweenObstacles(LidarObstacle obstacle1,
                                              LidarObstacle obstacle2);

    /**
     * Gets all stored obstacles as a marker of points that can be rendered in
     * RViz
     *
     * @return all stored obstacles as a marker of points that can be rendered
     * in RViz
     */
    // TODO: when we update lidarObstacle to be a bit more obstract, improve
    // this as well
    // TODO: to represent things like radius
    // TODO: TEST ME
    visualization_msgs::Marker getConeRVizMarker();

    /**
     * Gets the lines determined from cones as a marker of lines that can be
     * rendered in RViz
     *
     * @return the lines determined from cones as a marker of lines that can be
     * rendered in RViz
     */
    // TODO: Can we add line endings?
    // TODO: TEST ME
    visualization_msgs::Marker getConeLinesRVizMarker();

    /**
     * Gets the "best" line from all the cone lines as a marker we can visualize
     * in RViz
     *
     * @return the lines determined from cones as a marker that can be rendered
     * in RViz
     */
    // TODO: Can we add line endings?
    // TODO: TEST ME
    // TODO: Add line_to_the_right to the constructor
    visualization_msgs::Marker
    getBestConeLineRVizMarker(bool line_to_the_right);

    bool collisionDetected();

  private:
    // **** END ZONE COLLISION DETECTION ****
    // TODO: Move this to its own node later.

    // The threshold for object hitscan distance
    double collision_distance;

    // If True, only uses front collision detection
    bool front_collision_only;

    // The angle which defines the front region relative to the front of the
    // robot
    // e.g. If set to PI/2, the region extends from PI/4 to -PI/4 relative to a
    // line
    //      extending from the front of the robot.
    double front_angle;

    // Parameters for side obstacle detection
    // The angles which define the side region of the robot
    double side_angle_max;
    double side_angle_min;

    // The percentage of the region to be filled before we consider it a
    // collision
    double region_fill_percentage;

    // ******************************

    // All the obstacles we currently have
    std::vector<LidarObstacle> obstacles;

    // The maximum distance between two obstacles for them to be considered the
    // same
    double max_obstacle_merging_distance;

    // The maximum distance a line can have from the robot on the y-axis before
    // being thrown out
    double max_distance_from_robot_accepted;

    // The maximum permitted distance between cones in the same group
    double cone_grouping_tolerance;

    // The mimimum length of an obstacle before it's considered a wall
    double min_wall_length;

    // True if there is an obstacle within collision_distance away
    bool collision_detected;
};

#endif // DRAG_RACE_LIDAROBSTACLEMANAGER_H
