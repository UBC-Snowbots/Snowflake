/*
 * Created By: Robyn Castro
 * Created On: October 14, 2018
 * Description: Processes surrounding environment data to create a regional
 * assessment of risk
 */

#ifndef RISKANALYSIS_H
#define RISKANALYSIS_H

// Messages
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>

#include "mapping_msgs_urc/RiskArea.h"
#include "mapping_msgs_urc/RiskAreaArray.h"

#include "sb_geom_msgs/Point2D.h"
#include "sb_geom_msgs/Polygon2D.h"

// Point Cloud
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <tr1/unordered_map>


// Standard Libraries
#include <numeric>

typedef struct {
    sb_geom_msgs::Polygon2D region_area;
    std::vector<geometry_msgs::Point> points;
} RegionOfPoints;

class RiskAnalysis {
  public:
    /**
     * Constructor for RiskAnalysis
     *
     * @param region_width determines the range in the y-axis [-region_width/2, region_width/2]
     * @param region_height determines the range in the x-axis [0, region_height]
     * @param num_vertical_cell_div determines the number of rows
     * @param num_horizontal_cell_div determines the number of columns
     * @param region_min_points the minimum amount of points to conclude riskiness of a risk area
     * @param risk_multiplier a multiplier applied to the risk after calculating the risk of the region
     */
    RiskAnalysis(float region_width,
                 float region_height,
                 int num_vertical_cell_div,
                 int num_horizontal_cell_div,
                 int region_min_points,
                 float risk_multiplier);

    /**
     * Required empty constructor
     */
    RiskAnalysis();

    /**
     * Assigns risk to regions with enough information (min_points_in_region).
     * Uses standard deviation of point height to measure risk.
     *
     * @param point_cloud the point cloud of the ground
     * @return regions with associated risk
     */
    mapping_msgs_urc::RiskAreaArray
    assessPointCloudRisk(pcl::PCLPointCloud2 point_cloud);

    /**
     * Creates a 2D vector of RegionOfPoints where...
     * # rows = num_vertical_cell_div
     * # cols = num_horizontal_cell_div
     *
     * Each region's height and width correspond to cell_width and
     * cell_height
     *
     * The points are intialised as empty
     *
     * @return 2D vector of RegionOfPoints
     */
    std::vector<std::vector<RegionOfPoints>>
    initialisePointRegions();

    /**
     * Fills each region's vector of points with the input point cloud.
     * The region chosen is dependent on the point's x and y values.
     *
     * @param pcl
     * @param regions
     */
    void fillPointRegions(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl,
                          std::vector<std::vector<RegionOfPoints>>& regions);

    /**
     * Assigns risk to regions with enough information (min_points_in_region).
     * Uses standard deviation of point height (z-value) to measure risk.
     *
     * @param regions 2D vector
     * @return regions with associated risk
     */
    mapping_msgs_urc::RiskAreaArray
    analysePointRegions(std::vector<std::vector<RegionOfPoints>> regions);

    /**
     * Calculates the standard deviation of a given vector of values
     *
     * @param values
     * @return float standard deviation of inputted floats
     */
    float calculateStandardDeviation(std::vector<float> values);

    /**
     * Creates a region in a location specified by the inputted row and column.
     *
     * The dimensions of the region are equal to cell_height and cell_width.
     *
     * @param row
     * @param column
     * @return sb_geom_msgs::Polygon2D region
     */
    sb_geom_msgs::Polygon2D getRegionAreaFromIndices(int row, int column);

    /**
     * Given an x value, return the correct row that the point should be in.
     *
     * @param x the x-value of a point
     * @return int the row that corresponds to the x-value
     */
    int determineRow(float x);

    /**
     * Given a y value, return the correct column that the point should be in.
     *
     * @param y the y-value of a point
     * @return int the column that corresponds to the y-value
     */
    int determineColumn(float y);

  private:
    double MAX_RISK = 1;

    // Dimensions of the region
    float region_width;
    float region_height;
    float risk_multiplier;

    int num_vertical_cell_div;
    int num_horizontal_cell_div;

    int region_min_points;

    // Dimension of cells
    float cell_width;
    float cell_height;
    int total_cells;
};

#endif // SNOWFLAKE_RISKANALYSIS_H
