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
     * Constructor
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
     * Fills each region's points with the specified 
     * @param pcl
     * @param regions
     */
    void fillPointRegions(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl,
                          std::vector<std::vector<RegionOfPoints>>& regions);

    mapping_msgs_urc::RiskAreaArray
    analysePointRegions(std::vector<std::vector<RegionOfPoints>> regions);

    float calculateStandardDeviation(std::vector<float> values);

    sb_geom_msgs::Polygon2D getRegionAreaFromIndices(int row, int column);

    int determineRow(float x);

    int determineColumn(float y);

  private:
    double MAX_RISK = 1;

    float region_width;
    float region_height;
    float risk_multiplier;

    int num_vertical_cell_div;
    int num_horizontal_cell_div;

    int region_min_points;

    float cell_width;
    float cell_height;
    int total_cells;
};

#endif // SNOWFLAKE_RISKANALYSIS_H
