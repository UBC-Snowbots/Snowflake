/*
 * Created By: Robyn Castro
 * Created On: October 14, 2018
 * Description: Processes surrounding environment data to create a regional assessment of risk
 */


#include "RiskAnalysis.h"

RiskAnalysis::RiskAnalysis(float region_width, float region_height, int num_vertical_cell_div, int num_horizontal_cell_div, int region_min_points) :
    region_width(region_width),
    region_height(region_height),
    num_vertical_cell_div(num_vertical_cell_div),
    num_horizontal_cell_div(num_horizontal_cell_div) {

    cell_width = region_width / num_horizontal_cell_div;
    cell_height = region_height / num_vertical_cell_div;
    total_cells = num_horizontal_cell_div * num_vertical_cell_div;
}

RiskAnalysis::RiskAnalysis() {
    // Empty Constructor
}

mapping_msgs_urc::RiskAreaArray RiskAnalysis::assessPointCloudRisk(pcl::PCLPointCloud2 point_cloud)
{
    mapping_msgs_urc::RiskAreaArray risk_areas;

    // Initialise regions with an area and associated points
    std::vector<std::vector<RegionOfPoints>> regions = initialisePointRegions(point_cloud);


    // Convert to PCLPointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(point_cloud, *pcl);

    fillPointRegions(pcl, regions);

    risk_areas = analysePointRegions(regions);

    return risk_areas;
}

std::vector<std::vector<RegionOfPoints>> RiskAnalysis::initialisePointRegions(pcl::PCLPointCloud2 point_cloud)
{
    std::vector<std::vector<RegionOfPoints>> regions;

    for (int i = 0; i < num_vertical_cell_div; i++) {
        std::vector<RegionOfPoints> new_region_row;
        for (int j = 0; j < num_horizontal_cell_div; j++) {
            RegionOfPoints new_region;

            sb_geom_msgs::Polygon2D region_area;
            region_area = getRegionAreaFromIndices(i, j);
            new_region.region_area = region_area;

            new_region_row.push_back(new_region);
        }
        regions.push_back(new_region_row);
    }

    return regions;
}

void RiskAnalysis::fillPointRegions(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl,
                                    std::vector<std::vector<RegionOfPoints>> &regions) {
    for (size_t i = 0; i < pcl->size(); i++) {
        // Convert PCLPointXYZ to geometry_msgs::Point
        geometry_msgs::Point cur_point;
        cur_point.x = pcl->points[i].x;
        cur_point.y = pcl->points[i].y;
        cur_point.z = pcl->points[i].z;

        // Determine which cell the point belongs to
        int col = determineRow(pcl->points[i].x);
        int row = determineColumn(pcl->points[i].y);

        bool validColumn = col >= 0 && col < regions[0].size();
        bool validRow = row >= 0 && row < regions.size();

        if (validColumn && validRow) {
            regions[row][col].points.push_back(cur_point);
        }
    }
}

mapping_msgs_urc::RiskAreaArray RiskAnalysis::analysePointRegions(std::vector<std::vector<RegionOfPoints>> regions) {

    mapping_msgs_urc::RiskAreaArray risk_areas;

    for (int i = 0; i < regions.size(); i++) {
        for (int j = 0; j < regions[0].size(); j++) {
            if (regions[i][j].points.size() > region_min_points) {
                std::vector<float> z_values;
                for (int k = 0; k < regions[i][j].points.size(); k++) {
                    z_values.push_back(regions[i][j].points[k].z);
                }

                // Determine risk of the area
                std_msgs::Float64 risk;
                risk.data = calculateStandardDeviation(z_values);

                // Add risk area onto the array
                mapping_msgs_urc::RiskArea risk_area;
                risk_area.score = risk;
                risk_area.area = regions[i][j].region_area;
                risk_areas.areas.push_back(risk_area);
            }
        }
    }

    return risk_areas;
}

float RiskAnalysis::calculateStandardDeviation(std::vector<float> values) {
    float sum = std::accumulate(values.begin(), values.end(), 0.0);
    float mean = sum / values.size();

    std::vector<float> diff(values.size());
    std::transform(values.begin(), values.end(), diff.begin(),
                   std::bind2nd(std::minus<float>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / values.size());

    return stdev;
}

sb_geom_msgs::Polygon2D RiskAnalysis::getRegionAreaFromIndices(int row, int column) {
    sb_geom_msgs::Point2D top_left_point;
    top_left_point.x = region_height - row*cell_height;
    top_left_point.y = region_width / 2.0 - column*cell_width;

    sb_geom_msgs::Point2D top_right_point;
    top_right_point.x = region_height - row*cell_height;
    top_right_point.y = region_width / 2.0 - (column + 1)*cell_width;

    sb_geom_msgs::Point2D bottom_left_point;
    bottom_left_point.x = region_width - (row + 1)*cell_height;
    bottom_left_point.y = region_width / 2.0 - column*cell_width;

    sb_geom_msgs::Point2D bottom_right_point;
    bottom_right_point.x = region_width - (row + 1.0)*cell_height;
    bottom_right_point.y = region_width / 2.0 - (column + 1.0)*cell_width;

    sb_geom_msgs::Polygon2D region_area;
    region_area.points.push_back(top_left_point);
    region_area.points.push_back(top_right_point);
    region_area.points.push_back(bottom_right_point);
    region_area.points.push_back(bottom_left_point);

    return region_area;

}

int RiskAnalysis::determineRow(float x)
{
    int row = x / cell_height;
    return row;
}

int RiskAnalysis::determineColumn(float y)
{
    int col = (y + region_width / 2) / cell_width;
    return col;
}