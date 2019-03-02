/*
 * Created By: Robyn Castro
 * Created On: October 31, 2018
 * Description: Tests calculation of risks
 */

#include "./TestUtils.h"
#include "mapping_msgs_urc/RiskAreaArray.h"

#include <RiskAnalysis.h>
#include <gtest/gtest.h>

pcl::PCLPointCloud2 empty_cloud = pcl::PCLPointCloud2();

/**
 * Runs tests to verify the inputted point_region has the proper dimensions specified
 * by all the other inputs
 *
 * @param point_region
 * @param region_width
 * @param region_height
 * @param num_vertical_cell_divs
 * @param num_horizontal_cell_divs
 */
void testRegionDimensions(std::vector<std::vector<RegionOfPoints>> point_region,
                          float region_width,
                          float region_height,
                          int num_vertical_cell_divs,
                          int num_horizontal_cell_divs);


/**
 *
 * @param risk_analysis
 * @param region_width
 * @param region_height
 * @param num_vertical_cell_divs
 * @param num_horizontal_cell_divs
 */
void testRegionLocations(RiskAnalysis risk_analysis,
                         float region_width,
                         float region_height,
                         int num_vertical_cell_divs,
                         int num_horizontal_cell_divs);
/**
 * Runs tests to determine equality of points
 */
void EXPECT_POINT_EQ(sb_geom_msgs::Point2D expected_val, sb_geom_msgs::Point2D actual_val) {
    float abs_error = 0.0001;

    EXPECT_NEAR(expected_val.x, actual_val.x, abs_error);
    EXPECT_NEAR(expected_val.y, actual_val.y, abs_error);
}
 
 /**
  * Creates a point with specified values
  */
sb_geom_msgs::Point2D createPoint(float x, float y) {
    sb_geom_msgs::Point2D point;
    point.x = x;
    point.y = y;
    
    return point;
}

TEST(RegionInitialisation, DivisionsEqualToDimensions) {
    float region_width           = 10;
    float region_height          = 10;
    int num_vertical_cell_divs   = 10;
    int num_horizontal_cell_divs = 10;
    int min_points_in_region     = 1;
    float risk_multiplier = 1;
    RiskAnalysis risk_analysis   = RiskAnalysis(region_width,
                                              region_height,
                                              num_vertical_cell_divs,
                                              num_horizontal_cell_divs,
                                              min_points_in_region,
                                              risk_multiplier);

    std::vector<std::vector<RegionOfPoints>> point_region;

    point_region = risk_analysis.initialisePointRegions();

    EXPECT_EQ(num_vertical_cell_divs, point_region.size());
    EXPECT_EQ(num_horizontal_cell_divs, point_region[0].size());

    testRegionDimensions(point_region,
                         region_width,
                         region_height,
                         num_vertical_cell_divs,
                         num_horizontal_cell_divs);

    testRegionLocations(risk_analysis, region_width, region_height, num_vertical_cell_divs, num_horizontal_cell_divs);
}

TEST(RegionAnalysis, DivisionsEqualToDimensions) {
    float region_width           = 10;
    float region_height          = 10;
    int num_vertical_cell_divs   = 10;
    int num_horizontal_cell_divs = 10;
    int min_points_in_region     = 1;
    float risk_multiplier = 1;

    RiskAnalysis risk_analysis   = RiskAnalysis(region_width,
                                              region_height,
                                              num_vertical_cell_divs,
                                              num_horizontal_cell_divs,
                                              min_points_in_region,
                                              risk_multiplier);

    mapping_msgs_urc::RiskAreaArray pcl_risk =
    risk_analysis.assessPointCloudRisk(empty_cloud);
}

TEST(RegionInitialisation, NonIntegerCellDimensions) {
    float region_width           = 10;
    float region_height          = 10;
    int num_vertical_cell_divs   = 100;
    int num_horizontal_cell_divs = 100;
    int min_points_in_region     = 1;
    float risk_multiplier = 1;

    RiskAnalysis risk_analysis   = RiskAnalysis(region_width,
                                              region_height,
                                              num_vertical_cell_divs,
                                              num_horizontal_cell_divs,
                                              min_points_in_region,
                                              risk_multiplier);

    std::vector<std::vector<RegionOfPoints>> point_region;

    point_region = risk_analysis.initialisePointRegions();

    EXPECT_EQ(num_vertical_cell_divs, point_region.size());
    EXPECT_EQ(num_horizontal_cell_divs, point_region[0].size());

    testRegionDimensions(point_region,
                         region_width,
                         region_height,
                         num_vertical_cell_divs,
                         num_horizontal_cell_divs);

    testRegionLocations(risk_analysis, region_width, region_height, num_vertical_cell_divs, num_horizontal_cell_divs);

}

TEST(RegionAnalysis, NonIntegerCellDimensions) {
    float region_width           = 10;
    float region_height          = 10;
    int num_vertical_cell_divs   = 100;
    int num_horizontal_cell_divs = 100;
    int min_points_in_region     = 1;
    float risk_multiplier = 1;

    RiskAnalysis risk_analysis   = RiskAnalysis(region_width,
                                              region_height,
                                              num_vertical_cell_divs,
                                              num_horizontal_cell_divs,
                                              min_points_in_region,
                                              risk_multiplier);

    mapping_msgs_urc::RiskAreaArray pcl_risk =
    risk_analysis.assessPointCloudRisk(empty_cloud);
}

TEST(RegionInitialisation, OneDiv) {
    float region_width           = 2;
    float region_height          = 2;
    int num_vertical_cell_divs   = 2;
    int num_horizontal_cell_divs = 1;
    int min_points_in_region     = -1;
    float risk_multiplier = 1;

    RiskAnalysis risk_analysis   = RiskAnalysis(region_width,
                                                region_height,
                                                num_vertical_cell_divs,
                                                num_horizontal_cell_divs,
                                                min_points_in_region,
                                                risk_multiplier);

    std::vector<std::vector<RegionOfPoints>> point_region;

    point_region = risk_analysis.initialisePointRegions();

    EXPECT_EQ(num_vertical_cell_divs, point_region.size());
    EXPECT_EQ(num_horizontal_cell_divs, point_region[0].size());

    testRegionDimensions(point_region,
                         region_width,
                         region_height,
                         num_vertical_cell_divs,
                         num_horizontal_cell_divs);

    testRegionLocations(risk_analysis, region_width, region_height, num_vertical_cell_divs, num_horizontal_cell_divs);
}

TEST(RegionAnalysis, OneDiv) {
    float region_width           = 2;
    float region_height          = 2;
    int num_vertical_cell_divs   = 2;
    int num_horizontal_cell_divs = 1;
    int min_points_in_region     = -1;
    float risk_multiplier = 1;

    RiskAnalysis risk_analysis   = RiskAnalysis(region_width,
                                                region_height,
                                                num_vertical_cell_divs,
                                                num_horizontal_cell_divs,
                                                min_points_in_region,
                                                risk_multiplier);

    mapping_msgs_urc::RiskAreaArray pcl_risk =
            risk_analysis.assessPointCloudRisk(empty_cloud);
}

void testRegionDimensions(std::vector<std::vector<RegionOfPoints>> point_region,
                          float region_width,
                          float region_height,
                          int num_vertical_cell_divs,
                          int num_horizontal_cell_divs) {
    float cell_width  = region_width / num_horizontal_cell_divs;
    float cell_height = region_height / num_vertical_cell_divs;

    float abs_error = 0.001;
    for (int i = 0; i < point_region.size(); i++) {
        for (int j = 0; j < point_region[0].size(); j++) {
            sb_geom_msgs::Polygon2D cur_cell = point_region[i][j].region_area;

            // Top Left Point vs Bottom Left Point
            EXPECT_NEAR(cell_height, cur_cell.points[0].x - cur_cell.points[3].x, abs_error);

            // Top Right Point vs Bottom Right Point
            EXPECT_NEAR(cell_height, cur_cell.points[1].x - cur_cell.points[2].x, abs_error);

            // Top Left Point vs Top Right Point
            EXPECT_NEAR(cell_width, cur_cell.points[0].y - cur_cell.points[1].y, abs_error);

            // Bottom Left Point vs Bottom Right Point
            EXPECT_NEAR(cell_width, cur_cell.points[3].y - cur_cell.points[2].y, abs_error);
        }
    }
}

void testRegionLocations(RiskAnalysis risk_analysis,
                         float region_width,
                         float region_height,
                         int num_vertical_cell_divs,
                         int num_horizontal_cell_divs) {
    float cell_width  = region_width / num_horizontal_cell_divs;
    float cell_height = region_height / num_vertical_cell_divs;
    
    float cur_cell_top = region_height;
    for (int i = 0; i < num_vertical_cell_divs; i++) {
        float cur_cell_left = region_width / 2;
        
        for (int j = 0; j < num_horizontal_cell_divs; j++) {
            sb_geom_msgs::Polygon2D cur_region =
                    risk_analysis.getRegionAreaFromIndices(i, j);
            
            sb_geom_msgs::Point2D top_left_point = createPoint(cur_cell_top, cur_cell_left);
            sb_geom_msgs::Point2D top_right_point = createPoint(cur_cell_top, cur_cell_left - cell_width);
            sb_geom_msgs::Point2D bottom_right_point = createPoint(cur_cell_top - cell_height, cur_cell_left - cell_width);
            sb_geom_msgs::Point2D bottom_left_point = createPoint(cur_cell_top - cell_height, cur_cell_left);

            EXPECT_POINT_EQ(top_left_point, cur_region.points[0]);
            EXPECT_POINT_EQ(top_right_point, cur_region.points[1]);
            EXPECT_POINT_EQ(bottom_right_point, cur_region.points[2]);
            EXPECT_POINT_EQ(bottom_left_point, cur_region.points[3]);
            cur_cell_left -= cell_width;
        }

        cur_cell_top -= cell_height;
    }
}

TEST(StandardDeviation, OneDataPoint) {
    RiskAnalysis risk_analysis = RiskAnalysis();

    std::vector<float> data_set = {1};

    EXPECT_FLOAT_EQ(0.0, risk_analysis.calculateStandardDeviation(data_set));
}

TEST(StandardDeviation, TwoEqualDataPoints) {
    RiskAnalysis risk_analysis = RiskAnalysis();

    std::vector<float> data_set = {1, 1};

    EXPECT_FLOAT_EQ(0.0, risk_analysis.calculateStandardDeviation(data_set));
}

TEST(StandardDeviation, TwoUnequalEqualDataPoints) {
    RiskAnalysis risk_analysis = RiskAnalysis();

    std::vector<float> data_set = {1, 2};

    EXPECT_FLOAT_EQ(0.5, risk_analysis.calculateStandardDeviation(data_set));
}

TEST(StandardDeviation, NonIntegerDataPoints) {
    RiskAnalysis risk_analysis = RiskAnalysis();

    std::vector<float> data_set = {5.4, 3.4, 8, 100, 20, 5.5};

    EXPECT_NEAR(
    34.54487018, risk_analysis.calculateStandardDeviation(data_set), 0.01);
}

TEST(StandardDeviation, NegativeNonIntegerDataPoints) {
    RiskAnalysis risk_analysis = RiskAnalysis();

    std::vector<float> data_set = {-5.4, 3.4, -8, 100, -20, 5.5};

    EXPECT_NEAR(
    39.96858836, risk_analysis.calculateStandardDeviation(data_set), 0.01);
}

TEST(StandardDeviation, RandomDataPoints) {
    RiskAnalysis risk_analysis = RiskAnalysis();

    std::vector<float> data_set = {1, 2, -7, 5.4, 3.4, 8, 100, 20, -5};

    EXPECT_NEAR(
    31.21424176, risk_analysis.calculateStandardDeviation(data_set), 0.01);
}

TEST(StandardDeviation, RandomDataPointsLargeNumbers) {
    RiskAnalysis risk_analysis = RiskAnalysis();

    std::vector<float> data_set = {
    1, 2, -7, 5.4, 3.4, 8, 100, 20, -5000, 254000, 5, -200080};

    EXPECT_NEAR(
    93261.48782, risk_analysis.calculateStandardDeviation(data_set), 0.01);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
