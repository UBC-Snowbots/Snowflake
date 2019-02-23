/*
 * Created By: Robyn Castro
 * Created On: October 31, 2018
 * Description: Tests calculation of risks
 */

#include "./TestUtils.h"
#include "mapping_msgs_urc/RiskAreaArray.h"

#include <RiskAnalysis.h>
#include <gtest/gtest.h>

/*
 * Function Prototype for Risk Analysis
 *
 * RiskAnalysis(float region_width, float region_height, int
 * num_vertical_cell_div,
 * int num_horizontal_cell_div, int region_min_points);
 *
 */

pcl::PCLPointCloud2 empty_cloud = pcl::PCLPointCloud2();
void testRegionDimensions(std::vector<std::vector<RegionOfPoints>> point_region,
                          float region_width,
                          float region_height,
                          int num_vertical_cell_divs,
                          int num_horizontal_cell_divs);

TEST(RegionCreation, DivisionsEqualToDimensions) {
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

    sb_geom_msgs::Polygon2D top_left_region =
    risk_analysis.getRegionAreaFromIndices(0, 0);
    EXPECT_FLOAT_EQ(region_height, top_left_region.points[0].x);
    EXPECT_FLOAT_EQ(region_width / 2, top_left_region.points[0].y);

    sb_geom_msgs::Polygon2D top_right_region =
    risk_analysis.getRegionAreaFromIndices(0, 9);
    EXPECT_FLOAT_EQ(region_height, top_right_region.points[1].x);
    EXPECT_FLOAT_EQ(-region_width / 2, top_right_region.points[1].y);

    sb_geom_msgs::Polygon2D bottom_right_region =
    risk_analysis.getRegionAreaFromIndices(9, 9);
    EXPECT_FLOAT_EQ(0, bottom_right_region.points[2].x);
    EXPECT_FLOAT_EQ(-region_width / 2, bottom_right_region.points[2].y);

    sb_geom_msgs::Polygon2D bottom_left_region =
    risk_analysis.getRegionAreaFromIndices(9, 0);
    EXPECT_FLOAT_EQ(0, bottom_left_region.points[3].x);
    EXPECT_FLOAT_EQ(region_width / 2, bottom_left_region.points[3].y);

    sb_geom_msgs::Polygon2D test_region =
            risk_analysis.getRegionAreaFromIndices(2, 9);

    std::cout << risk_analysis.determineColumn(-4.5) << std::endl;
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

TEST(RegionCreation, NonIntegerCellDimensions) {
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

    sb_geom_msgs::Polygon2D top_left_region =
    risk_analysis.getRegionAreaFromIndices(0, 0);
    EXPECT_FLOAT_EQ(region_height, top_left_region.points[0].x);
    EXPECT_FLOAT_EQ(region_width / 2, top_left_region.points[0].y);

    sb_geom_msgs::Polygon2D top_right_region =
    risk_analysis.getRegionAreaFromIndices(0, 99);
    EXPECT_FLOAT_EQ(region_height, top_right_region.points[1].x);
    EXPECT_FLOAT_EQ(-region_width / 2, top_right_region.points[1].y);

    sb_geom_msgs::Polygon2D bottom_right_region =
    risk_analysis.getRegionAreaFromIndices(99, 99);
    EXPECT_NEAR(0, bottom_right_region.points[2].x, 0.001);
    EXPECT_FLOAT_EQ(-region_width / 2, bottom_right_region.points[2].y);

    sb_geom_msgs::Polygon2D bottom_left_region =
    risk_analysis.getRegionAreaFromIndices(99, 0);
    EXPECT_FLOAT_EQ(0, bottom_left_region.points[3].x);
    EXPECT_FLOAT_EQ(region_width / 2, bottom_left_region.points[3].y);

    int row = risk_analysis.determineRow(2.8);
    int col = risk_analysis.determineColumn(0);

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

    for (int i = 0; i < point_region.size(); i++) {
        for (int j = 0; j < point_region[0].size(); j++) {
            sb_geom_msgs::Polygon2D cur_cell = point_region[i][j].region_area;

            // Top Left Point vs Bottom Left Point
            EXPECT_FLOAT_EQ(cell_height,
                            cur_cell.points[0].x - cur_cell.points[3].x);

            // Top Right Point vs Bottom Right Point
            EXPECT_FLOAT_EQ(cell_height,
                            cur_cell.points[1].x - cur_cell.points[2].x);

            // Top Left Point vs Top Right Point
            EXPECT_FLOAT_EQ(cell_width,
                            cur_cell.points[0].y - cur_cell.points[1].y);

            // Bottom Left Point vs Bottom Right Point
            EXPECT_FLOAT_EQ(cell_width,
                            cur_cell.points[3].y - cur_cell.points[2].y);
        }
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