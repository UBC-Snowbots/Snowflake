/*
 * Created By: Gareth Ellis
 * Created On:  Nov. 11th, 2018
 * Description: Tests for the Polygon2D class
 */

#include "sb_geom/Polygon2D.h"
#include <gtest/gtest.h>

using namespace sb_geom;

class Polygon2DTest : public testing::Test {
  protected:
    Polygon2DTest(){};

    virtual void SetUp() {}
};

TEST_F(Polygon2DTest, constructing_from_Point2D_classes) {
    std::vector<sb_geom::Point2D> points = {{10, 20}, {34.22, 9}, {34, 9.33}};
    Polygon2D polygon2D(points);
    EXPECT_EQ(points, polygon2D.getBoundaryPoints());
}

TEST_F(Polygon2DTest, constructing_from_Point2D_msgs) {
    std::vector<sb_geom_msgs::Point2D> point_msgs;
    sb_geom_msgs::Point2D point2D_msg;
    point2D_msg.x = 10;
    point2D_msg.y = 20;
    point_msgs.emplace_back(point2D_msg);
    point2D_msg.x = 34.22;
    point2D_msg.y = 9;
    point_msgs.emplace_back(point2D_msg);
    point2D_msg.x = 34;
    point2D_msg.y = 9.33;
    point_msgs.emplace_back(point2D_msg);

    Polygon2D polygon2D(point_msgs);
    std::vector<sb_geom::Point2D> points(point_msgs.begin(), point_msgs.end());
    EXPECT_EQ(points, polygon2D.getBoundaryPoints());
}

TEST_F(Polygon2DTest, constructing_from_polygon2d_msg) {
    sb_geom_msgs::Polygon2D polygon2D_msg;
    sb_geom_msgs::Point2D point2D_msg;
    point2D_msg.x = 10;
    point2D_msg.y = 20;
    polygon2D_msg.points.emplace_back(point2D_msg);
    point2D_msg.x = 34.22;
    point2D_msg.y = 9;
    polygon2D_msg.points.emplace_back(point2D_msg);
    point2D_msg.x = 34;
    point2D_msg.y = 9.33;
    polygon2D_msg.points.emplace_back(point2D_msg);

    Polygon2D polygon2D(polygon2D_msg);
    std::vector<sb_geom::Point2D> points(polygon2D_msg.points.begin(),
                                         polygon2D_msg.points.end());
    EXPECT_EQ(points, polygon2D.getBoundaryPoints());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}