/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: Tests for LidarDecision
 */

#include <LidarDecision.h>
#include <gtest/gtest.h>

TEST(LidarDecision, obstacle_in_range){
    const sensor_msgs::LaserScan::ConstPtr & test_scan;
    test_scan->range_max = 3.1415;
    test_scan->range_min = 0.0000;
    test_scan->angle_increment = 0.001;
    int vector_size;
    vector_size = static_cast<int>((test_scan->range_max - test_scan->range_min)/test_scan.angle_increment);
    test_scan->ranges = std::vector<float> [vector_size];
    for (int i = 0;i < vector_size;i++){
        test_scan->ranges[i] = 999999999;
    }
    for (int i=100;i<200;i++)
    test_scan->ranges[i]=3;

    EXPECT_EQ(true, LidarDecision::obstacle_in_range(0.01, 2.00, 5, test_scan));
    EXPECT_EQ(true, LidarDecision::obstacle_in_range(0.199, 1.1, 5, test_scan));
    EXPECT_EQ(true, LidarDecision::obstacle_in_range(0.002, 0.11, 5, test_scan));
    EXPECT_EQ(false, LidarDecision::obstacle_in_range(0.21, 1.1, 5, test_scan));
    EXPECT_EQ(false, LidarDecision::obstacle_in_range(0.002, 0.90, 5, test_scan));

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
