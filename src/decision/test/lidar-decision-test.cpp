/*
 * Created By: Gareth Ellis
 * Created On: September 22, 2016
 * Description: Tests for LidarDecision
 */

#include <LidarDecision.h>
#include <gtest/gtest.h>
#include <vector>

TEST(LidarDecision, obstacle_in_range){
    sensor_msgs::LaserScan test_scan;
    test_scan.angle_max = 3.1415;
    test_scan.angle_min = 0.0000;
    test_scan.angle_increment = 0.01;
    int vector_size = 300;
    std::vector<float> vec(300);

    for (int i = 0;i < vector_size;i++){
        vec[i] = 999999;
    }
    for (int i=100;i<200;i++)
        vec[i]=3;
    test_scan.ranges = vec;
    sensor_msgs::LaserScan::ConstPtr test_scan_ptr(new sensor_msgs::LaserScan(test_scan));

//    vector_size = vec.size();
            //static_cast<int>((test_scan.range_max - test_scan->range_min)/test_scan->angle_increment);

    EXPECT_EQ(true, LidarDecision::obstacle_in_range(1.95, 2.20, 5, test_scan_ptr));
    EXPECT_EQ(true, LidarDecision::obstacle_in_range(0.00, 1.1, 5, test_scan_ptr));
    EXPECT_EQ(false, LidarDecision::obstacle_in_range(0.00, 0.98, 5, test_scan_ptr));
    EXPECT_EQ(false, LidarDecision::obstacle_in_range(2.11, 2.55, 5, test_scan_ptr));
    EXPECT_EQ(true, LidarDecision::obstacle_in_range(1.5, 1.7, 5, test_scan_ptr));

}

TEST(LidarDecision, manage_twist){
    sensor_msgs::LaserScan test_scan;
    test_scan.angle_max = 3.0000;
    test_scan.angle_min = 0.0000;
    test_scan.angle_increment = 0.01;
    int vector_size = 300;
    std::vector<float> vec(300);
    for (int i = 0;i < vector_size;i++){
        vec[i] = 999999;
    }
    for (int i=100;i<160;i++)
        vec[i]=3;
    test_scan.ranges = vec;
    sensor_msgs::LaserScan::ConstPtr test_scan_ptr(new sensor_msgs::LaserScan(test_scan));

    geometry_msgs::Twist result;

    //test LidarDecision::manage_twist
    EXPECT_EQ(13, LidarDecision::manage_twist(test_scan_ptr).linear.x);
    EXPECT_EQ(-13, LidarDecision::manage_twist(test_scan_ptr).angular.z);


}





int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    return 0;
}
