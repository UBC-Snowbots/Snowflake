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
    std::vector<float> fake_test_data(300);

    for (int i = 0;i < fake_test_data.size();i++){
        fake_test_data[i] = 999999;
    }
    for (int i=100;i<150;i++)
        fake_test_data[i]=13;
    test_scan.ranges = fake_test_data;
    sensor_msgs::LaserScan::ConstPtr test_scan_ptr(new sensor_msgs::LaserScan(test_scan));

    EXPECT_EQ(false, LidarDecision::obstacle_in_range(1.00, 1.50, 5, test_scan_ptr));
    //obstacle inside of angle range but outside distance
    EXPECT_EQ(true, LidarDecision::obstacle_in_range(1.00, 1.50, 15, test_scan_ptr));
    //in angle range within distance
    EXPECT_EQ(false, LidarDecision::obstacle_in_range(1.51, 2.50, 15, test_scan_ptr));
    //within distance but just outside of angle range
    EXPECT_EQ(false, LidarDecision::obstacle_in_range(2.00, 2.55, 5, test_scan_ptr));
    //out side distance and out of angle
    EXPECT_EQ(true, LidarDecision::obstacle_in_range(1.48, 1.7, 15, test_scan_ptr));
    // partly in range

}

TEST(LidarDecision, manage_twist){
    sensor_msgs::LaserScan test_scan;
    test_scan.angle_max = 3.0000;
    test_scan.angle_min = 0.0000;
    test_scan.angle_increment = 0.01;

    //current parameters:
    //distances for near mid far is 5 10 20
    //expected linear/angular velocity for near, mid, far is 33331/33332 44441/44442 55551/55552
    //and for no obstacle, linear speed is 100, angular is 0
    //angle from 0 to 300, 150 in middle

    //1. far left obstacle
    std::vector<float> fake_test_data_1(300);
    for (int i = 0;i < fake_test_data_1.size();i++){
        fake_test_data_1[i] = 999999;
    }
    for (int i=10;i<18;i++)
        fake_test_data_1[i]=18;
    test_scan.ranges = fake_test_data_1;
    sensor_msgs::LaserScan::ConstPtr test_scan_ptr_1(new sensor_msgs::LaserScan(test_scan));


    EXPECT_EQ(55551, LidarDecision::manage_twist(test_scan_ptr_1).linear.x);
    EXPECT_EQ(-55552, LidarDecision::manage_twist(test_scan_ptr_1).angular.z);

    //2. mid right obstacle
    std::vector<float> fake_test_data_2(300);
    for (int i = 0;i < fake_test_data_2.size();i++){
        fake_test_data_2[i] = 999999;
    }
    for (int i=160;i<299;i++)
        fake_test_data_2[i]=8;
    test_scan.ranges = fake_test_data_2;
    sensor_msgs::LaserScan::ConstPtr test_scan_ptr_2(new sensor_msgs::LaserScan(test_scan));

    EXPECT_EQ(44441, LidarDecision::manage_twist(test_scan_ptr_2).linear.x);
    EXPECT_EQ(44442, LidarDecision::manage_twist(test_scan_ptr_2).angular.z);

    //3. near right obstacle
    std::vector<float> fake_test_data_3(300);
    for (int i = 0;i < fake_test_data_3.size();i++){
        fake_test_data_3[i] = 999999;
    }
    for (int i=140;i<180;i++)
        fake_test_data_3[i]=3;
    test_scan.ranges = fake_test_data_3;
    sensor_msgs::LaserScan::ConstPtr test_scan_ptr_3(new sensor_msgs::LaserScan(test_scan));

    EXPECT_EQ(33331, LidarDecision::manage_twist(test_scan_ptr_3).linear.x);
    EXPECT_EQ(33332, LidarDecision::manage_twist(test_scan_ptr_3).angular.z);

}





int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    return 0;
}
