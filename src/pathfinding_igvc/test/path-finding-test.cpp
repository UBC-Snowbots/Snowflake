#include <PathFinding.h>
#include <gtest/gtest.h>


TEST(PathFinding, testWeightedSum1){

    //PathFinding test_node(NULL,NULL,NULL);

    std::vector<float> testVec;
    testVec.push_back(3);
    testVec.push_back(3);
    testVec.push_back(3);

    EXPECT_EQ(6.0, PathFinding::weightedXSum(testVec, 3));
}

TEST(PathFinding, testWeightedSum2){

    //PathFinding test_node(NULL,NULL,NULL);

    std::vector<float> testVec;
    testVec.push_back(1);
    testVec.push_back(2);
    testVec.push_back(3);

    EXPECT_EQ(1.0, PathFinding::weightedYSum(testVec, 1));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}