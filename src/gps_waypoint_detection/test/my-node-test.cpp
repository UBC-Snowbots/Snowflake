/*
 * Created By: Gareth Ellis
 * Created On: July 16th, 2016
 * Description: Tests for MyNode
 */

#include <MyNode.h>
#include <gtest/gtest.h>

TEST(MyNode, addExclamationPoint){
    EXPECT_EQ("!", MyClass::addCharacterToString("", "!"));
    EXPECT_EQ("Hello!", MyClass::addCharacterToString("Hello", "!"));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}