#include <gtest/gtest.h>

class BoilerPlateTest : public ::testing::Test {
protected:

};

TEST_F(BoilerPlateTest, TestThis) {
    //EXPECT_DOUBLE_EQ();
}

TEST_F(BoilerPlateTest, TestThat) {
    //EXPECT_THROW(, std::runtime_error);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
