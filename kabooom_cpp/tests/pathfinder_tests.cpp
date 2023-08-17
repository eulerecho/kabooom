#include <gtest/gtest.h>
#include <pathfinder.hpp>

class TrajectoryGeneratorTest : public ::testing::Test
{
protected:
    Map map{};
    Point start{};
    Point end{};
    std::vector<Point> expected_trajectory;

    void SetUp() override
    {
        map = Map(1.0f, 10, 10);
    }

    void TearDown() override
    {
    }
};

TEST_F(TrajectoryGeneratorTest, DiagonalTrajectory)
{
    start = {0, 0};
    end = {9, 9};

    for (int i = 0; i < 10; ++i)
    {
        expected_trajectory.push_back({i, i});
    }

    std::vector<Point> actual_trajectory = GetDummyTrajectory(map, start, end);

    EXPECT_EQ(expected_trajectory, actual_trajectory);
}

TEST_F(TrajectoryGeneratorTest, HorizontalTrajectory)
{
    start = {0, 5};
    end = {9, 5};

    for (int i = 0; i < 10; ++i)
    {
        expected_trajectory.push_back({i, 5});
    }

    std::vector<Point> actual_trajectory = GetDummyTrajectory(map, start, end);

    EXPECT_EQ(expected_trajectory, actual_trajectory);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
