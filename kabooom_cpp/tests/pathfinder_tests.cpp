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

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

class BackTrackTest : public ::testing::Test {
};

TEST_F(BackTrackTest, SingleNode) {
    auto current_node = std::make_shared<Node>(
        State{Point{5, 6}, Point{3, 4}, 1.0f}
    );

    std::vector<Point> result = BackTrack(current_node);
    EXPECT_EQ(result.size(), 1);
    
    Point expected_point{3, 4};
    EXPECT_EQ(result.at(0), expected_point);
}

TEST_F(BackTrackTest, MultipleNodes) {
    auto node1 = std::make_shared<Node>(
        State{Point{3, 3}, Point{1, 1}, 1.0f}
    );

    auto node2 = std::make_shared<Node>(
        State{Point{4, 4}, Point{2, 2}, 2.0f}, 
        node1
    );

    auto node3 = std::make_shared<Node>(
        State{Point{5, 5}, Point{3, 3}, 3.0f}, 
        node2
    );

    std::vector<Point> result = BackTrack(node3);
    ASSERT_EQ(result.size(), 3);
    
    Point expected_point1{1, 1};
    Point expected_point2{2, 2};
    Point expected_point3{3, 3};

    EXPECT_EQ(result.at(0), expected_point1);
    EXPECT_EQ(result.at(1), expected_point2);
    EXPECT_EQ(result.at(2), expected_point3);
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////


class BFSTest : public ::testing::Test {
protected:
    Map map{};
    Point start{};
    std::vector<Point> target_trajectory;
    std::vector<Point> expected_solution;

    void SetUp() override
    {
        map = Map(1.0f, 5, 5);
    }

    void TearDown() override
    {
    }
};

TEST_F(BFSTest, RightOnTheTarget) {

    target_trajectory = {Point{0,0},Point{0,1},Point{0,2},Point{0,3},Point{0,4}};
    start = {0, 0};
    expected_solution = {Point{0,0}};
    std::vector<Point> solution = bfs(start, target_trajectory, map);
    
    EXPECT_EQ(expected_solution, solution);
}

TEST_F(BFSTest, EndToEndDiagonal) {

    target_trajectory = {Point{0,0},Point{0,1},Point{0,2},Point{0,3},Point{0,4}};
    start = {4, 0};
    expected_solution = {Point{4,0},Point{3,1},Point{2,2},Point{1,3},Point{0,4}};
    std::vector<Point> solution = bfs(start, target_trajectory, map);
    
    EXPECT_EQ(expected_solution, solution);
}

TEST_F(BFSTest, TwoMovesAway) {

    target_trajectory = {Point{0,0},Point{0,1},Point{0,2},Point{0,3},Point{0,4}};
    start = {2, 2};
    auto expected_solution_time = 2;
    std::vector<Point> solution = bfs(start, target_trajectory, map);
    
    EXPECT_EQ(expected_solution_time, size(solution)-1);
}



///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
