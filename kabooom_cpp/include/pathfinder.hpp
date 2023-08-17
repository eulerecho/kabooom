#pragma once
#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <memory>
#include <random>

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

struct Point
{
    int x{}, y{};

    inline bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

inline std::ostream &operator<<(std::ostream &os, const Point &point)
{
    os << "(" << point.x << ", " << point.y << ")";
    return os;
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

struct State
{
    Point target_state{};
    Point ego_state{};
    float time_step{};

    State() : target_state(), ego_state(), time_step(0.0f) {}

    State(Point target, Point ego, float time = 0.0f) : target_state(target), ego_state(ego), time_step(time) {}
};

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

struct Node
{
    State state{};
    std::shared_ptr<Node> parent = nullptr;

    Node(State s, std::shared_ptr<Node> p = nullptr) : state(s), parent(p) {}

    bool intercepted() const
    {
        return this->state.target_state.x == this->state.ego_state.x &&
               this->state.target_state.y == this->state.ego_state.y;
    }
};

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

struct Map
{
    float resolution{};
    uint8_t width{}, height{};
    int cell_x{}, cell_y{};
    std::vector<std::vector<uint8_t>> grid{};

    Map(float resolution = 1.0f, uint8_t width = 5, uint8_t height = 5)
        : resolution(resolution), width(width), height(height)
    {

        cell_x = static_cast<int>(width / resolution);
        cell_y = static_cast<int>(height / resolution);
        grid = std::vector<std::vector<uint8_t>>(cell_x, std::vector<uint8_t>(cell_y, 0));
    }

    Point GetRandomPoint()
    {
        // Random number generator
        std::mt19937 rng(std::random_device{}());

        std::uniform_int_distribution<int> width_dist(0, cell_x - 1);
        std::uniform_int_distribution<int> height_dist(0, cell_y - 1);

        return Point{width_dist(rng), width_dist(rng)};
    }
};

std::vector<Point> GetDummyTrajectory(const Map &map, const Point &start, const Point &end);

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

std::vector<Point> bfs(const Point &start_point, const std::vector<Point> &trajectory, const Map &occupancy_map);
std::vector<Point> GetNeighbours(const Node &node, const std::vector<Point> &available_directions);
bool IsFeasible(const Point &neighbour, const Map &occupancy_map);
std::vector<Point> BackTrack(const std::shared_ptr<Node> &current_node);