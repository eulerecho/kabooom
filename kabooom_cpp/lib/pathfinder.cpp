#include <pathfinder.hpp>
#include <algorithm>
#include <random>
#include <ctime>

std::vector<Point> bfs(const Point &start_point, const std::vector<Point> &trajectory, const Map &occupancy_map)
{

    State start_state(trajectory.at(0), start_point);
    auto start = std::make_shared<Node>(start_state);

    std::deque<std::shared_ptr<Node>> queue;
    queue.push_back(start);

    std::vector<Point> available_directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!queue.empty())
    {
        auto current_node = std::move(queue.front());
        queue.pop_front();

        if (current_node->intercepted())
        {
            std::cout << "Intercepted the target!" << std::endl;
            return BackTrack(std::move(current_node));
        }

        for (const auto &neighbour : GetNeighbours(*current_node, available_directions))
        {
            if (IsFeasible(neighbour, occupancy_map))
            {
                int current_time = current_node->state.time_step + 1;

                if (current_time < static_cast<int>(trajectory.size()))
                {
                    Point target_point = trajectory[current_time];
                    Point ego_point = neighbour;
                    State next_state(target_point, ego_point, current_time);

                    auto next_node = std::make_shared<Node>(next_state, current_node);
                    queue.push_back(next_node);
                }
            }
        }
    }

    return {};
}

std::vector<Point> GetNeighbours(const Node &vertex, const std::vector<Point> &movement_directions)
{

    std::vector<Point> neighbours;
    const Point &ego = vertex.state.ego_state;

    for (const auto &dir : movement_directions)
    {
        Point neighbour;
        neighbour.x = ego.x + dir.x;
        neighbour.y = ego.y + dir.y;
        neighbours.push_back(neighbour);
    }

    return neighbours;
}

bool IsFeasible(const Point &neighbour, const Map &occupancy_map)
{

    int x = neighbour.x;
    int y = neighbour.y;

    return (0 <= x && x < occupancy_map.cell_x) &&
           (0 <= y && y < occupancy_map.cell_y) &&
           (occupancy_map.grid[x][y] == 0);
}

std::vector<Point> BackTrack(const std::shared_ptr<Node> &current_node)
{

    if (current_node == nullptr)
    {
        return {};
    }

    std::vector<Point> path;
    std::shared_ptr<Node> iter_node = current_node;

    while (iter_node != nullptr)
    {
        path.push_back(iter_node->state.ego_state);
        iter_node = iter_node->parent;
    }
    // Reverse the path to get the correct order from start to goal
    std::reverse(path.begin(), path.end());

    return path;
}

// Brehensam's line algorithm to generate a dummy trajectory from a start to a goal point
std::vector<Point> GetDummyTrajectory(const Map &map, const Point &start, const Point &end)
{

    std::vector<Point> trajectory;

    int dx = std::abs(end.x - start.x), sx = start.x < end.x ? 1 : -1;
    int dy = -std::abs(end.y - start.y), sy = start.y < end.y ? 1 : -1;
    int err = dx + dy, e2;

    int x = start.x, y = start.y;
    while (true)
    {
        trajectory.push_back({x, y});
        if (x == end.x && y == end.y)
            break;

        e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y += sy;
        }
    }

    return trajectory;
}

