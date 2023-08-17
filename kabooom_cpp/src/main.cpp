#include<pathfinder.hpp>

int main()
{
    Map occupancy_grid(1, 10, 10);

    auto target_trajectory = GetDummyTrajectory(occupancy_grid, occupancy_grid.GetRandomPoint(), occupancy_grid.GetRandomPoint());

    std::cout << "Target's trajectory: ";
    for (const auto &point : target_trajectory)
    {
        std::cout << point << "-> ";
    }

    std::cout << std::endl;

    auto start_point = occupancy_grid.GetRandomPoint();

    std::cout << "Start point:" << start_point << std::endl;

    auto path = bfs(start_point, target_trajectory, occupancy_grid);

    if (path.empty())
    {
        std::cout << "Cannot intercept with given initial condition!" << std::endl;
        return 0;
    }

    else

    {
        std::cout << "Shortest path to strike the target:";

        for (const auto &point : path)
        {
            std::cout << point << "->";
        }
    }
}
