import argparse
from grid import Map
from fox_traversals import *
import random
import matplotlib.pyplot as plt


def main(args):
    for i in range(args.num_iterations):
        dummy_map = Map(width=args.map_width,
                        height=args.map_height, resolution=args.map_resolution)
        circle = args.circle

        if not circle:
            dummy_map.add_n_obstacles(args.num_obstacles)
            trajectory = dummy_map.get_wall_trajectory()
            start = (int(random.random() * dummy_map.width / dummy_map.resolution),
                     int(random.random() * dummy_map.height / dummy_map.resolution))
        else:
            start = (int(random.random() * dummy_map.width / dummy_map.resolution),
                     int(random.random() * dummy_map.height / dummy_map.resolution))
            center = (random.random() * dummy_map.width,
                      random.random() * dummy_map.height)
            trajectory = dummy_map.get_circular_trajectory(center)

        tuple_of_trajs = tuple(tuple(sublist) for sublist in trajectory)

        if args.algorithm == 'dijkstra':
            path = dijkstra(start, tuple_of_trajs, dummy_map)
        elif args.algorithm == 'bfs':
            path = bfs(start, tuple_of_trajs, dummy_map)
        else:
            print("Select 'dijkstra' and 'bfs'.")
            return
        plt.show()
        dummy_map.simulate(path)
        plt.clf()
        plt.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Simulate path planning algorithms with various parameters.")
    parser.add_argument('--num_iterations', type=int, default=5,
                        help="Number of iterations to run the simulation.")
    parser.add_argument('--circle', action='store_true',
                        help="Whether to draw a circular trajectory or not.")
    parser.add_argument('--num_obstacles', type=int, default=20,
                        help="Number of obstacles to add to the map.")
    parser.add_argument('--map_width', type=int, default=5,
                        help="Width of the map.")
    parser.add_argument('--map_height', type=int,
                        default=5, help="Height of the map.")
    parser.add_argument('--map_resolution', type=int,
                        default=0.1, help="Resolution of the map.")
    parser.add_argument('--algorithm', type=str, default='dijkstra', choices=[
                        'dijkstra', 'bfs'], help="Path planning algorithm to use ('dijkstra' or 'bfs').")

    args = parser.parse_args()
    main(args)
