from collections import deque
from grid import Map
import numpy as np
from collections import deque
import heapq


def dijkstra(start_point: tuple, trajectory: tuple, occupancy_map: 'Map'):

    start = (start_point[0], start_point[1], trajectory[0], trajectory[1], 0)
    available_directions = np.array(
        [[0, 1], [1, 0], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]])
    queue = []
    heapq.heappush(queue, (0, start))
    visited = set()
    previous = {start: None}
    cost = {start: 0}

    while queue:
        current_cost, current_node = heapq.heappop(queue)

        if current_node[0] == current_node[2] and current_node[1] == current_node[3]:
            return backtrack(current_node, previous)

        if current_node in visited:
            continue

        visited.add(current_node)

        for neighbour in get_neighbours(current_node, available_directions):
            if is_feasible(neighbour, occupancy_map):
                current_time = current_node[-1] + 1

                if current_time < len(trajectory) and neighbour not in visited:
                    next_node = (neighbour[0], neighbour[1], trajectory[current_time]
                                 [0], trajectory[current_time][1], current_time)

                    # Dijkstra's distance update step
                    # assuming control action takes a unit time
                    new_cost = cost[current_node] + 1

                    if new_cost < cost.get(next_node, float('inf')):
                        cost[next_node] = new_cost
                        previous[next_node] = current_node
                        heapq.heappush(queue, (new_cost, next_node))

    return None


def bfs(start_point: tuple, trajectory: tuple, occupancy_map: Map):

    start = (start_point[0], start_point[1], trajectory[0], trajectory[1], 0)
    available_directions = np.array(
        [[0, 1], [1, 0], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]])
    queue = deque()
    queue.append(start)
    visited = set()
    previous = {start: None}

    while queue:
        current_node = queue.popleft()
        if current_node[0] == current_node[2] and current_node[1] == current_node[3]:
            return backtrack(current_node, previous)
        if current_node in visited:
            continue
        visited.add(current_node)
        for neighbour in get_neighbours(current_node, available_directions):
            if is_feasible(neighbour, occupancy_map):
                current_time = current_node[-1] + 1
                if current_time < len(trajectory) and neighbour not in visited:
                    next_node = (neighbour[0], neighbour[1], trajectory[current_time]
                                 [0], trajectory[current_time][1], current_time)
                    previous[next_node] = current_node
                    queue.append(next_node)

    return None


def get_neighbours(vertex: tuple, movement_directions: np.ndarray = np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])) -> tuple:

    neighbours = np.array([[vertex[0], vertex[1]]] *
                          len(movement_directions)) + movement_directions
    return [tuple(neighbour) for neighbour in neighbours]

# Free space and bound check


def is_feasible(neighbour: np.ndarray, occupancy_map: Map) -> bool:

    x, y = neighbour[0], neighbour[1]
    return (0 <= x < occupancy_map.grid.shape[0]) and (0 <= y < occupancy_map.grid.shape[1]) and (occupancy_map.grid[x, y] == 0)


def backtrack(current_node: tuple, previous: dict[tuple:float]) -> np.ndarray:

    path = []

    while current_node:
        grid_path = [current_node[0], current_node[1]]
        path.append(grid_path)
        current_node = previous[current_node]

    return path[::-1]
