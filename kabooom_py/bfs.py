from __future__ import annotations
from dataclasses import dataclass
from collections import deque
from grid import Map
import numpy as np
from math import isclose


class Point:
    x: int = 0
    y: int = 0


@dataclass(eq=False)
class State:
    target_state: Point = Point()
    ego_state: Point = Point()
    time_step: float = 0

    def __eq__(self, other):
        if not isinstance(other, State):
            return False
        # Floating points are scary!
        return (self.target_state == other.target_state and
                self.ego_state == other.ego_state and
                isclose(self.time_step, other.time_step, rel_tol=1e-9))


class Node:
    state: State = State()
    parent: Node = None

    def success(self) -> bool:
        return self.state.target_state == self.state.ego_state

    def __repr__(self) -> str:
        return f"({self.state.time_step},({self.state.ego_state.x},{self.state.ego_state.y}),({self.state.target_state.x},{self.state.target_state.y})) -> "


def bfs(start_point: list, trajectory: np.ndarray, occupancy_map: Map):

    start = Node(State(Point(trajectory[0][0], trajectory[0][1]),
                       Point(start_point[0], start_point[1])))
    queue = deque()
    queue.append(start)
    available_directions = np.array(
        [[0, 1], [1, 0], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]])

    while queue:
        current_node = queue.popleft()
        if current_node.success():
            return backtrack(current_node)

        for neighbour in get_neighbours(current_node, available_directions):
            if is_feasible(neighbour, occupancy_map):
                current_time = current_node.state.time_step + 1
                if current_time < len(trajectory):
                    target_point = Point(
                        trajectory[current_time][0], trajectory[current_time][1])
                    ego_point = Point(neighbour[0], neighbour[1])
                    next_state = State(target_point, ego_point, current_time)
                    next_node = Node(next_state, current_node)
                    queue.append(next_node)

    return None


def get_neighbours(vertex: Node, movement_directions: np.ndarray = np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])) -> np.ndarray:

    return np.array([[vertex.state.ego_state.x, vertex.state.ego_state.y]]*len(movement_directions)) + movement_directions

# Free space and bound check


def is_feasible(neighbour: np.ndarray, occupancy_map: Map):
    x, y = neighbour[0], neighbour[1]
    return (0 <= x < occupancy_map.grid.shape[0]) and (0 <= y < occupancy_map.grid.shape[1]) and (occupancy_map.grid[x, y] == 0)


def backtrack(current_node: Node):

    path = []

    while current_node:
        grid_path = [current_node.state.ego_state.x,
                     current_node.state.ego_state.y]
        path.append(grid_path)
        current_node = current_node.parent

    return path[::-1]
