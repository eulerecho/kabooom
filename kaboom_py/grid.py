import numpy as np
from enum import Enum
import random
import math
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Patch


class MapState(Enum):
    FREE = 0
    OCCUPIED = 1
    TARGET_STATE = 2
    EGO_STATE = 3


class Map:

    def __init__(self, resolution=0.1, width=5, height=5, obstacle_scale=[0.1, 0.25]) -> None:

        self.width, self.height = width, height
        self.resolution = resolution
        self.max_obstacle_width, self.max_obstacle_height = self.width * \
            obstacle_scale[1], self.height*obstacle_scale[1]
        self.min_obstacle_width, self.min_obstacle_height = self.width * \
            obstacle_scale[0], self.height*obstacle_scale[0]
        self.max_circle_radius = 0.5 * min(self.width, self.height)
        self.cell_x = int(self.width/self.resolution)
        self.cell_y = int(self.height/self.resolution)
        self.grid = np.zeros((self.cell_x, self.cell_y))
        self.trajectory = None
        self.colors = ['white', 'black', 'red', 'blue']
        self.cmap = mcolors.ListedColormap(self.colors)

    def _add_obstacle(self, start: list[int]) -> None:

        start_idx_x, start_idx_y = int(
            start[0]/self.resolution), int(start[1]/self.resolution)
        width = max(self.min_obstacle_width, random.random()
                    * self.max_obstacle_width)
        height = max(self.min_obstacle_height, random.random()
                     * self.max_obstacle_height)

        self.grid[start_idx_x:min(self.cell_x, int(start_idx_x+width/self.resolution)),
                  start_idx_y:min(self.cell_y, int(start_idx_y+height/self.resolution))] = 1

    def add_n_obstacles(self, n: int = 5) -> None:

        for i in range(n):
            x, y = round(random.random()*self.width,
                         1), round(random.random()*self.height, 1)
            self._add_obstacle([x, y])

    def get_wall_trajectory(self) -> np.ndarray:

        forward = [[i, 0] for i in range(
            int(min(self.width, self.height)/self.resolution))]
        self.trajectory = (forward + forward[::-1])*5
        return self.trajectory

    def get_circular_trajectory(self, center: tuple):
        angle = 2*random.random() * math.pi
        r = random.random() * self.max_circle_radius
        # Ensure the circle is entirely within the grid
        r = min(r,
                center[0] - 0, center[1] - 0,
                self.cell_x - center[0], self.cell_y - center[1])
        angles = np.arange(angle, angle + 1.5 * np.pi, 0.2)
        forward = [[int((center[0]-r * np.cos(theta))/self.resolution),
                    int((center[1]-r * np.sin(theta))/self.resolution)] for theta in angles]
        self.trajectory = (forward + forward[::-1])
        return self.trajectory

    def simulate(self, path: list) -> None:
        if not path:
            print(f"No interception possible!")
            return

        for idx, point in enumerate(path[:len(self.trajectory)]):
            if 0 <= point[0] < self.grid.shape[0] and 0 <= point[1] < self.grid.shape[1]:
                self.grid[point[0], point[1]] = MapState.EGO_STATE.value

            if idx < len(self.trajectory) and 0 <= self.trajectory[idx][0] < self.grid.shape[0] and 0 <= self.trajectory[idx][1] < self.grid.shape[1]:
                self.grid[self.trajectory[idx][0], self.trajectory[idx]
                          [1]] = MapState.TARGET_STATE.value

            ego_patch = Patch(color=self.cmap(
                MapState.EGO_STATE.value), label='Ego')
            target_patch = Patch(color=self.cmap(
                MapState.TARGET_STATE.value), label='Target')
            free_patch = Patch(color=self.cmap(
                MapState.FREE.value), label='Free')
            obstacle_patch = Patch(color=self.cmap(
                MapState.OCCUPIED.value), label='Occupied')

            plt.legend(handles=[ego_patch, target_patch, free_patch,
                       obstacle_patch], loc='upper left', bbox_to_anchor=(1.05, 1))
            plt.imshow(self.grid, cmap=self.cmap)
            plt.title('Simulation Time Step: {}'.format(idx))
            plt.xlabel('X Coordinate')
            plt.ylabel('Y Coordinate')
            plt.pause(0.1)

            if 0 <= point[0] < self.grid.shape[0] and 0 <= point[1] < self.grid.shape[1]:
                self.grid[point[0], point[1]] = MapState.FREE.value

            if idx < len(self.trajectory) and 0 <= self.trajectory[idx][0] < self.grid.shape[0] and 0 <= self.trajectory[idx][1] < self.grid.shape[1]:
                self.grid[self.trajectory[idx][0],
                          self.trajectory[idx][1]] = MapState.FREE.value

    def visualize(self) -> None:
        plt.imshow(self.grid)
        plt.show()
