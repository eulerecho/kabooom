import numpy as np
from enum import Enum
import random
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

class MapState(Enum):
    FREE = 0
    OCCUPIED = 1
    TARGET_STATE = 2
    EGO_STATE = 3


class Map:

    def __init__(self,resolution = 1, width = 5, height = 5, obstacle_scale = [0.1,0.25]) -> None:

        self.width, self.height  = width, height
        self.resolution = resolution
        self.max_obstacle_width, self.max_obstacle_height = self.width*obstacle_scale[1], self.height*obstacle_scale[1]
        self.min_obstacle_width, self.min_obstacle_height = self.width*obstacle_scale[0], self.height*obstacle_scale[0]
        self.cell_x = int(self.width/self.resolution)
        self.cell_y = int(self.height/self.resolution)
        self.grid = np.zeros((self.cell_x,self.cell_y))
        self.trajectory = None
        self.colors = ['white', 'black', 'red', 'blue']
        self.cmap = mcolors.ListedColormap(self.colors)

    # def _add_obstacle(self,start: list[int]) -> None:
        
    #     start_idx_x, start_idx_y = int(start[0]/self.resolution),int(start[1]/self.resolution)
    #     width = max(self.min_obstacle_width, random.random()*self.max_obstacle_width)
    #     height = max(self.min_obstacle_height, random.random()*self.max_obstacle_height)

    #     self.grid[start_idx_x:min(self.cell_x,int(start_idx_x+width/self.resolution)),
    #               start_idx_y:min(self.cell_y,int(start_idx_y+height/self.resolution))] =1 
    
    # def add_n_obstacles(self,n: int =5) -> None:

    #     for i in range(n):
    #         x,y = round(random.random()*self.width,1),round(random.random()*self.height,1)
    #         self._add_obstacle([x,y])

    def get_fake_trajectory(self,start:list[int],end:list[int]) -> None:
        
        forward = [[0,i] for i in range(min(self.width,self.height))]
        # self.trajectory = (forward + forward[::-1])*5
        self.trajectory = forward
        return self.trajectory

    
    def plot_trajectory(self):
        
        for point in self.trajectory:
            self.grid[point[0],point[1]]=1
            plt.imshow(self.grid)
            # plt.draw()  # Redraw the plot
            plt.pause(1)  # Pause for 1 second
            self.grid[point[0],point[1]]=0

    def simulate(self,path:list) -> None:
        idx= 0
        for point in path:
            self.grid[point[0],point[1]]= MapState.EGO_STATE.value
            self.grid[self.trajectory[idx][0],self.trajectory[idx][1]] = MapState.TARGET_STATE.value
            plt.imshow(self.grid,cmap=self.cmap)
            # plt.draw()  # Redraw the plot
            plt.pause(1)  # Pause for 1 second
            self.grid[point[0],point[1]] = MapState.FREE.value
            self.grid[self.trajectory[idx][0],self.trajectory[idx][1]]= MapState.FREE.value
            idx+=1

    def visualize(self) -> None:
        plt.imshow(self.grid)
        plt.show()



        



    

        