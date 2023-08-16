from grid import Map 
from bfs import *

a = Map()
trajectory = a.get_fake_trajectory([],[])

start = [3,3]
path = bfs(start,trajectory,a)
a.simulate(path)