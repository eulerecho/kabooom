# Kabooom!!

What happens when a missile is crusing its way, and then out of nowhere the interceptor smashes it in mid air, you guessed it right! Kaboom!! 

This repo is a 2D discritezd toy **search-and-destroy problem**, where given an agent's trajectory, the goal of the ego agent is to intercept it at minimal time. 


## Approaches

1. **Graph Based Search**: The idea here is to model the state as the target and ego pose together with time. And essentially doing a search in time, interception is done when at any state the ego pose is equal to the target pose and we seek to minimize this time.
2. **Optimal Control Approach**: The idea here is to write the system dynamics, constraits on controls and states and formulate a optimal control problem to minimize the time, while respecting the boundry conditions. On a 2D grid this can be solved as Reinforcement Learning problem through Value/Policy Iteration, but a general problem formulation has been attempted so that the structure can posibly be used in other dynamic system as well. 

## Project Structure

Three directories live in the repo:

- `kabooom_py`: Implementation of a state-based-search `bfs.py`, and another efficient hash-based Dijsktra and BFS `fox_travelers.py`. The main function generates random grids and the algorithm tries to find the solution
- `kabooom_cpp`: CPP implementation of the state-based BFS, unit tests included.
- `kabooom_py`: An initial attempt to formulate the minimum time problem as a optimal control problem using CaSaDi (WIP. solution incomplete, the solver segfaults!)


## Code Execution

## Python (kabooom_py)

Options

- `num_iterations`: (int, default=5):Number of different maps to run the simulation.
- `circle`: Include this flag to draw a circular trajectory.
- `num_obstacles`: (int, default=20):Number of obstacles to add to the map.
- `map_width`: (int, default=5):Width of the map.
- `map_height`: (int, default=5):Height of the map.
- `map_resolution`: (float, default=0.1):Resolution of the map.
- `algorithm`: ('dijkstra' or 'bfs', default='dijkstra'):Path planning algorithm to use.

```bash
python3 main.py [options]
```

## C++ (kabooom_cpp)
```bash
cd kabooom_cpp && mkdir build && cd build
cmake ..
make
./kabooom
ctest -V
```

## Future Work

Need to figure out figure out why the solver is not happy with the formulation.

