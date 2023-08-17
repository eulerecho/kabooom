import numpy as np

from trajopt import TrajectoryOptimizer

def main(control_params: dict) -> None:

    controller = TrajectoryOptimizer(*control_params.values())
    # Initial state of the robot
    state = np.array([0,0,0]).reshape(control_params['n_states'], 1)

    # Number of control intervals
    N = control_params['N']

    # Reference trajectory
    x_trajectory = np.linspace(0, 10, N + 1)  # x from 0 to 10, with N+1 points
    y_trajectory = np.full(N + 1, 2)          # y is constant at 2
    theta_trajectory = np.full(N + 1, 0)      # theta (orientation) is constant at 0 radians

    track_ref = np.vstack((x_trajectory, y_trajectory, theta_trajectory))

    control,time = controller.get_control(state, track_ref)

if __name__ == "__main__":

    control_params = {"n_states": 3, "n_control": 2, "N": 10}  
    main(control_params)
