from casadi import *
import numpy as np

class TrajectoryOptimizer:
    """!
    @brief A Trajectory Optimization class for a unicycle model that tries to minimize the time to reach
    a goal given by a target whose trajectory is known

    @param n_states: The number of states in the system.
    @param n_control: The number of control inputs in the system.
    @param N: The number of control intervals.
    """
    def __init__(self, n_states, n_control,N):

        self.n_states = n_states
        self.n_control = n_control
        self.N = N
        self.f = self._build_dynamics_func()
        self.F = self._rk4_integrator()
        self.M = self._transcribe_non_linear_program(N)

    def _build_dynamics_func(self):
        """!
        @brief Buildiing up the state transition
        @returns The dynamics function.
        """
        x = MX.sym("x")
        y = MX.sym("y")
        theta = MX.sym("theta")
        state = vertcat(x, y, theta)
        v = MX.sym("v")
        w = MX.sym("w")
        U = vertcat(v,w)

        ode = vertcat(v * cos(theta), v * sin(theta), w)
        f = Function('f', [state, U], [ode], ['X', 'U'], ['X_next'])
        return f

    def _rk4_integrator(self):
        """!
        @brief Implements the Runge-Kutta 4 (RK4) integration method for the state transition.
        @param self: The object pointer.
        @returns F: A CasADi function that represents the integration of the state over one time segment.
        """
        M = 4
        X0 = MX.sym('X0', self.n_states, 1)
        U = MX.sym('U', 2, 1)
        DT = MX.sym('DT')  # Duration of each segment
        
        X_next = X0
        for j in range(M):
            k1 = self.f(X_next, U)
            k2 = self.f(X_next + DT / 2 * k1, U)
            k3 = self.f(X_next + DT / 2 * k2, U)
            k4 = self.f(X_next + DT * k3, U)
            X_next = X_next + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        
        F = Function('F', [X0, U, DT], [X_next])
        return F

    def _transcribe_non_linear_program(self, N):
        """!
        @brief Transcribes the optimal control problem into a nonlinear programming problem.
        @param self: The object pointer.
        @param N: Integer, the number of discretization steps.
        @returns M: A CasADi function that represents the solution of the transcribed nonlinear program, 
                    returning the optimal control input sequence and the total time.
        """
        opti = Opti()
        
        # Decision variables
        X = opti.variable(3, N + 1)
        u = opti.variable(2, N)
        T = opti.variable()  # Total time
        
        # Parameters
        X_0 = opti.parameter(3, 1)
        TargetTrajectory = opti.parameter(3, N + 1)
        
        # Objective: Minimize total time
        opti.minimize(T)
        
        # Dynamics using RK4
        DT = T / N  # Duration of each time segment
        for k in range(N):
            X_next = self.F(X[:, k], u[:, k], DT)
            opti.subject_to(X[:, k + 1] == X_next)
        
        # Constraints
        opti.subject_to(X[:, 0] == X_0)
        opti.subject_to(X[:, N] == TargetTrajectory[:, N])  # Final state should match target
        opti.subject_to(opti.bounded(-0.5, u[0, :], 0.5))
        opti.subject_to(opti.bounded(-0.3, u[1, :], 0.3))
        opti.subject_to(T >= 0)  # Time should be positive
        
        # Solver options
        opts = {
            "qpsol": 'qrqp',
            "print_header": False,
            "print_iteration": False,
            "print_time": False,
            "print_status": False,
            "qpsol_options": {"print_iter": False, "print_header": False, "print_info": False}
        }
        opti.solver('sqpmethod', opts)
        M = opti.to_function('M', [X_0, TargetTrajectory], [u, T])
        
        return M

    def get_control(self, X0, ref):
        """!
        @brief Determines the optimal control inputs for the system by solving the underlying optimization problem.
        @param X0: The current state of the system.
        @param ref: The reference trajectory for the system.
        @return The optimal control inputs for the system and the total time T.
        """
        optimal_u, optimal_T = self.M(X0, ref).full()
        return np.array(optimal_u), float(optimal_T)

