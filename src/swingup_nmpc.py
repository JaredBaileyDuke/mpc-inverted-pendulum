"""
Nonlinear swing-up MPC for the cart-pole system.

State:
    [x, x_dot, theta, theta_dot]

Angle convention:
    theta = 0 means upright
    theta = pi means hanging straight down
"""

import time
import casadi as ca
import numpy as np

from src.config import (
    M,
    m,
    L,
    I,
    b,
    g,
    SWINGUP_HORIZON,
    SWINGUP_DT,
    SWINGUP_U_MAX,
    SWINGUP_X_MAX,
    SWINGUP_X_DOT_MAX,
    SWINGUP_THETA_DOT_MAX,
    SWINGUP_DELTA_U_MAX,
    SWINGUP_Q_X,
    SWINGUP_Q_X_DOT,
    SWINGUP_Q_THETA,
    SWINGUP_Q_THETA_DOT,
    SWINGUP_R_U,
    SWINGUP_R_DELTA_U,
    SWINGUP_QF_X,
    SWINGUP_QF_X_DOT,
    SWINGUP_QF_THETA,
    SWINGUP_QF_THETA_DOT,
    SWITCH_ANGLE_THRESHOLD,
    SWITCH_THETA_DOT_THRESHOLD,
    SWITCH_X_THRESHOLD,
    SWITCH_X_DOT_THRESHOLD,
)


def wrap_angle(angle):
    """
    Wrap an angle to [-pi, pi].

    Args:
        angle (float): Angle in radians.

    Returns:
        float: Wrapped angle in radians.
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi


def is_near_upright(state):
    """
    Check whether the state is inside the capture region for switching
    to the upright balancing controller later.

    Args:
        state (array-like): [x, x_dot, theta, theta_dot]

    Returns:
        bool: True if the state is near upright.
    """
    x = float(state[0])
    x_dot = float(state[1])
    theta = wrap_angle(float(state[2]))
    theta_dot = float(state[3])

    return (
        abs(theta) <= SWITCH_ANGLE_THRESHOLD
        and abs(theta_dot) <= SWITCH_THETA_DOT_THRESHOLD
        and abs(x) <= SWITCH_X_THRESHOLD
        and abs(x_dot) <= SWITCH_X_DOT_THRESHOLD
    )


class SwingUpNMPC:
    """
    Nonlinear MPC controller for cart-pole swing-up.

    The optimization problem is built once and reused with updated
    parameters and warm-start guesses.
    """

    def __init__(self):
        """Build the nonlinear MPC optimization problem."""
        self.N = SWINGUP_HORIZON
        self.dt = SWINGUP_DT
        self.nx = 4
        self.nu = 1

        self.opti = ca.Opti()

        # Parameters updated every solve
        self.x0_param = self.opti.parameter(self.nx)
        self.u_prev_param = self.opti.parameter(self.nu)

        # Decision variables
        self.X = self.opti.variable(self.nx, self.N + 1)
        self.U = self.opti.variable(self.nu, self.N)

        cost = 0.0

        # Initial state constraint
        self.opti.subject_to(self.X[:, 0] == self.x0_param)

        for k in range(self.N):
            x_k = self.X[:, k]
            u_k = self.U[:, k]

            x_pos = x_k[0]
            x_dot = x_k[1]
            theta = x_k[2]
            theta_dot = x_k[3]

            # 1 - cos(theta) is zero at upright and large near downward.
            theta_cost = 1.0 - ca.cos(theta)

            cost += SWINGUP_Q_X * x_pos**2
            cost += SWINGUP_Q_X_DOT * x_dot**2
            cost += SWINGUP_Q_THETA * theta_cost
            cost += SWINGUP_Q_THETA_DOT * theta_dot**2
            cost += SWINGUP_R_U * u_k[0] ** 2

            if k == 0:
                delta_u = u_k - self.u_prev_param
            else:
                delta_u = u_k - self.U[:, k - 1]

            cost += SWINGUP_R_DELTA_U * delta_u[0] ** 2

            # Nonlinear dynamics constraint
            x_next = self._rk4_step(x_k, u_k[0])
            self.opti.subject_to(self.X[:, k + 1] == x_next)

            # Hard constraints
            self.opti.subject_to(
                self.opti.bounded(-SWINGUP_U_MAX, u_k[0], SWINGUP_U_MAX)
            )
            self.opti.subject_to(
                self.opti.bounded(-SWINGUP_X_MAX, x_pos, SWINGUP_X_MAX)
            )
            self.opti.subject_to(
                self.opti.bounded(-SWINGUP_X_DOT_MAX, x_dot, SWINGUP_X_DOT_MAX)
            )
            self.opti.subject_to(
                self.opti.bounded(
                    -SWINGUP_THETA_DOT_MAX, theta_dot, SWINGUP_THETA_DOT_MAX
                )
            )
            self.opti.subject_to(
                self.opti.bounded(
                    -SWINGUP_DELTA_U_MAX,
                    delta_u[0],
                    SWINGUP_DELTA_U_MAX,
                )
            )

        # Terminal cost
        x_N = self.X[:, self.N]
        x_pos_N = x_N[0]
        x_dot_N = x_N[1]
        theta_N = x_N[2]
        theta_dot_N = x_N[3]

        terminal_theta_cost = 1.0 - ca.cos(theta_N)

        cost += SWINGUP_QF_X * x_pos_N**2
        cost += SWINGUP_QF_X_DOT * x_dot_N**2
        cost += SWINGUP_QF_THETA * terminal_theta_cost
        cost += SWINGUP_QF_THETA_DOT * theta_dot_N**2

        # Terminal hard constraints
        self.opti.subject_to(
            self.opti.bounded(-SWINGUP_X_MAX, x_pos_N, SWINGUP_X_MAX)
        )
        self.opti.subject_to(
            self.opti.bounded(-SWINGUP_X_DOT_MAX, x_dot_N, SWINGUP_X_DOT_MAX)
        )
        self.opti.subject_to(
            self.opti.bounded(
                -SWINGUP_THETA_DOT_MAX, theta_dot_N, SWINGUP_THETA_DOT_MAX
            )
        )

        self.opti.minimize(cost)

        solver_options = {
            "expand": True,
            "print_time": False,
            "ipopt": {
                "print_level": 0,
                "max_iter": 500,
                "tol": 1e-4,
                "acceptable_tol": 1e-3,
                "acceptable_iter": 5,
                "sb": "yes",
            },
        }
        self.opti.solver("ipopt", solver_options)

        self.last_X_guess = None
        self.last_U_guess = None

    def _continuous_dynamics(self, x, u):
        """
        Nonlinear cart-pole dynamics.

        Args:
            x (casadi.MX): State [x, x_dot, theta, theta_dot]
            u (casadi.MX): Scalar input force

        Returns:
            casadi.MX: State derivative
        """
        x_dot = x[1]
        theta = x[2]
        theta_dot = x[3]

        cos_theta = ca.cos(theta)
        sin_theta = ca.sin(theta)

        denom = (M + m) * (I + m * L**2) - (m * L * cos_theta) ** 2

        rhs1 = u - b * x_dot - m * L * theta_dot**2 * sin_theta
        rhs2 = m * g * L * sin_theta

        x_ddot = ((I + m * L**2) * rhs1 + m * L * cos_theta * rhs2) / denom
        theta_ddot = ((M + m) * rhs2 + m * L * cos_theta * rhs1) / denom

        return ca.vertcat(x_dot, x_ddot, theta_dot, theta_ddot)

    def _rk4_step(self, x, u):
        """
        One RK4 integration step.

        Args:
            x (casadi.MX): Current state
            u (casadi.MX): Current control input

        Returns:
            casadi.MX: Next state
        """
        k1 = self._continuous_dynamics(x, u)
        k2 = self._continuous_dynamics(x + 0.5 * self.dt * k1, u)
        k3 = self._continuous_dynamics(x + 0.5 * self.dt * k2, u)
        k4 = self._continuous_dynamics(x + self.dt * k3, u)

        return x + (self.dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def solve(self, current_state, previous_u=0.0):
        """
        Solve the nonlinear swing-up MPC problem.

        Args:
            current_state (array-like): Current state [x, x_dot, theta, theta_dot]
            previous_u (float): Previously applied control input

        Returns:
            tuple:
                u0 (float): First optimal control input
                predicted_states (np.ndarray): Predicted state trajectory
                predicted_inputs (np.ndarray): Predicted input trajectory
                solve_time (float): Solver runtime in seconds
        """
        current_state = np.asarray(current_state, dtype=float).reshape(-1)
        previous_u = float(previous_u)

        self.opti.set_value(self.x0_param, current_state)
        self.opti.set_value(self.u_prev_param, np.array([previous_u], dtype=float))

        if self.last_X_guess is not None:
            self.opti.set_initial(self.X, self.last_X_guess)
        if self.last_U_guess is not None:
            self.opti.set_initial(self.U, self.last_U_guess)
        else:
            X_guess = np.tile(current_state.reshape(-1, 1), (1, self.N + 1))
            U_guess = np.zeros((1, self.N))
            self.opti.set_initial(self.X, X_guess)
            self.opti.set_initial(self.U, U_guess)

        start_time = time.perf_counter()

        # Allow return at iteration/time limit without throwing immediately
        try:
            sol = self.opti.solve_limited()
            X_sol = np.array(sol.value(self.X), dtype=float)
            U_sol = np.array(sol.value(self.U), dtype=float)
        except Exception:
            # Fall back to the latest debug iterate
            X_sol = np.array(self.opti.debug.value(self.X), dtype=float)
            U_sol = np.array(self.opti.debug.value(self.U), dtype=float)

        end_time = time.perf_counter()
        solve_time = end_time - start_time

        if U_sol.ndim == 1:
            U_sol = U_sol.reshape(1, -1)

        # Basic sanity check
        if X_sol.size == 0 or U_sol.size == 0 or np.isnan(X_sol).any() or np.isnan(U_sol).any():
            raise RuntimeError("Swing-up NMPC did not return a usable iterate.")

        self.last_X_guess = np.hstack([X_sol[:, 1:], X_sol[:, -1:]])
        self.last_U_guess = np.hstack([U_sol[:, 1:], U_sol[:, -1:]])

        u0 = float(U_sol[0, 0])
        predicted_states = X_sol
        predicted_inputs = U_sol.flatten()

        return u0, predicted_states, predicted_inputs, solve_time