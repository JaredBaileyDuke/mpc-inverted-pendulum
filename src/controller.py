"""
Linear MPC controller for the cart-pole system.

This version builds the CVXPY problem once and reuses it by updating
parameters at each control step. That avoids repeatedly rebuilding
the optimization problem from scratch.
"""

import time
import cvxpy as cp
import numpy as np

from src.config import (
    N_HORIZON,
    Q_X,
    Q_X_DOT,
    Q_THETA,
    Q_THETA_DOT,
    QF_X,
    QF_X_DOT,
    QF_THETA,
    QF_THETA_DOT,
    R_U,
    R_DELTA_U,
    U_MAX,
    USE_TRACK_HARD_CONSTRAINT,
    X_MAX,
    USE_OPTIONAL_STATE_BOUNDS,
    X_DOT_MAX,
    THETA_MAX,
    THETA_DOT_MAX,
    REFERENCE_STATE,
)


class MPCController:
    """
    Reusable linear MPC controller for the cart-pole system.

    The optimization problem is constructed once during initialization.
    At each solve, only the parameter values are updated.
    """

    def __init__(self, A_d, B_d):
        """
        Initialize the MPC controller.

        Args:
            A_d (np.ndarray): Discrete-time state matrix.
            B_d (np.ndarray): Discrete-time input matrix.
        """
        self.A_d = A_d
        self.B_d = B_d

        self.n_states = 4
        self.n_inputs = 1
        self.horizon = N_HORIZON

        self.x_ref = np.array(REFERENCE_STATE, dtype=float)

        self.Q = np.diag([Q_X, Q_X_DOT, Q_THETA, Q_THETA_DOT])
        self.Qf = np.diag([QF_X, QF_X_DOT, QF_THETA, QF_THETA_DOT])
        self.R = np.array([[R_U]])
        self.R_delta = np.array([[R_DELTA_U]])

        # Parameters that change each solve
        self.current_state_param = cp.Parameter(self.n_states)
        self.previous_u_param = cp.Parameter(self.n_inputs)

        # Decision variables
        self.state = cp.Variable((self.n_states, self.horizon + 1))
        self.control = cp.Variable((self.n_inputs, self.horizon))

        cost = 0
        constraints = []

        # Initial state constraint
        constraints.append(self.state[:, 0] == self.current_state_param)

        for k in range(self.horizon):
            state_error = self.state[:, k] - self.x_ref

            # Standard quadratic tracking cost
            cost += cp.quad_form(state_error, self.Q)
            cost += cp.quad_form(self.control[:, k], self.R)

            # Penalize change in control for smoother motion
            if k == 0:
                delta_u = self.control[:, k] - self.previous_u_param
            else:
                delta_u = self.control[:, k] - self.control[:, k - 1]

            cost += cp.quad_form(delta_u, self.R_delta)

            # System dynamics
            constraints.append(
                self.state[:, k + 1] == self.A_d @ self.state[:, k] + self.B_d @ self.control[:, k]
            )

            # Force magnitude limit
            constraints.append(cp.abs(self.control[:, k]) <= U_MAX)

            # Hard rail safety limit
            if USE_TRACK_HARD_CONSTRAINT:
                constraints.append(cp.abs(self.state[0, k]) <= X_MAX)

            # Optional state limits
            if USE_OPTIONAL_STATE_BOUNDS:
                constraints.append(cp.abs(self.state[1, k]) <= X_DOT_MAX)
                constraints.append(cp.abs(self.state[2, k]) <= THETA_MAX)
                constraints.append(cp.abs(self.state[3, k]) <= THETA_DOT_MAX)

        # Terminal cost
        terminal_error = self.state[:, self.horizon] - self.x_ref
        cost += cp.quad_form(terminal_error, self.Qf)

        # Terminal constraints
        if USE_TRACK_HARD_CONSTRAINT:
            constraints.append(cp.abs(self.state[0, self.horizon]) <= X_MAX)

        if USE_OPTIONAL_STATE_BOUNDS:
            constraints.append(cp.abs(self.state[1, self.horizon]) <= X_DOT_MAX)
            constraints.append(cp.abs(self.state[2, self.horizon]) <= THETA_MAX)
            constraints.append(cp.abs(self.state[3, self.horizon]) <= THETA_DOT_MAX)

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def solve(self, current_state, previous_u=0.0):
        """
        Solve the MPC problem for the current state.

        Args:
            current_state (np.ndarray): Current 4-state vector.
            previous_u (float): Previously applied input.

        Returns:
            tuple:
                u0 (float): First optimal control input.
                predicted_states (np.ndarray): Predicted state trajectory.
                predicted_inputs (np.ndarray): Predicted input trajectory.
                solve_time (float): Time spent solving this MPC problem.
        """
        self.current_state_param.value = np.array(current_state, dtype=float)
        self.previous_u_param.value = np.array([previous_u], dtype=float)

        start_time = time.perf_counter()

        self.problem.solve(
            solver=cp.OSQP,
            warm_start=True,
            verbose=False,
            max_iter=100000,
            eps_abs=1e-5,
            eps_rel=1e-5,
            polish=True,
        )

        if self.problem.status not in ["optimal", "optimal_inaccurate"]:
            self.problem.solve(
                solver=cp.CLARABEL,
                verbose=False,
            )

        end_time = time.perf_counter()
        solve_time = end_time - start_time

        if self.problem.status not in ["optimal", "optimal_inaccurate"]:
            raise RuntimeError(f"MPC solve failed with status: {self.problem.status}")

        u0 = float(self.control[:, 0].value.item())
        predicted_states = self.state.value
        predicted_inputs = self.control.value.flatten()

        return u0, predicted_states, predicted_inputs, solve_time