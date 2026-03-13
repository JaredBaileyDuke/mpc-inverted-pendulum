"""
Model construction for the linearized cart-pole system.

This module builds:
- continuous-time state-space matrices A_c, B_c
- discrete-time matrices A_d, B_d using scipy.signal.cont2discrete
"""

import numpy as np
from scipy.signal import cont2discrete

from src.config import M, m, L, I, b, g, DT


def get_continuous_state_space():
    """
    Build the continuous-time linearized cart-pole model:

        x_dot = A_c x + B_c u

    with state:
        x = [x, x_dot, theta, theta_dot]^T

    Returns:
        tuple:
            A_c (np.ndarray): 4x4 continuous-time state matrix
            B_c (np.ndarray): 4x1 continuous-time input matrix
    """
    denominator = I * (M + m) + M * m * L**2

    A_c = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, -((I + m * L**2) * b) / denominator, (m**2 * g * L**2) / denominator, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, -(m * L * b) / denominator, (m * g * L * (M + m)) / denominator, 0.0],
    ])

    B_c = np.array([
        [0.0],
        [(I + m * L**2) / denominator],
        [0.0],
        [(m * L) / denominator],
    ])

    return A_c, B_c


def get_discrete_state_space():
    """
    Discretize the continuous-time model using zero-order hold.

    Returns:
        tuple:
            A_d (np.ndarray): 4x4 discrete-time state matrix
            B_d (np.ndarray): 4x1 discrete-time input matrix
    """
    A_c, B_c = get_continuous_state_space()

    C_c = np.eye(4)
    D_c = np.zeros((4, 1))

    A_d, B_d, _, _, _ = cont2discrete((A_c, B_c, C_c, D_c), DT)

    return A_d, B_d