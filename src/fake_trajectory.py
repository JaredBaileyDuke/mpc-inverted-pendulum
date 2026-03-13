"""
Generate a fake cart-pole trajectory for visualization testing.

This module does not simulate real cart-pole physics. It only creates
smooth example motion so the drawing and animation code can be tested
before adding real system dynamics.
"""

import numpy as np


def generate_fake_trajectory():
    """
    Create a fake trajectory for cart position and pole angle.

    Returns:
        tuple:
            t (np.ndarray): Time vector.
            cart_position_traj (np.ndarray): Cart position over time.
            theta_traj (np.ndarray): Pole angle over time.
    """
    time_vector = np.linspace(0, 10, 200)

    # Example motion:
    # - the cart moves left and right smoothly
    # - the pole angle oscillates independently
    cart_position_traj = 0.8 * np.sin(time_vector)
    theta_traj = 0.3 * np.cos(2 * time_vector)

    return time_vector, cart_position_traj, theta_traj