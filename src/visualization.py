"""
Visualization functions for the cart-pole system.

This module only handles drawing. It does not simulate system dynamics
or compute control inputs.

Angle convention:
    theta = 0 means the pole is perfectly upright.
"""

import numpy as np
import matplotlib.pyplot as plt

from src.config import (
    CART_WIDTH,
    CART_HEIGHT,
    POLE_LENGTH,
    X_LIMITS,
    Y_LIMITS,
    GROUND_Y,
)


def draw_cartpole(cart_position, theta, ax):
    """
    Draw the cart-pole for a single cart position and pole angle.

    Args:
        cart_position (float): Horizontal position of the cart center.
        theta (float): Pole angle in radians, where 0 means upright.
        ax (matplotlib.axes.Axes): Matplotlib axes to draw on.
    """
    ax.clear()

    # Draw the ground.
    ax.plot([X_LIMITS[0], X_LIMITS[1]], [GROUND_Y, GROUND_Y], "k-", linewidth=2)

    # Convert cart center position to the rectangle's bottom-left corner.
    cart_left = cart_position - CART_WIDTH / 2
    cart_bottom = GROUND_Y

    cart = plt.Rectangle(
        (cart_left, cart_bottom),
        CART_WIDTH,
        CART_HEIGHT,
        fill=False,
        linewidth=2,
    )
    ax.add_patch(cart)

    pivot_x = cart_position
    pivot_y = cart_bottom + CART_HEIGHT

    tip_x = pivot_x + POLE_LENGTH * np.sin(theta)
    tip_y = pivot_y + POLE_LENGTH * np.cos(theta)

    ax.plot([pivot_x, tip_x], [pivot_y, tip_y], linewidth=3)
    ax.plot(pivot_x, pivot_y, "o", markersize=6)

    ax.set_xlim(*X_LIMITS)
    ax.set_ylim(*Y_LIMITS)
    ax.set_aspect("equal")
    ax.grid(True)

    ax.set_title(
        f"Cart position x={cart_position:.2f}, pole angle theta={theta:.2f} rad"
    )


def draw_cartpole_from_state(state, ax):
    """
    Draw the cart-pole using the full state vector.

    Expected state format:
        state = [x, x_dot, theta, theta_dot]

    Args:
        state (array-like): State vector [x, x_dot, theta, theta_dot].
        ax (matplotlib.axes.Axes): Matplotlib axes to draw on.
    """
    cart_position = state[0]
    theta = state[2]

    draw_cartpole(cart_position, theta, ax)