"""
Main entry point for the cart-pole hybrid controller demo.

This version runs:
- nonlinear swing-up MPC from a dangling start
- then switches to linear upright MPC near the top

The plant is nonlinear for the whole simulation.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from src.simulation import simulate_hybrid_control
from src.visualization import draw_cartpole_from_state


def main():
    """
    Run the hybrid cart-pole demo.
    """
    # Start slightly off the exact downward equilibrium.
    initial_state = [0.0, 0.0, np.pi - 0.15, 0.0]

    time_vector, state_history, input_history, mode_history = simulate_hybrid_control(
        initial_state
    )

    # Plot states
    fig1, ax1 = plt.subplots()
    ax1.plot(time_vector, state_history[:, 0], label="x")
    ax1.plot(time_vector, state_history[:, 1], label="x_dot")
    ax1.plot(time_vector, state_history[:, 2], label="theta")
    ax1.plot(time_vector, state_history[:, 3], label="theta_dot")
    ax1.set_title("Hybrid Controller State History")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("State value")
    ax1.grid(True)
    ax1.legend()

    # Plot control input
    fig2, ax2 = plt.subplots()
    if len(input_history) > 0:
        ax2.plot(time_vector[:-1], input_history)
    ax2.set_title("Hybrid Controller Input Force History")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Force (N)")
    ax2.grid(True)

    # Plot controller mode
    fig3, ax3 = plt.subplots()
    if len(mode_history) > 0:
        mode_numeric = [0 if mode == "swingup" else 1 for mode in mode_history]
        ax3.plot(time_vector[:-1], mode_numeric)
    ax3.set_title("Controller Mode")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Mode")
    ax3.set_yticks([0, 1])
    ax3.set_yticklabels(["swingup", "upright"])
    ax3.grid(True)

    # Animate
    fig4, ax4 = plt.subplots(figsize=(8, 4))

    def update(frame_index):
        draw_cartpole_from_state(state_history[frame_index], ax4)

    ani = FuncAnimation(
        fig4,
        update,
        frames=len(state_history),
        interval=20,
        repeat=False,
    )

    _ = ani
    plt.show()


if __name__ == "__main__":
    main()