"""
Closed-loop simulation utilities for the cart-pole hybrid controller demo.

This version uses:
- nonlinear swing-up MPC when the pole is far from upright
- linear upright MPC when the pole is near upright

The plant is kept nonlinear for the entire simulation.

Important timing design:
- plant integrates every DT
- upright linear MPC solves every DT
- swing-up NMPC solves every SWINGUP_CONTROL_INTERVAL_STEPS * DT
  and holds its last control in between solves
"""

import numpy as np

from src.config import (
    DT,
    SIM_TIME,
    X_MAX,
    SWITCH_RETRY_COOLDOWN_STEPS,
    USE_SWINGUP_OVERRUN_GUARD,
    SWINGUP_CONTROL_INTERVAL_STEPS,
    SWINGUP_FALLBACK_KX,
    SWINGUP_FALLBACK_KV,
    U_MAX,
)
from src.controller import MPCController
from src.model import get_discrete_state_space
from src.swingup_nmpc import SwingUpNMPC, is_near_upright, wrap_angle

# Physical parameters for the nonlinear plant
from src.config import M, m, L, I, b, g


def nonlinear_cartpole_dynamics(state, u):
    """
    Nonlinear cart-pole dynamics.

    Args:
        state (np.ndarray): [x, x_dot, theta, theta_dot]
        u (float): Horizontal force applied to the cart

    Returns:
        np.ndarray: State derivative
    """
    x_dot = state[1]
    theta = state[2]
    theta_dot = state[3]

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    denom = (M + m) * (I + m * L**2) - (m * L * cos_theta) ** 2

    rhs1 = u - b * x_dot - m * L * theta_dot**2 * sin_theta
    rhs2 = m * g * L * sin_theta

    x_ddot = ((I + m * L**2) * rhs1 + m * L * cos_theta * rhs2) / denom
    theta_ddot = ((M + m) * rhs2 + m * L * cos_theta * rhs1) / denom

    return np.array([x_dot, x_ddot, theta_dot, theta_ddot], dtype=float)


def rk4_step(state, u, dt):
    """
    One RK4 integration step for the nonlinear plant.

    Args:
        state (np.ndarray): Current state
        u (float): Control input
        dt (float): Time step

    Returns:
        np.ndarray: Next state
    """
    k1 = nonlinear_cartpole_dynamics(state, u)
    k2 = nonlinear_cartpole_dynamics(state + 0.5 * dt * k1, u)
    k3 = nonlinear_cartpole_dynamics(state + 0.5 * dt * k2, u)
    k4 = nonlinear_cartpole_dynamics(state + dt * k3, u)

    next_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    next_state[2] = wrap_angle(next_state[2])

    return next_state


def get_upright_mpc_state(state):
    """
    Convert the nonlinear plant state into the representation expected by
    the linear upright MPC.

    Args:
        state (np.ndarray): Nonlinear plant state [x, x_dot, theta, theta_dot]

    Returns:
        np.ndarray: State formatted for the upright linear MPC
    """
    upright_state = np.array(state, dtype=float).copy()
    upright_state[2] = wrap_angle(upright_state[2])
    return upright_state


def safe_swingup_fallback_control(state):
    """
    Simple backup control used only if swing-up NMPC overruns.

    This is a cart-centering / braking action, not a full swing-up controller.
    Its job is to avoid rail violations when the NMPC solve is late.

    Args:
        state (np.ndarray): Current state [x, x_dot, theta, theta_dot]

    Returns:
        float: Safe fallback force command
    """
    x = float(state[0])
    x_dot = float(state[1])

    u = -SWINGUP_FALLBACK_KX * x - SWINGUP_FALLBACK_KV * x_dot
    u = float(np.clip(u, -U_MAX, U_MAX))
    return u


def simulate_hybrid_control(initial_state):
    """
    Simulate the cart-pole using a hybrid control architecture:
    - nonlinear swing-up MPC far from upright
    - linear upright MPC near upright

    Returns:
        tuple:
            time_vector (np.ndarray)
            state_history (np.ndarray)
            input_history (np.ndarray)
            mode_history (list[str])
    """
    num_steps = int(SIM_TIME / DT)
    current_state = np.array(initial_state, dtype=float)
    current_state[2] = wrap_angle(current_state[2])

    state_history = [current_state.copy()]
    input_history = []
    solve_times = []
    mode_history = []
    previous_u = 0.0
    swingup_overrun_count = 0
    swingup_fallback_count = 0

    # Swing-up solve timing / hold behavior
    last_swingup_u = 0.0
    last_swingup_solve_time = 0.0
    swingup_hold_count = 0

    # Build both controllers once
    swingup_controller = SwingUpNMPC()
    A_d, B_d = get_discrete_state_space()
    upright_controller = MPCController(A_d, B_d)

    # Warm up both controllers
    print("Running warm-up solves...")
    try:
        _, _, _, swingup_warmup_time = swingup_controller.solve(
            current_state=current_state,
            previous_u=previous_u,
        )
        _, _, _, upright_warmup_time = upright_controller.solve(
            current_state=get_upright_mpc_state(current_state),
            previous_u=previous_u,
        )
    except Exception as exc:
        raise RuntimeError(f"Warm-up solve failed: {exc}") from exc

    print(f"Swing-up warm-up solve time: {swingup_warmup_time:.6f} s")
    print(f"Upright warm-up solve time:  {upright_warmup_time:.6f} s\n")

    current_mode = "swingup"
    switch_cooldown = 0

    for step_index in range(num_steps):
        if abs(current_state[0]) > X_MAX:
            print(f"Stopping simulation: cart exceeded rail at step {step_index}.")
            break

        if switch_cooldown > 0:
            switch_cooldown -= 1

        used_overrun_guard = False
        used_fallback = False
        held_previous_swingup = False

        # Try handoff from swing-up to upright
        if (
            current_mode == "swingup"
            and switch_cooldown == 0
            and is_near_upright(current_state)
        ):
            wrapped_theta = wrap_angle(current_state[2])

            print(
                f"Attempting switch to upright MPC at step {step_index}. "
                f"state = [x={current_state[0]:.4f}, "
                f"x_dot={current_state[1]:.4f}, "
                f"theta_raw={current_state[2]:.4f}, "
                f"theta_wrapped={wrapped_theta:.4f}, "
                f"theta_dot={current_state[3]:.4f}]"
            )

            try:
                u0, _, _, solve_time = upright_controller.solve(
                    current_state=get_upright_mpc_state(current_state),
                    previous_u=previous_u,
                )

                current_mode = "upright"
                print(f"Switch successful at step {step_index}.")

            except Exception as exc:
                print(
                    f"Switch rejected at step {step_index}: upright MPC infeasible "
                    f"from current state ({exc}). Staying in swing-up mode."
                )
                switch_cooldown = SWITCH_RETRY_COOLDOWN_STEPS

                if step_index % SWINGUP_CONTROL_INTERVAL_STEPS == 0:
                    try:
                        u_candidate, _, _, solve_time = swingup_controller.solve(
                            current_state=current_state,
                            previous_u=previous_u,
                        )
                    except Exception as solve_exc:
                        print(
                            f"Stopping simulation: swingup controller solve failed "
                            f"at step {step_index}: {solve_exc}"
                        )
                        break

                    if USE_SWINGUP_OVERRUN_GUARD and solve_time > DT:
                        u0 = safe_swingup_fallback_control(current_state)
                        used_overrun_guard = True
                        used_fallback = True
                        swingup_overrun_count += 1
                        swingup_fallback_count += 1
                    else:
                        u0 = u_candidate
                        last_swingup_u = u_candidate
                        last_swingup_solve_time = solve_time
                else:
                    u0 = last_swingup_u
                    solve_time = last_swingup_solve_time
                    held_previous_swingup = True
                    swingup_hold_count += 1

        else:
            try:
                if current_mode == "swingup":
                    # Only solve swing-up NMPC every few plant steps.
                    if step_index % SWINGUP_CONTROL_INTERVAL_STEPS == 0:
                        u_candidate, _, _, solve_time = swingup_controller.solve(
                            current_state=current_state,
                            previous_u=previous_u,
                        )

                        if USE_SWINGUP_OVERRUN_GUARD and solve_time > DT:
                            u0 = safe_swingup_fallback_control(current_state)
                            used_overrun_guard = True
                            used_fallback = True
                            swingup_overrun_count += 1
                            swingup_fallback_count += 1
                        else:
                            u0 = u_candidate
                            last_swingup_u = u_candidate
                            last_swingup_solve_time = solve_time
                    else:
                        u0 = last_swingup_u
                        solve_time = last_swingup_solve_time
                        held_previous_swingup = True
                        swingup_hold_count += 1

                else:
                    u0, _, _, solve_time = upright_controller.solve(
                        current_state=get_upright_mpc_state(current_state),
                        previous_u=previous_u,
                    )

            except Exception as exc:
                print(
                    f"Stopping simulation: {current_mode} controller solve failed "
                    f"at step {step_index}: {exc}"
                )
                break

        solve_times.append(solve_time)
        mode_history.append(current_mode)

        extra_flags = []
        if used_overrun_guard:
            extra_flags.append("overrun_guard=ON")
        if used_fallback:
            extra_flags.append("fallback_centering=ON")
        if held_previous_swingup:
            extra_flags.append("swingup_hold=ON")

        extra_text = ""
        if extra_flags:
            extra_text = " | " + " | ".join(extra_flags)

        print(
            f"Step {step_index:03d} | mode={current_mode:7s} | "
            f"solve time: {solve_time:.6f} s{extra_text}"
        )

        next_state = rk4_step(current_state, u0, DT)

        input_history.append(u0)
        state_history.append(next_state.copy())

        current_state = next_state
        previous_u = u0

    if solve_times:
        solve_times_array = np.array(solve_times)

        print("\nSolve time summary:")
        print(f"  Average solve (all loop): {np.mean(solve_times_array):.6f} s")
        print(f"  Max solve (all loop):     {np.max(solve_times_array):.6f} s")

        if len(solve_times_array) > 5:
            steady_state_times = solve_times_array[5:]
            print(f"  Average solve (after 5):  {np.mean(steady_state_times):.6f} s")
            print(f"  Max solve (after 5):      {np.max(steady_state_times):.6f} s")

        swingup_count = sum(mode == "swingup" for mode in mode_history)
        upright_count = sum(mode == "upright" for mode in mode_history)
        print(f"  Swing-up steps:           {swingup_count}")
        print(f"  Upright steps:            {upright_count}")
        print(f"  Swing-up overrun guards:  {swingup_overrun_count}")
        print(f"  Swing-up fallback uses:   {swingup_fallback_count}")
        print(f"  Swing-up held inputs:     {swingup_hold_count}")

        swingup_times = [t for t, mode in zip(solve_times, mode_history) if mode == "swingup"]
        upright_times = [t for t, mode in zip(solve_times, mode_history) if mode == "upright"]

        if swingup_times:
            swingup_times = np.array(swingup_times)
            print(f"  Swing-up avg solve:       {np.mean(swingup_times):.6f} s")
            print(f"  Swing-up max solve:       {np.max(swingup_times):.6f} s")

        if upright_times:
            upright_times = np.array(upright_times)
            print(f"  Upright avg solve:        {np.mean(upright_times):.6f} s")
            print(f"  Upright max solve:        {np.max(upright_times):.6f} s")

    state_history = np.array(state_history)
    input_history = np.array(input_history)
    time_vector = np.arange(len(state_history)) * DT

    return time_vector, state_history, input_history, mode_history