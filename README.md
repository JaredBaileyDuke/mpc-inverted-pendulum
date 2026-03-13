# Cart-Pole Hybrid MPC

This project simulates a cart-pole system with a **hybrid control architecture**:

- **Nonlinear MPC (NMPC)** for swing-up from the hanging position
- **Linear MPC** for balancing near the upright equilibrium

The plant is simulated with the **nonlinear cart-pole dynamics** throughout the entire run.

## Overview

The cart-pole starts near the downward hanging position. The controller first uses a nonlinear swing-up MPC to bring the pendulum toward the upright position while respecting rail and input constraints. Once the pendulum enters a capture region near upright, control is handed off to a linear MPC designed around the upright equilibrium.

This gives the project two strengths:

- the swing-up phase can handle large nonlinear motion
- the upright phase can use a faster, simpler controller near the linearization point

## Features

- Nonlinear cart-pole simulation using RK4 integration
- Nonlinear swing-up MPC with CasADi + IPOPT
- Linear upright MPC with CVXPY + OSQP
- Automatic switching between swing-up and upright controllers
- Rail safety constraint
- Input magnitude constraint
- Input-rate limits during swing-up
- Overrun safeguard for slow swing-up solves
- Matplotlib plots and animation

## Project Structure

```text
.
├── main.py
└── src/
    ├── __init__.py
    ├── config.py
    ├── controller.py
    ├── model.py
    ├── simulation.py
    ├── swingup_nmpc.py
    └── visualization.py
```

## File Descriptions

### `main.py`
Runs the full hybrid simulation, generates plots, and animates the cart-pole.

### `src/config.py`
Stores:
- physical parameters
- timing parameters
- linear MPC tuning
- nonlinear swing-up MPC tuning
- switching thresholds
- fallback safety gains
- visualization settings

### `src/model.py`
Builds the **linearized** cart-pole model around the upright equilibrium and discretizes it for the upright MPC.

### `src/controller.py`
Contains the reusable **linear upright MPC controller** built with CVXPY.

### `src/swingup_nmpc.py`
Contains the **nonlinear swing-up MPC controller** built with CasADi.

### `src/simulation.py`
Runs the closed-loop simulation using:
- nonlinear plant dynamics
- swing-up NMPC
- upright linear MPC
- switching logic
- overrun handling

### `src/visualization.py`
Draws and animates the cart-pole.

## Controller Architecture

### 1. Swing-up controller
The swing-up controller is a **nonlinear MPC**. It is used when the pendulum is far from upright.

Its purpose is to:
- inject energy into the system
- move the pole toward upright
- respect cart rail and input limits

The swing-up MPC runs more slowly than the plant simulation and may hold its previous input between solve steps.

### 2. Upright controller
Once the pole is near upright, the controller switches to a **linear MPC** based on a linearization around:

```math
[x,\ \dot{x},\ \theta,\ \dot{\theta}] = [0,\ 0,\ 0,\ 0]
```

This controller is intended for:
- small-angle balancing
- fast solve times
- constrained stabilization near upright

### 3. Hybrid switching
The simulation checks whether the current state is inside a capture region near upright. If it is, the code attempts to hand off control to the linear upright MPC.

If the upright MPC is infeasible from that state, the switch is rejected and the nonlinear swing-up controller remains active.

## Timing Design

The project uses two different effective controller rates:

- **Plant simulation** runs at `DT`
- **Upright linear MPC** solves every plant step
- **Swing-up NMPC** solves every few plant steps and holds the last control between solves

This is intentional:
- balancing requires faster updates
- swing-up NMPC is more computationally expensive

### Overrun safeguard
If a swing-up NMPC solve takes too long, the code does **not** apply a stale delayed solution. Instead, it uses a fallback centering/braking action:

```math
u = -K_x x - K_v \dot{x}
```

clipped to the allowed input range.

This fallback is designed to reduce rail-risk during slow solves.

## State Definition

The state vector is:

```math
x =
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
```

where:

- `x` = cart position
- `x_dot` = cart velocity
- `theta` = pole angle
- `theta_dot` = pole angular velocity

### Angle convention
This project uses:

- `theta = 0` → pole upright
- `theta = \pi` → pole hanging downward

Angles are wrapped when needed so the upright controller always sees upright as being near `theta = 0`.

## Requirements

Install dependencies in your virtual environment:

```bash
pip install numpy scipy matplotlib cvxpy osqp casadi
```

## How to Run

From the project root:

```bash
python main.py
```

The program will:

1. warm up the controllers
2. run the hybrid simulation
3. print solve-time information
4. plot states and inputs
5. animate the cart-pole motion

## Important Parameters

Most tuning happens in `src/config.py`.

### Shared timing
- `DT` — plant simulation step
- `SIM_TIME` — total simulation duration

### Upright MPC
- `N_HORIZON`
- `Q_X`, `Q_X_DOT`, `Q_THETA`, `Q_THETA_DOT`
- `R_U`, `R_DELTA_U`
- `QF_*`

### Swing-up NMPC
- `SWINGUP_HORIZON`
- `SWINGUP_CONTROL_INTERVAL_STEPS`
- `SWINGUP_Q_*`
- `SWINGUP_QF_*`
- `SWINGUP_U_MAX`
- `SWINGUP_X_MAX`
- `SWINGUP_DELTA_U_MAX`

### Switching logic
- `SWITCH_ANGLE_THRESHOLD`
- `SWITCH_THETA_DOT_THRESHOLD`
- `SWITCH_X_THRESHOLD`
- `SWITCH_X_DOT_THRESHOLD`
- `SWITCH_RETRY_COOLDOWN_STEPS`

### Overrun fallback
- `SWINGUP_FALLBACK_KX`
- `SWINGUP_FALLBACK_KV`

## Current Behavior

At this stage, the code is able to:

- swing the pendulum up from near the downward position
- switch to the upright linear MPC
- maintain upright balance for the rest of the simulation

The upright linear MPC is much faster and more timing-friendly than the nonlinear swing-up phase.

The swing-up phase is the more computationally demanding part of the project.

## Known Limitations

- Swing-up NMPC can still produce occasional long solve times
- Overrun handling improves safety, but does not make the nonlinear MPC truly hard real-time
- The current implementation is suitable for simulation and prototyping, but not yet a guaranteed embedded real-time controller
- The fallback controller during swing-up overruns is a simple centering/braking action, not a full backup swing-up controller

## Future Improvements

Possible next steps:

- add logging to file
- separate timing summaries by mode more cleanly
- tune swing-up NMPC for fewer overrun events
- move swing-up NMPC to a faster backend
- test robustness across many randomized initial conditions
- add noise and disturbance testing
- add a real hardware interface

## Summary

This project demonstrates a practical hybrid MPC approach for the cart-pole:

- nonlinear MPC for large-angle swing-up
- linear MPC for upright balancing
- nonlinear plant simulation throughout
- automatic switching between controllers

It is a strong foundation for studying constrained control, hybrid control, and the tradeoff between controller accuracy and real-time computational cost.

