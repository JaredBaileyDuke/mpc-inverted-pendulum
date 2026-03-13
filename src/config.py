"""
Configuration values for the cart-pole project.

This file contains:
- physical model parameters
- linear upright MPC settings
- nonlinear swing-up MPC settings
- visualization settings
"""

# -----------------------------
# Physical parameters
# -----------------------------
M = 0.5
m = 0.2
L = 0.3
I = 0.006
b = 0.1
g = 9.81

# -----------------------------
# Shared timing
# -----------------------------
# Plant integration / visualization step
DT = 0.02
SIM_TIME = 12.0

# -----------------------------
# Upright linear MPC settings
# -----------------------------
# Upright controller still runs every DT.
N_HORIZON = 30

Q_X = 20.0
Q_X_DOT = 3.0
Q_THETA = 100.0
Q_THETA_DOT = 10.0

R_U = 0.1
R_DELTA_U = 0.2

QF_X = 40.0
QF_X_DOT = 6.0
QF_THETA = 150.0
QF_THETA_DOT = 15.0

U_MAX = 6.0

USE_TRACK_HARD_CONSTRAINT = True
X_MAX = 1.0

USE_OPTIONAL_STATE_BOUNDS = False
X_DOT_MAX = 10.0
THETA_MAX = 0.35
THETA_DOT_MAX = 10.0

REFERENCE_STATE = [0.0, 0.0, 0.0, 0.0]

# -----------------------------
# Swing-up nonlinear MPC settings
# -----------------------------
# Swing-up NMPC runs slower than the plant:
# every SWINGUP_CONTROL_INTERVAL_STEPS plant steps.
SWINGUP_CONTROL_INTERVAL_STEPS = 2
SWINGUP_DT = DT * SWINGUP_CONTROL_INTERVAL_STEPS

# Reduced horizon to help timing.
SWINGUP_HORIZON = 20

SWINGUP_U_MAX = U_MAX
SWINGUP_X_MAX = X_MAX
SWINGUP_X_DOT_MAX = 3.0
SWINGUP_THETA_DOT_MAX = 10.0
SWINGUP_DELTA_U_MAX = 6.0

SWINGUP_Q_X = 0.5
SWINGUP_Q_X_DOT = 0.10
SWINGUP_Q_THETA = 25.0
SWINGUP_Q_THETA_DOT = 0.05
SWINGUP_R_U = 0.001
SWINGUP_R_DELTA_U = 0.005

# Strengthen terminal preference for being near the center when approaching capture.
SWINGUP_QF_X = 20.0
SWINGUP_QF_X_DOT = 2.0
SWINGUP_QF_THETA = 400.0
SWINGUP_QF_THETA_DOT = 2.0

# Handoff region
SWITCH_ANGLE_THRESHOLD = 0.08
SWITCH_THETA_DOT_THRESHOLD = 0.40
SWITCH_X_THRESHOLD = 0.30
SWITCH_X_DOT_THRESHOLD = 0.35

# After a failed switch attempt, wait this many steps before trying again.
SWITCH_RETRY_COOLDOWN_STEPS = 10

# -----------------------------
# Real-time safeguards
# -----------------------------
# If a swing-up solve takes longer than DT, do not use the stale optimized
# input. Instead, apply a simple safe centering action.
USE_SWINGUP_OVERRUN_GUARD = True

# Backup centering controller gains used only during swing-up overruns.
SWINGUP_FALLBACK_KX = 8.0
SWINGUP_FALLBACK_KV = 4.0

# -----------------------------
# Visualization settings
# -----------------------------
CART_WIDTH = 0.4
CART_HEIGHT = 0.2
POLE_LENGTH = 1.0

X_LIMITS = (-3, 3)
Y_LIMITS = (-1.4, 1.6)

GROUND_Y = 0.0