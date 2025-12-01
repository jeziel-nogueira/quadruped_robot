# coding: utf-8
"""
config.py

Robot configuration constants.
"""

import math

# ----------------------------
# ROBOT CONFIG - EDIT THESE
# ----------------------------
# Physical lengths (meters)
L1 = 0.06   # thigh length
L2 = 0.06   # shank length
HIP_OFFSET = 0.02  # horizontal offset from hip yaw axis to thigh plane

# Servo channel mapping (example for one leg)
# Map: hip_yaw_channel, hip_pitch_channel, knee_channel (PCA9685 channels)
LEG0_CHANNELS = (0, 1, 2)  # replace with your wiring

# Servo angle limits (degrees)
HIP_YAW_MIN, HIP_YAW_MAX = -45.0, 45.0
HIP_PITCH_MIN, HIP_PITCH_MAX = -45.0, 45.0
KNEE_MIN, KNEE_MAX = -90.0, 10.0

# Controller timing
DT = 0.02  # control loop timestep (s). 50 Hz
CPG_FREQ = 0.8  # Hz - slow gait for MG996
CPG_DT = DT

# Trajectory parameters (meters)
STEP_LENGTH = 0.04   # fore-aft step length
STEP_HEIGHT = 0.03   # vertical lift in swing
BASE_X = 0.08        # neutral forward distance from hip to foot
BASE_Y = 0.05        # lateral offset for this leg (positive to the robot's right)
BASE_Z = -0.12       # rest foot height (negative if downward)

# Safety / smoothing
EMA_ALPHA = 0.2      # exponential moving average smoothing for angles
MAX_DEG_PER_SEC = 90.0  # max servo slew rate (deg/s) for ramping
