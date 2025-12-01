# coding: utf-8
"""
kinematics.py

Inverse Kinematics and Trajectory Generation.
"""

import math
import config
from utils import clamp, rad2deg

# ----------------------------
# CPG -> FOOT TRAJECTORY
# ----------------------------
def cpg_to_foot_trajectory(phases_leg, amps_leg, t,
                           step_length=config.STEP_LENGTH, step_height=config.STEP_HEIGHT,
                           base_x=config.BASE_X, base_y=config.BASE_Y, base_z=config.BASE_Z):
    """
    Convert three oscillator phases (hip, thigh, knee) for a single leg into a foot (x,y,z).
    phases_leg: list/tuple of 3 phases (rad) for hip, thigh, knee
    amps_leg: list/tuple of 3 amplitudes (unitless) corresponding to phases
    t: current time (s) - used to compute normalized cycle if needed
    Returns: (x, y, z) in meters in leg frame
    """
    # Use hip oscillator to control fore-aft position via cosine (stance/swing)
    hip_phase = phases_leg[0]
    thigh_phase = phases_leg[1]
    knee_phase = phases_leg[2]

    # normalize phase to [0, 2pi)
    def norm(p): 
        p = p % (2*math.pi)
        return p

    hp = norm(hip_phase)
    # duty cycle: simple split: swing when phase in [0, pi), stance [pi, 2pi)
    # We'll create an asymmetric waveform: swing uses a raised-sine for lift; stance pushes back.
    # x: forward/back relative to base_x
    # a simple param: cos for stance, cos for swing
    # map cos to -1..1 then to step_length
    x_swing = base_x + (step_length/2.0) * math.cos(hp)  # will oscillate around base_x
    # y: lateral is mostly constant + small modulation from hip phase
    y = base_y + 0.005 * math.sin(hp)  # small lateral sway

    # z: height - use thigh/knee phases combined to shape swing lift
    # We'll set z higher during half cycle (swing)
    if 0 <= hp < math.pi:
        # swing: use a smooth half-sine for lift
        z = base_z + step_height * math.sin(hp)
    else:
        # stance: keep near base_z with minor modulation
        z = base_z + 0.002 * math.sin(hp)

    return x_swing, y, z

# ----------------------------
# Inverse Kinematics (3DOF leg)
# ----------------------------
def leg_ik(x, y, z, L1=config.L1, L2=config.L2, hip_offset=config.HIP_OFFSET):
    """
    Compute hip_yaw (rad), hip_pitch (rad), knee (rad) for a 3DOF leg.
    Coordinate frame assumptions:
      - x forward (positive ahead of robot)
      - y lateral (positive to robot's right)
      - z up (positive upwards; typical base_z is negative)
    hip_yaw rotates around vertical axis to point foot at (x,y)
    hip_pitch and knee are computed in the leg sagittal plane after projecting.
    """
    # hip_yaw (point foot direction)
    hip_yaw = math.atan2(y, x) if not (x == 0 and y == 0) else 0.0

    # project to sagittal plane of thigh after hip offset
    r = math.sqrt(x*x + y*y)
    r_proj = max(0.0, r - hip_offset)  # remove hip lateral offset

    # distance from hip pitch axis to foot
    R2 = r_proj*r_proj + z*z
    # law of cosines for knee
    cosk = (R2 - L1*L1 - L2*L2) / (2.0 * L1 * L2)
    cosk = clamp(cosk, -1.0, 1.0)
    # choose knee bending "backwards" (negative angle) typical for quadruped
    knee = math.atan2(-math.sqrt(max(0.0, 1.0 - cosk*cosk)), cosk)

    # hip pitch
    gamma = math.atan2(z, r_proj)
    phi = math.atan2(L2 * math.sin(knee), L1 + L2 * math.cos(knee))
    hip_pitch = gamma - phi

    # convert to degrees for servo mapping later but return radians
    return hip_yaw, hip_pitch, knee

# ----------------------------
# ANGLE MAPPING: robot joint angles -> servo angles (deg)
# ----------------------------
def joint_angles_to_servo_degrees(hip_yaw, hip_pitch, knee,
                                  hip_yaw_deg_offset=0.0,
                                  hip_pitch_deg_offset=0.0,
                                  knee_deg_offset=0.0):
    """
    Map computed joint angles (radians) to servo degrees.
    Offsets allow to align mechanical zero positions.
    """
    # Convert radians to degrees
    hy = rad2deg(hip_yaw) + hip_yaw_deg_offset
    hp = rad2deg(hip_pitch) + hip_pitch_deg_offset
    kn = rad2deg(knee) + knee_deg_offset
    # Depending on your servo orientation you may need to flip signs
    # Example: knee might require inversion to go positive when extending
    return hy, hp, kn
