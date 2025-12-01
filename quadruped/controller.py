# coding: utf-8
"""
controller.py

Main controller logic and bench test.
"""

import time
import sys
import config
from drivers import PCA9685Driver
from utils import EMAFilter, SlewLimiter, clamp
from cpg import SimpleCPGNetwork
from kinematics import cpg_to_foot_trajectory, leg_ik, joint_angles_to_servo_degrees

# ----------------------------
# BENCH TEST: single-leg routine
# ----------------------------
def bench_test_one_leg(duration=10.0, hardware=False):
    """
    Runs a bench-test using the CPG -> trajectory -> IK -> servo mapping pipeline
    for a single leg using LEG0_CHANNELS. Prints values if hardware=False.
    """
    driver = PCA9685Driver() if hardware else PCA9685Driver()
    cpg = SimpleCPGNetwork(legs=4, joints_per_leg=3, freq=config.CPG_FREQ)
    # We'll use only leg 0 oscillators for the bench test
    ema_filters = {ch: EMAFilter(config.EMA_ALPHA, init_val=0.0) for ch in config.LEG0_CHANNELS}
    slew = {ch: SlewLimiter(config.MAX_DEG_PER_SEC, init_deg=0.0) for ch in config.LEG0_CHANNELS}

    t0 = time.time()
    t = 0.0
    
    print("Starting bench test (single leg). Press Ctrl+C to stop.")
    try:
        while t < duration:
            loop_start = time.time()
            phases, amps = cpg.step()
            leg_phases = phases[0]   # three phases for leg 0
            leg_amps = amps[0]

            # Generate foot trajectory from the 3 oscillator outputs
            fx, fy, fz = cpg_to_foot_trajectory(leg_phases, leg_amps, t,
                                               step_length=config.STEP_LENGTH, step_height=config.STEP_HEIGHT,
                                               base_x=config.BASE_X, base_y=config.BASE_Y, base_z=config.BASE_Z)

            # Compute IK
            hy, hp, kn = leg_ik(fx, fy, fz)

            # Map to servo degrees (adjust offsets if needed)
            hy_deg, hp_deg, kn_deg = joint_angles_to_servo_degrees(hy, hp, kn,
                                                                   hip_yaw_deg_offset=0.0,
                                                                   hip_pitch_deg_offset=0.0,
                                                                   knee_deg_offset=0.0)
            # Clip to mechanical limits
            hy_deg = clamp(hy_deg, config.HIP_YAW_MIN, config.HIP_YAW_MAX)
            hp_deg = clamp(hp_deg, config.HIP_PITCH_MIN, config.HIP_PITCH_MAX)
            kn_deg = clamp(kn_deg, config.KNEE_MIN, config.KNEE_MAX)

            # Smooth and slew-limit
            hy_deg = ema_filters[config.LEG0_CHANNELS[0]].update(hy_deg)
            hp_deg = ema_filters[config.LEG0_CHANNELS[1]].update(hp_deg)
            kn_deg = ema_filters[config.LEG0_CHANNELS[2]].update(kn_deg)

            hy_deg = slew[config.LEG0_CHANNELS[0]].update(hy_deg, config.DT)
            hp_deg = slew[config.LEG0_CHANNELS[1]].update(hp_deg, config.DT)
            kn_deg = slew[config.LEG0_CHANNELS[2]].update(kn_deg, config.DT)

            # Send to driver
            driver.set_servo_angle(config.LEG0_CHANNELS[0], hy_deg)
            driver.set_servo_angle(config.LEG0_CHANNELS[1], hp_deg)
            driver.set_servo_angle(config.LEG0_CHANNELS[2], kn_deg)

            # Timing
            loop_end = time.time()
            elapsed = loop_end - loop_start
            sleep_time = max(0.0, config.DT - elapsed)
            time.sleep(sleep_time)
            t = time.time() - t0
    except KeyboardInterrupt:
        print("Bench test stopped by user.")
    print("Bench test finished.")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "bench":
        bench_test_one_leg(duration=20.0, hardware=False)
    else:
        print("Quadruped controller module.")
        print("Run `python -m quadruped.controller bench` to run a 1-leg bench test (simulation).")
