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
# MAIN LOOP: Full Quadruped
# ----------------------------
def run_quadruped(duration=None, hardware=False):
    """
    Runs the full quadruped controller for all 4 legs.
    duration: seconds to run, or None for infinite.
    """
    driver = PCA9685Driver() if hardware else PCA9685Driver()
    cpg = SimpleCPGNetwork(legs=4, joints_per_leg=3, freq=config.CPG_FREQ)
    
    # Initialize filters for all channels
    all_channels = []
    for leg_channels in config.LEG_CHANNELS:
        all_channels.extend(leg_channels)
        
    ema_filters = {ch: EMAFilter(config.EMA_ALPHA, init_val=0.0) for ch in all_channels}
    slew = {ch: SlewLimiter(config.MAX_DEG_PER_SEC, init_deg=0.0) for ch in all_channels}

    t0 = time.time()
    t = 0.0
    
    print("Starting Quadruped Controller. Press Ctrl+C to stop.")
    try:
        while duration is None or t < duration:
            loop_start = time.time()
            phases, amps = cpg.step()
            
            # Iterate over all 4 legs
            for leg_idx in range(4):
                leg_channels = config.LEG_CHANNELS[leg_idx]
                leg_phases = phases[leg_idx]
                leg_amps = amps[leg_idx]
                
                # Get leg-specific defaults
                defaults = config.LEG_OFFSET_DEFAULTS[leg_idx]
                
                # Generate foot trajectory
                fx, fy, fz = cpg_to_foot_trajectory(leg_phases, leg_amps, t,
                                                   step_length=config.STEP_LENGTH, step_height=config.STEP_HEIGHT,
                                                   base_x=defaults['x'], base_y=defaults['y'], base_z=defaults['z'])

                # Compute IK
                hy, hp, kn = leg_ik(fx, fy, fz)

                # Map to servo degrees
                hy_deg, hp_deg, kn_deg = joint_angles_to_servo_degrees(hy, hp, kn)
                
                # Clip to limits
                hy_deg = clamp(hy_deg, config.HIP_YAW_MIN, config.HIP_YAW_MAX)
                hp_deg = clamp(hp_deg, config.HIP_PITCH_MIN, config.HIP_PITCH_MAX)
                kn_deg = clamp(kn_deg, config.KNEE_MIN, config.KNEE_MAX)

                # Smooth and slew-limit
                # Channel 0: Hip Yaw, 1: Hip Pitch, 2: Knee
                ch_yaw, ch_pitch, ch_knee = leg_channels
                
                hy_deg = ema_filters[ch_yaw].update(hy_deg)
                hp_deg = ema_filters[ch_pitch].update(hp_deg)
                kn_deg = ema_filters[ch_knee].update(kn_deg)

                hy_deg = slew[ch_yaw].update(hy_deg, config.DT)
                hp_deg = slew[ch_pitch].update(hp_deg, config.DT)
                kn_deg = slew[ch_knee].update(kn_deg, config.DT)

                # Send to driver
                driver.set_servo_angle(ch_yaw, hy_deg)
                driver.set_servo_angle(ch_pitch, hp_deg)
                driver.set_servo_angle(ch_knee, kn_deg)

            # Timing
            loop_end = time.time()
            elapsed = loop_end - loop_start
            sleep_time = max(0.0, config.DT - elapsed)
            time.sleep(sleep_time)
            t = time.time() - t0
            
    except KeyboardInterrupt:
        print("Controller stopped by user.")
    print("Controller finished.")

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "full":
        run_quadruped(duration=None, hardware=False)
    elif len(sys.argv) > 1 and sys.argv[1] == "bench":
        # Run a short bench test for 10 seconds
        run_quadruped(duration=10.0, hardware=False)
    else:
        print("Quadruped controller module.")
        print("Run `python -m quadruped.controller full` to run the full controller.")
        print("Run `python -m quadruped.controller bench` to run a 10s bench test.")
