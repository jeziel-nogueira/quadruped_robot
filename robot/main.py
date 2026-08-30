import os
import sys
import json
import time
import argparse
import logging

# Ensure the root workspace directory is in the path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from robot.controllers.cpg_network import CPGNetwork
from robot.hardware.interface import MockHardware
from robot.hardware.physical import PiHardware
from robot.telemetry.server import TelemetryServer

# Configure Logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger("RobotMain")

CONFIG_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "config.json"))

def load_config():
    try:
        with open(CONFIG_PATH, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Error loading configuration from {CONFIG_PATH}: {e}")
        sys.exit(1)

def save_config(config):
    try:
        with open(CONFIG_PATH, "w") as f:
            json.dump(config, f, indent=2)
        logger.info(f"Configuration successfully saved to {CONFIG_PATH}")
    except Exception as e:
        logger.error(f"Error saving configuration: {e}")

def main():
    parser = argparse.ArgumentParser(description="Quadruped Robot Locomotion System")
    parser.add_argument("--mock", action="store_true", help="Run with mock hardware simulator")
    parser.add_argument("--port", type=int, default=8000, help="Port to run telemetry server on")
    args = parser.parse_args()

    logger.info("Initializing Quadruped Robot...")
    
    # 1. Load config
    config = load_config()
    
    # 2. Select and initialize Hardware
    # If running on Windows or requested via --mock, force Mock Mode.
    is_windows = sys.platform.startswith("win")
    if args.mock or is_windows:
        logger.info("[Hardware] SELECT MODE: Mock (Simulation) Hardware")
        hardware = MockHardware()
    else:
        logger.info("[Hardware] SELECT MODE: Physical Raspberry Pi Hardware")
        hardware = PiHardware()
        
    if not hardware.connect():
        logger.error("Failed to connect to robot hardware. Exiting.")
        sys.exit(1)

    # 3. Initialize Control System (CPG solver)
    cpg = CPGNetwork(config)
    
    # 4. Initialize and Start Telemetry Server
    telemetry = TelemetryServer(host="0.0.0.0", port=args.port)
    telemetry.hardware = hardware  # Allow REST API to control servos for calibration
    telemetry.start()
    
    # Precise loop timing variables (target: 50Hz -> dt = 0.02s)
    target_dt = 0.02
    start_time = time.time()
    last_loop_time = time.time()
    
    logger.info("Quadruped Robot main loop running. Press Ctrl+C to stop.")
    
    try:
        while True:
            loop_start = time.time()
            
            # A. Check for UI commands (gait enable/disable, steering joystick)
            commands = telemetry.get_commands()
            cpg.set_gait_enabled(commands["gait_enabled"])
            cpg.set_steering(commands["steering"])
            
            # B. Check for parameter configuration updates from UI
            config_updates = telemetry.get_config_updates()
            if config_updates:
                # Merge updates into current config
                # Format expected: {"cpg": {"a_M": 1.1}, "feedback": {"K_y": 0.15}} etc.
                for section, params in config_updates.items():
                    if section in config and isinstance(params, dict):
                        config[section].update(params)
                # Apply new configuration to the live CPG solver
                cpg.load_config(config)
                # Persist config to config.json
                save_config(config)
            
            # C. Read sensors
            roll, pitch, yaw = hardware.read_imu()
            distance = hardware.read_distance()
            battery = hardware.read_battery_voltage()
            
            # D. Update CPG network to get next joint angles
            dt = loop_start - last_loop_time
            if dt <= 0.0:
                dt = target_dt
            last_loop_time = loop_start
            
            joint_angles = cpg.step(dt, yaw, pitch, distance)
            
            # E. Command Servos (only when locomotion gait is active)
            # When in standby/calibration, servos stay at their manually calibrated positions
            if commands.get("gait_enabled", False):
                hardware.set_servo_angles(joint_angles)
            
            # F. Prepare and Send Telemetry
            telemetry_payload = {
                "timestamp": round(loop_start - start_time, 2),
                "imu": {
                    "roll": round(roll, 2),
                    "pitch": round(pitch, 2),
                    "yaw": round(yaw, 2)
                },
                "distance": distance,
                "battery": round(battery, 2),
                "joint_angles": [round(a, 3) for a in joint_angles],
                "cpg_states": cpg.get_states(),
                "commands": commands,
                "config": config
            }
            telemetry.update_telemetry(telemetry_payload)
            
            # G. Precise Loop rate sleep
            elapsed = time.time() - loop_start
            sleep_time = target_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received. Shutting down...")
    finally:
        # Detach servos and cleanup range sensor GPIO pins
        hardware.disconnect()
        logger.info("Clean shutdown complete.")

if __name__ == "__main__":
    main()
