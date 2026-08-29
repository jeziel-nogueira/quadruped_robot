import time
import math
import random
import logging

from .servo_driver import ServoConfig, build_default_servo_configs, JOINT_NAMES

logger = logging.getLogger(__name__)


class HardwareInterface:
    def connect(self):
        raise NotImplementedError
        
    def set_servo_angles(self, angles_rad):
        """
        Set angles for all 12 servos.
        angles_rad: list of 12 float angles in radians.
        """
        raise NotImplementedError
        
    def read_imu(self):
        """
        Read IMU sensor.
        Returns (roll, pitch, yaw) in degrees.
        """
        raise NotImplementedError
        
    def read_distance(self):
        """
        Read ultrasonic range sensor.
        Returns distance in centimeters.
        """
        raise NotImplementedError

    def read_battery_voltage(self):
        """Returns battery voltage as a float."""
        raise NotImplementedError
        
    def disconnect(self):
        raise NotImplementedError


class MockServoDriver:
    """A mock servo driver that simulates PCA9685ServoDriver for calibration UI on Windows."""

    def __init__(self):
        self.servo_configs = build_default_servo_configs()
        self._angles = [90.0] * 12

    def set_angle_deg(self, servo_idx, angle_deg):
        if 0 <= servo_idx < 12:
            cfg = self.servo_configs[servo_idx]
            ticks = cfg.deg_to_ticks(angle_deg)
            self._angles[servo_idx] = angle_deg
            logger.debug(f"[MockServo] {cfg.name} → {angle_deg:.1f}° (ticks={ticks})")

    def set_angle_rad(self, servo_idx, angle_rad):
        angle_deg = math.degrees(angle_rad) + 90.0
        self.set_angle_deg(servo_idx, angle_deg)

    def set_all_angles(self, angles_rad):
        for i, a in enumerate(angles_rad):
            self.set_angle_rad(i, a)

    def center_all(self):
        for i in range(12):
            self._angles[i] = 90.0
        logger.info("[MockServo] All servos centered (90°).")

    def set_raw_ticks(self, channel, ticks):
        logger.debug(f"[MockServo] Raw ticks on channel {channel}: {ticks}")

    def detach_all(self):
        logger.info("[MockServo] All servos detached (simulated).")

    def get_configs_as_dicts(self):
        return [cfg.to_dict() for cfg in self.servo_configs]

    @staticmethod
    def load_configs_from_json(config_path):
        from .servo_driver import PCA9685ServoDriver
        return PCA9685ServoDriver.load_configs_from_json(config_path)


class MockHardware(HardwareInterface):
    def __init__(self):
        self.connected = False
        self.servo_angles = [0.0] * 12
        self.servo = MockServoDriver()
        self.start_time = time.time()
        self.mock_pitch = 0.0
        self.mock_yaw = 0.0

    def connect(self):
        self.connected = True
        print("[MockHardware] Connected successfully.")
        return True
        
    def set_servo_angles(self, angles_rad):
        self.servo_angles = list(angles_rad)
        # Mock feedback simulation
        # The movement of servos creates slight body pitch/yaw oscillations in simulation
        t = time.time() - self.start_time
        self.mock_pitch = 3.0 * math.sin(2.0 * math.pi * 1.5 * t) # 1.5 Hz pitch oscillation
        self.mock_yaw = 2.0 * math.sin(2.0 * math.pi * 0.5 * t)   # 0.5 Hz yaw oscillation
        
    def read_imu(self):
        # Return mock IMU pitch/yaw with some random noise
        noise = random.uniform(-0.1, 0.1)
        return (0.0, self.mock_pitch + noise, self.mock_yaw + noise)
        
    def read_distance(self):
        # Simulate obstacle distance
        # Every 20 seconds, simulate approaching a wall (distance decreases to 8cm then stays there for 3s)
        cycle_time = (time.time() - self.start_time) % 20.0
        if 8.0 <= cycle_time <= 13.0:
            # Approaching wall
            dist = 50.0 - 8.4 * (cycle_time - 8.0)
            return max(8.0, dist)
        else:
            return 80.0 # Clear path
            
    def read_battery_voltage(self):
        # Slowly drain battery from 8.4V down to 7.2V
        elapsed = time.time() - self.start_time
        voltage = 8.4 - 0.001 * elapsed
        return max(7.0, voltage)
        
    def disconnect(self):
        self.connected = False
        print("[MockHardware] Disconnected safely.")
