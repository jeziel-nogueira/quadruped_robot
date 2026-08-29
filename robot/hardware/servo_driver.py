import math
import json
import os
import logging

logger = logging.getLogger(__name__)

try:
    import Adafruit_PCA9685
    HAS_PCA9685 = True
except ImportError:
    HAS_PCA9685 = False


# Default servo pulse ticks (out of 4096) for a typical 180° servo at 50Hz PWM:
#   500us  → ~102 ticks  (0°)
#   1500us → ~307 ticks  (90° center)
#   2500us → ~512 ticks  (180°)
DEFAULT_TICK_MIN = 102
DEFAULT_TICK_CENTER = 307
DEFAULT_TICK_MAX = 512

# Joint names for readability
JOINT_NAMES = [
    "FL_hip_yaw",   "FL_hip_pitch",  "FL_knee",       # Front Left  (indices 0-2)
    "FR_hip_yaw",   "FR_hip_pitch",  "FR_knee",       # Front Right (indices 3-5)
    "HL_hip_yaw",   "HL_hip_pitch",  "HL_knee",       # Hind Left   (indices 6-8)
    "HR_hip_yaw",   "HR_hip_pitch",  "HR_knee",       # Hind Right  (indices 9-11)
]


class ServoConfig:
    """Per-servo calibration parameters."""

    def __init__(self, channel, name="servo",
                 tick_min=DEFAULT_TICK_MIN,
                 tick_center=DEFAULT_TICK_CENTER,
                 tick_max=DEFAULT_TICK_MAX,
                 inverted=False,
                 trim_deg=0.0,
                 limit_min_deg=0.0,
                 limit_max_deg=180.0):
        self.channel = channel          # PCA9685 channel (0-15)
        self.name = name                # Human-readable label
        self.tick_min = tick_min        # PWM tick at 0°
        self.tick_center = tick_center  # PWM tick at 90° (neutral)
        self.tick_max = tick_max        # PWM tick at 180°
        self.inverted = inverted        # If True, direction is reversed
        self.trim_deg = trim_deg        # Mechanical trim offset in degrees
        self.limit_min_deg = limit_min_deg  # Software limit (degrees)
        self.limit_max_deg = limit_max_deg  # Software limit (degrees)

    def deg_to_ticks(self, angle_deg):
        """Convert angle in degrees (0-180) to PCA9685 tick value."""
        # Apply trim
        angle_deg = angle_deg + self.trim_deg

        # Apply software limits
        angle_deg = max(self.limit_min_deg, min(self.limit_max_deg, angle_deg))

        # Invert if needed
        if self.inverted:
            angle_deg = 180.0 - angle_deg

        # Map 0-180° to tick_min..tick_max linearly
        ratio = angle_deg / 180.0
        ticks = self.tick_min + ratio * (self.tick_max - self.tick_min)
        return int(max(self.tick_min, min(self.tick_max, ticks)))

    def rad_to_ticks(self, angle_rad):
        """Convert angle in radians (centered at 0 = 90°) to PCA9685 ticks.
        
        The CPG controller outputs angles in radians around 0 (neutral).
        0 rad  → 90°  (center position)
        -π/2   → 0°   (full one way)
        +π/2   → 180° (full other way)
        """
        angle_deg = math.degrees(angle_rad) + 90.0
        return self.deg_to_ticks(angle_deg)

    def to_dict(self):
        return {
            "channel": self.channel,
            "name": self.name,
            "tick_min": self.tick_min,
            "tick_center": self.tick_center,
            "tick_max": self.tick_max,
            "inverted": self.inverted,
            "trim_deg": self.trim_deg,
            "limit_min_deg": self.limit_min_deg,
            "limit_max_deg": self.limit_max_deg,
        }

    @staticmethod
    def from_dict(data):
        return ServoConfig(
            channel=data.get("channel", 0),
            name=data.get("name", "servo"),
            tick_min=data.get("tick_min", DEFAULT_TICK_MIN),
            tick_center=data.get("tick_center", DEFAULT_TICK_CENTER),
            tick_max=data.get("tick_max", DEFAULT_TICK_MAX),
            inverted=data.get("inverted", False),
            trim_deg=data.get("trim_deg", 0.0),
            limit_min_deg=data.get("limit_min_deg", 0.0),
            limit_max_deg=data.get("limit_max_deg", 180.0),
        )


def build_default_servo_configs():
    """Build the default 12-servo config list with standard PCA9685 channel mapping."""
    # Channel mapping: FL on 0-2, FR on 4-6, HL on 8-10, HR on 12-14
    channels = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
    configs = []
    for i in range(12):
        configs.append(ServoConfig(
            channel=channels[i],
            name=JOINT_NAMES[i],
            tick_min=DEFAULT_TICK_MIN,
            tick_center=DEFAULT_TICK_CENTER,
            tick_max=DEFAULT_TICK_MAX,
            inverted=False,
            trim_deg=0.0,
            limit_min_deg=0.0,
            limit_max_deg=180.0,
        ))
    return configs


class PCA9685ServoDriver:
    """PCA9685-based 12-servo driver with per-servo calibration support."""

    def __init__(self, address=0x40, busnum=1, frequency=50, servo_configs=None):
        self.address = address
        self.busnum = busnum
        self.frequency = frequency
        self.pwm = None

        # Per-servo calibration
        if servo_configs is not None:
            self.servo_configs = servo_configs
        else:
            self.servo_configs = build_default_servo_configs()

    def connect(self):
        if not HAS_PCA9685:
            logger.warning("[ServoDriver] Adafruit_PCA9685 not installed. Running in mock/dry mode.")
            return False
        try:
            self.pwm = Adafruit_PCA9685.PCA9685(address=self.address, busnum=self.busnum)
            self.pwm.set_pwm_freq(self.frequency)
            logger.info(f"[ServoDriver] Connected to PCA9685 at {hex(self.address)}, freq={self.frequency}Hz")
            return True
        except Exception as e:
            logger.error(f"[ServoDriver] Connection error: {e}")
            return False

    def set_angle_deg(self, servo_idx, angle_deg):
        """Set servo by index (0-11) to an angle in degrees (0-180)."""
        if not self.pwm:
            return
        if servo_idx < 0 or servo_idx >= len(self.servo_configs):
            return
        cfg = self.servo_configs[servo_idx]
        ticks = cfg.deg_to_ticks(angle_deg)
        self.pwm.set_pwm(cfg.channel, 0, ticks)

    def set_angle_rad(self, servo_idx, angle_rad):
        """Set servo by index (0-11) from a CPG-space radians value (0 = center = 90°)."""
        if not self.pwm:
            return
        if servo_idx < 0 or servo_idx >= len(self.servo_configs):
            return
        cfg = self.servo_configs[servo_idx]
        ticks = cfg.rad_to_ticks(angle_rad)
        self.pwm.set_pwm(cfg.channel, 0, ticks)

    def set_all_angles(self, angles_rad):
        """Set all 12 servo angles from a list of radian values."""
        for i, angle in enumerate(angles_rad):
            self.set_angle_rad(i, angle)

    def center_all(self):
        """Move all servos to their calibrated center position (90°)."""
        if not self.pwm:
            return
        for cfg in self.servo_configs:
            self.pwm.set_pwm(cfg.channel, 0, cfg.tick_center)
        logger.info("[ServoDriver] All servos centered.")

    def set_raw_ticks(self, channel, ticks):
        """Low-level: set raw PWM ticks on a PCA9685 channel directly."""
        if not self.pwm:
            return
        ticks = int(max(0, min(4095, ticks)))
        self.pwm.set_pwm(channel, 0, ticks)

    def detach_all(self):
        """Turn off PWM signal on all servo channels (prevents buzzing)."""
        if not self.pwm:
            return
        for cfg in self.servo_configs:
            self.pwm.set_pwm(cfg.channel, 0, 0)
        logger.info("[ServoDriver] All servos detached (PWM off).")

    def get_configs_as_dicts(self):
        """Return all servo configs as a list of dicts (for saving to JSON)."""
        return [cfg.to_dict() for cfg in self.servo_configs]

    @staticmethod
    def load_configs_from_json(config_path):
        """Load servo configs from the 'servos' section of a config.json file."""
        try:
            with open(config_path, "r") as f:
                data = json.load(f)
            servos_data = data.get("servos", [])
            if servos_data:
                return [ServoConfig.from_dict(s) for s in servos_data]
        except Exception as e:
            logger.warning(f"[ServoDriver] Could not load servo config: {e}")
        return build_default_servo_configs()
