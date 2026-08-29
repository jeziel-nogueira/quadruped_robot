import os
import logging
from .interface import HardwareInterface
from .servo_driver import PCA9685ServoDriver, ServoConfig, build_default_servo_configs
from .imu_sensor import MPU6050Driver
from .range_sensor import HCSR04Driver

logger = logging.getLogger(__name__)

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config.json")

class PiHardware(HardwareInterface):
    def __init__(self, servo_addr=0x40, imu_addr=0x68, trig_pin=23, echo_pin=24):
        # Load per-servo calibration from config.json
        servo_configs = PCA9685ServoDriver.load_configs_from_json(CONFIG_PATH)
        self.servo = PCA9685ServoDriver(address=servo_addr, servo_configs=servo_configs)
        self.imu = MPU6050Driver(address=imu_addr)
        self.range = HCSR04Driver(trigger_pin=trig_pin, echo_pin=echo_pin)
        self.connected = False

    def connect(self):
        logger.info("[PiHardware] Initializing connection to physical peripherals...")
        
        # Connect to Servo Driver
        servo_ok = self.servo.connect()
        if not servo_ok:
            logger.warning("[PiHardware] Servo Driver connection failed or running in mock.")
            
        # Connect to IMU
        imu_ok = self.imu.connect()
        if not imu_ok:
            logger.warning("[PiHardware] IMU Sensor connection failed or running in mock.")
            
        # Connect to Range Sensor
        range_ok = self.range.connect()
        if not range_ok:
            logger.warning("[PiHardware] Ultrasonic Range Sensor connection failed or running in mock.")
            
        self.connected = True
        logger.info("[PiHardware] Peripheral initialization complete.")
        return True
        
    def set_servo_angles(self, angles_rad):
        if self.connected:
            self.servo.set_all_angles(angles_rad)
        
    def read_imu(self):
        if self.connected:
            return self.imu.read_angles()
        return (0.0, 0.0, 0.0)
        
    def read_distance(self):
        if self.connected:
            return self.range.read_distance()
        return 999.0
        
    def read_battery_voltage(self):
        # On a stock Raspberry Pi without an ADC (like ADS1115), we cannot measure analog voltage.
        # We will return a nominal 7.8V representing a running 2S LiPo battery.
        # In a custom board with an ADC, you would read the ADC pin here.
        return 7.8
        
    def disconnect(self):
        logger.info("[PiHardware] Safely disconnecting physical hardware...")
        try:
            self.servo.detach_all()
            self.range.cleanup()
        except Exception as e:
            logger.error(f"[PiHardware] Error during disconnect cleanup: {e}")
        self.connected = False
        logger.info("[PiHardware] Physical hardware disconnected.")
