import math
import time
import logging

logger = logging.getLogger(__name__)

try:
    import smbus2 as smbus
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False

class MPU6050Driver:
    # MPU6050 Registers
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_ENABLE = 0x38
    
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    TEMP_OUT_H = 0x41
    GYRO_XOUT_H = 0x43
    GYRO_YOUT_H = 0x45
    GYRO_ZOUT_H = 0x47

    def __init__(self, address=0x68, busnum=1):
        self.address = address
        self.busnum = busnum
        self.bus = None
        self.yaw = 0.0
        self.last_time = time.time()
        
    def connect(self):
        if not HAS_SMBUS:
            logger.warning("[IMUSensor] smbus2 library not installed. Running in mock/dry mode.")
            return False
        try:
            self.bus = smbus.SMBus(self.busnum)
            # Wake up the MPU6050 (write 0 to PWR_MGMT_1 register)
            self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0)
            logger.info(f"[IMUSensor] Connected to MPU6050 at address {hex(self.address)}")
            self.last_time = time.time()
            return True
        except Exception as e:
            logger.error(f"[IMUSensor] Error connecting to MPU6050: {e}")
            return False

    def _read_word_2c(self, reg):
        """Read 2 bytes from register as a signed integer."""
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def read_angles(self):
        """
        Reads raw data and returns (roll, pitch, yaw) in degrees.
        Roll and pitch are calculated from gravity vector; yaw is integrated.
        """
        if not self.bus:
            return 0.0, 0.0, 0.0
            
        try:
            # Read Accelerometer raw values
            # Full scale range +/- 2g (default) -> divider is 16384.0
            ax = self._read_word_2c(self.ACCEL_XOUT_H) / 16384.0
            ay = self._read_word_2c(self.ACCEL_YOUT_H) / 16384.0
            az = self._read_word_2c(self.ACCEL_ZOUT_H) / 16384.0
            
            # Read Gyroscope raw values
            # Full scale range +/- 250 deg/s (default) -> divider is 131.0
            gx = self._read_word_2c(self.GYRO_XOUT_H) / 131.0
            gy = self._read_word_2c(self.GYRO_YOUT_H) / 131.0
            gz = self._read_word_2c(self.GYRO_ZOUT_H) / 131.0
            
            # Calculate roll and pitch from accelerometer
            # Pitch: rotation around Y axis
            # Roll: rotation around X axis
            # Safeguard divide by zero
            denom_pitch = math.sqrt(ay**2 + az**2)
            denom_roll = math.sqrt(ax**2 + az**2)
            
            pitch = math.atan2(-ax, denom_pitch) * (180.0 / math.pi) if denom_pitch > 0 else 0.0
            roll = math.atan2(ay, denom_roll) * (180.0 / math.pi) if denom_roll > 0 else 0.0
            
            # Integrate yaw rate (gz is in deg/s) to get yaw angle
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            
            # Integrate yaw (only if time step is reasonable)
            if dt < 0.5:
                self.yaw += gz * dt
                
            # Normalize yaw between -180 and 180
            self.yaw = (self.yaw + 180) % 360 - 180
            
            return roll, pitch, self.yaw
            
        except Exception as e:
            logger.error(f"[IMUSensor] Error reading sensor data: {e}")
            return 0.0, 0.0, 0.0
