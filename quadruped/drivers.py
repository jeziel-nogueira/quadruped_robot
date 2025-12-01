# coding: utf-8
"""
drivers.py

Hardware drivers for servo control.
"""

# ----------------------------
# OPTIONAL HARDWARE DRIVER
# ----------------------------
class DummyServoDriver:
    """Fallback driver that prints angles instead of sending to hardware."""
    def __init__(self):
        self.last = {}

    def set_servo_angle(self, channel, angle_deg):
        # Here you would map angle_deg to servo pulse and send via PCA9685.
        print(f"[SIM] Channel {channel}: {angle_deg:.2f} deg") # Optional: uncomment for verbose logging
        self.last[channel] = angle_deg

try:
    from adafruit_servokit import ServoKit
    HAS_SERVOKIT = True
except Exception:
    HAS_SERVOKIT = False

class PCA9685Driver:
    def __init__(self, channels=16, freq=50):
        if HAS_SERVOKIT:
            self.kit = ServoKit(channels=channels)
            # ServoKit expects angle in degrees when using `.servo[x].angle = deg`
            self.hw = True
        else:
            print("Warning: adafruit_servokit not available — using DummyServoDriver.")
            self.kit = DummyServoDriver()
            self.hw = False

    def set_servo_angle(self, channel, angle_deg):
        angle_deg = float(angle_deg)
        # Clip to 0..180 internally if needed; user must ensure valid ranges
        if self.hw:
            try:
                # Some hobby servos accept 0-180 deg; set angle directly
                self.kit.servo[channel].angle = angle_deg
            except Exception as e:
                print(f"[HW ERR] Failed writing to servo {channel}: {e}")
        else:
            self.kit.set_servo_angle(channel, angle_deg)
