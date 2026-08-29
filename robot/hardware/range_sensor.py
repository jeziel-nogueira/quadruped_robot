import time
import logging

logger = logging.getLogger(__name__)

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False

class HCSR04Driver:
    def __init__(self, trigger_pin=23, echo_pin=24):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.connected = False

    def connect(self):
        if not HAS_GPIO:
            logger.warning("[RangeSensor] RPi.GPIO library not installed. Running in mock/dry mode.")
            return False
        try:
            # Configure GPIO pins
            # Use BCM coding
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            # Ensure Trigger is Low initially
            GPIO.output(self.trigger_pin, GPIO.LOW)
            time.sleep(0.1)
            self.connected = True
            logger.info(f"[RangeSensor] Configured HC-SR04 on TRIG={self.trigger_pin}, ECHO={self.echo_pin}")
            return True
        except Exception as e:
            logger.error(f"[RangeSensor] Error configuring GPIO: {e}")
            return False

    def read_distance(self):
        """
        Send trigger pulse and read distance.
        Returns distance in centimeters, or 999.0 on timeout/error.
        """
        if not self.connected or not HAS_GPIO:
            return 80.0 # Return standard clear path distance in dry run
            
        try:
            # 1. Send 10us Trigger pulse
            GPIO.output(self.trigger_pin, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.trigger_pin, GPIO.LOW)
            
            # 2. Measure pulse duration on Echo pin with timeout (max 30ms ~ 5m)
            timeout = 0.03
            start_trigger = time.time()
            
            pulse_start = time.time()
            while GPIO.input(self.echo_pin) == 0:
                pulse_start = time.time()
                if pulse_start - start_trigger > timeout:
                    return 999.0 # Timeout
                    
            pulse_end = time.time()
            while GPIO.input(self.echo_pin) == 1:
                pulse_end = time.time()
                if pulse_end - pulse_start > timeout:
                    return 999.0 # Timeout
                    
            pulse_duration = pulse_end - pulse_start
            
            # 3. Calculate distance (Speed of sound = 343 m/s or 34300 cm/s)
            # distance = time * speed / 2 (round trip)
            distance = (pulse_duration * 34300) / 2.0
            
            # Filter unrealistic values
            if distance < 2.0 or distance > 400.0:
                return 999.0
                
            return round(distance, 1)
            
        except Exception as e:
            logger.error(f"[RangeSensor] Error reading HC-SR04: {e}")
            return 999.0
            
    def cleanup(self):
        if HAS_GPIO:
            try:
                GPIO.cleanup((self.trigger_pin, self.echo_pin))
                logger.info("[RangeSensor] Cleaned up GPIO pins.")
            except Exception as e:
                logger.error(f"[RangeSensor] Error during GPIO cleanup: {e}")
