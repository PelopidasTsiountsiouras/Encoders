"""
AMT103 Encoder Distance and Rotation Tracker for 1/10 RC Car
Tracks wheel rotations and distance traveled using quadrature encoder
"""

import RPi.GPIO as GPIO
import time
import math
import logging
from threading import Lock
from datetime import datetime

class EncoderTracker:
    def __init__(self, pin_a, pin_b, ppr, wheel_diameter_mm, pin_index=None, logger=None):
        """
        Initialize the encoder tracker
        
        Args:
            pin_a: GPIO pin for encoder channel A
            pin_b: GPIO pin for encoder channel B
            ppr: Pulses per revolution of the encoder
            wheel_diameter_mm: Wheel diameter in millimeters
            pin_index: GPIO pin for index/Z channel (optional)
            logger: Optional logger instance
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.pin_index = pin_index
        self.ppr = ppr
        self.wheel_diameter_mm = wheel_diameter_mm
        self.logger = logger or logging.getLogger(__name__)
        
        # Calculate wheel circumference
        self.wheel_circumference_mm = math.pi * wheel_diameter_mm
        
        # With quadrature decoding, we get 4x the base PPR
        self.counts_per_revolution = ppr * 4
        
        # Distance per encoder count
        self.mm_per_count = self.wheel_circumference_mm / self.counts_per_revolution
        
        # Encoder state
        self.encoder_count = 0
        self.last_a_state = 0
        self.last_b_state = 0
        self.lock = Lock()
        
        # Index pulse tracking
        self.index_count = 0
        self.last_index_time = 0
        
        # Speed tracking
        self.last_count = 0
        self.last_time = time.time()
        
        # Setup GPIO
        self._setup_gpio()
        
        index_info = f", Index=GPIO{pin_index}" if pin_index else ", No Index"
        self.logger.info(f"Encoder initialized: PPR={ppr}, Wheel={wheel_diameter_mm}mm{index_info}, "
                        f"Counts/Rev={self.counts_per_revolution}, mm/count={self.mm_per_count:.4f}")
        
    def _setup_gpio(self):
        """Setup GPIO pins and interrupts"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Read initial state
        self.last_a_state = GPIO.input(self.pin_a)
        
        # Attach interrupt to channel A (both rising and falling edges)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._encoder_callback)
        
        self.logger.debug(f"GPIO setup complete: A=GPIO{self.pin_a}, B=GPIO{self.pin_b}")
        
    def _encoder_callback(self, channel):
        """
        Interrupt callback for encoder counting
        Uses quadrature decoding to determine direction
        """
        with self.lock:
            a_state = GPIO.input(self.pin_a)
            b_state = GPIO.input(self.pin_b)
            
            # Only process if A state changed
            if a_state != self.last_a_state:
                # Determine direction based on A and B relationship
                if a_state == b_state:
                    self.encoder_count += 1  # Forward/Clockwise
                else:
                    self.encoder_count -= 1  # Backward/Counter-clockwise
                    
                self.last_a_state = a_state
    
    def get_count(self):
        """Get current encoder count"""
        with self.lock:
            return self.encoder_count
    
    def reset_count(self):
        """Reset encoder count to zero"""
        with self.lock:
            old_count = self.encoder_count
            self.encoder_count = 0
            self.last_count = 0
            self.last_time = time.time()
            self.logger.info(f"Encoder reset (previous count: {old_count})")
    
    def get_rotations(self):
        """Get number of wheel rotations (can be negative for reverse)"""
        count = self.get_count()
        return count / self.counts_per_revolution
    
    def get_distance_mm(self):
        """Get distance traveled in millimeters"""
        count = self.get_count()
        return count * self.mm_per_count
    
    def get_distance_cm(self):
        """Get distance traveled in meters"""
        return self.get_distance_mm() / 10.0
    
    def get_speed_cmps(self):
        """
        Calculate current speed in meters per second
        Returns speed based on counts since last call
        """
        current_time = time.time()
        current_count = self.get_count()
        
        time_diff = current_time - self.last_time
        count_diff = current_count - self.last_count
        
        if time_diff > 0:
            # Calculate speed
            distance_mm = count_diff * self.mm_per_count
            speed_mm_per_sec = distance_mm / time_diff
            speed_cm_per_sec = speed_mm_per_sec / 10.0
            
            # Update for next calculation
            self.last_time = current_time
            self.last_count = current_count
            
            return speed_cm_per_sec
        
        return 0.0
    
    def cleanup(self):
        """Cleanup GPIO"""
        self.logger.info("Cleaning up GPIO")
        GPIO.cleanup()


def setup_logging(log_to_file=True, log_level=logging.INFO):
    """
    Setup logging configuration
    
    Args:
        log_to_file: If True, logs to file in addition to console
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
    """
    # Create logger
    logger = logging.getLogger('encoder_tracker')
    logger.setLevel(log_level)
    
    # Clear existing handlers
    logger.handlers.clear()
    
    # Create formatters
    detailed_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    simple_formatter = logging.Formatter(
        '%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(log_level)
    console_handler.setFormatter(simple_formatter)
    logger.addHandler(console_handler)
    
    # File handler (optional)
    if log_to_file:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_filename = f'encoder_log_{timestamp}.log'
        file_handler = logging.FileHandler(log_filename)
        file_handler.setLevel(log_level)
        file_handler.setFormatter(detailed_formatter)
        logger.addHandler(file_handler)
        logger.info(f"Logging to file: {log_filename}")
    
    return logger


def main():
    """Example usage of the encoder tracker"""
    
    # Configuration - ADJUST THESE TO MATCH YOUR SETUP
    ENCODER_PIN_A = 17      # GPIO pin for channel A
    ENCODER_PIN_B = 27      # GPIO pin for channel B
    PIN_INDEX = 22
    ENCODER_PPR = 48       # Pulses per revolution (check your AMT103 model)
    WHEEL_DIAMETER_MM = 60  # Typical 1/10 scale car wheel diameter
    
    # Setup logging
    logger = setup_logging(
        log_to_file=True,          # Save logs to file
        log_level=logging.INFO     # Change to logging.DEBUG for more detail
    )
    
    logger.info("=" * 60)
    logger.info("RC Car Encoder Distance Tracker Started")
    logger.info("=" * 60)
    logger.info(f"Configuration: PPR={ENCODER_PPR}, Wheel Diameter={WHEEL_DIAMETER_MM}mm")
    logger.info(f"GPIO Pins: A={ENCODER_PIN_A}, B={ENCODER_PIN_B}")
    logger.info("=" * 60)
    logger.info("Rotate the wheel to see measurements...")
    logger.info("Press Ctrl+C to exit")
    logger.info("")
    
    # Create encoder tracker
    encoder = EncoderTracker(
        pin_a=ENCODER_PIN_A,
        pin_b=ENCODER_PIN_B,
        ppr=ENCODER_PPR,
        wheel_diameter_mm=WHEEL_DIAMETER_MM,
        pin_index=None,
        logger=logger
    )
    
    # Tracking variables for periodic logging
    log_interval = 1.0  # Log detailed stats every second
    last_log_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            
            # Get measurements
            count = encoder.get_count()
            rotations = encoder.get_rotations()
            distance_mm = encoder.get_distance_mm()
            distance_cm = encoder.get_distance_cm()
            speed_cmps = encoder.get_speed_cmps()
            
            # Log detailed stats periodically
            if current_time - last_log_time >= log_interval:
                logger.info(
                    f"Stats - Count: {count:6d} | Rotations: {rotations:7.2f} | "
                    f"Distance: {distance_cm:6.2f}cm | Speed: {speed_cmps:5.1f} cm/s"
                )
                last_log_time = current_time
            
            time.sleep(0.1)  # Update 10 times per second
            
    except KeyboardInterrupt:
        logger.info("")
        logger.info("=" * 60)
        logger.info("Session Ended - Final Statistics")
        logger.info("=" * 60)
        logger.info(f"Total Count: {encoder.get_count()}")
        logger.info(f"Total Rotations: {encoder.get_rotations():.2f}")
        logger.info(f"Total Distance: {encoder.get_distance_cm():.3f} centimeters ({encoder.get_distance_mm():.1f} mm)")
        logger.info("=" * 60)
        
    finally:
        encoder.cleanup()
        logger.info("GPIO cleaned up. Goodbye!")


if __name__ == "__main__":
    main()