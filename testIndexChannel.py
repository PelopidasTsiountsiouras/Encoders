#!/usr/bin/env python3
"""
AMT103 Encoder Distance and Rotation Tracker for 1/10 RC Car
Fixed version with diagnostics and proper quadrature decoding
"""

import RPi.GPIO as GPIO
import time
import math
import logging
from threading import Lock
from datetime import datetime
from collections import deque


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
        self.lock = Lock()
        
        # State tracking for proper quadrature
        self.state = 0  # Current state (0-3)
        
        # Index pulse tracking
        self.index_count = 0
        self.last_index_time = 0
        self.index_positions = []  # Track encoder count at each index pulse
        self.counts_per_index = []  # Track counts between index pulses
        
        # Speed tracking with smoothing
        self.speed_history = deque(maxlen=5)
        self.last_count = 0
        self.last_time = time.time()
        
        # Diagnostics
        self.invalid_transitions = 0
        self.total_transitions = 0
        
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
        
        # Setup index channel if provided
        if self.pin_index:
            GPIO.setup(self.pin_index, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(self.pin_index, GPIO.RISING, callback=self._index_callback)
        
        # Read initial state
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        self.state = (a << 1) | b
        
        # Attach interrupts to both channels
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._encoder_callback, bouncetime=1)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._encoder_callback, bouncetime=1)
        
        index_msg = f", Index=GPIO{self.pin_index}" if self.pin_index else ""
        self.logger.debug(f"GPIO setup complete: A=GPIO{self.pin_a}, B=GPIO{self.pin_b}{index_msg}")
        self.logger.debug(f"Initial state: {self.state:02b} (A={a}, B={b})")
        
    def _encoder_callback(self, channel):
        """
        Interrupt callback for encoder counting
        Simplified and robust quadrature decoding
        """
        # Read both channels
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        
        # Calculate new state
        new_state = (a << 1) | b
        
        with self.lock:
            old_state = self.state
            
            # Skip if no state change (debouncing)
            if new_state == old_state:
                return
            
            self.total_transitions += 1
            
            # Quadrature encoder state machine
            # States: 00(0), 01(1), 11(3), 10(2)
            # CW:  0->2->3->1->0
            # CCW: 0->1->3->2->0
            
            # Create lookup for direction based on state transitions
            # Key: (old_state << 2) | new_state
            transition = (old_state << 2) | new_state
            
            # Valid forward transitions (CW)
            if transition in [0b0010, 0b1011, 0b1101, 0b0100]:  # 0->2, 2->3, 3->1, 1->0
                self.encoder_count += 1
            # Valid backward transitions (CCW)  
            elif transition in [0b0001, 0b0111, 0b1110, 0b1000]:  # 0->1, 1->3, 3->2, 2->0
                self.encoder_count -= 1
            else:
                # Invalid transition (noise or missed pulse)
                self.invalid_transitions += 1
                self.logger.debug(f"Invalid transition: {old_state:02b} -> {new_state:02b}")
            
            self.state = new_state
    
    def _index_callback(self, channel):
        """Callback for index/Z pulse"""
        with self.lock:
            current_time = time.time()
            self.index_count += 1
            
            if self.last_index_time > 0:
                time_diff = current_time - self.last_index_time
                rpm = 60.0 / time_diff if time_diff > 0 else 0
                self.logger.debug(f"Index pulse #{self.index_count}, RPM: {rpm:.1f}")
            
            self.last_index_time = current_time
    
    def get_count(self):
        """Get current encoder count"""
        with self.lock:
            return self.encoder_count
    
    def get_index_count(self):
        """Get number of complete revolutions detected by index pulse"""
        with self.lock:
            return self.index_count
    
    def reset_count(self):
        """Reset encoder count to zero"""
        with self.lock:
            old_count = self.encoder_count
            self.encoder_count = 0
            self.last_count = 0
            self.last_time = time.time()
            self.speed_history.clear()
            self.logger.info(f"Encoder reset (previous count: {old_count})")
    
    def get_rotations(self):
        """Get number of wheel rotations"""
        count = self.get_count()
        return count / self.counts_per_revolution
    
    def get_distance_mm(self):
        """Get distance traveled in millimeters"""
        count = self.get_count()
        return count * self.mm_per_count
    
    def get_distance_cm(self):
        """Get distance traveled in centimeters"""
        return self.get_distance_mm() / 10.0
    
    def get_distance_m(self):
        """Get distance traveled in meters"""
        return self.get_distance_mm() / 1000.0
    
    def get_speed_mps(self):
        """Calculate current speed in meters per second with smoothing"""
        current_time = time.time()
        current_count = self.get_count()
        
        time_diff = current_time - self.last_time
        count_diff = current_count - self.last_count
        
        if time_diff > 0:
            distance_mm = count_diff * self.mm_per_count
            speed_mm_per_sec = distance_mm / time_diff
            speed_m_per_sec = speed_mm_per_sec / 1000.0
            
            # Add to history for smoothing
            self.speed_history.append(speed_m_per_sec)
            
            # Update for next calculation
            self.last_time = current_time
            self.last_count = current_count
            
            # Return smoothed average
            if len(self.speed_history) > 0:
                return sum(self.speed_history) / len(self.speed_history)
        
        return 0.0
    
    def get_speed_cmps(self):
        """Get current speed in cm/s"""
        return self.get_speed_mps() * 100.0
    
    def get_speed_kmh(self):
        """Get current speed in km/h"""
        return self.get_speed_mps() * 3.6
    
    def get_diagnostics(self):
        """Get diagnostic information"""
        with self.lock:
            return {
                'total_transitions': self.total_transitions,
                'invalid_transitions': self.invalid_transitions,
                'error_rate': (self.invalid_transitions / max(1, self.total_transitions)) * 100
            }
    
    def cleanup(self):
        """Cleanup GPIO"""
        self.logger.info("Cleaning up GPIO")
        GPIO.cleanup()


def setup_logging(log_to_file=True, log_level=logging.INFO):
    """Setup logging configuration"""
    logger = logging.getLogger('encoder_tracker')
    logger.setLevel(log_level)
    logger.handlers.clear()
    
    detailed_formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    simple_formatter = logging.Formatter(
        '%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )
    
    console_handler = logging.StreamHandler()
    console_handler.setLevel(log_level)
    console_handler.setFormatter(simple_formatter)
    logger.addHandler(console_handler)
    
    if log_to_file:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_filename = f'encoder_log_{timestamp}.log'
        file_handler = logging.FileHandler(log_filename)
        file_handler.setLevel(log_level)
        file_handler.setFormatter(detailed_formatter)
        logger.addHandler(file_handler)
        logger.info(f"Logging to file: {log_filename}")
    
    return logger


def run_diagnostic_test(encoder, logger):
    """Run a diagnostic test to check encoder wiring"""
    logger.info("")
    logger.info("=" * 60)
    logger.info("DIAGNOSTIC MODE")
    logger.info("=" * 60)
    logger.info("Please rotate the wheel SLOWLY in ONE direction")
    logger.info("Watching for 10 seconds...")
    logger.info("")
    
    start_count = encoder.get_count()
    start_time = time.time()
    
    time.sleep(10)
    
    end_count = encoder.get_count()
    end_time = time.time()
    count_diff = end_count - start_count
    
    logger.info("=" * 60)
    logger.info("DIAGNOSTIC RESULTS")
    logger.info("=" * 60)
    logger.info(f"Start count: {start_count}")
    logger.info(f"End count: {end_count}")
    logger.info(f"Count difference: {count_diff}")
    logger.info(f"Direction: {'FORWARD' if count_diff > 0 else 'BACKWARD' if count_diff < 0 else 'NO MOVEMENT'}")
    
    diag = encoder.get_diagnostics()
    logger.info(f"Total transitions: {diag['total_transitions']}")
    logger.info(f"Invalid transitions: {diag['invalid_transitions']}")
    logger.info(f"Error rate: {diag['error_rate']:.1f}%")
    
    if diag['error_rate'] > 10:
        logger.warning("HIGH ERROR RATE! Check wiring and connections")
    
    if abs(count_diff) < 10:
        logger.warning("Very few counts detected! Check encoder connections")
    elif count_diff < 0:
        logger.info("SUGGESTION: If you rotated forward but got negative counts,")
        logger.info("            swap your A and B channel wires")
    
    logger.info("=" * 60)
    logger.info("")


def main():
    """Main program with diagnostic mode"""
    
    # Configuration
    ENCODER_PIN_A = 17
    ENCODER_PIN_B = 27
    ENCODER_PIN_INDEX = 22  # Set to your Index/X channel GPIO pin (or None if not using)
    ENCODER_PPR = 48
    WHEEL_DIAMETER_MM = 60
    
    # Setup logging
    logger = setup_logging(log_to_file=True, log_level=logging.INFO)
    
    logger.info("=" * 60)
    logger.info("RC Car Encoder Distance Tracker")
    logger.info("=" * 60)
    logger.info(f"Configuration: PPR={ENCODER_PPR}, Wheel={WHEEL_DIAMETER_MM}mm")
    index_msg = f", Index={ENCODER_PIN_INDEX}" if ENCODER_PIN_INDEX else ", No Index"
    logger.info(f"GPIO Pins: A={ENCODER_PIN_A}, B={ENCODER_PIN_B}{index_msg}")
    logger.info("=" * 60)
    
    # Ask user if they want to run diagnostics
    logger.info("")
    logger.info("Options:")
    logger.info("  1. Run diagnostic test (recommended first time)")
    logger.info("  2. Start normal tracking")
    logger.info("")
    
    try:
        choice = input("Enter choice (1 or 2): ").strip()
    except:
        choice = "2"
    
    # Create encoder
    encoder = EncoderTracker(
        pin_a=ENCODER_PIN_A,
        pin_b=ENCODER_PIN_B,
        ppr=ENCODER_PPR,
        wheel_diameter_mm=WHEEL_DIAMETER_MM,
        pin_index=ENCODER_PIN_INDEX,
        logger=logger
    )
    
    # Run diagnostic if requested
    if choice == "1":
        run_diagnostic_test(encoder, logger)
        encoder.reset_count()
    
    # Normal tracking mode
    logger.info("Starting normal tracking mode...")
    logger.info("Press Ctrl+C to exit")
    logger.info("")
    
    log_interval = 1.0
    last_log_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            
            count = encoder.get_count()
            rotations = encoder.get_rotations()
            distance_cm = encoder.get_distance_cm()
            speed_cmps = encoder.get_speed_cmps()
            
            if current_time - last_log_time >= log_interval:
                index_info = f" | Index: {encoder.get_index_count()}" if ENCODER_PIN_INDEX else ""
                logger.info(
                    f"Stats - Count: {count:6d} | Rotations: {rotations:7.2f} | "
                    f"Distance: {distance_cm:7.2f}cm | Speed: {speed_cmps:6.1f} cm/s{index_info}"
                )
                last_log_time = current_time
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("")
        logger.info("=" * 60)
        logger.info("Session Ended - Final Statistics")
        logger.info("=" * 60)
        logger.info(f"Total Count: {encoder.get_count()}")
        logger.info(f"Total Rotations: {encoder.get_rotations():.2f}")
        if ENCODER_PIN_INDEX:
            logger.info(f"Index Pulses: {encoder.get_index_count()}")
        logger.info(f"Total Distance: {encoder.get_distance_cm():.2f} cm ({encoder.get_distance_m():.3f} m)")
        
        diag = encoder.get_diagnostics()
        logger.info(f"Diagnostics - Error rate: {diag['error_rate']:.1f}%")
        logger.info("=" * 60)
        
    finally:
        encoder.cleanup()
        logger.info("GPIO cleaned up. Goodbye!")


if __name__ == "__main__":
    main()