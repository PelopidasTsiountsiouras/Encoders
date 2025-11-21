#!/usr/bin/env python3
"""
Advanced Encoder Tracker with Auto-Calibration, Drift Correction, and Health Monitoring
"""

import RPi.GPIO as GPIO
import time
import math
import logging
import numpy as np
from threading import Lock
from datetime import datetime
from collections import deque

# Import your H5Logger
from h5_logger import H5Logger  # Assuming saved as h5_logger.py


class AdvancedEncoderTracker:
    def __init__(self, pin_a, pin_b, ppr_nominal, wheel_diameter_mm, pin_index=None, logger=None):
        """
        Advanced encoder tracker with auto-calibration and health monitoring
        
        Args:
            pin_a: GPIO pin for encoder channel A
            pin_b: GPIO pin for encoder channel B
            ppr_nominal: Nominal pulses per revolution (will be calibrated)
            wheel_diameter_mm: Wheel diameter in millimeters
            pin_index: GPIO pin for index/Z channel (required for calibration)
            logger: Optional logger instance
        """
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.pin_index = pin_index
        self.ppr_nominal = ppr_nominal
        self.wheel_diameter_mm = wheel_diameter_mm
        self.logger = logger or logging.getLogger(__name__)
        
        # Calibration state
        self.ppr_calibrated = ppr_nominal
        self.counts_per_revolution = ppr_nominal * 4  # Initial value
        self.calibration_complete = False
        self.calibration_samples = []
        self.calibration_target = 5  # Number of revolutions to average
        
        # Calculate wheel circumference
        self.wheel_circumference_mm = math.pi * wheel_diameter_mm
        self.mm_per_count = self.wheel_circumference_mm / self.counts_per_revolution
        
        # Encoder state
        self.encoder_count = 0
        self.corrected_count = 0.0  # Drift-corrected count (float)
        self.state = 0
        
        # Index pulse tracking
        self.index_count = 0
        self.last_index_time = 0
        self.last_index_count = 0
        self.index_positions = []  # Encoder count at each index
        self.index_times = []  # Time at each index
        self.counts_between_index = []  # Counts between index pulses
        
        # Drift correction
        self.drift_correction_enabled = False
        self.cumulative_drift = 0
        self.drift_history = deque(maxlen=10)
        
        # Speed tracking
        self.speed_history = deque(maxlen=5)
        self.last_count = 0
        self.last_time = time.time()
        self.index_based_speed = 0.0  # Speed calculated from index pulses
        
        # Error detection
        self.invalid_transitions = 0
        self.total_transitions = 0
        self.missed_index_pulses = 0
        self.count_jumps = 0
        self.last_count_check = 0
        
        # Health monitoring
        self.health_status = {
            'overall': 'INITIALIZING',
            'signal_quality': 100.0,
            'drift_magnitude': 0.0,
            'speed_consistency': 100.0,
            'index_reliability': 100.0,
            'last_update': time.time()
        }
        
        # Mismatch detection
        self.index_count_mismatches = 0
        self.expected_counts_per_rev = []
        
        # Setup GPIO
        self._setup_gpio()
        
        if pin_index:
            self.logger.info("Index channel detected - auto-calibration available")
        else:
            self.logger.warning("No index channel - calibration and drift correction disabled")
        
        self.logger.info(f"Encoder initialized: PPR={ppr_nominal} (nominal), "
                        f"Wheel={wheel_diameter_mm}mm")
        
    def _setup_gpio(self):
        """Setup GPIO pins and interrupts"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        if self.pin_index:
            GPIO.setup(self.pin_index, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(self.pin_index, GPIO.RISING, 
                                callback=self._index_callback, bouncetime=10)
        
        # Read initial state
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        self.state = (a << 1) | b
        
        # Attach interrupts
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, 
                            callback=self._encoder_callback, bouncetime=1)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, 
                            callback=self._encoder_callback, bouncetime=1)
        
    def _encoder_callback(self, channel):
        """Enhanced encoder callback with error detection"""
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        new_state = (a << 1) | b
        old_state = self.state
        
        if new_state == old_state:
            return
        
        self.total_transitions += 1
        transition = (old_state << 2) | new_state
        
        # Detect count jumps (potential slipping)
        count_before = self.encoder_count
        
        # Valid forward transitions
        if transition in [0b0010, 0b1011, 0b1101, 0b0100]:
            self.encoder_count += 1
        # Valid backward transitions
        elif transition in [0b0001, 0b0111, 0b1110, 0b1000]:
            self.encoder_count -= 1
        else:
            self.invalid_transitions += 1
        
        # Detect sudden jumps (skip detection)
        count_diff = abs(self.encoder_count - self.last_count_check)
        if count_diff > 50 and self.total_transitions > 100:  # Threshold
            self.count_jumps += 1
            self.logger.warning(f"Count jump detected: {count_diff} counts")
        
        self.last_count_check = self.encoder_count
        self.state = new_state
    
    def _index_callback(self, channel):
        """Enhanced index callback with calibration and drift correction"""
        current_time = time.time()
        current_count = self.encoder_count
        
        self.index_count += 1
        self.index_positions.append(current_count)
        self.index_times.append(current_time)
        
        # Calculate counts since last index
        if len(self.index_positions) > 1:
            counts_this_rev = current_count - self.index_positions[-2]
            self.counts_between_index.append(counts_this_rev)
            
            # Auto-calibration logic
            if not self.calibration_complete:
                self.calibration_samples.append(counts_this_rev)
                
                if len(self.calibration_samples) >= self.calibration_target:
                    self._perform_calibration()
            
            # Drift correction (only after calibration)
            if self.calibration_complete and self.drift_correction_enabled:
                self._apply_drift_correction(counts_this_rev)
            
            # Index-based speed calculation
            if len(self.index_times) > 1:
                time_diff = self.index_times[-1] - self.index_times[-2]
                if time_diff > 0:
                    # Distance = one full circumference
                    self.index_based_speed = (self.wheel_circumference_mm / time_diff) / 1000.0  # m/s
            
            # Mismatch detection
            expected = self.counts_per_revolution
            deviation = abs(counts_this_rev - expected)
            if deviation > expected * 0.1:  # 10% tolerance
                self.index_count_mismatches += 1
                self.logger.debug(f"Index mismatch: expected {expected}, got {counts_this_rev}")
        
        self.last_index_time = current_time
        self.last_index_count = current_count
        
    def _perform_calibration(self):
        """Auto-calibrate counts per revolution using index pulses"""
        if len(self.calibration_samples) < self.calibration_target:
            return
        
        # Remove outliers (keep middle 80%)
        samples = np.array(self.calibration_samples)
        q1, q3 = np.percentile(samples, [10, 90])
        filtered = samples[(samples >= q1) & (samples <= q3)]
        
        if len(filtered) > 0:
            self.counts_per_revolution = int(np.mean(filtered))
            self.ppr_calibrated = self.counts_per_revolution / 4
            self.mm_per_count = self.wheel_circumference_mm / self.counts_per_revolution
            
            std_dev = np.std(filtered)
            self.calibration_complete = True
            self.drift_correction_enabled = True
            
            self.logger.info("=" * 60)
            self.logger.info("AUTO-CALIBRATION COMPLETE")
            self.logger.info(f"  Nominal PPR: {self.ppr_nominal}")
            self.logger.info(f"  Calibrated PPR: {self.ppr_calibrated:.2f}")
            self.logger.info(f"  Counts/Rev: {self.counts_per_revolution}")
            self.logger.info(f"  Std Dev: {std_dev:.2f} counts ({std_dev/self.counts_per_revolution*100:.2f}%)")
            self.logger.info(f"  mm/count: {self.mm_per_count:.4f}")
            self.logger.info("=" * 60)
            
    def _apply_drift_correction(self, actual_counts):
        """Apply drift correction based on index pulses"""
        expected_counts = self.counts_per_revolution
        drift = actual_counts - expected_counts
        
        self.drift_history.append(drift)
        self.cumulative_drift += drift
        
        # Calculate correction factor
        correction = -drift / expected_counts  # Proportional correction
        self.corrected_count = self.encoder_count * (1.0 + correction)
        
        if abs(drift) > expected_counts * 0.05:  # Log if >5% drift
            avg_drift = np.mean(self.drift_history) if self.drift_history else 0
            self.logger.debug(f"Drift: {drift} counts ({drift/expected_counts*100:.1f}%), "
                            f"Avg: {avg_drift:.1f}")
    
    def update_health_status(self):
        """Calculate real-time health metrics"""
        current_time = time.time()
        
        # 1. Signal Quality (based on error rate)
        if self.total_transitions > 0:
            error_rate = (self.invalid_transitions / self.total_transitions) * 100
            signal_quality = max(0, 100 - error_rate * 10)  # Scale error impact
        else:
            signal_quality = 100
        
        # 2. Drift Magnitude
        if len(self.drift_history) > 0:
            avg_drift = abs(np.mean(self.drift_history))
            drift_percent = (avg_drift / self.counts_per_revolution) * 100
            drift_magnitude = min(100, drift_percent * 10)  # Scale to 0-100
        else:
            drift_magnitude = 0
        
        # 3. Speed Consistency (variance in speed readings)
        if len(self.speed_history) > 2:
            speed_var = np.var(list(self.speed_history))
            speed_mean = np.mean(list(self.speed_history))
            if speed_mean > 0.1:  # Only if moving
                cv = (np.sqrt(speed_var) / speed_mean) * 100  # Coefficient of variation
                speed_consistency = max(0, 100 - cv * 5)
            else:
                speed_consistency = 100
        else:
            speed_consistency = 100
        
        # 4. Index Reliability (if available)
        if self.pin_index and self.index_count > 0:
            mismatch_rate = (self.index_count_mismatches / self.index_count) * 100
            index_reliability = max(0, 100 - mismatch_rate * 10)
        else:
            index_reliability = 100 if not self.pin_index else 0
        
        # Overall health score (weighted average)
        overall_score = (
            signal_quality * 0.4 +
            (100 - drift_magnitude) * 0.2 +
            speed_consistency * 0.2 +
            index_reliability * 0.2
        )
        
        # Determine status
        if overall_score >= 90:
            status = 'EXCELLENT'
        elif overall_score >= 75:
            status = 'GOOD'
        elif overall_score >= 60:
            status = 'FAIR'
        elif overall_score >= 40:
            status = 'POOR'
        else:
            status = 'CRITICAL'
        
        self.health_status = {
            'overall': status,
            'overall_score': overall_score,
            'signal_quality': signal_quality,
            'drift_magnitude': drift_magnitude,
            'speed_consistency': speed_consistency,
            'index_reliability': index_reliability,
            'last_update': current_time
        }
        
        return self.health_status
    
    def get_count(self, use_corrected=True):
        """Get encoder count (corrected or raw)"""
        if use_corrected and self.drift_correction_enabled:
            return int(self.corrected_count)
        return self.encoder_count
    
    def get_index_count(self):
        """Get number of complete revolutions"""
        return self.index_count
    
    def reset_count(self):
        """Reset encoder count and correction"""
        old_count = self.encoder_count
        self.encoder_count = 0
        self.corrected_count = 0.0
        self.cumulative_drift = 0
        self.last_count = 0
        self.last_time = time.time()
        self.speed_history.clear()
        self.logger.info(f"Encoder reset (previous count: {old_count})")
    
    def get_rotations(self, use_corrected=True):
        """Get number of wheel rotations"""
        count = self.get_count(use_corrected)
        return count / self.counts_per_revolution
    
    def get_distance_mm(self, use_corrected=True):
        """Get distance traveled in millimeters"""
        count = self.get_count(use_corrected)
        return count * self.mm_per_count
    
    def get_distance_cm(self, use_corrected=True):
        """Get distance traveled in centimeters"""
        return self.get_distance_mm(use_corrected) / 10.0
    
    def get_distance_m(self, use_corrected=True):
        """Get distance traveled in meters"""
        return self.get_distance_mm(use_corrected) / 1000.0
    
    def get_speed_mps(self, use_index=False):
        """Calculate speed in m/s (from counts or index)"""
        if use_index and self.index_based_speed > 0:
            return self.index_based_speed
        
        current_time = time.time()
        current_count = self.get_count(use_corrected=True)
        
        time_diff = current_time - self.last_time
        count_diff = current_count - self.last_count
        
        if time_diff > 0:
            distance_mm = count_diff * self.mm_per_count
            speed_m_per_sec = (distance_mm / time_diff) / 1000.0
            
            self.speed_history.append(speed_m_per_sec)
            self.last_time = current_time
            self.last_count = current_count
            
            if len(self.speed_history) > 0:
                return sum(self.speed_history) / len(self.speed_history)
        
        return 0.0
    
    def get_speed_cmps(self, use_index=False):
        """Get speed in cm/s"""
        return self.get_speed_mps(use_index) * 100.0
    
    def get_speed_kmh(self, use_index=False):
        """Get speed in km/h"""
        return self.get_speed_mps(use_index) * 3.6
    
    def get_calibration_status(self):
        """Get calibration information"""
        return {
            'complete': self.calibration_complete,
            'nominal_ppr': self.ppr_nominal,
            'calibrated_ppr': self.ppr_calibrated,
            'counts_per_rev': self.counts_per_revolution,
            'samples_collected': len(self.calibration_samples),
            'drift_correction': self.drift_correction_enabled
        }
    
    def get_diagnostics(self):
        """Get comprehensive diagnostic information"""
        return {
            'total_transitions': self.total_transitions,
            'invalid_transitions': self.invalid_transitions,
            'error_rate': (self.invalid_transitions / max(1, self.total_transitions)) * 100,
            'count_jumps': self.count_jumps,
            'missed_index': self.missed_index_pulses,
            'index_mismatches': self.index_count_mismatches,
            'cumulative_drift': self.cumulative_drift,
            'avg_drift': np.mean(self.drift_history) if self.drift_history else 0,
            'health_status': self.health_status
        }
    
    def cleanup(self):
        """Cleanup GPIO"""
        self.logger.info("Cleaning up GPIO")
        GPIO.cleanup()


class EncoderDataLogger:
    """Enhanced data logger with health monitoring"""
    
    def __init__(self, filename, log_interval=0.1):
        self.filename = filename
        self.log_interval = log_interval
        self.start_time = time.time()
        
    def log_sample(self, encoder):
        """Log comprehensive encoder data"""
        current_time = time.time()
        timestamp = current_time - self.start_time
        
        # Update health before logging
        health = encoder.update_health_status()
        
        # Main encoder data
        data = np.array([
            timestamp,
            encoder.get_count(use_corrected=False),  # Raw count
            encoder.get_count(use_corrected=True),   # Corrected count
            encoder.get_rotations(use_corrected=True),
            encoder.get_distance_cm(use_corrected=True),
            encoder.get_speed_cmps(use_index=False),
            encoder.get_speed_cmps(use_index=True),  # Index-based speed
            encoder.get_index_count(),
            health['overall_score'],
            health['signal_quality'],
            health['drift_magnitude'],
            health['speed_consistency'],
            health['index_reliability']
        ], dtype=np.float64)
        
        with H5Logger(self.filename, dset="encoder_data", dtype=np.float64) as h5:
            h5.append(data)
        
        # Calibration status (log once per second)
        if int(timestamp) != int(timestamp - self.log_interval):
            cal = encoder.get_calibration_status()
            diag = encoder.get_diagnostics()
            
            diag_data = np.array([
                timestamp,
                diag['total_transitions'],
                diag['invalid_transitions'],
                diag['error_rate'],
                diag['count_jumps'],
                diag['index_mismatches'],
                diag['cumulative_drift'],
                diag['avg_drift'],
                1.0 if cal['complete'] else 0.0,
                cal['calibrated_ppr']
            ], dtype=np.float64)
            
            with H5Logger(self.filename, dset="diagnostics", dtype=np.float64) as h5:
                h5.append(diag_data)
        
        return data, health


def run_calibration_sequence(encoder, logger):
    """Interactive calibration sequence"""
    logger.info("")
    logger.info("=" * 60)
    logger.info("AUTO-CALIBRATION SEQUENCE")
    logger.info("=" * 60)
    logger.info("Please rotate the wheel SLOWLY and STEADILY")
    logger.info(f"Need {encoder.calibration_target} complete revolutions")
    logger.info("The index channel will detect each revolution")
    logger.info("")
    logger.info("Starting calibration...")
    
    while not encoder.calibration_complete:
        cal_status = encoder.get_calibration_status()
        samples = cal_status['samples_collected']
        target = encoder.calibration_target
        
        logger.info(f"Progress: {samples}/{target} revolutions detected")
        time.sleep(1)
    
    logger.info("Calibration complete!")
    logger.info("")


def main_with_advanced_features():
    """Main program with all advanced features"""
    
    # Configuration
    ENCODER_PIN_A = 17
    ENCODER_PIN_B = 27
    ENCODER_PIN_INDEX = 22  # REQUIRED for advanced features
    ENCODER_PPR = 48  # Nominal value
    WHEEL_DIAMETER_MM = 60
    LOG_INTERVAL = 0.1
    
    # Setup logging
    logger = logging.getLogger('advanced_encoder')
    logger.setLevel(logging.INFO)
    logger.handlers.clear()
    
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s',
                                 datefmt='%H:%M:%S')
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    h5_filename = f'encoder_advanced_{timestamp}.h5'
    
    logger.info("=" * 60)
    logger.info("ADVANCED ENCODER TRACKER")
    logger.info("=" * 60)
    logger.info("Features:")
    logger.info("  ✓ Auto-calibration")
    logger.info("  ✓ Drift correction")
    logger.info("  ✓ Index-based speed")
    logger.info("  ✓ Error detection")
    logger.info("  ✓ Health monitoring")
    logger.info("=" * 60)
    logger.info(f"Data file: {h5_filename}")
    logger.info("")
    
    # Create encoder
    encoder = AdvancedEncoderTracker(
        pin_a=ENCODER_PIN_A,
        pin_b=ENCODER_PIN_B,
        ppr_nominal=ENCODER_PPR,
        wheel_diameter_mm=WHEEL_DIAMETER_MM,
        pin_index=ENCODER_PIN_INDEX,
        logger=logger
    )
    
    # Run calibration
    if ENCODER_PIN_INDEX:
        try:
            input("Press Enter to start calibration (or Ctrl+C to skip)...")
            run_calibration_sequence(encoder, logger)
        except KeyboardInterrupt:
            logger.info("\nCalibration skipped")
    
    # Create data logger
    data_logger = EncoderDataLogger(h5_filename, LOG_INTERVAL)
    
    logger.info("")
    logger.info("Starting data collection with health monitoring...")
    logger.info("Press Ctrl+C to stop")
    logger.info("")
    
    last_log_time = time.time()
    last_display_time = time.time()
    last_health_display = time.time()
    
    try:
        while True:
            current_time = time.time()
            
            # Log data
            if current_time - last_log_time >= LOG_INTERVAL:
                data, health = data_logger.log_sample(encoder)
                last_log_time = current_time
            
            # Display stats every second
            if current_time - last_display_time >= 1.0:
                logger.info(
                    f"Count: {encoder.get_count():6d} | "
                    f"Corrected: {encoder.get_count(True):6d} | "
                    f"Dist: {encoder.get_distance_cm():7.2f}cm | "
                    f"Speed: {encoder.get_speed_cmps():.1f}cm/s"
                )
                last_display_time = current_time
            
            # Display health every 5 seconds
            if current_time - last_health_display >= 5.0:
                health = encoder.health_status
                logger.info(
                    f"Health: {health['overall']} ({health['overall_score']:.0f}/100) | "
                    f"Signal: {health['signal_quality']:.0f}% | "
                    f"Drift: {health['drift_magnitude']:.1f}%"
                )
                last_health_display = current_time
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        logger.info("")
        logger.info("=" * 60)
        logger.info("SESSION SUMMARY")
        logger.info("=" * 60)
        
        cal = encoder.get_calibration_status()
        diag = encoder.get_diagnostics()
        health = encoder.update_health_status()
        
        logger.info(f"\nCalibration:")
        logger.info(f"  Status: {'Complete' if cal['complete'] else 'Not performed'}")
        if cal['complete']:
            logger.info(f"  Calibrated PPR: {cal['calibrated_ppr']:.2f}")
            logger.info(f"  Drift correction: {'Enabled' if cal['drift_correction'] else 'Disabled'}")
        
        logger.info(f"\nDistance:")
        logger.info(f"  Raw count: {encoder.get_count(False)}")
        logger.info(f"  Corrected count: {encoder.get_count(True)}")
        logger.info(f"  Total distance: {encoder.get_distance_cm():.2f} cm")
        logger.info(f"  Revolutions: {encoder.get_rotations():.2f}")
        
        logger.info(f"\nHealth Status: {health['overall']} ({health['overall_score']:.1f}/100)")
        logger.info(f"  Signal quality: {health['signal_quality']:.1f}%")
        logger.info(f"  Drift magnitude: {health['drift_magnitude']:.1f}%")
        logger.info(f"  Speed consistency: {health['speed_consistency']:.1f}%")
        logger.info(f"  Index reliability: {health['index_reliability']:.1f}%")
        
        logger.info(f"\nDiagnostics:")
        logger.info(f"  Error rate: {diag['error_rate']:.2f}%")
        logger.info(f"  Count jumps: {diag['count_jumps']}")
        logger.info(f"  Index mismatches: {diag['index_mismatches']}")
        logger.info(f"  Avg drift: {diag['avg_drift']:.1f} counts")
        
        logger.info(f"\nData saved to: {h5_filename}")
        logger.info("=" * 60)
        
    finally:
        encoder.cleanup()


if __name__ == "__main__":
    main_with_advanced_features()