#!/usr/bin/env python3
"""
AMT103 Quadrature Encoder Reader for Raspberry Pi 5
Measures wheel rotations, speed, and distance with noise filtering
"""

import RPi.GPIO as GPIO
import time
import threading
from collections import deque
import math

class AMT103Encoder:
    def __init__(self, pin_a=17, pin_b=27, pin_index=22, ppr=48, wheel_diameter_cm=6.5):
        """
        Initialize the encoder reader
        
        Args:
            pin_a: GPIO pin for channel A (BCM numbering)
            pin_b: GPIO pin for channel B (BCM numbering)
            pin_index: GPIO pin for index channel (BCM numbering)
            ppr: Pulses per revolution of the encoder
            wheel_diameter_cm: Wheel diameter in centimeters
        """
        self.PIN_A = pin_a
        self.PIN_B = pin_b
        self.PIN_INDEX = pin_index
        self.PPR = ppr
        self.WHEEL_DIAMETER = wheel_diameter_cm
        self.WHEEL_CIRCUMFERENCE = math.pi * wheel_diameter_cm  # cm per revolution
        
        # Encoder state variables
        self.encoder_count = 0  # Raw encoder counts (4x resolution)
        self.last_count = 0
        self.full_rotations = 0  # Complete rotations measured by index
        self.index_count = 0  # Number of index pulses detected
        
        # Velocity calculation variables
        self.velocity_window = deque(maxlen=10)  # Moving average filter
        self.last_time = time.time()
        self.current_speed_cms = 0.0  # Speed in cm/s
        self.total_distance_cm = 0.0  # Total distance traveled
        
        # Thread safety
        self.lock = threading.Lock()
        
        # Debouncing parameters (in microseconds)
        self.last_a_time = 0
        self.last_b_time = 0
        self.last_index_time = 0
        self.debounce_time = 100  # 100 microseconds debounce
        
        # State tracking for quadrature decoding
        self.last_encoded = 0
        
        # Setup GPIO
        self._setup_gpio()
        
        # Start velocity calculation thread
        self.running = True
        self.calc_thread = threading.Thread(target=self._calculate_velocity, daemon=True)
        self.calc_thread.start()
        
        print(f"Encoder initialized:")
        print(f"  PPR: {self.PPR}")
        print(f"  Wheel diameter: {self.WHEEL_DIAMETER} cm")
        print(f"  Wheel circumference: {self.WHEEL_CIRCUMFERENCE:.2f} cm")
        print(f"  Counts per revolution (4x): {self.PPR * 4}")
    
    def _setup_gpio(self):
        """Setup GPIO pins and interrupts"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup input pins with pull-up resistors for noise immunity
        GPIO.setup(self.PIN_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.PIN_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.PIN_INDEX, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Add interrupt handlers for quadrature decoding
        GPIO.add_event_detect(self.PIN_A, GPIO.BOTH, callback=self._encoder_callback)
        GPIO.add_event_detect(self.PIN_B, GPIO.BOTH, callback=self._encoder_callback)
        GPIO.add_event_detect(self.PIN_INDEX, GPIO.RISING, callback=self._index_callback)
    
    def _encoder_callback(self, channel):
        """
        Callback for encoder channels A and B
        Implements 4x quadrature decoding with debouncing
        """
        current_time = time.time() * 1e6  # Convert to microseconds
        
        # Software debouncing
        if channel == self.PIN_A:
            if current_time - self.last_a_time < self.debounce_time:
                return
            self.last_a_time = current_time
        elif channel == self.PIN_B:
            if current_time - self.last_b_time < self.debounce_time:
                return
            self.last_b_time = current_time
        
        # Read current state of both channels
        a_state = GPIO.input(self.PIN_A)
        b_state = GPIO.input(self.PIN_B)
        
        # Encode current state (2 bits: AB)
        encoded = (a_state << 1) | b_state
        
        # Calculate state transition
        sum_val = (self.last_encoded << 2) | encoded
        
        # Quadrature decoding lookup
        # Forward: 0b0001, 0b0111, 0b1110, 0b1000 (+1)
        # Backward: 0b0010, 0b1011, 0b1101, 0b0100 (-1)
        with self.lock:
            if sum_val in (0b0001, 0b0111, 0b1110, 0b1000):
                self.encoder_count += 1
            elif sum_val in (0b0010, 0b1011, 0b1101, 0b0100):
                self.encoder_count -= 1
        
        self.last_encoded = encoded
    
    def _index_callback(self, channel):
        """
        Callback for index channel
        Used for rotation verification and error correction
        """
        current_time = time.time() * 1e6
        
        # Debounce index signal
        if current_time - self.last_index_time < self.debounce_time * 10:  # Longer debounce for index
            return
        self.last_index_time = current_time
        
        with self.lock:
            self.index_count += 1
            self.full_rotations += 1
            
            # Optional: Error correction based on index pulse
            # Check if encoder count matches expected count at index
            expected_count = self.full_rotations * self.PPR * 4
            actual_count = self.encoder_count
            error = abs(actual_count - expected_count)
            
            # If error is significant, adjust (but only if it's within reasonable bounds)
            if error > self.PPR * 2 and error < self.PPR * 10:
                print(f"Warning: Encoder drift detected. Error: {error} counts")
                # Optionally reset or adjust: self.encoder_count = expected_count
    
    def _calculate_velocity(self):
        """
        Background thread to calculate velocity using moving average filter
        Updates every 50ms
        """
        while self.running:
            time.sleep(0.05)  # 50ms update rate (20Hz)
            
            current_time = time.time()
            with self.lock:
                current_count = self.encoder_count
            
            # Calculate counts since last update
            delta_counts = current_count - self.last_count
            delta_time = current_time - self.last_time
            
            if delta_time > 0:
                # Calculate instantaneous speed
                rotations = delta_counts / (self.PPR * 4)  # 4x decoding
                distance = rotations * self.WHEEL_CIRCUMFERENCE  # cm
                speed = distance / delta_time  # cm/s
                
                # Add to moving average filter (reduces noise)
                self.velocity_window.append(speed)
                
                # Calculate filtered speed (moving average)
                self.current_speed_cms = sum(self.velocity_window) / len(self.velocity_window)
                
                # Update total distance
                self.total_distance_cm += abs(distance)
            
            self.last_count = current_count
            self.last_time = current_time
    
    def get_rotations(self):
        """Get current rotation count (can be fractional)"""
        with self.lock:
            return self.encoder_count / (self.PPR * 4)
    
    def get_full_rotations(self):
        """Get complete rotations counted by index channel"""
        with self.lock:
            return self.full_rotations
    
    def get_index_count(self):
        """Get number of index pulses detected"""
        with self.lock:
            return self.index_count
    
    def get_speed_cms(self):
        """Get current speed in cm/s (filtered)"""
        return self.current_speed_cms
    
    def get_speed_ms(self):
        """Get current speed in m/s"""
        return self.current_speed_cms / 100.0
    
    def get_distance_cm(self):
        """Get total distance traveled in cm"""
        return self.total_distance_cm
    
    def get_distance_m(self):
        """Get total distance traveled in meters"""
        return self.total_distance_cm / 100.0
    
    def get_encoder_count(self):
        """Get raw encoder count"""
        with self.lock:
            return self.encoder_count
    
    def reset(self):
        """Reset all counters"""
        with self.lock:
            self.encoder_count = 0
            self.last_count = 0
            self.full_rotations = 0
            self.index_count = 0
            self.total_distance_cm = 0.0
            self.velocity_window.clear()
        print("Encoder reset")
    
    def get_status(self):
        """Get comprehensive encoder status"""
        return {
            'encoder_count': self.get_encoder_count(),
            'rotations': self.get_rotations(),
            'full_rotations': self.get_full_rotations(),
            'index_count': self.get_index_count(),
            'speed_cms': self.get_speed_cms(),
            'speed_ms': self.get_speed_ms(),
            'distance_cm': self.get_distance_cm(),
            'distance_m': self.get_distance_m()
        }
    
    def cleanup(self):
        """Clean up GPIO and stop threads"""
        self.running = False
        self.calc_thread.join()
        GPIO.cleanup([self.PIN_A, self.PIN_B, self.PIN_INDEX])
        print("Encoder cleanup complete")


def main():
    """Example usage"""
    print("=== AMT103 Encoder Test ===\n")
    
    # Initialize encoder
    # Adjust wheel_diameter_cm to match your actual wheel
    encoder = AMT103Encoder(
        pin_a=17,
        pin_b=27,
        pin_index=22,
        ppr=48,
        wheel_diameter_cm=6.5  # Adjust this to your wheel diameter
    )
    
    try:
        print("\nEncoder running. Press Ctrl+C to stop.\n")
        
        while True:
            time.sleep(1.0)  # Update display every second
            
            status = encoder.get_status()
            
            print(f"\r{'='*60}")
            print(f"Raw Count: {status['encoder_count']:8d} | "
                  f"Rotations: {status['rotations']:8.2f}")
            print(f"Index Rotations: {status['full_rotations']:5d} | "
                  f"Index Count: {status['index_count']:5d}")
            print(f"Speed: {status['speed_cms']:6.2f} cm/s | "
                  f"{status['speed_ms']:6.4f} m/s")
            print(f"Distance: {status['distance_cm']:8.2f} cm | "
                  f"{status['distance_m']:8.4f} m")
            print(f"{'='*60}")
            
    except KeyboardInterrupt:
        print("\n\nStopping encoder...")
    finally:
        encoder.cleanup()
        print("Done!")


if __name__ == "__main__":
    main()