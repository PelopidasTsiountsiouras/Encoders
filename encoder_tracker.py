import RPi.GPIO as GPIO
import time
import math
import logging
from threading import Lock
from collections import deque


class EncoderTracker:
    """
    Fully patched quadrature encoder tracker with:
    - State-machine decoding
    - Illegal transition suppression
    - Auto direction correction from index
    - Auto A/B polarity swap detection
    - First index pulse ignored (startup position)
    - Distance, rotation, speed estimation
    """

    def __init__(self, pin_a, pin_b, ppr, wheel_diameter_mm, pin_index=None, logger=None):
        self.GPIO = GPIO
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.pin_index = pin_index
        self.logger = logger or logging.getLogger("EncoderTracker")

        self.ppr = ppr
        self.counts_per_revolution = ppr * 4

        # Wheel geometry
        self.wheel_diameter_mm = wheel_diameter_mm
        self.wheel_circumference_mm = math.pi * wheel_diameter_mm
        self.mm_per_count = self.wheel_circumference_mm / self.counts_per_revolution

        # State
        self.encoder_count = 0
        self.index_count = 0
        self.last_index_time = 0
        self.last_time = time.time()
        self.last_count = 0
        self.speed_history = deque(maxlen=5)
        self.lock = Lock()

        # Diagnostics
        self.total_transitions = 0
        self.a_transitions = 0
        self.b_transitions = 0

        # First index detection flag
        self.first_index_seen = False

        # Direction correction
        self.direction_inverted = False

        # A/B swap detection
        self.swap_check_active = True
        self.swap_detect_counter = 0

        # Quadrature state lookup table
        self.quad_table = [
            0, +1, -1, 0,
           -1, 0, 0, +1,
           +1, 0, 0, -1,
            0, -1, +1, 0
        ]

        self._setup_gpio()

        msg = f"Encoder initialized: PPR={ppr}, Wheel={wheel_diameter_mm}mm, "
        if pin_index:
            msg += f"Index={pin_index}, "
        msg += f"Counts/Rev={self.counts_per_revolution}, mm/count={self.mm_per_count:.3f}"
        self.logger.info(msg)

    # ---------------------------------------------------------
    # GPIO SETUP
    # ---------------------------------------------------------

    def _setup_gpio(self):
        GPIO = self.GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        if self.pin_index:
            GPIO.setup(self.pin_index, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(self.pin_index, GPIO.RISING, callback=self._index_callback)

        # Initial quadrature state
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        self.last_state = (a << 1) | b

        # Interrupts, no debounce (critical)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._encoder_callback)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._encoder_callback)

    # ---------------------------------------------------------
    # QUADRATURE CALLBACK
    # ---------------------------------------------------------

    def _encoder_callback(self, channel):
        GPIO = self.GPIO

        with self.lock:
            a = GPIO.input(self.pin_a)
            b = GPIO.input(self.pin_b)
            new_state = (a << 1) | b

            self.total_transitions += 1
            if channel == self.pin_a: self.a_transitions += 1
            else: self.b_transitions += 1

            idx = (self.last_state << 2) | new_state
            delta = self.quad_table[idx]

            if delta != 0:  # Valid state transition
                if self.direction_inverted:
                    delta = -delta

                self.encoder_count += delta
                self.last_state = new_state

                if self.swap_check_active:
                    if delta < 0:
                        self.swap_detect_counter += 1
                    else:
                        self.swap_detect_counter -= 1

                return

            # Invalid transition -> ignore
            self.last_state = new_state

    # ---------------------------------------------------------
    # INDEX CALLBACK (truth reference)
    # ---------------------------------------------------------

    def _index_callback(self, channel):
        now = time.time()

        with self.lock:
            # FIRST INDEX PULSE → DO NOT COUNT IT
            if not self.first_index_seen:
                self.first_index_seen = True
                self.logger.info("Index reference detected (startup). Ignoring this pulse.")
                self.last_index_time = now
                return

            # REAL ROTATION (full revolution)
            self.index_count += 1

            # Use quadrature estimate to validate/repair direction
            rotation_estimate = self.encoder_count / self.counts_per_revolution

            if rotation_estimate < 0:
                self.direction_inverted = not self.direction_inverted
                self.logger.warning("Direction corrected by index pulse!")
                self.encoder_count = abs(self.encoder_count)

            # A/B swap detection
            if self.swap_check_active and abs(self.swap_detect_counter) > 200:
                self.direction_inverted = not self.direction_inverted
                self.logger.error("A/B swap detected — automatically corrected.")
                self.swap_check_active = False

            # RPM calculation
            if self.last_index_time > 0:
                dt = now - self.last_index_time
                if dt > 0:
                    rpm = 60.0 / dt
                    self.logger.debug(f"Index #{self.index_count}, RPM={rpm:.1f}")

            self.last_index_time = now

    # ---------------------------------------------------------
    # GETTERS
    # ---------------------------------------------------------

    def get_count(self):
        with self.lock:
            return self.encoder_count

    def get_index_count(self):
        with self.lock:
            return self.index_count

    def get_rotations(self):
        return self.get_count() / self.counts_per_revolution

    def get_distance_mm(self):
        return self.get_count() * self.mm_per_count

    def get_distance_cm(self):
        return self.get_distance_mm() / 10

    def get_distance_m(self):
        return self.get_distance_mm() / 1000

    def get_speed_mps(self):
        now = time.time()
        cnt = self.get_count()
        dt = now - self.last_time
        dc = cnt - self.last_count

        if dt <= 0:
            return 0

        speed = (dc * self.mm_per_count) / 1000 / dt
        self.speed_history.append(speed)

        self.last_time = now
        self.last_count = cnt

        return sum(self.speed_history) / len(self.speed_history)

    def get_speed_cmps(self):
        return self.get_speed_mps() * 100

    def get_speed_kmh(self):
        return self.get_speed_mps() * 3.6

    def get_diagnostics(self):
        with self.lock:
            if self.total_transitions == 0:
                balance = 0
            else:
                balance = abs(self.a_transitions - self.b_transitions) / self.total_transitions * 100

            return {
                "A transitions": self.a_transitions,
                "B transitions": self.b_transitions,
                "Total transitions": self.total_transitions,
                "Signal balance %": balance,
                "Direction inverted": self.direction_inverted
            }

    def cleanup(self):
        self.GPIO.cleanup()
        self.logger.info("GPIO cleaned up.")
