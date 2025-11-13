import time
from collections import deque
from gpiozero import DigitalInputDevice


class QuadratureEncoder:
    def __init__(self, pin_a, pin_b, resolution, period=0.1):
        """
        Initialize the encoder.

        :param pin_a: GPIO pin connected to channel A of the encoder.
        :param pin_b: GPIO pin connected to channel B of the encoder.
        :param resolution: Counts per revolution (CPR) of the encoder.
        :param period: Time interval (in seconds) for speed calculation.
        """
        self.pin_a = DigitalInputDevice(pin_a, pull_up=True)
        self.pin_b = DigitalInputDevice(pin_b, pull_up=True)
        self.resolution = resolution
        self.period = period

        self.counter = 0
        self.last_counter = 0
        self.speed = 0.0
        self.distance = 0.0
        self.measure_distance = False

        # Moving average filter
        self.filter_window = deque(maxlen=5)

        # Attach event listeners
        self.pin_a.when_activated = self._update_count
        self.pin_a.when_deactivated = self._update_count

    def _update_count(self):
        """Update the encoder count based on pin states."""
        a = self.pin_a.value
        b = self.pin_b.value
        if a == b:
            self.counter += 1
        else:
            self.counter -= 1

    def reset(self):
        """Reset the encoder count."""
        self.counter = 0

    def get_count(self):
        """Return the current count."""
        return self.counter

    def get_speed_rps(self):
        """
        Calculate the speed in rotations per second.

        :return: Speed in rotations per second.
        """
        delta_count = self.counter - self.last_counter
        self.speed = delta_count / self.resolution / self.period
        self.last_counter = self.counter

        # Apply filtering
        self.filter_window.append(self.speed)
        return sum(self.filter_window) / len(self.filter_window)  # Filtered speed

    def get_speed_rpm(self):
        """
        Calculate the speed in rotations per minute.

        :return: Speed in rotations per minute.
        """
        return self.get_speed_rps() * 60

    def start_distance_measurement(self):
        """Start measuring distance."""
        self.distance = 0.0
        self.measure_distance = True

    def stop_distance_measurement(self):
        """Stop measuring distance."""
        self.measure_distance = False

    def get_traveled_distance(self):
        """
        Calculate the traveled distance in encoder steps.

        :return: Traveled distance.
        """
        if self.measure_distance:
            self.distance += self.get_count()
        return self.distance


# Example usage
if __name__ == "__main__":
    PIN_A = 11  # Example GPIO pin for channel A
    PIN_B = 13  # Example GPIO pin for channel B
    RESOLUTION = 48  # CPR of the encoder
    PERIOD = 0.1  # Period for speed calculation in seconds

    encoder = QuadratureEncoder(PIN_A, PIN_B, RESOLUTION, PERIOD)

    try:
        while True:
            time.sleep(PERIOD)
            print(f"Count: {encoder.get_count()}")
            print(f"Speed (RPS): {encoder.get_speed_rps():.2f}")
            print(f"Speed (RPM): {encoder.get_speed_rpm():.2f}")
    except KeyboardInterrupt:
        print("Exiting...")
