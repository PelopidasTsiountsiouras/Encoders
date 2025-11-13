import RPi.GPIO as GPIO
import time

A_PIN = 17
B_PIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("Rotate the wheel slowly and watch for changes:")
try:
    while True:
        print(f"A: {GPIO.input(A_PIN)}, B: {GPIO.input(B_PIN)}")
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()