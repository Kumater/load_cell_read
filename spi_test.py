import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(11, GPIO.OUT)  # GPIO11 = SCLK = Pin 23

print("Toggling GPIO11 - probe Pin 23 now")
try:
    while True:
        GPIO.output(11, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(11, GPIO.LOW)
        time.sleep(0.001)
except KeyboardInterrupt:
    GPIO.cleanup()