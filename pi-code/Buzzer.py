import RPi.GPIO as GPIO
import time

# Use the BCM pin numbering (GPIO 24)
GPIO.setmode(GPIO.BCM)

# Set up GPIO 24 as an output pin
GPIO.setup(24, GPIO.OUT)

try:
    print("Sending pop sound...")
    # Set pin HIGH (3.3V)
    GPIO.output(24, GPIO.HIGH)

    # Wait for a very short time
    time.sleep(0.1) 

    # Set pin LOW (0V)
    GPIO.output(24, GPIO.LOW)
    print("Done.")

finally:
    # Clean up the GPIO settings
    GPIO.cleanup()