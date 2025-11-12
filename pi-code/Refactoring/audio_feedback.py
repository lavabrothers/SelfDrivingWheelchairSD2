# File: audio_feedback.py
# Asynchronous beeper control for GPIO pin.

import RPi.GPIO as GPIO
import asyncio

# --- Configuration ---
BEEP_PIN = 24       # The GPIO pin your speaker is on
BEEP_DURATION = 0.08 # Length of one beep in seconds
PAUSE_DURATION = 0.08 # Pause between beeps

def initialize_beeper():
    """Sets up the GPIO pin for the beeper."""
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BEEP_PIN, GPIO.OUT)
        GPIO.output(BEEP_PIN, GPIO.LOW)
        print(f"Beeper initialized on GPIO {BEEP_PIN}")
        return True
    except Exception as e:
        print(f"Error initializing beeper: {e}")
        return False

def cleanup_beeper():
    """Cleans up the GPIO pin."""
    print("Cleaning up beeper GPIO.")
    GPIO.cleanup(BEEP_PIN)

async def play_beep(times: int):
    """
    Plays a number of beeps asynchronously.
    """
    for _ in range(times):
        try:
            GPIO.output(BEEP_PIN, GPIO.HIGH)
            await asyncio.sleep(BEEP_DURATION)
            GPIO.output(BEEP_PIN, GPIO.LOW)
            await asyncio.sleep(PAUSE_DURATION)
        except Exception as e:
            print(f"Error during beep: {e}")
            break # Stop beeping if there's an error